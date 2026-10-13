/// @file ocp_perception_track.hpp
/// @brief Perception-constrained target tracking under a CT-STL bounded-staleness
///        invariant, as a receding-horizon registry OCP.
///
/// Ported from the validated solver example
///   ALIPDDP-main/problem_examples/STC/quad_perception/quad_blind_chase_stc.cpp
/// which is itself a paper-faithful port of
///   [CTSTL] Uzun & Acikmese 2026 (CT-STL / D-GMSR augmentation), and
///   [CTLOS] Hayner et al. 2025 (conic line-of-sight constraint).
///
/// SPEC
///   Phi =  G( staleness <= Delta )                      CT-STL invariant
///       /\ G(  see  => |v| <= V_FAST )                  STC, channel 0
///       /\ G( ~see  => |v| <= V_SLOW )                  STC, channel 1
///       /\ G(  mode => range band /\ image steady )     STC, channel 2
///
///   see  = in FOV /\ unoccluded              (gates the SPEED policy)
///   mode = in FOV /\ within R_TRACK /\ clear (gates the fix gate + quality)
///
/// The speed policy is deliberately INVERTED from the obvious one: fast when it
/// can see (the estimate is good, so aggressive motion is safe and it is how
/// distance gets closed), slow when blind (the estimate is degrading). Recovery
/// is therefore expensive, which pushes the planner to pre-position BEFORE an
/// occlusion rather than sprint afterwards.
///
/// WHY AN INVARIANT AND NOT F/G. Invariants are the one temporal class that
/// decomposes soundly over a receding window: if every window satisfies it and
/// the windows tile the mission, the mission satisfies it. Eventually/until
/// formulas do NOT decompose and would need a terminal ingredient. This is what
/// makes the RH deployment claim defensible.
///
/// ── RECEDING-HORIZON REBASING (the part with no single-shot analogue) ────────
/// The augmented states fall into three groups with THREE DIFFERENT policies.
/// Getting these wrong corrupts the monitor silently while everything looks fine.
///
///   zeta_G, xi_G : RESET to 0 every solve. They are per-horizon accumulators
///                  normalized by |I| = T_horizon. Carrying them integrates over
///                  an ever-growing window against the wrong normalization and
///                  makes Gamma meaningless. Paper ICs: eta(0)=1 -> zeta(0)=0.
///   s            : CARRIED, and initialized from the TRACKER, not the plan.
///                  It is a real physical quantity (time since the last good
///                  fix); replanning does not reset it.
///   p_hat, v_hat : RE-INITIALIZED from the current target estimate. A fresh
///                  camera observation resets it; while blind, prepare_extra
///                  first coasts the last observation to the solve time.
///   IDX_DT       : rebased to 0, as in every other variable-dt OCP here.
///
/// ── STATE LAYOUT (matters for the replayer contract) ─────────────────────────
///   x (NX=25): [ r(3) v(3) q(4) w(3) | gp gy | t | p_hat(3) v_hat(3) s | zg xg ]
///                0-2   3-5  6-9 10-12  13 14  15   16-18   19-21     22   23 24
///   u (NU=7) : [ fz Mx My Mz theta | gp_rate gy_rate ]
///
/// The gimbal angles sit below the time index so a future actuated-gimbal
/// adapter can replay them. MAVROS has no such command path, so the default
/// camera is fixed body-forward and PT_ENABLE_GIMBAL must be explicitly enabled
/// only by a platform that executes those states.

#pragma once
#include "target/predictor.hpp"

#include "optimal_control_problem.h"
#include "alipddp/alipddp.h"
#include "dynamics/quad_6dof_dynamics_aug.h"
#include "planner_core/types.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <any>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

namespace PerceptionTrackOCP {

inline double envOrQ(const char* n, double d) {
    const char* v = std::getenv(n); return v ? std::atof(v) : d;
}
inline int envOrInt(const char* n, int d) {
    const char* v = std::getenv(n); return v ? std::atoi(v) : d;
}

// ── Dimensions ───────────────────────────────────────────────────────────────
constexpr int NX      = 25;
constexpr int NU      = 7;
constexpr int NX_PHYS = 13;
constexpr int IDX_GP  = 13;
constexpr int IDX_GY  = 14;
constexpr int IDX_DT  = 15;   // == state_dim, per the replayer contract
constexpr int IDX_AUG0 = IDX_GP;
constexpr int IDX_PH  = 16;
constexpr int IDX_VH  = 19;
constexpr int IDX_S   = 22;
constexpr int IDX_ZG  = 23;
constexpr int IDX_XG  = 24;
constexpr int IDX_FZ    = 0;
constexpr int IDX_THETA = 4;
constexpr int IDX_GPR   = 5;
constexpr int IDX_GYR   = 6;
constexpr int N_CH      = 3;   // regime channels

inline int    horizonN()  { return envOrInt("PT_N", 20); }
inline double horizonT()  { return envOrQ("PT_T", 3.0); }
inline int    rk4Sub()    { return std::max(1, envOrInt("PT_RK4_SUB", 2)); }
inline double dtInit()    { return horizonT() / std::max(1, horizonN()); }
inline double dtMin()     { return envOrQ("PT_DT_MIN", 0.05); }
inline double dtMax()     { return envOrQ("PT_DT_MAX", 0.30); }

// ── Vehicle (moments are IXX-normalized: M is numerically an angular accel) ───
inline double massKg()  { return envOrQ("PT_MASS", 0.027); }
constexpr double IXX = 1.66e-5, IYY = 1.66e-5, IZZ = 2.92e-5;
constexpr double J_SCALE = 1.0 / IXX;
constexpr double FMIN = 0.12, FMAX = 0.52;
// Controls 1:3 are expressed in the scaled-inertia dynamics and are therefore
// numerically angular accelerations, not raw N*m. The old motor-derived value
// was multiplied by J_SCALE twice in effect and allowed ~360 rad/s^2, which made
// one bad RH update unrecoverable.
inline double tauMax() { return envOrQ("PT_TAU_MAX", 8.0); }
inline Eigen::Matrix3d inertia() {
    Eigen::Matrix3d J; J.setZero();
    J(0,0)=IXX*J_SCALE; J(1,1)=IYY*J_SCALE; J(2,2)=IZZ*J_SCALE; return J;
}
inline double hoverThrust() { return massKg() * 9.81; }

// ── Camera ([CTLOS] conic form) ──────────────────────────────────────────────
inline double fovHx()    { return envOrQ("PT_FOV_HX_DEG", 30.0) * M_PI / 180.0; }
inline double fovHy()    { return envOrQ("PT_FOV_HY_DEG", 22.0) * M_PI / 180.0; }
inline double rMinSafe() { return envOrQ("PT_RMIN", 0.8); }
inline double rMaxSens() { return envOrQ("PT_RMAX", 12.0); }
inline double rTrack()   { return envOrQ("PT_R_TRACK", 3.0); }
inline double occMargin(){ return envOrQ("PT_OCC_MARGIN", 0.05); }
inline double scFov()    { return envOrQ("PT_SC_FOV", 1.0); }
inline double scRng()    { return envOrQ("PT_SC_RNG", 1.0); }
inline double scClr()    { return envOrQ("PT_SC_CLR", 0.5); }
inline double gmsrC()    { return envOrQ("PT_GMSR_C", 1e-3); }
inline double gpLim()    { return envOrQ("PT_GP_LIM_DEG", 60.0) * M_PI / 180.0; }
inline double gyLim()    { return envOrQ("PT_GY_LIM_DEG", 90.0) * M_PI / 180.0; }
inline double gRateLim() { return envOrQ("PT_GRATE_LIM", 0.6); }
// MAVROS currently commands only the vehicle trajectory; it has no gimbal
// actuator path. Keep the state/control layout stable, but use a fixed
// body-forward camera unless a platform with an actuated gimbal opts in.
inline bool enableGimbal() { return envOrInt("PT_ENABLE_GIMBAL", 0) != 0; }

// ── Estimator + invariant ────────────────────────────────────────────────────
inline double kFix()     { return envOrQ("PT_K_FIX", 6.0); }
inline double kFixV()    { return envOrQ("PT_K_FIX_V", 4.0); }
inline double epsFix()   { return envOrQ("PT_EPS_FIX", 0.15); }
inline double tFix()     { return envOrQ("PT_T_FIX", 0.25); }
inline double deltaBound(){ return envOrQ("PT_DELTA", 2.0); }
inline double logScale() { return envOrQ("PT_LOG_SCALE", 1.0); }
inline double penScale() { return envOrQ("PT_PEN_SCALE", 1.0); }
inline double ctgmsrC()  { return envOrQ("PT_CTGMSR_C", 1e-4); }
inline double ctgmsrEps(){ return envOrQ("PT_CTGMSR_EPS", 1e-8); }
inline double wStl()     { return envOrQ("PT_W_STL", 500.0); }
// ── TERMINAL INGREDIENT (required for RH; has no single-shot analogue) ───────
// Over a short lookahead the invariant G(staleness <= Delta) CANNOT BIND: with
// Delta = 2 s in a 3 s window staleness may not reach the bound, xi stays 0,
// and Gamma alone is satisfied at the initial guess. The running tracking model
// stabilizes ordinary MPC behavior; this terminal ingredient additionally
// prices the visibility/staleness state handed to the next horizon.
//
// The fix is a terminal cost-to-go approximating the invariant's value BEYOND
// the window:
//   * staleness left on the clock at t_f is proximity to a future violation;
//   * ending the window BLIND means the violation is already inbound, so the
//     terminal state should lie in the "can see it" set with a margin.
// This is the standard MPC terminal-cost/terminal-set recipe, and it is what
// gives any recursive-feasibility argument a foothold.
inline double wTermS()   { return envOrQ("PT_W_TERM_S", 200.0); }
inline double wTermV()   { return envOrQ("PT_W_TERM_V", 2000.0); }
inline double rhoTarget(){ return envOrQ("PT_RHO_TARGET", 0.8); }
// Altitude margin is also part of the cost-to-go: nothing in the spec references
// altitude, so with only a hard floor the quad sinks until it lands ON the
// constraint -- and once the MEASURED state violates a hard constraint the IPM
// cannot initialize its slacks and the next plan is garbage. Keeping a terminal
// margin is what stops the loop walking into its own infeasible region.
inline double zSafe()    { return envOrQ("PT_Z_SAFE", 1.5); }
inline double wTermZ()   { return envOrQ("PT_W_TERM_Z", 300.0); }
// Terminal RELATIVE-velocity matching. This is the stabilizing part of the
// cost-to-go: nothing in the spec regulates velocity (the regime rows bound it
// only through an INTEGRAL, not pointwise), so over a short window the loop has
// no reason to settle and speed ramps monotonically until it runs away
// (measured: 0.46 -> 5.8 -> 60 m/s). Asking each window to END moving with the
// target is the standard tracking cost-to-go and is what makes the RH loop
// contractive rather than merely feasible.
inline double wTermVR()  { return envOrQ("PT_W_TERM_VR", 60.0); }
// Terminal RANGE-BAND term. rho_mode only penalizes being too FAR (R_TRACK) and
// says nothing about being too close, and the band is otherwise enforced only
// through an INTEGRAL regime row. Result without this: the loop closes 2.5 -> 0.8 m,
// past the band's lower edge, at which point the FOV geometry fails and it goes
// blind. "End the window in the good tracking configuration" has to include the
// band, not just the mode sign.
inline double wTermR()   { return envOrQ("PT_W_TERM_R", 150.0); }

// ── Regime obligations ───────────────────────────────────────────────────────
inline double vFast()      { return envOrQ("PT_V_FAST", 2.5); }
inline double vSlow()      { return envOrQ("PT_V_SLOW", 1.1); }
inline double rFollowLo()  { return envOrQ("PT_R_FOLLOW_LO", 1.2); }
inline double rFollowHi()  { return envOrQ("PT_R_FOLLOW_HI", 3.0); }
inline double sMaxImg()    { return envOrQ("PT_S_MAX", 1.0); }
inline double wFollow()    { return envOrQ("PT_W_FOLLOW", 1.0); }
inline double wVcap()      { return envOrQ("PT_W_VCAP", 1.0); }
inline double wImg()       { return envOrQ("PT_W_IMG", 1.0); }
inline double stcStepEps() { return envOrQ("PT_STC_STEP_EPS", 1e-5); }

// ── Safety / scene ───────────────────────────────────────────────────────────
inline double zFloor()    { return envOrQ("PT_Z_FLOOR", 0.3); }
inline double zCeil()     { return envOrQ("PT_Z_CEIL", 2.2); }
inline double wallClear() { return envOrQ("PT_WALL_CLEAR", 0.25); }
inline double rSepMin()   { return envOrQ("PT_R_SEP_MIN", 0.3); }
inline double rCtrl()     { return envOrQ("PT_R_CTRL", 1e-2); }
inline double rMom()      { return envOrQ("PT_R_MOM", 1e-1); }
inline double rGim()      { return envOrQ("PT_R_GIM", 1e-2); }
// Free-timestep regularizer. WITHOUT THIS quu(theta,theta) = 0, quu is singular,
// the backward pass cannot produce a step and the solver returns after ZERO
// iterations no matter what the cost or the constraints are. The repo's own
// quad_single_horizon OCP carries exactly this (R_THETA / THETA_REF); both of my
// ports omitted it. It also prefers a uniform mesh, which is desirable anyway.
inline double rTheta()    { return envOrQ("PT_R_THETA", 10.0); }
inline double wAtt()      { return envOrQ("PT_W_ATT", 2.0); }
inline double wOm()       { return envOrQ("PT_W_OM", 1.0); }
// Running tracking terms make the receding-horizon policy contractive between
// terminal nodes. STL still defines the custody deadline; these terms provide
// the ordinary MPC shaping needed to approach the last known target smoothly.
inline double wRunVR()     { return envOrQ("PT_W_RUN_VR", 8.0); }
inline double wRunR()      { return envOrQ("PT_W_RUN_R", 20.0); }
inline double wRunZ()      { return envOrQ("PT_W_RUN_Z", 30.0); }
inline double wRunVis()    { return envOrQ("PT_W_RUN_VIS", 40.0); }
inline double wRunAtt()    { return envOrQ("PT_W_RUN_ATT", 2.0); }
inline double wRunOm()     { return envOrQ("PT_W_RUN_OM", 0.2); }
inline double wRunSpeed()  { return envOrQ("PT_W_RUN_SPEED", 200.0); }

/// Obstacle set. On the vehicle these come from ROS params; the geometry is
/// assumed KNOWN a priori because the occlusion predicate must predict where
/// the line of sight will be blocked over the horizon — a proximity sensor
/// cannot supply that without an intervening mapping layer.
struct Pillar { Eigen::Vector3d lo, hi; };
inline std::vector<Pillar>& pillars() {
    static std::vector<Pillar> p;
    return p;
}
inline void setPillars(const std::vector<Pillar>& p) { pillars() = p; }

template <typename S> inline S posPart(const S& z) { return z > S(0) ? z : S(0); }
template <typename S> inline S negPart(const S& z) { return z < S(0) ? -z : S(0); }

// ── Constant-accel target predictor (same contract as the landing OCPs) ──────
using ConstantAccelPredictor = target_models::ConstantAccelPredictor;

/// `staleness_init` is the RH carry: time since the last valid target fix, taken
/// from the TargetTracker rather than from the previous plan.
struct PerceptionTrackExtra {
    std::shared_ptr<ConstantAccelPredictor> predictor;
    double staleness_init = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
//  D-GMSR conjunction ([CTSTL] Sec 3.1.1). Sound AND complete — sign-exact,
//  unlike a tanh product or an LSE smooth-min.
// ─────────────────────────────────────────────────────────────────────────────
inline double gmsrAnd(const double* v, int n, double c) {
    bool anyneg = false;
    for (int i = 0; i < n; ++i) if (v[i] <= 0) { anyneg = true; break; }
    if (anyneg) {
        double sums = 0;
        for (int i = 0; i < n; ++i) if (v[i] <= 0) sums += v[i]*v[i];
        return std::sqrt(c) - std::sqrt(c + sums / n);
    }
    double logmult = 0;
    for (int i = 0; i < n; ++i) logmult += 2.0 * std::log(v[i]);
    const double inner = std::pow(c, (double)n) + std::exp(logmult);
    return std::sqrt(std::pow(inner, 1.0 / n)) - std::sqrt(c);
}

/// EXACT segment-vs-AABB separation by the separating-axis theorem (6 axes).
/// Signed: > 0 disjoint, < 0 pierced. Nothing is sampled, so a thin pillar
/// cannot be missed.
inline double satSeparation(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
    const Eigen::Vector3d cs = 0.5*(p0+p1), hs = 0.5*(p1-p0);
    const Eigen::Vector3d cb = 0.5*(lo+hi), hb = 0.5*(hi-lo);
    const Eigen::Vector3d d = cs - cb, dir = p1 - p0;
    double best = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i)
        best = std::max(best, std::abs(d(i)) - (hb(i) + std::abs(hs(i))));
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d e = Eigen::Vector3d::Zero(); e(i) = 1.0;
        Eigen::Vector3d a = dir.cross(e);
        const double na = a.norm();
        if (na < 1e-9) continue;
        a /= na;
        const double rb = hb(0)*std::abs(a(0)) + hb(1)*std::abs(a(1)) + hb(2)*std::abs(a(2));
        best = std::max(best, std::abs(d.dot(a)) - rb);
    }
    return best;
}

/// Minimum SAT clearance of the camera->target segment against the whole array.
///
/// EXACT branch-and-bound, not an approximation: the separation along a box axis
/// is already a valid SAT separating-axis value, so it lower-bounds nothing --
/// but the AABB-vs-AABB gap DOES lower-bound the true minimum distance, and any
/// pillar whose cheap bound already exceeds the running minimum cannot become
/// the minimum. Skipping those is exact.
///
/// This matters because losClearance sits inside the finite-differenced
/// constraint Jacobians and dominates solve time (measured: ~28 ms per pillar
/// per solve). Culling is the cheapest available speedup and changes no result.
inline double losClearance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
    const auto& P = pillars();
    if (P.empty()) return 1e3;
    const Eigen::Vector3d slo = p0.cwiseMin(p1), shi = p0.cwiseMax(p1);
    double m = std::numeric_limits<double>::infinity();
    for (const auto& p : P) {
        // cheap axis-aligned gap: <= true separation, so a valid lower bound
        double lb = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < 3; ++i)
            lb = std::max(lb, std::max(p.lo(i) - shi(i), slo(i) - p.hi(i)));
        if (lb >= m) continue;                       // cannot beat the running min
        m = std::min(m, satSeparation(p0, p1, p.lo, p.hi));
    }
    return m;
}

/// Signed distance to the obstacle set (positive outside). Used for body
/// clearance: unlike a penetration measure this has a nonzero gradient
/// everywhere feasible, so the solver sees the pillar coming.
inline double pillarSdf(const Eigen::Vector3d& p) {
    const auto& P = pillars();
    if (P.empty()) return 1e3;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pl : P) {
        Eigen::Vector3d q;
        for (int i = 0; i < 3; ++i) q(i) = std::max(pl.lo(i)-p(i), p(i)-pl.hi(i));
        const Eigen::Vector3d qp = q.cwiseMax(0.0);
        best = std::min(best, qp.norm() + std::min(0.0, q.maxCoeff()));
    }
    return best;
}

inline Eigen::Matrix3d rotZ(double a) {
    Eigen::Matrix3d R; R << std::cos(a),-std::sin(a),0, std::sin(a),std::cos(a),0, 0,0,1; return R;
}
inline Eigen::Matrix3d rotY(double a) {
    Eigen::Matrix3d R; R << std::cos(a),0,std::sin(a), 0,1,0, -std::sin(a),0,std::cos(a); return R;
}
/// Body->sensor rotation. Sensor +z is the boresight ([CTLOS] convention); the
/// trailing Ry(pi/2) maps body +x onto sensor +z.
inline Eigen::Matrix3d camRbs(const Eigen::VectorXd& x) {
    const double gy = enableGimbal() ? x(IDX_GY) : 0.0;
    const double gp = enableGimbal() ? x(IDX_GP) : 0.0;
    return rotZ(gy) * rotY(gp) * rotY(M_PI/2.0);
}
inline Eigen::Vector3d camAxis(const Eigen::VectorXd& x) {
    const Eigen::Vector4d q = x.segment<4>(6);
    return Quad6DOFVarTime<double>::calcC(q) * camRbs(x).col(2);
}
inline Eigen::Vector3d camAngularRate(const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    const Eigen::Vector4d q = x.segment<4>(6);
    const Eigen::Matrix3d Rwb = Quad6DOFVarTime<double>::calcC(q);
    const Eigen::Vector3d wb = x.segment<3>(10);
    Eigen::Vector3d wg = Eigen::Vector3d::Zero();
    if (enableGimbal()) {
        wg = u(IDX_GYR)*Eigen::Vector3d::UnitZ()
           + rotZ(x(IDX_GY))*(u(IDX_GPR)*Eigen::Vector3d::UnitY());
    }
    return Rwb * (wb + wg);
}

/// [CTLOS] conic FOV robustness: rho = c^T p_s_s - ||A p_s_s||, positive inside.
/// Homogeneous of degree 1 — no range normalization, so pointing and range stay
/// independent predicates.
inline double rhoFov(const Eigen::Vector3d& p) {
    // With the fixed body-forward mount, sensor x is body vertical and sensor y
    // is body lateral. Apply the horizontal half-angle to lateral and the
    // vertical half-angle to vertical.
    const double ax = p(1)/std::tan(fovHx()), ay = p(0)/std::tan(fovHy());
    return p(2) - std::sqrt(ax*ax + ay*ay + 1e-12);
}

struct VisTerms {
    double rho_see = 0.0, rho_mode = 0.0, rho_d = 0.0, rho_fov = 0.0;
    double range = 0.0, clearance = 0.0;
    bool in_mode = false;
};

/// Visibility as the planner sees it: every predicate on the ESTIMATE p_hat.
inline VisTerms visTerms(const Eigen::VectorXd& x) {
    VisTerms o;
    const Eigen::Vector3d r = x.head(3), ph = x.segment<3>(IDX_PH);
    const Eigen::Vector4d q = x.segment<4>(6);
    const Eigen::Vector3d pss = camRbs(x).transpose()
                              * (Quad6DOFVarTime<double>::calcC(q).transpose() * (ph - r));
    o.rho_fov = rhoFov(pss);
    o.range = std::sqrt((ph-r).squaredNorm() + 1e-12);
    o.clearance = losClearance(r, ph);
    o.rho_d = rTrack() - o.range;

    // TWO trigger levels. rho_see must NOT include distance: if being far put the
    // quad in the slow regime it would be slow exactly when it needs to close.
    const double msee[3] = { o.rho_fov/scFov(), (rMaxSens()-o.range)/scRng(),
                             (o.clearance-occMargin())/scClr() };
    o.rho_see = gmsrAnd(msee, 3, gmsrC());
    const double mmode[4] = { o.rho_fov/scFov(), o.rho_d/scRng(),
                              (rMaxSens()-o.range)/scRng(),
                              (o.clearance-occMargin())/scClr() };
    o.rho_mode = gmsrAnd(mmode, 4, gmsrC());
    o.in_mode = (o.rho_mode > 0.0);
    return o;
}

/// Measurement gate — the SENSOR MODEL, deliberately SMOOTH.
/// The logical implications are exact (squared signed parts); this is physics: a
/// real sensor degrades continuously at the FOV edge. It is also load-bearing
/// numerically — with a hard gate d(staleness)/d(position) is identically zero
/// while blind, so the optimizer gets no gradient telling it how to regain the
/// fix.
inline double fixGate(double rho_mode) {
    return 0.5 * (1.0 + std::tanh(rho_mode / epsFix()));
}

/// CT-STL ALWAYS readout ([CTSTL] p.6). NOTE the branch assignment: for ALWAYS
/// the POSITIVE part drives the geometric (log) state and the NEGATIVE part the
/// arithmetic one. EVENTUALLY swaps them — reusing an F readout here silently
/// inverts the operator.
inline double ctgmsrAlwaysRho(double zeta, double xi) {
    const double z = std::max(-700.0, std::min(700.0, zeta));
    return std::sqrt(ctgmsrC() + std::exp(z)) - std::sqrt(ctgmsrC() + posPart(xi));
}
inline double stlFormulaRho(const Eigen::VectorXd& x) {
    return ctgmsrAlwaysRho(x(IDX_ZG), x(IDX_XG));
}
inline void stlFormulaGrad(const Eigen::VectorXd& x, double& dz, double& dx) {
    const double z = std::max(-700.0, std::min(700.0, x(IDX_ZG)));
    const double eta = std::exp(z);
    dz = 0.5 * eta / std::sqrt(ctgmsrC() + eta);
    const double xi = posPart(x(IDX_XG));
    dx = (x(IDX_XG) > 0.0) ? (-0.5 / std::sqrt(ctgmsrC() + xi)) : 0.0;
}

/// Three implications, two triggers, three channels ([CTSTL] Sec 4.2 product
/// form: chi = [trigger]^2_sign * violation^2, EXACTLY zero when the trigger is
/// inactive). Nothing commands searching or closing — both emerge from the
/// staleness deadline.
inline void regimeDensities(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double* chi) {
    const VisTerms vt = visTerms(x);
    const Eigen::Vector3d v = x.segment<3>(3), vh = x.segment<3>(IDX_VH);
    const Eigen::Vector3d ph = x.segment<3>(IDX_PH);
    const double sp2 = v.squaredNorm();
    const double see_p = posPart(vt.rho_see), see_n = negPart(vt.rho_see);
    const double mode_p = posPart(vt.rho_mode);

    const double nf = negPart(vFast()*vFast() - sp2) / (vFast()*vFast());
    chi[0] = see_p*see_p * (wVcap()*nf*nf);

    const double ns = negPart(vSlow()*vSlow() - sp2) / (vSlow()*vSlow());
    chi[1] = see_n*see_n * (wVcap()*ns*ns);

    const double nlo = negPart(vt.range - rFollowLo());
    const double nhi = negPart(rFollowHi() - vt.range);
    const Eigen::Vector3d los = ph - x.head(3);
    const double r2 = los.squaredNorm() + 1e-12;
    const Eigen::Vector3d wr = los.cross(v - vh)/r2 - camAngularRate(x, u);
    const Eigen::Vector3d lh = los / std::sqrt(r2);
    const double s2 = (wr - lh*lh.dot(wr)).squaredNorm();
    const double ni = negPart(sMaxImg()*sMaxImg() - s2) / (sMaxImg()*sMaxImg());
    chi[2] = mode_p*mode_p * (wFollow()*(nlo*nlo + nhi*nhi) + wImg()*ni*ni);
}

}  // namespace PerceptionTrackOCP

namespace PerceptionTrackOCP {

// ─────────────────────────────────────────────────────────────────────────────
//  Dynamics. Inherits Quad6DOFVarTime to reach the protected xdot_impl /
//  rk4_jac_impl, but does its own substepping: the base's public propagate()
//  advances only the physical block in one full step, which is not the same map
//  as RK4_SUB composed, quaternion-renormalized substeps and would leave every
//  augmented state unintegrated.
//
//  The target enters ONLY through the predictor — there is no global target.
// ─────────────────────────────────────────────────────────────────────────────
template <typename Scalar>
class PerceptionDyn : public Quad6DOFVarTime<Scalar> {
public:
    explicit PerceptionDyn(std::shared_ptr<ConstantAccelPredictor> pred)
        : pred_(std::move(pred)), sub_(rk4Sub()) {
        this->dim_x = NX; this->dim_u = NU; this->dim_x_out = NX;
        this->setMass(massKg());
        this->setGravity(Eigen::Vector3d(0, 0, -9.81));
        this->setJb(inertia());
    }

    Eigen::VectorXd physSubstep(const Eigen::VectorXd& xp, const Eigen::VectorXd& up,
                                double h) const {
        const Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
        const Eigen::VectorXd k1 = this->xdot_impl(xp, up, a0);
        const Eigen::VectorXd k2 = this->xdot_impl((xp + 0.5*h*k1).eval(), up, a0);
        const Eigen::VectorXd k3 = this->xdot_impl((xp + 0.5*h*k2).eval(), up, a0);
        const Eigen::VectorXd k4 = this->xdot_impl((xp + h*k3).eval(), up, a0);
        Eigen::VectorXd xn = xp + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
        xn.segment(6, 4).normalize();
        return xn;
    }

    void augSubstep(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double h,
                    Eigen::VectorXd& xn) const {
        xn(IDX_DT) = x(IDX_DT) + h;
        xn(IDX_GP) = x(IDX_GP) + h*u(IDX_GPR);
        xn(IDX_GY) = x(IDX_GY) + h*u(IDX_GYR);

        const VisTerms vt0 = visTerms(x);
        const double g = fixGate(vt0.rho_mode);
        const Eigen::Vector3d ph = x.segment<3>(IDX_PH), vh = x.segment<3>(IDX_VH);
        // "truth" the estimator corrects toward is the PREDICTED target — under
        // RH that is itself a prediction, which is honest: the planner cannot do
        // better than its own target model.
        const Eigen::Vector3d pt = pred_->predictPos(x(IDX_DT));
        const Eigen::Vector3d vt = pred_->predictVel(x(IDX_DT));
        xn.segment<3>(IDX_PH) = ph + h*(vh + g*kFix()*(pt - ph));
        xn.segment<3>(IDX_VH) = vh + h*(g*kFixV()*(vt - vh));

        // Multiplicative-decay staleness: smooth and structurally nonnegative,
        // so no max(0,.) clamp and no CFL guard. The clamped form put a KINK
        // exactly where the solver spends most of its time (s pinned at 0 while
        // tracking) and broke the Jacobians there.
        const double s0 = x(IDX_S);
        xn(IDX_S) = (s0 + h*(1.0 - g)) * std::exp(-g*h/tFix());

        const double rho = deltaBound() - xn(IDX_S);
        const double lp = posPart(rho)/logScale(), ln = negPart(rho)*penScale();
        const double Tn = std::max(horizonT(), 1e-3);
        xn(IDX_ZG) = x(IDX_ZG) + h*std::log(lp*lp + ctgmsrEps())/Tn;
        xn(IDX_XG) = x(IDX_XG) + h*(ln*ln)/Tn;
    }

    Eigen::VectorXd advanceSubstep(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                                   double h, double* acc) const {
        double d0[N_CH] = {0,0,0};
        if (acc) regimeDensities(x, u, d0);
        Eigen::VectorXd xn(NX);
        xn.head(NX_PHYS) = physSubstep(x.head(NX_PHYS), u.head(4), h);
        augSubstep(x, u, h, xn);
        if (acc) {
            double d1[N_CH];
            regimeDensities(xn, u, d1);
            for (int c = 0; c < N_CH; ++c) acc[c] += 0.5*h*(d0[c] + d1[c]);
        }
        return xn;
    }

    Eigen::VectorXd stepD(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        const double h = u(IDX_THETA)/sub_;
        Eigen::VectorXd xn = x;
        for (int s = 0; s < sub_; ++s) xn = advanceSubstep(xn, u, h, nullptr);
        return xn;
    }
    void intervalIntegrals(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double* a) const {
        for (int c = 0; c < N_CH; ++c) a[c] = 0.0;
        const double h = u(IDX_THETA)/sub_;
        Eigen::VectorXd xn = x;
        for (int s = 0; s < sub_; ++s) xn = advanceSubstep(xn, u, h, a);
    }
    void safetySubstages(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                         std::vector<double>& out) const {
        out.assign(4*sub_, 0.0);
        const double h = u(IDX_THETA)/sub_;
        Eigen::VectorXd xn = x;
        for (int s = 0; s < sub_; ++s) {
            xn = advanceSubstep(xn, u, h, nullptr);
            const double z = xn(2);
            out[4*s+0] = zFloor() - z;
            out[4*s+1] = z - zCeil();
            out[4*s+2] = wallClear() - pillarSdf(xn.head(3));
            out[4*s+3] = rSepMin() - (xn.head(3) - pred_->predictPos(xn(IDX_DT))).norm();
        }
    }

    Vector<Scalar> f(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        return stepD(x.template cast<double>(), u.template cast<double>()).template cast<Scalar>();
    }
    Matrix<Scalar> fx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureJac(x.template cast<double>(), u.template cast<double>());
        return Jx_.template cast<Scalar>();
    }
    Matrix<Scalar> fu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureJac(x.template cast<double>(), u.template cast<double>());
        return Ju_.template cast<Scalar>();
    }

private:
    /// STRUCTURED substep Jacobian. The physical 13-block is autonomous (no
    /// augmented state enters xdot_impl), so this is block-triangular: the
    /// expensive top-left block comes from the base's exact rk4_jac_impl (which
    /// folds in the quaternion-normalization Jacobian), and only the cheap
    /// augmented rows are finite-differenced, over the 17 columns they can
    /// actually depend on.
    void substepJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double h,
                         Eigen::MatrixXd& Jx, Eigen::MatrixXd& Ju) const {
        Jx = Eigen::MatrixXd::Zero(NX, NX);
        Ju = Eigen::MatrixXd::Zero(NX, NU);
        const auto j = this->rk4_jac_impl(x.head(NX_PHYS), u.head(4), h, Eigen::Vector3d::Zero());
        Jx.block(0,0,NX_PHYS,NX_PHYS) = j.dxdx;
        Ju.block(0,0,NX_PHYS,4) = j.dxdu;
        Ju.block(0,IDX_THETA,NX_PHYS,1) = j.dxdT / sub_;

        static const int cols[] = {0,1,2,6,7,8,9,IDX_DT,IDX_GP,IDX_GY,
                                   IDX_PH,IDX_PH+1,IDX_PH+2,IDX_VH,IDX_VH+1,IDX_VH+2,IDX_S};
        const double eps = 1e-6;
        Eigen::VectorXd ap(NX), am(NX);
        for (int c = 0; c < (int)(sizeof(cols)/sizeof(cols[0])); ++c) {
            const int i = cols[c];
            Eigen::VectorXd xp = x, xm = x; xp(i) += eps; xm(i) -= eps;
            augSubstep(xp, u, h, ap); augSubstep(xm, u, h, am);
            for (int r = IDX_AUG0; r < NX; ++r) Jx(r,i) = (ap(r)-am(r))/(2.0*eps);
        }
        Jx(IDX_ZG,IDX_ZG) = 1.0; Jx(IDX_XG,IDX_XG) = 1.0;
        for (int i : {IDX_THETA, IDX_GPR, IDX_GYR}) {
            Eigen::VectorXd up = u, um = u;
            const double e = (i == IDX_THETA) ? eps*sub_ : eps;
            up(i) += e; um(i) -= e;
            augSubstep(x, up, (i==IDX_THETA)?(up(IDX_THETA)/sub_):h, ap);
            augSubstep(x, um, (i==IDX_THETA)?(um(IDX_THETA)/sub_):h, am);
            for (int r = IDX_AUG0; r < NX; ++r) Ju(r,i) = (ap(r)-am(r))/(2.0*e);
        }
    }
    void ensureJac(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        if (cache_valid_ && x.isApprox(cx_) && u.isApprox(cu_)) return;
        const double h = u(IDX_THETA)/sub_;
        Jx_ = Eigen::MatrixXd::Identity(NX, NX);
        Ju_ = Eigen::MatrixXd::Zero(NX, NU);
        Eigen::MatrixXd Jk, Juk;
        Eigen::VectorXd xk = x;
        for (int s = 0; s < sub_; ++s) {
            substepJacobian(xk, u, h, Jk, Juk);
            Ju_ = Jk*Ju_ + Juk;
            Jx_ = Jk*Jx_;
            xk = advanceSubstep(xk, u, h, nullptr);
        }
        cx_ = x; cu_ = u; cache_valid_ = true;
    }
    std::shared_ptr<ConstantAccelPredictor> pred_;
    int sub_;
    mutable Eigen::MatrixXd Jx_, Ju_;
    mutable Eigen::VectorXd cx_, cu_;
    mutable bool cache_valid_ = false;
};

// ── Cost. STL supplies the custody deadline; a Gauss-Newton running tracking
//    model supplies the stabilizing MPC cost-to-go between terminal nodes. ─────
inline Eigen::VectorXd stageResiduals(const Eigen::VectorXd& x,
                                      const Eigen::VectorXd& u) {
    Eigen::VectorXd r = Eigen::VectorXd::Zero(19);
    const double th = std::max(u(IDX_THETA), 1e-8);
    const double sr_ctrl = std::sqrt(std::max(0.0, rCtrl())*th);
    const double sr_mom = std::sqrt(std::max(0.0, rMom())*th);
    const double sr_gim = std::sqrt(std::max(0.0, rGim())*th);
    r(0) = sr_ctrl*(u(IDX_FZ) - hoverThrust());
    r.segment<3>(1) = sr_mom*u.segment<3>(1);
    r(4) = sr_gim*u(IDX_GPR);
    r(5) = sr_gim*u(IDX_GYR);
    r(6) = std::sqrt(std::max(0.0, rTheta()))*(u(IDX_THETA) - dtInit());

    const double sr_v = std::sqrt(std::max(0.0, wRunVR())*th);
    const double sr_r = std::sqrt(std::max(0.0, wRunR())*th);
    const double sr_z = std::sqrt(std::max(0.0, wRunZ())*th);
    const double sr_vis = std::sqrt(std::max(0.0, wRunVis())*th);
    const double sr_att = std::sqrt(std::max(0.0, wRunAtt())*th);
    const double sr_om = std::sqrt(std::max(0.0, wRunOm())*th);
    r.segment<3>(7) = sr_v*(x.segment<3>(3) - x.segment<3>(IDX_VH));
    const double range = (x.segment<3>(IDX_PH) - x.head(3)).norm();
    r(10) = sr_r*(range - 0.5*(rFollowLo() + rFollowHi()));
    r(11) = sr_z*(x(2) - zSafe());
    r(12) = sr_vis*posPart(rhoTarget() - visTerms(x).rho_mode);
    r(13) = sr_att*x(7);
    r(14) = sr_att*x(8);
    r.segment<3>(15) = sr_om*x.segment<3>(10);
    const double visible = fixGate(visTerms(x).rho_see);
    const double speed_cap = visible*vFast() + (1.0-visible)*vSlow();
    r(18) = std::sqrt(std::max(0.0, wRunSpeed())*th)
          * posPart(x.segment<3>(3).norm() - speed_cap);
    return r;
}

template <typename Scalar>
class PtStageCost : public StageCostBase<Scalar> {
public:
    Scalar q(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        return Scalar(stageResiduals(x.template cast<double>(),
                                     u.template cast<double>()).squaredNorm());
    }
    Vector<Scalar> qx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureCache(x.template cast<double>(), u.template cast<double>());
        return (2.0*Jx_.transpose()*r_).template cast<Scalar>(); }
    Vector<Scalar> qu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureCache(x.template cast<double>(), u.template cast<double>());
        return (2.0*Ju_.transpose()*r_).template cast<Scalar>(); }
    Matrix<Scalar> qxx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureCache(x.template cast<double>(), u.template cast<double>());
        return (2.0*Jx_.transpose()*Jx_).template cast<Scalar>(); }
    Matrix<Scalar> quu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureCache(x.template cast<double>(), u.template cast<double>());
        return (2.0*Ju_.transpose()*Ju_).template cast<Scalar>(); }
    Matrix<Scalar> qxu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        ensureCache(x.template cast<double>(), u.template cast<double>());
        return (2.0*Jx_.transpose()*Ju_).template cast<Scalar>(); }
private:
    void ensureCache(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        if (cache_valid_ && cx_.size() == x.size() && cu_.size() == u.size()
            && (cx_ - x).cwiseAbs().maxCoeff() < 1e-14
            && (cu_ - u).cwiseAbs().maxCoeff() < 1e-14)
            return;
        r_ = stageResiduals(x, u);
        Jx_ = Eigen::MatrixXd::Zero(r_.size(), x.size());
        Ju_ = Eigen::MatrixXd::Zero(r_.size(), u.size());
        constexpr double h = 1e-6;
        for (int i = 0; i < x.size(); ++i) {
            Eigen::VectorXd xp = x, xm = x;
            xp(i) += h; xm(i) -= h;
            Jx_.col(i) = (stageResiduals(xp, u) - stageResiduals(xm, u))/(2.0*h);
        }
        for (int i = 0; i < u.size(); ++i) {
            Eigen::VectorXd up = u, um = u;
            up(i) += h; um(i) -= h;
            Ju_.col(i) = (stageResiduals(x, up) - stageResiduals(x, um))/(2.0*h);
        }
        cx_ = x; cu_ = u; cache_valid_ = true;
    }
    mutable Eigen::VectorXd r_, cx_, cu_;
    mutable Eigen::MatrixXd Jx_, Ju_;
    mutable bool cache_valid_ = false;
};

/// Terminal cost = attitude/rate regularizer + soft STL reward + the RH
/// TERMINAL COST-TO-GO (see wTermS/wTermV above). Without the last part the
/// short-window problem has no objective at all.
inline double terminalCostToGo(const Eigen::VectorXd& x) {
    Eigen::VectorXd r(8);
    const double rng = (x.segment<3>(IDX_PH) - x.head(3)).norm();
    r(0) = std::sqrt(std::max(0.0, wTermS()))
         * posPart(x(IDX_S)) / std::max(deltaBound(), 1e-6);
    r(1) = std::sqrt(std::max(0.0, wTermV()))
         * posPart(rhoTarget() - visTerms(x).rho_mode);
    r(2) = std::sqrt(std::max(0.0, wTermZ())) * posPart(zSafe() - x(2));
    r.segment<3>(3) = std::sqrt(std::max(0.0, wTermVR()))
                    * (x.segment<3>(3) - x.segment<3>(IDX_VH));
    r(6) = std::sqrt(std::max(0.0, wTermR())) * posPart(rng - rFollowHi());
    r(7) = std::sqrt(std::max(0.0, wTermR())) * posPart(rFollowLo() - rng);
    return r.squaredNorm();
}

inline Eigen::VectorXd terminalResiduals(const Eigen::VectorXd& x) {
    Eigen::VectorXd r(8);
    const double rng = (x.segment<3>(IDX_PH) - x.head(3)).norm();
    r(0) = std::sqrt(std::max(0.0, wTermS()))
         * posPart(x(IDX_S)) / std::max(deltaBound(), 1e-6);
    r(1) = std::sqrt(std::max(0.0, wTermV()))
         * posPart(rhoTarget() - visTerms(x).rho_mode);
    r(2) = std::sqrt(std::max(0.0, wTermZ())) * posPart(zSafe() - x(2));
    r.segment<3>(3) = std::sqrt(std::max(0.0, wTermVR()))
                    * (x.segment<3>(3) - x.segment<3>(IDX_VH));
    r(6) = std::sqrt(std::max(0.0, wTermR())) * posPart(rng - rFollowHi());
    r(7) = std::sqrt(std::max(0.0, wTermR())) * posPart(rFollowLo() - rng);
    return r;
}

inline Eigen::MatrixXd terminalResidualJacobian(const Eigen::VectorXd& x) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(8, NX);
    constexpr double h = 1e-6;
    static const int cols[] = {0,1,2,3,4,5,6,7,8,9,IDX_GP,IDX_GY,
                               IDX_PH,IDX_PH+1,IDX_PH+2,
                               IDX_VH,IDX_VH+1,IDX_VH+2,IDX_S};
    for (int c : cols) {
        Eigen::VectorXd xp = x, xm = x;
        xp(c) += h;
        xm(c) -= h;
        J.col(c) = (terminalResiduals(xp) - terminalResiduals(xm))/(2.0*h);
    }
    return J;
}

template <typename Scalar>
class PtTermCost : public TerminalCostBase<Scalar> {
public:
    Scalar p(const Vector<Scalar>& x) const override {
        const Eigen::VectorXd xd = x.template cast<double>();
        const Eigen::Vector4d q = xd.segment<4>(6);
        return Scalar(wAtt()*(q(1)*q(1)+q(2)*q(2))
                    + wOm()*xd.segment<3>(10).squaredNorm()
                    - wStl()*stlFormulaRho(xd)
                    + terminalCostToGo(xd)); }
    Vector<Scalar> px(const Vector<Scalar>& x) const override {
        const Eigen::VectorXd xd = x.template cast<double>();
        Eigen::VectorXd g = Eigen::VectorXd::Zero(NX);
        g(7) = 2.0*wAtt()*xd(7); g(8) = 2.0*wAtt()*xd(8);
        g.segment<3>(10) = 2.0*wOm()*xd.segment<3>(10);
        double dz, dx; stlFormulaGrad(xd, dz, dx);
        g(IDX_ZG) = -wStl()*dz; g(IDX_XG) = -wStl()*dx;
        const Eigen::VectorXd r = terminalResiduals(xd);
        const Eigen::MatrixXd J = terminalResidualJacobian(xd);
        g += 2.0*J.transpose()*r;
        if (!g.allFinite()) g.setZero();
        return g.template cast<Scalar>(); }
    Matrix<Scalar> pxx(const Vector<Scalar>& x) const override {
        const Eigen::VectorXd xd = x.template cast<double>();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(NX, NX);
        H(7,7) = 2.0*wAtt(); H(8,8) = 2.0*wAtt();
        for (int i = 10; i < 13; ++i) H(i,i) = 2.0*wOm();
        const Eigen::MatrixXd J = terminalResidualJacobian(xd);
        H.noalias() += 2.0*J.transpose()*J;
        return H.template cast<Scalar>(); }
};

// ── Constraints ──────────────────────────────────────────────────────────────
template <typename S> class PtInputCon : public StageConstraintBase<S> { public:
    PtInputCon() { this->constraint_type = ConstraintType::NO; this->dim_c = 6; }
    Vector<S> c(const Vector<S>&, const Vector<S>& u) const override {
        Vector<S> o(6);
        o(0) = S(FMIN) - u(IDX_FZ);          o(1) = u(IDX_FZ) - S(FMAX);
        o(2) = u(IDX_GPR) - S(gRateLim());   o(3) = S(-gRateLim()) - u(IDX_GPR);
        o(4) = u(IDX_GYR) - S(gRateLim());   o(5) = S(-gRateLim()) - u(IDX_GYR);
        return o; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>&) const override {
        return Matrix<S>::Zero(6, x.size()); }
    Matrix<S> cu(const Vector<S>&, const Vector<S>& u) const override {
        Matrix<S> J = Matrix<S>::Zero(6, u.size());
        J(0,IDX_FZ)=S(-1); J(1,IDX_FZ)=S(1);
        J(2,IDX_GPR)=S(1); J(3,IDX_GPR)=S(-1);
        J(4,IDX_GYR)=S(1); J(5,IDX_GYR)=S(-1); return J; }};

template <typename S> class PtMomentCon : public StageConstraintBase<S> { public:
    PtMomentCon() { this->constraint_type = ConstraintType::SOC; this->dim_c = 4; }
    Vector<S> c(const Vector<S>&, const Vector<S>& u) const override {
        Vector<S> cn(4); cn << S(tauMax()), u(1), u(2), u(3); return -cn; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>&) const override {
        return Matrix<S>::Zero(4, x.size()); }
    Matrix<S> cu(const Vector<S>&, const Vector<S>& u) const override {
        Matrix<S> J = Matrix<S>::Zero(4, u.size());
        J(1,1)=J(2,2)=J(3,3)=S(-1); return J; }};

template <typename S> class PtThetaCon : public StageConstraintBase<S> { public:
    PtThetaCon() { this->constraint_type = ConstraintType::NO; this->dim_c = 2; }
    Vector<S> c(const Vector<S>&, const Vector<S>& u) const override {
        Vector<S> o(2); o(0) = u(IDX_THETA) - S(dtMax()); o(1) = S(dtMin()) - u(IDX_THETA); return o; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>&) const override {
        return Matrix<S>::Zero(2, x.size()); }
    Matrix<S> cu(const Vector<S>&, const Vector<S>& u) const override {
        Matrix<S> J = Matrix<S>::Zero(2, u.size());
        J(0,IDX_THETA)=S(1); J(1,IDX_THETA)=S(-1); return J; }};

template <typename S> class PtGimbalAngleCon : public StageConstraintBase<S> { public:
    PtGimbalAngleCon() { this->constraint_type = ConstraintType::NO; this->dim_c = 4; }
    Vector<S> c(const Vector<S>& x, const Vector<S>&) const override {
        Vector<S> o(4);
        o(0)=x(IDX_GP)-S(gpLim()); o(1)=S(-gpLim())-x(IDX_GP);
        o(2)=x(IDX_GY)-S(gyLim()); o(3)=S(-gyLim())-x(IDX_GY); return o; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>&) const override {
        Matrix<S> J = Matrix<S>::Zero(4, x.size());
        J(0,IDX_GP)=S(1); J(1,IDX_GP)=S(-1); J(2,IDX_GY)=S(1); J(3,IDX_GY)=S(-1); return J; }
    Matrix<S> cu(const Vector<S>&, const Vector<S>& u) const override {
        return Matrix<S>::Zero(4, u.size()); }};

/// Gimbal angle box at the terminal node. Also ensures the problem always has
/// at least one terminal constraint: with none at all (soft STL arm, no horizon
/// pin) the solver crashes inside the first backward pass.
template <typename S> class PtGimbalAngleTermCon : public TerminalConstraintBase<S> { public:
    PtGimbalAngleTermCon() { this->constraint_type = ConstraintType::NO; this->dim_cT = 4; }
    Vector<S> cT(const Vector<S>& x) const override {
        Vector<S> o(4);
        o(0)=x(IDX_GP)-S(gpLim()); o(1)=S(-gpLim())-x(IDX_GP);
        o(2)=x(IDX_GY)-S(gyLim()); o(3)=S(-gyLim())-x(IDX_GY); return o; }
    Matrix<S> cTx(const Vector<S>& x) const override {
        Matrix<S> J = Matrix<S>::Zero(4, x.size());
        J(0,IDX_GP)=S(1); J(1,IDX_GP)=S(-1); J(2,IDX_GY)=S(1); J(3,IDX_GY)=S(-1); return J; }};

/// The STL row: -Gamma <= 0 in the boundary map ([CTSTL] p.7).
template <typename S> class PtStlBoundaryCon : public TerminalConstraintBase<S> { public:
    PtStlBoundaryCon() { this->constraint_type = ConstraintType::NO; this->dim_cT = 1; }
    Vector<S> cT(const Vector<S>& x) const override {
        return (Vector<S>(1) << S(-stlFormulaRho(x.template cast<double>()))).finished(); }
    Matrix<S> cTx(const Vector<S>& x) const override {
        double dz, dx; stlFormulaGrad(x.template cast<double>(), dz, dx);
        Matrix<S> J = Matrix<S>::Zero(1, x.size());
        J(0,IDX_ZG) = S(-dz); J(0,IDX_XG) = S(-dx); return J; }};

/// The STC rows: dim_c = N_CH, so each channel keeps its own dual while the
/// interval integral (which computes all channels at once) is swept ONCE.
template <typename S> class PtRegimeCon : public StageConstraintBase<S> {
public:
    /// `eps` is per-channel and already widened to whatever the MEASURED state
    /// satisfies. The regime rows are hard state-dependent inequalities, so a
    /// measured state can violate them outright -- e.g. arriving blind at
    /// |v| = 1.7 m/s against V_SLOW = 1.1 gives chi_1 ~ 4.7 versus a 1e-5 bound.
    /// That made the whole solve infeasible and it returned after ZERO
    /// iterations. Every hard state-dependent row needs this, not just the
    /// geometric safety ones.
    PtRegimeCon(std::shared_ptr<PerceptionDyn<S>> d, const Eigen::Vector3d& eps)
        : dyn_(std::move(d)), eps_(eps) {
        this->constraint_type = ConstraintType::NO; this->dim_c = N_CH; }
    Vector<S> c(const Vector<S>& x, const Vector<S>& u) const override {
        double a[N_CH];
        dyn_->intervalIntegrals(x.template cast<double>(), u.template cast<double>(), a);
        Vector<S> o(N_CH);
        for (int i = 0; i < N_CH; ++i) o(i) = S(a[i] - eps_(i));
        return o; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>& u) const override {
        const Eigen::VectorXd xd = x.template cast<double>(), ud = u.template cast<double>();
        Matrix<S> J = Matrix<S>::Zero(N_CH, x.size());
        const double h = 1e-6; double ap[N_CH], am[N_CH];
        for (int i = 0; i <= IDX_VH+2; ++i) {   // skips s, zeta, xi: no density depends on them
            Eigen::VectorXd xp = xd, xm = xd; xp(i)+=h; xm(i)-=h;
            dyn_->intervalIntegrals(xp, ud, ap); dyn_->intervalIntegrals(xm, ud, am);
            for (int r = 0; r < N_CH; ++r) J(r,i) = S((ap[r]-am[r])/(2.0*h));
        }
        return J; }
    Matrix<S> cu(const Vector<S>& x, const Vector<S>& u) const override {
        const Eigen::VectorXd xd = x.template cast<double>(), ud = u.template cast<double>();
        Matrix<S> J = Matrix<S>::Zero(N_CH, u.size());
        const double h = 1e-6; double ap[N_CH], am[N_CH];
        for (int i = 0; i < u.size(); ++i) {
            Eigen::VectorXd up = ud, um = ud; up(i)+=h; um(i)-=h;
            dyn_->intervalIntegrals(xd, up, ap); dyn_->intervalIntegrals(xd, um, am);
            for (int r = 0; r < N_CH; ++r) J(r,i) = S((ap[r]-am[r])/(2.0*h));
        }
        return J; }
private:
    std::shared_ptr<PerceptionDyn<S>> dyn_; Eigen::Vector3d eps_;
};

template <typename S> class PtSafetyCon : public StageConstraintBase<S> {
public:
    /// `relax` carries per-row bounds already widened to whatever the MEASURED
    /// state satisfies (see relaxedSafetyBounds). Without this a state that has
    /// drifted inside the clearance margin makes the FIRST row infeasible, the
    /// IPM cannot initialize its slacks, the solve returns after ZERO
    /// iterations, and the loop replays a stale plan open-loop until it flies
    /// away. Feasibility restoration, not a softened requirement: the bound is
    /// only ever widened to the current violation and the plan is still pushed
    /// back out because the cost prefers it.
    PtSafetyCon(std::shared_ptr<PerceptionDyn<S>> d, const Eigen::Vector4d& relax)
        : dyn_(std::move(d)), sub_(rk4Sub()), relax_(relax) {
        this->constraint_type = ConstraintType::NO; this->dim_c = 4*rk4Sub(); }
    Vector<S> c(const Vector<S>& x, const Vector<S>& u) const override {
        std::vector<double> o;
        dyn_->safetySubstages(x.template cast<double>(), u.template cast<double>(), o);
        Vector<S> r(4*sub_);
        for (int i = 0; i < 4*sub_; ++i) r(i) = S(o[i] - relax_(i % 4));
        return r; }
    Matrix<S> cx(const Vector<S>& x, const Vector<S>& u) const override {
        const Eigen::VectorXd xd = x.template cast<double>(), ud = u.template cast<double>();
        Matrix<S> J = Matrix<S>::Zero(4*sub_, x.size());
        const double h = 1e-6; std::vector<double> a, b;
        for (int i = 0; i <= IDX_DT; ++i) {          // position + time only
            if (i == IDX_GP || i == IDX_GY) continue;
            Eigen::VectorXd xp = xd, xm = xd; xp(i)+=h; xm(i)-=h;
            dyn_->safetySubstages(xp, ud, a); dyn_->safetySubstages(xm, ud, b);
            for (int r = 0; r < 4*sub_; ++r) J(r,i) = S((a[r]-b[r])/(2.0*h));
        }
        return J; }
    Matrix<S> cu(const Vector<S>& x, const Vector<S>& u) const override {
        const Eigen::VectorXd xd = x.template cast<double>(), ud = u.template cast<double>();
        Matrix<S> J = Matrix<S>::Zero(4*sub_, u.size());
        const double h = 1e-6; std::vector<double> a, b;
        for (int i = 0; i < u.size(); ++i) {
            Eigen::VectorXd up = ud, um = ud; up(i)+=h; um(i)-=h;
            dyn_->safetySubstages(xd, up, a); dyn_->safetySubstages(xd, um, b);
            for (int r = 0; r < 4*sub_; ++r) J(r,i) = S((a[r]-b[r])/(2.0*h));
        }
        return J; }
private:
    std::shared_ptr<PerceptionDyn<S>> dyn_; int sub_;
    Eigen::Vector4d relax_;
};

/// Per-channel regime bound, widened so the interval integral at x0 is feasible.
inline Eigen::Vector3d relaxedRegimeEps(const PerceptionDyn<double>& dyn,
                                        const Eigen::VectorXd& x0,
                                        const Eigen::VectorXd& u0) {
    double a[N_CH];
    dyn.intervalIntegrals(x0, u0, a);
    Eigen::Vector3d e;
    const double slack = envOrQ("PT_RELAX_CHI", 1.15);
    for (int i = 0; i < N_CH; ++i) e(i) = std::max(stcStepEps(), a[i] * slack);
    return e;
}

/// How much each safety row must be widened for x0 to be strictly feasible.
inline Eigen::Vector4d relaxedSafetyBounds(const Eigen::VectorXd& x0,
                                           const ConstantAccelPredictor& pred) {
    const double eps = envOrQ("PT_RELAX_EPS", 0.02);
    Eigen::Vector4d r = Eigen::Vector4d::Zero();
    const double z = x0(2);
    r(0) = std::max(0.0, (zFloor() - z) + eps);
    r(1) = std::max(0.0, (z - zCeil()) + eps);
    r(2) = std::max(0.0, (wallClear() - pillarSdf(x0.head(3))) + eps);
    r(3) = std::max(0.0, (rSepMin() - (x0.head(3) - pred.predictPos(0.0)).norm()) + eps);
    return r;
}

inline Param getSolverParams() {
    Param p;
    // Deliberately minimal: leave max_inner_iter / max_step_iter / mu at the
    // solver defaults, exactly as the landing OCPs do. Overriding them here made
    // the solver return after ZERO iterations.
    p.reg1_min = envOrQ("PT_REG1", 1e-3);
    p.reg2_min = envOrQ("PT_REG2", 1.0);
    p.mu_mul   = 0.1;
    p.rho      = envOrQ("PT_RHO", 5.0);
    p.rhoT     = envOrQ("PT_RHOT", 5.0);
    p.rho_mul  = 10.0;
    p.tolerance = envOrQ("PT_TOL", 1e-6);
    p.max_iter  = envOrInt("PT_MAX_ITER", 30);   // realtime budget
    // Console I/O perturbs solve timing in the realtime loop.
    p.verbose = envOrInt("PT_SOLVER_VERBOSE", 0) != 0;
    p.is_quaternion_in_state = false;
    p.use_ddp_terms = false;
    return p;
}

inline void clampWarmControl(Eigen::VectorXd& u) {
    if (u.size() < NU) { u = Eigen::VectorXd::Zero(NU); u(IDX_FZ) = hoverThrust(); }
    u(IDX_FZ) = std::clamp(u(IDX_FZ), FMIN, FMAX);
    for (int i = 1; i < 4; ++i) u(i) = std::clamp(u(i), -tauMax(), tauMax());
    u(IDX_THETA) = std::clamp(u(IDX_THETA), dtMin(), dtMax());
    u(IDX_GPR) = std::clamp(u(IDX_GPR), -gRateLim(), gRateLim());
    u(IDX_GYR) = std::clamp(u(IDX_GYR), -gRateLim(), gRateLim());
    if (!u.allFinite()) { u.setZero(); u(IDX_FZ) = hoverThrust(); u(IDX_THETA) = dtInit(); }
}

/// Reject physically implausible plans so PlannerCore restores the warm start
/// instead of executing them. The landing OCPs carry this and the port dropped
/// it; without it an unconverged solve returns a finite-but-wild trajectory,
/// `success` is set purely on trajectory SIZE, and the loop executes garbage --
/// which is how a tracking failure turns into a 60 m/s runaway.
inline bool plausibleTrajectory(const std::vector<Eigen::VectorXd>& X,
                                const std::vector<Eigen::VectorXd>& U) {
    const double v_lim = envOrQ("PT_PLAUS_V", 6.0);       // m/s
    const double w_lim = envOrQ("PT_PLAUS_W", 30.0);      // rad/s
    const double z_lo  = zFloor() - envOrQ("PT_PLAUS_Z_SLACK", 0.5);
    const double z_hi  = zCeil()  + envOrQ("PT_PLAUS_Z_SLACK", 0.5);
    for (const auto& x : X) {
        if (x.size() < NX_PHYS) return false;
        if (x.segment<3>(3).norm() > v_lim) return false;
        if (x.segment<3>(10).norm() > w_lim) return false;
        if (x(2) < z_lo || x(2) > z_hi) return false;
    }
    for (const auto& u : U) if (u.size() >= 4 && u.segment<3>(1).norm() > 4.0 * tauMax())
        return false;
    return true;
}

inline bool finiteTrajectory(const std::vector<Eigen::VectorXd>& X,
                             const std::vector<Eigen::VectorXd>& U) {
    for (const auto& x : X) if (!x.allFinite()) return false;
    for (const auto& u : U) if (!u.allFinite()) return false;
    return true;
}

/// Build the OCP for one RH solve.
///
/// x0_full carries the measured physical state; the augmented block is set here
/// per the three rebasing policies documented at the top of this file.
inline std::shared_ptr<OptimalControlProblem<double>> create(
        const Eigen::VectorXd& x0_in,
        const std::vector<Eigen::VectorXd>& prev_U,
        const std::vector<Eigen::VectorXd>& prev_X,
        const std::vector<Eigen::MatrixXd>& /*prev_K*/,
        std::shared_ptr<ConstantAccelPredictor> pred,
        double staleness_init) {
    const int N = horizonN();
    auto dyn = std::make_shared<PerceptionDyn<double>>(pred);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(NX);
    const int n_copy = std::min<int>(x0_in.size(), NX);
    x0.head(n_copy) = x0_in.head(n_copy);
    if (x0.segment<4>(6).norm() < 1e-6) x0(6) = 1.0;

    // ── the three rebasing policies ─────────────────────────────────────────
    x0(IDX_DT) = 0.0;                              // rebase horizon time
    x0.segment<3>(IDX_PH) = pred->predictPos(0.0); // re-init from the fresh or
    x0.segment<3>(IDX_VH) = pred->predictVel(0.0); //   coasted target estimate
    x0(IDX_S)  = std::max(0.0, staleness_init);    // CARRY, from the tracker
    x0(IDX_ZG) = 0.0;                              // monitor accumulators RESET
    x0(IDX_XG) = 0.0;                              //   (eta(0)=1 -> zeta(0)=0)

    const Eigen::Vector4d relax = relaxedSafetyBounds(x0, *pred);
    Eigen::VectorXd u_probe = Eigen::VectorXd::Zero(NU);
    u_probe(IDX_FZ) = std::clamp(hoverThrust(), FMIN, FMAX);
    u_probe(IDX_THETA) = std::clamp(dtInit(), dtMin(), dtMax());
    const Eigen::Vector3d chi_eps = relaxedRegimeEps(*dyn, x0, u_probe);
    auto problem = std::make_shared<OptimalControlProblem<double>>(N);
    for (int k = 0; k < N; ++k) {
        problem->setStageDynamics(k, dyn);
        problem->setStageCost(k, std::make_shared<PtStageCost<double>>());
        problem->addStageConstraint(k, std::make_shared<PtInputCon<double>>());
        if (!envOrInt("PT_NO_MOMENT", 0))
            problem->addStageConstraint(k, std::make_shared<PtMomentCon<double>>());
        problem->addStageConstraint(k, std::make_shared<PtThetaCon<double>>());
        if (enableGimbal() && !envOrInt("PT_NO_GIMBAL", 0))
            problem->addStageConstraint(k, std::make_shared<PtGimbalAngleCon<double>>());
        if (!envOrInt("PT_NO_SAFETY", 0))
            problem->addStageConstraint(k, std::make_shared<PtSafetyCon<double>>(dyn, relax));
        if (!envOrInt("PT_NO_REGIME", 0))
            problem->addStageConstraint(k, std::make_shared<PtRegimeCon<double>>(dyn, chi_eps));
    }
    problem->setTerminalCost(std::make_shared<PtTermCost<double>>());
    if (enableGimbal())
        problem->addTerminalConstraint(std::make_shared<PtGimbalAngleTermCon<double>>());
    if (envOrInt("PT_HARD_STL", 0))
        problem->addTerminalConstraint(std::make_shared<PtStlBoundaryCon<double>>());
    problem->setInitialState(0, x0);

    // Warm start: shift-and-rebase. prev_X arrives already shifted by
    // QuadrotorMPC, so only the augmented block needs re-basing; the monitor
    // states must be zeroed rather than carried (see the header note).
    Eigen::VectorXd u_hover = Eigen::VectorXd::Zero(NU);
    u_hover(IDX_FZ) = std::clamp(hoverThrust(), FMIN, FMAX);
    u_hover(IDX_THETA) = std::clamp(dtInit(), dtMin(), dtMax());

    std::vector<Eigen::VectorXd> U(N, u_hover);
    for (int k = 0; k < N && k < (int)prev_U.size(); ++k) {
        Eigen::VectorXd u = prev_U[k];
        if (u.size() != NU) u = u_hover;
        clampWarmControl(u);
        U[k] = u;
    }
    Eigen::VectorXd xg = x0;
    for (int k = 0; k < N; ++k) {
        problem->setInitialControl(k, U[k]);
        xg = dyn->stepD(xg, U[k]);
        problem->setInitialState(k + 1, xg);
    }
    (void)prev_X;
    return problem;
}

inline OCPDescriptor descriptor() {
    OCPDescriptor d;
    d.name = "perception_track";
    d.dt = envOrQ("PT_OCP_DT", 0.05);
    d.default_n_replay = envOrInt("PT_N_REPLAY", 4);
    d.default_mass_kg = massKg();
    d.warm_start = OCPDescriptor::WarmStart::Feedback;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.drone_odom_mode = OCPDescriptor::DroneOdomMode::Absolute;
    d.variable_dt = true;
    d.state_dim = IDX_DT;      // 15 -> gimbal angles are interpolated by the replayer
    d.control_dim = NU;
    d.needs_target_trajectory = false;
    d.skip_trajectory_validation = true;   // augmented state is not a pose
    d.state_names  = {"rx","ry","rz","vx","vy","vz","qw","qx","qy","qz",
                      "wx","wy","wz","gp","gy"};
    d.control_names = {"fz","Mx","My","Mz","theta","gp_rate","gy_rate"};

    d.getSolverParams = getSolverParams;
    d.sanitize_warm_control = [](Eigen::VectorXd& u) { clampWarmControl(u); };
    d.post_process_result = [](SolverResult& r, const TargetSnapshot&) {
        if (r.success && (!finiteTrajectory(r.state_trajectory, r.control_trajectory)
                       || !plausibleTrajectory(r.state_trajectory, r.control_trajectory)))
            r.success = false;
    };
    d.prepare_extra = [](const PlannerConfig&, double now_sec, const TargetSnapshot& t) {
        PerceptionTrackExtra ex;
        ex.predictor = std::make_shared<ConstantAccelPredictor>();
        // The target topics represent camera observations. When they go stale,
        // coast the last observation to the current solve time instead of
        // pretending it was observed again or jumping immediately to Delta.
        double observation_stamp = 0.0;
        if (t.odom_stamp_sec > 0.0 && t.accel_stamp_sec > 0.0)
            observation_stamp = std::min(t.odom_stamp_sec, t.accel_stamp_sec);
        else
            observation_stamp = std::max(t.odom_stamp_sec, t.accel_stamp_sec);
        const double age = observation_stamp > 0.0
            ? std::max(0.0, now_sec - observation_stamp)
            : deltaBound();
        const Eigen::Vector3d p_now =
            t.position + age*t.velocity + 0.5*age*age*t.acceleration;
        const Eigen::Vector3d v_now = t.velocity + age*t.acceleration;
        ex.predictor->setState(p_now, v_now, t.acceleration);
        ex.staleness_init = age;
        return std::any(ex);
    };
    d.create = [](const OCPCreateArgs& a) {
        PerceptionTrackExtra ex;
        if (a.extra.has_value()) {
            try { ex = std::any_cast<PerceptionTrackExtra>(a.extra); }
            catch (const std::bad_any_cast&) {}
        }
        auto pred = ex.predictor;
        if (!pred) {
            pred = std::make_shared<ConstantAccelPredictor>();
            pred->setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), a.target_accel);
        }
        return create(a.current_state, a.prev_U, a.prev_X, a.prev_K, pred, ex.staleness_init);
    };
    return d;
}

}  // namespace PerceptionTrackOCP
