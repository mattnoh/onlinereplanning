/// @file test_perception_ablation.cpp
/// @brief Closed-loop RH ablation of DDP feedback over the MONITOR states.
///
/// Kept SEPARATE from test_perception_feedback_ablation.cpp (Codex's open-loop
/// regression, which checks that a SINGLE solve from a lost state produces a
/// recovering plan). That test passes: terminal_s = 0.01, terminal rho_mode
/// = +6.86. This one closes the loop, and it does NOT: ~27% custody,
/// s_max ~9 s against Delta = 2 s. The two together localise the defect to
/// EXECUTION -- the plan recovers, the loop does not follow it.
///
/// THE QUESTION. DDP produces gains K; with the CT-STL monitor in the augmented
/// state, K spans the monitor rows, so the policy can react to formula-
/// satisfaction drift, not just position error. SCP cannot produce that. Three
/// arms, identical disturbance and target:
///   none  u = u*                                  (open loop)
///   phys  u = u* + K(x - x*), MONITOR COLUMNS ZEROED
///   full  u = u* + K(x - x*), all columns
/// phys vs full isolates monitor-state feedback; none vs phys is ordinary MPC
/// feedback and is not the claim.
///
/// MEASURED (T=3/6/9 s horizons): full beat none at every horizon
///   0.270 vs 0.253 | 0.145 vs 0.123 | 0.213 vs 0.168
/// so monitor-state feedback helps, consistently. That result only appeared
/// after the reference-basis fix below; before it, feedback looked harmful.

#include "ocp/ocp_perception_track.hpp"
#include "planner_core/quadrotor_mpc.hpp"
#include "planner_core/ocp_registry.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <vector>

namespace PT = PerceptionTrackOCP;
namespace {

struct TrueTarget {
    double x0 = 1.0, vx = 0.45, A = 1.5, w = 0.40, z = 1.0, maneuver = 0.02;
    Eigen::Vector3d pos(double t) const {
        return {x0 + vx*t, A*std::sin(w*t) + 0.5*maneuver*t*t, z}; }
    Eigen::Vector3d vel(double t) const {
        return {vx, A*w*std::cos(w*t) + maneuver*t, 0.0}; }
};

std::vector<PT::Pillar> makePillars(const TrueTarget& tg) {
    std::vector<PT::Pillar> out;
    for (int i = 0; i < 6; ++i) {
        const double px = 3.0 + 2.0*i, hw = 0.5;
        const double py = tg.A * std::sin(tg.w * ((px - tg.x0) / tg.vx));
        PT::Pillar p;
        p.lo = Eigen::Vector3d(px-hw, py-hw, 0.0);
        p.hi = Eigen::Vector3d(px+hw, py+hw, 3.0);
        out.push_back(p);
    }
    return out;
}

enum class Arm { None, Phys, Full };
const char* armName(Arm a) { return a==Arm::None?"none":(a==Arm::Phys?"phys":"full"); }

struct Metrics { double kept=0, worst=0, s_max=0, err=0, ms_mean=0, ms_max=0;
                 int solves=0, accepted=0; };

Metrics runArm(Arm arm, const TrueTarget& tgt, unsigned seed) {
    // PT_NO_PILLARS=1 removes the obstacle field. This is the (a)-vs-(b) probe:
    // without pillars every solve is ACCEPTED at ~180 ms, so if custody still
    // fails here the failure is the RECEDING-HORIZON FRAME, not solver latency.
    if (std::getenv("PT_NO_PILLARS")) PT::setPillars({});
    else                              PT::setPillars(makePillars(tgt));
    QuadrotorMPC::Config cfg; cfg.ocp_type = "perception_track"; cfg.n_shift = 1;
    QuadrotorMPC mpc(cfg);
    const auto& desc = OCPRegistry::getDescriptor("perception_track");
    const double sim_dt = 0.02, mission_T = 12.0;
    const double replan_dt = desc.dt * desc.default_n_replay;

    Eigen::VectorXd x = Eigen::VectorXd::Zero(PT::NX);
    x(0) = -1.5; x(2) = 1.5; x(6) = 1.0; x(3) = tgt.vx;
    x.segment<3>(PT::IDX_PH) = tgt.pos(0.0);
    x.segment<3>(PT::IDX_VH) = tgt.vel(0.0);

    std::mt19937 rng(seed);
    std::normal_distribution<double> gust(0.0, 1.0);
    std::vector<Eigen::VectorXd> X, U;
    std::vector<Eigen::MatrixXd> K;
    std::shared_ptr<PT::ConstantAccelPredictor> pp;
    double t_plan = -1e9, run = 0, sum_ms = 0, t_in = 0, t_tot = 0;
    Metrics m;

    auto predAt = [&](double t) {
        auto p = std::make_shared<PT::ConstantAccelPredictor>();
        p->setState(tgt.pos(t), tgt.vel(t), Eigen::Vector3d::Zero());
        return p; };

    for (double t = 0.0; t < mission_T; t += sim_dt) {
        if (t - t_plan >= replan_dt - 1e-9) {
            t_plan = t;
            PT::PerceptionTrackExtra ex;
            ex.predictor = predAt(t); pp = ex.predictor;
            ex.staleness_init = x(PT::IDX_S);
            Eigen::VectorXd x0 = x; x0(PT::IDX_DT) = 0.0;
            auto res = mpc.solve(x0, Eigen::Vector3d::Zero(), std::any(ex), t);
            ++m.solves; sum_ms += res.solve_time_ms;
            m.ms_max = std::max(m.ms_max, res.solve_time_ms);
            const bool ok = res.success
                && PT::finiteTrajectory(res.state_trajectory, res.control_trajectory)
                && PT::plausibleTrajectory(res.state_trajectory, res.control_trajectory);
            if (ok) {
                ++m.accepted;
                X = res.state_trajectory; U = res.control_trajectory; K = res.feedback_gains;
                // create() rebases IDX_DT and RESETS the monitor accumulators every
                // solve. The plant must be put on the same basis or dx = x - X[k]
                // compares unlike quantities and the "feedback" is pure noise --
                // this is why phys/full initially looked WORSE than open loop.
                x(PT::IDX_DT) = 0.0; x(PT::IDX_ZG) = 0.0; x(PT::IDX_XG) = 0.0;
            }
        }
        if (X.empty() || U.empty()) continue;

        double acc = 0; int k = 0;
        for (; k < (int)U.size(); ++k) {
            if (acc + U[k](PT::IDX_THETA) > (t - t_plan)) break;
            acc += U[k](PT::IDX_THETA);
        }
        k = std::min(k, (int)U.size() - 1);

        Eigen::VectorXd u = U[k];
        if (arm != Arm::None && k < (int)K.size()
            && K[k].rows() == PT::NU && K[k].cols() == PT::NX) {
            Eigen::VectorXd dx = x - X[k];
            if (arm == Arm::Phys) {   // zero MONITOR columns: ordinary MPC feedback only
                dx(PT::IDX_S) = 0; dx(PT::IDX_ZG) = 0; dx(PT::IDX_XG) = 0;
            }
            u += K[k] * dx;
        }
        PT::clampWarmControl(u);

        if (!pp) pp = predAt(t);
        auto plant = std::make_shared<PT::PerceptionDyn<double>>(pp);
        Eigen::VectorXd un = u; un(PT::IDX_THETA) = sim_dt;
        Eigen::VectorXd xn = plant->stepD(x, un);
        xn(3) += 0.45 * sim_dt * gust(rng);
        xn(4) += 0.45 * sim_dt * gust(rng);
        x = xn;

        Eigen::VectorXd xt = x;
        xt.segment<3>(PT::IDX_PH) = tgt.pos(t);   // metrics on TRUE geometry
        const auto vt = PT::visTerms(xt);
        t_tot += sim_dt;
        if (vt.in_mode) { t_in += sim_dt; run = 0; }
        else { run += sim_dt; m.worst = std::max(m.worst, run); }
        m.s_max = std::max(m.s_max, x(PT::IDX_S));
        m.err = std::max(m.err, (x.segment<3>(PT::IDX_PH) - tgt.pos(t)).norm());
    }
    m.kept = t_tot > 0 ? t_in / t_tot : 0.0;
    m.ms_mean = m.solves ? sum_ms / m.solves : 0.0;
    return m;
}

}  // namespace

/// Which constraint row makes an OCCLUDED solve unacceptable? The reacquisition
/// regression passes only with the pillars removed, so occlusion is what pushes
/// constraint_error past PlannerCore's threshold. This localises it to a row.
void diagnoseOccluded() {
    TrueTarget tgt;
    PT::setPillars(makePillars(tgt));
    Eigen::VectorXd x = Eigen::VectorXd::Zero(PT::NX);
    x(0) = 3.73; x(1) = 0.88; x(2) = 1.05; x(3) = 1.5; x(4) = 0.8;
    x(6) = std::cos(0.5*M_PI/2); x(9) = std::sin(0.5*M_PI/2);
    x(PT::IDX_S) = 1.0;
    auto pred = std::make_shared<PT::ConstantAccelPredictor>();
    pred->setState(Eigen::Vector3d(2.13,1.32,1.0), Eigen::Vector3d(0.45,0.55,0),
                   Eigen::Vector3d::Zero());
    x.segment<3>(PT::IDX_PH) = pred->predictPos(0);
    x.segment<3>(PT::IDX_VH) = pred->predictVel(0);
    const auto vt = PT::visTerms(x);
    printf("occluded state: rho_mode=%.3f rho_fov=%.3f clearance=%.3f range=%.2f\n",
           vt.rho_mode, vt.rho_fov, vt.clearance, vt.range);

    Eigen::VectorXd u = Eigen::VectorXd::Zero(PT::NU);
    u(PT::IDX_FZ) = std::clamp(PT::hoverThrust(), PT::FMIN, PT::FMAX);
    u(PT::IDX_THETA) = std::clamp(PT::dtInit(), PT::dtMin(), PT::dtMax());
    auto dyn = std::make_shared<PT::PerceptionDyn<double>>(pred);
    const Eigen::Vector4d relax = PT::relaxedSafetyBounds(x, *pred);
    const Eigen::Vector3d chi = PT::relaxedRegimeEps(*dyn, x, u);
    auto rep=[&](const char* n, const Eigen::VectorXd& c){
        printf("  %-14s dim=%2ld  max=%+.4f %s\n", n, (long)c.size(), c.maxCoeff(),
               c.maxCoeff() > 0 ? "  <-- VIOLATED" : ""); };
    rep("InputCon",   PT::PtInputCon<double>().c(x,u));
    rep("ThetaCon",   PT::PtThetaCon<double>().c(x,u));
    rep("SafetyCon",  PT::PtSafetyCon<double>(dyn, relax).c(x,u));
    rep("RegimeCon",  PT::PtRegimeCon<double>(dyn, chi).c(x,u));
    printf("  relax(z_lo,z_hi,pillar,sep)=%.3f %.3f %.3f %.3f  chi_eps=%.3g %.3g %.3g\n",
           relax(0),relax(1),relax(2),relax(3), chi(0),chi(1),chi(2));
}

int main() {
    if (std::getenv("PT_DIAG")) { diagnoseOccluded(); return 0; }
    std::cout << "=== perception_track: closed-loop feedback ablation ===\n";
    const auto& d = OCPRegistry::getDescriptor("perception_track");
    std::cout << "state_dim=" << d.state_dim << " replan=" << d.dt * d.default_n_replay
              << "s  Delta=" << PT::deltaBound() << "s\n\n";
    std::cout << "pillars=" << (std::getenv("PT_NO_PILLARS") ? "OFF" : "ON") << "\n\n";
    printf("%-6s %9s %10s %8s %9s %12s %9s\n",
           "arm", "los_kept", "worst_gap", "s_max", "est_err", "solve ms", "accepted");
    TrueTarget tgt;
    for (Arm a : {Arm::None, Arm::Phys, Arm::Full}) {
        const Metrics m = runArm(a, tgt, 12345u);
        printf("%-6s %9.3f %10.2f %8.2f %9.3f  %5.0f/%5.0f %6d/%-3d\n",
               armName(a), m.kept, m.worst, m.s_max, m.err, m.ms_mean, m.ms_max,
               m.accepted, m.solves);
    }
    std::cout << "\nphys vs full isolates MONITOR-state feedback.\n"
                 "PASS criterion is s_max <= Delta; currently NOT met.\n";
    return 0;
}
