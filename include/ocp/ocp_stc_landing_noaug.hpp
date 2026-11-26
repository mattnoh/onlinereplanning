/// @file ocp_stc_landing_noaug.hpp
/// @brief Interval CT-cSTC landing on a moving target ("stc_landing_noaug"), ProxDDP.
///
/// Relative-frame quadrotor landing (model in aligator_ocp.hpp):
///   state   = [13 physical | IDX_DT time since horizon start]
///   control = [fz, Mx, My, Mz | IDX_THETA interval duration]
///
/// State-triggered constraints (staging, line of sight, landing speed/tilt/
/// rate/glideslope) and the always-on tilt/rate limits are enforced per
/// interval as the RK4 integral of their violation density (ctcs_stc::Spec,
/// Form::Density), one row per stage:
///   sum_i w_i integral([-g_i]_+^2 [c_i]_+^2 dt) / eps - 1  <=  0
/// All numbers live in Params; preset() holds the configurations, selected per
/// process by STC_PRESET.
#pragma once
#include "ocp/aligator_ocp.hpp"
#include "parameters.hpp"
#include "planner_core/types.hpp"
#include "target/predictor.hpp"

#include "ctcs_stc/stage_function.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <any>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace StcLandingNoAugOCP {

using aligator_ocp::V;
using aligator_ocp::M;
namespace quad = aligator_ocp::quad;
constexpr int NX = 13, NU = 4, NX_SS = quad::NX, NU_SS = quad::NU, IDX_DT = quad::IDX_T,
              IDX_THETA = quad::IDX_THETA;

// Optimizer tightening inside the physical bounds, applied to intervals from
// CTCS_TIGHTEN_FROM_NODE on. Zero means none.
struct CtcsMargins {
    double trigger = 0.;        // trigger altitudes/radius moved outward [m]
    double geometric = 0.;      // glideslope offset [m]
    double speed = 0.;          // landing speed [m/s]
    double tilt_deg = 0.;       // landing tilt [deg]
    double path_tilt_deg = 0.;  // approach tilt [deg]
    double rate = 0.;           // body rate [rad/s]
    double los = 0.;            // LOS cone distance [m], tapered to an angle at the apex
    double stage_radius = 0.;   // staging radius [m]
    double gs_angle_deg = 0.;   // glideslope half angle [deg]
    double stage_height = 0.;   // staging height [m]
};

// Every number this OCP uses. Presets below hold the configurations.
struct Params {
    // Vehicle (Crazyflie-scale model)
    double mass = 0.027, ixx = 1.66e-5, iyy = 1.66e-5, izz = 2.92e-5, l_arm = 0.046;
    double actuator_fmin = 0.08, actuator_fmax = 0.60;  // per-motor sum limits [N]
    double fmin = 0.12, fmax = 0.52;                    // collective thrust bounds [N]
    // Horizon: intervals, intervals executed per replan, interval duration [s]
    int horizon = 80, nex = 7;
    double th0 = 0.07, thl = 0.05, thh = 0.12;
    // Stage cost
    double stage_w_time = 0.04, r_theta = 0.0, theta_ref = 0.10, r_ctrl_thr = 2.0, r_ctrl_mom = 0.6;
    // Terminal cost
    double term_w_pos_xy = 1e5, term_w_pos_z = 1e4, term_w_vel_xy = 900.0, term_w_vel_z = 900.0;
    double term_w_tilt = 200.0, term_w_om = 50.0, term_vz_ref = 0.0;
    double cost_scale = 1e-2;  // whole objective, for ProxDDP's AL conditioning
    // Physical bounds: approach (always) and landing (below alt_trig)
    double tilt_phase0_deg = 11.0, omega_phase0_max = 0.35, spd_phase0_max = 3.0;
    double tilt_stc_deg = 0.95, alpha_theta_stc = 1.0, omega_stc_max = 0.019, spd_stc_max = 0.75;
    double gs_cone_deg = 15.0, gs_apex_offset = 0.1, los_cone_deg = 30.0;
    // Triggers
    double alt_trig = 0.8;  // landing phase below [m]
    double los_alt_trig = 1.8, los_trigger_scale = 0.25, los_trigger_floor = 0.0;
    double z_stage = 1.8, stage_radius = 1.10, stage_trigger_scale = 0.25, stage_trigger_floor = 0.0;
    // CTCS: each consequence is a dimensionless fraction of its own limit
    // (Elango 2025, Sec. 6) with unit weight, against one per-interval budget:
    //   row = sum_i integral([-g_i]_+^2 [c_i / limit_i]_+^2) / ctcs_step_eps - 1 <= 0
    // Length residuals (glideslope, LOS, staging) are measured in gs_length or the
    // trigger scales.
    double ctcs_step_eps = 3e-6, gs_length = 0.1;
    CtcsMargins margins;
    // Control scaling seen by the solver: u = center + scale .* u~
    double f_scale = 0.05, m_scale = 0.1;
    double max_feedback_du = 0.05;  // feedback warm start: largest accepted correction
    bool hover_seed = false;        // cold seed: hover thrust at th0 instead of the staged PD guess
    aligator_ocp::SolverSettings solver;
};

// ── Presets ───────────────────────────────────────────────────────────────────
//   n80    single horizon, 80 intervals                      = ctcsBase()
//   ic100  the 100-IC study profile (no tightening, terminal 2500/900)
//   sitl   PX4/Gazebo measured-state replanning (all target motions)
inline Params ctcsBase() {
    Params p;
    p.margins.trigger = 0.05;  // m
    p.margins.geometric = 0.02;  // m
    p.margins.speed = 0.05;  // m/s
    p.margins.tilt_deg = 0.2;
    p.margins.path_tilt_deg = 0.5;
    p.margins.rate = 0.005;  // rad/s
    p.margins.los = 0.02;  // m
    p.margins.stage_radius = 0.05;  // m
    p.margins.stage_height = 0.02;  // m
    p.solver.tol = 1e-4;
    p.solver.mu_init = 0.1;
    p.solver.max_iters = 3000;
    return p;
}

inline Params preset(const std::string& name) {
    if (name.empty() || name == "n80") return ctcsBase();

    // The 100-IC study (paper Sec. VII-A, Table II): terminal weights 2500/900,
    // no optimizer tightening (every interval uses the physical rules).
    if (name == "ic100") {
        Params p = ctcsBase();
        p.term_w_pos_xy = p.term_w_pos_z = 2500;
        p.margins = CtcsMargins{};
        return p;
    }

    if (name == "sitl") {
        Params p = ctcsBase();
        p.horizon = 40;
        p.th0 = 0.14;  p.thl = 0.06;  p.thh = 0.24;
        p.stage_w_time = 0.2;
        p.ctcs_step_eps *= 80.0 / 60;
        // physical bounds
        p.tilt_phase0_deg = 6;
        p.omega_phase0_max = 0.12;  // rad/s
        p.spd_phase0_max = 2.0;  // m/s
        p.tilt_stc_deg = 5;
        p.omega_stc_max = 0.15;  // rad/s
        p.spd_stc_max = 1.0;  // m/s
        p.gs_cone_deg = 25;
        p.gs_apex_offset = 0.25;  // m
        p.los_cone_deg = 40;
        p.los_alt_trig = 1.5;  // m
        p.stage_radius = 1.2;  // m
        // wider optimizer margins to absorb tracking error
        p.margins.trigger = 0.2;
        p.margins.geometric = 0.05;
        p.margins.speed = 0.2;
        p.margins.rate = 0.02;
        p.margins.gs_angle_deg = 3;
        p.margins.los = 0.09;
        p.margins.stage_radius = 0.7;
        p.margins.stage_height = 0.12;
        p.solver.max_iters = 200;
        return p;
    }

    throw std::invalid_argument("unknown stc_landing_noaug preset: " + name);
}

// STC_PRESET selects the configuration for this process; STC_TUNE_* override
// individual numbers for tuning sweeps.
inline Params loadParams() {
    const char* n = std::getenv("STC_PRESET");
    Params p = preset(n ? n : "");
    const auto num = [](const char* key, double& v) { if (const char* s = std::getenv(key)) v = std::stod(s); };
    double max_iters = static_cast<double>(p.solver.max_iters);
    num("STC_TUNE_W_TIME", p.stage_w_time);
    double hs = p.hover_seed;
    num("STC_TUNE_HOVER_SEED", hs);
    p.hover_seed = hs != 0;
    num("STC_TUNE_R_THETA", p.r_theta);
    num("STC_TUNE_THETA_REF", p.theta_ref);
    num("STC_TUNE_EPS", p.ctcs_step_eps);
    num("STC_TUNE_MU", p.solver.mu_init);
    num("STC_TUNE_TOL", p.solver.tol);
    num("STC_TUNE_ACCEPT", p.solver.accept_tol);
    num("STC_TUNE_INTERVAL_ACCEPT", p.solver.interval_accept_tol);
    num("STC_TUNE_COST_SCALE", p.cost_scale);
    num("STC_TUNE_MAX_ITERS", max_iters);
    p.solver.max_iters = static_cast<std::size_t>(max_iters);
    double flag = p.solver.nonlinear_rollout;
    num("STC_TUNE_NONLINEAR_ROLLOUT", flag);
    p.solver.nonlinear_rollout = flag != 0;
    flag = p.solver.exact_hessian;
    num("STC_TUNE_EXACT_HESSIAN", flag);
    p.solver.exact_hessian = flag != 0;
    double step = p.solver.step_acceptance;
    num("STC_TUNE_STEP", step);
    p.solver.step_acceptance = static_cast<int>(step);
    flag = p.solver.verbose;
    num("STC_TUNE_VERBOSE", flag);
    p.solver.verbose = flag != 0;
    return p;
}
inline const Params P = loadParams();

inline const double J_SCALE = 1.0 / P.ixx;
inline const double TAU_MAX = P.l_arm * (P.actuator_fmax / 4.0 - P.actuator_fmin / 4.0) * J_SCALE;
inline const Eigen::Matrix3d J_B =
    (Eigen::Matrix3d() << P.ixx * J_SCALE, 0, 0, 0, P.iyy * J_SCALE, 0, 0, 0, P.izz * J_SCALE).finished();
inline const Eigen::Vector3d GRAVITY{0, 0, -9.81};
inline const double HOVER_THRUST = P.mass * 9.81;
inline const double LOS_CONE_TAN = std::tan(P.los_cone_deg * M_PI / 180.0);

inline quad::Body body() { return quad::Body(P.mass, P.ixx, P.iyy, P.izz); }
inline quad::ControlScaling scaling() {
    quad::ControlScaling s;
    s.center << HOVER_THRUST, 0, 0, 0, 0;
    s.scale << P.f_scale, P.m_scale, P.m_scale, P.m_scale, 1.0;
    return s;
}

inline double gsConeTan(const CtcsMargins& m) { return std::tan((P.gs_cone_deg - m.gs_angle_deg) * M_PI / 180.0); }
inline CtcsMargins ctcsMargins(bool tightened) { return tightened ? P.margins : CtcsMargins{}; }
// Trigger activation at the boundary. Triggers weight their consequence by
// the squared depth past the boundary, so with a zero floor a row fades out
// exactly at its trigger. Tightened rows move the trigger outward by the
// trigger margins, so at the PHYSICAL boundary they already carry the depth
// margin/scale. Physical rows (untightened intervals) get that same activation
// as a floor at the physical boundary.
struct TriggerFloors { double landing = 0., los = 0., stage = 0.; };
inline TriggerFloors triggerFloors(bool tightened) {
    if (tightened) return {0., P.los_trigger_floor, P.stage_trigger_floor};
    return {P.margins.trigger, std::max(P.los_trigger_floor, P.margins.trigger / P.los_trigger_scale),
            std::max(P.stage_trigger_floor, P.margins.stage_radius / P.stage_trigger_scale)};
}

// Intervals before this node use the physical rows (tightened=false), integrated
// with CTCS_PHYSICAL_SUBSTEPS RK4 substeps: they have no margin to absorb a
// violation between quadrature points.
constexpr int CTCS_TIGHTEN_FROM_NODE = 4;
constexpr int CTCS_PHYSICAL_SUBSTEPS = 8;

/// How the conditional rules enter the OCP (the 100-IC arms).
enum class Enforcement {
    Interval,  ///< one interval-integral row per stage (the method)
    Node,      ///< nodal [-g]_+ c <= 0 per implication (Szmuk 2018)
    Penalty    ///< the same interval integral as a stage cost, no row
};

using ConstantAccelPredictor = target_models::ConstantAccelPredictor;
struct StcLandingNoAugExtra { std::shared_ptr<ConstantAccelPredictor> predictor; };

// ── The rules ─────────────────────────────────────────────────────────────────
inline Eigen::Vector3d boresight(const V& x) {
    const double w = x(6), a = x(7), b = x(8), c = x(9);
    return {2 * (a * c + w * b), 2 * (b * c - w * a), w * w - a * a - b * b + c * c};
}

/// The landing rules with the given tightening, form and interval integration.
/// Weights are divided by the budget so the Density row reads sum w I / eps - 1.
inline ctcs_stc::Spec<double> landingRules(bool tightened, ctcs_stc::Form form, int substeps,
                                           const Eigen::Vector3d& a_target) {
    const auto margins = ctcsMargins(tightened);
    const auto floors = triggerFloors(tightened);
    if (margins.trigger < 0. || margins.trigger >= P.stage_radius || margins.stage_radius < 0. ||
        margins.stage_radius >= P.stage_radius || margins.geometric < 0. || margins.stage_height < 0. ||
        margins.speed < 0. || margins.speed >= P.spd_stc_max || margins.tilt_deg < 0. ||
        margins.tilt_deg >= P.alpha_theta_stc * P.tilt_stc_deg || margins.path_tilt_deg < 0. ||
        margins.path_tilt_deg >= P.tilt_phase0_deg || margins.rate < 0. || margins.rate >= P.omega_stc_max ||
        margins.los < 0. || margins.los / P.gs_apex_offset >= LOS_CONE_TAN || margins.gs_angle_deg < 0. ||
        margins.gs_angle_deg >= P.gs_cone_deg || margins.geometric >= gsConeTan(margins) * P.gs_apex_offset)
        throw std::invalid_argument("invalid CTCS physical tightening margins");

    const bool density = form != ctcs_stc::Form::Original2018;
    ctcs_stc::Spec<double> spec(form, density ? 1.0 : 0.0);
    const double inv_eps = 1.0 / P.ctcs_step_eps;
    const auto zero = [](const V& x) -> V { return V::Zero(x.size()); };
    const auto always = [](const V&) { return -1.0; };
    const auto tilt = [](const V& x) { return x(7) * x(7) + x(8) * x(8); };
    const auto dtilt = [](const V& x) -> V { V g = V::Zero(x.size()); g(7) = 2 * x(7); g(8) = 2 * x(8); return g; };
    const auto omega = [](const V& x) { return x.segment<3>(10).squaredNorm(); };
    const auto domega = [](const V& x) -> V { V g = V::Zero(x.size()); g.segment<3>(10) = 2 * x.segment<3>(10); return g; };
    const auto tiltLimit = [](double degrees) { return std::pow(std::sin(0.5 * degrees * M_PI / 180.0), 2); };
    const auto add = [&](const std::string& name, std::function<double(const V&)> trigger,
                         std::function<V(const V&)> dtrigger, std::function<double(const V&)> consequence,
                         std::function<V(const V&)> dconsequence, double scale) {
        spec.add(name).triggerWhenNegative(std::move(trigger), std::move(dtrigger))
            .consequence([consequence](const V& x, const V&) { return consequence(x); },
                         [dconsequence](const V& x, const V&) { return dconsequence(x); },
                         [](const V&, const V& u) -> V { return V::Zero(u.size()); })
            .scale(scale)
            .weight(density ? inv_eps : 1.0);
    };

    const double looseTilt = tiltLimit(P.tilt_phase0_deg - margins.path_tilt_deg);
    const double looseOmega2 = std::pow(P.omega_phase0_max - margins.rate, 2);
    add("loose_tilt", always, zero, [=](const V& x) { return tilt(x) - looseTilt; }, dtilt, 1.0 / looseTilt);
    add("loose_omega", always, zero, [=](const V& x) { return omega(x) - looseOmega2; }, domega, 1.0 / looseOmega2);

    const double r_stage = P.stage_radius - margins.stage_radius;
    add("staging",
        [=](const V& x) {
            const double m = (std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12) - r_stage) / P.stage_trigger_scale;
            return m > 0 ? -(floors.stage + m) : 0.0;
        },
        [=](const V& x) -> V {
            V g = V::Zero(x.size());
            const double r = std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12);
            if (r > r_stage) { g(0) = -x(0) / (r * P.stage_trigger_scale); g(1) = -x(1) / (r * P.stage_trigger_scale); }
            return g;
        },
        [=](const V& x) { return P.z_stage + margins.stage_height - x(2); },
        [](const V& x) -> V { V g = V::Zero(x.size()); g(2) = -1; return g; }, 1.0 / P.stage_trigger_scale);

    const double z_los = P.los_alt_trig + margins.trigger, los_tan = LOS_CONE_TAN;
    add("los",
        [=](const V& x) {
            const double m = (z_los - x(2)) / P.los_trigger_scale;
            return m > 0 ? -(floors.los + m) : 0.0;
        },
        [=](const V& x) -> V { V g = V::Zero(x.size()); if (x(2) < z_los) g(2) = 1.0 / P.los_trigger_scale; return g; },
        // Distance margin tapered to an angular margin at the apex.
        [=](const V& x) {
            const double axial = x.head<3>().dot(boresight(x));
            const double lateral2 = x.head<3>().squaredNorm() - axial * axial;
            return std::sqrt(std::max(0.0, lateral2) + 1e-12) - los_tan * axial +
                   margins.los * std::tanh(axial / P.gs_apex_offset);
        },
        [=](const V& x) -> V {
            const double w = x(6), a = x(7), b = x(8), c = x(9);
            const Eigen::Vector3d axis = boresight(x);
            const double axial = x.head<3>().dot(axis);
            V da = V::Zero(x.size());
            da.head<3>() = axis;
            Eigen::Matrix<double, 3, 4> db;
            db << 2 * b, 2 * c, 2 * w, 2 * a, -2 * a, -2 * w, 2 * c, 2 * b, 2 * w, -2 * a, -2 * b, 2 * c;
            da.segment<4>(6) = db.transpose() * x.head<3>();
            const double lateral2 = x.head<3>().squaredNorm() - axial * axial;
            const double taper = std::tanh(axial / P.gs_apex_offset);
            V g = (-los_tan + margins.los / P.gs_apex_offset * (1 - taper * taper)) * da;
            if (lateral2 > 0) {
                V dl = -2 * axial * da;
                dl.head<3>() += 2 * x.head<3>();
                g += dl / (2 * std::sqrt(lateral2 + 1e-12));
            }
            return g;
        },
        1.0 / P.los_trigger_scale);

    const double z_land = P.alt_trig + margins.trigger;
    const auto land = [=](const V& x) { const double d = z_land - x(2); return d > 0 ? -(floors.landing + d) : -d; };
    const auto dland = [](const V& x) -> V { V g = V::Zero(x.size()); g(2) = 1; return g; };
    const double v2 = std::pow(P.spd_stc_max - margins.speed, 2);
    add("landing_speed", land, dland, [=](const V& x) { return x.segment<3>(3).squaredNorm() - v2; },
        [](const V& x) -> V { V g = V::Zero(x.size()); g.segment<3>(3) = 2 * x.segment<3>(3); return g; },
        1.0 / v2);
    const double landTilt = tiltLimit(P.alpha_theta_stc * P.tilt_stc_deg - margins.tilt_deg);
    add("landing_tilt", land, dland, [=](const V& x) { return tilt(x) - landTilt; }, dtilt, 1.0 / landTilt);
    const double om2 = std::pow(P.omega_stc_max - margins.rate, 2);
    add("landing_omega", land, dland, [=](const V& x) { return omega(x) - om2; }, domega, 1.0 / om2);
    const double gs_tan = gsConeTan(margins);
    add("landing_glideslope", land, dland,
        [=](const V& x) {
            return std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12) - gs_tan * (x(2) + P.gs_apex_offset) + margins.geometric;
        },
        [=](const V& x) -> V {
            V g = V::Zero(x.size());
            const double r = std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12);
            g(0) = x(0) / r; g(1) = x(1) / r; g(2) = -gs_tan;
            return g;
        },
        1.0 / P.gs_length);

    if (density) {
        const auto b = body();
        const auto s = scaling();
        ctcs_stc::IntervalDynamics<double> dyn;
        dyn.xdot = [b, s, a_target](const V& x, const V& u, double) {
            V f = V::Zero(NX_SS);
            f.head(NX) = quad::xdot(b, x.head(NX), s.physical(u).head(NU), a_target);
            f(IDX_DT) = 1.0;
            return f;
        };
        dyn.fx = [b, s](const V& x, const V& u, double) {
            M a = M::Zero(NX_SS, NX_SS);
            a.topLeftCorner(NX, NX) = quad::A(b, x.head(NX), s.physical(u).head(NU));
            return a;
        };
        dyn.fu = [b, s](const V& x, const V&, double) {
            M m = M::Zero(NX_SS, NU_SS);
            m.topLeftCorner(NX, NU) = quad::B(b, x.head(NX)) * s.scale.head(NU).asDiagonal();
            return m;
        };
        dyn.idx_theta = IDX_THETA;
        dyn.substeps = substeps;
        spec.setIntervalDynamics(dyn);
    }
    return spec;
}

// ── Costs ─────────────────────────────────────────────────────────────────────
/// cost_scale (w_t theta + r_theta (theta - theta_ref)^2 + r_thr (f - mg)^2 + r_mom |M|^2)
/// [+ w_pen * interval integral of the rules, Enforcement::Penalty only]
struct StageCost : aligator::CostAbstractTpl<double> {
    using Base = aligator::CostAbstractTpl<double>;
    using Data = aligator::CostDataAbstractTpl<double>;
    quad::ControlScaling s = scaling();
    std::shared_ptr<const ctcs_stc::Spec<double>> penalty;  // optional
    double w_pen = 0.0;
    explicit StageCost(const aligator_ocp::Space& sp) : Base(sp, NU_SS) {}
    void evaluate(const ConstVectorRef& x, const ConstVectorRef& uu, Data& d) const override {
        const V u = s.physical(uu);
        d.value_ = P.cost_scale * (P.stage_w_time * u(IDX_THETA) + P.r_theta * std::pow(u(IDX_THETA) - P.theta_ref, 2) +
                                   P.r_ctrl_thr * std::pow(u(0) - HOVER_THRUST, 2) + P.r_ctrl_mom * u.segment<3>(1).squaredNorm());
        if (penalty) d.value_ += w_pen * (penalty->c(V(x), V(uu))(0) + 1.0);  // w_pen sum w I / eps
    }
    void computeGradients(const ConstVectorRef& x, const ConstVectorRef& uu, Data& d) const override {
        const V u = s.physical(uu);
        d.Lx_.setZero();
        d.Lu_.setZero();
        d.Lu_(0) = 2 * P.r_ctrl_thr * (u(0) - HOVER_THRUST);
        d.Lu_.segment<3>(1) = 2 * P.r_ctrl_mom * u.segment<3>(1);
        d.Lu_(IDX_THETA) = P.stage_w_time + 2 * P.r_theta * (u(IDX_THETA) - P.theta_ref);
        d.Lu_ = P.cost_scale * d.Lu_.cwiseProduct(s.scale);
        if (penalty) {
            d.Lx_ += w_pen * penalty->cx(V(x), V(uu)).row(0).transpose();
            d.Lu_ += w_pen * penalty->cu(V(x), V(uu)).row(0).transpose();
        }
    }
    void computeHessians(const ConstVectorRef& x, const ConstVectorRef& uu, Data& d) const override {
        d.Lxx_.setZero();
        d.Lxu_.setZero();
        d.Luu_.setZero();
        d.Luu_(0, 0) = 2 * P.r_ctrl_thr;
        for (int j = 1; j < NU; ++j) d.Luu_(j, j) = 2 * P.r_ctrl_mom;
        d.Luu_(IDX_THETA, IDX_THETA) = 2 * P.r_theta;
        d.Luu_ = P.cost_scale * s.scale.asDiagonal() * d.Luu_ * s.scale.asDiagonal();
        if (penalty) {
            const M H = penalty->lagrangianHessian(V(x), V(uu), V::Constant(1, w_pen));
            d.Lxx_ += H.topLeftCorner(NX_SS, NX_SS);
            d.Lxu_ += H.topRightCorner(NX_SS, NU_SS);
            d.Luu_ += H.bottomRightCorner(NU_SS, NU_SS);
        }
    }
};

/// cost_scale 0.5 (wp_xy |r_xy|^2 + wp_z z^2 + wv_xy |v_xy|^2 + wv_z (vz - vz_ref)^2 + wt (qx^2+qy^2) + wo |w|^2)
struct TerminalCost : aligator::CostAbstractTpl<double> {
    using Base = aligator::CostAbstractTpl<double>;
    using Data = aligator::CostDataAbstractTpl<double>;
    V w = V::Zero(NX_SS), ref = V::Zero(NX_SS);
    explicit TerminalCost(const aligator_ocp::Space& sp) : Base(sp, 0) {
        w << P.term_w_pos_xy, P.term_w_pos_xy, P.term_w_pos_z, P.term_w_vel_xy, P.term_w_vel_xy, P.term_w_vel_z, 0,
            P.term_w_tilt, P.term_w_tilt, 0, P.term_w_om, P.term_w_om, P.term_w_om, 0;
        w *= P.cost_scale;
        ref(5) = P.term_vz_ref;
    }
    void evaluate(const ConstVectorRef& x, const ConstVectorRef&, Data& d) const override {
        d.value_ = 0.5 * (w.array() * (x - ref).array().square()).sum();
    }
    void computeGradients(const ConstVectorRef& x, const ConstVectorRef&, Data& d) const override {
        d.Lx_ = w.cwiseProduct(x - ref);
        d.Lu_.setZero();
    }
    void computeHessians(const ConstVectorRef&, const ConstVectorRef&, Data& d) const override {
        d.Lxx_ = w.asDiagonal();
        d.Lxu_.setZero();
        d.Luu_.setZero();
    }
};

// ── Ordinary rows (each <= 0, scaled to an O(1) margin) ───────────────────────
//   thrust min/max, |M|^2 <= tau^2, theta min/max, z >= 0, |v|^2 <= v_max^2
struct Bounds : aligator::StageFunctionTpl<double> {
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;
    quad::ControlScaling s = scaling();
    Bounds() : Base(NX_SS, NU_SS, 7) {}
    void evaluate(const ConstVectorRef& x, const ConstVectorRef& uu, Data& d) const override {
        const V u = s.physical(uu);
        const double fr = P.fmax - P.fmin, tr = P.thh - P.thl;
        d.value_ << (P.fmin - u(0)) / fr, (u(0) - P.fmax) / fr, u.segment<3>(1).squaredNorm() / (TAU_MAX * TAU_MAX) - 1,
            (u(IDX_THETA) - P.thh) / tr, (P.thl - u(IDX_THETA)) / tr, -x(2),
            x.segment<3>(3).squaredNorm() / (P.spd_phase0_max * P.spd_phase0_max) - 1;
    }
    void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& uu, Data& d) const override {
        const V u = s.physical(uu);
        const double fr = P.fmax - P.fmin, tr = P.thh - P.thl;
        d.Jx_.setZero();
        d.Ju_.setZero();
        d.Ju_(0, 0) = -1 / fr;
        d.Ju_(1, 0) = 1 / fr;
        d.Ju_.block<1, 3>(2, 1) = 2 * u.segment<3>(1).transpose() / (TAU_MAX * TAU_MAX);
        d.Ju_(3, IDX_THETA) = 1 / tr;
        d.Ju_(4, IDX_THETA) = -1 / tr;
        d.Jx_(5, 2) = -1;
        d.Jx_.block<1, 3>(6, 3) = 2 * x.segment<3>(3).transpose() / (P.spd_phase0_max * P.spd_phase0_max);
        for (int j = 0; j < NU_SS; ++j) d.Ju_.col(j) *= s.scale(j);
    }
};

// ── Warm-start / validity helpers ─────────────────────────────────────────────
inline void clampWarmControl(V& u) {
    if (u.size() < NU_SS) return;
    u(0) = std::clamp(u(0), P.fmin, P.fmax);
    for (int j = 1; j < 4; ++j) u(j) = std::clamp(u(j), -TAU_MAX, TAU_MAX);
    u(IDX_THETA) = std::clamp(u(IDX_THETA), P.thl, P.thh);
}

inline bool finiteTrajectory(const std::vector<V>& X, const std::vector<V>& U) {
    for (const auto& x : X) if (!x.allFinite()) return false;
    for (const auto& u : U) if (!u.allFinite()) return false;
    return true;
}

inline bool plausibleTrajectory(const std::vector<V>& X, int max_node, int n_solve) {
    const double dt_limit = static_cast<double>(n_solve) * P.thh + 1.0;
    const int last = std::min(max_node, static_cast<int>(X.size()) - 1);
    for (int k = 0; k <= last; ++k) {
        const auto& x = X[k];
        if (x.size() <= IDX_DT) return false;
        if (x(2) < -0.05) return false;
        if (x.head(3).norm() > 100.0) return false;
        if (x.segment(3, 3).norm() > 50.0) return false;
        if (x.segment(10, 3).norm() > 50.0) return false;
        if (x(IDX_DT) < -1e-8 || x(IDX_DT) > dt_limit) return false;
    }
    return true;
}

inline V ensureNoAugStateSize(const V& x) {
    if (x.size() == NX_SS) return x;
    V out = V::Zero(NX_SS);
    out.head(std::min<int>(x.size(), NX)) = x.head(std::min<int>(x.size(), NX));
    return out;
}

// ── Native initial guess ─────────────────────────────────────────────────────
// Rollout under a PD attitude/rate loop with velocity feedback toward a staged
// reference: approach above the LOS trigger / staging height and descend only
// along a height inside a fraction of the glideslope. Only the initial guess.
inline V nativeGuessControl(const V& x, const Eigen::Vector3d& target_accel) {
    constexpr double kp = 1., kd = 2., amax = 2.;
    constexpr double kpos = .3, kvs = .9, vmax = 1.;
    constexpr double hold = .25, slope = .6;
    const Eigen::Vector3d v = x.segment<3>(3), w = x.segment<3>(10);
    Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();
    const double rxy = x.head<2>().norm();
    const double cap = std::max(P.z_stage, P.los_alt_trig) + hold;
    const double zref = std::min(cap, rxy / (slope * std::tan(P.gs_cone_deg * M_PI / 180.)) - P.gs_apex_offset);
    Eigen::Vector3d vref(-kpos * x(0), -kpos * x(1), kpos * (std::max(zref, 0.) - x(2)));
    if (vref.head<2>().norm() > vmax) vref.head<2>() *= vmax / vref.head<2>().norm();
    vref.z() = std::clamp(vref.z(), -.5, .5);
    Eigen::Vector3d a = kvs * (vref - v);
    a.head<2>() = a.head<2>().cwiseMax(-amax).cwiseMin(amax);
    constexpr double floor_z = .05;
    if (v.z() < 0.) a.z() = std::max(a.z(), v.z() * v.z() / (2. * std::max(x(2) - floor_z, .02)));
    const Eigen::Vector3d f = a + target_accel - GRAVITY;
    const Eigen::Vector3d bz = q * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d err = q.conjugate() * bz.cross(f.normalized());
    V u = V::Zero(NU_SS);
    u(0) = P.mass * f.dot(bz);
    u.segment<3>(1) = J_B * (kp * err - kd * w);
    u(IDX_THETA) = P.th0;
    return u;
}

// ── Factory ───────────────────────────────────────────────────────────────────
inline std::shared_ptr<aligator_ocp::Ocp> create(const V& x0_rel, const std::vector<V>& prev_U = {},
                                                 const std::vector<V>& prev_X = {},
                                                 const std::vector<M>& prev_K = {},
                                                 std::shared_ptr<ConstantAccelPredictor> predictor = nullptr,
                                                 int requested_horizon = 0,
                                                 Enforcement enforcement = Enforcement::Interval,
                                                 double w_pen = 0.0) {
    const int N = requested_horizon > 0 ? requested_horizon : P.horizon;
    if (N > P.horizon) throw std::invalid_argument("remaining landing horizon exceeds configured maneuver");
    if (!predictor) predictor = std::make_shared<ConstantAccelPredictor>();
    const Eigen::Vector3d a_target = predictor->predictAccel(0.0);

    aligator_ocp::Space space(NX_SS);
    const quad::Dynamics dyn(space, body(), scaling(), a_target);
    const TerminalCost terminal(space);
    StageCost cost(space);

    // Intervals before CTCS_TIGHTEN_FROM_NODE use the physical implications.
    const auto form = enforcement == Enforcement::Node ? ctcs_stc::Form::Original2018 : ctcs_stc::Form::Density;
    const auto tight = landingRules(true, form, 1, a_target);
    const auto physical = landingRules(false, form, CTCS_PHYSICAL_SUBSTEPS, a_target);
    std::shared_ptr<const ctcs_stc::Spec<double>> tight_pen, physical_pen;
    if (enforcement == Enforcement::Penalty) {
        tight_pen = std::make_shared<const ctcs_stc::Spec<double>>(tight);
        physical_pen = std::make_shared<const ctcs_stc::Spec<double>>(physical);
    }

    std::vector<xyz::polymorphic<aligator_ocp::StageModel>> stages;
    for (int k = 0; k < N; ++k) {
        const bool phys = k < CTCS_TIGHTEN_FROM_NODE;
        StageCost c = cost;
        if (enforcement == Enforcement::Penalty) { c.penalty = phys ? physical_pen : tight_pen; c.w_pen = w_pen; }
        aligator_ocp::StageModel stage(c, dyn);
        stage.addConstraint(Bounds(), aligator_ocp::Orthant());
        if (enforcement != Enforcement::Penalty)
            stage.addConstraint(ctcs_stc::StageFunction(NX_SS, NU_SS, phys ? physical : tight,
                                                        ctcs_stc::Curvature::GaussNewton),
                                aligator_ocp::Orthant());
        stages.emplace_back(stage);
    }

    V x0 = V::Zero(NX_SS);
    x0.head(NX) = x0_rel.head(NX);
    auto ocp = std::make_shared<aligator_ocp::Ocp>();
    ocp->problem = std::make_shared<aligator_ocp::Problem>(x0, stages, terminal);
    const auto s = scaling();
    ocp->toPhysical = [s](const V& u) { return s.physical(u); };
    ocp->toSolver = [s](const V& u) { return s.solver(u); };
    ocp->step = [dyn](const V& x, const V& u) { return dyn.next(x, u); };

    // Rebase the shifted warm-start references: IDX_DT is a per-solve clock,
    // but the core shifts by index, so prev_X[0] carries the old horizon's t0.
    std::vector<V> Xw = prev_X;
    if (!Xw.empty()) {
        const double t0_shift = ensureNoAugStateSize(Xw[0])(IDX_DT);
        bool finite = true;
        for (auto& x : Xw) {
            x = ensureNoAugStateSize(x);
            if (!x.allFinite()) { finite = false; break; }
            x(IDX_DT) = std::max(0.0, x(IDX_DT) - t0_shift);
        }
        if (finite) Xw[0] = x0; else Xw.clear();
    }
    const bool use_feedback = static_cast<int>(prev_U.size()) >= N && static_cast<int>(Xw.size()) > N &&
                              static_cast<int>(prev_K.size()) >= N;

    // Seed (physical controls), rolled out with the OCP's own map, z floored at 0.
    ocp->xs.assign(N + 1, x0);
    ocp->us.assign(N, V::Zero(NU_SS));
    V sim = x0;
    for (int k = 0; k < N; ++k) {
        V uk;
        if (static_cast<int>(prev_U.size()) >= N) {
            uk = prev_U[k];
            if (use_feedback && prev_K[k].rows() == NU_SS && prev_K[k].cols() == NX_SS) {
                const V u_fb = uk + prev_K[k] * (sim - Xw[k]);
                const V du = u_fb - uk;
                const bool wild = !u_fb.allFinite() || (P.max_feedback_du > 0 && du.norm() > P.max_feedback_du) ||
                                  u_fb(0) < 0.5 * P.fmin || u_fb(0) > 1.5 * P.fmax ||
                                  u_fb.segment<3>(1).cwiseAbs().maxCoeff() > 2.0 * TAU_MAX ||
                                  u_fb(IDX_THETA) < 0.0 || u_fb(IDX_THETA) > 2.0 * P.thh;
                if (!wild) uk = u_fb;
            }
        } else {
            if (P.hover_seed) { uk = V::Zero(NU_SS); uk(0) = HOVER_THRUST; uk(IDX_THETA) = P.th0; }
            else uk = nativeGuessControl(sim, a_target);
        }
        clampWarmControl(uk);
        ocp->us[k] = s.solver(uk);
        sim = dyn.next(sim, ocp->us[k]);
        sim(2) = std::max(sim(2), 0.0);
        ocp->xs[k + 1] = sim;
    }
    return ocp;
}

// ── Descriptor ────────────────────────────────────────────────────────────────
inline OCPDescriptor descriptor(Enforcement enforcement = Enforcement::Interval, double w_pen = 0.0) {
    OCPDescriptor d;
    d.disarm_on_landing_finish = true;
    d.name = "stc_landing_noaug";
    d.dt = P.th0;
    d.default_n_replay = P.nex;
    d.default_mass_kg = P.mass;
    d.warm_start = OCPDescriptor::WarmStart::Feedback;
    // A shifted plan from a different measured root leaves the CTCS solve
    // uncertified; the staged nativeGuessControl seed converges.
    d.native_guess_for_measured_root = true;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.world_acceleration = [](const V& x, const V& u) {
        if (x.size() < 13 || u.size() < 1 || !x.allFinite() || !u.allFinite())
            throw std::invalid_argument("invalid DDP acceleration input");
        Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
        if (q.norm() < 1e-8) throw std::invalid_argument("invalid DDP attitude");
        return Eigen::Vector3d(q.normalized() * Eigen::Vector3d(0, 0, u(0) / P.mass) + Eigen::Vector3d(0, 0, -9.81));
    };
    // Invert PX4 PositionControl's native acceleration conversion at the
    // nominal DDP state (coordinate/input conversion, no measured-error feedback).
    d.mavros_acceleration = [physical = d.world_acceleration](const V& x, const V& u) {
        const Eigen::Vector3d acceleration = physical(x, u);
        Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
        const Eigen::Vector3d axis = q.normalized() * Eigen::Vector3d::UnitZ();
        Eigen::Vector3d feedforward = acceleration;
        if (comando_parameters::defaults.px4.decouple_acceleration) {
            if (axis.z() <= 1e-6) throw std::invalid_argument("PX4 acceleration mapping requires upright thrust axis");
            feedforward.head<2>() = 9.81 * axis.head<2>() / axis.z();
        }
        feedforward.x() += comando_parameters::defaults.px4.velocity_d_xy * acceleration.x();
        feedforward.y() += comando_parameters::defaults.px4.velocity_d_xy * acceleration.y();
        feedforward.z() += comando_parameters::defaults.px4.velocity_d_z * acceleration.z();
        return feedforward;
    };
    d.coordinates = OCPDescriptor::Coordinates::TargetRelative;
    d.variable_dt = true;
    d.state_dim = 13;
    d.control_dim = 4;
    d.skip_altitude_validation = true;
    d.skip_trajectory_validation = true;
    d.log_state_headers = OCPLoggerDefaults::getStateHeaders13D();
    d.state_names = OCPLoggerDefaults::getStateNames13D();
    d.control_names = OCPLoggerDefaults::getControlNames4D();
    d.extract_actual_state_row = OCPLoggerDefaults::getActualStateRow13D;
    d.make_hover_state = OCPLoggerDefaults::makeHoverState13D;
    d.post_process_result = [](SolverResult& r, const TargetSnapshot& t) {
        r.is_relative_plan = t.valid;
        r.target_snapshot_pos = t.valid ? t.position : Eigen::Vector3d::Zero();
        r.target_snapshot_vel = t.valid ? t.velocity : Eigen::Vector3d::Zero();
        r.target_snapshot_acc = t.valid ? t.acceleration : Eigen::Vector3d::Zero();
        if (r.success && (!finiteTrajectory(r.state_trajectory, r.control_trajectory) ||
                          !plausibleTrajectory(r.state_trajectory, std::min(P.nex, P.horizon), P.horizon)))
            r.success = false;
    };
    d.prepare_extra = [](const PlannerConfig&, double, const TargetSnapshot& tgt) {
        StcLandingNoAugExtra ex;
        ex.predictor = std::make_shared<ConstantAccelPredictor>();
        ex.predictor->setState(tgt.position, tgt.velocity, tgt.acceleration);
        return std::any(ex);
    };
    d.sanitize_warm_control = [](V& u) { clampWarmControl(u); };
    d.make_model = [](const Eigen::Vector3d& accel) {
        aligator_ocp::Space space(NX_SS);
        const quad::Dynamics dyn(space, body(), quad::ControlScaling{}, accel);
        OCPDescriptor::Model model;
        model.nx = NX_SS; model.nu = NU_SS; model.horizon = P.horizon; model.dt = P.th0;
        model.hover = V::Zero(NU_SS); model.hover(0) = HOVER_THRUST; model.hover(IDX_THETA) = P.th0;
        model.stage_duration = [](const V&, const V& u) { return u(IDX_THETA); };
        model.initial_state = [](const V& physical, const V&) { V x = V::Zero(NX_SS); x.head(NX) = physical.head(NX); return x; };
        model.bounded = [](V u) { clampWarmControl(u); return u; };
        model.flow = [dyn](const V& x, const V& u, double h, bool jac) {
            V control = u;
            control(IDX_THETA) = h;
            OCPDescriptor::Flow result;
            result.x = dyn.next(x, control);
            if (jac) {
                aligator::ExplicitDynamicsDataTpl<double> data(dyn);
                dyn.dForward(x, control, data);
                result.a = data.Jx();
                result.b = data.Ju();
                result.b.col(IDX_THETA).setZero();
            }
            return result;
        };
        return model;
    };
    d.solver_settings = [] { return P.solver; };
    d.create = [enforcement, w_pen](const OCPCreateArgs& a) {
        StcLandingNoAugExtra ex;
        if (a.extra.has_value()) {
            try { ex = std::any_cast<StcLandingNoAugExtra>(a.extra); } catch (const std::bad_any_cast&) {}
        }
        auto pred = ex.predictor;
        if (!pred) {
            pred = std::make_shared<ConstantAccelPredictor>();
            pred->setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), a.target_accel);
        }
        return create(a.current_state, a.prev_U, a.prev_X, a.prev_K, pred, a.horizon, enforcement, w_pen);
    };
    if (enforcement == Enforcement::Node) d.name = "stc_landing_noaug_node";
    if (enforcement == Enforcement::Penalty) d.name = "stc_landing_noaug_penalty";
    return d;
}

}  // namespace StcLandingNoAugOCP
