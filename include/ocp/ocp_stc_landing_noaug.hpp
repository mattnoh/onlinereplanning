/// @file ocp_stc_landing_noaug.hpp
/// @brief Interval CT-cSTC landing on a moving target ("stc_landing_noaug").
///
/// Relative-frame quadrotor landing:
///   state   = [13 physical | IDX_DT time since horizon start]
///   control = [fz, Mx, My, Mz | IDX_THETA interval duration]
///
/// State-triggered constraints (staging, line of sight, landing speed/tilt/
/// rate/glideslope) are enforced per interval as the RK4 integral of their
/// violation density, kept within a budget eps:
///   integral(H dt) over [t_k, t_k + theta_k]  <=  eps
/// All numbers live in Params; preset() holds the validated configurations,
/// selected per process by STC_PRESET.
#pragma once
#include "target/predictor.hpp"
#include "parameters.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <any>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "optimal_control_problem.h"
#include "constraint/stc_stage_constraint.h"
#include "alipddp/alipddp.h"
#include "planner_core/types.hpp"

namespace StcLandingNoAugOCP {

constexpr int NX=13, NU=4, NX_SS=14, NU_SS=5, IDX_DT=13, IDX_THETA=4;

// Optimizer tightening inside the physical bounds, applied to intervals from
// CTCS_TIGHTEN_FROM_NODE on. Zero means none.
struct CtcsMargins {
    double trigger=0.;        // trigger altitudes/radius moved outward [m]
    double geometric=0.;      // glideslope offset [m]
    double speed=0.;          // landing speed [m/s]
    double tilt_deg=0.;       // landing tilt [deg]
    double path_tilt_deg=0.;  // approach tilt [deg]
    double rate=0.;           // body rate [rad/s]
    double los=0.;            // LOS cone distance [m], tapered to an angle at the apex
    double stage_radius=0.;   // staging radius [m]
    double gs_angle_deg=0.;   // glideslope half angle [deg]
    double stage_height=0.;   // staging height [m]
};

// Every number this OCP uses. Presets below hold the validated configurations.
struct Params {
    // Vehicle (Crazyflie-scale model)
    double mass=0.027, ixx=1.66e-5, iyy=1.66e-5, izz=2.92e-5, l_arm=0.046;
    double actuator_fmin=0.08, actuator_fmax=0.60;  // per-motor sum limits [N]
    double fmin=0.12, fmax=0.52;                    // collective thrust bounds [N]
    // Horizon: intervals, intervals executed per replan, interval duration [s]
    int horizon=30, nex=7;
    double th0=0.07, thl=0.05, thh=0.12;
    // Stage cost
    double w_eps=1.0, w_pos_xy=0.0, w_pos_z=0.0, w_vel=0.0;
    double stage_w_time=0.04, r_theta=10.0, theta_ref=0.10, r_ctrl_thr=2.0, r_ctrl_mom=0.6;
    double w_touch_vel=10.0, touch_alt=0.25, touch_scale=0.25;
    double cap_cost_radius=0.50, k_cap=20.0, cap_desc_radius=0.12, k_desc=40.0;
    // Terminal cost
    double term_w_pos_xy=2500.0, term_w_pos_z=2500.0, term_w_vel_xy=900.0, term_w_vel_z=900.0;
    double term_w_tilt=200.0, term_w_om=50.0, term_vz_ref=0.0;
    // Physical bounds: approach (always) and landing (below alt_trig)
    double tilt_phase0_deg=11.0, omega_phase0_max=0.35, spd_phase0_max=3.0;
    double tilt_stc_deg=0.95, alpha_theta_stc=1.0, omega_stc_max=0.019, spd_stc_max=0.75;
    double gs_cone_deg=15.0, gs_apex_offset=0.1, los_cone_deg=30.0;
    // Triggers
    double alt_trig=0.8;                                  // landing phase below [m]
    double los_alt_trig=1.8, los_trigger_scale=0.25, los_trigger_floor=0.0;
    double z_stage=1.8, stage_radius=1.10, stage_trigger_scale=0.25, stage_trigger_floor=0.0;
    // CTCS density weights and per-interval budget
    double w_land_speed=1.0, w_land_tilt=1.0e5, w_land_omega=100.0, w_land_gs=5000.0;
    double w_stc_los=50.0, w_stc_stage=10.0;
    double ctcs_y_scale=10.0, ctcs_step_eps=1.6e-3;
    CtcsMargins margins;
    bool scale_bounds=false;        // divide thrust/duration rows by their range
    double max_feedback_du=0.05;    // feedback warm start: largest accepted correction
    Param solver=solverDefaults();
    static Param solverDefaults(){
        Param p;
        p.max_iter = 180;
        p.tolerance = 1e-6;
        p.mu_mul = 0.1;
        p.rho = 5;  p.rhoT = 5;  p.rho_mul = 10;
        p.reg1_min = 1e-3;  p.reg2_min = 0.1;
        p.verbose = false;
        p.is_quaternion_in_state = false;
        p.use_ddp_terms = false;
        return p;
    }
};

// ── Presets ───────────────────────────────────────────────────────────────────
// Validated configurations (see support/test/frozen_node_baseline.json). Each
// one states only how it differs from what it builds on.
//   default          quad NODE frozen runs (the struct defaults above)
//   wall_clock_node  non-SITL advancing-time quad NODE
//   n80              frozen quad CTCS, 80 intervals   = ctcsBase()
//   n60              frozen quad CTCS, 60 intervals   = n80 rescaled to 60
//   wall_clock       non-SITL advancing-time quad CTCS
//   sitl             PX4/Gazebo measured-state replanning (all target motions)

// Shared by every CTCS preset: the interval rows carry the constraints, so
// the touchdown/theta shaping costs are off and the density weights are high.
inline Params ctcsBase(){
    Params p;
    p.horizon = 80;
    // costs
    p.w_eps = 0.0;
    p.r_theta = 0.0;
    p.w_touch_vel = 0.0;
    p.term_w_pos_xy = 1e5;
    p.term_w_pos_z  = 1e4;
    // CTCS density weights and per-interval budget
    p.w_land_speed = 100;
    p.w_land_tilt  = 3e8;
    p.w_land_omega = 1e6;
    p.w_land_gs    = 7500;
    p.w_stc_los    = 200;
    p.w_stc_stage  = 100;
    p.ctcs_step_eps = 1e-5;
    p.ctcs_y_scale  = 0.5;
    p.scale_bounds  = true;
    // optimizer margins inside the physical bounds
    p.margins.trigger       = 0.05;  // m
    p.margins.geometric     = 0.02;  // m
    p.margins.speed         = 0.05;  // m/s
    p.margins.tilt_deg      = 0.2;
    p.margins.path_tilt_deg = 0.5;
    p.margins.rate          = 0.005; // rad/s
    p.margins.los           = 0.02;  // m
    p.margins.stage_radius  = 0.05;  // m
    p.margins.stage_height  = 0.02;  // m
    // solver
    p.solver.max_iter = 1200;
    p.solver.mu  = 0.1;  p.solver.muT  = 0.1;
    p.solver.rho = 100;  p.solver.rhoT = 100;
    return p;
}

inline Params preset(const std::string& name){
    if(name.empty()||name=="default")return Params{};

    if(name=="wall_clock_node"){
        Params p;
        p.horizon = 40;
        p.thh = 0.16;
        p.solver.max_iter = 120;
        return p;
    }

    if(name=="n80")return ctcsBase();

    if(name=="n60"){
        // Same total horizon time as n80 with fewer, longer intervals.
        Params p = ctcsBase();
        const double s = 80.0/60;
        p.horizon = 60;
        p.th0 *= s;  p.thl *= s;  p.thh *= s;
        p.ctcs_step_eps *= s;
        p.ctcs_y_scale = 5;
        p.solver.rho = 50;  p.solver.rhoT = 50;
        return p;
    }

    if(name=="wall_clock"){
        Params p = ctcsBase();
        p.term_w_pos_xy = 2e5;
        p.ctcs_y_scale = 2;
        return p;
    }

    if(name=="sitl"){
        Params p = ctcsBase();
        p.horizon = 40;
        p.th0 = 0.14;  p.thl = 0.06;  p.thh = 0.24;
        p.ctcs_step_eps *= 80.0/60;    // the n60 budget
        p.ctcs_y_scale = 5;
        p.solver.mu = 1.0;             // initial barrier (terminal stays 0.1)
        p.solver.rho = 50;  p.solver.rhoT = 50;
        // physical bounds sized for PX4 tracking
        p.tilt_phase0_deg  = 8;
        p.omega_phase0_max = 0.2;      // rad/s
        p.spd_phase0_max   = 2.0;      // m/s
        p.tilt_stc_deg     = 5;
        p.omega_stc_max    = 0.15;     // rad/s
        p.spd_stc_max      = 1.0;      // m/s
        p.gs_cone_deg      = 31;
        p.gs_apex_offset   = 0.25;     // m
        p.los_cone_deg     = 57;
        p.los_alt_trig     = 1.5;      // m
        p.stage_radius     = 1.2;      // m
        // wider optimizer margins to absorb tracking error
        p.margins.trigger      = 0.2;
        p.margins.geometric    = 0.05;
        p.margins.speed        = 0.2;
        p.margins.rate         = 0.06;
        p.margins.gs_angle_deg = 3;
        p.margins.los          = 0.09;
        p.margins.stage_radius = 0.7;
        p.margins.stage_height = 0.12;
        return p;
    }

    throw std::invalid_argument("unknown stc_landing_noaug preset: "+name);
}
// STC_PRESET selects the configuration for this process.
inline const Params P=[]{const char* n=std::getenv("STC_PRESET");return preset(n?n:"");}();

inline const double J_SCALE = 1.0/P.ixx;
inline const double TAU_MAX = P.l_arm*(P.actuator_fmax/4.0-P.actuator_fmin/4.0)*J_SCALE;
inline const Eigen::Matrix3d J_B = (Eigen::Matrix3d()<<P.ixx*J_SCALE,0,0,0,P.iyy*J_SCALE,0,0,0,P.izz*J_SCALE).finished();
inline const Eigen::Vector3d GRAVITY{0,0,-9.81};
inline const double HOVER_THRUST = P.mass*9.81;
inline const double LOS_CONE_TAN = std::tan(P.los_cone_deg*M_PI/180.0);

inline double gsConeTan(const CtcsMargins& m){return std::tan((P.gs_cone_deg-m.gs_angle_deg)*M_PI/180.0);}
inline CtcsMargins ctcsMargins(bool tightened){return tightened?P.margins:CtcsMargins{};}
// Trigger activation at the boundary. Triggers weight their consequence by
// the squared depth past the boundary, so with a zero floor a row fades out
// exactly at its trigger. Tightened rows move the trigger outward by the
// trigger margins, so at the PHYSICAL boundary they already carry the depth
// margin/scale. Physical rows (untightened intervals) get that same activation
// as a floor at the physical boundary; without it a plan can cross a physical
// trigger in violation at almost no cost.
struct TriggerFloors { double landing=0., los=0., stage=0.; };
inline TriggerFloors triggerFloors(bool tightened){
    if(tightened)return {0.,P.los_trigger_floor,P.stage_trigger_floor};
    return {P.margins.trigger,
            std::max(P.los_trigger_floor,P.margins.trigger/P.los_trigger_scale),
            std::max(P.stage_trigger_floor,P.margins.stage_radius/P.stage_trigger_scale)};
}

// ── Sigmoid helpers ───────────────────────────────────────────────────────────
template<typename Scalar>
inline Scalar sigmaLandVal(const Scalar& rxy2) {
    const Scalar g = Scalar(P.cap_cost_radius * P.cap_cost_radius) - rxy2;
    return Scalar(0.5) * (Scalar(1) + std::tanh(Scalar(P.k_cap) * g));
}

template<typename Scalar>
inline void sigmaLandGrad(const Scalar& rxy2, const Scalar& x0, const Scalar& x1,
                          Scalar& sig, Scalar& gx0, Scalar& gx1) {
    const Scalar g       = Scalar(P.cap_cost_radius * P.cap_cost_radius) - rxy2;
    const Scalar th      = std::tanh(Scalar(P.k_cap) * g);
    sig     = Scalar(0.5) * (Scalar(1) + th);
    const Scalar dsig_dg = Scalar(0.5) * Scalar(P.k_cap) * (Scalar(1) - th * th);
    gx0 = dsig_dg * Scalar(-2) * x0;
    gx1 = dsig_dg * Scalar(-2) * x1;
}

template<typename Scalar>
inline Scalar sigmaDescVal(const Scalar& rxy2, const Scalar& /*z*/) {
    const Scalar g = Scalar(P.cap_desc_radius * P.cap_desc_radius) - rxy2;
    return Scalar(0.5) * (Scalar(1) + std::tanh(Scalar(P.k_desc) * g));
}

template<typename Scalar>
inline void sigmaDescGrad(const Scalar& rxy2, const Scalar& x0, const Scalar& x1,
                          const Scalar& /*z*/,
                          Scalar& sig, Scalar& gx0, Scalar& gx1, Scalar& gz) {
    const Scalar g    = Scalar(P.cap_desc_radius * P.cap_desc_radius) - rxy2;
    const Scalar th   = std::tanh(Scalar(P.k_desc) * g);
    sig = Scalar(0.5) * (Scalar(1) + th);
    const Scalar dsig = Scalar(0.5) * Scalar(P.k_desc) * (Scalar(1) - th * th);
    gx0 = dsig * Scalar(-2) * x0;
    gx1 = dsig * Scalar(-2) * x1;
    gz  = Scalar(0);
}

template<typename Scalar>
inline Scalar posPart(const Scalar& v) {
    return (v > Scalar(0)) ? v : Scalar(0);
}

// Landing trigger activation: floor + depth below the trigger altitude.
template<typename Scalar>
inline Scalar landingAltitudeTrigger(const Scalar& z,double altitude=P.alt_trig,double floor=0.) {
    const Scalar depth = Scalar(altitude) - z;
    return depth > Scalar(0) ? Scalar(floor) + depth : Scalar(0);
}

// ── Constant-accel target predictor ───────────────────────────────────────────
// Reset from the TargetSnapshot at every solve (see prepare_extra), then held
// as a constant-acceleration model across the horizon.
using ConstantAccelPredictor = target_models::ConstantAccelPredictor;

struct StcLandingNoAugExtra {
    std::shared_ptr<ConstantAccelPredictor> predictor;
};

template<typename Scalar>
Scalar evalCtcsIntegrand(const Vector<Scalar>& x,
                         const Vector<Scalar>& u,bool tightened=true);

// ── Relative dynamics with constant-accel target prediction (NO y-row) ───────
template<typename Scalar>
class Quad6DOFVarTimeRelativePred : public Quad6DOFVarTimeRelative<Scalar> {
    std::shared_ptr<ConstantAccelPredictor> pred_;

public:
    explicit Quad6DOFVarTimeRelativePred(std::shared_ptr<ConstantAccelPredictor> pred)
        : Quad6DOFVarTimeRelative<Scalar>(), pred_(std::move(pred)) {
        this->dim_x = NX_SS;
    }

private:
    double ctcsRate(const Eigen::VectorXd& x_phys, const Eigen::VectorXd& u_full) const {
        Eigen::VectorXd x_aug(NX_SS);
        x_aug.setZero();
        x_aug.segment(0, this->NX_PHYS) = x_phys;
        return evalCtcsIntegrand<double>(x_aug, u_full) / P.ctcs_y_scale;
    }

public:
    // Continuous host dynamics for StcStageConstraint's interval quadrature.
    // IDX_DT is integrated alongside the physical state; tau must not be
    // added a second time when sampling the predictor.
    Vector<Scalar> intervalRates(const Vector<Scalar>& x, const Vector<Scalar>& u) const {
        Vector<Scalar> out = Vector<Scalar>::Zero(NX_SS);
        out.head(NX) = this->xdot_impl(x.head(NX).template cast<double>(),
            u.head(NU).template cast<double>(),
            pred_->predictAccel(static_cast<double>(x(IDX_DT)))).template cast<Scalar>();
        out(IDX_DT) = Scalar(1);
        return out;
    }

    Matrix<Scalar> intervalRatesX(const Vector<Scalar>& x, const Vector<Scalar>& u) const {
        Matrix<Scalar> out = Matrix<Scalar>::Zero(NX_SS, NX_SS);
        out.topLeftCorner(NX, NX) = this->Amat_impl(
            x.head(NX).template cast<double>(), u.head(NU).template cast<double>()).template cast<Scalar>();
        // ConstantAccelPredictor has zero acceleration derivative in time.
        return out;
    }

    Matrix<Scalar> intervalRatesU(const Vector<Scalar>& x, const Vector<Scalar>&) const {
        Matrix<Scalar> out = Matrix<Scalar>::Zero(NX_SS, NU_SS);
        out.topLeftCorner(NX, NU) = this->Bmat_impl(x.head(NX).template cast<double>()).template cast<Scalar>();
        return out;
    }

    // The SAME RK4 interval integral the accumulator would have added to y,
    // computed directly from (x,u). No augmented state is involved.
    double intervalIntegral(const Vector<Scalar>& x, const Vector<Scalar>& u) const {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        auto u_full = u.template cast<double>();
        double Th   = static_cast<double>(u(IDX_THETA));
        double tabs = static_cast<double>(x(IDX_DT));

        Eigen::Vector3d a1 = pred_->predictAccel(tabs);
        auto k1 = this->xdot_impl(xd, ud, a1);
        double h1 = ctcsRate(xd, u_full);
        Eigen::Vector3d a2 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x2 = xd + 0.5*Th*k1;
        auto k2 = this->xdot_impl(x2, ud, a2);
        double h2 = ctcsRate(x2, u_full);
        Eigen::Vector3d a3 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x3 = xd + 0.5*Th*k2;
        auto k3 = this->xdot_impl(x3, ud, a3);
        double h3 = ctcsRate(x3, u_full);
        Eigen::Vector3d a4 = pred_->predictAccel(tabs + Th);
        Eigen::VectorXd x4 = xd + Th*k3;
        (void)a4;
        double h4 = ctcsRate(x4, u_full);
        return (Th/6.0)*(h1 + 2.0*h2 + 2.0*h3 + h4);
    }

    Vector<Scalar> f(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        double Th   = static_cast<double>(u(IDX_THETA));
        double tabs = static_cast<double>(x(IDX_DT));

        Eigen::Vector3d a1 = pred_->predictAccel(tabs);
        auto k1 = this->xdot_impl(xd, ud, a1);

        Eigen::Vector3d a2 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x2 = xd + 0.5*Th*k1;
        auto k2 = this->xdot_impl(x2, ud, a2);

        Eigen::Vector3d a3 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x3 = xd + 0.5*Th*k2;
        auto k3 = this->xdot_impl(x3, ud, a3);

        Eigen::Vector3d a4 = pred_->predictAccel(tabs + Th);
        Eigen::VectorXd x4 = xd + Th*k3;
        auto k4 = this->xdot_impl(x4, ud, a4);

        Eigen::VectorXd xn = xd + (Th/6.0)*(k1 + 2*k2 + 2*k3 + k4);
        xn.segment(6, 4).normalize();

        const_cast<Quad6DOFVarTimeRelativePred*>(this)
            ->setTargetAccel(pred_->predictAccel(tabs + 0.5*Th));

        Vector<Scalar> res(NX_SS);
        res.setZero();
        res.segment(0, this->NX_PHYS) = xn.template cast<Scalar>();
        res(IDX_DT) = x(IDX_DT) + u(IDX_THETA);
        return res;
    }

    Matrix<Scalar> fx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        const double Th   = static_cast<double>(u(IDX_THETA));
        const double tabs = static_cast<double>(x(IDX_DT));
        const Eigen::Vector3d amid = pred_->predictAccel(tabs + 0.5 * Th);
        auto j = this->rk4_jac_impl(xd, ud, Th, amid);

        Matrix<Scalar> Fx = Matrix<Scalar>::Zero(NX_SS, NX_SS);
        Fx.block(0, 0, this->NX_PHYS, this->NX_PHYS) = j.dxdx.template cast<Scalar>();
        Fx(IDX_DT, IDX_DT) = Scalar(1.0);
        const_cast<Quad6DOFVarTimeRelativePred*>(this)->setTargetAccel(amid);
        return Fx;
    }

    Matrix<Scalar> fu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        const double Th   = static_cast<double>(u(IDX_THETA));
        const double tabs = static_cast<double>(x(IDX_DT));
        const Eigen::Vector3d amid = pred_->predictAccel(tabs + 0.5 * Th);
        auto j = this->rk4_jac_impl(xd, ud, Th, amid);

        Matrix<Scalar> Fu = Matrix<Scalar>::Zero(NX_SS, NU_SS);
        Fu.block(0, 0, this->NX_PHYS, this->NU_PHYS) = j.dxdu.template cast<Scalar>();
        Fu.block(0, IDX_THETA, this->NX_PHYS, 1) = j.dxdT.template cast<Scalar>();
        Fu(IDX_DT, IDX_THETA) = Scalar(1.0);
        const_cast<Quad6DOFVarTimeRelativePred*>(this)->setTargetAccel(amid);
        return Fu;
    }

    // 14-dim propagate for warm-start rollouts (tracks IDX_DT).
    Eigen::VectorXd propagate14(const Eigen::VectorXd& x14,
                                const Eigen::VectorXd& u_phys,
                                double Th) const {
        double tabs = x14(IDX_DT);
        auto xd = x14.segment(0, this->NX_PHYS);

        Eigen::Vector3d a1 = pred_->predictAccel(tabs);
        auto k1 = this->xdot_impl(xd, u_phys, a1);
        Eigen::Vector3d a2 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x2 = xd + 0.5*Th*k1;
        auto k2 = this->xdot_impl(x2, u_phys, a2);
        Eigen::Vector3d a3 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x3 = xd + 0.5*Th*k2;
        auto k3 = this->xdot_impl(x3, u_phys, a3);
        Eigen::Vector3d a4 = pred_->predictAccel(tabs + Th);
        Eigen::VectorXd x4 = xd + Th*k3;
        auto k4 = this->xdot_impl(x4, u_phys, a4);

        Eigen::VectorXd xn14(NX_SS);
        xn14.setZero();
        xn14.segment(0, this->NX_PHYS) = xd + (Th/6.0)*(k1+2*k2+2*k3+k4);
        xn14.segment(6, 4).normalize();
        xn14(IDX_DT) = x14(IDX_DT) + Th;
        return xn14;
    }
};

// ── CT-cSTC integrand ─────────────────────────────────────────────────────────
template<typename Scalar>
struct CtcsIntegrandTerms {
    Scalar a_stage            = Scalar(0);
    Scalar a_los              = Scalar(0);
    Scalar a_altitude_state   = Scalar(0);
    Scalar loose_tilt         = Scalar(0);
    Scalar loose_omega        = Scalar(0);
    Scalar speed              = Scalar(0);
    Scalar tilt               = Scalar(0);
    Scalar omega              = Scalar(0);
    Scalar glideslope         = Scalar(0);
    Scalar los                = Scalar(0);
    Scalar path_value         = Scalar(0);
    Scalar staging_value      = Scalar(0);
    Scalar los_value          = Scalar(0);
    Scalar landing_value      = Scalar(0);
    Scalar stc_value          = Scalar(0);
    Scalar value              = Scalar(0);
};

template<typename Scalar>
inline CtcsIntegrandTerms<Scalar> evalCtcsIntegrandTerms(const Vector<Scalar>& x,
                                                         const Vector<Scalar>& u,bool tightened=true) {
    CtcsIntegrandTerms<Scalar> out;
    const auto margins=ctcsMargins(tightened);
    const auto floors=triggerFloors(tightened);

    const Scalar rxy2     = x(0)*x(0) + x(1)*x(1);
    out.a_altitude_state  = landingAltitudeTrigger<Scalar>(x(2),P.alt_trig+margins.trigger,floors.landing);

    const Scalar v2     = x.template segment<3>(3).squaredNorm();
    const Scalar qperp2 = x(7)*x(7) + x(8)*x(8);
    const Scalar om2    = x.template segment<3>(10).squaredNorm();
    const Scalar rxy    = std::sqrt(rxy2 + Scalar(1e-12));
    const Scalar stage_radius_margin =
        (rxy - Scalar(P.stage_radius-margins.stage_radius)) / Scalar(P.stage_trigger_scale);
    out.a_stage =
        (stage_radius_margin > Scalar(0))
            ? Scalar(floors.stage) + stage_radius_margin
            : Scalar(0);

    const Scalar loose_tilt_lim =
        Scalar(std::pow(std::sin(0.5 * (P.tilt_phase0_deg-margins.path_tilt_deg) * M_PI / 180.0), 2));
    out.loose_tilt  = posPart(qperp2 - loose_tilt_lim);
    out.loose_omega = posPart(om2 - Scalar(std::pow(P.omega_phase0_max-margins.rate,2)));

    const Scalar tilt_lim =
        Scalar(std::pow(std::sin(0.5 * (P.alpha_theta_stc * P.tilt_stc_deg-margins.tilt_deg) * M_PI / 180.0), 2));
    out.speed      = posPart(v2     - Scalar(std::pow(P.spd_stc_max-margins.speed,2)));
    out.tilt       = posPart(qperp2 - tilt_lim);
    out.omega      = posPart(om2    - Scalar(std::pow(P.omega_stc_max-margins.rate,2)));
    const Scalar gs_val = rxy - Scalar(gsConeTan(margins)) * (x(2) + Scalar(P.gs_apex_offset));
    out.glideslope = posPart(gs_val+Scalar(margins.geometric));

    const Scalar qw = x(6), qx = x(7), qy = x(8), qz = x(9);
    const Scalar bz_x = Scalar(2) * (qx * qz + qw * qy);
    const Scalar bz_y = Scalar(2) * (qy * qz - qw * qx);
    const Scalar bz_z = qw*qw - qx*qx - qy*qy + qz*qz;
    const Scalar cam_x = -bz_x;
    const Scalar cam_y = -bz_y;
    const Scalar cam_z = -bz_z;
    const Scalar los_x = -x(0);
    const Scalar los_y = -x(1);
    const Scalar los_z = -x(2);
    const Scalar axial = los_x * cam_x + los_y * cam_y + los_z * cam_z;
    const Scalar los_norm2 = los_x*los_x + los_y*los_y + los_z*los_z;
    const Scalar lat2 = posPart(los_norm2 - axial * axial);
    const Scalar lateral = std::sqrt(lat2 + Scalar(1e-12));
    const Scalar los_alt_margin =
        (Scalar(P.los_alt_trig+margins.trigger) - x(2)) / Scalar(P.los_trigger_scale);
    out.a_los =
        (los_alt_margin > Scalar(0))
            ? Scalar(floors.los) + los_alt_margin
            : Scalar(0);
    const double los_tan=LOS_CONE_TAN;
    out.los = posPart(lateral - Scalar(los_tan) * axial
        + Scalar(margins.los)*std::tanh(axial/Scalar(P.gs_apex_offset)));

    // NOTE: no thrust STC blocks — faithful to quad_single_horizon_noaug_stc,
    // whose integrand carries only stage/LOS/landing. Thrust limits stay
    // enforced as hard Fmin/Fmax stage constraints.
    const Scalar path_sum =
        out.loose_tilt  * out.loose_tilt  +
        out.loose_omega * out.loose_omega;

    const Scalar landing_sum =
        Scalar(P.w_land_speed) * out.speed      * out.speed      +
        Scalar(P.w_land_tilt)  * out.tilt       * out.tilt       +
        Scalar(P.w_land_omega) * out.omega      * out.omega      +
        Scalar(P.w_land_gs)    * out.glideslope * out.glideslope;

    out.path_value = path_sum;

    const Scalar stage_height_viol = posPart(Scalar(P.z_stage+margins.stage_height) - x(2));
    out.staging_value = Scalar(P.w_stc_stage) *
                        out.a_stage * out.a_stage *
                        stage_height_viol * stage_height_viol;
    out.los_value = Scalar(P.w_stc_los) *
                    out.a_los * out.a_los *
                    out.los * out.los;
    out.landing_value =
        out.a_altitude_state * out.a_altitude_state * landing_sum;

    out.stc_value =
        out.staging_value      +
        out.los_value          +
        out.landing_value;

    out.value =
        out.path_value +
        out.stc_value;

    return out;
}

template<typename Scalar>
Scalar evalCtcsIntegrand(const Vector<Scalar>& x,
                         const Vector<Scalar>& u,bool tightened) {
    return evalCtcsIntegrandTerms(x, u, tightened).value;
}

// RK4 substeps of the physical rows' interval integral.
constexpr int CTCS_PHYSICAL_SUBSTEPS=8;
inline int ctcsPhysicalSubsteps(){return CTCS_PHYSICAL_SUBSTEPS;}

// Intervals before this node use the physical rows (tightened=false).
constexpr int CTCS_TIGHTEN_FROM_NODE=4;
inline int ctcsTightenFromNode(){return CTCS_TIGHTEN_FROM_NODE;}

// ── Stage cost ────────────────────────────────────────────────────────────────
template<typename Scalar>
class CTcSTCStageCost : public StageCostBase<Scalar> {
    Scalar eps_;
    Scalar w_pos_xy_, w_pos_z_, w_vel_;
    Scalar rf_x_, rf_y_, rf_z_;

public:
    CTcSTCStageCost(double eps, double w_pos_xy, double w_pos_z, double w_vel,
                    const Eigen::Vector3d& r_follow)
        : eps_(Scalar(eps)),
          w_pos_xy_(Scalar(w_pos_xy)), w_pos_z_(Scalar(w_pos_z)),
          w_vel_(Scalar(w_vel)),
          rf_x_(Scalar(r_follow(0))), rf_y_(Scalar(r_follow(1))), rf_z_(Scalar(r_follow(2))) {}

    Scalar q(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        const Scalar rxy2 = x(0)*x(0) + x(1)*x(1);
        const Scalar sc   = sigmaLandVal(rxy2);
        const Scalar sd   = sigmaDescVal(rxy2, x(2));

        const Scalar rref_x = (Scalar(1) - sc) * rf_x_;
        const Scalar rref_y = (Scalar(1) - sc) * rf_y_;
        const Scalar rref_z = (Scalar(1) - sc) * rf_z_ + sc*(Scalar(1)-sd)*Scalar(P.z_stage);

        const Scalar drx = x(0) - rref_x;
        const Scalar dry = x(1) - rref_y;
        const Scalar drz = x(2) - rref_z;
        const Scalar v2  = x.template segment<3>(3).squaredNorm();
        const Scalar touch_margin = (Scalar(P.touch_alt) - x(2)) / Scalar(P.touch_scale);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);

        const Scalar pos_cost = w_pos_xy_*(drx*drx + dry*dry) + w_pos_z_*drz*drz;
        const Scalar vel_cost = sd * w_vel_ * v2;
        const Scalar touch_vel_cost = Scalar(P.w_touch_vel) * touch * touch * v2;
        const Scalar reg      = eps_*(v2 + x.template segment<3>(10).squaredNorm());

        const Scalar dT  = u(0) - Scalar(HOVER_THRUST);
        const Scalar ctrl_cost = Scalar(P.r_ctrl_thr)*dT*dT
                               + Scalar(P.r_ctrl_mom)*(u(1)*u(1) + u(2)*u(2) + u(3)*u(3));

        const Scalar dTheta = u(IDX_THETA) - Scalar(P.theta_ref);
        const Scalar time_reg = Scalar(P.r_theta) * dTheta * dTheta;

        return Scalar(P.stage_w_time)*u(IDX_THETA) + time_reg + reg
             + pos_cost + vel_cost + touch_vel_cost + ctrl_cost;
    }

    Vector<Scalar> qx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)u;
        const Scalar rxy2 = x(0)*x(0) + x(1)*x(1);

        Scalar sc, sc_gx0, sc_gx1;
        sigmaLandGrad(rxy2, x(0), x(1), sc, sc_gx0, sc_gx1);

        Scalar sd, sd_gx0, sd_gx1, sd_gz;
        sigmaDescGrad(rxy2, x(0), x(1), x(2), sd, sd_gx0, sd_gx1, sd_gz);

        const Scalar Zs    = Scalar(P.z_stage);
        const Scalar rref_x = (Scalar(1) - sc) * rf_x_;
        const Scalar rref_y = (Scalar(1) - sc) * rf_y_;
        const Scalar rref_z = (Scalar(1) - sc) * rf_z_ + sc*(Scalar(1)-sd)*Zs;

        const Scalar drx = x(0) - rref_x;
        const Scalar dry = x(1) - rref_y;
        const Scalar drz = x(2) - rref_z;
        const Scalar v2  = x.template segment<3>(3).squaredNorm();
        const Scalar touch_margin = (Scalar(P.touch_alt) - x(2)) / Scalar(P.touch_scale);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);
        const Scalar dtouch_dz = touch_margin > Scalar(0) ? -Scalar(1) / Scalar(P.touch_scale) : Scalar(0);

        const Scalar ddrx_dx0 = Scalar(1) + sc_gx0 * rf_x_;
        const Scalar ddrx_dx1 = sc_gx1 * rf_x_;
        const Scalar ddry_dx0 = sc_gx0 * rf_y_;
        const Scalar ddry_dx1 = Scalar(1) + sc_gx1 * rf_y_;

        const Scalar ddrz_dx0 = sc_gx0*(rf_z_ - Zs*(Scalar(1)-sd)) + sc*sd_gx0*Zs;
        const Scalar ddrz_dx1 = sc_gx1*(rf_z_ - Zs*(Scalar(1)-sd)) + sc*sd_gx1*Zs;
        const Scalar ddrz_dz  = Scalar(1) + sc * sd_gz * Zs;

        Vector<Scalar> g = Vector<Scalar>::Zero(NX_SS);

        g(0) = Scalar(2)*w_pos_xy_*(drx*ddrx_dx0 + dry*ddry_dx0)
             + Scalar(2)*w_pos_z_ * drz * ddrz_dx0;
        g(1) = Scalar(2)*w_pos_xy_*(drx*ddrx_dx1 + dry*ddry_dx1)
             + Scalar(2)*w_pos_z_ * drz * ddrz_dx1;
        g(2) = Scalar(2)*w_pos_z_ * drz * ddrz_dz;

        g(0) += sd_gx0 * w_vel_ * v2;
        g(1) += sd_gx1 * w_vel_ * v2;
        g(2) += sd_gz  * w_vel_ * v2;
        g.template segment<3>(3) =
            (Scalar(2)*eps_ + Scalar(2)*sd*w_vel_ + Scalar(2)*Scalar(P.w_touch_vel)*touch*touch)
            * x.template segment<3>(3);
        g(2) += Scalar(2)*Scalar(P.w_touch_vel)*touch*dtouch_dz*v2;
        g.template segment<3>(10) = Scalar(2)*eps_ * x.template segment<3>(10);

        return g;
    }

    Vector<Scalar> qu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)x;
        Vector<Scalar> g = Vector<Scalar>::Zero(NU_SS);
        g(IDX_THETA) = Scalar(P.stage_w_time) + Scalar(2 * P.r_theta) * (u(IDX_THETA) - Scalar(P.theta_ref));
        g(0) = Scalar(2 * P.r_ctrl_thr) * (u(0) - Scalar(HOVER_THRUST));
        g(1) = Scalar(2 * P.r_ctrl_mom) * u(1);
        g(2) = Scalar(2 * P.r_ctrl_mom) * u(2);
        g(3) = Scalar(2 * P.r_ctrl_mom) * u(3);
        return g;
    }

    Matrix<Scalar> qxx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)u;
        const Scalar rxy2 = x(0)*x(0) + x(1)*x(1);

        Scalar sc, sc_gx0, sc_gx1;
        sigmaLandGrad(rxy2, x(0), x(1), sc, sc_gx0, sc_gx1);
        Scalar sd, sd_gx0, sd_gx1, sd_gz;
        sigmaDescGrad(rxy2, x(0), x(1), x(2), sd, sd_gx0, sd_gx1, sd_gz);
        const Scalar Zs = Scalar(P.z_stage);
        const Scalar touch_margin = (Scalar(P.touch_alt) - x(2)) / Scalar(P.touch_scale);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);
        const Scalar dtouch_dz = touch_margin > Scalar(0) ? -Scalar(1) / Scalar(P.touch_scale) : Scalar(0);
        const Scalar v2  = x.template segment<3>(3).squaredNorm();

        const Scalar ddrx_dx0 = Scalar(1) + sc_gx0 * rf_x_;
        const Scalar ddrx_dx1 = sc_gx1 * rf_x_;
        const Scalar ddry_dx0 = sc_gx0 * rf_y_;
        const Scalar ddry_dx1 = Scalar(1) + sc_gx1 * rf_y_;
        const Scalar ddrz_dx0 = sc_gx0*(rf_z_ - Zs*(Scalar(1)-sd)) + sc*sd_gx0*Zs;
        const Scalar ddrz_dx1 = sc_gx1*(rf_z_ - Zs*(Scalar(1)-sd)) + sc*sd_gx1*Zs;
        const Scalar ddrz_dz  = Scalar(1) + sc * sd_gz * Zs;

        Matrix<Scalar> H = Matrix<Scalar>::Zero(NX_SS, NX_SS);

        H(0,0) = Scalar(2)*w_pos_xy_*(ddrx_dx0*ddrx_dx0 + ddry_dx0*ddry_dx0)
               + Scalar(2)*w_pos_z_*ddrz_dx0*ddrz_dx0;
        H(1,1) = Scalar(2)*w_pos_xy_*(ddrx_dx1*ddrx_dx1 + ddry_dx1*ddry_dx1)
               + Scalar(2)*w_pos_z_*ddrz_dx1*ddrz_dx1;
        H(2,2) = Scalar(2)*w_pos_z_*ddrz_dz*ddrz_dz;
        H(0,1) = H(1,0) = Scalar(2)*w_pos_xy_*(ddrx_dx0*ddrx_dx1 + ddry_dx0*ddry_dx1)
                         + Scalar(2)*w_pos_z_*ddrz_dx0*ddrz_dx1;
        H(0,2) = H(2,0) = Scalar(2)*w_pos_z_*ddrz_dx0*ddrz_dz;
        H(1,2) = H(2,1) = Scalar(2)*w_pos_z_*ddrz_dx1*ddrz_dz;

        for (int i = 3;  i < 6;  ++i) {
            H(i,i) = Scalar(2)*eps_ + Scalar(2)*sd*w_vel_
                   + Scalar(2)*Scalar(P.w_touch_vel)*touch*touch;
            // Gauss-Newton curvature of sqrt(P.w_touch_vel)*touch(z)*v.
            // Its mixed block is 2*w*touch*dtouch*v, matching both diagonals.
            H(2,i) += Scalar(2)*Scalar(P.w_touch_vel)*touch*dtouch_dz*x(i);
            H(i,2) += Scalar(2)*Scalar(P.w_touch_vel)*touch*dtouch_dz*x(i);
        }
        H(2,2) += Scalar(2)*Scalar(P.w_touch_vel)*dtouch_dz*dtouch_dz*v2;
        for (int i = 10; i < 13; ++i) H(i,i) = Scalar(2)*eps_;
        return H;
    }

    Matrix<Scalar> quu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)x; (void)u;
        Matrix<Scalar> H = Matrix<Scalar>::Zero(NU_SS, NU_SS);
        H(0, 0) = Scalar(2 * P.r_ctrl_thr);
        H(1, 1) = Scalar(2 * P.r_ctrl_mom);
        H(2, 2) = Scalar(2 * P.r_ctrl_mom);
        H(3, 3) = Scalar(2 * P.r_ctrl_mom);
        H(IDX_THETA, IDX_THETA) = Scalar(2 * P.r_theta);
        return H;
    }

    Matrix<Scalar> qxu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)x; (void)u;
        return Matrix<Scalar>::Zero(NX_SS, NU_SS);
    }
};

// ── Terminal cost ─────────────────────────────────────────────────────────────
template<typename Scalar>
class RelTermCost : public TerminalCostBase<Scalar> {
    double wp_xy_, wp_z_, wv_xy_, wv_z_, watt_, wom_, vz_ref_;
public:
    explicit RelTermCost(double wp_xy=500.0, double wp_z=500.0,
                         double wv_xy=50.0, double wv_z=100.0,
                         double watt=200.0, double wom=50.0, double vz_ref=-0.1)
        : wp_xy_(wp_xy), wp_z_(wp_z), wv_xy_(wv_xy), wv_z_(wv_z),
          watt_(watt), wom_(wom), vz_ref_(vz_ref) {}

    Scalar p(const Vector<Scalar>& x) const override {
        Scalar ep   = x.template segment<2>(0).squaredNorm();
        Scalar ez   = x(2)*x(2);
        Scalar evxy = x.template segment<2>(3).squaredNorm();
        Scalar evz  = (x(5) - Scalar(vz_ref_)) * (x(5) - Scalar(vz_ref_));
        Scalar etilt = x(7)*x(7) + x(8)*x(8);
        Scalar eom  = x.template segment<3>(10).squaredNorm();
        return Scalar(0.5) * (
               Scalar(wp_xy_)*ep + Scalar(wp_z_)*ez
             + Scalar(wv_xy_)*evxy + Scalar(wv_z_)*evz
             + Scalar(watt_)*etilt
             + Scalar(wom_)*eom);
    }

    Vector<Scalar> px(const Vector<Scalar>& x) const override {
        Vector<Scalar> g = Vector<Scalar>::Zero(x.size());
        g.segment(0,2) = Scalar(wp_xy_)*x.segment(0,2);
        g(2)           = Scalar(wp_z_)*x(2);
        g.segment(3,2) = Scalar(wv_xy_)*x.segment(3,2);
        g(5)           = Scalar(wv_z_)*(x(5) - Scalar(vz_ref_));
        g(7) = Scalar(watt_)*x(7);
        g(8) = Scalar(watt_)*x(8);
        g.segment(10,3) = Scalar(wom_)*x.segment(10,3);
        return g;
    }

    Matrix<Scalar> pxx(const Vector<Scalar>& x) const override {
        Matrix<Scalar> H = Matrix<Scalar>::Zero(x.size(), x.size());
        H(0,0)=H(1,1) = Scalar(wp_xy_);
        H(2,2) = Scalar(wp_z_);
        H(3,3)=H(4,4) = Scalar(wv_xy_);
        H(5,5) = Scalar(wv_z_);
        H(7,7)=H(8,8) = Scalar(watt_);
        H(10,10)=H(11,11)=H(12,12) = Scalar(wom_);
        return H;
    }
};

// ── Stage constraints ─────────────────────────────────────────────────────────
template<typename Scalar>
class FminCon : public StageConstraintBase<Scalar> { public:
    FminCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << Scalar(P.fmin)-u(0)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(1, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,u.size()); J(0,0)=Scalar(-1); return J; }};

template<typename Scalar>
class FmaxCon : public StageConstraintBase<Scalar> { public:
    FmaxCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << u(0)-Scalar(P.fmax)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(1, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,u.size()); J(0,0)=Scalar(1); return J; }};

template<typename Scalar>
class MomentCon : public StageConstraintBase<Scalar> { Scalar tau_; public:
    MomentCon() : tau_(static_cast<Scalar>(TAU_MAX)) {
        this->constraint_type=ConstraintType::SOC; this->dim_c=4; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Vector<Scalar> cn(4); cn << tau_, u(1), u(2), u(3); return -cn; }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(4, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(4,u.size());
        J(1,1)=J(2,2)=J(3,3)=Scalar(-1); return J; }};

template<typename Scalar>
class ThetaBounds : public StageConstraintBase<Scalar> { Scalar lo_, hi_; public:
    ThetaBounds(double lo, double hi) : lo_(lo), hi_(hi) {
        this->constraint_type=ConstraintType::NO; this->dim_c=2; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Vector<Scalar> cn(2); cn(0)=u(IDX_THETA)-hi_; cn(1)=lo_-u(IDX_THETA); return cn; }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(2, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(2,u.size());
        J(0,IDX_THETA)=Scalar(1); J(1,IDX_THETA)=Scalar(-1); return J; }};

template<typename Scalar>
class ScaledCon : public StageConstraintBase<Scalar> {
    std::shared_ptr<StageConstraintBase<Scalar>> original_;
    Scalar inverse_;
public:
    ScaledCon(std::shared_ptr<StageConstraintBase<Scalar>> original,double scale)
        :original_(std::move(original)),inverse_(Scalar(1./scale)){
        this->constraint_type=original_->getConstraintType();this->dim_c=original_->getDimC();
    }
    Vector<Scalar> c(const Vector<Scalar>& x,const Vector<Scalar>& u) const override{return original_->c(x,u)*inverse_;}
    Matrix<Scalar> cx(const Vector<Scalar>& x,const Vector<Scalar>& u) const override{return original_->cx(x,u)*inverse_;}
    Matrix<Scalar> cu(const Vector<Scalar>& x,const Vector<Scalar>& u) const override{return original_->cu(x,u)*inverse_;}
};

template<typename Scalar>
class ZFloorCon : public StageConstraintBase<Scalar> { public:
    ZFloorCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return (Vector<Scalar>(1) << -x(2)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,x.size()); J(0,2)=Scalar(-1); return J; }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return Matrix<Scalar>::Zero(1, u.size()); }};

template<typename Scalar>
class GeneralSpeedCon : public StageConstraintBase<Scalar> { public:
    GeneralSpeedCon() { this->constraint_type=ConstraintType::SOC; this->dim_c=4; }
    Vector<Scalar> c(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        Vector<Scalar> cn(4); cn << Scalar(P.spd_phase0_max), x(3), x(4), x(5); return -cn; }
    Matrix<Scalar> cx(const Vector<Scalar>&, const Vector<Scalar>&) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(4,NX_SS);
        J(1,3)=Scalar(-1); J(2,4)=Scalar(-1); J(3,5)=Scalar(-1); return J; }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(4, NU_SS); }};

// The shared interface owns the production CTCS row and its interval
// sensitivities. The host's original integrand/integral above remain an
// independent diagnostic reference. Weights and epsilon both use Y_SCALE.
template<typename Scalar, typename Dynamics>
std::shared_ptr<StcStageConstraint<Scalar>> makeCtcsConstraint(
    std::shared_ptr<Dynamics> dyn,
    double eps = P.ctcs_step_eps / P.ctcs_y_scale,
    int idx_theta = IDX_THETA, double fixed_dt = 0.0, int substeps = 1,bool tightened=true) {
    using Vec = Vector<Scalar>;
    using StateFn = std::function<Scalar(const Vec&)>;
    using GradFn = std::function<Vec(const Vec&)>;
    const auto margins=ctcsMargins(tightened);
    const auto floors=triggerFloors(tightened);
    if(margins.trigger<0.||margins.trigger>=P.stage_radius||margins.stage_radius<0.||margins.stage_radius>=P.stage_radius||margins.geometric<0.||margins.stage_height<0.||margins.speed<0.||margins.speed>=P.spd_stc_max||
        margins.tilt_deg<0.||margins.tilt_deg>=P.alpha_theta_stc*P.tilt_stc_deg||margins.path_tilt_deg<0.||margins.path_tilt_deg>=P.tilt_phase0_deg||margins.rate<0.||margins.rate>=P.omega_stc_max||
        margins.los<0.||margins.los/P.gs_apex_offset>=LOS_CONE_TAN||
        margins.gs_angle_deg<0.||margins.gs_angle_deg>=P.gs_cone_deg||
        margins.geometric>=gsConeTan(margins)*P.gs_apex_offset)
        throw std::invalid_argument("invalid CTCS physical tightening margins");
    auto row=std::make_shared<StcStageConstraint<Scalar>>(StcForm::Density, eps);
    const auto zero = [](const Vec& x) -> Vec { return Vec::Zero(x.size()); };
    const auto always = [](const Vec&) -> Scalar { return Scalar(-1); };
    const auto tilt = [](const Vec& x) -> Scalar { return x(7)*x(7) + x(8)*x(8); };
    const auto dtilt = [](const Vec& x) -> Vec {
        Vec g = Vec::Zero(x.size()); g(7) = Scalar(2)*x(7); g(8) = Scalar(2)*x(8); return g;
    };
    const auto omega = [](const Vec& x) -> Scalar { return x.template segment<3>(10).squaredNorm(); };
    const auto domega = [](const Vec& x) -> Vec {
        Vec g = Vec::Zero(x.size()); g.template segment<3>(10) = Scalar(2)*x.template segment<3>(10); return g;
    };
    const auto tiltLimit = [](double degrees) {
        return Scalar(std::pow(std::sin(0.5 * degrees * M_PI / 180.0), 2));
    };
    auto add = [&](const std::string& name, StateFn trigger, GradFn dtrigger,
                   StateFn consequence, GradFn dconsequence, double weight) {
        row->add(name).triggerWhenNegative(std::move(trigger), std::move(dtrigger))
            .consequence(
                [consequence](const Vec& x, const Vec&) -> Scalar { return consequence(x); },
                [dconsequence](const Vec& x, const Vec&) -> Vec { return dconsequence(x); },
                [](const Vec&, const Vec& u) -> Vec { return Vec::Zero(u.size()); })
            .weight(Scalar(weight / P.ctcs_y_scale));
    };
    const Scalar looseTilt = tiltLimit(P.tilt_phase0_deg-margins.path_tilt_deg);
    add("loose_tilt", always, zero,
        [=](const Vec& x) -> Scalar { return tilt(x) - looseTilt; }, dtilt, 1.0);
    add("loose_omega", always, zero,
        [=](const Vec& x) -> Scalar { return omega(x) - Scalar(std::pow(P.omega_phase0_max-margins.rate,2)); }, domega, 1.0);

    {
        StateFn stageTrigger = [=](const Vec& x) -> Scalar {
            const Scalar margin = (std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) -
                                   Scalar(P.stage_radius-margins.stage_radius)) / Scalar(P.stage_trigger_scale);
            return margin > Scalar(0) ? -(Scalar(floors.stage) + margin) : Scalar(0);
        };
        GradFn dstage = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            const Scalar r = std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
            if (r > Scalar(P.stage_radius-margins.stage_radius)) {
                g(0) = -x(0)/(r*Scalar(P.stage_trigger_scale));
                g(1) = -x(1)/(r*Scalar(P.stage_trigger_scale));
            }
            return g;
        };
        add("staging", stageTrigger, dstage,
            [=](const Vec& x) -> Scalar { return Scalar(P.z_stage+margins.stage_height) - x(2); },
            [](const Vec& x) -> Vec { Vec g = Vec::Zero(x.size()); g(2) = Scalar(-1); return g; }, P.w_stc_stage);

        StateFn losTrigger = [=](const Vec& x) -> Scalar {
            const Scalar m = (Scalar(P.los_alt_trig+margins.trigger) - x(2))/Scalar(P.los_trigger_scale);
            return m > Scalar(0) ? -(Scalar(floors.los) + m) : Scalar(0);
        };
        GradFn dlosTrigger = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (x(2) < Scalar(P.los_alt_trig+margins.trigger)) g(2) = Scalar(1)/Scalar(P.los_trigger_scale);
            return g;
        };
        // Body boresight and its quaternion derivatives, without normalizing
        // intermediate RK4 states (the deployed interval uses them as-is).
        const auto boresight = [](const Vec& x) -> Eigen::Matrix<Scalar,3,1> {
            const Scalar w=x(6), a=x(7), b=x(8), c=x(9);
            return {Scalar(2)*(a*c+w*b), Scalar(2)*(b*c-w*a), w*w-a*a-b*b+c*c};
        };
        const double los_tan=LOS_CONE_TAN;
        StateFn los = [=](const Vec& x) -> Scalar {
            const Scalar axial = x.template head<3>().dot(boresight(x));
            const Scalar lateral2 = x.template head<3>().squaredNorm() - axial*axial;
            // Taper the distance margin to an angular margin at the apex.
            // For axial<0, |tanh(s)|<=|s| and margin/range<tan(angle) keep the
            // entire rear half-space infeasible. The landing point is retained.
            return std::sqrt(posPart(lateral2)+Scalar(1e-12)) - Scalar(los_tan)*axial
                + Scalar(margins.los)*std::tanh(axial/Scalar(P.gs_apex_offset));
        };
        GradFn dlos = [=](const Vec& x) -> Vec {
            const Scalar w=x(6), a=x(7), b=x(8), c=x(9);
            const auto axis = boresight(x);
            const Scalar axial = x.template head<3>().dot(axis);
            Vec da = Vec::Zero(x.size()); da.template head<3>() = axis;
            Eigen::Matrix<Scalar,3,4> db;
            db << Scalar(2)*b, Scalar(2)*c, Scalar(2)*w, Scalar(2)*a,
                  -Scalar(2)*a, -Scalar(2)*w, Scalar(2)*c, Scalar(2)*b,
                  Scalar(2)*w, -Scalar(2)*a, -Scalar(2)*b, Scalar(2)*c;
            da.template segment<4>(6) = db.transpose()*x.template head<3>();
            const Scalar lateral2 = x.template head<3>().squaredNorm()-axial*axial;
            const Scalar taper=std::tanh(axial/Scalar(P.gs_apex_offset));
            Vec g = (-Scalar(los_tan)+Scalar(margins.los/P.gs_apex_offset)*(Scalar(1)-taper*taper))*da;
            if (lateral2 > Scalar(0)) {
                Vec dl = -Scalar(2)*axial*da; dl.template head<3>() += Scalar(2)*x.template head<3>();
                g += dl/(Scalar(2)*std::sqrt(lateral2+Scalar(1e-12)));
            }
            return g;
        };
        add("los", losTrigger, dlosTrigger, los, dlos, P.w_stc_los);

        StateFn landTrigger = [=](const Vec& x) -> Scalar {
            const Scalar depth = Scalar(P.alt_trig+margins.trigger) - x(2);
            return depth > Scalar(0) ? -(Scalar(floors.landing) + depth) : -depth;
        };
        GradFn dland = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size()); g(2) = Scalar(1); return g;
        };
        add("landing_speed", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return x.template segment<3>(3).squaredNorm()-Scalar(std::pow(P.spd_stc_max-margins.speed,2)); },
            [](const Vec& x) -> Vec { Vec g=Vec::Zero(x.size()); g.template segment<3>(3)=Scalar(2)*x.template segment<3>(3); return g; }, P.w_land_speed);
        const Scalar landTilt = tiltLimit(P.alpha_theta_stc*P.tilt_stc_deg-margins.tilt_deg);
        add("landing_tilt", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return tilt(x)-landTilt; }, dtilt, P.w_land_tilt);
        add("landing_omega", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return omega(x)-Scalar(std::pow(P.omega_stc_max-margins.rate,2)); }, domega, P.w_land_omega);
        add("landing_glideslope", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) - Scalar(gsConeTan(margins))*(x(2)+Scalar(P.gs_apex_offset))+Scalar(margins.geometric); },
            [=](const Vec& x) -> Vec {
                Vec g=Vec::Zero(x.size()); const Scalar r=std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
                g(0)=x(0)/r; g(1)=x(1)/r; g(2)=-Scalar(gsConeTan(margins)); return g;
            }, P.w_land_gs);
    }
    StcIntervalDynamics<Scalar> host;
    host.idx_theta = idx_theta;
    host.dt = fixed_dt;
    host.substeps = substeps;
    host.xdot = [dyn](const Vec& x, const Vec& u, Scalar) -> Vec { return dyn->intervalRates(x,u); };
    host.fx = [dyn](const Vec& x, const Vec& u, Scalar) -> Matrix<Scalar> { return dyn->intervalRatesX(x,u); };
    host.fu = [dyn](const Vec& x, const Vec& u, Scalar) -> Matrix<Scalar> { return dyn->intervalRatesU(x,u); };
    row->setIntervalDynamics(std::move(host));
    return row;
}

inline Eigen::VectorXd ensureNoAugStateSize(const Eigen::VectorXd& x) {
    if (x.size() == NX_SS) return x;
    Eigen::VectorXd out = Eigen::VectorXd::Zero(NX_SS);
    const int n = std::min<int>(x.size(), NX);
    out.head(n) = x.head(n);
    return out;
}

// ── Solver params ─────────────────────────────────────────────────────────────
inline Param getSolverParams() {
    return P.solver;
}

// ── Warm-start / validity helpers ─────────────────────────────────────────────
inline void clampWarmControl(Eigen::VectorXd& u) {
    if (u.size() < NU_SS) return;
    u(0) = std::clamp((double)u(0), (double)P.fmin, (double)P.fmax);
    u(1) = std::clamp((double)u(1), -TAU_MAX, TAU_MAX);
    u(2) = std::clamp((double)u(2), -TAU_MAX, TAU_MAX);
    u(3) = std::clamp((double)u(3), -TAU_MAX, TAU_MAX);
    u(IDX_THETA) = std::clamp((double)u(IDX_THETA), P.thl, P.thh);
}

inline bool finiteTrajectory(const std::vector<Eigen::VectorXd>& X,
                             const std::vector<Eigen::VectorXd>& U) {
    for (const auto& x : X) if (!x.allFinite()) return false;
    for (const auto& u : U) if (!u.allFinite()) return false;
    return true;
}

inline bool plausibleTrajectory(const std::vector<Eigen::VectorXd>& X,
                                int max_node, int n_solve) {
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

// ── Native initial guess ─────────────────────────────────────────────────────
// Rollout under a PD attitude/rate loop with velocity feedback toward a staged
// reference: approach the target above the LOS trigger / staging height and
// descend only along a height that stays inside a fraction of the glideslope.
// A measured root carries small body rates; with zero moments they never decay,
// so a uniform-hover guess tilts and drifts over a multi-second horizon and the
// solver starts at CTCS errors ~1e5. Without the staged reference a cold start
// far from the target descends into the LOS-triggered band outside the cone and
// stalls there. Only the initial guess changes; the OCP is unchanged.
inline Eigen::VectorXd nativeGuessControl(const Eigen::VectorXd& x,const Eigen::Vector3d& target_accel){
    constexpr double kp=1.,kd=2.,amax=2.;            // attitude PD, horizontal accel cap
    constexpr double kpos=.3,kvs=.9,vmax=1.;          // position -> velocity -> accel
    constexpr double hold=.25,slope=.6;               // hold height above max(P.z_stage,LOS trigger); GS fraction
    const Eigen::Vector3d v=x.segment<3>(3),w=x.segment<3>(10);
    Eigen::Quaterniond q(x(6),x(7),x(8),x(9));q.normalize();
    const double rxy=x.head<2>().norm();
    const double cap=std::max(P.z_stage,P.los_alt_trig)+hold;
    const double zref=std::min(cap,rxy/(slope*std::tan(P.gs_cone_deg*M_PI/180.))-P.gs_apex_offset);
    Eigen::Vector3d vref(-kpos*x(0),-kpos*x(1),kpos*(std::max(zref,0.)-x(2)));
    if(vref.head<2>().norm()>vmax)vref.head<2>()*=vmax/vref.head<2>().norm();
    vref.z()=std::clamp(vref.z(),-.5,.5);
    Eigen::Vector3d a=kvs*(vref-v);
    a.head<2>()=a.head<2>().cwiseMax(-amax).cwiseMin(amax);
    // While descending, brake at least enough to stop above the floor, so the
    // guess rollout never passes through the hard z>=0 row (weak damping from
    // ~0.5 m at -0.6 m/s otherwise drives it into the floor and the barrier
    // line search stalls on BOUNDARY_INEQ).
    constexpr double floor_z=.05;
    if(v.z()<0.)a.z()=std::max(a.z(),v.z()*v.z()/(2.*std::max(x(2)-floor_z,.02)));
    // Relative frame: the vehicle must also match the target's acceleration.
    const Eigen::Vector3d f=a+target_accel-GRAVITY;
    const Eigen::Vector3d bz=q*Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d err=q.conjugate()*bz.cross(f.normalized());
    Eigen::VectorXd u=Eigen::VectorXd::Zero(NU_SS);
    u(0)=P.mass*f.dot(bz);
    u.segment<3>(1)=J_B*(kp*err-kd*w);
    u(IDX_THETA)=P.th0;
    return u;
}

// ── Factory ───────────────────────────────────────────────────────────────────
inline std::shared_ptr<OptimalControlProblem<double>> create(
    const Eigen::VectorXd& x0_rel,
    const std::vector<Eigen::VectorXd>& prev_U = {},
    const std::vector<Eigen::VectorXd>& prev_X = {},
    const std::vector<Eigen::MatrixXd>& prev_K = {},
    std::shared_ptr<ConstantAccelPredictor> predictor = nullptr,
    int requested_horizon = 0)
{
    const int N = requested_horizon > 0 ? requested_horizon : P.horizon;
    if(N>P.horizon)throw std::invalid_argument("remaining landing horizon exceeds configured maneuver");
    auto problem = std::make_shared<OptimalControlProblem<double>>(N);

    if (!predictor) {
        predictor = std::make_shared<ConstantAccelPredictor>();
    }

    auto dyn = std::make_shared<Quad6DOFVarTimeRelativePred<double>>(predictor);
    dyn->setMass(P.mass);
    dyn->setGravity(GRAVITY);
    dyn->setJb(J_B);
    dyn->setTargetAccel(predictor->predictAccel(0.0));

    auto tcost  = std::make_shared<RelTermCost<double>>(
        P.term_w_pos_xy, P.term_w_pos_z,
        P.term_w_vel_xy, P.term_w_vel_z,
        P.term_w_tilt, P.term_w_om, P.term_vz_ref);
    auto cfmin  = std::make_shared<FminCon<double>>();
    auto cfmax  = std::make_shared<FmaxCon<double>>();
    auto cmom   = std::make_shared<MomentCon<double>>();
    auto cth    = std::make_shared<ThetaBounds<double>>(P.thl, P.thh);
    auto czfl   = std::make_shared<ZFloorCon<double>>();
    auto cspeed = std::make_shared<GeneralSpeedCon<double>>();
    // The constraint sees the SCALED integral, so the physical budget
    // P.ctcs_step_eps is divided by P.ctcs_y_scale (same zero set as unscaled).
    // Hard-wired: this OCP has no other enforcement mode.
    auto cstep = makeCtcsConstraint<double>(dyn);
    // Intervals before node CTCS_TIGHTEN_FROM_NODE use the physical
    // implications (no consequence tightening; trigger floors keep the
    // physical boundary sharp, see triggerFloors). The measured root is fixed,
    // so a root inside the physical set but in the margin band would otherwise
    // make the tightened OCP infeasible from its first interval. Without
    // tightening margins both rows are identical. Physical rows have no margin
    // to absorb a violation between quadrature points (a trigger crossed
    // between the RK4 stage points is invisible to a one-step integral), so
    // they integrate with ctcsPhysicalSubsteps() RK4 substeps.
    const int tighten_from=ctcsTightenFromNode();
    auto cstep_physical = tighten_from>0 ?
        makeCtcsConstraint<double>(dyn, P.ctcs_step_eps / P.ctcs_y_scale, IDX_THETA, 0.0, ctcsPhysicalSubsteps(), false) : cstep;
    const double eps_cost = P.w_eps;
    const Eigen::Vector3d r_follow(0.0, 0.0, P.z_stage);
    auto stage_cost = std::make_shared<CTcSTCStageCost<double>>(
        eps_cost,
        P.w_pos_xy,
        P.w_pos_z,
        P.w_vel,
        r_follow);

    for (int k = 0; k < N; ++k) {
        problem->setStageDynamics(k, dyn);
        problem->setStageCost(k, stage_cost);
        if(P.scale_bounds){
            // Same positive row factors as the supplied quad_sh_ctcs baseline.
            problem->addStageConstraint(k,std::make_shared<ScaledCon<double>>(cfmin,P.fmax-P.fmin));
            problem->addStageConstraint(k,std::make_shared<ScaledCon<double>>(cfmax,P.fmax-P.fmin));
            problem->addStageConstraint(k,cmom);
            problem->addStageConstraint(k,std::make_shared<ScaledCon<double>>(cth,P.thh-P.thl));
        }else{
            problem->addStageConstraint(k,cfmin);
            problem->addStageConstraint(k,cfmax);
            problem->addStageConstraint(k,cmom);
            problem->addStageConstraint(k,cth);
        }
        problem->addStageConstraint(k, czfl);
        problem->addStageConstraint(k, cspeed);
        problem->addStageConstraint(k, k < tighten_from ? cstep_physical : cstep);
    }
    problem->setTerminalCost(tcost);
    // No CT-cSTC terminal constraint: the interval inequality above is the
    // entire enforcement (the terminal-eq y_N=0 belongs to the augmented arm).

    Eigen::VectorXd x0ss(NX_SS);
    x0ss.setZero();
    x0ss.segment(0, NX) = x0_rel.head(NX);
    x0ss(IDX_DT) = 0.0;
    problem->setInitialState(0, x0ss);

    // Rebase the shifted warm-start references: IDX_DT is a per-solve
    // accumulator, but the core's makeXshifted() does a plain index shift, so
    // prev_X[0] here is the previously executed node carrying the old
    // horizon's t0. Subtract it (benchmark makeXshifted equivalent) and pin
    // the first reference to the new initial state.
    std::vector<Eigen::VectorXd> Xw = prev_X;
    if (!Xw.empty()) {
        const Eigen::VectorXd x_base = ensureNoAugStateSize(Xw[0]);
        const double t0_shift = x_base(IDX_DT);
        bool finite = true;
        for (auto& x : Xw) {
            x = ensureNoAugStateSize(x);
            if (!x.allFinite()) { finite = false; break; }
            x(IDX_DT) = std::max(0.0, x(IDX_DT) - t0_shift);
        }
        if (finite) {
            Xw[0] = x0ss;
        } else {
            Xw.clear();
        }
    }

    const bool use_feedback =
        true &&
        static_cast<int>(prev_U.size()) >= N &&
        static_cast<int>(Xw.size()) > N &&
        static_cast<int>(prev_K.size()) >= N;

    Eigen::VectorXd sim14 = x0ss;
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd uk(NU_SS);
        uk.setZero();
        if (static_cast<int>(prev_U.size()) >= N) {
            uk = prev_U[k];
            if (use_feedback) {
                const Eigen::VectorXd& x_ref = Xw[k];
                const Eigen::MatrixXd& K = prev_K[k];
                Eigen::VectorXd dx = sim14 - x_ref;
                Eigen::VectorXd u_fb = uk;
                bool feedback_dim_ok = (K.rows() == NU_SS && K.cols() <= dx.size());
                if (feedback_dim_ok) {
                    u_fb.noalias() = uk + K * dx.head(K.cols());
                }
                Eigen::VectorXd du = Eigen::VectorXd::Zero(NU_SS);
                if (feedback_dim_ok) du = u_fb - uk;
                const double max_du = P.max_feedback_du;
                const bool wild_feedback =
                    !feedback_dim_ok ||
                    !u_fb.allFinite() ||
                    !du.allFinite() ||
                    (max_du > 0.0 && du.norm() > max_du) ||
                    u_fb(0) < 0.5 * P.fmin ||
                    u_fb(0) > 1.5 * P.fmax ||
                    std::abs(u_fb(1)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(2)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(3)) > 2.0 * TAU_MAX ||
                    u_fb(IDX_THETA) < 0.0 ||
                    u_fb(IDX_THETA) > 2.0 * P.thh;
                if (!wild_feedback) {
                    uk = u_fb;
                }
            }

        } else {
            // Native guess (see nativeGuessControl). It replaces the uniform
            // hover input of the retained single-horizon benchmark.
            uk = nativeGuessControl(sim14, predictor->predictAccel(sim14(13)));
        }
        clampWarmControl(uk);

        problem->setInitialControl(k, uk);
        Eigen::VectorXd sim14_next = dyn->propagate14(sim14, uk.segment(0, NU), uk(IDX_THETA));
        sim14_next(2) = std::max(sim14_next(2), 0.0);
        problem->setInitialState(k + 1, sim14_next);
        sim14 = sim14_next;
    }

    return problem;
}

// ── Descriptor ────────────────────────────────────────────────────────────────
inline OCPDescriptor descriptor() {
    OCPDescriptor d;d.disarm_on_landing_finish=true;
    d.name = "stc_landing_noaug";
    d.dt = P.th0;
    d.default_n_replay = P.nex;
    d.default_mass_kg = P.mass;
    d.warm_start = OCPDescriptor::WarmStart::Feedback;
    // A shifted plan from a different measured root leaves the CTCS solve
    // uncertified; the staged nativeGuessControl seed converges.
    d.native_guess_for_measured_root = true;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.world_acceleration=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u) {
        if(x.size()<13||u.size()<1||!x.allFinite()||!u.allFinite())throw std::invalid_argument("invalid DDP acceleration input");
        Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
        if(q.norm()<1e-8)throw std::invalid_argument("invalid DDP attitude");
        return Eigen::Vector3d(q.normalized()*Eigen::Vector3d(0,0,u(0)/P.mass)+Eigen::Vector3d(0,0,-9.81));
    };
    // Invert PX4 PositionControl's native acceleration conversion at the
    // nominal DDP state. With zero position/velocity error and integrator,
    // acc_internal = acc_feedforward - D_vel * acc_physical. In decoupled
    // mode its tilt axis is [acc_internal.xy, g], rather than thrust*q_z.
    // This is a coordinate/input conversion, without measured-error feedback.
    d.mavros_acceleration=[physical=d.world_acceleration](const Eigen::VectorXd& x,const Eigen::VectorXd& u) {
        const Eigen::Vector3d acceleration=physical(x,u);
        Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
        const Eigen::Vector3d axis=q.normalized()*Eigen::Vector3d::UnitZ();
        Eigen::Vector3d feedforward=acceleration;
        if(comando_parameters::defaults.px4.decouple_acceleration) {
            if(axis.z()<=1e-6)throw std::invalid_argument("PX4 acceleration mapping requires upright thrust axis");
            feedforward.head<2>()=9.81*axis.head<2>()/axis.z();
        }
        feedforward.x()+=comando_parameters::defaults.px4.velocity_d_xy*acceleration.x();
        feedforward.y()+=comando_parameters::defaults.px4.velocity_d_xy*acceleration.y();
        feedforward.z()+=comando_parameters::defaults.px4.velocity_d_z*acceleration.z();
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
        if (t.valid) {
            r.target_snapshot_pos = t.position;
            r.target_snapshot_vel = t.velocity;
            r.target_snapshot_acc = t.acceleration;
        } else {
            r.target_snapshot_pos.setZero();
            r.target_snapshot_vel.setZero();
            r.target_snapshot_acc.setZero();
        }
        // Ported benchmark validity gates: refuse nonfinite/implausible plans
        // so PlannerCore rejects the solve and restores the warm-start snapshot.
        if (r.success &&
            (!finiteTrajectory(r.state_trajectory, r.control_trajectory) ||
             !plausibleTrajectory(r.state_trajectory,
                                  std::min(P.nex, P.horizon), P.horizon))) {
            r.success = false;
        }
    };
    d.prepare_extra = [](const PlannerConfig&, double, const TargetSnapshot& tgt_snapshot) {
        StcLandingNoAugExtra ex;
        ex.predictor = std::make_shared<ConstantAccelPredictor>();
        ex.predictor->setState(tgt_snapshot.position,
                               tgt_snapshot.velocity,
                               tgt_snapshot.acceleration);
        return std::any(ex);
    };

    d.sanitize_warm_control = [](Eigen::VectorXd& u) { clampWarmControl(u); };
    d.make_model=[](const Eigen::Vector3d& accel) {
        auto pred=std::make_shared<ConstantAccelPredictor>();pred->a=accel;
        auto dyn=std::make_shared<Quad6DOFVarTimeRelativePred<double>>(pred);
        dyn->setMass(P.mass);dyn->setGravity(GRAVITY);dyn->setJb(J_B);
        OCPDescriptor::Model model;model.nx=NX_SS;model.nu=NU_SS;model.horizon=P.horizon;model.dt=P.th0;
        model.hover=Eigen::VectorXd::Zero(NU_SS);model.hover(0)=HOVER_THRUST;model.hover(IDX_THETA)=P.th0;
        model.stage_duration=[](const Eigen::VectorXd&,const Eigen::VectorXd& u){return u(IDX_THETA);};
        model.initial_state=[](const Eigen::VectorXd& physical,const Eigen::VectorXd&){
            Eigen::VectorXd x=Eigen::VectorXd::Zero(NX_SS);x.head(NX)=physical.head(NX);return x;
        };
        model.bounded=[](Eigen::VectorXd u){clampWarmControl(u);return u;};
        model.flow=[dyn](const Eigen::VectorXd& x,const Eigen::VectorXd& u,double h,bool jac) {
            auto control=u;control(IDX_THETA)=h;
            OCPDescriptor::Flow result;result.x=dyn->f(x,control);
            if(jac){result.a=dyn->fx(x,control);result.b=dyn->fu(x,control);result.b.col(IDX_THETA).setZero();}
            return result;
        };return model;
    };
    d.getSolverParams = getSolverParams;
    d.create = [](const OCPCreateArgs& a) {
        StcLandingNoAugExtra ex;
        if (a.extra.has_value()) {
            try { ex = std::any_cast<StcLandingNoAugExtra>(a.extra); }
            catch (const std::bad_any_cast&) {}
        }
        auto pred = ex.predictor;
        if (!pred) {
            pred = std::make_shared<ConstantAccelPredictor>();
            pred->setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           a.target_accel);
        }
        return create(a.current_state, a.prev_U, a.prev_X, a.prev_K, pred, a.horizon);
    };
    return d;
}

// Ordinary NODE baseline on this same landing model/cost. Keep the original
// unconditional input, floor, duration and global-speed rows. Exclude the STC
// interval row rather than making its conditional consequences unconditional.
inline std::shared_ptr<OptimalControlProblem<double>> ordinaryNodeProblem(
    const OptimalControlProblem<double>& source) {
    const int n=source.getHorizon();
    auto problem=std::make_shared<OptimalControlProblem<double>>(n);
    for(int k=0;k<n;++k){
        problem->setStageDynamics(k,source.getDynamics(k));
        problem->setStageCost(k,source.getStageCost(k));
        problem->setInitialState(k,source.getInitialState(k));
        problem->setInitialControl(k,source.getInitialControl(k));
        int excluded=0;
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
            for(const auto& row:source.getStageConstraints(k,type)){
                if(auto stc=std::dynamic_pointer_cast<StcStageConstraint<double>>(row)){
                    if(stc->getForm()==StcForm::Density)++excluded;
                }
                else problem->addStageConstraint(k,row);
            }
        if(excluded!=1)throw std::runtime_error("ordinary NODE expects exactly one original STC row");
    }
    problem->setInitialState(n,source.getInitialState(n));
    problem->setTerminalCost(source.getTerminalCost());
    for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
        if(!source.getTerminalConstraints(type).empty())
            throw std::runtime_error("ordinary NODE baseline must not introduce terminal constraints");
    return problem;
}

inline OCPDescriptor ordinaryNodeDescriptor(){
    auto d=descriptor();d.name="stc_landing_noaug_node";
    d.native_guess_for_measured_root=false;  // recorded node baseline uses the shift
    const auto original=d.create;
    d.create=[original](const OCPCreateArgs& args){return ordinaryNodeProblem(*original(args));};
    return d;
}
}  // namespace StcLandingNoAugOCP
