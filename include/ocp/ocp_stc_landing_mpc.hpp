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

namespace StcLandingMpcOCP::Formulation {

constexpr int NX=13, NU=4, NX_SS=14, NU_SS=5, IDX_DT=13, IDX_THETA=4;
// Every number this OCP uses.
struct Params {
    double mass=0.027;
    double ixx=1.66e-5;
    double iyy=1.66e-5;
    double izz=2.92e-5;
    double actuator_fmin=0.08;
    double actuator_fmax=0.60;
    double fmin=0.12;
    double fmax=0.52;
    double l_arm=0.046;
    int horizon=30;
    int nex=7;
    double th0=0.07;
    double thl=0.05;
    double thh=0.12;
    double term_w_pos_xy=2500.0;
    double term_w_pos_z=2500.0;
    double term_w_vel_xy=900.0;
    double term_w_vel_z=900.0;
    double term_w_tilt=200.0;
    double term_w_om=50.0;
    double term_vz_ref=0.0;
    double stage_w_time=0.04;
    double r_theta=10.0;
    double theta_ref=0.10;
    double r_ctrl_thr=2.0;
    double r_ctrl_mom=0.6;
    double w_touch_vel=10.0;
    double touch_alt=0.25;
    double touch_scale=0.25;
    double w_stage_target_xy=0.0;
    double w_stage_target_z=0.0;
    double w_stage_target_vel=0.0;
    double w_land_speed=1.0;
    double w_land_tilt=1.0e5;
    double w_land_omega=100.0;
    double w_land_gs=5000.0;
    double w_stc_los=50.0;
    double w_stc_stage=10.0;
    double cap_cost_radius=0.50;
    double k_cap=20.0;
    double cap_desc_radius=0.12;
    double k_desc=40.0;
    double gs_apex_offset=0.1;
    double vz_land_max=0.5;
    double theta_phase0_deg=11.0;
    double omega_phase0_max=0.35;
    double spd_phase0_max=3.0;
    double theta_stc_deg=0.95;
    double alpha_theta_stc=1.0;
    double omega_stc_max=0.019;
    double gs_cone_half_angle_deg=15.0;
    double spd_stc_max=0.75;
    double los_cone_half_angle_deg=30.0;
    double los_alt_trig=1.8;
    double los_trigger_scale=0.25;
    double los_trigger_floor=0.0;
    double alt_trig=0.8;
    double cap_land_radius=0.45;
    double z_stage=1.8;
    double stage_radius=1.10;
    double stage_trigger_scale=0.25;
    double stage_trigger_floor=0.0;
    double stage_settle_tilt_deg=12.0;
    double stage_settle_omega_max=0.50;
    double ctcs_y_scale=10.0;
    double ctcs_step_eps=1.6e-3;
    bool triggered_stc=true, capture_landing_trigger=false, feedback_warm_start=true;
    std::string stage_trigger="radius";
    double slew_thrust_weight=0.1, slew_rate_weight=2.0, angular_rate_weight=0.0;  // direct-MPC command costs
    int mpc_max_iter=60, warm_iterations=8;
    Param solver=[]{Param p;p.reg1_min=1e-3;p.reg2_min=0.1;p.mu_mul=0.1;p.rho=5;p.rhoT=5;p.rho_mul=10;p.tolerance=1e-6;
        p.max_iter=180;p.verbose=false;p.is_quaternion_in_state=false;p.use_ddp_terms=false;return p;}();
};
// Validated configurations:
//   default        code values
//   mc             MC1/MC2 SITL campaigns (support/scripts/run_mc_current_mpc.sh)
//   mc_rerun_c1    mc with the 1.6 deg landing tilt cap
//   direct_launch  launch/ros1/direct_mpc_sitl.launch
inline Params preset(const std::string& name){
    Params p;
    if(name.empty()||name=="default")return p;
    if(name=="mc"){
        p.horizon=40;p.thh=.16;p.w_stage_target_xy=.2;p.w_stage_target_z=.2;p.w_stage_target_vel=.2;
        p.w_land_speed=1000;p.w_land_tilt=100000;p.w_land_omega=100;p.w_land_gs=20000;
        p.w_stc_los=1000;p.w_stc_stage=100;p.cap_desc_radius=.12;p.omega_stc_max=.012;
        p.gs_cone_half_angle_deg=12;p.spd_stc_max=.5;p.los_cone_half_angle_deg=25;
        p.los_alt_trig=1.2;p.los_trigger_floor=1;p.alt_trig=1.0;p.z_stage=1.5;
        p.stage_trigger_floor=1;p.ctcs_step_eps=.00001;p.angular_rate_weight=1000;
        p.slew_thrust_weight=.1;p.slew_rate_weight=2;p.solver.max_iter=120;p.mpc_max_iter=120;
        p.warm_iterations=24;
        return p;}
    if(name=="mc_rerun_c1"){
        p.horizon=40;p.thh=.16;p.w_stage_target_xy=.2;p.w_stage_target_z=.2;p.w_stage_target_vel=.2;
        p.w_land_speed=1000;p.w_land_tilt=100000;p.w_land_omega=100;p.w_land_gs=20000;
        p.w_stc_los=1000;p.w_stc_stage=100;p.cap_desc_radius=.12;p.theta_stc_deg=1.6;
        p.omega_stc_max=.012;p.gs_cone_half_angle_deg=12;p.spd_stc_max=.5;
        p.los_cone_half_angle_deg=25;p.los_alt_trig=1.2;p.los_trigger_floor=1;p.alt_trig=1.0;
        p.z_stage=1.5;p.stage_trigger_floor=1;p.ctcs_step_eps=.00001;p.angular_rate_weight=1000;
        p.slew_thrust_weight=.1;p.slew_rate_weight=2;p.solver.max_iter=120;p.mpc_max_iter=120;
        p.warm_iterations=24;
        return p;}
    if(name=="direct_launch"){
        p.w_land_gs=20000;p.los_alt_trig=1.2;p.z_stage=1.3;p.ctcs_step_eps=0.0016;
        p.mpc_max_iter=120;p.ctcs_y_scale=10;
        return p;}
    throw std::invalid_argument("unknown stc_landing_mpc preset: "+name);
}
// STC_MPC_PRESET selects the configuration for this process.
inline const Params P=[]{const char* n=std::getenv("STC_MPC_PRESET");return preset(n?n:"");}();
inline const double J_SCALE = 1.0/P.ixx;
inline const double TAU_MAX = P.l_arm*(P.actuator_fmax/4.0-P.actuator_fmin/4.0)*J_SCALE;
inline const Eigen::Matrix3d J_B = (Eigen::Matrix3d()<<P.ixx*J_SCALE,0,0,0,P.iyy*J_SCALE,0,0,0,P.izz*J_SCALE).finished();
inline const Eigen::Vector3d GRAVITY{0,0,-9.81};
inline const double HOVER_THRUST = P.mass*9.81;
inline const double GS_CONE_TAN = std::tan(P.gs_cone_half_angle_deg*M_PI/180.0);
inline const double LOS_CONE_TAN = std::tan(P.los_cone_half_angle_deg*M_PI/180.0);

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

template<typename Scalar>
inline Scalar landingAltitudeTrigger(const Scalar& z, const Scalar& rxy2) {
    const Scalar trig_alt = posPart(Scalar(P.alt_trig) - z);
    if (!P.capture_landing_trigger) return trig_alt;  // benchmark default
    const Scalar cap2 = Scalar(P.cap_land_radius * P.cap_land_radius);
    const Scalar trig_cap = posPart(cap2 - rxy2) / cap2;
    return (trig_alt / Scalar(P.alt_trig)) * trig_cap;
}

inline double computeSigmaLand(const Eigen::VectorXd& x) {
    const double rxy2 = x(0)*x(0) + x(1)*x(1);
    return sigmaLandVal<double>(rxy2);
}

inline double computeTrigAlt(const Eigen::VectorXd& x) {
    const double rxy2 = x(0)*x(0) + x(1)*x(1);
    return landingAltitudeTrigger<double>(x(2), rxy2);
}

// ── Constant-accel target predictor ───────────────────────────────────────────
// Reset from the TargetSnapshot at every solve (see prepare_extra), then held
// as a constant-acceleration model across the horizon.
using ConstantAccelPredictor = target_models::ConstantAccelPredictor;

template<typename Scalar>
Scalar evalCtcsIntegrand(const Vector<Scalar>& x,
                         const Vector<Scalar>& u);

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
                                                         const Vector<Scalar>& u) {
    CtcsIntegrandTerms<Scalar> out;

    const Scalar rxy2     = x(0)*x(0) + x(1)*x(1);
    out.a_altitude_state  = landingAltitudeTrigger<Scalar>(x(2), rxy2);

    const Scalar v2     = x.template segment<3>(3).squaredNorm();
    const Scalar qperp2 = x(7)*x(7) + x(8)*x(8);
    const Scalar om2    = x.template segment<3>(10).squaredNorm();
    const Scalar rxy    = std::sqrt(rxy2 + Scalar(1e-12));
    const Scalar stage_radius_margin =
        (rxy - Scalar(P.stage_radius)) / Scalar(P.stage_trigger_scale);
    const Scalar stage_radius_trig =
        (stage_radius_margin > Scalar(0))
            ? Scalar(P.stage_trigger_floor) + stage_radius_margin
            : Scalar(0);
    const Scalar stage_settle_tilt_lim =
        Scalar(std::pow(std::sin(0.5 * P.stage_settle_tilt_deg * M_PI / 180.0), 2));
    const Scalar stage_settle_tilt = posPart(qperp2 - stage_settle_tilt_lim);
    const Scalar stage_settle_omega =
        posPart(om2 - Scalar(P.stage_settle_omega_max * P.stage_settle_omega_max));
    const std::string stage_mode = P.stage_trigger;
    if (stage_mode == "settle") {
        out.a_stage = stage_settle_tilt + stage_settle_omega;
    } else {
        out.a_stage = stage_radius_trig;
    }

    const Scalar loose_tilt_lim =
        Scalar(std::pow(std::sin(0.5 * P.theta_phase0_deg * M_PI / 180.0), 2));
    out.loose_tilt  = posPart(qperp2 - loose_tilt_lim);
    out.loose_omega = posPart(om2 - Scalar(P.omega_phase0_max * P.omega_phase0_max));

    const Scalar tilt_lim =
        Scalar(std::pow(std::sin(0.5 * P.alpha_theta_stc * P.theta_stc_deg * M_PI / 180.0), 2));
    out.speed      = posPart(v2     - Scalar(P.spd_stc_max   * P.spd_stc_max));
    out.tilt       = posPart(qperp2 - tilt_lim);
    out.omega      = posPart(om2    - Scalar(P.omega_stc_max * P.omega_stc_max));
    const Scalar gs_val = rxy - Scalar(GS_CONE_TAN) * (x(2) + Scalar(P.gs_apex_offset));
    out.glideslope = posPart(gs_val);

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
        (Scalar(P.los_alt_trig) - x(2)) / Scalar(P.los_trigger_scale);
    out.a_los =
        (los_alt_margin > Scalar(0))
            ? Scalar(P.los_trigger_floor) + los_alt_margin
            : Scalar(0);
    out.los = posPart(lateral - Scalar(LOS_CONE_TAN) * axial);

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

    const bool trig_stc_on = P.triggered_stc;

    if (trig_stc_on) {
        const Scalar stage_height_viol = posPart(Scalar(P.z_stage) - x(2));
        out.staging_value = Scalar(P.w_stc_stage) *
                            out.a_stage * out.a_stage *
                            stage_height_viol * stage_height_viol;

        out.los_value = Scalar(P.w_stc_los) *
                        out.a_los * out.a_los *
                        out.los * out.los;

        out.landing_value =
            out.a_altitude_state * out.a_altitude_state * landing_sum;
    }

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
                         const Vector<Scalar>& u) {
    return evalCtcsIntegrandTerms(x, u).value;
}

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

    static void bodyZHorizontal(const Vector<Scalar>& x, Scalar& bx, Scalar& by) {
        const Scalar qw = x(6), qx = x(7), qy = x(8), qz = x(9);
        bx = Scalar(2) * (qx * qz + qw * qy);
        by = Scalar(2) * (qy * qz - qw * qx);
    }

    static Matrix<Scalar> bodyZHorizontalJacobian(const Vector<Scalar>& x) {
        const Scalar qw = x(6), qx = x(7), qy = x(8), qz = x(9);
        Matrix<Scalar> J = Matrix<Scalar>::Zero(2, 4);
        J(0, 0) = Scalar(2) * qy;  J(0, 1) = Scalar(2) * qz;
        J(0, 2) = Scalar(2) * qw;  J(0, 3) = Scalar(2) * qx;
        J(1, 0) = Scalar(-2) * qx; J(1, 1) = Scalar(-2) * qw;
        J(1, 2) = Scalar(2) * qz;  J(1, 3) = Scalar(2) * qy;
        return J;
    }

    Scalar p(const Vector<Scalar>& x) const override {
        Scalar ep   = x.template segment<2>(0).squaredNorm();
        Scalar ez   = x(2)*x(2);
        Scalar evxy = x.template segment<2>(3).squaredNorm();
        Scalar evz  = (x(5) - Scalar(vz_ref_)) * (x(5) - Scalar(vz_ref_));
        Scalar bx, by;
        bodyZHorizontal(x, bx, by);
        Scalar eom  = x.template segment<3>(10).squaredNorm();
        return Scalar(0.5) * (
               Scalar(wp_xy_)*ep + Scalar(wp_z_)*ez
             + Scalar(wv_xy_)*evxy + Scalar(wv_z_)*evz
             + Scalar(watt_)*(bx*bx + by*by)
             + Scalar(wom_)*eom);
    }

    Vector<Scalar> px(const Vector<Scalar>& x) const override {
        Vector<Scalar> g = Vector<Scalar>::Zero(x.size());
        g.segment(0,2) = Scalar(wp_xy_)*x.segment(0,2);
        g(2)           = Scalar(wp_z_)*x(2);
        g.segment(3,2) = Scalar(wv_xy_)*x.segment(3,2);
        g(5)           = Scalar(wv_z_)*(x(5) - Scalar(vz_ref_));
        Scalar bx, by;
        bodyZHorizontal(x, bx, by);
        Matrix<Scalar> Jtilt = bodyZHorizontalJacobian(x);
        Vector<Scalar> etilt(2);
        etilt << bx, by;
        g.segment(6,4) += Scalar(watt_) * Jtilt.transpose() * etilt;
        g.segment(10,3) = Scalar(wom_)*x.segment(10,3);
        return g;
    }

    Matrix<Scalar> pxx(const Vector<Scalar>& x) const override {
        Matrix<Scalar> H = Matrix<Scalar>::Zero(x.size(), x.size());
        H(0,0)=H(1,1) = Scalar(wp_xy_);
        H(2,2) = Scalar(wp_z_);
        H(3,3)=H(4,4) = Scalar(wv_xy_);
        H(5,5) = Scalar(wv_z_);
        Matrix<Scalar> Jtilt = bodyZHorizontalJacobian(x);
        H.block(6,6,4,4) += Scalar(watt_) * Jtilt.transpose() * Jtilt;
        H(10,10)=H(11,11)=H(12,12) = Scalar(wom_);
        return H;
    }
};

// ── Stage constraints ─────────────────────────────────────────────────────────
template<typename Scalar, typename Dynamics>
std::shared_ptr<StcStageConstraint<Scalar>> makeCtcsConstraint(
    std::shared_ptr<Dynamics> dyn,
    double eps = P.ctcs_step_eps / P.ctcs_y_scale,
    int idx_theta = IDX_THETA, double fixed_dt = 0.0, int substeps = 1) {
    using Vec = Vector<Scalar>;
    using StateFn = std::function<Scalar(const Vec&)>;
    using GradFn = std::function<Vec(const Vec&)>;
    auto row = std::make_shared<StcStageConstraint<Scalar>>(StcForm::Density, eps);
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
    const Scalar looseTilt = tiltLimit(P.theta_phase0_deg);
    add("loose_tilt", always, zero,
        [=](const Vec& x) -> Scalar { return tilt(x) - looseTilt; }, dtilt, 1.0);
    add("loose_omega", always, zero,
        [=](const Vec& x) -> Scalar { return omega(x) - Scalar(P.omega_phase0_max*P.omega_phase0_max); }, domega, 1.0);

    if (P.triggered_stc) {
        const bool settle = P.stage_trigger == "settle";
        const Scalar settleTilt = tiltLimit(P.stage_settle_tilt_deg);
        StateFn stageTrigger = [=](const Vec& x) -> Scalar {
            if (settle) return -(posPart(tilt(x) - settleTilt) +
                posPart(omega(x) - Scalar(P.stage_settle_omega_max*P.stage_settle_omega_max)));
            const Scalar margin = (std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) -
                                   Scalar(P.stage_radius)) / Scalar(P.stage_trigger_scale);
            return margin > Scalar(0) ? -(Scalar(P.stage_trigger_floor) + margin) : Scalar(0);
        };
        GradFn dstage = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (settle) {
                if (tilt(x) > settleTilt) g -= dtilt(x);
                if (omega(x) > Scalar(P.stage_settle_omega_max*P.stage_settle_omega_max)) g -= domega(x);
            } else {
                const Scalar r = std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
                if (r > Scalar(P.stage_radius)) {
                    g(0) = -x(0)/(r*Scalar(P.stage_trigger_scale));
                    g(1) = -x(1)/(r*Scalar(P.stage_trigger_scale));
                }
            }
            return g;
        };
        add("staging", stageTrigger, dstage,
            [](const Vec& x) -> Scalar { return Scalar(P.z_stage) - x(2); },
            [](const Vec& x) -> Vec { Vec g = Vec::Zero(x.size()); g(2) = Scalar(-1); return g; }, P.w_stc_stage);

        StateFn losTrigger = [](const Vec& x) -> Scalar {
            const Scalar m = (Scalar(P.los_alt_trig) - x(2))/Scalar(P.los_trigger_scale);
            return m > Scalar(0) ? -(Scalar(P.los_trigger_floor) + m) : Scalar(0);
        };
        GradFn dlosTrigger = [](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (x(2) < Scalar(P.los_alt_trig)) g(2) = Scalar(1)/Scalar(P.los_trigger_scale);
            return g;
        };
        // Body boresight and its quaternion derivatives, without normalizing
        // intermediate RK4 states (the deployed interval uses them as-is).
        const auto boresight = [](const Vec& x) -> Eigen::Matrix<Scalar,3,1> {
            const Scalar w=x(6), a=x(7), b=x(8), c=x(9);
            return {Scalar(2)*(a*c+w*b), Scalar(2)*(b*c-w*a), w*w-a*a-b*b+c*c};
        };
        StateFn los = [=](const Vec& x) -> Scalar {
            const Scalar axial = x.template head<3>().dot(boresight(x));
            const Scalar lateral2 = x.template head<3>().squaredNorm() - axial*axial;
            return std::sqrt(posPart(lateral2)+Scalar(1e-12)) - Scalar(LOS_CONE_TAN)*axial;
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
            Vec g = -Scalar(LOS_CONE_TAN)*da;
            if (lateral2 > Scalar(0)) {
                Vec dl = -Scalar(2)*axial*da; dl.template head<3>() += Scalar(2)*x.template head<3>();
                g += dl/(Scalar(2)*std::sqrt(lateral2+Scalar(1e-12)));
            }
            return g;
        };
        add("los", losTrigger, dlosTrigger, los, dlos, P.w_stc_los);

        const bool capture = P.capture_landing_trigger;
        StateFn landTrigger = [=](const Vec& x) -> Scalar {
            if (!capture) return x(2) - Scalar(P.alt_trig);
            return -landingAltitudeTrigger<Scalar>(x(2), x(0)*x(0)+x(1)*x(1));
        };
        GradFn dland = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (!capture) { g(2) = Scalar(1); return g; }
            const Scalar cap2 = Scalar(P.cap_land_radius*P.cap_land_radius);
            const Scalar altitude = posPart(Scalar(P.alt_trig)-x(2));
            const Scalar lateral = posPart(cap2-x(0)*x(0)-x(1)*x(1));
            const Scalar denominator = Scalar(P.alt_trig)*cap2;
            if (altitude > Scalar(0)) g(2) = lateral/denominator;
            if (lateral > Scalar(0)) {
                g(0) = Scalar(2)*x(0)*altitude/denominator;
                g(1) = Scalar(2)*x(1)*altitude/denominator;
            }
            return g;
        };
        add("landing_speed", landTrigger, dland,
            [](const Vec& x) -> Scalar { return x.template segment<3>(3).squaredNorm()-Scalar(P.spd_stc_max*P.spd_stc_max); },
            [](const Vec& x) -> Vec { Vec g=Vec::Zero(x.size()); g.template segment<3>(3)=Scalar(2)*x.template segment<3>(3); return g; }, P.w_land_speed);
        const Scalar landTilt = tiltLimit(P.alpha_theta_stc*P.theta_stc_deg);
        add("landing_tilt", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return tilt(x)-landTilt; }, dtilt, P.w_land_tilt);
        add("landing_omega", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return omega(x)-Scalar(P.omega_stc_max*P.omega_stc_max); }, domega, P.w_land_omega);
        add("landing_glideslope", landTrigger, dland,
            [](const Vec& x) -> Scalar { return std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) - Scalar(GS_CONE_TAN)*(x(2)+Scalar(P.gs_apex_offset)); },
            [](const Vec& x) -> Vec {
                Vec g=Vec::Zero(x.size()); const Scalar r=std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
                g(0)=x(0)/r; g(1)=x(1)/r; g(2)=-Scalar(GS_CONE_TAN); return g;
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

inline Param getSolverParams() { return P.solver; }

// ── Warm-start / validity helpers ─────────────────────────────────────────────
}

namespace StcLandingMpcOCP {
namespace Ref = Formulation;
inline const int NX = comando_parameters::defaults.mpc.native_px4 ? 34 : 18; // physical, native plant, command memory
constexpr int NU = 4;
inline const int MEMORY = NX-4;
constexpr int ROTOR=14, INTEGRAL=18, GYRO=21, GYRO_RATE=24, DERIVATIVE=27;
constexpr double DT = 0.1;   // direct-MPC grid [s]
constexpr int N = 40;

inline double nativeGravity(){return comando_parameters::defaults.mpc.native_px4?comando_parameters::defaults.px4.gravity:9.81;}
inline double rateTau() { return std::max(0.02, comando_parameters::defaults.mpc.rate_tau); }
inline int substeps() { return comando_parameters::defaults.mpc.native_px4 ? std::max(2,static_cast<int>(std::ceil(DT*4*M_PI*comando_parameters::defaults.px4.gyro_cutoff_hz))) : std::max(2,static_cast<int>(std::ceil(DT/(0.65*rateTau())))); }
inline double rateLimit() { return comando_parameters::defaults.mpc.rate_limit; }
inline double thrustMax() { return std::min(Ref::P.fmax/Ref::P.mass,nativeGravity()*std::pow((1+comando_parameters::defaults.mpc.motor_speed_offset)/std::max(.1,comando_parameters::defaults.mpc.hover_thrust+comando_parameters::defaults.mpc.motor_speed_offset),comando_parameters::defaults.mpc.motor_thrust_exponent)); }
inline Eigen::VectorXd bounded(Eigen::VectorXd u) {
    if(u.size()!=NU || !u.allFinite()) throw std::runtime_error("invalid MPC control");
    u(0)=std::clamp(u(0), Ref::P.fmin/Ref::P.mass, thrustMax());
    for(int i=1;i<NU;++i) u(i)=std::clamp(u(i),-rateLimit(),rateLimit());
    return u;
}
inline Eigen::VectorXd hover() { Eigen::VectorXd u=Eigen::VectorXd::Zero(NU);u(0)=nativeGravity();return u; }

class Dynamics : public Ref::Quad6DOFVarTimeRelativePred<double> {
public:
    explicit Dynamics(std::shared_ptr<Ref::ConstantAccelPredictor> pred)
        : Ref::Quad6DOFVarTimeRelativePred<double>(std::move(pred)) {
        setMass(1.0);setGravity(Ref::GRAVITY);
        if(comando_parameters::defaults.mpc.native_px4){setJb(comando_parameters::defaults.px4.inertia.asDiagonal());setGravity(Eigen::Vector3d(0,0,-nativeGravity()));}else setJb(Eigen::Matrix3d::Identity());
        dim_x=NX;dim_u=NU;
    }
    struct Rates {Eigen::VectorXd v;Eigen::MatrixXd a,b;};
    static double rotorHover() {
        const auto& p=comando_parameters::defaults.px4;
        return std::sqrt(p.mass*p.gravity/(4*p.motor_constant));
    }
    static Eigen::VectorXd initialState(const Eigen::VectorXd& physical,const Eigen::VectorXd& applied) {
        if(physical.size()==NX)return physical;
        Eigen::VectorXd x=Eigen::VectorXd::Zero(NX);x.head(13)=physical.head(13);x.tail(NU)=applied;
        if(comando_parameters::defaults.mpc.native_px4) {
            const auto& p=comando_parameters::defaults.px4;
            x.segment<4>(ROTOR).setConstant((p.motor_min+(p.motor_max-p.motor_min)*collectiveSignal(applied(0)))/rotorHover());
            x.segment<3>(GYRO)=physical.segment<3>(10);
        }
        return x;
    }
    static Eigen::Matrix<double,4,3> nativeMixer() {
        return comando_parameters::defaults.px4.mixer_frd*Eigen::Vector3d(1,-1,-1).asDiagonal();
    }
    static Eigen::Matrix<double,3,4> nativeMoments() {
        const auto& p=comando_parameters::defaults.px4;Eigen::Matrix<double,3,4> result;
        result.row(0)=-p.rotor_position_frd.col(1).transpose();
        result.row(1)=-p.rotor_position_frd.col(0).transpose();
        result.row(2)<<-p.moment_ratio,-p.moment_ratio,p.moment_ratio,p.moment_ratio;
        return result;
    }
    static double collectiveSignal(double acceleration) {
        const auto& p=comando_parameters::defaults.mpc;
        return (p.hover_thrust+p.motor_speed_offset)*std::pow(std::max(1e-9,acceleration)/nativeGravity(),1/p.motor_thrust_exponent)-p.motor_speed_offset;
    }
    // Native controller states: normalized rotor speeds, I/P in rate units,
    // gyro low-pass output and derivative/frequency, and D-filter alpha/frequency.
    // The native PID is a predicted plant subsystem, never a second control law.
    Rates rates(const Eigen::VectorXd& x,const Eigen::VectorXd& u,bool jac) const {
        Rates out{Eigen::VectorXd::Zero(NX),Eigen::MatrixXd{},Eigen::MatrixXd{}};
        if(jac){out.a=Eigen::MatrixXd::Zero(NX,NX);out.b=Eigen::MatrixXd::Zero(NX,NU);}
        if(!comando_parameters::defaults.mpc.native_px4) {
            out.v.head(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRates(x.head(14),u);
            out.v.segment<3>(10)=(u.tail<3>()-x.segment<3>(10))/rateTau();
            if(jac) {
                out.a.topLeftCorner(14,14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesX(x.head(14),u);
                out.a.block(10,0,3,NX).setZero();out.a.block<3,3>(10,10)=-Eigen::Matrix3d::Identity()/rateTau();
                out.b.topRows(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesU(x.head(14),u).leftCols(NU);
                out.b.block(10,0,3,NU).setZero();out.b.block<3,3>(10,1)=Eigen::Matrix3d::Identity()/rateTau();
            }
            return out;
        }
        const auto& p=comando_parameters::defaults.px4;
        const double wh=rotorHover(),force_scale=p.motor_constant*wh*wh;
        const Eigen::Vector4d rotor=x.segment<4>(ROTOR);
        const Eigen::Vector4d forces=force_scale*rotor.array().square().matrix();
        Eigen::Vector4d physical;physical(0)=forces.sum()/p.mass;physical.tail<3>()=nativeMoments()*forces;
        out.v.head(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRates(x.head(14),physical);
        if(jac) {
            out.a.topLeftCorner(14,14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesX(x.head(14),physical);
            Eigen::Matrix4d df;df.row(0).setConstant(1/p.mass);df.bottomRows<3>()=nativeMoments();
            df*= (2*force_scale*rotor).asDiagonal();
            out.a.block(0,ROTOR,14,4)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesU(x.head(14),physical).leftCols(4)*df;
        }
        const double wg=2*M_PI*p.gyro_cutoff_hz,wd=2*M_PI*p.derivative_cutoff_hz;
        const Eigen::Vector3d error=u.tail<3>()-x.segment<3>(GYRO);
        const Eigen::Vector3d torque=p.rate_p.cwiseProduct(error+x.segment<3>(INTEGRAL))-
            p.rate_d.cwiseProduct(wd*x.segment<3>(DERIVATIVE));
        const Eigen::Matrix<double,4,3> mix=nativeMixer();
        const Eigen::Vector4d raw_signal=Eigen::Vector4d::Constant(collectiveSignal(u(0)))+mix*torque;
        for(int i=0;i<4;++i) {
            // Model actuator saturation here; do not change the DDP command.
            const double signal=std::clamp(raw_signal(i),0.,1.);
            const double goal=(p.motor_min+(p.motor_max-p.motor_min)*signal)/wh;
            const double tau=goal>rotor(i)?p.motor_tau_up:p.motor_tau_down;
            out.v(ROTOR+i)=(goal-rotor(i))/tau;
            if(jac) {
                out.a(ROTOR+i,ROTOR+i)=-1/tau;
                const double gain=raw_signal(i)>0&&raw_signal(i)<1 ? (p.motor_max-p.motor_min)/(wh*tau):0.;
                out.a.block<1,3>(ROTOR+i,GYRO)=-gain*mix.row(i)*p.rate_p.asDiagonal();
                out.a.block<1,3>(ROTOR+i,INTEGRAL)=gain*mix.row(i)*p.rate_p.asDiagonal();
                out.a.block<1,3>(ROTOR+i,DERIVATIVE)=-gain*wd*mix.row(i)*p.rate_d.asDiagonal();
                out.b.block<1,3>(ROTOR+i,1)=gain*mix.row(i)*p.rate_p.asDiagonal();
                const auto& mp=comando_parameters::defaults.mpc;
                const double base=std::max(1e-9,u(0))/nativeGravity();
                out.b(ROTOR+i,0)=gain*(mp.hover_thrust+mp.motor_speed_offset)/
                    (nativeGravity()*mp.motor_thrust_exponent)*std::pow(base,1/mp.motor_thrust_exponent-1);
            }
        }
        const double error_scale=400*M_PI/180;
        for(int i=0;i<3;++i) {
            const double e=error(i),factor=std::max(0.,1-e*e/(error_scale*error_scale));
            const double gain=p.rate_i(i)/p.rate_p(i);
            out.v(INTEGRAL+i)=gain*factor*e;
            if(jac) {
                const double derivative=factor>0?gain*(1-3*e*e/(error_scale*error_scale)):0.;
                out.a(INTEGRAL+i,GYRO+i)=-derivative;out.b(INTEGRAL+i,1+i)=derivative;
            }
        }
        out.v.segment<3>(GYRO)=wg*x.segment<3>(GYRO_RATE);
        out.v.segment<3>(GYRO_RATE)=wg*(x.segment<3>(10)-x.segment<3>(GYRO)-std::sqrt(2.)*x.segment<3>(GYRO_RATE));
        out.v.segment<3>(DERIVATIVE)=out.v.segment<3>(10)-wd*x.segment<3>(DERIVATIVE);
        if(jac) {
            out.a.block<3,3>(GYRO,GYRO_RATE)=wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,10)=wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,GYRO)=-wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,GYRO_RATE)=-std::sqrt(2.)*wg*Eigen::Matrix3d::Identity();
            out.a.middleRows(DERIVATIVE,3)=out.a.middleRows(10,3);
            out.a.block<3,3>(DERIVATIVE,DERIVATIVE)=-wd*Eigen::Matrix3d::Identity();
        }
        return out;
    }
    Eigen::VectorXd intervalRates(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,false).v;}
    Eigen::MatrixXd intervalRatesX(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,true).a;}
    Eigen::MatrixXd intervalRatesU(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,true).b;}
    struct Flow {Eigen::VectorXd x;Eigen::MatrixXd a,b;};
    Flow flow(Eigen::VectorXd x,const Eigen::VectorXd& u,double duration,bool jac=true) const {
        Eigen::MatrixXd a=Eigen::MatrixXd::Identity(NX,NX),b=Eigen::MatrixXd::Zero(NX,NU);
        const int steps=std::max(1,static_cast<int>(std::ceil(duration/(DT/substeps()))));
        const double h=duration/steps;
        for(int j=0;j<steps;++j) {
            auto r1=rates(x,u,jac);const auto& k1=r1.v;Eigen::VectorXd p2=x+h/2*k1;
            auto r2=rates(p2,u,jac);const auto& k2=r2.v;Eigen::VectorXd p3=x+h/2*k2;
            auto r3=rates(p3,u,jac);const auto& k3=r3.v;Eigen::VectorXd p4=x+h*k3;
            auto r4=rates(p4,u,jac);const auto& k4=r4.v;
            if(jac) {
                Eigen::MatrixXd a1=r1.a*a,b1=r1.a*b+r1.b;
                Eigen::MatrixXd a2=r2.a*(a+h/2*a1),b2=r2.a*(b+h/2*b1)+r2.b;
                Eigen::MatrixXd a3=r3.a*(a+h/2*a2),b3=r3.a*(b+h/2*b2)+r3.b;
                Eigen::MatrixXd a4=r4.a*(a+h*a3),b4=r4.a*(b+h*b3)+r4.b;
                a+=h/6*(a1+2*a2+2*a3+a4);b+=h/6*(b1+2*b2+2*b3+b4);
            }
            x+=h/6*(k1+2*k2+2*k3+k4);
        }
        const Eigen::Vector4d q=x.segment<4>(6);const double norm=q.norm();
        if(norm<1e-8) throw std::runtime_error("invalid MPC quaternion");
        Eigen::Matrix4d dq=(Eigen::Matrix4d::Identity()-q*q.transpose()/(norm*norm))/norm;
        a.middleRows(6,4)=dq*a.middleRows(6,4).eval();b.middleRows(6,4)=dq*b.middleRows(6,4).eval();
        x.segment<4>(6)=q/norm;x.segment(MEMORY,NU)=u;
        a.bottomRows(NU).setZero();b.bottomRows(NU).setIdentity();return {x,a,b};
    }
private:
    mutable Eigen::VectorXd cache_x_,cache_u_;
    mutable Flow cache_;
    const Flow& jacobian(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {
        if(cache_x_.size()!=x.size() || cache_u_.size()!=u.size() || !(cache_x_.array()==x.array()).all() || !(cache_u_.array()==u.array()).all()) {
            cache_=flow(x,u,DT);cache_x_=x;cache_u_=u;
        }
        return cache_;
    }
public:
    Eigen::VectorXd f(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return flow(x,u,DT,false).x;}
    Eigen::MatrixXd fx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return jacobian(x,u).a;}
    Eigen::MatrixXd fu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return jacobian(x,u).b;}
};

class Cost : public StageCostBase<double> {
    Ref::CTcSTCStageCost<double> ref_{1.0,Ref::P.w_stage_target_xy,
        Ref::P.w_stage_target_z,Ref::P.w_stage_target_vel,
        Eigen::Vector3d(0,0,Ref::P.z_stage)};
    Eigen::Vector4d r_,s_;
    Eigen::VectorXd oldU(const Eigen::VectorXd& u) const {Eigen::VectorXd v=Eigen::VectorXd::Zero(5);v(0)=u(0)*Ref::P.mass;v(4)=DT;return v;}
    // Penalize landing angular motion without slowing the high-altitude
    // capture. The smooth residual is beta(z)*omega; its Hessian uses GN.
    std::pair<double,double> rateGate(double altitude) const {
        const double span=std::max(.1,Ref::P.z_stage-Ref::P.alt_trig);
        const double slope=12/span, center=.5*(Ref::P.z_stage+Ref::P.alt_trig);
        const double beta=1/(1+std::exp(std::clamp(slope*(altitude-center),-60.,60.)));
        return {beta,-slope*beta*(1-beta)};
    }
public:
    Cost() {r_<<0.02,0.2,0.2,0.2;s_<<Ref::P.slew_thrust_weight,Ref::P.slew_rate_weight,Ref::P.slew_rate_weight,Ref::P.slew_rate_weight;}
    double q(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        const auto [beta, derivative]=rateGate(x(2));(void)derivative;
        return ref_.q(x.head(14),oldU(u))+(r_.array()*(u-hover()).array().square()).sum()+(s_.array()*(u-x.tail(NU)).array().square()).sum()
            +Ref::P.angular_rate_weight*beta*beta*x.segment<3>(10).squaredNorm();
    }
    Eigen::VectorXd qx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.qx(x.head(14),oldU(u));g.tail(NU)=2*s_.cwiseProduct(x.tail(NU)-u);
        const auto [beta, derivative]=rateGate(x(2));const double weight=Ref::P.angular_rate_weight;
        g.segment<3>(10)+=2*weight*beta*beta*x.segment<3>(10);
        g(2)+=2*weight*beta*derivative*x.segment<3>(10).squaredNorm();return g;
    }
    Eigen::VectorXd qu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=2*r_.cwiseProduct(u-hover())+2*s_.cwiseProduct(u-x.tail(NU));g(0)+=Ref::P.mass*ref_.qu(x.head(14),oldU(u))(0);return g;
    }
    Eigen::MatrixXd qxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.qxx(x.head(14),oldU(u));h.bottomRightCorner(NU,NU)=2*s_.asDiagonal();
        const auto [beta, derivative]=rateGate(x(2));const double weight=Ref::P.angular_rate_weight;
        h.block<3,3>(10,10).diagonal().array()+=2*weight*beta*beta;
        h.block<3,1>(10,2)+=2*weight*beta*derivative*x.segment<3>(10);
        h.block<1,3>(2,10)+=2*weight*beta*derivative*x.segment<3>(10).transpose();
        h(2,2)+=2*weight*derivative*derivative*x.segment<3>(10).squaredNorm();return h;
    }
    Eigen::MatrixXd quu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=2*(r_+s_).asDiagonal();h(0,0)+=Ref::P.mass*Ref::P.mass*ref_.quu(x.head(14),oldU(u))(0,0);return h;
    }
    Eigen::MatrixXd qxu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NU);h.bottomRows(NU)=-2*s_.asDiagonal();return h;
    }
};
class Terminal : public TerminalCostBase<double> {
    Ref::RelTermCost<double> ref_{Ref::P.term_w_pos_xy,Ref::P.term_w_pos_z,Ref::P.term_w_vel_xy,Ref::P.term_w_vel_z,Ref::P.term_w_tilt,Ref::P.term_w_om,Ref::P.term_vz_ref};
public:
    double p(const Eigen::VectorXd& x) const override {return ref_.p(x.head(14));}
    Eigen::VectorXd px(const Eigen::VectorXd& x) const override {Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.px(x.head(14));return g;}
    Eigen::MatrixXd pxx(const Eigen::VectorXd& x) const override {Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.pxx(x.head(14));return h;}
};
class Bounds : public StageConstraintBase<double> {
public:
    explicit Bounds(bool include_floor=true){constraint_type=ConstraintType::NO;dim_c=2*NU+(include_floor?1:0);}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd v(dim_c);v(0)=Ref::P.fmin/Ref::P.mass-u(0);v(1)=u(0)-thrustMax();
        for(int i=1;i<NU;++i){v(2*i)=u(i)-rateLimit();v(2*i+1)=-u(i)-rateLimit();}if(dim_c>2*NU)v(8)=-x(2);return v;
    }
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd a=Eigen::MatrixXd::Zero(dim_c,NX);if(dim_c>2*NU)a(8,2)=-1;return a;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd b=Eigen::MatrixXd::Zero(dim_c,NU);b(0,0)=-1;b(1,0)=1;for(int i=1;i<NU;++i){b(2*i,i)=1;b(2*i+1,i)=-1;}return b;}
};
class SpeedBound : public StageConstraintBase<double> {
public:
    SpeedBound(){constraint_type=ConstraintType::SOC;dim_c=4;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&) const override {Eigen::VectorXd v(4);v(0)=Ref::P.spd_phase0_max;v.tail(3)=x.segment(3,3);return -v;}
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd a=Eigen::MatrixXd::Zero(4,NX);a.block<3,3>(1,3)=-Eigen::Matrix3d::Identity();return a;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {return Eigen::MatrixXd::Zero(4,NU);}
};
inline std::shared_ptr<OptimalControlProblem<double>> create(const OCPCreateArgs& args) {
    auto pred=std::make_shared<Ref::ConstantAccelPredictor>();pred->a=args.target_accel;
    auto dyn=std::make_shared<Dynamics>(pred);auto problem=std::make_shared<OptimalControlProblem<double>>(N);
    auto cost=std::make_shared<Cost>();auto bounds=std::make_shared<Bounds>();auto initial_bounds=std::make_shared<Bounds>(false);auto speed=std::make_shared<SpeedBound>();
    auto stc=Ref::makeCtcsConstraint<double>(dyn,Ref::P.ctcs_step_eps/Ref::P.ctcs_y_scale,-1,DT,substeps());
    Eigen::VectorXd x=Dynamics::initialState(args.current_state,hover());x(13)=0.;
    for(int k=0;k<N;++k){
        if(k>0&&k<static_cast<int>(args.prev_X.size())&&args.prev_X[k].size()==NX) {
            x=args.prev_X[k];x(13)=k*DT;
            x.tail(NU)=problem->getInitialControl(k-1);
        }
        problem->setInitialState(k,x);Eigen::VectorXd u=hover();
        if(k<static_cast<int>(args.prev_U.size())&&args.prev_U[k].size()==NU)u=bounded(args.prev_U[k]);
        else if(comando_parameters::defaults.mpc.native_px4)u.tail<3>()=-x.segment<3>(INTEGRAL);
        problem->setInitialControl(k,u);problem->setStageDynamics(k,dyn);problem->setStageCost(k,cost);
        // Measured point constraints cannot be repaired by optimizing u0.
        // CTCS still constrains the entire first predicted interval.
        problem->addStageConstraint(k,k==0?initial_bounds:bounds);
        if(k>0)problem->addStageConstraint(k,speed);
        problem->addStageConstraint(k,stc);x=dyn->f(x,u);
    }
    if(static_cast<int>(args.prev_X.size())==N+1&&args.prev_X[N].size()==NX) {
        x=args.prev_X[N];x(13)=N*DT;x.tail(NU)=problem->getInitialControl(N-1);
    }
    problem->setInitialState(N,x);problem->setTerminalCost(std::make_shared<Terminal>());return problem;
}
inline OCPDescriptor descriptor(){OCPDescriptor d;d.disarm_on_landing_finish=true; d.state_dim=13; d.control_dim=4; d.skip_altitude_validation=true; d.skip_trajectory_validation=true; d.coordinates=OCPDescriptor::Coordinates::TargetRelative; d.log_state_headers=OCPLoggerDefaults::getStateHeaders13D(); d.state_names=OCPLoggerDefaults::getStateNames13D(); d.extract_actual_state_row=OCPLoggerDefaults::getActualStateRow13D; d.make_hover_state=OCPLoggerDefaults::makeHoverState13D;d.name="stc_landing_mpc";d.execution_mode=planner_core::ExecutionMode::DirectMpc;d.dt=DT;d.variable_dt=false;
    d.make_model=[](const Eigen::Vector3d& accel) {
        auto pred=std::make_shared<Ref::ConstantAccelPredictor>(); pred->a=accel;
        auto dynamics=std::make_shared<Dynamics>(pred);
        OCPDescriptor::Model model; model.nx=NX;model.nu=NU;model.horizon=N;model.dt=DT;
        model.hover=hover();model.bounded=bounded;model.initial_state=Dynamics::initialState;
        model.flow=[dynamics](const Eigen::VectorXd& x,const Eigen::VectorXd& u,double t,bool jac) {
            auto f=dynamics->flow(x,u,t,jac);return OCPDescriptor::Flow{f.x,f.a,f.b};
        };return model;
    };
    d.command_mode=OCPDescriptor::CommandMode::CmdBodyRate;d.default_n_replay=1;d.default_mass_kg=1.0;

    d.control_names={"thrust_accel","roll_rate","pitch_rate","yaw_rate"};d.use_predicted_handoff_state=false;
    d.post_process_result=[](SolverResult& r,const TargetSnapshot& t){r.is_relative_plan=t.valid;r.target_snapshot_pos=t.position;r.target_snapshot_vel=t.velocity;r.target_snapshot_acc=t.acceleration;};
    d.validate_target=[](const TargetSnapshot& t,double now,double max_age){return t.valid&&t.odom_stamp_sec>0&&now-t.odom_stamp_sec>=0&&now-t.odom_stamp_sec<=max_age;};
    d.sanitize_warm_control=[](Eigen::VectorXd& u){u=bounded(u);};d.create=create;
    d.getSolverParams=[](){
        auto p=Ref::getSolverParams();p.max_iter=Ref::P.mpc_max_iter;
        p.use_feasibility_ms=true;p.feasibility_ms_exact_rows={13,NX-4,NX-3,NX-2,NX-1};
        p.reg1_min=1e-4;p.reg2_min=1e-2;
        return p;
    };return d;
}
}
