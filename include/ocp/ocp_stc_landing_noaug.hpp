/// @file ocp_stc_landing_noaug.hpp
/// @brief No-aug interval CT-cSTC receding-horizon landing OCP ("stc_landing_noaug").
///
/// Ported from ALIPDDP-main/problem_examples/STC/quad_single_horizon_noaug_stc.cpp
/// (the "interval_noaug" benchmark arm), grafted onto the receding-horizon
/// machinery of ocp_stc_landing.hpp (which is the terminal-eq port of
/// quad_landing_rh_clean_stc.cpp). Relative-frame landing on a moving target:
///
///   state   = [13 physical | IDX_DT cumulative time]        (NO accumulator)
///   control = [fz, Mx, My, Mz | IDX_THETA free per-step dt]
///
/// Instead of carrying the CT-cSTC violation as an augmented state y with a
/// terminal EQ y_N = 0, the SAME RK4 interval integral of the STC density H
/// is integrated by StcStageConstraint directly from (x_k, u_k) inside a
/// per-stage inequality, using the host dynamics and analytic Jacobians:
///
///   integral(H dt) over [t_k, t_k + theta_k]  <=  eps      (hard-wired, always on)
///
/// This isolates state augmentation from the constraint functional: the zero
/// set is the same, but there is no y row/column in the Riccati recursion and
/// no y defect. SZMUK_CTCS_MODE is intentionally IGNORED here (the benchmark
/// file's in-code default was a vacuous terminal_eq stub — see its header).
///
/// Tuning (noaug_basin champion): SZMUK_CTCS_STEP_EPS = 1.6e-3 with
/// SZMUK_CTCS_Y_SCALE = 10 — a ~10x looser budget and much smaller Y_SCALE
/// than the augmented arm, because Y_SCALE existed to condition the y row of
/// the augmented dynamics and there is no such row here. These are the
/// defaults below; export the SZMUK_* vars before launching to retune.
///
/// Warm-start note: the planner core's makeXshifted() does a plain index
/// shift, so create() rebases IDX_DT against the new horizon start
/// (prev_X[0] after the shift == the previously executed node), exactly as
/// ocp_stc_landing.hpp does — minus the y-rebase, which no longer exists.
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

using comando_parameters::defaults;
using comando_parameters::envOrQ;
using comando_parameters::envOrInt;
constexpr int NX=13, NU=4, NX_SS=14, NU_SS=5, IDX_DT=13, IDX_THETA=4;
inline const auto& MASS = defaults.vehicle.mass;
inline const auto& IXX = defaults.vehicle.ixx;
inline const auto& IYY = defaults.vehicle.iyy;
inline const auto& IZZ = defaults.vehicle.izz;
inline const auto& ACTUATOR_FMIN = defaults.vehicle.actuator_fmin;
inline const auto& ACTUATOR_FMAX = defaults.vehicle.actuator_fmax;
inline const auto& FMIN = defaults.vehicle.fmin;
inline const auto& FMAX = defaults.vehicle.fmax;
inline const auto& L_ARM = defaults.vehicle.l_arm;
inline const auto& HORIZON = defaults.horizon.horizon;
inline const auto& NEX = defaults.horizon.nex;
inline const auto& TH0 = defaults.horizon.th0;
inline const auto& THL = defaults.horizon.thl;
inline const auto& THH = defaults.horizon.thh;
inline const auto& TERM_W_POS_XY = defaults.cost.term_w_pos_xy;
inline const auto& TERM_W_POS_Z = defaults.cost.term_w_pos_z;
inline const auto& TERM_W_VEL_XY = defaults.cost.term_w_vel_xy;
inline const auto& TERM_W_VEL_Z = defaults.cost.term_w_vel_z;
inline const auto& TERM_W_TILT = defaults.cost.term_w_tilt;
inline const auto& TERM_W_OM = defaults.cost.term_w_om;
inline const auto& TERM_VZ_REF = defaults.cost.term_vz_ref;
inline const auto& STAGE_W_TIME = defaults.cost.stage_w_time;
inline const auto& R_THETA = defaults.cost.r_theta;
inline const auto& THETA_REF = defaults.cost.theta_ref;
inline const auto& R_CTRL_THR = defaults.cost.r_ctrl_thr;
inline const auto& R_CTRL_MOM = defaults.cost.r_ctrl_mom;
inline const auto& W_TOUCH_VEL = defaults.cost.w_touch_vel;
inline const auto& TOUCH_ALT = defaults.cost.touch_alt;
inline const auto& TOUCH_SCALE = defaults.cost.touch_scale;
inline const auto& W_STAGE_TARGET_XY = defaults.cost.w_stage_target_xy;
inline const auto& W_STAGE_TARGET_Z = defaults.cost.w_stage_target_z;
inline const auto& W_STAGE_TARGET_VEL = defaults.cost.w_stage_target_vel;
inline const auto& W_LAND_SPEED = defaults.cost.w_land_speed;
inline const auto& W_LAND_TILT = defaults.cost.w_land_tilt;
inline const auto& W_LAND_OMEGA = defaults.cost.w_land_omega;
inline const auto& W_LAND_GS = defaults.cost.w_land_gs;
inline const auto& W_STC_LOS = defaults.cost.w_stc_los;
inline const auto& W_STC_STAGE = defaults.cost.w_stc_stage;
inline const auto& CAP_COST_RADIUS = defaults.cost.cap_cost_radius;
inline const auto& K_CAP = defaults.cost.k_cap;
inline const auto& CAP_DESC_RADIUS = defaults.cost.cap_desc_radius;
inline const auto& K_DESC = defaults.cost.k_desc;
inline const auto& GS_APEX_OFFSET = defaults.constraints.gs_apex_offset;
inline const auto& VZ_LAND_MAX = defaults.constraints.vz_land_max;
inline const auto& THETA_PHASE0_DEG = defaults.constraints.theta_phase0_deg;
inline const auto& OMEGA_PHASE0_MAX = defaults.constraints.omega_phase0_max;
inline const auto& SPD_PHASE0_MAX = defaults.constraints.spd_phase0_max;
inline const auto& THETA_STC_DEG = defaults.constraints.theta_stc_deg;
inline const auto& ALPHA_THETA_STC = defaults.constraints.alpha_theta_stc;
inline const auto& OMEGA_STC_MAX = defaults.constraints.omega_stc_max;
inline const auto& GS_CONE_HALF_ANGLE_DEG = defaults.constraints.gs_cone_half_angle_deg;
inline const auto& SPD_STC_MAX = defaults.constraints.spd_stc_max;
inline const auto& LOS_CONE_HALF_ANGLE_DEG = defaults.constraints.los_cone_half_angle_deg;
inline const auto& LOS_ALT_TRIG = defaults.constraints.los_alt_trig;
inline const auto& LOS_TRIGGER_SCALE = defaults.constraints.los_trigger_scale;
inline const auto& LOS_TRIGGER_FLOOR = defaults.constraints.los_trigger_floor;
inline const auto& ALT_TRIG = defaults.constraints.alt_trig;
inline const auto& CAP_LAND_RADIUS = defaults.constraints.cap_land_radius;
inline const auto& Z_STAGE = defaults.constraints.z_stage;
inline const auto& STAGE_RADIUS = defaults.constraints.stage_radius;
inline const auto& STAGE_TRIGGER_SCALE = defaults.constraints.stage_trigger_scale;
inline const auto& STAGE_TRIGGER_FLOOR = defaults.constraints.stage_trigger_floor;
inline const auto& STAGE_SETTLE_TILT_DEG = defaults.constraints.stage_settle_tilt_deg;
inline const auto& STAGE_SETTLE_OMEGA_MAX = defaults.constraints.stage_settle_omega_max;
inline const auto& CTCS_Y_SCALE = defaults.constraints.ctcs_y_scale;
inline const auto& CTCS_STEP_EPS = defaults.constraints.ctcs_step_eps;
inline const double J_SCALE = 1.0/IXX;
inline const double TAU_MAX = L_ARM*(ACTUATOR_FMAX/4.0-ACTUATOR_FMIN/4.0)*J_SCALE;
inline const Eigen::Matrix3d J_B = (Eigen::Matrix3d()<<IXX*J_SCALE,0,0,0,IYY*J_SCALE,0,0,0,IZZ*J_SCALE).finished();
inline const Eigen::Vector3d GRAVITY{0,0,-9.81};
inline const double HOVER_THRUST = MASS*9.81;
inline const double GS_CONE_TAN = std::tan(GS_CONE_HALF_ANGLE_DEG*M_PI/180.0);
inline const double LOS_CONE_TAN = std::tan(LOS_CONE_HALF_ANGLE_DEG*M_PI/180.0);
inline bool useTriggeredLandingStc(){return defaults.constraints.triggered_stc;}
inline bool useRhCaptureLandingTrigger(){return defaults.constraints.capture_landing_trigger;}
inline bool useRhFeedbackWarmStart(){return defaults.horizon.feedback_warm_start;}
inline std::string stageTriggerMode(){return defaults.constraints.stage_trigger;}

// ── Sigmoid helpers ───────────────────────────────────────────────────────────
template<typename Scalar>
inline Scalar sigmaLandVal(const Scalar& rxy2) {
    const Scalar g = Scalar(CAP_COST_RADIUS * CAP_COST_RADIUS) - rxy2;
    return Scalar(0.5) * (Scalar(1) + std::tanh(Scalar(K_CAP) * g));
}

template<typename Scalar>
inline void sigmaLandGrad(const Scalar& rxy2, const Scalar& x0, const Scalar& x1,
                          Scalar& sig, Scalar& gx0, Scalar& gx1) {
    const Scalar g       = Scalar(CAP_COST_RADIUS * CAP_COST_RADIUS) - rxy2;
    const Scalar th      = std::tanh(Scalar(K_CAP) * g);
    sig     = Scalar(0.5) * (Scalar(1) + th);
    const Scalar dsig_dg = Scalar(0.5) * Scalar(K_CAP) * (Scalar(1) - th * th);
    gx0 = dsig_dg * Scalar(-2) * x0;
    gx1 = dsig_dg * Scalar(-2) * x1;
}

template<typename Scalar>
inline Scalar sigmaDescVal(const Scalar& rxy2, const Scalar& /*z*/) {
    const Scalar g = Scalar(CAP_DESC_RADIUS * CAP_DESC_RADIUS) - rxy2;
    return Scalar(0.5) * (Scalar(1) + std::tanh(Scalar(K_DESC) * g));
}

template<typename Scalar>
inline void sigmaDescGrad(const Scalar& rxy2, const Scalar& x0, const Scalar& x1,
                          const Scalar& /*z*/,
                          Scalar& sig, Scalar& gx0, Scalar& gx1, Scalar& gz) {
    const Scalar g    = Scalar(CAP_DESC_RADIUS * CAP_DESC_RADIUS) - rxy2;
    const Scalar th   = std::tanh(Scalar(K_DESC) * g);
    sig = Scalar(0.5) * (Scalar(1) + th);
    const Scalar dsig = Scalar(0.5) * Scalar(K_DESC) * (Scalar(1) - th * th);
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
    const Scalar trig_alt = posPart(Scalar(ALT_TRIG) - z);
    if (!useRhCaptureLandingTrigger()) return trig_alt;  // benchmark default
    const Scalar cap2 = Scalar(CAP_LAND_RADIUS * CAP_LAND_RADIUS);
    const Scalar trig_cap = posPart(cap2 - rxy2) / cap2;
    return (trig_alt / Scalar(ALT_TRIG)) * trig_cap;
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

struct StcLandingNoAugExtra {
    std::shared_ptr<ConstantAccelPredictor> predictor;
};

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
        return evalCtcsIntegrand<double>(x_aug, u_full) / CTCS_Y_SCALE;
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
        (rxy - Scalar(STAGE_RADIUS)) / Scalar(STAGE_TRIGGER_SCALE);
    const Scalar stage_radius_trig =
        (stage_radius_margin > Scalar(0))
            ? Scalar(STAGE_TRIGGER_FLOOR) + stage_radius_margin
            : Scalar(0);
    const Scalar stage_settle_tilt_lim =
        Scalar(std::pow(std::sin(0.5 * STAGE_SETTLE_TILT_DEG * M_PI / 180.0), 2));
    const Scalar stage_settle_tilt = posPart(qperp2 - stage_settle_tilt_lim);
    const Scalar stage_settle_omega =
        posPart(om2 - Scalar(STAGE_SETTLE_OMEGA_MAX * STAGE_SETTLE_OMEGA_MAX));
    const std::string stage_mode = stageTriggerMode();
    if (stage_mode == "settle") {
        out.a_stage = stage_settle_tilt + stage_settle_omega;
    } else {
        out.a_stage = stage_radius_trig;
    }

    const Scalar loose_tilt_lim =
        Scalar(std::pow(std::sin(0.5 * THETA_PHASE0_DEG * M_PI / 180.0), 2));
    out.loose_tilt  = posPart(qperp2 - loose_tilt_lim);
    out.loose_omega = posPart(om2 - Scalar(OMEGA_PHASE0_MAX * OMEGA_PHASE0_MAX));

    const Scalar tilt_lim =
        Scalar(std::pow(std::sin(0.5 * ALPHA_THETA_STC * THETA_STC_DEG * M_PI / 180.0), 2));
    out.speed      = posPart(v2     - Scalar(SPD_STC_MAX   * SPD_STC_MAX));
    out.tilt       = posPart(qperp2 - tilt_lim);
    out.omega      = posPart(om2    - Scalar(OMEGA_STC_MAX * OMEGA_STC_MAX));
    const Scalar gs_val = rxy - Scalar(GS_CONE_TAN) * (x(2) + Scalar(GS_APEX_OFFSET));
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
        (Scalar(LOS_ALT_TRIG) - x(2)) / Scalar(LOS_TRIGGER_SCALE);
    out.a_los =
        (los_alt_margin > Scalar(0))
            ? Scalar(LOS_TRIGGER_FLOOR) + los_alt_margin
            : Scalar(0);
    out.los = posPart(lateral - Scalar(LOS_CONE_TAN) * axial);

    // NOTE: no thrust STC blocks — faithful to quad_single_horizon_noaug_stc,
    // whose integrand carries only stage/LOS/landing. Thrust limits stay
    // enforced as hard Fmin/Fmax stage constraints.
    const Scalar path_sum =
        out.loose_tilt  * out.loose_tilt  +
        out.loose_omega * out.loose_omega;

    const Scalar landing_sum =
        Scalar(W_LAND_SPEED) * out.speed      * out.speed      +
        Scalar(W_LAND_TILT)  * out.tilt       * out.tilt       +
        Scalar(W_LAND_OMEGA) * out.omega      * out.omega      +
        Scalar(W_LAND_GS)    * out.glideslope * out.glideslope;

    out.path_value = path_sum;

    const bool trig_stc_on = useTriggeredLandingStc();

    if (trig_stc_on) {
        const Scalar stage_height_viol = posPart(Scalar(Z_STAGE) - x(2));
        out.staging_value = Scalar(W_STC_STAGE) *
                            out.a_stage * out.a_stage *
                            stage_height_viol * stage_height_viol;

        out.los_value = Scalar(W_STC_LOS) *
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
        const Scalar rref_z = (Scalar(1) - sc) * rf_z_ + sc*(Scalar(1)-sd)*Scalar(Z_STAGE);

        const Scalar drx = x(0) - rref_x;
        const Scalar dry = x(1) - rref_y;
        const Scalar drz = x(2) - rref_z;
        const Scalar v2  = x.template segment<3>(3).squaredNorm();
        const Scalar touch_margin = (Scalar(TOUCH_ALT) - x(2)) / Scalar(TOUCH_SCALE);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);

        const Scalar pos_cost = w_pos_xy_*(drx*drx + dry*dry) + w_pos_z_*drz*drz;
        const Scalar vel_cost = sd * w_vel_ * v2;
        const Scalar touch_vel_cost = Scalar(W_TOUCH_VEL) * touch * touch * v2;
        const Scalar reg      = eps_*(v2 + x.template segment<3>(10).squaredNorm());

        const Scalar dT  = u(0) - Scalar(HOVER_THRUST);
        const Scalar ctrl_cost = Scalar(R_CTRL_THR)*dT*dT
                               + Scalar(R_CTRL_MOM)*(u(1)*u(1) + u(2)*u(2) + u(3)*u(3));

        const Scalar dTheta = u(IDX_THETA) - Scalar(THETA_REF);
        const Scalar time_reg = Scalar(R_THETA) * dTheta * dTheta;

        return Scalar(STAGE_W_TIME)*u(IDX_THETA) + time_reg + reg
             + pos_cost + vel_cost + touch_vel_cost + ctrl_cost;
    }

    Vector<Scalar> qx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)u;
        const Scalar rxy2 = x(0)*x(0) + x(1)*x(1);

        Scalar sc, sc_gx0, sc_gx1;
        sigmaLandGrad(rxy2, x(0), x(1), sc, sc_gx0, sc_gx1);

        Scalar sd, sd_gx0, sd_gx1, sd_gz;
        sigmaDescGrad(rxy2, x(0), x(1), x(2), sd, sd_gx0, sd_gx1, sd_gz);

        const Scalar Zs    = Scalar(Z_STAGE);
        const Scalar rref_x = (Scalar(1) - sc) * rf_x_;
        const Scalar rref_y = (Scalar(1) - sc) * rf_y_;
        const Scalar rref_z = (Scalar(1) - sc) * rf_z_ + sc*(Scalar(1)-sd)*Zs;

        const Scalar drx = x(0) - rref_x;
        const Scalar dry = x(1) - rref_y;
        const Scalar drz = x(2) - rref_z;
        const Scalar v2  = x.template segment<3>(3).squaredNorm();
        const Scalar touch_margin = (Scalar(TOUCH_ALT) - x(2)) / Scalar(TOUCH_SCALE);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);
        const Scalar dtouch_dz = touch_margin > Scalar(0) ? -Scalar(1) / Scalar(TOUCH_SCALE) : Scalar(0);

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
            (Scalar(2)*eps_ + Scalar(2)*sd*w_vel_ + Scalar(2)*Scalar(W_TOUCH_VEL)*touch*touch)
            * x.template segment<3>(3);
        g(2) += Scalar(2)*Scalar(W_TOUCH_VEL)*touch*dtouch_dz*v2;
        g.template segment<3>(10) = Scalar(2)*eps_ * x.template segment<3>(10);

        return g;
    }

    Vector<Scalar> qu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)x;
        Vector<Scalar> g = Vector<Scalar>::Zero(NU_SS);
        g(IDX_THETA) = Scalar(STAGE_W_TIME) + Scalar(2 * R_THETA) * (u(IDX_THETA) - Scalar(THETA_REF));
        g(0) = Scalar(2 * R_CTRL_THR) * (u(0) - Scalar(HOVER_THRUST));
        g(1) = Scalar(2 * R_CTRL_MOM) * u(1);
        g(2) = Scalar(2 * R_CTRL_MOM) * u(2);
        g(3) = Scalar(2 * R_CTRL_MOM) * u(3);
        return g;
    }

    Matrix<Scalar> qxx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)u;
        const Scalar rxy2 = x(0)*x(0) + x(1)*x(1);

        Scalar sc, sc_gx0, sc_gx1;
        sigmaLandGrad(rxy2, x(0), x(1), sc, sc_gx0, sc_gx1);
        Scalar sd, sd_gx0, sd_gx1, sd_gz;
        sigmaDescGrad(rxy2, x(0), x(1), x(2), sd, sd_gx0, sd_gx1, sd_gz);
        const Scalar Zs = Scalar(Z_STAGE);
        const Scalar touch_margin = (Scalar(TOUCH_ALT) - x(2)) / Scalar(TOUCH_SCALE);
        const Scalar touch = touch_margin > Scalar(0) ? touch_margin : Scalar(0);
        const Scalar dtouch_dz = touch_margin > Scalar(0) ? -Scalar(1) / Scalar(TOUCH_SCALE) : Scalar(0);
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
                   + Scalar(2)*Scalar(W_TOUCH_VEL)*touch*touch;
            // Gauss-Newton curvature of sqrt(W_TOUCH_VEL)*touch(z)*v.
            // Its mixed block is 2*w*touch*dtouch*v, matching both diagonals.
            H(2,i) += Scalar(2)*Scalar(W_TOUCH_VEL)*touch*dtouch_dz*x(i);
            H(i,2) += Scalar(2)*Scalar(W_TOUCH_VEL)*touch*dtouch_dz*x(i);
        }
        H(2,2) += Scalar(2)*Scalar(W_TOUCH_VEL)*dtouch_dz*dtouch_dz*v2;
        for (int i = 10; i < 13; ++i) H(i,i) = Scalar(2)*eps_;
        return H;
    }

    Matrix<Scalar> quu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        (void)x; (void)u;
        Matrix<Scalar> H = Matrix<Scalar>::Zero(NU_SS, NU_SS);
        H(0, 0) = Scalar(2 * R_CTRL_THR);
        H(1, 1) = Scalar(2 * R_CTRL_MOM);
        H(2, 2) = Scalar(2 * R_CTRL_MOM);
        H(3, 3) = Scalar(2 * R_CTRL_MOM);
        H(IDX_THETA, IDX_THETA) = Scalar(2 * R_THETA);
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
template<typename Scalar>
class FminCon : public StageConstraintBase<Scalar> { public:
    FminCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << Scalar(FMIN)-u(0)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(1, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,u.size()); J(0,0)=Scalar(-1); return J; }};

template<typename Scalar>
class FmaxCon : public StageConstraintBase<Scalar> { public:
    FmaxCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << u(0)-Scalar(FMAX)).finished(); }
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
        Vector<Scalar> cn(4); cn << Scalar(SPD_PHASE0_MAX), x(3), x(4), x(5); return -cn; }
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
    double eps = CTCS_STEP_EPS / CTCS_Y_SCALE,
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
            .weight(Scalar(weight / CTCS_Y_SCALE));
    };
    const Scalar looseTilt = tiltLimit(THETA_PHASE0_DEG);
    add("loose_tilt", always, zero,
        [=](const Vec& x) -> Scalar { return tilt(x) - looseTilt; }, dtilt, 1.0);
    add("loose_omega", always, zero,
        [=](const Vec& x) -> Scalar { return omega(x) - Scalar(OMEGA_PHASE0_MAX*OMEGA_PHASE0_MAX); }, domega, 1.0);

    if (useTriggeredLandingStc()) {
        const bool settle = stageTriggerMode() == "settle";
        const Scalar settleTilt = tiltLimit(STAGE_SETTLE_TILT_DEG);
        StateFn stageTrigger = [=](const Vec& x) -> Scalar {
            if (settle) return -(posPart(tilt(x) - settleTilt) +
                posPart(omega(x) - Scalar(STAGE_SETTLE_OMEGA_MAX*STAGE_SETTLE_OMEGA_MAX)));
            const Scalar margin = (std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) -
                                   Scalar(STAGE_RADIUS)) / Scalar(STAGE_TRIGGER_SCALE);
            return margin > Scalar(0) ? -(Scalar(STAGE_TRIGGER_FLOOR) + margin) : Scalar(0);
        };
        GradFn dstage = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (settle) {
                if (tilt(x) > settleTilt) g -= dtilt(x);
                if (omega(x) > Scalar(STAGE_SETTLE_OMEGA_MAX*STAGE_SETTLE_OMEGA_MAX)) g -= domega(x);
            } else {
                const Scalar r = std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
                if (r > Scalar(STAGE_RADIUS)) {
                    g(0) = -x(0)/(r*Scalar(STAGE_TRIGGER_SCALE));
                    g(1) = -x(1)/(r*Scalar(STAGE_TRIGGER_SCALE));
                }
            }
            return g;
        };
        add("staging", stageTrigger, dstage,
            [](const Vec& x) -> Scalar { return Scalar(Z_STAGE) - x(2); },
            [](const Vec& x) -> Vec { Vec g = Vec::Zero(x.size()); g(2) = Scalar(-1); return g; }, W_STC_STAGE);

        StateFn losTrigger = [](const Vec& x) -> Scalar {
            const Scalar m = (Scalar(LOS_ALT_TRIG) - x(2))/Scalar(LOS_TRIGGER_SCALE);
            return m > Scalar(0) ? -(Scalar(LOS_TRIGGER_FLOOR) + m) : Scalar(0);
        };
        GradFn dlosTrigger = [](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (x(2) < Scalar(LOS_ALT_TRIG)) g(2) = Scalar(1)/Scalar(LOS_TRIGGER_SCALE);
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
        add("los", losTrigger, dlosTrigger, los, dlos, W_STC_LOS);

        const bool capture = useRhCaptureLandingTrigger();
        StateFn landTrigger = [=](const Vec& x) -> Scalar {
            if (!capture) return x(2) - Scalar(ALT_TRIG);
            return -landingAltitudeTrigger<Scalar>(x(2), x(0)*x(0)+x(1)*x(1));
        };
        GradFn dland = [=](const Vec& x) -> Vec {
            Vec g = Vec::Zero(x.size());
            if (!capture) { g(2) = Scalar(1); return g; }
            const Scalar cap2 = Scalar(CAP_LAND_RADIUS*CAP_LAND_RADIUS);
            const Scalar altitude = posPart(Scalar(ALT_TRIG)-x(2));
            const Scalar lateral = posPart(cap2-x(0)*x(0)-x(1)*x(1));
            const Scalar denominator = Scalar(ALT_TRIG)*cap2;
            if (altitude > Scalar(0)) g(2) = lateral/denominator;
            if (lateral > Scalar(0)) {
                g(0) = Scalar(2)*x(0)*altitude/denominator;
                g(1) = Scalar(2)*x(1)*altitude/denominator;
            }
            return g;
        };
        add("landing_speed", landTrigger, dland,
            [](const Vec& x) -> Scalar { return x.template segment<3>(3).squaredNorm()-Scalar(SPD_STC_MAX*SPD_STC_MAX); },
            [](const Vec& x) -> Vec { Vec g=Vec::Zero(x.size()); g.template segment<3>(3)=Scalar(2)*x.template segment<3>(3); return g; }, W_LAND_SPEED);
        const Scalar landTilt = tiltLimit(ALPHA_THETA_STC*THETA_STC_DEG);
        add("landing_tilt", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return tilt(x)-landTilt; }, dtilt, W_LAND_TILT);
        add("landing_omega", landTrigger, dland,
            [=](const Vec& x) -> Scalar { return omega(x)-Scalar(OMEGA_STC_MAX*OMEGA_STC_MAX); }, domega, W_LAND_OMEGA);
        add("landing_glideslope", landTrigger, dland,
            [](const Vec& x) -> Scalar { return std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12)) - Scalar(GS_CONE_TAN)*(x(2)+Scalar(GS_APEX_OFFSET)); },
            [](const Vec& x) -> Vec {
                Vec g=Vec::Zero(x.size()); const Scalar r=std::sqrt(x(0)*x(0)+x(1)*x(1)+Scalar(1e-12));
                g(0)=x(0)/r; g(1)=x(1)/r; g(2)=-Scalar(GS_CONE_TAN); return g;
            }, W_LAND_GS);
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
inline Param getSolverParams() { return defaults.solver; }

// ── Warm-start / validity helpers ─────────────────────────────────────────────
inline void clampWarmControl(Eigen::VectorXd& u) {
    if (u.size() < NU_SS) return;
    u(0) = std::clamp((double)u(0), (double)FMIN, (double)FMAX);
    u(1) = std::clamp((double)u(1), -TAU_MAX, TAU_MAX);
    u(2) = std::clamp((double)u(2), -TAU_MAX, TAU_MAX);
    u(3) = std::clamp((double)u(3), -TAU_MAX, TAU_MAX);
    u(IDX_THETA) = std::clamp((double)u(IDX_THETA), THL, THH);
}

inline bool finiteTrajectory(const std::vector<Eigen::VectorXd>& X,
                             const std::vector<Eigen::VectorXd>& U) {
    for (const auto& x : X) if (!x.allFinite()) return false;
    for (const auto& u : U) if (!u.allFinite()) return false;
    return true;
}

inline bool plausibleTrajectory(const std::vector<Eigen::VectorXd>& X,
                                int max_node, int n_solve) {
    const double dt_limit = static_cast<double>(n_solve) * THH + 1.0;
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

// ── Factory ───────────────────────────────────────────────────────────────────
inline std::shared_ptr<OptimalControlProblem<double>> create(
    const Eigen::VectorXd& x0_rel,
    const std::vector<Eigen::VectorXd>& prev_U = {},
    const std::vector<Eigen::VectorXd>& prev_X = {},
    const std::vector<Eigen::MatrixXd>& prev_K = {},
    std::shared_ptr<ConstantAccelPredictor> predictor = nullptr)
{
    const int N = HORIZON;
    auto problem = std::make_shared<OptimalControlProblem<double>>(N);

    if (!predictor) {
        predictor = std::make_shared<ConstantAccelPredictor>();
    }

    auto dyn = std::make_shared<Quad6DOFVarTimeRelativePred<double>>(predictor);
    dyn->setMass(MASS);
    dyn->setGravity(GRAVITY);
    dyn->setJb(J_B);
    dyn->setTargetAccel(predictor->predictAccel(0.0));

    auto tcost  = std::make_shared<RelTermCost<double>>(
        TERM_W_POS_XY, TERM_W_POS_Z,
        TERM_W_VEL_XY, TERM_W_VEL_Z,
        TERM_W_TILT, TERM_W_OM, TERM_VZ_REF);
    auto cfmin  = std::make_shared<FminCon<double>>();
    auto cfmax  = std::make_shared<FmaxCon<double>>();
    auto cmom   = std::make_shared<MomentCon<double>>();
    auto cth    = std::make_shared<ThetaBounds<double>>(THL, THH);
    auto czfl   = std::make_shared<ZFloorCon<double>>();
    auto cspeed = std::make_shared<GeneralSpeedCon<double>>();
    // The constraint sees the SCALED integral, so the physical budget
    // CTCS_STEP_EPS is divided by CTCS_Y_SCALE (same zero set as unscaled).
    // Hard-wired: this OCP has no other enforcement mode.
    auto cstep = makeCtcsConstraint<double>(dyn);

    const double eps_cost = envOrQ("SZMUK_W_EPS", 1.0);
    const Eigen::Vector3d r_follow(0.0, 0.0, Z_STAGE);
    auto stage_cost = std::make_shared<CTcSTCStageCost<double>>(
        eps_cost,
        envOrQ("SZMUK_W_POS_XY", W_STAGE_TARGET_XY),
        envOrQ("SZMUK_W_POS_Z", W_STAGE_TARGET_Z),
        envOrQ("SZMUK_W_VEL", W_STAGE_TARGET_VEL),
        r_follow);

    for (int k = 0; k < N; ++k) {
        problem->setStageDynamics(k, dyn);
        problem->setStageCost(k, stage_cost);
        problem->addStageConstraint(k, cfmin);
        problem->addStageConstraint(k, cfmax);
        problem->addStageConstraint(k, cmom);
        problem->addStageConstraint(k, cth);
        problem->addStageConstraint(k, czfl);
        problem->addStageConstraint(k, cspeed);
        problem->addStageConstraint(k, cstep);
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
        useRhFeedbackWarmStart() &&
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
                const double max_du = envOrQ("SZMUK_RH_MAX_FB_DU", 0.05);
                const bool wild_feedback =
                    !feedback_dim_ok ||
                    !u_fb.allFinite() ||
                    !du.allFinite() ||
                    (max_du > 0.0 && du.norm() > max_du) ||
                    u_fb(0) < 0.5 * FMIN ||
                    u_fb(0) > 1.5 * FMAX ||
                    std::abs(u_fb(1)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(2)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(3)) > 2.0 * TAU_MAX ||
                    u_fb(IDX_THETA) < 0.0 ||
                    u_fb(IDX_THETA) > 2.0 * THH;
                if (!wild_feedback) {
                    uk = u_fb;
                }
            }
        } else {
            // Geometric straight-line seed. Unlike the augmented arm (soft AL
            // terminal EQ), the interval inequality here is a HARD per-node
            // constraint, so a seed that dives below the staging floor while
            // still outside the staging radius starts the IPM deeply
            // infeasible and stalls the cold solve. Aim at (0,0,Z_STAGE)
            // until laterally captured, and only then at the origin.
            const Eigen::Vector3d p0 = sim14.head(3);
            const Eigen::Vector3d v0 = sim14.segment(3, 3);
            double T_g = static_cast<double>(N - k) * TH0;
            if (T_g < TH0) T_g = TH0;
            const double rxy_seed = p0.head(2).norm();
            Eigen::Vector3d p_goal(0.0, 0.0,
                                   rxy_seed > STAGE_RADIUS ? Z_STAGE : 0.0);
            Eigen::Vector3d a_req = 2.0 * ((p_goal - p0) - v0 * T_g) / (T_g * T_g);
            a_req.z() = std::max(a_req.z(), (-VZ_LAND_MAX - v0.z()) / T_g);
            const double tabs_ws = sim14(IDX_DT);
            const Eigen::Vector3d a_tgt_ws = predictor->predictAccel(tabs_ws);
            const Eigen::Vector3d fw = MASS * (a_req + a_tgt_ws - GRAVITY);
            uk(0) = fw.norm();
            uk(IDX_THETA) = TH0;
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
    d.dt = TH0;
    d.default_n_replay = NEX;
    d.default_mass_kg = MASS;
    d.warm_start = OCPDescriptor::WarmStart::Feedback;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.world_acceleration=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u) {
        if(x.size()<13||u.size()<1||!x.allFinite()||!u.allFinite())throw std::invalid_argument("invalid DDP acceleration input");
        Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
        if(q.norm()<1e-8)throw std::invalid_argument("invalid DDP attitude");
        return Eigen::Vector3d(q.normalized()*Eigen::Vector3d(0,0,u(0)/MASS)+Eigen::Vector3d(0,0,-9.81));
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
        if(defaults.px4.decouple_acceleration) {
            if(axis.z()<=1e-6)throw std::invalid_argument("PX4 acceleration mapping requires upright thrust axis");
            feedforward.head<2>()=9.81*axis.head<2>()/axis.z();
        }
        feedforward.x()+=defaults.px4.velocity_d_xy*acceleration.x();
        feedforward.y()+=defaults.px4.velocity_d_xy*acceleration.y();
        feedforward.z()+=defaults.px4.velocity_d_z*acceleration.z();
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
                                  std::min(NEX, HORIZON), HORIZON))) {
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
        dyn->setMass(MASS);dyn->setGravity(GRAVITY);dyn->setJb(J_B);
        OCPDescriptor::Model model;model.nx=NX_SS;model.nu=NU_SS;model.horizon=HORIZON;model.dt=TH0;
        model.hover=Eigen::VectorXd::Zero(NU_SS);model.hover(0)=HOVER_THRUST;model.hover(IDX_THETA)=TH0;
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
        return create(a.current_state, a.prev_U, a.prev_X, a.prev_K, pred);
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
                if(std::dynamic_pointer_cast<StcStageConstraint<double>>(row))++excluded;
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
    const auto original=d.create;
    d.create=[original](const OCPCreateArgs& args){return ordinaryNodeProblem(*original(args));};
    return d;
}
}  // namespace StcLandingNoAugOCP
