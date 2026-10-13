#pragma once
#include <Eigen/Dense>
#include "param.h"
#include <algorithm>
#include <cstdlib>
#include <string>
#include <stdexcept>

// Shared data/defaults only. Dynamics, costs, constraints, and their derivatives
// belong to each complete OCP file. Construct Parameters to override defaults
// in C++; fromEnvironment preserves the saved startup tuning names.
namespace comando_parameters {
inline double envOrQ(const char* name,double fallback) {
    const char* value=std::getenv(name);return value ? std::atof(value) : fallback;
}
inline int envOrInt(const char* name,int fallback) {
    const char* value=std::getenv(name);return value ? std::atoi(value) : fallback;
}

struct VehicleParameters {
    double mass = 0.027;
    double ixx = 1.66e-5;
    double iyy = 1.66e-5;
    double izz = 2.92e-5;
    double actuator_fmin = 0.08;
    double actuator_fmax = 0.60;
    double fmin = 0.12;
    double fmax = 0.52;
    double l_arm = 0.046;
};

struct HorizonParameters {
    static constexpr double command_dt = .01;
    int horizon = 30;
    int nex = 7;
    double th0 = 0.07;
    double thl = 0.05;
    double thh = 0.12;
    bool feedback_warm_start = true;
};

struct CostParameters {
    double term_w_pos_xy = 2500.0;
    double term_w_pos_z = 2500.0;
    double term_w_vel_xy = 900.0;
    double term_w_vel_z = 900.0;
    double term_w_tilt = 200.0;
    double term_w_om = 50.0;
    double term_vz_ref = 0.0;
    double stage_w_time = 0.04;
    double r_theta = 10.0;
    double theta_ref = 0.10;
    double r_ctrl_thr = 2.0;
    double r_ctrl_mom = 0.6;
    double w_touch_vel = 10.0;
    double touch_alt = 0.25;
    double touch_scale = 0.25;
    double w_stage_target_xy = 0.0;
    double w_stage_target_z = 0.0;
    double w_stage_target_vel = 0.0;
    double w_land_speed = 1.0;
    double w_land_tilt = 1.0e5;
    double w_land_omega = 100.0;
    double w_land_gs = 5000.0;
    double w_stc_los = 50.0;
    double w_stc_stage = 10.0;
    double cap_cost_radius = 0.50;
    double k_cap = 20.0;
    double cap_desc_radius = 0.12;
    double k_desc = 40.0;
    double stage_attitude = 2.0;
    double stage_angular_rate = 5.0;
};

struct ConstraintsParameters {
    double gs_apex_offset = 0.1;
    double vz_land_max = 0.5;
    double theta_phase0_deg = 11.0;
    double omega_phase0_max = 0.35;
    double spd_phase0_max = 3.0;
    double theta_stc_deg = 0.95;
    double alpha_theta_stc = 1.0;
    double omega_stc_max = 0.019;
    double gs_cone_half_angle_deg = 15.0;
    double spd_stc_max = 0.75;
    double los_cone_half_angle_deg = 30.0;
    double los_alt_trig = 1.8;
    double los_trigger_scale = 0.25;
    double los_trigger_floor = 0.0;
    double alt_trig = 0.8;
    double cap_land_radius = 0.45;
    double z_stage = 1.8;
    double stage_radius = 1.10;
    double stage_trigger_scale = 0.25;
    double stage_trigger_floor = 0.0;
    double stage_settle_tilt_deg = 12.0;
    double stage_settle_omega_max = 0.50;
    double ctcs_y_scale = 10.0;
    double ctcs_step_eps = 1.6e-3;
    bool triggered_stc = true;
    bool capture_landing_trigger = false;
    std::string stage_trigger = "radius";
};

struct MpcParameters {
    static constexpr int horizon = 40;
    static constexpr double dt = 0.1;
    double rate_tau = 0.08;
    double rate_limit = 1.0;
    double hover_thrust = 0.7282821834;
    double motor_thrust_exponent = 1.0;
    double motor_speed_offset = 0.0;
    double max_policy_age = 1.0;
    double slew_thrust_weight = 0.1;
    double slew_rate_weight = 2.0;
    int max_iter = 60;
};
inline Param solverDefaults() {
    Param p;p.reg1_min=1e-3;p.reg2_min=0.1;p.mu_mul=0.1;
    p.rho=5;p.rhoT=5;p.rho_mul=10;p.tolerance=1e-6;p.max_iter=180;
    p.verbose=false;p.is_quaternion_in_state=false;p.use_ddp_terms=false;
    return p;
}
struct Parameters {
    VehicleParameters vehicle;
    HorizonParameters horizon;
    CostParameters cost;
    ConstraintsParameters constraints;
    MpcParameters mpc;
    Param solver = solverDefaults();
};
inline Parameters fromEnvironment() {
    Parameters p;
    p.horizon.horizon = std::max(1, std::getenv("SZMUK_RH_N") ? envOrInt("SZMUK_RH_N",30) : envOrInt("SZMUK_N",30));
    p.horizon.nex=std::max(1,envOrInt("SZMUK_RH_NEX",p.horizon.nex));
    p.horizon.th0 = envOrQ("SZMUK_TH0",p.horizon.th0);
    p.horizon.thl = envOrQ("SZMUK_THL",p.horizon.thl);
    p.horizon.thh = envOrQ("SZMUK_THH",p.horizon.thh);
    p.cost.term_w_pos_xy = envOrQ("SZMUK_TERM_W_POS_XY",p.cost.term_w_pos_xy);
    p.cost.term_w_pos_z = envOrQ("SZMUK_TERM_W_POS_Z",p.cost.term_w_pos_z);
    p.cost.term_w_vel_xy = envOrQ("SZMUK_TERM_W_VEL_XY",p.cost.term_w_vel_xy);
    p.cost.term_w_vel_z = envOrQ("SZMUK_TERM_W_VEL_Z",p.cost.term_w_vel_z);
    p.cost.term_w_tilt = envOrQ("SZMUK_TERM_W_TILT",p.cost.term_w_tilt);
    p.cost.term_w_om = envOrQ("SZMUK_TERM_W_OM",p.cost.term_w_om);
    p.cost.term_vz_ref = envOrQ("SZMUK_TERM_VZ_REF",p.cost.term_vz_ref);
    p.cost.stage_w_time = envOrQ("SZMUK_W_TIME",p.cost.stage_w_time);
    p.cost.r_theta = envOrQ("SZMUK_R_THETA",p.cost.r_theta);
    p.cost.theta_ref = envOrQ("SZMUK_THETA_REF",p.cost.theta_ref);
    p.cost.r_ctrl_thr = envOrQ("SZMUK_R_THR",p.cost.r_ctrl_thr);
    p.cost.r_ctrl_mom = envOrQ("SZMUK_R_MOM",p.cost.r_ctrl_mom);
    p.cost.w_touch_vel = envOrQ("SZMUK_W_TOUCH_VEL",p.cost.w_touch_vel);
    p.cost.touch_alt = envOrQ("SZMUK_TOUCH_ALT",p.cost.touch_alt);
    p.cost.touch_scale = envOrQ("SZMUK_TOUCH_SCALE",p.cost.touch_scale);
    p.cost.w_stage_target_xy = envOrQ("SZMUK_W_STAGE_TARGET_XY",p.cost.w_stage_target_xy);
    p.cost.w_stage_target_z = envOrQ("SZMUK_W_STAGE_TARGET_Z",p.cost.w_stage_target_z);
    p.cost.w_stage_target_vel = envOrQ("SZMUK_W_STAGE_TARGET_VEL",p.cost.w_stage_target_vel);
    p.cost.w_land_speed = envOrQ("SZMUK_W_LAND_SPEED",p.cost.w_land_speed);
    p.cost.w_land_tilt = envOrQ("SZMUK_W_LAND_TILT",p.cost.w_land_tilt);
    p.cost.w_land_omega = envOrQ("SZMUK_W_LAND_OMEGA",p.cost.w_land_omega);
    p.cost.w_land_gs = envOrQ("SZMUK_W_LAND_GS",p.cost.w_land_gs);
    p.cost.w_stc_los = envOrQ("SZMUK_W_STC_LOS",p.cost.w_stc_los);
    p.cost.w_stc_stage = envOrQ("SZMUK_W_STC_STAGE",p.cost.w_stc_stage);
    p.constraints.theta_phase0_deg = envOrQ("SZMUK_TILT_PHASE0_DEG",p.constraints.theta_phase0_deg);
    p.constraints.omega_phase0_max = envOrQ("SZMUK_OMEGA_PHASE0_MAX",p.constraints.omega_phase0_max);
    p.constraints.spd_phase0_max = envOrQ("SZMUK_SPD_PHASE0_MAX",p.constraints.spd_phase0_max);
    p.constraints.theta_stc_deg = envOrQ("SZMUK_TILT_STC_DEG",p.constraints.theta_stc_deg);
    p.constraints.alpha_theta_stc = envOrQ("SZMUK_ALPHA_THETA_STC",p.constraints.alpha_theta_stc);
    p.constraints.omega_stc_max = envOrQ("SZMUK_OMEGA_STC_MAX",p.constraints.omega_stc_max);
    p.constraints.gs_cone_half_angle_deg = envOrQ("SZMUK_GS_CONE_DEG",p.constraints.gs_cone_half_angle_deg);
    p.constraints.spd_stc_max = envOrQ("SZMUK_SPD_STC_MAX",p.constraints.spd_stc_max);
    p.constraints.los_cone_half_angle_deg = envOrQ("SZMUK_LOS_CONE_DEG",p.constraints.los_cone_half_angle_deg);
    p.constraints.los_alt_trig = envOrQ("SZMUK_LOS_ALT_TRIG",p.constraints.los_alt_trig);
    p.constraints.los_trigger_scale = envOrQ("SZMUK_LOS_TRIGGER_SCALE",p.constraints.los_trigger_scale);
    p.constraints.los_trigger_floor = envOrQ("SZMUK_LOS_TRIGGER_FLOOR",p.constraints.los_trigger_floor);
    p.constraints.alt_trig = envOrQ("SZMUK_ALT_TRIG",p.constraints.alt_trig);
    p.constraints.cap_land_radius = envOrQ("SZMUK_RH_CAP_LAND_RADIUS",p.constraints.cap_land_radius);
    p.constraints.z_stage = envOrQ("SZMUK_Z_STAGE",p.constraints.z_stage);
    p.constraints.stage_radius = envOrQ("SZMUK_STAGE_RADIUS",p.constraints.stage_radius);
    p.constraints.stage_trigger_scale = envOrQ("SZMUK_STAGE_TRIGGER_SCALE",p.constraints.stage_trigger_scale);
    p.constraints.stage_trigger_floor = envOrQ("SZMUK_STAGE_TRIGGER_FLOOR",p.constraints.stage_trigger_floor);
    p.constraints.stage_settle_tilt_deg = envOrQ("SZMUK_STAGE_SETTLE_TILT_DEG",p.constraints.stage_settle_tilt_deg);
    p.constraints.stage_settle_omega_max = envOrQ("SZMUK_STAGE_SETTLE_OMEGA_MAX",p.constraints.stage_settle_omega_max);
    p.constraints.ctcs_y_scale = envOrQ("SZMUK_CTCS_Y_SCALE",10.0); if (p.constraints.ctcs_y_scale <= 0) p.constraints.ctcs_y_scale=1.0;
    p.constraints.ctcs_step_eps = envOrQ("SZMUK_CTCS_STEP_EPS",p.constraints.ctcs_step_eps);
    const char* value=std::getenv("SZMUK_QUAD_TRIG_STC");p.constraints.triggered_stc=!(value&&std::string(value)=="0");
    value=std::getenv("SZMUK_RH_CAPTURE_LANDING_TRIGGER");p.constraints.capture_landing_trigger=value&&std::string(value)!="0";
    value=std::getenv("SZMUK_STAGE_TRIGGER");if(value)p.constraints.stage_trigger=value;
    value=std::getenv("SZMUK_RH_FEEDBACK_WARM_START");p.horizon.feedback_warm_start=!(value&&std::string(value)=="0");
    p.solver.reg2_min=envOrQ("SZMUK_REG2_MIN",p.solver.reg2_min);
    p.solver.rho=envOrQ("SZMUK_RHO",p.solver.rho);p.solver.rhoT=envOrQ("SZMUK_RHOT",p.solver.rhoT);
    p.solver.rho_mul=envOrQ("SZMUK_RHO_MUL",p.solver.rho_mul);
    p.solver.tolerance=envOrQ("SZMUK_TOL",p.solver.tolerance);
    p.solver.max_iter=envOrInt("SZMUK_MAX_ITER",p.solver.max_iter);
    p.solver.verbose=envOrInt("SZMUK_SOLVER_VERBOSE",0)!=0;
    p.mpc.rate_tau=envOrQ("MPC_RATE_TAU",p.mpc.rate_tau);p.mpc.rate_limit=envOrQ("MPC_RATE_LIMIT",p.mpc.rate_limit);
    p.mpc.hover_thrust=envOrQ("MPC_HOVER_THRUST",p.mpc.hover_thrust);
    p.mpc.motor_thrust_exponent=envOrQ("MPC_MOTOR_THRUST_EXPONENT",p.mpc.motor_thrust_exponent);
    p.mpc.motor_speed_offset=envOrQ("MPC_MOTOR_SPEED_OFFSET",p.mpc.motor_speed_offset);
    if(p.mpc.motor_thrust_exponent<=0||p.mpc.motor_speed_offset<0)
        throw std::invalid_argument("invalid motor thrust mapping parameters");
    p.mpc.max_policy_age=envOrQ("MPC_MAX_POLICY_AGE",p.mpc.max_policy_age);
    p.mpc.max_iter=static_cast<int>(envOrQ("MPC_MAX_ITER",p.mpc.max_iter));
    p.mpc.slew_thrust_weight=envOrQ("MPC_SLEW_THR",p.mpc.slew_thrust_weight);
    p.mpc.slew_rate_weight=envOrQ("MPC_SLEW_RATE",p.mpc.slew_rate_weight);
    return p;
}
inline Parameters hoverDefaults() {
    Parameters p;p.vehicle.mass=0.0282;p.vehicle.fmin=0.08;p.vehicle.fmax=0.42;
    p.horizon.horizon=100;p.horizon.th0=0.05;p.horizon.nex=4;
    p.cost.w_stage_target_xy=15;p.cost.w_stage_target_z=15;p.cost.w_stage_target_vel=20;
    p.cost.r_ctrl_thr=0.1;p.cost.r_ctrl_mom=0.1;
    p.cost.term_w_pos_xy=600;p.cost.term_w_pos_z=600;
    p.cost.term_w_vel_xy=100;p.cost.term_w_vel_z=100;p.cost.term_w_tilt=200;p.cost.term_w_om=100;
    p.solver=Param{};p.solver.reg1_min=1e-6;p.solver.reg2_min=1;p.solver.mu_mul=0.1;
    p.solver.rho=20;p.solver.rho_mul=9;p.solver.tolerance=1e-3;p.solver.max_iter=200;
    p.solver.is_quaternion_in_state=false;
    return p;
}
inline const Parameters defaults = fromEnvironment();
}
