#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <stdexcept>

// Platform data only (PX4 tracking model, motor mapping, setpoint timing).
// OCP costs, bounds and solver settings live in each OCP file.
namespace comando_parameters {
inline double envOrQ(const char* name,double fallback) {
    const char* value=std::getenv(name);return value ? std::atof(value) : fallback;
}
inline int envOrInt(const char* name,int fallback) {
    const char* value=std::getenv(name);return value ? std::atoi(value) : fallback;
}

// Platform settings shared by the controller and the MAVROS transport.
// Every OCP keeps its own costs, bounds and solver numbers in its own file.
inline constexpr double command_dt = .01;   // setpoint period [s]

// PX4 rate loop and motor thrust mapping used by the direct-MPC plants and
// the MAVROS body-rate interface.
struct MpcParameters {
    double rate_tau = 0.08;
    double rate_limit = 1.0;
    double hover_thrust = 0.7282821834;
    double motor_thrust_exponent = 1.0;
    double motor_speed_offset = 0.0;
    double max_policy_age = 1.0;
    bool native_px4 = false;
};
// Native PX4 position-controller coefficients, not planner feedback gains.
// They must match MPC_ACC_DECOUPLE and MPC_XY/Z_VEL_D_ACC on the FCU.
struct Px4TrackingParameters {
    // PX4 multicopter_position_control_gain_params.c / mc_att_control_params.c.
    // Local tracking-error prediction; these must match the FCU parameters.
    Eigen::Vector3d position_p{.95,.95,1.};
    Eigen::Vector3d velocity_p{1.8,1.8,4.};
    Eigen::Vector3d attitude_p{4.,4.,2.8};
    bool decouple_acceleration = true;
    double velocity_d_xy = .2;
    double velocity_d_z = 0.;
    // x500 native plant defaults. Physical rotor geometry differs from the
    // FCU allocation geometry; keep both as data rather than fitted gains.
    double mass = 2.0643076923076915;
    double gravity = 9.8;
    Eigen::Vector3d inertia{.02383948067755798,.023942405487905428,.04399995371400395};
    double motor_constant = 8.54858e-6, moment_ratio = .016;
    double motor_min = 150., motor_max = 1000.;
    double motor_tau_up = .0125, motor_tau_down = .025;
    double gyro_cutoff_hz = 40., derivative_cutoff_hz = 20.;
    Eigen::Vector3d rate_p{.15,.15,.2}, rate_i{.2,.2,.1}, rate_d{.003,.003,0.};
    Eigen::Matrix<double,4,2> rotor_position_frd =
        (Eigen::Matrix<double,4,2>()<<.174,.174,-.174,-.174,.174,-.174,-.174,.174).finished();
    Eigen::Matrix<double,4,3> mixer_frd =
        (Eigen::Matrix<double,4,3>()<<-.4377327513863796,.7071067811865471,.9090909275634227,
          .43773275138637974,-.707106781186547,1.,
          .43773275138637924,.7071067811865477,-.9090909275634222,
          -.4377327513863796,-.7071067811865482,-1.).finished();
};
struct Parameters {
    MpcParameters mpc;
    Px4TrackingParameters px4;
};
inline Parameters fromEnvironment() {
    Parameters p;
    p.mpc.rate_tau=envOrQ("MPC_RATE_TAU",p.mpc.rate_tau);p.mpc.rate_limit=envOrQ("MPC_RATE_LIMIT",p.mpc.rate_limit);
    p.mpc.native_px4=envOrInt("MPC_NATIVE_PX4",0)!=0;
    p.mpc.hover_thrust=envOrQ("MPC_HOVER_THRUST",p.mpc.hover_thrust);
    p.mpc.motor_thrust_exponent=envOrQ("MPC_MOTOR_THRUST_EXPONENT",p.mpc.motor_thrust_exponent);
    p.mpc.motor_speed_offset=envOrQ("MPC_MOTOR_SPEED_OFFSET",p.mpc.motor_speed_offset);
    p.px4.decouple_acceleration=envOrInt("PX4_ACC_DECOUPLE",p.px4.decouple_acceleration)!=0;
    p.px4.velocity_d_xy=envOrQ("PX4_VEL_D_XY",p.px4.velocity_d_xy);
    p.px4.velocity_d_z=envOrQ("PX4_VEL_D_Z",p.px4.velocity_d_z);
    if(p.mpc.motor_thrust_exponent<=0||p.mpc.motor_speed_offset<0)
        throw std::invalid_argument("invalid motor thrust mapping parameters");
    p.mpc.max_policy_age=envOrQ("MPC_MAX_POLICY_AGE",p.mpc.max_policy_age);
    return p;
}
inline const Parameters defaults = fromEnvironment();
}
