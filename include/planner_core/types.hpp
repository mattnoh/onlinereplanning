/// @file planner_core/types.hpp
/// @brief ROS-free planner/OCP data contracts for future ROS1/ROS2 wrappers.
#pragma once

#include <Eigen/Dense>

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "optimal_control_problem.h"
#include "alipddp/alipddp.h"

/// Snapshot of target state taken at solve time.
/// Passed to transform_state, validate_target, post_process_result callbacks.
struct TargetSnapshot {
    bool valid = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector4d orientation = Eigen::Vector4d(1,0,0,0);
    double odom_stamp_sec = 0.0;
    double accel_stamp_sec = 0.0;
};

struct PlannerConfig {
    std::string ocp_type = "";
    std::string drone_name = "gogogo";
    bool enable_logging = true;
    std::string platform = "crazyflie";
    std::string solver_type = "alipddp";
    std::string mode = "";

    double hover_target_x = 0.0;
    double hover_target_y = 0.0;
    double hover_target_z = 0.0;

    double circle_center_x = 0.0;
    double circle_center_y = 0.0;
    double circle_center_z = 0.2;
    double circle_R = 1.0;
    double circle_omega = 0.4;
    double circle_phi0 = 0.0;

    int n_replay = 0;

    bool drone_state_is_relative = false;
    std::string drone_odom_topic = "";
    std::string body_relative_odom_topic = "/drone/body_relative_odom";

    std::string target_odom_topic = "/target/odom";
    std::string target_accel_topic = "/target/accel";
    bool debug_body_relative_trace = false;

    bool enable_terminal_freeze = true;
    double terminal_freeze_enter_pos = 0.20;
    double terminal_freeze_enter_vel = 0.10;
    bool terminal_freeze_require_vel = false;
    double terminal_freeze_exit_pos = 0.20;

    bool start_paused = true;
    int command_seq = 0;

    double ocp_dt = 0.0;
    double dt = 0.0;
    double mass_kg = 0.0;
    bool open_loop_abort_on_divergence = false;
    double open_loop_abort_max_z_error_m = 0.50;
    double open_loop_abort_max_vz_error_mps = 1.00;

    double t_start_abs = 0.0;

    bool isConfigured() const {
        return !ocp_type.empty() && !mode.empty();
    }
};

namespace OCPLoggerDefaults {

inline Eigen::VectorXd makeHoverState13D(const Eigen::VectorXd& current_state) {
    Eigen::VectorXd x_hover = (current_state.size() >= 13)
        ? current_state
        : Eigen::VectorXd::Zero(13);
    x_hover.segment(3, 3).setZero();
    x_hover(6) = 1.0;
    x_hover.segment(7, 3).setZero();
    x_hover.segment(10, 3).setZero();
    return x_hover;
}

inline std::vector<std::string> getStateHeaders13D() {
    return {
        "x","y","z","vx","vy","vz","qw","qx","qy","qz","wx","wy","wz",
        "abs_x","abs_y","abs_z","abs_vx","abs_vy","abs_vz","abs_qw","abs_qx","abs_qy","abs_qz","abs_wx","abs_wy","abs_wz",
        "rel_x","rel_y","rel_z","rel_vx","rel_vy","rel_vz","rel_qw","rel_qx","rel_qy","rel_qz","rel_wx","rel_wy","rel_wz",
        "tgt_x","tgt_y","tgt_z","tgt_vx","tgt_vy","tgt_vz"
    };
}

inline std::vector<double> getActualStateRow13D(const Eigen::VectorXd& state, const TargetSnapshot& tgt, const std::string& coord_mode) {
    if (state.size() < 13) return std::vector<double>(45, 0.0);
    std::vector<double> row;
    row.reserve(45);
    Eigen::VectorXd x_abs = state.head(13);
    Eigen::VectorXd x_rel = state.head(13);
    if (coord_mode == "relative") {
        x_abs.segment(0, 3) += tgt.position;
        x_abs.segment(3, 3) += tgt.velocity;
    } else {
        x_rel.segment(0, 3) -= tgt.position;
        x_rel.segment(3, 3) -= tgt.velocity;
    }
    for (int i=0; i<13; ++i) row.push_back(state(i));
    for (int i=0; i<13; ++i) row.push_back(x_abs(i));
    for (int i=0; i<13; ++i) row.push_back(x_rel(i));
    row.push_back(tgt.position.x()); row.push_back(tgt.position.y()); row.push_back(tgt.position.z());
    row.push_back(tgt.velocity.x()); row.push_back(tgt.velocity.y()); row.push_back(tgt.velocity.z());
    return row;
}

inline std::vector<std::string> getStateNames13D() {
    return {"px","py","pz","vx","vy","vz","qw","qx","qy","qz","wx","wy","wz"};
}

inline std::vector<std::string> getControlNames4D() {
    return {"fz","mx","my","mz"};
}

inline std::vector<std::string> getStateNames22D() {
    return {"p0","p1","p2","v0","v1","v2","qw","qx","qy","qz",
            "wx","wy","wz","OmN0","OmN1","OmN2","aT0","aT1","aT2",
            "bN0","bN1","bN2"};
}

inline std::vector<std::string> getStateHeaders23D_Body() {
    return {
        "p0","p1","p2",
        "v0","v1","v2",
        "qw","qx","qy","qz",
        "wx","wy","wz",
        "drone_x","drone_y","drone_z",
        "p_err",
        "tgt_x","tgt_y","tgt_z","tgt_vx","tgt_vy","tgt_vz"
    };
}

inline std::vector<double> getActualStateRow23D_Body(
    const Eigen::VectorXd& state,
    const TargetSnapshot& tgt,
    const std::string&)
{
    constexpr int N_OUT  = 19;
    std::vector<double> row;
    row.reserve(N_OUT);

    auto s = [&](int i) -> double {
        return (i < (int)state.size()) ? state(i) : 0.0;
    };

    row.push_back(s(0)); row.push_back(s(1)); row.push_back(s(2));
    row.push_back(s(3)); row.push_back(s(4)); row.push_back(s(5));
    row.push_back(s(6)); row.push_back(s(7)); row.push_back(s(8)); row.push_back(s(9));
    row.push_back(s(10)); row.push_back(s(11)); row.push_back(s(12));

    {
        const double qw = s(6), qx = s(7), qy = s(8), qz = s(9);
        Eigen::Matrix3d R_WB;
        R_WB << 1 - 2*qy*qy - 2*qz*qz,  2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
                2*qx*qy + 2*qz*qw,      1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw,
                2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,      1 - 2*qx*qx - 2*qy*qy;
        Eigen::Vector3d p_T_b(s(0), s(1), s(2));
        Eigen::Vector3d drone_w = tgt.position - R_WB * p_T_b;
        row.push_back(drone_w(0)); row.push_back(drone_w(1)); row.push_back(drone_w(2));
        row.push_back(p_T_b.norm());
    }

    row.push_back(tgt.position.x()); row.push_back(tgt.position.y()); row.push_back(tgt.position.z());
    row.push_back(tgt.velocity.x()); row.push_back(tgt.velocity.y()); row.push_back(tgt.velocity.z());

    return row;
}

inline Eigen::VectorXd makeHoverState23D_Body(const Eigen::VectorXd& current_state) {
    Eigen::VectorXd x_hover = (current_state.size() >= 22)
        ? current_state
        : Eigen::VectorXd::Zero(22);
    x_hover.segment(3, 3).setZero();
    x_hover(6) = 1.0;
    x_hover.segment(7, 3).setZero();
    x_hover.segment(10, 3).setZero();
    return x_hover;
}

}  // namespace OCPLoggerDefaults

namespace planner_logging { struct SolveLogMeta; }

struct SolverResult {
    bool success = false;
    Eigen::VectorXd next_state;
    std::vector<Eigen::VectorXd> state_trajectory;
    std::vector<Eigen::VectorXd> control_trajectory;
    // DDP feedback gains, one per stage. Exposed with the solver result for
    // OCP-level policy evaluation and replay implementations that opt in.
    std::vector<Eigen::MatrixXd> feedback_gains;
    double solve_time_ms = 0.0;
    double constraint_error = 0.0;
    int solve_iters = 0;
    std::chrono::steady_clock::time_point solve_start_time;
    std::chrono::steady_clock::time_point solve_finish_time;
    std::chrono::steady_clock::time_point solve_timestamp;

    bool is_relative_plan = false;
    Eigen::Vector3d target_snapshot_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_acc = Eigen::Vector3d::Zero();
    Eigen::Vector4d target_snapshot_quat = Eigen::Vector4d(1,0,0,0);
    Eigen::Vector3d target_snapshot_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_beta = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> target_world_pos_trajectory;
    std::vector<Eigen::Vector3d> target_world_vel_trajectory;
    std::string target_motion_source = "snapshot";
    std::any extra;
};

struct OCPCreateArgs {
    Eigen::VectorXd current_state;
    Eigen::VectorXd terminal_state;
    std::vector<Eigen::VectorXd> prev_U;
    std::vector<Eigen::VectorXd> prev_X;
    std::vector<Eigen::MatrixXd> prev_K;
    Eigen::Vector3d target_accel;
    std::any extra;
    double t_abs = 0.0;
};

namespace planner_core { enum class ExecutionMode { RhReplanning, DirectMpc }; }

struct OCPDescriptor {
    planner_core::ExecutionMode execution_mode=planner_core::ExecutionMode::RhReplanning;
    enum class CommandMode  { CmdFullState, CmdBodyRate };
    enum class DroneOdomMode { Absolute, AbsoluteShiftedTarget, BodyFrameRelative, TargetFrameRelative };
    enum class WarmStart    { Shift, Feedback };

    std::string name;
    double dt = 0.05;
    int default_n_replay = 4;
    double default_mass_kg = 0.027;
    WarmStart warm_start = WarmStart::Shift;
    CommandMode command_mode = CommandMode::CmdFullState;
    DroneOdomMode drone_odom_mode = DroneOdomMode::Absolute;

    bool disarm_on_landing_finish = false;
    bool skip_altitude_validation = false;
    bool skip_trajectory_validation = false;
    bool variable_dt = false;
    bool use_predicted_handoff_state = true;

    int state_dim = 13;
    int control_dim = 4;

    std::vector<std::string> log_state_headers;
    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
    std::function<std::vector<double>(
        const Eigen::VectorXd&, const TargetSnapshot&, const std::string&)>
        extract_actual_state_row;

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> make_hover_state;
    std::function<Eigen::VectorXd(
        const Eigen::VectorXd& x_drone, const TargetSnapshot&)>
        transform_state;
    std::function<bool(const TargetSnapshot&, double now_sec, double max_age)>
        validate_target;
    std::function<void(SolverResult&, const TargetSnapshot&)>
        post_process_result;
    std::function<std::any(const PlannerConfig&, double t_abs, const TargetSnapshot&)>
        prepare_extra;
    std::function<void(planner_logging::SolveLogMeta&, const std::any& extra_params,
                       const std::any& runtime_cfg)>
        prepare_log_meta;
    std::function<void(Eigen::VectorXd&,
                       const std::vector<Eigen::VectorXd>&,
                       int)> merge_prev_augmented;
    std::function<void(Eigen::VectorXd&)> sanitize_warm_control;
    std::function<Eigen::VectorXd(
        const Eigen::VectorXd& x_ocp,
        const TargetSnapshot& target)> reconstruct_world_state;

    std::function<Param()> getSolverParams;
    std::function<std::shared_ptr<OptimalControlProblem<double>>(const OCPCreateArgs&)>
        create;
};

namespace planner_core {

enum class Platform {
    Crazyflie,
    Mavros,
};

enum class CommandKind {
    None,
    FullState,
    Hover,
    BodyRate,
};

struct PlannerInputState {
    Eigen::VectorXd state;
    std::string frame_label = "absolute";
    std::chrono::steady_clock::time_point stamp{};
};

struct TargetInputState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector4d orientation = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
    bool valid = false;
};

struct PlannerCommand {
    CommandKind kind = CommandKind::None;
    Eigen::VectorXd state;
    Eigen::VectorXd control;
    Eigen::Vector4d hover = Eigen::Vector4d::Zero();  // [vx_body, vy_body, z_world, yaw_rate]
};

struct PlannerPath {
    std::vector<Eigen::VectorXd> states;
    std::string frame_id = "world";
};

enum class DiagnosticSeverity {
    Info,
    Warn,
    Error,
};

struct PlannerEvent {
    std::string level = "info";
    std::string message;
    double value = 0.0;
};

struct PlannerDiagnostic {
    DiagnosticSeverity severity = DiagnosticSeverity::Info;
    std::string code;
    std::string message;
    std::map<std::string, double> values;
    Eigen::VectorXd state_a;
    Eigen::VectorXd state_b;
    Eigen::VectorXd control_a;
    Eigen::VectorXd control_b;
};

struct PlannerCoreConfig {
    std::string ocp_type;
    Platform platform = Platform::Crazyflie;
    std::string solver_type = "alipddp";
    double ocp_dt = 0.05;
    int n_replay = 4;
    double max_constraint_error = 1.0;
    bool skip_trajectory_validation = false;
    bool enable_terminal_freeze = true;
    double terminal_freeze_enter_pos = 0.20;
    double terminal_freeze_enter_vel = 0.10;
    bool terminal_freeze_require_vel = false;
    double terminal_freeze_exit_pos = 0.20;
    Eigen::Vector3d terminal_position_abs = Eigen::Vector3d::Zero();
    Eigen::VectorXd terminal_state;
    std::string body_relative_odom_topic = "/drone/body_relative_odom";
    std::string target_frame_odom_topic = "/drone/target_frame_odom";
    double solve_lead_guard_sec = 0.05;
    double initial_solve_lead_sec = 0.20;
    // Seeding the replan from the MEASURED state.
    //
    //   x0 = plan(t_activation) + clamp(alpha * (live(now) - plan(now)))
    //
    // applied to the POSITION block only. alpha is the fraction of tracking
    // error PX4 still leaves at the activation lead, measured from bag odom vs
    // published PositionTarget (LS fit e(t+tau) = rho e(t) at tau ~0.48 s):
    // rho_pos 0.642/0.774 in SITL and 0.698 on Crazyflie hardware.
    //
    // Attitude and body rates are NEVER seeded: the STC bounds on them are
    // tighter than the vehicle's real behaviour (omega <= 0.019 rad/s vs
    // 0.03-0.50 flown), so a measured attitude/rate seed is infeasible 37-60%
    // of the time and diverges the IPM. Velocity is feasible but its error is
    // not persistent on hardware (rho_vel = -0.10), so it defaults off too.
    //
    // The clamp MUST scale with altitude. tools_seed_probe sweeps the feasible
    // seed perturbation along a descent: the budget tracks the glideslope cone
    // and collapses from 0.40 m at rel z 1.6 to 0.02 m at rel z 0.06, so a
    // constant clamp would have to be 0.02 m and correct nothing. The rule
    // min(cap, per_m * rel_z) at (0.15, 0.12) passed 21/21 phases; the shipped
    // (0.10, 0.10) is strictly smaller.
    double seed_alpha_pos = 0.70;         // 0 = pre-existing predicted-handoff seed
    double seed_alpha_vel = 0.0;
    double seed_clamp_pos_cap_m = 0.10;   // absolute cap on the applied correction
    double seed_clamp_pos_per_m = 0.10;   // and per metre of relative altitude
    double seed_clamp_vel = 0.30;         // probe: 0.30 m/s feasible at every phase
    // Below this relative altitude the seed correction is switched off entirely
    // and the plain predicted seed is used. Flown at 0.10/0.10 with no cutoff,
    // every solve from rel z ~0.45 to touchdown was rejected (constraint 2-9)
    // for the last 1.15 s of descent. There is also nothing to gain there: by
    // that point the measured tracking error has already fallen to 0.017-0.05 m,
    // while the large errors this mechanism exists to correct (0.12-0.18 m)
    // occur mid-descent.
    double seed_min_rel_z = 0.60;
    double recovery_pos_err = 0.50;
    double recovery_vel_err = 2.00;
    double max_first_solve_age_sec = 1.00;
    double max_relative_position_norm = 10.0;
    double max_relative_vertical_abs = 5.0;
    double max_relative_velocity_norm = 8.0;
};

struct PlannerCoreInput {
    Eigen::VectorXd current_state;
    TargetSnapshot target_snapshot;
    OCPDescriptor::DroneOdomMode active_odom_mode = OCPDescriptor::DroneOdomMode::Absolute;
    std::string active_odom_topic;
    std::chrono::steady_clock::time_point now{};
    double absolute_time_sec = 0.0;
};

struct PlannerSolveLogData {
    int solve_num = 0;
    double solve_time_ms = 0.0;
    int solve_iters = 0;
    bool is_relative_plan = false;
    double ocp_dt = 0.05;
    std::string coord_mode = "absolute";
    Eigen::Vector3d target_snapshot_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_acc = Eigen::Vector3d::Zero();
    Eigen::Vector4d target_snapshot_quat = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d target_snapshot_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_snapshot_beta = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> target_world_pos_trajectory;
    std::vector<Eigen::Vector3d> target_world_vel_trajectory;
};

struct PlannerCoreStepResult {
    bool solve_attempted = false;
    bool solve_accepted = false;
    bool has_replay_sample = false;
    bool terminal_freeze_engaged = false;
    bool terminal_freeze_released = false;
    bool stale = false;
    std::string rejection_reason;
    std::string x0_source = "live";
    double handoff_pos_err = 0.0;
    double handoff_vel_err = 0.0;
    double replan_delay_sec = 0.0;
    double active_elapsed_now_sec = 0.0;
    double activation_elapsed_sec = 0.0;
    double activation_wall_time_sec = 0.0;
    double solve_lead_sec = 0.0;
    double solve_finish_late_by_sec = 0.0;
    int solve_num = -1;
    SolverResult solve_result;
    PlannerPath path;
    PlannerCommand command;
    PlannerSolveLogData solve_log;
    Eigen::VectorXd actual_state_for_log;
    TargetSnapshot target_for_log;
    std::vector<PlannerDiagnostic> diagnostics;
};

}  // namespace planner_core
