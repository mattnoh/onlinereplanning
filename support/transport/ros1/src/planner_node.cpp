/// @file planner_node.cpp  (ROS1 Noetic branch)
/// @brief CoManDO planner node — MAVROS platform, roscpp.

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "parameters.hpp"
#include "planner_core/adapters.hpp"
#include "planner_core/types.hpp"
#include "platform/mavros.hpp"
#include "platform/frame_normalizer.hpp"
#include "platform/target_tracker.hpp"
#include "platform/state_monitor.hpp"
#include "trajectory_replayer.hpp"
#include "planner_runtime_config.hpp"
#include "planner_logging.hpp"
#include "planner_core/controller.hpp"
#include "hover_controller.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <vector>
#include <deque>
#include <algorithm>
#include <sstream>
#include <array>
#include <limits>

using Clock = std::chrono::steady_clock;

class PlannerNode {
public:
    explicit PlannerNode(ros::NodeHandle& nh) : nh_(nh) {
        PlannerRuntimeConfig runtime_cfg = loadPlannerRuntimeConfig(nh_);
        runtime_cfg.t_start_abs = ros::Time::now().toSec();
        runtime_cfg_ = runtime_cfg;

        ocp_type_      = runtime_cfg.ocp_type;
        drone_name_    = runtime_cfg.drone_name;
        logging_enabled_ = runtime_cfg.enable_logging;
        platform_      = runtime_cfg.platform;
        solver_type_   = runtime_cfg.solver;
        mode_          = runtime_cfg.mode;
        n_replay_      = runtime_cfg.n_replay;
        mass_kg_       = runtime_cfg.mass_kg;

        nh_.param<double>("hover_thrust", hover_thrust_d_, 0.3);
        hover_thrust_param_ = float(hover_thrust_d_);
        nh_.param<bool>  ("skip_trajectory_validation", skip_trajectory_validation_, false);
        std::string log_dir;
        nh_.param<std::string>("log_dir", log_dir, std::string{});
        logger_.setLogRoot(log_dir);
        nh_.param<double>("max_first_solve_age_sec",    max_first_solve_age_sec_,    1.0);
        nh_.param<double>("max_constraint_error",       max_constraint_error_,       max_constraint_error_);
        nh_.param<double>("max_relative_position_norm", max_relative_position_norm_, 10.0);
        nh_.param<double>("max_relative_vertical_abs",  max_relative_vertical_abs_,  5.0);
        nh_.param<double>("max_relative_velocity_norm", max_relative_velocity_norm_,  8.0);

        open_loop_abort_on_divergence_    = runtime_cfg.open_loop_abort_on_divergence;
        open_loop_abort_max_z_error_m_    = runtime_cfg.open_loop_abort_max_z_error_m;
        open_loop_abort_max_vz_error_mps_ = runtime_cfg.open_loop_abort_max_vz_error_mps;
        world_frame_ = runtime_cfg.world_frame;
        drone_odom_topic_ = runtime_cfg.drone_odom_topic;
        target_odom_topic_               = runtime_cfg.target_odom_topic;
        target_accel_topic_              = runtime_cfg.target_accel_topic;
        seed_alpha_pos_                  = runtime_cfg.seed_alpha_pos;
        seed_alpha_vel_                  = runtime_cfg.seed_alpha_vel;
        seed_clamp_pos_cap_m_            = runtime_cfg.seed_clamp_pos_cap_m;
        seed_clamp_pos_per_m_            = runtime_cfg.seed_clamp_pos_per_m;
        seed_clamp_vel_                  = runtime_cfg.seed_clamp_vel;
        seed_min_rel_z_                  = runtime_cfg.seed_min_rel_z;
        enable_terminal_freeze_          = runtime_cfg.enable_terminal_freeze;
        terminal_freeze_enter_pos_       = runtime_cfg.terminal_freeze_enter_pos;
        terminal_freeze_enter_vel_       = runtime_cfg.terminal_freeze_enter_vel;
        terminal_freeze_require_vel_     = runtime_cfg.terminal_freeze_require_vel;
        terminal_freeze_exit_pos_        = runtime_cfg.terminal_freeze_exit_pos;
        command_paused_.store(runtime_cfg.start_paused);
        last_command_seq_ = runtime_cfg.command_seq;

        hover_target_.x() = runtime_cfg.hover_target_x;
        hover_target_.y() = runtime_cfg.hover_target_y;
        hover_target_.z() = runtime_cfg.hover_target_z;

        if (runtime_cfg.isConfigured()) {
            ocp_dt_ = runtime_cfg.ocp_dt;
            const auto& desc = OCPRegistry::getDescriptor(ocp_type_);
            command_mode_            = desc.command_mode;
            state_dim_               = desc.state_dim;
            control_dim_             = desc.control_dim;
            custom_make_hover_state_ = desc.make_hover_state;
            setTerminalTarget(hover_target_);
            rebuildMpcSolver();
            is_configured_ = true;
        } else {
            ocp_dt_                  = 0.0;

            is_configured_           = false;
        }

        frames_=std::make_shared<platform::FrameNormalizer>(world_frame_);
        std::string velocity_frame,target_velocity_frame,command_frame;
        nh_.param<std::string>("state_velocity_frame",velocity_frame,"body");
        nh_.param<std::string>("target_velocity_frame",target_velocity_frame,"world");
        nh_.param<std::string>("command_frame",command_frame,"map");
        if((velocity_frame!="body"&&velocity_frame!="world")||(target_velocity_frame!="body"&&target_velocity_frame!="world"))throw std::runtime_error("velocity frame must be body or world");
        mavros_handles_.odom_topic=drone_odom_topic_;
        mavros_handles_.normalize_odom=[this,velocity_frame](const nav_msgs::Odometry& msg){return frames_->odometry(msg,velocity_frame=="body");};
        mavros_handles_.command_to_local=[this,command_frame](const Eigen::VectorXd& x){return frames_->command(x,command_frame);};
        if (platform_ == "mavros") {
            platform::mavros::setup(
                nh_, hover_thrust_param_,
                state_monitor_.droneState(),
                state_monitor_.stateMutex(),
                mavros_handles_);
        } else {
            ROS_ERROR("Unknown platform: %s (only 'mavros' supported on ROS1 branch)", platform_.c_str());
            throw std::runtime_error("Unknown platform: " + platform_);
        }

        platform::target_tracker::setup(
            nh_,
            target_odom_topic_, target_accel_topic_,
            state_monitor_.targetState(), state_monitor_.targetMutex(), target_handles_,
            [this,target_velocity_frame](const nav_msgs::Odometry& msg){return frames_->odometry(msg,target_velocity_frame=="body");},
            [this](const geometry_msgs::AccelStamped& msg){return frames_->acceleration(msg);});

        traj_pub_ = nh_.advertise<nav_msgs::Path>("/" + drone_name_ + "/planned_trajectory", 10);

        if (is_configured_) {
            createModeTimers();
        }

        // Poll ROS param server at 10 Hz for runtime OCP/target/seq changes.
        param_poll_timer_ = nh_.createTimer(
            ros::Duration(0.1), &PlannerNode::pollParameters, this);

        if (is_configured_) {
            ROS_INFO("Ready mode=%s platform=%s solver=%s ocp=%s ocp_dt=%.3fs n_replay=%d paused=%s",
                mode_.c_str(), platform_.c_str(), solver_type_.c_str(),
                ocp_type_.c_str(), ocp_dt_, n_replay_,
                command_paused_.load() ? "true" : "false");
        } else {
            ROS_WARN("Planner started UNCONFIGURED. Set ocp_type and mode params to start.");
        }
        ROS_INFO("Input state mode: %s (drone_odom_topic='%s')",
            currentFrameLabel(), drone_odom_topic_.c_str());
    }

private:
    // ── Parameter poll (replaces ROS2 on_set_parameters callback) ────────────
    void pollParameters(const ros::TimerEvent&) {
        // A waiting writer can block replay readers until an in-flight solve
        // finishes. Parameter changes can retry on the next poll; references
        // must keep their wall-clock cadence throughout that solve.
        std::unique_lock<std::shared_mutex> session(session_mutex_,std::try_to_lock);
        if (!session.owns_lock()) return;
        std::lock_guard<std::mutex> lk(command_mutex_);

        std::string new_ocp_type = ocp_type_;
        std::string new_mode     = mode_;
        int         new_n_replay = n_replay_;
        Eigen::Vector3d new_target = hover_target_;
        bool profile_changed = false;
        bool ocp_type_changed = false;

        // Initialize to current values: getParam() leaves the output untouched
        // when the param is absent, and most of these are unset until the
        // first profile is applied.
        std::string p_ocp = ocp_type_, p_mode = mode_;
        int p_n_replay = n_replay_, p_seq = last_command_seq_;
        double p_x = hover_target_.x(), p_y = hover_target_.y(), p_z = hover_target_.z();
        bool p_diverge = open_loop_abort_on_divergence_;
        double p_ze = open_loop_abort_max_z_error_m_, p_vze = open_loop_abort_max_vz_error_mps_;

        nh_.getParam("ocp_type",   p_ocp);
        nh_.getParam("mode",       p_mode);
        nh_.getParam("n_replay",   p_n_replay);
        nh_.getParam("hover_target_x", p_x);
        nh_.getParam("hover_target_y", p_y);
        nh_.getParam("hover_target_z", p_z);
        nh_.getParam("command_seq",    p_seq);
        nh_.getParam("open_loop_abort_on_divergence",    p_diverge);
        nh_.getParam("open_loop_abort_max_z_error_m",   p_ze);
        nh_.getParam("open_loop_abort_max_vz_error_mps",p_vze);
        nh_.getParam("land_action", land_action_);

        if (!p_ocp.empty() && p_ocp != ocp_type_) {
            new_ocp_type = p_ocp;
            try { (void)OCPRegistry::getDT(new_ocp_type); }
            catch (const std::exception& e) {
                ROS_ERROR("Invalid ocp_type '%s': %s", p_ocp.c_str(), e.what());
                return;
            }
            profile_changed = true;
            ocp_type_changed = true;
        }
        if (!p_mode.empty() && p_mode != mode_) {
            if (p_mode != "mpc" && p_mode != "open_loop") {
                ROS_ERROR("mode must be 'mpc' or 'open_loop'");
                return;
            }
            new_mode = p_mode;
            profile_changed = true;
        }
        if (p_n_replay > 0 && p_n_replay != n_replay_) { new_n_replay = p_n_replay; profile_changed = true; }
        if (p_x != hover_target_.x()) { new_target.x() = p_x; profile_changed = true; }
        if (p_y != hover_target_.y()) { new_target.y() = p_y; profile_changed = true; }
        if (p_z != hover_target_.z()) { new_target.z() = p_z; profile_changed = true; }

        open_loop_abort_on_divergence_    = p_diverge;
        open_loop_abort_max_z_error_m_    = p_ze;
        open_loop_abort_max_vz_error_mps_ = p_vze;

        if (profile_changed) {
            nh_.setParam("ordinary_start_verified",false);
            feasible_start_since_={};
            if (new_ocp_type.empty()) {
                ROS_ERROR("Cannot apply profile: ocp_type is not set.");
                return;
            }
            ocp_type_ = new_ocp_type;
            mode_     = new_mode;
            n_replay_ = new_n_replay;

            if (ocp_type_changed || !is_configured_) {
                const auto& desc = OCPRegistry::getDescriptor(ocp_type_);
                ocp_dt_       = desc.dt;
                mass_kg_      = desc.default_mass_kg;
                command_mode_ = desc.command_mode;
                state_dim_    = desc.state_dim;
                control_dim_  = desc.control_dim;
                custom_make_hover_state_ = desc.make_hover_state;
                if (n_replay_ == 0) n_replay_ = desc.default_n_replay;
            }

            {
                // Serialize against in-flight solves: rebuildMpcSolver()
                // destroys the solver PlannerCore's trySolve may be using on
                // the other spinner thread.
                std::lock_guard<std::mutex> slk(solver_mutex_);
                setTerminalTarget(new_target);
                rebuildMpcSolver();
            }

            is_configured_ = !ocp_type_.empty() && !mode_.empty();
            // Idempotent per-timer; also needed when an already-configured
            // node switches mode (each callback guards on the active mode).
            if (is_configured_) createModeTimers();

            resetForNewCommand();
            command_paused_.store(true);

            ROS_INFO("Profile applied: ocp=%s mode=%s n_replay=%d target=[%.3f,%.3f,%.3f]",
                ocp_type_.c_str(), mode_.c_str(), n_replay_,
                hover_target_.x(), hover_target_.y(), hover_target_.z());
            logging_initialized_=false;
            ensureLoggingInitialized();
        }

        if (p_seq > last_command_seq_) {
            if (!is_configured_) {
                ROS_ERROR("Cannot start: configure ocp_type and mode first.");
                return;
            }
            // Target-relative OCPs solve against the target snapshot; starting
            // without one silently lands on garbage. Hold the command (seq not
            // consumed) until /target/odom is fresh — it auto-starts once the
            // target publisher is up.
            if (OCPRegistry::getDescriptor(ocp_type_).coordinates == OCPDescriptor::Coordinates::TargetRelative) {
                bool target_fresh;
                {
                    std::lock_guard<std::mutex> tlk(state_monitor_.targetMutex());
                    target_fresh = state_monitor_.targetState()
                                       .isOdomFresh(ros::Time::now(), 1.0);
                }
                if (!target_fresh) {
                    ROS_ERROR_THROTTLE(2.0,
                        "Cannot start %s: no fresh /target/odom (is "
                        "target_launch running?). Waiting for target data.",
                        ocp_type_.c_str());
                    return;
                }
            }
            if (planner_core_.isDirect() &&
                std::abs(hover_thrust_d_ - comando_parameters::defaults.mpc.hover_thrust) > 1e-3) {
                ROS_ERROR_THROTTLE(2.0,"Direct MPC requires hover_thrust to match MPC_HOVER_THRUST");
                return;
            }
            const auto& descriptor=OCPRegistry::getDescriptor(ocp_type_);
            if(descriptor.feasible_start) {
                if(!hasState()||!hasFreshTargetState()){feasible_start_since_={};return;}
                const auto target=getTargetSnapshot();
                const auto live=planner_core::prepareOcpState(descriptor,getCurrentState(),target);
                if(!hasState()||!hasFreshTargetState()||!descriptor.feasible_start(live)) {
                    feasible_start_since_={};ROS_WARN_THROTTLE(2,"Waiting for feasible cone start for %s",ocp_type_.c_str());return;
                }
                if(feasible_start_since_==Clock::time_point{})feasible_start_since_=Clock::now();
                if(std::chrono::duration<double>(Clock::now()-feasible_start_since_).count()<0.5)return;
                ROS_INFO("Verified feasible cone start: r=[%.4f,%.4f,%.4f] v=%.4f omega=%.5f",live(0),live(1),live(2),live.segment<3>(3).norm(),live.segment<3>(10).norm());
            }
            last_command_seq_ = p_seq;
            nh_.setParam("control_stop_reason",std::string{});
            if (platform_ == "mavros") platform::mavros::resetForNewFlight(mavros_handles_);
            if (hasState() && is_configured_ && mass_kg_ > 0.0) {
                const Eigen::VectorXd x_now = convertStateToAbsoluteFrame(getCurrentState());
                paused_hover_state_ = hover_controller::makeHoverState(
                    x_now, state_dim_, custom_make_hover_state_);
                maintain_hover_hold_ = true;
            } else {
                maintain_hover_hold_ = false;
            }
            command_paused_.store(false);
            ocp_active_.store(true);
            resetForNewCommand();
            if(OCPRegistry::getDescriptor(ocp_type_).feasible_start)nh_.setParam("ordinary_start_verified",true);
            ROS_INFO("Accepted command_seq=%d. Starting OCP=%s mode=%s n_replay=%d.",
                last_command_seq_, ocp_type_.c_str(), mode_.c_str(), n_replay_);
        }
    }

    // ── Open-loop phase ───────────────────────────────────────────────────────
    enum class OpenLoopPhase { Idle, Replay, Hold };

    static const char* openLoopPhaseName(OpenLoopPhase p) {
        switch (p) {
            case OpenLoopPhase::Replay: return "replay";
            case OpenLoopPhase::Hold:   return "hold";
            default:                    return "idle";
        }
    }

    void stopReplayTimer() {
        if (replay_timer_.isValid()) { replay_timer_.stop(); replay_timer_ = ros::Timer{}; }
    }
    void stopOpenLoopHoldTimer() {
        if (open_loop_hold_timer_.isValid()) { open_loop_hold_timer_.stop(); open_loop_hold_timer_ = ros::Timer{}; }
    }
    void setOpenLoopPhase(OpenLoopPhase p) { open_loop_phase_ = p; }

    void startOpenLoopHoldTimer() {
        if (open_loop_hold_timer_.isValid()) return;
        const double hold_s = std::max(ocp_dt_, 0.02);
        open_loop_hold_timer_ = nh_.createTimer(
            ros::Duration(hold_s), &PlannerNode::openLoopHoldTick, this);
    }

    // ── Helper name strings ───────────────────────────────────────────────────
    static const char* commandModeName(OCPDescriptor::CommandMode m) {
        return m == OCPDescriptor::CommandMode::CmdBodyRate ? "cmd_bodyrate" : "cmd_full_state";
    }



    planner_core::Platform corePlatform() const { return planner_core::Platform::Mavros; }

    static const char* diagnosticSeverityName(planner_core::DiagnosticSeverity s) {
        switch (s) {
            case planner_core::DiagnosticSeverity::Error: return "error";
            case planner_core::DiagnosticSeverity::Warn:  return "warn";
            default: return "info";
        }
    }

    static std::array<float,4> hoverArray(const Eigen::Vector4d& h) {
        return {float(h(0)), float(h(1)), float(h(2)), float(h(3))};
    }

    // ── OCP drone odom mode ───────────────────────────────────────────────────


    // ── State / target access ─────────────────────────────────────────────────
    bool hasState() const {
        std::lock_guard<std::mutex> lock(state_monitor_.stateMutex());
        return mavros_handles_.odom_received;
    }

    bool hasFreshTargetState() const {
        if (ocp_type_.empty()) return true;
        const auto desc = OCPRegistry::getDescriptor(ocp_type_);
        if (!desc.validate_target) return true;
        const double now_sec = ros::Time::now().toSec();
        TargetSnapshot snapshot = state_monitor_.getTargetSnapshot(true, ros::Time::now());
        return desc.validate_target(snapshot, now_sec, 0.2);
    }

    TargetSnapshot getTargetSnapshot() const {
        if (ocp_type_.empty()) return TargetSnapshot{true};
        bool needs_target = OCPRegistry::getDescriptor(ocp_type_).validate_target != nullptr;
        return state_monitor_.getTargetSnapshot(needs_target, ros::Time::now());
    }

    bool isTargetValidForDescriptor(const OCPDescriptor& desc,
                                    const TargetSnapshot& snapshot) const {
        if (desc.validate_target)
            return desc.validate_target(snapshot, ros::Time::now().toSec(), 0.2);
        return snapshot.valid;
    }

    Eigen::VectorXd getCurrentState() const {
        std::lock_guard<std::mutex> lock(state_monitor_.stateMutex());
        return mavros_handles_.current;
    }

    Eigen::VectorXd convertStateToAbsoluteFrame(const Eigen::VectorXd& state) const {return state;}


    const char* currentFrameLabel() const {return ocp_type_.empty()?"absolute":planner_core::frameLabel(OCPRegistry::getDescriptor(ocp_type_).coordinates);}


    // ── Solver / MPC ──────────────────────────────────────────────────────────
    void setTerminalTarget(const Eigen::Vector3d& xyz) {
        hover_target_         = xyz;
        terminal_position_abs_= xyz;
        Eigen::VectorXd term  = Eigen::VectorXd::Zero(state_dim_);
        if (state_dim_ >= 7) { term(0)=xyz.x(); term(1)=xyz.y(); term(2)=xyz.z(); term(6)=1.0; }
        terminal_state_ = term;
        if (planner_core_configured_ && !planner_core_.isDirect()) planner_core_.setTerminalTarget(xyz);
    }

    void rebuildMpcSolver() {
        if (solver_type_ != "alipddp") return;
        planner_core::PlannerCoreConfig core_cfg;
        core_cfg.world_frame=world_frame_;
        core_cfg.ocp_type                = ocp_type_;
        core_cfg.platform                = corePlatform();
        core_cfg.solver_type             = solver_type_;
        core_cfg.ocp_dt                  = ocp_dt_;
        core_cfg.n_replay                = n_replay_;
        core_cfg.max_constraint_error    = max_constraint_error_;
        core_cfg.skip_trajectory_validation = skip_trajectory_validation_;
        core_cfg.seed_alpha_pos          = seed_alpha_pos_;
        core_cfg.seed_alpha_vel          = seed_alpha_vel_;
        core_cfg.seed_clamp_pos_cap_m    = seed_clamp_pos_cap_m_;
        core_cfg.seed_clamp_pos_per_m    = seed_clamp_pos_per_m_;
        core_cfg.seed_clamp_vel          = seed_clamp_vel_;
        core_cfg.seed_min_rel_z          = seed_min_rel_z_;
        core_cfg.enable_terminal_freeze  = enable_terminal_freeze_;
        core_cfg.terminal_freeze_enter_pos = terminal_freeze_enter_pos_;
        core_cfg.terminal_freeze_enter_vel = terminal_freeze_enter_vel_;
        core_cfg.terminal_freeze_require_vel = terminal_freeze_require_vel_;
        core_cfg.terminal_freeze_exit_pos  = terminal_freeze_exit_pos_;
        core_cfg.terminal_position_abs   = terminal_position_abs_;
        core_cfg.terminal_state          = terminal_state_;
        core_cfg.max_first_solve_age_sec   = max_first_solve_age_sec_;
        core_cfg.max_relative_position_norm = max_relative_position_norm_;
        core_cfg.max_relative_vertical_abs  = max_relative_vertical_abs_;
        core_cfg.max_relative_velocity_norm = max_relative_velocity_norm_;
        planner_core_.configure(core_cfg);
        if (planner_core_.isDirect()) {
            std::string directory; nh_.param<std::string>("log_dir", directory, "/tmp");
            planner_core_.openControlLog(directory + "/direct_mpc_controls.csv");
        }
        planner_core_configured_ = true;
    }

    void createModeTimers() {
        if (mode_ == "mpc") {
            if (solver_timer_.isValid()) solver_timer_.setPeriod(ros::Duration(0.001));
            if (mpc_replay_timer_.isValid()) mpc_replay_timer_.setPeriod(ros::Duration(planner_core_.commandPeriod()));
            if (!solver_timer_.isValid()) {
                solver_timer_ = nh_.createTimer(
                    ros::Duration(0.001), &PlannerNode::solverLoop, this);
            }
            if (!mpc_replay_timer_.isValid() && ocp_dt_ > 0) {
                mpc_replay_timer_ = nh_.createTimer(
                    ros::Duration(planner_core_.commandPeriod()), &PlannerNode::mpcReplayTick, this);
                ROS_INFO("MPC replay timer created (ocp_dt=%.3f valid=%d)",
                         ocp_dt_, mpc_replay_timer_.isValid() ? 1 : 0);
            }
        } else if (mode_ == "open_loop") {
            if (!startup_timer_.isValid()) {
                startup_timer_ = nh_.createTimer(
                    ros::Duration(0.05), &PlannerNode::openLoopStartupCheck, this);
            }
        }
    }

    void resetForNewCommand() {
        ordinary_control_started_=false;
        stopReplayTimer();
        stopOpenLoopHoldTimer();
        setOpenLoopPhase(OpenLoopPhase::Idle);
        planner_core_.reset();
        is_primed_.store(false);
        terminal_freeze_.store(false);
        stale_warning_count_ = 0;
        replay_ticks_since_solve_.store(0);
        last_accepted_solve_timestamp_ = Clock::time_point{};
        last_replan_delay_sec_ = 0.0;
    }

    // ── Command publishing ────────────────────────────────────────────────────
    void publishCommandAbsolute(const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
        std::optional<Eigen::Vector3d> acceleration;
        if(!ocp_type_.empty()&&u.size()>0&&u(0)>0) {
            const auto& descriptor=OCPRegistry::getDescriptor(ocp_type_);
            if(descriptor.world_acceleration)acceleration=descriptor.world_acceleration(s,u);
        }
        if (platform_ == "mavros") platform::mavros::publishFullStateCommand(mavros_handles_, s,acceleration);
    }

    void publishPausedHoverHoldTick() {
        bool prepare=false;nh_.param<bool>("prepare_cone_start",prepare,false);
        if(prepare && hasState() && hasFreshTargetState()) {
            auto hold=getCurrentState();const auto target=getTargetSnapshot();
            if(target.valid) {
                const auto now=Clock::now();const double h=prep_last_==Clock::time_point{}?.05:std::clamp(std::chrono::duration<double>(now-prep_last_).count(),.001,.1);
                const auto& limits=comando_parameters::defaults.constraints;
                const double acceleration_limit=.6*9.81*std::tan(limits.alpha_theta_stc*limits.theta_stc_deg*M_PI/180);
                const double jerk_limit=.5*9.81*limits.omega_stc_max;
                const double jerk=prep_last_==Clock::time_point{}?0:(target.acceleration-prep_last_accel_).head<2>().norm()/h;
                if(!prep_filter_&&(target.acceleration.head<2>().norm()>acceleration_limit||jerk>jerk_limit)) {
                    prep_filter_=true;prep_position_=hold.head<3>();prep_velocity_=hold.segment<3>(3);prep_acceleration_.setZero();
                    ROS_INFO("Cone preparation: limiting reference acceleration and jerk for high-curvature target motion.");
                }
                if(prep_filter_) {
                    Eigen::Vector3d requested=.5*(target.position-prep_position_)+(target.velocity-prep_velocity_)+target.acceleration;
                    requested.z()=0;const double norm=requested.head<2>().norm();if(norm>acceleration_limit)requested.head<2>()*=acceleration_limit/norm;
                    Eigen::Vector3d change=requested-prep_acceleration_;const double delta=change.norm();if(delta>jerk_limit*h)change*=jerk_limit*h/delta;
                    prep_acceleration_+=change;prep_position_+=h*prep_velocity_+.5*h*h*prep_acceleration_;prep_velocity_+=h*prep_acceleration_;
                    hold.head<3>()=prep_position_;hold.segment<3>(3)=prep_velocity_;
                } else {hold.head<3>()=target.position;hold.segment<3>(3)=target.velocity;}
                hold(2)=target.position.z()+1.85;hold(5)=target.velocity.z();
                prep_last_=now;prep_last_accel_=target.acceleration;
                publishCommandAbsolute(hold,Eigen::VectorXd::Zero(4));return;
            }
        }
        if(!prepare){prep_last_={};prep_filter_=false;}

        if (command_mode_ == OCPDescriptor::CommandMode::CmdBodyRate) {
            // MAVROS keepalive handled by heartbeat_timer in mavros.hpp
            return;
        }


        const Eigen::VectorXd x_now_abs = convertStateToAbsoluteFrame(getCurrentState());
        hover_controller::doHoverHoldTick(
            is_configured_, mass_kg_, state_dim_, custom_make_hover_state_,
            maintain_hover_hold_, hasState(), x_now_abs, paused_hover_state_,
            [this](const auto& x, const auto& u) { publishCommandAbsolute(x, u); });
        if (ocp_active_.load()) {
            planner_logging::SolverEventLogRow row;
            row.solve_num = -1; row.event = "stale"; row.reason = "paused_hover_hold_tick"; row.coord_mode = "absolute";
            logger_.logSolverEvent(row);
        }
    }

    void holdHoverAndPause(const std::string& reason) {

        hover_controller::enterHoverHold(
            reason, is_configured_, mass_kg_, state_dim_, custom_make_hover_state_,
            hasState(), convertStateToAbsoluteFrame(getCurrentState()),
            command_paused_, maintain_hover_hold_, paused_hover_state_,
            [this](const auto& x, const auto& u) { publishCommandAbsolute(x, u); },
            [this](const char* r) {
                ROS_INFO("Command finished (%s). Holding hover and waiting for next command_seq.", r);
            },
            [this]() { resetForNewCommand(); ocp_active_.store(false); });
        nh_.setParam("control_stop_reason",reason);
    }

    void publishCommand(const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
        const auto& desc = OCPRegistry::getDescriptor(ocp_type_);
        const auto cmd = planner_core::makePlannerCommand(corePlatform(), desc, s, u, last_target_snapshot_);
        publishCommand(cmd);
    }

    // Touchdown → stop: on terminal freeze with land_action set, hand control
    // to PX4 (AUTO.LAND or disarm) instead of freezing the last setpoint.
    void maybeEngageLandAction() {
        if (platform_ != "mavros" || land_action_ == "none" || land_action_.empty()) return;
        if (land_action_ != "auto_land" && land_action_ != "disarm" &&
            land_action_ != "force_disarm") {
            ROS_ERROR_THROTTLE(5.0, "Unknown land_action '%s' (use none|auto_land|disarm|force_disarm)",
                               land_action_.c_str());
            return;
        }
        platform::mavros::engageLandAction(mavros_handles_, land_action_);
        command_paused_.store(true);
        ocp_active_.store(false);
        maintain_hover_hold_ = false;
        ROS_INFO("Touchdown: OCP stopped, land action '%s' handed to PX4.", land_action_.c_str());
    }

    void publishCommand(const planner_core::PlannerCommand& cmd) {
        if(!ordinary_control_started_ && !ocp_type_.empty() && OCPRegistry::getDescriptor(ocp_type_).feasible_start) {
            ordinary_control_started_=true;nh_.setParam("prepare_cone_start",false);
            ROS_INFO("Ordinary landing control started; cone preparation handed over.");
        }

        if (cmd.kind == planner_core::CommandKind::BodyRate && platform_ == "mavros") {
            platform::mavros::publishBodyRateCommand(mavros_handles_, cmd.control);
        } else if (cmd.kind == planner_core::CommandKind::FullState && platform_ == "mavros") {
            platform::mavros::publishFullStateCommand(mavros_handles_, cmd.state,cmd.acceleration);
        }
    }

    void publishTrajectory(const std::vector<Eigen::VectorXd>& traj) {
        nav_msgs::Path path;
        path.header.stamp    = ros::Time::now();
        path.header.frame_id = world_frame_;
        for (const auto& s : traj) {
            if (s.size() < 10) continue;
            geometry_msgs::PoseStamped p;
            p.header.frame_id = world_frame_;
            p.pose.position.x = s(0); p.pose.position.y = s(1); p.pose.position.z = s(2);
            p.pose.orientation.w = s(6); p.pose.orientation.x = s(7);
            p.pose.orientation.y = s(8); p.pose.orientation.z = s(9);
            path.poses.push_back(p);
        }
        traj_pub_.publish(path);
    }

    // ── Diagnostics / logging ─────────────────────────────────────────────────
    void logCoreDiagnostics(const std::vector<planner_core::PlannerDiagnostic>& diagnostics) {
        for (const auto& d : diagnostics) {
            if (d.code == "handoff") {
                const auto val = [&](const char* k){ const auto it=d.values.find(k); return it==d.values.end()?0.0:it->second; };
                ROS_INFO("[handoff] solve %.0f->%.0f age=%.3fs latency=%.3fs state_jump=%.4f control_jump=%.4f hover_z_jump=%.4f",
                    val("previous_solve_num"),val("new_solve_num"),
                    val("active_plan_age_sec"),val("solve_latency_sec"),
                    val("state_jump_norm"),val("control_jump_norm"),val("hover_z_jump"));
                continue;
            }
            if (d.code == "solve_rejected") {
                const auto val = [&](const char* k){ const auto it=d.values.find(k); return it==d.values.end()?0.0:it->second; };
                ROS_WARN("[PlannerCore][solve_rejected] %s solve=%.1fms constraint=%.3g "
                    "max_rel_p=%.3f/%.3f max_rel_z=%.3f/%.3f max_rel_v=%.3f/%.3f",
                    d.message.c_str(), val("solve_time_ms"), val("constraint_error"),
                    val("max_relative_position_norm"), val("relative_position_limit"),
                    val("max_relative_vertical_abs"),  val("relative_vertical_limit"),
                    val("max_relative_velocity_norm"), val("relative_velocity_limit"));
                continue;
            }
            if (d.severity == planner_core::DiagnosticSeverity::Error)
                ROS_ERROR("[PlannerCore][%s] %s", d.code.c_str(), d.message.c_str());
            else if (d.severity == planner_core::DiagnosticSeverity::Warn)
                ROS_WARN("[PlannerCore][%s] %s", d.code.c_str(), d.message.c_str());
            else
                ROS_INFO("[PlannerCore][%s] %s", d.code.c_str(), d.message.c_str());
        }
    }

    bool ensureLoggingInitialized() {
        if (!logging_enabled_)  return false;
        if (logging_initialized_) return true;
        if (!is_configured_ || ocp_type_.empty() || mode_.empty()) return false;
        auto dec = OCPRegistry::getDescriptor(ocp_type_);
        logging_initialized_ = logger_.initialize(
            drone_name_, ocp_type_, mode_, solver_type_, std::string{},
            mass_kg_, state_dim_, dec.state_names, dec.control_names, dec.log_state_headers,
            command_mode_, dec.extract_actual_state_row);
        return logging_initialized_;
    }

    planner_logging::SolveLogMeta toRosLogMeta(const planner_core::PlannerSolveLogData& data,
                                               const SolverResult& solve_result) {
        planner_logging::SolveLogMeta meta;
        meta.solve_num = data.solve_num; meta.solve_time_ms = data.solve_time_ms;
        meta.solve_iters = data.solve_iters; meta.is_relative_plan = data.is_relative_plan;
        meta.ocp_dt = data.ocp_dt; meta.coord_mode = data.coord_mode;
        meta.target_snapshot_pos  = data.target_snapshot_pos;
        meta.target_snapshot_vel  = data.target_snapshot_vel;
        meta.target_snapshot_acc  = data.target_snapshot_acc;
        meta.target_snapshot_quat = data.target_snapshot_quat;
        meta.target_snapshot_omega= data.target_snapshot_omega;
        meta.target_snapshot_beta = data.target_snapshot_beta;
        meta.target_world_pos_trajectory = data.target_world_pos_trajectory;
        meta.target_world_vel_trajectory = data.target_world_vel_trajectory;
        auto desc = OCPRegistry::getDescriptor(ocp_type_);
        if (desc.prepare_log_meta) desc.prepare_log_meta(meta, solve_result.extra, runtime_cfg_);
        return meta;
    }

    static double diagnosticValue(const planner_core::PlannerDiagnostic& d, const std::string& k, double fb = std::numeric_limits<double>::quiet_NaN()) {
        const auto it = d.values.find(k); return it == d.values.end() ? fb : it->second;
    }
    static const planner_core::PlannerDiagnostic* findDiagnostic(
        const std::vector<planner_core::PlannerDiagnostic>& diags, const std::string& code) {
        for (const auto& d : diags) if (d.code == code) return &d;
        return nullptr;
    }

    void logSolverEvents(const planner_core::PlannerCoreStepResult& result) {
        if (!logging_enabled_ || !logging_initialized_) return;
        auto coord_mode = [&]()->std::string{ return result.solve_log.coord_mode.empty() ? currentFrameLabel() : result.solve_log.coord_mode; };
        if (result.solve_accepted) {
            planner_logging::SolverEventLogRow row;
            row.solve_num = result.solve_num; row.event = "accepted"; row.coord_mode = coord_mode();
            row.x0_source = result.x0_source; row.solve_time_ms = result.solve_result.solve_time_ms;
            row.solve_iters = result.solve_result.solve_iters; row.constraint_error = result.solve_result.constraint_error;
            row.active_elapsed_now_sec = result.active_elapsed_now_sec;
            row.activation_elapsed_sec = result.activation_elapsed_sec;
            row.activation_wall_time_sec = result.activation_wall_time_sec;
            row.solve_lead_sec = result.solve_lead_sec;
            row.solve_finish_late_by_sec = result.solve_finish_late_by_sec;
            row.handoff_pos_err = result.handoff_pos_err; row.handoff_vel_err = result.handoff_vel_err;
            row.replan_delay_sec = result.replan_delay_sec;
            logger_.logSolverEvent(row);
        } else if (result.solve_attempted) {
            planner_logging::SolverEventLogRow row;
            row.solve_num = result.solve_num; row.event = "rejected"; row.reason = result.rejection_reason;
            row.coord_mode = coord_mode(); row.x0_source = result.x0_source;
            row.solve_time_ms = result.solve_result.solve_time_ms;
            row.solve_iters = result.solve_result.solve_iters;
            row.constraint_error = result.solve_result.constraint_error;
            row.active_elapsed_now_sec = result.active_elapsed_now_sec;
            row.activation_elapsed_sec = result.activation_elapsed_sec;
            row.activation_wall_time_sec = result.activation_wall_time_sec;
            row.solve_lead_sec = result.solve_lead_sec;
            row.solve_finish_late_by_sec = result.solve_finish_late_by_sec;
            row.handoff_pos_err = result.handoff_pos_err; row.handoff_vel_err = result.handoff_vel_err;
            if (const auto* d = findDiagnostic(result.diagnostics, "solve_rejected")) row.extra_values = d->values;
            logger_.logSolverEvent(row);
        }
    }

    void logReplaySolverEvents(const planner_core::PlannerCoreStepResult& replay) {
        if (!logging_enabled_ || !logging_initialized_) return;
        if (replay.stale) {
            planner_logging::SolverEventLogRow row;
            row.solve_num = replay.solve_num; row.event = "stale"; row.reason = "trajectory_stale";
            row.coord_mode = currentFrameLabel();
            if (const auto* d = findDiagnostic(replay.diagnostics, "trajectory_stale")) row.extra_values = d->values;
            logger_.logSolverEvent(row);
        }
    }

    planner_core::PlannerCoreInput makeCoreInput(const Eigen::VectorXd& state,
                                                 const TargetSnapshot& target_snapshot) {
        planner_core::PlannerCoreInput input;
        input.state_frame=world_frame_;
        input.current_state       = state;
        input.target_snapshot     = target_snapshot;
        input.now                 = Clock::now();
        input.absolute_time_sec        = ros::Time::now().toSec();
        return input;
    }

    // ── MPC solver loop ───────────────────────────────────────────────────────
    void solverLoop(const ros::TimerEvent&) {
        std::shared_lock<std::shared_mutex> session(session_mutex_);
        if (mode_ != "mpc" || command_paused_.load()) return;
        if (!hasState()) {
            ROS_WARN_THROTTLE(2.0, "Waiting for state... platform=%s relative_mode=%s odom_topic='%s'",
                platform_.c_str(), "false", drone_odom_topic_.c_str());
            return;
        }
        if (!hasFreshTargetState()) {
            ROS_WARN_THROTTLE(2.0, "Waiting for fresh target state...");
            return;
        }

        Eigen::VectorXd x0_abs = getCurrentState();
        TargetSnapshot target_snapshot = getTargetSnapshot();

        if (!is_flying_) {
            is_flying_ = true;
            ROS_INFO("State received - starting RH MPC (%s)", solver_type_.c_str());
            // MAVROS arming is observed via /mavros/state (heartbeat in mavros.hpp).
            ensureLoggingInitialized();
        }

        planner_core::PlannerCoreStepResult core_result;
        {
            std::lock_guard<std::mutex> slk(solver_mutex_);
            core_result = planner_core_.trySolve(makeCoreInput(x0_abs, target_snapshot));
        }
        if (!core_result.diagnostics.empty() &&
            core_result.rejection_reason != "waiting_for_handoff_time" &&
            core_result.rejection_reason != "waiting_for_pending_handoff" &&
            core_result.rejection_reason != "terminal_freeze_active")
            logCoreDiagnostics(core_result.diagnostics);
        logSolverEvents(core_result);
        if (!core_result.solve_accepted) {
            if (core_result.terminal_freeze_engaged) {
                terminal_freeze_.store(true);
                maybeEngageLandAction();
            }
            if (core_result.terminal_freeze_released) terminal_freeze_.store(false);
            return;
        }

        stale_warning_count_ = 0;
        replay_ticks_since_solve_.store(0);
        last_replan_delay_sec_ = core_result.replan_delay_sec;

        ROS_INFO("[%s %d] %.1fms iters=%d x0=[%.3f,%.3f,%.3f] source=%s",
            planner_core_.isDirect() ? "Direct MPC" : "RH", core_result.solve_num, core_result.solve_result.solve_time_ms,
            core_result.solve_result.solve_iters,
            core_result.solve_result.state_trajectory.front().size()>0 ? core_result.solve_result.state_trajectory.front()(0):0.0,
            core_result.solve_result.state_trajectory.front().size()>1 ? core_result.solve_result.state_trajectory.front()(1):0.0,
            core_result.solve_result.state_trajectory.front().size()>2 ? core_result.solve_result.state_trajectory.front()(2):0.0,
            core_result.x0_source.c_str());

        is_primed_.store(true);
        terminal_freeze_.store(false);
        solve_count_ = planner_core_.solveCount();
        publishTrajectory(core_result.path.states);

        if (logging_enabled_ && logging_initialized_) {
            auto meta = toRosLogMeta(core_result.solve_log, core_result.solve_result);
            logger_.logSolveTrajectory(core_result.solve_result.state_trajectory,
                                       core_result.solve_result.control_trajectory, meta);
        }
    }

    // ── MPC replay tick ───────────────────────────────────────────────────────
    void mpcReplayTick(const ros::TimerEvent&) {
        std::shared_lock<std::shared_mutex> session(session_mutex_);
        if (mode_ != "mpc") return;
        if (command_paused_.load()) { publishPausedHoverHoldTick(); return; }
        if (!is_primed_.load()) {
            // Keep the moving startup reference current while DDP solves.
            // Re-streaming one frozen position with a nonzero target velocity
            // creates a position/velocity inconsistency in PX4's cascade.
            if(!ordinary_control_started_)publishPausedHoverHoldTick();
            return;
        }

        last_target_snapshot_ = getTargetSnapshot();
        planner_core::PlannerCoreStepResult replay;
        if (planner_core_.isDirect()) {
            bool fresh;
            { std::lock_guard<std::mutex> lock(state_monitor_.stateMutex());
              fresh = std::chrono::duration<double>(Clock::now()-mavros_handles_.last_odom).count() < 0.2; }
            if (!fresh || !hasFreshTargetState()) { holdHoverAndPause("direct MPC stale state"); return; }
            replay = planner_core_.sample(makeCoreInput(getCurrentState(),last_target_snapshot_));
            if (replay.stale) { holdHoverAndPause("direct MPC expired policy"); return; }
        } else replay = planner_core_.sample(makeCoreInput(getCurrentState(),last_target_snapshot_));
        if (!replay.has_replay_sample) return;
        logCoreDiagnostics(replay.diagnostics);
        logReplaySolverEvents(replay);
        replay_ticks_since_solve_.fetch_add(1);

        if (replay.stale) {
            ++stale_warning_count_;
            ROS_WARN("Trajectory stale, clamping to terminal");
            if (stale_warning_count_ >= 3) { holdHoverAndPause("trajectory stale 3x"); return; }
        } else {
            stale_warning_count_ = 0;
        }
        publishCommand(replay.command);
    }

    // ── Open-loop startup / replay / hold ─────────────────────────────────────
    void transitionOpenLoopToHold(const std::string& reason) {
        stopReplayTimer();
        command_paused_.store(true);
        maintain_hover_hold_ = true;
        setOpenLoopPhase(OpenLoopPhase::Hold);
        startOpenLoopHoldTimer();
        ROS_INFO("[OpenLoop] Replay complete (%s). Transitioning to hold.", reason.c_str());
        publishPausedHoverHoldTick();
    }

    void abortOpenLoopReplay(const std::string& reason) {
        stopReplayTimer();
        setOpenLoopPhase(OpenLoopPhase::Idle);
        command_paused_.store(true);
        maintain_hover_hold_ = false;
        resetForNewCommand();
        ocp_active_.store(false);
        ROS_WARN("[OpenLoop] Replay aborted: %s", reason.c_str());
    }

    void openLoopStartupCheck(const ros::TimerEvent&) {
        std::shared_lock<std::shared_mutex> session(session_mutex_);
        if (mode_ != "open_loop") return;
        if (command_paused_.load()) {
            if (open_loop_phase_ == OpenLoopPhase::Hold) publishPausedHoverHoldTick();
            return;
        }
        if (!hasState()) {
            ROS_WARN_THROTTLE(2.0, "[OpenLoop] Waiting for state...");
            return;
        }
        auto desc = OCPRegistry::getDescriptor(ocp_type_);
        TargetSnapshot target_snapshot = getTargetSnapshot();
        if ((desc.validate_target) &&
            !isTargetValidForDescriptor(desc, target_snapshot)) {
            ROS_WARN_THROTTLE(2.0, "[OpenLoop] IDLE — waiting for valid target state...");
            publishPausedHoverHoldTick();
            return;
        }

        startup_timer_.stop();

        ROS_INFO("[OpenLoop] Solving...");
        planner_core::PlannerCoreStepResult core_out;
        {
            std::lock_guard<std::mutex> slk(solver_mutex_);
            core_out = planner_core_.solveOpenLoop(
                makeCoreInput(getCurrentState(), target_snapshot));
        }
        logCoreDiagnostics(core_out.diagnostics);
        if (!core_out.solve_accepted) {
            ROS_ERROR("[OpenLoop] FAILED (%s)", core_out.rejection_reason.c_str());
            return;
        }
        const SolverResult& result = core_out.solve_result;

        ol_ref_X_ = result.state_trajectory;
        ol_ref_U_ = result.control_trajectory;

        if (ensureLoggingInitialized()) {
            auto meta = toRosLogMeta(core_out.solve_log, result);
            logger_.logSolveTrajectory(ol_ref_X_, ol_ref_U_, meta);
        }
        publishTrajectory(core_out.path.states);

        ol_replay_step_ = 0;
        ol_replay_start_time_ = Clock::now();
        setOpenLoopPhase(OpenLoopPhase::Replay);
        stopOpenLoopHoldTimer();
        stopReplayTimer();
        replay_timer_ = nh_.createTimer(
            ros::Duration(ocp_dt_), &PlannerNode::openLoopReplayTick, this);
        ROS_INFO("[OpenLoop] Replaying %zu steps", ol_ref_X_.size());
    }

    void openLoopReplayTick(const ros::TimerEvent&) {
        std::shared_lock<std::shared_mutex> session(session_mutex_);
        if (open_loop_phase_ != OpenLoopPhase::Replay || command_paused_.load()) return;
        const int N = static_cast<int>(ol_ref_X_.size()) - 1;
        if (ol_replay_step_ >= N) {
            if (hasState() && is_configured_) {
                paused_hover_state_ = hover_controller::makeHoverState(
                    convertStateToAbsoluteFrame(getCurrentState()), state_dim_, custom_make_hover_state_);
                maintain_hover_hold_ = true;
            }
            transitionOpenLoopToHold("nominal horizon end");
            return;
        }
        const int step = ol_replay_step_;
        const Eigen::VectorXd& x_cmd = ol_ref_X_[step];
        const Eigen::VectorXd u_cmd = (step < static_cast<int>(ol_ref_U_.size()))
            ? ol_ref_U_[step] : Eigen::VectorXd::Zero(4);

        if (open_loop_abort_on_divergence_ && x_cmd.size() >= 6 && hasState()) {
            const Eigen::VectorXd x_act = convertStateToAbsoluteFrame(getCurrentState());
            if (x_act.size() >= 6) {
                const double ze  = std::abs(x_act(2) - x_cmd(2));
                const double vze = std::abs(x_act(5) - x_cmd(5));
                if (ze > open_loop_abort_max_z_error_m_ || vze > open_loop_abort_max_vz_error_mps_) {
                    std::ostringstream ss;
                    ss << "divergence at step=" << step << " z_err=" << ze << " vz_err=" << vze;
                    abortOpenLoopReplay(ss.str());
                    return;
                }
            }
        }
        last_target_snapshot_ = getTargetSnapshot();
        publishCommand(x_cmd, u_cmd);
        ++ol_replay_step_;
    }

    void openLoopHoldTick(const ros::TimerEvent&) {
        std::shared_lock<std::shared_mutex> session(session_mutex_);
        if (mode_ != "open_loop" || open_loop_phase_ != OpenLoopPhase::Hold) return;
        publishPausedHoverHoldTick();
    }

    // ── Member variables ──────────────────────────────────────────────────────
    ros::NodeHandle& nh_;

    std::string ocp_type_, platform_, solver_type_, mode_, drone_name_;
    std::string land_action_ = "none";  // none | auto_land | disarm (per-profile rosparam)
    Clock::time_point prep_last_{};
    bool prep_filter_=false;
    Eigen::Vector3d prep_position_=Eigen::Vector3d::Zero(),prep_velocity_=Eigen::Vector3d::Zero(),prep_acceleration_=Eigen::Vector3d::Zero(),prep_last_accel_=Eigen::Vector3d::Zero();
    bool ordinary_control_started_=false;
    Clock::time_point feasible_start_since_{};
    std::shared_ptr<platform::FrameNormalizer> frames_;
    std::string world_frame_="world";
    std::string drone_odom_topic_;
    std::string target_odom_topic_         = "/target/odom";
    std::string target_accel_topic_        = "/target/accel";

    double seed_alpha_pos_            = 0.70;
    double seed_alpha_vel_            = 0.0;
    double seed_clamp_pos_cap_m_      = 0.10;
    double seed_clamp_pos_per_m_      = 0.10;
    double seed_clamp_vel_            = 0.30;
    double seed_min_rel_z_            = 0.60;

    bool enable_terminal_freeze_      = true;
    double terminal_freeze_enter_pos_ = 0.20;
    double terminal_freeze_enter_vel_ = 0.10;
    bool terminal_freeze_require_vel_ = false;
    double terminal_freeze_exit_pos_  = 0.20;

    bool logging_enabled_;
    double ocp_dt_ = 0.05;
    PlannerRuntimeConfig runtime_cfg_;
    double mass_kg_ = 0.027;
    bool open_loop_abort_on_divergence_    = false;
    double open_loop_abort_max_z_error_m_  = 0.50;
    double open_loop_abort_max_vz_error_mps_ = 1.00;
    int    n_replay_                       = 4;
    double max_constraint_error_           = 1.0;
    bool   skip_trajectory_validation_     = false;
    double max_first_solve_age_sec_        = 1.0;
    double max_relative_position_norm_     = 10.0;
    double max_relative_vertical_abs_      = 5.0;
    double max_relative_velocity_norm_     = 8.0;
    OCPDescriptor::CommandMode command_mode_ = OCPDescriptor::CommandMode::CmdFullState;
    int state_dim_   = 13;
    int control_dim_ = 4;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> custom_make_hover_state_ = nullptr;

    Eigen::Vector3d hover_target_      = Eigen::Vector3d::Zero();
    Eigen::VectorXd terminal_state_    = Eigen::VectorXd();
    double hover_thrust_d_             = 0.3;
    float  hover_thrust_param_         = 0.3f;

    platform::mavros::Handles         mavros_handles_;
    platform::target_tracker::Handles target_handles_;

    TargetSnapshot last_target_snapshot_;

    ros::Timer solver_timer_;
    ros::Timer mpc_replay_timer_;
    ros::Timer startup_timer_;
    ros::Timer replay_timer_;
    ros::Timer open_loop_hold_timer_;
    ros::Timer param_poll_timer_;

    ros::Publisher traj_pub_;

    planner_core::Controller planner_core_;
    bool planner_core_configured_ = false;
    StateMonitor state_monitor_;

    bool is_flying_    = false;
    int  solve_count_  = 0;
    std::atomic<int>  replay_ticks_since_solve_{0};
    std::atomic<bool> is_primed_{false};
    std::atomic<bool> terminal_freeze_{false};
    Eigen::Vector3d terminal_position_abs_ = Eigen::Vector3d::Zero();
    Clock::time_point last_accepted_solve_timestamp_{};
    double last_replan_delay_sec_ = 0.0;

    std::vector<Eigen::VectorXd> ol_ref_X_, ol_ref_U_;
    int ol_replay_step_ = 0;
    Clock::time_point ol_replay_start_time_;
    OpenLoopPhase open_loop_phase_ = OpenLoopPhase::Idle;

    bool logging_initialized_ = false;
    planner_logging::CsvLogger logger_;

    std::atomic<bool> command_paused_{true};
    int  last_command_seq_ = 0;
    int  stale_warning_count_ = 0;
    bool maintain_hover_hold_ = false;
    Eigen::VectorXd paused_hover_state_ = Eigen::VectorXd();
    std::shared_mutex session_mutex_;
    std::mutex command_mutex_;
    std::mutex solver_mutex_;  // serializes solver rebuilds against in-flight solves

    std::atomic<bool> ocp_active_{false};
    bool is_configured_ = true;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "comando_planner");
    ros::NodeHandle nh("~");

    PlannerNode node(nh);

    // Two spinner threads: one for sensor callbacks, one for solver/replay timers.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
