#pragma once
// Private solver and RH/MPC runtime implementation. Tests may inspect these
// internals; ROS adapters use the public Controller interface.
#include "alipddp/alipddp.h"
#include "dynamics/quad_6dof_target_frame.h"
#include "parameters.hpp"
#include "optimal_control_problem.h"
#include "planner_core/adapters.hpp"
#include "planner_core/controller.hpp"
#include "planner_core/types.hpp"
#include "trajectory_replayer.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <any>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Evaluate the returned primal directly, independently of IPM slacks and
// termination labels. This certifies the transcribed OCP, including its CTCS
// epsilon allowance; it is not a certificate of unrelaxed physical STCs.
struct PrimalAudit {
    double constraints=0, dynamics=0;
    bool feasible(double tolerance) const {
        return std::isfinite(constraints)&&std::isfinite(dynamics)&&
               constraints<=tolerance&&dynamics<=tolerance;
    }
};
inline PrimalAudit auditPrimal(const OptimalControlProblem<double>& problem,
    const std::vector<Eigen::VectorXd>& x,const std::vector<Eigen::VectorXd>& u,
    const Param& parameters) {
    const double invalid=std::numeric_limits<double>::infinity();
    if(x.size()!=problem.getHorizon()+1||u.size()!=problem.getHorizon())return {invalid,invalid};
    for(const auto& v:x)if(!v.allFinite())return {invalid,invalid};
    for(const auto& v:u)if(!v.allFinite())return {invalid,invalid};
    PrimalAudit result;
    const auto residual=[](const Eigen::VectorXd& c,ConstraintType type){
        if(!c.allFinite())return std::numeric_limits<double>::infinity();
        if(c.size()==0)return 0.;
        if(type==ConstraintType::SOC)return std::max(0.,c(0)+c.tail(c.size()-1).norm());
        return type==ConstraintType::EQ?c.cwiseAbs().maxCoeff():std::max(0.,c.maxCoeff());
    };
    for(int k=0;k<problem.getHorizon();++k){
        if(x[k].size()!=problem.getInitialState(k).size()||u[k].size()!=problem.getInitialControl(k).size())return {invalid,invalid};
        auto next=problem.getDynamics(k)->f(x[k],u[k]);
        if(next.size()!=x[k+1].size()||!next.allFinite())return {invalid,invalid};
        if(parameters.is_quaternion_in_state){
            const int q=parameters.quaternion_idx;
            if(next.segment(q,4).dot(x[k+1].segment(q,4))<0)next.segment(q,4)*=-1;
        }
        result.dynamics=std::max(result.dynamics,(next-x[k+1]).lpNorm<Eigen::Infinity>());
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
            for(const auto& row:problem.getStageConstraints(k,type))
                result.constraints=std::max(result.constraints,residual(row->c(x[k],u[k]),type));
    }
    for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
        for(const auto& row:problem.getTerminalConstraints(type))
            result.constraints=std::max(result.constraints,residual(row->cT(x.back()),type));
    result.dynamics=std::max(result.dynamics,(x.front()-problem.getInitialState(0)).lpNorm<Eigen::Infinity>());
    return result;
}

class OcpSolver {
public:
    struct WarmStartSnapshot {
        std::vector<Eigen::VectorXd> prev_X;
        std::vector<Eigen::VectorXd> prev_U;
        std::vector<Eigen::MatrixXd> prev_K;
        double last_solve_ms = 0.0;
    };

    struct Config {
        std::string ocp_type = "hover";

        // Planner-provided terminal state for OCPs that use it.
        Eigen::VectorXd terminal_state;

        // Warm-start shift count (PlannerNode sets this from replay settings).
        int n_shift = 1;
        bool reuse_solver = false;

        Config() {
            terminal_state = Eigen::VectorXd::Zero(13);
            terminal_state(2) = 1.0;
            terminal_state(6) = 1.0;
        }
    };

    struct Result {
        bool                          success       = false;
        Eigen::VectorXd               next_state;
        std::vector<Eigen::VectorXd>  state_trajectory;
        std::vector<Eigen::VectorXd>  control_trajectory;
        std::vector<Eigen::MatrixXd>  feedback_gains;
        double                        solve_time_ms = 0.0;
        double                        constraint_error = 0.0;
        // Feasibility and optimal convergence are separate evidence. This
        // diagnostic does not alter delivery of the returned first control.
        double optimality_error = std::numeric_limits<double>::infinity();
        bool converged = false;
        PrimalAudit primal;
        std::string termination;
        int                           solve_iters   = 0;
        std::chrono::steady_clock::time_point solve_start_time;
        std::chrono::steady_clock::time_point solve_finish_time;
        // Temporary compatibility alias while planner call sites migrate.
        std::chrono::steady_clock::time_point solve_timestamp;
        std::any                      extra_params;
    };

    explicit OcpSolver(const Config& config = Config());
    ~OcpSolver() = default;

    Result solve(const Eigen::VectorXd& current_state,
                 const Eigen::Vector3d& target_accel = Eigen::Vector3d::Zero(),
                 const std::any& extra_params = {},
                 double t_abs = 0.0);

    void setTerminalState(const Eigen::VectorXd& terminal);

    double getOcpDt() const;
    double last_solve_ms_ = 0.0;

    /// Returns true if at least one solve has completed and prev_X_ is populated.
    bool hasPreviousTrajectory() const { return !prev_X_.empty(); }
    /// Read-only access to the last solved state trajectory (size N+1, each NX_SS-dim).
    const std::vector<Eigen::VectorXd>& getPrevX() const { return prev_X_; }
    WarmStartSnapshot snapshotWarmStart() const;
    void restoreWarmStart(const WarmStartSnapshot& snapshot);
    void keepLatestAsUnexecutedWarmStart();
    void setNextWarmStartShift(int shift) { next_warm_start_shift_ = std::max(0, shift); }

private:
    void setupProblem(const OCPCreateArgs& args);

    std::vector<Eigen::VectorXd> makeUwarm(int n_shift) const;
    std::vector<Eigen::VectorXd> makeXshifted(int n_shift) const;
    std::vector<Eigen::MatrixXd> makeKshifted(int n_shift) const;

    Config                                         config_;
    std::shared_ptr<OptimalControlProblem<double>> problem_;
    std::shared_ptr<ALIPDDP<double>>               solver_;
    Param                                          solver_params_;

    std::vector<Eigen::VectorXd> prev_X_;
    std::vector<Eigen::VectorXd> prev_U_;
    std::vector<Eigen::MatrixXd> prev_K_;
    Eigen::Vector3d              target_accel_ = Eigen::Vector3d::Zero();
    bool                         next_warm_start_unshifted_ = false;
    int                          next_warm_start_shift_ = -1;
};

namespace planner_core {
// File I/O never runs on the controller sampling thread. Memory is bounded.
class AsyncCsvWriter {
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> rows_;
    std::ofstream file_;
    std::thread worker_;
    std::atomic<bool> enabled_{false};
    bool stopping_=false;
    std::atomic<unsigned> dropped_{0};
public:
    ~AsyncCsvWriter(){stop();}
    void stop(){
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;ready_.notify_one();}
        if(worker_.joinable())worker_.join();
        enabled_=false;file_.close();
    }
    void open(const std::string& path,const std::string& header){
        stop();file_.open(path);if(!file_)return;
        file_<<header;file_.flush();
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=false;rows_.clear();dropped_=0;}
        enabled_=true;
        worker_=std::thread([this]{
            for(;;){
                std::string row;
                {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return stopping_||!rows_.empty();});
                 if(rows_.empty()&&stopping_)break;row=std::move(rows_.front());rows_.pop_front();}
                file_<<row;
            }
            file_.flush();
        });
    }
    explicit operator bool()const{return enabled_.load();}
    void write(std::string row){
        if(!enabled_)return;
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_)return;
        if(rows_.size()==2048){++dropped_;return;}
        rows_.push_back(std::move(row));ready_.notify_one();
    }
    unsigned dropped()const{return dropped_.load();}
};
}

namespace planner_core {

class PlannerCore {
    friend struct PlannerCoreTestAccess;
public:
    void configure(const PlannerCoreConfig& config) {
        config_ = config;
        desc_ = OCPRegistry::getDescriptor(config_.ocp_type);
        if (config_.ocp_dt <= 0.0) {
            config_.ocp_dt = desc_.dt;
        }
        if (config_.n_replay < 1) {
            config_.n_replay = desc_.default_n_replay;
        }
        if (config_.terminal_state.size() == 0) {
            setTerminalTarget(config_.terminal_position_abs);
        }
        rebuildSolver();
        configured_ = true;
        reset();
    }

    void reset() {
        replayer_.clear();
        {std::lock_guard<std::mutex> lock(plan_targets_mutex_);plan_targets_.clear();}
        diagnostics_.clear();
        primed_ = false;
        terminal_freeze_ = false;
        solve_count_ = 0;
        last_replan_delay_sec_ = 0.0;
        last_accepted_solve_timestamp_ = Clock::time_point{};
        solve_time_ema_sec_ = 0.0;
    }

    void setTerminalTarget(const Eigen::Vector3d& xyz) {
        config_.terminal_position_abs = xyz;
        Eigen::VectorXd terminal = Eigen::VectorXd::Zero(std::max(1, desc_.state_dim));
        if (terminal.size() >= 7) {
            terminal(0) = xyz.x();
            terminal(1) = xyz.y();
            terminal(2) = xyz.z();
            terminal(6) = 1.0;
        }
        config_.terminal_state = terminal;
        if (mpc_) {
            mpc_->setTerminalState(config_.terminal_state);
        }
    }

    PlannerCoreStepResult trySolve(const PlannerCoreInput& input) {
        PlannerCoreStepResult out;
        if (!configured_) {
            reject(out, "not_configured", "PlannerCore is not configured");
            return out;
        }
        if (config_.solver_type != "alipddp" || !mpc_) {
            reject(out, "solver_unavailable", "ALIPDDP solver is unavailable");
            return out;
        }
        const auto frame_check = validateFrameContract(input.state_frame, config_.world_frame);
        if (!frame_check.ok) {
            reject(out, "frame_contract", frame_check.reason);
            return out;
        }

        Eigen::VectorXd x0 = prepareOcpState(desc_, input.current_state, input.target_snapshot);
        HandoffSchedule handoff;
        if (!prepareHandoffSchedule(input, out, handoff)) {
            return out;
        }
        x0 = selectInitialState(x0, input.target_snapshot, handoff,
                                &out.handoff_pos_err, &out.handoff_vel_err,
                                out.x0_source, out);
        if (!handoff.first_plan && handoff.warm_start_shift > 0) {
            mpc_->setNextWarmStartShift(handoff.warm_start_shift);
        }

        if(!primed_ && desc_.feasible_start && !desc_.feasible_start(x0)) {out.rejection_reason="initial_state_infeasible";return out;}
        const bool apply_registry_transform = (desc_.coordinates == OCPDescriptor::Coordinates::TargetRelative);
        const double pos_err = apply_registry_transform && x0.size() >= 3
            ? x0.segment(0, 3).norm()
            : absolutePositionError(input.current_state);
        const double vel_err = apply_registry_transform && x0.size() >= 6
            ? x0.segment(3, 3).norm()
            : absoluteVelocityError(input.current_state);

        if (terminal_freeze_) {
            if (pos_err > config_.terminal_freeze_exit_pos) {
                terminal_freeze_ = false;
                out.terminal_freeze_released = true;
                addDiagnostic(out, DiagnosticSeverity::Info, "terminal_freeze_released",
                              "Terminal freeze released", {{"pos_err", pos_err}});
            } else {
                reject(out, "terminal_freeze_active", "Terminal freeze is active");
                return out;
            }
        }

        const bool require_freeze_velocity =
            config_.terminal_freeze_require_vel || (desc_.coordinates == OCPDescriptor::Coordinates::TargetRelative);
        const bool freeze_condition = (pos_err < config_.terminal_freeze_enter_pos) &&
            (!require_freeze_velocity || vel_err < config_.terminal_freeze_enter_vel);
        if (config_.enable_terminal_freeze && primed_ && freeze_condition) {
            terminal_freeze_ = true;
            out.terminal_freeze_engaged = true;
            reject(out, "terminal_freeze_engaged", "Terminal freeze engaged");
            addDiagnostic(out, DiagnosticSeverity::Info, "terminal_freeze_engaged",
                          "Terminal freeze engaged",
                          {{"pos_err", pos_err},
                           {"vel_err", vel_err},
                           {"require_vel", require_freeze_velocity ? 1.0 : 0.0}});
            return out;
        }

        PlannerConfig planner_config;
        planner_config.ocp_type = config_.ocp_type;
        planner_config.solver_type = config_.solver_type;
        planner_config.mode = "mpc";
        planner_config.n_replay = config_.n_replay;
        planner_config.ocp_dt = config_.ocp_dt;
        planner_config.dt = config_.ocp_dt;
        planner_config.mass_kg = desc_.default_mass_kg;
        planner_config.t_start_abs = input.absolute_time_sec;

        std::any extra;
        if (desc_.prepare_extra) {
            extra = desc_.prepare_extra(planner_config, input.absolute_time_sec, input.target_snapshot);
        }

        const Eigen::Vector3d target_accel =
            input.target_snapshot.valid ? input.target_snapshot.acceleration : Eigen::Vector3d::Zero();
        const auto warm_start_before_solve = mpc_->snapshotWarmStart();
        auto raw = mpc_->solve(x0, target_accel, extra, input.absolute_time_sec);
        SolverResult result = toSolverResult(raw, input.target_snapshot);
        if (desc_.post_process_result) {
            desc_.post_process_result(result, input.target_snapshot);
        }
        const bool keep_controls_seed = shouldKeepRejectedControlsSeed(result);
        PlannerCoreStepResult solved_out = acceptSolvedResult(input, x0, result, out);
        if (!solved_out.solve_accepted) {
            if (keep_controls_seed && solved_out.rejection_reason == "first_solve_stale") {
                mpc_->keepLatestAsUnexecutedWarmStart();
            } else {
                mpc_->restoreWarmStart(warm_start_before_solve);
            }
        }
        return solved_out;
    }

    PlannerCoreStepResult acceptSolvedResult(const PlannerCoreInput& input,
                                             const Eigen::VectorXd& x0,
                                             SolverResult result,
                                             PlannerCoreStepResult out = {}) {
        (void)input;
        out.solve_attempted = true;
        out.solve_num = solve_count_;
        const bool first_plan = !primed_;
        if (!isSolveAcceptable(result, out.rejection_reason, first_plan)) {
            const auto rel_stats = relativeTrajectoryStats(result.state_trajectory);
            addDiagnostic(out, DiagnosticSeverity::Warn, "solve_rejected", out.rejection_reason,
                          {{"constraint_error", result.constraint_error},
                           {"primal_constraint_violation", result.primal_constraint_violation},
                           {"dynamics_error", result.dynamics_error},
                           {"solve_time_ms", result.solve_time_ms},
                           {"active_elapsed_now", out.active_elapsed_now_sec},
                           {"activation_elapsed", out.activation_elapsed_sec},
                           {"activation_wall_time_sec", out.activation_wall_time_sec},
                           {"solve_lead_sec", out.solve_lead_sec},
                           {"max_relative_position_norm", rel_stats.max_position_norm},
                           {"max_relative_position_node", static_cast<double>(rel_stats.max_position_node)},
                           {"max_relative_vertical_abs", rel_stats.max_vertical_abs},
                           {"max_relative_vertical_node", static_cast<double>(rel_stats.max_vertical_node)},
                           {"max_relative_velocity_norm", rel_stats.max_velocity_norm},
                           {"max_relative_velocity_node", static_cast<double>(rel_stats.max_velocity_node)},
                           {"relative_position_limit", config_.max_relative_position_norm},
                           {"relative_vertical_limit", config_.max_relative_vertical_abs},
                           {"relative_velocity_limit", config_.max_relative_velocity_norm}});
            out.solve_result = std::move(result);
            return out;
        }

        const auto previous_plan_origin = last_accepted_solve_timestamp_;
        const double previous_replan_delay_sec = last_replan_delay_sec_;
        last_replan_delay_sec_ = replayAdvanceTime(result.state_trajectory);

        Clock::time_point plan_origin_time = result.solve_finish_time;
        if (!first_plan) {
            if (out.activation_elapsed_sec > 0.0 && previous_plan_origin != Clock::time_point{}) {
                plan_origin_time = previous_plan_origin + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(out.activation_elapsed_sec));
            } else {
                plan_origin_time = previous_plan_origin + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(previous_replan_delay_sec));
            }
        }
        const auto earliest_activation_time = plan_origin_time;
        const auto plan_target=predictTargetSnapshot(input.target_snapshot,
            std::chrono::duration<double>(plan_origin_time-input.now).count());
        {
            std::lock_guard<std::mutex> lock(plan_targets_mutex_);
            plan_targets_[solve_count_]={plan_target,plan_origin_time};
            while(plan_targets_.size()>3)plan_targets_.erase(plan_targets_.begin());
        }
        result.target_snapshot_pos=plan_target.position;
        result.target_snapshot_vel=plan_target.velocity;
        result.target_snapshot_acc=plan_target.acceleration;
        out.solve_finish_late_by_sec =
            std::chrono::duration<double>(result.solve_finish_time - plan_origin_time).count();

        TrajectoryReplayer::Propagate propagate;
        if(desc_.make_model) {
            auto model=desc_.make_model(plan_target.acceleration);
            propagate=[model](const Eigen::VectorXd& state,const Eigen::VectorXd& control,double duration) {
                return model.flow(state,control,duration,false).x;
            };
        }

        replayer_.updatePlan(result.state_trajectory,
                             result.control_trajectory,
                             result.solve_time_ms,
                             solve_count_,
                             plan_origin_time,
                             earliest_activation_time,
                             config_.ocp_dt,
                             config_.n_replay,
                             desc_.variable_dt,
                             desc_.state_dim,
                             result.feedback_gains,std::move(propagate));

        last_accepted_solve_timestamp_ = plan_origin_time;
        primed_ = true;
        updateSolveTimeEma(result.solve_time_ms);
        out.solve_accepted = true;
        out.replan_delay_sec = last_replan_delay_sec_;
        out.path.states = makeTrajectoryForPublishing(result);
        out.solve_result = result;
        out.solve_log = makeSolveLogData(result, solve_count_);
        addDiagnostic(out, DiagnosticSeverity::Info, "solve_accepted", "Solve accepted",
                      {{"solve_time_ms", result.solve_time_ms},
                       {"replan_delay_sec", last_replan_delay_sec_},
                       {"active_elapsed_now", out.active_elapsed_now_sec},
                       {"activation_elapsed", out.activation_elapsed_sec},
                       {"activation_wall_time_sec", out.activation_wall_time_sec},
                       {"solve_lead_sec", out.solve_lead_sec},
                       {"solve_finish_late_by_sec", out.solve_finish_late_by_sec},
                       {"x0_px", x0.size() > 0 ? x0(0) : 0.0},
                       {"x0_py", x0.size() > 1 ? x0(1) : 0.0},
                       {"x0_pz", x0.size() > 2 ? x0(2) : 0.0}});
        ++solve_count_;
        return out;
    }

    PlannerCoreStepResult sampleReplay(const std::chrono::steady_clock::time_point& now,
                                       const TargetSnapshot& target_snapshot) {
        PlannerCoreStepResult out;
        if (!primed_) {
            return out;
        }
        auto replay = replayer_.sample(now, config_.ocp_dt);
        if (!replay.has_plan) {
            return out;
        }
        out.has_replay_sample = true;
        out.solve_num = replay.active_solve_num;
        const auto command_target=targetForPlan(replay.active_solve_num,now,target_snapshot);
        out.command = makePlannerCommand(config_.platform, desc_, replay.x_cmd, replay.u_cmd,
                                         command_target);
        out.stale = replay.elapsed > replay.horizon_end + 0.2;
        if (out.stale) {
            addDiagnostic(out, DiagnosticSeverity::Warn, "trajectory_stale",
                          "Trajectory sample is past horizon",
                          {{"elapsed", replay.elapsed}, {"horizon_end", replay.horizon_end}});
        }
        if (auto diag = replayer_.consumeLastHandoffDiagnostic()) {
            PlannerDiagnostic d;
            d.severity = DiagnosticSeverity::Info;
            d.code = "handoff";
            d.message = "Active plan swapped to pending plan";
            d.values = {
                {"previous_solve_num", static_cast<double>(diag->previous_solve_num)},
                {"new_solve_num", static_cast<double>(diag->new_solve_num)},
                {"active_plan_age_sec", diag->active_plan_age_sec},
                {"solve_latency_sec", diag->solve_latency_sec},
                {"state_jump_norm", diag->state_jump_norm},
                {"control_jump_norm", diag->control_jump_norm},
            };
            const auto old_cmd = makePlannerCommand(config_.platform, desc_, diag->old_state_cmd,
                                                    diag->old_control_cmd, targetForPlan(diag->previous_solve_num,now,target_snapshot));
            const auto new_cmd = makePlannerCommand(config_.platform, desc_, diag->new_state_cmd,
                                                    diag->new_control_cmd, targetForPlan(diag->new_solve_num,now,target_snapshot));
            if (old_cmd.kind == CommandKind::Hover && new_cmd.kind == CommandKind::Hover) {
                d.values["hover_z_jump"] = std::abs(new_cmd.hover(2) - old_cmd.hover(2));
            }
            d.state_a = diag->old_state_cmd;
            d.state_b = diag->new_state_cmd;
            d.control_a = diag->old_control_cmd;
            d.control_b = diag->new_control_cmd;
            out.diagnostics.push_back(std::move(d));
        }
        return out;
    }

    /// One-shot open-loop solve. Runs the same descriptor plumbing as
    /// trySolve (frame contract, state transform, prepare_extra,
    /// post-processing, circle-target absolute reconstruction) but with no
    /// handoff scheduling, terminal freeze, or replayer involvement — the
    /// wrapper replays the returned solve_result.state_trajectory itself.
    /// path.states carries the world-frame trajectory for publishing.
    PlannerCoreStepResult solveOpenLoop(const PlannerCoreInput& input) {
        PlannerCoreStepResult out;
        if (!configured_) {
            reject(out, "not_configured", "PlannerCore is not configured");
            return out;
        }
        if (config_.solver_type != "alipddp" || !mpc_) {
            reject(out, "solver_unavailable", "ALIPDDP solver is unavailable");
            return out;
        }
        const auto frame_check = validateFrameContract(input.state_frame, config_.world_frame);
        if (!frame_check.ok) {
            reject(out, "frame_contract", frame_check.reason);
            return out;
        }

        const Eigen::VectorXd x0 =
            prepareOcpState(desc_, input.current_state, input.target_snapshot);
        out.x0_source = "live_open_loop";
        out.solve_attempted = true;
        out.solve_num = 0;

        PlannerConfig planner_config;
        planner_config.ocp_type = config_.ocp_type;
        planner_config.solver_type = config_.solver_type;
        planner_config.mode = "open_loop";
        planner_config.n_replay = config_.n_replay;
        planner_config.ocp_dt = config_.ocp_dt;
        planner_config.dt = config_.ocp_dt;
        planner_config.mass_kg = desc_.default_mass_kg;
        planner_config.t_start_abs = input.absolute_time_sec;

        std::any extra;
        if (desc_.prepare_extra) {
            extra = desc_.prepare_extra(planner_config, input.absolute_time_sec, input.target_snapshot);
        }

        const Eigen::Vector3d target_accel =
            input.target_snapshot.valid ? input.target_snapshot.acceleration
                                        : Eigen::Vector3d::Zero();
        auto raw = mpc_->solve(x0, target_accel, extra, input.absolute_time_sec);
        SolverResult result = toSolverResult(raw, input.target_snapshot);
        if (desc_.post_process_result) {
            desc_.post_process_result(result, input.target_snapshot);
        }

        if (!result.success || !validStructure(result)) {
            reject(out, "solve_failed", "Open-loop solve failed");
            out.solve_result = std::move(result);
            return out;
        }

        out.solve_accepted = true;
        out.path.states = makeTrajectoryForPublishing(result);
        out.solve_log = makeSolveLogData(result, 0);
        out.solve_result = std::move(result);
        return out;
    }

    std::vector<PlannerDiagnostic> consumeDiagnostics() {
        auto out = diagnostics_;
        diagnostics_.clear();
        return out;
    }

    bool isPrimed() const { return primed_; }
    int solveCount() const { return solve_count_; }
    double lastReplanDelaySec() const { return last_replan_delay_sec_; }

private:
    using Clock = std::chrono::steady_clock;

    struct HandoffSchedule {
        bool first_plan = true;
        double active_elapsed_now = 0.0;
        double activation_elapsed = 0.0;
        double solve_lead_sec = 0.0;
        int warm_start_shift = 0;
        Clock::time_point active_origin{};
        Clock::time_point activation_time{};
    };

    struct RelativeTrajectoryStats {
        double max_position_norm = 0.0;
        double max_vertical_abs = 0.0;
        double max_velocity_norm = 0.0;
        int max_position_node = -1;
        int max_vertical_node = -1;
        int max_velocity_node = -1;
    };

    void rebuildSolver() {
        if (config_.solver_type != "alipddp") {
            mpc_.reset();
            return;
        }
        OcpSolver::Config cfg;
        cfg.ocp_type = config_.ocp_type;
        cfg.terminal_state = config_.terminal_state;
        cfg.n_shift = config_.n_replay;
        mpc_ = std::make_unique<OcpSolver>(cfg);
    }

    static SolverResult toSolverResult(const OcpSolver::Result& r,
                                       const TargetSnapshot& target_snapshot) {
        SolverResult result;
        result.success = r.success;
        result.next_state = r.next_state;
        result.state_trajectory = r.state_trajectory;
        result.feedback_gains = r.feedback_gains;
        result.control_trajectory = r.control_trajectory;
        result.solve_time_ms = r.solve_time_ms;
        result.constraint_error = r.constraint_error;
        result.optimality_error = r.optimality_error;
        result.solver_converged = r.converged;
        result.primal_constraint_violation = r.primal.constraints;
        result.dynamics_error = r.primal.dynamics;
        result.solver_termination = r.termination;
        result.solve_iters = r.solve_iters;
        result.solve_start_time = r.solve_start_time;
        result.solve_finish_time = r.solve_finish_time;
        result.solve_timestamp = r.solve_timestamp;
        result.extra = r.extra_params;
        result.target_snapshot_pos = target_snapshot.position;
        result.target_snapshot_vel = target_snapshot.velocity;
        result.target_snapshot_acc = target_snapshot.acceleration;
        result.target_snapshot_quat = target_snapshot.orientation;
        result.target_snapshot_omega = target_snapshot.angular_velocity;
        result.target_snapshot_beta = target_snapshot.angular_acceleration;
        result.target_motion_source = "snapshot";
        return result;
    }

    bool prepareHandoffSchedule(const PlannerCoreInput& input,
                                PlannerCoreStepResult& out,
                                HandoffSchedule& handoff) const {
        handoff.first_plan = !primed_;
        if (!primed_) {
            out.x0_source = "live_first";
            return true;
        }

        if (replayer_.hasPendingPlan()) {
            reject(out, "waiting_for_pending_handoff", "Pending plan has not activated yet");
            return false;
        }

        handoff.active_origin = replayer_.activePlanOriginTime();
        handoff.active_elapsed_now = replayer_.activeElapsedAt(input.now);
        handoff.solve_lead_sec = solveLeadSec();
        // A short replay interval can be smaller than the solve latency
        // (notably n_replay=1 for the variable-DT landing OCP).  Scheduling
        // every successor at origin + replay_delay then leaves its origin in
        // the past; repeating that choice makes active-plan age accumulate
        // without bound and the predicted handoff x0 drifts away from the
        // vehicle.  Always choose a still-future activation.  The replay
        // delay remains the minimum amount of the active plan that must be
        // consumed, while solve_lead reserves enough wall time for this solve.
        const double earliest_activation = std::max(
            last_replan_delay_sec_,
            handoff.active_elapsed_now + handoff.solve_lead_sec);
        handoff.activation_elapsed = earliest_activation;
        // The warm start is shifted by whole trajectory nodes.  Activating an
        // interpolated fraction of a variable-DT node while seeding from a
        // different node creates an initial dynamics defect that compounds
        // across replans. Snap the handoff to the same future node boundary.
        if (!replayer_.activeNodeAtOrAfter(
                earliest_activation, config_.ocp_dt,
                &handoff.warm_start_shift, &handoff.activation_elapsed)) {
            reject(out, "handoff_beyond_active_horizon",
                   "No active-trajectory node remains after the required activation time");
            if (!out.diagnostics.empty()) {
                out.diagnostics.back().values = {
                    {"active_elapsed_now", handoff.active_elapsed_now},
                    {"earliest_activation", earliest_activation},
                    {"active_horizon_end", replayer_.activeHorizonEnd(config_.ocp_dt)},
                    {"solve_lead_sec", handoff.solve_lead_sec},
                };
            }
            return false;
        }
        handoff.activation_time =
            handoff.active_origin + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(handoff.activation_elapsed));

        out.active_elapsed_now_sec = handoff.active_elapsed_now;
        out.activation_elapsed_sec = handoff.activation_elapsed;
        out.activation_wall_time_sec =
            std::chrono::duration<double>(handoff.activation_time.time_since_epoch()).count();
        out.solve_lead_sec = handoff.solve_lead_sec;

        // Solve now. The replayer holds the result until activation_time.
        // Waiting for activation-lead here is unstable without persisting the
        // chosen boundary: every timer poll advances `active_elapsed_now`,
        // selects the following node, and chases the boundary to horizon end.
        return true;
    }

    Eigen::VectorXd selectInitialState(const Eigen::VectorXd& live_x0,
                                       const TargetSnapshot& target_snapshot,
                                       const HandoffSchedule& handoff,
                                       double* handoff_pos_err,
                                       double* handoff_vel_err,
                                       std::string& x0_source,
                                       PlannerCoreStepResult& out) const {
        if (handoff_pos_err) *handoff_pos_err = 0.0;
        if (handoff_vel_err) *handoff_vel_err = 0.0;
        x0_source = handoff.first_plan ? "live_first" : "live";
        if (handoff.first_plan || !desc_.use_predicted_handoff_state) {
            return live_x0;
        }
        TrajectoryReplayer::ReplaySample predicted;
        if (!replayer_.sampleActiveAtElapsed(handoff.activation_elapsed, config_.ocp_dt, &predicted) ||
            !predicted.has_plan ||
            predicted.x_cmd.size() < live_x0.size()) {
            x0_source = "live_no_prediction";
            addDiagnostic(out, DiagnosticSeverity::Warn, "handoff_prediction_missing",
                          "Predicted handoff sample unavailable; using live state",
                          {{"activation_elapsed", handoff.activation_elapsed}});
            return live_x0;
        }
        const double dt_to_activation =
            std::max(0.0, handoff.activation_elapsed - handoff.active_elapsed_now);
        const TargetSnapshot predicted_target = predictTargetSnapshot(target_snapshot, dt_to_activation);
        Eigen::VectorXd predicted_x0 = preparePredictedHandoffState(predicted.x_cmd,
            targetAtPlanElapsed(predicted.active_solve_num,handoff.activation_elapsed,predicted_target),
            predicted_target,live_x0.size());
        // tracking_x0 is the ACTIVE plan sampled at *now*, so live_x0 -
        // tracking_x0 is the current tracking error. Without this sample there
        // is no error to measure and none to carry into the seed.
        Eigen::VectorXd tracking_x0 = predicted_x0;
        bool have_tracking_sample = false;
        TrajectoryReplayer::ReplaySample active_now;
        if (replayer_.sampleActiveAtElapsed(handoff.active_elapsed_now, config_.ocp_dt, &active_now) &&
            active_now.has_plan &&
            active_now.x_cmd.size() >= live_x0.size()) {
            tracking_x0 = preparePredictedHandoffState(active_now.x_cmd,
                targetAtPlanElapsed(active_now.active_solve_num,handoff.active_elapsed_now,target_snapshot),
                target_snapshot,live_x0.size());
            have_tracking_sample = true;
        } else {
            addDiagnostic(out, DiagnosticSeverity::Warn, "handoff_tracking_sample_missing",
                          "Active-plan sample unavailable for tracking-error check",
                          {{"active_elapsed_now", handoff.active_elapsed_now}});
        }
        double pos_err = 0.0;
        double vel_err = 0.0;
        if (live_x0.size() >= 6 && tracking_x0.size() >= 6) {
            pos_err = (live_x0.segment(0, 3) - tracking_x0.segment(0, 3)).norm();
            vel_err = (live_x0.segment(3, 3) - tracking_x0.segment(3, 3)).norm();
        }
        if (handoff_pos_err) *handoff_pos_err = pos_err;
        if (handoff_vel_err) *handoff_vel_err = vel_err;
        if (pos_err > config_.recovery_pos_err || vel_err > config_.recovery_vel_err) {
            x0_source = "recovery_live";
            addDiagnostic(out, DiagnosticSeverity::Warn, "handoff_recovery_live",
                          "Tracking error exceeds recovery threshold; replanning from live state",
                          {{"handoff_pos_err_now", pos_err},
                           {"handoff_vel_err_now", vel_err},
                           {"activation_elapsed", handoff.activation_elapsed}});
            return live_x0;
        }
        // Carry the persistent, FEASIBLE part of the tracking error onto the
        // predicted seed so the plan responds to where the vehicle actually is.
        // Position only, clamped — see PlannerCoreConfig::seed_alpha_pos for why
        // attitude/rates are excluded and why the clamp scales with altitude.
        if (have_tracking_sample) {
            const Eigen::VectorXd seeded =
                applyMeasuredSeed(predicted_x0, live_x0, tracking_x0);
            if (!seeded.isApprox(predicted_x0)) {
                x0_source = "measured_seed";
                return seeded;
            }
        }
        x0_source = "predicted_handoff";
        return predicted_x0;
    }

    /// x0 = predicted + clamp(alpha * (live - tracking)) on the position (and
    /// optionally velocity) block. Every other element of `predicted` — the
    /// quaternion, the body rates, and any trailing cumulative-time slot — is
    /// returned untouched, which is what keeps the seed feasible for the OCP's
    /// hard constraints.
    Eigen::VectorXd applyMeasuredSeed(const Eigen::VectorXd& predicted,
                                      const Eigen::VectorXd& live,
                                      const Eigen::VectorXd& tracking) const {
        const int n = static_cast<int>(predicted.size());
        if (live.size() != n || tracking.size() != n || n < 3) {
            return predicted;
        }
        // Terminal approach: the feasible seed budget collapses with the
        // glideslope cone and the tracking error is already small, so stop
        // correcting and hand the terminal phase the plain predicted seed.
        if (predicted(2) < config_.seed_min_rel_z) {
            return predicted;
        }
        Eigen::VectorXd out = predicted;

        if (config_.seed_alpha_pos > 0.0) {
            // The feasible budget tracks the glideslope cone, which closes as
            // the vehicle descends, so the clamp is proportional to the
            // relative altitude with an absolute cap.
            const double clamp = std::min(config_.seed_clamp_pos_cap_m,
                                          config_.seed_clamp_pos_per_m *
                                              std::max(0.0, predicted(2)));
            Eigen::Vector3d c =
                config_.seed_alpha_pos * (live.head(3) - tracking.head(3));
            const double norm = c.norm();
            if (norm > clamp && norm > 1e-12) {
                c *= clamp / norm;
            }
            out.head(3) += c;
        }

        if (config_.seed_alpha_vel > 0.0 && n >= 6) {
            Eigen::Vector3d c =
                config_.seed_alpha_vel * (live.segment(3, 3) - tracking.segment(3, 3));
            const double norm = c.norm();
            if (norm > config_.seed_clamp_vel && norm > 1e-12) {
                c *= config_.seed_clamp_vel / norm;
            }
            out.segment(3, 3) += c;
        }
        return out;
    }

    Eigen::VectorXd preparePredictedHandoffState(const Eigen::VectorXd& predicted_sample,
        const TargetSnapshot& old_target,const TargetSnapshot& new_target,int live_size) const {
        auto world=reconstructWorldState(desc_,predicted_sample,old_target);
        auto relative=prepareOcpState(desc_,world,new_target);
        return relative.head(std::min(live_size,static_cast<int>(relative.size())));
    }


    static TargetSnapshot predictTargetSnapshot(const TargetSnapshot& target, double dt) {
        TargetSnapshot predicted = target;
        if (!target.valid || dt <= 0.0) {
            return predicted;
        }
        predicted.position = target.position + target.velocity * dt +
            0.5 * target.acceleration * dt * dt;
        predicted.velocity = target.velocity + target.acceleration * dt;
        predicted.angular_velocity = target.angular_velocity + target.angular_acceleration * dt;
        predicted.orientation = integrateQuaternion(target.orientation,
                                                    target.angular_velocity,
                                                    target.angular_acceleration,
                                                    dt);
        return predicted;
    }

    static Eigen::Vector4d integrateQuaternion(const Eigen::Vector4d& q_vec,
                                               const Eigen::Vector3d& omega0,
                                               const Eigen::Vector3d& beta,
                                               double dt) {
        Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
        q.normalize();
        const Eigen::Vector3d rotvec = omega0 * dt + 0.5 * beta * dt * dt;
        const double angle = rotvec.norm();
        if (angle > 1e-12) {
            const Eigen::AngleAxisd aa(angle, rotvec / angle);
            q = q * Eigen::Quaterniond(aa);
            q.normalize();
        }
        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    }

    double solveLeadSec() const {
        const double learned = (solve_time_ema_sec_ > 0.0)
            ? solve_time_ema_sec_
            : config_.initial_solve_lead_sec;
        return std::max(0.0, learned + config_.solve_lead_guard_sec);
    }

    void updateSolveTimeEma(double solve_time_ms) {
        const double solve_sec = std::max(0.0, solve_time_ms * 0.001);
        if (solve_time_ema_sec_ <= 0.0) {
            solve_time_ema_sec_ = solve_sec;
            return;
        }
        constexpr double alpha = 0.30;
        solve_time_ema_sec_ = alpha * solve_sec + (1.0 - alpha) * solve_time_ema_sec_;
    }

    double absolutePositionError(const Eigen::VectorXd& x) const {
        if (x.size() < 3) {
            return 0.0;
        }
        return (x.segment(0, 3) - config_.terminal_position_abs).norm();
    }

    static double absoluteVelocityError(const Eigen::VectorXd& x) {
        if (x.size() < 6) {
            return 0.0;
        }
        return x.segment(3, 3).norm();
    }

    double stateNodeTime(const Eigen::VectorXd& s, int index) const {
        if (s.size() > desc_.state_dim) {
            return s(desc_.state_dim);
        }
        return index * config_.ocp_dt;
    }

    double replayAdvanceTime(const std::vector<Eigen::VectorXd>& traj) const {
        if (traj.size() < 2) {
            return std::max(config_.ocp_dt, 1e-3);
        }
        const int idx = std::min(config_.n_replay, static_cast<int>(traj.size()) - 1);
        const double t_adv = stateNodeTime(traj[idx], idx);
        return std::max(t_adv, std::max(config_.ocp_dt, 1e-3));
    }

    bool validStructure(const SolverResult& r) const {
        if(r.state_trajectory.size()<2 || r.control_trajectory.size()+1!=r.state_trajectory.size()) return false;
        for(const auto& x:r.state_trajectory) if(x.size()<desc_.state_dim || !x.allFinite()) return false;
        for(const auto& u:r.control_trajectory) if(u.size()<desc_.control_dim || !u.allFinite()) return false;
        return std::isfinite(r.constraint_error);
    }

    bool isSolveAcceptable(const SolverResult& result,
                           std::string& reason,
                           bool first_plan) const {
        if (!result.success || !validStructure(result)) {
            reason = "solve_failed";
            return false;
        }
        if (!std::isfinite(result.constraint_error) || result.constraint_error > config_.max_constraint_error) {
            reason = "constraint_error";
            return false;
        }
        if (config_.skip_trajectory_validation || desc_.skip_trajectory_validation) {
            return true;
        }
        std::string bounds_detail;
        if (!validateTrajectory(result.state_trajectory, result.control_trajectory,
                                bounds_detail)) {
            reason = "physical_bounds:" + bounds_detail;
            return false;
        }
        return true;
    }

    bool shouldKeepRejectedControlsSeed(const SolverResult&) const {return false;}




    RelativeTrajectoryStats relativeTrajectoryStats(const std::vector<Eigen::VectorXd>& X) const {
        RelativeTrajectoryStats stats;
        for (int i = 0; i < static_cast<int>(X.size()); ++i) {
            const auto& x = X[i];
            if (x.size() < 6) {
                continue;
            }
            const double p_norm = x.segment(0, 3).norm();
            const double z_abs = std::abs(x(2));
            const double v_norm = x.segment(3, 3).norm();
            if (p_norm > stats.max_position_norm) {
                stats.max_position_norm = p_norm;
                stats.max_position_node = i;
            }
            if (z_abs > stats.max_vertical_abs) {
                stats.max_vertical_abs = z_abs;
                stats.max_vertical_node = i;
            }
            if (v_norm > stats.max_velocity_norm) {
                stats.max_velocity_norm = v_norm;
                stats.max_velocity_node = i;
            }
        }
        return stats;
    }

    bool validateTrajectory(const std::vector<Eigen::VectorXd>& X,
                            const std::vector<Eigen::VectorXd>& U,
                            std::string& detail) const {
        if (X.size() < 2) { detail = "trajectory_too_short"; return false; }
        for (int i = 0; i < static_cast<int>(X.size()); ++i) {
            const auto& x = X[i];
            if (x.size() < desc_.state_dim) continue;
            if (!desc_.skip_altitude_validation && x.size() >= 3 && x(2) < -0.05) {
                detail = "altitude node=" + std::to_string(i) +
                         " z=" + std::to_string(x(2)) + " floor=-0.05";
                return false;
            }
            if (x.size() >= 6 && x.segment(3, 3).norm() > 20.0) {
                detail = "velocity node=" + std::to_string(i) +
                         " |v|=" + std::to_string(x.segment(3, 3).norm()) + " limit=20";
                return false;
            }
            if (x.size() >= 13 && x.segment(10, 3).norm() > 50.0) {
                detail = "body_rate node=" + std::to_string(i) +
                         " |w|=" + std::to_string(x.segment(10, 3).norm()) + " limit=50";
                return false;
            }
        }
        for (int i = 0; i < static_cast<int>(U.size()); ++i) {
            const auto& u = U[i];
            if (u.size() > 0 && (u(0) < -0.1 || u(0) > 50.0)) {
                detail = "thrust node=" + std::to_string(i) +
                         " u0=" + std::to_string(u(0)) + " range=[-0.1,50]";
                return false;
            }
        }
        return true;
    }

    std::vector<Eigen::VectorXd> makeTrajectoryForPublishing(const SolverResult& result) const {
        auto states=result.state_trajectory;
        if(!result.is_relative_plan) return states;
        for(int i=0;i<static_cast<int>(states.size());++i) {
            const double time=stateNodeTime(states[i],i);
            TargetSnapshot target;target.valid=true;
            target.position=result.target_snapshot_pos+result.target_snapshot_vel*time+0.5*result.target_snapshot_acc*time*time;
            target.velocity=result.target_snapshot_vel+result.target_snapshot_acc*time;
            states[i]=reconstructWorldState(desc_,states[i],target);
        }
        return states;
    }


    PlannerSolveLogData makeSolveLogData(const SolverResult& result, int solve_num) const {
        PlannerSolveLogData meta;
        meta.solve_num = solve_num;
        meta.solve_time_ms = result.solve_time_ms;
        meta.solve_iters = result.solve_iters;
        meta.is_relative_plan = result.is_relative_plan;
        meta.ocp_dt = config_.ocp_dt;
        meta.target_snapshot_pos = result.target_snapshot_pos;
        meta.target_snapshot_vel = result.target_snapshot_vel;
        meta.target_snapshot_acc = result.target_snapshot_acc;
        meta.target_snapshot_quat = result.target_snapshot_quat;
        meta.target_snapshot_omega = result.target_snapshot_omega;
        meta.target_snapshot_beta = result.target_snapshot_beta;
        meta.target_world_pos_trajectory = result.target_world_pos_trajectory;
        meta.target_world_vel_trajectory = result.target_world_vel_trajectory;
        meta.coord_mode = frameLabel(desc_.coordinates);
        if (result.is_relative_plan) {
            meta.coord_mode = "relative";
        }
        return meta;
    }

    void reject(PlannerCoreStepResult& out, std::string code, std::string message) const {
        out.rejection_reason = code;
        addDiagnostic(out, DiagnosticSeverity::Warn, std::move(code), std::move(message), {});
    }

    static void addDiagnostic(PlannerCoreStepResult& out,
                              DiagnosticSeverity severity,
                              std::string code,
                              std::string message,
                              std::map<std::string, double> values) {
        PlannerDiagnostic d;
        d.severity = severity;
        d.code = std::move(code);
        d.message = std::move(message);
        d.values = std::move(values);
        out.diagnostics.push_back(std::move(d));
    }

    PlannerCoreConfig config_;
    struct PlanTarget {TargetSnapshot snapshot;Clock::time_point origin;};
    mutable std::mutex plan_targets_mutex_;
    std::map<int,PlanTarget> plan_targets_;
    TargetSnapshot targetAtPlanElapsed(int number,double elapsed,const TargetSnapshot& fallback) const {
        std::lock_guard<std::mutex> lock(plan_targets_mutex_);
        const auto found=plan_targets_.find(number);
        return found==plan_targets_.end()?fallback:predictTargetSnapshot(found->second.snapshot,elapsed);
    }
    TargetSnapshot targetForPlan(int number,Clock::time_point now,const TargetSnapshot& fallback) const {
        std::lock_guard<std::mutex> lock(plan_targets_mutex_);
        const auto found=plan_targets_.find(number);
        if(found==plan_targets_.end())return fallback;
        return predictTargetSnapshot(found->second.snapshot,
            std::chrono::duration<double>(now-found->second.origin).count());
    }
    OCPDescriptor desc_;
    bool configured_ = false;
    std::atomic<bool> primed_{false};
    bool terminal_freeze_ = false;
    int solve_count_ = 0;
    double last_replan_delay_sec_ = 0.0;
    double solve_time_ema_sec_ = 0.0;
    Clock::time_point last_accepted_solve_timestamp_{};
    TrajectoryReplayer replayer_;
    std::unique_ptr<OcpSolver> mpc_;
    std::vector<PlannerDiagnostic> diagnostics_;
};

}  // namespace planner_core

namespace planner_core {
// Align only the nominal warm seed; the new OCP root remains measured.
inline OcpSolver::WarmStartSnapshot shiftMpcSeed(
    const OcpSolver::Result& old, const OCPDescriptor::Model& old_model,
    const TargetSnapshot& before, const TargetSnapshot& after, double age) {
    OcpSolver::WarmStartSnapshot warm;
    age=std::max(0.0,age);
    for(int k=0;k<=old_model.horizon;++k){
        const double time=age+k*old_model.dt;
        const int i=std::min(static_cast<int>(time/old_model.dt),old_model.horizon);
        const int control=std::min(i,old_model.horizon-1);
        const double remainder=std::max(0.0,time-i*old_model.dt);
        auto state=old.state_trajectory[i];
        if(remainder>1e-9)state=old_model.flow(state,old.control_trajectory[control],remainder,false).x;
        const double next_time=k*old_model.dt;
        // Preserve world position/velocity under the new target forecast.
        state.head<3>()+=before.position+before.velocity*time+.5*before.acceleration*time*time-
            (after.position+after.velocity*next_time+.5*after.acceleration*next_time*next_time);
        state.segment<3>(3)+=before.velocity+before.acceleration*time-
            (after.velocity+after.acceleration*next_time);
        state(13)=next_time;
        warm.prev_X.push_back(state);
        if(k<old_model.horizon){
            const double fraction=std::min(1.0,remainder/old_model.dt);
            const int next=std::min(control+1,old_model.horizon-1);
            warm.prev_U.push_back((1-fraction)*old.control_trajectory[control]+
                                  fraction*old.control_trajectory[next]);
        }
    }
    return warm;
}
class DirectMpcController {
    using Clock=std::chrono::steady_clock;
    struct Policy {
        OcpSolver::Result result;
        Clock::time_point origin;
        Clock::time_point measured_at;
        int number=0;
        TargetSnapshot target;
    };
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::thread worker_;
    bool stop_=false,busy_=false;
    std::optional<PlannerCoreInput> request_;
    std::optional<PlannerCoreStepResult> completed_;
    std::optional<Policy> policy_,pending_;
    PlannerCoreConfig config_;
    OCPDescriptor desc_;
    Clock::time_point last_apply_{};
    Eigen::VectorXd last_u_=(Eigen::Vector4d()<<9.81,0,0,0).finished();
    int count_=0;
    AsyncCsvWriter controls_;
    static double seconds(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double>(a-b).count();}
    Eigen::VectorXd relative(const Eigen::VectorXd& x,const TargetSnapshot& t) const {
        Eigen::VectorXd state=x;
        state.head(13)=prepareOcpState(desc_,x.head(13),t);
        return state;
    }

    void run() {
        OcpSolver::Config cfg;cfg.ocp_type=config_.ocp_type;cfg.n_shift=0;cfg.reuse_solver=true;OcpSolver solver(cfg);
        for(;;) {
            PlannerCoreInput input;Eigen::VectorXd applied;int number;std::optional<Policy> prior;
            {
                std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[&]{return stop_||request_.has_value();});
                if(stop_)return;input=*request_;request_.reset();applied=last_u_;number=count_;prior=policy_;
            }
            try {
            const auto origin=input.now;
            auto dyn=desc_.make_model(input.target_snapshot.acceleration);
            const auto physical=relative(input.current_state,input.target_snapshot);
            Eigen::VectorXd x;
            if(dyn.initial_state)x=dyn.initial_state(physical,applied);
            else {x=Eigen::VectorXd::Zero(dyn.nx);x.head(13)=physical;x.tail(4)=applied;}

            // Every DDP problem starts from measured physical state. The
            // last published input is command memory for the OCP slew cost;
            // it is not a fictitious control used to project the initial state.
            x(13)=0;
            if(prior){
                const auto old_model=desc_.make_model(prior->target.acceleration);
                // The previous trajectory starts at its measurement timestamp,
                // before its solve completed and u0 began publication.
                const double age=seconds(origin,prior->measured_at);
                auto warm=shiftMpcSeed(prior->result,old_model,prior->target,input.target_snapshot,age);
                // Factory fixes the root, clock and command memory, leaving
                // FDDP to close the remaining measured/predicted state gaps.
                solver.restoreWarmStart(warm);solver.setNextWarmStartShift(0);
            }
            auto snapshot=solver.snapshotWarmStart();
            auto raw=solver.solve(x,input.target_snapshot.acceleration,{},input.absolute_time_sec);
            PlannerCoreStepResult out;out.solve_attempted=true;out.solve_num=number;out.x0_source="measured";
            out.solve_result.success=raw.success;out.solve_result.state_trajectory=raw.state_trajectory;
            out.solve_result.control_trajectory=raw.control_trajectory;out.solve_result.feedback_gains=raw.feedback_gains;
            out.solve_result.solve_time_ms=raw.solve_time_ms;out.solve_result.solve_iters=raw.solve_iters;
            out.solve_result.optimality_error=raw.optimality_error;out.solve_result.solver_converged=raw.converged;out.solve_result.constraint_error=raw.constraint_error;out.solve_result.primal_constraint_violation=raw.primal.constraints;out.solve_result.dynamics_error=raw.primal.dynamics;out.solve_result.solver_termination=raw.termination;out.solve_result.solve_start_time=raw.solve_start_time;out.solve_result.solve_finish_time=raw.solve_finish_time;
            out.solve_result.is_relative_plan=true;out.solve_result.target_snapshot_pos=input.target_snapshot.position;
            out.solve_result.target_snapshot_vel=input.target_snapshot.velocity;out.solve_result.target_snapshot_acc=input.target_snapshot.acceleration;
            bool valid=raw.success&&std::isfinite(raw.constraint_error)&&raw.constraint_error<=config_.max_constraint_error&&
                raw.state_trajectory.size()==dyn.horizon+1&&raw.control_trajectory.size()==dyn.horizon&&raw.feedback_gains.size()==dyn.horizon;
            for(const auto& v:raw.state_trajectory)valid=valid&&v.size()==dyn.nx&&v.allFinite()&&v.head(3).norm()<100&&v.segment(3,3).norm()<20;
            for(const auto& v:raw.control_trajectory)valid=valid&&v.size()==4&&v.allFinite();
            for(const auto& k:raw.feedback_gains)valid=valid&&k.rows()==4&&k.cols()==dyn.nx&&k.allFinite();
            valid=valid&&seconds(Clock::now(),origin)<0.5;
            out.solve_accepted=valid;out.rejection_reason=valid ? "" : (!raw.success ? "direct_mpc_primal_infeasible" : "direct_mpc_invalid_or_stale");
            out.solve_log.ocp_dt=dyn.dt;out.solve_log.coord_mode="relative";
            out.solve_log.solve_num=number;out.solve_log.is_relative_plan=true;
            out.solve_log.target_snapshot_pos=input.target_snapshot.position;out.solve_log.target_snapshot_vel=input.target_snapshot.velocity;out.solve_log.target_snapshot_acc=input.target_snapshot.acceleration;
            for(int i=0;i<static_cast<int>(raw.state_trajectory.size());++i){auto world=raw.state_trajectory[i];double t=i*dyn.dt;
                TargetSnapshot target=input.target_snapshot;target.position+=target.velocity*t+0.5*target.acceleration*t*t;target.velocity+=target.acceleration*t;world=reconstructWorldState(desc_,world,target);
                out.path.states.push_back(world.head(13));}
            if(!valid)solver.restoreWarmStart(snapshot);
            {
                std::lock_guard<std::mutex> lock(mutex_);if(stop_)return;
                if(valid){pending_=Policy{raw,Clock::now(),input.now,number,input.target_snapshot};++count_;}
                completed_=std::move(out);busy_=false;
            }
            } catch(const std::exception& e) {
                std::lock_guard<std::mutex> lock(mutex_);
                PlannerCoreStepResult out;out.solve_attempted=true;out.rejection_reason=std::string("direct_mpc_exception: ")+e.what();
                completed_=std::move(out);busy_=false;
            }
        }
    }
public:
    ~DirectMpcController(){stop();}
    void stop(){ {std::lock_guard<std::mutex> lock(mutex_);stop_=true;ready_.notify_all();} if(worker_.joinable())worker_.join(); }
    void configure(const PlannerCoreConfig& c){stop();std::lock_guard<std::mutex> lock(mutex_);config_=c;desc_=OCPRegistry::getDescriptor(c.ocp_type);if(!desc_.make_model)throw std::runtime_error("OCP lacks MPC model");stop_=false;busy_=false;
        request_.reset();completed_.reset();policy_.reset();pending_.reset();last_apply_={};count_=0;last_u_=desc_.make_model(Eigen::Vector3d::Zero()).hover;
        worker_=std::thread([this]{run();});}
    void reset(){PlannerCoreConfig c; {std::lock_guard<std::mutex> lock(mutex_);c=config_;} configure(c);}
    void openControlLog(const std::string& path){controls_.open(path,"time,policy,age,nom_thrust,nom_roll,nom_pitch,nom_yaw,fb_thrust,fb_roll,fb_pitch,fb_yaw,thrust,roll,pitch,yaw,saturation,jump,feedback_limited,slew_limited,pos_error,vel_error\n");}
    PlannerCoreStepResult trySolve(const PlannerCoreInput& input){
        std::lock_guard<std::mutex> lock(mutex_);PlannerCoreStepResult out;
        if(completed_){out=std::move(*completed_);completed_.reset();}
        if(!validateFrameContract(input.state_frame,config_.world_frame).ok){out.rejection_reason="frame_contract";return out;}
        if(input.current_state.size()<13||!input.current_state.allFinite()||!input.target_snapshot.valid||!input.target_snapshot.position.allFinite()||!input.target_snapshot.velocity.allFinite()||!input.target_snapshot.acceleration.allFinite()){out.rejection_reason="direct_mpc_invalid_state";return out;}
        auto rel=relative(input.current_state,input.target_snapshot);
        if(!policy_&&desc_.feasible_start&&!desc_.feasible_start(rel)){out.rejection_reason="initial_state_infeasible";return out;}
        if(policy_&&config_.enable_terminal_freeze&&rel.head(3).norm()<config_.terminal_freeze_enter_pos&&rel.segment(3,3).norm()<config_.terminal_freeze_enter_vel){out.terminal_freeze_engaged=true;out.solve_accepted=false;out.rejection_reason="terminal_freeze_engaged";return out;}
        // Consume each completed first input before requesting the next
        // measured-state solve. Publication holds this input, never the tail
        // of the optimized trajectory, while the worker solves again.
        if(!busy_&&!pending_){
            request_=input;busy_=true;ready_.notify_one();}
        return out;
    }
    PlannerCoreStepResult sample(Clock::time_point now,const Eigen::VectorXd& measured,const TargetSnapshot& target){
        std::lock_guard<std::mutex> lock(mutex_);PlannerCoreStepResult out;
        if(pending_&&now>=pending_->origin){policy_=std::move(pending_);pending_.reset();}
        if(!policy_)return out;out.has_replay_sample=true;out.solve_num=policy_->number;
        auto dyn=desc_.make_model(policy_->target.acceleration);
        const double age=seconds(now,policy_->origin);
        if(age<0||age>=std::min(dyn.horizon*dyn.dt,comando_parameters::defaults.mpc.max_policy_age)||measured.size()<13||!measured.allFinite()||!target.valid||!target.position.allFinite()||!target.velocity.allFinite()){out.stale=true;return out;}
        const auto& r=policy_->result;Eigen::VectorXd nominal=r.control_trajectory.front();

        Eigen::VectorXd reference=dyn.flow(r.state_trajectory.front(),nominal,std::min(age,dyn.dt),false).x;
        reference.tail(4)=r.state_trajectory.front().tail(4);
        Eigen::VectorXd live=reference;live.head(13)=relative(measured,target).head(13);
        if(live.segment<4>(6).dot(reference.segment<4>(6))<0)live.segment<4>(6)*=-1;
        Eigen::VectorXd delta=live-reference;
        const Eigen::VectorXd feedback=Eigen::VectorXd::Zero(4);
        // Only u[0] is applied. Repeated publication holds it until a new
        // measured-state DDP solution supplies the next first input.
        const Eigen::VectorXd u=nominal;
        const double jump=(u-last_u_).norm();last_u_=u;last_apply_=now;
        out.command.kind=CommandKind::BodyRate;out.command.control=u;out.command.state=reference.head(13);
        if(controls_){std::ostringstream trace;trace<<std::setprecision(17);trace<<std::chrono::duration<double>(now.time_since_epoch()).count()<<','<<policy_->number<<','<<age;
            for(const auto& v:{nominal,feedback,u})for(int j=0;j<4;++j)trace<<','<<v(j);trace<<','<<0<<','<<jump<<','<<0<<','<<0<<','<<delta.head(3).norm()<<','<<delta.segment(3,3).norm()<<'\n';controls_.write(trace.str());}
        return out;
    }
};
}
