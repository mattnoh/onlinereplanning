#pragma once
#include "controller_internal.hpp"

namespace planner_core {
// Explicit simulation-time measured-root RH. The caller freezes time while solve()
// blocks, then applies exactly K scheduled intervals before the next solve.
// This is not an asynchronous computation-delay compensation mechanism.
class SynchronousMpcController {
    PlannerCoreConfig config_;
    std::shared_ptr<const OCPDescriptor> descriptor_;
    std::unique_ptr<OcpSolver> solver_;
    OCPDescriptor::Model model_;
    std::optional<OcpSolver::Result> accepted_;
    TargetSnapshot target_;
    Eigen::VectorXd applied_;
    double origin_=0.;
    double prefix_duration_=0.;
    std::vector<double> node_times_;
    int intervals_=0, count_=0;
    static TargetSnapshot predictTargetSnapshot(const TargetSnapshot& input,double dt) {
        auto target=input;
        target.position+=input.velocity*dt+.5*input.acceleration*dt*dt;
        target.velocity+=input.acceleration*dt;
        return target;
    }
    OcpSolver::WarmStartSnapshot shiftVariablePrefix(const TargetSnapshot& after) const {
        // Execute whole stages: retain their duration controls without fractional
        // resampling. Extend the fixed node count with the last input/duration.
        OcpSolver::WarmStartSnapshot guess;
        const int N=config_.preserve_remaining_trajectory?model_.horizon-intervals_:model_.horizon;
        auto tail=accepted_->state_trajectory.back();
        double tail_time=node_times_.back();
        const double last_duration=node_times_.back()-node_times_[model_.horizon-1];
        for(int k=0;k<=N;++k){
            const int source=intervals_+k;
            Eigen::VectorXd state;
            double old_time;
            if(source<=model_.horizon){state=accepted_->state_trajectory[source];old_time=node_times_[source];}
            else {tail=model_.flow(tail,accepted_->control_trajectory.back(),last_duration,false).x;
                tail_time+=last_duration;state=tail;old_time=tail_time;}
            const double new_time=old_time-prefix_duration_;
            state.head<3>()+=target_.position+target_.velocity*old_time+.5*target_.acceleration*old_time*old_time-
                (after.position+after.velocity*new_time+.5*after.acceleration*new_time*new_time);
            state.segment<3>(3)+=target_.velocity+target_.acceleration*old_time-
                (after.velocity+after.acceleration*new_time);
            // The original OCP factory rebases its per-solve clock using X[0].
            guess.prev_X.push_back(state);
            if(k<N)guess.prev_U.push_back(accepted_->control_trajectory[std::min(source,model_.horizon-1)]);
        }
        return guess;
    }
public:
    void configure(const PlannerCoreConfig& config,
                   std::shared_ptr<const OCPDescriptor> descriptor={}) {
        config_=config;
        descriptor_=descriptor?std::move(descriptor):
            std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor(config.ocp_type));
        if(!descriptor_->make_model)
            throw std::invalid_argument("synchronous RH requires a model adapter");
        model_=descriptor_->make_model(Eigen::Vector3d::Zero());
        if(descriptor_->variable_dt&&!model_.stage_duration)
            throw std::invalid_argument("variable-grid synchronous RH requires stage durations");
        intervals_=config.n_replay>0?config.n_replay:descriptor_->default_n_replay;
        if(intervals_<1||intervals_>model_.horizon||!(model_.dt>0.))
            throw std::invalid_argument("invalid synchronous MPC execution interval count");
        OcpSolver::Config cfg;cfg.ocp_type=config.ocp_type;cfg.descriptor=descriptor_;
        cfg.n_shift=intervals_;
        if(config.terminal_state.size())cfg.terminal_state=config.terminal_state;
        // The reference solves each horizon with its original work allowance.
        solver_=std::make_unique<OcpSolver>(cfg);
        accepted_.reset();applied_=model_.hover;origin_=0.;prefix_duration_=0.;node_times_.clear();count_=0;
    }
    ExecutionMode executionMode() const {return descriptor_->execution_mode;}
    void reset(){configure(config_,descriptor_);}
    void stop(){solver_.reset();accepted_.reset();}
    PlannerCoreStepResult solve(const PlannerCoreInput& input) {
        PlannerCoreStepResult out;
        if(!solver_){out.rejection_reason="not_configured";return out;}
        if(!validateFrameContract(input.state_frame,config_.world_frame).ok){out.rejection_reason="frame_contract";return out;}
        if(!std::isfinite(input.absolute_time_sec)||input.current_state.size()<descriptor_->state_dim||
           !input.current_state.allFinite()||!input.target_snapshot.valid||
           !input.target_snapshot.position.allFinite()||!input.target_snapshot.velocity.allFinite()||
           !input.target_snapshot.acceleration.allFinite()){
            out.rejection_reason="synchronous_mpc_invalid_input";return out;
        }
        if(accepted_&&std::abs(input.absolute_time_sec-(origin_+prefix_duration_))>1e-9){
            out.rejection_reason="synchronous_mpc_cycle_time";return out;
        }
        auto model=descriptor_->make_model(input.target_snapshot.acceleration);
        if(config_.preserve_remaining_trajectory&&accepted_){
            model.horizon=model_.horizon-intervals_;
            if(model.horizon<=0){out.rejection_reason="finite_maneuver_completed";return out;}
        }
        auto root=prepareOcpState(*descriptor_,input.current_state,input.target_snapshot);
        if(model.initial_state)root=model.initial_state(root,applied_);
        if(descriptor_->state_dim==13&&root.size()>13)root(13)=0.;
        if(!accepted_&&descriptor_->feasible_start&&!descriptor_->feasible_start(root)){
            out.rejection_reason="initial_state_infeasible";return out;
        }
        const auto snapshot=solver_->snapshotWarmStart();
        if(accepted_&&descriptor_->coordinates==OCPDescriptor::Coordinates::TargetRelative){
            auto warm=descriptor_->variable_dt?shiftVariablePrefix(input.target_snapshot):
                shiftMpcSeed(*accepted_,model_,target_,input.target_snapshot,prefix_duration_);
            solver_->restoreWarmStart(warm);solver_->setNextWarmStartShift(0);
        }
        if(config_.preserve_remaining_trajectory)solver_->setNextHorizon(model.horizon);
        const auto raw=solver_->solve(root,input.target_snapshot.acceleration,{},input.absolute_time_sec);
        out.solve_attempted=true;out.solve_num=count_;out.x0_source="measured";
        auto& result=out.solve_result;
        result.success=raw.success;result.state_trajectory=raw.state_trajectory;
        result.control_trajectory=raw.control_trajectory;result.feedback_gains=raw.feedback_gains;
        result.solve_time_ms=raw.solve_time_ms;result.solve_iters=raw.solve_iters;
        result.constraint_error=raw.constraint_error;result.primal_constraint_violation=raw.primal.constraints;
        result.dynamics_error=raw.primal.dynamics;result.optimality_error=raw.optimality_error;
        result.solver_converged=raw.converged;result.solver_termination=raw.termination;
        result.solve_start_time=raw.solve_start_time;result.solve_finish_time=raw.solve_finish_time;
        result.solve_timestamp=raw.solve_timestamp;
        const bool relative=descriptor_->coordinates==OCPDescriptor::Coordinates::TargetRelative;
        result.is_relative_plan=relative;result.target_snapshot_pos=input.target_snapshot.position;
        result.target_snapshot_vel=input.target_snapshot.velocity;result.target_snapshot_acc=input.target_snapshot.acceleration;
        if(descriptor_->post_process_result)descriptor_->post_process_result(result,input.target_snapshot);
        bool valid=raw.success&&result.success&&raw.state_trajectory.size()==static_cast<size_t>(model.horizon+1)&&
            raw.control_trajectory.size()==static_cast<size_t>(model.horizon);
        for(const auto& x:raw.state_trajectory)valid=valid&&x.size()==model.nx&&x.allFinite();
        for(const auto& u:raw.control_trajectory)valid=valid&&u.size()==model.nu&&u.allFinite();
        if(!valid){solver_->restoreWarmStart(snapshot);out.rejection_reason="synchronous_mpc_primal_not_certified";return out;}
        std::vector<double> times{0.};long double elapsed=0.;
        for(int k=0;k<model.horizon;++k){
            const double h=model.stage_duration?model.stage_duration(raw.state_trajectory[k],raw.control_trajectory[k]):model.dt;
            if(!std::isfinite(h)||h<=0.){solver_->restoreWarmStart(snapshot);out.rejection_reason="synchronous_rh_invalid_duration";return out;}
            elapsed+=h;times.push_back(static_cast<double>(elapsed));
        }
        // Wall solve duration is deliberately not added to simulation time.
        accepted_=raw;model_=model;target_=input.target_snapshot;origin_=input.absolute_time_sec;
        intervals_=std::min(config_.n_replay>0?config_.n_replay:descriptor_->default_n_replay,model.horizon);
        node_times_=std::move(times);prefix_duration_=node_times_[intervals_];
        out.solve_accepted=true;++count_;
        out.solve_log.solve_num=out.solve_num;out.solve_log.ocp_dt=model.dt;
        out.solve_log.coord_mode=relative?"relative":"world";
        for(int k=0;k<=model.horizon;++k){
            const auto target=predictTargetSnapshot(target_,node_times_[k]);
            out.path.states.push_back(reconstructWorldState(*descriptor_,raw.state_trajectory[k],target).head(descriptor_->state_dim));
        }
        return out;
    }
    PlannerCoreStepResult sample(const PlannerCoreInput& input) {
        PlannerCoreStepResult out;if(!accepted_)return out;
        const double age=input.absolute_time_sec-origin_;
        out.has_replay_sample=true;out.solve_num=count_-1;
        const double roundoff=1e-12*std::max(1.,prefix_duration_);
        if(!std::isfinite(age)||age<0.||age>=prefix_duration_-roundoff){out.stale=true;return out;}
        const int k=std::min(int(std::upper_bound(node_times_.begin(),node_times_.end(),age+roundoff)-node_times_.begin())-1,intervals_-1);
        const auto& u=accepted_->control_trajectory[k];
        const double remainder=std::max(0.,age-node_times_[k]);
        auto state=accepted_->state_trajectory[k];
        if(remainder>1e-12)state=model_.flow(state,u,remainder,false).x;
        out.applied_control_index=k;
        out.command.kind=descriptor_->command_mode==OCPDescriptor::CommandMode::CmdBodyRate?CommandKind::BodyRate:CommandKind::FullState;
        out.command.control=u;
        out.command.state=reconstructWorldState(*descriptor_,state,predictTargetSnapshot(target_,age)).head(descriptor_->state_dim);
        applied_=u;return out;
    }
};
}
