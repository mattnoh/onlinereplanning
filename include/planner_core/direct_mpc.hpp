#pragma once
#include "planner_core/quadrotor_mpc.hpp"
#include "planner_core/types.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>
#include <optional>
#include <fstream>

namespace planner_core {
class DirectMpcController {
    using Clock=std::chrono::steady_clock;
    struct Policy {
        QuadrotorMPC::Result result;
        Clock::time_point origin;
        int number=0;
        Eigen::Vector3d acceleration=Eigen::Vector3d::Zero();
    };
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::thread worker_;
    bool stop_=false,busy_=false;
    std::optional<PlannerCoreInput> request_;
    std::optional<PlannerCoreStepResult> completed_;
    std::optional<Policy> policy_,pending_;
    PlannerCoreConfig config_;
    Clock::time_point last_request_{},last_apply_{};
    Eigen::VectorXd last_u_=StcLandingMpcOCP::hover();
    double lead_=0;
    int count_=0;
    std::ofstream controls_;
    static double seconds(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double>(a-b).count();}
    static Eigen::VectorXd relative(const Eigen::VectorXd& x,const TargetSnapshot& t){
        Eigen::VectorXd r=x.head(13);r.head(3)-=t.position;r.segment<3>(3)-=t.velocity;return r;
    }
    void run() {
        QuadrotorMPC::Config cfg;cfg.ocp_type="stc_landing_mpc";cfg.n_shift=0;QuadrotorMPC solver(cfg);
        for(;;) {
            PlannerCoreInput input;Eigen::VectorXd applied;double lead;int number;std::optional<Policy> prior;
            {
                std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[&]{return stop_||request_.has_value();});
                if(stop_)return;input=*request_;request_.reset();applied=last_u_;lead=policy_ ? std::min(lead_,0.2) : 0.0;number=count_;prior=policy_;
            }
            try {
            const auto origin=input.now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(lead));
            Eigen::VectorXd x=Eigen::VectorXd::Zero(StcLandingMpcOCP::NX);x.head(13)=relative(input.current_state,input.target_snapshot);x.tail(4)=applied;
            auto pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();pred->a=input.target_snapshot.acceleration;
            StcLandingMpcOCP::Dynamics dyn(pred);
            // Project through the currently applied command over the measured
            // solve lead. High-rate feedback corrects prediction error.
            if(lead>0)x=dyn.flow(x,applied,lead,false).x;
            x(13)=0;
            if(prior){
                QuadrotorMPC::WarmStartSnapshot warm;
                const auto& old=prior->result;
                const double age=std::max(0.0,seconds(origin,prior->origin));
                for(int k=0;k<StcLandingMpcOCP::N;++k){
                    double s=age/StcLandingMpcOCP::DT+k;int i=std::min(static_cast<int>(s),StcLandingMpcOCP::N-1);
                    int j=std::min(i+1,StcLandingMpcOCP::N-1);double a=std::min(1.0,s-i);
                    warm.prev_U.push_back((1-a)*old.control_trajectory[i]+a*old.control_trajectory[j]);
                }
                // Factory rolls out these resampled controls from the live x0.
                solver.restoreWarmStart(warm);solver.setNextWarmStartShift(0);
            }
            auto snapshot=solver.snapshotWarmStart();
            auto raw=solver.solve(x,input.target_snapshot.acceleration,{},input.ros_time_sec+lead);
            PlannerCoreStepResult out;out.solve_attempted=true;out.solve_num=number;out.x0_source="measured_latency_projected";
            out.solve_result.success=raw.success;out.solve_result.state_trajectory=raw.state_trajectory;
            out.solve_result.control_trajectory=raw.control_trajectory;out.solve_result.feedback_gains=raw.feedback_gains;
            out.solve_result.solve_time_ms=raw.solve_time_ms;out.solve_result.solve_iters=raw.solve_iters;
            out.solve_result.constraint_error=raw.constraint_error;out.solve_result.solve_start_time=raw.solve_start_time;out.solve_result.solve_finish_time=raw.solve_finish_time;
            out.solve_result.is_relative_plan=true;out.solve_result.target_snapshot_pos=input.target_snapshot.position;
            out.solve_result.target_snapshot_vel=input.target_snapshot.velocity;out.solve_result.target_snapshot_acc=input.target_snapshot.acceleration;
            bool valid=raw.success&&std::isfinite(raw.constraint_error)&&raw.constraint_error<=config_.max_constraint_error&&
                raw.state_trajectory.size()==StcLandingMpcOCP::N+1&&raw.control_trajectory.size()==StcLandingMpcOCP::N&&raw.feedback_gains.size()==StcLandingMpcOCP::N;
            for(const auto& v:raw.state_trajectory)valid=valid&&v.size()==StcLandingMpcOCP::NX&&v.allFinite()&&v.head(3).norm()<100&&v.segment(3,3).norm()<20;
            for(const auto& v:raw.control_trajectory)valid=valid&&v.size()==4&&v.allFinite();
            for(const auto& k:raw.feedback_gains)valid=valid&&k.rows()==4&&k.cols()==StcLandingMpcOCP::NX&&k.allFinite();
            valid=valid&&seconds(Clock::now(),origin)<0.5;
            out.solve_accepted=valid;out.rejection_reason=valid ? "" : "direct_mpc_invalid_or_stale";
            out.solve_log.ocp_dt=StcLandingMpcOCP::DT;out.solve_log.coord_mode="relative";
            out.solve_log.solve_num=number;out.solve_log.is_relative_plan=true;
            out.solve_log.target_snapshot_pos=input.target_snapshot.position;out.solve_log.target_snapshot_vel=input.target_snapshot.velocity;out.solve_log.target_snapshot_acc=input.target_snapshot.acceleration;
            for(int i=0;i<static_cast<int>(raw.state_trajectory.size());++i){auto world=raw.state_trajectory[i];double t=lead+i*StcLandingMpcOCP::DT;
                world.head(3)+=input.target_snapshot.position+input.target_snapshot.velocity*t+0.5*input.target_snapshot.acceleration*t*t;
                out.path.states.push_back(world.head(13));}
            if(!valid)solver.restoreWarmStart(snapshot);
            {
                std::lock_guard<std::mutex> lock(mutex_);if(stop_)return;
                if(valid){pending_=Policy{raw,origin,number,input.target_snapshot.acceleration};++count_;lead_=0.8*lead_+0.2*raw.solve_time_ms/1000;}
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
    void configure(const PlannerCoreConfig& c){stop();std::lock_guard<std::mutex> lock(mutex_);config_=c;stop_=false;busy_=false;
        request_.reset();completed_.reset();policy_.reset();pending_.reset();last_request_={};last_apply_={};lead_=0;count_=0;last_u_=StcLandingMpcOCP::hover();
        worker_=std::thread([this]{run();});}
    void reset(){PlannerCoreConfig c; {std::lock_guard<std::mutex> lock(mutex_);c=config_;} configure(c);}
    void openControlLog(const std::string& path){std::lock_guard<std::mutex> lock(mutex_);controls_.close();controls_.open(path);
        controls_<<"time,policy,age,nom_thrust,nom_roll,nom_pitch,nom_yaw,fb_thrust,fb_roll,fb_pitch,fb_yaw,thrust,roll,pitch,yaw,saturation,jump\n";}
    PlannerCoreStepResult trySolve(const PlannerCoreInput& input){
        std::lock_guard<std::mutex> lock(mutex_);PlannerCoreStepResult out;
        if(completed_){out=std::move(*completed_);completed_.reset();}
        if(input.current_state.size()<13||!input.current_state.allFinite()||!input.target_snapshot.valid){out.rejection_reason="direct_mpc_invalid_state";return out;}
        auto rel=relative(input.current_state,input.target_snapshot);
        if(policy_&&config_.enable_terminal_freeze&&rel.head(3).norm()<config_.terminal_freeze_enter_pos&&rel.segment(3,3).norm()<config_.terminal_freeze_enter_vel){out.terminal_freeze_engaged=true;out.solve_accepted=false;out.rejection_reason="terminal_freeze_engaged";return out;}
        if(!busy_&&(last_request_==Clock::time_point{}||seconds(input.now,last_request_)>=0.1)){
            request_=input;busy_=true;last_request_=input.now;ready_.notify_one();}
        return out;
    }
    PlannerCoreStepResult sample(Clock::time_point now,const Eigen::VectorXd& measured,const TargetSnapshot& target){
        std::lock_guard<std::mutex> lock(mutex_);PlannerCoreStepResult out;
        if(pending_&&now>=pending_->origin){policy_=std::move(pending_);pending_.reset();}
        if(!policy_)return out;out.has_replay_sample=true;out.solve_num=policy_->number;
        const double age=seconds(now,policy_->origin);
        if(age<0||age>=StcLandingMpcOCP::N*StcLandingMpcOCP::DT||measured.size()<13||!measured.allFinite()){out.stale=true;return out;}
        int i=std::min(static_cast<int>(age/StcLandingMpcOCP::DT),StcLandingMpcOCP::N-1);
        const auto& r=policy_->result;Eigen::VectorXd nominal=r.control_trajectory[i];
        auto pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();
        pred->a=policy_->acceleration;
        StcLandingMpcOCP::Dynamics dyn(pred);
        Eigen::VectorXd reference=dyn.flow(r.state_trajectory[i],nominal,age-i*StcLandingMpcOCP::DT,false).x;
        reference.tail(4)=r.state_trajectory[i].tail(4);
        Eigen::VectorXd live=reference;live.head(13)=relative(measured,target);live.tail(4)=last_u_;
        if(live.segment<4>(6).dot(reference.segment<4>(6))<0)live.segment<4>(6)*=-1;
        Eigen::VectorXd delta=live-reference;delta(13)=0;
        Eigen::VectorXd feedback=r.feedback_gains[i]*delta;Eigen::VectorXd raw=nominal+feedback;
        if(!raw.allFinite()){out.stale=true;return out;}
        Eigen::VectorXd u=StcLandingMpcOCP::bounded(raw);
        const bool saturated=(u-raw).norm()>1e-9;
        const double h=last_apply_==Clock::time_point{}?0.02:std::clamp(seconds(now,last_apply_),0.0,0.05);
        Eigen::Vector4d slew;slew<<20,5,5,5;
        for(int j=0;j<4;++j)u(j)=std::clamp(u(j),last_u_(j)-slew(j)*h,last_u_(j)+slew(j)*h);
        const double jump=(u-last_u_).norm();last_u_=u;last_apply_=now;
        out.command.kind=CommandKind::BodyRate;out.command.control=u;out.command.state=reference.head(13);
        if(controls_){controls_<<std::chrono::duration<double>(now.time_since_epoch()).count()<<','<<policy_->number<<','<<age;
            for(const auto& v:{nominal,feedback,u})for(int j=0;j<4;++j)controls_<<','<<v(j);controls_<<','<<saturated<<','<<jump<<'\n';}
        return out;
    }
};
}
