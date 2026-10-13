#include "planner_core/controller.hpp"
#include "planner_core/planner_core.hpp"
#include "planner_core/direct_mpc.hpp"
#include <shared_mutex>
namespace planner_core {
struct Controller::Impl {
    mutable std::shared_mutex lifecycle;
    std::mutex solver;
    PlannerCoreConfig config;
    PlannerCore rh;
    DirectMpcController direct;
    ExecutionMode mode=ExecutionMode::RhReplanning;
    int accepted=0;
    bool configured=false;
};
Controller::Controller():impl_(std::make_unique<Impl>()){}
Controller::~Controller()=default;
void Controller::configure(const PlannerCoreConfig& c) {
    std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);
    impl_->direct.stop();impl_->config=c;impl_->accepted=0;
    impl_->mode=OCPRegistry::getDescriptor(c.ocp_type).execution_mode;
    if(impl_->mode==ExecutionMode::DirectMpc) impl_->direct.configure(c);
    else impl_->rh.configure(c);
    impl_->configured=true;
}
void Controller::reset(){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->accepted=0;if(!impl_->configured)return;if(impl_->mode==ExecutionMode::DirectMpc)impl_->direct.reset();else impl_->rh.reset();}
void Controller::stop(){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->direct.stop();impl_->rh.reset();impl_->configured=false;}
void Controller::setTerminalTarget(const Eigen::Vector3d& p){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);impl_->config.terminal_position_abs=p;if(impl_->configured && impl_->mode==ExecutionMode::RhReplanning)impl_->rh.setTerminalTarget(p);}
PlannerCoreStepResult Controller::trySolve(const PlannerCoreInput& in){
    std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);if(!impl_->configured)return {};
    std::lock_guard<std::mutex> solve(impl_->solver);
    auto out=impl_->mode==ExecutionMode::DirectMpc?impl_->direct.trySolve(in):impl_->rh.trySolve(in);
    if(out.solve_accepted)++impl_->accepted;
    return out;
}
PlannerCoreStepResult Controller::solveOpenLoop(const PlannerCoreInput& in){std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);std::lock_guard<std::mutex> solve(impl_->solver);if(impl_->mode==ExecutionMode::DirectMpc){PlannerCoreStepResult out;out.rejection_reason="direct_mpc_requires_feedback";return out;}return impl_->rh.solveOpenLoop(in);}
PlannerCoreStepResult Controller::sample(const PlannerCoreInput& in){std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);if(!impl_->configured)return {};return impl_->mode==ExecutionMode::DirectMpc?impl_->direct.sample(in.now,in.current_state,in.target_snapshot):impl_->rh.sampleReplay(in.now,in.target_snapshot);}
ExecutionMode Controller::executionMode() const {std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);return impl_->mode;}
double Controller::commandPeriod() const {std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);return impl_->mode==ExecutionMode::DirectMpc?.02:impl_->config.ocp_dt;}
double Controller::solvePollPeriod() const {return isDirect()?.1:.001;}
int Controller::solveCount() const {std::shared_lock<std::shared_mutex> guard(impl_->lifecycle);std::lock_guard<std::mutex> solve(impl_->solver);return impl_->accepted;}
void Controller::openControlLog(const std::string& path){std::unique_lock<std::shared_mutex> guard(impl_->lifecycle);if(impl_->mode==ExecutionMode::DirectMpc)impl_->direct.openControlLog(path);}
}
