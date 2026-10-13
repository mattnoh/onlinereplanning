#include "planner_core/controller.hpp"
#include "planner_core/ocp_registry.hpp"
#include <stdexcept>
#include <iostream>
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 planner_core::Controller controller;
 planner_core::PlannerCoreConfig config;config.ocp_type="hover";config.ocp_dt=.05;
 controller.configure(config);check(!controller.isDirect(),"hover must use RH");
 controller.setTerminalTarget(Eigen::Vector3d(0,0,1));controller.reset();
 config.ocp_type="stc_landing_mpc";controller.configure(config);
 check(controller.isDirect() && controller.commandPeriod()==.02,"direct MPC cadence");
 planner_core::PlannerCoreInput input;input.current_state=Eigen::VectorXd::Zero(13);input.current_state(6)=1;
 check(controller.trySolve(input).rejection_reason=="direct_mpc_invalid_state","invalid target must reject");
 check(controller.solveOpenLoop(input).rejection_reason=="direct_mpc_requires_feedback","direct open-loop must reject");
 controller.reset();check(!controller.sample(input).has_replay_sample,"reset must clear policy");
 config.ocp_type="hover";controller.configure(config);check(!controller.isDirect(),"mode switch");controller.stop();
 bool absent=false;try{OCPRegistry::getDescriptor("stc_landing_noaug_legacy");}catch(const std::runtime_error&){absent=true;}check(absent,"legacy formulation still registered");
 std::cout<<"Controller lifecycle/mode/invalid-input tests passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
