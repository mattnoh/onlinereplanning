#include "ocp/ocp_landing_rh.hpp"
#include "ocp/ocp_landing_mpc.hpp"
#include "controller_internal.hpp"
#include "planner_core/controller.hpp"
#include "planner_core/types.hpp"
#include <stdexcept>
#include <iostream>
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 for(const char* name:{"hover","landing_rh","stc_landing_noaug"}) {
  auto descriptor=OCPRegistry::getDescriptor(name);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(13);x(6)=1;x(2)=1;
  Eigen::VectorXd u=Eigen::VectorXd::Zero(4);u(0)=descriptor.default_mass_kg*9.81;
  TargetSnapshot target;target.valid=true;target.acceleration<<.1,.2,0;
  auto command=planner_core::makePlannerCommand(planner_core::Platform::Mavros,descriptor,x,u,target);
  check(command.acceleration.has_value()&&command.acceleration->norm()<1e-12,"DDP hover acceleration must include gravity exactly once");
  x.segment<4>(6)<<std::cos(.1),0,std::sin(.1),0;
  auto acceleration=descriptor.world_acceleration(x,u);
  check(std::abs(acceleration.x()-9.81*std::sin(.2))<1e-12,"DDP thrust acceleration direction");
  check(std::abs(acceleration.z()-9.81*(std::cos(.2)-1))<1e-12,"DDP thrust acceleration magnitude");
  check((planner_core::makePlannerCommand(planner_core::Platform::Mavros,descriptor,x,u,target).acceleration.value()-acceleration).norm()<1e-12,"target acceleration must not be added twice to world feedforward");
 }
 planner_core::Controller controller;
 planner_core::PlannerCoreConfig config;config.ocp_type="hover";config.ocp_dt=.05;
 controller.configure(config);check(!controller.isDirect(),"hover must use RH");
 controller.setTerminalTarget(Eigen::Vector3d(0,0,1));controller.reset();
 config.ocp_type="stc_landing_mpc";controller.configure(config);
 check(controller.isDirect() && controller.commandPeriod()==.01,"direct MPC cadence");
 planner_core::PlannerCoreInput input;input.current_state=Eigen::VectorXd::Zero(13);input.current_state(6)=1;
 check(controller.trySolve(input).rejection_reason=="direct_mpc_invalid_state","invalid target must reject");
 check(controller.solveOpenLoop(input).rejection_reason=="direct_mpc_requires_feedback","direct open-loop must reject");
 controller.reset();check(!controller.sample(input).has_replay_sample,"reset must clear policy");
 config.ocp_type="hover";controller.configure(config);check(!controller.isDirect(),"mode switch");controller.stop();
 bool absent=false;try{OCPRegistry::getDescriptor("stc_landing_noaug_legacy");}catch(const std::runtime_error&){absent=true;}check(absent,"legacy formulation still registered");
 for(const std::string type:{"landing_rh","landing_mpc"}){
  const auto& descriptor=OCPRegistry::getDescriptor(type);
  check(descriptor.coordinates==OCPDescriptor::Coordinates::TargetRelative,"ordinary landing coordinates");
  auto model=descriptor.make_model(Eigen::Vector3d::Zero());
  Eigen::VectorXd x=Eigen::VectorXd::Zero(model.nx);x(2)=1.85;x(6)=1;
  check(descriptor.feasible_start(x),"inside-cone start should be feasible");
  auto bad=x;bad(0)=1;check(!descriptor.feasible_start(bad),"outside-cone start must reject");
  bad=x;bad(10)=.03;check(!descriptor.feasible_start(bad),"excessive angular rate must reject");
  OCPCreateArgs args;args.current_state=x;
  auto problem=descriptor.create(args);
  check(problem->getTerminalConstraints(ConstraintType::NO).size()==1,"terminal path constraints missing");
  check(problem->getTerminalConstraints(ConstraintType::SOC).size()==5,"terminal conic limits missing");
  auto limits=type=="landing_rh"?std::shared_ptr<StageConstraintBase<double>>(std::make_shared<LandingRhOCP::LandingLimits>(model.nx,model.nu)):std::shared_ptr<StageConstraintBase<double>>(std::make_shared<LandingMpcOCP::Ref::LandingLimits>(model.nx,model.nu));
  x.head(3)<<.1,.2,1.5;x.segment(3,3)<<.2,-.1,.05;x.segment(6,4)<<1.01,.004,-.006,.03;x.segment(10,3)<<.01,.005,-.002;
  const auto analytic=limits->cx(x,model.hover);Eigen::MatrixXd numeric(6,model.nx);
  for(int j=0;j<model.nx;++j){auto plus=x,minus=x;plus(j)+=1e-7;minus(j)-=1e-7;numeric.col(j)=(limits->c(plus,model.hover)-limits->c(minus,model.hover))/(2e-7);}
  check((analytic-numeric).cwiseAbs().maxCoeff()<2e-5,"ordinary constraint Jacobian mismatch");
  for(const auto& row:problem->getStageConstraints(0,ConstraintType::SOC)){
   const auto gradient=row->cx(x,model.hover);Eigen::MatrixXd differences(row->getDimC(),model.nx);
   for(int j=0;j<model.nx;++j){auto plus=x,minus=x;plus(j)+=1e-7;minus(j)-=1e-7;differences.col(j)=(row->c(plus,model.hover)-row->c(minus,model.hover))/(2e-7);}
   check((gradient-differences).cwiseAbs().maxCoeff()<2e-5,"native cone Jacobian mismatch");
  }

  x.setZero();x(2)=1.85;x(6)=1;
  OcpSolver::Config solver_config;solver_config.ocp_type=type;OcpSolver solver(solver_config);
  auto result=solver.solve(x);
  check(result.success && result.constraint_error<=1.0,"ordinary constrained solve failed residual gate");
  double violation=0;for(const auto& state:result.state_trajectory)violation=std::max(violation,limits->c(state,model.hover).maxCoeff());
  std::cout<<type<<" residual="<<result.constraint_error<<" max_node_constraint="<<violation<<" ms="<<result.solve_time_ms<<"\n";
  check(violation<1e-3,"ordinary cold solve violates node constraints");
  // Measured moving-target activation that exposed the scalar-row cold-start failure.
  Eigen::VectorXd moving(13);moving<<-.07320069131,-.04604300079,1.832031775,.07864153001,-.004976944706,.008655228599,.999980267207,-.004416340294,.003462269278,-.00282379661,.01105888095,.001028963132,-.002000458539;
  moving.segment<4>(6).normalize();
  OcpSolver moving_solver(solver_config);auto moving_result=moving_solver.solve(moving,Eigen::Vector3d(.04306807449,.07358261346,0));
  check(moving_result.success && moving_result.constraint_error<=1,"moving ordinary cold start failed");
  for(const auto& state:moving_result.state_trajectory)check(limits->c(state,model.hover).maxCoeff()<1e-3,"moving ordinary plan violates node limits");
  config.ocp_type=type;controller.configure(config);check(controller.isDirect()==(type=="landing_mpc"),"ordinary execution mode selection");controller.stop();
 }
 std::cout<<"Controller lifecycle/mode/invalid-input tests passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
