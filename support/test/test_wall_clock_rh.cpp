// Wall-clock, non-SITL plant through the production asynchronous Controller API.
// No frozen optimization time, state teleportation or trajectory tracking reset.
#include "pointmass_reference_adapter.hpp"
#include "planner_core/controller.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include <thread>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <iostream>
using Clock=std::chrono::steady_clock;
int main(int argc,char** argv){try{
 if(argc!=5&&argc!=6)throw std::invalid_argument("usage: test_wall_clock_rh quad|pointmass node|ctcs SECONDS OUTPUT [delay-compensated]");
 const bool quad=std::string(argv[1])=="quad",ctcs=std::string(argv[2])=="ctcs";
 if((!quad&&std::string(argv[1])!="pointmass")||(!ctcs&&std::string(argv[2])!="node"))throw std::invalid_argument("invalid model/mode");
 const bool compensated=argc==6&&std::string(argv[5])=="delay-compensated";
 if(argc==6&&!compensated)throw std::invalid_argument("unknown timing mode");
 const double seconds=std::stod(argv[3]);const std::filesystem::path out(argv[4]);
 if(seconds<=0||seconds>120||!std::filesystem::create_directories(out))throw std::invalid_argument("invalid duration/new output path");
 auto desc=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor(ctcs?"stc_landing_noaug":"stc_landing_noaug_node")):pointmassReferenceDescriptor(100,false,!ctcs);
 // Explicit full measured root: the default ROS blended/predicted root is NOT
 // silently counted as measured feedback in this experiment.
 desc->use_predicted_handoff_state=false;
 // Match the retained shifted-control initial guess; no feedback-guided seed.
 desc->warm_start=OCPDescriptor::WarmStart::Shift;
 const Eigen::Vector3d acc=quad?Eigen::Vector3d(.04306807449,.07358261346,0):Eigen::Vector3d::Zero();
 auto targetAt=[&](double t){TargetSnapshot x;x.valid=true;x.position=.5*acc*t*t;x.velocity=acc*t;x.acceleration=acc;return x;};
 const auto model=desc->make_model(Eigen::Vector3d::Zero());
 Vec plant=Vec::Zero(quad?13:4);
 if(quad){if(ctcs)plant<<2.,1.5,1.85,0,0,0,1,0,0,0,0,0,0;
 else plant<<-.07320069131,-.04604300079,1.832031775,.07864153001,-.004976944706,.008655228599,.999980267207,-.004416340294,.003462269278,-.00282379661,.01105888095,.001028963132,-.002000458539;
 plant.segment<4>(6).normalize();}else plant<<-10,0,0,0;
 if(model.initial_state)plant=model.initial_state(plant,model.hover);
 std::mutex state_mutex;double sim=0.;std::int64_t ticks=0;Vec expected_root;double max_root_error=0.;
 std::shared_ptr<OptimalControlProblem<double>> captured;double native_budget_ratio=0.;
 const auto create=desc->create;
 desc->create=[&](const OCPCreateArgs& args){
  if(expected_root.size())max_root_error=std::max(max_root_error,(args.current_state.head(desc->state_dim)-expected_root.head(desc->state_dim)).lpNorm<Eigen::Infinity>());
  captured=create(args);return captured;
 };
 planner_core::PlannerCoreConfig cfg;cfg.ocp_type=desc->name;cfg.ocp_dt=desc->dt;cfg.n_replay=4;cfg.enable_terminal_freeze=false;
 cfg.max_first_solve_age_sec=1.;cfg.measured_state_delay_compensation=compensated;
 if(compensated){cfg.initial_solve_lead_sec=comando_parameters::envOrQ("WALL_RH_INITIAL_LEAD",1.2);cfg.solve_lead_guard_sec=comando_parameters::envOrQ("WALL_RH_GUARD",.3);}
 planner_core::Controller controller;controller.configure(cfg,desc);
 const auto origin=Clock::now();std::atomic<bool> stop{false},busy{false};
 std::atomic<int> accepted{0},attempted{0};
 double max_request_age=0.,max_state_change_during_solve=0.,max_native=0.;std::string failure;
 std::ofstream solves(out/"solves.jsonl"),plans(out/"plans.jsonl"),trace(out/"plant.csv");plans<<std::setprecision(17);solves<<std::setprecision(17);trace<<std::setprecision(17)<<"t,solving";for(int j=0;j<plant.size();++j)trace<<",x"<<j;trace<<'\n';trace<<0<<','<<0;for(int j=0;j<plant.size();++j)trace<<','<<plant(j);trace<<'\n';
 std::thread worker([&]{try{
  while(!stop.load()){
   planner_core::PlannerCoreInput in;Vec before;
   {std::lock_guard<std::mutex> lock(state_mutex);in.current_state=plant;before=plant;in.absolute_time_sec=sim;in.target_snapshot=targetAt(sim);in.now=origin+std::chrono::milliseconds(ticks);}
   in.state_frame="world";expected_root=planner_core::prepareOcpState(*desc,in.current_state,in.target_snapshot);busy=true;
   auto result=controller.trySolve(in);busy=false;
   if(result.solve_attempted){
    ++attempted;double after_time;Vec after;
    {std::lock_guard<std::mutex> lock(state_mutex);after_time=sim;after=plant;}
    max_request_age=std::max(max_request_age,after_time-in.absolute_time_sec);
    max_state_change_during_solve=std::max(max_state_change_during_solve,(after.head(desc->state_dim)-before.head(desc->state_dim)).lpNorm<Eigen::Infinity>());
    max_native=std::max(max_native,result.solve_result.primal_constraint_violation);
    if(captured){
     std::ofstream request(out/(result.solve_accepted?"latest-request.txt":"rejected-request.txt"));request<<std::setprecision(17);
     request<<"model "<<argv[1]<<"\nepoch "<<in.absolute_time_sec<<"\nmeasurement "<<in.current_state.transpose()<<"\n";
     auto vectors=[&](const char* label,const auto& values){request<<label<<' '<<values.size()<<'\n';for(const auto& x:values)request<<x.size()<<' '<<x.transpose()<<'\n';};
     std::vector<Vec> guess_x,guess_u;for(int k=0;k<=captured->getHorizon();++k){guess_x.push_back(captured->getInitialState(k));if(k<captured->getHorizon())guess_u.push_back(captured->getInitialControl(k));}
     vectors("prepared_X",guess_x);vectors("prepared_U",guess_u);vectors("returned_X",result.solve_result.state_trajectory);vectors("returned_U",result.solve_result.control_trajectory);
     if(!result.solve_accepted)for(size_t k=0;k<result.solve_result.control_trajectory.size();++k)
      for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})for(const auto& row:captured->getStageConstraints(k,type)){
       const auto c=row->c(result.solve_result.state_trajectory[k],result.solve_result.control_trajectory[k]);
       const double excess=type==ConstraintType::SOC?c(0)+c.tail(c.size()-1).norm():type==ConstraintType::EQ?c.cwiseAbs().maxCoeff():c.maxCoeff();
       if(excess>desc->getSolverParams().tolerance)request<<"rejected_stage "<<k<<" type "<<int(type)<<" residual "<<c.transpose()<<'\n';
      }
    }
    if(ctcs&&captured&&result.solve_accepted)for(size_t k=0;k<result.solve_result.control_trajectory.size();++k)
     for(const auto& row:captured->getStageConstraints(k,ConstraintType::NO))
      if(auto interval=std::dynamic_pointer_cast<StcStageConstraint<double>>(row))
       native_budget_ratio=std::max(native_budget_ratio,interval->intervalIntegral(result.solve_result.state_trajectory[k],result.solve_result.control_trajectory[k])/interval->getEpsilon());
    solves<<"{\"request_time\":"<<in.absolute_time_sec<<",\"finish_plant_time\":"<<after_time<<",\"accepted\":"<<result.solve_accepted<<",\"source\":"<<std::quoted(result.x0_source)<<",\"reason\":"<<std::quoted(result.rejection_reason)<<",\"solve_ms\":"<<result.solve_result.solve_time_ms<<",\"native_violation\":"<<result.solve_result.primal_constraint_violation<<",\"late_by\":"<<result.solve_finish_late_by_sec;
    if(result.solve_accepted){
     const double activation=std::chrono::duration<double>(result.solve_result.solve_finish_time-origin).count()-result.solve_finish_late_by_sec;
     plans<<"{\"solve_num\":"<<result.solve_num<<",\"activation_epoch\":"<<activation;
     auto matrix=[&](const char* label,const auto& values){plans<<",\""<<label<<"\":[";for(size_t k=0;k<values.size();++k){if(k)plans<<',';plans<<'[';for(int j=0;j<values[k].size();++j){if(j)plans<<',';plans<<values[k](j);}plans<<']';}plans<<']';};
     matrix("states",result.solve_result.state_trajectory);matrix("controls",result.solve_result.control_trajectory);plans<<"}\n";plans.flush();
     solves<<",\"activation_epoch\":"<<activation<<",\"node_times\":[";
     for(size_t k=0;k<result.solve_result.state_trajectory.size();++k){
      if(k)solves<<',';const auto& x=result.solve_result.state_trajectory[k];
      solves<<activation+(desc->variable_dt?x(desc->state_dim):k*cfg.ocp_dt);
     }solves<<']';
    }solves<<"}\n";solves.flush();
    if(result.solve_accepted)++accepted;
    else {failure=result.rejection_reason;stop=true;}
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
 }catch(const std::exception& e){failure=e.what();busy=false;stop=true;}});
 int samples_while_solving=0,stale_samples=0;bool reached=false;double max_hard=0.,max_obstacle=0.,max_physical_stc=0.,tracking_squared=0.,max_tracking_error=0.;
 try{
  while(!stop.load()&&((sim<seconds&&!reached)||busy.load())){
   const double wall=std::chrono::duration<double>(Clock::now()-origin).count();
   if(wall-sim<.001){std::this_thread::sleep_for(std::chrono::microseconds(500));continue;}
   planner_core::PlannerCoreInput in;
   {std::lock_guard<std::mutex> lock(state_mutex);in.current_state=plant;in.absolute_time_sec=sim;in.target_snapshot=targetAt(sim);in.now=origin+std::chrono::milliseconds(ticks);}
   in.state_frame="world";auto sample=controller.sample(in);
   Vec u=model.hover;if(sample.has_replay_sample&&!sample.stale){
    if(sample.command.control.size()>u.size())throw std::runtime_error("runtime control dimension exceeds model");
    u.head(sample.command.control.size())=sample.command.control;
   }
   if(sample.stale)++stale_samples;if(busy.load())++samples_while_solving;
   {std::lock_guard<std::mutex> lock(state_mutex);
    plant=model.flow(plant,u,.001,false).x;++ticks;sim=.001*ticks;
    if(!plant.allFinite())throw std::runtime_error("nonfinite physical plant");
    const auto rel=planner_core::prepareOcpState(*desc,plant,targetAt(sim));
    if(quad){max_hard=std::max({max_hard,-rel(2),rel.segment<3>(3).norm()-StcLandingNoAugOCP::SPD_PHASE0_MAX});
     if(ctcs){const auto terms=StcLandingNoAugOCP::evalCtcsIntegrandTerms<double>(rel,u,false);
      max_physical_stc=std::max({max_physical_stc,terms.loose_tilt,terms.loose_omega});
      if(terms.a_stage>0)max_physical_stc=std::max(max_physical_stc,StcLandingNoAugOCP::Z_STAGE-rel(2));
      if(terms.a_los>0)max_physical_stc=std::max(max_physical_stc,terms.los);
      if(terms.a_altitude_state>0)max_physical_stc=std::max({max_physical_stc,terms.speed,terms.tilt,terms.omega,terms.glideslope});
     }
     if(accepted.load()>0)reached=rel.head<3>().norm()<=.1&&rel.segment<3>(3).norm()<=.3;
    }else{const double tracking_error=(rel.head<2>()-reference(sim)).norm();tracking_squared+=.001*tracking_error*tracking_error;max_tracking_error=std::max(max_tracking_error,tracking_error);Problem p;for(int i=0;i<NOBS;++i)max_obstacle=std::max(max_obstacle,obstacleG(p,rel.head<2>(),i));}
    trace<<sim<<','<<busy.load();for(int j=0;j<plant.size();++j)trace<<','<<plant(j);trace<<'\n';
   }
  }
 }catch(...){stop=true;worker.join();throw;}
 stop=true;worker.join();controller.stop();
 const double wall=std::chrono::duration<double>(Clock::now()-origin).count();
 // Success here is runtime completion, NOT a CTCS or obstacle certificate.
 const bool complete=failure.empty()&&(quad?reached:sim>=seconds-.001)&&accepted.load()>1&&(compensated||max_root_error<=1e-12)&&samples_while_solving>0;
 std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\"model\":"<<std::quoted(argv[1])<<",\"ctcs\":"<<ctcs<<",\"runtime_complete\":"<<complete<<",\"reached\":"<<reached<<",\"simulation_time\":"<<sim<<",\"wall_seconds\":"<<wall<<",\"attempted\":"<<attempted.load()<<",\"accepted\":"<<accepted.load()<<",\"delay_compensated\":"<<compensated<<",\"measurement_to_root_change\":"<<max_root_error<<",\"measured_root_error\":"<<(compensated?std::string("null"):std::to_string(max_root_error))<<",\"max_request_age_seconds\":"<<max_request_age<<",\"plant_change_during_solve\":"<<max_state_change_during_solve<<",\"plant_steps_while_solving\":"<<samples_while_solving<<",\"stale_samples\":"<<stale_samples<<",\"native_violation\":"<<max_native<<",\"native_CTCS_budget_ratio\":"<<native_budget_ratio<<",\"physical_STC_excess\":"<<max_physical_stc<<",\"hard_constraint_excess\":"<<max_hard<<",\"obstacle_excess_diagnostic\":"<<max_obstacle<<",\"tracking_position_rmse\":"<<std::sqrt(tracking_squared/std::max(sim,.001))<<",\"max_tracking_error\":"<<max_tracking_error<<",\"failure\":"<<std::quoted(failure)<<",\"continuous_CTCS_certified\":false}\n";
 std::cout<<"WALL_CLOCK accepted="<<accepted<<" attempted="<<attempted<<" measurement_to_root_change="<<max_root_error<<" moving_during_solve="<<max_state_change_during_solve<<" reached="<<reached<<" failure="<<failure<<'\n';return complete?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
