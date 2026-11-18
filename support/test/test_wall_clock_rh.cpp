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
extern char** environ;
namespace saved_wall_audit {
namespace Q=StcLandingNoAugOCP;
struct Saved{int id=0;double epoch;std::vector<Vec>x,u;std::shared_ptr<OptimalControlProblem<double>> problem;std::shared_ptr<StcStageConstraint<double>> physical;Problem p;};
Vec readvec(std::istream& in){int n;if(!(in>>n)||n<=0||n>10000)throw std::runtime_error("invalid saved vector dimension");Vec x(n);for(int i=0;i<n;++i)in>>x(i);return x;}
int run(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("input output required");std::ifstream in(argv[1]);bool quad,ctcs;int count;if(!(in>>quad>>ctcs>>count)||count<=0||count>10000)throw std::runtime_error("invalid saved plan count");
 auto desc=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor(ctcs?"stc_landing_noaug":"stc_landing_noaug_node")):pointmassReferenceDescriptor(100,false,!ctcs);
 const double tolerance=desc->getSolverParams().tolerance;
 const Eigen::Vector3d acc=quad?Eigen::Vector3d(.04306807449,.07358261346,0.):Eigen::Vector3d::Zero();const auto model=desc->make_model(acc),world=desc->make_model(Eigen::Vector3d::Zero());
 std::vector<Saved> plans;plans.reserve(count);double planned_physical=0,planned_hard=0,native=0,native_ratio=0,dense_ratio=0,planned_all_obstacles=0,defect=0;std::map<std::string,double> planned_terms,executed_terms;double peak=0.,peak_time=0.;int peak_plan=-1;std::string peak_name;
 auto hard=[&](const Saved& p,const Vec& x,const Vec&u){double v=0.;for(auto type:{ConstraintType::NO,ConstraintType::SOC})for(const auto&row:p.problem->getStageConstraints(0,type)){
  if(std::dynamic_pointer_cast<StcStageConstraint<double>>(row))continue;const Vec c=row->c(x,u);if(c.size())v=std::max(v,type==ConstraintType::SOC?c(0)+c.tail(c.size()-1).norm():c.maxCoeff());}return v;};
 auto physical=[&](const Saved&p,const Vec&x,const Vec&u,std::map<std::string,double>&terms){double v=0.;if(quad){if(ctcs)for(int j=0;j<p.physical->numImplications();++j)if(p.physical->triggered(j,x)){const auto&im=p.physical->implication(j);const double c=im.consequence(x,u);v=std::max(v,c);if(&terms==&planned_terms&&c>peak){peak=c;peak_time=x(13);peak_plan=p.id;peak_name=im.name;}terms[im.name]=std::max(terms[im.name],c);}}else for(int j=0;j<NOBS;++j)if(p.p.mask[j]){double c=obstacleG(p.p,x.head<2>(),j);v=std::max(v,c);terms["active_obstacle_"+std::to_string(j)]=std::max(terms["active_obstacle_"+std::to_string(j)],c);}return v;};
 auto density=[&](const Saved&p,const Vec&x,const Vec&u,int k){if(quad)return Q::evalCtcsIntegrand<double>(x,u,k>=Q::ctcsTightenFromNode())/Q::CTCS_STEP_EPS;double v=0.;for(int j=0;j<NOBS;++j)if(p.p.mask[j]){const double c=std::max(0.,obstacleG(p.p,x.head<2>(),j)+p.p.delta);v+=c*c/p.p.eps;}return v;};
 for(int z=0;z<count;++z){Saved p;p.id=z;int nx,nu;in>>p.epoch>>nx>>nu;for(int i=0;i<nx;++i)p.x.push_back(readvec(in));for(int i=0;i<nu;++i)p.u.push_back(readvec(in));
  OCPCreateArgs args;args.current_state=p.x.front();args.target_accel=acc;args.horizon=nu;args.t_abs=p.epoch;p.problem=desc->create(args);
  for(int j=0;j<NOBS;++j)p.p.mask[j]=(p.p.s_diag.cwiseProduct(p.x.front().head<2>()-p.p.centers[j])).norm()<=6.;
  if(quad&&ctcs){auto dyn=std::dynamic_pointer_cast<Q::Quad6DOFVarTimeRelativePred<double>>(p.problem->getDynamics(0));p.physical=Q::makeCtcsConstraint<double>(dyn,Q::CTCS_STEP_EPS/Q::CTCS_Y_SCALE,Q::IDX_THETA,0.,1,false);}
  native=std::max(native,auditPrimal(*p.problem,p.x,p.u,desc->getSolverParams()).constraints);
  for(int k=0;k<nu;++k){auto x=p.x[k];const auto&u=p.u[k];const double h=quad?u(4):model.dt;const int n=std::max(1,int(std::ceil(h/.001)));const double dt=h/n;double integral=0.;
   if(ctcs)for(const auto&c:p.problem->getStageConstraints(k,ConstraintType::NO))if(auto row=std::dynamic_pointer_cast<StcStageConstraint<double>>(c))native_ratio=std::max(native_ratio,row->intervalIntegral(x,u)/row->getEpsilon());
   for(int i=0;i<n;++i){auto mid=model.flow(x,u,.5*dt,false).x;auto next=model.flow(x,u,dt,false).x;
    if(ctcs){planned_physical=std::max({planned_physical,physical(p,x,u,planned_terms),physical(p,mid,u,planned_terms),physical(p,next,u,planned_terms)});integral+=dt*(density(p,x,u,k)+4*density(p,mid,u,k)+density(p,next,u,k))/6.;}
    planned_hard=std::max({planned_hard,hard(p,x,u),hard(p,mid,u),hard(p,next,u)});
    if(!quad)for(int j=0;j<NOBS;++j)planned_all_obstacles=std::max({planned_all_obstacles,obstacleG(p.p,x.head<2>(),j),obstacleG(p.p,mid.head<2>(),j)});x=next;
   }defect=std::max(defect,(x-p.x[k+1]).lpNorm<Eigen::Infinity>());dense_ratio=std::max(dense_ratio,integral);
  }
  if(ctcs)planned_physical=std::max(planned_physical,physical(p,p.x.back(),p.u.back(),planned_terms));plans.push_back(std::move(p));
 }
 int samples;in>>samples;std::vector<double> t;std::vector<Vec> state;t.reserve(samples);state.reserve(samples);for(int i=0;i<samples;++i){double v;in>>v;t.push_back(v);state.push_back(readvec(in));}
 double executed_physical=0,executed_hard=0,executed_ratio=0,integral=0;int current_plan=-1,current_stage=-1;int intervals=0;double executed_node_hard=0;
 auto relative=[&](const Vec&x,double epoch){TargetSnapshot target;target.valid=true;target.position=.5*acc*epoch*epoch;target.velocity=acc*epoch;return planner_core::prepareOcpState(*desc,x,target);};
 for(int i=0;i+1<samples;++i){int z=-1;for(int j=0;j<count;++j)if(plans[j].epoch<=t[i]+1e-10)z=j;if(z<0)continue;const auto&p=plans[z];int k=0;double elapsed=t[i]-p.epoch;
  if(quad){while(k+1<int(p.u.size())&&p.x[k+1](13)<=elapsed+1e-10)++k;}else k=std::min(int(p.u.size())-1,int(std::floor((elapsed+1e-10)/model.dt)));
  const bool node_changed=z!=current_plan||k!=current_stage;
  if(node_changed){integral=0.;++intervals;current_plan=z;current_stage=k;}
  const auto&u=p.u[k];const double dt=t[i+1]-t[i];const auto a=relative(state[i],t[i]),b=relative(world.flow(state[i],u,.5*dt,false).x,t[i]+.5*dt),c=relative(state[i+1],t[i+1]);
  if(node_changed)executed_node_hard=std::max(executed_node_hard,hard(p,a,u));
  if(ctcs){executed_physical=std::max({executed_physical,physical(p,a,u,executed_terms),physical(p,b,u,executed_terms),physical(p,c,u,executed_terms)});integral+=dt*(density(p,a,u,k)+4*density(p,b,u,k)+density(p,c,u,k))/6.;executed_ratio=std::max(executed_ratio,integral);}
  executed_hard=std::max({executed_hard,hard(p,a,u),hard(p,b,u),hard(p,c,u)});
 }
 const bool audit_pass=native<=tolerance&&executed_node_hard<=tolerance&&(!ctcs||(planned_physical<=tolerance&&executed_physical<=tolerance&&planned_hard<=tolerance&&executed_hard<=tolerance));
 std::ofstream out(argv[2]);out<<std::setprecision(17)<<"{\"audit_tolerance\":"<<tolerance<<",\"saved_constraint_audit_pass\":"<<audit_pass<<",\"executed_node_hard_excess\":"<<executed_node_hard<<",\"plans\":"<<count<<",\"samples\":"<<samples<<",\"native_violation\":"<<native<<",\"planned_physical_STC_or_active_obstacle_excess\":"<<planned_physical<<",\"executed_physical_STC_or_active_obstacle_excess\":"<<executed_physical<<",\"planned_hard_excess\":"<<planned_hard<<",\"executed_hard_excess\":"<<executed_hard<<",\"native_budget_ratio\":"<<native_ratio<<",\"planned_dense_budget_ratio\":"<<dense_ratio<<",\"executed_budget_ratio\":"<<executed_ratio<<",\"planned_all_obstacles_diagnostic\":"<<planned_all_obstacles<<",\"dense_dynamics_defect\":"<<defect<<",\"executed_intervals\":"<<intervals;
 auto terms=[&](const char*name,const auto& map){out<<",\""<<name<<"\":{";bool first=true;for(const auto&[key,value]:map){if(!first)out<<',';first=false;out<<std::quoted(key)<<':'<<value;}out<<'}';};out<<",\"planned_peak_solve\":"<<peak_plan<<",\"planned_peak_horizon_time\":"<<peak_time<<",\"planned_peak_implication\":"<<std::quoted(peak_name);terms("planned_implications",planned_terms);terms("executed_implications",executed_terms);out<<"}\n";
 return audit_pass?0:2;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}

} // namespace saved_wall_audit

int main(int argc,char** argv){try{
 if(argc==4&&std::string(argv[1])=="--audit-saved")return saved_wall_audit::run(argc-1,argv+1);
 if(argc!=5&&argc!=6)throw std::invalid_argument("usage: test_wall_clock_rh quad|pointmass node|ctcs SECONDS OUTPUT [delay-compensated]");
 const bool quad=std::string(argv[1])=="quad",ctcs=std::string(argv[2])=="ctcs";
 if((!quad&&std::string(argv[1])!="pointmass")||(!ctcs&&std::string(argv[2])!="node"))throw std::invalid_argument("invalid model/mode");
 const bool compensated=argc==6&&std::string(argv[5])=="delay-compensated";
 if(argc==6&&!compensated)throw std::invalid_argument("unknown timing mode");
 const double seconds=std::stod(argv[3]);const std::filesystem::path out(argv[4]);
 if(seconds<=0||seconds>120||!std::filesystem::create_directories(out))throw std::invalid_argument("invalid duration/new output path");
 {std::ofstream profile(out/"profile.json");profile<<'{';bool first=true;
  for(char** e=environ;*e;++e){const std::string entry(*e);const auto equal=entry.find('=');const std::string key=entry.substr(0,equal);
   if(key.rfind("SZMUK_",0)&&key.rfind("STC_",0)&&key.rfind("MPC_",0)&&key.rfind("WALL_RH_",0))continue;
   if(!first)profile<<',';first=false;profile<<std::quoted(key)<<':'<<std::quoted(entry.substr(equal+1));
  }profile<<"}\n";}
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
 cfg.use_native_initial_guess=comando_parameters::envOrInt("WALL_RH_NATIVE_GUESS",0)!=0;
 if(compensated){cfg.initial_solve_lead_sec=comando_parameters::envOrQ("WALL_RH_INITIAL_LEAD",1.2);cfg.solve_lead_guard_sec=comando_parameters::envOrQ("WALL_RH_GUARD",.3);}
 planner_core::Controller controller;controller.configure(cfg,desc);
 const auto origin=Clock::now();std::atomic<bool> stop{false},busy{false};
 std::atomic<int> accepted{0},attempted{0},rejected{0};
 // Like the planner node: a rejected solve keeps the previous plan running.
 // 0 (default) aborts on the first rejection, as recorded runs did.
 const int max_consecutive_rejections=comando_parameters::envOrInt("WALL_RH_MAX_CONSECUTIVE_REJECTIONS",0);int consecutive_rejections=0;
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
    if(result.solve_accepted){++accepted;consecutive_rejections=0;}
    else {++rejected;if(++consecutive_rejections>max_consecutive_rejections){failure=result.rejection_reason;stop=true;}}
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
 }catch(const std::exception& e){failure=e.what();busy=false;stop=true;}});
 int samples_while_solving=0,stale_samples=0;bool reached=false;std::string peak_physical_term;double peak_physical_time=0.;double max_hard=0.,max_obstacle=0.,max_physical_stc=0.,tracking_squared=0.,max_tracking_error=0.;
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
      const auto note=[&](const char* name,double value){if(value>max_physical_stc){max_physical_stc=value;peak_physical_term=name;peak_physical_time=sim;}};
      note("loose_tilt",terms.loose_tilt);note("loose_omega",terms.loose_omega);
      if(terms.a_stage>0)note("staging",StcLandingNoAugOCP::Z_STAGE-rel(2));
      if(terms.a_los>0)note("los",terms.los);
      if(terms.a_altitude_state>0){note("landing_speed",terms.speed);note("landing_tilt",terms.tilt);note("landing_omega",terms.omega);note("glideslope",terms.glideslope);}
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
 std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\"model\":"<<std::quoted(argv[1])<<",\"ctcs\":"<<ctcs<<",\"runtime_complete\":"<<complete<<",\"reached\":"<<reached<<",\"simulation_time\":"<<sim<<",\"wall_seconds\":"<<wall<<",\"attempted\":"<<attempted.load()<<",\"accepted\":"<<accepted.load()<<",\"delay_compensated\":"<<compensated<<",\"measurement_to_root_change\":"<<max_root_error<<",\"measured_root_error\":"<<(compensated?std::string("null"):std::to_string(max_root_error))<<",\"max_request_age_seconds\":"<<max_request_age<<",\"plant_change_during_solve\":"<<max_state_change_during_solve<<",\"plant_steps_while_solving\":"<<samples_while_solving<<",\"stale_samples\":"<<stale_samples<<",\"native_violation\":"<<max_native<<",\"native_CTCS_budget_ratio\":"<<native_budget_ratio<<",\"physical_STC_excess\":"<<max_physical_stc<<",\"hard_constraint_excess\":"<<max_hard<<",\"obstacle_excess_diagnostic\":"<<max_obstacle<<",\"tracking_position_rmse\":"<<std::sqrt(tracking_squared/std::max(sim,.001))<<",\"max_tracking_error\":"<<max_tracking_error<<",\"rejected\":"<<rejected<<",\"physical_STC_peak_term\":"<<std::quoted(peak_physical_term)<<",\"physical_STC_peak_time\":"<<peak_physical_time<<",\"failure\":"<<std::quoted(failure)<<",\"continuous_CTCS_certified\":false}\n";
 std::cout<<"WALL_CLOCK accepted="<<accepted<<" attempted="<<attempted<<" measurement_to_root_change="<<max_root_error<<" moving_during_solve="<<max_state_change_during_solve<<" reached="<<reached<<" failure="<<failure<<'\n';return complete?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
