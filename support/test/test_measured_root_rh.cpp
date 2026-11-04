// Frozen simulation-time measured-root RH, through the public full-trajectory
// interface. Original NODE OCPs, whole K-stage execution, fresh numerical solves.
#include "pointmass_reference_adapter.hpp"
#include "planner_core/controller.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include <filesystem>
#include <stdexcept>
#include <sstream>
#include <cstring>

static void requireRh(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static void ordinaryNodeContract(){
    namespace Q=StcLandingNoAugOCP;
    OCPCreateArgs args;args.current_state=Vec::Zero(Q::NX_SS);args.current_state(2)=1.85;args.current_state(6)=1.;args.target_accel=Eigen::Vector3d::Zero();
    const auto source=Q::descriptor().create(args),node=Q::ordinaryNodeProblem(*source);
    requireRh(source->getHorizon()==node->getHorizon()&&source->getTerminalCost()==node->getTerminalCost(),"ordinary NODE changed horizon or terminal cost");
    for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})requireRh(node->getTerminalConstraints(type).empty(),"ordinary NODE added a terminal constraint");
    for(int k=0;k<node->getHorizon();++k){
        requireRh(node->getDynamics(k)==source->getDynamics(k)&&node->getStageCost(k)==source->getStageCost(k),"ordinary NODE changed model or cost");
        requireRh((node->getInitialState(k)-source->getInitialState(k)).norm()==0.&&(node->getInitialControl(k)-source->getInitialControl(k)).norm()==0.,"ordinary NODE changed initial guess");
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ}){
            size_t index=0;
            for(const auto& row:source->getStageConstraints(k,type))if(!std::dynamic_pointer_cast<StcStageConstraint<double>>(row)){
                requireRh(index<node->getStageConstraints(k,type).size()&&node->getStageConstraints(k,type)[index++]==row,"ordinary NODE changed row or row order");
            }
            requireRh(index==node->getStageConstraints(k,type).size(),"ordinary NODE contains an extra row");
        }
    }
    Vec x=args.current_state,u=node->getInitialControl(0);x(2)=-.01;
    bool floor_rejected=false,speed_rejected=false;
    for(const auto& row:node->getStageConstraints(0,ConstraintType::NO))floor_rejected|=row->c(x,u).maxCoeff()>0.;
    x=args.current_state;x(3)=2*Q::SPD_PHASE0_MAX;
    for(const auto& row:node->getStageConstraints(0,ConstraintType::SOC)){
        const Vec c=row->c(x,u);speed_rejected|=c(0)+c.tail(c.size()-1).norm()>0.;
    }
    requireRh(floor_rejected&&speed_rejected,"ordinary NODE constraints are inactive");
    std::cout<<"ORDINARY_NODE_CONTRACT original_model_cost_guess_rows=1 no_STC=1 no_terminal_constraints=1 floor_active=1 global_speed_active=1\n";
}
// Diagnostic only: replay one saved, fully constructed request with console
// logging enabled. Numerical parameters and trajectory data are unchanged.
static void diagnoseSavedRequest(bool quad,const std::string& path){
    std::ifstream input(path);requireRh(bool(input),"saved request missing");
    OCPCreateArgs args;std::vector<Vec> prepared_x,prepared_u,returned_x,returned_u;
    std::string line;
    while(std::getline(input,line)){
        std::stringstream row(line);std::string label;row>>label;
        if(label=="epoch")row>>args.t_abs;
        if(label=="root"){args.current_state.resize(quad?14:4);for(int j=0;j<args.current_state.size();++j)row>>args.current_state(j);}
        if(label=="target_acceleration")for(int j=0;j<3;++j)row>>args.target_accel(j);
        if(label=="prepared_X"||label=="prepared_U"||label=="returned_X"||label=="returned_U"){
            auto& values=label=="prepared_X"?prepared_x:label=="prepared_U"?prepared_u:label=="returned_X"?returned_x:returned_u;
            int count;row>>count;
            for(int k=0;k<count;++k){std::getline(input,line);std::stringstream data(line);int dim;data>>dim;Vec value(dim);for(int j=0;j<dim;++j)data>>value(j);values.push_back(value);}
        }
    }
    auto descriptor=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor("landing_rh")):pointmassReferenceDescriptor(100,false,true);
    auto problem=descriptor->create(args);const int N=problem->getHorizon();
    requireRh(prepared_x.size()==size_t(N+1)&&prepared_u.size()==size_t(N),"saved request shape mismatch");
    for(int k=0;k<=N;++k)problem->setInitialState(k,prepared_x[k]);
    for(int k=0;k<N;++k)problem->setInitialControl(k,prepared_u[k]);
    auto parameters=descriptor->getSolverParams();parameters.verbose=true;
    ALIPDDP<double> solver(*problem);solver.init(parameters);
    const auto initial=auditPrimal(*problem,solver.getXTraj(),solver.getUTraj(),parameters);
    std::cout<<std::setprecision(17)<<"REPLAY_INITIAL max_iter="<<parameters.max_iter<<" max_inner="<<parameters.max_inner_iter
        <<" mu="<<parameters.mu<<" muT="<<parameters.muT<<" mu_min="<<parameters.mu_min<<" rho="<<parameters.rho
        <<" rhoT="<<parameters.rhoT<<" rho_max="<<parameters.rho_max<<" tolerance="<<parameters.tolerance
        <<" native="<<initial.constraints<<" dynamics="<<initial.dynamics<<'\n';
    solver.solve();
    const auto x=solver.getResX(),u=solver.getResU();
    const auto identical=[](const auto& a,const auto& b){
        if(a.size()!=b.size())return false;
        for(size_t k=0;k<a.size();++k)if(a[k].size()!=b[k].size()||std::memcmp(a[k].data(),b[k].data(),a[k].size()*sizeof(double)))return false;
        return true;
    };
    requireRh(identical(x,returned_x)&&identical(u,returned_u),"diagnostic replay differs from saved result");
    const auto final=auditPrimal(*problem,x,u,parameters);const auto& p=solver.getParam();
    std::cout<<"REPLAY_FINAL bit_identical=1 iter="<<solver.getNumIter()<<" cost_records="<<solver.getAllCost().size()
        <<" termination="<<solver.getTerminationReason()<<" mu="<<p.mu<<" muT="<<p.muT<<" rho="<<p.rho
        <<" rhoT="<<p.rhoT<<" error="<<solver.getError()<<" optimality="<<solver.getOptError()
        <<" native="<<final.constraints<<" dynamics="<<final.dynamics<<'\n';
}
static void measuredGuessContract(){
    auto descriptor=pointmassReferenceDescriptor(100,false,true);
    OCPCreateArgs args;args.current_state=Vec::Zero(NX);args.current_state(0)=-10;
    args.target_accel=Eigen::Vector3d::Zero();
    args.prev_X.push_back(args.current_state);
    Problem p;p.substeps=100;
    for(int k=0;k<p.N;++k){
        Vec u(NU);u<<.4,-.1;args.prev_U.push_back(u);
        args.prev_X.push_back(intervalFlow(p,args.prev_X.back(),u,false).xn);
    }
    const auto matched=descriptor->create(args);
    for(int k=0;k<=p.N;++k)
        requireRh((matched->getInitialState(k)-args.prev_X[k]).cwiseAbs().maxCoeff()==0.,"matched initial guess changed");
    args.current_state(0)+=.01;args.current_state(2)+=.02;
    const auto measured=descriptor->create(args);
    std::vector<Vec> x,u;
    for(int k=0;k<=p.N;++k)x.push_back(measured->getInitialState(k));
    for(int k=0;k<p.N;++k){u.push_back(measured->getInitialControl(k));
        requireRh((u.back()-args.prev_U[k]).norm()==0.,"measured-root rollout changed controls");}
    const auto audit=auditPrimal(*measured,x,u,descriptor->getSolverParams());
    requireRh((x.front()-args.current_state).norm()==0.&&audit.dynamics<=1e-12,"measured-root guess has a shooting defect");
    std::cout<<"MEASURED_GUESS_CONTRACT dynamics="<<audit.dynamics<<" native="<<audit.constraints<<'\n';
}
int main(int argc,char** argv){try{
    if(argc==2&&std::string(argv[1])=="--ordinary-node-contract"){ordinaryNodeContract();return 0;}
    if(argc==4&&std::string(argv[1])=="--diagnose-request"){
        requireRh(std::string(argv[2])=="quad"||std::string(argv[2])=="pointmass","unknown replay model");
        diagnoseSavedRequest(std::string(argv[2])=="quad",argv[3]);return 0;
    }
    if(argc==2&&std::string(argv[1])=="--root-consistency"){measuredGuessContract();return 0;}
    if(argc!=5&&argc!=6)throw std::invalid_argument("usage: test_measured_root_rh pointmass|quad matched|fine CYCLES OUTPUT [--native-initial-guess]");
    const bool native_guess=argc==6&&std::string(argv[5])=="--native-initial-guess";
    requireRh(argc==5||native_guess,"unknown initial-guess option");
    const bool quad=std::string(argv[1])=="quad",fine=std::string(argv[2])=="fine";
    requireRh(quad||std::string(argv[1])=="pointmass","unknown model");
    requireRh(fine||std::string(argv[2])=="matched","unknown plant");
    const int bound=std::stoi(argv[3]),K=4;
    const std::filesystem::path out(argv[4]);
    requireRh(bound>0&&bound<=40&&std::filesystem::create_directories(out),"invalid cycle bound/output");
    auto descriptor=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor("stc_landing_noaug_node")):
        pointmassReferenceDescriptor(100,false,true);
    const auto params=descriptor->getSolverParams();
    std::shared_ptr<OptimalControlProblem<double>> captured_problem;
    OCPCreateArgs captured_args;
    const auto create=descriptor->create;
    descriptor->create=[&](const OCPCreateArgs& args){
        captured_args=args;
        if(native_guess){captured_args.prev_X.clear();captured_args.prev_U.clear();captured_args.prev_K.clear();}
        captured_problem=create(captured_args);return captured_problem;
    };
    // Exact nonzero measured-root SH control already recorded by test_controller.
    const Eigen::Vector3d acceleration=quad?Eigen::Vector3d(.04306807449,.07358261346,0):Eigen::Vector3d::Zero();
    auto targetAt=[&](double t){TargetSnapshot target;target.valid=true;
        target.position=.5*acceleration*t*t;target.velocity=acceleration*t;target.acceleration=acceleration;return target;};
    auto model=descriptor->make_model(acceleration);
    const auto world_model=descriptor->make_model(Eigen::Vector3d::Zero());
    planner_core::PlannerCoreConfig config;config.ocp_type=descriptor->name;config.n_replay=K;config.enable_terminal_freeze=false;
    planner_core::Controller controller;controller.configureSynchronous(config,descriptor);
    Eigen::VectorXd plant=Eigen::VectorXd::Zero(quad?13:4);
    if(quad){plant<<-.07320069131,-.04604300079,1.832031775,.07864153001,-.004976944706,.008655228599,
        .999980267207,-.004416340294,.003462269278,-.00282379661,.01105888095,.001028963132,-.002000458539;
        plant.segment<4>(6).normalize();}else plant<<-10,0,0,0;
    if(model.initial_state)plant=model.initial_state(plant,model.hover);
    planner_core::PlannerCoreInput input;input.state_frame="world";input.target_snapshot.valid=true;
    double time=0.,root_error=0.,native_error=0.,executed_error=0.,predicted_dense_error=0.,prediction_error=0.;
    bool reached=false;int accepted=0;double anchor_difference=0.;
    std::ofstream log(out/"cycles.jsonl");log<<std::setprecision(17);
    std::ofstream executed(out/"executed.csv");executed<<std::setprecision(17)<<"t";
    for(int j=0;j<model.nx;++j)executed<<",x"<<j;executed<<'\n';
    auto physical=[&](const Eigen::VectorXd& x,const Eigen::VectorXd& u){
        if(quad){
            double excess=0.;
            for(auto type:{ConstraintType::NO,ConstraintType::SOC})
                for(const auto& row:captured_problem->getStageConstraints(0,type)){
                    const Vec c=row->c(x,u);
                    excess=std::max(excess,type==ConstraintType::SOC?c(0)+c.tail(c.size()-1).norm():c.maxCoeff());
                }
            return excess;
        }
        Problem p;double worst=0.;for(int i=0;i<NOBS;++i)worst=std::max(worst,obstacleG(p,x.head<2>(),i));
        return std::max(worst,u.norm()-p.umax);
    };
    const auto saveVectors=[](std::ostream& stream,const char* label,const auto& vectors){
        stream<<label<<' '<<vectors.size()<<'\n';for(const auto& v:vectors)stream<<v.size()<<' '<<v.transpose()<<'\n';
    };
    for(int cycle=0;cycle<bound&&!reached;++cycle){
        input.current_state=plant;input.absolute_time_sec=time;input.now=std::chrono::steady_clock::now();input.target_snapshot=targetAt(time);
        const auto measured=plant;
        auto result=controller.solveSynchronous(input);
        requireRh((plant-measured).norm()==0.,"plant advanced during frozen solve");
        std::ofstream request(out/("request-"+std::to_string(cycle)+".txt"));request<<std::setprecision(17);
        request<<"epoch "<<time<<"\nroot "<<captured_args.current_state.transpose()<<"\ntarget_acceleration "<<acceleration.transpose()<<'\n';
        std::vector<Eigen::VectorXd> prepared_x,prepared_u;
        if(captured_problem){for(int k=0;k<=model.horizon;++k)prepared_x.push_back(captured_problem->getInitialState(k));
            for(int k=0;k<model.horizon;++k)prepared_u.push_back(captured_problem->getInitialControl(k));}
        saveVectors(request,"prepared_X",prepared_x);saveVectors(request,"prepared_U",prepared_u);
        saveVectors(request,"returned_X",result.solve_result.state_trajectory);saveVectors(request,"returned_U",result.solve_result.control_trajectory);
        if(!result.solve_accepted&&captured_problem){
            request<<"rejected_native "<<result.solve_result.primal_constraint_violation<<" tolerance "<<params.tolerance<<'\n';
            for(int repeat=0;repeat<2;++repeat){
                ALIPDDP<double> fresh(*captured_problem);fresh.init(params);fresh.solve();
                const auto x=fresh.getResX(),u=fresh.getResU();const auto audit=auditPrimal(*captured_problem,x,u,params);
                double diff=0.;for(size_t k=0;k<x.size();++k)diff=std::max(diff,(x[k]-result.solve_result.state_trajectory[k]).lpNorm<Eigen::Infinity>());
                for(size_t k=0;k<u.size();++k)diff=std::max(diff,(u[k]-result.solve_result.control_trajectory[k]).lpNorm<Eigen::Infinity>());
                request<<"fresh_repeat "<<repeat<<" difference "<<diff<<" native "<<audit.constraints<<" dynamics "<<audit.dynamics<<" termination "<<fresh.getTerminationReason()<<'\n';
                saveVectors(request,"fresh_X",x);saveVectors(request,"fresh_U",u);
                std::cout<<"FAILED_REQUEST_FRESH repeat="<<repeat<<" difference="<<diff<<" native="<<audit.constraints<<std::endl;
            }
            request.flush();
        }
        requireRh(result.solve_accepted,"measured-root solve rejected");
        const auto& xs=result.solve_result.state_trajectory;const auto& us=result.solve_result.control_trajectory;
        const auto relative=planner_core::prepareOcpState(*descriptor,measured,input.target_snapshot);
        root_error=std::max(root_error,(xs.front().head(descriptor->state_dim)-relative.head(descriptor->state_dim)).lpNorm<Eigen::Infinity>());
        requireRh(root_error<=1e-12&&result.x0_source=="measured","root changed/projected/predicted");
        if(quad)requireRh(xs.front()(13)==0.,"quad per-solve time state was not reset");
        requireRh(result.path.states.size()==xs.size(),"public interface omitted full state trajectory");
        OCPCreateArgs args;args.current_state=relative;args.target_accel=acceleration;args.t_abs=time;
        auto problem=create(args);
        const auto audit=auditPrimal(*problem,xs,us,params);
        native_error=std::max(native_error,audit.constraints);
        requireRh(audit.feasible(params.tolerance),"returned NODE trajectory violates native constraints/dynamics");
        if(cycle==0){
            ALIPDDP<double> single(*problem);single.init(params);single.solve();
            const auto ax=single.getResX(),au=single.getResU();
            for(size_t k=0;k<xs.size();++k)anchor_difference=std::max(anchor_difference,(ax[k]-xs[k]).lpNorm<Eigen::Infinity>());
            for(size_t k=0;k<us.size();++k)anchor_difference=std::max(anchor_difference,(au[k]-us[k]).lpNorm<Eigen::Infinity>());
            requireRh(anchor_difference<=1e-12,"first RH solve differs from identical single-horizon control");
        }
        std::vector<double> durations;double horizon=0.;
        for(size_t k=0;k<us.size();++k){
            const double h=model.stage_duration?model.stage_duration(xs[k],us[k]):model.dt;
            requireRh(h>0.&&std::isfinite(h),"invalid optimized stage duration");durations.push_back(h);horizon+=h;
            const int n=std::max(1,int(std::ceil(h/.001)));auto state=xs[k];
            for(int q=0;q<n;++q){state=model.flow(state,us[k],h/n,false).x;predicted_dense_error=std::max(predicted_dense_error,physical(state,us[k]));}
        }
        double prefix=0.;
        for(int k=0;k<K;++k){
            const auto interval_start=plant;const int n=std::max(1,int(std::ceil(durations[k]/.001)));const double h=durations[k]/n;
            for(int q=0;q<n;++q){
                input.absolute_time_sec=time+prefix+q*h;input.current_state=plant;
                const auto sample=controller.sample(input);
                requireRh(sample.has_replay_sample&&!sample.stale&&sample.applied_control_index==k,"wrong stage/clock during prefix execution");
                requireRh((sample.command.control-us[k]).norm()<=1e-12,"controller changed planned ZOH input");
                plant=fine?world_model.flow(plant,us[k],h,false).x:world_model.flow(interval_start,us[k],(q+1)*h,false).x;
                executed_error=std::max(executed_error,physical(planner_core::prepareOcpState(*descriptor,plant,targetAt(time+prefix+(q+1)*h)),us[k]));
                executed<<time+prefix+(q+1)*h;for(int j=0;j<plant.size();++j)executed<<','<<plant(j);executed<<'\n';
            }
            // The plant's elapsed-time coordinate is global; the OCP clock is
            // reset each solve. Compare measured physical states only.
            const auto predicted_world=planner_core::reconstructWorldState(*descriptor,xs[k+1],targetAt(time+prefix+durations[k]));
            prediction_error=std::max(prediction_error,(plant.head(descriptor->state_dim)-predicted_world.head(descriptor->state_dim)).lpNorm<Eigen::Infinity>());
            if(!fine)requireRh(prediction_error<=params.tolerance,"model-matched independent plant disagrees at node");
            prefix+=durations[k];
        }
        time+=prefix;++accepted;input.absolute_time_sec=time;
        requireRh(controller.sample(input).stale,"completed prefix silently extended");
        const auto measured_relative=planner_core::prepareOcpState(*descriptor,plant,targetAt(time));
        reached=quad&&measured_relative.head<3>().norm()<=.1&&measured_relative.segment<3>(3).norm()<=.3;
        log<<"{\"cycle\":"<<cycle<<",\"epoch\":"<<time<<",\"executed_duration\":"<<prefix<<",\"horizon_duration\":"<<horizon
           <<",\"native_violation\":"<<audit.constraints<<",\"root_error\":"<<root_error<<",\"native_initial_guess\":"<<native_guess
           <<",\"optimality_error\":"<<result.solve_result.optimality_error
           <<",\"solve_ms\":"<<result.solve_result.solve_time_ms<<",\"iterations\":"<<result.solve_result.solve_iters<<",\"executed_physical_excess\":"<<executed_error
           <<",\"planned_dense_excess\":"<<predicted_dense_error<<",\"distance\":"<<plant.head(quad?3:2).norm()<<"}\n";log.flush();
        std::cout<<"MEASURED_RH model="<<(quad?"quad":"pointmass")<<" cycle="<<cycle<<" t="<<time<<" native="<<audit.constraints<<" executed="<<executed_error<<" planned_dense="<<predicted_dense_error<<std::endl;
    }
    const bool progress=quad?reached:(plant(0)>0.&&(plant.head<2>()-reference(time)).norm()<1.);
    const bool native_pass=accepted>0&&(quad?reached:accepted==bound)&&progress&&root_error<=1e-12&&native_error<=params.tolerance;
    // NODE acceptance covers shooting nodes. Dense checks remain diagnostics.
    const bool node_pass=native_pass;
    std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\"accepted\":"<<accepted<<",\"reached\":"<<reached<<",\"simulation_time\":"<<time
        <<",\"native_solve_pass\":"<<native_pass<<",\"node_loop_pass\":"<<node_pass<<",\"planned_continuous_pass\":"<<(predicted_dense_error<=1e-6)<<",\"planned_dense_excess\":"<<predicted_dense_error
        <<",\"executed_continuous_pass\":"<<(executed_error<=1e-6)<<",\"executed_physical_excess\":"<<executed_error<<",\"native_violation\":"<<native_error<<",\"root_error\":"<<root_error<<",\"anchor_difference\":"<<anchor_difference
        <<",\"prediction_error\":"<<prediction_error<<"}\n";
    return node_pass?0:2;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
