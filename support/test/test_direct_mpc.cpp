#include "ocp/ocp_stc_landing_mpc.hpp"
#include "ocp/ocp_landing_mpc.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "controller_internal.hpp"
#include "planner_core/adapters.hpp"
#include <iostream>
#include <thread>
#include <stdexcept>
#include <map>
#include "../tools/analysis/constraint_audit.hpp"
static void check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
// A diagnostic scalar normalization preserves the physical objective's
// minimizer. Apply the identical factor to its value and every derivative.
class AuditStageScale : public StageCostBase<double> {
    std::shared_ptr<StageCostBase<double>> cost_;double scale_;
public:
    AuditStageScale(std::shared_ptr<StageCostBase<double>> cost,double scale):cost_(std::move(cost)),scale_(scale){}
    double q(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->q(x,u);}
    Eigen::VectorXd qx(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->qx(x,u);}
    Eigen::VectorXd qu(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->qu(x,u);}
    Eigen::MatrixXd qxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->qxx(x,u);}
    Eigen::MatrixXd qxu(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->qxu(x,u);}
    Eigen::MatrixXd quu(const Eigen::VectorXd& x,const Eigen::VectorXd& u)const override{return scale_*cost_->quu(x,u);}
};
class AuditTerminalScale : public TerminalCostBase<double> {
    std::shared_ptr<TerminalCostBase<double>> cost_;double scale_;
public:
    AuditTerminalScale(std::shared_ptr<TerminalCostBase<double>> cost,double scale):cost_(std::move(cost)),scale_(scale){}
    double p(const Eigen::VectorXd& x)const override{return scale_*cost_->p(x);}
    Eigen::VectorXd px(const Eigen::VectorXd& x)const override{return scale_*cost_->px(x);}
    Eigen::MatrixXd pxx(const Eigen::VectorXd& x)const override{return scale_*cost_->pxx(x);}
};
int main(int argc,char** argv){try{
    if(argc==3&&std::string(argv[1])=="--open-loop"){
        using namespace constraint_audit;
        const std::string type=argv[2];
        check(type=="landing_mpc"||type=="landing_mpc_ctcs","open-loop audit requires a base MPC formulation");
        const auto& descriptor=OCPRegistry::getDescriptor(type);
        OCPCreateArgs args;args.current_state=Eigen::VectorXd::Zero(13);
        args.current_state(0)=.518;args.current_state(2)=1.85;args.current_state(6)=1;
        auto problem=descriptor.create(args);const auto model=descriptor.make_model(Eigen::Vector3d::Zero());
        auto parameters=descriptor.getSolverParams();
        // Diagnostic-only overrides, always printed with the result. Physical
        // settings continue to come from the same production OCP factory.
        parameters.max_inner_iter=comando_parameters::envOrInt("AUDIT_INNER",parameters.max_inner_iter);
        parameters.max_step_iter=comando_parameters::envOrInt("AUDIT_STEPS",parameters.max_step_iter);
        parameters.mu=parameters.muT=comando_parameters::envOrQ("AUDIT_MU",parameters.mu);
        parameters.inner_tolerance=comando_parameters::envOrQ("AUDIT_INNER_TOL",parameters.inner_tolerance);
        const double scale=comando_parameters::envOrQ("AUDIT_COST_SCALE",1.);
        check(std::isfinite(scale)&&scale>0,"cost scale must be positive and finite");
        if(scale!=1.){
            for(int k=0;k<model.horizon;++k)problem->setStageCost(k,std::make_shared<AuditStageScale>(problem->getStageCost(k),scale));
            problem->setTerminalCost(std::make_shared<AuditTerminalScale>(problem->getTerminalCost(),scale));
        }
        std::cout.precision(17);
        const auto& limits=LandingMpcOCP::Formulation::P;
        std::cout<<"CONFIG type="<<type<<" N="<<model.horizon<<" dt="<<model.dt
            <<" speed="<<limits.speed_max<<" tilt_deg="<<limits.tilt_scale*limits.tilt_max_deg
            <<" rate="<<limits.omega_max<<" gs_deg="<<limits.gs_cone_half_angle_deg
            <<" apex="<<limits.gs_apex_offset<<" los_deg="<<limits.los_cone_half_angle_deg
            <<" ctcs_budget="<<limits.ctcs_step_eps<<" tolerance="<<parameters.tolerance
            <<" max_iter="<<parameters.max_iter<<" inner="<<parameters.max_inner_iter
            <<" steps="<<parameters.max_step_iter<<" mu="<<parameters.mu
            <<" inner_tol="<<parameters.inner_tolerance<<" cost_scale="<<scale<<std::endl;
        std::vector<Eigen::VectorXd> seed_x,seed_u;
        for(int k=0;k<=model.horizon;++k)seed_x.push_back(problem->getInitialState(k));
        for(int k=0;k<model.horizon;++k)seed_u.push_back(problem->getInitialControl(k));
        const auto seed=auditPrimal(*problem,seed_x,seed_u,parameters);
        std::cout<<"SEED constraints="<<seed.constraints<<" dynamics="<<seed.dynamics<<std::endl;
        ALIPDDP<double> solver(*problem);solver.init(parameters);
        const auto start=std::chrono::steady_clock::now();solver.solve();
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::cout<<std::defaultfloat<<std::setprecision(17);
        const auto states=solver.getResX(),controls=solver.getResU();
        const auto primal=auditPrimal(*problem,states,controls,parameters);
        {
            auto adjoint=[&](bool central){
                Eigen::VectorXd costate=problem->getTerminalCost()->px(states.back());
                double maximum=0.;int offset=0;
                for(auto type:{ConstraintType::NO,ConstraintType::SOC})for(const auto& row:problem->getTerminalConstraints(type)){
                    const int n=row->getDimCT();costate+=row->cTx(states.back()).transpose()*solver.getTerminalY().segment(offset,n);offset+=n;
                }
                for(int k=model.horizon-1;k>=0;--k){
                    Eigen::VectorXd gx=problem->getStageCost(k)->qx(states[k],controls[k]);
                    Eigen::VectorXd gu=problem->getStageCost(k)->qu(states[k],controls[k]);offset=0;
                    for(auto type:{ConstraintType::NO,ConstraintType::SOC})for(const auto& row:problem->getStageConstraints(k,type)){
                        const int n=row->getDimC();Eigen::VectorXd y=solver.getYTraj()[k].segment(offset,n);
                        if(central&&type==ConstraintType::NO){
                            const auto c=row->c(states[k],controls[k]);
                            for(int j=0;j<n;++j)y(j)=c(j)<0?-solver.getParam().mu/c(j):std::numeric_limits<double>::quiet_NaN();
                        }
                        gx+=row->cx(states[k],controls[k]).transpose()*y;gu+=row->cu(states[k],controls[k]).transpose()*y;offset+=n;
                    }
                    gu+=problem->getDynamics(k)->fu(states[k],controls[k]).transpose()*costate;
                    costate=gx+problem->getDynamics(k)->fx(states[k],controls[k]).transpose()*costate;
                    maximum=gu.allFinite()?std::max(maximum,gu.lpNorm<Eigen::Infinity>()):std::numeric_limits<double>::infinity();
                }
                return maximum;
            };
            double complementarity=0,min_slack=1e100,max_dual=0;
            for(int k=0;k<model.horizon;++k)if(!solver.getSTraj()[k].isZero(0)){
                min_slack=std::min(min_slack,solver.getSTraj()[k].minCoeff());max_dual=std::max(max_dual,solver.getYTraj()[k].lpNorm<Eigen::Infinity>());
                if(type=="landing_mpc_ctcs")complementarity=std::max(complementarity,
                    (solver.getSTraj()[k].cwiseProduct(solver.getYTraj()[k]).array()-solver.getParam().mu).abs().maxCoeff());
            }
            std::cout<<"INDEPENDENT lagrangian_gradient="<<adjoint(false)<<" barrier_gradient="<<adjoint(true)
                <<" complementarity="<<complementarity<<" min_slack="<<min_slack<<" max_dual="<<max_dual<<" final_mu="<<solver.getParam().mu<<'\n';
            if(const char* file=std::getenv("AUDIT_DUMP")){
                std::ofstream dump(file);dump<<std::setprecision(17)<<model.horizon<<' '<<model.nx<<' '<<model.nu<<'\n';
                for(const auto& x:states)dump<<x.transpose()<<'\n';for(const auto& u:controls)dump<<u.transpose()<<'\n';
            }
        }

        Stats root,nodes,local,rollout;root.add(residuals(states.front(),true),0);
        auto continuous=states.front();double defect=0;
        for(size_t k=0;k<controls.size();++k){
            nodes.add(residuals(states[k+1],true),0);auto point=states[k];
            for(double t=0;t<model.dt-1e-12;){
                const double h=std::min(.001,model.dt-t);
                point=model.flow(point,controls[k],h,false).x;
                continuous=model.flow(continuous,controls[k],h,false).x;
                local.add(residuals(point,true),h);rollout.add(residuals(continuous,true),h);t+=h;
            }
            defect=std::max(defect,(point.head(13)-states[k+1].head(13)).lpNorm<Eigen::Infinity>());
        }
        bool physical=true;
        for(size_t j=0;j<NC;++j)physical=physical&&root.maxima[j]<=tolerances[j]&&nodes.maxima[j]<=tolerances[j]
            &&local.maxima[j]<=tolerances[j]&&rollout.maxima[j]<=tolerances[j];
        const bool converged=solver.getTerminationReason()=="Optimal Solution"&&std::isfinite(solver.getOptError())
            &&solver.getOptError()<=parameters.tolerance;
        const bool progress=states.back().head<3>().norm()<states.front().head<3>().norm()-.05;
        const bool pass=converged&&primal.feasible(1e-6)&&physical&&progress;
        std::cout<<"RESULT termination="<<solver.getTerminationReason()<<" iterations="<<solver.getNumIter()
            <<" optimality="<<solver.getOptError()<<" internal_error="<<solver.getError()
            <<" constraints="<<primal.constraints<<" dynamics="<<primal.dynamics<<" dense_defect="<<defect
            <<" distance="<<states.front().head<3>().norm()<<" -> "<<states.back().head<3>().norm()
            <<" solve_ms="<<ms<<" converged="<<converged<<" physical="<<physical<<" pass="<<pass<<'\n';
        for(const auto& entry:{std::make_pair("root",root),std::make_pair("future_nodes",nodes),
                std::make_pair("dense_intervals",local),std::make_pair("continuous_rollout",rollout)}){
            std::cout<<entry.first<<' ';entry.second.write(std::cout);std::cout<<'\n';
        }
        return pass?0:1;
    }
    if(argc==3&&std::string(argv[1])=="--synchronous"){
        // Zero-deviation closed loop: each 0.1 s solve receives the exact plant
        // state, its u0 is held for 100 x 1 ms steps of the same relative
        // model, and the target moves at constant velocity with zero
        // acceleration, so the relative flow is exact. No asynchronous timing.
        using namespace constraint_audit;
        const std::string type=argv[2];
        check(type=="landing_mpc"||type=="landing_mpc_ctcs","synchronous audit requires a base MPC formulation");
        const auto& descriptor=OCPRegistry::getDescriptor(type);
        const auto model=descriptor.make_model(Eigen::Vector3d::Zero());
        const int steps=static_cast<int>(std::lround(model.dt/.001));
        check(std::abs(model.dt-.1)<1e-12&&std::abs(steps*.001-model.dt)<1e-12,"synchronous audit requires fixed 0.1 s intervals");
        // Same optional diagnostic overrides as --open-loop.
        auto parameters=descriptor.getSolverParams();
        parameters.max_inner_iter=comando_parameters::envOrInt("AUDIT_INNER",parameters.max_inner_iter);
        parameters.max_step_iter=comando_parameters::envOrInt("AUDIT_STEPS",parameters.max_step_iter);
        parameters.mu=parameters.muT=comando_parameters::envOrQ("AUDIT_MU",parameters.mu);
        parameters.inner_tolerance=comando_parameters::envOrQ("AUDIT_INNER_TOL",parameters.inner_tolerance);
        const double scale=comando_parameters::envOrQ("AUDIT_COST_SCALE",1.);
        check(std::isfinite(scale)&&scale>0,"cost scale must be positive and finite");
        const int max_solves=comando_parameters::envOrInt("AUDIT_MPC_SOLVES",400);
        check(max_solves>0&&max_solves<=400,"AUDIT_MPC_SOLVES must lie in [1,400] (40 s)");
        const bool fresh=comando_parameters::envOrInt("AUDIT_FRESH_SOLVER",0)!=0;
        const int warm_iterations=StcLandingMpcOCP::Formulation::P.warm_iterations;
        std::ofstream dump;if(const char* file=std::getenv("AUDIT_DUMP"))dump.open(file);
        if(dump.is_open())dump<<std::setprecision(17);
        const Eigen::Vector3d target_velocity=.455*Eigen::Vector3d(std::cos(.6),std::sin(.6),0);
        std::cout.precision(17);
        const auto& limits=LandingMpcOCP::Formulation::P;
        std::cout<<"CONFIG type="<<type<<" synchronous=1 N="<<model.horizon<<" dt="<<model.dt<<" plant_step=.001"
            <<" solver="<<(fresh?"fresh":"retained")<<" warm_iterations="<<warm_iterations<<" max_solves="<<max_solves
            <<" target_velocity="<<target_velocity.transpose()<<" speed="<<limits.speed_max
            <<" tilt_deg="<<limits.tilt_scale*limits.tilt_max_deg<<" rate="<<limits.omega_max
            <<" gs_deg="<<limits.gs_cone_half_angle_deg<<" apex="<<limits.gs_apex_offset
            <<" los_deg="<<limits.los_cone_half_angle_deg<<" ctcs_budget="<<limits.ctcs_step_eps
            <<" tolerance="<<parameters.tolerance<<" max_iter="<<parameters.max_iter<<" inner="<<parameters.max_inner_iter
            <<" steps="<<parameters.max_step_iter<<" mu="<<parameters.mu<<" inner_tol="<<parameters.inner_tolerance
            <<" cost_scale="<<scale<<std::endl;
        // Signed worst row value: negative is margin, positive is violation.
        auto rowValue=[](const Eigen::VectorXd& c,ConstraintType kind){
            if(!c.allFinite())return std::numeric_limits<double>::infinity();
            if(c.size()==0)return -std::numeric_limits<double>::infinity();
            if(kind==ConstraintType::SOC)return c(0)+c.tail(c.size()-1).norm();
            return kind==ConstraintType::EQ?c.cwiseAbs().maxCoeff():c.maxCoeff();
        };
        auto stageRow=[&](const OptimalControlProblem<double>& p,int k,const Eigen::VectorXd& x,const Eigen::VectorXd& u){
            double value=-std::numeric_limits<double>::infinity();
            for(auto kind:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
                for(const auto& row:p.getStageConstraints(k,kind))value=std::max(value,rowValue(row->c(x,u),kind));
            return value;
        };
        auto terminalRow=[&](const OptimalControlProblem<double>& p,const Eigen::VectorXd& x){
            double value=-std::numeric_limits<double>::infinity();
            for(auto kind:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
                for(const auto& row:p.getTerminalConstraints(kind))value=std::max(value,rowValue(row->cT(x),kind));
            return value;
        };
        // Hold one input over an interval in 1 ms steps, sampling every step.
        auto hold=[&](Eigen::VectorXd x,const Eigen::VectorXd& u,Stats& stats,std::vector<Eigen::VectorXd>* samples=nullptr){
            for(int i=0;i<steps;++i){
                x=model.flow(x,u,.001,false).x;stats.add(residuals(x,true),.001);
                if(samples)samples->push_back(x);
            }
            return x;
        };
        auto exceeds=[](const Stats& stats){
            for(size_t j=0;j<NC;++j)if(!(stats.maxima[j]<=tolerances[j]))return true;
            return false;
        };
        auto maxima=[](std::ostream& out,const Stats& stats){for(size_t j=0;j<NC;++j)out<<(j?",":"")<<stats.maxima[j];};
        auto trajectory=[&](const char* tag,const std::vector<Eigen::VectorXd>& rows){
            if(!dump.is_open())return;
            for(size_t k=0;k<rows.size();++k)dump<<tag<<' '<<k<<' '<<rows[k].transpose()<<'\n';
        };

        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);physical(0)=.518;physical(2)=1.85;physical(6)=1;
        // The plant carries the full model state: physical 13, clock and the
        // last applied input as command memory (plus native states, if any).
        Eigen::VectorXd plant=model.initial_state(physical,model.hover);
        TargetSnapshot target;target.valid=true;target.velocity=target_velocity;
        TargetSnapshot previous_target=target;
        OcpSolver::Result previous;bool have_previous=false;
        std::unique_ptr<ALIPDDP<double>> solver;
        Stats roots,executed,future_nodes,future_dense,seed_tail_dense;
        double seed_constraints=0,seed_dynamics=0,shift_gap=0,seed_tail_stage=-1e100,seed_tail_terminal=-1e100;
        double returned_constraints=0,returned_dynamics=0,max_optimality=0,first_row=-1e100,tail_row=-1e100,terminal_row=-1e100;
        int solves=0,converged_count=0,future_violations=0;bool finished=false;std::string failure;
        for(int j=0;j<max_solves&&!finished&&failure.empty();++j){
            target.position=target_velocity*(j*model.dt);
            // Measured root: exact plant state, clock rebased to the new epoch.
            Eigen::VectorXd root=plant;root(13)=0;
            Stats root_stats;root_stats.add(residuals(root,true),0);roots.add(residuals(root,true),0);
            OCPCreateArgs args;args.current_state=root;args.target_accel=target.acceleration;
            OcpSolver::WarmStartSnapshot warm;double gap=std::numeric_limits<double>::quiet_NaN();
            if(have_previous){
                // Core shift with age = DT: node k samples the old trajectory at
                // (k+1)DT, and the new terminal node integrates the old final
                // control for one further DT. The factory then fixes the
                // measured root, rebases the clock to k*DT and sets each node's
                // command memory to the preceding seed control.
                warm=planner_core::shiftMpcSeed(previous,model,previous_target,target,model.dt);
                args.prev_X=warm.prev_X;args.prev_U=warm.prev_U;
                gap=(warm.prev_X.front().head(13)-root.head(13)).lpNorm<Eigen::Infinity>();
                shift_gap=std::max(shift_gap,gap);
            }
            auto problem=descriptor.create(args);
            if(scale!=1.){
                for(int k=0;k<model.horizon;++k)problem->setStageCost(k,std::make_shared<AuditStageScale>(problem->getStageCost(k),scale));
                problem->setTerminalCost(std::make_shared<AuditTerminalScale>(problem->getTerminalCost(),scale));
            }
            std::vector<Eigen::VectorXd> seed_x,seed_u;
            for(int k=0;k<=model.horizon;++k)seed_x.push_back(problem->getInitialState(k));
            for(int k=0;k<model.horizon;++k)seed_u.push_back(problem->getInitialControl(k));
            const auto seed=auditPrimal(*problem,seed_x,seed_u,parameters);
            seed_constraints=std::max(seed_constraints,seed.constraints);seed_dynamics=std::max(seed_dynamics,seed.dynamics);
            // New tail: last seed interval and terminal node.
            const int last=model.horizon-1;
            const double tail_stage=stageRow(*problem,last,seed_x[last],seed_u[last]),tail_terminal=terminalRow(*problem,seed_x.back());
            seed_tail_stage=std::max(seed_tail_stage,tail_stage);seed_tail_terminal=std::max(seed_tail_terminal,tail_terminal);
            Stats tail_dense;hold(seed_x[last],seed_u[last],tail_dense);
            for(size_t i=0;i<NC;++i){seed_tail_dense.maxima[i]=std::max(seed_tail_dense.maxima[i],tail_dense.maxima[i]);seed_tail_dense.duration[i]+=tail_dense.duration[i];}
            seed_tail_dense.samples+=tail_dense.samples;seed_tail_dense.total+=tail_dense.total;

            const bool retained=!fresh&&solver;
            const int budget=retained?warm_iterations:parameters.max_iter;
            std::ostringstream record;record<<std::setprecision(17)<<"SOLVE "<<j<<" t="<<j*model.dt
                <<" mode="<<(retained?"retained":fresh&&have_previous?"fresh_warm":"cold")<<" budget="<<budget
                <<" root_distance="<<root.head<3>().norm()<<" root_speed="<<root.segment<3>(3).norm()<<" root_physical=";
            maxima(record,root_stats);
            record<<" seed_constraints="<<seed.constraints<<" seed_dynamics="<<seed.dynamics<<" shift_root_gap="<<gap
                <<" seed_tail_stage_row="<<tail_stage<<" seed_tail_terminal_row="<<tail_terminal<<" seed_tail_dense=";
            maxima(record,tail_dense);
            std::vector<Eigen::VectorXd> states,controls,samples;
            try{
                if(retained)solver->updateProblem(*problem,warm_iterations);
                else{solver=std::make_unique<ALIPDDP<double>>(*problem);solver->init(parameters);}
                const auto start=std::chrono::steady_clock::now();solver->solve();
                const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                ++solves;states=solver->getResX();controls=solver->getResU();
                const auto primal=auditPrimal(*problem,states,controls,parameters);
                const bool converged=solver->getTerminationReason()=="Optimal Solution"&&std::isfinite(solver->getOptError())
                    &&solver->getOptError()<=parameters.tolerance;
                converged_count+=converged;
                returned_constraints=std::max(returned_constraints,primal.constraints);returned_dynamics=std::max(returned_dynamics,primal.dynamics);
                max_optimality=std::max(max_optimality,solver->getOptError());
                record<<" termination=\""<<solver->getTerminationReason()<<"\" iterations="<<solver->getNumIter()
                    <<" optimality="<<solver->getOptError()<<" internal_error="<<solver->getError()<<" final_mu="<<solver->getParam().mu
                    <<" converged="<<converged<<" constraints="<<primal.constraints<<" dynamics="<<primal.dynamics<<" solve_ms="<<ms;
                if(!primal.feasible(1e-6))failure="returned_primal";
                else{
                    const double first=stageRow(*problem,0,states[0],controls[0]),tail=stageRow(*problem,last,states[last],controls[last]);
                    const double terminal=terminalRow(*problem,states.back());
                    first_row=std::max(first_row,first);tail_row=std::max(tail_row,tail);terminal_row=std::max(terminal_row,terminal);
                    // Future audit of the returned plan: nodes and every interval
                    // densely from its own node. Reported, not a stop criterion.
                    Stats nodes,dense;
                    for(int k=0;k<model.horizon;++k){nodes.add(residuals(states[k+1],true),0);hold(states[k],controls[k],dense);}
                    for(size_t i=0;i<NC;++i){
                        future_nodes.maxima[i]=std::max(future_nodes.maxima[i],nodes.maxima[i]);future_dense.maxima[i]=std::max(future_dense.maxima[i],dense.maxima[i]);
                        future_nodes.duration[i]+=nodes.duration[i];future_dense.duration[i]+=dense.duration[i];
                    }
                    future_nodes.samples+=nodes.samples;future_dense.samples+=dense.samples;future_dense.total+=dense.total;
                    future_violations+=exceeds(nodes)||exceeds(dense);
                    // Execute exactly u0, unclipped, on the 1 ms plant.
                    const Eigen::VectorXd u0=controls.front();Stats interval;
                    plant=hold(root,u0,interval,&samples);
                    for(size_t i=0;i<NC;++i){executed.maxima[i]=std::max(executed.maxima[i],interval.maxima[i]);executed.duration[i]+=interval.duration[i];}
                    executed.samples+=interval.samples;executed.total+=interval.total;
                    record<<" first_interval_row="<<first<<" tail_row="<<tail<<" terminal_row="<<terminal
                        <<" u0="<<u0.transpose().format(Eigen::IOFormat(17,Eigen::DontAlignCols,",",","))<<" future_nodes=";
                    maxima(record,nodes);record<<" future_dense=";maxima(record,dense);record<<" executed=";maxima(record,interval);
                    record<<" end_distance="<<plant.head<3>().norm()<<" end_speed="<<plant.segment<3>(3).norm();
                    if(exceeds(interval))failure="executed_physical";
                    finished=plant.head<3>().norm()<.003&&plant.segment<3>(3).norm()<.01;
                }
            }catch(const std::exception& e){failure=std::string("exception: ")+e.what();record<<" exception=\""<<e.what()<<'"';}
            if(dump.is_open())dump<<record.str()<<'\n';
            if(!failure.empty()){
                std::cout<<"FAIL solve="<<j<<" reason="<<failure<<'\n'<<record.str()<<'\n';
                // First failing solve: seed, returned primal and executed samples.
                trajectory("FAIL_SEED_X",seed_x);trajectory("FAIL_SEED_U",seed_u);
                trajectory("FAIL_X",states);trajectory("FAIL_U",controls);trajectory("FAIL_PLANT",samples);
                break;
            }
            previous.state_trajectory=states;previous.control_trajectory=controls;have_previous=true;previous_target=target;
        }
        if(failure.empty()&&!finished)failure="incomplete_max_solves";
        const bool pass=failure.empty();
        std::cout<<"RESULT solves="<<solves<<" finished="<<finished<<" time="<<solves*model.dt
            <<" distance="<<plant.head<3>().norm()<<" speed="<<plant.segment<3>(3).norm()
            <<" converged_solves="<<converged_count<<" max_optimality="<<max_optimality
            <<" seed_constraints="<<seed_constraints<<" seed_dynamics="<<seed_dynamics<<" shift_root_gap="<<shift_gap
            <<" seed_tail_stage_row="<<seed_tail_stage<<" seed_tail_terminal_row="<<seed_tail_terminal
            <<" returned_constraints="<<returned_constraints<<" returned_dynamics="<<returned_dynamics
            <<" first_interval_row="<<first_row<<" tail_row="<<tail_row<<" terminal_row="<<terminal_row
            <<" future_violation_solves="<<future_violations<<" failure="<<(pass?"none":failure)<<" pass="<<pass<<'\n';
        for(const auto& entry:{std::make_pair("roots",roots),std::make_pair("seed_tail_dense",seed_tail_dense),
                std::make_pair("future_nodes",future_nodes),std::make_pair("future_dense",future_dense),std::make_pair("executed",executed)}){
            std::cout<<entry.first<<' ';entry.second.write(std::cout);std::cout<<'\n';
        }
        return pass?0:1;
    }
    if(argc==3&&std::string(argv[1])=="--initial-guess"){
        // Reproduce a feasible near-cone start with tightened limits, without
        // conflating an inadmissible seed with a failed optimizer iteration.
        const std::string type=argv[2];
        check(type=="landing_mpc"||type=="landing_mpc_ctcs","initial-guess audit requires ordinary MPC or its CTCS transcription");
        const auto& descriptor=OCPRegistry::getDescriptor(type);
        OCPCreateArgs args;args.current_state=Eigen::VectorXd::Zero(13);
        args.current_state(0)=.518;args.current_state(2)=1.85;args.current_state(6)=1;
        auto problem=descriptor.create(args);const auto model=descriptor.make_model(Eigen::Vector3d::Zero());
        std::vector<Eigen::VectorXd> states,controls;
        for(int k=0;k<=model.horizon;++k)states.push_back(problem->getInitialState(k));
        for(int k=0;k<model.horizon;++k)controls.push_back(problem->getInitialControl(k));
        const auto audit=auditPrimal(*problem,states,controls,descriptor.getSolverParams());
        double dense_excess=0;
        const auto rows=LandingMpcOCP::Ref::landingRows(model.nx,model.nu);
        for(int k=0;k<model.horizon;++k){
            auto point=states[k];
            for(int j=0;j<100;++j){
                point=model.flow(point,controls[k],model.dt/100,false).x;
                for(const auto& row:rows){
                    const auto c=row->c(point,controls[k]);
                    const double residual=c(0)+(row->getConstraintType()==ConstraintType::SOC?c.tail(c.size()-1).norm():0.);
                    dense_excess=std::max(dense_excess,residual);
                }
            }
        }
        std::cout<<type<<" initial-guess constraints="<<audit.constraints<<" dynamics="<<audit.dynamics
            <<" dense_excess="<<dense_excess<<" distance="<<states.front().head<3>().norm()<<" -> "<<states.back().head<3>().norm()<<'\n';
        check(audit.feasible(1e-6)&&dense_excess<=1e-6,"near-cone cold seed must obey the configured physical limits");
        check(states.back().head<3>().norm()<states.front().head<3>().norm()-.05,"feasible initial guess must make nontrivial landing progress");
        return 0;
    }
    check(argc==1,"usage: test_direct_mpc [--initial-guess|--open-loop|--synchronous landing_mpc|landing_mpc_ctcs]");
    {
        // A diagnostic must fail closed on invalid physical states, including
        // NaN quaternions that std::max(0, NaN) could otherwise hide.
        auto invalid=Eigen::VectorXd::Zero(13).eval();
        for(const auto& state:{invalid,Eigen::VectorXd::Constant(13,std::numeric_limits<double>::quiet_NaN()).eval()}){
            const auto values=constraint_audit::residuals(state,true);
            for(double value:values)check(std::isinf(value),"physical audit must reject invalid states");
        }
    }
    // A zero slack residual/returned trajectory must not certify an infeasible
    // primal. Check every cone type, terminal rows and exact-memory dynamics.
    {
        OptimalControlProblem<double> problem(1);Param parameters;
        problem.setInitialState(0,Eigen::VectorXd::Zero(1));
        problem.setInitialState(1,Eigen::VectorXd::Zero(1));
        problem.setInitialControl(0,Eigen::VectorXd::Zero(1));
        problem.setStageDynamics(std::make_shared<LinearDiscreteDynamics<double>>(Eigen::MatrixXd::Identity(1,1),Eigen::MatrixXd::Identity(1,1)));
        std::vector<Eigen::VectorXd> states(2,Eigen::VectorXd::Zero(1)),controls(1,Eigen::VectorXd::Zero(1));
        check(auditPrimal(problem,states,controls,parameters).feasible(1e-7),"feasible primal rejected");
        states[1](0)=.1;parameters.feasibility_ms_exact_rows={0};
        check(!auditPrimal(problem,states,controls,parameters).feasible(1e-7),"dynamics gap hidden by exact-row setting");
        states[1](0)=0;
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ}){
            auto p=problem;Eigen::VectorXd offset=Eigen::VectorXd::Constant(type==ConstraintType::SOC?2:1,.1);
            p.addStageConstraint(std::make_shared<LinearStageConstraint<double>>(Eigen::MatrixXd::Zero(offset.size(),1),Eigen::MatrixXd::Zero(offset.size(),1),offset,type));
            check(!auditPrimal(p,states,controls,parameters).feasible(1e-7),"stage constraint violation accepted");
            auto terminal=problem;terminal.addTerminalConstraint(std::make_shared<LinearTerminalConstraint<double>>(Eigen::MatrixXd::Zero(offset.size(),1),offset,type));
            check(!auditPrimal(terminal,states,controls,parameters).feasible(1e-7),"terminal constraint violation accepted");
        }
    }
    namespace O=StcLandingMpcOCP;
    {
        // Isolate CTCS from triggers and changed dynamics/costs. Six ordinary
        // path limits fold into one interval row; no nodal cones remain.
        namespace L=LandingMpcOCP;
        auto prediction=std::make_shared<L::Ref::ConstantAccelPredictor>();
        auto dynamics=std::make_shared<L::Dynamics>(prediction);
        if(!comando_parameters::defaults.mpc.native_px4){
            // Independent continuous-time reference for the actuator state.
            // This catches integration error that an FD Jacobian test cannot:
            // differentiating an inaccurate discrete map can still pass FD.
            Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);physical(6)=1;
            const auto root=L::Dynamics::initialState(physical,L::hover());
            auto command=L::hover();command.tail<3>().setConstant(L::rateLimit()/std::sqrt(3.));
            for(double duration:{L::DT,.037}){
                const auto propagated=dynamics->flow(root,command,duration,false).x;
                const Eigen::Vector3d exact=(1-std::exp(-duration/L::rateTau()))*command.tail<3>();
                check((propagated.segment<3>(10)-exact).norm()<1e-6,"ordinary MPC rate propagation must match the continuous actuator response");
            }
        }
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);
        physical(0)=.04;physical(1)=.03;physical(2)=1.85;physical(6)=1;physical(10)=.03;
        auto state=L::Dynamics::initialState(physical,L::hover());auto control=L::hover();control(1)=.03;
        {
            // Independent references for the shared base OCP: geometric
            // residuals and central differences do not call its Jacobians.
            auto probe=state;probe(0)=.4;probe(1)=.2;probe(2)=1.2;
            probe.segment<3>(3)<<.08,-.06,-.1;
            probe.segment<4>(6)<<.99,.02,-.03,.01;probe.segment<4>(6).normalize();
            probe.segment<3>(10)<<.004,-.003,.002;
            const auto rows=L::Ref::landingRows(L::NX,L::NU);
            const Eigen::Quaterniond orientation(probe(6),probe(7),probe(8),probe(9));
            const Eigen::Vector3d axis=orientation*Eigen::Vector3d::UnitZ();
            const double axial=axis.dot(probe.head<3>());
            const double tilt=2*std::asin(std::hypot(probe(7),probe(8)));
            const auto& limits=LandingMpcOCP::Formulation::P;
            const std::array<double,6> expected={-probe(2),probe.segment<3>(3).norm()/limits.speed_max-1,
                probe.segment<3>(10).norm()/limits.omega_max-1,
                probe.head<2>().norm()/std::tan(limits.gs_cone_half_angle_deg*M_PI/180)-probe(2)-limits.gs_apex_offset,
                std::sin(tilt/2)/std::sin(limits.tilt_scale*limits.tilt_max_deg*M_PI/360)-1,
                (probe.head<3>()-axial*axis).norm()-std::tan(limits.los_cone_half_angle_deg*M_PI/180)*axial};
            const auto fx=dynamics->fx(probe,control),fu=dynamics->fu(probe,control);
            L::Cost stage;L::Terminal terminal;
            for(size_t i=0;i<rows.size();++i){
                const auto c=rows[i]->c(probe,control);
                const double actual=c(0)+(rows[i]->getConstraintType()==ConstraintType::SOC?c.tail(c.size()-1).norm():0.);
                check(std::abs(actual-expected[i])<1e-10,"ordinary constraint sign and physical scaling");
            }
            for(int j=0;j<L::NX+L::NU;++j){
                auto xp=probe,xm=probe,up=control,um=control;const double h=1e-7;
                if(j<L::NX){xp(j)+=h;xm(j)-=h;}else{up(j-L::NX)+=h;um(j-L::NX)-=h;}
                const Eigen::VectorXd fd=(dynamics->f(xp,up)-dynamics->f(xm,um))/(2*h);
                const Eigen::VectorXd analytic=j<L::NX?fx.col(j).eval():fu.col(j-L::NX).eval();
                check((fd-analytic).norm()/(1+fd.norm())<1e-5,"ordinary dynamics Jacobian");
                const double cost_fd=(stage.q(xp,up)-stage.q(xm,um))/(2*h);
                const double cost_g=j<L::NX?stage.qx(probe,control)(j):stage.qu(probe,control)(j-L::NX);
                check(std::abs(cost_fd-cost_g)/(1+std::abs(cost_fd))<1e-5,"ordinary stage objective gradient");
                if(j<L::NX){
                    const double term_fd=(terminal.p(xp)-terminal.p(xm))/(2*h);
                    check(std::abs(term_fd-terminal.px(probe)(j))/(1+std::abs(term_fd))<1e-5,"ordinary terminal objective gradient");
                }
                for(const auto& row:rows){
                    const Eigen::VectorXd row_fd=(row->c(xp,up)-row->c(xm,um))/(2*h);
                    const Eigen::VectorXd row_g=j<L::NX?row->cx(probe,control).col(j).eval():row->cu(probe,control).col(j-L::NX).eval();
                    check((row_fd-row_g).norm()/(1+row_fd.norm())<1e-5,"ordinary constraint Jacobian");
                }
            }
        }
        auto folded=L::foldedLimits(dynamics);
        check(folded->getDimC()==1&&folded->numImplications()==6,"ordinary CTCS must fold all six limits into one row");
        check(folded->c(state,control)(0)>0,"always-active CTCS must detect an angular-rate violation above the STC trigger altitude");
        const auto& parameters=LandingMpcOCP::Formulation::P;
        const double rate_residual=.03/parameters.omega_max-1.;
        const double expected=L::DT*rate_residual*rate_residual/parameters.ctcs_step_eps-1.;
        check(std::abs(folded->c(state,control)(0)-expected)<1e-10,"folded CTCS must integrate the physical residual with the declared budget and scale");
        const auto cx=folded->cx(state,control),cu=folded->cu(state,control);
        for(int j=0;j<state.size()+control.size();++j){
            auto xp=state,xm=state,up=control,um=control;const double h=1e-7;
            if(j<state.size()){xp(j)+=h;xm(j)-=h;}else{up(j-state.size())+=h;um(j-state.size())-=h;}
            const double fd=(folded->c(xp,up)(0)-folded->c(xm,um)(0))/(2*h);
            const double analytic=j<state.size()?cx(0,j):cu(0,j-state.size());
            check(std::abs(fd-analytic)/(1+std::abs(fd))<1e-5,"ordinary folded CTCS interval Jacobian");
        }
        if(!comando_parameters::defaults.mpc.native_px4){
            // Rate dynamics are linear here, so these Hessian blocks have
            // an independent FD reference, including tangential norm curvature.
            auto curved_state=state;curved_state.segment<3>(10)<<.03,.015,.005;
            auto curved_control=control;curved_control.tail<3>()=curved_state.segment<3>(10);
            const auto hessian=folded->lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            const std::vector<int> indices={10,11,12,L::NX+1,L::NX+2,L::NX+3};
            auto gradient=[&](const auto& x,const auto& u){
                Eigen::VectorXd g(L::NX+L::NU);g.head(L::NX)=folded->cx(x,u).row(0).transpose();
                g.tail(L::NU)=folded->cu(x,u).row(0).transpose();return g;
            };
            for(int column:indices){
                auto xp=curved_state,xm=curved_state,up=curved_control,um=curved_control;const double h=1e-7;
                if(column<L::NX){xp(column)+=h;xm(column)-=h;}else{up(column-L::NX)+=h;um(column-L::NX)-=h;}
                const Eigen::VectorXd fd=(gradient(xp,up)-gradient(xm,um))/(2*h);
                for(int row:indices)check(std::abs(fd(row)-hessian(row,column))/(1+std::abs(fd(row)))<1e-5,"folded CTCS norm curvature");
            }
            check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(hessian).eigenvalues().minCoeff()/(1+hessian.norm())>-1e-12,"folded CTCS convex curvature must remain positive semidefinite");
            curved_state(0)=.4;curved_state(2)=.6;curved_state(3)=.9;curved_state(7)=.03;curved_state.segment<4>(6).normalize();
            const auto full=folded->lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            const auto gn=folded->StcStageConstraint<double>::lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>((full-gn).eval()).eigenvalues().minCoeff()/(1+full.norm()+gn.norm())>-1e-12,"one-pass CTCS curvature must preserve all original residual GN terms");
        }
        OCPCreateArgs args;args.current_state=state;
        auto problem=OCPRegistry::create("landing_mpc_ctcs",args);
        for(int k=0;k<L::N;++k){
            const auto rows=problem->getStageConstraints(k,ConstraintType::NO);
            check(rows.size()==2&&rows[0]->getDimC()==8&&rows[1]->getDimC()==1,"CTCS variant must retain only input bounds and one folded path row");
            check(problem->getStageConstraints(k,ConstraintType::SOC).empty(),"nodal SOC constraints must not conceal CTCS behavior");
        }
        check(problem->getTerminalConstraints(ConstraintType::NO).empty()&&problem->getTerminalConstraints(ConstraintType::SOC).empty(),"terminal path limits must also be supplied by the final CTCS interval");
        L::Cost cost;L::Terminal terminal;
        check(problem->getStageCost(0)->q(state,control)==cost.q(state,control),"CTCS comparison must preserve the ordinary stage cost");
        check(problem->getTerminalCost()->p(state)==terminal.p(state),"CTCS comparison must preserve the ordinary terminal cost");
    }
    auto pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();pred->a<<.1,-.1,0;
    auto dyn=std::make_shared<O::Dynamics>(pred);
    Eigen::VectorXd physical(13);physical<<.4,-.3,1.3,.2,.1,-.1,.99,.02,-.03,.01,.05,-.04,.02;
    Eigen::VectorXd x=O::Dynamics::initialState(physical,O::hover());x(13)=.4;
    x.segment<4>(6).normalize();auto u=O::hover();u.tail(3)<<.08,-.06,.02;
    auto model=OCPRegistry::getDescriptor("stc_landing_mpc").make_model(pred->a);
    {
        // A changed forecast must not move the old nominal world trajectory.
        // Include an off-grid age and extension beyond the old terminal node.
        OcpSolver::Result old;old.state_trajectory.push_back(x);
        for(int k=0;k<model.horizon;++k){
            old.control_trajectory.push_back(u);
            old.state_trajectory.push_back(model.flow(old.state_trajectory.back(),u,model.dt,false).x);
        }
        TargetSnapshot before,after;before.position<<1,2,.2;before.velocity<<.3,-.2,0;
        before.acceleration=pred->a;after.position<<1.1,1.8,.3;after.velocity<<.4,-.1,.02;
        after.acceleration<<-.2,.3,0;
        const double age=.037;
        const auto warm=planner_core::shiftMpcSeed(old,model,before,after,age);
        for(int k=0;k<=model.horizon;++k){
            const double time=age+k*model.dt,next=k*model.dt;
            const int i=std::min(static_cast<int>(time/model.dt),model.horizon);
            const auto previous=model.flow(old.state_trajectory[i],u,time-i*model.dt,false).x;
            const Eigen::Vector3d world_p=previous.head<3>()+before.position+before.velocity*time+.5*before.acceleration*time*time;
            const Eigen::Vector3d world_v=previous.segment<3>(3)+before.velocity+before.acceleration*time;
            check((warm.prev_X[k].head<3>()+after.position+after.velocity*next+.5*after.acceleration*next*next-world_p).norm()<1e-10,"MPC warm shift must preserve nominal world position");
            check((warm.prev_X[k].segment<3>(3)+after.velocity+after.acceleration*next-world_v).norm()<1e-10,"MPC warm shift must preserve nominal world velocity");
            check(std::abs(warm.prev_X[k](13)-next)<1e-12,"MPC warm clock must start at the new measurement epoch");
        }
        check((warm.prev_X.back().head(13)-old.state_trajectory.back().head(13)).norm()>1e-4,"MPC terminal warm state must advance beyond the old horizon");
    }
    auto direct=dyn->flow(x,u,O::DT,true);auto through_interface=model.flow(x,u,O::DT,true);
    check((direct.x-through_interface.x).norm()<1e-12 && (direct.a-through_interface.a).norm()<1e-12 && (direct.b-through_interface.b).norm()<1e-12,"MPC model interface must preserve dynamics and Jacobians");
    const auto a=dyn->fx(x,u), b=dyn->fu(x,u);double maxerr=0;double h=1e-6;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;
        if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        Eigen::VectorXd fd=(dyn->f(xp,up)-dyn->f(xm,um))/(2*h);
        Eigen::VectorXd analytic=j<O::NX? a.col(j).eval():b.col(j-O::NX).eval();
        const double error=(fd-analytic).norm();
        if(error>maxerr&&error>1e-5) {
            Eigen::Index row;const double peak=(fd-analytic).cwiseAbs().maxCoeff(&row);
            std::cerr<<"Jacobian column="<<j<<" row="<<row<<" error="<<peak<<" finite="<<fd(row)<<" analytic="<<analytic(row)<<'\n';
        }
        maxerr=std::max(maxerr,error);
    }
    check(maxerr<1e-5,"dynamics Jacobians");
    x(2)=.45; x.segment<3>(3)<<.6,.4,-.5;
    auto row=StcLandingNoAugOCP::makeCtcsConstraint<double>(dyn,StcLandingNoAugOCP::P.ctcs_step_eps/StcLandingNoAugOCP::P.ctcs_y_scale,-1,O::DT,O::substeps());
    auto cx=row->cx(x,u),cu=row->cu(x,u);
    double cerror=0;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        double fd=(row->c(xp,up)(0)-row->c(xm,um)(0))/(2*h);double analytic=j<O::NX?cx(0,j):cu(0,j-O::NX);cerror=std::max(cerror,std::abs(fd-analytic)/(1+std::abs(fd)));}
    check(cerror<1e-5,"CTCS Jacobians");
    {
        O::Cost cost;auto state=x;state(2)=.5*(O::Ref::P.z_stage+O::Ref::P.alt_trig);
        const auto gx=cost.qx(state,u),gu=cost.qu(state,u);
        for(int j=0;j<O::NX+O::NU;++j){auto xp=state,xm=state,up=u,um=u;
            if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
            const double fd=(cost.q(xp,up)-cost.q(xm,um))/(2*h);
            const double analytic=j<O::NX?gx(j):gu(j-O::NX);
            check(std::abs(fd-analytic)/(1+std::abs(fd))<1e-5,"landing rate-cost gradient");
        }
    }
    {
        // The touchdown residual has a product of altitude and velocity.
        // Check GN curvature against its independent residual Jacobian.
        using R=StcLandingNoAugOCP::CTcSTCStageCost<double>;
        const double weight=StcLandingNoAugOCP::P.w_touch_vel;
        R full(1.,0.,0.,0.,Eigen::Vector3d::Zero());
        Eigen::VectorXd state=Eigen::VectorXd::Zero(StcLandingNoAugOCP::NX_SS);
        state(2)=.1;state(3)=.6;state(4)=-.3;state(5)=-.4;state(6)=1;
        const auto input=Eigen::VectorXd::Zero(StcLandingNoAugOCP::NU_SS);
        const double touch=(StcLandingNoAugOCP::P.touch_alt-state(2))/StcLandingNoAugOCP::P.touch_scale;
        Eigen::MatrixXd j=Eigen::MatrixXd::Zero(3,state.size());
        j.block<3,1>(0,2)=-state.segment<3>(3)/StcLandingNoAugOCP::P.touch_scale;
        j.block<3,3>(0,3)=touch*Eigen::Matrix3d::Identity();
        Eigen::MatrixXd expected=2*weight*j.transpose()*j;
        expected.block<3,3>(3,3).diagonal().array()+=2;
        expected.block<3,3>(10,10).diagonal().array()+=2;
        check((full.qxx(state,input)-expected).norm()<1e-10,"RH touchdown GN curvature must match residual Jacobian");
        auto model_state=O::Dynamics::initialState(state,O::hover());O::Cost mpc_cost;
        check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(mpc_cost.qxx(model_state,O::hover())).eigenvalues().minCoeff()>-1e-8,"MPC GN curvature must remain positive semidefinite at touchdown");
    }
    Eigen::VectorXd odom=Eigen::VectorXd::Zero(13);odom(3)=1;odom(6)=std::sqrt(.5);odom(9)=std::sqrt(.5);odom(10)=.2;
    auto world=planner_core::bodyOdometryToWorld(odom);check(std::abs(world(4)-1)<1e-12&&std::abs(world(3))<1e-12&&world(10)==.2,"odometry frame conversion");
    check(std::abs(planner_core::normalizedThrust(9.81,.7)-.7)<1e-12&&planner_core::normalizedThrust(30,.7)==1,"thrust normalization");
    for(double requested:{0.,3.,6.,9.81,12.}) {
        const double signal=planner_core::normalizedThrust(requested,.7282821834,2,150./850);
        if(signal>0&&signal<1) {
            const double physical=9.81*std::pow((signal+150./850)/(.7282821834+150./850),2);
            check(std::abs(physical-requested)<1e-12,"motor thrust inverse must preserve DDP acceleration");
        }
    }
    O::SpeedBound speed;StcLandingNoAugOCP::GeneralSpeedCon<double> legacy_speed;
    check((speed.c(x,u)-legacy_speed.c(x.head(14),u)).norm()<1e-12,"speed cone convention");
    OCPCreateArgs args;args.current_state=x;args.target_accel=pred->a;auto problem=O::create(args);
    check(problem->getInitialControl(0).size()==4,"MPC control layout");
    check(problem->getStageConstraints(0,ConstraintType::NO).size()==2,
          "input bounds and CTCS must constrain the first predicted interval");
    check(problem->getStageConstraints(1,ConstraintType::SOC).size()==1,
          "CTCS MPC future speed bound must remain enforced");
    check(std::abs(dyn->f(x,u)(13)-x(13)-O::DT)<1e-12,"fixed time grid");
    check(O::descriptor().command_mode==OCPDescriptor::CommandMode::CmdBodyRate,"direct command type");
    {
        OCPCreateArgs hover_args;hover_args.current_state=Eigen::VectorXd::Zero(13);
        hover_args.current_state(2)=O::Ref::P.z_stage+.3;hover_args.current_state(6)=1;
        auto feasible=O::create(hover_args);
        for(double mu:{1.,.01,1e-6}){
            ALIPDDP<double> seeded(*feasible);
            auto parameters=O::descriptor().getSolverParams();parameters.mu=parameters.muT=mu;
            seeded.init(parameters);
            check(seeded.error<1e-7,"recentring feasible CTCS must not inject a barrier-dependent slack residual");
        }
    }
    if(comando_parameters::defaults.mpc.native_px4) {
        const auto ordinary=OCPRegistry::getDescriptor("landing_mpc").make_model(pred->a);
        check((ordinary.flow(x,u,.1,true).x-dyn->flow(x,u,.1,true).x).norm()<1e-12,"ordinary and CTCS must predict the same native plant");
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);physical(6)=1;
        auto hover_state=O::Dynamics::initialState(physical,O::hover());
        auto hover_pred=std::make_shared<O::Ref::ConstantAccelPredictor>();O::Dynamics native(hover_pred);
        const auto stationary=native.flow(hover_state,O::hover(),.5,false).x;
        check(stationary.head(13).isApprox(physical,1e-8),"source-derived native hover equilibrium must conserve physical state");
        hover_state.segment<3>(O::INTEGRAL)<<.006,-.004,.002;
        auto native_rates=native.intervalRates(hover_state,O::hover());
        check(native_rates.segment<4>(O::ROTOR).norm()>.01,"native integrator must affect actuator prediction");
        auto cancelling=O::hover();cancelling.tail<3>()=-hover_state.segment<3>(O::INTEGRAL);
        check(native.intervalRates(hover_state,cancelling).segment<4>(O::ROTOR).norm()<1e-10,"native integral state must enter the same PID equations used by PX4");
        check(native.flow(hover_state,O::hover(),.1,false).x.segment<3>(10).norm()>1e-3,"first-order zero-rate projection must not erase native integrator drift");
        OcpSolver::Config config;config.ocp_type="landing_mpc";OcpSolver solver(config);
        hover_state(2)=1.85;
        const auto solved=solver.solve(hover_state);
        check(solved.success,"native DDP solve must return a trajectory");
        const LandingMpcOCP::Ref::LandingLimits limits(O::NX,O::NU);
        double excess=0.;for(const auto& state:solved.state_trajectory)excess=std::max(excess,limits.c(state,O::hover()).maxCoeff());
        std::cout<<"native cold solve: error="<<solved.constraint_error<<" physical_excess="<<excess<<" ms="<<solved.solve_time_ms<<'\n';
        check(excess<1e-4,"native DDP must predict feasible physical states with nonzero rate integral");
        std::cout<<"PASS native dynamics="<<maxerr<<" ctcs="<<cerror<<'\n';return 0;
    }
    {
        // A failed initial solve must not seed the next measured-state problem
        // or switch it to the shorter warm iteration budget.
        const auto model=OCPRegistry::getDescriptor("landing_mpc").make_model(Eigen::Vector3d::Zero());
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);
        physical(0)=.04;physical(2)=1.85;
        physical(6)=std::cos(M_PI/8);physical(8)=std::sin(M_PI/8);
        OcpSolver::Config config;config.ocp_type="landing_mpc";config.reuse_solver=true;
        OcpSolver startup(config);
        check(!startup.solve(model.initial_state(physical,model.hover)).success,"infeasible first measured state must fail");
        const auto failed_cache=startup.snapshotWarmStart();
        check(failed_cache.prev_X.empty()&&failed_cache.prev_U.empty()&&failed_cache.prev_K.empty(),"failed primal must not enter the warm-start cache");
        physical(6)=1;physical(8)=0;
        check(startup.solve(model.initial_state(physical,model.hover)).success,"valid state after failed startup must receive a full initial solve");
    }
    planner_core::PlannerCoreConfig cfg;cfg.ocp_type="stc_landing_mpc";cfg.platform=planner_core::Platform::Mavros;cfg.enable_terminal_freeze=false;
    planner_core::DirectMpcController controller;controller.configure(cfg);
    planner_core::PlannerCoreInput input;input.current_state=Eigen::VectorXd::Zero(13);input.current_state<<1.98,0,StcLandingNoAugOCP::P.z_stage+0.3,0,0,0,1,0,0,0,0,0,0;
    input.target_snapshot.valid=true;input.target_snapshot.position<<0,0,.2;
    input.now=std::chrono::steady_clock::now();auto started=input.now;controller.trySolve(input);
    check(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<.02,"submission blocks command loop");
    planner_core::PlannerCoreStepResult solved;
    for(int i=0;i<2500;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));input.now=std::chrono::steady_clock::now();auto attempt=controller.trySolve(input);if(attempt.solve_attempted)solved=attempt;if(solved.solve_accepted)break;}
    std::cout<<"cold solve: accepted="<<solved.solve_accepted<<" error="<<solved.solve_result.constraint_error<<" ms="<<solved.solve_result.solve_time_ms<<'\n';
    check(solved.solve_accepted,"cold direct MPC solve rejected");
    check(std::isfinite(solved.solve_result.optimality_error),"solver optimality evidence must reach the core result");
    check(solved.solve_result.solver_converged==
        (solved.solve_result.solver_termination=="Optimal Solution"&&
         solved.solve_result.optimality_error<=O::descriptor().getSolverParams().tolerance&&
         solved.solve_result.primal_constraint_violation<=O::descriptor().getSolverParams().tolerance&&
         solved.solve_result.dynamics_error<=O::descriptor().getSolverParams().tolerance),
         "core convergence evidence must include termination, optimality and primal residuals");
    check(solved.x0_source=="measured","MPC must use measured initial state");
    Eigen::VectorXd expected=input.current_state;
    expected.head<3>()-=input.target_snapshot.position;
    expected.segment<3>(3)-=input.target_snapshot.velocity;
    check((solved.solve_result.state_trajectory.front().head(13)-expected).norm()<1e-10,
          "DDP root must equal measured state, without future projection");
    auto now=std::chrono::steady_clock::now();auto sample=controller.sample(now,input.current_state,input.target_snapshot);
    for(int i=0;i<500&&!sample.has_replay_sample;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));now=std::chrono::steady_clock::now();sample=controller.sample(now,input.current_state,input.target_snapshot);}
    check(sample.has_replay_sample&&!sample.stale&&sample.command.control.allFinite(),"DDP command sample");
    check(sample.solve_num==solved.solve_num,"first DDP result must be applied before the next solve");
    check((sample.command.control-solved.solve_result.control_trajectory.front()).norm()<1e-12,
          "MPC must apply first optimized input");
    auto held=controller.sample(now+std::chrono::milliseconds(250),input.current_state,input.target_snapshot);
    check((held.command.control-sample.command.control).norm()<1e-12,
          "publication must not advance through optimized trajectory");
    input.current_state.segment<4>(6)*=-1;auto flipped=controller.sample(now,input.current_state,input.target_snapshot);
    check((sample.command.control-flipped.command.control).norm()<1e-10,"quaternion sign must not alter DDP control");
    check(controller.sample(now+std::chrono::seconds(5),input.current_state,input.target_snapshot).stale,"expired policy");
    controller.stop();
    // Each measured-state DDP solve supplies only its first input to PX4.
    for(const std::string type:{"stc_landing_mpc","landing_mpc"}) {
    cfg.ocp_type=type;
    planner_core::DirectMpcController moving;moving.configure(cfg);
    auto moving_pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();O::Dynamics plant(moving_pred);
    Eigen::VectorXd plant_state=O::Dynamics::initialState(Eigen::VectorXd::Zero(13),O::hover());plant_state(0)=-.6;plant_state(1)=-2;plant_state(2)=StcLandingNoAugOCP::P.z_stage+.1;plant_state(6)=1;plant_state.tail(4)=O::hover();
    const Eigen::Vector3d target_velocity(.134,.435,0);plant_state.segment<3>(3)=-target_velocity;
    if(type=="landing_mpc") {
        // Viable cone start: match the moving target's velocity before
        // engaging the ordinary all-time constraints, as in SITL staging.
        plant_state.head<3>()<<.04,.03,1.85;
        plant_state.segment<3>(3).setZero();
    }
    const double initial_distance=plant_state.head(3).norm();
    auto begin=std::chrono::steady_clock::now(),previous=begin;
    Eigen::VectorXd applied=O::hover();int accepted=0,ticks=0,recovery_updates=0;bool disturbed=false;double max_omega=0,max_warm_ms=0;
    std::map<int,std::vector<Eigen::VectorXd>> policies;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<5) {
        auto tick=std::chrono::steady_clock::now();double dt=std::chrono::duration<double>(tick-previous).count();previous=tick;
        plant_state=plant.flow(plant_state,applied,dt,false).x;
        double elapsed=std::chrono::duration<double>(tick-begin).count();input.now=tick;
        if(type=="landing_mpc"&&!disturbed&&elapsed>=1.) {
            plant_state(10)=.03;disturbed=true;
        }
        input.target_snapshot.velocity=target_velocity;input.target_snapshot.position=target_velocity*elapsed+Eigen::Vector3d(0,0,.2);
        input.current_state=plant_state.head(13);input.current_state.head(3)+=input.target_snapshot.position;input.current_state.segment<3>(3)+=target_velocity;
        auto solve=moving.trySolve(input);if(solve.solve_accepted){++accepted;if(disturbed)++recovery_updates;if(accepted>1)max_warm_ms=std::max(max_warm_ms,solve.solve_result.solve_time_ms);policies[solve.solve_num]=solve.solve_result.control_trajectory;
}
        auto feedback=moving.sample(tick,input.current_state,input.target_snapshot);
        if(solve.solve_attempted&&!solve.solve_accepted)
            std::cout<<"DDP rejected: error="<<solve.solve_result.constraint_error<<" reason="<<solve.rejection_reason<<'\n';
        if(feedback.stale)std::cout<<"DDP expired: accepted="<<accepted<<" ticks="<<ticks<<" state="<<plant_state.head(13).transpose()<<std::endl;
        check(!feedback.stale,"moving-target DDP policy expired");
        if(feedback.has_replay_sample){
            auto found=policies.find(feedback.solve_num);
            check(found!=policies.end(),"applied policy must come from a DDP solve");
            check((feedback.command.control-found->second.front()).norm()<1e-12,
                  "published control must equal the first optimized DDP control");
            applied=feedback.command.control;++ticks;
        }
        max_omega=std::max(max_omega,plant_state.segment<3>(10).norm());
        std::this_thread::sleep_until(tick+std::chrono::milliseconds(10));
    }
    moving.stop();std::cout<<type<<" moving model: accepted="<<accepted<<" ticks="<<ticks<<" distance="<<plant_state.head(3).norm()<<" omega="<<max_omega<<" max_warm_ms="<<max_warm_ms<<'\n';
    if(type=="landing_mpc")check(disturbed&&recovery_updates>=30,"measured rate excursion must not stop MPC updates");
    check(accepted>=30&&ticks>200&&plant_state.head(3).norm()<initial_distance,"moving-target closed loop");
    }
    std::cout<<"PASS dynamics="<<maxerr<<" ctcs="<<cerror<<" measured-state first-input DDP MPC, CTCS and ordinary moving targets\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
