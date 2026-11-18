// Frozen simulation-time measured-root RH, through the public full-trajectory
// interface. Original NODE/CTCS OCPs, whole K-stage execution, fresh numerical solves.
#include "pointmass_reference_adapter.hpp"
#include "planner_core/controller.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include <filesystem>
#include <stdexcept>
#include <sstream>
#include <cstring>
#include <map>

extern char** environ;
static std::map<std::string,std::string> requestEnvironment(){
    std::map<std::string,std::string> values;
    for(char** entry=environ;*entry;++entry){
        const std::string value(*entry);const auto equal=value.find('=');
        const auto name=value.substr(0,equal);
        if(name.rfind("SZMUK_",0)==0||name.rfind("MPC_",0)==0||name=="OMP_NUM_THREADS"||name=="OPENBLAS_NUM_THREADS")
            values[name]=value.substr(equal+1);
    }
    return values;
}

static void requireRh(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
// Test-only tuning; production descriptors and solver equations are unchanged.
static void configureTestParameters(OCPDescriptor& descriptor){
    const auto original=descriptor.getSolverParams;
    descriptor.getSolverParams=[original]{
        auto p=original();
        p.mu=comando_parameters::envOrQ("SZMUK_TEST_MU",p.mu);p.muT=p.mu;
        p.mu_mul=comando_parameters::envOrQ("SZMUK_TEST_MU_MUL",p.mu_mul);
        requireRh(p.mu>0.&&p.mu_mul>0.&&p.mu_mul<1.,"invalid test barrier parameters");
        return p;
    };
}
// Fixed-duration test adapter: eliminate theta from the decision vector.
// Reuse the same quad equations, costs, ordinary rows and CTCS implications.
namespace FixedQuadTime {
namespace Q=StcLandingNoAugOCP;
static Vec lift(const Vec& u,double dt){
    requireRh(u.size()==Q::NU,"fixed-duration quad must have four controls");
    Vec result(Q::NU_SS);result.head(Q::NU)=u;result(Q::IDX_THETA)=dt;return result;
}
class Dynamics : public DiscreteDynamicsBase<double> {
    std::shared_ptr<Q::Quad6DOFVarTimeRelativePred<double>> original_;
    double duration_;
public:
    Dynamics(std::shared_ptr<Q::Quad6DOFVarTimeRelativePred<double>> original,double dt)
        :original_(std::move(original)),duration_(dt){this->dim_x=Q::NX_SS;this->dim_u=Q::NU;this->dt=dt;}
    Vec f(const Vec& x,const Vec& u) const override{return original_->f(x,lift(u,duration_));}
    Mat fx(const Vec& x,const Vec& u) const override{return original_->fx(x,lift(u,duration_));}
    Mat fu(const Vec& x,const Vec& u) const override{return original_->fu(x,lift(u,duration_)).leftCols(Q::NU);}
    Vec intervalRates(const Vec& x,const Vec& u) const{return original_->intervalRates(x,lift(u,duration_));}
    Mat intervalRatesX(const Vec& x,const Vec& u) const{return original_->intervalRatesX(x,lift(u,duration_));}
    Mat intervalRatesU(const Vec& x,const Vec& u) const{return original_->intervalRatesU(x,lift(u,duration_)).leftCols(Q::NU);}
};
class Cost : public StageCostBase<double> {
    std::shared_ptr<StageCostBase<double>> original_;double dt_;
public:
    Cost(std::shared_ptr<StageCostBase<double>> original,double dt):original_(std::move(original)),dt_(dt){}
    double q(const Vec& x,const Vec& u) const override{return original_->q(x,lift(u,dt_));}
    Vec qx(const Vec& x,const Vec& u) const override{return original_->qx(x,lift(u,dt_));}
    Vec qu(const Vec& x,const Vec& u) const override{return original_->qu(x,lift(u,dt_)).head(Q::NU);}
    Mat qxx(const Vec& x,const Vec& u) const override{return original_->qxx(x,lift(u,dt_));}
    Mat quu(const Vec& x,const Vec& u) const override{return original_->quu(x,lift(u,dt_)).topLeftCorner(Q::NU,Q::NU);}
    Mat qxu(const Vec& x,const Vec& u) const override{return original_->qxu(x,lift(u,dt_)).leftCols(Q::NU);}
};
class Row : public StageConstraintBase<double> {
    std::shared_ptr<StageConstraintBase<double>> original_;double dt_;
public:
    Row(std::shared_ptr<StageConstraintBase<double>> original,double dt):original_(std::move(original)),dt_(dt){
        this->constraint_type=original_->getConstraintType();this->dim_c=original_->getDimC();}
    Vec c(const Vec& x,const Vec& u) const override{return original_->c(x,lift(u,dt_));}
    Mat cx(const Vec& x,const Vec& u) const override{return original_->cx(x,lift(u,dt_));}
    Mat cu(const Vec& x,const Vec& u) const override{return original_->cu(x,lift(u,dt_)).leftCols(Q::NU);}
    Mat lagrangianHessian(const Vec& x,const Vec& u,const Vec& multiplier) const override{
        return original_->lagrangianHessian(x,lift(u,dt_),multiplier).topLeftCorner(x.size()+Q::NU,x.size()+Q::NU);}
};
static std::shared_ptr<OCPDescriptor> descriptor(const OCPDescriptor& original,double dt){
    requireRh(std::isfinite(dt)&&dt>0.&&dt>=Q::P.thl&&dt<=Q::P.thh,"fixed dt must lie in the original duration range");
    auto result=std::make_shared<OCPDescriptor>(original);result->name=original.name+"_fixed_dt";
    result->dt=dt;result->variable_dt=false;
    result->sanitize_warm_control=[dt](Vec& u){auto v=lift(u,dt);Q::clampWarmControl(v);u=v.head(Q::NU);};
    result->make_model=[make=original.make_model,dt](const Eigen::Vector3d& acceleration){
        auto model=make(acceleration);const auto flow=model.flow;
        model.nu=Q::NU;model.dt=dt;model.hover=model.hover.head(Q::NU).eval();
        model.stage_duration=[dt](const Vec&,const Vec&){return dt;};
        model.bounded=[dt](Vec u){auto v=lift(u,dt);Q::clampWarmControl(v);return Vec(v.head(Q::NU));};
        model.flow=[flow,dt](const Vec& x,const Vec& u,double h,bool jac){
            auto value=flow(x,lift(u,dt),h,jac);if(jac)value.b=value.b.leftCols(Q::NU).eval();return value;};
        return model;
    };
    result->create=[create=original.create,dt](const OCPCreateArgs& input){
        auto args=input;for(auto& u:args.prev_U)u=lift(u,dt);
        for(auto& k:args.prev_K){Mat full=Mat::Zero(Q::NU_SS,k.cols());full.topRows(Q::NU)=k;k=std::move(full);}
        const auto source=create(args);const int n=source->getHorizon();
        auto native=std::dynamic_pointer_cast<Q::Quad6DOFVarTimeRelativePred<double>>(source->getDynamics(0));
        requireRh(bool(native),"fixed dt requires the original noaug quad model");
        auto dynamics=std::make_shared<Dynamics>(native,dt);auto problem=std::make_shared<OptimalControlProblem<double>>(n);
        Vec state=source->getInitialState(0);problem->setInitialState(0,state);
        std::shared_ptr<StcStageConstraint<double>> fixed_interval;
        for(int k=0;k<n;++k){
            problem->setStageDynamics(k,dynamics);problem->setStageCost(k,std::make_shared<Cost>(source->getStageCost(k),dt));
            const Vec control=source->getInitialControl(k).head(Q::NU);problem->setInitialControl(k,control);
            state=dynamics->f(state,control);problem->setInitialState(k+1,state);
            for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
                for(const auto& row:source->getStageConstraints(k,type)){
                    if(std::dynamic_pointer_cast<Q::ThetaBounds<double>>(row))continue;
                    if(auto interval=std::dynamic_pointer_cast<StcStageConstraint<double>>(row)){
                        // Preserve the original shared-row/cache lifetime across stages.
                        if(!fixed_interval)fixed_interval=Q::makeCtcsConstraint<double>(dynamics,interval->getEpsilon(),-1,dt);
                        problem->addStageConstraint(k,fixed_interval);
                    }else problem->addStageConstraint(k,std::make_shared<Row>(row,dt));
                }
        }
        problem->setTerminalCost(source->getTerminalCost());
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
            for(const auto& row:source->getTerminalConstraints(type))problem->addTerminalConstraint(row);
        return problem;
    };
    return result;
}
}

static void fixedDurationContract(double dt){
    namespace Q=StcLandingNoAugOCP;
    const auto free=Q::descriptor();const auto fixed=FixedQuadTime::descriptor(free,dt);
    OCPCreateArgs args;args.current_state=Vec::Zero(Q::NX_SS);args.current_state(2)=1.85;args.current_state(6)=1.;
    args.target_accel=Eigen::Vector3d(.04306807449,.07358261346,0.);
    const auto a=free.create(args),b=fixed->create(args);double error=0.,fd_error=0.;
    const auto compare=[&](const Mat& x,const Mat& y){
        requireRh(x.rows()==y.rows()&&x.cols()==y.cols(),"fixed duration adapter shape mismatch");
        error=std::max(error,(x-y).cwiseAbs().maxCoeff()/(1.+x.cwiseAbs().maxCoeff()));
    };
    for(int sample=0;sample<3;++sample){
        Vec x=args.current_state,u(4);u<<Q::HOVER_THRUST,.001,-.002,.0005;
        if(sample){x(0)=.1;x(1)=.05;x(2)=sample==1?.4:1.79;x(3)=.2;x(7)=.02;x(8)=-.01;x.segment<4>(6).normalize();x(10)=.03;}
        const auto full=FixedQuadTime::lift(u,dt);
        requireRh(b->getDynamics(0)->getDimU()==4,"duration remains a decision variable");
        compare(a->getDynamics(0)->f(x,full),b->getDynamics(0)->f(x,u));
        compare(a->getDynamics(0)->fx(x,full),b->getDynamics(0)->fx(x,u));
        compare(a->getDynamics(0)->fu(x,full).leftCols(4),b->getDynamics(0)->fu(x,u));
        requireRh(std::abs(a->getStageCost(0)->q(x,full)-b->getStageCost(0)->q(x,u))<1e-12,"fixed duration changed objective");
        compare(a->getStageCost(0)->qx(x,full),b->getStageCost(0)->qx(x,u));
        compare(a->getStageCost(0)->qu(x,full).head(4),b->getStageCost(0)->qu(x,u));
        compare(a->getStageCost(0)->qxx(x,full),b->getStageCost(0)->qxx(x,u));
        compare(a->getStageCost(0)->quu(x,full).topLeftCorner(4,4),b->getStageCost(0)->quu(x,u));
        compare(a->getStageCost(0)->qxu(x,full).leftCols(4),b->getStageCost(0)->qxu(x,u));
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ}){
            size_t index=0;
            for(const auto& old:a->getStageConstraints(0,type)){
                if(std::dynamic_pointer_cast<Q::ThetaBounds<double>>(old))continue;
                const auto row=b->getStageConstraints(0,type).at(index++);
                compare(old->c(x,full),row->c(x,u));compare(old->cx(x,full),row->cx(x,u));
                compare(old->cu(x,full).leftCols(4),row->cu(x,u));
                compare(old->lagrangianHessian(x,full,Vec::Ones(old->getDimC())).topLeftCorner(x.size()+4,x.size()+4),row->lagrangianHessian(x,u,Vec::Ones(row->getDimC())));
                if(auto interval=std::dynamic_pointer_cast<StcStageConstraint<double>>(row)){
                    requireRh(interval->numImplications()==8,"fixed duration dropped CTCS implications");
                    for(int j=0;j<4;++j){auto plus=u,minus=u;plus(j)+=1e-6;minus(j)-=1e-6;
                        const double fd=(row->c(x,plus)(0)-row->c(x,minus)(0))/(2e-6);
                        fd_error=std::max(fd_error,std::abs(fd-row->cu(x,u)(0,j))/(1.+std::abs(fd)));}
                }
            }
            requireRh(index==b->getStageConstraints(0,type).size(),"fixed duration added a constraint");
            requireRh(a->getTerminalConstraints(type).size()==b->getTerminalConstraints(type).size(),"fixed duration changed terminal constraints");
        }
    }
    requireRh(error<1e-12&&fd_error<2e-5,"fixed duration adapter parity/Jacobian failed");
    std::cout<<"FIXED_DURATION_CONTRACT dt="<<dt<<" controls=4 independent_duration_decisions=0 parity="<<error<<" CTCS_FD_error="<<fd_error<<'\n';
}

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
    x=args.current_state;x(3)=2*Q::P.spd_phase0_max;
    for(const auto& row:node->getStageConstraints(0,ConstraintType::SOC)){
        const Vec c=row->c(x,u);speed_rejected|=c(0)+c.tail(c.size()-1).norm()>0.;
    }
    requireRh(floor_rejected&&speed_rejected,"ordinary NODE constraints are inactive");
    std::cout<<"ORDINARY_NODE_CONTRACT original_model_cost_guess_rows=1 no_STC=1 no_terminal_constraints=1 floor_active=1 global_speed_active=1\n";
}
// Diagnostic only: replay one saved, fully constructed request with console
// logging enabled. Numerical parameters and trajectory data are unchanged.
static void diagnoseSavedRequest(bool quad,const std::string& path,const std::string& legacy_ocp={}){
    std::ifstream input(path);requireRh(bool(input),"saved request missing");
    OCPCreateArgs args;std::vector<Vec> prepared_x,prepared_u,returned_x,returned_u;
    std::string line,ocp=legacy_ocp;std::map<std::string,std::string> environment;bool environment_saved=false;double fixed_dt=0.;
    while(std::getline(input,line)){
        std::stringstream row(line);std::string label;row>>label;
        if(label=="ocp")row>>ocp;
        if(label=="environment"){std::string name,value;row>>std::quoted(name)>>std::quoted(value);environment[name]=value;}
        if(label=="environment_end")environment_saved=true;
        if(label=="epoch")row>>args.t_abs;
        if(label=="horizon")row>>args.horizon;
        if(label=="fixed_dt")row>>fixed_dt;
        if(label=="root"){args.current_state.resize(quad?14:4);for(int j=0;j<args.current_state.size();++j)row>>args.current_state(j);}
        if(label=="target_acceleration")for(int j=0;j<3;++j)row>>args.target_accel(j);
        if(label=="prepared_X"||label=="prepared_U"||label=="returned_X"||label=="returned_U"){
            auto& values=label=="prepared_X"?prepared_x:label=="prepared_U"?prepared_u:label=="returned_X"?returned_x:returned_u;
            int count;row>>count;
            for(int k=0;k<count;++k){std::getline(input,line);std::stringstream data(line);int dim;data>>dim;Vec value(dim);for(int j=0;j<dim;++j)data>>value(j);values.push_back(value);}
        }
    }
    requireRh(!ocp.empty(),"legacy request has no OCP metadata; specify the actual descriptor explicitly");
    requireRh(!environment_saved||environment==requestEnvironment(),"saved request environment differs; replay with its recorded SZMUK/MPC/thread settings");
    requireRh(quad?(ocp=="stc_landing_noaug"||ocp=="stc_landing_noaug_node"||ocp=="stc_landing_noaug_fixed_dt"||ocp=="stc_landing_noaug_node_fixed_dt"):
        (ocp=="existing_pointmass_ctcs"||ocp=="existing_pointmass_node"),"saved OCP does not match requested model/family");
    auto descriptor=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor(fixed_dt>0.?ocp.substr(0,ocp.size()-9):ocp)):
        pointmassReferenceDescriptor(100,false,ocp=="existing_pointmass_node");
    if(fixed_dt>0.)descriptor=FixedQuadTime::descriptor(*descriptor,fixed_dt);
    configureTestParameters(*descriptor);
    auto problem=descriptor->create(args);const int N=problem->getHorizon();
    requireRh(prepared_x.size()==size_t(N+1)&&prepared_u.size()==size_t(N),"saved request shape mismatch");
    for(int k=0;k<=N;++k)problem->setInitialState(k,prepared_x[k]);
    for(int k=0;k<N;++k)problem->setInitialControl(k,prepared_u[k]);
    auto parameters=descriptor->getSolverParams();parameters.verbose=true;
    ALIPDDP<double> solver(*problem);solver.init(parameters);
    const auto initial=auditPrimal(*problem,solver.getXTraj(),solver.getUTraj(),parameters);
    std::cout<<std::setprecision(17)<<"REPLAY_INITIAL ocp="<<ocp<<" max_iter="<<parameters.max_iter<<" max_inner="<<parameters.max_inner_iter
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
    std::cout<<std::defaultfloat<<std::setprecision(17)<<"REPLAY_FINAL bit_identical=1 iter="<<solver.getNumIter()<<" cost_records="<<solver.getAllCost().size()
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
    if(argc==3&&std::string(argv[1])=="--fixed-dt-contract"){fixedDurationContract(std::stod(argv[2]));return 0;}
    if(argc==2&&std::string(argv[1])=="--ordinary-node-contract"){ordinaryNodeContract();return 0;}
    if((argc==4||argc==5)&&std::string(argv[1])=="--diagnose-request"){
        requireRh(std::string(argv[2])=="quad"||std::string(argv[2])=="pointmass","unknown replay model");
        diagnoseSavedRequest(std::string(argv[2])=="quad",argv[3],argc==5?argv[4]:"");return 0;
    }
    if(argc==2&&std::string(argv[1])=="--root-consistency"){measuredGuessContract();return 0;}
    if(argc<5||argc>16)throw std::invalid_argument("usage: test_measured_root_rh pointmass|quad matched|fine CYCLES OUTPUT [--ctcs] [--reference-initial|--staging-initial] [--native-initial-guess] [--audit-loop] [--stationary-target] [--remaining-horizon] [--fixed-dt SECONDS] [--single-horizon] [--replay-intervals K] [--plant-acceleration-x MPS2]");
    bool ctcs=false,native_guess=false,reference_initial=false,staging_initial=false,audit_loop=false,stationary_target=false,remaining_horizon=false,single_horizon=false;double fixed_dt=0.,plant_acceleration_x=0.;
    int replay_intervals=4;
    std::string initial_state_file;
    for(int i=5;i<argc;++i){
        const std::string option=argv[i];
        if(option=="--ctcs")ctcs=true;
        else if(option=="--reference-initial")reference_initial=true;
        else if(option=="--native-initial-guess")native_guess=true;
        else if(option=="--staging-initial")staging_initial=true;
        else if(option=="--audit-loop")audit_loop=true;
        else if(option=="--stationary-target")stationary_target=true;
        else if(option=="--remaining-horizon")remaining_horizon=true;
        else if(option=="--single-horizon")single_horizon=true;
        else if(option=="--initial-state"){requireRh(i+1<argc,"missing initial state file");initial_state_file=argv[++i];}
        else if(option=="--replay-intervals"){requireRh(i+1<argc,"missing replay interval count");replay_intervals=std::stoi(argv[++i]);requireRh(replay_intervals>0,"invalid replay interval count");}
        else if(option=="--plant-acceleration-x"){requireRh(i+1<argc,"missing plant acceleration");plant_acceleration_x=std::stod(argv[++i]);requireRh(std::isfinite(plant_acceleration_x),"invalid plant acceleration");}
        else if(option=="--fixed-dt"){requireRh(i+1<argc,"missing fixed dt");fixed_dt=std::stod(argv[++i]);requireRh(fixed_dt>0.,"invalid fixed dt");}
        else throw std::invalid_argument("unknown option "+option);
    }
    const bool quad=std::string(argv[1])=="quad",fine=std::string(argv[2])=="fine";
    // A single pointmass horizon is never re-sensed: it sees every obstacle.
    const double sensing_radius=single_horizon?std::numeric_limits<double>::infinity():6.;
    requireRh(quad||std::string(argv[1])=="pointmass","unknown model");
    requireRh(!reference_initial||quad,"reference initial condition is quad-only");
    requireRh(!remaining_horizon||quad,"remaining landing horizon is quad-only");
    requireRh(!staging_initial||(quad&&!reference_initial),"staging initial condition is quad-only and excludes reference initial");
    requireRh(fine||std::string(argv[2])=="matched","unknown plant");
    requireRh(fixed_dt==0.||quad,"fixed dt is quad-only");
    requireRh(fixed_dt==0.||!remaining_horizon,"fixed-dt comparison uses a fixed node count");
    requireRh(plant_acceleration_x==0.||fine,"plant disturbance requires fine independent integration");
    const int bound=std::stoi(argv[3]);
    requireRh(!single_horizon||bound==1,"single horizon requires one solve");
    const std::filesystem::path out(argv[4]);
    requireRh(bound>0&&bound<=400&&std::filesystem::create_directories(out),"invalid cycle bound/output");
    auto descriptor=quad?std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor(ctcs?"stc_landing_noaug":"stc_landing_noaug_node")):
        pointmassReferenceDescriptor(100,false,!ctcs,sensing_radius);
    if(fixed_dt>0.)descriptor=FixedQuadTime::descriptor(*descriptor,fixed_dt);
    configureTestParameters(*descriptor);
    const auto params=descriptor->getSolverParams();
    std::shared_ptr<OptimalControlProblem<double>> captured_problem;
    std::shared_ptr<StcStageConstraint<double>> physical_stc;
    OCPCreateArgs captured_args;
    std::vector<Vec> prepared_x,prepared_u;
    const auto create=descriptor->create;
    descriptor->create=[&](const OCPCreateArgs& args){
        captured_args=args;
        if(native_guess){captured_args.prev_X.clear();captured_args.prev_U.clear();captured_args.prev_K.clear();}
        captured_problem=create(captured_args);
        if(quad&&ctcs){
            if(fixed_dt>0.){
                auto dyn=std::dynamic_pointer_cast<FixedQuadTime::Dynamics>(captured_problem->getDynamics(0));
                requireRh(bool(dyn),"fixed physical audit requires the original quad adapter");
                physical_stc=StcLandingNoAugOCP::makeCtcsConstraint<double>(dyn,
                    StcLandingNoAugOCP::P.ctcs_step_eps/StcLandingNoAugOCP::P.ctcs_y_scale,-1,fixed_dt,1,false);
            }else{
                auto dyn=std::dynamic_pointer_cast<StcLandingNoAugOCP::Quad6DOFVarTimeRelativePred<double>>(captured_problem->getDynamics(0));
                requireRh(bool(dyn),"physical STC audit requires the original quad model");
                physical_stc=StcLandingNoAugOCP::makeCtcsConstraint<double>(dyn,
                    StcLandingNoAugOCP::P.ctcs_step_eps/StcLandingNoAugOCP::P.ctcs_y_scale,
                    StcLandingNoAugOCP::IDX_THETA,0.,1,false);
            }
        }
        prepared_x.clear();prepared_u.clear();
        for(int k=0;k<=captured_problem->getHorizon();++k){
            prepared_x.push_back(captured_problem->getInitialState(k));
            if(k<captured_problem->getHorizon())prepared_u.push_back(captured_problem->getInitialControl(k));
        }
        return captured_problem;
    };
    // Exact nonzero measured-root SH control already recorded by test_controller.
    const Eigen::Vector3d acceleration=quad&&!stationary_target?Eigen::Vector3d(.04306807449,.07358261346,0):Eigen::Vector3d::Zero();
    auto targetAt=[&](double t){TargetSnapshot target;target.valid=true;
        target.position=.5*acceleration*t*t;target.velocity=acceleration*t;target.acceleration=acceleration;return target;};
    auto model=descriptor->make_model(acceleration);
    const auto world_model=descriptor->make_model(Eigen::Vector3d::Zero());
    // Single horizon: one solve, every interval executed.
    const int K=single_horizon?(quad?StcLandingNoAugOCP::P.horizon:model.horizon):replay_intervals;
    // Exogenous plant acceleration is absent from the solver's prediction.
    // Applied only to the independently integrated physical state, never its
    // measured root or the constraints. Zero preserves the historical path.
    const auto plantFlow=[&](const Eigen::VectorXd& state,const Eigen::VectorXd& control,double h){
        Eigen::VectorXd next=world_model.flow(state,control,h,false).x;
        next(0)+=.5*plant_acceleration_x*h*h;
        next(quad?3:2)+=plant_acceleration_x*h;
        return next;
    };
    planner_core::PlannerCoreConfig config;config.ocp_type=descriptor->name;config.n_replay=K;config.enable_terminal_freeze=false;
    config.preserve_remaining_trajectory=remaining_horizon;
    planner_core::Controller controller;controller.configureSynchronous(config,descriptor);
    Eigen::VectorXd plant=Eigen::VectorXd::Zero(quad?13:4);
    if(quad){
        if(reference_initial)plant<<2.4,2.,2.,0.,-2.,0.,1.,0.,0.,0.,0.,0.,0.;
        // Existing noaug activation test's settled staging-hover start.
        else if(staging_initial)plant<<2.,1.5,StcLandingNoAugOCP::P.z_stage+.05,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.;
        else plant<<-.07320069131,-.04604300079,1.832031775,.07864153001,-.004976944706,.008655228599,
        .999980267207,-.004416340294,.003462269278,-.00282379661,.01105888095,.001028963132,-.002000458539;
        plant.segment<4>(6).normalize();}else plant<<-10,0,0,0;
    if(!initial_state_file.empty()){
        std::ifstream source(initial_state_file);requireRh(bool(source),"initial state file missing");
        for(int j=0;j<plant.size();++j)requireRh(bool(source>>plant(j)),"initial state file has too few values");
        std::string extra;requireRh(!(source>>extra)&&plant.allFinite(),"invalid initial state file");
        if(quad){requireRh(plant.segment<4>(6).norm()>1e-8,"invalid measured quaternion");plant.segment<4>(6).normalize();}
    }
    if(model.initial_state)plant=model.initial_state(plant,model.hover);
    planner_core::PlannerCoreInput input;input.state_frame="world";input.target_snapshot.valid=true;
    double time=0.,root_error=0.,native_error=0.,executed_error=0.,predicted_dense_error=0.,prediction_error=0.;
    bool reached=false;int accepted=0,standalone_comparisons=0;double anchor_difference=0.,shift_error=0.,guess_dynamics_error=0.;
    std::vector<Vec> previous_x,previous_u;
    TargetSnapshot previous_target;
    double previous_prefix=0.;
    Problem audit_problem;
    double native_budget_ratio=0.,planned_budget_ratio=0.,executed_budget_ratio=0.,initial_budget_ratio=0.,density_parity_error=0.,dense_state_defect=0.,planned_stc_excess=0.;
    int ctcs_rows_seen=0;
    std::map<std::string,double> trigger_time,inactive_time,triggered_consequence;
    std::map<std::string,bool> initial_trigger,last_trigger;
    std::map<std::string,int> activations,deactivations;
    std::ofstream budgets,trigger_events;
    if(ctcs){budgets.open(out/"ctcs_intervals.csv");budgets<<std::setprecision(17)
        <<"cycle,phase,stage,duration,physical_budget,native_budget,native_integral_ratio,independent_integral_ratio\n";
        trigger_events.open(out/"trigger_events.csv");trigger_events<<std::setprecision(17)
            <<"cycle,t,implication,active,consequence,lateral_radius,relative_altitude\n";
    }
    const auto rowAt=[&](int k){
        std::shared_ptr<StcStageConstraint<double>> result;
        for(const auto& row:captured_problem->getStageConstraints(k,ConstraintType::NO))
            if(auto stc=std::dynamic_pointer_cast<StcStageConstraint<double>>(row);stc&&stc->getForm()==StcForm::Density){
                requireRh(!result,"more than one CTCS interval row");result=std::move(stc);
            }
        return result;
    };
    const auto observeTriggers=[&](const Vec& x,const Vec& u,int cycle,double epoch,double duration){
        const auto row=quad?physical_stc:rowAt(0);if(!row)return;
        for(int j=0;j<row->numImplications();++j){
            const auto& im=row->implication(j);const bool active=row->triggered(j,x);
            const bool first=initial_trigger.count(im.name)==0;
            if(first)initial_trigger[im.name]=active;
            if(first||last_trigger[im.name]!=active){
                if(!first){if(active)++activations[im.name];else ++deactivations[im.name];}
                trigger_events<<cycle<<','<<epoch<<','<<im.name<<','<<active<<','<<im.consequence(x,u)<<','<<x.head<2>().norm()<<','<<(quad?x(2):0.)<<'\n';
            }
            last_trigger[im.name]=active;
            if(active){
                trigger_time[im.name]+=duration;
                triggered_consequence[im.name]=std::max(triggered_consequence[im.name],std::max(0.,im.consequence(x,u)));
            }else inactive_time[im.name]+=duration;
        }
    };
    // Interval k uses the physical row before ctcsTightenFromNode(), as the OCP does.
    const auto densityRatio=[&](const Vec& x,const Vec& u,int k){
        if(quad)return StcLandingNoAugOCP::evalCtcsIntegrand<double>(x,u,k>=StcLandingNoAugOCP::ctcsTightenFromNode())/StcLandingNoAugOCP::P.ctcs_step_eps;
        double density=0.;for(int j=0;j<NOBS;++j)if(audit_problem.mask[j]){
            const double excess=std::max(0.,obstacleG(audit_problem,x.head<2>(),j)+audit_problem.delta);
            density+=excess*excess;
        }
        return density/audit_problem.eps;
    };
    const auto physicalStcExcess=[&](const Vec& x,const Vec& u){
        double excess=0.;if(quad&&physical_stc)for(int j=0;j<physical_stc->numImplications();++j)
            if(physical_stc->triggered(j,x))excess=std::max(excess,physical_stc->implication(j).consequence(x,u));
        return excess;
    };
    // Where the largest planned physical STC excess occurs (diagnostic).
    std::string planned_peak_name;int planned_peak_cycle=-1,planned_peak_k=-1;double planned_peak_z=0.;
    int current_cycle=0;
    const auto notePlanned=[&](const Vec& x,const Vec& u,int k){
        if(!(quad&&physical_stc))return;
        for(int j=0;j<physical_stc->numImplications();++j)if(physical_stc->triggered(j,x)){
            double c=physical_stc->implication(j).consequence(x,u);
            // The row's LOS lateral distance is sqrt(lateral^2+1e-12) (a
            // derivative regulariser, 1e-6 m at the apex); grade the exact one.
            // LOS is not graded within 5 cm of the pad (same as the SITL plan audit):
            // the cone apex is on the pad, so touchdown makes it degenerate.
            if(physical_stc->implication(j).name=="los"&&x.head<3>().norm()<.05)continue;
            if(physical_stc->implication(j).name=="los"){
                const Eigen::Vector3d axis(2*(x(7)*x(9)+x(6)*x(8)),2*(x(8)*x(9)-x(6)*x(7)),x(6)*x(6)-x(7)*x(7)-x(8)*x(8)+x(9)*x(9));
                const double axial=x.head<3>().dot(axis),lateral2=std::max(0.,x.head<3>().squaredNorm()-axial*axial);
                c-=std::sqrt(lateral2+1e-12)-std::sqrt(lateral2);
            }
            if(c>planned_stc_excess){planned_stc_excess=c;planned_peak_name=physical_stc->implication(j).name;planned_peak_cycle=current_cycle;planned_peak_k=k;planned_peak_z=x(2);}
        }
    };
    const double physical_budget=quad?StcLandingNoAugOCP::P.ctcs_step_eps:audit_problem.eps;

    std::ofstream log(out/"cycles.jsonl");log<<std::setprecision(17);
    std::ofstream executed(out/"executed.csv");executed<<std::setprecision(17)<<"t";
    for(int j=0;j<model.nx;++j)executed<<",x"<<j;executed<<'\n';
    auto physical=[&](const Eigen::VectorXd& x,const Eigen::VectorXd& u){
        if(quad){
            double excess=0.;
            for(auto type:{ConstraintType::NO,ConstraintType::SOC})
                for(const auto& row:captured_problem->getStageConstraints(0,type)){
                    if(std::dynamic_pointer_cast<StcStageConstraint<double>>(row))continue;
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
    for(int cycle=0;cycle<bound&&!reached;++cycle){current_cycle=cycle;
        input.current_state=plant;input.absolute_time_sec=time;input.now=std::chrono::steady_clock::now();input.target_snapshot=targetAt(time);
        const auto measured=plant;
        captured_problem.reset();
        auto result=controller.solveSynchronous(input);
        if(captured_problem)model.horizon=captured_problem->getHorizon();
        requireRh((plant-measured).norm()==0.,"plant advanced during frozen solve");
        std::ofstream request(out/("request-"+std::to_string(cycle)+".txt"));request<<std::setprecision(17);
        request<<"ocp "<<descriptor->name<<"\nhorizon "<<model.horizon<<"\nfixed_dt "<<fixed_dt<<"\nepoch "<<time<<"\nroot "<<captured_args.current_state.transpose()<<"\ntarget_acceleration "<<acceleration.transpose()<<'\n';
        for(const auto& [name,value]:requestEnvironment())request<<"environment "<<std::quoted(name)<<' '<<std::quoted(value)<<'\n';
        request<<"environment_end\n";
        if(captured_problem){
            for(int k=0;k<=model.horizon;++k)requireRh((prepared_x[k]-captured_problem->getInitialState(k)).norm()==0.,"solve mutated the constructed state initial guess");
            for(int k=0;k<model.horizon;++k)requireRh((prepared_u[k]-captured_problem->getInitialControl(k)).norm()==0.,"solve mutated the constructed control initial guess");
        }
        saveVectors(request,"prepared_X",prepared_x);saveVectors(request,"prepared_U",prepared_u);
        saveVectors(request,"returned_X",result.solve_result.state_trajectory);saveVectors(request,"returned_U",result.solve_result.control_trajectory);
        if(ctcs&&captured_problem)for(int k=0;k<model.horizon;++k)if(const auto row=rowAt(k)){
            const double ratio=row->intervalIntegral(prepared_x[k],prepared_u[k])/row->getEpsilon();
            initial_budget_ratio=std::max(initial_budget_ratio,ratio);
            budgets<<cycle<<",initial_guess,"<<k<<','<<(model.stage_duration?model.stage_duration(prepared_x[k],prepared_u[k]):model.dt)<<','<<physical_budget<<','<<row->getEpsilon()<<','<<ratio<<",\n";
        }
        if(ctcs&&captured_problem&&cycle==0)observeTriggers(captured_args.current_state,prepared_u.front(),cycle,time,0.);
        if(!result.solve_accepted&&captured_problem&&!result.solve_result.state_trajectory.empty()){
            request<<"rejected_native "<<result.solve_result.primal_constraint_violation<<" tolerance "<<params.tolerance<<'\n';
            const auto& rejected_x=result.solve_result.state_trajectory;
            const auto& rejected_u=result.solve_result.control_trajectory;
            for(size_t k=0;k<rejected_u.size();++k)
                for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ})
                    for(const auto& row:captured_problem->getStageConstraints(k,type)){
                        const Vec c=row->c(rejected_x[k],rejected_u[k]);
                        const double excess=type==ConstraintType::SOC?c(0)+c.tail(c.size()-1).norm():type==ConstraintType::EQ?c.cwiseAbs().maxCoeff():c.maxCoeff();
                        if(excess>params.tolerance)request<<"rejected_stage "<<k<<" type "<<int(type)<<" ctcs "<<bool(std::dynamic_pointer_cast<StcStageConstraint<double>>(row))<<" residual "<<c.transpose()<<'\n';
                    }
            for(int repeat=0;repeat<comando_parameters::envOrInt("SZMUK_TEST_REPLAY_REPEATS",2);++repeat){
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
        requireRh(result.solve_accepted,("measured-root solve rejected: "+result.rejection_reason).c_str());
        const auto& xs=result.solve_result.state_trajectory;const auto& us=result.solve_result.control_trajectory;
        if(fixed_dt>0.)for(const auto& u:us)requireRh(u.size()==4,"fixed duration remained a decision variable");
        const auto relative=planner_core::prepareOcpState(*descriptor,measured,input.target_snapshot);
        root_error=std::max(root_error,(xs.front().head(descriptor->state_dim)-relative.head(descriptor->state_dim)).lpNorm<Eigen::Infinity>());
        requireRh(root_error<=1e-12&&result.x0_source=="measured","root changed/projected/predicted");
        if(audit_loop){
            requireRh((prepared_x.front()-captured_args.current_state).lpNorm<Eigen::Infinity>()<=1e-12,"constructed initial guess replaced measured root");
            // A request seeded by a native guess (CLI flag, or the OCP's own
            // measured-root guess) carries no shifted plan to check.
            if(!previous_u.empty()&&!captured_args.prev_U.empty()){
                requireRh(captured_args.prev_U.size()==previous_u.size()-(remaining_horizon?K:0),"shift changed initial-guess control count");
                for(size_t k=0;k<captured_args.prev_U.size();++k){
                    const auto& expected=previous_u[std::min(k+size_t(K),previous_u.size()-1)];
                    shift_error=std::max(shift_error,(captured_args.prev_U[k]-expected).lpNorm<Eigen::Infinity>());
                }
                if(quad){
                    double old_time=previous_prefix;
                    for(size_t k=0;k+K<previous_x.size();++k){
                        auto expected=previous_x[k+K];const double new_time=old_time-previous_prefix;
                        if(fixed_dt>0.)expected(13)=new_time;
                        expected.head<3>()+=previous_target.position+previous_target.velocity*old_time+.5*previous_target.acceleration*old_time*old_time-
                            (input.target_snapshot.position+input.target_snapshot.velocity*new_time+.5*input.target_snapshot.acceleration*new_time*new_time);
                        expected.segment<3>(3)+=previous_target.velocity+previous_target.acceleration*old_time-
                            (input.target_snapshot.velocity+input.target_snapshot.acceleration*new_time);
                        shift_error=std::max(shift_error,(captured_args.prev_X[k]-expected).lpNorm<Eigen::Infinity>());
                        if(k+K<previous_u.size())old_time+=model.stage_duration(previous_x[k+K],previous_u[k+K]);
                    }
                }
                requireRh(shift_error<=1e-12,"RH shift changed retained controls, durations or target frame");
            }
            guess_dynamics_error=std::max(guess_dynamics_error,auditPrimal(*captured_problem,prepared_x,prepared_u,params).dynamics);
        }
        if(ctcs){
            for(int j=0;j<NOBS;++j)audit_problem.mask[j]=(audit_problem.s_diag.cwiseProduct(relative.head<2>()-audit_problem.centers[j])).norm()<=sensing_radius;
            const auto row=rowAt(0);
            if(quad){
                requireRh(row&&row->numImplications()==8,"original quad STC implications are disabled or missing");
                requireRh(std::abs(row->getEpsilon()*StcLandingNoAugOCP::P.ctcs_y_scale-physical_budget)<1e-15,"quad CTCS physical budget changed");
            }
            if(row){
                requireRh(row->getForm()==StcForm::Density&&row->getEpsilon()>0.,"invalid CTCS interval row");
                for(int j=0;j<row->numImplications();++j)requireRh(row->implication(j).weight>=0.,"negative CTCS density weight");
                ++ctcs_rows_seen;
            }
            observeTriggers(relative,us.front(),cycle,time,0.);
            if(quad&&(reference_initial||staging_initial)&&cycle==0)
                requireRh(initial_trigger["staging"],"reference case must start with radius-based staging active");
        }
        if(quad)requireRh(xs.front()(13)==0.,"quad per-solve time state was not reset");
        requireRh(result.path.states.size()==xs.size(),"public interface omitted full state trajectory");
        // Audit and anchor against the exact constructed request, including
        // its initial guess, rather than rebuilding a different cold guess.
        const auto problem=captured_problem;
        const auto audit=auditPrimal(*problem,xs,us,params);
        native_error=std::max(native_error,audit.constraints);
        requireRh(audit.feasible(params.tolerance),"returned trajectory violates native constraints/dynamics");
        if(cycle==0||audit_loop){
            ALIPDDP<double> single(*problem);single.init(params);single.solve();
            const auto ax=single.getResX(),au=single.getResU();
            for(size_t k=0;k<xs.size();++k)anchor_difference=std::max(anchor_difference,(ax[k]-xs[k]).lpNorm<Eigen::Infinity>());
            for(size_t k=0;k<us.size();++k)anchor_difference=std::max(anchor_difference,(au[k]-us[k]).lpNorm<Eigen::Infinity>());
            requireRh(anchor_difference<=1e-12,"RH solve differs from fresh standalone solve of identical constructed request");
            ++standalone_comparisons;
        }
        std::vector<double> durations;double horizon=0.;
        for(size_t k=0;k<us.size();++k){
            const double h=model.stage_duration?model.stage_duration(xs[k],us[k]):model.dt;
            requireRh(h>0.&&std::isfinite(h),"invalid optimized stage duration");durations.push_back(h);horizon+=h;
            const int n=std::max(1,int(std::ceil(h/.001)));const double step=h/n;auto state=xs[k];double integral=0.;
            const auto row=ctcs?rowAt(static_cast<int>(k)):nullptr;
            if(ctcs)notePlanned(xs[k],us[k],int(k));
            const double native_ratio=row?row->intervalIntegral(xs[k],us[k])/row->getEpsilon():0.;
            if(row){
                native_budget_ratio=std::max(native_budget_ratio,native_ratio);
                const double reference=densityRatio(xs[k],us[k],int(k));
                const double error=std::abs(row->density(xs[k],us[k])/row->getEpsilon()-reference)/(1.+std::abs(reference));
                density_parity_error=std::max(density_parity_error,error);
                requireRh(error<=1e-11,"independent CTCS density disagrees with original row");
            }
            for(int q=0;q<n;++q){
                const auto next=model.flow(state,us[k],step,false).x;
                if(ctcs){
                    const auto mid=model.flow(state,us[k],.5*step,false).x;
                    notePlanned(mid,us[k],int(k));notePlanned(next,us[k],int(k));
                    predicted_dense_error=std::max(predicted_dense_error,physical(mid,us[k]));
                    integral+=step*(densityRatio(state,us[k],int(k))+4.*densityRatio(mid,us[k],int(k))+densityRatio(next,us[k],int(k)))/6.;
                }
                state=next;predicted_dense_error=std::max(predicted_dense_error,physical(state,us[k]));
            }
            if(ctcs){
                dense_state_defect=std::max(dense_state_defect,(state-xs[k+1]).lpNorm<Eigen::Infinity>());
                planned_budget_ratio=std::max(planned_budget_ratio,integral);
                budgets<<cycle<<",planned,"<<k<<','<<h<<','<<physical_budget<<','<<(row?row->getEpsilon():0.)<<','<<native_ratio<<','<<integral<<'\n';
            }
        }
        if(ctcs)notePlanned(xs.back(),us.back(),int(us.size())-1);
        double prefix=0.;
        for(int k=0;k<std::min(K,int(us.size()));++k){
            const auto interval_start=plant;const int n=std::max(1,int(std::ceil(durations[k]/.001)));const double h=durations[k]/n;
            double integral=0.;const auto row=ctcs?rowAt(k):nullptr;
            for(int q=0;q<n;++q){
                input.absolute_time_sec=time+prefix+q*h;input.current_state=plant;
                const auto sample=controller.sample(input);
                requireRh(sample.has_replay_sample&&!sample.stale&&sample.applied_control_index==k,"wrong stage/clock during prefix execution");
                requireRh((sample.command.control-us[k]).norm()<=1e-12,"controller changed planned ZOH input");
                const auto next=fine?plantFlow(plant,us[k],h):world_model.flow(interval_start,us[k],(q+1)*h,false).x;
                if(ctcs){
                    const auto mid=fine?plantFlow(plant,us[k],.5*h):world_model.flow(interval_start,us[k],(q+.5)*h,false).x;
                    const auto a=planner_core::prepareOcpState(*descriptor,plant,targetAt(time+prefix+q*h));
                    const auto b=planner_core::prepareOcpState(*descriptor,mid,targetAt(time+prefix+(q+.5)*h));
                    const auto c=planner_core::prepareOcpState(*descriptor,next,targetAt(time+prefix+(q+1)*h));
                    integral+=h*(densityRatio(a,us[k],int(k))+4.*densityRatio(b,us[k],int(k))+densityRatio(c,us[k],int(k)))/6.;
                    executed_error=std::max(executed_error,physical(b,us[k]));
                    observeTriggers(b,us[k],cycle,time+prefix+(q+.5)*h,h);
                }
                plant=next;
                executed_error=std::max(executed_error,physical(planner_core::prepareOcpState(*descriptor,plant,targetAt(time+prefix+(q+1)*h)),us[k]));
                executed<<time+prefix+(q+1)*h;for(int j=0;j<plant.size();++j)executed<<','<<plant(j);executed<<'\n';
            }
            // The plant's elapsed-time coordinate is global; the OCP clock is
            // reset each solve. Compare measured physical states only.
            const auto predicted_world=planner_core::reconstructWorldState(*descriptor,xs[k+1],targetAt(time+prefix+durations[k]));
            prediction_error=std::max(prediction_error,(plant.head(descriptor->state_dim)-predicted_world.head(descriptor->state_dim)).lpNorm<Eigen::Infinity>());
            if(!fine)requireRh(prediction_error<=params.tolerance,"model-matched independent plant disagrees at node");
            if(ctcs){
                observeTriggers(planner_core::prepareOcpState(*descriptor,plant,targetAt(time+prefix+durations[k])),us[k],cycle,time+prefix+durations[k],0.);
                executed_budget_ratio=std::max(executed_budget_ratio,integral);
                budgets<<cycle<<",executed,"<<k<<','<<durations[k]<<','<<physical_budget<<','<<(row?row->getEpsilon():0.)<<','<<(row?row->intervalIntegral(xs[k],us[k])/row->getEpsilon():0.)<<','<<integral<<'\n';budgets.flush();
            }
            prefix+=durations[k];
        }
        time+=prefix;++accepted;input.absolute_time_sec=time;
        previous_x=xs;previous_u=us;previous_target=input.target_snapshot;previous_prefix=prefix;
        requireRh(controller.sample(input).stale,"completed prefix silently extended");
        const auto measured_relative=planner_core::prepareOcpState(*descriptor,plant,targetAt(time));
        reached=quad&&measured_relative.head<3>().norm()<=.1&&measured_relative.segment<3>(3).norm()<=.3;
        log<<"{\"cycle\":"<<cycle<<",\"epoch\":"<<time<<",\"executed_duration\":"<<prefix<<",\"horizon_duration\":"<<horizon
           <<",\"native_violation\":"<<audit.constraints<<",\"root_error\":"<<root_error<<",\"native_initial_guess\":"<<native_guess
           <<",\"ctcs\":"<<ctcs<<",\"native_ctcs_ratio\":"<<native_budget_ratio<<",\"planned_ctcs_ratio\":"<<planned_budget_ratio<<",\"executed_ctcs_ratio\":"<<executed_budget_ratio
           <<",\"optimality_error\":"<<result.solve_result.optimality_error
           <<",\"termination\":"<<jstr(result.solve_result.solver_termination)
           <<",\"solve_ms\":"<<result.solve_result.solve_time_ms<<",\"iterations\":"<<result.solve_result.solve_iters<<",\"executed_physical_excess\":"<<executed_error
           <<",\"planned_dense_excess\":"<<predicted_dense_error<<",\"distance\":"<<plant.head(quad?3:2).norm()<<"}\n";log.flush();
        std::cout<<"MEASURED_RH model="<<(quad?"quad":"pointmass")<<" cycle="<<cycle<<" t="<<time<<" ctcs="<<ctcs<<" native="<<audit.constraints<<" budget_native="<<native_budget_ratio<<" budget_planned="<<planned_budget_ratio<<" budget_executed="<<executed_budget_ratio<<" executed="<<executed_error<<" planned_dense="<<predicted_dense_error<<std::endl;
    }
    // Course progress belongs to the 30 s replanning run. A single horizon has no
    // replanning clock: it is graded on certification, root, executed=planned
    // and physical excess; its tracking error is reported, not gated.
    const double reference_error=quad?0.:(plant.head<2>()-reference(time)).norm();
    const bool progress=quad?reached:(single_horizon||(plant(0)>0.&&reference_error<1.));
    const bool native_pass=accepted>0&&(quad?reached:accepted==bound)&&progress&&root_error<=1e-12&&native_error<=params.tolerance;
    // NODE acceptance covers shooting nodes. Dense checks remain diagnostics.
    const bool node_pass=native_pass;
    const bool staging_transition_pass=ctcs&&quad&&initial_trigger["staging"]&&deactivations["staging"]>0&&!last_trigger["staging"]&&trigger_time["staging"]>0.&&inactive_time["staging"]>0.;
    const bool loop_pass=native_pass&&(!audit_loop||standalone_comparisons==accepted)&&shift_error<=1e-12;
    double physical_stc_excess=0.;if(quad)for(const auto& [name,value]:triggered_consequence)physical_stc_excess=std::max(physical_stc_excess,value);
    const bool physical_stc_pass=executed_error<=params.tolerance&&physical_stc_excess<=params.tolerance&&planned_stc_excess<=params.tolerance;
    // Independent integrals diagnose quadrature. They do not redefine the
    // native CTCS row or substitute for checking the physical implications.
    const bool ctcs_pass=ctcs&&loop_pass&&ctcs_rows_seen>0&&physical_stc_pass&&(!(reference_initial||staging_initial)||staging_transition_pass);
    std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\"model\":"<<jstr(quad?"quad":"pointmass")<<",\"plant_execution\":"<<jstr(fine?"refined_1ms":"matched_discrete")<<",\"plant_acceleration_x\":"<<plant_acceleration_x<<",\"reference_initial\":"<<reference_initial<<",\"staging_initial\":"<<staging_initial<<",\"initial_ctcs_ratio\":"<<initial_budget_ratio<<",\"accepted\":"<<accepted<<",\"reached\":"<<reached<<",\"simulation_time\":"<<time
        <<",\"ctcs\":"<<ctcs<<",\"ctcs_loop_pass\":"<<ctcs_pass<<",\"staging_transition_pass\":"<<staging_transition_pass<<",\"native_ctcs_ratio\":"<<native_budget_ratio<<",\"planned_ctcs_ratio\":"<<planned_budget_ratio<<",\"executed_ctcs_ratio\":"<<executed_budget_ratio
        <<",\"loop_pass\":"<<loop_pass<<",\"physical_stc_pass\":"<<physical_stc_pass<<",\"physical_stc_excess\":"<<physical_stc_excess<<",\"standalone_comparisons\":"<<standalone_comparisons<<",\"shift_error\":"<<shift_error<<",\"initial_guess_dynamics_error\":"<<guess_dynamics_error
        <<",\"reference_error\":"<<reference_error<<",\"planned_physical_stc_excess\":"<<planned_stc_excess<<",\"planned_peak_implication\":"<<jstr(planned_peak_name)<<",\"planned_peak_cycle\":"<<planned_peak_cycle<<",\"planned_peak_interval\":"<<planned_peak_k<<",\"planned_peak_z\":"<<planned_peak_z<<",\"remaining_horizon\":"<<remaining_horizon<<",\"stationary_target\":"<<stationary_target
        <<",\"fixed_dt\":"<<fixed_dt<<",\"single_horizon\":"<<single_horizon<<",\"replay_intervals\":"<<K
        <<",\"physical_interval_budget\":"<<physical_budget<<",\"density_parity_error\":"<<density_parity_error<<",\"planned_dense_state_defect\":"<<dense_state_defect
        <<",\"native_solve_pass\":"<<native_pass<<",\"node_loop_pass\":"<<node_pass<<",\"planned_continuous_pass\":"<<(predicted_dense_error<=1e-6)<<",\"planned_dense_excess\":"<<predicted_dense_error
        <<",\"executed_continuous_pass\":"<<(executed_error<=1e-6)<<",\"executed_physical_excess\":"<<executed_error<<",\"native_violation\":"<<native_error<<",\"root_error\":"<<root_error<<",\"anchor_difference\":"<<anchor_difference
        <<",\"prediction_error\":"<<prediction_error<<"}\n";
    if(ctcs){
        std::ofstream triggers(out/"trigger_activity.json");triggers<<std::setprecision(17)<<"{\"active_seconds\":{";bool comma=false;
        for(const auto& [name,value]:trigger_time){if(comma)triggers<<',';comma=true;triggers<<jstr(name)<<':'<<value;}
        triggers<<"},\"max_positive_consequence_while_triggered\":{";comma=false;
        for(const auto& [name,value]:triggered_consequence){if(comma)triggers<<',';comma=true;triggers<<jstr(name)<<':'<<value;}
        const auto field=[&](const char* name,const auto& values){triggers<<"},\""<<name<<"\":{";comma=false;
            for(const auto& [key,value]:values){if(comma)triggers<<',';comma=true;triggers<<jstr(key)<<':'<<value;}};
        field("inactive_seconds",inactive_time);field("initial_active",initial_trigger);field("final_active",last_trigger);
        field("activations",activations);field("deactivations",deactivations);
        triggers<<"}}\n";
    }
    return (ctcs?ctcs_pass:node_pass)?0:2;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
