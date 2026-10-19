// Matched-protocol adapter. No optimizer, globalization or production edits.
#define main production_point_mass_main
#include POINT_MASS_DIAGNOSTIC_SOURCE
#undef main
#include <functional>
#include <set>
#include <type_traits>

// Explicitly supplied by the frozen profile; no implicit tuning defaults.
#define PROFILE_FIELDS(F) F(max_iter) F(max_inner_iter) F(tolerance) F(inner_tolerance) \
 F(use_strict_barrier_gate) F(mu) F(muT) F(mu_mul) F(mu_exp) F(mu_min) \
 F(rho) F(rhoT) F(rho_mul) F(rho_max) F(reg1_min) F(reg2_min) F(reg1_exp) \
 F(reg2_exp) F(max_regularization) F(max_step_iter) F(forward_filter) \
 F(forward_cost_threshold) F(use_ddp_terms) F(use_feasibility_ms) \
 F(feasibility_ms_merit_weight) F(feasibility_ms_min_step) \
 F(feasibility_ms_gap_tol) F(feasibility_ms_control_trust) F(is_quaternion_in_state)

class MatchedBuffer : public std::streambuf {
    std::string line_;std::function<void(const std::string&)> callback_;
public:
    explicit MatchedBuffer(std::function<void(const std::string&)> c):callback_(c){}
    int overflow(int ch) override {
        if(ch==traits_type::eof())return traits_type::not_eof(ch);
        if(ch=='\n'){callback_(line_);line_.clear();}else line_+=char(ch);
        return ch;
    }
};

int main(int argc,char**argv){try{
    if(argc!=3){std::cerr<<"case.txt output_prefix\n";return 2;}
    std::ifstream in(argv[1]);int N,fixed,steps;double B,tf,tfmin,tfmax,mo,mg;
    if(!(in>>N>>fixed>>steps>>B>>tf>>tfmin>>tfmax>>mo>>mg) || N<1 || steps<1 ||
       B<=0 || tf<=0 || tfmin<=0 || tfmax<tfmin || (fixed!=0 && fixed!=1))return 2;
    g_fixed_dt=fixed?tf/N:0.;g_row_norm=fixed?1/std::sqrt(tf/N):1.;
    g_tight_obs=mo;g_tight_glide=mg;g_glide_tan=1.;
    const int nu=fixed?3:4;const double epsilon=fixed?B/(tf/N):B;
    std::vector<Vec> X(N+1,Vec(6)),U(N,Vec(nu));
    for(auto&x:X)for(int j=0;j<6;++j)if(!(in>>x(j)))return 2;
    for(auto&u:U)for(int j=0;j<nu;++j)if(!(in>>u(j)))return 2;
    for(auto&u:U)if(!fixed && !(u(3)>0))return 2;
    Param p;std::set<std::string> seen;std::string key;double value;
    while(in>>key>>value){
        if(!seen.insert(key).second || !std::isfinite(value))return 2;
        bool known=false;
#define LOAD(name) if(key==#name){p.name=value;known=true;}
        PROFILE_FIELDS(LOAD)
#undef LOAD
        if(!known)return 2;
    }
    int required=0;
#define COUNT(name) ++required;
    PROFILE_FIELDS(COUNT)
#undef COUNT
    if(int(seen.size())!=required)return 2;
    p.feasibility_ms_exact_rows.clear();p.verbose=true;
    // The selected profile is invariant across every mesh, mode and case.
    if(p.use_strict_barrier_gate || p.forward_filter!=1 || !p.use_feasibility_ms)return 2;
    class FoldedRow : public StageConstraintBase<double>{
        std::shared_ptr<StcStageConstraint<double>> row_;double epsilon_;
    public:
        FoldedRow(std::shared_ptr<StcStageConstraint<double>> r,double e):row_(r),epsilon_(e){constraint_type=ConstraintType::NO;dim_c=1;}
        Vec c(const Vec&x,const Vec&u)const override{return row_->c(x,u)/epsilon_;}
        Mat cx(const Vec&x,const Vec&u)const override{return row_->cx(x,u)/epsilon_;}
        Mat cu(const Vec&x,const Vec&u)const override{return row_->cu(x,u)/epsilon_;}
        Mat lagrangianHessian(const Vec&x,const Vec&u,const Vec&)const override{return Mat::Zero(x.size()+u.size(),x.size()+u.size());}
    };
    auto row=std::make_shared<FoldedRow>(ctcsRow(6,nu,0,steps,Eigen::Vector3d(2.85355,2.14645,5.5),2.,epsilon),epsilon);
    OptimalControlProblem<double> problem(N);
    auto dyn=std::make_shared<PointMass>();auto cost=std::make_shared<InputCost>(1.,1e-5);
    for(int k=0;k<=N;++k){
        problem.setInitialState(k,X[k]);if(k==N)continue;
        problem.setInitialControl(k,U[k]);problem.setStageDynamics(k,dyn);problem.setStageCost(k,cost);
        problem.addStageConstraint(k,row);problem.addStageConstraint(k,std::make_shared<MaxInput>());
        problem.addStageConstraint(k,std::make_shared<InputCone>());
        if(!fixed)problem.addStageConstraint(k,std::make_shared<ThetaBox>(tfmin/N,tfmax/N));
    }
    problem.setTerminalCost(std::make_shared<ZeroTerminalCost>());
    problem.addTerminalConstraint(std::make_shared<TerminalEquality>());
    ALIPDDP<double> solver(problem);solver.init(p);solver.setTrajectorySeed(X,U);
    const double initial_cost=solver.getCost();
    const std::string prefix=argv[2];std::ofstream trace(prefix+".jsonl"),native(prefix+".native.log");
    trace<<std::setprecision(17);int events=0;
    auto save=[&](){
        const auto xs=solver.getResX(),us=solver.getResU();const auto& current=solver.getParam();
        trace<<"{\"iteration\":"<<(events++==0?0:solver.getNumIter())<<",\"mu\":"<<current.mu
             <<",\"muT\":"<<current.muT<<",\"rho\":"<<current.rho<<",\"rhoT\":"<<current.rhoT
             <<",\"X\":[";
        for(int k=0;k<=N;++k){if(k)trace<<',';trace<<'[';for(int j=0;j<6;++j){if(j)trace<<',';trace<<xs[k](j);}trace<<']';}
        trace<<"],\"U\":[";
        for(int k=0;k<N;++k){if(k)trace<<',';trace<<'[';for(int j=0;j<nu;++j){if(j)trace<<',';trace<<us[k](j);}trace<<']';}
        trace<<"]}"<<std::endl;
    };
    save();MatchedBuffer buffer([&](const std::string&line){native<<line<<'\n';std::istringstream parser(line);double v;int fields=0;while(parser>>v)++fields;if(fields==16)save();});
    auto* previous=std::cout.rdbuf(&buffer);solver.solve();std::cout.rdbuf(previous);save();
    std::cout<<std::setprecision(17)<<"{\"termination\":\""<<solver.getTerminationReason()
        <<"\",\"iterations\":"<<solver.getNumIter()<<",\"optimality\":"<<solver.getOptError()
        <<",\"initial_cost\":"<<initial_cost<<",\"final_cost\":"<<solver.getCost()
        <<",\"epsilon_legacy\":"<<epsilon<<",\"physical_budget\":"<<B<<",\"settings\":{";
    bool comma=false;
#define EMIT(name) if(comma)std::cout<<',';comma=true;std::cout<<"\"" #name "\":"; \
    if constexpr(std::is_same_v<decltype(p.name),bool>)std::cout<<(p.name?"true":"false");else std::cout<<p.name;
    PROFILE_FIELDS(EMIT)
#undef EMIT
    std::cout<<",\"feasibility_ms_exact_rows\":[]},\"normalized_interval_rows\":[";
    const auto xs=solver.getResX(),us=solver.getResU();
    for(int k=0;k<N;++k){if(k)std::cout<<',';std::cout<<row->c(xs[k],us[k])(0);}
    std::cout<<"]}"<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
