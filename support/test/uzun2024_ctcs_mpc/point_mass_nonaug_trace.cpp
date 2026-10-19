// Isolated research driver. Reuses production model/constraints/optimizer unchanged.
#define main production_point_mass_main
#include POINT_MASS_DIAGNOSTIC_SOURCE
#undef main
#include <functional>

class TraceBuffer : public std::streambuf {
public:
    explicit TraceBuffer(std::function<void(const std::string&)> callback):callback_(callback){}
protected:
    int overflow(int ch) override {
        if(ch==traits_type::eof())return traits_type::not_eof(ch);
        if(ch=='\n'){callback_(line_);line_.clear();}else line_+=char(ch);
        return ch;
    }
private:
    std::string line_; std::function<void(const std::string&)> callback_;
};

int main(int argc,char**argv){
    if(argc!=10 && argc!=11){std::cerr<<"N substeps epsilon margin_obs margin_glide seed_file trace_file normalized gauss_newton [max_inner_iter]\n";return 2;}
    const int N=std::stoi(argv[1]),substeps=std::stoi(argv[2]);const double eps=std::stod(argv[3]);
    g_fixed_dt=4./N;g_row_norm=1/std::sqrt(g_fixed_dt);g_tight_obs=std::stod(argv[4]);g_tight_glide=std::stod(argv[5]);
    g_glide_tan=1.;
    // Optional normalization is an isolated row-coordinate change, not a solver change.
    class Normalized : public StageConstraintBase<double>{
        std::shared_ptr<StcStageConstraint<double>> row_;double scale_;bool curvature_;
    public:
        Normalized(std::shared_ptr<StcStageConstraint<double>> r,double s,bool curvature):row_(r),scale_(s),curvature_(curvature){constraint_type=ConstraintType::NO;dim_c=1;}
        Vec c(const Vec&x,const Vec&u)const override{return row_->c(x,u)/scale_;}
        Mat cx(const Vec&x,const Vec&u)const override{return row_->cx(x,u)/scale_;}
        Mat cu(const Vec&x,const Vec&u)const override{return row_->cu(x,u)/scale_;}
        Mat lagrangianHessian(const Vec&x,const Vec&u,const Vec&m)const override{if(curvature_)return row_->lagrangianHessian(x,u,m/scale_);return Mat::Zero(x.size()+u.size(),x.size()+u.size());}
    };
    std::ifstream input(argv[6]);std::vector<Vec> X(N+1,Vec(6)),U(N,Vec(3));
    for(auto&x:X)for(int j=0;j<6;++j)if(!(input>>x(j)))return 3;
    for(auto&u:U)for(int j=0;j<3;++j)if(!(input>>u(j)))return 3;
    OptimalControlProblem<double> problem(N);
    auto dyn=std::make_shared<PointMass>();auto cost=std::make_shared<InputCost>(1.,1e-5);
    const bool normalized=std::stoi(argv[8])!=0;
    auto ctcs=std::make_shared<Normalized>(ctcsRow(6,3,0,substeps,Eigen::Vector3d(2.85355,2.14645,5.5),2.,eps),normalized?eps:1.,std::stoi(argv[9])!=0);
    for(int k=0;k<=N;++k){problem.setInitialState(k,X[k]);if(k==N)continue;
        problem.setInitialControl(k,U[k]);problem.setStageDynamics(k,dyn);problem.setStageCost(k,cost);
        problem.addStageConstraint(k,ctcs);problem.addStageConstraint(k,std::make_shared<MaxInput>());
        problem.addStageConstraint(k,std::make_shared<InputCone>());
    }
    problem.setTerminalCost(std::make_shared<ZeroTerminalCost>());
    problem.addTerminalConstraint(std::make_shared<TerminalEquality>());
    ALIPDDP<double> solver(problem);Param param;
    param.max_iter=1200;param.max_inner_iter=60;param.tolerance=1e-8;param.inner_tolerance=10.;
    if(argc==11){param.max_inner_iter=std::stoi(argv[10]);if(param.max_inner_iter<=0)return 2;}
    param.mu=param.muT=1e-3;param.mu_mul=.2;param.mu_min=1e-10;
    param.rho=param.rhoT=100.;param.rho_max=1e10;param.reg1_min=1e-6;param.reg2_min=1e-6;
    param.use_feasibility_ms=true;param.verbose=true;
    solver.init(param);solver.setTrajectorySeed(X,U);
    std::ofstream trace(argv[7]);trace<<std::setprecision(17);
    std::ofstream log(std::string(argv[7])+".native.log");
    const auto start=std::chrono::steady_clock::now();double overhead=0.;int count=0;
    auto save=[&](const std::string&native){
        const auto tick=std::chrono::steady_clock::now();
        const auto xs=solver.getResX(),us=solver.getResU();
        const auto& settings=solver.getParam();
        const double elapsed=std::chrono::duration<double>(tick-start).count();
        trace<<"{\"event\":"<<count++<<",\"iteration\":"<<(count==1?0:solver.getNumIter())<<",\"wall_seconds\":"<<elapsed
             <<",\"solve_seconds_excluding_observer\":"<<elapsed-overhead<<",\"mu\":"<<settings.mu
             <<",\"muT\":"<<settings.muT<<",\"rhoT\":"<<settings.rhoT
             <<",\"effective_inner_kappa\":"<<std::min(1.,settings.inner_tolerance)
             <<",\"rho\":"<<settings.rho<<",\"opterror\":"<<(count==1?0.:solver.getOptError())<<",\"X\":[";
        for(size_t k=0;k<xs.size();++k){if(k)trace<<',';trace<<'[';for(int j=0;j<6;++j){if(j)trace<<',';trace<<xs[k](j);}trace<<']';}
        trace<<"],\"U\":[";for(size_t k=0;k<us.size();++k){if(k)trace<<',';trace<<'[';for(int j=0;j<3;++j){if(j)trace<<',';trace<<us[k](j);}trace<<']';}
        trace<<"]}"<<std::endl;
        overhead+=std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();
    };
    save("");
    TraceBuffer buffer([&](const std::string&line){log<<line<<'\n';std::istringstream parser(line);double value;int fields=0;while(parser>>value)++fields;if(fields==16)save(line);});
    auto* previous=std::cout.rdbuf(&buffer);solver.solve();std::cout.rdbuf(previous);save("");
    std::cout<<"{\"termination\":\""<<solver.getTerminationReason()<<"\",\"iterations\":"<<solver.getNumIter()
             <<",\"trace_events\":"<<count<<",\"observer_seconds\":"<<overhead<<"}"<<std::endl;
}
