// Paper 2x2 adapter: folded DDP, node-only or CTCS arm, frozen solver profile.
// No optimizer, globalization or production edits; problem data come from case.txt.
#define main production_point_mass_main
#include POINT_MASS_DIAGNOSTIC_SOURCE
#undef main
#include <functional>
#include <set>
#include <type_traits>

#define PROFILE_FIELDS(F) F(max_iter) F(max_inner_iter) F(tolerance) F(inner_tolerance) \
 F(use_strict_barrier_gate) F(mu) F(muT) F(mu_mul) F(mu_exp) F(mu_min) \
 F(rho) F(rhoT) F(rho_mul) F(rho_max) F(reg1_min) F(reg2_min) F(reg1_exp) \
 F(reg2_exp) F(max_regularization) F(max_step_iter) F(forward_filter) \
 F(forward_cost_threshold) F(use_ddp_terms) F(use_feasibility_ms) \
 F(feasibility_ms_merit_weight) F(feasibility_ms_min_step) \
 F(feasibility_ms_gap_tol) F(feasibility_ms_control_trust) F(is_quaternion_in_state)

// |v|^2 - V^2 <= 0 at every node. Under ZOH the velocity is affine on each
// interval, so its norm peaks at a node: the nodal row bounds the whole path.
class SpeedBound : public StageConstraintBase<Scalar> {
public:
    explicit SpeedBound(double v) : v2_(v * v) { constraint_type = ConstraintType::NO; dim_c = 1; }
    Vec c(const Vec& x, const Vec&) const override { Vec r(1); r(0) = x.segment(3, 3).squaredNorm() - v2_; return r; }
    Mat cx(const Vec& x, const Vec&) const override { Mat J = Mat::Zero(1, x.size()); J.block(0, 3, 1, 3) = 2. * x.segment(3, 3).transpose(); return J; }
    Mat cu(const Vec&, const Vec& u) const override { return Mat::Zero(1, u.size()); }
private:
    double v2_;
};

class LineBuffer : public std::streambuf {
    std::string line_;std::function<void(const std::string&)> callback_;
public:
    explicit LineBuffer(std::function<void(const std::string&)> c):callback_(c){}
    int overflow(int ch) override {
        if(ch==traits_type::eof())return traits_type::not_eof(ch);
        if(ch=='\n'){callback_(line_);line_.clear();}else line_+=char(ch);
        return ch;
    }
};

int main(int argc,char**argv){try{
    if(argc!=3){std::cerr<<"case.txt output_prefix\n";return 2;}
    std::ifstream in(argv[1]);int N,fixed,steps,ctcs,logrow;double B,tf,tfmin,tfmax,mo,mg,vmax;
    if(!(in>>N>>fixed>>steps>>B>>tf>>tfmin>>tfmax>>mo>>mg>>ctcs>>vmax>>logrow) || N<1 || steps<1 || B<=0 || (logrow!=0 && logrow!=1) ||
       tf<=0 || tfmin<=0 || tfmax<tfmin || vmax<=0 || (fixed!=0 && fixed!=1) || (ctcs!=0 && ctcs!=1))return 2;
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
    if(p.use_strict_barrier_gate || p.forward_filter!=1 || !p.use_feasibility_ms)return 2;
    // Folded row, CTCS Gauss-Newton curvature off (frozen profile). With r = I/B - 1:
    //   linear: c = r;   log: c = log(2 + r) - log 2 = log(1 + I/B) - log 2.
    // Same feasible set {I <= B}; the log row compresses the dynamic range of large
    // violations (I/B ~ 1e10 on an obstacle-crossing guess) while dc = dr/(2+r).
    class FoldedRow : public StageConstraintBase<double>{
        std::shared_ptr<StcStageConstraint<double>> row_;double epsilon_;bool log_;
        double r(const Vec&x,const Vec&u)const{return row_->c(x,u)(0)/epsilon_;}
        double scale(const Vec&x,const Vec&u)const{return log_?1./(2.+r(x,u)):1.;}
    public:
        FoldedRow(std::shared_ptr<StcStageConstraint<double>> rw,double e,bool lg):row_(rw),epsilon_(e),log_(lg){constraint_type=ConstraintType::NO;dim_c=1;}
        Vec c(const Vec&x,const Vec&u)const override{Vec v(1);v(0)=log_?std::log(2.+r(x,u))-std::log(2.):r(x,u);return v;}
        Mat cx(const Vec&x,const Vec&u)const override{return row_->cx(x,u)/epsilon_*scale(x,u);}
        Mat cu(const Vec&x,const Vec&u)const override{return row_->cu(x,u)/epsilon_*scale(x,u);}
        Mat lagrangianHessian(const Vec&x,const Vec&u,const Vec&)const override{return Mat::Zero(x.size()+u.size(),x.size()+u.size());}
    };
    const Eigen::Vector3d centre(2.85355,2.14645,5.5);const double radius=2.;
    std::shared_ptr<FoldedRow> row;
    if(ctcs)row=std::make_shared<FoldedRow>(ctcsRow(6,nu,0,steps,centre,radius,epsilon),epsilon,logrow==1);
    auto obstacle=std::make_shared<Obstacle>(centre,radius,0);
    auto glide=std::make_shared<GlideSlopeProj>(0);
    OptimalControlProblem<double> problem(N);
    auto dyn=std::make_shared<PointMass>();auto cost=std::make_shared<InputCost>(1.,1e-5);
    auto speed=std::make_shared<SpeedBound>(vmax);
    for(int k=0;k<=N;++k){
        problem.setInitialState(k,X[k]);if(k==N)continue;
        problem.setInitialControl(k,U[k]);problem.setStageDynamics(k,dyn);problem.setStageCost(k,cost);
        if(ctcs)problem.addStageConstraint(k,row);
        else{problem.addStageConstraint(k,obstacle);problem.addStageConstraint(k,glide);}
        problem.addStageConstraint(k,speed);
        problem.addStageConstraint(k,std::make_shared<MaxInput>());
        problem.addStageConstraint(k,std::make_shared<InputCone>());
        if(!fixed)problem.addStageConstraint(k,std::make_shared<ThetaBox>(tfmin/N,tfmax/N));
    }
    problem.setTerminalCost(std::make_shared<ZeroTerminalCost>());
    problem.addTerminalConstraint(std::make_shared<TerminalEquality>());
    ALIPDDP<double> solver(problem);solver.init(p);solver.setTrajectorySeed(X,U);
    const double initial_cost=solver.getCost();
    std::ofstream native(std::string(argv[2])+".native.log");
    LineBuffer buffer([&](const std::string&line){native<<line<<'\n';});
    const auto start=std::chrono::steady_clock::now();
    auto* previous=std::cout.rdbuf(&buffer);solver.solve();std::cout.rdbuf(previous);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    const auto xs=solver.getResX(),us=solver.getResU();
    std::cout<<std::setprecision(17)<<"{\"termination\":\""<<solver.getTerminationReason()
        <<"\",\"iterations\":"<<solver.getNumIter()<<",\"optimality\":"<<solver.getOptError()
        <<",\"initial_cost\":"<<initial_cost<<",\"final_cost\":"<<solver.getCost()
        <<",\"solve_seconds\":"<<seconds<<",\"settings\":{";
    bool comma=false;
#define EMIT(name) if(comma)std::cout<<',';comma=true;std::cout<<"\"" #name "\":"; \
    if constexpr(std::is_same_v<decltype(p.name),bool>)std::cout<<(p.name?"true":"false");else std::cout<<p.name;
    PROFILE_FIELDS(EMIT)
#undef EMIT
    std::cout<<",\"feasibility_ms_exact_rows\":[]},\"normalized_interval_rows\":[";
    // Report I/B - 1 regardless of the row coordinates.
    for(int k=0;k<N&&ctcs;++k){if(k)std::cout<<',';const double v=row->c(xs[k],us[k])(0);std::cout<<(logrow?2.*std::exp(v)-2.:v);}
    std::cout<<"],\"X\":[";
    for(int k=0;k<=N;++k){if(k)std::cout<<',';std::cout<<'[';for(int j=0;j<6;++j){if(j)std::cout<<',';std::cout<<xs[k](j);}std::cout<<']';}
    std::cout<<"],\"U\":[";
    for(int k=0;k<N;++k){if(k)std::cout<<',';std::cout<<'[';for(int j=0;j<nu;++j){if(j)std::cout<<',';std::cout<<us[k](j);}std::cout<<']';}
    std::cout<<"]}"<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
