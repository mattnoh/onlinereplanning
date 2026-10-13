#pragma once
#include "ocp/ocp_stc_landing_noaug.hpp"

namespace StcLandingMpcOCP {
namespace Ref = StcLandingNoAugOCP;
constexpr int NX = 18; // physical 13, time, previous four applied controls
constexpr int NU = 4;
constexpr int MEMORY = 14;
constexpr double DT = 0.1;
constexpr int N = 40;
constexpr int SUBSTEPS = 4;
inline double rateTau() { return std::max(0.02, Ref::envOrQ("MPC_RATE_TAU", 0.08)); }
inline double rateLimit() { return Ref::envOrQ("MPC_RATE_LIMIT", 1.0); }
inline double thrustMax() { return std::min(Ref::FMAX/Ref::MASS,9.81/std::max(0.1,Ref::envOrQ("MPC_HOVER_THRUST",0.7282821834))); }
inline Eigen::VectorXd bounded(Eigen::VectorXd u) {
    if(u.size()!=NU || !u.allFinite()) throw std::runtime_error("invalid MPC control");
    u(0)=std::clamp(u(0), Ref::FMIN/Ref::MASS, thrustMax());
    for(int i=1;i<NU;++i) u(i)=std::clamp(u(i),-rateLimit(),rateLimit());
    return u;
}
inline Eigen::VectorXd hover() { Eigen::VectorXd u=Eigen::VectorXd::Zero(NU);u(0)=9.81;return u; }

class Dynamics : public Ref::Quad6DOFVarTimeRelativePred<double> {
public:
    explicit Dynamics(std::shared_ptr<Ref::ConstantAccelPredictor> pred)
        : Ref::Quad6DOFVarTimeRelativePred<double>(std::move(pred)) {
        setMass(1.0);setGravity(Ref::GRAVITY);setJb(Eigen::Matrix3d::Identity());
        dim_x=NX;dim_u=NU;
    }
    Eigen::VectorXd intervalRates(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {
        Eigen::VectorXd v=Eigen::VectorXd::Zero(NX);
        v.head(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRates(x.head(14),u);
        v.segment<3>(10)=(u.tail<3>()-x.segment<3>(10))/rateTau();return v;
    }
    Eigen::MatrixXd intervalRatesX(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {
        Eigen::MatrixXd a=Eigen::MatrixXd::Zero(NX,NX);
        a.topLeftCorner(14,14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesX(x.head(14),u);
        a.block(10,0,3,NX).setZero();a.block<3,3>(10,10)=-Eigen::Matrix3d::Identity()/rateTau();return a;
    }
    Eigen::MatrixXd intervalRatesU(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {
        Eigen::MatrixXd b=Eigen::MatrixXd::Zero(NX,NU);
        b.topRows(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesU(x.head(14),u);
        b.block(10,0,3,NU).setZero();b.block<3,3>(10,1)=Eigen::Matrix3d::Identity()/rateTau();return b;
    }
    struct Flow {Eigen::VectorXd x;Eigen::MatrixXd a,b;};
    Flow flow(Eigen::VectorXd x,const Eigen::VectorXd& u,double duration,bool jac=true) const {
        Eigen::MatrixXd a=Eigen::MatrixXd::Identity(NX,NX),b=Eigen::MatrixXd::Zero(NX,NU);
        const int steps=std::max(1,static_cast<int>(std::ceil(duration/(DT/SUBSTEPS))));
        const double h=duration/steps;
        for(int j=0;j<steps;++j) {
            auto k1=intervalRates(x,u);Eigen::VectorXd p2=x+h/2*k1;
            auto k2=intervalRates(p2,u);Eigen::VectorXd p3=x+h/2*k2;
            auto k3=intervalRates(p3,u);Eigen::VectorXd p4=x+h*k3;
            auto k4=intervalRates(p4,u);
            if(jac) {
                Eigen::MatrixXd a1=intervalRatesX(x,u)*a,b1=intervalRatesX(x,u)*b+intervalRatesU(x,u);
                Eigen::MatrixXd a2=intervalRatesX(p2,u)*(a+h/2*a1),b2=intervalRatesX(p2,u)*(b+h/2*b1)+intervalRatesU(p2,u);
                Eigen::MatrixXd a3=intervalRatesX(p3,u)*(a+h/2*a2),b3=intervalRatesX(p3,u)*(b+h/2*b2)+intervalRatesU(p3,u);
                Eigen::MatrixXd a4=intervalRatesX(p4,u)*(a+h*a3),b4=intervalRatesX(p4,u)*(b+h*b3)+intervalRatesU(p4,u);
                a+=h/6*(a1+2*a2+2*a3+a4);b+=h/6*(b1+2*b2+2*b3+b4);
            }
            x+=h/6*(k1+2*k2+2*k3+k4);
        }
        const Eigen::Vector4d q=x.segment<4>(6);const double norm=q.norm();
        if(norm<1e-8) throw std::runtime_error("invalid MPC quaternion");
        Eigen::Matrix4d dq=(Eigen::Matrix4d::Identity()-q*q.transpose()/(norm*norm))/norm;
        a.middleRows(6,4)=dq*a.middleRows(6,4).eval();b.middleRows(6,4)=dq*b.middleRows(6,4).eval();
        x.segment<4>(6)=q/norm;x.segment(MEMORY,NU)=u;
        a.bottomRows(NU).setZero();b.bottomRows(NU).setIdentity();return {x,a,b};
    }
    Eigen::VectorXd f(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return flow(x,u,DT,false).x;}
    Eigen::MatrixXd fx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return flow(x,u,DT).a;}
    Eigen::MatrixXd fu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return flow(x,u,DT).b;}
};

class Cost : public StageCostBase<double> {
    Ref::CTcSTCStageCost<double> ref_{1.0,0.0,0.0,0.0,Eigen::Vector3d(0,0,Ref::Z_STAGE)};
    Eigen::Vector4d r_,s_;
    Eigen::VectorXd oldU(const Eigen::VectorXd& u) const {Eigen::VectorXd v=Eigen::VectorXd::Zero(5);v(0)=u(0)*Ref::MASS;v(4)=DT;return v;}
public:
    Cost() {r_<<0.02,0.2,0.2,0.2;s_<<Ref::envOrQ("MPC_SLEW_THR",0.1),Ref::envOrQ("MPC_SLEW_RATE",2),Ref::envOrQ("MPC_SLEW_RATE",2),Ref::envOrQ("MPC_SLEW_RATE",2);}
    double q(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        return ref_.q(x.head(14),oldU(u))+(r_.array()*(u-hover()).array().square()).sum()+(s_.array()*(u-x.tail(NU)).array().square()).sum();
    }
    Eigen::VectorXd qx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.qx(x.head(14),oldU(u));g.tail(NU)=2*s_.cwiseProduct(x.tail(NU)-u);return g;
    }
    Eigen::VectorXd qu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=2*r_.cwiseProduct(u-hover())+2*s_.cwiseProduct(u-x.tail(NU));g(0)+=Ref::MASS*ref_.qu(x.head(14),oldU(u))(0);return g;
    }
    Eigen::MatrixXd qxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.qxx(x.head(14),oldU(u));h.bottomRightCorner(NU,NU)=2*s_.asDiagonal();return h;
    }
    Eigen::MatrixXd quu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=2*(r_+s_).asDiagonal();h(0,0)+=Ref::MASS*Ref::MASS*ref_.quu(x.head(14),oldU(u))(0,0);return h;
    }
    Eigen::MatrixXd qxu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NU);h.bottomRows(NU)=-2*s_.asDiagonal();return h;
    }
};
class Terminal : public TerminalCostBase<double> {
    Ref::RelTermCost<double> ref_{Ref::TERM_W_POS_XY,Ref::TERM_W_POS_Z,Ref::TERM_W_VEL_XY,Ref::TERM_W_VEL_Z,Ref::TERM_W_TILT,Ref::TERM_W_OM,Ref::TERM_VZ_REF};
public:
    double p(const Eigen::VectorXd& x) const override {return ref_.p(x.head(14));}
    Eigen::VectorXd px(const Eigen::VectorXd& x) const override {Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.px(x.head(14));return g;}
    Eigen::MatrixXd pxx(const Eigen::VectorXd& x) const override {Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.pxx(x.head(14));return h;}
};
class Bounds : public StageConstraintBase<double> {
public:
    Bounds(){constraint_type=ConstraintType::NO;dim_c=2*NU+1;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd v(dim_c);v(0)=Ref::FMIN/Ref::MASS-u(0);v(1)=u(0)-thrustMax();
        for(int i=1;i<NU;++i){v(2*i)=u(i)-rateLimit();v(2*i+1)=-u(i)-rateLimit();}v(8)=-x(2);return v;
    }
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd a=Eigen::MatrixXd::Zero(dim_c,NX);a(8,2)=-1;return a;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd b=Eigen::MatrixXd::Zero(dim_c,NU);b(0,0)=-1;b(1,0)=1;for(int i=1;i<NU;++i){b(2*i,i)=1;b(2*i+1,i)=-1;}return b;}
};
inline std::shared_ptr<OptimalControlProblem<double>> create(const OCPCreateArgs& args) {
    auto pred=std::make_shared<Ref::ConstantAccelPredictor>();pred->a=args.target_accel;
    auto dyn=std::make_shared<Dynamics>(pred);auto problem=std::make_shared<OptimalControlProblem<double>>(N);
    auto cost=std::make_shared<Cost>();auto bounds=std::make_shared<Bounds>();
    auto stc=Ref::makeCtcsConstraint<double>(dyn,Ref::CTCS_STEP_EPS/Ref::CTCS_Y_SCALE,-1,DT,SUBSTEPS);
    Eigen::VectorXd x=Eigen::VectorXd::Zero(NX);x.head(13)=args.current_state.head(13);
    x.tail(NU)=args.current_state.size()==NX ? args.current_state.tail(NU).eval() : hover();
    for(int k=0;k<N;++k){problem->setInitialState(k,x);Eigen::VectorXd u=hover();
        if(k<static_cast<int>(args.prev_U.size())&&args.prev_U[k].size()==NU)u=bounded(args.prev_U[k]);
        problem->setInitialControl(k,u);problem->setStageDynamics(k,dyn);problem->setStageCost(k,cost);
        problem->addStageConstraint(k,bounds);problem->addStageConstraint(k,stc);x=dyn->f(x,u);
    }
    problem->setInitialState(N,x);problem->setTerminalCost(std::make_shared<Terminal>());return problem;
}
inline OCPDescriptor descriptor(){auto d=Ref::descriptor();d.name="stc_landing_mpc";d.dt=DT;d.variable_dt=false;
    d.command_mode=OCPDescriptor::CommandMode::CmdBodyRate;d.default_n_replay=1;d.default_mass_kg=1.0;
    d.control_names={"thrust_accel","roll_rate","pitch_rate","yaw_rate"};d.use_predicted_handoff_state=false;
    d.post_process_result=[](SolverResult& r,const TargetSnapshot& t){r.is_relative_plan=t.valid;r.target_snapshot_pos=t.position;r.target_snapshot_vel=t.velocity;r.target_snapshot_acc=t.acceleration;};
    d.validate_target=[](const TargetSnapshot& t,double now,double max_age){return t.valid&&t.odom_stamp_sec>0&&now-t.odom_stamp_sec>=0&&now-t.odom_stamp_sec<=max_age;};
    d.sanitize_warm_control=[](Eigen::VectorXd& u){u=bounded(u);};d.create=create;
    d.getSolverParams=[](){auto p=Ref::getSolverParams();p.max_iter=static_cast<int>(Ref::envOrQ("MPC_MAX_ITER",60));p.use_feasibility_ms=true;p.reg1_min=1e-4;p.reg2_min=1e-2;return p;};return d;
}
}
