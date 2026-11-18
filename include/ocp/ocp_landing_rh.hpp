#pragma once
// Complete ordinary constrained landing OCP: no CTCS integration or state triggers.
#include "parameters.hpp"
#include "target/predictor.hpp"
#include "planner_core/types.hpp"
#include "dynamics/quad_6dof_dynamics_aug.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>
#include <any>

namespace LandingRhOCP {
constexpr int NX=13, NU=4, NX_SS=14, NU_SS=5, IDX_DT=13, IDX_THETA=4;
// Every number this OCP uses.
struct Params {
    double mass=0.027;
    double ixx=1.66e-5;
    double iyy=1.66e-5;
    double izz=2.92e-5;
    double actuator_fmin=0.08;
    double actuator_fmax=0.60;
    double fmin=0.12;
    double fmax=0.52;
    double l_arm=0.046;
    int horizon=30;
    int nex=7;
    double th0=0.07;
    double thl=0.05;
    double thh=0.12;
    double term_w_pos_xy=2500.0;
    double term_w_pos_z=2500.0;
    double term_w_vel_xy=900.0;
    double term_w_vel_z=900.0;
    double term_w_tilt=200.0;
    double term_w_om=50.0;
    double term_vz_ref=0.0;
    double stage_w_time=0.04;
    double r_theta=10.0;
    double theta_ref=0.10;
    double r_ctrl_thr=2.0;
    double r_ctrl_mom=0.6;
    double w_stage_target_xy=0.0;
    double w_stage_target_z=0.0;
    double w_stage_target_vel=0.0;
    double gs_apex_offset=0.1;
    double vz_land_max=0.5;
    double tilt_max_deg=0.95;
    double tilt_scale=1.0;
    double omega_max=0.019;
    double gs_cone_half_angle_deg=15.0;
    double speed_max=0.75;
    double los_cone_half_angle_deg=30.0;
    double w_eps=1.0;            // stage velocity/rate regulariser
    bool feedback_warm_start=true;
    double max_feedback_du=0.05;  // largest accepted feedback warm-start correction
    Param solver=[]{Param p;p.reg1_min=1e-3;p.reg2_min=0.1;p.mu_mul=0.1;p.rho=5;p.rhoT=5;p.rho_mul=10;p.tolerance=1e-6;
        p.max_iter=180;p.verbose=false;p.is_quaternion_in_state=false;p.use_ddp_terms=false;return p;}();
};
// Validated configurations:
//   default   code values (unit tests)
//   campaign  run_scenario_campaign.py SITL cells (RH_N=40, MAX_ITER=120, THH=0.16)
inline Params preset(const std::string& name){
    Params p;
    if(name.empty()||name=="default")return p;
    if(name=="campaign"){p.horizon=40;p.thh=0.16;p.solver.max_iter=120;return p;}
    throw std::invalid_argument("unknown landing_rh preset: "+name);
}
// LANDING_RH_PRESET selects the configuration for this process.
inline const Params P=[]{const char* n=std::getenv("LANDING_RH_PRESET");return preset(n?n:"");}();
inline const double J_SCALE = 1.0/P.ixx;
inline const double TAU_MAX = P.l_arm*(P.actuator_fmax/4.0-P.actuator_fmin/4.0)*J_SCALE;
inline const Eigen::Matrix3d J_B = (Eigen::Matrix3d()<<P.ixx*J_SCALE,0,0,0,P.iyy*J_SCALE,0,0,0,P.izz*J_SCALE).finished();
inline const Eigen::Vector3d GRAVITY{0,0,-9.81};
inline const double HOVER_THRUST = P.mass*9.81;
inline const double GS_CONE_TAN = std::tan(P.gs_cone_half_angle_deg*M_PI/180.0);
inline const double LOS_CONE_TAN = std::tan(P.los_cone_half_angle_deg*M_PI/180.0);
using ConstantAccelPredictor=target_models::ConstantAccelPredictor;
template<typename Scalar>
class Quad6DOFVarTimeRelativePred : public Quad6DOFVarTimeRelative<Scalar> {
    std::shared_ptr<ConstantAccelPredictor> pred_;

public:
    explicit Quad6DOFVarTimeRelativePred(std::shared_ptr<ConstantAccelPredictor> pred)
        : Quad6DOFVarTimeRelative<Scalar>(), pred_(std::move(pred)) {
        this->dim_x = NX_SS;
    }

private:


public:
    // IDX_DT is integrated alongside the physical state; tau must not be
    // added a second time when sampling the predictor.
    Vector<Scalar> intervalRates(const Vector<Scalar>& x, const Vector<Scalar>& u) const {
        Vector<Scalar> out = Vector<Scalar>::Zero(NX_SS);
        out.head(NX) = this->xdot_impl(x.head(NX).template cast<double>(),
            u.head(NU).template cast<double>(),
            pred_->predictAccel(static_cast<double>(x(IDX_DT)))).template cast<Scalar>();
        out(IDX_DT) = Scalar(1);
        return out;
    }

    Matrix<Scalar> intervalRatesX(const Vector<Scalar>& x, const Vector<Scalar>& u) const {
        Matrix<Scalar> out = Matrix<Scalar>::Zero(NX_SS, NX_SS);
        out.topLeftCorner(NX, NX) = this->Amat_impl(
            x.head(NX).template cast<double>(), u.head(NU).template cast<double>()).template cast<Scalar>();
        // ConstantAccelPredictor has zero acceleration derivative in time.
        return out;
    }

    Matrix<Scalar> intervalRatesU(const Vector<Scalar>& x, const Vector<Scalar>&) const {
        Matrix<Scalar> out = Matrix<Scalar>::Zero(NX_SS, NU_SS);
        out.topLeftCorner(NX, NU) = this->Bmat_impl(x.head(NX).template cast<double>()).template cast<Scalar>();
        return out;
    }



    Vector<Scalar> f(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        double Th   = static_cast<double>(u(IDX_THETA));
        double tabs = static_cast<double>(x(IDX_DT));

        Eigen::Vector3d a1 = pred_->predictAccel(tabs);
        auto k1 = this->xdot_impl(xd, ud, a1);

        Eigen::Vector3d a2 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x2 = xd + 0.5*Th*k1;
        auto k2 = this->xdot_impl(x2, ud, a2);

        Eigen::Vector3d a3 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x3 = xd + 0.5*Th*k2;
        auto k3 = this->xdot_impl(x3, ud, a3);

        Eigen::Vector3d a4 = pred_->predictAccel(tabs + Th);
        Eigen::VectorXd x4 = xd + Th*k3;
        auto k4 = this->xdot_impl(x4, ud, a4);

        Eigen::VectorXd xn = xd + (Th/6.0)*(k1 + 2*k2 + 2*k3 + k4);
        xn.segment(6, 4).normalize();

        const_cast<Quad6DOFVarTimeRelativePred*>(this)
            ->setTargetAccel(pred_->predictAccel(tabs + 0.5*Th));

        Vector<Scalar> res(NX_SS);
        res.setZero();
        res.segment(0, this->NX_PHYS) = xn.template cast<Scalar>();
        res(IDX_DT) = x(IDX_DT) + u(IDX_THETA);
        return res;
    }

    Matrix<Scalar> fx(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        const double Th   = static_cast<double>(u(IDX_THETA));
        const double tabs = static_cast<double>(x(IDX_DT));
        const Eigen::Vector3d amid = pred_->predictAccel(tabs + 0.5 * Th);
        auto j = this->rk4_jac_impl(xd, ud, Th, amid);

        Matrix<Scalar> Fx = Matrix<Scalar>::Zero(NX_SS, NX_SS);
        Fx.block(0, 0, this->NX_PHYS, this->NX_PHYS) = j.dxdx.template cast<Scalar>();
        Fx(IDX_DT, IDX_DT) = Scalar(1.0);
        const_cast<Quad6DOFVarTimeRelativePred*>(this)->setTargetAccel(amid);
        return Fx;
    }

    Matrix<Scalar> fu(const Vector<Scalar>& x, const Vector<Scalar>& u) const override {
        auto xd = x.segment(0, this->NX_PHYS).template cast<double>();
        auto ud = u.segment(0, this->NU_PHYS).template cast<double>();
        const double Th   = static_cast<double>(u(IDX_THETA));
        const double tabs = static_cast<double>(x(IDX_DT));
        const Eigen::Vector3d amid = pred_->predictAccel(tabs + 0.5 * Th);
        auto j = this->rk4_jac_impl(xd, ud, Th, amid);

        Matrix<Scalar> Fu = Matrix<Scalar>::Zero(NX_SS, NU_SS);
        Fu.block(0, 0, this->NX_PHYS, this->NU_PHYS) = j.dxdu.template cast<Scalar>();
        Fu.block(0, IDX_THETA, this->NX_PHYS, 1) = j.dxdT.template cast<Scalar>();
        Fu(IDX_DT, IDX_THETA) = Scalar(1.0);
        const_cast<Quad6DOFVarTimeRelativePred*>(this)->setTargetAccel(amid);
        return Fu;
    }

    // 14-dim propagate for warm-start rollouts (tracks IDX_DT).
    Eigen::VectorXd propagate14(const Eigen::VectorXd& x14,
                                const Eigen::VectorXd& u_phys,
                                double Th) const {
        double tabs = x14(IDX_DT);
        auto xd = x14.segment(0, this->NX_PHYS);

        Eigen::Vector3d a1 = pred_->predictAccel(tabs);
        auto k1 = this->xdot_impl(xd, u_phys, a1);
        Eigen::Vector3d a2 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x2 = xd + 0.5*Th*k1;
        auto k2 = this->xdot_impl(x2, u_phys, a2);
        Eigen::Vector3d a3 = pred_->predictAccel(tabs + 0.5*Th);
        Eigen::VectorXd x3 = xd + 0.5*Th*k2;
        auto k3 = this->xdot_impl(x3, u_phys, a3);
        Eigen::Vector3d a4 = pred_->predictAccel(tabs + Th);
        Eigen::VectorXd x4 = xd + Th*k3;
        auto k4 = this->xdot_impl(x4, u_phys, a4);

        Eigen::VectorXd xn14(NX_SS);
        xn14.setZero();
        xn14.segment(0, this->NX_PHYS) = xd + (Th/6.0)*(k1+2*k2+2*k3+k4);
        xn14.segment(6, 4).normalize();
        xn14(IDX_DT) = x14(IDX_DT) + Th;
        return xn14;
    }
};
template<typename Scalar>
class LandingStageCost : public StageCostBase<Scalar> {
    double eps_,wp_xy_,wp_z_,wv_;
public:
    LandingStageCost(double eps,double xy,double z,double velocity,const Eigen::Vector3d&)
        :eps_(eps),wp_xy_(xy),wp_z_(z),wv_(velocity){}
    Scalar q(const Vector<Scalar>& x,const Vector<Scalar>& u) const override {
        const Scalar thrust=u(0)-Scalar(HOVER_THRUST),time=u(IDX_THETA)-Scalar(P.theta_ref);
        return Scalar(P.stage_w_time)*u(IDX_THETA)+Scalar(P.r_theta)*time*time
            +Scalar(wp_xy_)*x.template head<2>().squaredNorm()+Scalar(wp_z_)*x(2)*x(2)
            +Scalar(eps_+wv_)*x.template segment<3>(3).squaredNorm()+Scalar(eps_)*x.template segment<3>(10).squaredNorm()
            +Scalar(P.r_ctrl_thr)*thrust*thrust+Scalar(P.r_ctrl_mom)*u.template segment<3>(1).squaredNorm();
    }
    Vector<Scalar> qx(const Vector<Scalar>& x,const Vector<Scalar>&) const override {
        auto g=Vector<Scalar>::Zero(x.size()).eval();g.template head<2>()=Scalar(2*wp_xy_)*x.template head<2>();g(2)=Scalar(2*wp_z_)*x(2);
        g.template segment<3>(3)=Scalar(2*(eps_+wv_))*x.template segment<3>(3);g.template segment<3>(10)=Scalar(2*eps_)*x.template segment<3>(10);return g;
    }
    Vector<Scalar> qu(const Vector<Scalar>&,const Vector<Scalar>& u) const override {
        auto g=Vector<Scalar>::Zero(u.size()).eval();g(0)=Scalar(2*P.r_ctrl_thr)*(u(0)-Scalar(HOVER_THRUST));g.template segment<3>(1)=Scalar(2*P.r_ctrl_mom)*u.template segment<3>(1);
        g(IDX_THETA)=Scalar(P.stage_w_time)+Scalar(2*P.r_theta)*(u(IDX_THETA)-Scalar(P.theta_ref));return g;
    }
    Matrix<Scalar> qxx(const Vector<Scalar>& x,const Vector<Scalar>&) const override {
        auto h=Matrix<Scalar>::Zero(x.size(),x.size()).eval();h(0,0)=h(1,1)=Scalar(2*wp_xy_);h(2,2)=Scalar(2*wp_z_);
        h.diagonal().template segment<3>(3).setConstant(Scalar(2*(eps_+wv_)));h.diagonal().template segment<3>(10).setConstant(Scalar(2*eps_));return h;
    }
    Matrix<Scalar> quu(const Vector<Scalar>&,const Vector<Scalar>& u) const override {
        auto h=Matrix<Scalar>::Zero(u.size(),u.size()).eval();h(0,0)=Scalar(2*P.r_ctrl_thr);h.diagonal().template segment<3>(1).setConstant(Scalar(2*P.r_ctrl_mom));h(IDX_THETA,IDX_THETA)=Scalar(2*P.r_theta);return h;
    }
    Matrix<Scalar> qxu(const Vector<Scalar>& x,const Vector<Scalar>& u) const override {return Matrix<Scalar>::Zero(x.size(),u.size());}
};

// Physical inequalities are active at every node, including the terminal node.
class LandingLimits : public StageConstraintBase<double> {
    int nx_,nu_;
    const double tilt2_=std::pow(std::sin(.5*P.tilt_scale*P.tilt_max_deg*M_PI/180),2);
public:
    LandingLimits(int nx=NX_SS,int nu=NU_SS):nx_(nx),nu_(nu){constraint_type=ConstraintType::NO;dim_c=6;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&) const override {
        const Eigen::Vector4d q=x.segment<4>(6);const double n=q.squaredNorm();
        const Eigen::Vector3d axis=Eigen::Quaterniond(q(0),q(1),q(2),q(3)).normalized()*Eigen::Vector3d::UnitZ();
        Eigen::VectorXd c(6);c<<-x(2),x.segment<3>(3).squaredNorm()/(P.speed_max*P.speed_max)-1,
            x.segment<3>(10).squaredNorm()/(P.omega_max*P.omega_max)-1,
            (q(1)*q(1)+q(2)*q(2))/(tilt2_*n)-1,
            x.head<2>().norm()-GS_CONE_TAN*(x(2)+P.gs_apex_offset),
            std::cos(P.los_cone_half_angle_deg*M_PI/180)*x.head<3>().norm()-x.head<3>().dot(axis);
        return c;
    }
    Eigen::MatrixXd cx(const Eigen::VectorXd& x,const Eigen::VectorXd&) const override {
        Eigen::MatrixXd g=Eigen::MatrixXd::Zero(6,nx_);g(0,2)=-1;
        g.block<1,3>(1,3)=2*x.segment<3>(3).transpose()/(P.speed_max*P.speed_max);
        g.block<1,3>(2,10)=2*x.segment<3>(10).transpose()/(P.omega_max*P.omega_max);
        const Eigen::Vector4d q=x.segment<4>(6);const double n=q.squaredNorm(),perp=q(1)*q(1)+q(2)*q(2);
        g.block<1,4>(3,6)=-2*perp*q.transpose()/(tilt2_*n*n);g(3,7)+=2*q(1)/(tilt2_*n);g(3,8)+=2*q(2)/(tilt2_*n);
        const double lateral=x.head<2>().norm();if(lateral>1e-12)g.block<1,2>(4,0)=x.head<2>().transpose()/lateral;g(4,2)=-GS_CONE_TAN;
        const double w=q(0),a=q(1),b=q(2),d=q(3);
        Eigen::Vector3d axis(2*(a*d+w*b),2*(b*d-w*a),w*w-a*a-b*b+d*d);
        Eigen::Matrix<double,3,4> db;db<<2*b,2*d,2*w,2*a,-2*a,-2*w,2*d,2*b,2*w,-2*a,-2*b,2*d;
        const double length=x.head<3>().norm();g.block<1,3>(5,0)=-axis.transpose()/n;
        if(length>1e-12)g.block<1,3>(5,0)+=std::cos(P.los_cone_half_angle_deg*M_PI/180)*x.head<3>().transpose()/length;
        g.block<1,4>(5,6)=-x.head<3>().transpose()*db/n+2*x.head<3>().dot(axis)*q.transpose()/(n*n);return g;
    }
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {return Eigen::MatrixXd::Zero(6,nu_);}
};
// Native Lorentz-cone rows avoid squaring the speed/rate bounds into scalar inequalities.
class FloorRow : public StageConstraintBase<double> {
    int nx_,nu_;
public:
    FloorRow(int nx,int nu):nx_(nx),nu_(nu){constraint_type=ConstraintType::NO;dim_c=1;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{return Eigen::VectorXd::Constant(1,-x(2));}
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{auto g=Eigen::MatrixXd::Zero(1,nx_).eval();g(0,2)=-1;return g;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{return Eigen::MatrixXd::Zero(1,nu_);}
};
class SphereRow : public StageConstraintBase<double> {
    int nx_,nu_,offset_;double limit_;
public:
    SphereRow(int nx,int nu,int offset,double limit):nx_(nx),nu_(nu),offset_(offset),limit_(limit){constraint_type=ConstraintType::SOC;dim_c=4;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{Eigen::VectorXd c(4);c(0)=-1;c.tail<3>()=-x.segment<3>(offset_)/limit_;return c;}
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{auto g=Eigen::MatrixXd::Zero(4,nx_).eval();g.block<3,3>(1,offset_)=-Eigen::Matrix3d::Identity()/limit_;return g;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{return Eigen::MatrixXd::Zero(4,nu_);}
};
class GlideslopeRow : public StageConstraintBase<double> {
    int nx_,nu_;
public:
    GlideslopeRow(int nx,int nu):nx_(nx),nu_(nu){constraint_type=ConstraintType::SOC;dim_c=3;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{Eigen::VectorXd c(3);c<<-x(2)-P.gs_apex_offset,-x(0)/GS_CONE_TAN,-x(1)/GS_CONE_TAN;return c;}
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{auto g=Eigen::MatrixXd::Zero(3,nx_).eval();g(0,2)=-1;g(1,0)=g(2,1)=-1/GS_CONE_TAN;return g;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{return Eigen::MatrixXd::Zero(3,nu_);}
};
class TiltRow : public StageConstraintBase<double> {
    int nx_,nu_;double sine_=std::sin(.5*P.tilt_scale*P.tilt_max_deg*M_PI/180);
public:
    TiltRow(int nx,int nu):nx_(nx),nu_(nu){constraint_type=ConstraintType::SOC;dim_c=3;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{Eigen::VectorXd c(3);c<<-1,-x(7)/(sine_*x.segment<4>(6).norm()),-x(8)/(sine_*x.segment<4>(6).norm());return c;}
    Eigen::MatrixXd cx(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{auto g=Eigen::MatrixXd::Zero(3,nx_).eval();const auto q=x.segment<4>(6).eval();const double n=q.norm();g.block<2,4>(1,6)=x.segment<2>(7)*q.transpose()/(sine_*n*n*n);g(1,7)-=1/(sine_*n);g(2,8)-=1/(sine_*n);return g;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{return Eigen::MatrixXd::Zero(3,nu_);}
};
class LosRow : public StageConstraintBase<double> {
    int nx_,nu_;
    static Eigen::Vector3d axis(const Eigen::VectorXd& x){return Eigen::Quaterniond(x(6),x(7),x(8),x(9)).normalized()*Eigen::Vector3d::UnitZ();}
public:
    LosRow(int nx,int nu):nx_(nx),nu_(nu){constraint_type=ConstraintType::SOC;dim_c=4;}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{const auto b=axis(x);const double axial=x.head<3>().dot(b);Eigen::VectorXd c(4);c(0)=-LOS_CONE_TAN*axial;c.tail<3>()=-(x.head<3>()-axial*b);return c;}
    Eigen::MatrixXd cx(const Eigen::VectorXd& x,const Eigen::VectorXd&)const override{
        auto g=Eigen::MatrixXd::Zero(4,nx_).eval();const auto q=x.segment<4>(6).eval();const double n=q.squaredNorm(),w=q(0),a=q(1),b=q(2),d=q(3);const auto z=axis(x);const double axial=x.head<3>().dot(z);
        Eigen::Matrix<double,3,4> db;db<<2*b,2*d,2*w,2*a,-2*a,-2*w,2*d,2*b,2*w,-2*a,-2*b,2*d;db=db/n-2*z*q.transpose()/n;
        g.block<1,3>(0,0)=-LOS_CONE_TAN*z.transpose();g.block<1,4>(0,6)=-LOS_CONE_TAN*x.head<3>().transpose()*db;
        g.block<3,3>(1,0)=-Eigen::Matrix3d::Identity()+z*z.transpose();g.block<3,4>(1,6)=axial*db+z*x.head<3>().transpose()*db;return g;
    }
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&)const override{return Eigen::MatrixXd::Zero(4,nu_);}
};
class TerminalRow : public TerminalConstraintBase<double> {
    std::shared_ptr<StageConstraintBase<double>> row_;int nu_;
public:
    TerminalRow(std::shared_ptr<StageConstraintBase<double>> row,int nu):row_(std::move(row)),nu_(nu){constraint_type=row_->getConstraintType();dim_cT=row_->getDimC();}
    Eigen::VectorXd cT(const Eigen::VectorXd& x)const override{return row_->c(x,Eigen::VectorXd::Zero(nu_));}
    Eigen::MatrixXd cTx(const Eigen::VectorXd& x)const override{return row_->cx(x,Eigen::VectorXd::Zero(nu_));}
};
inline std::vector<std::shared_ptr<StageConstraintBase<double>>> landingRows(int nx,int nu) {
    return {std::make_shared<FloorRow>(nx,nu),std::make_shared<SphereRow>(nx,nu,3,P.speed_max),std::make_shared<SphereRow>(nx,nu,10,P.omega_max),std::make_shared<GlideslopeRow>(nx,nu),std::make_shared<TiltRow>(nx,nu),std::make_shared<LosRow>(nx,nu)};
}
template<typename Scalar>
class RelTermCost : public TerminalCostBase<Scalar> {
    double wp_xy_, wp_z_, wv_xy_, wv_z_, watt_, wom_, vz_ref_;
public:
    explicit RelTermCost(double wp_xy=500.0, double wp_z=500.0,
                         double wv_xy=50.0, double wv_z=100.0,
                         double watt=200.0, double wom=50.0, double vz_ref=-0.1)
        : wp_xy_(wp_xy), wp_z_(wp_z), wv_xy_(wv_xy), wv_z_(wv_z),
          watt_(watt), wom_(wom), vz_ref_(vz_ref) {}

    static void bodyZHorizontal(const Vector<Scalar>& x, Scalar& bx, Scalar& by) {
        const Scalar qw = x(6), qx = x(7), qy = x(8), qz = x(9);
        bx = Scalar(2) * (qx * qz + qw * qy);
        by = Scalar(2) * (qy * qz - qw * qx);
    }

    static Matrix<Scalar> bodyZHorizontalJacobian(const Vector<Scalar>& x) {
        const Scalar qw = x(6), qx = x(7), qy = x(8), qz = x(9);
        Matrix<Scalar> J = Matrix<Scalar>::Zero(2, 4);
        J(0, 0) = Scalar(2) * qy;  J(0, 1) = Scalar(2) * qz;
        J(0, 2) = Scalar(2) * qw;  J(0, 3) = Scalar(2) * qx;
        J(1, 0) = Scalar(-2) * qx; J(1, 1) = Scalar(-2) * qw;
        J(1, 2) = Scalar(2) * qz;  J(1, 3) = Scalar(2) * qy;
        return J;
    }

    Scalar p(const Vector<Scalar>& x) const override {
        Scalar ep   = x.template segment<2>(0).squaredNorm();
        Scalar ez   = x(2)*x(2);
        Scalar evxy = x.template segment<2>(3).squaredNorm();
        Scalar evz  = (x(5) - Scalar(vz_ref_)) * (x(5) - Scalar(vz_ref_));
        Scalar bx, by;
        bodyZHorizontal(x, bx, by);
        Scalar eom  = x.template segment<3>(10).squaredNorm();
        return Scalar(0.5) * (
               Scalar(wp_xy_)*ep + Scalar(wp_z_)*ez
             + Scalar(wv_xy_)*evxy + Scalar(wv_z_)*evz
             + Scalar(watt_)*(bx*bx + by*by)
             + Scalar(wom_)*eom);
    }

    Vector<Scalar> px(const Vector<Scalar>& x) const override {
        Vector<Scalar> g = Vector<Scalar>::Zero(x.size());
        g.segment(0,2) = Scalar(wp_xy_)*x.segment(0,2);
        g(2)           = Scalar(wp_z_)*x(2);
        g.segment(3,2) = Scalar(wv_xy_)*x.segment(3,2);
        g(5)           = Scalar(wv_z_)*(x(5) - Scalar(vz_ref_));
        Scalar bx, by;
        bodyZHorizontal(x, bx, by);
        Matrix<Scalar> Jtilt = bodyZHorizontalJacobian(x);
        Vector<Scalar> etilt(2);
        etilt << bx, by;
        g.segment(6,4) += Scalar(watt_) * Jtilt.transpose() * etilt;
        g.segment(10,3) = Scalar(wom_)*x.segment(10,3);
        return g;
    }

    Matrix<Scalar> pxx(const Vector<Scalar>& x) const override {
        Matrix<Scalar> H = Matrix<Scalar>::Zero(x.size(), x.size());
        H(0,0)=H(1,1) = Scalar(wp_xy_);
        H(2,2) = Scalar(wp_z_);
        H(3,3)=H(4,4) = Scalar(wv_xy_);
        H(5,5) = Scalar(wv_z_);
        Matrix<Scalar> Jtilt = bodyZHorizontalJacobian(x);
        H.block(6,6,4,4) += Scalar(watt_) * Jtilt.transpose() * Jtilt;
        H(10,10)=H(11,11)=H(12,12) = Scalar(wom_);
        return H;
    }
};
template<typename Scalar>
class FminCon : public StageConstraintBase<Scalar> { public:
    FminCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << Scalar(P.fmin)-u(0)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(1, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,u.size()); J(0,0)=Scalar(-1); return J; }};
template<typename Scalar>
class FmaxCon : public StageConstraintBase<Scalar> { public:
    FmaxCon() { this->constraint_type=ConstraintType::NO; this->dim_c=1; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        return (Vector<Scalar>(1) << u(0)-Scalar(P.fmax)).finished(); }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(1, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(1,u.size()); J(0,0)=Scalar(1); return J; }};
template<typename Scalar>
class MomentCon : public StageConstraintBase<Scalar> { Scalar tau_; public:
    MomentCon() : tau_(static_cast<Scalar>(TAU_MAX)) {
        this->constraint_type=ConstraintType::SOC; this->dim_c=4; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Vector<Scalar> cn(4); cn << tau_, u(1), u(2), u(3); return -cn; }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(4, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(4,u.size());
        J(1,1)=J(2,2)=J(3,3)=Scalar(-1); return J; }};
template<typename Scalar>
class ThetaBounds : public StageConstraintBase<Scalar> { Scalar lo_, hi_; public:
    ThetaBounds(double lo, double hi) : lo_(lo), hi_(hi) {
        this->constraint_type=ConstraintType::NO; this->dim_c=2; }
    Vector<Scalar> c(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Vector<Scalar> cn(2); cn(0)=u(IDX_THETA)-hi_; cn(1)=lo_-u(IDX_THETA); return cn; }
    Matrix<Scalar> cx(const Vector<Scalar>& x, const Vector<Scalar>&) const override {
        return Matrix<Scalar>::Zero(2, x.size()); }
    Matrix<Scalar> cu(const Vector<Scalar>&, const Vector<Scalar>& u) const override {
        Matrix<Scalar> J=Matrix<Scalar>::Zero(2,u.size());
        J(0,IDX_THETA)=Scalar(1); J(1,IDX_THETA)=Scalar(-1); return J; }};
inline Eigen::VectorXd ensureNoAugStateSize(const Eigen::VectorXd& x) {
    if (x.size() == NX_SS) return x;
    Eigen::VectorXd out = Eigen::VectorXd::Zero(NX_SS);
    const int n = std::min<int>(x.size(), NX);
    out.head(n) = x.head(n);
    return out;
}
inline Param getSolverParams() { return P.solver; }
inline void clampWarmControl(Eigen::VectorXd& u) {
    if (u.size() < NU_SS) return;
    u(0) = std::clamp((double)u(0), (double)P.fmin, (double)P.fmax);
    u(1) = std::clamp((double)u(1), -TAU_MAX, TAU_MAX);
    u(2) = std::clamp((double)u(2), -TAU_MAX, TAU_MAX);
    u(3) = std::clamp((double)u(3), -TAU_MAX, TAU_MAX);
    u(IDX_THETA) = std::clamp((double)u(IDX_THETA), P.thl, P.thh);
}
inline bool finiteTrajectory(const std::vector<Eigen::VectorXd>& X,
                             const std::vector<Eigen::VectorXd>& U) {
    for (const auto& x : X) if (!x.allFinite()) return false;
    for (const auto& u : U) if (!u.allFinite()) return false;
    return true;
}
inline bool plausibleTrajectory(const std::vector<Eigen::VectorXd>& X,
                                int max_node, int n_solve) {
    const double dt_limit = static_cast<double>(n_solve) * P.thh + 1.0;
    const int last = std::min(max_node, static_cast<int>(X.size()) - 1);
    for (int k = 0; k <= last; ++k) {
        const auto& x = X[k];
        if (x.size() <= IDX_DT) return false;
        if (x(2) < -0.05) return false;
        if (x.head(3).norm() > 100.0) return false;
        if (x.segment(3, 3).norm() > 50.0) return false;
        if (x.segment(10, 3).norm() > 50.0) return false;
        if (x(IDX_DT) < -1e-8 || x(IDX_DT) > dt_limit) return false;
    }
    return true;
}
struct LandingExtra {std::shared_ptr<ConstantAccelPredictor> predictor;};
inline std::shared_ptr<OptimalControlProblem<double>> create(
    const Eigen::VectorXd& x0_rel,
    const std::vector<Eigen::VectorXd>& prev_U = {},
    const std::vector<Eigen::VectorXd>& prev_X = {},
    const std::vector<Eigen::MatrixXd>& prev_K = {},
    std::shared_ptr<ConstantAccelPredictor> predictor = nullptr)
{
    const int N = P.horizon;
    auto problem = std::make_shared<OptimalControlProblem<double>>(N);

    if (!predictor) {
        predictor = std::make_shared<ConstantAccelPredictor>();
    }

    auto dyn = std::make_shared<Quad6DOFVarTimeRelativePred<double>>(predictor);
    dyn->setMass(P.mass);
    dyn->setGravity(GRAVITY);
    dyn->setJb(J_B);
    dyn->setTargetAccel(predictor->predictAccel(0.0));

    auto tcost  = std::make_shared<RelTermCost<double>>(
        P.term_w_pos_xy, P.term_w_pos_z,
        P.term_w_vel_xy, P.term_w_vel_z,
        P.term_w_tilt, P.term_w_om, P.term_vz_ref);
    auto cfmin  = std::make_shared<FminCon<double>>();
    auto cfmax  = std::make_shared<FmaxCon<double>>();
    auto cmom   = std::make_shared<MomentCon<double>>();
    auto cth    = std::make_shared<ThetaBounds<double>>(P.thl, P.thh);
    auto limits = landingRows(NX_SS,NU_SS);
    const double eps_cost = P.w_eps;
    const Eigen::Vector3d r_follow=Eigen::Vector3d::Zero();
    auto stage_cost = std::make_shared<LandingStageCost<double>>(
        eps_cost,
        P.w_stage_target_xy,
        P.w_stage_target_z,
        P.w_stage_target_vel,
        r_follow);

    for (int k = 0; k < N; ++k) {
        problem->setStageDynamics(k, dyn);
        problem->setStageCost(k, stage_cost);
        problem->addStageConstraint(k, cfmin);
        problem->addStageConstraint(k, cfmax);
        problem->addStageConstraint(k, cmom);
        problem->addStageConstraint(k, cth);
        for(const auto& row:limits)problem->addStageConstraint(k,row);
    }
    problem->setTerminalCost(tcost);
    for(const auto& row:limits)problem->addTerminalConstraint(std::make_shared<TerminalRow>(row,NU_SS));

    Eigen::VectorXd x0ss(NX_SS);
    x0ss.setZero();
    x0ss.segment(0, NX) = x0_rel.head(NX);
    x0ss(IDX_DT) = 0.0;
    problem->setInitialState(0, x0ss);

    // Rebase the shifted warm-start references: IDX_DT is a per-solve
    // accumulator, but the core's makeXshifted() does a plain index shift, so
    // prev_X[0] here is the previously executed node carrying the old
    // horizon's t0. Subtract it (benchmark makeXshifted equivalent) and pin
    // the first reference to the new initial state.
    std::vector<Eigen::VectorXd> Xw = prev_X;
    if (!Xw.empty()) {
        const Eigen::VectorXd x_base = ensureNoAugStateSize(Xw[0]);
        const double t0_shift = x_base(IDX_DT);
        bool finite = true;
        for (auto& x : Xw) {
            x = ensureNoAugStateSize(x);
            if (!x.allFinite()) { finite = false; break; }
            x(IDX_DT) = std::max(0.0, x(IDX_DT) - t0_shift);
        }
        if (finite) {
            Xw[0] = x0ss;
        } else {
            Xw.clear();
        }
    }

    const bool use_feedback =
        P.feedback_warm_start &&
        static_cast<int>(prev_U.size()) >= N &&
        static_cast<int>(Xw.size()) > N &&
        static_cast<int>(prev_K.size()) >= N;

    Eigen::VectorXd sim14 = x0ss;
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd uk(NU_SS);
        uk.setZero();
        if (static_cast<int>(prev_U.size()) >= N) {
            uk = prev_U[k];
            if (use_feedback) {
                const Eigen::VectorXd& x_ref = Xw[k];
                const Eigen::MatrixXd& K = prev_K[k];
                Eigen::VectorXd dx = sim14 - x_ref;
                Eigen::VectorXd u_fb = uk;
                bool feedback_dim_ok = (K.rows() == NU_SS && K.cols() <= dx.size());
                if (feedback_dim_ok) {
                    u_fb.noalias() = uk + K * dx.head(K.cols());
                }
                Eigen::VectorXd du = Eigen::VectorXd::Zero(NU_SS);
                if (feedback_dim_ok) du = u_fb - uk;
                const double max_du = P.max_feedback_du;
                const bool wild_feedback =
                    !feedback_dim_ok ||
                    !u_fb.allFinite() ||
                    !du.allFinite() ||
                    (max_du > 0.0 && du.norm() > max_du) ||
                    u_fb(0) < 0.5 * P.fmin ||
                    u_fb(0) > 1.5 * P.fmax ||
                    std::abs(u_fb(1)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(2)) > 2.0 * TAU_MAX ||
                    std::abs(u_fb(3)) > 2.0 * TAU_MAX ||
                    u_fb(IDX_THETA) < 0.0 ||
                    u_fb(IDX_THETA) > 2.0 * P.thh;
                if (!wild_feedback) {
                    uk = u_fb;
                }
            }
        } else {
            // Straight-line descent seed; no staging conditions.
            const Eigen::Vector3d p0 = sim14.head(3);
            const Eigen::Vector3d v0 = sim14.segment(3, 3);
            double T_g = static_cast<double>(N - k) * P.th0;
            if (T_g < P.th0) T_g = P.th0;
            const Eigen::Vector3d p_goal=Eigen::Vector3d::Zero();
            Eigen::Vector3d a_req = 2.0 * ((p_goal - p0) - v0 * T_g) / (T_g * T_g);
            const double vertical_speed=-std::min(.5,std::sqrt(std::max(0.,p0.z()-.1)));
            a_req.z()=std::clamp(2*(vertical_speed-v0.z()),-.5,.5);
            const double tabs_ws = sim14(IDX_DT);
            const Eigen::Vector3d a_tgt_ws = predictor->predictAccel(tabs_ws);
            Eigen::Vector3d fw = P.mass * (a_req + a_tgt_ws - GRAVITY);
            const double horizontal_limit=fw.z()*std::tan(.9*P.tilt_scale*P.tilt_max_deg*M_PI/180);
            if(fw.head<2>().norm()>horizontal_limit)fw.head<2>()*=horizontal_limit/fw.head<2>().norm();
            Eigen::Quaterniond q(sim14(6),sim14(7),sim14(8),sim14(9));q.normalize();
            Eigen::Quaterniond goal=Eigen::Quaterniond::FromTwoVectors(q*Eigen::Vector3d::UnitZ(),fw.normalized())*q;
            Eigen::Quaterniond error=q.conjugate()*goal;if(error.w()<0)error.coeffs()*=-1;
            Eigen::Vector3d desired_rate=4*error.vec();
            if(desired_rate.norm()>.9*P.omega_max)desired_rate*=.9*P.omega_max/desired_rate.norm();
            const Eigen::Vector3d omega=sim14.segment<3>(10);
            uk.segment<3>(1)=J_B*(desired_rate-omega)/P.th0+omega.cross(J_B*omega);
            uk(0) = fw.norm();
            uk(IDX_THETA) = P.th0;
        }
        clampWarmControl(uk);

        problem->setInitialControl(k, uk);
        Eigen::VectorXd sim14_next = dyn->propagate14(sim14, uk.segment(0, NU), uk(IDX_THETA));
        sim14_next(2) = std::max(sim14_next(2), 0.0);
        problem->setInitialState(k + 1, sim14_next);
        sim14 = sim14_next;
    }

    return problem;
}
inline OCPDescriptor descriptor() {
    OCPDescriptor d;d.disarm_on_landing_finish=true;
    d.name = "landing_rh";
    d.dt = P.th0;
    d.default_n_replay = P.nex;
    d.default_mass_kg = P.mass;
    d.warm_start = OCPDescriptor::WarmStart::Feedback;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.world_acceleration=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u) {
        if(x.size()<13||u.size()<1||!x.allFinite()||!u.allFinite())throw std::invalid_argument("invalid DDP acceleration input");
        Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
        if(q.norm()<1e-8)throw std::invalid_argument("invalid DDP attitude");
        return Eigen::Vector3d(q.normalized()*Eigen::Vector3d(0,0,u(0)/P.mass)+Eigen::Vector3d(0,0,-9.81));
    };
    // Invert PX4 PositionControl's native acceleration conversion at the
    // nominal DDP state. With zero position/velocity error and integrator,
    // acc_internal = acc_feedforward - D_vel * acc_physical. In decoupled
    // mode its tilt axis is [acc_internal.xy, g], rather than thrust*q_z.
    // This is a coordinate/input conversion, without measured-error feedback.
    d.mavros_acceleration=[physical=d.world_acceleration](const Eigen::VectorXd& x,const Eigen::VectorXd& u) {
        const Eigen::Vector3d acceleration=physical(x,u);
        Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
        const Eigen::Vector3d axis=q.normalized()*Eigen::Vector3d::UnitZ();
        Eigen::Vector3d feedforward=acceleration;
        if(comando_parameters::defaults.px4.decouple_acceleration) {
            if(axis.z()<=1e-6)throw std::invalid_argument("PX4 acceleration mapping requires upright thrust axis");
            feedforward.head<2>()=9.81*axis.head<2>()/axis.z();
        }
        feedforward.x()+=comando_parameters::defaults.px4.velocity_d_xy*acceleration.x();
        feedforward.y()+=comando_parameters::defaults.px4.velocity_d_xy*acceleration.y();
        feedforward.z()+=comando_parameters::defaults.px4.velocity_d_z*acceleration.z();
        return feedforward;
    };
    d.coordinates = OCPDescriptor::Coordinates::TargetRelative;
    d.variable_dt = true;
    d.state_dim = 13;
    d.control_dim = 4;
    d.skip_altitude_validation = true;
    d.skip_trajectory_validation = true;
    d.log_state_headers = OCPLoggerDefaults::getStateHeaders13D();
    d.state_names = OCPLoggerDefaults::getStateNames13D();
    d.control_names = OCPLoggerDefaults::getControlNames4D();
    d.extract_actual_state_row = OCPLoggerDefaults::getActualStateRow13D;
    d.make_hover_state = OCPLoggerDefaults::makeHoverState13D;

    d.post_process_result = [](SolverResult& r, const TargetSnapshot& t) {
        r.is_relative_plan = t.valid;
        if (t.valid) {
            r.target_snapshot_pos = t.position;
            r.target_snapshot_vel = t.velocity;
            r.target_snapshot_acc = t.acceleration;
        } else {
            r.target_snapshot_pos.setZero();
            r.target_snapshot_vel.setZero();
            r.target_snapshot_acc.setZero();
        }
        // Ported benchmark validity gates: refuse nonfinite/implausible plans
        // so PlannerCore rejects the solve and restores the warm-start snapshot.
        if (r.success &&
            (!finiteTrajectory(r.state_trajectory, r.control_trajectory) ||
             !plausibleTrajectory(r.state_trajectory,
                                  std::min(P.nex, P.horizon), P.horizon))) {
            r.success = false;
        }
    };
    d.prepare_extra = [](const PlannerConfig&, double, const TargetSnapshot& tgt_snapshot) {
        LandingExtra ex;
        ex.predictor = std::make_shared<ConstantAccelPredictor>();
        ex.predictor->setState(tgt_snapshot.position,
                               tgt_snapshot.velocity,
                               tgt_snapshot.acceleration);
        return std::any(ex);
    };

    d.sanitize_warm_control = [](Eigen::VectorXd& u) { clampWarmControl(u); };
    d.feasible_start=[](const Eigen::VectorXd& x) {
        if(x.size()<13||!x.allFinite()||x.segment<4>(6).norm()<1e-8)return false;
        return LandingLimits(14,5).c(x,Eigen::VectorXd::Zero(5)).maxCoeff()<=0;
    };
    d.make_model=[](const Eigen::Vector3d& accel) {
        auto pred=std::make_shared<ConstantAccelPredictor>();pred->a=accel;
        auto dyn=std::make_shared<Quad6DOFVarTimeRelativePred<double>>(pred);
        dyn->setMass(P.mass);dyn->setGravity(GRAVITY);dyn->setJb(J_B);
        OCPDescriptor::Model model;model.nx=NX_SS;model.nu=NU_SS;model.horizon=P.horizon;model.dt=P.th0;
        model.hover=Eigen::VectorXd::Zero(NU_SS);model.hover(0)=HOVER_THRUST;model.hover(IDX_THETA)=P.th0;
        model.bounded=[](Eigen::VectorXd u){clampWarmControl(u);return u;};
        model.flow=[dyn](const Eigen::VectorXd& x,const Eigen::VectorXd& u,double h,bool jac) {
            auto control=u;control(IDX_THETA)=h;
            OCPDescriptor::Flow result;result.x=dyn->f(x,control);
            if(jac){result.a=dyn->fx(x,control);result.b=dyn->fu(x,control);result.b.col(IDX_THETA).setZero();}
            return result;
        };return model;
    };
    d.getSolverParams = getSolverParams;
    d.create = [](const OCPCreateArgs& a) {
        LandingExtra ex;
        if (a.extra.has_value()) {
            try { ex = std::any_cast<LandingExtra>(a.extra); }
            catch (const std::bad_any_cast&) {}
        }
        auto pred = ex.predictor;
        if (!pred) {
            pred = std::make_shared<ConstantAccelPredictor>();
            pred->setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           a.target_accel);
        }
        return create(a.current_state, a.prev_U, a.prev_X, a.prev_K, pred);
    };
    return d;
}
}
