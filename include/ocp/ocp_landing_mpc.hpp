#pragma once
// Complete ordinary landing MPC, with nodal or folded CTCS path constraints.
#include "parameters.hpp"
#include "target/predictor.hpp"
#include "planner_core/types.hpp"
#include "dynamics/quad_6dof_dynamics_aug.h"
#include "constraint/stc_stage_constraint.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>
#include <any>

namespace LandingMpcOCP::Formulation {
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
    double ctcs_step_eps=1.6e-3;  // folded-limits budget (landing_mpc_ctcs)
    double slew_thrust_weight=0.1, slew_rate_weight=2.0;  // direct-MPC command slew cost
    int mpc_max_iter=60;
    Param solver=[]{
        Param p;
        p.max_iter = 180;
        p.tolerance = 1e-6;
        p.mu_mul = 0.1;
        p.rho = 5;  p.rhoT = 5;  p.rho_mul = 10;
        p.reg1_min = 1e-3;  p.reg2_min = 0.1;
        p.verbose = false;
        p.is_quaternion_in_state = false;
        p.use_ddp_terms = false;
        return p;
    }();
};
// Validated configurations:
//   default   code values (unit tests)
//   campaign  run_scenario_campaign.py SITL cells (RH_N=40, MAX_ITER=120, THH=0.16 and MPC_MAX_ITER=120)
inline Params preset(const std::string& name){
    Params p;
    if(name.empty()||name=="default")return p;
    if(name=="campaign"){
        p.horizon = 40;
        p.thh = 0.16;
        p.solver.max_iter = 120;
        p.mpc_max_iter = 120;
        return p;
    }
    throw std::invalid_argument("unknown landing_mpc preset: "+name);
}
// LANDING_MPC_PRESET selects the configuration for this process.
inline const Params P=[]{const char* n=std::getenv("LANDING_MPC_PRESET");return preset(n?n:"");}();
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

inline Param getSolverParams() { return P.solver; }

}
namespace LandingMpcOCP {
namespace Ref = Formulation;
inline const int NX = comando_parameters::defaults.mpc.native_px4 ? 34 : 18; // physical, native plant, command memory
constexpr int NU = 4;
inline const int MEMORY = NX-4;
constexpr int ROTOR=14, INTEGRAL=18, GYRO=21, GYRO_RATE=24, DERIVATIVE=27;
constexpr double DT = 0.1;   // direct-MPC grid [s]
constexpr int N = 40;

inline double nativeGravity(){return comando_parameters::defaults.mpc.native_px4?comando_parameters::defaults.px4.gravity:9.81;}
inline double rateTau() { return std::max(0.02, comando_parameters::defaults.mpc.rate_tau); }
// Resolve the rate transient, not just RK stability. Coarse propagation can
// seed the next measured-root solve outside a tight physical constraint.
// The folded interval uses this same subdivision through its dynamics adapter.
inline int substeps() { return comando_parameters::defaults.mpc.native_px4 ? std::max(2,static_cast<int>(std::ceil(DT*4*M_PI*comando_parameters::defaults.px4.gyro_cutoff_hz))) : std::max(2,static_cast<int>(std::ceil(DT/(0.1*rateTau())))); }
inline double rateLimit() { return comando_parameters::defaults.mpc.rate_limit; }
inline double thrustMax() { return std::min(Ref::P.fmax/Ref::P.mass,nativeGravity()*std::pow((1+comando_parameters::defaults.mpc.motor_speed_offset)/std::max(.1,comando_parameters::defaults.mpc.hover_thrust+comando_parameters::defaults.mpc.motor_speed_offset),comando_parameters::defaults.mpc.motor_thrust_exponent)); }
inline Eigen::VectorXd bounded(Eigen::VectorXd u) {
    if(u.size()!=NU || !u.allFinite()) throw std::runtime_error("invalid MPC control");
    u(0)=std::clamp(u(0), Ref::P.fmin/Ref::P.mass, thrustMax());
    for(int i=1;i<NU;++i) u(i)=std::clamp(u(i),-rateLimit(),rateLimit());
    return u;
}
inline Eigen::VectorXd hover() { Eigen::VectorXd u=Eigen::VectorXd::Zero(NU);u(0)=nativeGravity();return u; }

class Dynamics : public Ref::Quad6DOFVarTimeRelativePred<double> {
public:
    explicit Dynamics(std::shared_ptr<Ref::ConstantAccelPredictor> pred)
        : Ref::Quad6DOFVarTimeRelativePred<double>(std::move(pred)) {
        setMass(1.0);setGravity(Ref::GRAVITY);
        if(comando_parameters::defaults.mpc.native_px4){setJb(comando_parameters::defaults.px4.inertia.asDiagonal());setGravity(Eigen::Vector3d(0,0,-nativeGravity()));}else setJb(Eigen::Matrix3d::Identity());
        dim_x=NX;dim_u=NU;
    }
    struct Rates {Eigen::VectorXd v;Eigen::MatrixXd a,b;};
    static double rotorHover() {
        const auto& p=comando_parameters::defaults.px4;
        return std::sqrt(p.mass*p.gravity/(4*p.motor_constant));
    }
    static Eigen::VectorXd initialState(const Eigen::VectorXd& physical,const Eigen::VectorXd& applied) {
        if(physical.size()==NX)return physical;
        Eigen::VectorXd x=Eigen::VectorXd::Zero(NX);x.head(13)=physical.head(13);x.tail(NU)=applied;
        if(comando_parameters::defaults.mpc.native_px4) {
            const auto& p=comando_parameters::defaults.px4;
            x.segment<4>(ROTOR).setConstant((p.motor_min+(p.motor_max-p.motor_min)*collectiveSignal(applied(0)))/rotorHover());
            x.segment<3>(GYRO)=physical.segment<3>(10);
        }
        return x;
    }
    static Eigen::Matrix<double,4,3> nativeMixer() {
        return comando_parameters::defaults.px4.mixer_frd*Eigen::Vector3d(1,-1,-1).asDiagonal();
    }
    static Eigen::Matrix<double,3,4> nativeMoments() {
        const auto& p=comando_parameters::defaults.px4;Eigen::Matrix<double,3,4> result;
        result.row(0)=-p.rotor_position_frd.col(1).transpose();
        result.row(1)=-p.rotor_position_frd.col(0).transpose();
        result.row(2)<<-p.moment_ratio,-p.moment_ratio,p.moment_ratio,p.moment_ratio;
        return result;
    }
    static double collectiveSignal(double acceleration) {
        const auto& p=comando_parameters::defaults.mpc;
        return (p.hover_thrust+p.motor_speed_offset)*std::pow(std::max(1e-9,acceleration)/nativeGravity(),1/p.motor_thrust_exponent)-p.motor_speed_offset;
    }
    // Native controller states: normalized rotor speeds, I/P in rate units,
    // gyro low-pass output and derivative/frequency, and D-filter alpha/frequency.
    // The native PID is a predicted plant subsystem, never a second control law.
    Rates rates(const Eigen::VectorXd& x,const Eigen::VectorXd& u,bool jac) const {
        Rates out{Eigen::VectorXd::Zero(NX),Eigen::MatrixXd{},Eigen::MatrixXd{}};
        if(jac){out.a=Eigen::MatrixXd::Zero(NX,NX);out.b=Eigen::MatrixXd::Zero(NX,NU);}
        if(!comando_parameters::defaults.mpc.native_px4) {
            out.v.head(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRates(x.head(14),u);
            out.v.segment<3>(10)=(u.tail<3>()-x.segment<3>(10))/rateTau();
            if(jac) {
                out.a.topLeftCorner(14,14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesX(x.head(14),u);
                out.a.block(10,0,3,NX).setZero();out.a.block<3,3>(10,10)=-Eigen::Matrix3d::Identity()/rateTau();
                out.b.topRows(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesU(x.head(14),u).leftCols(NU);
                out.b.block(10,0,3,NU).setZero();out.b.block<3,3>(10,1)=Eigen::Matrix3d::Identity()/rateTau();
            }
            return out;
        }
        const auto& p=comando_parameters::defaults.px4;
        const double wh=rotorHover(),force_scale=p.motor_constant*wh*wh;
        const Eigen::Vector4d rotor=x.segment<4>(ROTOR);
        const Eigen::Vector4d forces=force_scale*rotor.array().square().matrix();
        Eigen::Vector4d physical;physical(0)=forces.sum()/p.mass;physical.tail<3>()=nativeMoments()*forces;
        out.v.head(14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRates(x.head(14),physical);
        if(jac) {
            out.a.topLeftCorner(14,14)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesX(x.head(14),physical);
            Eigen::Matrix4d df;df.row(0).setConstant(1/p.mass);df.bottomRows<3>()=nativeMoments();
            df*= (2*force_scale*rotor).asDiagonal();
            out.a.block(0,ROTOR,14,4)=Ref::Quad6DOFVarTimeRelativePred<double>::intervalRatesU(x.head(14),physical).leftCols(4)*df;
        }
        const double wg=2*M_PI*p.gyro_cutoff_hz,wd=2*M_PI*p.derivative_cutoff_hz;
        const Eigen::Vector3d error=u.tail<3>()-x.segment<3>(GYRO);
        const Eigen::Vector3d torque=p.rate_p.cwiseProduct(error+x.segment<3>(INTEGRAL))-
            p.rate_d.cwiseProduct(wd*x.segment<3>(DERIVATIVE));
        const Eigen::Matrix<double,4,3> mix=nativeMixer();
        const Eigen::Vector4d raw_signal=Eigen::Vector4d::Constant(collectiveSignal(u(0)))+mix*torque;
        for(int i=0;i<4;++i) {
            // Model actuator saturation here; do not change the DDP command.
            const double signal=std::clamp(raw_signal(i),0.,1.);
            const double goal=(p.motor_min+(p.motor_max-p.motor_min)*signal)/wh;
            const double tau=goal>rotor(i)?p.motor_tau_up:p.motor_tau_down;
            out.v(ROTOR+i)=(goal-rotor(i))/tau;
            if(jac) {
                out.a(ROTOR+i,ROTOR+i)=-1/tau;
                const double gain=raw_signal(i)>0&&raw_signal(i)<1 ? (p.motor_max-p.motor_min)/(wh*tau):0.;
                out.a.block<1,3>(ROTOR+i,GYRO)=-gain*mix.row(i)*p.rate_p.asDiagonal();
                out.a.block<1,3>(ROTOR+i,INTEGRAL)=gain*mix.row(i)*p.rate_p.asDiagonal();
                out.a.block<1,3>(ROTOR+i,DERIVATIVE)=-gain*wd*mix.row(i)*p.rate_d.asDiagonal();
                out.b.block<1,3>(ROTOR+i,1)=gain*mix.row(i)*p.rate_p.asDiagonal();
                const auto& mp=comando_parameters::defaults.mpc;
                const double base=std::max(1e-9,u(0))/nativeGravity();
                out.b(ROTOR+i,0)=gain*(mp.hover_thrust+mp.motor_speed_offset)/
                    (nativeGravity()*mp.motor_thrust_exponent)*std::pow(base,1/mp.motor_thrust_exponent-1);
            }
        }
        const double error_scale=400*M_PI/180;
        for(int i=0;i<3;++i) {
            const double e=error(i),factor=std::max(0.,1-e*e/(error_scale*error_scale));
            const double gain=p.rate_i(i)/p.rate_p(i);
            out.v(INTEGRAL+i)=gain*factor*e;
            if(jac) {
                const double derivative=factor>0?gain*(1-3*e*e/(error_scale*error_scale)):0.;
                out.a(INTEGRAL+i,GYRO+i)=-derivative;out.b(INTEGRAL+i,1+i)=derivative;
            }
        }
        out.v.segment<3>(GYRO)=wg*x.segment<3>(GYRO_RATE);
        out.v.segment<3>(GYRO_RATE)=wg*(x.segment<3>(10)-x.segment<3>(GYRO)-std::sqrt(2.)*x.segment<3>(GYRO_RATE));
        out.v.segment<3>(DERIVATIVE)=out.v.segment<3>(10)-wd*x.segment<3>(DERIVATIVE);
        if(jac) {
            out.a.block<3,3>(GYRO,GYRO_RATE)=wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,10)=wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,GYRO)=-wg*Eigen::Matrix3d::Identity();
            out.a.block<3,3>(GYRO_RATE,GYRO_RATE)=-std::sqrt(2.)*wg*Eigen::Matrix3d::Identity();
            out.a.middleRows(DERIVATIVE,3)=out.a.middleRows(10,3);
            out.a.block<3,3>(DERIVATIVE,DERIVATIVE)=-wd*Eigen::Matrix3d::Identity();
        }
        return out;
    }
    Eigen::VectorXd intervalRates(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,false).v;}
    Eigen::MatrixXd intervalRatesX(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,true).a;}
    Eigen::MatrixXd intervalRatesU(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {return rates(x,u,true).b;}
    struct Flow {Eigen::VectorXd x;Eigen::MatrixXd a,b;};
    Flow flow(Eigen::VectorXd x,const Eigen::VectorXd& u,double duration,bool jac=true) const {
        Eigen::MatrixXd a=Eigen::MatrixXd::Identity(NX,NX),b=Eigen::MatrixXd::Zero(NX,NU);
        const int steps=std::max(1,static_cast<int>(std::ceil(duration/(DT/substeps()))));
        const double h=duration/steps;
        for(int j=0;j<steps;++j) {
            auto r1=rates(x,u,jac);const auto& k1=r1.v;Eigen::VectorXd p2=x+h/2*k1;
            auto r2=rates(p2,u,jac);const auto& k2=r2.v;Eigen::VectorXd p3=x+h/2*k2;
            auto r3=rates(p3,u,jac);const auto& k3=r3.v;Eigen::VectorXd p4=x+h*k3;
            auto r4=rates(p4,u,jac);const auto& k4=r4.v;
            if(jac) {
                Eigen::MatrixXd a1=r1.a*a,b1=r1.a*b+r1.b;
                Eigen::MatrixXd a2=r2.a*(a+h/2*a1),b2=r2.a*(b+h/2*b1)+r2.b;
                Eigen::MatrixXd a3=r3.a*(a+h/2*a2),b3=r3.a*(b+h/2*b2)+r3.b;
                Eigen::MatrixXd a4=r4.a*(a+h*a3),b4=r4.a*(b+h*b3)+r4.b;
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
private:
    mutable Eigen::VectorXd cache_x_,cache_u_;
    mutable Flow cache_;
    const Flow& jacobian(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const {
        if(cache_x_.size()!=x.size() || cache_u_.size()!=u.size() || !(cache_x_.array()==x.array()).all() || !(cache_u_.array()==u.array()).all()) {
            cache_=flow(x,u,DT);cache_x_=x;cache_u_=u;
        }
        return cache_;
    }
public:
    Eigen::VectorXd f(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return flow(x,u,DT,false).x;}
    Eigen::MatrixXd fx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return jacobian(x,u).a;}
    Eigen::MatrixXd fu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {return jacobian(x,u).b;}
};

class Cost : public StageCostBase<double> {
    Ref::LandingStageCost<double> ref_{1.0,0.0,0.0,0.0,Eigen::Vector3d::Zero()};
    Eigen::Vector4d r_,s_;
    Eigen::VectorXd oldU(const Eigen::VectorXd& u) const {Eigen::VectorXd v=Eigen::VectorXd::Zero(5);v(0)=u(0)*Ref::P.mass;v(4)=DT;return v;}
public:
    Cost() {r_<<0.02,0.2,0.2,0.2;s_<<Ref::P.slew_thrust_weight,Ref::P.slew_rate_weight,Ref::P.slew_rate_weight,Ref::P.slew_rate_weight;}
    double q(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        return ref_.q(x.head(14),oldU(u))+(r_.array()*(u-hover()).array().square()).sum()+(s_.array()*(u-x.tail(NU)).array().square()).sum();
    }
    Eigen::VectorXd qx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.qx(x.head(14),oldU(u));g.tail(NU)=2*s_.cwiseProduct(x.tail(NU)-u);return g;
    }
    Eigen::VectorXd qu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd g=2*r_.cwiseProduct(u-hover())+2*s_.cwiseProduct(u-x.tail(NU));g(0)+=Ref::P.mass*ref_.qu(x.head(14),oldU(u))(0);return g;
    }
    Eigen::MatrixXd qxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.qxx(x.head(14),oldU(u));h.bottomRightCorner(NU,NU)=2*s_.asDiagonal();return h;
    }
    Eigen::MatrixXd quu(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::MatrixXd h=2*(r_+s_).asDiagonal();h(0,0)+=Ref::P.mass*Ref::P.mass*ref_.quu(x.head(14),oldU(u))(0,0);return h;
    }
    Eigen::MatrixXd qxu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NU);h.bottomRows(NU)=-2*s_.asDiagonal();return h;
    }
};
class Terminal : public TerminalCostBase<double> {
    Ref::RelTermCost<double> ref_{Ref::P.term_w_pos_xy,Ref::P.term_w_pos_z,Ref::P.term_w_vel_xy,Ref::P.term_w_vel_z,Ref::P.term_w_tilt,Ref::P.term_w_om,Ref::P.term_vz_ref};
public:
    double p(const Eigen::VectorXd& x) const override {return ref_.p(x.head(14));}
    Eigen::VectorXd px(const Eigen::VectorXd& x) const override {Eigen::VectorXd g=Eigen::VectorXd::Zero(NX);g.head(14)=ref_.px(x.head(14));return g;}
    Eigen::MatrixXd pxx(const Eigen::VectorXd& x) const override {Eigen::MatrixXd h=Eigen::MatrixXd::Zero(NX,NX);h.topLeftCorner(14,14)=ref_.pxx(x.head(14));return h;}
};
class Bounds : public StageConstraintBase<double> {
public:
    explicit Bounds(bool include_floor=true){constraint_type=ConstraintType::NO;dim_c=2*NU+(include_floor?1:0);}
    Eigen::VectorXd c(const Eigen::VectorXd& x,const Eigen::VectorXd& u) const override {
        Eigen::VectorXd v(dim_c);v(0)=Ref::P.fmin/Ref::P.mass-u(0);v(1)=u(0)-thrustMax();
        for(int i=1;i<NU;++i){v(2*i)=u(i)-rateLimit();v(2*i+1)=-u(i)-rateLimit();}if(dim_c>2*NU)v(8)=-x(2);return v;
    }
    Eigen::MatrixXd cx(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd a=Eigen::MatrixXd::Zero(dim_c,NX);if(dim_c>2*NU)a(8,2)=-1;return a;}
    Eigen::MatrixXd cu(const Eigen::VectorXd&,const Eigen::VectorXd&) const override {Eigen::MatrixXd b=Eigen::MatrixXd::Zero(dim_c,NU);b(0,0)=-1;b(1,0)=1;for(int i=1;i<NU;++i){b(2*i,i)=1;b(2*i+1,i)=-1;}return b;}
};
// The same six ordinary limits, always active. SOC membership c0+|ctail|<=0
// becomes a scalar consequence; there are no altitude/radius/settling triggers.
// Speed, rate and tilt already use dimensionless relative-limit residuals.
// Floor and cone distances use a one-metre reference. Normalize the row by
// its integral budget so feasibility tolerance is relative to that budget.
// Retain the convex norm curvature inside the squared violation residual.
// No changes to constraint values, Jacobians, budgets or DDP control delivery.
class FoldedNormCurvature : public StcStageConstraint<double> {
    std::shared_ptr<Dynamics> dynamics_;
public:
    FoldedNormCurvature(std::shared_ptr<Dynamics> dynamics,double epsilon)
        :StcStageConstraint<double>(StcForm::Density,epsilon),dynamics_(std::move(dynamics)){}
    Eigen::MatrixXd lagrangianHessian(const Eigen::VectorXd& x,const Eigen::VectorXd& u,
        const Eigen::VectorXd& multiplier) const override {
        Eigen::MatrixXd result=Eigen::MatrixXd::Zero(NX+NU,NX+NU);
        Eigen::VectorXd state=x;
        Eigen::MatrixXd sensitivity=Eigen::MatrixXd::Zero(NX,NX+NU);sensitivity.leftCols(NX).setIdentity();
        const double h=DT/substeps();
        auto add=[&](const Eigen::VectorXd& point,const Eigen::MatrixXd& derivative,double weight){
            // Compute the ordinary residual GN terms in this same RK pass,
            // avoiding a second integration solely for the norm correction.
            for(int i=0;i<numImplications();++i){
                const auto& term=implication(i);
                if(term.scale*term.consequence(point,u)<=0.)continue;
                const Eigen::VectorXd jacobian=derivative.transpose()*(term.scale*term.consequence_dx(point,u));
                result.noalias()+=(2*multiplier(0)*h*weight*term.weight)*(jacobian*jacobian.transpose());
            }
            for(const auto& entry:{std::pair<int,double>{3,Ref::P.speed_max},{10,Ref::P.omega_max}}){
                const auto v=point.segment<3>(entry.first).eval();const double norm=v.norm();
                const double residual=norm/entry.second-1.;if(residual<=0.)continue;
                const Eigen::Matrix3d curvature=(Eigen::Matrix3d::Identity()-v*v.transpose()/(norm*norm))/(norm*entry.second);
                const auto jacobian=derivative.middleRows(entry.first,3).eval();
                result.noalias()+=(2*multiplier(0)*h*weight*residual*implication(entry.first==3?1:2).weight)*
                    (jacobian.transpose()*curvature*jacobian);
            }
        };
        auto rates=[&](const Eigen::VectorXd& point,const Eigen::MatrixXd& derivative){
            Eigen::VectorXd velocity=dynamics_->intervalRates(point,u);
            Eigen::MatrixXd tangent=dynamics_->intervalRatesX(point,u)*derivative;
            tangent.rightCols(NU)+=dynamics_->intervalRatesU(point,u);
            return std::make_pair(velocity,tangent);
        };
        for(int i=0;i<substeps();++i){
            add(state,sensitivity,1./6);const auto first=rates(state,sensitivity);
            auto p2=(state+h/2*first.first).eval();auto s2=(sensitivity+h/2*first.second).eval();
            add(p2,s2,1./3);const auto second=rates(p2,s2);
            auto p3=(state+h/2*second.first).eval();auto s3=(sensitivity+h/2*second.second).eval();
            add(p3,s3,1./3);const auto third=rates(p3,s3);
            auto p4=(state+h*third.first).eval();auto s4=(sensitivity+h*third.second).eval();
            add(p4,s4,1./6);const auto fourth=rates(p4,s4);
            state+=h/6*(first.first+2*second.first+2*third.first+fourth.first);
            sensitivity+=h/6*(first.second+2*second.second+2*third.second+fourth.second);
        }
        return result;
    }
};
inline std::shared_ptr<StcStageConstraint<double>> foldedLimits(std::shared_ptr<Dynamics> dynamics) {
    const double budget=Ref::P.ctcs_step_eps;
    if(!(budget>0.)||!std::isfinite(budget))throw std::invalid_argument("landing_mpc_ctcs requires a finite positive CTCS_STEP_EPS");
    auto folded=std::make_shared<FoldedNormCurvature>(dynamics,1.);
    const auto limits=Ref::landingRows(NX,NU);
    const char* names[]={"floor","speed","angular_rate","glideslope","tilt","los"};
    for(size_t i=0;i<limits.size();++i){
        const auto row=limits[i];
        folded->add(names[i]).triggerWhenNegative(
            [](const Eigen::VectorXd&){return -1.;},
            [](const Eigen::VectorXd& x){return Eigen::VectorXd::Zero(x.size()).eval();})
            .consequence(
                [row](const Eigen::VectorXd& x,const Eigen::VectorXd& u){
                    const auto c=row->c(x,u);
                    return c(0)+(row->getConstraintType()==ConstraintType::SOC?c.tail(c.size()-1).norm():0.);
                },
                [row](const Eigen::VectorXd& x,const Eigen::VectorXd& u){
                    const auto c=row->c(x,u);const auto jacobian=row->cx(x,u);
                    Eigen::VectorXd gradient=jacobian.row(0).transpose();
                    if(row->getConstraintType()==ConstraintType::SOC){
                        const double norm=c.tail(c.size()-1).norm();
                        if(norm>1e-12)gradient+=jacobian.bottomRows(c.size()-1).transpose()*c.tail(c.size()-1)/norm;
                    }
                    return gradient;
                },
                [](const Eigen::VectorXd&,const Eigen::VectorXd& u){return Eigen::VectorXd::Zero(u.size()).eval();})
            .weight(1./budget);
    }
    StcIntervalDynamics<double> interval;interval.idx_theta=-1;interval.dt=DT;interval.substeps=substeps();
    interval.xdot=[dynamics](const auto& x,const auto& u,double){return dynamics->intervalRates(x,u);};
    interval.fx=[dynamics](const auto& x,const auto& u,double){return dynamics->intervalRatesX(x,u);};
    interval.fu=[dynamics](const auto& x,const auto& u,double){return dynamics->intervalRatesU(x,u);};
    folded->setIntervalDynamics(std::move(interval));return folded;
}
inline std::shared_ptr<OptimalControlProblem<double>> create(const OCPCreateArgs& args,bool ctcs=false) {
    auto pred=std::make_shared<Ref::ConstantAccelPredictor>();pred->a=args.target_accel;
    auto dyn=std::make_shared<Dynamics>(pred);auto problem=std::make_shared<OptimalControlProblem<double>>(N);
    auto cost=std::make_shared<Cost>();auto bounds=std::make_shared<Bounds>();auto initial_bounds=std::make_shared<Bounds>(false);
    auto limits=Ref::landingRows(NX,NU);
    auto folded=ctcs?foldedLimits(dyn):nullptr;
    Eigen::VectorXd x=Dynamics::initialState(args.current_state,hover());x(13)=0.;
    for(int k=0;k<N;++k){
        if(k>0&&k<static_cast<int>(args.prev_X.size())&&args.prev_X[k].size()==NX) {
            x=args.prev_X[k];x(13)=k*DT;
            x.tail(NU)=problem->getInitialControl(k-1);
        }
        problem->setInitialState(k,x);Eigen::VectorXd u=hover();
        if(k<static_cast<int>(args.prev_U.size())&&args.prev_U[k].size()==NU)u=bounded(args.prev_U[k]);
        else {
            // Initial guess only: reserve clearance from both cones before
            // descending, and honor the configured speed rather than a fixed
            // 0.5 m/s seed. The LOS clearance allows the full permitted tilt.
            const double radius=x.head<2>().norm();
            const double los_slope=std::tan(std::max(.1,Ref::P.los_cone_half_angle_deg-
                Ref::P.tilt_scale*Ref::P.tilt_max_deg)*M_PI/180);
            const double cone_height=std::max({0.,radius/Ref::GS_CONE_TAN-Ref::P.gs_apex_offset,radius/los_slope});
            const double vertical_speed=-std::min({.5,.9*Ref::P.speed_max,std::sqrt(std::max(0.,x(2)-cone_height-.1))});
            const double acceleration=std::clamp(2*(vertical_speed-x(5)),-.5,.5);
            const double remaining=std::max(DT,(N-k)*DT);
            Eigen::Vector3d force;force.head<2>()=args.target_accel.head<2>()-2*(x.head<2>()+x.segment<2>(3)*remaining)/(remaining*remaining);
            force.z()=nativeGravity()+acceleration+args.target_accel.z();
            const double horizontal_limit=force.z()*std::tan(.9*Ref::P.tilt_scale*Ref::P.tilt_max_deg*M_PI/180);
            if(force.head<2>().norm()>horizontal_limit)force.head<2>()*=horizontal_limit/force.head<2>().norm();
            Eigen::Quaterniond q(x(6),x(7),x(8),x(9));q.normalize();
            Eigen::Quaterniond goal=Eigen::Quaterniond::FromTwoVectors(q*Eigen::Vector3d::UnitZ(),force.normalized())*q;
            Eigen::Quaterniond error=q.conjugate()*goal;if(error.w()<0)error.coeffs()*=-1;
            Eigen::Vector3d rate=4*error.vec();if(rate.norm()>.9*Ref::P.omega_max)rate*=.9*Ref::P.omega_max/rate.norm();
            u(0)=force.norm();u.tail<3>()=rate;
            // Initial guess only: account for the native I/P state so the
            // warm rollout is feasible before DDP optimizes these inputs.
            if(comando_parameters::defaults.mpc.native_px4)u.tail<3>()-=x.segment<3>(INTEGRAL);
        }
        problem->setInitialControl(k,u);problem->setStageDynamics(k,dyn);problem->setStageCost(k,cost);
        // x0 is measured data, not a decision variable. Keep input bounds
        // at k=0; constrain every future state and the terminal state.
        problem->addStageConstraint(k,ctcs||k==0?initial_bounds:bounds);
        if(ctcs)problem->addStageConstraint(k,folded);
        else if(k>0)for(size_t j=1;j<limits.size();++j)problem->addStageConstraint(k,limits[j]);
        x=dyn->f(x,u);
    }
    if(static_cast<int>(args.prev_X.size())==N+1&&args.prev_X[N].size()==NX) {
        x=args.prev_X[N];x(13)=N*DT;x.tail(NU)=problem->getInitialControl(N-1);
    }
    problem->setInitialState(N,x);problem->setTerminalCost(std::make_shared<Terminal>());
    if(!ctcs)for(const auto& row:limits)problem->addTerminalConstraint(std::make_shared<Ref::TerminalRow>(row,NU));
    return problem;
}
inline OCPDescriptor descriptor(){OCPDescriptor d;d.disarm_on_landing_finish=true; d.state_dim=13; d.control_dim=4; d.skip_altitude_validation=true; d.skip_trajectory_validation=true; d.coordinates=OCPDescriptor::Coordinates::TargetRelative; d.log_state_headers=OCPLoggerDefaults::getStateHeaders13D(); d.state_names=OCPLoggerDefaults::getStateNames13D(); d.extract_actual_state_row=OCPLoggerDefaults::getActualStateRow13D; d.make_hover_state=OCPLoggerDefaults::makeHoverState13D;d.name="landing_mpc";d.execution_mode=planner_core::ExecutionMode::DirectMpc;d.dt=DT;d.variable_dt=false;
    d.make_model=[](const Eigen::Vector3d& accel) {
        auto pred=std::make_shared<Ref::ConstantAccelPredictor>(); pred->a=accel;
        auto dynamics=std::make_shared<Dynamics>(pred);
        OCPDescriptor::Model model; model.nx=NX;model.nu=NU;model.horizon=N;model.dt=DT;
        model.hover=hover();model.bounded=bounded;model.initial_state=Dynamics::initialState;
        model.flow=[dynamics](const Eigen::VectorXd& x,const Eigen::VectorXd& u,double t,bool jac) {
            auto f=dynamics->flow(x,u,t,jac);return OCPDescriptor::Flow{f.x,f.a,f.b};
        };return model;
    };
    d.command_mode=OCPDescriptor::CommandMode::CmdBodyRate;d.default_n_replay=1;d.default_mass_kg=1.0;

    d.control_names={"thrust_accel","roll_rate","pitch_rate","yaw_rate"};d.use_predicted_handoff_state=false;
    d.post_process_result=[](SolverResult& r,const TargetSnapshot& t){r.is_relative_plan=t.valid;r.target_snapshot_pos=t.position;r.target_snapshot_vel=t.velocity;r.target_snapshot_acc=t.acceleration;};
    d.validate_target=[](const TargetSnapshot& t,double now,double max_age){return t.valid&&t.odom_stamp_sec>0&&now-t.odom_stamp_sec>=0&&now-t.odom_stamp_sec<=max_age;};
    d.sanitize_warm_control=[](Eigen::VectorXd& u){u=bounded(u);};d.create=[](const OCPCreateArgs& args){return create(args);};
    d.feasible_start=[](const Eigen::VectorXd& x) {
        if(x.size()<13||!x.allFinite()||x.segment<4>(6).norm()<1e-8)return false;
        return Ref::LandingLimits(18,4).c(x,Eigen::VectorXd::Zero(4)).maxCoeff()<=0;
    };
    d.getSolverParams=[](){auto p=Ref::getSolverParams();p.max_iter=Ref::P.mpc_max_iter;p.use_feasibility_ms=true;p.feasibility_ms_exact_rows={13,NX-4,NX-3,NX-2,NX-1};p.reg1_min=1e-4;p.reg2_min=1e-2;return p;};return d;
}
inline OCPDescriptor ctcsDescriptor(){
    auto d=descriptor();d.name="landing_mpc_ctcs";
    // Small folded budgets need a lower initial barrier and warm-update floor.
    // This changes continuation, not the objective or integral allowance.
    const auto original=d.getSolverParams;
    d.getSolverParams=[original](){auto p=original();p.mu=p.muT=1e-5;return p;};
    d.create=[](const OCPCreateArgs& args){return create(args,true);};return d;
}
}
