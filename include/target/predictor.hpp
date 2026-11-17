#pragma once
#include <Eigen/Dense>
#include <cmath>
namespace target_models {
struct ConstantAccelPredictor {
    Eigen::Vector3d p=Eigen::Vector3d::Zero(), v=Eigen::Vector3d::Zero(), a=Eigen::Vector3d::Zero();
    // Zero retains the original constant-acceleration model exactly. Nonzero
    // rotates measured horizontal acceleration at a locally constant rate.
    double turn_rate=0.;
    static double inferTurnRate(const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration) {
        const double speed2=velocity.head<2>().squaredNorm();
        return speed2>1e-8?(velocity.x()*acceleration.y()-velocity.y()*acceleration.x())/speed2:0.;
    }
    void setState(const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration) {p=position;v=velocity;a=acceleration;}
    Eigen::Vector3d predictPos(double t) const {
        if(turn_rate==0.)return p+v*t+0.5*a*t*t;
        const double z=turn_rate*t,z2=z*z;
        const double c=std::abs(z)<1e-4?t*t*(.5-z2/24.+z2*z2/720.):(1-std::cos(z))/(turn_rate*turn_rate);
        const double s=std::abs(z)<1e-4?turn_rate*t*t*t*(1./6.-z2/120.+z2*z2/5040.):(z-std::sin(z))/(turn_rate*turn_rate);
        Eigen::Vector3d out=p+v*t;out.x()+=c*a.x()-s*a.y();out.y()+=s*a.x()+c*a.y();out.z()+=.5*a.z()*t*t;return out;
    }
    Eigen::Vector3d predictVel(double t) const {
        if(turn_rate==0.)return v+a*t;
        const double z=turn_rate*t,z2=z*z;
        const double c=std::abs(z)<1e-4?t*(1-z2/6.+z2*z2/120.):std::sin(z)/turn_rate;
        const double s=std::abs(z)<1e-4?.5*turn_rate*t*t*(1-z2/12.+z2*z2/360.):(1-std::cos(z))/turn_rate;
        Eigen::Vector3d out=v;out.x()+=c*a.x()-s*a.y();out.y()+=s*a.x()+c*a.y();out.z()+=a.z()*t;return out;
    }
    Eigen::Vector3d predictAccel(double t) const {
        if(turn_rate==0.)return a;
        const double c=std::cos(turn_rate*t),s=std::sin(turn_rate*t);
        return {c*a.x()-s*a.y(),s*a.x()+c*a.y(),a.z()};
    }
    Eigen::Vector3d predictJerk(double t) const {const auto at=predictAccel(t);return {-turn_rate*at.y(),turn_rate*at.x(),0.};}
};

}
