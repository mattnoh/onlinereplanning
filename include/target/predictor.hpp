#pragma once
#include <Eigen/Dense>
#include <cmath>
namespace target_models {
// Target prediction over the OCP horizon. tau<=0 holds the acceleration
// constant (original behaviour). tau>0 decays it as a0*exp(-t/tau), so a
// measured acceleration is trusted over the near term but cannot extrapolate
// into unbounded velocity change across a multi-second horizon.
struct ConstantAccelPredictor {
    Eigen::Vector3d p=Eigen::Vector3d::Zero(), v=Eigen::Vector3d::Zero(), a=Eigen::Vector3d::Zero();
    double tau=0.;
    void setState(const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration) {p=position;v=velocity;a=acceleration;}
    Eigen::Vector3d predictPos(double t) const {
        if(tau<=0.)return p+v*t+0.5*a*t*t;
        return p+v*t+a*tau*(t-tau*(1.-std::exp(-t/tau)));
    }
    Eigen::Vector3d predictVel(double t) const {return tau<=0.?Eigen::Vector3d(v+a*t):Eigen::Vector3d(v+a*tau*(1.-std::exp(-t/tau)));}
    Eigen::Vector3d predictAccel(double t) const {return tau<=0.?a:Eigen::Vector3d(a*std::exp(-t/tau));}
    Eigen::Vector3d predictJerk(double t) const {return tau<=0.?Eigen::Vector3d::Zero():Eigen::Vector3d(-a*std::exp(-t/tau)/tau);}
};

}
