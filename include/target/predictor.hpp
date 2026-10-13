#pragma once
#include <Eigen/Dense>
namespace target_models {
struct ConstantAccelPredictor {
    Eigen::Vector3d p=Eigen::Vector3d::Zero(), v=Eigen::Vector3d::Zero(), a=Eigen::Vector3d::Zero();
    void setState(const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration) {p=position;v=velocity;a=acceleration;}
    Eigen::Vector3d predictPos(double t) const {return p+v*t+0.5*a*t*t;}
    Eigen::Vector3d predictVel(double t) const {return v+a*t;}
    Eigen::Vector3d predictAccel(double) const {return a;}
};
struct PolynomialPredictor {
    Eigen::Vector3d position=Eigen::Vector3d::Zero(), velocity=Eigen::Vector3d::Zero(), acceleration=Eigen::Vector3d::Zero(), jerk=Eigen::Vector3d::Zero(), snap=Eigen::Vector3d::Zero();
    Eigen::Vector3d predictPos(double t) const {const double t2=t*t,t3=t2*t,t4=t3*t;return position+velocity*t+0.5*acceleration*t2+(1.0/6.0)*jerk*t3+(1.0/24.0)*snap*t4;}
    Eigen::Vector3d predictVel(double t) const {return velocity+acceleration*t+0.5*jerk*t*t+(1.0/6.0)*snap*t*t*t;}
    Eigen::Vector3d predictAccel(double t) const {return acceleration+jerk*t+0.5*snap*t*t;}
};
}
