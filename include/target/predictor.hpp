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

}
