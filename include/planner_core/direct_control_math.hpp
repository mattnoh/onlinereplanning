#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace planner_core {
inline Eigen::VectorXd bodyOdometryToWorld(Eigen::VectorXd x) {
    if(x.size()<13||!x.allFinite())throw std::runtime_error("invalid body-frame odometry");
    const Eigen::Quaterniond q(x(6),x(7),x(8),x(9));
    if(q.norm()<1e-8)throw std::runtime_error("invalid odometry quaternion");
    x.segment<3>(3)=q.normalized()*x.segment<3>(3).eval();return x;
}
inline double normalizedThrust(double acceleration,double hover) {
    if(!std::isfinite(acceleration)||!std::isfinite(hover))return 0;
    return std::clamp(acceleration*hover/9.81,0.0,1.0);
}
}
