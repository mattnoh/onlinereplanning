/// @file planner_core/adapters.hpp
/// @brief ROS-free frame validation and platform-command conversion.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

#include <algorithm>
#include <string>

#include "dynamics/quad_6dof_target_frame.h"
#include "planner_core/types.hpp"

namespace planner_core {

struct FrameCheck {
    bool ok = true;
    std::string reason;
};

inline const char* frameLabel(OCPDescriptor::Coordinates c) {
    return c == OCPDescriptor::Coordinates::TargetRelative ? "relative" : "absolute";
}
inline FrameCheck validateFrameContract(const std::string& input,const std::string& world) {
    return input == world ? FrameCheck{} : FrameCheck{false,"input state is not in the configured world frame"};
}
inline Eigen::VectorXd prepareOcpState(const OCPDescriptor& desc,const Eigen::VectorXd& world,const TargetSnapshot& target) {
    Eigen::VectorXd x=world;
    if(desc.coordinates==OCPDescriptor::Coordinates::TargetRelative) {
        if(!target.valid) throw std::runtime_error("target-relative state requires a valid target");
        x.head(3)-=target.position;x.segment(3,3)-=target.velocity;
    }
    return x;
}
inline Eigen::VectorXd reconstructWorldState(const OCPDescriptor& desc,const Eigen::VectorXd& state,const TargetSnapshot& target) {
    Eigen::VectorXd x=state;
    if(desc.coordinates==OCPDescriptor::Coordinates::TargetRelative) {
        if(!target.valid) throw std::runtime_error("world reconstruction requires a valid target");
        x.head(3)+=target.position;x.segment(3,3)+=target.velocity;
    }
    return x;
}
// Static world-frame alignment. Body rates are unaffected by changing world axes.
inline Eigen::VectorXd alignWorldState(Eigen::VectorXd x,const Eigen::Quaterniond& rotation,const Eigen::Vector3d& translation,bool body_velocity) {
    if(x.size()<13 || !x.allFinite() || !rotation.coeffs().allFinite() || rotation.norm()<1e-8)
        throw std::runtime_error("invalid estimator state/transform");
    Eigen::Quaterniond q(x(6),x(7),x(8),x(9));if(q.norm()<1e-8)throw std::runtime_error("invalid estimator quaternion");q.normalize();
    if(body_velocity)x.segment<3>(3)=q*x.segment<3>(3).eval();
    const auto r=rotation.normalized();x.head<3>()=r*x.head<3>().eval()+translation;
    x.segment<3>(3)=r*x.segment<3>(3).eval();q=r*q;
    x.segment<4>(6)<<q.w(),q.x(),q.y(),q.z();return x;
}
inline Eigen::Vector4d makeCrazyflieHoverCommand(const OCPDescriptor& desc,const Eigen::VectorXd& state,const Eigen::VectorXd& control,const TargetSnapshot& target) {
    Eigen::VectorXd world=reconstructWorldState(desc,state,target);
    Eigen::Quaterniond q(world(6),world(7),world(8),world(9));q.normalize();
    Eigen::Vector3d body=q.conjugate()*world.segment<3>(3);
    Eigen::Vector4d hover;hover<<std::clamp(body.x(),-1.0,1.0),std::clamp(body.y(),-1.0,1.0),std::clamp(world(2),0.1,3.0),control.size()>3?control(3):0.0;
    return hover;
}

inline PlannerCommand makePlannerCommand(Platform platform,
                                         const OCPDescriptor& desc,
                                         const Eigen::VectorXd& state,
                                         const Eigen::VectorXd& control,
                                         const TargetSnapshot& target) {
    PlannerCommand cmd;
    cmd.state = state;
    cmd.control = control;

    if (desc.command_mode == OCPDescriptor::CommandMode::CmdBodyRate) {
        if (platform == Platform::Crazyflie) {
            cmd.kind = CommandKind::Hover;
            cmd.hover = makeCrazyflieHoverCommand(desc, state, control, target);
        } else {
            cmd.kind = CommandKind::BodyRate;
        }
        return cmd;
    }

    cmd.kind = CommandKind::FullState;
    cmd.state = reconstructWorldState(desc,state,target);
    if(desc.world_acceleration)cmd.acceleration=desc.world_acceleration(cmd.state,control);
    return cmd;
}

}  // namespace planner_core

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
