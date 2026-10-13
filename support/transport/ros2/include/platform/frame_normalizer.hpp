#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include "planner_core/adapters.hpp"
#include <optional>
namespace platform {
class FrameNormalizer {
    rclcpp::Node* node_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    std::string world_;
public:
    FrameNormalizer(rclcpp::Node* node,std::string world):node_(node),buffer_(node->get_clock()),listener_(buffer_),world_(std::move(world)){}
    Eigen::VectorXd state(const Eigen::VectorXd& x,const std::string& frame,const rclcpp::Time& stamp,bool body=false) {
        if(frame.empty())throw std::runtime_error("estimator frame_id is empty");
        Eigen::Quaterniond rotation=Eigen::Quaterniond::Identity();Eigen::Vector3d translation=Eigen::Vector3d::Zero();
        if(frame!=world_) {
            auto tf=buffer_.lookupTransform(world_,frame,stamp,rclcpp::Duration::from_seconds(0));
            const auto& q=tf.transform.rotation;const auto& p=tf.transform.translation;
            rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);translation<<p.x,p.y,p.z;
        }
        return planner_core::alignWorldState(x,rotation,translation,body);
    }
    std::optional<nav_msgs::msg::Odometry> odometry(const nav_msgs::msg::Odometry& msg,bool body) {
        try {
            Eigen::VectorXd x(13);const auto& p=msg.pose.pose.position;const auto& q=msg.pose.pose.orientation;
            const auto& v=msg.twist.twist.linear;const auto& w=msg.twist.twist.angular;
            x<<p.x,p.y,p.z,v.x,v.y,v.z,q.w,q.x,q.y,q.z,w.x,w.y,w.z;
            x=state(x,msg.header.frame_id,msg.header.stamp,body);
            auto out=msg;out.header.frame_id=world_;
            auto& op=out.pose.pose.position;op.x=x(0);op.y=x(1);op.z=x(2);
            auto& ov=out.twist.twist.linear;ov.x=x(3);ov.y=x(4);ov.z=x(5);
            auto& oq=out.pose.pose.orientation;oq.w=x(6);oq.x=x(7);oq.y=x(8);oq.z=x(9);return out;
        }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,"Rejecting odometry frame: %s",e.what());return std::nullopt;}
    }
    std::optional<geometry_msgs::msg::AccelStamped> acceleration(const geometry_msgs::msg::AccelStamped& msg) {
        try {
            Eigen::VectorXd x=Eigen::VectorXd::Zero(13);x(6)=1;
            x.segment<3>(3)<<msg.accel.linear.x,msg.accel.linear.y,msg.accel.linear.z;
            x=state(x,msg.header.frame_id,msg.header.stamp);
            auto out=msg;out.header.frame_id=world_;out.accel.linear.x=x(3);out.accel.linear.y=x(4);out.accel.linear.z=x(5);return out;
        }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,"Rejecting acceleration frame: %s",e.what());return std::nullopt;}
    }
    Eigen::VectorXd command(const Eigen::VectorXd& world,const std::string& frame) {
        // World -> PX4 local ENU, including attitude/yaw.
        if(frame==world_)return world;
        auto tf=buffer_.lookupTransform(frame,world_,rclcpp::Time(0,0,RCL_ROS_TIME),rclcpp::Duration::from_seconds(0));
        const auto& q=tf.transform.rotation;const auto& p=tf.transform.translation;
        return planner_core::alignWorldState(world,Eigen::Quaterniond(q.w,q.x,q.y,q.z),Eigen::Vector3d(p.x,p.y,p.z),false);
    }
};
}
