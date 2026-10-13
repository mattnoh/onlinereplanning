/// @file target_tracker.hpp  (ROS1 Noetic branch)
/// @brief Target-state subscriptions (odom + optional live accel + predicted accel).
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <Eigen/Dense>
#include <mutex>
#include <string>

namespace platform {
namespace target_tracker {

struct TargetState {
    Eigen::Vector3d position          = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity          = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration      = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity  = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector4d orientation       = Eigen::Vector4d(1,0,0,0);

    ros::Time odom_timestamp;
    ros::Time accel_timestamp;

    bool odom_received  = false;
    bool accel_received = false;

    bool isOdomFresh(const ros::Time& now, double max_age_sec = 0.2) const {
        if (!odom_received || odom_timestamp.isZero()) return false;
        const double age = (now - odom_timestamp).toSec();
        const double kClockTol = 0.001;
        return !(age < -kClockTol || age >= max_age_sec);
    }

    bool isAccelFresh(const ros::Time& now, double max_age_sec = 0.2) const {
        if (!accel_received || accel_timestamp.isZero()) return false;
        const double age = (now - accel_timestamp).toSec();
        const double kClockTol = 0.001;
        return !(age < -kClockTol || age >= max_age_sec);
    }

    bool isFresh(const ros::Time& now, double max_age_sec = 0.2) const {
        return isOdomFresh(now, max_age_sec) && isAccelFresh(now, max_age_sec);
    }

    bool valid(const ros::Time& now) const { return isFresh(now); }

    double worstAgeSeconds(const ros::Time& now) const {
        if (!odom_received || !accel_received) return 1e9;
        return std::max((now - odom_timestamp).toSec(),
                        (now - accel_timestamp).toSec());
    }

};

struct Handles {
    ros::Subscriber odom_sub;
    ros::Subscriber accel_sub;

    void reset() {
        odom_sub.shutdown();
        accel_sub.shutdown();
    }
};

inline void setup(
    ros::NodeHandle&       nh,
    const std::string&     odom_topic,
    const std::string&     accel_topic,
    TargetState&           target_state,
    std::mutex&            target_mutex,
    Handles&               handles)
{
    handles.odom_sub = nh.subscribe<nav_msgs::Odometry>(
        odom_topic, 10,
        [&target_state, &target_mutex](const nav_msgs::Odometry::ConstPtr& msg) {
            std::lock_guard<std::mutex> lk(target_mutex);

            target_state.position << msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z;
            target_state.velocity << msg->twist.twist.linear.x,
                                     msg->twist.twist.linear.y,
                                     msg->twist.twist.linear.z;
            Eigen::Vector3d prev_omega = target_state.angular_velocity;
            target_state.angular_velocity << msg->twist.twist.angular.x,
                                             msg->twist.twist.angular.y,
                                             msg->twist.twist.angular.z;
            target_state.orientation << msg->pose.pose.orientation.w,
                                        msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z;

            ros::Time new_ts = (msg->header.stamp.isZero()) ? ros::Time::now() : msg->header.stamp;

            if (target_state.odom_received) {
                double dt = (new_ts - target_state.odom_timestamp).toSec();
                if (dt > 1e-6 && dt < 0.5) {
                    target_state.angular_acceleration =
                        (target_state.angular_velocity - prev_omega) / dt;
                }
            }
            target_state.odom_received = true;
            target_state.odom_timestamp = new_ts;
        });

    if (!accel_topic.empty()) {
        handles.accel_sub = nh.subscribe<geometry_msgs::AccelStamped>(
            accel_topic, 10,
            [&target_state, &target_mutex](const geometry_msgs::AccelStamped::ConstPtr& msg) {
                std::lock_guard<std::mutex> lk(target_mutex);
                target_state.acceleration << msg->accel.linear.x,
                                             msg->accel.linear.y,
                                             msg->accel.linear.z;
                target_state.angular_acceleration << msg->accel.angular.x,
                                                     msg->accel.angular.y,
                                                     msg->accel.angular.z;
                target_state.accel_received = true;
                target_state.accel_timestamp =
                    msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
            });
    }

    ROS_INFO("[TargetTracker] Subscribed odom: %s  accel: %s",
        odom_topic.c_str(),
        accel_topic.empty() ? "<disabled>" : accel_topic.c_str());
}

}  // namespace target_tracker
}  // namespace platform
