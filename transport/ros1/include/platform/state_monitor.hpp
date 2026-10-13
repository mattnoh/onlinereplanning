/// @file state_monitor.hpp  (ROS1 Noetic branch)
/// @brief Thread-safe state ownership for planner sensor and target streams.
#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>

#include <mutex>
#include <string>
#include <utility>

#include "platform/target_tracker.hpp"
#include "planner_core/ocp_registry.hpp"

class StateMonitor {
public:
    explicit StateMonitor(double target_state_max_age_sec = 0.2)
        : target_state_max_age_sec_(target_state_max_age_sec) {}

    Eigen::VectorXd& droneState() { return drone_state_; }
    platform::target_tracker::TargetState& targetState() { return target_state_; }
    std::mutex& stateMutex() { return state_mutex_; }
    std::mutex& stateMutex() const { return state_mutex_; }
    std::mutex& targetMutex() { return target_mutex_; }

    bool hasFreshTargetState(bool needs_target, const ros::Time& now) const {
        if (!needs_target) return true;
        std::lock_guard<std::mutex> lk(target_mutex_);
        return target_state_.isFresh(now, target_state_max_age_sec_);
    }

    TargetSnapshot getTargetSnapshot(bool /*needs_target*/, const ros::Time& now) const {
        TargetSnapshot s;
        std::lock_guard<std::mutex> lk(target_mutex_);
        s.position = target_state_.position;
        s.velocity = target_state_.velocity;
        s.acceleration = target_state_.acceleration;
        s.angular_velocity = target_state_.angular_velocity;
        s.angular_acceleration = target_state_.angular_acceleration;
        s.orientation = target_state_.orientation;
        s.odom_stamp_sec  = target_state_.odom_timestamp.toSec();
        s.accel_stamp_sec = target_state_.accel_timestamp.toSec();
        s.valid = target_state_.isFresh(now, target_state_max_age_sec_);
        return s;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> getTargetPositionVelocity() const {
        std::lock_guard<std::mutex> lk(target_mutex_);
        return {target_state_.position, target_state_.velocity};
    }

    bool hasTargetTrajectory(const ros::Time& now) const {
        std::lock_guard<std::mutex> lk(target_mutex_);
        return target_state_.predicted_accel.isFresh(now, 2.0);
    }

    target_models::TargetAccelBuffer getTargetAccelBuffer() const {
        std::lock_guard<std::mutex> lk(target_mutex_);
        return target_state_.predicted_accel.accel_buffer;
    }

private:
    Eigen::VectorXd drone_state_ = Eigen::VectorXd::Zero(13);
    platform::target_tracker::TargetState target_state_;

    mutable std::mutex state_mutex_;
    mutable std::mutex target_mutex_;
    double target_state_max_age_sec_ = 0.2;
};
