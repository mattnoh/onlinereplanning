/// Target state subscriptions: odometry and optional live acceleration.
/// The OCP predictor consumes the latest position/velocity/acceleration snapshot.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <string>

namespace platform {
namespace target_tracker {

struct TargetState {
    Eigen::Vector3d position     = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  // Ω_N in world frame
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero(); // β_N in world frame
    Eigen::Vector4d orientation  = Eigen::Vector4d(1,0,0,0);     // target quat [qw,qx,qy,qz]

    // Separate timestamps per topic so each can be checked independently.
    rclcpp::Time odom_timestamp {0, 0, RCL_ROS_TIME};
    rclcpp::Time accel_timestamp{0, 0, RCL_ROS_TIME};

    // Startup flags: true after the first message of each type ever arrives.
    // Used only to gate early-startup "waiting for first message" warnings.
    // Do NOT use these to assess current freshness — call isFresh() instead.
    bool odom_received  = false;
    bool accel_received = false;

    /// Odom-only freshness check, used by OCPs that only need target pose/velocity.
    bool isOdomFresh(const rclcpp::Time& now, double max_age_sec = 0.2) const {
        if (!odom_received) return false;
        if (odom_timestamp.nanoseconds() <= 0) return false;
        const double odom_age = (now - odom_timestamp).seconds();
        const double kClockTol = 0.001;
        return !(odom_age < -kClockTol || odom_age >= max_age_sec);
    }

    /// Live acceleration freshness check (diagnostic topic).
    bool isAccelFresh(const rclcpp::Time& now, double max_age_sec = 0.2) const {
        if (!accel_received) return false;
        if (accel_timestamp.nanoseconds() <= 0) return false;
        const double accel_age = (now - accel_timestamp).seconds();
        const double kClockTol = 0.001;
        return !(accel_age < -kClockTol || accel_age >= max_age_sec);
    }

    /// Returns true only if BOTH topics have delivered at least one message
    /// AND both of their most recent messages are younger than max_age_sec.
    ///
    /// max_age_sec default = 0.2 s (5 Hz minimum acceptable rate).
    /// Raise to 0.5 s for slow targets; lower to 0.1 s for tight requirements.
    bool isFresh(const rclcpp::Time& now, double max_age_sec = 0.2) const {
        return isOdomFresh(now, max_age_sec) && isAccelFresh(now, max_age_sec);
    }

    /// Convenience: is the state currently valid (fresh with default tolerance)?
    bool valid(const rclcpp::Time& now) const { return isFresh(now); }

    /// Age of the older of the two topics — useful for diagnostics / logging.
    double worstAgeSeconds(const rclcpp::Time& now) const {
        if (!odom_received || !accel_received) return 1e9;
        const double oa = (now - odom_timestamp).seconds();
        const double aa = (now - accel_timestamp).seconds();
        return std::max(oa, aa);
    }

};

struct Handles {
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_sub;

    void reset() {
        odom_sub.reset();
        accel_sub.reset();
    }
};

inline void setup(
    rclcpp::Node*                        node,
    rclcpp::CallbackGroup::SharedPtr     callback_group,
    const std::string&                   odom_topic,
    const std::string&                   accel_topic,
    TargetState&                         target_state,
    std::mutex&                          target_mutex,
    Handles&                             handles)
{
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group;

    // ── Odometry callback ────────────────────────────────────────────────────
    handles.odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [&target_state, &target_mutex, node](
            const nav_msgs::msg::Odometry::SharedPtr msg)
        {
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

            rclcpp::Time new_timestamp =
                (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                    ? rclcpp::Time(msg->header.stamp)
                    : node->now();

            if (target_state.odom_received) {
                double dt = (new_timestamp - target_state.odom_timestamp).seconds();
                if (dt > 1e-6 && dt < 0.5) {
                    target_state.angular_acceleration =
                        (target_state.angular_velocity - prev_omega) / dt;
                }
            }

            target_state.odom_received = true;

            // Update ONLY the odom timestamp; accel timestamp is unaffected.
            target_state.odom_timestamp = new_timestamp;
        },
        opts);

    // ── Acceleration callback ────────────────────────────────────────────────
    if (!accel_topic.empty()) {
        handles.accel_sub = node->create_subscription<geometry_msgs::msg::AccelStamped>(
            accel_topic, 10,
            [&target_state, &target_mutex, node](
                const geometry_msgs::msg::AccelStamped::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(target_mutex);

                target_state.acceleration << msg->accel.linear.x,
                                             msg->accel.linear.y,
                                             msg->accel.linear.z;
                
                // Read optional angular acceleration from the AccelStamped message.
                // This overrides the finite-difference estimate from odom if both exist.
                target_state.angular_acceleration << msg->accel.angular.x,
                                                     msg->accel.angular.y,
                                                     msg->accel.angular.z;
                
                target_state.accel_received = true;

                // Update ONLY the accel timestamp; odom timestamp is unaffected.
                target_state.accel_timestamp =
                    (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                        ? rclcpp::Time(msg->header.stamp)
                        : node->now();
            },
            opts);
    }

    RCLCPP_INFO(node->get_logger(),
        "[TargetTracker] Subscribed odom: %s  accel(diagnostics): %s",
        odom_topic.c_str(),
        accel_topic.empty() ? "<disabled>" : accel_topic.c_str());
}

} // namespace target_tracker
} // namespace platform
