/// @file target_publisher.cpp  (ROS1 Noetic branch)
/// @brief Publishes synthetic target state for the CoManDO planner.
///
/// Source: "model" (circle or figure8) — fully synthetic, no external input.
///
/// Publishes:
///   /target/odom   (nav_msgs/Odometry)
///   /target/accel  (geometry_msgs/AccelStamped)
///
/// Optional relative odometry outputs (controlled by planning_frame param):
///   "shifted"      → /drone/relative_odometry
///   "body_frame"   → /drone/body_relative_odom
///   "target_frame" → /drone/target_frame_odom
///
/// Simulated relative sensing (orthogonal to planning_frame), two stages that
/// talk only over a ROS topic so a real estimator can replace stage 1:
///   sim_relative_estimate=true  → stage 1 publishes the body-frame relative
///       target measurement on relative_estimate_topic
///   target_source="relative_estimate" → stage 2 subscribes to that topic and
///       synthesizes world-frame /target/odom + /target/accel from it
///       (target z pinned to pad_z); ground truth moves to /target/odom_gt.
/// See docs/dynamic_target_hardware_integration.md for the message contract.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <cmath>

#include "target/circular_target.hpp"
#include "target/figure8_target.hpp"
#include "target/straight_target.hpp"
#include "target/stop_and_go_target.hpp"
#include "target/segmented_accel_target.hpp"
#include "target/target_model_config.hpp"
#include <random>

namespace {
constexpr double DEG2RAD = M_PI / 180.0;

std::string normalizePlanningFrame(const std::string& raw) {
    if (raw.empty() || raw == "none" || raw == "world") return "world";
    if (raw == "shifted_world") return "shifted";
    return raw;
}
}

class TargetPublisherNode {
public:
    explicit TargetPublisherNode(ros::NodeHandle& nh) : nh_(nh) {
        std::string traj, frame_raw, drone_odom_mode_legacy;
        nh_.param<std::string>("target_trajectory", traj, "circle");
        nh_.param<double>     ("target_yaw_rate",   target_yaw_rate_, 0.3);
        nh_.param<double>     ("publish_hz",         publish_hz_,      100.0);
        nh_.param<std::string>("frame_id",           frame_id_,        "world");
        nh_.param<std::string>("drone_odom_topic",   drone_odom_topic_, "/drone/odom");
        nh_.param<std::string>("drone_pose_topic",   drone_pose_topic_, "/drone/pose");
        nh_.param<std::string>("planning_frame",     frame_raw,        "world");
        nh_.param<std::string>("drone_odom_mode",    drone_odom_mode_legacy, "");
        nh_.param<bool>       ("debug_body_relative_trace", debug_body_relative_trace_, false);
        nh_.param<std::string>("body_relative_input_angular_unit", body_relative_angular_unit_, "deg_s");
        nh_.param<std::string>("target_source",           target_source_,           "ground_truth");
        nh_.param<bool>       ("sim_relative_estimate",   sim_relative_estimate_,   false);
        nh_.param<bool>       ("sim_visibility_gate",     sim_visibility_gate_,     true);
        nh_.param<double>     ("sim_camera_hfov_deg",     sim_camera_hfov_deg_,     30.0);
        nh_.param<double>     ("sim_camera_vfov_deg",     sim_camera_vfov_deg_,     22.0);
        nh_.param<double>     ("sim_camera_max_range",    sim_camera_max_range_,    12.0);
        nh_.param<double>     ("pad_z",                   pad_z_,                   0.0);
        nh_.param<std::string>("relative_estimate_topic", relative_estimate_topic_, "/drone/relative_target_estimate");
        nh_.param<bool>       ("publish_ground_truth_debug", publish_gt_debug_,     true);
        nh_.param<std::string>("drone_odom_twist_frame",  drone_odom_twist_frame_,  "body");

        if (target_source_ != "ground_truth" && target_source_ != "relative_estimate") {
            ROS_WARN("[TargetPublisher] unknown target_source='%s', falling back to ground_truth",
                     target_source_.c_str());
            target_source_ = "ground_truth";
        }
        if (drone_odom_twist_frame_ != "body" && drone_odom_twist_frame_ != "world") {
            ROS_WARN("[TargetPublisher] unknown drone_odom_twist_frame='%s', falling back to body",
                     drone_odom_twist_frame_.c_str());
            drone_odom_twist_frame_ = "body";
        }
        synthesize_from_estimate_ = (target_source_ == "relative_estimate");

        if (!frame_raw.empty())
            drone_odom_mode_ = normalizePlanningFrame(frame_raw);
        else if (!drone_odom_mode_legacy.empty())
            drone_odom_mode_ = normalizePlanningFrame(drone_odom_mode_legacy);
        else
            drone_odom_mode_ = "world";

        target_trajectory_ = traj;

        // Motion-shape params (none of these existed before; the models used
        // their in-header defaults and could not be configured at runtime).
        double c_x, c_y, c_z, radius, omega, amp_x, amp_y, speed, heading;
        double sg_cruise, sg_ramp, sg_dwell, pa_accel, pa_hold, ra_sigma, ra_hold, vmax;
        int    motion_seed;
        nh_.param<double>("target_center_x", c_x, 0.0);
        nh_.param<double>("target_center_y", c_y, 0.0);
        nh_.param<double>("target_center_z", c_z, 0.2);
        nh_.param<double>("target_radius",   radius, 1.0);
        nh_.param<double>("target_omega",    omega, 0.4);
        nh_.param<double>("target_amp_x",    amp_x, 1.0);
        nh_.param<double>("target_amp_y",    amp_y, 1.0);
        nh_.param<double>("target_speed",    speed, 0.4);
        nh_.param<double>("target_heading",  heading, 0.0);
        nh_.param<double>("target_v_max",    vmax, 0.5);
        double span; nh_.param<double>("target_span", span, 2.0);
        nh_.param<double>("target_cruise_s", sg_cruise, 3.0);
        nh_.param<double>("target_ramp_s",   sg_ramp, 1.0);
        nh_.param<double>("target_dwell_s",  sg_dwell, 2.0);
        nh_.param<double>("target_accel_mag", pa_accel, 0.15);
        nh_.param<double>("target_accel_hold", pa_hold, 2.0);
        nh_.param<double>("target_accel_sigma", ra_sigma, 0.20);
        nh_.param<double>("target_random_hold", ra_hold, 2.0);
        nh_.param<int>   ("target_motion_seed", motion_seed, 1);
        const Eigen::Vector3d ctr(c_x, c_y, c_z);

        // Per-trial shape randomisation. WITHOUT this, target_motion_seed
        // reached only the random_accel branch, so the other five motions flew
        // a byte-identical target path in every trial of a campaign cell and
        // the only trial-to-trial variation was SITL bring-up jitter shifting
        // where the target happened to be at engagement. Straight and
        // stop_and_go were the worst case: heading and speed profile were
        // literally constant across all 20 trials.
        //
        // Drawn from one generator in a fixed order so a (motion, level, trial)
        // triple stays exactly reproducible. `shape_random` defaults ON; set it
        // false to reproduce a pre-fix run.
        bool shape_random; nh_.param<bool>("target_shape_random", shape_random, true);
        std::mt19937 shape_rng(static_cast<unsigned>(motion_seed));
        std::uniform_real_distribution<double> u2pi(0.0, 2.0 * M_PI);
        std::uniform_int_distribution<int>     u8(0, 7);
        const double rnd_phase   = shape_random ? u2pi(shape_rng) : 0.0;
        const double rnd_heading = shape_random ? u2pi(shape_rng) : heading;
        const int    rnd_k0      = shape_random ? u8(shape_rng)   : 0;
        // Phase offset in SECONDS, spanning a full shuttle cycle (4*span at
        // `speed`) so the drone can engage anywhere along the stroke.
        const double shuttle_T   = (speed > 1e-9 && span > 0.0)
                                       ? 4.0 * span / speed : 0.0;
        std::uniform_real_distribution<double> ushuttle(0.0, std::max(shuttle_T, 1e-9));
        const double rnd_t_off   = shape_random ? ushuttle(shape_rng) : 0.0;
        ROS_INFO("[TargetPublisher] shape_random=%d seed=%d -> phase=%.3f rad, "
                 "heading=%.3f rad, k0=%d, t_offset=%.2f s",
                 static_cast<int>(shape_random), motion_seed, rnd_phase,
                 rnd_heading, rnd_k0, rnd_t_off);

        // One SyntheticTarget replaces the old if/else over eagerly-constructed
        // members. An unknown key is now FATAL instead of silently degrading to
        // a circle (the previous behaviour would have made a mis-typed campaign
        // scenario look like a passing circle run).
        if (traj == "circle") {
            target_models::CircularTarget m;
            m.center = ctr; m.R = radius; m.omega = omega; m.phi0 = rnd_phase;
            motion_ = target_config::SyntheticTarget::from(m);
        } else if (traj == "figure8") {
            target_models::Figure8Target m;
            m.center = ctr; m.amp_x = amp_x; m.amp_y = amp_y; m.omega = omega;
            m.phi0 = rnd_phase;
            motion_ = target_config::SyntheticTarget::from(m);
        } else if (traj == "straight") {
            target_models::StraightTarget m;
            m.center = ctr; m.speed = speed; m.heading = rnd_heading; m.span = span;
            m.t_offset = rnd_t_off;
            motion_ = target_config::SyntheticTarget::from(m);
        } else if (traj == "stop_and_go") {
            target_models::StopAndGoTarget m;
            m.center = ctr; m.speed = speed; m.heading = rnd_heading;
            m.t_cruise = sg_cruise; m.t_ramp = sg_ramp; m.t_dwell = sg_dwell; m.span = span;
            m.t_offset = rnd_t_off;
            motion_ = target_config::SyntheticTarget::from(m);
        } else if (traj == "piecewise_accel") {
            motion_ = target_config::SyntheticTarget::from(
                target_models::makePiecewiseAccelTarget(ctr, pa_accel, pa_hold, vmax, speed,
                                                       rnd_heading, rnd_k0));
        } else if (traj == "random_accel") {
            motion_ = target_config::SyntheticTarget::from(
                target_models::makeRandomAccelTarget(ctr, ra_sigma, ra_hold, vmax, speed,
                                                     static_cast<unsigned>(motion_seed),
                                                     rnd_heading));
        } else {
            ROS_FATAL("[TargetPublisher] unknown target_trajectory '%s' (expected: circle, "
                      "figure8, straight, stop_and_go, piecewise_accel, random_accel)",
                      traj.c_str());
            throw std::runtime_error("unknown target_trajectory: " + traj);
        }

        // Target-state estimate uncertainty. Per-run constant bias (drawn once)
        // plus per-sample white noise, applied to what the PLANNER sees; the
        // ground-truth debug topics stay clean so metrics score against truth.
        int noise_seed;
        nh_.param<double>("target_noise_sigma_pos", noise_sigma_pos_, 0.0);
        nh_.param<double>("target_noise_sigma_vel", noise_sigma_vel_, 0.0);
        nh_.param<double>("target_noise_sigma_acc", noise_sigma_acc_, 0.0);
        nh_.param<double>("target_noise_bias_frac", noise_bias_frac_, 0.5);
        nh_.param<int>   ("target_noise_seed",      noise_seed, 1);
        noise_rng_.seed(static_cast<unsigned>(noise_seed));
        {
            std::normal_distribution<double> nb(0.0, 1.0);
            // NOTE the explicit return type. With `auto`, this lambda deduces
            // an Eigen EXPRESSION TEMPLATE that holds a reference to the
            // temporary Vector3d, which dies at the end of the return
            // statement -- the stored bias then reads freed memory. That
            // produced a constant ~5e254 bias on x (y happened to survive) and
            // blew every solve up to constraint=nan / max_rel_p=inf.
            auto drawBias = [&](double sig) -> Eigen::Vector3d {
                return Eigen::Vector3d(nb(noise_rng_), nb(noise_rng_), 0.0)
                       * (noise_bias_frac_ * sig);
            };
            noise_bias_pos_ = drawBias(noise_sigma_pos_);
            noise_bias_vel_ = drawBias(noise_sigma_vel_);
            noise_bias_acc_ = drawBias(noise_sigma_acc_);
        }
        if (!noise_bias_pos_.allFinite() || !noise_bias_vel_.allFinite() ||
            !noise_bias_acc_.allFinite()) {
            ROS_FATAL("[TargetPublisher] non-finite noise bias computed "
                      "(p=[%g %g] v=[%g %g] a=[%g %g])",
                      noise_bias_pos_.x(), noise_bias_pos_.y(),
                      noise_bias_vel_.x(), noise_bias_vel_.y(),
                      noise_bias_acc_.x(), noise_bias_acc_.y());
            throw std::runtime_error("non-finite target noise bias");
        }
        noise_active_ = (noise_sigma_pos_ > 0.0 || noise_sigma_vel_ > 0.0 ||
                         noise_sigma_acc_ > 0.0);
        if (noise_active_) {
            ROS_WARN("[TargetPublisher] target-estimate NOISE active: sigma p/v/a = "
                     "%.3f/%.3f/%.3f, bias frac %.2f, seed %d",
                     noise_sigma_pos_, noise_sigma_vel_, noise_sigma_acc_,
                     noise_bias_frac_, noise_seed);
        }

        t0_ = ros::Time::now().toSec();

        odom_pub_  = nh_.advertise<nav_msgs::Odometry>("/target/odom",  10);
        accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/target/accel", 10);

        // In relative-estimate mode /target/odom carries the synthesized target,
        // so ground truth is diverted to the _gt topics for round-trip checks.
        if (synthesize_from_estimate_ && publish_gt_debug_) {
            gt_odom_pub_  = nh_.advertise<nav_msgs::Odometry>("/target/odom_gt", 10);
            gt_accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/target/accel_gt", 10);
        }
        if (sim_relative_estimate_) {
            rel_estimate_pub_ = nh_.advertise<nav_msgs::Odometry>(relative_estimate_topic_, 10);
        }

        if (drone_odom_mode_ == "shifted") {
            abs_rel_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/drone/relative_odometry", 10);
        } else if (drone_odom_mode_ == "body_frame") {
            body_rel_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/drone/body_relative_odom", 10);
        } else if (drone_odom_mode_ == "target_frame") {
            target_frame_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/drone/target_frame_odom", 10);
        }

        if (drone_odom_mode_ != "world" || sim_relative_estimate_ || synthesize_from_estimate_) {
            drone_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
                drone_odom_topic_, 10,
                [this](const nav_msgs::Odometry::ConstPtr& msg) {
                    std::lock_guard<std::mutex> lk(drone_mutex_);
                    last_drone_odom_ = *msg; has_drone_odom_ = true;
                });
        }
        if (drone_odom_mode_ == "body_frame" || drone_odom_mode_ == "target_frame") {
            drone_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
                drone_pose_topic_, 10,
                [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    std::lock_guard<std::mutex> lk(drone_mutex_);
                    last_drone_pose_ = *msg; has_drone_pose_ = true;
                });
        }

        if (synthesize_from_estimate_) {
            rel_estimate_sub_ = nh_.subscribe(relative_estimate_topic_, 10,
                                              &TargetPublisherNode::relativeEstimateCallback, this);
        }

        ROS_INFO("[TargetPublisher] trajectory=%s  planning_frame=%s  %.0f Hz",
            target_trajectory_.c_str(), drone_odom_mode_.c_str(), publish_hz_);
        ROS_INFO("[TargetPublisher] target_source=%s  sim_relative_estimate=%s  pad_z=%.3f"
                 "  rel_topic=%s  twist_frame=%s",
            target_source_.c_str(), sim_relative_estimate_ ? "true" : "false", pad_z_,
            relative_estimate_topic_.c_str(), drone_odom_twist_frame_.c_str());
        if (sim_relative_estimate_) {
            ROS_INFO("[TargetPublisher] simulated camera gate=%s  half-FOV=(%.1f, %.1f) deg"
                     "  max_range=%.2f m",
                     sim_visibility_gate_ ? "on" : "off",
                     sim_camera_hfov_deg_, sim_camera_vfov_deg_, sim_camera_max_range_);
        }

        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_hz_),
                                 &TargetPublisherNode::timerCallback, this);
    }

private:
    void timerCallback(const ros::TimerEvent&) {
        const ros::Time stamp = ros::Time::now();
        const double t_rel = stamp.toSec() - t0_;

        Eigen::Vector3d p = motion_.pos(t_rel);
        Eigen::Vector3d v = motion_.vel(t_rel);
        Eigen::Vector3d a = motion_.accel(t_rel);
        const double psi = target_yaw_rate_ * t_rel;
        const Eigen::Quaterniond q_tgt(std::cos(psi/2.0), 0.0, 0.0, std::sin(psi/2.0));
        last_target_quat_ = q_tgt;

        // Ground truth: to /target/* normally, to /target/*_gt when the
        // synthesized estimate owns /target/*.
        if (!synthesize_from_estimate_) {
            // The planner-facing topics carry the (optionally corrupted)
            // ESTIMATE; /target/*_gt always carries truth so the campaign can
            // score touchdown error against the real target, not the estimate.
            if (noise_active_) {
                publishGroundTruth(stamp,
                                   corrupt(p, noise_bias_pos_, noise_sigma_pos_),
                                   corrupt(v, noise_bias_vel_, noise_sigma_vel_),
                                   corrupt(a, noise_bias_acc_, noise_sigma_acc_),
                                   q_tgt, odom_pub_, accel_pub_);
                if (publish_gt_debug_) {
                    publishGroundTruth(stamp, p, v, a, q_tgt, gt_odom_pub_, gt_accel_pub_);
                }
            } else {
                publishGroundTruth(stamp, p, v, a, q_tgt, odom_pub_, accel_pub_);
            }
        } else if (publish_gt_debug_) {
            publishGroundTruth(stamp, p, v, a, q_tgt, gt_odom_pub_, gt_accel_pub_);
        }

        // Relative odometry outputs
        nav_msgs::Odometry drone_odom_copy;
        geometry_msgs::PoseStamped drone_pose_copy;
        bool have_odom = false, have_pose = false;
        {
            std::lock_guard<std::mutex> lk(drone_mutex_);
            have_odom = has_drone_odom_; have_pose = has_drone_pose_;
            if (have_odom) drone_odom_copy = last_drone_odom_;
            if (have_pose) drone_pose_copy = last_drone_pose_;
        }

        // Stage 1 — simulated body-frame relative measurement of the target.
        // Uses the drone odom's own attitude (not drone_pose_topic) so stage 1
        // and stage 2 always rotate with the same source.
        if (sim_relative_estimate_) {
            if (!have_odom) {
                ROS_WARN_THROTTLE(2.0, "[TargetPublisher] sim_relative_estimate: waiting for drone odom on %s",
                                  drone_odom_topic_.c_str());
            } else if (odomAgeSec(drone_odom_copy, stamp) > kDroneOdomMaxAgeSec) {
                ROS_WARN_THROTTLE(2.0, "[TargetPublisher] sim_relative_estimate: drone odom stale (%.2f s)",
                                  odomAgeSec(drone_odom_copy, stamp));
            } else {
                const Eigen::Matrix3d R = droneRotation(drone_odom_copy);
                const Eigen::Vector3d p_d(drone_odom_copy.pose.pose.position.x,
                                          drone_odom_copy.pose.pose.position.y,
                                          drone_odom_copy.pose.pose.position.z);
                const Eigen::Vector3d v_d = droneWorldVelocity(drone_odom_copy, R);
                // Pure frame projection (no omega x r): matches de-rotated camera
                // measurements and inverts exactly in stage 2.
                const Eigen::Vector3d p_rel_b = R.transpose() * (p - p_d);
                const Eigen::Vector3d v_rel_b = R.transpose() * (v - v_d);

                const bool detected = !sim_visibility_gate_ || cameraDetects(p_rel_b);
                reportDetectionTransition(detected, p_rel_b.norm());
                if (detected) {
                    nav_msgs::Odometry est;
                    est.header.stamp    = stamp;
                    est.header.frame_id = "drone_body";
                    est.child_frame_id  = "target_in_drone_body";
                    est.pose.pose.position.x = p_rel_b.x();
                    est.pose.pose.position.y = p_rel_b.y();
                    est.pose.pose.position.z = p_rel_b.z();
                    est.pose.pose.orientation.w = 1.0;
                    est.pose.covariance[21] = -1.0;   // orientation unmeasured
                    est.twist.twist.linear.x = v_rel_b.x();
                    est.twist.twist.linear.y = v_rel_b.y();
                    est.twist.twist.linear.z = v_rel_b.z();
                    rel_estimate_pub_.publish(est);
                }
            }
        }

        if (drone_odom_mode_ == "shifted" && have_odom) {
            nav_msgs::Odometry rel = drone_odom_copy;
            rel.header.stamp = stamp; rel.header.frame_id = frame_id_; rel.child_frame_id = "drone_absolute_relative";
            rel.pose.pose.position.x -= p.x(); rel.pose.pose.position.y -= p.y(); rel.pose.pose.position.z -= p.z();
            rel.twist.twist.linear.x -= v.x(); rel.twist.twist.linear.y -= v.y(); rel.twist.twist.linear.z -= v.z();
            abs_rel_odom_pub_.publish(rel);
        }

        if (drone_odom_mode_ == "body_frame" && have_odom && have_pose) {
            const Eigen::Vector3d tgt_w(p.x(), p.y(), p.z());
            const Eigen::Vector3d drn_w(drone_pose_copy.pose.position.x, drone_pose_copy.pose.position.y, drone_pose_copy.pose.position.z);
            const Eigen::Vector3d drn_v(drone_odom_copy.twist.twist.linear.x, drone_odom_copy.twist.twist.linear.y, drone_odom_copy.twist.twist.linear.z);
            Eigen::Quaterniond q(drone_pose_copy.pose.orientation.w, drone_pose_copy.pose.orientation.x,
                                 drone_pose_copy.pose.orientation.y, drone_pose_copy.pose.orientation.z);
            q.normalize();
            const Eigen::Matrix3d R = q.toRotationMatrix();
            const Eigen::Vector3d p_t_body = R.transpose() * (tgt_w - drn_w);
            const Eigen::Vector3d v_d_body = R.transpose() * (drn_v - v);
            const Eigen::Quaterniond q_NB = last_target_quat_.conjugate() * q;

            nav_msgs::Odometry body_rel;
            body_rel.header.stamp = stamp; body_rel.header.frame_id = "drone_body"; body_rel.child_frame_id = "body_relative";
            body_rel.pose.pose.position.x = p_t_body.x(); body_rel.pose.pose.position.y = p_t_body.y(); body_rel.pose.pose.position.z = p_t_body.z();
            body_rel.pose.pose.orientation.w = q_NB.w(); body_rel.pose.pose.orientation.x = q_NB.x();
            body_rel.pose.pose.orientation.y = q_NB.y(); body_rel.pose.pose.orientation.z = q_NB.z();
            body_rel.twist.twist.linear.x = v_d_body.x(); body_rel.twist.twist.linear.y = v_d_body.y(); body_rel.twist.twist.linear.z = v_d_body.z();
            const double ang_scale = (body_relative_angular_unit_ == "rad_s") ? 1.0 : DEG2RAD;
            body_rel.twist.twist.angular.x = drone_odom_copy.twist.twist.angular.x * ang_scale;
            body_rel.twist.twist.angular.y = drone_odom_copy.twist.twist.angular.y * ang_scale;
            body_rel.twist.twist.angular.z = drone_odom_copy.twist.twist.angular.z * ang_scale;
            body_rel_odom_pub_.publish(body_rel);

            if (debug_body_relative_trace_) {
                ROS_INFO_THROTTLE(1.0, "[BodyRelTrace] pT=(%.3f,%.3f,%.3f) vdb=(%.3f,%.3f,%.3f)",
                    p_t_body.x(), p_t_body.y(), p_t_body.z(),
                    v_d_body.x(), v_d_body.y(), v_d_body.z());
            }
        }

        if (drone_odom_mode_ == "target_frame" && have_odom && have_pose) {
            const Eigen::Matrix3d R_NW = last_target_quat_.conjugate().toRotationMatrix();
            const Eigen::Vector3d Om_N = R_NW * Eigen::Vector3d(0, 0, target_yaw_rate_);
            const Eigen::Vector3d p_drone(drone_pose_copy.pose.position.x, drone_pose_copy.pose.position.y, drone_pose_copy.pose.position.z);
            const Eigen::Vector3d v_drone(drone_odom_copy.twist.twist.linear.x, drone_odom_copy.twist.twist.linear.y, drone_odom_copy.twist.twist.linear.z);
            const Eigen::Vector3d p_BN = R_NW * (p_drone - p);
            const Eigen::Vector3d v_BN = R_NW * (v_drone - v) - Om_N.cross(p_BN);
            Eigen::Quaterniond q_drn(drone_pose_copy.pose.orientation.w, drone_pose_copy.pose.orientation.x,
                                     drone_pose_copy.pose.orientation.y, drone_pose_copy.pose.orientation.z);
            q_drn.normalize();
            const Eigen::Quaterniond q_NB = last_target_quat_.conjugate() * q_drn;

            nav_msgs::Odometry tf_odom;
            tf_odom.header.stamp = stamp; tf_odom.header.frame_id = "target_frame"; tf_odom.child_frame_id = "drone_in_target_frame";
            tf_odom.pose.pose.position.x = p_BN.x(); tf_odom.pose.pose.position.y = p_BN.y(); tf_odom.pose.pose.position.z = p_BN.z();
            tf_odom.pose.pose.orientation.w = q_NB.w(); tf_odom.pose.pose.orientation.x = q_NB.x();
            tf_odom.pose.pose.orientation.y = q_NB.y(); tf_odom.pose.pose.orientation.z = q_NB.z();
            tf_odom.twist.twist.linear.x = v_BN.x(); tf_odom.twist.twist.linear.y = v_BN.y(); tf_odom.twist.twist.linear.z = v_BN.z();
            const double ang_scale = (body_relative_angular_unit_ == "rad_s") ? 1.0 : DEG2RAD;
            tf_odom.twist.twist.angular.x = drone_odom_copy.twist.twist.angular.x * ang_scale;
            tf_odom.twist.twist.angular.y = drone_odom_copy.twist.twist.angular.y * ang_scale;
            tf_odom.twist.twist.angular.z = drone_odom_copy.twist.twist.angular.z * ang_scale;
            target_frame_odom_pub_.publish(tf_odom);
        }
    }

    void publishGroundTruth(const ros::Time& stamp,
                            const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                            const Eigen::Vector3d& a, const Eigen::Quaterniond& q_tgt,
                            ros::Publisher& odom_pub, ros::Publisher& accel_pub) {
        nav_msgs::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id  = "target";
        odom.pose.pose.position.x = p.x(); odom.pose.pose.position.y = p.y(); odom.pose.pose.position.z = p.z();
        odom.pose.pose.orientation.w = q_tgt.w(); odom.pose.pose.orientation.x = q_tgt.x();
        odom.pose.pose.orientation.y = q_tgt.y(); odom.pose.pose.orientation.z = q_tgt.z();
        odom.twist.twist.linear.x = v.x(); odom.twist.twist.linear.y = v.y(); odom.twist.twist.linear.z = v.z();
        odom.twist.twist.angular.z = target_yaw_rate_;
        odom_pub.publish(odom);

        geometry_msgs::AccelStamped accel;
        accel.header.stamp    = stamp;
        accel.header.frame_id = frame_id_;
        accel.accel.linear.x = a.x(); accel.accel.linear.y = a.y(); accel.accel.linear.z = a.z();
        accel_pub.publish(accel);
    }

    /// Stage 2 — synthesize a world-frame target from a body-frame relative
    /// estimate. Exact inverse of stage 1 in x/y; z is pinned to pad_z.
    void relativeEstimateCallback(const nav_msgs::Odometry::ConstPtr& est) {
        ros::Time stamp = est->header.stamp;
        if (stamp.isZero()) stamp = ros::Time::now();

        nav_msgs::Odometry drone_odom_copy;
        {
            std::lock_guard<std::mutex> lk(drone_mutex_);
            if (!has_drone_odom_) {
                ROS_WARN_THROTTLE(2.0, "[TargetPublisher] relative_estimate: waiting for drone odom on %s",
                                  drone_odom_topic_.c_str());
                return;
            }
            drone_odom_copy = last_drone_odom_;
        }
        const double age = odomAgeSec(drone_odom_copy, stamp);
        if (age > kDroneOdomMaxAgeSec) {
            ROS_WARN_THROTTLE(2.0, "[TargetPublisher] relative_estimate: drone odom stale (%.2f s), skipping",
                              age);
            return;
        }

        const Eigen::Matrix3d R = droneRotation(drone_odom_copy);
        const Eigen::Vector3d p_d(drone_odom_copy.pose.pose.position.x,
                                  drone_odom_copy.pose.pose.position.y,
                                  drone_odom_copy.pose.pose.position.z);
        const Eigen::Vector3d v_d = droneWorldVelocity(drone_odom_copy, R);
        const Eigen::Vector3d p_rel_b(est->pose.pose.position.x,
                                      est->pose.pose.position.y,
                                      est->pose.pose.position.z);
        const Eigen::Vector3d v_rel_b(est->twist.twist.linear.x,
                                      est->twist.twist.linear.y,
                                      est->twist.twist.linear.z);
        // Rotate all three components before overriding z, otherwise a tilted
        // drone corrupts the world x/y.
        Eigen::Vector3d p_t = p_d + R * p_rel_b;
        Eigen::Vector3d v_t = v_d + R * v_rel_b;
        p_t.z() = pad_z_;
        v_t.z() = 0.0;

        nav_msgs::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id  = "target";
        odom.pose.pose.position.x = p_t.x(); odom.pose.pose.position.y = p_t.y(); odom.pose.pose.position.z = p_t.z();
        odom.pose.pose.orientation.w = 1.0;   // target yaw not part of this contract
        odom.twist.twist.linear.x = v_t.x(); odom.twist.twist.linear.y = v_t.y(); odom.twist.twist.linear.z = v_t.z();
        odom_pub_.publish(odom);

        // The tracker's freshness gate needs odom AND accel; the relative
        // estimate carries no acceleration, so publish zeros at the same stamp.
        geometry_msgs::AccelStamped accel;
        accel.header.stamp    = stamp;
        accel.header.frame_id = frame_id_;
        accel_pub_.publish(accel);
    }

    static double odomAgeSec(const nav_msgs::Odometry& odom, const ros::Time& now) {
        if (odom.header.stamp.isZero()) return 0.0;
        return (now - odom.header.stamp).toSec();
    }

    static Eigen::Matrix3d droneRotation(const nav_msgs::Odometry& odom) {
        Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                             odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        if (q.norm() < 1e-9) return Eigen::Matrix3d::Identity();
        q.normalize();
        return q.toRotationMatrix();
    }

    /// Fixed body-forward camera model used by the MAVROS perception OCP.
    /// A missing detection is represented by publishing nothing: the downstream
    /// target tracker's existing timestamp gate then exposes the observation age
    /// without adding a planner-core-specific message contract.
    bool cameraDetects(const Eigen::Vector3d& p_rel_b) const {
        const double range = p_rel_b.norm();
        if (!std::isfinite(range) || range > sim_camera_max_range_ || p_rel_b.x() <= 1e-6)
            return false;
        const double lateral =
            p_rel_b.y()/std::tan(sim_camera_hfov_deg_*DEG2RAD);
        const double vertical =
            p_rel_b.z()/std::tan(sim_camera_vfov_deg_*DEG2RAD);
        return p_rel_b.x() - std::sqrt(lateral*lateral + vertical*vertical) > 0.0;
    }

    void reportDetectionTransition(bool detected, double range) {
        if (!detection_state_initialized_ || detected != last_detection_state_) {
            ROS_INFO("[TargetPublisher] simulated target %s (range %.2f m)",
                     detected ? "ACQUIRED" : "LOST", range);
            detection_state_initialized_ = true;
            last_detection_state_ = detected;
        }
    }

    /// MAVROS /mavros/local_position/odom has child_frame_id=base_link, so its
    /// twist is body-frame per the Odometry spec. Both stages use this helper,
    /// so the round trip is exact regardless of the setting.
    Eigen::Vector3d droneWorldVelocity(const nav_msgs::Odometry& odom,
                                       const Eigen::Matrix3d& R) const {
        const Eigen::Vector3d twist(odom.twist.twist.linear.x,
                                    odom.twist.twist.linear.y,
                                    odom.twist.twist.linear.z);
        return (drone_odom_twist_frame_ == "world") ? twist : (R * twist);
    }

    static constexpr double kDroneOdomMaxAgeSec = 0.5;

    ros::NodeHandle& nh_;

    std::string target_trajectory_ = "circle";
    double target_yaw_rate_ = 0.3;
    double publish_hz_ = 100.0;
    std::string frame_id_ = "world";
    std::string drone_odom_topic_;
    std::string drone_pose_topic_;
    std::string drone_odom_mode_ = "world";
    bool debug_body_relative_trace_ = false;
    std::string body_relative_angular_unit_ = "deg_s";
    std::string target_source_ = "ground_truth";
    bool        synthesize_from_estimate_ = false;
    bool        sim_relative_estimate_ = false;
    bool        sim_visibility_gate_ = true;
    double      sim_camera_hfov_deg_ = 30.0;
    double      sim_camera_vfov_deg_ = 22.0;
    double      sim_camera_max_range_ = 12.0;
    bool        detection_state_initialized_ = false;
    bool        last_detection_state_ = false;
    double      pad_z_ = 0.0;
    std::string relative_estimate_topic_;
    bool        publish_gt_debug_ = true;
    std::string drone_odom_twist_frame_ = "body";
    double t0_ = 0.0;
    Eigen::Quaterniond last_target_quat_ = Eigen::Quaterniond::Identity();

    target_config::SyntheticTarget motion_;

    // Target-estimate uncertainty (see constructor).
    mutable std::mt19937 noise_rng_;
    double noise_sigma_pos_ = 0.0, noise_sigma_vel_ = 0.0, noise_sigma_acc_ = 0.0;
    double noise_bias_frac_ = 0.5;
    bool   noise_active_ = false;
    Eigen::Vector3d noise_bias_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d noise_bias_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d noise_bias_acc_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d corrupt(const Eigen::Vector3d& x, const Eigen::Vector3d& bias,
                            double sigma) const {
        if (sigma <= 0.0) return x + bias;
        std::normal_distribution<double> nd(0.0, sigma);
        return x + bias + Eigen::Vector3d(nd(noise_rng_), nd(noise_rng_), 0.0);
    }

    ros::Publisher  odom_pub_, accel_pub_;
    ros::Publisher  abs_rel_odom_pub_, body_rel_odom_pub_, target_frame_odom_pub_;
    ros::Publisher  gt_odom_pub_, gt_accel_pub_, rel_estimate_pub_;
    ros::Subscriber drone_odom_sub_, drone_pose_sub_, rel_estimate_sub_;
    ros::Timer      timer_;

    std::mutex drone_mutex_;
    nav_msgs::Odometry last_drone_odom_;
    geometry_msgs::PoseStamped last_drone_pose_;
    bool has_drone_odom_ = false;
    bool has_drone_pose_ = false;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle nh("~");
    TargetPublisherNode node(nh);
    ros::spin();
    return 0;
}
