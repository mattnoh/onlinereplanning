#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace target_models {

// Constant-velocity motion along a fixed heading.
//
// This is the CONTROL scenario for the target-motion campaign: the planner's
// ConstantAccelPredictor is EXACT here (a = 0 held over the horizon is the
// truth), so any landing failure on this motion is not a prediction-model
// problem. Every other motion violates the constant-acceleration assumption to
// some degree.
struct StraightTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};   // position at t = 0
    double speed = 0.4;                      // m/s, matches the flown circle
    double heading = 0.0;                    // rad, CCW from +x
    // Travel is folded into a shuttle over [-span, +span] so the target stays
    // in the landing area. A quadrotor cannot intercept a target that leaves at
    // constant velocity forever: an unbounded straight run put the target 20 m
    // away by engagement and every solve was rejected. Acceleration is still
    // exactly zero everywhere except the measure-zero reversals, so this
    // remains the case where the constant-acceleration predictor is exact.
    double span = 2.0;                       // m, <=0 disables the fold
    // Per-trial phase. The motion is otherwise a pure function of t, so
    // without this every trial of a campaign cell flies the IDENTICAL target
    // path and the Monte-Carlo samples only differ by bring-up jitter.
    double t_offset = 0.0;                   // s, added to t

    // Fold unbounded arc-length d into a triangle wave on [-span, span].
    void fold(double d, double& x, double& sign) const {
        if (span <= 0.0) { x = d; sign = 1.0; return; }
        const double S = span;
        double u = std::fmod(d, 4.0 * S);
        if (u < 0.0) u += 4.0 * S;
        if (u < S)            { x = u;            sign = 1.0; }
        else if (u < 3.0 * S) { x = 2.0 * S - u;  sign = -1.0; }
        else                  { x = u - 4.0 * S;  sign = 1.0; }
    }

    Eigen::Vector3d pos(double t) const {
        t += t_offset;
        double x, sg; fold(speed * t, x, sg);
        return center + Eigen::Vector3d(x * std::cos(heading), x * std::sin(heading), 0.0);
    }

    Eigen::Vector3d vel(double t) const {
        t += t_offset;
        double x, sg; fold(speed * t, x, sg);
        return Eigen::Vector3d(sg * speed * std::cos(heading),
                               sg * speed * std::sin(heading), 0.0);
    }

    Eigen::Vector3d accel(double) const { return Eigen::Vector3d::Zero(); }

    Eigen::VectorXd state(double t) const {
        Eigen::VectorXd s(6);
        s.head(3) = pos(t);
        s.tail(3) = vel(t);
        return s;
    }
};

}  // namespace target_models
