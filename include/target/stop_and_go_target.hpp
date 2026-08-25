#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace target_models {

// Stop-and-go motion along a fixed heading: cruise -> decelerate -> dwell ->
// accelerate, repeating. Position and velocity are continuous; acceleration is
// piecewise constant and JUMPS at every phase boundary, which is precisely the
// stress case for the planner's constant-acceleration prediction (it will hold
// whatever `a` it sampled across the entire horizon, straight through a stop).
//
// Phase layout, one period:
//   [0, Tc)                cruise at `speed`, a = 0
//   [Tc, Tc+Tr)            decel to 0,        a = -speed/Tr
//   [Tc+Tr, Tc+Tr+Td)      dwell at rest,     a = 0
//   [Tc+Tr+Td, Tc+2Tr+Td)  accel back up,     a = +speed/Tr
struct StopAndGoTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double speed = 0.4;         // m/s cruise
    double heading = 0.0;       // rad, CCW from +x
    double t_cruise = 3.0;      // s at speed
    double t_ramp = 1.0;        // s to decelerate (and to accelerate)
    double t_dwell = 2.0;       // s stopped
    double span = 2.0;          // m shuttle half-span (see StraightTarget::span)
    // Per-trial phase into the cruise/ramp/dwell cycle -- see
    // StraightTarget::t_offset for why this exists.
    double t_offset = 0.0;      // s, added to t

    void fold(double d, double& x, double& sign) const {
        if (span <= 0.0) { x = d; sign = 1.0; return; }
        const double S = span;
        double u = std::fmod(d, 4.0 * S);
        if (u < 0.0) u += 4.0 * S;
        if (u < S)            { x = u;           sign = 1.0; }
        else if (u < 3.0 * S) { x = 2.0 * S - u; sign = -1.0; }
        else                  { x = u - 4.0 * S; sign = 1.0; }
    }

    double period() const { return t_cruise + 2.0 * t_ramp + t_dwell; }

    // Distance covered in one full period.
    double periodDistance() const { return speed * t_cruise + speed * t_ramp; }

    // Scalar (arc-length, speed, tangential accel) at time t.
    void profile(double t, double& d, double& v, double& a) const {
        t += t_offset;
        const double T = period();
        const double n = std::floor(t / T);
        double u = t - n * T;                 // time within this period
        d = n * periodDistance();

        const double t1 = t_cruise;
        const double t2 = t1 + t_ramp;
        const double t3 = t2 + t_dwell;

        if (u < t1) {                          // cruise
            d += speed * u;
            v = speed;
            a = 0.0;
        } else if (u < t2) {                   // decelerate
            const double s = u - t1;
            const double acc = -speed / t_ramp;
            d += speed * t_cruise + speed * s + 0.5 * acc * s * s;
            v = speed + acc * s;
            a = acc;
        } else if (u < t3) {                   // dwell
            d += speed * t_cruise + 0.5 * speed * t_ramp;
            v = 0.0;
            a = 0.0;
        } else {                               // accelerate
            const double s = u - t3;
            const double acc = speed / t_ramp;
            d += speed * t_cruise + 0.5 * speed * t_ramp + 0.5 * acc * s * s;
            v = acc * s;
            a = acc;
        }
    }

    Eigen::Vector3d pos(double t) const {
        double d, v, a, x, sg;
        profile(t, d, v, a);
        fold(d, x, sg);
        return center + Eigen::Vector3d(x * std::cos(heading), x * std::sin(heading), 0.0);
    }

    Eigen::Vector3d vel(double t) const {
        double d, v, a, x, sg;
        profile(t, d, v, a);
        fold(d, x, sg);
        return Eigen::Vector3d(sg * v * std::cos(heading), sg * v * std::sin(heading), 0.0);
    }

    Eigen::Vector3d accel(double t) const {
        double d, v, a, x, sg;
        profile(t, d, v, a);
        fold(d, x, sg);
        return Eigen::Vector3d(sg * a * std::cos(heading), sg * a * std::sin(heading), 0.0);
    }

    Eigen::VectorXd state(double t) const {
        Eigen::VectorXd s(6);
        s.head(3) = pos(t);
        s.tail(3) = vel(t);
        return s;
    }
};

}  // namespace target_models
