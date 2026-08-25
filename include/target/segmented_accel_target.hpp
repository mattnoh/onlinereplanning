#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace target_models {

// Motion built from segments of EXACTLY constant acceleration.
//
// Within a segment the planner's ConstantAccelPredictor is exact; at every
// segment boundary the acceleration jumps and the prediction is wrong for the
// remainder of the horizon. Since the planner's horizon (~3.5 s) is longer than
// a typical hold, most solves straddle at least one jump.
//
// The whole segment table is precomputed in the constructor, so pos(t) is a
// pure function of t (required: the ROS timer calls it repeatedly, and runs
// must be reproducible from a seed). accel() is exact, not finite-differenced.
//
// Bounds: each drawn acceleration is corrected so the segment-end speed stays
// within `v_max`, and a weak centering term keeps the target from drifting away
// from the landing area. Both corrections are applied when the segment is
// built, so constant-acceleration exactness inside the segment is preserved.
struct SegmentedAccelTarget {
    struct Segment {
        double t0 = 0.0;
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
    };

    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double hold = 2.0;          // s per constant-acceleration segment
    double v_max = 0.5;         // m/s speed cap
    double k_center = 0.05;     // 1/s^2 weak restoring toward center
    double horizon = 900.0;     // s of precomputed table
    std::vector<Segment> segs;

    // `draw(k)` returns the raw acceleration for segment k, before bounding.
    template <typename DrawFn>
    void build(const Eigen::Vector3d& v0, DrawFn draw) {
        segs.clear();
        const int n = std::max(1, static_cast<int>(std::ceil(horizon / hold)));
        segs.reserve(n + 1);

        Eigen::Vector3d p = center;
        Eigen::Vector3d v = v0;
        for (int k = 0; k < n; ++k) {
            Eigen::Vector3d a = draw(k);
            a.z() = 0.0;                                   // planar target
            a -= k_center * (p - center);                  // keep it nearby
            a.z() = 0.0;

            // Correct `a` so the segment END speed respects the cap. Solving on
            // `a` (rather than clipping v afterwards) keeps the segment exactly
            // constant-acceleration, which is what accel() reports.
            Eigen::Vector3d v_end = v + a * hold;
            const double sp = v_end.norm();
            if (sp > v_max && sp > 1e-9) {
                v_end *= v_max / sp;
                a = (v_end - v) / hold;
            }

            Segment s;
            s.t0 = k * hold;
            s.p = p;
            s.v = v;
            s.a = a;
            segs.push_back(s);

            p = p + v * hold + 0.5 * a * hold * hold;
            v = v + a * hold;
            p.z() = center.z();
            v.z() = 0.0;
        }
        Segment tail;
        tail.t0 = n * hold;
        tail.p = p;
        tail.v = v;
        tail.a = Eigen::Vector3d::Zero();
        segs.push_back(tail);
    }

    const Segment& segmentAt(double t) const {
        if (segs.empty()) {
            static const Segment kZero{};
            return kZero;
        }
        if (t <= 0.0) return segs.front();
        const int k = std::min(static_cast<int>(t / hold),
                               static_cast<int>(segs.size()) - 1);
        return segs[k];
    }

    Eigen::Vector3d pos(double t) const {
        const Segment& s = segmentAt(t);
        const double dt = std::max(0.0, t - s.t0);
        Eigen::Vector3d p = s.p + s.v * dt + 0.5 * s.a * dt * dt;
        p.z() = center.z();
        return p;
    }

    Eigen::Vector3d vel(double t) const {
        const Segment& s = segmentAt(t);
        const double dt = std::max(0.0, t - s.t0);
        Eigen::Vector3d v = s.v + s.a * dt;
        v.z() = 0.0;
        return v;
    }

    Eigen::Vector3d accel(double t) const { return segmentAt(t).a; }

    Eigen::VectorXd state(double t) const {
        Eigen::VectorXd s(6);
        s.head(3) = pos(t);
        s.tail(3) = vel(t);
        return s;
    }
};

// Acceleration cycles deterministically through a fixed pattern of directions.
// Reproducible without a seed; the jumps are regular rather than stochastic.
// `dir_rot` rotates the whole direction cycle and the initial velocity, and
// `k0` phases into it. Both default to the original fixed pattern. Without
// them the piecewise target is a pure function of t and every trial of a
// campaign cell flies the IDENTICAL path -- see StraightTarget::t_offset.
inline SegmentedAccelTarget makePiecewiseAccelTarget(const Eigen::Vector3d& center,
                                                     double accel_mag,
                                                     double hold,
                                                     double v_max,
                                                     double speed0,
                                                     double dir_rot = 0.0,
                                                     int k0 = 0) {
    SegmentedAccelTarget t;
    t.center = center;
    t.hold = hold;
    t.v_max = v_max;
    static const double kDirs[8][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1},
                                       {0.7071, 0.7071}, {-0.7071, 0.7071},
                                       {-0.7071, -0.7071}, {0.7071, -0.7071}};
    const double cr = std::cos(dir_rot), sr = std::sin(dir_rot);
    t.build(Eigen::Vector3d(speed0 * cr, speed0 * sr, 0.0), [&](int k) {
        const auto& d = kDirs[((k + k0) % 8 + 8) % 8];
        // Rotate the unit direction, then scale -- a rotation preserves the
        // magnitude, so |a| is still exactly accel_mag.
        return Eigen::Vector3d(accel_mag * (cr * d[0] - sr * d[1]),
                               accel_mag * (sr * d[0] + cr * d[1]), 0.0);
    });
    return t;
}

// Acceleration resampled from N(0, sigma) every `hold` seconds. Seeded, so a
// trial is byte-reproducible from (motion, uncertainty level, trial index).
inline SegmentedAccelTarget makeRandomAccelTarget(const Eigen::Vector3d& center,
                                                  double sigma,
                                                  double hold,
                                                  double v_max,
                                                  double speed0,
                                                  unsigned seed,
                                                  double dir_rot = 0.0) {
    SegmentedAccelTarget t;
    t.center = center;
    t.hold = hold;
    t.v_max = v_max;
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, sigma);
    // The accelerations are already seeded; dir_rot only varies the initial
    // heading so this motion is randomised the same way as the others.
    t.build(Eigen::Vector3d(speed0 * std::cos(dir_rot),
                            speed0 * std::sin(dir_rot), 0.0), [&](int) {
        return Eigen::Vector3d(nd(rng), nd(rng), 0.0);
    });
    return t;
}

}  // namespace target_models
