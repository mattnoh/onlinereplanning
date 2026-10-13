#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

// Ground-truth target movement, independent of ROS and of OCP prediction.
// Preserve the saved MC1 equations, seeds, and random draw order.
namespace target_models {
struct TargetConfig {
    double R = 1.0, omega = 0.4, omega_f8 = 0.3;
    double amp_x = 1.0, amp_y = 1.0;
    double speed = 0.4, speed_sag = 0.4, span = 2.0;
    double cruise = 3.0, ramp = 1.0, dwell = 2.0;
    double accel_mag = 0.15, hold = 2.0;
    double sigma = 0.20;
    double phase = 0.0, heading = 0.0, t_off = 0.0;
    int    k0 = 0;
};
namespace detail {

struct CircularTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double R = 1.0;
    double omega = 0.4;
    double phi0 = 0.0;
    Eigen::Vector3d pos(double t) const {
        double ph = omega * t + phi0;
        return center + Eigen::Vector3d(R * std::cos(ph), R * std::sin(ph), 0.0);
    }
    Eigen::Vector3d vel(double t) const {
        double ph = omega * t + phi0;
        return Eigen::Vector3d(-R * omega * std::sin(ph), R * omega * std::cos(ph), 0.0);
    }
    Eigen::Vector3d accel(double t) const {
        double ph = omega * t + phi0;
        return Eigen::Vector3d(-R * omega * omega * std::cos(ph),
                               -R * omega * omega * std::sin(ph), 0.0);
    }
    Eigen::VectorXd state(double t) const {
        Eigen::VectorXd s(6);
        s.head(3) = pos(t);
        s.tail(3) = vel(t);
        return s;
    }
};

struct Figure8Target {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double amp_x = 1.0;
    double amp_y = 1.0;
    double omega = 0.1;
    double phi0 = 0.0;
    Eigen::Vector3d pos(double t) const {
        const double ph = omega * t + phi0;
        const double s = std::sin(ph);
        const double c = std::cos(ph);
        return center + Eigen::Vector3d(amp_x * c, amp_y * s * c, 0.0);
    }
    Eigen::Vector3d vel(double t) const {
        const double ph = omega * t + phi0;
        return Eigen::Vector3d(
            -amp_x * omega * std::sin(ph),
            amp_y * omega * std::cos(2.0 * ph),
            0.0);
    }
    Eigen::Vector3d accel(double t) const {
        const double ph = omega * t + phi0;
        return Eigen::Vector3d(
            -amp_x * omega * omega * std::cos(ph),
            -2.0 * amp_y * omega * omega * std::sin(2.0 * ph),
            0.0);
    }
    Eigen::VectorXd state(double t) const {
        Eigen::VectorXd s(6);
        s.head(3) = pos(t);
        s.tail(3) = vel(t);
        return s;
    }
};

struct StraightTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double speed = 0.4;
    double heading = 0.0;
    double span = 2.0;
    double t_offset = 0.0;
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

struct StopAndGoTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double speed = 0.4;
    double heading = 0.0;
    double t_cruise = 3.0;
    double t_ramp = 1.0;
    double t_dwell = 2.0;
    double span = 2.0;
    double t_offset = 0.0;
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
    double periodDistance() const { return speed * t_cruise + speed * t_ramp; }
    void profile(double t, double& d, double& v, double& a) const {
        t += t_offset;
        const double T = period();
        const double n = std::floor(t / T);
        double u = t - n * T;
        d = n * periodDistance();
        const double t1 = t_cruise;
        const double t2 = t1 + t_ramp;
        const double t3 = t2 + t_dwell;
        if (u < t1) {
            d += speed * u;
            v = speed;
            a = 0.0;
        } else if (u < t2) {
            const double s = u - t1;
            const double acc = -speed / t_ramp;
            d += speed * t_cruise + speed * s + 0.5 * acc * s * s;
            v = speed + acc * s;
            a = acc;
        } else if (u < t3) {
            d += speed * t_cruise + 0.5 * speed * t_ramp;
            v = 0.0;
            a = 0.0;
        } else {
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

struct SegmentedAccelTarget {
    struct Segment {
        double t0 = 0.0;
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
    };
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double hold = 2.0;
    double v_max = 0.5;
    double k_center = 0.05;
    double horizon = 900.0;
    std::vector<Segment> segs;
    template <typename DrawFn>
    void build(const Eigen::Vector3d& v0, DrawFn draw) {
        segs.clear();
        const int n = std::max(1, static_cast<int>(std::ceil(horizon / hold)));
        segs.reserve(n + 1);
        Eigen::Vector3d p = center;
        Eigen::Vector3d v = v0;
        for (int k = 0; k < n; ++k) {
            Eigen::Vector3d a = draw(k);
            a.z() = 0.0;
            a -= k_center * (p - center);
            a.z() = 0.0;
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
        return Eigen::Vector3d(accel_mag * (cr * d[0] - sr * d[1]),
                               accel_mag * (sr * d[0] + cr * d[1]), 0.0);
    });
    return t;
}
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
    t.build(Eigen::Vector3d(speed0 * std::cos(dir_rot),
                            speed0 * std::sin(dir_rot), 0.0), [&](int) {
        return Eigen::Vector3d(nd(rng), nd(rng), 0.0);
    });
    return t;
}

struct Range { double lo, hi; };
inline Range R_RADIUS{0.50, 1.80};
inline Range R_OMEGA {0.20, 0.55};
inline Range R_OMEGA_F8{0.20, 0.45};
inline Range R_AMP   {0.55, 1.35};
inline Range R_SPEED {0.20, 0.60};
inline Range R_SPEED_SAG{0.20, 0.50};
inline Range R_SPAN  {1.20, 3.50};
inline Range R_CRUISE{4.70, 6.00};
inline Range R_RAMP  {0.60, 1.60};
inline Range R_DWELL {1.60, 5.00};
inline Range R_ACCEL {0.06, 0.42};
inline Range R_HOLD  {1.50, 4.00};
inline Range R_SIGMA {0.05, 0.16};
inline constexpr double kCircAccelMax = 0.34;
inline constexpr double kCircSpeedMax = 0.85;
inline constexpr double kF8SpeedMax = 0.40;
inline constexpr double kF8AccelMax = 0.17;
inline constexpr double kSagBrakeMax = 0.45;
inline TargetConfig drawConfig(unsigned seed) {
    std::mt19937 g(seed);
    auto U = [&g](Range r) {
        return std::uniform_real_distribution<double>(r.lo, r.hi)(g);
    };
    TargetConfig sh;
    sh.phase   = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(g);
    sh.heading = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(g);
    sh.k0      = std::uniform_int_distribution<int>(0, 7)(g);
    sh.R       = U(R_RADIUS);
    sh.omega   = U(R_OMEGA);
    sh.omega_f8 = U(R_OMEGA_F8);
    sh.amp_x   = U(R_AMP);
    sh.amp_y   = U(R_AMP);
    sh.speed   = U(R_SPEED);
    sh.speed_sag = U(R_SPEED_SAG);
    sh.span    = U(R_SPAN);
    sh.cruise  = U(R_CRUISE);
    sh.ramp    = U(R_RAMP);
    sh.dwell   = U(R_DWELL);
    sh.accel_mag = U(R_ACCEL);
    sh.hold    = U(R_HOLD);
    sh.sigma   = U(R_SIGMA);
    if (sh.R > 1e-9) {
        sh.omega = std::min(sh.omega, std::sqrt(kCircAccelMax / sh.R));
        sh.omega = std::min(sh.omega, kCircSpeedMax / sh.R);
        sh.omega = std::max(sh.omega, R_OMEGA.lo);
    }
    const double amp = std::max(sh.amp_x, sh.amp_y);
    if (amp > 1e-9) {
        sh.omega_f8 = std::min(sh.omega_f8, kF8SpeedMax / amp);
        sh.omega_f8 = std::min(sh.omega_f8, std::sqrt(kF8AccelMax / amp));
        sh.omega_f8 = std::max(sh.omega_f8, R_OMEGA_F8.lo);
    }
    if (sh.speed_sag > 1e-9) {
        sh.ramp = std::max(sh.ramp, sh.speed_sag / kSagBrakeMax);
    }
    const double T = (sh.speed > 1e-9 && sh.span > 0.0)
                         ? 4.0 * sh.span / sh.speed : 0.0;
    sh.t_off = std::uniform_real_distribution<double>(0.0, std::max(T, 1e-9))(g);
    return sh;
}

}
class TargetGenerator {
    std::function<Eigen::Vector3d(double)> position_, velocity_, acceleration_;
    template<class Model> static TargetGenerator fromModel(const Model& model) {
        TargetGenerator out;
        out.position_=[model](double t){return model.pos(t);};
        out.velocity_=[model](double t){return model.vel(t);};
        out.acceleration_=[model](double t){return model.accel(t);};
        return out;
    }
public:
    Eigen::Vector3d pos(double t) const {return position_(t);}
    Eigen::Vector3d vel(double t) const {return velocity_(t);}
    Eigen::Vector3d accel(double t) const {return acceleration_(t);}
    static TargetConfig drawConfig(unsigned seed){return detail::drawConfig(seed);}
static TargetGenerator create(const std::string& motion,
                                           const TargetConfig& d, unsigned seed) {
    const Eigen::Vector3d ctr(0.0, 0.0, 0.2);
    if (motion == "circle") {
        detail::CircularTarget m;
        m.center = ctr; m.R = d.R; m.omega = d.omega; m.phi0 = d.phase;
        return fromModel(m);
    }
    if (motion == "figure8") {
        detail::Figure8Target m;
        m.center = ctr; m.amp_x = d.amp_x; m.amp_y = d.amp_y;
        m.omega = d.omega_f8; m.phi0 = d.phase;
        return fromModel(m);
    }
    if (motion == "straight") {
        detail::StraightTarget m;
        m.center = ctr; m.speed = d.speed; m.span = d.span;
        m.heading = d.heading; m.t_offset = d.t_off;
        return fromModel(m);
    }
    if (motion == "stop_and_go") {
        detail::StopAndGoTarget m;
        m.center = ctr; m.speed = d.speed_sag; m.span = d.span;
        m.heading = d.heading; m.t_offset = d.t_off;
        m.t_cruise = d.cruise; m.t_ramp = d.ramp; m.t_dwell = d.dwell;
        return fromModel(m);
    }
    if (motion == "piecewise_accel") {
        return fromModel(
            detail::makePiecewiseAccelTarget(ctr, d.accel_mag, d.hold,
                                                    0.5, d.speed, d.heading, d.k0));
    }
    if (motion != "random_accel") throw std::runtime_error("Unknown target motion: " + motion);
    return fromModel(
        detail::makeRandomAccelTarget(ctr, d.sigma, d.hold, 0.5,
                                             d.speed, seed, d.heading));
}

    static TargetGenerator defaults(const std::string& motion) {
        TargetConfig config;
        config.omega_f8=0.1; // Retain Crazyflie's original synthetic figure-eight default.
        return create(motion,config,1);
    }
};
}
