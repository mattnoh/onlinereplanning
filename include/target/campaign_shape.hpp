#pragma once

// Per-trial randomised target-motion shapes for the SITL campaigns.
//
// SINGLE SOURCE OF TRUTH. Both the ROS target_publisher and the offline
// feasibility screen (tools_motion_seed_screen) include this header, so the
// screen provably describes the campaign. Duplicating the draw logic in the
// two places is how they silently drift apart, and the screen is worthless
// the moment it stops matching what actually flies.
//
// Randomising only phase/heading is NOT enough: a circle at constant speed is
// rotationally symmetric, so with the drone staged at a fixed bearing from the
// target's heading every trial is the same problem rotated. The physical
// parameters have to vary.

#include "target/circular_target.hpp"
#include "target/figure8_target.hpp"
#include "target/straight_target.hpp"
#include "target/stop_and_go_target.hpp"
#include "target/segmented_accel_target.hpp"
#include "target/target_model_config.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>
#include <string>

namespace target_campaign {

/// Physical shape parameters of a target motion. Randomising only phase and
/// heading was useless: a circle is rotationally symmetric at constant speed,
/// so with the drone staged at a fixed bearing from the target's heading every
/// trial is the SAME problem rotated. The trajectory has to differ physically
/// -- radius, rate, speed, segment timing -- for the trials to be a sample.
struct Shape {
    double R = 1.0, omega = 0.4, omega_f8 = 0.3;   // circle / figure8
    double amp_x = 1.0, amp_y = 1.0;          // figure8 (omega shared)
    double speed = 0.4, speed_sag = 0.4, span = 2.0;  // straight / stop_and_go
    double cruise = 3.0, ramp = 1.0, dwell = 2.0;
    double accel_mag = 0.15, hold = 2.0;      // piecewise
    double sigma = 0.20;                      // random_accel
    double phase = 0.0, heading = 0.0, t_off = 0.0;
    int    k0 = 0;
};

/// Sampling ranges, chosen from the parameter sweeps in `psweep` mode.
/// Edit these, re-run `psweep`, and re-screen -- they are the study's design.
struct Range { double lo, hi; };
// Envelopes measured by `psweep` (13 points per parameter, alpha=0):
//   circle  radius 0.4..2.2   all ok
//   circle  omega  ok to 0.617, NO-LAND from 0.675 -- the driver is CENTRIPETAL
//                  ACCELERATION R*omega^2, not omega alone: R=2.2 at omega=0.4
//                  (a=0.35) lands, R=1.0 at omega=0.675 (a=0.46) does not.
//                  Hence the a_max cap below rather than a tighter omega range.
//   figure8 omega  ok to 0.471, consistently NO-LAND from 0.517
//   figure8 amp    ok to 1.375, consistently NO-LAND from 1.5
//   straight speed 0.15..0.90 ok;  span 1.0..4.0 all ok
//   piecewise accel 0.05..0.45 all ok; hold 0.8..4.0 ok
//   stop_and_go speed ok to ~0.525, consistently NO-LAND from 0.588
//   stop_and_go dwell ok from ~1.6;  cruise clean only from ~4.7
//   random_accel sigma ok to ~0.10, increasingly NO-LAND above
inline Range R_RADIUS{0.50, 1.80};
inline Range R_OMEGA {0.20, 0.55};     // circle, further constrained by the a_max cap
inline Range R_OMEGA_F8{0.20, 0.45};   // figure8 loses it earlier than circle
inline Range R_AMP   {0.55, 1.35};
inline Range R_SPEED {0.20, 0.60};     // straight
inline Range R_SPEED_SAG{0.20, 0.50};  // stop_and_go loses it earlier
inline Range R_SPAN  {1.20, 3.50};
inline Range R_CRUISE{4.70, 6.00};
inline Range R_RAMP  {0.60, 1.60};
inline Range R_DWELL {1.60, 5.00};
inline Range R_ACCEL {0.06, 0.42};
inline Range R_HOLD  {1.50, 4.00};
inline Range R_SIGMA {0.05, 0.16};

// Circle/figure8 difficulty is governed by the centripetal terms, so cap them
// directly instead of shrinking the radius and rate ranges until the worst
// corner happens to be safe -- that would throw away most of the variation.
inline constexpr double kCircAccelMax = 0.34;   // m/s^2, from the omega sweep
inline constexpr double kCircSpeedMax = 0.85;   // m/s,   from the radius sweep
// Figure-eight needs the same treatment. psweep varies ONE parameter with the
// rest nominal, so it maps a cross through the feasible set, not its shape:
// omega alone is fine to 0.471 and amplitude alone to 1.375, but both high
// together fails. Screening the box [0.20,0.45] x [0.55,1.35] gave 12/20.
// Peak speed on a Gerono lemniscate is ~amp*omega; the sweeps put the cliff at
// ~0.45-0.50 m/s from both directions, so cap the product.
inline constexpr double kF8SpeedMax = 0.40;
inline constexpr double kF8AccelMax = 0.17;
// Stop-and-go brakes from `speed` to rest in `ramp` seconds, so its peak
// deceleration is speed/ramp -- a quantity psweep never varied (it swept speed
// and the durations independently, never their ratio). The box allowed
// 0.50/0.60 = 0.83 m/s^2, double the 0.40 of the old fixed config, and the
// motion screened at 31/50 with rxy running to 8 m. Cap the ratio.
inline constexpr double kSagBrakeMax = 0.45;

/// Mirrors the draw order in target_publisher.cpp. Order matters -- change it
/// in one place and the screen stops describing the campaign.
inline Shape drawShape(unsigned seed) {
    std::mt19937 g(seed);
    auto U = [&g](Range r) {
        return std::uniform_real_distribution<double>(r.lo, r.hi)(g);
    };
    Shape sh;
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

    // Clamp the circle to the measured feasible corner. Drawing R and omega
    // independently puts ~a fifth of the draws past the centripetal limit,
    // and clamping omega (rather than redrawing) keeps the draw ORDER fixed,
    // which is what makes a (motion, level, trial) triple reproducible.
    if (sh.R > 1e-9) {
        sh.omega = std::min(sh.omega, std::sqrt(kCircAccelMax / sh.R));
        sh.omega = std::min(sh.omega, kCircSpeedMax / sh.R);
        sh.omega = std::max(sh.omega, R_OMEGA.lo);
    }
    // Same clamp-not-redraw rule as the circle: keeps the RNG call count per
    // trial fixed, which is what makes a (motion, level, trial) reproducible.
    const double amp = std::max(sh.amp_x, sh.amp_y);
    if (amp > 1e-9) {
        sh.omega_f8 = std::min(sh.omega_f8, kF8SpeedMax / amp);
        sh.omega_f8 = std::min(sh.omega_f8, std::sqrt(kF8AccelMax / amp));
        sh.omega_f8 = std::max(sh.omega_f8, R_OMEGA_F8.lo);
    }
    // Lengthen the ramp rather than lowering the speed: speed is the variable
    // the study wants spread, ramp duration is incidental.
    if (sh.speed_sag > 1e-9) {
        sh.ramp = std::max(sh.ramp, sh.speed_sag / kSagBrakeMax);
    }
    const double T = (sh.speed > 1e-9 && sh.span > 0.0)
                         ? 4.0 * sh.span / sh.speed : 0.0;
    sh.t_off = std::uniform_real_distribution<double>(0.0, std::max(T, 1e-9))(g);
    return sh;
}

inline target_config::SyntheticTarget buildTarget(const std::string& motion,
                                           const Shape& d, unsigned seed) {
    const Eigen::Vector3d ctr(0.0, 0.0, 0.2);
    if (motion == "circle") {
        target_models::CircularTarget m;
        m.center = ctr; m.R = d.R; m.omega = d.omega; m.phi0 = d.phase;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "figure8") {
        target_models::Figure8Target m;
        m.center = ctr; m.amp_x = d.amp_x; m.amp_y = d.amp_y;
        m.omega = d.omega_f8; m.phi0 = d.phase;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "straight") {
        target_models::StraightTarget m;
        m.center = ctr; m.speed = d.speed; m.span = d.span;
        m.heading = d.heading; m.t_offset = d.t_off;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "stop_and_go") {
        target_models::StopAndGoTarget m;
        m.center = ctr; m.speed = d.speed_sag; m.span = d.span;
        m.heading = d.heading; m.t_offset = d.t_off;
        m.t_cruise = d.cruise; m.t_ramp = d.ramp; m.t_dwell = d.dwell;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "piecewise_accel") {
        return target_config::SyntheticTarget::from(
            target_models::makePiecewiseAccelTarget(ctr, d.accel_mag, d.hold,
                                                    0.5, d.speed, d.heading, d.k0));
    }
    return target_config::SyntheticTarget::from(
        target_models::makeRandomAccelTarget(ctr, d.sigma, d.hold, 0.5,
                                             d.speed, seed, d.heading));
}

}  // namespace target_campaign
