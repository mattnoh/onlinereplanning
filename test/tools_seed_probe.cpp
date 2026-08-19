/// @file tools_seed_probe.cpp
/// @brief Offline screen for seeding the RH solve from the MEASURED state.
///
/// The planner seeds each replan from the active plan sampled at the activation
/// instant (`predicted_handoff`), which assumes the tracking error is zero at
/// handoff. It is not: PX4 leaves rho_pos ~0.64-0.77 (SITL) and 0.698 (Crazyflie
/// hardware) of the position error at the ~0.48 s lead, so the planner never
/// sees that the vehicle is off-reference and handoff_pos_err drifts to ~0.38 m.
///
/// Seeding from the measurement is the textbook fix, but a naive full-state
/// measured seed was flown and diverged the solver (constraint_error 3.4e8,
/// 78-84 rejects vs 0 for the predicted seed).
///
/// ESTABLISHED BY THIS PROBE: the divergence is not a warm-start defect, it is
/// that the seed must be FEASIBLE for the OCP's own hard constraints. The STC
/// bounds on attitude/body-rate are tighter than the vehicle's real behaviour
/// (omega <= 0.019 rad/s vs 0.03-0.5 flown; tilt <= 2 deg vs up to 11 flown), so
/// measured attitude/rate can never be seeded. Position and velocity can.
///
/// This probe sweeps a seed perturbation at SEVERAL POINTS ALONG THE DESCENT and
/// reports the largest magnitude that still passes the planner's acceptance
/// gate. The position clamp used by the planner must be <= the minimum of that
/// budget over the whole descent. Run this BEFORE flying any seeding change.

#include "planner_core/quadrotor_mpc.hpp"
#include "planner_core/ocp_registry.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <any>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace {

struct CircularTarget {
    Eigen::Vector3d center{0.0, 0.0, 0.2};
    double R = 1.0;
    double omega = 0.4;

    Eigen::Vector3d pos(double t) const {
        const double ph = omega * t;
        return center + Eigen::Vector3d(R * std::cos(ph), R * std::sin(ph), 0.0);
    }
    Eigen::Vector3d vel(double t) const {
        const double ph = omega * t;
        return {-R * omega * std::sin(ph), R * omega * std::cos(ph), 0.0};
    }
    Eigen::Vector3d accel(double t) const {
        const double ph = omega * t;
        return {-R * omega * omega * std::cos(ph), -R * omega * omega * std::sin(ph), 0.0};
    }
    TargetSnapshot snapshot(double t) const {
        TargetSnapshot s;
        s.valid = true;
        s.position = pos(t);
        s.velocity = vel(t);
        s.acceleration = accel(t);
        return s;
    }
};

enum Block { kNone = 0, kPos, kVel, kTilt, kOmega };

/// Perturb one block of the seed. Position/velocity are pushed RADIALLY OUTWARD
/// in xy — the worst direction, because it moves the seed toward the glideslope
/// cone boundary rather than away from it.
Eigen::VectorXd perturb(const Eigen::VectorXd& x0, Block b, double mag) {
    Eigen::VectorXd x = x0;
    Eigen::Vector2d radial(x0(0), x0(1));
    radial = (radial.norm() > 1e-6) ? radial.normalized() : Eigen::Vector2d(1.0, 0.0);
    switch (b) {
        case kPos:
            x(0) += mag * radial(0);
            x(1) += mag * radial(1);
            break;
        case kVel:
            x(3) += mag * radial(0);
            x(4) += mag * radial(1);
            break;
        case kTilt: {
            const double half = 0.5 * mag * M_PI / 180.0;
            Eigen::Quaterniond q_seed(x(6), x(7), x(8), x(9));
            Eigen::Quaterniond q = q_seed * Eigen::Quaterniond(std::cos(half), std::sin(half), 0, 0);
            q.normalize();
            x(6) = q.w(); x(7) = q.x(); x(8) = q.y(); x(9) = q.z();
            break;
        }
        case kOmega:
            for (int i = 10; i < 13; ++i) x(i) += mag / std::sqrt(3.0);
            break;
        default: break;
    }
    return x;
}

const char* blockName(Block b) {
    switch (b) {
        case kPos:   return "position (m)";
        case kVel:   return "velocity (m/s)";
        case kTilt:  return "tilt (deg)";
        case kOmega: return "omega (rad/s)";
        default:     return "none";
    }
}

}  // namespace

int main() {
    // The OCP reads SZMUK_* as statics before main; re-exec with a clean, pinned
    // copy of the deployed realtime tuning so the probe cannot be silently
    // changed by the invoking shell's exports.
    if (!std::getenv("SZMUK_TEST_ENV_READY")) {
        std::vector<std::string> inherited;
        for (char** e = environ; e && *e; ++e) {
            const std::string kv(*e);
            const std::string name = kv.substr(0, kv.find('='));
            if (name.rfind("SZMUK_", 0) == 0) inherited.push_back(name);
        }
        for (const auto& n : inherited) unsetenv(n.c_str());
        setenv("SZMUK_TEST_ENV_READY", "1", 1);
        setenv("SZMUK_RH_N", "40", 1);
        setenv("SZMUK_MAX_ITER", "120", 1);
        setenv("SZMUK_THH", "0.16", 1);
        setenv("SZMUK_W_LAND_GS", "20000", 1);
        setenv("SZMUK_Z_STAGE", "1.55", 1);
        setenv("SZMUK_LOS_ALT_TRIG", "1.2", 1);
        setenv("SZMUK_SOLVER_VERBOSE", "0", 1);
        execv("/proc/self/exe", (char* const[]){ (char*)"tools_seed_probe", nullptr });
        std::cerr << "re-exec failed\n";
        return 1;
    }

    const OCPDescriptor& desc = OCPRegistry::getDescriptor("stc_landing_noaug");
    const planner_core::PlannerCoreConfig gate;
    CircularTarget tgt;
    PlannerConfig planner_cfg;

    QuadrotorMPC::Config cfg;
    cfg.ocp_type = "stc_landing_noaug";
    cfg.n_shift = 1;                  // profile_stc_landing_noaug_sitl.yaml: n_replay: 1
    QuadrotorMPC mpc(cfg);

    // The planner picks the activation NODE each replan and shifts the warm start
    // by the same index; in SITL that was 0.29-0.48 s, i.e. ~3 nodes at theta~0.12.
    // Mirror that. Advancing NEX=7 nodes per solve diverges the nominal loop on
    // its own and makes every perturbation number meaningless.
    const int kExec = 3;

    Eigen::VectorXd x_rel0(13);
    x_rel0 << 2.0, 1.5, StcLandingNoAugOCP::Z_STAGE + 0.05,
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xq(13);
    xq.head(3)       = tgt.pos(0.0) + x_rel0.head(3);
    xq.segment(3, 3) = x_rel0.segment(3, 3);
    xq.segment(6, 7) = x_rel0.segment(6, 7);

    const std::vector<double> pos_mags =
        {0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30};
    const std::vector<double> vel_mags  = {0.10, 0.30};
    const std::vector<double> tilt_mags = {2.0, 3.5};
    const std::vector<double> om_mags   = {0.05, 0.14};

    std::cout << "\nSeed-perturbation budget along the descent"
              << "   (gate: constraint_error <= " << gate.max_constraint_error << ")\n"
              << "STC limits: omega<=" << StcLandingNoAugOCP::OMEGA_STC_MAX
              << " rad/s, speed<=" << StcLandingNoAugOCP::SPD_STC_MAX << " m/s\n\n";

    double t_abs = 0.0;
    double min_pos_budget = 1e9;
    int valid_phases = 0;
    int rule_failures = 0;
    // Altitude-scaled seed clamp under test (see design notes above).
    const double kClampMax  = 0.15;
    const double kClampFrac = 0.12;
    const int n_steps = 24;

    for (int step = 0; step < n_steps; ++step) {
        const TargetSnapshot snap = tgt.snapshot(t_abs);
        const Eigen::VectorXd x_rel = desc.transform_state(xq, snap);

        if (step > 0) {   // skip the cold activation solve
            std::cout << "-- phase " << step
                      << "   rel z=" << std::fixed << std::setprecision(3) << x_rel(2)
                      << "  rxy=" << x_rel.head(2).norm()
                      << "  |v|=" << x_rel.segment(3, 3).norm() << "\n";

            // CONTROL: the unperturbed seed must pass, otherwise this phase's
            // numbers mean nothing (this row is what caught the probe's own
            // diverging loop the first time).
            bool phase_valid = false;
            {
                const auto saved = mpc.snapshotWarmStart();
                mpc.setNextWarmStartShift(kExec);
                auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
                auto res = mpc.solve(x_rel, snap.acceleration, extra, t_abs);
                phase_valid = res.success && res.constraint_error <= gate.max_constraint_error;
                std::cout << "     " << std::left << std::setw(15) << "CONTROL none"
                          << ": constraint=" << std::scientific << std::setprecision(2)
                          << res.constraint_error << std::fixed
                          << (phase_valid ? "   ok\n" : "   ** PHASE INVALID, skipping **\n");
                mpc.restoreWarmStart(saved);
            }

            if (phase_valid) {
                ++valid_phases;
                auto sweep = [&](Block b, const std::vector<double>& mags) {
                    double best_pass = 0.0;
                    std::cout << "     " << std::left << std::setw(15) << blockName(b) << ": ";
                    for (double m : mags) {
                        const auto saved = mpc.snapshotWarmStart();
                        mpc.setNextWarmStartShift(kExec);
                        auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
                        auto res = mpc.solve(perturb(x_rel, b, m), snap.acceleration, extra, t_abs);
                        const bool ok = res.success &&
                                        res.constraint_error <= gate.max_constraint_error;
                        if (ok) best_pass = m;
                        std::cout << std::setprecision(2) << m << (ok ? "=ok " : "=REJ ");
                        mpc.restoreWarmStart(saved);
                    }
                    std::cout << "  -> budget " << std::setprecision(2) << best_pass << "\n";
                    return best_pass;
                };

                min_pos_budget = std::min(min_pos_budget, sweep(kPos, pos_mags));
                sweep(kVel, vel_mags);
                sweep(kTilt, tilt_mags);
                sweep(kOmega, om_mags);

                // THE ACTUAL RULE the planner will use: an altitude-scaled clamp.
                // A constant clamp cannot work — the feasible budget tracks the
                // glideslope cone, which collapses to ~0.02 m at touchdown.
                {
                    const double clamp = std::min(kClampMax, kClampFrac * x_rel(2));
                    const auto saved = mpc.snapshotWarmStart();
                    mpc.setNextWarmStartShift(kExec);
                    auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
                    auto res = mpc.solve(perturb(x_rel, kPos, clamp),
                                         snap.acceleration, extra, t_abs);
                    const bool ok = res.success &&
                                    res.constraint_error <= gate.max_constraint_error;
                    std::cout << "     " << std::left << std::setw(15) << "RULE clamp"
                              << ": " << std::setprecision(3) << clamp << " m -> "
                              << (ok ? "PASS" : "** FAIL **")
                              << "  (constraint=" << std::scientific << std::setprecision(2)
                              << res.constraint_error << std::fixed << ")\n";
                    if (!ok) rule_failures++;
                    mpc.restoreWarmStart(saved);
                }
            }
            std::cout << "\n";
        }

        // Nominal advance, exactly as the planner replays.
        if (step > 0) mpc.setNextWarmStartShift(kExec);
        auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
        auto res = mpc.solve(x_rel, snap.acceleration, extra, t_abs);
        if (!res.success) { std::cerr << "nominal solve " << step << " failed\n"; break; }
        const auto& X = res.state_trajectory;
        const int k = std::min<int>(kExec, (int)X.size() - 1);
        t_abs += X[k](StcLandingNoAugOCP::IDX_DT);
        xq = desc.reconstruct_world_state(X[k].head(13), tgt.snapshot(t_abs));
        if (X[k](2) < 0.05) break;   // touched down
    }

    std::cout << "MINIMUM position budget over " << valid_phases << " valid phases: "
              << std::fixed << std::setprecision(2) << min_pos_budget << " m\n"
              << "=> a CONSTANT clamp would have to be this small (useless).\n"
              << "RULE clamp = min(" << kClampMax << ", " << kClampFrac
              << " * rel_z): " << rule_failures << " failures over "
              << valid_phases << " phases.\n";
    return 0;
}
