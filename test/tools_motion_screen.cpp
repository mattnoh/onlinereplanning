/// @file tools_motion_screen.cpp
/// @brief Headless RH-descent screen for every target motion.
///
/// Runs a full receding-horizon landing against each target motion with no ROS
/// master and no PX4, ~6 s per descent, and reports whether the activation cold
/// solve converges and the descent completes inside the planner's acceptance
/// gate. A motion that cannot converge here will only waste SITL time, so run
/// this BEFORE launching a campaign.
///
/// The vehicle follows the plan ideally (same model as test_stc_landing_noaug),
/// so this screens the PLANNER against each motion; it does not measure
/// tracking. Tracking-dependent metrics come from the SITL campaign.

#include "planner_core/quadrotor_mpc.hpp"
#include "planner_core/ocp_registry.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "target/circular_target.hpp"
#include "target/figure8_target.hpp"
#include "target/straight_target.hpp"
#include "target/stop_and_go_target.hpp"
#include "target/segmented_accel_target.hpp"
#include "target/target_model_config.hpp"

#include <Eigen/Dense>

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

struct Scenario {
    std::string name;
    target_config::SyntheticTarget motion;
};

TargetSnapshot snapshotOf(const target_config::SyntheticTarget& m, double t) {
    TargetSnapshot s;
    s.valid = true;
    s.position = m.pos(t);
    s.velocity = m.vel(t);
    s.acceleration = m.accel(t);
    return s;
}

}  // namespace

int main() {
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
        execv("/proc/self/exe", (char* const[]){(char*)"tools_motion_screen", nullptr});
        std::cerr << "re-exec failed\n";
        return 1;
    }

    const Eigen::Vector3d ctr(0.0, 0.0, 0.2);
    std::vector<Scenario> scen;
    {
        target_models::CircularTarget m; m.center = ctr; m.R = 1.0; m.omega = 0.4;
        scen.push_back({"circle", target_config::SyntheticTarget::from(m)});
    }
    {
        target_models::Figure8Target m; m.center = ctr; m.omega = 0.3;
        scen.push_back({"figure8", target_config::SyntheticTarget::from(m)});
    }
    {
        target_models::StraightTarget m; m.center = ctr; m.speed = 0.4;
        scen.push_back({"straight", target_config::SyntheticTarget::from(m)});
    }
    {
        target_models::StopAndGoTarget m; m.center = ctr; m.speed = 0.4;
        scen.push_back({"stop_and_go", target_config::SyntheticTarget::from(m)});
    }
    scen.push_back({"piecewise_accel", target_config::SyntheticTarget::from(
        target_models::makePiecewiseAccelTarget(ctr, 0.15, 2.0, 0.5, 0.4))});
    scen.push_back({"random_accel", target_config::SyntheticTarget::from(
        target_models::makeRandomAccelTarget(ctr, 0.20, 2.0, 0.5, 0.4, 42))});

    const OCPDescriptor& desc = OCPRegistry::getDescriptor("stc_landing_noaug");
    const planner_core::PlannerCoreConfig gate;
    PlannerConfig planner_cfg;
    const int kExec = 3;

    std::cout << "\nHeadless RH descent per target motion "
              << "(gate: constraint_error <= " << gate.max_constraint_error << ")\n\n"
              << std::left << std::setw(17) << "motion"
              << std::right << std::setw(8) << "solves"
              << std::setw(8) << "rej"
              << std::setw(12) << "worst con"
              << std::setw(10) << "ms med"
              << std::setw(10) << "final z"
              << std::setw(11) << "final rxy" << "   verdict\n"
              << std::string(90, '-') << "\n";

    int failures = 0;
    for (const auto& sc : scen) {
        QuadrotorMPC::Config cfg;
        cfg.ocp_type = "stc_landing_noaug";
        cfg.n_shift = 1;
        QuadrotorMPC mpc(cfg);

        // Engage from the same settled staging offset the SITL profile uses.
        Eigen::VectorXd x_rel0(13);
        x_rel0 << 2.0, 1.5, StcLandingNoAugOCP::Z_STAGE + 0.05,
                  0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::VectorXd xq(13);
        xq.head(3) = sc.motion.pos(0.0) + x_rel0.head(3);
        xq.segment(3, 3) = x_rel0.segment(3, 3);
        xq.segment(6, 7) = x_rel0.segment(6, 7);

        double t_abs = 0.0, worst = 0.0, z = x_rel0(2), rxy = x_rel0.head(2).norm();
        int n = 0, rej = 0;
        std::vector<double> ms;
        bool cold_ok = true;

        for (int step = 0; step < 60; ++step) {
            const TargetSnapshot snap = snapshotOf(sc.motion, t_abs);
            const Eigen::VectorXd x_rel = desc.transform_state(xq, snap);
            if (step > 0) mpc.setNextWarmStartShift(kExec);
            auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
            auto res = mpc.solve(x_rel, snap.acceleration, extra, t_abs);
            ++n;
            ms.push_back(res.solve_time_ms);
            const bool ok = res.success && res.constraint_error <= gate.max_constraint_error;
            if (!ok) {
                ++rej;
                if (step == 0) { cold_ok = false; break; }
            }
            worst = std::max(worst, res.constraint_error);
            const auto& X = res.state_trajectory;
            const int k = std::min<int>(kExec, (int)X.size() - 1);
            t_abs += X[k](StcLandingNoAugOCP::IDX_DT);
            xq = desc.reconstruct_world_state(X[k].head(13), snapshotOf(sc.motion, t_abs));
            const Eigen::VectorXd xr = desc.transform_state(xq, snapshotOf(sc.motion, t_abs));
            z = xr(2);
            rxy = xr.head(2).norm();
            if (z < 0.05) break;
        }

        std::sort(ms.begin(), ms.end());
        const double med = ms.empty() ? 0.0 : ms[ms.size() / 2];
        const bool landed = cold_ok && z < 0.25 && rxy < 0.5;
        if (!landed) ++failures;
        std::cout << std::left << std::setw(17) << sc.name
                  << std::right << std::setw(8) << n
                  << std::setw(8) << rej
                  << std::setw(12) << std::scientific << std::setprecision(2) << worst
                  << std::setw(10) << std::fixed << std::setprecision(1) << med
                  << std::setw(10) << std::setprecision(3) << z
                  << std::setw(11) << rxy << "   "
                  << (!cold_ok ? "** COLD SOLVE FAILED **" : (landed ? "ok" : "** NO LANDING **"))
                  << "\n";
    }
    std::cout << "\n" << (scen.size() - failures) << "/" << scen.size()
              << " motions completed a headless descent.\n";
    return failures == 0 ? 0 : 1;
}
