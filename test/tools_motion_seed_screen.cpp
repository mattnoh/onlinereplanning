/// @file tools_motion_seed_screen.cpp
/// @brief Headless feasibility screen for EVERY randomised trial of a campaign.
///
/// tools_motion_screen checks one fixed instance per motion. Once
/// target_motion_seed actually randomises the shape (phase, heading, segment
/// index, shuttle offset), each trial is a DIFFERENT engagement geometry, and
/// none of them have been tuned for. This sweeps all (motion, trial) cells
/// headlessly -- no ROS master, no PX4, ~6 s per descent -- so an infeasible
/// random draw is found in minutes instead of after a 9-hour SITL campaign.
///
/// Seeding mirrors run_scenario_campaign.py::seed_for and the draw ORDER in
/// target_publisher.cpp exactly. If either changes, this screen stops being
/// predictive -- keep them in sync.
///
/// The vehicle follows the plan ideally, so this screens the PLANNER geometry,
/// not tracking. A cell that fails here would almost certainly fail in SITL;
/// a cell that passes here can still fail on tracking.

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
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace {

const std::vector<std::string> kMotions = {
    "circle", "figure8", "straight", "stop_and_go", "piecewise_accel", "random_accel"};

// zlib crc32 without linking zlib: the campaign only uses it modulo 1000.
uint32_t crc32_of(const std::string& s) {
    static uint32_t tbl[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            tbl[i] = c;
        }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (unsigned char ch : s) c = tbl[(c ^ ch) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

/// Mirrors run_scenario_campaign.py::seed_for. `level_idx` is the index into
/// UNCERT (zero=0, nominal=1, high=2).
int seed_for(const std::string& motion, int level_idx, int trial) {
    return static_cast<int>(crc32_of(motion) % 1000) * 10000 + level_idx * 100 + trial + 1;
}

struct Draw { double phase, heading, t_off; int k0; };

/// Mirrors the draw order in target_publisher.cpp. Order matters.
Draw drawShape(unsigned seed, double speed, double span) {
    std::mt19937 g(seed);
    std::uniform_real_distribution<double> u2pi(0.0, 2.0 * M_PI);
    std::uniform_int_distribution<int> u8(0, 7);
    Draw d;
    d.phase = u2pi(g);
    d.heading = u2pi(g);
    d.k0 = u8(g);
    const double T = (speed > 1e-9 && span > 0.0) ? 4.0 * span / speed : 0.0;
    std::uniform_real_distribution<double> us(0.0, std::max(T, 1e-9));
    d.t_off = us(g);
    return d;
}

target_config::SyntheticTarget buildTarget(const std::string& motion, const Draw& d,
                                           unsigned seed) {
    const Eigen::Vector3d ctr(0.0, 0.0, 0.2);
    if (motion == "circle") {
        target_models::CircularTarget m;
        m.center = ctr; m.R = 1.0; m.omega = 0.4; m.phi0 = d.phase;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "figure8") {
        target_models::Figure8Target m;
        m.center = ctr; m.omega = 0.3; m.phi0 = d.phase;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "straight") {
        target_models::StraightTarget m;
        m.center = ctr; m.speed = 0.4; m.span = 2.0;
        m.heading = d.heading; m.t_offset = d.t_off;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "stop_and_go") {
        target_models::StopAndGoTarget m;
        m.center = ctr; m.speed = 0.4; m.span = 2.0;
        m.heading = d.heading; m.t_offset = d.t_off;
        return target_config::SyntheticTarget::from(m);
    }
    if (motion == "piecewise_accel") {
        return target_config::SyntheticTarget::from(
            target_models::makePiecewiseAccelTarget(ctr, 0.15, 2.0, 0.5, 0.4,
                                                    d.heading, d.k0));
    }
    return target_config::SyntheticTarget::from(
        target_models::makeRandomAccelTarget(ctr, 0.20, 2.0, 0.5, 0.4, seed, d.heading));
}

TargetSnapshot snapshotOf(const target_config::SyntheticTarget& m, double t) {
    TargetSnapshot s;
    s.valid = true;
    s.position = m.pos(t);
    s.velocity = m.vel(t);
    s.acceleration = m.accel(t);
    return s;
}

struct Result {
    bool cold_ok = true;
    int n = 0, rej = 0;
    double worst = 0.0, z = 0.0, rxy = 0.0, ms_med = 0.0;
    bool landed = false;
};

/// Staging bearing measured FROM the target's own heading. `alpha < -10`
/// means "use the legacy fixed world offset (1.4, 1.4)".
double g_stage_alpha = -99.0;
const double kStageR = 1.9799;   // = |(1.4, 1.4)|, same engagement range

Result runDescent(const target_config::SyntheticTarget& motion,
                  const OCPDescriptor& desc,
                  const planner_core::PlannerCoreConfig& gate) {
    PlannerConfig planner_cfg;
    const int kExec = 3;
    QuadrotorMPC::Config cfg;
    cfg.ocp_type = "stc_landing_noaug";
    cfg.n_shift = 1;
    QuadrotorMPC mpc(cfg);

    // Staging offset MATCHES the campaign: run_scenario_campaign.py hovers at
    // target + (1.4, 1.4). tools_motion_screen used (2.0, 1.5), which is a
    // different engagement range and would make this screen non-predictive.
    // Fixed world offset (legacy) vs offset at a fixed bearing from the
    // target's heading. The latter makes engagement geometry identical in
    // every trial, so the campaign varies the MOTION and not the approach
    // bearing -- which was never meant to be a variable of this study.
    double ox = 1.4, oy = 1.4;
    if (g_stage_alpha > -10.0) {
        // A stop-and-go target engaged during its DWELL has |v| = 0, so there
        // is no heading to stage relative to. Falling back to psi = 0 picked an
        // arbitrary world bearing and was exactly why stop_and_go still failed
        // 5/20 after the staging fix. Look ahead to the next instant the target
        // is actually moving and use that direction instead.
        double psi = 0.0;
        for (double tp = 0.0; tp <= 30.0; tp += 0.05) {
            const Eigen::Vector3d v = motion.vel(tp);
            if (v.head(2).norm() > 0.05) { psi = std::atan2(v.y(), v.x()); break; }
        }
        ox = kStageR * std::cos(psi + g_stage_alpha);
        oy = kStageR * std::sin(psi + g_stage_alpha);
    }
    Eigen::VectorXd x_rel0(13);
    x_rel0 << ox, oy, StcLandingNoAugOCP::Z_STAGE + 0.05,
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd xq(13);
    xq.head(3) = motion.pos(0.0) + x_rel0.head(3);
    xq.segment(3, 3) = x_rel0.segment(3, 3);
    xq.segment(6, 7) = x_rel0.segment(6, 7);

    Result r;
    r.z = x_rel0(2);
    r.rxy = x_rel0.head(2).norm();
    double t_abs = 0.0;
    std::vector<double> ms;

    for (int step = 0; step < 60; ++step) {
        const TargetSnapshot snap = snapshotOf(motion, t_abs);
        const Eigen::VectorXd x_rel = desc.transform_state(xq, snap);
        if (step > 0) mpc.setNextWarmStartShift(kExec);
        auto extra = desc.prepare_extra(planner_cfg, t_abs, snap);
        auto res = mpc.solve(x_rel, snap.acceleration, extra, t_abs);
        ++r.n;
        ms.push_back(res.solve_time_ms);
        const bool ok = res.success && res.constraint_error <= gate.max_constraint_error;
        if (!ok) {
            ++r.rej;
            if (step == 0) { r.cold_ok = false; break; }
        }
        r.worst = std::max(r.worst, res.constraint_error);
        const auto& X = res.state_trajectory;
        const int k = std::min<int>(kExec, static_cast<int>(X.size()) - 1);
        t_abs += X[k](StcLandingNoAugOCP::IDX_DT);
        xq = desc.reconstruct_world_state(X[k].head(13), snapshotOf(motion, t_abs));
        const Eigen::VectorXd xr = desc.transform_state(xq, snapshotOf(motion, t_abs));
        r.z = xr(2);
        r.rxy = xr.head(2).norm();
        if (r.z < 0.05) break;
    }
    std::sort(ms.begin(), ms.end());
    r.ms_med = ms.empty() ? 0.0 : ms[ms.size() / 2];
    // z must be ON the ground, not THROUGH it: some diverged solves finished
    // at z = -0.32 m with constraint_error ~ 3e9 and were scored as landings.
    r.landed = r.cold_ok && r.z > -0.02 && r.z < 0.25 && r.rxy < 0.5
               && r.worst <= gate.max_constraint_error;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (!std::getenv("SZMUK_TEST_ENV_READY")) {
        std::vector<std::string> inherited;
        for (char** e = environ; e && *e; ++e) {
            const std::string kv(*e);
            const std::string name = kv.substr(0, kv.find('='));
            if (name.rfind("SZMUK_", 0) == 0) inherited.push_back(name);
        }
        for (const auto& n : inherited) unsetenv(n.c_str());
        setenv("SZMUK_TEST_ENV_READY", "1", 1);
        // Same planner env the campaign exports (PLANNER_ENV + launch args).
        setenv("SZMUK_RH_N", "40", 1);
        setenv("SZMUK_MAX_ITER", "120", 1);
        setenv("SZMUK_THH", "0.16", 1);
        setenv("SZMUK_W_LAND_GS", "20000", 1);
        setenv("SZMUK_Z_STAGE", "1.55", 1);
        setenv("SZMUK_LOS_ALT_TRIG", "1.2", 1);
        setenv("SZMUK_SOLVER_VERBOSE", "0", 1);
        execv("/proc/self/exe", argv);
        std::cerr << "re-exec failed\n";
        return 1;
    }

    // Sweep mode: map the feasible set of one shape parameter directly,
    // instead of inferring it from a handful of random draws.
    //   tools_motion_seed_screen sweep <motion> <param> <lo> <hi> <n>
    // param: toff | head
    // alpha sweep: for each candidate staging bearing, how many seeded trials
    // of every motion survive? Picks the staging geometry empirically instead
    // of reasoning about which bearings "should" be hard.
    if (argc > 1 && std::string(argv[1]) == "alpha") {
        const int trials = (argc > 2) ? std::atoi(argv[2]) : 6;
        const int n = (argc > 3) ? std::atoi(argv[3]) : 12;
        const OCPDescriptor& d0 = OCPRegistry::getDescriptor("stc_landing_noaug");
        const planner_core::PlannerCoreConfig g0;
        std::fprintf(stderr, "\nstaging-bearing sweep: %d trials x %zu motions per alpha\n\n%8s",
                     trials, kMotions.size(), "alpha");
        for (const auto& m : kMotions)
            std::fprintf(stderr, "%9s", m.substr(0, 8).c_str());
        std::fprintf(stderr, "%10s\n", "TOTAL");
        for (int i = 0; i < 8 + 9 * 6 + 10; ++i) std::fputc('-', stderr);
        std::fputc('\n', stderr);
        for (int i = 0; i < n; ++i) {
            g_stage_alpha = 2.0 * M_PI * i / n;
            int tot = 0;
            // Built as ONE string and written to stderr in a single call. The
            // planner logs "[MPC] solve ..." to stdout from inside the solve
            // loop, so a << chain to stdout gets interleaved mid-line and the
            // table came out shredded (columns eaten by the log filter).
            char row[512];
            int off = std::snprintf(row, sizeof row, "%8.3f", g_stage_alpha);
            for (const auto& mo : kMotions) {
                int ok = 0;
                for (int tr = 0; tr < trials; ++tr) {
                    const unsigned sd = static_cast<unsigned>(seed_for(mo, 0, tr));
                    const auto tg = buildTarget(mo, drawShape(sd, 0.4, 2.0), sd);
                    if (runDescent(tg, d0, g0).landed) ++ok;
                }
                tot += ok;
                off += std::snprintf(row + off, sizeof row - off, "%6d/%-2d", ok, trials);
            }
            std::snprintf(row + off, sizeof row - off, "%7d/%-2d\n", tot,
                          trials * (int)kMotions.size());
            std::fputs(row, stderr);
            std::fflush(stderr);
        }
        return 0;
    }

    if (argc > 1 && std::string(argv[1]) == "sweep") {
        const std::string mo = argv[2];
        const std::string what = argv[3];
        const double lo = std::atof(argv[4]), hi = std::atof(argv[5]);
        const int n = std::atoi(argv[6]);
        const OCPDescriptor& d0 = OCPRegistry::getDescriptor("stc_landing_noaug");
        const planner_core::PlannerCoreConfig g0;
        std::cout << "\nsweep " << mo << " " << what << " in [" << lo << ", " << hi
                  << "], " << n << " points\n\n"
                  << std::right << std::setw(9) << what << std::setw(10) << "|tv0|"
                  << std::setw(12) << "worst con" << std::setw(9) << "final z"
                  << std::setw(10) << "final rxy" << "  verdict\n"
                  << std::string(62, '-') << "\n";
        for (int i = 0; i < n; ++i) {
            const double v = lo + (hi - lo) * i / std::max(1, n - 1);
            Draw d{}; d.phase = 0.0; d.heading = 0.0; d.k0 = 0; d.t_off = 0.0;
            if (what == "toff") d.t_off = v; else d.heading = v;
            const auto tg = buildTarget(mo, d, 42u);
            const Result r = runDescent(tg, d0, g0);
            std::cout << std::right << std::fixed << std::setprecision(2) << std::setw(9) << v
                      << std::setw(10) << std::setprecision(3) << tg.vel(0.0).head(2).norm()
                      << std::setw(12) << std::scientific << std::setprecision(2) << r.worst
                      << std::setw(9) << std::fixed << std::setprecision(3) << r.z
                      << std::setw(10) << r.rxy << "  "
                      << (!r.cold_ok ? "COLD-FAIL" : (r.landed ? "ok" : "NO-LAND")) << "\n";
        }
        return 0;
    }

    const int trials = (argc > 1) ? std::atoi(argv[1]) : 20;
    const int level_idx = (argc > 2) ? std::atoi(argv[2]) : 0;   // 0 = "zero"
    const bool verbose  = (argc > 3) && std::atoi(argv[3]) != 0;
    if (argc > 4) g_stage_alpha = std::atof(argv[4]);

    const OCPDescriptor& desc = OCPRegistry::getDescriptor("stc_landing_noaug");
    const planner_core::PlannerCoreConfig gate;

    std::fprintf(stderr, "\nRandomised-trial feasibility screen (%d trials, level_idx=%d, "
                 "stage_alpha=%.3f)\n", trials, level_idx, g_stage_alpha);
    std::cout << "\nRandomised-trial feasibility screen  (" << trials
              << " trials x " << kMotions.size() << " motions, level_idx="
              << level_idx << ")\n"
              << "gate: constraint_error <= " << gate.max_constraint_error
              << ";  land = final z < 0.25 m and rxy < 0.5 m\n\n"
              << std::left << std::setw(17) << "motion"
              << std::right << std::setw(9) << "landed"
              << std::setw(9) << "cold!"
              << std::setw(12) << "worst con"
              << std::setw(10) << "ms med"
              << std::setw(11) << "rxy max" << "   failing trials\n"
              << std::string(100, '-') << "\n";

    int total = 0, total_ok = 0;
    for (const auto& mo : kMotions) {
        int ok = 0, cold_fail = 0;
        double worst = 0.0, rxy_max = 0.0;
        std::vector<double> meds;
        std::vector<int> bad;
        for (int tr = 0; tr < trials; ++tr) {
            const unsigned seed = static_cast<unsigned>(seed_for(mo, level_idx, tr));
            const Draw d = drawShape(seed, 0.4, 2.0);
            const auto target = buildTarget(mo, d, seed);
            const Result r = runDescent(target, desc, gate);
            if (verbose) {
                const Eigen::Vector3d tv = target.vel(0.0);
                // Angle between the target's velocity and the drone's offset
                // direction (drone sits at target + (1.4, 1.4)).
                const Eigen::Vector2d off(1.4, 1.4);
                const double cosang = (tv.head(2).norm() > 1e-9)
                    ? tv.head(2).normalized().dot(off.normalized()) : 0.0;
                std::fprintf(stderr,
                    "    %-16s t%-3d seed=%-9u head=%5.2f phase=%5.2f k0=%d "
                    "toff=%5.2f  |tv|=%.3f cos(tv,off)=%+.3f  ->  %s\n",
                    mo.c_str(), tr, seed, d.heading, d.phase, d.k0, d.t_off,
                    tv.head(2).norm(), cosang, r.landed ? "ok" : "FAIL");
            }
            ++total;
            if (r.landed) { ++ok; ++total_ok; } else { bad.push_back(tr); }
            if (!r.cold_ok) ++cold_fail;
            worst = std::max(worst, r.worst);
            rxy_max = std::max(rxy_max, r.rxy);
            meds.push_back(r.ms_med);
        }
        std::sort(meds.begin(), meds.end());
        std::string badstr;
        for (size_t i = 0; i < bad.size(); ++i)
            badstr += (i ? "," : "") + std::string("t") + std::to_string(bad[i]);
        std::fprintf(stderr, "%-17s%6d/%-2d%9d%12.2e%10.1f%11.3f   %s\n",
                     mo.c_str(), ok, trials, cold_fail, worst,
                     (meds.empty() ? 0.0 : meds[meds.size() / 2]), rxy_max,
                     badstr.empty() ? "-" : badstr.c_str());
        std::fflush(stderr);
    }
    std::fprintf(stderr, "\n%d/%d randomised trials completed a headless descent.\n",
                 total_ok, total);
    return (total_ok == total) ? 0 : 1;
}
