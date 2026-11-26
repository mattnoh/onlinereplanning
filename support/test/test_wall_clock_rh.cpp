// Wall-clock, non-SITL receding-horizon landing through the production
// asynchronous Controller API. The plant is the OCP's own model integrated at
// 1 ms in world coordinates (a perfect plant); the target either accelerates
// constantly or circles (WALL_RH_CIRCLE="R,omega"). Optimization time is real:
// the plant keeps moving while ProxDDP solves.
//
//   test_wall_clock_rh interval|node SECONDS OUTPUT_DIR
//   WALL_RH_START="rx ry rz [rvx rvy rvz]"  relative start (default per target)
//   WALL_RH_MAX_CONSECUTIVE_REJECTIONS=n    keep flying the previous plan (default 0)
#include "planner_core/adapters.hpp"
#include "planner_core/controller.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

using Clock = std::chrono::steady_clock;
using Vec = Eigen::VectorXd;
namespace Q = StcLandingNoAugOCP;

int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::invalid_argument("usage: test_wall_clock_rh interval|node SECONDS OUTPUT_DIR");
        const bool node = std::string(argv[1]) == "node";
        if (!node && std::string(argv[1]) != "interval") throw std::invalid_argument("mode must be interval or node");
        const double seconds = std::stod(argv[2]);
        const std::filesystem::path out(argv[3]);
        if (seconds <= 0 || seconds > 120 || !std::filesystem::create_directories(out))
            throw std::invalid_argument("invalid duration or existing output path");

        // Target motion.
        double R = 0, w = 0;
        const bool circle = std::getenv("WALL_RH_CIRCLE") != nullptr;
        if (circle) { char comma; std::istringstream(std::getenv("WALL_RH_CIRCLE")) >> R >> comma >> w; }
        const Eigen::Vector3d acc(.04306807449, .07358261346, 0);
        const auto targetAt = [&](double t) {
            TargetSnapshot x;
            x.valid = true;
            if (circle) {
                const double th = w * t;
                x.position << R * std::sin(th), R * (1 - std::cos(th)), 0;
                x.velocity << R * w * std::cos(th), R * w * std::sin(th), 0;
                x.acceleration << -R * w * w * std::sin(th), R * w * w * std::cos(th), 0;
            } else {
                x.position = .5 * acc * t * t; x.velocity = acc * t; x.acceleration = acc;
            }
            return x;
        };

        auto desc = std::make_shared<OCPDescriptor>(
            OCPRegistry::getDescriptor(node ? "stc_landing_noaug_node" : "stc_landing_noaug"));
        desc->use_predicted_handoff_state = false;  // the measured root is the root
        if (std::getenv("WALL_RH_WARM")) desc->native_guess_for_measured_root = false;  // shifted plan as seed
        const auto model = desc->make_model(Eigen::Vector3d::Zero());

        // Relative start; world plant = target(0) + relative.
        Vec rel0 = Vec::Zero(6);
        if (circle) rel0.head<3>() << 1.98, 0, 2.15; else rel0.head<3>() << 2., 1.5, 1.85;
        if (const char* s = std::getenv("WALL_RH_START")) {
            std::istringstream in(s);
            for (int j = 0; j < 6 && in >> rel0(j); ++j) {}
        }
        Vec plant = Vec::Zero(13);
        const auto t0 = targetAt(0);
        plant.head<3>() = t0.position + rel0.head<3>();
        plant.segment<3>(3) = t0.velocity + rel0.tail<3>();
        plant(6) = 1;
        plant = model.initial_state(plant, model.hover);

        planner_core::PlannerCoreConfig cfg;
        cfg.ocp_type = desc->name; cfg.ocp_dt = desc->dt; cfg.n_replay = 4; cfg.enable_terminal_freeze = false;
        cfg.max_first_solve_age_sec = 1.;
        planner_core::Controller controller;
        controller.configure(cfg, desc);

        // Physical audit: the untightened rules, raw consequences where triggered.
        const auto rules = Q::landingRules(false, ctcs_stc::Form::Density, 1, Eigen::Vector3d::Zero());
        std::map<std::string, double> peak;

        std::mutex m;
        double sim = 0.;
        std::int64_t ticks = 0;
        const auto origin = Clock::now();
        std::atomic<bool> stop{false}, busy{false};
        std::atomic<int> accepted{0}, attempted{0}, rejected{0};
        const char* mcr = std::getenv("WALL_RH_MAX_CONSECUTIVE_REJECTIONS");
        const int max_consecutive = mcr ? std::atoi(mcr) : 0;
        std::string failure;
        std::ofstream solves(out / "solves.jsonl"), trace(out / "plant.csv");
        solves << std::setprecision(10);
        trace << std::setprecision(10) << "t,solving,rx,ry,rz,rvx,rvy,rvz,qw,qx,qy,qz,wx,wy,wz\n";

        std::thread worker([&] {
            try {
                int consecutive = 0;
                while (!stop.load()) {
                    planner_core::PlannerCoreInput in;
                    {
                        std::lock_guard<std::mutex> lock(m);
                        in.current_state = plant; in.absolute_time_sec = sim; in.target_snapshot = targetAt(sim);
                        in.now = origin + std::chrono::milliseconds(ticks);
                    }
                    in.state_frame = "world";
                    busy = true;
                    auto r = controller.trySolve(in);
                    busy = false;
                    if (r.solve_attempted) {
                        ++attempted;
                        solves << "{\"t\":" << in.absolute_time_sec << ",\"accepted\":" << r.solve_accepted
                               << ",\"reason\":" << std::quoted(r.rejection_reason) << ",\"ms\":" << r.solve_result.solve_time_ms
                               << ",\"iters\":" << r.solve_result.solve_iters
                               << ",\"violation\":" << r.solve_result.constraint_error
                               << ",\"rows\":" << r.solve_result.primal_constraint_violation
                               << ",\"defect\":" << r.solve_result.dynamics_error << "}\n";
                        if (r.solve_accepted) { ++accepted; consecutive = 0; }
                        else { ++rejected; if (++consecutive > max_consecutive) { failure = r.rejection_reason; stop = true; } }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            } catch (const std::exception& e) { failure = e.what(); busy = false; stop = true; }
        });

        bool reached = false;
        double max_hard = 0., max_stc = 0.;
        std::string peak_term;
        int stale = 0;
        try {
            while (!stop.load() && ((sim < seconds && !reached) || busy.load())) {
                const double wall = std::chrono::duration<double>(Clock::now() - origin).count();
                if (wall - sim < .001) { std::this_thread::sleep_for(std::chrono::microseconds(500)); continue; }
                planner_core::PlannerCoreInput in;
                {
                    std::lock_guard<std::mutex> lock(m);
                    in.current_state = plant; in.absolute_time_sec = sim; in.target_snapshot = targetAt(sim);
                    in.now = origin + std::chrono::milliseconds(ticks);
                }
                in.state_frame = "world";
                const auto sample = controller.sample(in);
                Vec u = model.hover;
                if (sample.has_replay_sample && !sample.stale) u.head(sample.command.control.size()) = sample.command.control;
                if (sample.stale) ++stale;
                std::lock_guard<std::mutex> lock(m);
                plant = model.flow(plant, u, .001, false).x;
                sim = .001 * ++ticks;
                if (!plant.allFinite()) throw std::runtime_error("nonfinite plant");
                const Vec rel = planner_core::prepareOcpState(*desc, plant, targetAt(sim));
                max_hard = std::max({max_hard, -rel(2), rel.segment<3>(3).norm() - Q::P.spd_phase0_max});
                for (int j = 0; j < rules.numImplications(); ++j) {
                    if (!rules.triggered(j, rel)) continue;
                    const auto& im = rules.implication(j);
                    if (im.name == "los" && rel.head<3>().norm() < .05) continue;  // not graded within 5 cm
                    const double c = im.consequence(rel, u);
                    peak[im.name] = std::max(peak[im.name], c);
                    if (c > max_stc) { max_stc = c; peak_term = im.name; }
                }
                if (accepted.load() > 0) reached = rel.head<3>().norm() <= .1 && rel.segment<3>(3).norm() <= .3;
                trace << sim << ',' << busy.load();
                for (int j = 0; j < 13; ++j) trace << ',' << rel(j);
                trace << '\n';
            }
        } catch (...) { stop = true; worker.join(); throw; }
        stop = true;
        worker.join();
        controller.stop();

        std::ofstream summary(out / "summary.json");
        summary << std::setprecision(10) << "{\"mode\":" << std::quoted(argv[1]) << ",\"circle\":" << circle
                << ",\"reached\":" << reached << ",\"simulation_time\":" << sim << ",\"attempted\":" << attempted
                << ",\"accepted\":" << accepted << ",\"rejected\":" << rejected << ",\"stale_samples\":" << stale
                << ",\"hard_excess\":" << max_hard << ",\"physical_stc_excess\":" << max_stc
                << ",\"peak_term\":" << std::quoted(peak_term) << ",\"failure\":" << std::quoted(failure) << ",\"terms\":{";
        bool first = true;
        for (const auto& [k, v] : peak) { summary << (first ? "" : ",") << std::quoted(k) << ':' << v; first = false; }
        summary << "}}\n";
        std::cout << "WALL_CLOCK reached=" << reached << " accepted=" << accepted << " rejected=" << rejected
                  << " stc_excess=" << max_stc << " (" << peak_term << ") t=" << sim << " failure=" << failure << '\n';
        return reached ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
