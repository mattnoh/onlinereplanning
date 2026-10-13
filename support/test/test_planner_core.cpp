#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_internal.hpp"

namespace planner_core {
struct PlannerCoreTestAccess {
    using HandoffSchedule = PlannerCore::HandoffSchedule;
    template<class... Args>
    static bool prepareHandoffSchedule(const PlannerCore& core,Args&&... args) {
        return core.prepareHandoffSchedule(std::forward<Args>(args)...);
    }
    template<class... Args>
    static Eigen::VectorXd selectInitialState(const PlannerCore& core,Args&&... args) {
        return core.selectInitialState(std::forward<Args>(args)...);
    }
    static void setOdomMode(PlannerCore& core,OCPDescriptor::DroneOdomMode mode) {
        core.desc_.drone_odom_mode=mode;
    }
};
}

namespace {

using Clock = std::chrono::steady_clock;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Eigen::VectorXd state(double x, double vx = 0.0) {
    Eigen::VectorXd s = Eigen::VectorXd::Zero(13);
    s(0) = x;
    s(3) = vx;
    s(6) = 1.0;
    return s;
}

Eigen::VectorXd control(double thrust) {
    Eigen::VectorXd u = Eigen::VectorXd::Zero(4);
    u(0) = thrust;
    return u;
}

SolverResult solved(const Clock::time_point& finish, double x_offset) {
    SolverResult r;
    r.success = true;
    r.solve_finish_time = finish;
    r.solve_start_time = finish - std::chrono::milliseconds(20);
    r.solve_timestamp = r.solve_start_time;
    r.solve_time_ms = 20.0;
    r.solve_iters = 2;
    r.constraint_error = 0.0;
    r.state_trajectory = {state(x_offset), state(x_offset + 1.0), state(x_offset + 2.0)};
    r.control_trajectory = {control(9.81 + x_offset), control(10.81 + x_offset)};
    return r;
}

planner_core::PlannerCoreConfig hoverConfig() {
    planner_core::PlannerCoreConfig cfg;
    cfg.ocp_type = "hover";
    cfg.platform = planner_core::Platform::Crazyflie;
    cfg.solver_type = "alipddp";
    cfg.ocp_dt = 0.1;
    cfg.n_replay = 2;
    cfg.skip_trajectory_validation = false;
    cfg.max_constraint_error = 1.0;
    cfg.enable_terminal_freeze = false;
    cfg.terminal_position_abs = Eigen::Vector3d::Zero();
    cfg.solve_lead_guard_sec = 0.0;
    cfg.initial_solve_lead_sec = 0.02;
    return cfg;
}

planner_core::PlannerCoreInput inputAt(const Clock::time_point& now) {
    planner_core::PlannerCoreInput input;
    input.current_state = state(0.0);
    input.active_odom_mode = OCPDescriptor::DroneOdomMode::Absolute;
    input.now = now;
    input.absolute_time_sec = 1.0;
    input.target_snapshot.valid = true;
    return input;
}

}  // namespace

int main() {
    try {
        const auto t0 = Clock::now();

        {
            planner_core::PlannerCore core;core.configure(hoverConfig());
            const auto input=inputAt(t0);
            auto invalid=solved(t0,0);invalid.constraint_error=std::numeric_limits<double>::quiet_NaN();
            require(!core.acceptSolvedResult(input,input.current_state,invalid).solve_accepted,"NaN constraint must reject");
            invalid=solved(t0,0);invalid.state_trajectory[0](0)=std::numeric_limits<double>::quiet_NaN();
            require(!core.acceptSolvedResult(input,input.current_state,invalid).solve_accepted,"NaN state must reject");
            invalid=solved(t0,0);invalid.control_trajectory.pop_back();
            require(!core.acceptSolvedResult(input,input.current_state,invalid).solve_accepted,"inconsistent horizon must reject");
            invalid=solved(t0,0);invalid.state_trajectory[0].resize(3);
            require(!core.acceptSolvedResult(input,input.current_state,invalid).solve_accepted,"short state must reject");
        }
        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.ocp_type = "stc_landing_noaug";
            cfg.skip_trajectory_validation = true;
            core.configure(cfg);
            auto input = inputAt(t0);
            input.active_odom_mode = OCPDescriptor::DroneOdomMode::BodyFrameRelative;
            input.active_odom_topic = "/drone/body_relative_odom";
            const auto out = core.trySolve(input);
            require(!out.solve_accepted, "frame mismatch should reject solve");
            require(out.rejection_reason == "frame_contract", "frame mismatch reason");
        }

        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.ocp_type = "stc_landing_noaug";
            cfg.skip_trajectory_validation = true;
            cfg.max_first_solve_age_sec = 1.0;
            core.configure(cfg);
            // Exercise generic relative-frame guards with the active descriptor.
            planner_core::PlannerCoreTestAccess::setOdomMode(
                core,OCPDescriptor::DroneOdomMode::TargetFrameRelative);
            auto input = inputAt(t0);
            input.active_odom_mode = OCPDescriptor::DroneOdomMode::TargetFrameRelative;
            input.active_odom_topic = "/mavros/local_position/odom";
            auto stale = solved(t0, 0.0);
            stale.solve_time_ms = 2000.0;
            auto out = core.acceptSolvedResult(input, input.current_state, stale);
            require(!out.solve_accepted, "stale first relative solve should reject");
            require(out.rejection_reason == "first_solve_stale",
                    "stale first relative solve rejection reason");
        }

        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.ocp_type = "stc_landing_noaug";
            cfg.skip_trajectory_validation = true;
            core.configure(cfg);
            // Exercise generic relative-frame guards with the active descriptor.
            planner_core::PlannerCoreTestAccess::setOdomMode(
                core,OCPDescriptor::DroneOdomMode::TargetFrameRelative);
            auto input = inputAt(t0);
            input.active_odom_mode = OCPDescriptor::DroneOdomMode::TargetFrameRelative;
            input.active_odom_topic = "/mavros/local_position/odom";
            auto bad = solved(t0, 0.0);
            for(auto& x:bad.state_trajectory) { const int n=x.size(); x.conservativeResize(20);x.tail(20-n).setZero(); }
            bad.state_trajectory.back()(2) = -20.0;
            auto out = core.acceptSolvedResult(input, input.current_state, bad);
            require(!out.solve_accepted, "divergent relative trajectory should reject");
            require(out.rejection_reason == "relative_position_diverged" ||
                        out.rejection_reason == "relative_vertical_diverged",
                    "divergent relative trajectory rejection reason");
        }

        {
            planner_core::PlannerCore core;
            core.configure(hoverConfig());
            auto input = inputAt(t0);
            auto out = core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));
            require(out.solve_accepted, "first plan should be accepted");
            auto replay0 = core.sampleReplay(t0, input.target_snapshot);
            require(replay0.has_replay_sample, "first plan should activate immediately");
            require(std::abs(replay0.command.state(0)) < 1e-9, "first plan should sample node 0");
        }

        {
            planner_core::PlannerCore core;
            core.configure(hoverConfig());
            auto input = inputAt(t0);
            core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));
            core.acceptSolvedResult(input, input.current_state, solved(t0 + std::chrono::milliseconds(50), 10.0));
            auto before = core.sampleReplay(t0 + std::chrono::milliseconds(100), input.target_snapshot);
            require(before.has_replay_sample, "pending swap before sample should have plan");
            require(before.command.state(0) < 10.0, "pending plan activated too early");
            auto at_handoff = core.sampleReplay(t0 + std::chrono::milliseconds(200), input.target_snapshot);
            require(at_handoff.has_replay_sample, "handoff sample should have plan");
            require(at_handoff.command.state(0) >= 10.0, "pending plan did not activate at handoff");
            bool saw_handoff = false;
            for (const auto& d : at_handoff.diagnostics) {
                saw_handoff = saw_handoff || d.code == "handoff";
            }
            require(saw_handoff, "handoff diagnostic missing");
        }

        {
            planner_core::PlannerCore core;
            core.configure(hoverConfig());
            auto input = inputAt(t0);
            core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));
            auto bad = solved(t0 + std::chrono::milliseconds(50), 5.0);
            bad.constraint_error = 100.0;
            auto rejected = core.acceptSolvedResult(input, input.current_state, bad);
            require(!rejected.solve_accepted, "bad solve should reject");
            auto replay = core.sampleReplay(t0 + std::chrono::milliseconds(100), input.target_snapshot);
            require(replay.has_replay_sample, "previous plan should remain after rejection");
            require(replay.command.state(0) < 5.0, "rejected plan replaced active plan");
        }

        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.recovery_pos_err = 1000.0;
            cfg.recovery_vel_err = 1000.0;
            core.configure(cfg);
            auto input = inputAt(t0);
            core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));

            auto early = inputAt(t0 + std::chrono::milliseconds(150));
            planner_core::PlannerCoreStepResult early_out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule early_handoff;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,early, early_out, early_handoff),
                    "future handoff should solve immediately and wait as a pending plan");

            auto handoff = inputAt(t0 + std::chrono::milliseconds(180));
            planner_core::PlannerCoreStepResult out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,handoff, out, schedule),
                    "solve should start in lead window");
            double hp = 0.0, hv = 0.0;
            Eigen::VectorXd x0 = planner_core::PlannerCoreTestAccess::selectInitialState(core,
                handoff.current_state, handoff.target_snapshot, schedule,
                &hp, &hv, out.x0_source, out);
            require(out.x0_source == "predicted_handoff",
                    "lead-window solve should use predicted handoff state");
            require(x0.size() > 0 && std::abs(x0(0) - 2.0) < 1e-6,
                    "predicted handoff did not sample active plan at activation elapsed");
        }

        {
            // Measured-state seeding. The seed carries the persistent, FEASIBLE
            // part of the tracking error onto the predicted state: POSITION
            // only, clamped by relative altitude. Attitude and body rates must
            // never move — a measured attitude/rate seed is infeasible against
            // the STC bounds and diverges the solver.
            // The shared fixture plans at z = 0, where the altitude-scaled clamp
            // is correctly zero. Give this plan a realistic altitude so the seed
            // path is actually exercised.
            const double kSeedZ = 1.5;
            const auto solvedAtZ = [&](const Clock::time_point& finish) {
                SolverResult r = solved(finish, 0.0);
                for (auto& x : r.state_trajectory) x(2) = kSeedZ;
                return r;
            };

            const auto seed_with = [&](double alpha, double cap, double per_m) {
                planner_core::PlannerCore core;
                auto cfg = hoverConfig();
                cfg.recovery_pos_err = 1000.0;
                cfg.recovery_vel_err = 1000.0;
                cfg.seed_alpha_pos = alpha;
                cfg.seed_clamp_pos_cap_m = cap;
                cfg.seed_clamp_pos_per_m = per_m;
                core.configure(cfg);
                auto input = inputAt(t0);
                core.acceptSolvedResult(input, input.current_state, solvedAtZ(t0));

                auto early = inputAt(t0 + std::chrono::milliseconds(150));
                planner_core::PlannerCoreStepResult early_out;
                planner_core::PlannerCoreTestAccess::HandoffSchedule early_handoff;
                planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,early, early_out, early_handoff);

                auto handoff = inputAt(t0 + std::chrono::milliseconds(180));
                planner_core::PlannerCoreStepResult out;
                planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
                require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,handoff, out, schedule),
                        "measured-seed fixture: solve should start in lead window");
                double hp = 0.0, hv = 0.0;
                Eigen::VectorXd x = planner_core::PlannerCoreTestAccess::selectInitialState(core,
                    handoff.current_state, handoff.target_snapshot, schedule,
                    &hp, &hv, out.x0_source, out);
                return std::make_pair(x, out.x0_source);
            };

            // alpha = 0 must reproduce the pre-existing predicted seed exactly.
            const auto off = seed_with(0.0, 0.10, 0.10);
            require(off.second == "predicted_handoff",
                    "seed_alpha_pos = 0 must fall through to the predicted seed");
            require(std::abs(off.first(0) - 2.0) < 1e-12,
                    "seed_alpha_pos = 0 perturbed the predicted seed");

            // A generous clamp lets the full correction through; the fixture's
            // live state is x = 0 while the plan is already past x = 1.8, so the
            // correction is negative.
            const auto on = seed_with(0.5, 10.0, 10.0);
            require(on.second == "measured_seed",
                    "nonzero seed_alpha_pos must be reported as measured_seed");
            require(on.first(0) < off.first(0),
                    "measured seed did not move toward the live state");

            // Only the position block may move.
            for (int i = 3; i < off.first.size(); ++i) {
                require(std::abs(on.first(i) - off.first(i)) < 1e-12,
                        "measured seed modified a non-position state element");
            }

            // The clamp must bound the applied correction.
            const auto clamped = seed_with(0.5, 0.05, 10.0);
            const double moved = (clamped.first.head(3) - off.first.head(3)).norm();
            require(moved <= 0.05 + 1e-9,
                    "measured seed exceeded seed_clamp_pos_cap_m");
            require(moved > 0.0, "clamped measured seed applied no correction");

            // The clamp also scales with relative altitude: per_m = 0 forbids
            // any correction regardless of the cap.
            const auto by_alt = seed_with(0.5, 10.0, 0.0);
            require(by_alt.second == "predicted_handoff",
                    "zero per-metre clamp must leave the predicted seed untouched");
        }

        {
            // Regression: when solve latency is longer than one replay step,
            // the next origin must remain in the future instead of advancing
            // by an already-expired n_replay interval on every solve.
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.n_replay = 1;             // 0.1 s replay interval
            cfg.solve_lead_guard_sec = 0.02;
            core.configure(cfg);

            auto first = solved(t0, 0.0);
            first.solve_time_ms = 170.0;  // 0.19 s lead, longer than replay
            core.acceptSolvedResult(inputAt(t0), state(0.0), first);

            auto next_input = inputAt(t0 + std::chrono::milliseconds(10));
            planner_core::PlannerCoreStepResult next_out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule next_schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,next_input, next_out, next_schedule),
                    "long solve should start immediately when lead spans the replay interval");
            require(next_schedule.activation_elapsed > next_schedule.active_elapsed_now,
                    "handoff activation was scheduled in the past");
            require(next_schedule.activation_elapsed >= 0.19,
                    "handoff did not reserve the measured solve lead");

            auto second = solved(t0 + std::chrono::milliseconds(200), 1.0);
            second.solve_time_ms = 170.0;
            auto accepted = core.acceptSolvedResult(
                next_input, state(0.0), second, next_out);
            require(accepted.solve_accepted, "long-latency successor should be accepted");
            core.sampleReplay(t0 + std::chrono::milliseconds(210),
                              next_input.target_snapshot);

            auto third_input = inputAt(t0 + std::chrono::milliseconds(220));
            planner_core::PlannerCoreStepResult third_out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule third_schedule;
            planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,third_input, third_out, third_schedule);
            require(third_schedule.activation_elapsed >
                        third_schedule.active_elapsed_now,
                    "successive handoff reused an expired origin");
        }

        {
            // A required activation beyond the remaining active horizon must
            // be visible to the caller; never silently fall back to an
            // unaligned activation and mismatched warm-start shift.
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            core.configure(cfg);
            auto first = solved(t0, 0.0);
            first.solve_time_ms = 1000.0;
            core.acceptSolvedResult(inputAt(t0), state(0.0), first);

            planner_core::PlannerCoreStepResult out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(!planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,
                        inputAt(t0 + std::chrono::milliseconds(10)), out, schedule),
                    "handoff beyond the active horizon must be rejected");
            require(out.rejection_reason == "handoff_beyond_active_horizon",
                    "missing explicit beyond-horizon rejection reason");
        }

        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.recovery_pos_err = 0.25;
            cfg.recovery_vel_err = 1.0;
            core.configure(cfg);
            auto input = inputAt(t0);
            core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));

            auto recovery = inputAt(t0 + std::chrono::milliseconds(180));
            recovery.current_state = state(100.0);
            planner_core::PlannerCoreStepResult out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,recovery, out, schedule),
                    "recovery should be inside lead window");
            double hp = 0.0, hv = 0.0;
            Eigen::VectorXd x0 = planner_core::PlannerCoreTestAccess::selectInitialState(core,
                recovery.current_state, recovery.target_snapshot, schedule,
                &hp, &hv, out.x0_source, out);
            require(out.x0_source == "recovery_live",
                    "large tracking error should use explicit recovery_live source");
            require(x0.size() > 0 && std::abs(x0(0) - 100.0) < 1e-6,
                    "recovery_live did not use live state");
        }

        {
            planner_core::PlannerCore core;
            auto cfg = hoverConfig();
            cfg.recovery_pos_err = 0.05;
            cfg.recovery_vel_err = 1.0;
            core.configure(cfg);
            auto input = inputAt(t0);
            core.acceptSolvedResult(input, input.current_state, solved(t0, 0.0));

            auto handoff = inputAt(t0 + std::chrono::milliseconds(180));
            handoff.current_state = state(1.8);
            planner_core::PlannerCoreStepResult out;
            planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,handoff, out, schedule),
                    "handoff should be inside lead window");
            double hp = 0.0, hv = 0.0;
            Eigen::VectorXd x0 = planner_core::PlannerCoreTestAccess::selectInitialState(core,
                handoff.current_state, handoff.target_snapshot, schedule,
                &hp, &hv, out.x0_source, out);
            require(out.x0_source == "predicted_handoff",
                    "tracking error must compare against active plan at now, not future handoff");
            require(hp < 1e-6, "tracking error should be near zero at current active-plan time");
            require(x0.size() > 0 && std::abs(x0(0) - 2.0) < 1e-6,
                    "selected x0 should still be the future handoff state");
        }

        {
            OCPDescriptor desc;
            desc.drone_odom_mode = OCPDescriptor::DroneOdomMode::TargetFrameRelative;
            TargetSnapshot target;
            target.valid = true;
            target.position = Eigen::Vector3d(1.0, 2.0, 2.0);
            target.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);
            target.angular_velocity = Eigen::Vector3d(0.0, 0.0, 1.0);
            target.orientation = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
            Eigen::VectorXd x = state(0.0);
            x.segment(0, 3) = Eigen::Vector3d(0.0, 1.0, 0.2);
            x.segment(3, 3) = Eigen::Vector3d(0.2, 0.0, 0.0);
            const Eigen::Vector4d hover =
                planner_core::makeCrazyflieHoverCommand(desc, x, control(9.81), target);
            require(std::abs(hover(0) + 0.5) < 1e-6,
                    "TF hover vx must include target velocity and frame-rotation term");
            require(std::abs(hover(2) - 2.2) < 1e-6,
                    "TF hover z must reconstruct world z from target-frame position");
        }

        {
            OCPDescriptor desc;
            desc.drone_odom_mode = OCPDescriptor::DroneOdomMode::BodyFrameRelative;
            TargetSnapshot target;
            target.valid = true;
            target.position = Eigen::Vector3d(1.0, 2.0, 3.0);
            target.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);
            target.orientation = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
            Eigen::VectorXd x = state(0.0);
            x.segment(0, 3) = Eigen::Vector3d(0.0, 0.0, 1.0);
            x.segment(3, 3) = Eigen::Vector3d(0.2, 0.0, 0.0);
            const Eigen::Vector4d hover =
                planner_core::makeCrazyflieHoverCommand(desc, x, control(9.81), target);
            require(std::abs(hover(0) - 0.5) < 1e-6,
                    "BF hover vx must include target velocity in drone body frame");
            require(std::abs(hover(2) - 2.0) < 1e-6,
                    "BF hover z must reconstruct drone world altitude from p_T^B");
        }
    } catch (const std::exception& e) {
        std::cerr << "PlannerCore tests failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "PlannerCore tests passed\n";
    return 0;
}
