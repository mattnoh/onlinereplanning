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

    input.now = now;
    input.absolute_time_sec = 1.0;
    input.target_snapshot.valid = true;
    return input;
}

}  // namespace

int main() {
    try {
        const auto t0 = Clock::now();
        // Analytic delay predictor: full measured state, known committed input,
        // correct future epoch; signed acceleration is not quad thrust.
        {
            auto desc=std::make_shared<OCPDescriptor>();desc->name="analytic_pointmass";
            desc->state_dim=4;desc->control_dim=2;desc->dt=.1;
            desc->getSolverParams=[](){return Param{};};
            desc->make_model=[](const Eigen::Vector3d&){
                OCPDescriptor::Model model;model.nx=4;model.nu=2;model.dt=.1;model.horizon=20;
                model.hover=Eigen::Vector2d(-.25,0.);
                model.flow=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u,double h,bool){
                    OCPDescriptor::Flow result;result.x=x;
                    result.x.head<2>()+=h*x.tail<2>()+.5*h*h*u;
                    result.x.tail<2>()+=h*u;return result;
                };return model;
            };
            auto cfg=hoverConfig();cfg.ocp_type=desc->name;cfg.ocp_dt=.1;
            cfg.measured_state_delay_compensation=true;cfg.initial_solve_lead_sec=.2;cfg.solve_lead_guard_sec=.05;
            planner_core::PlannerCore core;core.configure(cfg,desc);
            auto input=inputAt(t0);input.current_state=Eigen::Vector4d(0.,0.,1.,0.);
            planner_core::PlannerCoreStepResult out;planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,input,out,schedule),"delay schedule missing");
            require(std::abs(std::chrono::duration<double>(schedule.activation_time-t0).count()-.25)<1e-9,"wrong activation timestamp");
            double p=0,v=0;std::string source;
            auto root=planner_core::PlannerCoreTestAccess::selectInitialState(core,input.current_state,input.target_snapshot,schedule,&p,&v,source,out);
            require(source=="measured_rollout"&&std::abs(root(0)-(.25-.125*.25*.25))<1e-10&&std::abs(root(2)-.9375)<1e-10,"measured rollout does not match analytic plant");
            input.current_state(2)=2.;
            auto changed=planner_core::PlannerCoreTestAccess::selectInitialState(core,input.current_state,input.target_snapshot,schedule,&p,&v,source,out);
            require(std::abs(changed(0)-root(0)-.25)<1e-10,"delay root ignored changed measurement");
            auto result=solved(t0,0.);result.state_trajectory=std::vector<Eigen::VectorXd>(8,input.current_state);
            result.control_trajectory=std::vector<Eigen::VectorXd>(7,Eigen::Vector2d(-.5,0));
            result.solve_start_time=t0;result.solve_finish_time=t0+std::chrono::milliseconds(20);
            const auto accepted=core.acceptSolvedResult(input,input.current_state,result);
            require(accepted.solve_accepted,"signed pointmass acceleration rejected as quad thrust");
            require(!core.sampleReplay(t0+std::chrono::milliseconds(50),input.target_snapshot).has_replay_sample,"first plan applied before scheduled activation");
            input.now=t0+std::chrono::milliseconds(300);input.current_state=Eigen::Vector4d(0.,0.,1.,0.);
            planner_core::PlannerCoreTestAccess::HandoffSchedule next;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,input,out,next),"successor delay schedule missing");
            auto successor=planner_core::PlannerCoreTestAccess::selectInitialState(core,input.current_state,input.target_snapshot,next,&p,&v,source,out);
            const double dt=std::chrono::duration<double>(next.activation_time-input.now).count();
            require(std::abs(successor(0)-(dt-.25*dt*dt))<1e-10&&std::abs(successor(2)-(1.-.5*dt))<1e-10,"successor root did not use measured state and committed signed controls");
        }
 // Shift-only initial guesses must never carry an old feedback policy into
 // a newly measured-root factory. Numerical solver state is fresh as before.
 {
  auto desc=std::make_shared<OCPDescriptor>(OCPRegistry::getDescriptor("stc_landing_noaug_node"));
  desc->warm_start=OCPDescriptor::WarmStart::Shift;const auto create=desc->create;bool saw_controls=false;
  desc->create=[&](const OCPCreateArgs& args){
   require(args.prev_K.empty(),"shift-only initial guess reused feedback gains");
   saw_controls=saw_controls||!args.prev_U.empty();return create(args);
  };
  OcpSolver::Config c;c.ocp_type=desc->name;c.descriptor=desc;c.n_shift=0;OcpSolver solver(c);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(13);x(2)=1.85;x(6)=1.;
  auto first=solver.solve(x);require(first.success&&!first.feedback_gains.empty(),"shift-only contract needs a valid previous feedback policy");
  auto next=solver.solve(x);require(next.success&&saw_controls,"shift-only contract did not exercise a subsequent initial guess");
 }

        for(const char* name:{"landing_rh","stc_landing_noaug"}) {
            auto cfg=hoverConfig();cfg.ocp_type=name;cfg.platform=planner_core::Platform::Mavros;cfg.ocp_dt=.1;cfg.n_replay=1;
            planner_core::PlannerCore core;core.configure(cfg);
            auto input=inputAt(t0);input.target_snapshot.position<<1,0,.2;
            input.target_snapshot.velocity<<.4,0,0;input.target_snapshot.acceleration<<.1,0,0;
            const auto desc=OCPRegistry::getDescriptor(name);auto model=desc.make_model(input.target_snapshot.acceleration);
            auto initial=Eigen::VectorXd::Zero(model.nx).eval();initial(2)=2;initial(6)=1;initial(3)=-.4;
            auto u=model.hover;u(4)=.1;
            auto result=solved(t0,0);result.target_snapshot_acc=input.target_snapshot.acceleration;result.state_trajectory={initial};result.control_trajectory={u,u};
            result.state_trajectory.push_back(model.flow(result.state_trajectory.back(),u,.1,false).x);
            result.state_trajectory.push_back(model.flow(result.state_trajectory.back(),u,.1,false).x);
            require(core.acceptSolvedResult(input,initial,result).solve_accepted,"reference plan acceptance");
            auto now=t0+std::chrono::milliseconds(50);
            auto original=core.sampleReplay(now,input.target_snapshot);
            auto changed=input.target_snapshot;changed.position.x()+=10;changed.velocity.x()=-.4;
            auto replay=core.sampleReplay(now,changed);
            require((original.command.state-replay.command.state).norm()<1e-12,"live estimator changes must trigger replanning, not deform an active DDP reference");
            require((replay.command.state.head(3)-Eigen::Vector3d(1,0,2.2)).norm()<1e-9,"world hover reference must retain its DDP origin");
            require(replay.command.state.segment<3>(3).norm()<1e-9,"world hover reference velocity must remain consistent");
            auto handoff=input;handoff.now=now;handoff.target_snapshot.position.x()=1+.4*.05+.5*.1*.05*.05+.02;
            handoff.target_snapshot.velocity.x()=.4+.1*.05;
            handoff.current_state=planner_core::prepareOcpState(desc,replay.command.state,handoff.target_snapshot);
            planner_core::PlannerCoreStepResult diagnostics;
            planner_core::PlannerCoreTestAccess::HandoffSchedule schedule;
            require(planner_core::PlannerCoreTestAccess::prepareHandoffSchedule(core,handoff,diagnostics,schedule),"target-update handoff scheduling");
            double position_error=0,velocity_error=0;
            auto seed=planner_core::PlannerCoreTestAccess::selectInitialState(core,handoff.current_state,handoff.target_snapshot,schedule,
                &position_error,&velocity_error,diagnostics.x0_source,diagnostics);
            const double ahead=schedule.activation_elapsed-schedule.active_elapsed_now;
            auto next_target=handoff.target_snapshot;
            next_target.position+=next_target.velocity*ahead+.5*next_target.acceleration*ahead*ahead;
            next_target.velocity+=next_target.acceleration*ahead;
            auto world_seed=planner_core::reconstructWorldState(desc,seed,next_target);
            require(position_error<1e-9&&velocity_error<1e-9,"tracking error must compare references in the same target frame");
            require((world_seed.head(3)-replay.command.state.head(3)).norm()<1e-9,"target estimate update must preserve the physical handoff state");

            auto before=core.sampleReplay(now-std::chrono::microseconds(10),changed);
            auto after=core.sampleReplay(now+std::chrono::microseconds(10),changed);
            require(((after.command.state.head(3)-before.command.state.head(3))/2e-5-replay.command.state.segment<3>(3)).norm()<1e-6,
                "world command velocity must differentiate its position");
        }


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
            const auto& desc=OCPRegistry::getDescriptor("stc_landing_noaug");
            TargetSnapshot target;target.valid=true;target.position<<3,-2,1;target.velocity<<.4,-.1,0;
            auto world=state(0);world.head(3)<<4,1,2;world.segment(3,3)<<.2,.3,-.1;
            auto relative=planner_core::prepareOcpState(desc,world,target);
            require((relative.head(3)-(world.head(3)-target.position)).norm()<1e-12,"target-relative position");
            require((relative.segment(6,7)-world.segment(6,7)).norm()<1e-12,"target yaw must not rotate attitude/rates");
            require((planner_core::reconstructWorldState(desc,relative,target)-world).norm()<1e-12,"frame round trip");
            Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
            auto aligned=planner_core::alignWorldState(world,rotation,Eigen::Vector3d(2,3,4),false);
            require((aligned.head(3)-(rotation*world.head(3).eval()+Eigen::Vector3d(2,3,4))).norm()<1e-12,"translated/rotated estimator pose");
            require((aligned.segment(3,3)-rotation*world.segment<3>(3).eval()).norm()<1e-12,"world velocity rotation");
            require((aligned.tail(3)-world.tail(3)).norm()<1e-12,"world rotation preserves body rates");
            auto body=world;body.segment(6,4)<<rotation.w(),rotation.x(),rotation.y(),rotation.z();body.segment(3,3)<<1,0,0;
            auto converted=planner_core::alignWorldState(body,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero(),true);
            require((converted.segment(3,3)-Eigen::Vector3d(0,1,0)).norm()<1e-12,"body velocity normalization");
            require(!planner_core::validateFrameContract("mocap","map").ok,"unaligned source must reject");
            planner_core::PlannerCore core;core.configure(hoverConfig());auto input=inputAt(t0);input.state_frame="mocap";
            require(core.trySolve(input).rejection_reason=="frame_contract","core must reject wrong input frame");
        }
    } catch (const std::exception& e) {
        std::cerr << "PlannerCore tests failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "PlannerCore tests passed\n";
    return 0;
}
