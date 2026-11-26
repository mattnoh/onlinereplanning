#include "trajectory_replayer.hpp"
#include "planner_core/types.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using Clock = std::chrono::steady_clock;

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<Eigen::VectorXd> makeFixedPlan(double offset, int nodes = 4) {
    std::vector<Eigen::VectorXd> xs;
    xs.reserve(nodes);
    for (int k = 0; k < nodes; ++k) {
        Eigen::VectorXd x(3);
        x << offset + static_cast<double>(k), 0.0, 0.0;
        xs.push_back(x);
    }
    return xs;
}

std::vector<Eigen::VectorXd> makeVariablePlan() {
    std::vector<Eigen::VectorXd> xs;
    const double times[] = {0.0, 0.05, 0.20};
    for (int k = 0; k < 3; ++k) {
        Eigen::VectorXd x(4);
        x << static_cast<double>(k), 0.0, 0.0, times[k];
        xs.push_back(x);
    }
    return xs;
}

std::vector<Eigen::VectorXd> makeControls(double offset, int count = 3) {
    std::vector<Eigen::VectorXd> us;
    us.reserve(count);
    for (int k = 0; k < count; ++k) {
        Eigen::VectorXd u(4);
        u << offset + static_cast<double>(k), 0.0, 0.0, 0.0;
        us.push_back(u);
    }
    return us;
}

void testFirstPlanActivationStartsAtNodeZero() {
    TrajectoryReplayer replayer;
    const auto base = Clock::now();
    const auto finish = base + std::chrono::milliseconds(300);
    replayer.updatePlan(makeFixedPlan(0.0), makeControls(0.0), 300.0, 0,
                        finish, finish, 0.1, 2, false, 3);

    const auto sample = replayer.sample(finish + std::chrono::milliseconds(10), 0.1);
    require(sample.has_plan, "first activation: expected active plan");
    require(sample.active_solve_num == 0, "first activation: wrong solve number");
    require(std::abs(sample.x_cmd(0) - 0.1) < 1e-6,
            "first activation: sampled too far into the horizon");
}

void testPendingPlanWaitsForActivation() {
    TrajectoryReplayer replayer;
    const auto base = Clock::now();
    replayer.updatePlan(makeFixedPlan(0.0), makeControls(0.0), 10.0, 0,
                        base, base, 0.1, 2, false, 3);
    replayer.updatePlan(makeFixedPlan(100.0), makeControls(10.0), 10.0, 1,
                        base + std::chrono::milliseconds(200),
                        base + std::chrono::milliseconds(200),
                        0.1, 2, false, 3);

    auto sample = replayer.sample(base + std::chrono::milliseconds(150), 0.1);
    require(sample.active_solve_num == 0, "pending activation: swapped too early");

    sample = replayer.sample(base + std::chrono::milliseconds(210), 0.1);
    require(sample.active_solve_num == 1, "pending activation: did not swap at handoff");
    require(sample.x_cmd(0) > 100.0 && sample.x_cmd(0) < 100.2,
            "pending activation: new plan not sampled from activation time");
}

void testLatePendingPlanSamplesFromActivationOrigin() {
    TrajectoryReplayer replayer;
    const auto base = Clock::now();
    replayer.updatePlan(makeFixedPlan(0.0), makeControls(0.0), 10.0, 0,
                        base, base, 0.1, 2, false, 3);
    replayer.updatePlan(makeFixedPlan(100.0), makeControls(10.0), 80.0, 1,
                        base + std::chrono::milliseconds(200),
                        base + std::chrono::milliseconds(200),
                        0.1, 2, false, 3);

    const auto sample = replayer.sample(base + std::chrono::milliseconds(250), 0.1);
    require(sample.active_solve_num == 1, "late pending activation: did not swap");
    require(std::abs(sample.x_cmd(0) - 100.5) < 1e-6,
            "late pending activation: new plan was not sampled from activation origin");
}

void testVariableDtUsesStateDtSlot() {
    TrajectoryReplayer replayer;
    const auto base = Clock::now();
    replayer.updatePlan(makeVariablePlan(), makeControls(0.0, 2), 10.0, 0,
                        base, base, 0.1, 1, true, 3);

    const auto sample = replayer.sample(base + std::chrono::milliseconds(125), 0.1);
    require(std::abs(sample.x_cmd(0) - 1.5) < 1e-6,
            "variable DT: did not interpolate using x[state_dim]");
}

void testJumpDiagnostics() {
    TrajectoryReplayer replayer;
    const auto base = Clock::now();
    replayer.updatePlan(makeFixedPlan(0.0), makeControls(0.0), 10.0, 0,
                        base, base, 0.1, 1, false, 3);
    replayer.updatePlan(makeFixedPlan(20.0), makeControls(50.0), 40.0, 1,
                        base + std::chrono::milliseconds(100),
                        base + std::chrono::milliseconds(100),
                        0.1, 1, false, 3);

    (void)replayer.sample(base + std::chrono::milliseconds(110), 0.1);
    const auto diag = replayer.consumeLastHandoffDiagnostic();
    require(diag.has_value(), "jump diagnostics: no diagnostic emitted");
    require(diag->previous_solve_num == 0 && diag->new_solve_num == 1,
            "jump diagnostics: wrong solve transition");
    require(diag->state_jump_norm > 10.0, "jump diagnostics: state jump not measured");
    require(diag->control_jump_norm > 10.0, "jump diagnostics: control jump not measured");
}

void testDynamicsConsistentSampling() {
    for(const char* name:{"stc_landing_noaug"}) {
        auto descriptor=OCPRegistry::getDescriptor(name);
        auto model=descriptor.make_model(Eigen::Vector3d(.05,-.03,0));
        auto x=Eigen::VectorXd::Zero(model.nx).eval();x(2)=2;x(6)=1;x(10)=.01;
        auto u=model.hover;u(0)=descriptor.default_mass_kg*(9.81+.3);u(4)=.1;
        std::vector<Eigen::VectorXd> states={x};
        states.push_back(model.flow(states.back(),u,.1,false).x);
        states.push_back(model.flow(states.back(),u,.1,false).x);
        TrajectoryReplayer replay;auto origin=Clock::now();
        replay.updatePlan(states,{u,u},0,1,origin,origin,.1,1,true,13,{},
            [model](const auto& state,const auto& input,double duration){return model.flow(state,input,duration,false).x;});
        auto sample=replay.sample(origin+std::chrono::milliseconds(25),.1);
        auto expected=model.flow(x,u,.025,false).x;
        require((sample.x_cmd-expected.head(13)).norm()<1e-12,"dense reference must use DDP host dynamics");
        auto linear=.75*states[0]+.25*states[1];
        require((sample.x_cmd-linear.head(13)).norm()>1e-6,"acceleration test must distinguish linear interpolation");
        TrajectoryReplayer::ReplaySample before,after;
        replay.sampleActiveAtElapsed(.025-1e-5,.1,&before);replay.sampleActiveAtElapsed(.025+1e-5,.1,&after);
        require(((after.x_cmd.head(3)-before.x_cmd.head(3))/(2e-5)-sample.x_cmd.segment<3>(3)).norm()<1e-6,
            "sampled reference velocity must be the derivative of reference position");
        require((sample.u_cmd-u.head(4)).norm()<1e-12,"dense sampling must not modify DDP control");
    }
}

}  // namespace

int main() {
    try {
        testFirstPlanActivationStartsAtNodeZero();
        testPendingPlanWaitsForActivation();
        testLatePendingPlanSamplesFromActivationOrigin();
        testVariableDtUsesStateDtSlot();
        testJumpDiagnostics();
        testDynamicsConsistentSampling();
    } catch (const std::exception& e) {
        std::cerr << "TrajectoryReplayer test failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "TrajectoryReplayer tests passed\n";
    return 0;
}
