/// @file test_perception_feedback_ablation.cpp
/// @brief Regression checks for perception_track derivatives, convergence, and
///        lost-target reacquisition.

#include "ocp/ocp_perception_track.hpp"
#include "planner_core/quadrotor_mpc.hpp"
#include "planner_core/ocp_registry.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <any>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace PT = PerceptionTrackOCP;

namespace {

void require(bool ok, const std::string& what) {
    if (!ok) {
        std::cerr << "[FAIL] " << what << "\n";
        std::exit(1);
    }
}

void checkCostModels() {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(PT::NX);
    x(0) = -0.5; x(2) = 1.4; x(6) = 1.0;
    x.segment<3>(3) << 0.4, -0.2, 0.1;
    x.segment<3>(PT::IDX_PH) << 3.0, 0.4, 1.0;
    x.segment<3>(PT::IDX_VH) << 0.5, 0.1, 0.0;
    x(PT::IDX_S) = 0.7;
    x(PT::IDX_ZG) = -0.2;
    x(PT::IDX_XG) = 0.3;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(PT::NU);
    u(PT::IDX_FZ) = PT::hoverThrust() + 0.03;
    u.segment<3>(1) << 0.01, -0.02, 0.015;
    u(PT::IDX_THETA) = PT::dtInit() + 0.06;
    u(PT::IDX_GPR) = 0.1;
    u(PT::IDX_GYR) = -0.08;

    constexpr double h = 1e-6;
    PT::PtStageCost<double> stage;
    const Eigen::VectorXd ga = stage.qu(x, u);
    Eigen::VectorXd gn = Eigen::VectorXd::Zero(PT::NU);
    for (int i = 0; i < PT::NU; ++i) {
        Eigen::VectorXd up = u, um = u;
        up(i) += h; um(i) -= h;
        gn(i) = (stage.q(x, up) - stage.q(x, um))/(2.0*h);
    }
    require((ga - gn).cwiseAbs().maxCoeff() < 2e-4,
            "stage q/qu must agree by finite differences");
    const Eigen::MatrixXd Hu = stage.quu(x, u);
    require((Hu - Hu.transpose()).norm() < 1e-8,
            "stage Gauss-Newton Hessian must be symmetric");
    require(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(Hu)
                .eigenvalues().minCoeff() > -1e-7,
            "stage Gauss-Newton Hessian must be positive semidefinite");

    PT::PtTermCost<double> terminal;
    const Eigen::VectorXd gta = terminal.px(x);
    Eigen::VectorXd gtn = Eigen::VectorXd::Zero(PT::NX);
    for (int i = 0; i < PT::NX; ++i) {
        Eigen::VectorXd xp = x, xm = x;
        xp(i) += h; xm(i) -= h;
        gtn(i) = (terminal.p(xp) - terminal.p(xm))/(2.0*h);
    }
    require((gta - gtn).cwiseAbs().maxCoeff() < 3e-3,
            "terminal p/px must agree by finite differences");
    const Eigen::MatrixXd Ht = terminal.pxx(x);
    require((Ht - Ht.transpose()).norm() < 1e-8,
            "terminal Gauss-Newton Hessian must be symmetric");
    require(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(Ht)
                .eigenvalues().minCoeff() > -1e-7,
            "terminal Gauss-Newton Hessian must be positive semidefinite");
    require(Ht.block<3,3>(3,3).trace() > 1.0,
            "terminal Hessian must retain relative-velocity curvature");
}

void checkObservationAge() {
    const auto& d = OCPRegistry::getDescriptor("perception_track");
    PlannerConfig cfg;
    TargetSnapshot t;
    t.valid = false;
    t.position = Eigen::Vector3d(1.0, 2.0, 0.5);
    t.velocity = Eigen::Vector3d(0.4, -0.2, 0.0);
    t.acceleration = Eigen::Vector3d(0.1, 0.0, 0.0);
    t.odom_stamp_sec = 8.0;
    t.accel_stamp_sec = 8.0;
    const auto ex = std::any_cast<PT::PerceptionTrackExtra>(
        d.prepare_extra(cfg, 10.0, t));
    require(std::abs(ex.staleness_init - 2.0) < 1e-12,
            "staleness must equal time since the last camera observation");
    const Eigen::Vector3d expected =
        t.position + 2.0*t.velocity + 0.5*4.0*t.acceleration;
    require((ex.predictor->predictPos(0.0) - expected).norm() < 1e-12,
            "lost-target estimate must coast to the current solve time");
}

struct SolveCheck {
    double error = 0.0;
    double solve_ms = 0.0;
    double initial_mode = 0.0;
    double terminal_mode = 0.0;
    double terminal_staleness = 0.0;
    double max_speed = 0.0;
};

SolveCheck solveCase(double yaw, double staleness) {
    QuadrotorMPC::Config cfg;
    cfg.ocp_type = "perception_track";
    cfg.n_shift = 1;
    QuadrotorMPC mpc(cfg);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(PT::NX);
    x(0) = -1.5; x(2) = 1.5; x(3) = 0.45;
    x(6) = std::cos(0.5*yaw);
    x(9) = std::sin(0.5*yaw);
    x.segment<3>(PT::IDX_PH) = Eigen::Vector3d(1.0, 0.0, 1.0);
    x.segment<3>(PT::IDX_VH) = Eigen::Vector3d(0.45, 0.6, 0.0);
    x(PT::IDX_S) = staleness;

    PT::PerceptionTrackExtra ex;
    ex.predictor = std::make_shared<PT::ConstantAccelPredictor>();
    ex.predictor->setState(x.segment<3>(PT::IDX_PH),
                           x.segment<3>(PT::IDX_VH),
                           Eigen::Vector3d::Zero());
    ex.staleness_init = staleness;

    const double initial_mode = PT::visTerms(x).rho_mode;
    const auto result = mpc.solve(x, Eigen::Vector3d::Zero(), std::any(ex), 0.0);
    require(result.success, "perception solve must return a trajectory");
    require(PT::finiteTrajectory(result.state_trajectory, result.control_trajectory),
            "perception solve must remain finite");
    require(PT::plausibleTrajectory(result.state_trajectory, result.control_trajectory),
            "perception solve must remain physically plausible");
    require(result.constraint_error < 1.0,
            "perception solve must pass PlannerCore's acceptance threshold");
    require(result.solve_time_ms < 1000.0,
            "perception solve must fit the deployed one-second acceptance budget");

    SolveCheck out;
    out.error = result.constraint_error;
    out.solve_ms = result.solve_time_ms;
    out.initial_mode = initial_mode;
    const auto& xf = result.state_trajectory.back();
    out.terminal_mode = PT::visTerms(xf).rho_mode;
    out.terminal_staleness = xf(PT::IDX_S);
    for (const auto& xx : result.state_trajectory)
        out.max_speed = std::max(out.max_speed, xx.segment<3>(3).norm());
    return out;
}

}  // namespace

int main() {
    unsetenv("PT_ENABLE_GIMBAL");
    PT::setPillars({});

    const auto& d = OCPRegistry::getDescriptor("perception_track");
    require(d.name == "perception_track", "perception_track must be registered");
    require(d.state_dim == PT::IDX_DT,
            "state_dim must equal the variable-time state index");
    require(d.control_dim == PT::NU,
            "descriptor must expose all perception OCP controls");
    require(d.variable_dt, "perception OCP must declare variable dt");

    checkCostModels();
    checkObservationAge();

    const SolveCheck visible = solveCase(0.0, 0.0);
    require(visible.initial_mode > 0.0 && visible.terminal_mode > 0.0,
            "visible target must remain acquired");
    require(visible.max_speed < PT::vFast() + 0.5,
            "visible-target plan must respect the fast speed policy");

    const SolveCheck lost = solveCase(M_PI/2.0, 1.0);
    require(lost.initial_mode < 0.0, "lost-target case must start outside the FOV");
    require(lost.terminal_mode > 0.0,
            "lost-target plan must reacquire the target by the terminal node");
    require(lost.terminal_staleness < 0.25,
            "reacquisition must reset predicted staleness");
    require(lost.max_speed < PT::vFast() + 0.5,
            "reacquisition plan must remain speed bounded");

    std::cout << "perception_track regression passed\n"
              << "  visible: error=" << visible.error
              << " solve_ms=" << visible.solve_ms
              << " terminal_mode=" << visible.terminal_mode << "\n"
              << "  lost:    error=" << lost.error
              << " solve_ms=" << lost.solve_ms
              << " terminal_mode=" << lost.terminal_mode
              << " terminal_s=" << lost.terminal_staleness << "\n";
    return 0;
}
