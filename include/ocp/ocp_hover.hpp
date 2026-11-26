/// @file ocp_hover.hpp
/// @brief World-frame hover / go-to ("hover"), ProxDDP. Used for SITL staging.
#pragma once

#include "ocp/aligator_ocp.hpp"
#include "planner_core/types.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <memory>

namespace HoverOCP {

using aligator_ocp::V;
using aligator_ocp::M;
namespace quad = aligator_ocp::quad;

inline constexpr int HORIZON = 100, DEFAULT_N_REPLAY = 4, NX = 13, NU = 4;
inline constexpr double DT = 0.05;
inline constexpr double MASS = 0.0282, DEFAULT_MASS_KG = MASS;
inline constexpr double IXX = 1.66e-5, IYY = 1.66e-5, IZZ = 2.92e-5;
inline constexpr double FMAX = 0.42, FMIN = 0.08;
inline constexpr double W_POS = 15, W_VEL = 20, W_ATT = 2.0, W_RATE = 5.0, W_THRUST = 0.1, W_MOMENT = 0.1;
inline constexpr double T_POS = 600, T_VEL = 100, T_ATT = 200, T_RATE = 100;
inline constexpr double COST_SCALE = 1e-2;

inline aligator_ocp::SolverSettings solverSettings() {
    aligator_ocp::SolverSettings s;
    s.tol = 1e-3;
    s.mu_init = 0.1;
    s.max_iters = 200;
    s.accept_tol = 1e-3;
    return s;
}

/// x_{k+1}: one RK4 step of DT, quaternion renormalised. 13 states, 4 controls.
struct Dynamics : aligator::ExplicitDynamicsModelTpl<double> {
    using Base = aligator::ExplicitDynamicsModelTpl<double>;
    using Data = aligator::ExplicitDynamicsDataTpl<double>;
    quad::Body body{MASS, IXX, IYY, IZZ};
    explicit Dynamics(const aligator_ocp::Space& s) : Base(s, NU) {}
    V next(const V& x, const V& u) const {
        const Eigen::Vector3d a = Eigen::Vector3d::Zero();
        const V k1 = quad::xdot(body, x, u, a), k2 = quad::xdot(body, x + 0.5 * DT * k1, u, a),
                k3 = quad::xdot(body, x + 0.5 * DT * k2, u, a), k4 = quad::xdot(body, x + DT * k3, u, a);
        V xn = x + DT / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
        xn.segment<4>(6).normalize();
        return xn;
    }
    void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override { d.xnext_ = next(x, u); }
    void dForward(const ConstVectorRef& xx, const ConstVectorRef& uu, Data& d) const override {
        const V x = xx, u = uu;
        const Eigen::Vector3d a = Eigen::Vector3d::Zero();
        const M I = M::Identity(NX, NX);
        const double h = DT;
        const V k1 = quad::xdot(body, x, u, a), x2 = x + 0.5 * h * k1, k2 = quad::xdot(body, x2, u, a),
                x3 = x + 0.5 * h * k2, k3 = quad::xdot(body, x3, u, a), x4 = x + h * k3, k4 = quad::xdot(body, x4, u, a);
        const M A1 = quad::A(body, x, u), A2 = quad::A(body, x2, u), A3 = quad::A(body, x3, u), A4 = quad::A(body, x4, u);
        const M B1 = quad::B(body, x), B2 = quad::B(body, x2), B3 = quad::B(body, x3), B4 = quad::B(body, x4);
        const M d2x = A2 * (I + 0.5 * h * A1), d3x = A3 * (I + 0.5 * h * d2x), d4x = A4 * (I + h * d3x);
        const M d2u = A2 * (0.5 * h * B1) + B2, d3u = A3 * (0.5 * h * d2u) + B3, d4u = A4 * (h * d3u) + B4;
        const Eigen::Vector4d qr = (x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)).segment<4>(6);
        const double n = qr.norm();
        M N = I;
        N.block<4, 4>(6, 6) = (Eigen::Matrix4d::Identity() - qr * qr.transpose() / (n * n)) / n;
        d.Jx() = N * (I + h / 6.0 * (A1 + 2 * d2x + 2 * d3x + d4x));
        d.Ju() = N * (h / 6.0 * (B1 + 2 * d2u + 2 * d3u + d4u));
    }
};

/// Quadratic tracking of a world target state; attitude error |q_v|^2 + (1 - q_w)^2.
struct Cost : aligator::CostAbstractTpl<double> {
    using Base = aligator::CostAbstractTpl<double>;
    using Data = aligator::CostDataAbstractTpl<double>;
    V wx = V::Zero(NX), ref = V::Zero(NX), wu = V::Zero(NU);
    Cost(const aligator_ocp::Space& s, const V& target, bool terminal) : Base(s, terminal ? 0 : NU) {
        const double p = terminal ? T_POS : W_POS, v = terminal ? T_VEL : W_VEL, q = terminal ? T_ATT : W_ATT,
                     w = terminal ? T_RATE : W_RATE;
        wx << p, p, p, v, v, v, q, q, q, q, w, w, w;
        wx *= COST_SCALE;
        ref.head<6>() = target.head<6>();
        ref(6) = 1.0;
        if (!terminal) wu << W_THRUST, W_MOMENT, W_MOMENT, W_MOMENT;
        wu *= COST_SCALE;
    }
    void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        d.value_ = (wx.array() * (x - ref).array().square()).sum();
        if (u.size()) d.value_ += (wu.array() * u.array().square()).sum();
    }
    void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        d.Lx_ = 2 * wx.cwiseProduct(x - ref);
        if (u.size()) d.Lu_ = 2 * wu.cwiseProduct(u);
    }
    void computeHessians(const ConstVectorRef&, const ConstVectorRef& u, Data& d) const override {
        d.Lxx_ = (2 * wx).asDiagonal();
        d.Lxu_.setZero();
        if (u.size()) d.Luu_ = (2 * wu).asDiagonal();
    }
};

/// FMIN <= f_z <= fmax, as rows scaled by the range.
struct Thrust : aligator::StageFunctionTpl<double> {
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;
    double fmax;
    explicit Thrust(double f) : Base(NX, NU, 2), fmax(f) {}
    void evaluate(const ConstVectorRef&, const ConstVectorRef& u, Data& d) const override {
        d.value_ << (FMIN - u(0)) / (fmax - FMIN), (u(0) - fmax) / (fmax - FMIN);
    }
    void computeJacobians(const ConstVectorRef&, const ConstVectorRef&, Data& d) const override {
        d.Jx_.setZero();
        d.Ju_.setZero();
        d.Ju_(0, 0) = -1 / (fmax - FMIN);
        d.Ju_(1, 0) = 1 / (fmax - FMIN);
    }
};

inline std::shared_ptr<aligator_ocp::Ocp> create(const V& current_state, const V& terminal_state, double fmax = FMAX) {
    aligator_ocp::Space space(NX);
    const Dynamics dyn(space);
    const Cost stage_cost(space, terminal_state, false), terminal(space, terminal_state, true);
    std::vector<xyz::polymorphic<aligator_ocp::StageModel>> stages;
    for (int k = 0; k < HORIZON; ++k) {
        aligator_ocp::StageModel stage(stage_cost, dyn);
        stage.addConstraint(Thrust(fmax), aligator_ocp::Orthant());
        stages.emplace_back(stage);
    }
    const V x0 = current_state.head(NX);
    auto ocp = std::make_shared<aligator_ocp::Ocp>();
    ocp->problem = std::make_shared<aligator_ocp::Problem>(x0, stages, terminal);
    ocp->step = [dyn](const V& x, const V& u) { return dyn.next(x, u); };
    V u0(NU);
    u0 << MASS * 9.81, 0, 0, 0;
    ocp->us.assign(HORIZON, u0);
    ocp->xs.assign(HORIZON + 1, x0);
    for (int k = 0; k < HORIZON; ++k) ocp->xs[k + 1] = dyn.next(ocp->xs[k], u0);
    return ocp;
}

inline OCPDescriptor descriptor() {
    OCPDescriptor d;
    d.name = "hover";
    d.dt = DT;
    d.default_n_replay = DEFAULT_N_REPLAY;
    d.default_mass_kg = DEFAULT_MASS_KG;
    d.warm_start = OCPDescriptor::WarmStart::Shift;
    d.command_mode = OCPDescriptor::CommandMode::CmdFullState;
    d.world_acceleration = [](const V& x, const V& u) {
        if (x.size() < 13 || u.size() < 1 || !x.allFinite() || !u.allFinite())
            throw std::invalid_argument("invalid DDP acceleration input");
        Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
        if (q.norm() < 1e-8) throw std::invalid_argument("invalid DDP attitude");
        return Eigen::Vector3d(q.normalized() * Eigen::Vector3d(0, 0, u(0) / MASS) + Eigen::Vector3d(0, 0, -9.81));
    };
    d.coordinates = OCPDescriptor::Coordinates::World;
    d.state_dim = 13;
    d.control_dim = 4;
    d.log_state_headers = OCPLoggerDefaults::getStateHeaders13D();
    d.state_names = OCPLoggerDefaults::getStateNames13D();
    d.control_names = OCPLoggerDefaults::getControlNames4D();
    d.extract_actual_state_row = OCPLoggerDefaults::getActualStateRow13D;
    d.make_hover_state = OCPLoggerDefaults::makeHoverState13D;
    d.solver_settings = solverSettings;
    d.create = [](const OCPCreateArgs& a) { return create(a.current_state, a.terminal_state); };
    return d;
}

}  // namespace HoverOCP
