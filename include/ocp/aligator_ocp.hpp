/// @file aligator_ocp.hpp
/// @brief What every OCP hands the planner core: an Aligator problem, its seed,
/// its discrete map and the ProxDDP settings, plus the shared quadrotor model.
#pragma once

#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/function-abstract.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include "ctcs_stc/stage_function.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace aligator_ocp {

using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;
using Space = aligator::context::VectorSpace;
using Problem = aligator::context::TrajOptProblem;
using StageModel = aligator::context::StageModel;
using Orthant = aligator::NegativeOrthantTpl<double>;

/// ProxDDP settings an OCP asks for. `accept_tol` gates the returned primal:
/// every stage row (all are `<= 0`) and the dynamics defect must be within it.
struct SolverSettings {
    double tol = 1e-4;
    double mu_init = 0.1;
    std::size_t max_iters = 1000;
    std::size_t max_al_iters = 100;
    bool exact_hessian = false;
    bool nonlinear_rollout = false;  // the nonlinear rollout NaNs on the landing OCP
    double accept_tol = 1e-3;
    // Interval-integral rows (sum w I / eps - 1) may end this fraction over
    // their budget: ALIPDDP's 1e-6 on I - eps at eps = 2.67e-6 allowed 0.375.
    double interval_accept_tol = 0.375;
    bool verbose = false;
    int step_acceptance = 0;  // 0 nonmonotone line search, 1 Armijo, 2 filter
    double reg_init = 1e-9, reg_min = 1e-10;  // primal regularisation (ProxDDP defaults)
    int ls_mode = 0;             // 0 primal merit, 1 primal-dual merit
    int multiplier_update = 0;   // 0 Newton, 1 primal, 2 primal-dual
    double dual_weight = 1.0, mu_update_factor = 0.01;
};

/// One solve's problem. Controls in `us` and in the solver are in SOLVER units;
/// `toPhysical` / `toSolver` convert (identity when the OCP does not scale).
struct Ocp {
    std::shared_ptr<Problem> problem;
    std::vector<V> xs, us;
    std::function<V(const V& x, const V& u_solver)> step;
    std::function<V(const V&)> toPhysical = [](const V& u) { return u; };
    std::function<V(const V&)> toSolver = [](const V& u) { return u; };
};

/// Largest positive row value over the ordinary stage constraints and over the
/// interval-integral rows, and the largest dynamics defect (including x0).
struct PrimalAudit {
    double constraints = 0, interval = 0, dynamics = 0;
    bool feasible(const SolverSettings& s) const {
        return std::isfinite(constraints) && std::isfinite(interval) && std::isfinite(dynamics) &&
               constraints <= s.accept_tol && dynamics <= s.accept_tol && interval <= s.interval_accept_tol;
    }
};

inline PrimalAudit auditPrimal(const Ocp& ocp, const std::vector<V>& xs, const std::vector<V>& us) {
    const double bad = std::numeric_limits<double>::infinity();
    const auto& stages = ocp.problem->stages_;
    if (xs.size() != stages.size() + 1 || us.size() != stages.size()) return {bad, bad};
    for (const auto& v : xs) if (!v.allFinite()) return {bad, bad};
    for (const auto& v : us) if (!v.allFinite()) return {bad, bad};
    PrimalAudit out;
    out.dynamics = (xs.front() - ocp.problem->getInitState()).lpNorm<Eigen::Infinity>();
    for (std::size_t k = 0; k < stages.size(); ++k) {
        const V next = ocp.step(xs[k], us[k]);
        if (!next.allFinite()) return {bad, bad};
        out.dynamics = std::max(out.dynamics, (next - xs[k + 1]).lpNorm<Eigen::Infinity>());
        const auto& stack = stages[k]->constraints_;
        for (std::size_t j = 0; j < stack.size(); ++j) {
            auto data = stack.funcs[j]->createData();
            stack.funcs[j]->evaluate(xs[k], us[k], *data);
            if (!data->value_.allFinite()) return {bad, bad};
            if (!data->value_.size()) continue;
            const bool row = dynamic_cast<const ctcs_stc::StageFunction*>(&*stack.funcs[j]) != nullptr;
            double& slot = row ? out.interval : out.constraints;
            slot = std::max(slot, data->value_.maxCoeff());
        }
    }
    return out;
}

// ── Quadrotor model shared by the OCPs ───────────────────────────────────────
//   x = [r (3), v (3), q = (qw,qx,qy,qz) (4), omega_B (3), t]       in R^14
//   u = [f_z [N], M / Ixx (3), theta [s]]                             in R^5
//   r' = v,  v' = C(q) e_z f_z / m + g - a_ext,  q' = 0.5 Omega(w) q,
//   w' = J^-1 (M - w x J w)   with J in units of Ixx.
namespace quad {
constexpr int NX = 14, NU = 5, NXP = 13, NUP = 4, IDX_T = 13, IDX_THETA = 4;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct Body {
    double mass = 0.027;
    Mat3 J = Mat3::Identity(), Jinv = Mat3::Identity();
    Vec3 gravity = Vec3(0, 0, -9.81);
    Body() = default;
    Body(double m, double ixx, double iyy, double izz) : mass(m) {
        J = Vec3(1.0, iyy / ixx, izz / ixx).asDiagonal();
        Jinv = J.inverse();
    }
};

inline Mat3 rotation(const Eigen::Vector4d& q) {
    const double w = q(0), x = q(1), y = q(2), z = q(3);
    Mat3 C;
    C << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
         2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
         2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
    return C;
}
inline Eigen::Matrix4d omegaMatrix(const Vec3& w) {
    Eigen::Matrix4d O;
    O << 0, -w(0), -w(1), -w(2), w(0), 0, w(2), -w(1), w(1), -w(2), 0, w(0), w(2), w(1), -w(0), 0;
    return O;
}
inline Mat3 skew(const Vec3& v) {
    Mat3 S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}

/// Rates of the 13 physical states. `up` = physical [f_z, M/Ixx].
inline V xdot(const Body& b, const V& x, const V& up, const Vec3& a_ext) {
    const Vec3 w = x.segment<3>(10);
    V d(NXP);
    d.segment<3>(0) = x.segment<3>(3);
    d.segment<3>(3) = rotation(x.segment<4>(6)).col(2) * up(0) / b.mass + b.gravity - a_ext;
    d.segment<4>(6) = 0.5 * omegaMatrix(w) * x.segment<4>(6);
    d.segment<3>(10) = b.Jinv * (up.segment<3>(1) - w.cross(b.J * w));
    return d;
}
inline M A(const Body& b, const V& x, const V& up) {
    const Eigen::Vector4d q = x.segment<4>(6);
    const Vec3 w = x.segment<3>(10);
    const double s = up(0) / b.mass;
    M a = M::Zero(NXP, NXP);
    a.block<3, 3>(0, 3).setIdentity();
    a.block<3, 1>(3, 6) = s * Vec3(2 * q(2), -2 * q(1), 0);
    a.block<3, 1>(3, 7) = s * Vec3(2 * q(3), -2 * q(0), -4 * q(1));
    a.block<3, 1>(3, 8) = s * Vec3(2 * q(0), 2 * q(3), -4 * q(2));
    a.block<3, 1>(3, 9) = s * Vec3(2 * q(1), 2 * q(2), 0);
    a.block<4, 4>(6, 6) = 0.5 * omegaMatrix(w);
    Eigen::Matrix<double, 4, 3> dq;
    dq << -q(1), -q(2), -q(3), q(0), -q(3), q(2), q(3), q(0), -q(1), -q(2), q(1), q(0);
    a.block<4, 3>(6, 10) = 0.5 * dq;
    a.block<3, 3>(10, 10) = b.Jinv * (skew(b.J * w) - skew(w) * b.J);
    return a;
}
inline M B(const Body& b, const V& x) {
    M m = M::Zero(NXP, NUP);
    m.block<3, 1>(3, 0) = rotation(x.segment<4>(6)).col(2) / b.mass;
    m.block<3, 3>(10, 1) = b.Jinv;
    return m;
}

/// Solver controls u~ map to physical ones as u = center + scale .* u~ (theta unscaled).
struct ControlScaling {
    V center = V::Zero(NU), scale = V::Ones(NU);
    V physical(const V& u) const { return center + scale.cwiseProduct(u); }
    V solver(const V& u) const { return (u - center).cwiseQuotient(scale); }
};

/// One RK4 step of length theta = u(4) (free time) or `fixed_dt`, quaternion
/// renormalised after the step, t += h. Jacobians by RK4 forward sensitivity
/// through the renormalisation and the control scaling.
struct Dynamics : aligator::ExplicitDynamicsModelTpl<double> {
    using Base = aligator::ExplicitDynamicsModelTpl<double>;
    using Data = aligator::ExplicitDynamicsDataTpl<double>;
    Body body;
    ControlScaling scaling;
    Vec3 a_ext = Vec3::Zero();
    double fixed_dt = 0.0;  // > 0: fixed step, theta ignored

    Dynamics(const Space& space, Body b, ControlScaling s, Vec3 a, double dt = 0.0)
        : Base(space, NU), body(std::move(b)), scaling(std::move(s)), a_ext(a), fixed_dt(dt) {}

    double stepLength(const ConstVectorRef& u) const { return fixed_dt > 0 ? fixed_dt : u(IDX_THETA); }

    V next(const V& x, const V& u) const {
        const V up = scaling.physical(u).head(NUP), xp = x.head(NXP);
        const double h = fixed_dt > 0 ? fixed_dt : u(IDX_THETA);
        const V k1 = xdot(body, xp, up, a_ext), k2 = xdot(body, xp + 0.5 * h * k1, up, a_ext),
                k3 = xdot(body, xp + 0.5 * h * k2, up, a_ext), k4 = xdot(body, xp + h * k3, up, a_ext);
        V xn(NX);
        xn.head(NXP) = xp + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
        xn.segment<4>(6).normalize();
        xn(IDX_T) = x(IDX_T) + h;
        return xn;
    }

    void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        d.xnext_ = next(x, u);
    }

    void dForward(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        const V up = scaling.physical(u).head(NUP), xp = x.head(NXP);
        const double h = stepLength(u);
        const M I = M::Identity(NXP, NXP);
        const V k1 = xdot(body, xp, up, a_ext), x2 = xp + 0.5 * h * k1, k2 = xdot(body, x2, up, a_ext),
                x3 = xp + 0.5 * h * k2, k3 = xdot(body, x3, up, a_ext), x4 = xp + h * k3,
                k4 = xdot(body, x4, up, a_ext);
        const M A1 = A(body, xp, up), A2 = A(body, x2, up), A3 = A(body, x3, up), A4 = A(body, x4, up);
        const M B1 = B(body, xp), B2 = B(body, x2), B3 = B(body, x3), B4 = B(body, x4);
        const M d1x = A1, d2x = A2 * (I + 0.5 * h * d1x), d3x = A3 * (I + 0.5 * h * d2x),
                d4x = A4 * (I + h * d3x);
        const M d1u = B1, d2u = A2 * (0.5 * h * d1u) + B2, d3u = A3 * (0.5 * h * d2u) + B3,
                d4u = A4 * (h * d3u) + B4;
        const V d2T = A2 * (0.5 * k1), d3T = A3 * (0.5 * k2 + 0.5 * h * d2T), d4T = A4 * (k3 + h * d3T);
        const M Jx = I + h / 6.0 * (d1x + 2 * d2x + 2 * d3x + d4x);
        const M Ju = h / 6.0 * (d1u + 2 * d2u + 2 * d3u + d4u);
        const V JT = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0 + h / 6.0 * (2 * d2T + 2 * d3T + d4T);
        const Eigen::Vector4d qr = (xp + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)).segment<4>(6);
        const double n = qr.norm();
        M N = I;
        N.block<4, 4>(6, 6) = (Eigen::Matrix4d::Identity() - qr * qr.transpose() / (n * n)) / n;
        d.Jx().setZero();
        d.Jx().topLeftCorner(NXP, NXP) = N * Jx;
        d.Jx()(IDX_T, IDX_T) = 1.0;
        d.Ju().setZero();
        d.Ju().topLeftCorner(NXP, NUP) = N * Ju * scaling.scale.head(NUP).asDiagonal();
        if (fixed_dt <= 0) {
            d.Ju().col(IDX_THETA).head(NXP) = N * JT * scaling.scale(IDX_THETA);
            d.Ju()(IDX_T, IDX_THETA) = scaling.scale(IDX_THETA);
        }
    }
};
}  // namespace quad

/// A fresh ProxDDP instance configured from `s` (single-threaded).
inline std::unique_ptr<aligator::context::SolverProxDDP> makeSolver(const SolverSettings& s) {
    auto solver = std::make_unique<aligator::context::SolverProxDDP>(s.tol, s.mu_init, s.max_iters,
        s.verbose ? aligator::VERBOSE : aligator::QUIET,
        s.step_acceptance == 1 ? aligator::StepAcceptanceStrategy::LINESEARCH_ARMIJO
            : s.step_acceptance == 2 ? aligator::StepAcceptanceStrategy::FILTER
            : aligator::StepAcceptanceStrategy::LINESEARCH_NONMONOTONE,
        s.exact_hessian ? aligator::HessianApprox::EXACT : aligator::HessianApprox::GAUSS_NEWTON);
    solver->max_al_iters = s.max_al_iters;
    solver->rollout_type_ = s.nonlinear_rollout ? aligator::RolloutType::NONLINEAR : aligator::RolloutType::LINEAR;
    solver->ls_mode = s.ls_mode ? aligator::LinesearchMode::PRIMAL_DUAL : aligator::LinesearchMode::PRIMAL;
    solver->multiplier_update_mode = s.multiplier_update == 1 ? aligator::MultiplierUpdateMode::PRIMAL
        : s.multiplier_update == 2 ? aligator::MultiplierUpdateMode::PRIMAL_DUAL : aligator::MultiplierUpdateMode::NEWTON;
    solver->dual_weight = s.dual_weight;
    solver->bcl_params.mu_update_factor = s.mu_update_factor;
    solver->reg_init = s.reg_init;
    solver->reg_min = s.reg_min;
    solver->preg_ = s.reg_init;
    solver->setNumThreads(1);
    return solver;
}

}  // namespace aligator_ocp
