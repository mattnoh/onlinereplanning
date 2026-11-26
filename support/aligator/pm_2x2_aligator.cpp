// Point-mass 2x2 DDP arm on ProxDDP, with the case.txt / JSON interface of the
// paper's point_mass_2x2 driver (support/test/uzun2024_ctcs_mpc).
//
// Problem (paper Sec. V-A): x = (p, v), u = (a[, theta]), exact constant-
// acceleration flow, cost sum theta (1 + 1e-5 |a|^2), x_N = (0,0,1,0,0,0),
// |a| <= 1.5 g, 20 deg input cone, |v| <= V at nodes, theta box (free time).
// Node arm: obstacle and glide residual <= 0 at the nodes. CTCS arm: one
// interval row  integral(sum [(g_i + delta_i)/R]_+^2) / B - 1 <= 0  (ctcs_stc).
//
//   pm_2x2_aligator case.txt out_prefix
// case.txt: "N fixed substeps B tf tfmin tfmax delta_obs delta_glide ctcs vmax logrow",
// X (N+1 rows of 6), U (N rows of 3 or 4), then "key value" ProxDDP settings.
#include "ocp/aligator_ocp.hpp"

#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/state-error.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>

using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;
namespace {
constexpr double G = 9.81;
const Eigen::Vector3d CENTRE(2.85355, 2.14645, 5.5);
constexpr double RADIUS = 2.0, GLIDE_TAN = 1.0;
const double TAN20 = std::tan(20.0 * M_PI / 180.0);
int NU = 4;
double FIXED_DT = 0;

double theta(const V& u) { return FIXED_DT > 0 ? FIXED_DT : u(3); }
Eigen::Vector3d acc(const V& u) { Eigen::Vector3d a = u.head<3>(); a(2) -= G; return a; }

struct Flow : aligator::ExplicitDynamicsModelTpl<double> {
    using Base = aligator::ExplicitDynamicsModelTpl<double>;
    using Data = aligator::ExplicitDynamicsDataTpl<double>;
    explicit Flow(const aligator_ocp::Space& s) : Base(s, NU) {}
    static V next(const V& x, const V& u) {
        const double t = theta(u);
        V o(6);
        o.head<3>() = x.head<3>() + t * x.segment<3>(3) + 0.5 * t * t * acc(u);
        o.tail<3>() = x.segment<3>(3) + t * acc(u);
        return o;
    }
    void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override { d.xnext_ = next(x, u); }
    void dForward(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        const double t = theta(u);
        d.Jx().setIdentity();
        d.Jx().block<3, 3>(0, 3) = t * Eigen::Matrix3d::Identity();
        d.Ju().setZero();
        d.Ju().block<3, 3>(0, 0) = 0.5 * t * t * Eigen::Matrix3d::Identity();
        d.Ju().block<3, 3>(3, 0) = t * Eigen::Matrix3d::Identity();
        if (FIXED_DT <= 0) {
            d.Ju().block<3, 1>(0, 3) = x.segment<3>(3) + t * acc(u);
            d.Ju().block<3, 1>(3, 3) = acc(u);
        }
    }
};

/// sum theta (1 + 1e-5 |a|^2)
struct Cost : aligator::CostAbstractTpl<double> {
    using Base = aligator::CostAbstractTpl<double>;
    using Data = aligator::CostDataAbstractTpl<double>;
    explicit Cost(const aligator_ocp::Space& s) : Base(s, NU) {}
    void evaluate(const ConstVectorRef&, const ConstVectorRef& u, Data& d) const override {
        d.value_ = theta(u) * (1 + 1e-5 * u.head<3>().squaredNorm());
    }
    void computeGradients(const ConstVectorRef&, const ConstVectorRef& u, Data& d) const override {
        d.Lx_.setZero();
        d.Lu_.setZero();
        d.Lu_.head<3>() = 2e-5 * theta(u) * u.head<3>();
        if (FIXED_DT <= 0) d.Lu_(3) = 1 + 1e-5 * u.head<3>().squaredNorm();
    }
    void computeHessians(const ConstVectorRef&, const ConstVectorRef& u, Data& d) const override {
        d.Lxx_.setZero();
        d.Lxu_.setZero();
        d.Luu_.setZero();
        d.Luu_.topLeftCorner<3, 3>() = 2e-5 * theta(u) * Eigen::Matrix3d::Identity();
        if (FIXED_DT <= 0) { d.Luu_.block<3, 1>(0, 3) = 2e-5 * u.head<3>(); d.Luu_.block<1, 3>(3, 0) = 2e-5 * u.head<3>().transpose(); }
    }
};

/// Ordinary rows (<= 0): |a|^2/(1.5g)^2 - 1, cone |a_xy|^2 - tan^2(20) a_z^2 and -a_z,
/// |v|^2/V^2 - 1, theta box (free time), and the node arm's obstacle/glide rows.
struct Rows : aligator::StageFunctionTpl<double> {
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;
    double vmax, lo, hi;
    bool nodal;
    static int count(bool nodal) { return 4 + (FIXED_DT <= 0 ? 2 : 0) + (nodal ? 2 : 0); }
    Rows(double v, double l, double h, bool n) : Base(6, NU, count(n)), vmax(v), lo(l), hi(h), nodal(n) {}
    void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        const double am = 1.5 * G;
        int i = 0;
        d.value_(i++) = u.head<3>().squaredNorm() / (am * am) - 1;
        d.value_(i++) = (u(0) * u(0) + u(1) * u(1) - TAN20 * TAN20 * u(2) * u(2)) / (am * am);
        d.value_(i++) = -u(2) / am;
        d.value_(i++) = x.segment<3>(3).squaredNorm() / (vmax * vmax) - 1;
        if (FIXED_DT <= 0) { d.value_(i++) = (u(3) - hi) / (hi - lo); d.value_(i++) = (lo - u(3)) / (hi - lo); }
        if (nodal) {
            d.value_(i++) = (RADIUS - (x.head<3>() - CENTRE).norm()) / RADIUS;
            d.value_(i++) = (std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12) - GLIDE_TAN * x(2)) / RADIUS;
        }
    }
    void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u, Data& d) const override {
        const double am = 1.5 * G;
        d.Jx_.setZero();
        d.Ju_.setZero();
        int i = 0;
        d.Ju_.block<1, 3>(i++, 0) = 2 * u.head<3>().transpose() / (am * am);
        d.Ju_(i, 0) = 2 * u(0) / (am * am); d.Ju_(i, 1) = 2 * u(1) / (am * am); d.Ju_(i++, 2) = -2 * TAN20 * TAN20 * u(2) / (am * am);
        d.Ju_(i++, 2) = -1 / am;
        d.Jx_.block<1, 3>(i++, 3) = 2 * x.segment<3>(3).transpose() / (vmax * vmax);
        if (FIXED_DT <= 0) { d.Ju_(i++, 3) = 1 / (hi - lo); d.Ju_(i++, 3) = -1 / (hi - lo); }
        if (nodal) {
            const Eigen::Vector3d r = x.head<3>() - CENTRE;
            d.Jx_.block<1, 3>(i++, 0) = -r.transpose() / (std::max(r.norm(), 1e-12) * RADIUS);
            const double q = std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12);
            d.Jx_(i, 0) = x(0) / (q * RADIUS); d.Jx_(i, 1) = x(1) / (q * RADIUS); d.Jx_(i++, 2) = -GLIDE_TAN / RADIUS;
        }
    }
};

ctcs_stc::Spec<double> intervalRow(double B, double d_obs, double d_glide, int substeps) {
    ctcs_stc::Spec<double> spec(ctcs_stc::Form::Density, 1.0);
    const auto always = [](const V&) { return -1.0; };
    const auto zero = [](const V& x) -> V { return V::Zero(x.size()); };
    const auto zu = [](const V&, const V& u) -> V { return V::Zero(u.size()); };
    spec.add("obstacle").triggerWhenNegative(always, zero)
        .consequence([d_obs](const V& x, const V&) { return RADIUS - (x.head<3>() - CENTRE).norm() + d_obs; },
                     [](const V& x, const V&) -> V {
                         V g = V::Zero(6);
                         const Eigen::Vector3d r = x.head<3>() - CENTRE;
                         g.head<3>() = -r / std::max(r.norm(), 1e-12);
                         return g;
                     }, zu)
        .scale(1.0 / RADIUS).weight(1.0 / B);
    spec.add("glide_slope").triggerWhenNegative(always, zero)
        .consequence([d_glide](const V& x, const V&) { return std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12) - GLIDE_TAN * x(2) + d_glide; },
                     [](const V& x, const V&) -> V {
                         V g = V::Zero(6);
                         const double q = std::sqrt(x(0) * x(0) + x(1) * x(1) + 1e-12);
                         g(0) = x(0) / q; g(1) = x(1) / q; g(2) = -GLIDE_TAN;
                         return g;
                     }, zu)
        .scale(1.0 / RADIUS).weight(1.0 / B);
    ctcs_stc::IntervalDynamics<double> dyn;
    dyn.xdot = [](const V& x, const V& u, double) { V f(6); f.head<3>() = x.segment<3>(3); f.tail<3>() = acc(u); return f; };
    dyn.fx = [](const V&, const V&, double) { M a = M::Zero(6, 6); a.block<3, 3>(0, 3).setIdentity(); return a; };
    dyn.fu = [](const V&, const V&, double) { M b = M::Zero(6, NU); b.block<3, 3>(3, 0).setIdentity(); return b; };
    dyn.idx_theta = FIXED_DT > 0 ? -1 : 3;
    dyn.dt = FIXED_DT;
    dyn.substeps = substeps;
    spec.setIntervalDynamics(dyn);
    return spec;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 3) { std::cerr << "case.txt output_prefix\n"; return 2; }
        std::ifstream in(argv[1]);
        int N, fixed, steps, ctcs, logrow;
        double B, tf, tfmin, tfmax, mo, mg, vmax;
        if (!(in >> N >> fixed >> steps >> B >> tf >> tfmin >> tfmax >> mo >> mg >> ctcs >> vmax >> logrow) || N < 1) return 2;
        FIXED_DT = fixed ? tf / N : 0.;
        NU = fixed ? 3 : 4;
        std::vector<V> X(N + 1, V(6)), U(N, V(NU));
        for (auto& x : X) for (int j = 0; j < 6; ++j) if (!(in >> x(j))) return 2;
        for (auto& u : U) for (int j = 0; j < NU; ++j) if (!(in >> u(j))) return 2;
        std::map<std::string, double> settings;
        std::string key;
        double value;
        while (in >> key >> value) settings[key] = value;
        const auto get = [&](const char* k, double d) { auto it = settings.find(k); return it == settings.end() ? d : it->second; };
        aligator_ocp::SolverSettings s;
        s.tol = get("tol", 1e-8);
        s.mu_init = get("mu_init", 1e-2);
        s.max_iters = static_cast<std::size_t>(get("max_iters", 2000));
        s.max_al_iters = static_cast<std::size_t>(get("max_al_iters", 100));
        s.exact_hessian = get("exact_hessian", 0) != 0;
        s.nonlinear_rollout = get("nonlinear_rollout", 0) != 0;
        s.step_acceptance = static_cast<int>(get("step_acceptance", 0));
        s.reg_init = get("reg_init", 1e-9);
        s.reg_min = get("reg_min", 1e-10);

        aligator_ocp::Space space(6);
        const Flow flow(space);
        const Cost cost(space);
        const auto row = intervalRow(B, mo, mg, steps);
        std::vector<xyz::polymorphic<aligator_ocp::StageModel>> stages;
        for (int k = 0; k < N; ++k) {
            aligator_ocp::StageModel stage(cost, flow);
            stage.addConstraint(Rows(vmax, tfmin / N, tfmax / N, !ctcs), aligator_ocp::Orthant());
            if (ctcs) stage.addConstraint(ctcs_stc::StageFunction(6, NU, row, get("row_curvature", 0) != 0 ? ctcs_stc::Curvature::GaussNewton : ctcs_stc::Curvature::None), aligator_ocp::Orthant());
            stages.emplace_back(stage);
        }
        V xf(6);
        xf << 0, 0, 1, 0, 0, 0;
        struct Zero : aligator::CostAbstractTpl<double> {
            using Base = aligator::CostAbstractTpl<double>;
            explicit Zero(const aligator_ocp::Space& sp) : Base(sp, 0) {}
            void evaluate(const ConstVectorRef&, const ConstVectorRef&, CostData& d) const override { d.value_ = 0; }
            void computeGradients(const ConstVectorRef&, const ConstVectorRef&, CostData& d) const override { d.Lx_.setZero(); d.Lu_.setZero(); }
            void computeHessians(const ConstVectorRef&, const ConstVectorRef&, CostData& d) const override { d.Lxx_.setZero(); d.Lxu_.setZero(); d.Luu_.setZero(); }
        };
        aligator_ocp::Problem problem(X[0], stages, Zero(space));
        problem.addTerminalConstraint(aligator::StateErrorResidualTpl<double>(space, 0, xf), aligator::EqualityConstraintTpl<double>());

        auto solver = aligator_ocp::makeSolver(s);
        solver->setup(problem);
        bool flag = false;
        std::string termination = "converged";
        const auto start = std::chrono::steady_clock::now();
        try { flag = solver->run(problem, X, U); if (!flag) termination = "not_converged"; }
        catch (const std::exception& e) { termination = std::string("threw: ") + e.what(); }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const auto& r = solver->results_;

        std::cout << std::setprecision(17) << "{\"termination\":\"" << (flag ? std::string("converged") : termination.substr(termination.size() > 90 ? termination.size() - 90 : 0))
                  << "\",\"iterations\":" << r.num_iters << ",\"optimality\":" << std::max(r.prim_infeas, r.dual_infeas)
                  << ",\"solve_seconds\":" << seconds << ",\"settings\":{";
        bool comma = false;
        for (const auto& [k, v] : settings) { if (comma) std::cout << ','; comma = true; std::cout << '"' << k << "\":" << v; }
        std::cout << "},\"normalized_interval_rows\":[";
        for (int k = 0; k < N && ctcs; ++k) { if (k) std::cout << ','; std::cout << row.c(r.xs[k], r.us[k])(0); }
        std::cout << "],\"X\":[";
        for (int k = 0; k <= N; ++k) { if (k) std::cout << ','; std::cout << '['; for (int j = 0; j < 6; ++j) { if (j) std::cout << ','; std::cout << r.xs[k](j); } std::cout << ']'; }
        std::cout << "],\"U\":[";
        for (int k = 0; k < N; ++k) { if (k) std::cout << ','; std::cout << '['; for (int j = 0; j < NU; ++j) { if (j) std::cout << ','; std::cout << r.us[k](j); } std::cout << ']'; }
        std::cout << "]}" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
