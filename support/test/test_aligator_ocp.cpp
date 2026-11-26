// Finite-difference checks of the landing OCP's analytic derivatives (dynamics,
// ordinary rows, interval rule rows in both tightenings, stage cost), at states
// inactive, active and across every trigger, then one single-horizon solve.
#include "ocp/ocp_stc_landing_noaug.hpp"

#include <chrono>
#include <cstdio>
#include <functional>

namespace Q = StcLandingNoAugOCP;
using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;

static int failures = 0;

static M fd(const std::function<V(const V&)>& f, const V& z, double h = 1e-6) {
    const V f0 = f(z);
    M J(f0.size(), z.size());
    for (int i = 0; i < z.size(); ++i) {
        V a = z, b = z;
        a(i) += h; b(i) -= h;
        J.col(i) = (f(a) - f(b)) / (2 * h);
    }
    return J;
}

static void check(const char* name, const M& analytic, const M& numeric, double rtol = 1e-4) {
    const double err = (analytic - numeric).cwiseAbs().maxCoeff();
    const double scale = std::max(1.0, numeric.cwiseAbs().maxCoeff());
    const bool ok = err <= rtol * scale;
    failures += !ok;
    std::printf("[%s] %-44s max|err| %.3e (scale %.2e)\n", ok ? "pass" : "FAIL", name, err, scale);
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    aligator_ocp::Space space(Q::NX_SS);
    const Eigen::Vector3d a_t(0.05, -0.03, 0);
    const aligator_ocp::quad::Dynamics dyn(space, Q::body(), Q::scaling(), a_t);
    const Q::Bounds bounds;
    Q::StageCost cost(space);
    const auto s = Q::scaling();

    struct Case { const char* name; V x; };
    const auto state = [](double rx, double ry, double rz, double vx, double vy, double vz, double tilt, double w) {
        V x = V::Zero(Q::NX_SS);
        x << rx, ry, rz, vx, vy, vz, std::cos(tilt / 2), std::sin(tilt / 2) * 0.8, std::sin(tilt / 2) * 0.6, 0, w, -w, 0.5 * w, 0.3;
        return x;
    };
    const double z_land = Q::P.alt_trig + Q::P.margins.trigger, z_los = Q::P.los_alt_trig + Q::P.margins.trigger;
    const std::vector<Case> cases = {
        {"far, high (stage active)", state(2.5, 1.0, 1.6, -0.8, -0.2, -0.1, 0.12, 0.2)},
        {"inside LOS band", state(0.4, 0.3, z_los - 0.2, -0.3, 0.1, -0.4, 0.08, 0.1)},
        {"just above landing trigger", state(0.2, 0.1, z_land + 0.01, -0.1, 0.0, -0.9, 0.05, 0.05)},
        {"below landing trigger", state(0.15, -0.1, z_land - 0.3, 0.2, 0.1, -1.1, 0.1, 0.18)},
    };
    V u = s.solver((V(Q::NU_SS) << Q::HOVER_THRUST * 1.1, 0.02, -0.01, 0.005, 0.1).finished());

    for (const auto& c : cases) {
        std::printf("-- %s\n", c.name);
        aligator::ExplicitDynamicsDataTpl<double> dd(dyn);
        dyn.dForward(c.x, u, dd);
        check("dynamics d/dx", dd.Jx(), fd([&](const V& x) { return dyn.next(x, u); }, c.x));
        check("dynamics d/du", dd.Ju(), fd([&](const V& uu) { return dyn.next(c.x, uu); }, u));

        auto bd = bounds.createData();
        bounds.computeJacobians(c.x, u, *bd);
        const auto bval = [&](const V& x, const V& uu) { auto d = bounds.createData(); bounds.evaluate(x, uu, *d); return V(d->value_); };
        check("bounds d/dx", bd->Jx_, fd([&](const V& x) { return bval(x, u); }, c.x));
        check("bounds d/du", bd->Ju_, fd([&](const V& uu) { return bval(c.x, uu); }, u));

        for (bool tight : {true, false}) {
            const auto rules = Q::landingRules(tight, ctcs_stc::Form::Density, tight ? 1 : Q::CTCS_PHYSICAL_SUBSTEPS, a_t);
            const auto val = [&](const V& x, const V& uu) { return rules.c(x, uu); };
            const std::string tag = tight ? "interval row (tightened)" : "interval row (physical)";
            std::printf("   %s value %.4e\n", tag.c_str(), val(c.x, u)(0));
            check((tag + " d/dx").c_str(), rules.cx(c.x, u), fd([&](const V& x) { return val(x, u); }, c.x, 1e-7));
            check((tag + " d/du").c_str(), rules.cu(c.x, u), fd([&](const V& uu) { return val(c.x, uu); }, u, 1e-7));
        }

        auto cd = cost.createData();
        cost.computeGradients(c.x, u, *cd);
        const auto cval = [&](const V& uu) { auto d = cost.createData(); cost.evaluate(c.x, uu, *d); return V::Constant(1, d->value_).eval(); };
        check("stage cost d/du", cd->Lu_.transpose(), fd(cval, u));
    }

    // One single-horizon solve (n80 preset) from the stc_landing example start.
    V x0 = V::Zero(13);
    x0 << 2.4, 2.0, 2.0, 0, -2, 0, 1, 0, 0, 0, 0, 0, 0;
    auto ocp = Q::create(x0);
    auto solver = aligator_ocp::makeSolver(Q::P.solver);
    solver->setup(*ocp->problem);
    const auto t0 = std::chrono::steady_clock::now();
    bool flag = false;
    try { flag = solver->run(*ocp->problem, ocp->xs, ocp->us); }
    catch (const std::exception& e) { std::printf("single horizon: solver threw: %s\n", e.what()); }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const auto& r = solver->results_;
    const std::vector<V> xs(r.xs.begin(), r.xs.end()), us(r.us.begin(), r.us.end());
    const auto audit = aligator_ocp::auditPrimal(*ocp, xs, us);
    double T = 0;
    for (const auto& uk : us) T += ocp->toPhysical(uk)(Q::IDX_THETA);
    std::printf("single horizon: flag=%d iters=%zu %.0f ms  rows<=%.2e interval<=%.2e  defect %.2e  T=%.2f s  |r_N|=%.3f |v_N|=%.3f\n",
                flag, static_cast<std::size_t>(r.num_iters), ms, audit.constraints, audit.interval, audit.dynamics, T,
                xs.back().head<3>().norm(), xs.back().segment<3>(3).norm());

    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
