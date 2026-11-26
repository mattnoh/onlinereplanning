// Single-horizon landing on the 100 common initial conditions (STC_PRESET=n80).
// One cold ProxDDP solve per sample; writes the grader's node/dense CSVs
// (20 RK4 samples per interval) and one results row per sample.
//
//   landing_ic100 SAMPLES_CSV OUT_DIR interval|node|penalty [first last] [w_pen]
#include "ocp/ocp_stc_landing_noaug.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace Q = StcLandingNoAugOCP;
using V = Eigen::VectorXd;

static V rk4(const aligator_ocp::quad::Body& b, const V& x, const V& up, double h) {
    const Eigen::Vector3d a = Eigen::Vector3d::Zero();
    const V k1 = aligator_ocp::quad::xdot(b, x, up, a), k2 = aligator_ocp::quad::xdot(b, x + 0.5 * h * k1, up, a),
            k3 = aligator_ocp::quad::xdot(b, x + 0.5 * h * k2, up, a), k4 = aligator_ocp::quad::xdot(b, x + h * k3, up, a);
    V xn = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
    xn.segment<4>(6).normalize();
    return xn;
}

int main(int argc, char** argv) {
    if (argc < 4) { std::fprintf(stderr, "usage: landing_ic100 SAMPLES_CSV OUT_DIR interval|node|penalty [first last] [w_pen]\n"); return 1; }
    const std::string arm = argv[3];
    const auto enforcement = arm == "node" ? Q::Enforcement::Node : arm == "penalty" ? Q::Enforcement::Penalty : Q::Enforcement::Interval;
    const int first = argc > 5 ? std::atoi(argv[4]) : 0, last = argc > 5 ? std::atoi(argv[5]) : 99;
    const double w_pen = argc > 6 ? std::atof(argv[6]) : 0.0;
    const std::filesystem::path out(argv[2]);
    std::filesystem::create_directories(out / "runs");

    std::ifstream in(argv[1]);
    std::string line;
    std::getline(in, line);  // sample,regime,draw,r0x,r0y,r0z,v0x,v0y,v0z
    std::ofstream res(out / "results.csv", std::ios::app);
    if (res.tellp() == 0) res << "sample,arm,flag,iterations,solver_only_s,rows,interval,defect,T_star,artifact_dir\n";
    res << std::setprecision(12);

    const auto body = Q::body();
    const auto physical = Q::landingRules(false, ctcs_stc::Form::Density, 1, Eigen::Vector3d::Zero());
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> c;
        while (std::getline(ss, cell, ',')) c.push_back(cell);
        const int sample = std::stoi(c[0]);
        if (sample < first || sample > last) continue;
        V x0 = V::Zero(13);
        for (int j = 0; j < 6; ++j) x0(j) = std::stod(c[3 + j]);
        x0(6) = 1;

        auto ocp = Q::create(x0, {}, {}, {}, nullptr, 0, enforcement, w_pen);
        auto solver = aligator_ocp::makeSolver(Q::P.solver);
        solver->setup(*ocp->problem);
        bool flag = false;
        const auto t0 = std::chrono::steady_clock::now();
        try { flag = solver->run(*ocp->problem, ocp->xs, ocp->us); } catch (const std::exception& e) { std::fprintf(stderr, "sample %d: %s\n", sample, e.what()); }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const auto& r = solver->results_;
        const std::vector<V> xs(r.xs.begin(), r.xs.end()), us(r.us.begin(), r.us.end());
        const auto audit = aligator_ocp::auditPrimal(*ocp, xs, us);

        char name[32];
        std::snprintf(name, sizeof name, "sample_%03d", sample);
        const auto dir = out / "runs" / name;
        std::filesystem::create_directories(dir);
        std::ofstream fn(dir / "nodes.csv"), fd(dir / "dense.csv");
        const char* hdr = "node,t_acc,rx,ry,rz,rvx,rvy,rvz,qw,qx,qy,qz,wx,wy,wz,fz_B,Mx,My,Mz,Theta,ctcs_integrand\n";
        fn << hdr << std::setprecision(12);
        fd << hdr << std::setprecision(12);
        const auto row = [&](std::ofstream& f, int k, double t, const V& x, const V& us_k) {
            const V up = ocp->toPhysical(us_k);
            f << k << ',' << t;
            for (int j = 0; j < 13; ++j) f << ',' << x(j);
            for (int j = 0; j < 5; ++j) f << ',' << up(j);
            V xx = V::Zero(Q::NX_SS);
            xx.head(13) = x;
            f << ',' << physical.density(xx, us_k) * Q::P.ctcs_step_eps << '\n';
        };
        constexpr int SUB = 20;
        double t = 0;
        const int N = static_cast<int>(us.size());
        for (int k = 0; k <= N; ++k) {
            const V& uk = us[std::min(k, N - 1)];
            row(fn, k, t, xs[k].head(13), uk);
            if (k == N) { row(fd, k, t, xs[k].head(13), uk); break; }
            const V up = ocp->toPhysical(uk);
            V x = xs[k].head(13);
            const double h = up(Q::IDX_THETA) / SUB;
            for (int i = 0; i < SUB; ++i) { row(fd, k, t + i * h, x, uk); x = rk4(body, x, up.head(4), h); }
            t += up(Q::IDX_THETA);
        }
        res << sample << ',' << arm << ',' << flag << ',' << r.num_iters << ',' << secs << ',' << audit.constraints << ','
            << audit.interval << ',' << audit.dynamics << ',' << t << ',' << "runs/" << name << '\n';
        res.flush();
        std::printf("sample %3d flag=%d iters=%4zu %.2f s rows %.1e interval %.1e |r_N|=%.3f\n", sample, flag,
                    static_cast<std::size_t>(r.num_iters), secs, audit.constraints, audit.interval, xs.back().head<3>().norm());
        std::fflush(stdout);
    }
    return 0;
}
