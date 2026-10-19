// Reuse the actual selected revision's CTCS row; this entrypoint never solves.
#define main historical_optimizer_main
#include POINT_MASS_DIAGNOSTIC_SOURCE
#undef main
#include <type_traits>
#include <utility>

template<class Row, class = void> struct HasCurvature : std::false_type {};
template<class Row> struct HasCurvature<Row, std::void_t<decltype(std::declval<Row>().lagrangianHessian(
    std::declval<Vec>(),std::declval<Vec>(),std::declval<Vec>()))>> : std::true_type {};
template<class Row> Mat unitCurvature(Row& row,const Vec& x,const Vec& u) {
    if constexpr (HasCurvature<Row>::value) return row.lagrangianHessian(x,u,Vec::Ones(1));
    else return Mat::Zero(x.size()+u.size(),x.size()+u.size());
}

int main() {
    std::cout << std::setprecision(17);
    // Stream protocol: x[6], acceleration[3], dt, eps, margins[2],
    // row_norm, substeps, fixed_time, glide_tan, obstacle[3], radius.
    Vec x(6), u(4);
    double dt, eps, norm, radius; int steps, fixed;
    Eigen::Vector3d obs;
    while (std::cin >> x(0)) {
        for (int j=1; j<6; ++j) std::cin >> x(j);
        for (int j=0; j<3; ++j) std::cin >> u(j);
        if (!(std::cin >> dt >> eps >> g_tight_obs >> g_tight_glide
              >> norm >> steps >> fixed >> g_glide_tan
              >> obs(0) >> obs(1) >> obs(2) >> radius)) return 2;
        if (!(dt>0 && eps>0 && steps>0 && radius>0)) return 2;
        u(3)=dt; g_fixed_dt=fixed ? dt : 0.; g_row_norm=norm;
        const int nu=fixed ? 3 : 4;
        Vec controls=u.head(nu);
        auto row=ctcsRow(6,nu,0,steps,obs,radius,eps);
        const double value=row->c(x,controls)(0);
        const Mat Jx=row->cx(x,controls), Ju=row->cu(x,controls);
        const double cached=row->c(x,controls)(0);
        std::cout << "{\"row\":" << value << ",\"cached_row\":" << cached << ",\"jacobian\":[";
        for(int j=0; j<6+nu; ++j) {
            if(j) std::cout << ',';
            std::cout << (j<6 ? Jx(0,j) : Ju(0,j-6));
        }
        std::cout << "],\"finite_difference\":[";
        for(int j=0; j<6+nu; ++j) {
            Vec xp=x,xm=x,up=controls,um=controls;
            double h=1e-6;
            if(j<6) {xp(j)+=h; xm(j)-=h;} else {up(j-6)+=h; um(j-6)-=h;}
            if(j) std::cout << ',';
            std::cout << (row->c(xp,up)(0)-row->c(xm,um)(0))/(2*h);
        }
        const Mat H=unitCurvature(*row,x,controls);
        std::cout << "],\"has_constraint_curvature\":"
                  << (HasCurvature<StcStageConstraint<Scalar>>::value ? "true" : "false")
                  << ",\"unit_multiplier_hessian\":[";
        for(int i=0;i<H.rows();++i) {
            if(i) std::cout << ',';
            std::cout << '[';
            for(int j=0;j<H.cols();++j) {if(j)std::cout << ',';std::cout << H(i,j);}
            std::cout << ']';
        }
        std::cout << "]}" << std::endl;
    }
}
