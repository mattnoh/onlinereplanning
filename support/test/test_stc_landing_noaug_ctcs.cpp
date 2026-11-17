// Compare the shared interface against the deployed independent RK4 integral.
#include "ocp/ocp_stc_landing_noaug.hpp"
#include <iostream>
#include <random>
#include <stdexcept>

int main() {
    using namespace StcLandingNoAugOCP;
    try {
        auto pred = std::make_shared<ConstantAccelPredictor>();
        pred->a = Eigen::Vector3d(0.13, -0.09, 0.02);
        auto dyn = std::make_shared<Quad6DOFVarTimeRelativePred<double>>(pred);
        dyn->setMass(MASS); dyn->setGravity(GRAVITY); dyn->setJb(J_B);
        auto row = makeCtcsConstraint<double>(dyn);
        if (row->getForm() != StcForm::Density || row->getDimC() != 1 ||
            row->numImplications() != (useTriggeredLandingStc() ? 8 : 2))
            throw std::runtime_error("wrong CTCS form or implication count");
        std::mt19937 gen(20261013);
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        double value_error=0, gradient_error=0;
        for (int sample=0; sample<160; ++sample) {
            Eigen::VectorXd x=Eigen::VectorXd::Zero(NX_SS), u=Eigen::VectorXd::Zero(NU_SS);
            x(0)=2.0*unit(gen); x(1)=2.0*unit(gen); x(2)=0.9+0.8*unit(gen);
            if(sample%4==0) { x(0)=0.1; x(1)=0.1; x(2)=0.4; }
            for (int i=3;i<NX;++i) x(i)=0.3*unit(gen);
            x(6)=1.0; x.segment<4>(6).normalize(); x(IDX_DT)=2.0+unit(gen);
            u(0)=HOVER_THRUST*(1.0+0.2*unit(gen));
            for(int i=1;i<NU;++i) u(i)=0.002*unit(gen);
            u(IDX_THETA)=0.1+0.04*unit(gen);
            const double density=evalCtcsIntegrand<double>(x,u)/CTCS_Y_SCALE;
            const double integral=dyn->intervalIntegral(x,u);
            const double row_value=row->c(x,u)(0)+CTCS_STEP_EPS/CTCS_Y_SCALE;
            value_error=std::max(value_error,std::abs(row->density(x,u)-density)/(1.0+std::abs(density)));
            value_error=std::max(value_error,std::abs(row_value-integral)/(1.0+std::abs(integral)));
            const auto jx=row->cx(x,u); const auto ju=row->cu(x,u);
            for(int i=0;i<NX_SS+NU_SS;++i) {
                // The physical profile can scale the integral by 1e7 relative
                // to the historical Y=1e6 profile. A 1e-7 two-point difference
                // then loses the small partials in cancellation. Use an
                // independent five-point stencil at a larger physical step.
                const double value=i<NX_SS?x(i):u(i-NX_SS);
                const double h=1e-5*(1.+std::abs(value));
                auto xp=x, xm=x, up=u, um=u;
                auto xpp=x,xmm=x,upp=u,umm=u;
                if(i<NX_SS) {xp(i)+=h; xm(i)-=h;} else {up(i-NX_SS)+=h; um(i-NX_SS)-=h;}
                if(i<NX_SS) {xpp(i)+=2*h;xmm(i)-=2*h;} else {upp(i-NX_SS)+=2*h;umm(i-NX_SS)-=2*h;}
                const double fd=(-dyn->intervalIntegral(xpp,upp)+8.*dyn->intervalIntegral(xp,up)
                    -8.*dyn->intervalIntegral(xm,um)+dyn->intervalIntegral(xmm,umm))/(12*h);
                const double analytic=i<NX_SS ? jx(0,i) : ju(0,i-NX_SS);
                gradient_error=std::max(gradient_error,std::abs(fd-analytic)/(1.0+std::abs(fd)));
            }
        }
        std::cout << "CTCS parity: value_error=" << value_error
                  << " gradient_error=" << gradient_error << '\n';
        if(value_error>1e-12 || gradient_error>2e-5)
            throw std::runtime_error("CTCS value/Jacobian parity failed");
        // Physical auditing uses the original implication, never the tightened
        // trigger or the historical grader's excluded switching band.
        auto physical=makeCtcsConstraint<double>(dyn,CTCS_STEP_EPS/CTCS_Y_SCALE,IDX_THETA,0.,1,false);
        const auto check=[&](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
        const auto index=[&](const char* name){for(int j=0;j<row->numImplications();++j)if(row->implication(j).name==name)return j;throw std::runtime_error("missing original implication");};
        Eigen::VectorXd x=Eigen::VectorXd::Zero(NX_SS),u=Eigen::VectorXd::Zero(NU_SS);x(6)=1.;u(0)=HOVER_THRUST;u(IDX_THETA)=TH0;
        if(useTriggeredLandingStc()&&stageTriggerMode()=="radius"&&CTCS_MARGINS.trigger>0.){
            const int stage=index("staging"),los=index("los"),land=index("landing_speed");
            x(0)=STAGE_RADIUS-.5*CTCS_MARGINS.trigger;x(2)=Z_STAGE+.05;
            check(row->triggered(stage,x)&&!physical->triggered(stage,x),"staging preactivation has the wrong polarity");
            x(0)=STAGE_RADIUS-CTCS_MARGINS.trigger-.01;
            check(!row->triggered(stage,x),"staging became always active");
            x(2)=LOS_ALT_TRIG+.5*CTCS_MARGINS.trigger;
            check(row->triggered(los,x)&&!physical->triggered(los,x),"LOS audit uses the tightened trigger");
            x(0)=.1;x(2)=ALT_TRIG+.5*CTCS_MARGINS.trigger;
            check(row->triggered(land,x)&&!physical->triggered(land,x),"landing audit uses the tightened trigger");
        }
        x.setZero();x(6)=.9;x(7)=.1;x(8)=.2;x(9)=.3;
        RelTermCost<double> attitude_only(0.,0.,0.,0.,1.,0.,0.);
        check(std::abs(attitude_only.p(x)-.025)<1e-14,"SH terminal attitude objective changed");
        auto gradient=attitude_only.px(x);auto hessian=attitude_only.pxx(x);
        check(gradient(6)==0.&&gradient(9)==0.&&gradient(7)==.1&&gradient(8)==.2&&hessian(7,7)==1.&&hessian(8,8)==1.,"SH terminal attitude derivatives changed");
        std::cout << "PASS: original physical trigger audit and SH terminal-cost contract\n";
        std::cout << "PASS: 160 noaug density/interval/Jacobian samples\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
