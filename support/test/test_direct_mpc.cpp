#include "ocp/ocp_stc_landing_mpc.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "ocp/ocp_landing_mpc.hpp"
#include "controller_internal.hpp"
#include "planner_core/adapters.hpp"
#include <iostream>
#include <thread>
#include <stdexcept>
static void check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
int main(){try{
    namespace O=StcLandingMpcOCP;
    auto pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();pred->a<<.1,-.1,0;
    auto dyn=std::make_shared<O::Dynamics>(pred);
    Eigen::VectorXd x=Eigen::VectorXd::Zero(O::NX);x<<.4,-.3,1.3,.2,.1,-.1,.99,.02,-.03,.01,.05,-.04,.02,.4,9.81,0,0,0;
    x.segment<4>(6).normalize();auto u=O::hover();u.tail(3)<<.08,-.06,.02;
    auto model=OCPRegistry::getDescriptor("stc_landing_mpc").make_model(pred->a);
    auto direct=dyn->flow(x,u,O::DT,true);auto through_interface=model.flow(x,u,O::DT,true);
    check((direct.x-through_interface.x).norm()<1e-12 && (direct.a-through_interface.a).norm()<1e-12 && (direct.b-through_interface.b).norm()<1e-12,"MPC model interface must preserve dynamics and Jacobians");
    // The OCP defines signed physical sets; their derivatives must remain
    // informative inside the set, unlike a positive-part violation integral.
    for(const char* type:{"landing_mpc","stc_landing_mpc"}) {
        auto tracking_model=OCPRegistry::getDescriptor(type).make_model(Eigen::Vector3d::Zero());
        Eigen::VectorXd initial=Eigen::VectorXd::Zero(tracking_model.nx);
        initial(2)=.5;initial(6)=1;initial.tail<4>()=tracking_model.hover;
        auto probe=initial;probe(0)=.02;probe(1)=-.01;probe(7)=.001;probe(8)=-.002;
        probe.segment<4>(6).normalize();probe.segment<3>(10)<<.001,.002,-.003;
        const auto limits=tracking_model.constraints(probe);
        double derivative_error=0;
        for(int j=0;j<tracking_model.nx;++j) {
            auto plus=probe,minus=probe;plus(j)+=1e-7;minus(j)-=1e-7;
            Eigen::VectorXd fd=(tracking_model.constraints(plus).value-tracking_model.constraints(minus).value)/(2e-7);
            derivative_error=std::max(derivative_error,(fd-limits.jacobian.col(j)).norm());
        }
        check(derivative_error<1e-4,"tracking physical set derivatives");
        auto previous=Eigen::Vector4d(tracking_model.hover);
        double worst_residual=0,max_ms=0;
        // A large correction that the old unconstrained feedback would apply.
        // Optimize the coupled limits, not individual command clipping.
        for(int tick=0;tick<100;++tick) {
            Eigen::Vector4d desired=tracking_model.hover;
            desired(1)=tick<50?.2:-.2;
            auto result=planner_core::constrainedTracking(tracking_model,initial,desired,previous,.01);
            max_ms=std::max(max_ms,result.elapsed_ms);
            for(int sub=0;sub<10;++sub) {
                initial=tracking_model.flow(initial,result.control,.001,false).x;
                worst_residual=std::max(worst_residual,tracking_model.constraints(initial).value.maxCoeff());
            }
            previous=result.control;
        }
        std::cout<<type<<" constrained tracking: residual="<<worst_residual<<" max_ms="<<max_ms<<" derivatives="<<derivative_error<<'\n';
        check(worst_residual<1e-4,"sampled constrained tracking must preserve physical sets");
        auto impossible=Eigen::VectorXd(initial);
        impossible.setZero();impossible(2)=.005;impossible(5)=-.7;impossible(6)=1;
        impossible.tail<4>()=tracking_model.hover;
        auto recovery=planner_core::constrainedTracking(tracking_model,impossible,tracking_model.hover,tracking_model.hover,.01);
        check(recovery.predicted_residual>1e-3,"nonviable initial state must not be reported feasible");

    }
    const auto a=dyn->fx(x,u), b=dyn->fu(x,u);double maxerr=0;double h=1e-6;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;
        if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        Eigen::VectorXd fd=(dyn->f(xp,up)-dyn->f(xm,um))/(2*h);
        Eigen::VectorXd analytic=j<O::NX? a.col(j).eval():b.col(j-O::NX).eval();maxerr=std::max(maxerr,(fd-analytic).norm());
    }
    check(maxerr<1e-5,"dynamics Jacobians");
    x(2)=.45; x.segment<3>(3)<<.6,.4,-.5;
    auto row=StcLandingNoAugOCP::makeCtcsConstraint<double>(dyn,StcLandingNoAugOCP::CTCS_STEP_EPS/StcLandingNoAugOCP::CTCS_Y_SCALE,-1,O::DT,O::substeps());
    auto cx=row->cx(x,u),cu=row->cu(x,u);
    double cerror=0;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        double fd=(row->c(xp,up)(0)-row->c(xm,um)(0))/(2*h);double analytic=j<O::NX?cx(0,j):cu(0,j-O::NX);cerror=std::max(cerror,std::abs(fd-analytic)/(1+std::abs(fd)));}
    check(cerror<1e-5,"CTCS Jacobians");
    Eigen::VectorXd odom=Eigen::VectorXd::Zero(13);odom(3)=1;odom(6)=std::sqrt(.5);odom(9)=std::sqrt(.5);odom(10)=.2;
    auto world=planner_core::bodyOdometryToWorld(odom);check(std::abs(world(4)-1)<1e-12&&std::abs(world(3))<1e-12&&world(10)==.2,"odometry frame conversion");
    check(std::abs(planner_core::normalizedThrust(9.81,.7)-.7)<1e-12&&planner_core::normalizedThrust(30,.7)==1,"thrust normalization");
    O::SpeedBound speed;StcLandingNoAugOCP::GeneralSpeedCon<double> legacy_speed;
    check((speed.c(x,u)-legacy_speed.c(x.head(14),u)).norm()<1e-12,"speed cone convention");
    OCPCreateArgs args;args.current_state=x;args.target_accel=pred->a;auto problem=O::create(args);
    check(problem->getInitialControl(0).size()==4,"MPC control layout");
    check(std::abs(dyn->f(x,u)(13)-x(13)-O::DT)<1e-12,"fixed time grid");
    check(O::descriptor().command_mode==OCPDescriptor::CommandMode::CmdBodyRate,"direct command type");
    planner_core::PlannerCoreConfig cfg;cfg.ocp_type="stc_landing_mpc";cfg.platform=planner_core::Platform::Mavros;cfg.enable_terminal_freeze=false;
    planner_core::DirectMpcController controller;controller.configure(cfg);
    planner_core::PlannerCoreInput input;input.current_state=Eigen::VectorXd::Zero(13);input.current_state<<1.98,0,StcLandingNoAugOCP::Z_STAGE+0.3,0,0,0,1,0,0,0,0,0,0;
    input.target_snapshot.valid=true;input.target_snapshot.position<<0,0,.2;
    input.now=std::chrono::steady_clock::now();auto started=input.now;controller.trySolve(input);
    check(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<.02,"submission blocks command loop");
    planner_core::PlannerCoreStepResult solved;
    for(int i=0;i<2500;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));input.now=std::chrono::steady_clock::now();auto attempt=controller.trySolve(input);if(attempt.solve_attempted)solved=attempt;if(solved.solve_accepted)break;}
    std::cout<<"cold solve: accepted="<<solved.solve_accepted<<" error="<<solved.solve_result.constraint_error<<" ms="<<solved.solve_result.solve_time_ms<<'\n';
    check(solved.solve_accepted,"cold direct MPC solve rejected");
    auto now=std::chrono::steady_clock::now();auto sample=controller.sample(now,input.current_state,input.target_snapshot);
    for(int i=0;i<100&&!sample.has_replay_sample;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));now=std::chrono::steady_clock::now();sample=controller.sample(now,input.current_state,input.target_snapshot);}
    check(sample.has_replay_sample&&!sample.stale&&sample.command.control.allFinite(),"feedback sample");
    input.current_state.segment<4>(6)*=-1;auto flipped=controller.sample(now,input.current_state,input.target_snapshot);
    check((sample.command.control-flipped.command.control).norm()<1e-10,"quaternion sign feedback");
    check(controller.sample(now+std::chrono::seconds(5),input.current_state,input.target_snapshot).stale,"expired policy");
    controller.stop();
    // Moving-target closed loop exercises the 100 Hz feedback / 10 Hz memory
    // cadence and latency projection, independently of PX4 transport.
    planner_core::DirectMpcController moving;moving.configure(cfg);
    auto moving_pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();O::Dynamics plant(moving_pred);
    Eigen::VectorXd plant_state=Eigen::VectorXd::Zero(O::NX);plant_state(0)=-.6;plant_state(1)=-2;plant_state(2)=StcLandingNoAugOCP::Z_STAGE+.1;plant_state(6)=1;plant_state.tail(4)=O::hover();
    const Eigen::Vector3d target_velocity(.134,.435,0);plant_state.segment<3>(3)=-target_velocity;
    const double initial_distance=plant_state.head(3).norm();
    auto begin=std::chrono::steady_clock::now(),previous=begin;
    Eigen::VectorXd applied=O::hover();int accepted=0,ticks=0;double max_omega=0,max_physical_residual=0;
    Eigen::VectorXd physical_maxima=Eigen::VectorXd::Zero(14);
    auto moving_limits=OCPRegistry::getDescriptor("stc_landing_mpc").make_model(Eigen::Vector3d::Zero());
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<5) {
        auto tick=std::chrono::steady_clock::now();double dt=std::chrono::duration<double>(tick-previous).count();previous=tick;
        for(double elapsed=0;elapsed<dt;) {
            const double step=std::min(.001,dt-elapsed);
            plant_state=plant.flow(plant_state,applied,step,false).x;elapsed+=step;
            auto physical=moving_limits.constraints(plant_state).value;
            physical_maxima=physical_maxima.cwiseMax(physical);
            max_physical_residual=std::max(max_physical_residual,physical.maxCoeff());
        }
        double elapsed=std::chrono::duration<double>(tick-begin).count();input.now=tick;
        input.target_snapshot.velocity=target_velocity;input.target_snapshot.position=target_velocity*elapsed+Eigen::Vector3d(0,0,.2);
        input.current_state=plant_state.head(13);input.current_state.head(3)+=input.target_snapshot.position;input.current_state.segment<3>(3)+=target_velocity;
        auto solve=moving.trySolve(input);if(solve.solve_accepted)++accepted;
        auto feedback=moving.sample(tick,input.current_state,input.target_snapshot);
        check(!feedback.stale,"moving-target feedback expired");
        if(feedback.has_replay_sample){applied=feedback.command.control;++ticks;}
        max_omega=std::max(max_omega,plant_state.segment<3>(10).norm());
        auto physical=moving_limits.constraints(plant_state).value;
        physical_maxima=physical_maxima.cwiseMax(physical);
        max_physical_residual=std::max(max_physical_residual,physical.maxCoeff());
        std::this_thread::sleep_until(tick+std::chrono::milliseconds(10));
    }
    moving.stop();std::cout<<"moving model: accepted="<<accepted<<" ticks="<<ticks<<" distance="<<plant_state.head(3).norm()<<" omega="<<max_omega<<" constraint="<<max_physical_residual<<'\n';
    std::cout<<"physical maxima="<<physical_maxima.transpose()<<std::endl;
    check(max_physical_residual<1e-4,"integrated moving-model physical constraint satisfaction");
    check(accepted>=3&&ticks>100&&plant_state.head(3).norm()<initial_distance,"moving-target closed loop");
    std::cout<<"PASS dynamics="<<maxerr<<" ctcs="<<cerror<<" async feedback/expiry\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
