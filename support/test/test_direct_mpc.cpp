#include "ocp/ocp_stc_landing_mpc.hpp"
#include "ocp/ocp_landing_mpc.hpp"
#include "ocp/ocp_stc_landing_noaug.hpp"
#include "controller_internal.hpp"
#include "planner_core/adapters.hpp"
#include <iostream>
#include <thread>
#include <stdexcept>
#include <map>
static void check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
int main(){try{
    // A zero slack residual/returned trajectory must not certify an infeasible
    // primal. Check every cone type, terminal rows and exact-memory dynamics.
    {
        OptimalControlProblem<double> problem(1);Param parameters;
        problem.setInitialState(0,Eigen::VectorXd::Zero(1));
        problem.setInitialState(1,Eigen::VectorXd::Zero(1));
        problem.setInitialControl(0,Eigen::VectorXd::Zero(1));
        problem.setStageDynamics(std::make_shared<LinearDiscreteDynamics<double>>(Eigen::MatrixXd::Identity(1,1),Eigen::MatrixXd::Identity(1,1)));
        std::vector<Eigen::VectorXd> states(2,Eigen::VectorXd::Zero(1)),controls(1,Eigen::VectorXd::Zero(1));
        check(auditPrimal(problem,states,controls,parameters).feasible(1e-7),"feasible primal rejected");
        states[1](0)=.1;parameters.feasibility_ms_exact_rows={0};
        check(!auditPrimal(problem,states,controls,parameters).feasible(1e-7),"dynamics gap hidden by exact-row setting");
        states[1](0)=0;
        for(auto type:{ConstraintType::NO,ConstraintType::SOC,ConstraintType::EQ}){
            auto p=problem;Eigen::VectorXd offset=Eigen::VectorXd::Constant(type==ConstraintType::SOC?2:1,.1);
            p.addStageConstraint(std::make_shared<LinearStageConstraint<double>>(Eigen::MatrixXd::Zero(offset.size(),1),Eigen::MatrixXd::Zero(offset.size(),1),offset,type));
            check(!auditPrimal(p,states,controls,parameters).feasible(1e-7),"stage constraint violation accepted");
            auto terminal=problem;terminal.addTerminalConstraint(std::make_shared<LinearTerminalConstraint<double>>(Eigen::MatrixXd::Zero(offset.size(),1),offset,type));
            check(!auditPrimal(terminal,states,controls,parameters).feasible(1e-7),"terminal constraint violation accepted");
        }
    }
    namespace O=StcLandingMpcOCP;
    {
        // Isolate CTCS from triggers and changed dynamics/costs. Six ordinary
        // path limits fold into one interval row; no nodal cones remain.
        namespace L=LandingMpcOCP;
        auto prediction=std::make_shared<L::Ref::ConstantAccelPredictor>();
        auto dynamics=std::make_shared<L::Dynamics>(prediction);
        if(!comando_parameters::defaults.mpc.native_px4){
            // Independent continuous-time reference for the actuator state.
            // This catches integration error that an FD Jacobian test cannot:
            // differentiating an inaccurate discrete map can still pass FD.
            Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);physical(6)=1;
            const auto root=L::Dynamics::initialState(physical,L::hover());
            auto command=L::hover();command.tail<3>().setConstant(L::rateLimit()/std::sqrt(3.));
            for(double duration:{L::DT,.037}){
                const auto propagated=dynamics->flow(root,command,duration,false).x;
                const Eigen::Vector3d exact=(1-std::exp(-duration/L::rateTau()))*command.tail<3>();
                check((propagated.segment<3>(10)-exact).norm()<1e-6,"ordinary MPC rate propagation must match the continuous actuator response");
            }
        }
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);
        physical(0)=.04;physical(1)=.03;physical(2)=1.85;physical(6)=1;physical(10)=.03;
        auto state=L::Dynamics::initialState(physical,L::hover());auto control=L::hover();control(1)=.03;
        auto folded=L::foldedLimits(dynamics);
        check(folded->getDimC()==1&&folded->numImplications()==6,"ordinary CTCS must fold all six limits into one row");
        check(folded->c(state,control)(0)>0,"always-active CTCS must detect an angular-rate violation above the STC trigger altitude");
        const auto& parameters=comando_parameters::defaults.constraints;
        const double rate_residual=.03/parameters.omega_stc_max-1.;
        const double expected=L::DT*rate_residual*rate_residual/parameters.ctcs_step_eps-1.;
        check(std::abs(folded->c(state,control)(0)-expected)<1e-10,"folded CTCS must integrate the physical residual with the declared budget and scale");
        const auto cx=folded->cx(state,control),cu=folded->cu(state,control);
        for(int j=0;j<state.size()+control.size();++j){
            auto xp=state,xm=state,up=control,um=control;const double h=1e-7;
            if(j<state.size()){xp(j)+=h;xm(j)-=h;}else{up(j-state.size())+=h;um(j-state.size())-=h;}
            const double fd=(folded->c(xp,up)(0)-folded->c(xm,um)(0))/(2*h);
            const double analytic=j<state.size()?cx(0,j):cu(0,j-state.size());
            check(std::abs(fd-analytic)/(1+std::abs(fd))<1e-5,"ordinary folded CTCS interval Jacobian");
        }
        if(!comando_parameters::defaults.mpc.native_px4){
            // Rate dynamics are linear here, so these Hessian blocks have
            // an independent FD reference, including tangential norm curvature.
            auto curved_state=state;curved_state.segment<3>(10)<<.03,.015,.005;
            auto curved_control=control;curved_control.tail<3>()=curved_state.segment<3>(10);
            const auto hessian=folded->lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            const std::vector<int> indices={10,11,12,L::NX+1,L::NX+2,L::NX+3};
            auto gradient=[&](const auto& x,const auto& u){
                Eigen::VectorXd g(L::NX+L::NU);g.head(L::NX)=folded->cx(x,u).row(0).transpose();
                g.tail(L::NU)=folded->cu(x,u).row(0).transpose();return g;
            };
            for(int column:indices){
                auto xp=curved_state,xm=curved_state,up=curved_control,um=curved_control;const double h=1e-7;
                if(column<L::NX){xp(column)+=h;xm(column)-=h;}else{up(column-L::NX)+=h;um(column-L::NX)-=h;}
                const Eigen::VectorXd fd=(gradient(xp,up)-gradient(xm,um))/(2*h);
                for(int row:indices)check(std::abs(fd(row)-hessian(row,column))/(1+std::abs(fd(row)))<1e-5,"folded CTCS norm curvature");
            }
            check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(hessian).eigenvalues().minCoeff()/(1+hessian.norm())>-1e-12,"folded CTCS convex curvature must remain positive semidefinite");
            curved_state(0)=.4;curved_state(2)=.6;curved_state(3)=.9;curved_state(7)=.03;curved_state.segment<4>(6).normalize();
            const auto full=folded->lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            const auto gn=folded->StcStageConstraint<double>::lagrangianHessian(curved_state,curved_control,Eigen::VectorXd::Ones(1));
            check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>((full-gn).eval()).eigenvalues().minCoeff()/(1+full.norm()+gn.norm())>-1e-12,"one-pass CTCS curvature must preserve all original residual GN terms");
        }
        OCPCreateArgs args;args.current_state=state;
        auto problem=OCPRegistry::create("landing_mpc_ctcs",args);
        for(int k=0;k<L::N;++k){
            const auto rows=problem->getStageConstraints(k,ConstraintType::NO);
            check(rows.size()==2&&rows[0]->getDimC()==8&&rows[1]->getDimC()==1,"CTCS variant must retain only input bounds and one folded path row");
            check(problem->getStageConstraints(k,ConstraintType::SOC).empty(),"nodal SOC constraints must not conceal CTCS behavior");
        }
        check(problem->getTerminalConstraints(ConstraintType::NO).empty()&&problem->getTerminalConstraints(ConstraintType::SOC).empty(),"terminal path limits must also be supplied by the final CTCS interval");
        L::Cost cost;L::Terminal terminal;
        check(problem->getStageCost(0)->q(state,control)==cost.q(state,control),"CTCS comparison must preserve the ordinary stage cost");
        check(problem->getTerminalCost()->p(state)==terminal.p(state),"CTCS comparison must preserve the ordinary terminal cost");
    }
    auto pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();pred->a<<.1,-.1,0;
    auto dyn=std::make_shared<O::Dynamics>(pred);
    Eigen::VectorXd physical(13);physical<<.4,-.3,1.3,.2,.1,-.1,.99,.02,-.03,.01,.05,-.04,.02;
    Eigen::VectorXd x=O::Dynamics::initialState(physical,O::hover());x(13)=.4;
    x.segment<4>(6).normalize();auto u=O::hover();u.tail(3)<<.08,-.06,.02;
    auto model=OCPRegistry::getDescriptor("stc_landing_mpc").make_model(pred->a);
    {
        // A changed forecast must not move the old nominal world trajectory.
        // Include an off-grid age and extension beyond the old terminal node.
        OcpSolver::Result old;old.state_trajectory.push_back(x);
        for(int k=0;k<model.horizon;++k){
            old.control_trajectory.push_back(u);
            old.state_trajectory.push_back(model.flow(old.state_trajectory.back(),u,model.dt,false).x);
        }
        TargetSnapshot before,after;before.position<<1,2,.2;before.velocity<<.3,-.2,0;
        before.acceleration=pred->a;after.position<<1.1,1.8,.3;after.velocity<<.4,-.1,.02;
        after.acceleration<<-.2,.3,0;
        const double age=.037;
        const auto warm=planner_core::shiftMpcSeed(old,model,before,after,age);
        for(int k=0;k<=model.horizon;++k){
            const double time=age+k*model.dt,next=k*model.dt;
            const int i=std::min(static_cast<int>(time/model.dt),model.horizon);
            const auto previous=model.flow(old.state_trajectory[i],u,time-i*model.dt,false).x;
            const Eigen::Vector3d world_p=previous.head<3>()+before.position+before.velocity*time+.5*before.acceleration*time*time;
            const Eigen::Vector3d world_v=previous.segment<3>(3)+before.velocity+before.acceleration*time;
            check((warm.prev_X[k].head<3>()+after.position+after.velocity*next+.5*after.acceleration*next*next-world_p).norm()<1e-10,"MPC warm shift must preserve nominal world position");
            check((warm.prev_X[k].segment<3>(3)+after.velocity+after.acceleration*next-world_v).norm()<1e-10,"MPC warm shift must preserve nominal world velocity");
            check(std::abs(warm.prev_X[k](13)-next)<1e-12,"MPC warm clock must start at the new measurement epoch");
        }
        check((warm.prev_X.back().head(13)-old.state_trajectory.back().head(13)).norm()>1e-4,"MPC terminal warm state must advance beyond the old horizon");
    }
    auto direct=dyn->flow(x,u,O::DT,true);auto through_interface=model.flow(x,u,O::DT,true);
    check((direct.x-through_interface.x).norm()<1e-12 && (direct.a-through_interface.a).norm()<1e-12 && (direct.b-through_interface.b).norm()<1e-12,"MPC model interface must preserve dynamics and Jacobians");
    const auto a=dyn->fx(x,u), b=dyn->fu(x,u);double maxerr=0;double h=1e-6;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;
        if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        Eigen::VectorXd fd=(dyn->f(xp,up)-dyn->f(xm,um))/(2*h);
        Eigen::VectorXd analytic=j<O::NX? a.col(j).eval():b.col(j-O::NX).eval();
        const double error=(fd-analytic).norm();
        if(error>maxerr&&error>1e-5) {
            Eigen::Index row;const double peak=(fd-analytic).cwiseAbs().maxCoeff(&row);
            std::cerr<<"Jacobian column="<<j<<" row="<<row<<" error="<<peak<<" finite="<<fd(row)<<" analytic="<<analytic(row)<<'\n';
        }
        maxerr=std::max(maxerr,error);
    }
    check(maxerr<1e-5,"dynamics Jacobians");
    x(2)=.45; x.segment<3>(3)<<.6,.4,-.5;
    auto row=StcLandingNoAugOCP::makeCtcsConstraint<double>(dyn,StcLandingNoAugOCP::CTCS_STEP_EPS/StcLandingNoAugOCP::CTCS_Y_SCALE,-1,O::DT,O::substeps());
    auto cx=row->cx(x,u),cu=row->cu(x,u);
    double cerror=0;
    for(int j=0;j<O::NX+O::NU;++j){auto xp=x,xm=x,up=u,um=u;if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
        double fd=(row->c(xp,up)(0)-row->c(xm,um)(0))/(2*h);double analytic=j<O::NX?cx(0,j):cu(0,j-O::NX);cerror=std::max(cerror,std::abs(fd-analytic)/(1+std::abs(fd)));}
    check(cerror<1e-5,"CTCS Jacobians");
    {
        O::Cost cost;auto state=x;state(2)=.5*(O::Ref::Z_STAGE+O::Ref::ALT_TRIG);
        const auto gx=cost.qx(state,u),gu=cost.qu(state,u);
        for(int j=0;j<O::NX+O::NU;++j){auto xp=state,xm=state,up=u,um=u;
            if(j<O::NX){xp(j)+=h;xm(j)-=h;}else{up(j-O::NX)+=h;um(j-O::NX)-=h;}
            const double fd=(cost.q(xp,up)-cost.q(xm,um))/(2*h);
            const double analytic=j<O::NX?gx(j):gu(j-O::NX);
            check(std::abs(fd-analytic)/(1+std::abs(fd))<1e-5,"landing rate-cost gradient");
        }
    }
    {
        // The touchdown residual has a product of altitude and velocity.
        // Check GN curvature against its independent residual Jacobian.
        using R=StcLandingNoAugOCP::CTcSTCStageCost<double>;
        const double weight=StcLandingNoAugOCP::W_TOUCH_VEL;
        R full(1.,0.,0.,0.,Eigen::Vector3d::Zero());
        Eigen::VectorXd state=Eigen::VectorXd::Zero(StcLandingNoAugOCP::NX_SS);
        state(2)=.1;state(3)=.6;state(4)=-.3;state(5)=-.4;state(6)=1;
        const auto input=Eigen::VectorXd::Zero(StcLandingNoAugOCP::NU_SS);
        const double touch=(StcLandingNoAugOCP::TOUCH_ALT-state(2))/StcLandingNoAugOCP::TOUCH_SCALE;
        Eigen::MatrixXd j=Eigen::MatrixXd::Zero(3,state.size());
        j.block<3,1>(0,2)=-state.segment<3>(3)/StcLandingNoAugOCP::TOUCH_SCALE;
        j.block<3,3>(0,3)=touch*Eigen::Matrix3d::Identity();
        Eigen::MatrixXd expected=2*weight*j.transpose()*j;
        expected.block<3,3>(3,3).diagonal().array()+=2;
        expected.block<3,3>(10,10).diagonal().array()+=2;
        check((full.qxx(state,input)-expected).norm()<1e-10,"RH touchdown GN curvature must match residual Jacobian");
        auto model_state=O::Dynamics::initialState(state,O::hover());O::Cost mpc_cost;
        check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(mpc_cost.qxx(model_state,O::hover())).eigenvalues().minCoeff()>-1e-8,"MPC GN curvature must remain positive semidefinite at touchdown");
    }
    Eigen::VectorXd odom=Eigen::VectorXd::Zero(13);odom(3)=1;odom(6)=std::sqrt(.5);odom(9)=std::sqrt(.5);odom(10)=.2;
    auto world=planner_core::bodyOdometryToWorld(odom);check(std::abs(world(4)-1)<1e-12&&std::abs(world(3))<1e-12&&world(10)==.2,"odometry frame conversion");
    check(std::abs(planner_core::normalizedThrust(9.81,.7)-.7)<1e-12&&planner_core::normalizedThrust(30,.7)==1,"thrust normalization");
    for(double requested:{0.,3.,6.,9.81,12.}) {
        const double signal=planner_core::normalizedThrust(requested,.7282821834,2,150./850);
        if(signal>0&&signal<1) {
            const double physical=9.81*std::pow((signal+150./850)/(.7282821834+150./850),2);
            check(std::abs(physical-requested)<1e-12,"motor thrust inverse must preserve DDP acceleration");
        }
    }
    O::SpeedBound speed;StcLandingNoAugOCP::GeneralSpeedCon<double> legacy_speed;
    check((speed.c(x,u)-legacy_speed.c(x.head(14),u)).norm()<1e-12,"speed cone convention");
    OCPCreateArgs args;args.current_state=x;args.target_accel=pred->a;auto problem=O::create(args);
    check(problem->getInitialControl(0).size()==4,"MPC control layout");
    check(problem->getStageConstraints(0,ConstraintType::NO).size()==2,
          "input bounds and CTCS must constrain the first predicted interval");
    check(problem->getStageConstraints(1,ConstraintType::SOC).size()==1,
          "CTCS MPC future speed bound must remain enforced");
    check(std::abs(dyn->f(x,u)(13)-x(13)-O::DT)<1e-12,"fixed time grid");
    check(O::descriptor().command_mode==OCPDescriptor::CommandMode::CmdBodyRate,"direct command type");
    {
        OCPCreateArgs hover_args;hover_args.current_state=Eigen::VectorXd::Zero(13);
        hover_args.current_state(2)=O::Ref::Z_STAGE+.3;hover_args.current_state(6)=1;
        auto feasible=O::create(hover_args);
        for(double mu:{1.,.01,1e-6}){
            ALIPDDP<double> seeded(*feasible);
            auto parameters=O::descriptor().getSolverParams();parameters.mu=parameters.muT=mu;
            seeded.init(parameters);
            check(seeded.error<1e-7,"recentring feasible CTCS must not inject a barrier-dependent slack residual");
        }
    }
    if(comando_parameters::defaults.mpc.native_px4) {
        const auto ordinary=OCPRegistry::getDescriptor("landing_mpc").make_model(pred->a);
        check((ordinary.flow(x,u,.1,true).x-dyn->flow(x,u,.1,true).x).norm()<1e-12,"ordinary and CTCS must predict the same native plant");
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);physical(6)=1;
        auto hover_state=O::Dynamics::initialState(physical,O::hover());
        auto hover_pred=std::make_shared<O::Ref::ConstantAccelPredictor>();O::Dynamics native(hover_pred);
        const auto stationary=native.flow(hover_state,O::hover(),.5,false).x;
        check(stationary.head(13).isApprox(physical,1e-8),"source-derived native hover equilibrium must conserve physical state");
        hover_state.segment<3>(O::INTEGRAL)<<.006,-.004,.002;
        auto native_rates=native.intervalRates(hover_state,O::hover());
        check(native_rates.segment<4>(O::ROTOR).norm()>.01,"native integrator must affect actuator prediction");
        auto cancelling=O::hover();cancelling.tail<3>()=-hover_state.segment<3>(O::INTEGRAL);
        check(native.intervalRates(hover_state,cancelling).segment<4>(O::ROTOR).norm()<1e-10,"native integral state must enter the same PID equations used by PX4");
        check(native.flow(hover_state,O::hover(),.1,false).x.segment<3>(10).norm()>1e-3,"first-order zero-rate projection must not erase native integrator drift");
        OcpSolver::Config config;config.ocp_type="landing_mpc";OcpSolver solver(config);
        hover_state(2)=1.85;
        const auto solved=solver.solve(hover_state);
        check(solved.success,"native DDP solve must return a trajectory");
        const LandingMpcOCP::Ref::LandingLimits limits(O::NX,O::NU);
        double excess=0.;for(const auto& state:solved.state_trajectory)excess=std::max(excess,limits.c(state,O::hover()).maxCoeff());
        std::cout<<"native cold solve: error="<<solved.constraint_error<<" physical_excess="<<excess<<" ms="<<solved.solve_time_ms<<'\n';
        check(excess<1e-4,"native DDP must predict feasible physical states with nonzero rate integral");
        std::cout<<"PASS native dynamics="<<maxerr<<" ctcs="<<cerror<<'\n';return 0;
    }
    {
        // A failed initial solve must not seed the next measured-state problem
        // or switch it to the shorter warm iteration budget.
        const auto model=OCPRegistry::getDescriptor("landing_mpc").make_model(Eigen::Vector3d::Zero());
        Eigen::VectorXd physical=Eigen::VectorXd::Zero(13);
        physical(0)=.04;physical(2)=1.85;
        physical(6)=std::cos(M_PI/8);physical(8)=std::sin(M_PI/8);
        OcpSolver::Config config;config.ocp_type="landing_mpc";config.reuse_solver=true;
        OcpSolver startup(config);
        check(!startup.solve(model.initial_state(physical,model.hover)).success,"infeasible first measured state must fail");
        const auto failed_cache=startup.snapshotWarmStart();
        check(failed_cache.prev_X.empty()&&failed_cache.prev_U.empty()&&failed_cache.prev_K.empty(),"failed primal must not enter the warm-start cache");
        physical(6)=1;physical(8)=0;
        check(startup.solve(model.initial_state(physical,model.hover)).success,"valid state after failed startup must receive a full initial solve");
    }
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
    check(solved.x0_source=="measured","MPC must use measured initial state");
    Eigen::VectorXd expected=input.current_state;
    expected.head<3>()-=input.target_snapshot.position;
    expected.segment<3>(3)-=input.target_snapshot.velocity;
    check((solved.solve_result.state_trajectory.front().head(13)-expected).norm()<1e-10,
          "DDP root must equal measured state, without future projection");
    auto now=std::chrono::steady_clock::now();auto sample=controller.sample(now,input.current_state,input.target_snapshot);
    for(int i=0;i<500&&!sample.has_replay_sample;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));now=std::chrono::steady_clock::now();sample=controller.sample(now,input.current_state,input.target_snapshot);}
    check(sample.has_replay_sample&&!sample.stale&&sample.command.control.allFinite(),"DDP command sample");
    check(sample.solve_num==solved.solve_num,"first DDP result must be applied before the next solve");
    check((sample.command.control-solved.solve_result.control_trajectory.front()).norm()<1e-12,
          "MPC must apply first optimized input");
    auto held=controller.sample(now+std::chrono::milliseconds(250),input.current_state,input.target_snapshot);
    check((held.command.control-sample.command.control).norm()<1e-12,
          "publication must not advance through optimized trajectory");
    input.current_state.segment<4>(6)*=-1;auto flipped=controller.sample(now,input.current_state,input.target_snapshot);
    check((sample.command.control-flipped.command.control).norm()<1e-10,"quaternion sign must not alter DDP control");
    check(controller.sample(now+std::chrono::seconds(5),input.current_state,input.target_snapshot).stale,"expired policy");
    controller.stop();
    // Each measured-state DDP solve supplies only its first input to PX4.
    for(const std::string type:{"stc_landing_mpc","landing_mpc"}) {
    cfg.ocp_type=type;
    planner_core::DirectMpcController moving;moving.configure(cfg);
    auto moving_pred=std::make_shared<StcLandingNoAugOCP::ConstantAccelPredictor>();O::Dynamics plant(moving_pred);
    Eigen::VectorXd plant_state=O::Dynamics::initialState(Eigen::VectorXd::Zero(13),O::hover());plant_state(0)=-.6;plant_state(1)=-2;plant_state(2)=StcLandingNoAugOCP::Z_STAGE+.1;plant_state(6)=1;plant_state.tail(4)=O::hover();
    const Eigen::Vector3d target_velocity(.134,.435,0);plant_state.segment<3>(3)=-target_velocity;
    if(type=="landing_mpc") {
        // Viable cone start: match the moving target's velocity before
        // engaging the ordinary all-time constraints, as in SITL staging.
        plant_state.head<3>()<<.04,.03,1.85;
        plant_state.segment<3>(3).setZero();
    }
    const double initial_distance=plant_state.head(3).norm();
    auto begin=std::chrono::steady_clock::now(),previous=begin;
    Eigen::VectorXd applied=O::hover();int accepted=0,ticks=0,recovery_updates=0;bool disturbed=false;double max_omega=0,max_warm_ms=0;
    std::map<int,std::vector<Eigen::VectorXd>> policies;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<5) {
        auto tick=std::chrono::steady_clock::now();double dt=std::chrono::duration<double>(tick-previous).count();previous=tick;
        plant_state=plant.flow(plant_state,applied,dt,false).x;
        double elapsed=std::chrono::duration<double>(tick-begin).count();input.now=tick;
        if(type=="landing_mpc"&&!disturbed&&elapsed>=1.) {
            plant_state(10)=.03;disturbed=true;
        }
        input.target_snapshot.velocity=target_velocity;input.target_snapshot.position=target_velocity*elapsed+Eigen::Vector3d(0,0,.2);
        input.current_state=plant_state.head(13);input.current_state.head(3)+=input.target_snapshot.position;input.current_state.segment<3>(3)+=target_velocity;
        auto solve=moving.trySolve(input);if(solve.solve_accepted){++accepted;if(disturbed)++recovery_updates;if(accepted>1)max_warm_ms=std::max(max_warm_ms,solve.solve_result.solve_time_ms);policies[solve.solve_num]=solve.solve_result.control_trajectory;
}
        auto feedback=moving.sample(tick,input.current_state,input.target_snapshot);
        if(solve.solve_attempted&&!solve.solve_accepted)
            std::cout<<"DDP rejected: error="<<solve.solve_result.constraint_error<<" reason="<<solve.rejection_reason<<'\n';
        if(feedback.stale)std::cout<<"DDP expired: accepted="<<accepted<<" ticks="<<ticks<<" state="<<plant_state.head(13).transpose()<<std::endl;
        check(!feedback.stale,"moving-target DDP policy expired");
        if(feedback.has_replay_sample){
            auto found=policies.find(feedback.solve_num);
            check(found!=policies.end(),"applied policy must come from a DDP solve");
            check((feedback.command.control-found->second.front()).norm()<1e-12,
                  "published control must equal the first optimized DDP control");
            applied=feedback.command.control;++ticks;
        }
        max_omega=std::max(max_omega,plant_state.segment<3>(10).norm());
        std::this_thread::sleep_until(tick+std::chrono::milliseconds(10));
    }
    moving.stop();std::cout<<type<<" moving model: accepted="<<accepted<<" ticks="<<ticks<<" distance="<<plant_state.head(3).norm()<<" omega="<<max_omega<<" max_warm_ms="<<max_warm_ms<<'\n';
    if(type=="landing_mpc")check(disturbed&&recovery_updates>=30,"measured rate excursion must not stop MPC updates");
    check(accepted>=30&&ticks>200&&plant_state.head(3).norm()<initial_distance,"moving-target closed loop");
    }
    std::cout<<"PASS dynamics="<<maxerr<<" ctcs="<<cerror<<" measured-state first-input DDP MPC, CTCS and ordinary moving targets\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
