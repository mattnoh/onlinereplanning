#pragma once
// Reuse the existing reference OCP implementation; do not copy its equations.
#define main pointmass_reference_cli_main
#include COMANDO_POINTMASS_REFERENCE_SOURCE
#undef main
#include "controller_internal.hpp"
#include <filesystem>
namespace {
template<class T> struct OwnedReference {
    std::shared_ptr<Problem> data;
    T object;
    template<class... Args> OwnedReference(std::shared_ptr<Problem> p,Args&&... args)
        :data(std::move(p)),object(*data,std::forward<Args>(args)...){ }
};
template<class T,class... Args> std::shared_ptr<T> owned(std::shared_ptr<Problem> p,Args&&... args){
    auto lifetime=std::make_shared<OwnedReference<T>>(std::move(p),std::forward<Args>(args)...);
    return std::shared_ptr<T>(lifetime,&lifetime->object);
}
struct OwnedRow {std::shared_ptr<Problem> data;std::shared_ptr<StcStageConstraint<double>> row;};
// Both variants have four physical states; CTCS is an interval row, not an
// accumulator state. NODE uses the reference's own stage and terminal rows.
std::shared_ptr<OptimalControlProblem<double>> referenceProblemWithCache(const OCPCreateArgs& args,bool shared_cache,bool node_constraints=false){
    auto p=std::any_cast<std::shared_ptr<Problem>>(args.extra);
    auto result=std::make_shared<OptimalControlProblem<double>>(p->N);
    const bool any=std::any_of(p->mask.begin(),p->mask.end(),[](bool v){return v;});
    auto dynamics=owned<DragDynamics>(p);
    std::vector<Vec> controls(p->N,Vec::Zero(NU)),states(p->N+1,args.current_state);
    if(!args.prev_U.empty())controls=args.prev_U;
    const bool warm=!args.prev_X.empty();if(warm)states=args.prev_X;
    const bool changed_root=warm&&(states.front()-args.current_state).cwiseAbs().maxCoeff()!=0.;
    states[0]=args.current_state;
    // Preserve the benchmark's four-interval tail rebuild; the caller chooses
    // execution cadence. Rebuilding extra tail nodes does not execute them.
    // A measured root invalidates the old shooting trajectory when it differs
    // from prediction. Re-integrate the same controls, without state projection.
    // Exact matched requests retain the reference's original tail-only rebuild.
    for(int k=warm&&!changed_root?p->N-4:0;k<p->N;++k)states[k+1]=intervalFlow(*p,states[k],controls[k],false).xn;
    for(int k=0;k<=p->N;++k){
        result->setInitialState(k,states[k]);if(k==p->N)break;
        const auto flow=shared_cache?std::make_shared<FlowCache>():nullptr;
        result->setInitialControl(k,controls[k]);
        result->setStageDynamics(k,flow?owned<DragDynamics>(p,flow):dynamics);
        result->setStageCost(k,owned<IntegratedTrackingCost>(p,p->epoch+k*p->dt,false,flow));
        result->addStageConstraint(k,std::make_shared<InputNorm>(p->umax));
        if(any&&node_constraints){
            result->addStageConstraint(k,owned<NodalObstacleStage>(p));
        }else if(any){
            auto lifetime=std::make_shared<OwnedRow>();lifetime->data=p;lifetime->row=makeCtcsRow(*p);
            result->addStageConstraint(k,std::shared_ptr<StcStageConstraint<double>>(lifetime,lifetime->row.get()));
        }
    }
    result->setTerminalCost(std::make_shared<ZeroTerminalCost>());
    if(any&&node_constraints)result->addTerminalConstraint(owned<NodalObstacleTerminal>(p));
    return result;
 }
std::shared_ptr<OptimalControlProblem<double>> referenceProblem(const OCPCreateArgs& args){return referenceProblemWithCache(args,false);}
}

namespace {
// sensing_radius: obstacles whose scaled distance from the root is within it
// enter the horizon's frozen mask (6, the reference's sensing range). A single
// horizon is never re-sensed, so it must see every obstacle (infinity).
std::shared_ptr<OCPDescriptor> pointmassReferenceDescriptor(int substeps=100,bool shared_cache=false,bool node_constraints=false,double sensing_radius=6.){
    auto descriptor=std::make_shared<OCPDescriptor>();descriptor->name=node_constraints?"existing_pointmass_node":"existing_pointmass_ctcs";
    descriptor->state_dim=NX;descriptor->control_dim=NU;descriptor->dt=.5;
    descriptor->coordinates=OCPDescriptor::Coordinates::World;
    descriptor->getSolverParams=[](){Param p;p.verbose=false;p.max_iter=300;p.tolerance=1e-6;p.reg1_min=1e-6;p.reg2_min=1.;p.use_ddp_terms=false;p.use_feasibility_ms=true;return p;};
    descriptor->create=[substeps,shared_cache,node_constraints,sensing_radius](const OCPCreateArgs& args){
        auto data=std::make_shared<Problem>();data->substeps=substeps;data->epoch=args.t_abs;
        for(int i=0;i<NOBS;++i)data->mask[i]=(data->s_diag.cwiseProduct(args.current_state.head<2>()-data->centers[i])).norm()<=sensing_radius;
        auto bound=args;bound.extra=data;return referenceProblemWithCache(bound,shared_cache,node_constraints);
    };
    descriptor->make_model=[substeps](const Eigen::Vector3d&){
        OCPDescriptor::Model model;model.nx=NX;model.nu=NU;model.horizon=16;model.dt=.5;model.hover=Vec::Zero(NU);
        model.initial_state=[](const Vec& physical,const Vec&){return physical;};
        model.flow=[substeps](const Vec& x,const Vec& u,double dt,bool jac){
            if(dt<=0)return OCPDescriptor::Flow{x,Mat::Identity(NX,NX),Mat::Zero(NX,NU)};
            Problem p;p.dt=dt;p.substeps=std::max(1,static_cast<int>(std::ceil(dt/(.5/substeps))));
            const auto f=intervalFlow(p,x,u,jac);return OCPDescriptor::Flow{f.xn,f.Ax,f.Bu};
        };return model;
    };
    return descriptor;
}
}
