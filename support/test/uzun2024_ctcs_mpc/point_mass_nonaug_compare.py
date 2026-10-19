"""Direct paper-style non-augmented prox-linear versus production folded DDP.

Fixed-time N=6, same root, returned stage-policy seed, physical objective,
constraints and C++ CTCS numerical map. No accumulator state is introduced.
The obstacle problem is nonconvex: Section 5's convex global theorem does not
apply. This is an isolated method comparison, not an author-code reproduction.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np

ROOT_STATE=np.array([5.,5.,10.,-.2,-.2,-.3])
GOAL=np.array([0.,0.,1.,0.,0.,0.])
OBS=[2.85355,2.14645,5.5]
TAN20=np.tan(np.deg2rad(20.))


def rollout(U,dt=4/6):
    X=[ROOT_STATE.copy()]
    for u in U:
        x=X[-1]; a=u-[0,0,9.81]
        X.append(np.r_[x[:3]+dt*x[3:]+.5*dt*dt*a,x[3:]+dt*a])
    return np.array(X)


def physical_objective(U,dt=4/6): return 4.+1e-5*dt*float(np.sum(np.asarray(U)**2))


class Oracle:
    def __init__(self,binary,row,steps):
        self.process=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.row=row;self.steps=steps;self.calls=0;self.seconds=0.
    def evaluate(self,X,U):
        start=time.perf_counter();self.calls+=1
        r=self.row; N=len(U); dt=4/N;eps=float(r['eps'])
        requests=[' '.join(map(str,[*map(float,x),*map(float,u),dt,eps,float(r['tighten_obs']),
            float(r['tighten_glide']),1/np.sqrt(dt),self.steps,1,1.,*OBS,2.])) for x,u in zip(X[:-1],U)]
        self.process.stdin.write('\n'.join(requests)+'\n');self.process.stdin.flush()
        result=[json.loads(self.process.stdout.readline()) for _ in U]
        self.seconds+=time.perf_counter()-start
        return np.array([v['row']/eps for v in result]),np.array([v['jacobian'] for v in result])/eps
    def close(self):
        self.process.stdin.close();self.process.wait(timeout=30);self.process.stdout.close()


class ProxProblem:
    def __init__(self,N):
        import cvxpy as cp
        self.cp=cp;dt=4/N
        self.X=cp.Variable((N+1,6));self.U=cp.Variable((N,3));self.e=cp.Variable(N,nonneg=True)
        self.Jx=cp.Parameter((N,6));self.Ju=cp.Parameter((N,3));self.bias=cp.Parameter(N)
        self.reg=cp.Parameter(nonneg=True);self.Px=cp.Parameter((N+1,6));self.Pu=cp.Parameter((N,3))
        self.gamma=cp.Parameter(nonneg=True)
        accel=self.U-np.array([0.,0.,9.81])
        constraints=[self.X[0]==ROOT_STATE,self.X[-1]==GOAL,
            self.X[1:,:3]==self.X[:-1,:3]+dt*self.X[:-1,3:]+.5*dt*dt*accel,
            self.X[1:,3:]==self.X[:-1,3:]+dt*accel,
            cp.norm(self.U,axis=1)<=1.5*9.81,cp.norm(self.U[:,:2],axis=1)<=TAN20*self.U[:,2]]
        linear_rows=self.bias+cp.sum(cp.multiply(self.Jx,self.X[:-1]),axis=1)+cp.sum(cp.multiply(self.Ju,self.U),axis=1)
        constraints+=[self.e>=linear_rows]
        # Convex quadratic physical cost is retained in J of the composite
        # model; only xi is linearized. This is a valid prox-linear split.
        obj=1e-5*dt*cp.sum_squares(self.U)+self.gamma*cp.sum(self.e)
        obj+=.5*self.reg*(cp.sum_squares(self.X)+cp.sum_squares(self.U))
        obj+=cp.sum(cp.multiply(self.Px,self.X))+cp.sum(cp.multiply(self.Pu,self.U))
        self.problem=cp.Problem(cp.Minimize(obj),constraints)
    def solve(self,X,U,c,J,rho,gamma):
        self.Jx.value=J[:,:6];self.Ju.value=J[:,6:]
        self.bias.value=c-np.sum(J[:,:6]*X[:-1],axis=1)-np.sum(J[:,6:]*U,axis=1)
        self.reg.value=1/rho;self.Px.value=-X/rho;self.Pu.value=-U/rho;self.gamma.value=gamma
        self.problem.solve(solver='CLARABEL',tol_gap_abs=1e-10,tol_gap_rel=1e-10,tol_feas=1e-10,max_iter=300,warm_start=True)
        if self.problem.status not in ('optimal','optimal_inaccurate'): raise RuntimeError(self.problem.status)
        return self.X.value.copy(),self.U.value.copy(),self.problem.status


def run_prox(out,oracle,X,U,gamma,max_iter=250):
    problem=ProxProblem(len(U));rho=1000.;records=[];trials=[]
    start=time.perf_counter();c,J=oracle.evaluate(X,U)
    (out/'initial_integral_model.json').write_text(json.dumps({'normalized_rows':c.tolist(),'normalized_jacobians':J.tolist()},indent=2)+'\n')
    record=lambda iteration:dict(iteration=iteration,wall_seconds=time.perf_counter()-start,
        X=X.tolist(),U=U.tolist(),objective=physical_objective(U),max_numerical_row=float(max(c)),
        gamma=gamma,rho=rho,oracle_calls=oracle.calls,oracle_seconds=oracle.seconds)
    records.append(record(0));reason='iteration_limit'
    for iteration in range(1,max_iter+1):
        merit=physical_objective(U)+gamma*float(np.sum(np.maximum(c,0.)))
        accepted=False
        for bt in range(40):
            xp,up,status=problem.solve(X,U,c,J,rho,gamma)
            cp,jp=oracle.evaluate(xp,up)
            norm2=float(np.sum((xp-X)**2)+np.sum((up-U)**2))
            merit_new=physical_objective(up)+gamma*float(np.sum(np.maximum(cp,0.)))
            accepted=merit_new<=merit-.05/rho*norm2+1e-12
            trials.append(dict(iteration=iteration,backtrack=bt,rho=rho,gamma=gamma,
                merit=merit,trial_merit=merit_new,step_norm=norm2**.5,accepted=accepted,solver_status=status))
            if accepted: break
            rho*=.5
        if not accepted: reason='line_search_failed';break
        mapping=norm2**.5/rho;X,U,c,J=xp,up,cp,jp
        records.append(dict(record(iteration),prox_gradient_mapping=mapping,accepted_rho=rho,backtracks=bt))
        if mapping<1e-7:
            if max(c)<=1e-4: reason='prox_gradient_and_budget';break
            gamma*=10.
        rho=min(rho*1.5,1e6)
    (out/'prox_trace.json').write_text(json.dumps(records,indent=2)+'\n')
    (out/'prox_trials.json').write_text(json.dumps(trials,indent=2)+'\n')
    return records,dict(termination=reason,iterations=len(records)-1,oracle_calls=oracle.calls,
                        oracle_seconds=oracle.seconds,wall_seconds=time.perf_counter()-start,trial_count=len(trials))


def grade_trace(records,row):
    sys.path.insert(0,'/home/lab/projects/solver/benchmark/openscvx_point_mass_ctcs')
    from run_comparison import segment
    args=argparse.Namespace(mode='fixed',N=6,fixed_tf=4.,budget='interval',tighten_obs=float(row['tighten_obs']),tighten_glide=float(row['tighten_glide']))
    result=[];eps=float(row['eps']); cache={}
    for r in records:
        X,U=np.array(r['X']),np.array(r['U']);key=U.tobytes()+X.tobytes()
        if key in cache: audit=cache[key]
        else:
            propagated=ROOT_STATE.copy();ints=[];local=[];pen=[];glide=[];gaps=[]
            for k,u in enumerate(U):
                end,_,_,integral,_=segment(X[k],u,4/6,args);local.append(integral)
                gaps.append(float(np.max(abs(end-X[k+1]))))
                propagated,po,gs,integral,_=segment(propagated,u,4/6,args)
                ints.append(integral);pen.append(po);glide.append(gs)
            audit=dict(objective=physical_objective(U),max_true_I_over_epsilon=max(ints)/eps,
                max_local_true_I_over_epsilon=max(local)/eps,max_physical_excess=max(max(pen),max(glide)),
                terminal_error=float(np.max(abs(propagated-GOAL))),shooting_gap=max(gaps),
                input_excess=float(np.max(np.linalg.norm(U,axis=1)-1.5*9.81)),
                cone_excess=float(np.max(np.linalg.norm(U[:,:2],axis=1)-TAN20*U[:,2])))
            audit['common_feasible']=bool(audit['max_true_I_over_epsilon']<=1.0001 and audit['max_physical_excess']<=1e-7
                and audit['terminal_error']<=1e-6 and audit['shooting_gap']<=1e-8 and max(audit['input_excess'],audit['cone_excess'])<=1e-7)
            audit['strict_budget']=bool(audit['max_true_I_over_epsilon']<=1.)
            cache[key]=audit
        result.append(dict(r,independent=audit))
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--snapshot',type=Path,required=True);ap.add_argument('--oracle',type=Path,required=True)
    ap.add_argument('--ddp',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--substeps',type=int,default=512);ap.add_argument('--gamma',type=float,default=1e-6)
    ap.add_argument('--max-prox-iter',type=int,default=250)
    ap.add_argument('--seed-ctcs-blend',type=float,default=.1)
    ap.add_argument('--ddp-normalized',action='store_true')
    ap.add_argument('--ddp-no-gn',action='store_true')
    ap.add_argument('--ddp-strict-gate',action='store_true')
    ap.add_argument('--ddp-max-inner',type=int,default=60)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    row=json.loads((args.snapshot/'inputs/fixed_DDP_ctcs_N6/selected_row.json').read_text())
    nodes=list(csv.DictReader((args.snapshot/'inputs/fixed_DDP_node_N6/nodes.csv').open()))
    U=np.array([[float(r[s]) for s in ('ux','uy','uz')] for r in nodes[:-1]])
    ctcs_nodes=list(csv.DictReader((args.snapshot/'inputs/fixed_DDP_ctcs_N6/nodes.csv').open()))
    ctcs_U=np.array([[float(r[s]) for s in ('ux','uy','uz')] for r in ctcs_nodes[:-1]])
    if not 0<=args.seed_ctcs_blend<=1: raise ValueError('seed blend must lie in [0,1]')
    U=(1-args.seed_ctcs_blend)*U+args.seed_ctcs_blend*ctcs_U;X=rollout(U)
    seed=out/'seed.txt';np.savetxt(seed,np.r_[X.ravel(),U.ravel()],fmt='%.17g')
    (out/'configuration.json').write_text(json.dumps(dict(vars(args),snapshot=str(args.snapshot),oracle=str(args.oracle),ddp=str(args.ddp),output=str(out),selected_row=row,
        seed='exact rollout of a shared convex blend of frozen stage and CTCS controls; blend avoids near-zero input-cone slack',scope='direct non-augmented method comparison; nonconvex extension of paper Section 5'),indent=2,default=str)+'\n')
    command=[str(args.ddp),'6',str(args.substeps),row['eps'],row['tighten_obs'],row['tighten_glide'],str(seed),str(out/'ddp_trace.jsonl'),str(int(args.ddp_normalized)),str(int(not args.ddp_no_gn))]
    command += [str(args.ddp_max_inner),str(int(args.ddp_strict_gate))]
    result=subprocess.run(command,capture_output=True,text=True,check=True)
    (out/'ddp_summary.json').write_text(result.stdout);ddp=[json.loads(line) for line in (out/'ddp_trace.jsonl').read_text().splitlines()]
    oracle=Oracle(args.oracle,row,args.substeps)
    try: prox,summary=run_prox(out,oracle,X.copy(),U.copy(),args.gamma,args.max_prox_iter)
    finally: oracle.close()
    ddp,prox=grade_trace(ddp,row),grade_trace(prox,row)
    (out/'ddp_audited.json').write_text(json.dumps(ddp,indent=2)+'\n')
    (out/'prox_audited.json').write_text(json.dumps(prox,indent=2)+'\n')
    common_ref=min(r['independent']['objective'] for arm in (ddp,prox) for r in arm if r['independent']['common_feasible']) if any(r['independent']['common_feasible'] for arm in (ddp,prox) for r in arm) else None
    def brief(arm):
        feasible=[r for r in arm if r['independent']['common_feasible']]
        quality=[r for r in feasible if common_ref is not None and r['independent']['objective']<=common_ref+1e-6]
        return dict(events=len(arm),final=arm[-1]['independent'],first_feasible_iteration=None if not feasible else feasible[0]['iteration'],
            first_feasible_seconds=None if not feasible else feasible[0]['wall_seconds'],
            first_common_quality_iteration=None if not quality else quality[0]['iteration'])
    results=dict(ddp=brief(ddp),prox=brief(prox),prox_native=summary,common_best_feasible_objective=common_ref,
        caution='prototype wall times include different language/oracle/solver overhead; not an intrinsic algorithm speed ranking')
    (out/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'serif'})
    fig,axs=plt.subplots(1,2,figsize=(11,4))
    for arm,label,color in ((ddp,'Folded DDP','#3155a6'),(prox,'Non-augmented prox-linear','#b44d22')):
        it=[r['iteration'] for r in arm]
        axs[0].plot(it,[r['independent']['objective']-4 for r in arm],label=label,color=color)
        axs[1].semilogy(it,[max(1e-5,r['independent']['max_true_I_over_epsilon']) for r in arm],label=label,color=color)
    axs[0].set(ylabel='Physical effort cost (objective minus 4)',xlabel='Native iteration')
    axs[1].axhline(1,color='black',ls='--',lw=1);axs[1].set(ylabel='Independent propagated max I / epsilon',xlabel='Native iteration')
    for ax in axs:ax.grid(alpha=.2);ax.legend()
    fig.suptitle('Same fixed-time N=6 point-mass OCP; iteration types have different work')
    fig.tight_layout();fig.savefig(out/'nonaug_convergence.png',dpi=180);plt.close(fig)
    for source in (Path(__file__),Path(__file__).with_name('point_mass_nonaug_trace.cpp')):shutil.copy2(source,out/source.name)
    inv={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
    (out/'sha256_inventory.json').write_text(json.dumps(inv,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__': main()
