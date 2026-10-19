"""Matched point-mass calibration: frozen DDP and one lifted-SCP candidate.

All solver cores remain untouched. Only calibration bearings are executable
until the complete protocol is frozen; historical best-of-pool code is unused.
"""
import argparse
import dataclasses
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback

import numpy as np
from point_mass_paper_protocol import candidate, digest, shared_seed, frozen_ddp_profile
from point_mass_certificate import certify, flow, certify_input, interval_difference, tightening
from point_mass_saved_audit import reference_integral

HERE=Path(__file__).resolve().parent
SCP=Path('/home/lab/projects/solver/SCP/OpenSCvx')
SOLVER=HERE.parents[3]/'ALIPDDP-main'


def write_json(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def scp_profile():
    return dict(status='one_candidate_no_tuning_pool',max_iter=200,lam_cost=1.,
                lam_prox=.1,lam_vc=100.,ep_tr=1e-8,ep_vc=1e-10,ep_vb=1e-8,
                autotuner='AugmentedLagrangian',ode_tol=1e-10,max_step_divisor=32,
                solver='CLARABEL',cvx_tolerance=1e-10,cvx_max_iter=300,
                density='physical_density / physical_budget',increment_bound=1.)


def prepare(out):
    out.mkdir(parents=True,exist_ok=False)
    protocol=candidate();protocol['status']='calibration_only_ddp_profile_frozen'
    protocol['profiles']['lifted_openscvx']=scp_profile()
    protocol['source_pins']={}
    for label,repo in [('ddp',SOLVER),('openscvx',SCP)]:
        dirty=subprocess.check_output(['git','status','--short','--untracked-files=no'],cwd=repo,text=True)
        if dirty:raise ValueError(f'{label} tracked source is dirty')
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
        protocol['source_pins'][label]=commit
    if protocol['source_pins']['ddp']!=frozen_ddp_profile()['solver_commit']:
        raise ValueError('DDP core differs from the frozen user-selected revision')
    protocol['calibration_work_budget']=dict(profile_trials_per_method=1,
        cases=[c['id'] for c in protocol['calibration_cases']],meshes=[6],
        time_modes=['fixed','free'],native_caps={'folded_ddp':1200,'lifted_openscvx':200},
        note='six solves per primary method, no retuning/retries; native iterations are different work units')
    write_json(out/'protocol.json',protocol)
    jobs=[]
    for case in protocol['calibration_cases']:
        for mode in protocol['time_modes']:
            job=dict(case=case,mode=mode,N=6,seed=shared_seed(case,6),
                     protocol_sha256=digest(protocol))
            name=f"{case['id']}_{mode}_N6";write_json(out/(name+'.json'),job);jobs.append(name)
    write_json(out/'manifest.json',dict(status='calibration_only',protocol_sha256=digest(protocol),
        ddp_profile_sha256=digest(protocol['profiles']['folded_ddp']),jobs=jobs,
        test_bearings_executed=0,paper_campaign_frozen=False))
    return jobs


def check_source_pins(protocol):
    for label,repo in [('ddp',SOLVER),('openscvx',SCP)]:
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
        dirty=subprocess.check_output(['git','status','--short','--untracked-files=no'],cwd=repo,text=True)
        if dirty or commit!=protocol['source_pins'][label]:raise ValueError(f'{label} source pin changed')


def ddp_run(protocol,job,out,binary):
    profile=protocol['profiles']['folded_ddp'];seed=job['seed'];N=job['N'];fixed=job['mode']=='fixed'
    U=np.asarray(seed['U'])
    if not fixed:U=np.c_[U,seed['durations_s']]
    values=[N,int(fixed),profile['optimizer_rk4_substeps'],protocol['physical_budget_per_interval'],
            protocol['fixed_tf_s'],*protocol['free_tf_bounds_s'],
            protocol['tightening_m']['obstacle'],protocol['tightening_m']['glide_residual']]
    text=' '.join(map(str,values))+'\n'
    text+=' '.join(map(str,np.asarray(seed['X']).ravel()))+'\n'
    text+=' '.join(map(str,U.ravel()))+'\n'
    for name,value in profile['settings'].items():
        if name=='feasibility_ms_exact_rows':
            if value:raise ValueError('frozen exact rows must remain empty')
            continue
        text+=name+' '+str(int(value) if isinstance(value,bool) else value)+'\n'
    (out/'case.txt').write_text(text)
    command=[str(binary),str(out/'case.txt'),str(out/'trace')];write_json(out/'command.json',command)
    with (out/'native_summary.json').open('w') as stdout,(out/'stderr.log').open('w') as stderr:
        subprocess.run(command,stdout=stdout,stderr=stderr,check=True)
    summary=json.loads((out/'native_summary.json').read_text())
    if summary['settings']!=profile['settings']:raise ValueError('runtime DDP settings differ from frozen profile')
    records=[json.loads(line) for line in (out/'trace.jsonl').read_text().splitlines()]
    initial=records[0]
    if not np.array_equal(initial['X'],seed['X']) or not np.array_equal(initial['U'],U):
        raise ValueError('DDP did not receive the shared seed')
    seed_cost=float(np.sum(np.asarray(seed['durations_s'])*(1+1e-5*np.sum(np.asarray(seed['U'])**2,axis=1))))
    if not np.isclose(summary['initial_cost'],seed_cost,rtol=0,atol=1e-12):raise ValueError('DDP objective differs from shared physical objective')
    write_json(out/'seed_received.json',seed)
    last=records[-1];controls=np.asarray(last['U'])
    return dict(X=last['X'],U=controls[:,:3].tolist(),
        durations_s=(seed['durations_s'] if fixed else controls[:,3].tolist()),
        numerical_intervals=[(c+1)*protocol['physical_budget_per_interval'] for c in summary['normalized_interval_rows']],
        native_status=summary['termination'],native_optimality=summary['optimality'])


def build_scp(protocol,job):
    os.environ.setdefault('JAX_PLATFORMS','cpu');os.environ.setdefault('OMP_NUM_THREADS','1')
    os.environ.setdefault('XDG_CACHE_HOME','/tmp/openscvx-matched-calibration-cache')
    sys.path.insert(0,str(SCP));import openscvx as ox
    import diffrax
    N=job['N'];n=N+1;seed=job['seed'];cfg=protocol['profiles']['lifted_openscvx']
    X=np.asarray(seed['X']);U=np.asarray(seed['U']);dt=np.asarray(seed['durations_s']);tf=float(sum(dt))
    pos=ox.State('position',(3,));vel=ox.State('velocity',(3,));q=ox.State('objective',(1,));y=ox.State('y',(1,))
    for state,bound in [(pos,12000.),(vel,1000.)]:
        state.min=[-bound]*3;state.max=[bound]*3;state.scaling_min=[-10.]*3;state.scaling_max=[10.]*3
    pos.initial=X[0,:3];pos.final=job['case']['terminal_state'][:3];pos.guess=X[:,:3]
    vel.initial=X[0,3:];vel.final=job['case']['terminal_state'][3:];vel.guess=X[:,3:]
    q.min=[0.];q.max=[100.];q.scaling_min=[0.];q.scaling_max=[10.]
    q.initial=[0.];q.final=[ox.Minimize(tf)]
    q.guess=np.r_[0.,np.cumsum(dt*(1.+1e-5*np.sum(U*U,axis=1)))][:,None]
    y.min=[-1000.];y.max=[1000.];y.scaling_min=[-1.];y.scaling_max=[1.]
    y.initial=[0.];y.final=[ox.Free(0.)];y.guess=np.zeros((n,1))
    a=ox.Control('acceleration',(3,),parameterization='ZOH')
    a.min=[-1.5*9.81,-1.5*9.81,0.];a.max=[1.5*9.81]*3;a.guess=np.vstack([U,U[-1]])
    fixed=job['mode']=='fixed';lo,hi=protocol['free_tf_bounds_s']
    clock=ox.Time(initial=0.,final=tf if fixed else ox.Free(tf),min=0.,max=hi,
        guess=np.r_[0.,np.cumsum(dt)][:,None],time_dilation_min=tf if fixed else lo,
        time_dilation_max=tf if fixed else hi,time_dilation_guess=np.r_[N*dt,N*dt[-1]][:,None])
    factor=1./(2.*np.sqrt(protocol['physical_budget_per_interval']))
    obs=factor*(2.-ox.linalg.Norm(pos-np.asarray(protocol['obstacle']['center']))+protocol['tightening_m']['obstacle'])
    glide=factor*(ox.Sqrt(pos[0]**2+pos[1]**2+1e-12)-pos[2]+protocol['tightening_m']['glide_residual'])
    density=ox.Square(ox.PositivePart(obs))+ox.Square(ox.PositivePart(glide))
    constraints=[(ox.linalg.Norm(a)<=1.5*9.81).convex(),
                 (ox.linalg.Norm(a[:2])<=np.tan(np.deg2rad(20.))*a[2]).convex()]
    constraints += [(y.at(k+1)-y.at(k)<=1.).convex() for k in range(N)]
    problem=ox.Problem(states=[pos,vel,q,y],controls=[a],time=clock,N=n,float_dtype='float64',
        dynamics={'position':vel,'velocity':a-np.array([0.,0.,9.81]),
                  'objective':1.+1e-5*(a@a),'y':density},constraints=constraints,
        algorithm={'k_max':cfg['max_iter'],'lam_cost':cfg['lam_cost'],'lam_prox':cfg['lam_prox'],
                   'lam_vc':cfg['lam_vc'],'ep_tr':cfg['ep_tr'],'ep_vc':cfg['ep_vc'],
                   'ep_vb':cfg['ep_vb'],'autotuner':ox.AugmentedLagrangian(lam_cost_relax=1.)},
        discretizer={'type':'VectorizeDiscretizeLinearize','ode_solver':'Dopri8','diffrax_kwargs':{
            'rtol':cfg['ode_tol'],'atol':cfg['ode_tol'],'num_substeps':30,
            'stepsize_controller':diffrax.PIDController(rtol=cfg['ode_tol'],atol=cfg['ode_tol'],
                dtmax=1./(N*cfg['max_step_divisor']))}},
        solver={'cvx_solver':'CLARABEL','solver_args':{'tol_gap_abs':cfg['cvx_tolerance'],
            'tol_gap_rel':cfg['cvx_tolerance'],'tol_feas':cfg['cvx_tolerance'],
            'max_iter':cfg['cvx_max_iter'],'ignore_dpp':True}})
    return problem


def weights(state):
    return {name:np.asarray(getattr(state,name)).tolist() for name in
            ('lam_prox','lam_vc','lam_cost','lam_vb_nodal','lam_vb_cross')}


def scp_run(protocol,job,out):
    problem=build_scp(protocol,job);problem.initialize()
    slices=problem.slices;state=problem.state;seed=job['seed'];N=job['N'];B=protocol['physical_budget_per_interval']
    def physical(state):
        x=np.asarray(state.x);u=np.asarray(state.u)
        return np.c_[x[:,slices['position']],x[:,slices['velocity']]],u[:-1,slices['acceleration']],u[:-1,slices['_time_dilation']].ravel()/N
    X,U,dt=physical(state)
    if not np.array_equal(X,seed['X']) or not np.array_equal(U,seed['U']) or not np.allclose(dt,seed['durations_s'],rtol=0,atol=1e-15):
        raise ValueError('SCP did not receive the shared physical seed')
    write_json(out/'seed_received.json',dict(X=X.tolist(),U=U.tolist(),durations_s=dt.tolist()))
    write_json(out/'initial_weights.json',weights(state))
    result=problem.solve();state=problem.state;X,U,dt=physical(state)
    write_json(out/'final_weights.json',weights(state))
    history={}
    for field in dataclasses.fields(problem.history):
        if field.name=='discretizations':continue
        value=getattr(problem.history,field.name)
        try:history[field.name]=np.asarray(value)
        except ValueError:pass
    np.savez_compressed(out/'native_history.npz',**history)
    rawX,rawU=np.asarray(state.x),np.asarray(state.u);np.savez_compressed(out/'native_final.npz',X=rawX,U=rawU)
    write_json(out/'slices.json',{k:[v.start,v.stop] for k,v in slices.items()})
    ycol=slices['y'].start;increments=np.diff(rawX[:,ycol])*B
    # Distinct from optimized increments: invoke the actual numerical map.
    import jax
    numerical=problem.discretizer.get_solver(problem.lowered.dynamics,problem.settings)
    propagated=np.asarray(jax.jit(numerical)(rawX,rawU,problem._parameters)[3])
    native=(propagated[:,ycol]-rawX[:-1,ycol])*B
    return dict(X=X.tolist(),U=U.tolist(),durations_s=dt.tolist(),
        optimized_accumulator_increments=increments.tolist(),numerical_intervals=native.tolist(),
        accumulator_defects=(increments-native).tolist(),native_status='converged' if result.converged else 'iteration_limit',
        native_J_tr=float(state.J_tr),native_J_vc=float(state.J_vc),native_J_vb=float(state.J_vb))


def audit(protocol,job,policy):
    X=np.asarray(policy['X']);U=np.asarray(policy['U']);dt=np.asarray(policy['durations_s']);N=job['N']
    if X.shape!=(N+1,6) or U.shape!=(N,3) or dt.shape!=(N,) or not all(np.all(np.isfinite(a)) for a in (X,U,dt)) or np.any(dt<=0):
        return dict(status='fail_invalid_policy')
    B=protocol['physical_budget_per_interval'];margins=tuple(protocol['tightening_m'][k] for k in ('obstacle','glide_residual'))
    propagated=job['case']['initial_state'];items=[];gaps=[]
    for k,(u,duration) in enumerate(zip(U,dt)):
        local=certify(X[k],u,duration,B,margins=margins)
        cert=certify(propagated,u,duration,B,margins=margins)
        reference,error=reference_integral(propagated,u,duration,margins,np.array(protocol['obstacle']['center']),2.,1.)
        next_local=np.asarray(flow(X[k],u,duration),float);gaps.append(float(np.max(abs(next_local-X[k+1]))))
        difference=interval_difference(X[k],propagated,duration)
        native=policy['numerical_intervals'][k]
        extra={}
        if 'optimized_accumulator_increments' in policy:
            extra=dict(accumulator_increment=policy['optimized_accumulator_increments'][k],native_integral=native)
        lower_bound=protocol['fixed_tf_s']/N if job['mode']=='fixed' else protocol['free_tf_bounds_s'][0]/N
        tight=(tightening(local,B,native,duration,margins,position_difference=difference,
                         duration_lower_bound=lower_bound,**extra)
               if duration>=lower_bound else {'status':'uncertified','reason':'duration below declared lower bound'})
        items.append(dict(interval=k,local=local,propagated=cert,reference_integral=reference,
            reference_error_estimate=error,physical_budget=B,numerical_integral=native,
            tightening=tight,input=certify_input(u)))
        propagated=flow(propagated,u,duration)
    def combine(values):return 'fail' if 'fail' in values else 'uncertified' if 'uncertified' in values else 'pass'
    terminal=float(np.max(abs(np.asarray(propagated,float)-job['case']['terminal_state'])))
    status=dict(physical=combine([r['propagated']['physical_status'] for r in items]),
        integral=combine([r['propagated']['integral_status'] for r in items]),
        input=combine([r['input']['status'] for r in items]),
        tightening=combine([r['tightening']['status'] for r in items]))
    time_error=abs(float(sum(dt))-protocol['fixed_tf_s']) if job['mode']=='fixed' else max(0.,protocol['free_tf_bounds_s'][0]-float(sum(dt)),float(sum(dt))-protocol['free_tf_bounds_s'][1],float(max(protocol['free_tf_bounds_s'][0]/N-dt)),float(max(dt-protocol['free_tf_bounds_s'][1]/N)))
    status['common_primal_pass']=all(status[k]=='pass' for k in ('physical','integral','input')) and terminal<=1e-8 and max(gaps)<=1e-8 and time_error<=1e-8
    return dict(status=status,objective=float(np.sum(dt*(1+1e-5*np.sum(U*U,axis=1)))),
        final_time_s=float(sum(dt)),terminal_error_inf=terminal,shooting_gap_inf=max(gaps),
        duration_bound_error=time_error,max_reference_integral_over_budget=max(r['reference_integral']/B for r in items),intervals=items,
        note='Budget/physical certificates are distinct from certified numerical-map error; tight integral-width accuracy remains a separate release check.')


def worker(root,job_name,method,binary):
    protocol=json.loads((root/'protocol.json').read_text());job=json.loads((root/(job_name+'.json')).read_text())
    if job['case']['id'] not in {c['id'] for c in protocol['calibration_cases']}:raise ValueError('test solves are not authorized by calibration manifest')
    if job['protocol_sha256']!=digest(protocol):raise ValueError('protocol hash changed')
    check_source_pins(protocol);out=root/(method+'_'+job_name);out.mkdir(exist_ok=False)
    try:
        policy=ddp_run(protocol,job,out,binary) if method=='folded_ddp' else scp_run(protocol,job,out)
        write_json(out/'policy.json',policy);write_json(out/'audit.json',audit(protocol,job,policy))
        write_json(out/'status.json',dict(status='completed',method=method))
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc());write_json(out/'status.json',dict(status='failed',method=method));raise


def calibrate(root,binary):
    if (root/'calibration_results.json').exists():raise ValueError('completed/partial attempt exists; use a new preparation directory')
    shutil.copy2(binary,root/'ddp_adapter');binary=root/'ddp_adapter'
    write_json(root/'ddp_binary.json',dict(source=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest()))
    manifest=json.loads((root/'manifest.json').read_text());results=[]
    for name in manifest['jobs']:
        for method in ['folded_ddp','lifted_openscvx']:
            command=[sys.executable,str(Path(__file__).resolve()),'--worker',str(root),name,method,str(binary)]
            with (root/(method+'_'+name+'.log')).open('w') as log:
                try:r=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=900);code=r.returncode
                except subprocess.TimeoutExpired:code='timeout'
            results.append(dict(job=name,method=method,exit_code=code));write_json(root/'calibration_results.json',results)
            print(name,method,'exit',code,flush=True)
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--prepare',type=Path);group.add_argument('--calibrate',type=Path)
    group.add_argument('--worker',nargs=4)
    parser.add_argument('--ddp-binary',type=Path)
    args=parser.parse_args()
    if args.prepare:print(prepare(args.prepare))
    elif args.calibrate:
        if not args.ddp_binary:parser.error('--calibrate requires --ddp-binary')
        calibrate(args.calibrate.resolve(),args.ddp_binary.resolve())
    else:worker(Path(args.worker[0]),args.worker[1],args.worker[2],Path(args.worker[3]))
