"""Extend the comparison's immutable saved-policy audit with validated bounds."""
import argparse
import csv
from fractions import Fraction as F
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import numpy as np
from point_mass_certificate import (q, flow, certify, certify_input, tightening, interval_difference,
                                   rounding_radius, rounding_flow_error, enclosure)


def numerical_integral(x,u,dt,steps,margins,obs,R,tangent,exact_stages=False):
    """Read-only reconstruction of the production RK4 density quadrature."""
    x=np.array(x,dtype=float); u=np.array(u,dtype=float)
    dt=float(dt)
    h=dt/steps
    def rhs(z): return np.r_[z[3:],u-[0,0,9.81]]
    def density(z):
        p=z[:3]
        c=[R-np.linalg.norm(p-obs)+margins[0],
           np.sqrt(p[0]**2+p[1]**2+1e-12)-tangent*p[2]+margins[1]]
        return sum(max(0.,v)**2 for v in c)/R**2
    total=0.
    for j in range(steps):
        k1=rhs(x); a=x+h*k1/2; k2=rhs(a)
        b=x+h*k2/2; k3=rhs(b); c=x+h*k3; k4=rhs(c)
        if exact_stages:
            stages=[np.asarray(flow(x,u,t),float) for t in (0,h/2,h/2,h)]
        else: stages=[x,a,b,c]
        total+=h*sum(w*density(z) for w,z in zip((1,2,2,1),stages))/6
        x=x+h*(k1+2*k2+2*k3+k4)/6
    return total


def reference_integral(x,u,dt,margins,obs,R,tangent):
    """Independent high-accuracy, root-split reference; not a certificate."""
    from scipy.integrate import quad
    from numpy.polynomial import Polynomial as P
    dt=float(dt)
    x,u=np.asarray(x,float),np.asarray(u,float)
    acc=u-[0,0,9.81]
    pos=[P([x[j],x[j+3],.5*acc[j]]) for j in range(3)]
    d2=sum((p-o)**2 for p,o in zip(pos,obs))
    r2=pos[0]**2+pos[1]**2+1e-12
    roots=[0.,dt]
    for poly in (d2.deriv(),d2-(R+margins[0])**2,
                 r2-(tangent*pos[2]-margins[1])**2,
                 r2.deriv()**2-4*tangent**2*pos[2].deriv()**2*r2):
        roots.extend(float(v.real) for v in poly.roots() if abs(v.imag)<1e-7 and 0<v.real<dt)
    def density(t):
        p=np.array([f(t) for f in pos])
        return (max(0.,R-np.linalg.norm(p-obs)+margins[0])**2+
                max(0.,np.sqrt(p[0]**2+p[1]**2+1e-12)-tangent*p[2]+margins[1])**2)/R**2
    return quad(density,0.,dt,points=sorted(set(roots)),epsabs=1e-22,epsrel=1e-10,limit=300)


def per_interval_errors(increment,native):
    return [float(a-b) for a,b in zip(increment,native)]


def native_reintegration(case, setting, raw, X, U):
    """Actual OpenSCvx Dopri8/JAX discretization, using returned arrays; no solve."""
    from run_comparison import build
    import jax
    cfg=argparse.Namespace(**setting)
    problem=build(cfg)
    problem.settings.sim.__post_init__()
    solver=problem._discretizer.get_solver(problem._lowered.dynamics,problem.settings)
    fn=jax.jit(solver)
    fn_value=jax.jit(lambda x,u: solver(x,u,problem._parameters)[3])
    result=fn(X,U,problem._parameters)
    A,B,C,prop=[np.asarray(r) for r in result[:4]]
    def col(k):
        if cfg.budget=='crossnode': return raw['y'][0]
        return raw['ctcs'][0 if cfg.budget=='total' else k]
    idx=[col(k) for k in range(cfg.N)]
    native=np.array([(prop[k,c]-X[k,c])/cfg.density_scale for k,c in enumerate(idx)])
    # Each perturbation changes only the interval being differentiated. The
    # vectorized adaptive integrator shares error control across intervals.
    jacs=np.empty((cfg.N,10)); sweeps={}
    for step in (1e-4,1e-5,1e-6,1e-7):
        step_fds=np.empty_like(jacs)
        for k,c in enumerate(idx):
            for j in range(10):
                xp,up=np.array(X),np.array(U); xm,um=np.array(X),np.array(U)
                if j<6:
                    xp[k,j]+=step; xm[k,j]-=step
                    jac=A[k,c,j]/cfg.density_scale
                else:
                    col_u=raw['acceleration'][0]+j-6 if j<9 else raw['_time_dilation'][0]
                    up[k,col_u]+=step; um[k,col_u]-=step
                    jac=B[k,c,col_u]/cfg.density_scale
                pp=np.asarray(fn_value(xp,up)); pm=np.asarray(fn_value(xm,um))
                fd=(pp[k,c]-pm[k,c])/(2*step*cfg.density_scale)
                if j==9: fd*=cfg.N; jac*=cfg.N
                step_fds[k,j]=fd; jacs[k,j]=jac
        sweeps[str(step)]=step_fds.tolist()
    fds=np.asarray(sweeps['1e-06'])
    jax.clear_caches()
    return native, jacs, fds, sweeps


def matched_cases(out, evaluator, native_scp):
    """Identical physical inputs for DDP, actual SCP/JAX, and root-split references."""
    template=out/'inputs/free_SCP_ctcs_N6'
    setting=json.loads((template/'settings.json').read_text())['args']
    raw=json.loads((template/'slices.json').read_text())
    obs=np.array([2.85355,2.14645,5.5]); R=2.
    normal=np.r_[obs[:2]/np.linalg.norm(obs[:2]),0.]
    tangent=np.array([-normal[1],normal[0],0.])
    center=obs-1.9999*normal
    cases=[('inactive',np.array([0,0,12.]),np.zeros(3)),
           ('active',obs-normal,np.zeros(3)),
           ('grazing',obs-2*normal,np.zeros(3)),
           ('boundary_crossing',obs-2.1*normal,normal),
           ('narrow_positive',center-1.02*tangent,2*tangent),
           ('inactive_moving',np.array([0,0,12.]),np.array([.1,0,0]))]
    results={}
    for mode in ('fixed','free'):
        cfg=dict(setting,mode=mode,N=6,fixed_tf=6.,tighten_obs=0.,tighten_glide=0.)
        X=np.zeros((7,10)); U=np.tile([0.,0.,9.81,6.],(7,1))
        for k,(_,p,v) in enumerate(cases): X[k,:6]=np.r_[p,v]
        native=jacs=fds=None; sweeps={}
        if native_scp: native,jacs,fds,sweeps=native_reintegration(template,cfg,raw,X,U)
        rows=[]; requests=[]
        for k,(label,_,_) in enumerate(cases):
            x=X[k,:6]; u=U[k,:3]; ref,err=reference_integral(x,u,1.,(0,0),obs,R,1.)
            row={'case':label,'state':x.tolist(),'acceleration':u.tolist(),'duration':1.,
                 'reference_integral':ref,'reference_error_estimate':err,
                 'certificate':certify(x,u,1.,1e-11,obstacle=obs),
                 'scp_native_integral':None if native is None else float(native[k]),
                 'scp_jax_derivative':None if jacs is None else jacs[k].tolist(),
                 'scp_fd_step_sweep':{step:values[k] for step,values in sweeps.items()}}
            # True-integral finite differences expose discretization error as well.
            true_fd=[]
            for j in range(10):
                xp,xm=x.copy(),x.copy(); up,um=u.copy(),u.copy(); hp=hm=1.; h=1e-5
                if j<6: xp[j]+=h; xm[j]-=h
                elif j<9: up[j-6]+=h; um[j-6]-=h
                else: hp+=h; hm-=h
                true_fd.append((reference_integral(xp,up,hp,(0,0),obs,R,1.)[0]-reference_integral(xm,um,hm,(0,0),obs,R,1.)[0])/(2*h))
            row['true_integral_fd_derivative']=true_fd
            requests.append(' '.join(map(str,[*map(float,x),*map(float,u),1.,1e-11,0.,0.,1.,8,int(mode=='fixed'),1.,*obs,R])))
            rows.append(row)
        if evaluator:
            proc=subprocess.run([str(evaluator)],input='\n'.join(requests)+'\n',text=True,capture_output=True,check=True)
            for row,line in zip(rows,proc.stdout.splitlines()): row['ddp']=json.loads(line)
        results[mode]=rows
    (out/'matched_cases.json').write_text(json.dumps(results,indent=2)+'\n')


def write_report(out, summary, records):
    lines=['# Folded DDP / lifted SCP: saved nominal point-mass certificate', '',
           'No optimization or production quadrature changes. Existing selection gates and plots are unchanged.', '',
           'The exact lifted accumulator eliminates to the folded inequality when the flow, density, time measure and budgets match. Folding adds no mathematical relaxation.', '',
           '| Enforcement | DDP | SCP |', '|---|---|---|',
           '| Representation | Nonlinear interval integral row | Accumulator dynamics and convex increment row |',
           '| Evaluation | Fixed RK4 stages | Adaptive Dopri8 augmented dynamics |',
           '| Algorithm | Primal-dual slack/dual updates, backward/forward passes, line search | Linearized augmented dynamics, penalized virtual controls; nonconvex stage rows use virtual buffers |',
           '| Error channels | Integral approximation and physical shooting gaps | Those channels plus accumulator defect |',
           '| Scaling | Historical point mass I-epsilon; paper MPC I/epsilon-1 | Density scaling multiplies dynamics and increment budget |',
           '| Stopping | Revision-dependent optimality, primal, schedule checks | Scaled step, virtual controls and buffers |', '',
           'Fixed-time mean-square rows use physical budget epsilon times duration. Free-time rows use epsilon. All derivatives and defects below undo density/row scaling; duration derivatives use dI/dduration = N dI/ds.', '',
           'Physical path, ZOH input, integral and tightening classifications are separate. Strict zero input satisfaction is distinguished from the saved tolerance-based base gate; terminal checks, physical shooting gaps and termination flags remain separate diagnostics. A tightening test failure means the configured margin does not meet the reported conservative sufficient bound; it does not establish physical penetration. Unknown adapted stage-buffer penalties remain explicitly unrecorded.', '',
           '| Policy | Strict path | Strict ZOH inputs | Integral budget | Sufficient tightening |', '|---|---|---|---|---|']
    for record in records:
        lines.append('| {key} | {physical_path_status} | {strict_input_status} | {integral_status} | {tightening_status} |'.format(**record))
    n6=next(r for r in records if r['key']=='free_DDP_ctcs_N6')['intervals'][3]
    bounds=n6['propagated']['integral']
    lines+=['', '## N=6 reproduction', '',
            f"Interval 3 has independently replayed integral {n6['reference_integrals']['propagated']['value']:.12g} against 1e-11. The rigorous propagated enclosure is [{bounds['lower']:.12g}, {bounds['upper']:.12g}]. Physical clearance is certified separately.", '',
            '| Substeps | Actual C++ integral | Exact flow at the same stage times |', '|---|---|---|']
    for row in n6['quadrature']:
        lines.append(f"| {row['substeps']} | {row.get('cpp_integral',float('nan')):.12g} | {row['exact_flow_at_stage_times']:.12g} |")
    lines+=['', 'The C++ evaluator reproduces the Python RK4 reconstruction. Both 4/8 grids miss the positive pocket, including when stage states are replaced by exact-flow samples. At 16 substeps the substantial difference between the two maps also exposes intermediate RK4 stage error. Exact RK4 endpoints do not imply exact intermediate stage samples. These results isolate numerical evaluation of the recorded policy; they do not reconstruct its optimizer history.', '',
            '## Derivatives and accumulator residuals', '',
            'Each CTCS policy JSON includes local and propagated rational bounds, root-split reference integrals, actual C++ row/cache values and analytic/finite-difference derivatives. SCP additionally exports optimized accumulator increments, native Dopri8 increments, per-interval accumulator defects and JAX derivatives with four finite-difference steps. Perturbations change one interval at a time. Adaptive numerical-map derivatives need not agree across changing integration branches; discrepancies are evidence, not suppressed failures.', '',
            'matched_cases.json evaluates identical inactive, active, near-grazing, boundary-crossing and narrow-positive inputs using both implementations and the true-integral reference. Inactive dimensions and duration coordinates are retained.', '',
            '## Certificate and limits', '',
            'Enclosures use exact rational Bernstein polynomial bounds and integer-square-root brackets on exact constant-acceleration flow. Largest-uncertainty bisection stops when the budget and physical decisions are proved, or integral width is at most 1e-4 of the budget; the limits are 100,000 subdivisions and depth 60. Every enclosure includes a root-split high-accuracy reference (the reference itself is not a certificate). Exhaustion and ambiguous zero contacts remain uncertified.', '',
            'For each constraint, the report computes a conservative speed/derivative bound, effective budget (row residual, certified evaluation error, plus lifted accumulator defect), sufficient margin and reserve in metres. The small-budget condition uses the minimum recorded interval duration. Propagated local-to-global position error is included once as the residual error E. No margins, budgets or controls are changed.', '',
            'Binary SCP policies retain exact rational duration s/N. CSV values certify the serialized doubles; conditional original-control enclosures also include the documented 12-significant-digit export rounding, decimal conversion and accumulated propagation error. These conditional enclosures do not certify unknown pre-export controls without that rounding assumption.', '',
            '## Attribution', '',
            'The source archives capture actual working-tree files and separately record base commits and dirty status. The historical point-mass checkout is not represented as the paper MPC revision, and neither is claimed to reconstruct an unknown historical build. Saved OpenSCvx commit attribution is retained per policy. Binary hashes, build caches, saved inputs and executed audit sources are inventoried.', '']
    for label,meta in summary['provenance'].items():
        lines.append(f"- {label}: {meta['commit']} ({meta['tracked_changes']} tracked working-tree changes; see {meta['working_tree_status_file']}).")
    lines+=['', 'The certificate covers the recorded nominal point-mass model. MPC reset/shift/tail behavior, nonlinear landing guarantees and flight tracking uncertainty remain outside this gate.', '']
    (out/'REPORT.md').write_text('\n'.join(lines))


def audit_snapshot(snapshot, destination, evaluator, native_scp=False, max_subdivisions=100000):
    from run_comparison import P0,V0,OBS,RADIUS,segment
    out=Path(destination).resolve(); out.mkdir(parents=True,exist_ok=False)
    source=out/'source'; source.mkdir(); records=[]
    snapshot=Path(snapshot).resolve()
    # Snapshot only saved selections, never live pools. Copy exact selected inputs.
    shutil.copytree(snapshot/'inputs',out/'inputs')
    shutil.copy2(snapshot/'audit.json',out/'prior_audit.json')
    for name in ('point_mass_compare_fair.py','point_mass_certificate.py','point_mass_saved_audit.py',
                 'point_mass_saved_policy.cpp','test_point_mass_certificate.py'):
        shutil.copy2(Path(__file__).parent/name,source/name)
    shutil.copytree(Path(__file__).parent/'fixtures',source/'fixtures')
    if evaluator:
        evaluator=Path(evaluator).resolve(); shutil.copy2(evaluator,source/evaluator.name)
    provenance={}
    roots={'historical_point_mass':'/home/lab/projects/solver/DDP/ALIPDDP-ctstc-al',
           'paper_mpc':str(Path(__file__).resolve().parents[4]/'ALIPDDP-main'),
           'openscvx':'/home/lab/projects/solver/SCP/OpenSCvx'}
    for label,path in roots.items():
        root=Path(path)
        commit=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
        status=subprocess.check_output(['git','-C',str(root),'status','--porcelain'],text=True)
        (source/f'{label}_git_status.txt').write_text(status)
        provenance[label]={'path':str(root),'commit':commit,'working_tree_status_file':f'source/{label}_git_status.txt',
                           'tracked_changes':sum(not line.startswith('??') for line in status.splitlines())}
        with tarfile.open(source/f'{label}_source.tar.gz','w:gz') as archive:
            # All source/header files, including tracked working-tree differences.
            for relative in subprocess.check_output(['git','-C',str(root),'ls-files'],text=True).splitlines():
                file=root/relative
                if file.is_file() and file.suffix in ('.h','.hpp','.tpp','.cpp','.py','.txt','.toml'):
                    archive.add(file,arcname=relative)
    historical=Path(roots['historical_point_mass'])
    for path in (historical/'build/CMakeCache.txt',historical/'build/point_mass_ctcs_compare',
                 Path('/home/lab/mavros_ws/build/comando_planner/CMakeCache.txt')):
        if path.exists(): shutil.copy2(path,source/(('historical_' if historical in path.parents else 'diagnostic_')+path.name))
    shutil.copy2(Path(__import__('run_comparison').__file__),source/'run_comparison.py')
    original=[r for r in json.loads((snapshot/'audit.json').read_text())['policies']]
    for prior in original:
        key=prior['key']; case=out/'inputs'/key
        row=json.loads((case/'selected_row.json').read_text()); method=prior['method']
        mode=prior['mode']; N=int(row['N']); ctcs=prior['arm']=='ctcs'
        rounding=None; Xraw=Uraw=raw=setting=None
        if method=='DDP':
            rr=list(csv.DictReader((case/'nodes.csv').open()))
            X=np.array([[float(r[k]) for k in ('px','py','pz','vx','vy','vz')] for r in rr])
            U=np.array([[float(r[k]) for k in ('ux','uy','uz')] for r in rr[:-1]])
            dt=np.array([float(r['theta']) for r in rr[:-1]])
            obs=np.array([float(row[k]) for k in ('obs_x','obs_y','obs_z')]); R=float(row['obs_r'])
            tangent=float(row['glide_tan'])
            rounding=rr
            row_norm=float(row['row_norm'])
        else:
            setting=json.loads((case/'settings.json').read_text())['args']
            raw=json.loads((case/'slices.json').read_text()); z=np.load(case/'native_arrays.npz')
            Xraw=z['x'] if 'x' in z else z['X'][-1]; Uraw=z['u'] if 'u' in z else z['U'][-1]
            X=Xraw[:,:6]; U=Uraw[:-1,slice(*raw['acceleration'])]
            dt=np.array([q(s)/N for s in Uraw[:-1,slice(*raw['_time_dilation'])].ravel()],dtype=object)
            obs=OBS; R=RADIUS; tangent=1.
            row_norm=np.sqrt(N/setting['fixed_tf']) if mode=='fixed' else 1.
        eps=float(row['eps']) if ctcs else 1e-11
        margins=(float(row['tighten_obs']),float(row['tighten_glide']))
        native=jacs=fds=None; sweeps={}; native_error=None
        if method=='SCP' and ctcs and native_scp:
            try: native,jacs,fds,sweeps=native_reintegration(case,setting,raw,Xraw,Uraw)
            except Exception as exc: native_error=f'{type(exc).__name__}: {exc}'
        replay=list(map(q,np.r_[P0,V0])); dx=[F(0)]*6
        intervals=[]; requests=[]; request_meta=[]
        for k,(u,h) in enumerate(zip(U,dt)):
            B=q(eps)*(q(h) if mode=='fixed' else 1)
            kwargs=dict(margins=margins,obstacle=obs,radius=R,glide_tan=tangent,max_subdivisions=max_subdivisions)
            local=certify(X[k],u,h,B,**kwargs)
            propagated=certify(replay,u,h,B,**kwargs)
            E=interval_difference(X[k],replay,h)
            references={}
            for name,start in (('local',X[k]),('propagated',replay)):
                val,err=reference_integral(start,u,h,margins,obs,R,tangent)
                bounds=(local if name=='local' else propagated)['integral']
                contains=bounds['lower']<=val+err+1e-22 and bounds['upper']>=val-err-1e-22
                if not contains: raise AssertionError((key,k,name,bounds,val,err))
                references[name]={'value':val,'quadrature_error_estimate':err,'contained_by_certificate':contains}
            rec={'k':k,'input_certificate':certify_input(u,cone_tangent=float(np.tan(np.deg2rad(20.)))), 'reference_integrals':references,'duration':float(h),'exact_duration_rational':str(q(h)),'physical_budget':enclosure(B,B),'local':local,'propagated':propagated,
                 'local_to_propagated_position_bound_m':enclosure(E,E),
                 'shooting_gap':float(np.max(abs(np.asarray(flow(X[k],u,h),float)-X[k+1])))}
            if ctcs:
                steps=max(1,int(row['substeps'])) if method=='DDP' else max(10,int(setting['max_step_divisor']))
                Q=numerical_integral(X[k],u,h,steps,margins,obs,R,tangent)
                rec['reconstructed_ddp_integral']=Q
                if method=='SCP':
                    c=raw['y'][0] if setting['budget']=='crossnode' else raw['ctcs'][0 if setting['budget']=='total' else k]
                    increment=float((Xraw[k+1,c]-Xraw[k,c])/setting['density_scale'])/(row_norm**2)
                    nat=None if native is None else float(native[k])/(row_norm**2)
                    rec['optimized_accumulator_increment']=increment
                    rec['native_numerical_increment']=nat
                    rec['accumulator_defect']=None if nat is None else per_interval_errors([increment],[nat])[0]
                    rec['increment_row_residual']=increment-float(B)
                    rec['native_minus_certified_local']=None if nat is None else {
                        'lower':nat-local['integral']['upper'],'upper':nat-local['integral']['lower']}
                    rec['scp_jax_jacobian_physical']=None if jacs is None else (jacs[k]/row_norm**2).tolist()
                    rec['scp_fd_jacobian_physical']=None if fds is None else (fds[k]/row_norm**2).tolist()
                    rec['scp_fd_step_sweep_physical']={step:(np.asarray(values[k])/row_norm**2).tolist() for step,values in sweeps.items()}
                    rec['tightening']=tightening(local,B,Q,h,margins,E,increment,nat,R,tangent,duration_lower_bound=min(dt))
                else:
                    rec['tightening']=tightening(local,B,Q,h,margins,E,radius=R,glide_tan=tangent,duration_lower_bound=min(dt))
                resolutions=(4,8,16,32,64,128) if (mode=='free' and method=='DDP' and N==6) else (steps,)
                rec['quadrature']=[]
                for n in resolutions:
                    reconstructed=numerical_integral(X[k],u,h,n,margins,obs,R,tangent)
                    exact=numerical_integral(X[k],u,h,n,margins,obs,R,tangent,True)
                    rec['quadrature'].append({'substeps':n,'python_rk4':reconstructed,'exact_flow_at_stage_times':exact})
                    requests.append(' '.join(map(str,[*map(float,X[k]),*map(float,u),float(h),eps,*margins,row_norm,n,int(mode=='fixed'),tangent,*map(float,obs),R])))
                    request_meta.append((rec['quadrature'][-1],row_norm,k,n))
            if rounding:
                names=('ux','uy','uz'); du=[rounding_radius(rounding[k][s]) for s in names]
                dh=rounding_radius(rounding[k]['theta'])
                dx,Eround=rounding_flow_error(replay,u,h,dx,du,dh)
                rec['csv_original_policy_conditional']=certify(replay,u,h,B,position_error=Eround,duration_error=dh,**kwargs)
                if mode=='fixed':
                    bounds=rec['csv_original_policy_conditional']['integral']
                    rec['csv_original_policy_conditional']['integral_status']=(
                        'pass' if F(bounds['upper_rational'])<=q(eps)*(q(h)-dh) else
                        'fail' if F(bounds['lower_rational'])>q(eps)*(q(h)+dh) else 'uncertified')
                rec['csv_original_policy_conditional']['assumption']='nearest rounding to 12 significant digits (recorded C++ setprecision); model constants and margins as recorded'
            intervals.append(rec); replay=flow(replay,u,h)
        if evaluator and requests:
            proc=subprocess.run([str(evaluator)],input='\n'.join(requests)+'\n',text=True,capture_output=True,check=True)
            results=[json.loads(line) for line in proc.stdout.splitlines()]
            if len(results)!=len(requests): raise RuntimeError('C++ evaluator response count mismatch')
            for (target,norm,k,n),result in zip(request_meta,results):
                cpp=(result['row']+eps)/(norm*norm)
                target['cpp']=result; target['cpp_integral']=cpp
                target['cpp_physical_jacobian']=[v/(norm*norm) for v in result['jacobian']]
                target['cpp_physical_fd_jacobian']=[v/(norm*norm) for v in result['finite_difference']]
                target['derivative_coordinates']=['px','py','pz','vx','vy','vz','ax','ay','az']+(['duration'] if mode=='free' else [])
                target['cpp_python_abs_difference']=abs(cpp-target['python_rk4'])
                if abs(cpp-target['python_rk4'])>max(1e-22,abs(cpp)*1e-8):
                    raise AssertionError((key,k,n,cpp,target['python_rk4']))
        def status(name,part='propagated'):
            vals=[r[part][name] for r in intervals]
            return 'fail' if 'fail' in vals else 'uncertified' if 'uncertified' in vals else 'pass'
        result={'key':key,'method':method,'evaluator_base_revision':provenance['historical_point_mass' if method=='DDP' else 'openscvx']['commit'],
                'saved_openscvx_commit':None if method=='DDP' else json.loads((case/'settings.json').read_text())['openscvx_commit'],
                'policy_values':'serialized CSV doubles' if rounding else 'returned binary arrays; fallback final history if returned arrays absent',
                'physical_path_status':status('physical_status'),
                'strict_input_status':('fail' if any(r['input_certificate']['status']=='fail' for r in intervals) else 'uncertified' if any(r['input_certificate']['status']=='uncertified' for r in intervals) else 'pass'),
                'physical_status':('fail' if status('physical_status')=='fail' or any(r['input_certificate']['status']=='fail' for r in intervals) else 'uncertified' if status('physical_status')=='uncertified' or any(r['input_certificate']['status']=='uncertified' for r in intervals) else 'pass'),
                'integral_status':status('integral_status') if ctcs else None,
                'tightening_status':('fail' if any(r['tightening']['status']=='fail' for r in intervals) else 'uncertified' if any(r['tightening']['status']=='uncertified' for r in intervals) else 'pass') if ctcs else None,
                'native_reintegration_error':native_error,'prior_acceptance_unchanged':prior,
                'stage_virtual_buffer_penalty':None if method!='SCP' else {'initial':0.,'adaptation':setting['autotuner'],'adapted':'not saved in historical arrays; cannot reconstruct from J_vb alone'},
                'intervals':intervals}
        (out/f'{key}.json').write_text(json.dumps(result,indent=2)+'\n'); records.append(result)
        print(key,result['physical_status'],result['integral_status'],result['tightening_status'],flush=True)
    matched_cases(out,evaluator,native_scp)
    import platform
    from importlib.metadata import version
    summary={'runtime':{'python':platform.python_version(),'packages':{name:version(name) for name in ('numpy','scipy','jax','diffrax')}},'provenance':provenance,'snapshot':str(snapshot),'native_scp_requested':native_scp,
             'scope':'recorded nominal point mass only; original acceptance thresholds and plots unchanged',
             'policies':[{k:r[k] for k in ('key','physical_status','physical_path_status','strict_input_status','integral_status','tightening_status','native_reintegration_error')} for r in records]}
    (out/'certification.json').write_text(json.dumps(summary,indent=2)+'\n')
    write_report(out,summary,records)
    inventory={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
    (out/'sha256_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    return summary
