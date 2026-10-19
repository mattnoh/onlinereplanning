"""Matched local CTCS structure experiments; no trajectory optimizer campaigns.

Compare actual C++ rows/revision curvature and solve small controlled convex
subproblems to isolate elimination, accumulator metrics and virtual controls.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
import numpy as np


def evaluate(binary, matched):
    requests=[]; cases=[]
    for mode,rows in matched.items():
        for case in rows:
            requests.append(' '.join(map(str,[*case['state'],*case['acceleration'],case['duration'],
                1e-11,0.,0.,1.,8,int(mode=='fixed'),1.,2.85355,2.14645,5.5,2.])))
            cases.append((mode,case['case']))
    result=subprocess.run([str(binary)],input='\n'.join(requests)+'\n',capture_output=True,text=True,check=True)
    parsed=[json.loads(line) for line in result.stdout.splitlines()]
    if len(parsed)!=len(cases): raise ValueError('evaluator response count')
    return dict(zip(cases,parsed))


def spectrum(matrix):
    H=np.asarray(matrix); eigen=np.linalg.eigvalsh((H+H.T)/2)
    cutoff=max(1e-25,np.max(abs(eigen))*1e-10)
    return {'eigenvalues':eigen.tolist(),'rank':int(sum(eigen>cutoff)), 'cutoff':float(cutoff)}


def convex_experiment(metric=0., virtual_penalty=None, density_scale=1.):
    """Same physical quadratic model, exactly equivalent lifted/condensed forms.

    Toy dimensionless rows deliberately exclude ODE/quadrature/initialization
    differences. This proves a mechanism, not full-solver performance.
    """
    import cvxpy as cp
    A=np.array([[1.,.2],[.3,1.],[.7,-.4]])
    r=np.array([.05,-.1,.02]); H=np.diag([1.,2.]); gradient=np.array([-.5,.3])
    C=np.tril(np.ones((3,3))); D=density_scale
    def solve(lifted):
        d=cp.Variable(2); constraints=[]; obj=.5*cp.quad_form(d,H)+gradient@d
        v=cp.Variable(3) if virtual_penalty is not None else np.zeros(3)
        if lifted:
            dy=cp.Variable(4); constraints+=[dy[0]==0.,dy[1:]-dy[:-1]==D*(A@d+v)]
            constraints+=[D*(1+r)+dy[1:]-dy[:-1]<=D]
            obj+=metric*cp.sum_squares(dy[1:])
        else:
            constraints+=[r+A@d+v<=0.]
            obj+=metric*cp.sum_squares(D*C@(A@d+v))
        if virtual_penalty is not None:
            # Original nu is in scaled accumulator units D*v (S_y=1).
            obj+=virtual_penalty*D*cp.norm1(v)
        problem=cp.Problem(cp.Minimize(obj),constraints)
        problem.solve(solver='CLARABEL',tol_gap_abs=1e-11,tol_gap_rel=1e-11,tol_feas=1e-11,max_iter=300)
        if problem.status!='optimal': raise RuntimeError(problem.status)
        actual_v=np.zeros(3) if virtual_penalty is None else np.asarray(v.value)
        return {'step':d.value.tolist(),'objective':float(problem.value),'virtual_control_physical':actual_v.tolist(),
                'linearized_true_integral_row':(r+A@d.value).tolist(),
                'increment_row':(r+A@d.value+actual_v).tolist()}
    folded,lifted=solve(False),solve(True)
    difference=float(np.max(abs(np.array(folded['step'])-lifted['step'])))
    if difference>2e-6: raise AssertionError(('condensation mismatch',difference))
    return {'metric':metric,'virtual_penalty':virtual_penalty,'density_scale':D,
            'folded_condensed':folded,'lifted':lifted,'step_difference':difference,
            'condensed_accumulator_metric_hessian':(2*metric*D**2*A.T@C.T@C@A).tolist()}


def grazing_flatness():
    """True sphere integral and shape derivative; shared by folded/lifted maps."""
    from scipy.integrate import quad
    rows=[]; R=2.; speed=2.
    for h in (1e-2,1e-3,1e-4,1e-5,1e-6):
        width=np.sqrt(2*R*h-h*h)/speed
        def integrand(t):
            distance=np.sqrt((R-h)**2+(speed*t)**2)
            return (R-distance)**2/R**2
        def derivative(t):
            distance=np.sqrt((R-h)**2+(speed*t)**2)
            return 2*(R-distance)*(R-h)/(R**2*distance)
        integral=quad(integrand,-width,width,epsabs=1e-28,epsrel=1e-10)[0]
        gradient=quad(derivative,-width,width,epsabs=1e-25,epsrel=1e-10)[0]
        rows.append({'residual_height_m':h,'true_integral':integral,'true_dI_dheight':gradient,
                     'support_duration_s':2*width})
    for previous,current in zip(rows[:-1],rows[1:]):
        current['observed_integral_log_slope']=float(np.log(current['true_integral']/previous['true_integral'])/np.log(.1))
        current['observed_gradient_log_slope']=float(np.log(current['true_dI_dheight']/previous['true_dI_dheight'])/np.log(.1))
    return rows


def stopping_metric_probe(scp_root):
    """Evaluate pinned source's actual J_vc assignments on a controlled slack.

    CVX dynamics put nu in already scaled coordinates. Extract metric AST
    statements verbatim so the probe follows the pinned implementation.
    """
    import cvxpy as cp
    import jax
    jax.config.update("jax_enable_x64",True)
    import jax.numpy as jnp
    import sys
    sys.path.insert(0,str(scp_root))
    from openscvx.algorithms.base import Algorithm
    source=scp_root/'openscvx/algorithms/scvx/iteration.py'
    tree=ast.parse(source.read_text()); statements=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('VC','J_vc') for t in node.targets):
            statements.append(node)
    code=compile(ast.Module(body=sorted(statements,key=lambda n:n.lineno),type_ignores=[]),str(source),'exec')
    records=[]
    for scale in (1.,10.,100.):
        nu=cp.Variable(1)
        # Exactly the pinned CVX dynamics equation, with forced one-unit defect.
        problem=cp.Problem(cp.Minimize(cp.norm1(nu)),[np.array([[1/scale]])@np.array([1.])==nu])
        problem.solve(solver='CLARABEL',tol_gap_abs=1e-12,tol_feas=1e-12,tol_gap_rel=1e-12)
        environment={'jnp':jnp,'inv_S_x':jnp.array([[1/scale]]),'solution':SimpleNamespace(nu=jnp.array([nu.value]))}
        exec(code,environment)
        metric=float(environment['J_vc']); canonical=float(np.sum(abs(nu.value)))
        state=SimpleNamespace(J_tr=0.,J_vb=0.,J_vc=metric,ep_tr=1e-3,ep_vb=1e-3,ep_vc=.05)
        records.append({'state_scale':scale,'physical_defect':1.,'solver_scaled_nu':float(nu.value[0]),
                        'once_scaled_metric':canonical,'actual_source_metric':metric,
                        'actual_source_converged_at_ep_vc_005':bool(Algorithm.converged(None,state)),
                        'once_scaled_converged_at_ep_vc_005':canonical<.05})
    return {'source':str(source),'assignments':[ast.unparse(n) for n in statements], 'cases':records,
            'scope':'controlled metric-contract reproduction; not attribution of a historical stopping event'}


def saved_initial_accumulator_metrics(snapshot):
    """Measure initial proximal geometry on actual saved crossnode profiles.

    The extra metric ratio is its largest eigenvalue relative to uniform
    physical proximal weights, before imposing the physical dynamics rows.
    Thus it is a geometry bound, not a measured convergence effect.
    """
    import sys
    benchmark=Path('/home/lab/projects/solver/benchmark/openscvx_point_mass_ctcs')
    sys.path.insert(0,str(benchmark))
    from run_comparison import build
    records=[]
    for path in sorted(snapshot.glob('*_SCP_ctcs_N*.json')):
        audit=json.loads(path.read_text()); case=snapshot/'inputs'/audit['key']
        settings=json.loads((case/'settings.json').read_text())['args']
        if settings['budget']!='crossnode': continue
        problem=build(argparse.Namespace(**settings)); problem.settings.sim.__post_init__()
        Sx=np.diag(np.asarray(problem.settings.sim.S_x)); Su=np.diag(np.asarray(problem.settings.sim.S_u))
        slices=json.loads((case/'slices.json').read_text()); y_scale=float(Sx[slices['y'][0]])
        N=settings['N']; rownorm2=N/settings['fixed_tf'] if settings['mode']=='fixed' else 1.
        D=settings['density_scale']*rownorm2/y_scale
        physical_scales=np.r_[Sx[:6],Su[:3],Su[slices['_time_dilation'][0]]/N if settings['mode']=='free' else 0.]
        norms=np.array([np.linalg.norm(np.array(r['scp_jax_jacobian_physical'])*physical_scales)**2 for r in audit['intervals']])
        C=np.tril(np.ones((N,N))); gram=D*D*(C*norms)@C.T
        ratio=float(np.linalg.eigvalsh(gram)[-1])
        B=settings['eps']*(float(audit['intervals'][0]['duration']) if settings['mode']=='fixed' else 1.)
        records.append({'key':audit['key'],'initial_uniform_lam_prox':settings['lam_prox'],
            'maximum_added_accumulator_to_physical_proximal_metric_ratio':ratio,
            'accumulator_scale_S_y':y_scale,'effective_density_scale_in_physical_integral_units':D,
            'initial_virtual_control_cost_of_one_interval_budget':settings['lam_vc']*D*B,
            'weights_scope':'initial uniform configured weights only; adapted weights absent in historical arrays'})
    return records


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--snapshot',type=Path,required=True)
    ap.add_argument('--historical',type=Path,required=True)
    ap.add_argument('--current',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--scp-root',type=Path,default=Path('/home/lab/projects/solver/SCP/OpenSCvx'))
    args=ap.parse_args(); out=args.output; out.mkdir(parents=True,exist_ok=True)
    if (out/'results.json').exists(): raise FileExistsError('refuse to overwrite completed research')
    matched=json.loads((args.snapshot/'matched_cases.json').read_text())
    old,new=evaluate(args.historical,matched),evaluate(args.current,matched)
    curvature=[]
    for key in old:
        a,b=old[key],new[key]; J=np.array(b['jacobian']); H=np.array(b['unit_multiplier_hessian'])
        unit=J/np.linalg.norm(J) if np.linalg.norm(J)>0 else np.zeros_like(J)
        projection=np.eye(len(J))-np.outer(unit,unit)
        curvature.append({'mode':key[0],'case':key[1],'historical':a,'current':b,
            'value_difference':abs(a['row']-b['row']),
            'jacobian_difference':float(np.max(abs(np.array(a['jacobian'])-J))),
            'historical_ctcs_specific_curvature':spectrum(a['unit_multiplier_hessian']),
            'current_ctcs_specific_curvature':spectrum(H),
            'scalar_row_slack_outer_product':spectrum(np.outer(J,J)),
            'current_curvature_orthogonal_to_integral_gradient':spectrum(projection@H@projection)})
    settings=[(0.,None,1.),(.5,None,1.),(.5,None,10.),(.005,None,10.),(0.,.01,1.),(0.,10.,1.),(.5,.01,1.),(.5,.01,10.),(.005,.001,10.)]
    results={'scope':'source-pinned local optimization structure, not trajectory performance ranking',
             'curvature':curvature,'convex_experiments':[convex_experiment(*setting) for setting in settings],
             'scp_stopping_metric':stopping_metric_probe(args.scp_root),'grazing_flatness':grazing_flatness(),'saved_initial_accumulator_metrics':saved_initial_accumulator_metrics(args.snapshot)}
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    shutil.copy2(Path(__file__),out/Path(__file__).name)
    for name,src in [('historical',Path('/home/lab/projects/solver/DDP/ALIPDDP-ctstc-al')),
                     ('current',Path(__file__).resolve().parents[4]/'ALIPDDP-main'),('scp',args.scp_root)]:
        commit=subprocess.check_output(['git','-C',str(src),'rev-parse','HEAD'],text=True).strip()
        (out/(name+'_base_revision.txt')).write_text(commit+'\n')
    inventory={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*'))
               if p.is_file() and p.name!='sha256_inventory.json'}
    (out/'sha256_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    print('Matched C++ cases:',len(curvature))
    for row in curvature:
        if row['case']=='active': print(row['mode'],'GN rank',row['current_ctcs_specific_curvature']['rank'],
                                        'slack outer rank',row['scalar_row_slack_outer_product']['rank'],
                                        'orthogonal GN rank',row['current_curvature_orthogonal_to_integral_gradient']['rank'])
    print('Maximum condensed/lifted step difference:',max(r['step_difference'] for r in results['convex_experiments']))
    print('Stopping metric probe:',json.dumps(results['scp_stopping_metric']['cases']))


if __name__=='__main__': main()
