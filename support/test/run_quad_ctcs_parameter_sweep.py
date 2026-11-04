import os,json,csv,itertools,subprocess,time,math,re,argparse
from pathlib import Path
parser=argparse.ArgumentParser(description='Resumable fixed-dt quad CTCS parameter grid; production defaults stay unchanged.')
parser.add_argument('--binary',required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
p=args.output;p.mkdir(parents=True,exist_ok=True)
binary=str(Path(args.binary).resolve())
base={k:v for k,v in os.environ.items() if not k.startswith(('SZMUK_','STC_','MPC_'))}
base.update(SZMUK_RH_N='80',SZMUK_N='80',SZMUK_TH0='0.1',SZMUK_THL='0.05',SZMUK_THH='0.12',SZMUK_MAX_ITER='1200',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',SZMUK_TEST_REPLAY_REPEATS='0')
axes={'budget':[1e-6,1e-5,1e-4,0.0016,0.016],'scale':[1.,10.],'reg2':[.01,.1,1.],'mu':[.01,1.],'mu_mul':[.1,.5]}
manifest={'axes':axes,'cadence_intervals':[1,4,16],'dt':.1,'horizon':80,'physical_tightening':'unchanged; default profile','tolerance':1e-6,'max_iter':1200,'forward_filter':1,'case_timeout_seconds':60,'max_simulation_time_seconds':20,'solver_code_changed':False,'base_environment':base,'binary':binary,'scope':'Existing fixed-duration quad CTCS test adapter; staging root; perfect matched plant; frozen solve time; no historical-baseline equivalence claim.'}
# Do not persist unrelated environment values.
manifest['base_environment']={k:v for k,v in base.items() if k.startswith(('SZMUK_','OMP_','OPENBLAS_'))}
(p/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
start=time.monotonic();records=[json.loads(s) for s in (p/'results.jsonl').read_text().splitlines()] if (p/'results.jsonl').exists() else [];done={r['id'] for r in records};counts={'completed':len(records),'native_pass':sum(r['native_pass'] for r in records),'physical_pass':sum(r['physical_pass'] for r in records),'timeouts':sum(r['timedout'] for r in records)}
for config_id,values in enumerate(itertools.product(*axes.values())):
 conf=dict(zip(axes,values));env=dict(base)
 env.update(SZMUK_CTCS_STEP_EPS=str(conf['budget']),SZMUK_CTCS_Y_SCALE=str(conf['scale']),SZMUK_REG2_MIN=str(conf['reg2']),SZMUK_TEST_MU=str(conf['mu']),SZMUK_TEST_MU_MUL=str(conf['mu_mul']))
 for k in [0,1,4,16]:
  name=f'{config_id:03d}-'+('single' if not k else f'k{k}');out=p/name
  if name in done:continue
  if out.exists():
   for f in out.iterdir():f.unlink()
   out.rmdir()
  command=[binary,'quad','matched',str(1 if not k else math.ceil(20/(.1*k))),str(out),'--ctcs','--staging-initial','--audit-loop','--fixed-dt','0.1']
  command+=['--single-horizon'] if not k else ['--replay-intervals',str(k)]
  t=time.monotonic();timedout=False
  with (p/(name+'.log')).open('w') as log:
   try:rc=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60).returncode
   except subprocess.TimeoutExpired:rc=124;timedout=True
  summary=json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else None
  cycles=[json.loads(s) for s in (out/'cycles.jsonl').read_text().splitlines()] if (out/'cycles.jsonl').exists() else []
  tail=(p/(name+'.log')).read_text()[-1200:]
  failure=re.findall(r'constraints=([0-9.eE+\-]+) dynamics=([0-9.eE+\-]+) termination=([^\n]+)',tail)
  initial_max=None
  if failure and (out/'ctcs_intervals.csv').exists():
   with (out/'ctcs_intervals.csv').open() as f:
    ratios=[float(r['native_integral_ratio']) for r in csv.DictReader(f) if int(r['cycle'])==len(cycles) and r['phase']=='initial_guess']
   initial_max=max(ratios) if ratios else None
  record={'id':name,'config':conf,'cadence_intervals':k,'returncode':rc,'timedout':timedout,'wall_seconds':time.monotonic()-t,'accepted':len(cycles),'sim_time':cycles[-1]['epoch'] if cycles else 0,'native_pass':bool(summary and summary['native_solve_pass']),'physical_pass':bool(summary and summary['physical_stc_pass']),'ctcs_pass':bool(summary and summary['ctcs_loop_pass']),'summary':summary,'failure':{'constraints':float(failure[-1][0]),'dynamics':float(failure[-1][1]),'termination':failure[-1][2],'initial_max_ctcs_ratio':initial_max} if failure else None,'last_output':tail if not summary else None}
  records.append(record)
  with (p/'results.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
  counts['completed']+=1;counts['native_pass']+=record['native_pass'];counts['physical_pass']+=record['physical_pass'];counts['timeouts']+=timedout
  progress={**counts,'total':480,'elapsed_seconds':time.monotonic()-start,'last_id':name,'last_accepted':len(cycles),'last_native_pass':record['native_pass'],'last_ctcs_pass':record['ctcs_pass']}
  (p/'progress.json').write_text(json.dumps(progress)+'\n')
  if counts['completed']%12==0 or record['ctcs_pass']:print(json.dumps(progress),flush=True)
(p/'complete.json').write_text(json.dumps({**counts,'total':480,'elapsed_seconds':time.monotonic()-start})+'\n')
print('COMPLETE',json.dumps(counts),flush=True)
