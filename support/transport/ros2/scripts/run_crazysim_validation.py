import argparse,os,pathlib,subprocess,signal,time,json,csv,math
# Default: the colcon workspace this package is checked out in (WS/src/<package>).
WS=pathlib.Path(__file__).resolve().parents[6]
p=argparse.ArgumentParser();p.add_argument('--planner',default=str(WS/'install/comando_planner/lib/comando_planner/comando_planner'));p.add_argument('--target',default=str(WS/'install/comando_planner/lib/comando_planner/target_publisher'));p.add_argument('--motion',default='circle');p.add_argument('--out',required=True);p.add_argument('--direct',action='store_true');p.add_argument('--calibrate',action='store_true');p.add_argument('--rate',type=float,default=0);p.add_argument('--thrust',type=float,default=9.81);p.add_argument('--axis',choices=['roll','pitch','yaw'],default='yaw');p.add_argument('--calibrated',action='store_true');p.add_argument('--rate-ki',type=float,default=None);p.add_argument('--rate-scale',type=float,default=None);a=p.parse_args()
out=pathlib.Path(a.out);out.mkdir(parents=True,exist_ok=True);procs=[]
setup='export ROS_DOMAIN_ID=77; source /opt/ros/humble/setup.bash; source /home/lab/projects/cf/online/ros_ws/install/setup.bash; '
fw='/home/lab/projects/cf/online/CrazySim/crazyflie-firmware'
def run(cmd,timeout=20):return subprocess.run(['bash','-lc',setup+cmd],capture_output=True,text=True,timeout=timeout)
def spawn(cmd,name,prelude=True):
 f=(out/(name+'.log')).open('w');pr=subprocess.Popen(['bash','-lc',(setup if prelude else '')+cmd],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(pr);return pr
# Record odometry and target directly, without ROS bag/OOM.
recorder='''import rclpy,csv,time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,TwistStamped
rclpy.init();n=Node('comando_validation_recorder');f=open('STATE_FILE','w',buffering=1);w=csv.writer(f);w.writerow(['time','topic','x','y','z','vx','vy','vz','qw','qx','qy','qz','wx','wy','wz']);
def odom(m,topic):
 p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;o=m.twist.twist.angular;w.writerow([time.time(),topic,p.x,p.y,p.z,v.x,v.y,v.z,q.w,q.x,q.y,q.z,o.x,o.y,o.z])
def pose(m):
 p=m.pose.position;q=m.pose.orientation;w.writerow([time.time(),'pose',p.x,p.y,p.z,0,0,0,q.w,q.x,q.y,q.z,0,0,0])
def control(m):w.writerow([time.time(),'control',m.twist.linear.z,0,0,0,0,0,0,0,0,0,m.twist.angular.x,m.twist.angular.y,m.twist.angular.z])
n.create_subscription(Odometry,'/cf_1/odom',lambda m:odom(m,'drone'),10);n.create_subscription(PoseStamped,'/cf_1/pose',pose,10);n.create_subscription(Odometry,'/target/odom',lambda m:odom(m,'target'),10);n.create_subscription(TwistStamped,'/cf_1/cmd_bodyrate',control,10)
rclpy.spin(n)
'''.replace('STATE_FILE',str(out/'states.csv'))
(out/'record.py').write_text(recorder)
def ready(label,predicate,seconds=45):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if predicate():print('READY',label,flush=True);return
  if any(pr.poll() is not None for pr in procs):raise RuntimeError('process exited while waiting '+label)
  time.sleep(1)
 raise TimeoutError(label)
def pose_z():
 rows=list(csv.DictReader((out/'states.csv').open())) if (out/'states.csv').exists() else []
 rows=[r for r in rows if r['topic']=='pose'];return float(rows[-1]['z']) if rows else None
try:
 # Run the installed binary, never rebuild firmware. Local setup exports plugin/model paths.
 spawn(f'cd {fw}; source tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash {fw} {fw}/sitl_make/build; exec gz sim -s -r tools/crazyflie-simulation/simulator_files/gazebo/worlds/crazysim_default.sdf -v 2','gazebo',False)
 time.sleep(4)
 run(f'python3 {fw}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py {fw}/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja {fw}/tools/crazyflie-simulation/simulator_files/gazebo --cffirm_udp_port 19950 --cflib_udp_port 19850 --cf_id 0 --cf_name cf --output-file {out}/cf.sdf')
 created=run(f'''gz service -s /world/crazysim_default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 --req 'sdf_filename: "{out}/cf.sdf", pose: {{position: {{x:0,y:0,z:0.5}}}}, name: "crazyflie_0", allow_renaming: 1' ''');print(created.stdout,created.stderr,flush=True)
 spawn(f'cd {out}; source {fw}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash {fw} {fw}/sitl_make/build; export CF2_SIM_MODEL=gz_crazyflie; exec {fw}/sitl_make/build/cf2 19950','firmware',False)
 time.sleep(3)
 spawn('ros2 launch crazyflie launch.py backend:=cflib gui:=False mocap:=False','server')
 spawn('python3 '+str(out/'record.py'),'recorder')
 ready('pose',lambda:pose_z() is not None,60)
 if not a.calibrate:
  mode='gazebo_figure8' if a.motion=='figure8' else 'gazebo_circle'
  args=''
  if a.motion=='straight':
   with (out/'target.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['t','x','y','z','vx','vy','vz','ax','ay','az']);w.writerows([[i*.01,.4*i*.01,0,.2,.4,0,0,0,0,0] for i in range(20001)])
   mode='trace_replay';args=f' -p trace_csv_path:={out}/target.csv'
  target_command=f'{a.target} --ros-args -p target_mode:={mode} -p planning_frame:=world'+args
  if a.motion!='straight':spawn(target_command,'target')
  env='SZMUK_Z_STAGE=1.3 SZMUK_LOS_ALT_TRIG=1.2 MPC_RATE_LIMIT=.35 MPC_MAX_ITER=120 MPC_MAX_POLICY_AGE=2.0 MPC_HOVER_THRUST=.5460495715192737 MPC_RATE_TAU=.08' if a.direct else 'STC_PRESET=sitl'
  spawn(f'{env} {a.planner} --ros-args -p drone_name:=cf_1 -p platform:=crazyflie -p log_dir:={out} -p enable_logging:=true','planner')
  ready('planner',lambda:'comando_planner' in run('ros2 node list').stdout)
 for service,type_,body,wait in [('/cf_1/arm','Arm','{arm: true}',1),('/cf_1/takeoff','Takeoff','{group_mask: 0, height: 1.0, duration: {sec: 3, nanosec: 0}}',5),('/cf_1/go_to','GoTo','{group_mask: 0, relative: false, goal: {x: 2.0, y: 2.0, z: 1.8}, yaw: 0.0, duration: {sec: 6, nanosec: 0}}',9)]:
  res=run(f"ros2 service call {service} crazyflie_interfaces/srv/{type_} '{body}'",30);print(service,res.stdout[-400:],res.stderr[-200:],flush=True);time.sleep(wait)
 ready('staging',lambda:pose_z() is not None and abs(pose_z()-1.8)<.15,15)
 if a.direct or a.calibrated:
  calibration_params="""import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
rclpy.init();n=Node('comando_calibration_parameters');c=n.create_client(SetParameters,'/crazyflie_server/set_parameters')
if not c.wait_for_service(timeout_sec=15):raise RuntimeError('backend parameter service unavailable')
request=SetParameters.Request()
for key,value in [('bodyrate_T_max_ms2',17.96540188229731),('bodyrate_T_min_ms2',2.9945330704142568)]:
 p=Parameter();p.name=key;p.value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=value);request.parameters.append(p)
f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10);result=f.result();print(result,flush=True)
if result is None or not all(r.successful for r in result.results):raise RuntimeError('backend calibration rejected')
if RATE_KI is not None:
 for axis in ['roll','pitch','yaw']:
  request=SetParameters.Request();p=Parameter();p.name='all.params.pid_rate.'+axis+'_ki';p.value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=RATE_KI);request.parameters=[p]
  f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10);result=f.result();print(result,flush=True)
  if result is None or not all(r.successful for r in result.results):raise RuntimeError('rate tuning rejected')
if RATE_SCALE is not None:
 for axis in ['roll','pitch']:
  for gain,default in [('kp',250.0),('ki',500.0),('kd',2.5)]:
   request=SetParameters.Request();p=Parameter();p.name='all.params.pid_rate.'+axis+'_'+gain;p.value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=default*RATE_SCALE);request.parameters=[p]
   f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10);result=f.result();print(result,flush=True)
   if result is None or not all(r.successful for r in result.results):raise RuntimeError('rate scaling rejected')
"""
  calibration_params=calibration_params.replace('RATE_KI',repr(a.rate_ki)).replace('RATE_SCALE',repr(a.rate_scale))
  (out/'calibration_params.py').write_text(calibration_params);res=run('python3 '+str(out/'calibration_params.py'));print(res.stdout,res.stderr,flush=True)
  if res.returncode:raise RuntimeError('backend calibration parameter failed')
 if not a.calibrate and a.motion=='straight':spawn(target_command,'target');time.sleep(2)
 activation=time.time()
 if a.calibrate:
  script=f'''import rclpy,time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
rclpy.init();n=Node('calibration');pub=n.create_publisher(TwistStamped,'/cf_1/cmd_bodyrate',10);start=time.monotonic()
while time.monotonic()-start<3:
 m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.twist.linear.z={float(a.thrust)!r};m.twist.angular.{dict(roll='x',pitch='y',yaw='z')[a.axis]}={float(a.rate)!r} if time.monotonic()-start>1 else 0.0;pub.publish(m);rclpy.spin_once(n,timeout_sec=0);time.sleep(.02)
'''
  (out/'step.py').write_text(script);res=run('python3 '+str(out/'step.py'),10);print(res.stderr,flush=True)
  if res.returncode:raise RuntimeError('calibration publisher failed')
 else:
  trigger="""import rclpy,time,math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
rclpy.init();n=Node('comando_validation_trigger');client=n.create_client(SetParametersAtomically,'/comando_planner/set_parameters_atomically');client.wait_for_service(timeout_sec=10)
phi=[None]
def target(m):phi[0]=math.atan2(m.pose.pose.position.y,m.pose.pose.position.x)%(2*math.pi)
n.create_subscription(Odometry,'/target/odom',target,10)
start=time.monotonic()
while MOTION=='circle' and time.monotonic()-start<20:
 rclpy.spin_once(n,timeout_sec=.005)
 if phi[0] is not None and abs((phi[0]-4.294+math.pi)%(2*math.pi)-math.pi)<.01:break
request=SetParametersAtomically.Request()
for name,value in [('ocp_type',OCP),('mode','mpc'),('n_replay',NREPLAY),('command_seq',1)]:
 p=Parameter();p.name=name;p.value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value) if isinstance(value,str) else ParameterValue(type=ParameterType.PARAMETER_INTEGER,integer_value=value);request.parameters.append(p)
print('ACTIVATION',time.time(),flush=True);future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=10);result=future.result();print(result,flush=True)
if result is None or not result.result.successful:raise RuntimeError('profile transaction failed')
""".replace('MOTION',repr(a.motion)).replace('OCP',repr('stc_landing_mpc' if a.direct else 'stc_landing_noaug')).replace('NREPLAY',str(1 if a.direct else 4))
  (out/'trigger.py').write_text(trigger);res=run('python3 '+str(out/'trigger.py'),40);print(res.stdout,res.stderr,flush=True)
  if res.returncode:raise RuntimeError('profile activation failed')
  time.sleep(25)
 print('FINAL Z',pose_z(),flush=True)
 (out/'cell.json').write_text(json.dumps(dict(motion=a.motion,direct=a.direct,activation=activation,final_z=pose_z(),calibrate=a.calibrate,rate_ki=a.rate_ki,rate_scale=a.rate_scale,axis=a.axis,thrust=a.thrust,rate=a.rate),indent=2))
except Exception as error:
 (out/'cell.json').write_text(json.dumps(dict(motion=a.motion,direct=a.direct,calibrate=a.calibrate,result='HARNESS_FAILED',error=str(error),rate_ki=a.rate_ki,rate_scale=a.rate_scale),indent=2))
 raise
finally:
 for pr in reversed(procs):
  try:os.killpg(pr.pid,signal.SIGINT)
  except ProcessLookupError:pass
 for pr in procs:
  try:pr.wait(timeout=5)
  except subprocess.TimeoutExpired:
   try:os.killpg(pr.pid,signal.SIGKILL)
   except ProcessLookupError:pass
