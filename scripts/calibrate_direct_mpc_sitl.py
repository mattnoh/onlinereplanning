#!/usr/bin/env python3
"""Sequential prebuilt-PX4 hover/thrust and small body-rate step calibration."""
import argparse,csv,json,os,time,statistics
import run_scenario_campaign as harness

def fly(out):
    import rospy
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import AttitudeTarget,PositionTarget,State
    from mavros_msgs.srv import SetMode,CommandBool
    rospy.init_node('direct_mpc_calibration',anonymous=True)
    state=State();odom=None;thrust=[]
    def state_cb(m):
        nonlocal state;state=m
    def odom_cb(m):
        nonlocal odom;odom=m
    def att_cb(m): thrust.append((time.monotonic(),m.thrust))
    rospy.Subscriber('/mavros/state',State,state_cb)
    rospy.Subscriber('/mavros/local_position/odom',Odometry,odom_cb)
    rospy.Subscriber('/mavros/setpoint_raw/target_attitude',AttitudeTarget,att_cb)
    pos=rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1)
    att=rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=1)
    mode=rospy.ServiceProxy('/mavros/set_mode',SetMode);arm=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    def hold():
        m=PositionTarget();m.header.stamp=rospy.Time.now();m.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
        m.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
        m.position.z=2;pos.publish(m)
    start=time.monotonic();last=0
    while time.monotonic()-start<30:
        hold()
        if time.monotonic()-start>2 and time.monotonic()-last>1:
            mode(custom_mode='OFFBOARD');arm(value=True);last=time.monotonic()
        if odom and state.armed and state.mode=='OFFBOARD' and time.monotonic()-start>15 and abs(odom.pose.pose.position.z-2)<.1:break
        time.sleep(.02)
    if not state.armed or not odom: raise RuntimeError('calibration failed to arm')
    start=time.monotonic()
    while time.monotonic()-start<5: hold();time.sleep(.02)
    vals=[v for t,v in thrust if t>time.monotonic()-4 and 0.05<v<.95]
    if len(vals)<5:raise RuntimeError('missing achieved hover thrust telemetry')
    hover=statistics.median(vals);print('measured hover thrust',hover,flush=True)
    rows=[];start=time.monotonic()
    while time.monotonic()-start<8:
        t=time.monotonic()-start;axis=0 if t<4 else 1;phase=(t%4)
        rate=.1 if 1<=phase<1.4 else -.1 if 2<=phase<2.4 else 0
        m=AttitudeTarget();m.header.stamp=rospy.Time.now();m.type_mask=AttitudeTarget.IGNORE_ATTITUDE
        if axis==0:m.body_rate.x=rate
        else:m.body_rate.y=rate
        z=odom.pose.pose.position.z;vz=odom.twist.twist.linear.z
        m.thrust=max(.1,min(.9,hover*(9.81+3*(2-z)-2*vz)/9.81));att.publish(m)
        w=odom.twist.twist.angular;rows.append([t,rate if axis==0 else 0,rate if axis==1 else 0,w.x,w.y,z,m.thrust,state.mode]);time.sleep(.02)
    start=time.monotonic()
    while time.monotonic()-start<3:hold();time.sleep(.02)
    mode(custom_mode='AUTO.LAND');time.sleep(5)
    with open(os.path.join(out,'rate_steps.csv'),'w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['t','cmd_roll','cmd_pitch','roll','pitch','z','thrust','mode']);writer.writerows(rows)
    fits=[]
    for tau in [.02,.03,.04,.05,.06,.08,.10,.15]:
        wr=wp=0;error=0
        for a,b in zip(rows,rows[1:]):
            dt=b[0]-a[0];alpha=1-__import__('math').exp(-dt/tau)
            wr+=alpha*(a[1]-wr);wp+=alpha*(a[2]-wp);error+=(wr-b[3])**2+(wp-b[4])**2
        fits.append((error/len(rows),tau))
    result={'hover_thrust':hover,'rate_tau':min(fits)[1],'rate_fit_mse':dict((str(t),e) for e,t in fits),'z_min':min(r[5] for r in rows),'z_max':max(r[5] for r in rows),'offboard_all':all(r[7]=='OFFBOARD' for r in rows)}
    json.dump(result,open(os.path.join(out,'calibration.json'),'w'),indent=2);print(json.dumps(result),flush=True)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);a=ap.parse_args();out=os.path.abspath(a.out);os.makedirs(out,exist_ok=True)
    harness.teardown();procs=[]
    try:
        px=os.path.join(out,'px4.log');procs.append(harness.spawn(f'cd {harness.PX4}/build/px4_sitl_default/rootfs && HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ../bin/px4 2>&1 | grep --line-buffered -vE \'^pxh>|\\[2K\'',px))
        harness.wait_for('PX4',lambda:'Startup script returned successfully' in open(px).read(),60)
        procs.append(harness.spawn(f'{harness.SETUP} && roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"',os.path.join(out,'mavros.log')))
        harness.wait_for('MAVROS',harness.fcu_connected,45)
        fly(out)
    finally:harness.teardown(procs)
if __name__=='__main__':main()
