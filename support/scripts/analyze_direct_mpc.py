#!/usr/bin/env python3
"""Audit feedback timing, bounded command slew, and offboard continuity."""
import argparse,csv,json,pathlib,statistics
import rosbag

def analyze(cell):
    rows=list(csv.DictReader((cell/'direct_mpc_controls.csv').open()))
    periods=[float(b['time'])-float(a['time']) for a,b in zip(rows,rows[1:])]
    commands=[];states=[];positions=[];starts=[];stops=[]
    with rosbag.Bag(str(next(cell.glob('flight_*.bag')))) as bag:
        for topic,msg,t in bag.read_messages(topics=['/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/local','/mavros/state','/rosout']):
            if topic.endswith('/attitude'):commands.append(t.to_sec())
            elif topic.endswith('/local'):positions.append(t.to_sec())
            elif topic=='/mavros/state':states.append((t.to_sec(),msg.mode,msg.armed))
            elif 'Starting OCP=' in msg.msg and 'landing' in msg.msg:starts.append(t.to_sec())
            elif 'Touchdown: OCP stopped' in msg.msg:stops.append(t.to_sec())
    start=max(starts) if starts else float("-inf");end=min((t for t in stops if t>=start),default=float("inf"))
    commands=[t for t in commands if start<=t<=end]
    if not commands:raise RuntimeError("No direct-control interval: "+str(cell))
    first,last=min(commands),max(commands)
    active=[s for s in states if first<=s[0]<=last]
    interleaved=[t for t in positions if first+.1<t<last-.1]
    max_slew=[0.0]*4
    for a,b,dt in zip(rows,rows[1:],periods):
        if dt<=0:continue
        for i,key in enumerate(['thrust','roll','pitch','yaw']):max_slew[i]=max(max_slew[i],abs(float(b[key])-float(a[key]))/dt)
    return dict(motion=cell.name.split('__')[0],feedback_samples=len(rows),median_command_period_s=statistics.median(periods),
        max_command_period_s=max(periods),max_slew=max_slew,max_policy_age_s=max(float(r['age']) for r in rows),
        saturated_samples=sum(int(r['saturation']) for r in rows),feedback_limited_samples=sum(int(r['feedback_limited']) for r in rows),
        offboard_throughout=bool(active) and all(s[1]=='OFFBOARD' for s in active),position_commands_during_direct_control=len(interleaved))

def physical_audit(cell, audit_bin):
    """Score the controlled flight and densely propagate accepted nominal horizons."""
    import os,sys,subprocess,math
    import numpy as np
    import rosbag
    sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/"tools/analysis"))
    import bag_to_stc_csv as converter
    environment=json.loads((cell/'planner_environment.json').read_text())
    previous=dict(os.environ)
    os.environ.update(environment)
    try:
        bag=next(cell.glob('*.bag'))
        ocp=next(line.split(':',1)[1].strip() for line in (cell/'landing_profile.yaml').read_text().splitlines() if line.startswith('ocp_type:'))
        ordinary=ocp.startswith('landing_')
        starts=[];stops=[]
        with rosbag.Bag(str(bag)) as stream:
            for _,msg,stamp in stream.read_messages(topics=['/rosout']):
                if 'Starting OCP='+ocp+' ' in msg.msg:starts.append(stamp.to_sec())
                if 'Touchdown: OCP stopped' in msg.msg:stops.append(stamp.to_sec())
        if not starts:raise RuntimeError('No landing activation: '+str(cell))
        start=starts[-1];end=min((t for t in stops if t>=start),default=float('inf'))
        drone,target=converter.read_bag(bag)
        c=converter.resolve_constants()
        active=drone[(drone[:,0]>=start)&(drone[:,0]<=end)]
        t=active[:,0]
        if len(t)<2:raise RuntimeError('No controlled airborne interval')
        tgt=np.column_stack([np.interp(t,target[:,0],target[:,k]) for k in range(1,7)])
        relative=active[:,1:4]-tgt[:,:3];velocity=active[:,4:7]-tgt[:,3:]
        q=active[:,7:11];q=q/np.linalg.norm(q,axis=1)[:,None]
        tilt=2*np.degrees(np.arcsin(np.clip(np.linalg.norm(q[:,1:3],axis=1),0,1)))
        omega=np.linalg.norm(active[:,11:14],axis=1)
        axis=np.column_stack([2*(q[:,1]*q[:,3]+q[:,0]*q[:,2]),2*(q[:,2]*q[:,3]-q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2)])
        norm=np.linalg.norm(relative,axis=1)
        los=np.degrees(np.arccos(np.clip(np.sum(axis*relative,axis=1)/np.maximum(norm,1e-12),-1,1)))
        land=np.ones(len(t),dtype=bool) if ordinary else relative[:,2]<c['alt_trig']
        los_active=np.ones(len(t),dtype=bool) if ordinary else relative[:,2]<c['los_alt_trig']
        rxy=np.linalg.norm(relative[:,:2],axis=1)
        residuals={
            'altitude_floor_m':-relative[:,2],
            'speed_m_s':np.linalg.norm(velocity,axis=1)-np.where(land,c['v_land_stc_max'],c['v_phase0_max']),
            'tilt_deg':tilt-np.where(land,c['tilt_land_stc_deg'],c['tilt_phase0_deg']),
            'angular_rate_rad_s':omega-np.where(land,c['omega_land_stc_max'],c['omega_phase0_max']),
            'glideslope_m':np.where(land,rxy-c['gs_cone_tan']*(relative[:,2]+c['gs_apex_offset']),0),
            'los_deg':np.where(los_active,los-c['los_cone_deg'],0),
            'staging_height_m':np.zeros(len(t)) if ordinary else np.where(rxy>c['stage_radius'],c['z_stage']-relative[:,2],0),
        }
        dt=np.r_[np.diff(t),0]
        stats={}
        for name,values in residuals.items():
            tolerance=.01 if name.endswith('_deg') else 1e-4
            stats[name]={'max_violation':float(np.maximum(values,0).max()),'violation_duration_sec':float(dt[values>tolerance].sum()),'tolerance':tolerance}
        result={'ocp':ocp,'controlled_duration_sec':float(t[-1]-t[0]),'measured_constraints':stats,'initial_relative_position_m':relative[0].tolist(),'initial_relative_speed_m_s':float(np.linalg.norm(velocity[0])),'initial_tilt_deg':float(tilt[0]),'initial_angular_rate_rad_s':float(omega[0])}
        if audit_bin:
            run=subprocess.run([str(audit_bin),ocp,str(cell)],env=os.environ.copy(),capture_output=True,text=True,check=True)
            result['predicted']=json.loads(run.stdout)
        (cell/'constraint_audit.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    finally:
        os.environ.clear();os.environ.update(previous)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('campaign',type=pathlib.Path);ap.add_argument('--physical',action='store_true');ap.add_argument('--audit-bin',type=pathlib.Path);args=ap.parse_args()
    if args.physical:
        result=[physical_audit(p,args.audit_bin) for p in sorted(args.campaign.glob('*__zero__t0')) if (p/'cell.json').exists()]
        (args.campaign/'constraint_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return
    result=[analyze(p) for p in sorted(args.campaign.glob('*__zero__t0')) if (p/'cell.json').exists()]
    (args.campaign/'control_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
