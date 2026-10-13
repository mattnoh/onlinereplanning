#!/usr/bin/env python3
"""Audit feedback timing, bounded command slew, and offboard continuity."""
import argparse,csv,json,pathlib,statistics
import rosbag

def analyze(cell):
    rows=list(csv.DictReader((cell/'direct_mpc_controls.csv').open()))
    periods=[float(b['time'])-float(a['time']) for a,b in zip(rows,rows[1:])]
    commands=[];states=[];positions=[]
    with rosbag.Bag(str(next(cell.glob('flight_*.bag')))) as bag:
        for topic,msg,t in bag.read_messages(topics=['/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/local','/mavros/state']):
            if topic.endswith('/attitude'):commands.append(t.to_sec())
            elif topic.endswith('/local'):positions.append(t.to_sec())
            else:states.append((t.to_sec(),msg.mode,msg.armed))
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

def main():
    ap=argparse.ArgumentParser();ap.add_argument('campaign',type=pathlib.Path);args=ap.parse_args()
    result=[analyze(p) for p in sorted(args.campaign.glob('*__zero__t0')) if (p/'cell.json').exists()]
    (args.campaign/'control_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
