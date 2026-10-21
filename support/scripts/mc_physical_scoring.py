#!/usr/bin/env python3
"""Physical scoring of MC1/MC2 campaign cells from their flight bags.

One definition is applied to historical (RH) and current (MPC) campaigns, using
the ORIGINAL MC1/MC2 physical limits that every original cell recorded in its
constraints.csv (identical across all 360 cells). Optimizer tightening in a
newer preset never changes these criteria.

Two windows, reported separately:

  original   The historical scenario_metrics.py window: after the last sample
             above 1.3 m relative altitude, samples below the 0.8 m landing
             trigger, trimmed at the force-disarm free fall. Four landing
             consequences: glideslope, speed, tilt, angular rate.
  active     Landing-OCP activation to touchdown handoff (or policy stop / end
             of record): all eight physical categories with their trigger
             logic (staging outside the stage radius, LOS below its trigger,
             landing limits below the landing trigger, phase-0 limits above).

For each constraint: maximum positive excess (physical units), violation
duration at the odometry resolution (excess above a reporting tolerance), and
whether it was violated. Target truth comes from /target/odom_gt when the bag
has it; otherwise /target/odom, which under estimate noise is the corrupted
estimate (the historical MC2 bags) -- every row records which one was used.
With --fixed-circle-truth, a noisy cell whose target was the MC2-era fixed
circle (centre (0,0,0.2), R=1, omega=0.4, phase 0 at publisher start) is scored
against that analytic circle with the publisher start time and constant
horizontal bias fitted to the estimate; the fit residual is recorded.

Usage: mc_physical_scoring.py <campaign_dir> [--out scores.csv] [--fixed-circle-truth]
"""
import argparse
import csv
import json
import math
import pathlib
import statistics

import numpy as np
import rosbag

# Original MC1/MC2 physical limits (constraints.csv of every original cell).
ORIGINAL = dict(alt_trig=0.8, cap_radius=0.45, z_stage=1.4, stage_radius=1.1,
                los_alt_trig=1.2, los_cone_deg=30.0, v_phase0_max=3.0,
                v_land_stc_max=0.75, omega_phase0_max=0.35,
                omega_land_stc_max=0.019, tilt_phase0_deg=11.0,
                tilt_land_stc_deg=2.0, gs_cone_deg=15.0, gs_apex_offset=0.1)
TOL = dict(deg=0.01, si=1e-4)
LAND4 = ("glideslope_m", "speed_m_s", "tilt_deg", "angular_rate_rad_s")
ALL8 = ("altitude_floor_m", "speed_m_s", "tilt_deg", "angular_rate_rad_s",
        "glideslope_m", "los_deg", "los_cone_m", "staging_height_m")


def stamp_of(m, bt):
    h = m.header.stamp.to_sec()
    return h if abs(h - bt.to_sec()) < 10.0 else bt.to_sec()


def quat_rot(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def read_bag(path):
    drone, target, target_gt, rosout = [], [], [], []
    with rosbag.Bag(str(path)) as bag:
        for topic, m, bt in bag.read_messages(topics=["/mavros/local_position/odom", "/target/odom",
                                                      "/target/odom_gt", "/rosout"]):
            if topic == "/rosout":
                rosout.append((bt.to_sec(), m.msg))
                continue
            p, v = m.pose.pose.position, m.twist.twist.linear
            if topic == "/mavros/local_position/odom":
                q, w = m.pose.pose.orientation, m.twist.twist.angular
                qv = np.array([q.w, q.x, q.y, q.z]); qv /= np.linalg.norm(qv)
                vw = quat_rot(qv) @ np.array([v.x, v.y, v.z])
                drone.append((stamp_of(m, bt), p.x, p.y, p.z, *vw, *qv, w.x, w.y, w.z))
            else:
                (target_gt if topic.endswith("_gt") else target).append(
                    (stamp_of(m, bt), p.x, p.y, p.z, v.x, v.y, v.z))
    return np.array(drone), np.array(target), np.array(target_gt), rosout


def fixed_circle_truth(est, R=1.0, omega=0.4, centre=(0.0, 0.0, 0.2)):
    """Analytic MC2 circle fitted to a noisy estimate: unknown publisher start
    t0 and constant horizontal bias b. Returns truth on the estimate stamps and
    the fit diagnostics."""
    t, xy = est[:, 0], est[:, 1:3]-np.array(centre[:2])
    def fit(t0):
        ph = omega*(t-t0)
        model = R*np.column_stack([np.cos(ph), np.sin(ph)])
        b = (xy-model).mean(axis=0)
        return float(((xy-model-b)**2).sum()), b
    period = 2*math.pi/omega
    grid = t[0]-np.linspace(0, period, 2000, endpoint=False)
    best = min(grid, key=lambda t0: fit(t0)[0])
    lo, hi = best-period/2000, best+period/2000
    for _ in range(60):  # golden-section refinement
        m1, m2 = lo+(hi-lo)*0.382, lo+(hi-lo)*0.618
        if fit(m1)[0] < fit(m2)[0]:
            hi = m2
        else:
            lo = m1
    t0 = 0.5*(lo+hi)
    sse, b = fit(t0)
    ph = omega*(t-t0)
    truth = np.column_stack([t, centre[0]+R*np.cos(ph), centre[1]+R*np.sin(ph), np.full(len(t), centre[2]),
                             -R*omega*np.sin(ph), R*omega*np.cos(ph), np.zeros(len(t))])
    return truth, dict(fit_rms_m=math.sqrt(sse/len(t)), bias_x=float(b[0]), bias_y=float(b[1]))


def residuals(rel, vel, q, omega_b, c, window):
    """Physical residuals (positive = excess) on each sample."""
    tilt = 2*np.degrees(np.arcsin(np.clip(np.linalg.norm(q[:, 1:3], axis=1), 0, 1)))
    omega = np.linalg.norm(omega_b, axis=1)
    axis = np.column_stack([2*(q[:, 1]*q[:, 3]+q[:, 0]*q[:, 2]), 2*(q[:, 2]*q[:, 3]-q[:, 0]*q[:, 1]),
                            1-2*(q[:, 1]**2+q[:, 2]**2)])
    rxy = np.linalg.norm(rel[:, :2], axis=1)
    gs = rxy - math.tan(math.radians(c["gs_cone_deg"]))*(rel[:, 2]+c["gs_apex_offset"])
    speed = np.linalg.norm(vel, axis=1)
    if window == "original":
        return dict(glideslope_m=gs, speed_m_s=speed-c["v_land_stc_max"],
                    tilt_deg=tilt-c["tilt_land_stc_deg"],
                    angular_rate_rad_s=omega-c["omega_land_stc_max"])
    land = rel[:, 2] < c["alt_trig"]
    los_on = rel[:, 2] < c["los_alt_trig"]
    # Same geometry as analyze_direct_mpc.py: angle between the body vertical
    # axis and the target-to-vehicle vector, and its spatial cone residual.
    norm = np.maximum(np.linalg.norm(rel, axis=1), 1e-12)
    along = np.sum(axis*rel, axis=1)
    los = np.degrees(np.arccos(np.clip(along/norm, -1, 1)))
    lateral = np.linalg.norm(rel - along[:, None]*axis, axis=1)
    return dict(
        altitude_floor_m=-rel[:, 2],
        speed_m_s=speed-np.where(land, c["v_land_stc_max"], c["v_phase0_max"]),
        tilt_deg=tilt-np.where(land, c["tilt_land_stc_deg"], c["tilt_phase0_deg"]),
        angular_rate_rad_s=omega-np.where(land, c["omega_land_stc_max"], c["omega_phase0_max"]),
        glideslope_m=np.where(land, gs, 0.0),
        los_deg=np.where(los_on, los-c["los_cone_deg"], 0.0),
        los_cone_m=np.where(los_on, lateral-math.tan(math.radians(c["los_cone_deg"]))*along, 0.0),
        staging_height_m=np.where(rxy > c["stage_radius"], c["z_stage"]-rel[:, 2], 0.0))


def summarize(t, res):
    dt = np.r_[np.diff(t), 0.0] if len(t) else np.zeros(0)
    out = {}
    for name, v in res.items():
        tol = TOL["deg"] if name.endswith("_deg") else TOL["si"]
        if len(v) == 0:
            out[name] = dict(max=0.0, duration_s=0.0, violated=False)
            continue
        out[name] = dict(max=float(np.maximum(v, 0).max()), duration_s=float(dt[v > tol].sum()),
                         violated=bool((v > tol).any()))
    return out


def solver_stats(cell):
    f = cell/"solver_events.csv"
    if not f.exists():
        return {}
    acc, rej, ms, ms_warm = 0, 0, [], []
    for r in csv.DictReader(f.open()):
        if r["event"] == "accepted":
            acc += 1
            try:
                ms.append(float(r["solve_time_ms"]))
            except ValueError:
                pass
        elif r["event"] == "rejected":
            rej += 1
    ms_warm = ms[1:]
    pct = lambda v, p: float(np.percentile(v, p)) if v else float("nan")
    return dict(accepted=acc, rejected=rej, solve_ms_cold=ms[0] if ms else float("nan"),
                solve_ms_median=pct(ms_warm or ms, 50), solve_ms_p95=pct(ms_warm or ms, 95),
                solve_ms_max=max(ms) if ms else float("nan"))


def score_cell(cell, c=ORIGINAL, fixed_circle=False):
    meta = json.loads((cell/"cell.json").read_text())
    row = dict(motion=meta["motion"], level=meta["uncertainty"], trial=meta["trial"], seed=meta["seed"],
               result=meta["result"], teardown=meta.get("teardown", ""), cell=cell.name)
    row.update(solver_stats(cell))
    bags = sorted(cell.glob("flight_*.bag"))
    if not bags:
        row["scored"] = "no_bag"
        return row
    drone, target, gt, rosout = read_bag(bags[0])
    noisy = meta.get("sigma", {}).get("target_noise_sigma_pos", 0.0) > 0
    if len(gt):
        truth, row["target_truth"] = gt, "odom_gt"
    elif fixed_circle and meta["motion"] == "circle":
        truth, diag = fixed_circle_truth(target)
        row["target_truth"] = "fixed_circle_fit"
        row.update({f"truth_{k}": v for k, v in diag.items()})
    else:
        truth, row["target_truth"] = target, ("odom_noisy_estimate" if noisy else "odom")
    t = drone[:, 0]
    tgt = np.column_stack([np.interp(t, truth[:, 0], truth[:, k]) for k in range(1, 7)])
    rel, vel = drone[:, 1:4]-tgt[:, :3], drone[:, 4:7]-tgt[:, 3:]
    q, om = drone[:, 7:11], drone[:, 11:14]

    # Window "original" (historical scenario_metrics.py).
    i0 = max([i for i in range(len(t)) if rel[i, 2] > 1.3], default=None)
    lo = i0 if i0 is not None else 0
    cut = len(t)
    for i in range(lo+1, len(t)):
        dti = t[i]-t[i-1]
        if dti > 0 and (vel[i, 2]-vel[i-1, 2])/dti < -6.0 and rel[i, 2] < 0.4:
            cut = i
            break
    idx = np.arange(lo, cut)
    win = idx[rel[idx, 2] < c["alt_trig"]]
    res = residuals(rel[win], vel[win], q[win], om[win], c, "original")
    for name, s in summarize(t[win], res).items():
        row.update({f"orig_{name}_max": s["max"], f"orig_{name}_dur_s": s["duration_s"],
                    f"orig_{name}_viol": int(s["violated"])})
    row["orig_window_s"] = float(t[win][-1]-t[win][0]) if len(win) > 1 else 0.0
    row["orig_any_viol"] = int(any(row[f"orig_{n}_viol"] for n in LAND4))
    if cut > lo:
        row["td_lateral_m"] = float(np.linalg.norm(rel[cut-1, :2]))
        row["td_rel_speed_m_s"] = float(np.linalg.norm(vel[cut-1]))

    # Window "active": landing activation to handoff/stop.
    starts = [ts for ts, msg in rosout if "Starting OCP=" in msg and "landing" in msg]
    stops = [ts for ts, msg in rosout if "Touchdown: OCP stopped" in msg or "Command finished (direct MPC" in msg]
    if starts:
        a = starts[-1]
        b = min([s for s in stops if s >= a], default=t[cut-1] if cut else t[-1])
        sel = (t >= a) & (t <= b)
        res = residuals(rel[sel], vel[sel], q[sel], om[sel], c, "active")
        for name, s in summarize(t[sel], res).items():
            row.update({f"act_{name}_max": s["max"], f"act_{name}_dur_s": s["duration_s"],
                        f"act_{name}_viol": int(s["violated"])})
        row["act_window_s"] = float(b-a)
        row["act_any_viol"] = int(any(row[f"act_{n}_viol"] for n in ALL8))
        row["completion_s"] = float(b-a) if meta["result"] == "LANDED" else float("nan")
    row["scored"] = "ok"
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path)
    ap.add_argument("--fixed-circle-truth", action="store_true",
                    help="score historical MC2 circle cells against the fitted analytic circle")
    args = ap.parse_args()
    cells = sorted(p for p in args.campaign.iterdir()
                   if p.is_dir() and not p.name.startswith("_") and (p/"cell.json").exists())
    rows = []
    for cell in cells:
        try:
            rows.append(score_cell(cell, fixed_circle=args.fixed_circle_truth))
        except Exception as e:  # keep the cell in the table with its error
            meta = json.loads((cell/"cell.json").read_text())
            rows.append(dict(motion=meta["motion"], level=meta["uncertainty"], trial=meta["trial"],
                             result=meta["result"], cell=cell.name, scored=f"error:{e}"))
        print(cell.name, rows[-1].get("scored"), flush=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    out = args.out or args.campaign/"physical_scores.csv"
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print("wrote", out, len(rows), "cells")


if __name__ == "__main__":
    main()
