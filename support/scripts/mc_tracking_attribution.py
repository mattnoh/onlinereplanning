#!/usr/bin/env python3
"""Plan-vs-execution attribution of measured landing-window constraint excess.

For receding-horizon campaigns the commanded reference sent to PX4
(/mavros/setpoint_raw/local: position, velocity, acceleration feedforward) is
recorded on the flight clock. At every executed sample of the historical
landing window (same window and limits as mc_physical_scoring.py) the
reference is held from its latest message and scored with the same physical
residual as the vehicle:

  glideslope  reference position vs the true target
  speed       reference velocity minus target velocity
  tilt, rate  the accepted plan's own attitude and body rates (all_solves.csv),
              because the historical references carry no acceleration
              feedforward: PX4 formed the attitude itself. Plans are placed on
              the flight clock through their activation times, anchored at
              the landing start (rosout "Starting OCP" minus the first solve
              duration); the attribution is repeated with the anchor shifted by
              -0.1/+0.1 s and the spread reported.

The triangle inequality bounds executed excess by reference excess plus the
tracking error of the same quantity. Each violating sample's excess is split
into a reference part min(e_exec, [e_ref]_+) and a tracking part (remainder).
Reported per constraint: violating samples, samples with the reference inside
the limit, and the tracking share of summed excess.

Usage: mc_tracking_attribution.py <campaign_dir> --out attribution.csv [--fixed-circle-truth]
"""
import argparse
import csv
import json
import math
import pathlib

import numpy as np
import rosbag

from mc_physical_scoring import ORIGINAL, TOL, fixed_circle_truth, read_bag

G = 9.81


def references(bag):
    out = []
    with rosbag.Bag(str(bag)) as b:
        for _, m, bt in b.read_messages(topics=["/mavros/setpoint_raw/local"]):
            out.append((bt.to_sec(), m.position.x, m.position.y, m.position.z,
                        m.velocity.x, m.velocity.y, m.velocity.z,
                        m.acceleration_or_force.x, m.acceleration_or_force.y,
                        m.acceleration_or_force.z, m.type_mask))
    return np.array(out)


def thrust_axis(acc):
    b = acc + np.array([0.0, 0.0, G])
    return b/np.linalg.norm(b, axis=1)[:, None]


def plans(cell):
    """Accepted plans: (start on the planner clock, node times, tilt deg, |omega|)."""
    ev = [r for r in csv.DictReader(open(cell/"solver_events.csv")) if r["event"] == "accepted"]
    nodes = {}
    for r in csv.DictReader(open(cell/"all_solves.csv")):
        nodes.setdefault(int(r["solve_num"]), []).append(r)
    out = []
    for r in ev:
        k = int(r["solve_num"])
        if k not in nodes:
            continue
        act = float(r["activation_wall_time_sec"])
        start = act if act > 0 else float(r["timestamp"])
        rows = nodes[k]
        tn = np.array([float(x["t"]) for x in rows])
        q = np.array([[float(x[c]) for c in ("qw", "qx", "qy", "qz")] for x in rows])
        q /= np.linalg.norm(q, axis=1)[:, None]
        tilt = 2*np.degrees(np.arcsin(np.clip(np.hypot(q[:, 1], q[:, 2]), 0, 1)))
        om = np.linalg.norm(np.array([[float(x[c]) for c in ("wx", "wy", "wz")] for x in rows]), axis=1)
        out.append((start, tn, tilt, om))
    first = ev[0]
    anchor_mono = float(first["timestamp"]) - float(first["solve_time_ms"])/1000.0
    return out, anchor_mono


def planned_at(pl, mono):
    """Tilt and |omega| of the latest plan active at each planner-clock time."""
    starts = np.array([p[0] for p in pl])
    tilt = np.full(len(mono), np.nan)
    om = np.full(len(mono), np.nan)
    k = np.searchsorted(starts, mono, side="right")-1
    for i, kk in enumerate(k):
        if kk < 0:
            continue
        s, tn, ti, w = pl[kk]
        e = mono[i]-s
        tilt[i] = np.interp(e, tn, ti)
        om[i] = np.interp(e, tn, w)
    return tilt, om


def attribute_cell(cell, c=ORIGINAL, fixed_circle=False):
    meta = json.loads((cell/"cell.json").read_text())
    bag = next(cell.glob("flight_*.bag"))
    drone, target, gt, rosout = read_bag(bag)
    truth = gt if len(gt) else (fixed_circle_truth(target)[0] if fixed_circle and meta["motion"] == "circle" else target)
    ref = references(bag)
    t = drone[:, 0]
    tgt = np.column_stack([np.interp(t, truth[:, 0], truth[:, k]) for k in range(1, 7)])
    rel, vel = drone[:, 1:4]-tgt[:, :3], drone[:, 4:7]-tgt[:, 3:]
    # Historical landing window (identical to mc_physical_scoring.py "orig").
    i0 = max([i for i in range(len(t)) if rel[i, 2] > 1.3], default=0)
    cut = len(t)
    for i in range(i0+1, len(t)):
        dti = t[i]-t[i-1]
        if dti > 0 and (vel[i, 2]-vel[i-1, 2])/dti < -6.0 and rel[i, 2] < 0.4:
            cut = i
            break
    idx = np.arange(i0, cut)
    win = idx[rel[idx, 2] < c["alt_trig"]]
    row = dict(motion=meta["motion"], level=meta["uncertainty"], trial=meta["trial"], result=meta["result"],
               samples=int(len(win)))
    if len(win) < 2 or len(ref) == 0:
        return row
    j = np.clip(np.searchsorted(ref[:, 0], t[win], side="right")-1, 0, len(ref)-1)  # zero-order hold
    R = ref[j]
    row["ref_has_feedforward"] = int(np.mean(np.abs(R[:, 7:10]).sum(axis=1) > 0) > 0.5)
    tw, tg = t[win], tgt[win]
    rel_e, vel_e = rel[win], vel[win]
    rel_r, vel_r = R[:, 1:4]-tg[:, :3], R[:, 4:7]-tg[:, 3:]
    q = drone[win, 7:11]
    b_e = np.column_stack([2*(q[:, 1]*q[:, 3]+q[:, 0]*q[:, 2]), 2*(q[:, 2]*q[:, 3]-q[:, 0]*q[:, 1]),
                           1-2*(q[:, 1]**2+q[:, 2]**2)])
    tan_gs = math.tan(math.radians(c["gs_cone_deg"]))
    gs = lambda r: np.hypot(r[:, 0], r[:, 1]) - tan_gs*(r[:, 2]+c["gs_apex_offset"])
    tilt = lambda b: np.degrees(np.arccos(np.clip(b[:, 2], -1, 1)))
    pl, anchor_mono = plans(cell)
    start_epoch = [ts for ts, msg in rosout if "Starting OCP=" in msg and "landing" in msg][-1]
    offset = start_epoch - anchor_mono
    tilt_e = tilt(b_e)-c["tilt_land_stc_deg"]
    rate_e = np.linalg.norm(drone[win, 11:14], axis=1)-c["omega_land_stc_max"]
    for shift in (-0.1, 0.1):
        pt, po = planned_at(pl, tw-offset+shift)
        for name, e, r in (("tilt_deg", tilt_e, pt-c["tilt_land_stc_deg"]),
                           ("angular_rate_rad_s", rate_e, po-c["omega_land_stc_max"])):
            tol = TOL["deg"] if name.endswith("_deg") else TOL["si"]
            v = (e > tol) & np.isfinite(r)
            ep = np.maximum(e, 0)
            row[f"{name}_tracking_sum_shift{shift:+.1f}"] = float((ep-np.minimum(ep, np.maximum(r, 0)))[v].sum())
    pt, po = planned_at(pl, tw-offset)
    pairs = dict(
        glideslope_m=(gs(rel_e), gs(rel_r)),
        speed_m_s=(np.linalg.norm(vel_e, axis=1)-c["v_land_stc_max"],
                   np.linalg.norm(vel_r, axis=1)-c["v_land_stc_max"]),
        tilt_deg=(tilt_e, pt-c["tilt_land_stc_deg"]),
        angular_rate_rad_s=(rate_e, po-c["omega_land_stc_max"]))
    for name, (e, r) in pairs.items():
        tol = TOL["deg"] if name.endswith("_deg") else TOL["si"]
        v = (e > tol) & np.isfinite(r)
        ep = np.maximum(e, 0)
        plan_part = np.minimum(ep, np.maximum(r, 0))
        row[f"{name}_viol_samples"] = int(v.sum())
        row[f"{name}_viol_ref_inside"] = int((v & (r <= tol)).sum())
        row[f"{name}_excess_sum"] = float(ep[v].sum())
        row[f"{name}_tracking_sum"] = float((ep-plan_part)[v].sum())
    row["pos_track_med_m"] = float(np.median(np.linalg.norm(drone[win, 1:4]-R[:, 1:4], axis=1)))
    row["vel_track_med_m_s"] = float(np.median(np.linalg.norm(drone[win, 4:7]-R[:, 4:7], axis=1)))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--fixed-circle-truth", action="store_true")
    args = ap.parse_args()
    rows = []
    for cell in sorted(p for p in args.campaign.iterdir() if (p/"cell.json").exists()):
        try:
            rows.append(attribute_cell(cell, fixed_circle=args.fixed_circle_truth))
        except Exception as e:
            rows.append(dict(cell=cell.name, error=str(e)))
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print("wrote", args.out, len(rows))


if __name__ == "__main__":
    main()
