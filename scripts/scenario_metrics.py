#!/usr/bin/env python3
"""Aggregate a scenario campaign into per-trial and per-cell statistics.

Reads every cell directory written by run_scenario_campaign.py and emits:
  samples.csv  one row per trial
  summary.csv  one row per (motion, uncertainty), with land rate + quantiles

Failed and timed-out runs stay in the denominator: a cell that never lands must
show as a 0% land rate, never as a missing row.

Usage:  scenario_metrics.py <campaign_dir> [--gt]
"""
import argparse
import csv
import json
import math
import os
import subprocess


def q(xs, p):
    xs = sorted(v for v in xs if v is not None and not math.isnan(v))
    if not xs:
        return ""
    if len(xs) == 1:
        return round(xs[0], 4)
    i = p * (len(xs) - 1)
    lo, hi = int(math.floor(i)), int(math.ceil(i))
    return round(xs[lo] + (xs[hi] - xs[lo]) * (i - lo), 4)


def med(xs):
    return q(xs, 0.5)


def solver_stats(cell):
    """Per-solve metrics straight out of the planner's own event log."""
    f = os.path.join(cell, "solver_events.csv")
    out = dict(n_accept=0, n_reject=0, solve_ms_med="", solve_ms_p90="", solve_ms_max="",
               deadline_miss_rate="", track_err_med="", track_err_max="",
               vel_err_med="", worst_constraint="", src_measured=0, src_predicted=0,
               src_recovery=0)
    if not os.path.exists(f):
        return out
    acc = [r for r in csv.DictReader(open(f)) if r["event"] == "accepted"]
    rej = [r for r in csv.DictReader(open(f)) if r["event"] == "rejected"]
    out["n_accept"], out["n_reject"] = len(acc), len(rej)
    if not acc:
        return out
    fl = lambda r, k: float(r[k]) if r.get(k) not in (None, "", "nan") else float("nan")
    ms = [fl(r, "solve_time_ms") for r in acc]
    late = [fl(r, "solve_finish_late_by_sec") for r in acc]
    pe = [fl(r, "handoff_pos_err") for r in acc]
    ve = [fl(r, "handoff_vel_err") for r in acc]
    ce = [fl(r, "constraint_error") for r in acc]
    out["solve_ms_med"], out["solve_ms_p90"] = med(ms), q(ms, 0.9)
    out["solve_ms_max"] = round(max(ms), 2)
    # Deadline miss: the solve finished after the activation instant it was
    # scheduled for. `solve_finish_late_by_sec` is already logged per solve.
    ok = [v for v in late if not math.isnan(v)]
    out["deadline_miss_rate"] = round(sum(1 for v in ok if v > 0) / len(ok), 4) if ok else ""
    out["track_err_med"], out["track_err_max"] = med(pe), round(max(pe), 4)
    out["vel_err_med"] = med(ve)
    out["worst_constraint"] = max(v for v in ce if not math.isnan(v))
    for r in acc:
        s = r.get("x0_source", "")
        if s == "measured_seed":
            out["src_measured"] += 1
        elif s == "predicted_handoff":
            out["src_predicted"] += 1
        elif s == "recovery_live":
            out["src_recovery"] += 1
    return out


def ensure_executed_csv(cell):
    """moving_executed.csv / constraints.csv come from the bag, not the planner.
    Convert on demand so the campaign runner does not have to pay for it inline."""
    ex = os.path.join(cell, "moving_executed.csv")
    if os.path.exists(ex):
        return
    bags = [b for b in os.listdir(cell) if b.startswith("flight_") and b.endswith(".bag")]
    if not bags:
        return
    logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    # ABSOLUTE paths: the command cd's into logs/ first, so a relative cell path
    # (which is what you get from `scenario_metrics.py logs/<campaign>`)
    # resolved to logs/logs/<campaign>/... and the conversion silently did
    # nothing -- leaving every touchdown and STC-violation column blank.
    acell = os.path.abspath(cell)
    cmd = ("source /opt/ros/noetic/setup.bash && "
           "source /home/lab/mavros_ws/devel/setup.bash && "
           f"cd {os.path.abspath(logs)} && timeout 900 python3 bag_to_stc_csv.py "
           f"{os.path.join(acell, bags[0])} --out {acell}")
    r = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
    if not os.path.exists(ex):
        # Do not swallow it: a failed conversion is indistinguishable from a
        # crashed run once the columns are blank.
        print(f"  [warn] bag->csv failed for {os.path.basename(cell)}: "
              f"{(r.stderr or r.stdout).strip().splitlines()[-1:] or ['no output']}")


def flight_stats(cell):
    """Flown STC violation + touchdown error, scored against that run's own
    constraints.csv and gated on the logged CTCS activation flags."""
    ex = os.path.join(cell, "moving_executed.csv")
    cn = os.path.join(cell, "constraints.csv")
    out = dict(v_target_max="", v_target_med="", beta_at_activation="", beta_abs="",
               trig_land_z="", trig_land_t="", trig_land_rxy="",
               trig_los_z="", trig_los_t="", trig_los_rxy="",
               trig_stage_off_z="", trig_stage_off_t="", trig_stage_off_rxy="",
               td_lateral="", td_z="", viol_glideslope="", viol_speed="",
               viol_tilt="", viol_omega="", max_stc_viol="")
    if not (os.path.exists(ex) and os.path.exists(cn)):
        return out
    C = {r["param"]: float(r["value"]) for r in csv.DictReader(open(cn))}
    rows = list(csv.DictReader(open(ex)))
    if not rows:
        return out
    f = lambda r, k: float(r[k])
    # Trim the force_disarm free-fall: the motor cut is not a tracking failure.
    cut = len(rows)
    for i in range(1, len(rows)):
        dt = f(rows[i], "t_acc") - f(rows[i - 1], "t_acc")
        if dt <= 0:
            continue
        if (f(rows[i], "rvz") - f(rows[i - 1], "rvz")) / dt < -6.0 and f(rows[i], "rz") < 0.4:
            cut = i
            break
    rows = rows[:cut]
    if not rows:
        return out
    # Restrict to the DESCENT. moving_executed.csv covers the whole flight, and
    # during takeoff the drone sits on the ground far from the target -- rz is
    # then below alt_trig, so an alt_trig-only filter counts takeoff as landing
    # and reports metre-scale glideslope "violations". Start after the last time
    # the drone was at staging altitude.
    last_high = None
    for i, r in enumerate(rows):
        if f(r, "rz") > 1.3:
            last_high = i
    if last_high is not None:
        rows = rows[last_high:]
    if not rows:
        return out
    pp = lambda v: v if v > 0 else 0.0
    gs = sp = ti = om = 0.0
    for r in rows:
        if f(r, "rz") >= C.get("alt_trig", 0.8):
            continue                      # landing STC block not active yet
        rxy = math.hypot(f(r, "rx"), f(r, "ry"))
        v = math.sqrt(sum(f(r, k) ** 2 for k in ("rvx", "rvy", "rvz")))
        tilt = 2 * math.degrees(math.asin(min(1.0, math.hypot(f(r, "qx"), f(r, "qy")))))
        omn = math.sqrt(sum(f(r, k) ** 2 for k in ("wx", "wy", "wz")))
        gs = max(gs, pp(rxy - C["gs_cone_tan"] * (f(r, "rz") + C["gs_apex_offset"])))
        sp = max(sp, pp(v - C["v_land_stc_max"]))
        ti = max(ti, pp(tilt - C["tilt_land_stc_deg"]))
        om = max(om, pp(omn - C["omega_land_stc_max"]))
    # Achieved staging bearing AT ACTIVATION, computed here rather than in the
    # runner. The runner can only sample it before the trigger (stale by the
    # trigger latency) or after (by which point the drone is already descending
    # and the target->drone bearing has moved for reasons that have nothing to
    # do with the target). From the bag it is exact: take the first sample of
    # the descent window, where rx/ry are already target-relative.
    # WHERE and WHEN each state-triggered constraint switched on. The CTCS
    # activation flags are already logged per sample; record the altitude and
    # the time-since-descent-start at which each first goes positive, so the
    # trigger behaviour can be compared across motions instead of assumed
    # identical. a_stage is a de-activation (it is on far from the pad and
    # turns OFF on capture), so its LAST active sample is the meaningful one.
    t0d = f(rows[0], "t_acc")
    for key, col in (("land", "ctcs_a_altitude_state"), ("los", "ctcs_a_los")):
        if col not in rows[0]:
            continue
        for r in rows:
            if f(r, col) > 1e-9:
                out[f"trig_{key}_z"] = round(f(r, "rz"), 4)
                out[f"trig_{key}_t"] = round(f(r, "t_acc") - t0d, 3)
                out[f"trig_{key}_rxy"] = round(math.hypot(f(r, "rx"), f(r, "ry")), 4)
                break
    if "ctcs_a_stage" in rows[0]:
        last = None
        for r in rows:
            if f(r, "ctcs_a_stage") > 1e-9:
                last = r
        if last is not None:
            out["trig_stage_off_z"] = round(f(last, "rz"), 4)
            out["trig_stage_off_t"] = round(f(last, "t_acc") - t0d, 3)
            out["trig_stage_off_rxy"] = round(math.hypot(f(last, "rx"), f(last, "ry")), 4)

    # Peak target speed over the descent, measured from the bag rather than
    # from the drawn parameters. This is the variable that turned out to
    # explain every failure in MC1, so it belongs in samples.csv.
    tv = [math.hypot(f(r, "tvx"), f(r, "tvy")) for r in rows]
    if tv:
        out["v_target_max"] = round(max(tv), 4)
        out["v_target_med"] = round(sorted(tv)[len(tv) // 2], 4)
    r0 = rows[0]
    tvx, tvy = f(r0, "tvx"), f(r0, "tvy")
    if math.hypot(tvx, tvy) > 1e-6:
        b = math.atan2(f(r0, "ry"), f(r0, "rx")) - math.atan2(tvy, tvx)
        b = (b + math.pi) % (2 * math.pi) - math.pi
        out["beta_at_activation"] = round(b, 4)
        out["beta_abs"] = round(abs(b), 4)
    td = rows[-1]
    out.update(td_lateral=round(math.hypot(f(td, "rx"), f(td, "ry")), 4),
               td_z=round(f(td, "rz"), 4),
               viol_glideslope=round(gs, 4), viol_speed=round(sp, 4),
               viol_tilt=round(ti, 4), viol_omega=round(om, 4),
               max_stc_viol=round(max(gs, sp, ti, om), 4))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    args = ap.parse_args()

    rows = []
    for d in sorted(os.listdir(args.campaign)):
        cell = os.path.join(args.campaign, d)
        meta_f = os.path.join(cell, "cell.json")
        if not os.path.isdir(cell) or not os.path.exists(meta_f):
            continue
        m = json.load(open(meta_f))
        r = dict(motion=m["motion"], uncertainty=m["uncertainty"], trial=m["trial"],
                 seed=m["seed"], result=m["result"], wall_s=m["wall_s"],
                 landed=1 if m["result"] == "LANDED" else 0,
                 # Engagement geometry actually achieved. The offline screen is
                 # parameterised on this bearing, so if a cell fails it is the
                 # first thing to check -- a large stage_beta_err means the run
                 # flew a geometry the screen never validated, which is a
                 # harness problem, not a planner result.
                 stage_alpha=m.get("stage_alpha", ""),
                 stage_beta=m.get("stage_beta", ""),
                 stage_beta_err=m.get("stage_beta_err", ""),
                 stage_beta_wait_s=m.get("stage_beta_wait_s", ""))
        ensure_executed_csv(cell)
        r.update(solver_stats(cell))
        r.update(flight_stats(cell))
        rows.append(r)

    if not rows:
        print("no cells found")
        return
    sp = os.path.join(args.campaign, "samples.csv")
    with open(sp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    num = ["trig_land_z", "trig_land_t", "trig_los_z", "trig_los_t",
           "trig_stage_off_z", "trig_stage_off_t",
           "v_target_max", "beta_abs", "stage_beta_err", "td_lateral", "max_stc_viol", "viol_glideslope", "track_err_med",
           "track_err_max", "solve_ms_med", "solve_ms_p90", "deadline_miss_rate",
           "n_accept", "n_reject"]
    summ = []
    for mo in sorted({r["motion"] for r in rows}):
        for lv in ["zero", "nominal", "high"]:
            g = [r for r in rows if r["motion"] == mo and r["uncertainty"] == lv]
            if not g:
                continue
            s = dict(motion=mo, uncertainty=lv, n=len(g),
                     land_rate=round(sum(r["landed"] for r in g) / len(g), 3),
                     results=";".join(sorted({r["result"] for r in g})))
            for k in num:
                vals = [r[k] for r in g if isinstance(r[k], (int, float))]
                s[k + "_med"] = med(vals)
            summ.append(s)
    up = os.path.join(args.campaign, "summary.csv")
    with open(up, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)

    print(f"{len(rows)} trials -> {sp}\n{len(summ)} cells -> {up}\n")
    hdr = (f"{'motion':17}{'uncert':9}{'n':>3}{'land':>7}{'td_lat':>10}{'stc_viol':>10}"
           f"{'trk_err':>10}{'ms_med':>9}{'miss':>8}{'acc/rej':>10}")
    print(hdr + "\n" + "-" * len(hdr))
    for s in summ:
        print(f"{s['motion']:17}{s['uncertainty']:9}{s['n']:>3}"
              f"{s['land_rate']*100:>6.0f}%{str(s['td_lateral_med']):>10}"
              f"{str(s['max_stc_viol_med']):>10}{str(s['track_err_med_med']):>10}"
              f"{str(s['solve_ms_med_med']):>9}{str(s['deadline_miss_rate_med']):>8}"
              f"{str(s['n_accept_med'])+'/'+str(s['n_reject_med']):>10}")


if __name__ == "__main__":
    main()
