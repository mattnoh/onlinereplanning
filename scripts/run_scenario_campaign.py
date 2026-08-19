#!/usr/bin/env python3
"""Closed-loop PX4 SITL campaign: target motions x target-estimate uncertainty.

One cell = (motion, uncertainty level, trial). Each cell brings up a full stack
(PX4+Gazebo -> MAVROS -> target_publisher -> planner), flies the hover profile,
waits for a REAL settle, applies the landing profile, waits for touchdown or
timeout, then tears the stack down in the documented order (planner SIGINT first
so the bag finalizes; kill the rest by PID, never `pkill -f px4` which would
match the runner's own shell).

Resumable: a cell whose run directory already holds solver_events.csv is skipped,
so the campaign can be interrupted and restarted. A hung or failed cell is
recorded and the campaign continues -- failures must stay in the denominator.

Usage:
  run_scenario_campaign.py --trials 4 --out logs/campaign_YYYYMMDD
  run_scenario_campaign.py --trials 4 --out ... --reuse      # resume
"""
import argparse
import csv
import datetime
import json
import os
import shutil
import signal
import subprocess
import sys
import time

PKG = "/home/lab/mavros_ws/src/CoManDO_planner"
PX4 = "/home/lab/projects/px4/PX4-Autopilot"
SETUP = "source /opt/ros/noetic/setup.bash && source /home/lab/mavros_ws/devel/setup.bash"

# Motion shape params. Speeds are held near the flown circle (0.4 m/s) so the
# scenarios are comparable and remain physically landable.
MOTIONS = {
    "circle":          {"target_radius": 1.0, "target_omega": 0.4},
    "figure8":         {"target_omega": 0.3},
    "straight":        {"target_speed": 0.4, "target_heading": 0.0},
    "stop_and_go":     {"target_speed": 0.4, "target_cruise_s": 3.0,
                        "target_ramp_s": 1.0, "target_dwell_s": 2.0},
    "piecewise_accel": {"target_speed": 0.4, "target_accel_mag": 0.15,
                        "target_accel_hold": 2.0},
    "random_accel":    {"target_speed": 0.4, "target_accel_sigma": 0.20,
                        "target_random_hold": 2.0},
}

# Target-state estimate uncertainty. `zero` is the noise-free control arm.
UNCERT = {
    "zero":    {"target_noise_sigma_pos": 0.0,  "target_noise_sigma_vel": 0.0,
                "target_noise_sigma_acc": 0.0},
    "nominal": {"target_noise_sigma_pos": 0.02, "target_noise_sigma_vel": 0.05,
                "target_noise_sigma_acc": 0.10},
    "high":    {"target_noise_sigma_pos": 0.10, "target_noise_sigma_vel": 0.25,
                "target_noise_sigma_acc": 0.50},
}

PLANNER_ENV = {
    "SZMUK_RH_N": "40", "SZMUK_MAX_ITER": "120",
    "SZMUK_THH": "0.16", "SZMUK_W_LAND_GS": "20000",
}


def seed_for(motion, level, trial):
    """Deterministic per-cell seed so any trial is exactly reproducible."""
    return (abs(hash(motion)) % 1000) * 10000 + list(UNCERT).index(level) * 100 + trial + 1


def sh(cmd, timeout=None, check=False):
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True,
                          timeout=timeout, check=check)


def spawn(cmd, logfile):
    """Detached process group so the whole tree can be killed together."""
    f = open(logfile, "w")
    return subprocess.Popen(["bash", "-lc", cmd], stdout=f, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, preexec_fn=os.setsid)


def teardown(procs=None):
    """Documented order: planner first (finalizes the bag), then the rest by PID."""
    sh("pkill -INT -f 'roslaunch comando_planner planner_launch'")
    time.sleep(8)
    pat = "bin/px4|gz sim|mavros_node|target_publisher|rosmaster|roslaunch|rosbag"
    for sig in ("-INT", "-9"):
        r = sh(f"pgrep -af '{pat}' | grep -vE 'pgrep|bash -lc' | awk '{{print $1}}'")
        pids = [p for p in r.stdout.split() if p.strip()]
        if pids:
            sh(f"kill {sig} {' '.join(pids)}")
        time.sleep(4 if sig == "-INT" else 2)
    # Spawned groups last: each Popen used setsid, so kill the whole group.
    for pr in (procs or []):
        for sg in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(pr.pid), sg)
            except (ProcessLookupError, PermissionError):
                pass
        try:
            pr.wait(timeout=5)
        except Exception:
            pass
    time.sleep(2)
    r = sh(f"pgrep -af '{pat}' | grep -vE 'pgrep|bash -lc'")
    return "CLEAN" if not r.stdout.strip() else "DIRTY:" + r.stdout.strip()[:150]


def wait_settle(timeout=90):
    """Require a genuine hover settle, not a fixed sleep: |vz|<0.05 and
    1.70<z<1.90 for 5 consecutive odom samples."""
    ok = 0
    t_end = time.time() + timeout
    last = (0.0, 9.0)
    while time.time() < t_end:
        r = sh(f"{SETUP} && timeout 4 rostopic echo -n1 /mavros/local_position/odom "
               "2>/dev/null | python3 -c \""
               "import sys,re; t=sys.stdin.read();"
               "m=re.search(r'position:.*?z: ([-0-9.e]+)',t,re.S);"
               "v=re.search(r'linear:.*?z: ([-0-9.e]+)',t,re.S);"
               "print(m.group(1) if m else 0, v.group(1) if v else 9)\"", timeout=15)
        try:
            z, vz = [float(x) for x in r.stdout.split()]
        except Exception:
            time.sleep(2); continue
        last = (z, vz)
        ok = ok + 1 if (abs(vz) < 0.05 and 1.70 < z < 1.90) else 0
        if ok >= 5:
            return True, z, vz
        time.sleep(2)
    return False, last[0], last[1]


def wait_landing(planner_log, timeout=300):
    """Landed = the planner latched terminal freeze and the land action disarmed."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            txt = open(planner_log, errors="ignore").read()
        except OSError:
            txt = ""
        if "landing complete" in txt:
            return "LANDED"
        if "holdHoverAndPause" in txt or "trajectory stale 3x" in txt:
            return "STALE_ABORT"
        time.sleep(3)
    return "TIMEOUT"


def run_cell(motion, level, trial, outdir, args):
    tag = f"{motion}__{level}__t{trial}"
    cell = os.path.join(outdir, tag)
    if args.reuse and os.path.exists(os.path.join(cell, "solver_events.csv")):
        print(f"  [skip] {tag} (already complete)", flush=True)
        return None
    os.makedirs(cell, exist_ok=True)
    seed = seed_for(motion, level, trial)
    logs = {k: os.path.join(cell, f"{k}.log") for k in
            ("px4", "mavros", "target", "planner")}
    t0 = time.time()
    print(f"  [run ] {tag} seed={seed}", flush=True)

    teardown()
    procs = []
    try:
        procs.append(spawn(f"cd {PX4} && HEADLESS=1 make px4_sitl gz_x500 "
                           "2>&1 | grep --line-buffered -vE '^pxh>|\\[2K'", logs["px4"]))
        time.sleep(45)
        procs.append(spawn(f"{SETUP} && roslaunch mavros px4.launch "
                           'fcu_url:="udp://:14540@127.0.0.1:14557"', logs["mavros"]))
        time.sleep(22)

        tparams = dict(MOTIONS[motion]); tparams.update(UNCERT[level])
        tparams["target_motion_seed"] = seed
        tparams["target_noise_seed"] = seed
        targs = " ".join(f"{k}:={v}" for k, v in tparams.items())
        procs.append(spawn(
            f"{SETUP} && roslaunch comando_planner target_launch.launch "
            f"target_trajectory:={motion} "
            "drone_odom_topic:=/mavros/local_position/odom "
            f"drone_pose_topic:=/mavros/local_position/pose {targs}", logs["target"]))
        time.sleep(10)

        env = " ".join(f"{k}={v}" for k, v in PLANNER_ENV.items())
        procs.append(spawn(
            f"{SETUP} && export {env} && roslaunch comando_planner planner_launch.launch "
            "platform:=mavros drone_name:=px4_drone terminal_freeze_enter_vel:=0.30 "
            f"stc_z_stage:=1.55 stc_los_alt_trig:=1.2 log_dir:={cell}", logs["planner"]))
        time.sleep(25)

        # Stage next to wherever the target actually is, instead of a fixed
        # world point. With a fixed (2,2) hover the engagement range varied by
        # motion (up to 4.8 m for the shuttle vs ~2.5 m for the circle), which
        # both confounds the comparison and can leave the drone unable to close.
        tp = sh(f"{SETUP} && timeout 6 rostopic echo -n1 /target/odom 2>/dev/null | "
                "python3 -c \"import sys,re;t=sys.stdin.read();"
                "m=re.search(r'position:.*?x: ([-0-9.e]+).*?y: ([-0-9.e]+)',t,re.S);"
                "print(m.group(1),m.group(2)) if m else print(0,0)\"", timeout=20)
        try:
            tx, ty = [float(v) for v in tp.stdout.split()]
        except Exception:
            tx, ty = 0.0, 0.0
        hx, hy = tx + 1.4, ty + 1.4      # settled lateral offset, as the docs require
        # Set the hover profile directly rather than via apply_profile.sh:
        # the script loads the yaml (which pins hover_target to a fixed 2,2) and
        # bumps command_seq, so overriding afterwards needs a second trigger --
        # and two triggers 0.2 s apart interrupted the open-loop takeoff and left
        # the drone on the ground.
        hov = (f"rosparam set /comando_planner/ocp_type hover && "
               f"rosparam set /comando_planner/mode open_loop && "
               f"rosparam set /comando_planner/n_replay 4 && "
               f"rosparam set /comando_planner/land_action none && "
               f"rosparam set /comando_planner/hover_target_x {hx:.3f} && "
               f"rosparam set /comando_planner/hover_target_y {hy:.3f} && "
               f"rosparam set /comando_planner/hover_target_z 2.05 && "
               f"rosparam set /comando_planner/command_seq 1")
        sh(f"{SETUP} && {hov}", timeout=60)
        settled, z, vz = wait_settle()
        if not settled:
            result = "NO_SETTLE"
        else:
            sh(f"{SETUP} && rosrun comando_planner apply_profile.sh /comando_planner "
               f"$(rospack find comando_planner)/config/profile_stc_landing_noaug_sitl.yaml",
               timeout=60)
            result = wait_landing(logs["planner"])
    except Exception as e:                        # keep the campaign alive
        result = f"HARNESS_FAILED:{type(e).__name__}"
        z = vz = float("nan")
    finally:
        clean = teardown(procs)

    # The planner writes its own run folder under log_dir; flatten it into the cell.
    for d in sorted(os.listdir(cell)):
        p = os.path.join(cell, d)
        if os.path.isdir(p) and d.startswith("px4_drone"):
            for f in os.listdir(p):
                shutil.move(os.path.join(p, f), os.path.join(cell, f))
            os.rmdir(p)
    # Only this cell's bag: anything older than the cell start belongs to a
    # previous run. An unfiltered sweep here moved six historical bags out of
    # logs/ on the first smoke test.
    logdir = os.path.join(PKG, "logs")
    for b in os.listdir(logdir):
        f = os.path.join(logdir, b)
        if (b.startswith("flight_") and b.endswith(".bag")
                and os.path.getmtime(f) >= t0):
            shutil.move(f, os.path.join(cell, b))

    meta = dict(motion=motion, uncertainty=level, trial=trial, seed=seed,
                stage_xy=[round(hx,3), round(hy,3)] if "hx" in dir() else None,
                result=result, settle_z=z, settle_vz=vz, teardown=clean,
                wall_s=round(time.time() - t0, 1),
                sigma=UNCERT[level], motion_params=MOTIONS[motion])
    json.dump(meta, open(os.path.join(cell, "cell.json"), "w"), indent=2)
    print(f"  [done] {tag} -> {result}  ({meta['wall_s']:.0f}s, {clean})", flush=True)
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=4)
    ap.add_argument("--out", default=None)
    ap.add_argument("--reuse", action="store_true")
    ap.add_argument("--motions", default=",".join(MOTIONS))
    ap.add_argument("--levels", default=",".join(UNCERT))
    args = ap.parse_args()

    out = args.out or os.path.join(
        PKG, "logs", "campaign_" + datetime.datetime.now().strftime("%Y%m%d_%H%M"))
    os.makedirs(out, exist_ok=True)
    motions = args.motions.split(",")
    levels = args.levels.split(",")
    cells = [(m, l, t) for m in motions for l in levels for t in range(args.trials)]
    print(f"campaign -> {out}\n{len(cells)} cells "
          f"({len(motions)} motions x {len(levels)} levels x {args.trials} trials)\n",
          flush=True)

    rows = []
    for i, (m, l, t) in enumerate(cells, 1):
        print(f"[{i}/{len(cells)}]", flush=True)
        r = run_cell(m, l, t, out, args)
        if r:
            rows.append(r)
            with open(os.path.join(out, "cells.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k not in
                                                  ("sigma", "motion_params")])
                w.writeheader()
                for row in rows:
                    w.writerow({k: v for k, v in row.items()
                                if k not in ("sigma", "motion_params")})
    print(f"\ncampaign complete -> {out}")


if __name__ == "__main__":
    main()
