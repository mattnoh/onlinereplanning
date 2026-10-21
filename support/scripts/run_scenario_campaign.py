#!/usr/bin/env python3
"""Closed-loop PX4 SITL campaign: target motions x target-estimate uncertainty.

One cell = (motion, uncertainty level, trial). Each cell brings up a full stack
(PX4+Gazebo -> MAVROS -> target_publisher -> planner), flies the hover profile,
waits for a REAL settle, applies the landing profile, waits for touchdown or
timeout, then tears the stack down in the documented order (planner SIGINT first
so the bag finalizes; kill the rest by PID, never `pkill -f px4` which would
match the runner's own shell).

Resumable: a cell is skipped only when its cell.json holds a final result and
the campaign configuration hash it was flown with matches the current one.
Partial cells are moved to _attempts/ and rerun; every attempt is appended to
ledger.csv. With --continue-on-failure a failed cell is recorded and the
campaign continues -- failures must stay in the denominator. Failures before
the landing profile is applied (no settle, never armed, harness errors) are
infrastructure attempts: they are preserved and retried up to --infra-retries
times. Failures after activation are final outcomes and are not retried.

Usage:
  run_scenario_campaign.py --trials 4 --out logs/campaign_YYYYMMDD
  run_scenario_campaign.py --trials 4 --out ... --reuse      # resume
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import zlib

PKG = os.environ.get("COMANDO_PACKAGE", os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
PX4 = "/home/lab/projects/px4/PX4-Autopilot"
# Package actually launched (an overlay workspace may run another revision);
# its launch files and commit define the campaign identity.
RUNPKG = os.environ.get("COMANDO_RUN_PACKAGE", PKG)
SETUP = os.environ.get("COMANDO_ROS_SETUP", "source /opt/ros/noetic/setup.bash && source /home/lab/mavros_ws/devel/setup.bash")

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

# Controller outcomes that count in a campaign denominator. Anything else is an
# infrastructure attempt (the controller under test never got the landing).
VALID_RESULTS = ("LANDED", "TIMEOUT", "POLICY_EXPIRED", "STALE_STATE",
                 "UNCOMMANDED_DISARM", "INITIALIZATION_FAILED", "NO_FEASIBLE_START")
DEVEL = os.environ.get("COMANDO_DEVEL_LIB",
                       "/home/lab/mavros_ws/devel/.private/comando_planner/lib/comando_planner")


# Staging geometry. STAGE_R matches the |(1.4, 1.4)| engagement range the
# campaign has always used; STAGE_ALPHA is the bearing of the drone from the
# target, measured from the target's velocity direction, and is chosen from
# tools_motion_seed_screen's `alpha` sweep.
STAGE_R = 1.9799
STAGE_ALPHA = float(os.environ.get("CAMPAIGN_STAGE_ALPHA", "0.0"))
BEARING_TOL = float(os.environ.get("CAMPAIGN_BEARING_TOL", "0.35"))   # rad
BEARING_WAIT_S = float(os.environ.get("CAMPAIGN_BEARING_WAIT", "30")) # s
# Seconds between await_bearing returning and the landing OCP actually
# activating (apply_profile.sh round trip + the planner's 10 Hz param poll).
TRIGGER_LEAD = float(os.environ.get("CAMPAIGN_TRIGGER_LEAD", "2.5"))  # s


def read_target():
    """(x, y, vx, vy) of /target/odom, or zeros if it cannot be read."""
    out = sh(f"{SETUP} && timeout 6 rostopic echo -n1 /target/odom 2>/dev/null | "
             "python3 -c \"import sys,re;t=sys.stdin.read();"
             "p=re.search(r'position:.*?x: ([-0-9.e]+).*?y: ([-0-9.e]+)',t,re.S);"
             "v=re.search(r'twist:.*?linear:.*?x: ([-0-9.e]+).*?y: ([-0-9.e]+)',t,re.S);"
             "print(p.group(1) if p else 0, p.group(2) if p else 0,"
             " v.group(1) if v else 0, v.group(2) if v else 0)\"", timeout=20)
    try:
        vals = [float(v) for v in out.stdout.split()]
        return vals if len(vals) == 4 else [0.0, 0.0, 0.0, 0.0]
    except Exception:
        return [0.0, 0.0, 0.0, 0.0]


def read_drone_xy():
    out = sh(f"{SETUP} && timeout 6 rostopic echo -n1 /mavros/local_position/pose "
             "2>/dev/null | python3 -c \"import sys,re;t=sys.stdin.read();"
             "m=re.search(r'position:.*?x: ([-0-9.e]+).*?y: ([-0-9.e]+)',t,re.S);"
             "print(m.group(1),m.group(2)) if m else print(0,0)\"", timeout=20)
    try:
        return [float(v) for v in out.stdout.split()][:2]
    except Exception:
        return [0.0, 0.0]


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def bearing_probe():
    """(beta, rate) in rad and rad/s, or (None, None). One rospy process reads
    both topics in ~0.2 s; the old two-rostopic-echo poll took 3-4 s."""
    r = sh(f"{SETUP} && timeout 20 python3 {PKG}/support/scripts/_bearing_probe.py", timeout=30)
    try:
        b, d = r.stdout.strip().split()[:2]
        b, d = float(b), float(d)
        if math.isnan(b):
            return None, None
        return b, (None if math.isnan(d) else d)
    except Exception:
        return None, None


def await_bearing(alpha, tol, budget, straight=False):
    """Time the landing trigger so the staging bearing is `alpha` AT ACTIVATION.

    Two latencies defeated the earlier attempts and both are compensated here:

      * the probe itself takes ~1.2 s (it samples twice to get the rate), so a
        verify-after-sleep loop measured beta AFTER it had already drifted past
        the window -- at omega=0.55 rad/s, 1.2 s is 0.66 rad, nearly twice the
        +-0.35 rad tolerance. The loop could never converge and burned its
        budget overshooting. There is now exactly ONE probe and no verify
        before the trigger.
      * apply_profile.sh takes ~TRIGGER_LEAD seconds to actually activate the
        landing OCP, during which the target keeps moving. We therefore aim at
        the bearing the target will have THEN, not now.

    Returns (predicted beta at activation, seconds waited).
    """
    t0 = time.time()
    beta, rate = bearing_probe()
    if beta is None:
        return None, time.time() - t0
    if straight or rate is None or abs(rate) < 1e-3:
        # A folded straight target has constant bearing between reversals.
        # Its next reversal can align the requested approach; firing now at
        # beta=pi silently tests the opposite side of the saved scenario.
        while abs(wrap_pi(alpha-beta)) > tol and time.time()-t0 < budget:
            time.sleep(.5)
            observed, _ = bearing_probe()
            if observed is not None:beta=observed
        return beta, time.time() - t0
    # Where beta will be when the trigger lands if we fire right now.
    beta_at_trig = wrap_pi(beta + rate * TRIGGER_LEAD)
    err = wrap_pi(alpha - beta_at_trig)
    if abs(err) <= tol:
        return beta_at_trig, time.time() - t0
    dt = err / rate
    period = 2.0 * math.pi / abs(rate)
    while dt < 0.0:
        dt += period
    if dt > budget - (time.time() - t0):
        return beta_at_trig, time.time() - t0
    time.sleep(dt)
    return alpha, time.time() - t0


def seed_for(motion, level, trial):
    """Deterministic per-cell seed so any trial is exactly reproducible.

    Uses crc32, NOT hash(): Python randomises str hashing per process
    (PYTHONHASHSEED), so hash('circle') returned 878/36/53 on three
    consecutive runs and the seeds were never actually reproducible.
    """
    return (zlib.crc32(motion.encode()) % 1000) * 10000 \
        + list(UNCERT).index(level) * 100 + trial + 1


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_identity(repo, paths=()):
    head = subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--", *paths],
                           capture_output=True, text=True).stdout.strip()
    return dict(commit=head, dirty_paths=dirty.splitlines())


def campaign_identity(args):
    """Everything that defines the flown controller and scenario. Commit ids
    are recorded separately: checkpoint commits during a campaign must not
    change the identity of an unchanged binary/configuration."""
    env = {k: v for k, v in os.environ.items() if k.startswith(("SZMUK_", "MPC_", "PX4_"))}
    def same(a, b):
        try:
            return float(a) == float(b)
        except ValueError:
            return a == b
    clash = {k: (env[k], v) for k, v in PLANNER_ENV.items() if k in env and not same(env[k], v)}
    if clash:
        raise SystemExit(f"runner PLANNER_ENV would override the sourced preset: {clash}")
    env.update(PLANNER_ENV)
    launch = os.path.join(RUNPKG, "support/transport/ros1/launch")
    ident = dict(
        planner_environment=dict(sorted(env.items())),
        profile_sha256=sha256(args.profile),
        planner_binary_sha256=sha256(os.path.join(DEVEL, "comando_planner")),
        target_publisher_sha256=sha256(os.path.join(DEVEL, "target_publisher")),
        planner_launch_sha256=sha256(os.path.join(launch, "planner_launch.launch")),
        target_launch_sha256=sha256(os.path.join(launch, "target_launch.launch")),
        hover_thrust=args.hover_thrust, stc_z_stage=args.stc_z_stage,
        stc_los_alt_trig=args.stc_los_alt_trig, start=args.start,
        staging=args.staging, bearing_policy=args.bearing_policy,
        shape_random=args.shape_random, landing_timeout=args.landing_timeout,
        motions={m: MOTIONS[m] for m in args.motions.split(",")},
        levels={l: UNCERT[l] for l in args.levels.split(",")})
    blob = json.dumps(ident, sort_keys=True).encode()
    return hashlib.sha256(blob).hexdigest()[:16], ident


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


def wait_for(desc, probe, timeout, poll=2.0):
    """Poll `probe` (a zero-arg predicate) until true. Replaces a fixed sleep.

    The old fixed sleeps (45 + 22 + 10 + 25 = 102 s) dominated the ~170 s per
    run. They are kept as the TIMEOUT here, so the worst case is unchanged and
    the typical case is much faster. Returns the seconds waited.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if probe():
                return time.time() - t0
        except Exception:
            pass
        time.sleep(poll)
    print(f"  [warn] {desc} not ready after {timeout:.0f}s; continuing", flush=True)
    return time.time() - t0


def live_exe(name):
    """Resolved executable of the running node, from /proc (not the launch file)."""
    r = sh(f"pgrep -f '/comando_planner/{name}' | head -1")
    pid = r.stdout.strip()
    try:
        return os.path.realpath(f"/proc/{pid}/exe") if pid else ""
    except OSError:
        return ""


def topic_alive(topic):
    r = sh(f"{SETUP} && timeout 4 rostopic echo -n1 {topic} >/dev/null 2>&1 "
           "&& echo YES", timeout=12)
    return "YES" in r.stdout


def fcu_connected():
    r = sh(f"{SETUP} && timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null "
           "| grep -m1 '^connected:'", timeout=12)
    return "True" in r.stdout


def node_alive(name):
    r = sh(f"{SETUP} && timeout 5 rosnode list 2>/dev/null | grep -c {name}", timeout=14)
    return r.stdout.strip() not in ("", "0")


def fcu_armed_offboard():
    """Planner readiness is ARMED + OFFBOARD, not merely 'the node exists'.

    rosnode list shows comando_planner within a second or two of roslaunch,
    but the planner then runs a 2 s setpoint warmup and retries OFFBOARD+ARM
    against /mavros/state. Gating on node existence fired the hover profile
    DURING that sequence: the planner logged "New command while disarmed ???
    re-engaging OFFBOARD+ARM", accepted command_seq=1 at the instant of
    arming, and flew the open-loop hover from the ground, settling at
    z = 1.59 instead of inside the 1.70-1.90 gate -> NO_SETTLE on run 1.
    """
    # No -m1: it stops at the FIRST match, so only one of the two lines would
    # ever be seen and the mode check would silently never run.
    r = sh(f"{SETUP} && timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null "
           "| grep -E '^armed:|^mode:'", timeout=12)
    out = r.stdout
    return ("armed: True" in out) and ("OFFBOARD" in out)


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
    """Landed = the FCU disarmed after having been armed.

    Do NOT poll the planner log: roslaunch buffers redirected stdout, so
    "landing complete" only appears in the file after the process exits. The
    first campaign run landed successfully and was still recorded as a TIMEOUT
    because of that. `land_action: force_disarm` cuts the motors at touchdown,
    so the arm state is the real-time signal.
    """
    t_end = time.time() + timeout
    seen_armed = False
    while time.time() < t_end:
        # A paused controller cannot finish this command without a new command
        # sequence. Preserve the failure and finalize its bag immediately.
        stopped = sh(f"{SETUP} && rosparam get /comando_planner/control_stop_reason 2>/dev/null", timeout=15).stdout.strip()
        if "direct MPC expired policy" in stopped:
            return "POLICY_EXPIRED"
        if "direct MPC stale state" in stopped:
            return "STALE_STATE"
        r = sh(f"{SETUP} && timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null "
               "| grep -E '^armed:'", timeout=15)
        txt = r.stdout.strip()
        if "True" in txt:
            seen_armed = True
        elif "False" in txt and seen_armed:
            return "LANDED"
        time.sleep(3)
    # Fall back to the (now flushed) log before declaring a timeout.
    try:
        if "landing complete" in open(planner_log, errors="ignore").read():
            return "LANDED"
    except OSError:
        pass
    return "TIMEOUT" if seen_armed else "NEVER_ARMED"


class BinaryMismatch(RuntimeError):
    """The launched node is not the binary in the campaign identity."""


def completed(cell, config_hash):
    """A finished cell with the same campaign identity, or None."""
    try:
        meta = json.load(open(os.path.join(cell, "cell.json")))
    except (OSError, ValueError):
        return None
    if meta.get("config_hash") == config_hash and meta.get("result") and meta.get("final", True):
        return meta
    return None


def preserve_attempt(cell, outdir, label):
    """Move an attempt out of the way without deleting it."""
    if not os.path.isdir(cell):
        return None
    dst = os.path.join(outdir, "_attempts", f"{os.path.basename(cell)}__{label}")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.move(cell, dst)
    return dst


def ledger(outdir, **row):
    path = os.path.join(outdir, "ledger.csv")
    fields = ["time", "tag", "attempt", "event", "result", "teardown", "activated",
              "wall_s", "config_hash", "location"]
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if new:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in fields})


def run_cell(motion, level, trial, outdir, args):
    tag = f"{motion}__{level}__t{trial}"
    cell = os.path.join(outdir, tag)
    os.makedirs(cell, exist_ok=True)
    seed = seed_for(motion, level, trial)
    logs = {k: os.path.join(cell, f"{k}.log") for k in
            ("px4", "mavros", "target", "planner")}
    t0 = time.time()
    # Save inputs before launch, including inherited controller tuning.
    env_snapshot = {k:v for k,v in os.environ.items() if k.startswith(("SZMUK_", "MPC_", "PX4_"))}
    env_snapshot.update(PLANNER_ENV)
    env_snapshot.update(SZMUK_Z_STAGE=str(args.stc_z_stage), SZMUK_LOS_ALT_TRIG=str(args.stc_los_alt_trig))
    json.dump(env_snapshot, open(os.path.join(cell, "planner_environment.json"), "w"), indent=2)
    shutil.copyfile(args.profile, os.path.join(cell, "landing_profile.yaml"))
    revision = subprocess.run(["git", "-C", RUNPKG, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    json.dump(dict(planner_commit=revision, setup=SETUP, config_hash=args.config_hash,
                   solver=git_identity(args.solver_dir, ("alipddp", "problem_maker"))),
              open(os.path.join(cell, "source.json"), "w"), indent=2)
    activated = False
    print(f"  [run ] {tag} seed={seed}", flush=True)

    teardown()
    procs = []
    # Bound before the try: a NO_SETTLE run or an exception skips the gate
    # entirely, and meta below reads both unconditionally.
    beta, beta_wait = None, 0.0
    planner_exe = target_exe = ""
    try:
        # Use the installed SITL binary; a campaign must not rebuild PX4.
        procs.append(spawn(f"cd {PX4}/build/px4_sitl_default/rootfs && "
                           "HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ../bin/px4 "
                           "2>&1 | grep --line-buffered -vE '^pxh>|\\[2K'", logs["px4"]))
        # PX4 is up once it is accepting the offboard UDP link; mavros
        # connecting is the real gate, so just wait for the log banner.
        wait_for("px4", lambda: "Ready for takeoff" in open(logs["px4"]).read()
                 or "INFO  [commander]" in open(logs["px4"]).read(), 60)
        procs.append(spawn(f"{SETUP} && roslaunch mavros px4.launch "
                           'fcu_url:="udp://:14540@127.0.0.1:14557"', logs["mavros"]))
        wait_for("mavros<->fcu", fcu_connected, 45)

        tparams = dict(MOTIONS[motion]); tparams.update(UNCERT[level])
        tparams["target_motion_seed"] = seed
        tparams["target_noise_seed"] = seed
        tparams["target_shape_random"] = "true" if args.shape_random else "false"
        targs = " ".join(f"{k}:={v}" for k, v in tparams.items())
        procs.append(spawn(
            f"{SETUP} && roslaunch comando_planner target_launch.launch "
            f"target_trajectory:={motion} "
            "drone_odom_topic:=/mavros/local_position/odom "
            f"drone_pose_topic:=/mavros/local_position/pose {targs}", logs["target"]))
        wait_for("/target/odom", lambda: topic_alive("/target/odom"), 25)

        env = " ".join(f"{k}={v}" for k, v in PLANNER_ENV.items())
        procs.append(spawn(
            f"{SETUP} && export {env} && roslaunch comando_planner planner_launch.launch "
            f"platform:=mavros drone_name:=px4_drone hover_thrust:={args.hover_thrust} terminal_freeze_enter_vel:=0.30 "
            f"stc_z_stage:={args.stc_z_stage} stc_los_alt_trig:={args.stc_los_alt_trig} log_dir:={cell}", logs["planner"]))
        wait_for("planner node", lambda: node_alive("comando_planner"), 45)
        planner_exe, target_exe = live_exe("comando_planner"), live_exe("target_publisher")
        expected = [os.path.realpath(os.path.join(DEVEL, n)) for n in ("comando_planner", "target_publisher")]
        if [planner_exe, target_exe] != expected:
            raise BinaryMismatch(f"running {planner_exe}, {target_exe}; expected {expected}")
        # ...then wait for it to actually be flying-ready, and give the
        # position hold a moment to stabilise before overriding the target.
        wait_for("armed+offboard", fcu_armed_offboard, 60)
        time.sleep(6)

        # Stage at a fixed bearing FROM THE TARGET'S OWN HEADING, not at a
        # fixed world offset. The old `target + (1.4, 1.4)` meant that once
        # target_motion_seed randomised the heading, the approach bearing
        # became an uncontrolled variable -- and tools_motion_seed_screen
        # showed ~25-30% of bearings are outright infeasible (the cold solve
        # is rejected and the vehicle never leaves the staging hover).
        # Engagement geometry is not what MC1 is asking about, so it is held
        # fixed and only the motion varies.
        # A stop-and-go target may be in its DWELL right now, with |v| = 0 and
        # therefore no heading to stage relative to. Wait for it to be moving
        # rather than defaulting to an arbitrary world bearing -- that default
        # was worth 5/20 failures on stop_and_go in the offline screen.
        tx = ty = tvx = tvy = 0.0
        t_psi = time.time()
        while time.time() - t_psi < 30.0:
            tx, ty, tvx, tvy = read_target()
            if math.hypot(tvx, tvy) > 0.05:
                break
            time.sleep(1.0)
        psi = math.atan2(tvy, tvx) if math.hypot(tvx, tvy) > 1e-6 else 0.0
        hx = tx + STAGE_R * math.cos(psi + STAGE_ALPHA)
        hy = ty + STAGE_R * math.sin(psi + STAGE_ALPHA)
        if args.staging == "offset":
            # MC2 (runner 025351a): fixed world offset, no bearing gate.
            hx, hy = tx + 1.4, ty + 1.4
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
            if getattr(args,"start","original")=="cone":
                sh(f"{SETUP} && rosparam set /comando_planner/prepare_cone_start true",timeout=15)
                time.sleep(3)
            # The target keeps moving during the ~25 s settle, so the bearing
            # we staged at is not the bearing at activation. Wait (briefly)
            # for it to come back into the screened band rather than firing
            # blind; record what we actually got either way.
            gated = getattr(args,"start","original")!="cone" and args.staging=="bearing"
            # original: MC1's await_bearing (543424a) -- no straight-reversal
            # wait, fire after the budget and record the bearing achieved.
            beta, beta_wait = await_bearing(STAGE_ALPHA, BEARING_TOL, BEARING_WAIT_S,
                                            straight=(motion=="straight" and args.bearing_policy=="strict")) if gated else (None,0.0)
            if gated and args.bearing_policy=="strict" and (beta is None or abs(wrap_pi(beta-STAGE_ALPHA))>BEARING_TOL):
                raise RuntimeError(f"Requested approach bearing unavailable: beta={beta}, requested={STAGE_ALPHA}")
            activated = True
            sh(f"{SETUP} && rosrun comando_planner apply_profile.sh /comando_planner "
               f"{args.profile}",
               timeout=60)
            if getattr(args,"start","original")=="cone":
                activation=wait_for("feasible cone activation",lambda: "true" in sh(f"{SETUP} && rosparam get /comando_planner/ordinary_start_verified",timeout=10).stdout.lower(),30,poll=.25)
                if activation<30:
                    def started():
                        for folder,_,files in os.walk(cell):
                            if "solver_events.csv" in files:
                                if any(row.get("event")=="accepted" for row in csv.DictReader(open(os.path.join(folder,"solver_events.csv")))):return True
                        return False
                    ready=wait_for("ordinary first accepted plan",started,15,poll=.25)
                    result=wait_landing(logs["planner"], args.landing_timeout) if ready<15 else "INITIALIZATION_FAILED"
                else:
                    result="NO_FEASIBLE_START"
            else:
                result = wait_landing(logs["planner"], args.landing_timeout)
    except BinaryMismatch as e:
        print(f"  [error] {e}", flush=True)
        result = "BINARY_MISMATCH"
        z = vz = float("nan")
    except Exception as e:                        # keep the campaign alive
        print(f"  [error] {e}",flush=True)
        result = f"HARNESS_FAILED:{type(e).__name__}"
        z = vz = float("nan")
    finally:
        clean = teardown(procs)

    if result=="LANDED" and "Touchdown: OCP stopped" not in open(logs["planner"],errors="ignore").read():
        result="UNCOMMANDED_DISARM"

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
                profile=args.profile, hover_thrust=args.hover_thrust, start=getattr(args,"start","original"),
                stage_xy=[round(hx,3), round(hy,3)] if "hx" in dir() else None,
                result=result, settle_z=z, settle_vz=vz, teardown=clean,
                # Geometry actually achieved at activation. stage_alpha is what
                # we aimed for; stage_beta is what the target's motion left us
                # with after the settle wait. Divergence between them is the
                # thing to check first if a cell fails.
                stage_alpha=round(STAGE_ALPHA, 4),
                stage_beta=(round(beta, 4) if beta is not None else ""),
                stage_beta_err=(round(abs(wrap_pi(beta - STAGE_ALPHA)), 4)
                                if beta is not None else ""),
                stage_beta_wait_s=round(beta_wait, 1),
                activated=activated, config_hash=args.config_hash,
                planner_exe=planner_exe, target_exe=target_exe,
                staging=args.staging, bearing_policy=args.bearing_policy,
                shape_random=args.shape_random, landing_timeout=args.landing_timeout,
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
    ap.add_argument("--start", choices=("original","cone"),default="original",help="cone: target-following preparation and measured initial-feasibility gate")
    ap.add_argument("--profile", default=os.path.join(PKG,"config/ros1/profile_stc_landing_noaug_sitl.yaml"))
    ap.add_argument("--hover-thrust", type=float, default=0.3)
    ap.add_argument("--stc-z-stage", type=float, default=1.55)
    ap.add_argument("--stc-los-alt-trig", type=float, default=1.2)
    ap.add_argument("--motions", default=",".join(MOTIONS))
    ap.add_argument("--levels", default=",".join(UNCERT))
    ap.add_argument("--staging", choices=("bearing", "offset"), default="bearing",
                    help="offset: MC2's target+(1.4,1.4) world offset without a bearing gate")
    ap.add_argument("--bearing-policy", choices=("strict", "original"), default="strict",
                    help="original: MC1's record-and-proceed bearing timing")
    ap.add_argument("--shape-random", type=int, choices=(0, 1), default=1,
                    help="0: legacy fixed target shape (MC2)")
    ap.add_argument("--landing-timeout", type=float, default=300.0)
    ap.add_argument("--continue-on-failure", action="store_true")
    ap.add_argument("--infra-retries", type=int, default=2)
    ap.add_argument("--trial-offset", type=int, default=0)
    ap.add_argument("--solver-dir", default=os.environ.get("COMANDO_SOLVER_DIR",
                    os.path.join(os.path.dirname(PKG), "ALIPDDP-main")))
    args = ap.parse_args()
    args.profile = os.path.abspath(args.profile)
    args.config_hash, identity = campaign_identity(args)

    out = os.path.abspath(args.out) if args.out else os.path.join(
        PKG, "logs", "campaign_" + datetime.datetime.now().strftime("%Y%m%d_%H%M"))
    os.makedirs(out, exist_ok=True)
    motions = args.motions.split(",")
    levels = args.levels.split(",")
    cells = [(m, l, t) for m in motions for l in levels
             for t in range(args.trial_offset, args.trial_offset + args.trials)]
    print(f"campaign -> {out}\n{len(cells)} cells "
          f"({len(motions)} motions x {len(levels)} levels x {args.trials} trials)"
          f"  config {args.config_hash}\n", flush=True)
    manifest = os.path.join(out, "campaign_manifest.json")
    previous = json.load(open(manifest)) if os.path.exists(manifest) else None
    if previous and previous.get("config_hash") != args.config_hash and args.reuse:
        raise SystemExit(f"--reuse with a different configuration: {previous.get('config_hash')} "
                         f"!= {args.config_hash}; use a new --out")
    json.dump(dict(config_hash=args.config_hash, identity=identity, argv=sys.argv,
                   planner=git_identity(RUNPKG), runner=git_identity(PKG), setup=SETUP, solver=git_identity(args.solver_dir, ("alipddp", "problem_maker")),
                   started=previous.get("started") if previous else datetime.datetime.now().isoformat()),
              open(manifest, "w"), indent=2)

    rows = []
    for i, (m, l, t) in enumerate(cells, 1):
        print(f"[{i}/{len(cells)}]", flush=True)
        tag = f"{m}__{l}__t{t}"
        cell = os.path.join(out, tag)
        r = completed(cell, args.config_hash) if args.reuse else None
        if r:
            print(f"  [skip] {tag} ({r['result']})", flush=True)
        else:
            if os.path.isdir(cell):
                dst = preserve_attempt(cell, out, "partial_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
                ledger(out, time=datetime.datetime.now().isoformat(), tag=tag, event="PARTIAL_PRESERVED",
                       config_hash=args.config_hash, location=dst)
            for attempt in range(1, args.infra_retries + 2):
                r = run_cell(m, l, t, out, args)
                if r["result"] == "BINARY_MISMATCH":
                    json.dump(r, open(os.path.join(cell, "cell.json"), "w"), indent=2)
                    ledger(out, time=datetime.datetime.now().isoformat(), tag=tag, attempt=attempt,
                           event="BINARY_MISMATCH", result=r["result"], config_hash=args.config_hash, location=cell)
                    raise SystemExit("launched binary differs from the campaign identity; stopping")
                # Retry only when the controller under test never received the
                # landing command; a post-activation failure is an outcome.
                infra = r["result"] not in VALID_RESULTS and not r["activated"]
                last = attempt == args.infra_retries + 1
                r["attempt"] = attempt
                r["final"] = (not infra) or last
                json.dump(r, open(os.path.join(cell, "cell.json"), "w"), indent=2)
                ledger(out, time=datetime.datetime.now().isoformat(), tag=tag, attempt=attempt,
                       event="INFRA_RETRY" if (infra and not last) else ("INFRA_FINAL" if infra else "OUTCOME"),
                       result=r["result"], teardown=r["teardown"], activated=r["activated"],
                       wall_s=r["wall_s"], config_hash=args.config_hash, location=cell)
                if r["final"]:
                    break
                preserve_attempt(cell, out, f"a{attempt}")
        if r:
            rows.append(r)
            with open(os.path.join(out, "cells.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k not in
                                                  ("sigma", "motion_params")])
                w.writeheader()
                for row in rows:
                    w.writerow({k: v for k, v in row.items()
                                if k not in ("sigma", "motion_params")})
            if not args.continue_on_failure and (r["result"]!="LANDED" or r["teardown"]!="CLEAN"):
                raise SystemExit(f"Stopped after {m}: {r['result']} / {r['teardown']}")
    print(f"\ncampaign complete -> {out}")


if __name__ == "__main__":
    main()
