# Direct thrust/body-rate MPC (`mpc` branch)

`stc_landing_mpc` sends optimized thrust acceleration and body rates to MAVROS
`setpoint_raw/attitude`. PX4's position and attitude controllers are bypassed;
its rate controller remains active. `stc_landing_noaug` is retained for comparison.

The horizon is 40 fixed intervals of 0.1 s. Solver state contains the physical
13-state relative position/velocity, absolute quaternion and body angular rate,
prediction time, and previous four controls (18 states total). The eight STC
implications use `StcStageConstraint::Density`, the original scaled interval
budget, and analytic derivatives. The original 3 m/s speed bound is retained.
RK4 subdivision adapts to the identified rate response (two substeps at 0.08 s).
The optimizer penalizes control changes; output also has hard slew limits.

A separate worker optimizes at up to 10 Hz without blocking 50 Hz feedback.
Accepted policies activate by timestamp, without RH replay-node handoffs.
Measured state is projected through the active feedback policy during the
estimated solve lead. First-policy initialization projects the staging hold.
The local quadratic DDP/Riccati tracking backward pass uses the actual 0.02 s
command interval. The optimizer's 0.1 s IPM/barrier gains are retained with its
result but are not reused as a 50 Hz physical controller. Quaternion signs are
aligned, and timing/control-memory states are excluded from fast feedback.

MAVROS body-frame linear velocity is rotated into world ENU for this OCP.
Thrust acceleration maps to normalized throttle using calibrated hover thrust;
activation refuses a mismatch between the adapter and model settings. Commands
are bounded, corrections are limited, and slew is limited to 20 m/s³ thrust
acceleration and 5 rad/s² body-rate changes. Rejected solves retain the valid
policy; stale state or policy expiry invokes position hold and pauses.
Touchdown retains the existing PX4 landing handoff.

## Validated x500 SITL profile

`launch/direct_mpc_sitl.launch` supplies the settings below. These are
simulator-specific estimates; this work does not validate flight hardware.

| Setting | Value |
|---|---:|
| Normalized hover thrust | 0.7282821834 |
| Effective rate response time | 0.08 s |
| Body-rate command bound | 0.35 rad/s |
| Maximum optimizer iterations | 120 |
| Maximum policy age | 2 s |
| Staging floor / LOS trigger altitude | 1.3 m / 1.2 m |
| Raw CTCS interval epsilon / scaling | 0.0016 / 10 |
| Landing glideslope weight | 20000 |

The staging floor is lower than the prior MC1 setting of 1.55 m to provide
headroom for feedback using the actual vehicle state. Other STC thresholds and
weights retain the saved settings. The hover command remains 2.05 m, producing
approximately 1.8 m measured altitude with the existing staging profile.

With prebuilt PX4, MAVROS, and the target publisher running:

```sh
source /opt/ros/noetic/setup.zsh
source /home/lab/mavros_ws/devel/setup.zsh
roslaunch comando_planner direct_mpc_sitl.launch
# First complete the existing staging hover, then activate:
rosrun comando_planner apply_profile.sh /comando_planner \
  /home/lab/mavros_ws/src/CoManDO_planner/config/ros1/profile_stc_landing_mpc_sitl.yaml
```

Calibration used steady hover and ±0.1 rad/s roll/pitch steps. The vehicle stayed
in OFFBOARD and between 1.9918 and 2.0205 m altitude during the steps. The model
uses an effective first-order rate response and linear thrust mapping around
hover; these are approximations, not a full actuator model.

## Tests and SITL results

Build one target at a time with `catkin build comando_planner --no-deps -j1 -p1
--make-args <target>`. `test_direct_mpc` enables Eigen dimension assertions and
checks dynamics/CTCS Jacobians, speed-cone convention, velocity frames, thrust
units, asynchronous submission, quaternion signs, expiry, and a moving-target
closed loop. `test_stc_landing_noaug_ctcs` verifies legacy interface parity.

The checked campaign used planner commit `604510a`, the same three saved MC1
trial-0 seeds and target-motion settings, zero noise, and the calibrated profile.
Full artifacts are in `logs/direct_mpc_20261013/mc1_checked/`. Compact manifests,
metrics and control audits are committed in `docs/benchmarks/direct_mpc/`.

| Motion | Result | Touchdown lateral error (m) | Median accepted solve (ms) | Accepted / rejected solves |
|---|---|---:|---:|---:|
| straight | LANDED | 0.0310 | 226.2001 | 28 / 2 |
| circle | LANDED | 0.0215 | 299.0240 | 24 / 0 |
| figure8 | LANDED | 0.0380 | 326.9183 | 13 / 10 |

Feedback period was 20 ms; the largest observed gap was 20.44 ms. All three
stayed in OFFBOARD, respected command slew limits, and sent **zero position
commands during direct control**. Figure8 retained a valid policy through
rejections, reaching a maximum policy age of 1.79 s. Optimization is slower than
10 Hz in these runs, while command feedback remains 50 Hz.

The preceding noaug-interface checks had lateral errors of 0.0543, 0.1070,
and 0.2910 m for straight, circle, and figure8 respectively. These are single
trials with different controller tuning and engagement timing, not evidence of
a general accuracy improvement. Flown STC violations remain nonzero; touchdown
success does not establish exact constraint satisfaction. RH handoff/deadline
metrics are not meaningful for this direct controller; use `control_audit.json`.

Earlier unsuccessful attempts are preserved under `logs/direct_mpc_20261013/`.
They exposed first-policy timing, feedback-gain cadence, Jacobian dimensions,
and insufficient real-state staging headroom. The final checked campaign uses
the corrected implementation; failed trials are not counted as successful runs.

`direct_mpc_controls.csv` records nominal controls, applied feedback, output,
limits, policy age and tracking errors. `support/scripts/analyze_direct_mpc.py` audits
command timing/slew and PX4 mode from those logs and the bag.
