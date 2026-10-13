# Noaug CTCS interface migration

The `stc_landing_noaug` OCP now adds a `StcStageConstraint<double>` in
`StcForm::Density` instead of its local `CtcsIntervalStepIneqCon`.
`makeCtcsConstraint()` authors the eight existing implications: loose tilt,
loose angular speed, staging, LOS, landing speed, landing tilt, landing
angular speed, and landing glideslope. Every implication supplies analytic
trigger and consequence derivatives.

The interface uses the host's continuous relative dynamics and Jacobians,
one RK4 step per interval, and the same constant-acceleration target predictor.
The 14-state noaug layout, aggregate interval budget, tuning, trigger modes,
and weight scaling are preserved. Each weight is divided by `CTCS_Y_SCALE`,
as is `CTCS_STEP_EPS`. The original density and independent interval integral
remain available for diagnostics and regression comparisons.

## Baseline before implementation changes

Run on 2026-10-13 from planner commit `ed2bb74`, solver commit `d0c05cfb`.
Full bags, cell metadata, solver traces, executed trajectories, and metrics:
`logs/ctcs_interface_20261013/baseline_saved/`.
The run manifest includes the baseline planner binary SHA-256. A compact
copy of the manifest and metrics is saved in `docs/benchmarks/ctcs_noaug/`.

One noise-free trial per motion, using MC1 trial-0 seeds and configuration:

| Motion | Seed | Result | Touchdown lateral error (m) | Median solve time (ms) | Rejected solves |
|---|---:|---|---:|---:|---:|
| circle | 8570001 | LANDED | 0.1031 | 132.8057 | 0 |
| figure8 | 7060001 | LANDED | 0.1610 | 128.2894 | 0 |
| straight | 3230001 | LANDED | 0.0366 | 135.9274 | 0 |

These three trials are smoke checks, not a repeat of the full MC1 campaign.
In particular, successful touchdown does not imply zero flown STC violation;
the stored metrics include the observed violations and activation geometry.

Settings: `SZMUK_RH_N=40`, `SZMUK_MAX_ITER=120`, `SZMUK_THH=0.16`,
`SZMUK_W_LAND_GS=20000`, `stc_z_stage=1.55`, `stc_los_alt_trig=1.2`,
`terminal_freeze_enter_vel=0.30`, and the saved
`config/profile_stc_landing_noaug_sitl.yaml` (`n_replay=1`). The saved hover
command is 2.05 m, compensating for the measured SITL hover offset to reach
approximately 1.8 m. Target settings and seeds are recorded in the manifest.

The first launch attempted an unnecessary PX4 rebuild and was interrupted.
A subsequent attempt with a 1.8 m hover command failed the settle gate; that
command was restored to the saved 2.05 m before the baseline reported above.
Those attempts are retained in the separate `baseline/` and
`baseline_prebuilt/` directories and are excluded from this comparison.

## Verification

`test_stc_landing_noaug_ctcs` compares the shared row against the independent
original density/integral at 160 states and controls, including capture-region
states. It checks every state/control Jacobian entry, including free time.
Run it with the default environment, `SZMUK_STAGE_TRIGGER=settle`,
`SZMUK_RH_CAPTURE_LANDING_TRIGGER=1`, and `SZMUK_QUAD_TRIG_STC=0`.

The existing `test_stc_landing_noaug` additionally exercises the cold solve,
planner acceptance gate, and five warm-started receding-horizon solves.

Build each target separately with one job:

```sh
source /opt/ros/noetic/setup.zsh
catkin build comando_planner --no-deps -j1 -p1 --make-args test_stc_landing_noaug_ctcs
catkin build comando_planner --no-deps -j1 -p1 --make-args test_stc_landing_noaug
catkin build comando_planner --no-deps -j1 -p1 --make-args comando_planner
```

All parity modes passed (including nonzero stage/LOS trigger floors).
Maximum relative value discrepancy was below `6e-16`, and maximum normalized
Jacobian discrepancy against finite differences was below `3.7e-7`.
The existing noaug regression passed its cold solve and all five warm-started
solves; relative distance decreased from 3.70945 m to 0.114301 m.

## Post-migration SITL check

Planner commit `a450204`, same saved tuning, trial seeds, and motion settings.
Full artifacts: `logs/ctcs_interface_20261013/ctcs_interface/`.
Manifest and per-trial metrics are also committed in
`docs/benchmarks/ctcs_noaug/ctcs_manifest.json` and `ctcs_samples.csv`.

| Motion | Result | Lateral error before → after (m) | Median solve time before → after (ms) | Rejected solves after |
|---|---|---:|---:|---:|
| circle | LANDED | 0.1031 → 0.1070 | 132.8057 → 84.8193 | 0 |
| figure8 | LANDED | 0.1610 → 0.2910 | 128.2894 → 81.5908 | 0 |
| straight | LANDED | 0.0366 → 0.0543 | 135.9274 → 88.5059 | 0 |

All three cases had zero logged deadline misses and clean teardown. Solve
times were lower in these checks; touchdown error increased, especially for
figure8. These are single closed-loop trials, and target phase/engagement
geometry varies with wall-clock timing, so they do not establish a general
landing-accuracy improvement or identify the cause of the figure8 difference.
The independent parity checks establish that the migrated row preserves the
original density and interval budget; it now uses analytic interval derivatives
in place of differences of the complete interval integral.
