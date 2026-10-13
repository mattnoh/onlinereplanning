# MAVROS / PX4 SITL

The active package is `/home/lab/mavros_ws/src/CoManDO_planner`. The old package is archived under `comando_planner_old` and excluded from catkin discovery. Only `hover`, `stc_landing_noaug`, and `stc_landing_mpc` are registered.

## Build

```sh
source /opt/ros/noetic/setup.bash
source /home/lab/mavros_ws/devel/setup.bash
catkin build comando_planner --no-deps -j1 -p1 --make-args comando_planner
catkin build comando_planner --no-deps -j1 -p1 --make-args target_publisher
```

Build one executable at a time. Tests are opt-in (`-DCOMANDO_BUILD_TESTS=ON`) and have six focused targets; research benchmark executables are no longer part of this package.

## Flight modes

RH uses `stc_landing_noaug`, a 14-state variable-time OCP with the shared CTCS interval interface. It sends planned position/velocity/yaw commands to PX4. MPC uses `stc_landing_mpc`, an 18-state fixed-grid OCP sending thrust acceleration and body rates, evaluated from physical feedback at 50 Hz. Both use the shared controller core, parameter defaults and constant-acceleration predictor.

```sh
roslaunch comando_planner target_launch.launch target_trajectory:=circle
roslaunch comando_planner planner_launch.launch platform:=mavros
rosrun comando_planner apply_profile.sh /comando_planner \
  $(rospack find comando_planner)/config/ros1/profile_hover_sitl.yaml
```

After staging, apply `config/ros1/profile_stc_landing_noaug_sitl.yaml` for RH. For direct MPC, launch the planner through `direct_mpc_sitl.launch` from startup, then apply `config/ros1/profile_stc_landing_mpc_sitl.yaml`. This ensures the adapter and MPC model agree on calibrated hover thrust. The launcher supplies the validated x500 values: hover thrust `0.7282821834`, rate response `0.08 s`, rate bound `0.35 rad/s`, maximum 120 iterations, and policy expiry `2 s`.

The profile script increments `command_seq` to activate a command. Environment-based mathematical settings are loaded at process startup; restart to change them. Sensor values pass into the core in world ENU, with MAVROS body-frame linear velocity rotated at the ROS boundary. Rate/thrust packets use the existing MAVROS attitude interface. Invalid/stale feedback or an expired policy pauses direct MPC into hold.

## Saved MC1 checks

RH: `SZMUK_RH_N=40`, `SZMUK_MAX_ITER=120`, `SZMUK_THH=0.16`, `SZMUK_W_LAND_GS=20000`, staging floor `1.55 m`, LOS trigger `1.2 m`, replay prefix 1. Direct MPC uses staging floor `1.3 m` and the calibration above. Terminal freeze velocity threshold is `0.30 m/s`; the hover staging command is `2.05 m`.

The sequential campaign saves the applied profile, inherited SZMUK/MPC environment, source revision, deterministic motion seed and activation geometry per cell. Example for one RH target:

```sh
python3 support/scripts/run_scenario_campaign.py --motions circle --levels zero \
  --trials 1 --out /tmp/comando-rh-check
python3 support/scripts/scenario_metrics.py /tmp/comando-rh-check
```

Use one target at a time and the prebuilt PX4 simulator. Do not launch a second simulator or an uncapped build alongside it. The completed six RH/MPC flights are recorded in `CLEAN_PLANNER_VALIDATION.md`; successful landing does not imply exact flown STC satisfaction.

## Existing plots

`support/tools/analysis/analyze_flight.py` converts a bag and invokes the existing 3D/state plotters. `plot_planned_states.py` and `plot_sitl_planned_overlay.py` show planner states and horizon overlays. `rviz.launch` uses the retained marker script and RViz settings. Generated flight/plot outputs belong in a run directory, not in the source tree.
