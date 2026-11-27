# CoManDO planner — Aligator, ROS 2 (Crazyflie / CrazySim)

Receding-horizon landing on a moving platform with continuous-time and state-triggered
constraints (CT-cSTC). The OCP is solved with Aligator's ProxDDP (Jallet et al., T-RO 2025);
the constraints come from [CTCSTCDDP](https://github.com/mattnoh/CTCSTCDDP), the CTCS/STC
interface for Aligator. This branch (`aligator-ros2`) drives a Crazyflie through crazyswarm2.
The MAVROS / PX4 version is branch `aligator-ros1`.

**Status:** builds and runs the full CrazySim stack. The landing does not complete yet: online
solves reach the iteration cap infeasible and are rejected, so the vehicle holds at staging.

## Layout
```
include/ocp/                    OCPs (stc_landing_noaug, hover) on Aligator
include/planner_core/, src/     ROS-free core: OCP registry, solve/acceptance, replay
support/transport/ros2/         ROS 2 node, Crazyflie platform adapter, launch files, CrazySim runner
config/ros2/                    hover and landing profiles
support/test/                   OCP derivative checks, perfect-plant receding-horizon test
third_party/ctcs_stc            CTCSTCDDP (git submodule)
aligator                        Aligator 0.19.1-19 (git submodule; built once, see below)
```

## Setup
Ubuntu 22.04, ROS 2 Humble, a colcon workspace with
[crazyswarm2](https://github.com/IMRCLab/crazyswarm2) (`crazyflie`, `crazyflie_interfaces`)
and [CrazySim](https://github.com/gtfactslab/CrazySim) with its SITL firmware built
(`crazyflie-firmware/sitl_make/build/cf2`). Run every ROS 2 command in bash.

1. Clone with submodules into the `src/` of an overlay workspace:
   ```bash
   mkdir -p ~/landing/ros2/src && cd ~/landing/ros2/src
   git clone --recursive -b aligator-ros2 git@github.com:mattnoh/onlinereplanning.git CoManDO_planner_ALIGATOR
   ```
2. Build Aligator against the system Eigen 3.4 / Boost (fmt >= 10 and mimalloc must be in
   `$PREFIX`; Ubuntu 22.04 does not ship them):
   ```bash
   PREFIX=~/opt/aligator-sys CoManDO_planner_ALIGATOR/support/aligator/build_aligator.sh
   ```
   CMake finds it through `COMANDO_ALIGATOR_PREFIX` (default `~/opt/aligator-sys`).
3. Build the planner on top of the crazyswarm2 workspace:
   ```bash
   cd ~/landing/ros2
   source /opt/ros/humble/setup.bash
   source ~/projects/cf/online/ros_ws/install/setup.bash    # the crazyswarm2 workspace
   colcon build --packages-select comando_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## CrazySim SITL
One command runs a flight without the Gazebo window: Gazebo server (`gz sim -s`), Crazyflie
model and `cf2` firmware, crazyflie server (`backend:=cflib`), target publisher, planner
(`STC_PRESET=sitl`); then arm, take off, go to staging (2, 2, 1.8), trigger
`stc_landing_noaug` (mode mpc, n_replay 4) and record for 25 s.
```bash
export CRAZYFLIE_WS=~/projects/cf/online/ros_ws                          # crazyswarm2 workspace
export CRAZYSIM_FIRMWARE=~/projects/cf/online/CrazySim/crazyflie-firmware
python3 src/CoManDO_planner_ALIGATOR/support/transport/ros2/scripts/run_crazysim_validation.py \
  --motion circle --out ~/landing_runs/cf_circle          # or straight, figure8
```
The planner and target binaries default to this workspace's `install/`. `--out` collects
`planner.log`, `states.csv` (odometry, target, commands), the planner's `solver_events.csv`
and `cell.json` (final height). Solver settings can be overridden per run with `STC_TUNE_*`
environment variables (e.g. `STC_TUNE_MAX_ITERS=300`).

Manual bring-up: CrazySim's `sitl_singleagent.sh -m crazyflie`, `ros2 launch crazyflie
launch.py backend:=cflib gui:=False mocap:=False`, `STC_PRESET=sitl ros2 launch comando_planner
planner_launch.py drone_name:=cf_1 platform:=crazyflie record_bag:=false`,
`ros2 launch comando_planner target_launch.py target_mode:=gazebo_circle`, staging with
`/cf_1/arm`, `/cf_1/takeoff`, `/cf_1/go_to`, then `ros2 launch comando_planner ocp_launch.py
ocp_type:=stc_landing_noaug mode:=mpc n_replay:=4 command_seq:=1`.

## Tests
`colcon build --packages-select comando_planner --cmake-args -DCOMANDO_BUILD_TESTS=ON`, then
`test_aligator_ocp` (finite-difference checks of dynamics, costs and every constraint row)
and `test_wall_clock_rh` (perfect-plant receding horizon).

## Citing
Aligator / ProxDDP: Jallet et al., IEEE T-RO 2025. CTCS/STC: see CTCSTCDDP's `CITATIONS.bib`.
