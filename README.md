# CoManDO planner — Aligator, ROS 1 (MAVROS / PX4)

Receding-horizon landing on a moving platform with continuous-time and state-triggered
constraints (CT-cSTC). The OCP is solved with Aligator's ProxDDP (Jallet et al., T-RO 2025);
the constraints come from [CTCSTCDDP](https://github.com/mattnoh/CTCSTCDDP), the CTCS/STC
interface for Aligator. This branch (`aligator-ros1`) drives a PX4 quadrotor through MAVROS.
The Crazyflie / ROS 2 version is branch `aligator-ros2`.

**Status:** builds and runs the full SITL stack. The landing does not complete yet: online
solves reach the iteration cap infeasible and are rejected, so the vehicle holds at staging.

## Layout
```
include/ocp/                    OCPs (stc_landing_noaug, hover) on Aligator
include/planner_core/, src/     ROS-free core: OCP registry, solve/acceptance, replay
support/transport/ros1/         ROS 1 node, MAVROS platform adapter, launch files
config/ros1/                    hover and landing profiles
support/scripts/                PX4 SITL campaign runner
support/test/                   OCP derivative checks, perfect-plant receding-horizon test
third_party/ctcs_stc            CTCSTCDDP (git submodule); carries Aligator as its own submodule
```

## Setup
Ubuntu 22.04, ROS Noetic, a catkin workspace that already builds `mavros`/`mavlink`
(this package is built as an overlay on it), PX4-Autopilot with the `gz_x500` SITL target.

1. Clone with submodules into the `src/` of an overlay workspace:
   ```bash
   mkdir -p ~/landing/ros1/src && cd ~/landing/ros1/src
   git clone --recursive -b aligator-ros1 git@github.com:mattnoh/onlinereplanning.git CoManDO_planner_ALIGATOR
   ```
2. Build Aligator against the system Eigen 3.4 / Boost that ROS links (fmt >= 10 and
   mimalloc must be in `$PREFIX`; Ubuntu 22.04 does not ship them):
   ```bash
   PREFIX=~/opt/aligator-sys CoManDO_planner_ALIGATOR/support/aligator/build_aligator.sh
   ```
   CMake finds it through `COMANDO_ALIGATOR_PREFIX` (default `~/opt/aligator-sys`).
3. Build the planner, extending the workspace that has MAVROS:
   ```bash
   cd ~/landing/ros1
   source /opt/ros/noetic/setup.bash
   catkin config --extend ~/mavros_ws/devel
   catkin build comando_planner -j2
   source devel/setup.bash
   ```

## PX4 SITL
The campaign runner brings up PX4 (`HEADLESS=1`, Gazebo server only) → MAVROS →
target publisher → planner, flies the hover profile, applies the landing profile and
tears everything down. It uses the workspace the package is checked out in.
```bash
export COMANDO_PX4_DIR=~/projects/px4/PX4-Autopilot   # built px4_sitl_default
python3 src/CoManDO_planner_ALIGATOR/support/scripts/run_scenario_campaign.py \
  --trials 1 --motions circle --levels zero --out ~/landing_runs/px4_circle
```
Keep `--out` and `ROS_LOG_DIR` outside the repository. `--motions`, `--levels` and `--trials`
select the campaign cells; `--reuse` resumes one.

Manual bring-up: `roslaunch comando_planner planner_launch.launch drone_name:=px4_drone
platform:=mavros`, then load a profile from `config/ros1/` and bump
`/comando_planner/command_seq`.

## Tests
`catkin build comando_planner --cmake-args -DCOMANDO_BUILD_TESTS=ON`, then
`test_aligator_ocp` (finite-difference checks of dynamics, costs and every constraint row)
and `test_wall_clock_rh` (perfect-plant receding horizon).

## Citing
Aligator / ProxDDP: Jallet et al., IEEE T-RO 2025. CTCS/STC: see CTCSTCDDP's `CITATIONS.bib`.
