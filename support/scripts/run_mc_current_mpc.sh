#!/usr/bin/env bash
# MC1/MC2 rerun with the current folded cSTC-DDP measured-state MPC.
#
# The startup preset is sourced before every planner start (the runner spawns
# a fresh planner per cell and inherits this environment). Staging and LOS
# trigger altitudes are part of the stc_landing_mpc "mc" preset.
# Trial definitions follow the original campaigns:
#   MC1 (2026-08-25, runner 543424a): six motions x 50, zero noise, seeded shape
#       randomization, heading-relative staging, record-and-proceed bearing.
#   MC2 (2026-08-20, runner 025351a): circle x {zero,nominal,high} x 20, legacy
#       fixed circle (R=1, omega=0.4), staging at target+(1.4,1.4).
# Usage: run_mc_current_mpc.sh {commission|commission_mc2|tune|tune_mc2|mc1|mc2}
# The controller is the circle-verified revision (planner e2f3fdd + solver
# 2789d974, plus infrastructure-only commit 91ac726) built in the overlay
# workspace /home/lab/mavros_ws_mc; the runner itself comes from this repository.
set -eo pipefail
P=/home/lab/mavros_ws/src/CoManDO_planner
WS=${MC_WS:-/home/lab/mavros_ws_mc}
OUT=$P/logs/landing/ros1
export COMANDO_ROS_SETUP="source /opt/ros/noetic/setup.bash && source $WS/devel/setup.bash"
export COMANDO_RUN_PACKAGE=$WS/src/CoManDO_planner
export COMANDO_SOLVER_DIR=$WS/src/ALIPDDP-main
export COMANDO_DEVEL_LIB=$WS/devel/.private/comando_planner/lib/comando_planner
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
source $P/config/ros1/tuning_stc_mpc_sitl.env
# Optional candidate overrides layered on the preset (tuning only).
[ -n "$MC_TUNE_ENV" ] && source "$MC_TUNE_ENV"
RUN=(python3 -u $P/support/scripts/run_scenario_campaign.py
     --profile $P/config/ros1/profile_stc_landing_mpc_sitl.yaml
     --hover-thrust .7282821834
     --start original --landing-timeout 300 --continue-on-failure --reuse)
case "$1" in
  commission)  # outside both denominators
    "${RUN[@]}" --motions circle --levels zero --trials 1 --staging bearing \
      --bearing-policy original --shape-random 1 --out $OUT/mc_commissioning_20261021/mc1_circle_e2f3fdd ;;
  commission_mc2)  # outside both denominators; checks fixed shape and truth topic
    "${RUN[@]}" --motions circle --levels high --trials 1 --staging offset \
      --shape-random 0 --out $OUT/mc_commissioning_20261021/mc2_high_e2f3fdd ;;
  tune)  # held-out seeds (trial >= 50, disjoint from MC1 trials 0-49)
    : "${MC_TUNE_NAME:?set MC_TUNE_NAME}"
    "${RUN[@]}" --motions "${MC_TUNE_MOTIONS:-straight,figure8,stop_and_go,piecewise_accel,random_accel,circle}" \
      --levels zero --trials "${MC_TUNE_TRIALS:-1}" --trial-offset "${MC_TUNE_OFFSET:-50}" \
      --staging bearing --bearing-policy original --shape-random 1 \
      --out $OUT/mc_tuning_20261021/$MC_TUNE_NAME ;;
  tune_mc2)  # held-out trials >= 20 (MC2 uses 0-19)
    : "${MC_TUNE_NAME:?set MC_TUNE_NAME}"
    "${RUN[@]}" --motions circle --levels "${MC_TUNE_LEVELS:-nominal,high}" \
      --trials "${MC_TUNE_TRIALS:-1}" --trial-offset "${MC_TUNE_OFFSET:-20}" \
      --staging offset --shape-random 0 --out $OUT/mc_tuning_20261021/${MC_TUNE_NAME}_mc2 ;;
  mc1)
    "${RUN[@]}" --motions circle,figure8,straight,stop_and_go,piecewise_accel,random_accel \
      --levels zero --trials 50 --staging bearing --bearing-policy original \
      --shape-random 1 --out $OUT/mc1_current_mpc_20261021 ;;
  mc2)
    "${RUN[@]}" --motions circle --levels zero,nominal,high --trials 20 --staging offset \
      --shape-random 0 --out $OUT/mc2_current_mpc_20261021 ;;
  *) echo "usage: $0 {commission|commission_mc2|tune|tune_mc2|mc1|mc2}" >&2; exit 2 ;;
esac
