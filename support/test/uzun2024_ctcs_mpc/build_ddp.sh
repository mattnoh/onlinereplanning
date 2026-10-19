#!/usr/bin/env bash
# Build one target sequentially through the planner workspace, with normal linkage.
set -euo pipefail
workspace=${MAVROS_WORKSPACE:-/home/lab/mavros_ws}
if [[ -z ${ROS_DISTRO:-} && -f /opt/ros/noetic/setup.bash ]]; then
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
fi
cd "$workspace"
catkin build comando_planner --no-deps -j1 -p1 \
    --cmake-args -DBUILD_UZUN_PAPER_MPC=ON -DCMAKE_EXE_LINKER_FLAGS:STRING= \
    --make-args uzun2024_ctcs_mpc
