#!/usr/bin/env bash
# Build the native example against this planner's selected solver checkout.
set -euo pipefail
planner_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
build_root=${COMANDO_BUILD_DIR:-"$planner_root/build"}
solver_root=${COMANDO_SOLVER_DIR:-"$planner_root/../ALIPDDP-main"}
cmake -S "$planner_root" -B "$build_root" \
    -DCMAKE_BUILD_TYPE=Release -DCOMANDO_SOLVER_DIR="$solver_root" \
    -DBUILD_UZUN_PAPER_MPC=ON
cmake --build "$build_root" --target uzun2024_ctcs_mpc -j1
