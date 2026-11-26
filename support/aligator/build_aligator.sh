#!/bin/bash
# Build Aligator (the ctcs_stc submodule's third_party/aligator) against the
# system Eigen 3.4 / Boost that the ROS node links, into $PREFIX.
# fmt >= 10 and mimalloc are not on Ubuntu 22.04: copy prebuilt ones into
# $PREFIX first (conda-forge fmt 12.1 / mimalloc 3.2 need only GLIBCXX 3.4.21).
set -e
PREFIX=${PREFIX:-$HOME/opt/aligator-sys}
SRC=${SRC:-$(git -C "$(dirname "$0")/../../third_party/ctcs_stc" rev-parse --show-toplevel)/third_party/aligator}
BUILD=${BUILD:-$HOME/opt/build/aligator}
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DCMAKE_IGNORE_PREFIX_PATH="$HOME/miniforge3" \
  -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_PINOCCHIO_SUPPORT=OFF -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF -DINSTALL_DOCUMENTATION=OFF \
  -DBUILD_CROCODDYL_COMPAT=OFF
nice cmake --build "$BUILD" -j2
cmake --install "$BUILD"
