#!/bin/bash
# Build Aligator (the top-level aligator/ submodule) against the
# system Eigen 3.4 / Boost that the ROS node links, into $PREFIX.
# fmt >= 10 and mimalloc are not on Ubuntu 22.04: copy prebuilt ones into
# $PREFIX first (conda-forge fmt 12.1 / mimalloc 3.2 need only GLIBCXX 3.4.21).
set -e
PREFIX=${PREFIX:-$HOME/opt/aligator-sys}
SRC=${SRC:-$(cd "$(dirname "$0")/../../aligator" && pwd)}  # the solver submodule (Aligator 0.19.1-19, 4cf745f9)
BUILD=${BUILD:-$HOME/opt/build/aligator}
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DCMAKE_IGNORE_PREFIX_PATH="$HOME/miniforge3" \
  -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_PINOCCHIO_SUPPORT=OFF -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF -DINSTALL_DOCUMENTATION=OFF \
  -DBUILD_CROCODDYL_COMPAT=OFF
nice cmake --build "$BUILD" -j2
cmake --install "$BUILD"
