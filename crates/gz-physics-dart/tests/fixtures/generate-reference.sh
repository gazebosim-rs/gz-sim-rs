#!/bin/sh
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
#
# Rebuilds reference.json from the installed gz-physics dartsim plugin.
# Usage: generate-reference.sh <gz install prefix> <work directory>
set -eu
prefix=$1
work=$2
here=$(cd "$(dirname "$0")" && pwd)
export PKG_CONFIG_PATH="$prefix/lib/pkgconfig"
flags=$(pkg-config --cflags --libs gz-physics gz-physics-sdf gz-plugin-loader sdformat gz-math-eigen3)
# shellcheck disable=SC2086
g++ -std=c++17 -O2 "$here/reference.cc" $flags -Wl,-rpath,"$prefix/lib" -o "$work/reference"
LD_LIBRARY_PATH="$prefix/lib" "$work/reference" "$prefix/lib/libgz-physics-dartsim-plugin.so" \
  > "$here/reference.json"
