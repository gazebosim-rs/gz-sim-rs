# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(gz_physics_mujoco_reference CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(gz-physics REQUIRED COMPONENTS sdf)
find_package(gz-plugin REQUIRED COMPONENTS loader)
find_package(sdformat REQUIRED)
add_executable(mujoco_reference mujoco_reference.cc)
target_link_libraries(mujoco_reference
  gz-physics::gz-physics gz-physics::sdf gz-plugin::loader sdformat::sdformat)
