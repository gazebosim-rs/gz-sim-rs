// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include "Model.hh"

using namespace gz;

void Vector(const math::Vector3d &value)
{
  std::cout << '[' << value.X() << ',' << value.Y() << ',' << value.Z() << ']';
}

void Quaternion(const math::Quaterniond &value)
{
  std::cout << '[' << value.X() << ',' << value.Y() << ',' << value.Z() << ',' << value.W() << ']';
}

void Emit(const char *name, uint64_t nanos, const math::Pose3d &pose,
          const math::Vector3d &linear, const math::Vector3d &angular)
{
  physics::tpelib::Model model;
  model.SetPose(pose);
  model.SetLinearVelocity(linear);
  model.SetAngularVelocity(angular);
  model.UpdatePose(static_cast<double>(nanos) / 1e9);
  std::cout << "{\"name\":\"" << name << "\",\"dt_nanos\":" << nanos << ",\"position\":";
  Vector(pose.Pos());
  std::cout << ",\"orientation\":";
  Quaternion(pose.Rot());
  std::cout << ",\"velocity\":";
  Vector(linear);
  std::cout << ",\"angular_velocity\":";
  Vector(angular);
  std::cout << ",\"expected_position\":";
  Vector(model.GetPose().Pos());
  std::cout << ",\"expected_orientation\":";
  Quaternion(model.GetPose().Rot());
  std::cout << '}';
}

int main()
{
  std::cout << std::setprecision(17) << '[';
  // Motion assertion from Model.BasicAPI (2020 OSRF), apart from its API/lifecycle assertions.
  Emit("original_model_motion", 100000000, math::Pose3d::Zero,
       math::Vector3d(.1, .1, .1), math::Vector3d(1, 0, 0));
  for (uint64_t nanos : {0ULL, 1ULL, 1000ULL, 1000000ULL, 10000000ULL, 100000000ULL, 330000000ULL, 1500000000ULL})
  {
    std::cout << ',';
    Emit("rotated_model", nanos, math::Pose3d(1, 2, 3, .1, .2, .3),
         math::Vector3d(.1, -.2, .3), math::Vector3d(.3, -.4, .5));
  }
  std::cout << ',';
  Emit("stationary_threshold", 1000000000, math::Pose3d(1, 2, 3, .1, .2, .3),
       math::Vector3d(1e-3, -1e-3, 0), math::Vector3d(-1e-3, 1e-3, 0));
  std::cout << ',';
  Emit("above_threshold", 1000000000, math::Pose3d::Zero,
       math::Vector3d(std::nextafter(1e-3, 1.0), 0, 0), math::Vector3d::Zero);
  std::cout << ',';
  Emit("tiny_angular_with_translation", 1000000000, math::Pose3d::Zero,
       math::Vector3d(1, 0, 0), math::Vector3d(1e-200, 0, 0));
  std::cout << "]\n";
}
