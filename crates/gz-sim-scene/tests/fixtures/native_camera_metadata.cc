// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <sdf/Camera.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/SemanticPose.hh>
#include <sdf/Sensor.hh>
#include <sdf/World.hh>

int main(int argc, char **argv) {
  if (argc != 2) throw std::runtime_error("expected fixture path");
  sdf::Root root;
  const auto errors = root.Load(argv[1]);
  if (!errors.empty()) throw std::runtime_error(errors.front().Message());
  const auto *sensor = root.WorldByIndex(0)->ModelByIndex(0)->LinkByIndex(0)->SensorByIndex(0);
  const auto *camera = sensor ? sensor->CameraSensor() : nullptr;
  if (!camera) throw std::runtime_error("camera missing");
  if (camera->ImageWidth() != 64 || camera->ImageHeight() != 64 ||
      camera->AntiAliasingValue() != 4 || camera->HasLensIntrinsics() ||
      camera->HasLensProjection() || std::abs(camera->HorizontalFov().Radian() - M_PI / 2) > 1e-12 ||
      std::abs(sensor->RawPose().Pos().X() - 1.0) > 1e-12)
    throw std::runtime_error("camera metadata mismatch");
  gz::math::Pose3d sensorModelPose;
  const auto poseErrors = sensor->SemanticPose().Resolve(sensorModelPose, "__model__");
  if (!poseErrors.empty()) throw std::runtime_error(poseErrors.front().Message());
  gz::math::Pose3d modelWorldPose;
  const auto modelErrors = root.WorldByIndex(0)->ModelByIndex(0)->SemanticPose().Resolve(modelWorldPose, "world");
  if (!modelErrors.empty()) throw std::runtime_error(modelErrors.front().Message());
  const auto worldPose = modelWorldPose * sensorModelPose;
  if (std::abs(worldPose.Pos().X() - 1000000000.0) > 1e-7 ||
      std::abs(worldPose.Pos().Y() - 3.0) > 1e-12 ||
      std::abs(worldPose.Rot().Yaw() - M_PI / 2) > 1e-12)
    throw std::runtime_error("sensor world pose mismatch");
  std::cout << "64x64 aa=4 absent-K absent-P hfov=pi/2 sensor-x=1 world=(1e9,3,0) yaw=pi/2\n";
}
