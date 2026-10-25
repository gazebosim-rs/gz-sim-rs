// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
#include "Model.hh"
using namespace gz;
int main()
{
  std::vector<std::unique_ptr<physics::tpelib::Model>> models;
  for (int i = 0; i < 1000; ++i)
  {
    auto model = std::make_unique<physics::tpelib::Model>();
    model->SetPose(math::Pose3d(i * 0.001, 2, 3, 0, 0, 0));
    model->SetLinearVelocity(math::Vector3d(1, .5, .25));
    model->SetAngularVelocity(math::Vector3d(.1, .2, .3));
    models.push_back(std::move(model));
  }
  const auto start = std::chrono::steady_clock::now();
  for (int step = 0; step < 1000; ++step)
    for (auto &model : models) model->UpdatePose(.001);
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
  double checksum = 0;
  for (const auto &model : models)
  {
    const auto pose = model->GetPose();
    checksum += pose.Pos().X(); checksum += pose.Pos().Y(); checksum += pose.Pos().Z(); checksum += pose.Rot().W();
  }
  const auto pose = models[0]->GetPose();
  std::cout << std::setprecision(17) << "{\"nanos\":" << nanos << ",\"checksum\":" << checksum
    << ",\"position\":[" << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z()
    << "],\"orientation\":[" << pose.Rot().X() << ',' << pose.Rot().Y() << ',' << pose.Rot().Z() << ',' << pose.Rot().W() << "]}\n";
}
