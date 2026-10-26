// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include "Link.hh"
#include "Model.hh"
#include "World.hh"

int main()
{
  using namespace gz;
  using namespace physics::tpelib;
  std::cout << std::setprecision(17) << std::boolalpha << '[';
  bool first = true;
  for (bool fixed : {false, true})
  {
    World world;
    world.SetTimeStep(0.01);
    auto &model = static_cast<Model &>(world.AddModel());
    model.SetPose(math::Pose3d(0, 0, 5, 0, 0, 0));
    model.AddLink();
    model.SetStatic(fixed);
    model.SetLinearVelocity(math::Vector3d(1, 0, 0));
    model.SetAngularVelocity(math::Vector3d(0, 0, 1));
    for (int step = 1; step <= 3; ++step)
    {
      world.Step();
      const auto pose = model.GetWorldPose();
      const auto velocity = model.GetLinearVelocity();
      const auto angular = model.GetAngularVelocity();
      if (!first) std::cout << ',';
      first = false;
      std::cout << "{\"static\":" << model.GetStatic()
        << ",\"iteration\":" << step
        << ",\"position\":[" << pose.X() << ',' << pose.Y() << ',' << pose.Z()
        << "],\"orientation\":[" << pose.Rot().X() << ',' << pose.Rot().Y()
        << ',' << pose.Rot().Z() << ',' << pose.Rot().W()
        << "],\"velocity\":[" << velocity.X() << ',' << velocity.Y() << ','
        << velocity.Z() << "],\"angular_velocity\":[" << angular.X() << ','
        << angular.Y() << ',' << angular.Z() << "]}";
    }
  }
  std::cout << "]\n";
}
