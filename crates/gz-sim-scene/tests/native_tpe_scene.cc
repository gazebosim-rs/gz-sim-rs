// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include "World.hh"
#include "Model.hh"
#include "Link.hh"

int main()
{
  using namespace gz;
  using namespace physics::tpelib;
  std::cout << std::setprecision(17) << '[';
  for (int moving = 0; moving < 2; ++moving)
  {
    World world;
    world.SetTimeStep(0.001);
    auto &model = static_cast<Model &>(world.AddModel());
    model.SetPose(math::Pose3d(0, 0, 5, 0, 0, 0.5));
    auto &link = static_cast<Link &>(model.AddLink());
    link.SetPose(math::Pose3d(0.5, 0, 0, 0, 0, 0.7));
    model.SetLinearVelocity(math::Vector3d(moving, 0, 0));
    model.SetAngularVelocity(math::Vector3d(0, 0, 2 * moving));
    for (int step = 0; step < 100; ++step) world.Step();
    if (moving) std::cout << ',';
    const auto pose = link.GetWorldPose();
    const auto &p = pose.Pos();
    const auto &q = pose.Rot();
    std::cout << "{\"moving\":" << moving << ",\"position\":["
      << p.X() << ',' << p.Y() << ',' << p.Z() << "],\"orientation\":["
      << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W() << "]}";
  }
  std::cout << "]\n";
}
