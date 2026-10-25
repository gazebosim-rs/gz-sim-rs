// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include "World.hh"
#include "Model.hh"
#include "Link.hh"
#include "Collision.hh"
#include "Shape.hh"

int main()
{
  using namespace gz;
  using namespace physics::tpelib;
  std::cout << std::setprecision(17) << '[';
  for (int mode = 0; mode < 11; ++mode)
  {
    World world;
    auto &a = static_cast<Model &>(world.AddModel());
    auto &b = static_cast<Model &>(world.AddModel());
    auto &la = static_cast<Link &>(a.AddLink());
    auto &lb = static_cast<Link &>(b.AddLink());
    auto &ca = static_cast<Collision &>(la.AddCollision());
    auto &cb = static_cast<Collision &>(lb.AddCollision());
    BoxShape sa;
    sa.SetSize(math::Vector3d(4, 4, 4));
    BoxShape sb;
    sb.SetSize(math::Vector3d(2, 2, 2));
    ca.SetShape(sa);
    if (mode == 8) {
      SphereShape sphere;
      sphere.SetRadius(1);
      cb.SetShape(sphere);
    } else cb.SetShape(sb);
    b.SetPose(math::Pose3d(mode == 1 ? 3 : mode == 2 ? 3.001 : 1, 0, 0, 0, 0, 0));
    a.SetStatic(mode == 3 || mode == 4);
    b.SetStatic(mode == 3);
    ca.SetCollideBitmask(mode == 5 ? 1 : mode == 6 ? 3 : 0xffff);
    cb.SetCollideBitmask(mode == 5 || mode == 6 ? 2 : 0xffff);
    if (mode == 7 || mode == 8) lb.SetPose(math::Pose3d(0, 0, 0, 0, 0, 0.7));
    if (mode == 9) {
      b.SetPose(math::Pose3d(1, 0, 0, 0, 0, 0.3));
      lb.SetPose(math::Pose3d(0.5, 0, 0, 0, 0, 0.7));
      auto &second = static_cast<Link &>(b.AddLink());
      second.SetPose(math::Pose3d(-3, 1, 0, 0, 0, 0));
      auto &collision = static_cast<Collision &>(second.AddCollision());
      BoxShape small;
      small.SetSize(math::Vector3d(1, 1, 1));
      collision.SetShape(small);
    }
    if (mode == 10) {
      a.SetPose(math::Pose3d(1e308, 0, 0, 0, 0, 0));
      b.SetPose(math::Pose3d(1e308, 0, 0, 0, 0, 0));
    }
    world.Step();
    if (mode) std::cout << ',';
    std::cout << "{\"mode\":" << mode << ",\"contacts\":[";
    bool first = true;
    for (const auto &contact : world.GetContacts())
    {
      if (!first) std::cout << ',';
      first = false;
      if (!((contact.entity1 == a.GetId() && contact.entity2 == b.GetId()) ||
            (contact.entity2 == a.GetId() && contact.entity1 == b.GetId()))) return 2;
      std::cout << '[' << contact.point.X() << ',' << contact.point.Y() << ',' << contact.point.Z() << ']';
    }
    std::cout << "]}";
  }
  std::cout << "]\n";
}
