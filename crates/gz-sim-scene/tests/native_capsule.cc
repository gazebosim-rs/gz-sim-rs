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
  CapsuleShape capsule;
  capsule.SetRadius(0.6);
  capsule.SetLength(2.8);
  const auto bounds = capsule.GetBoundingBox();
  std::cout << std::setprecision(17) << "{\"radius\":" << capsule.GetRadius()
      << ",\"length\":" << capsule.GetLength() << ",\"min\":["
      << bounds.Min().X() << ',' << bounds.Min().Y() << ',' << bounds.Min().Z()
      << "],\"max\":[" << bounds.Max().X() << ',' << bounds.Max().Y() << ','
      << bounds.Max().Z() << "],\"rows\":[";
  for (int mode = 0; mode < 8; ++mode)
  {
    World world;
    world.SetTimeStep(0.001);
    auto &a = static_cast<Model &>(world.AddModel());
    auto &b = static_cast<Model &>(world.AddModel());
    auto &la = static_cast<Link &>(a.AddLink());
    auto &lb = static_cast<Link &>(b.AddLink());
    auto &ca = static_cast<Collision &>(la.AddCollision());
    auto &cb = static_cast<Collision &>(lb.AddCollision());
    ca.SetShape(capsule);
    cb.SetShape(capsule);
    b.SetPose(math::Pose3d(mode == 1 ? 1.2 : mode == 2 ? 1.201 : 1.0, 0, 0, 0, 0, 0));
    if (mode == 3) b.SetPose(math::Pose3d(0, 0, 4.0, 0, 0, 0));
    if (mode == 4 || mode == 5)
    {
      b.SetPose(math::Pose3d(0.5, 0.2, 0.1, mode == 5 ? 0.3 : 0,
          mode == 5 ? -0.2 : 0, mode == 5 ? 0.5 : 0));
      lb.SetPose(math::Pose3d(0.25, 0.1, 0.2, 0.4, 0.2, 0.7));
    }
    a.SetStatic(mode == 6 || mode == 7);
    b.SetStatic(mode == 6);
    world.Step();
    if (mode) std::cout << ',';
    std::cout << "{\"mode\":" << mode << ",\"contacts\":[";
    bool first = true;
    for (const auto &contact : world.GetContacts())
    {
      if (!((contact.entity1 == a.GetId() && contact.entity2 == b.GetId()) ||
          (contact.entity2 == a.GetId() && contact.entity1 == b.GetId()))) return 2;
      if (!first) std::cout << ',';
      first = false;
      std::cout << '[' << contact.point.X() << ',' << contact.point.Y() << ',' << contact.point.Z() << ']';
    }
    std::cout << "]}";
  }
  std::cout << "]}\n";
}
