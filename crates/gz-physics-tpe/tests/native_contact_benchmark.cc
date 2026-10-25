// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <unordered_map>
#include <vector>
#include "World.hh"
#include "Model.hh"
#include "Link.hh"
#include "Collision.hh"
#include "Shape.hh"

int main()
{
  using namespace gz;
  using namespace physics::tpelib;
  World world;
  world.SetTimeStep(0.001);
  std::vector<Model *> models;
  std::unordered_map<std::size_t, int> ordinals;
  for (int i = 0; i < 1000; ++i)
  {
    const bool fixed = (i / 2) % 2 == 1;
    auto &model = static_cast<Model &>(world.AddModel());
    model.SetPose(math::Pose3d((i / 2) * 3.0 + (i % 2) * 0.75, 0, 0, 0, 0, 0));
    model.SetStatic(fixed);
    model.SetLinearVelocity(math::Vector3d(fixed ? 0 : 0.01, 0, 0));
    auto &link = static_cast<Link &>(model.AddLink());
    auto &collision = static_cast<Collision &>(link.AddCollision());
    BoxShape shape;
    shape.SetSize(math::Vector3d(1, 1, 1));
    collision.SetShape(shape);
    models.push_back(&model);
    ordinals[model.GetId()] = i;
  }
  // Prime geometry caches, spatial index and output without advancing motion.
  world.SetTimeStep(0);
  world.Step();
  world.Step();
  if (world.GetContacts().size() != 250) return 7;
  world.SetTimeStep(0.001);
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 500; ++i) world.Step();
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count();
  if (std::abs(world.GetTime() - 0.5) >= 1e-12) return 8;
  double positionChecksum = 0;
  for (int i = 0; i < 1000; ++i)
  {
    const auto pose = models[i]->GetPose();
    const double expected = (i / 2) * 3.0 + (i % 2) * 0.75 +
      ((i / 2) % 2 == 0 ? 0.005 : 0);
    if (std::abs(pose.X() - expected) >= 1e-9 || pose.Y() != 0 || pose.Z() != 0 ||
        pose.Rot() != math::Quaterniond::Identity) return 2;
    positionChecksum += pose.X();
  }
  const auto contacts = world.GetContacts();
  if (contacts.size() != 250) return 3;
  double contactChecksum = 0;
  std::set<int> seen;
  for (const auto &contact : contacts)
  {
    const int a = ordinals.at(contact.entity1);
    const int b = ordinals.at(contact.entity2);
    if (a == b || a / 2 != b / 2 || (a / 2) % 2 != 0 ||
        std::abs(contact.point.X() - ((a / 2) * 3.0 + 0.375 + 0.005)) >= 1e-9 ||
        contact.point.Y() != 0 || contact.point.Z() != 0) return 4;
    if (!seen.insert(a / 2).second) return 5;
    contactChecksum += contact.point.X();
  }
  for (int i = 0; i < 500; i += 2) if (seen.count(i) != 1) return 6;
  std::cout << std::setprecision(17) << "{\"nanos\":" << nanos
    << ",\"models\":1000,\"steps\":500,\"contacts\":250,\"position_checksum\":"
    << positionChecksum << ",\"contact_checksum\":" << contactChecksum << "}\n";
}
