// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Proposed native oracle SOURCE, not a captured native result.
// Usage: compound-com <pinned dartsim plugin> <new output directory>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<
    gz::physics::ForwardStep, gz::physics::SetFreeGroupWorldVelocity,
    gz::physics::GetModelFromWorld, gz::physics::GetLinkFromModel,
    gz::physics::LinkFrameSemantics, gz::physics::sdf::ConstructSdfWorld>;

static bool maps(const std::filesystem::path &path) {
  std::ifstream source("/proc/self/maps");
  std::ofstream saved(path);
  saved << source.rdbuf();
  return source && saved;
}

static void vector(const Eigen::Vector3d &value) {
  std::cout << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

static std::string collision(int index, const std::string &pose,
                             const std::string &geometry) {
  return "<collision name='c" + std::to_string(index) + "'><pose>" + pose +
      "</pose><geometry>" + geometry + "</geometry></collision>";
}

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const std::filesystem::path output(argv[2]);
  if (!std::filesystem::create_directory(output)) return 3;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  if (!engine || !maps(output / "before.maps")) return 4;
  const std::string sphere = "<sphere><radius>0.2</radius></sphere>";
  const std::string box = "<box><size>0.6 0.4 0.8</size></box>";
  const std::string cylinder =
      "<cylinder><radius>0.2</radius><length>1</length></cylinder>";
  const std::string capsule =
      "<capsule><radius>0.2</radius><length>0.8</length></capsule>";
  const std::vector<std::string> shapes = {
      collision(0, "0 0 0 0 0 0", sphere),
      collision(0, "-0.4 0 0 0 0 0", box) +
          collision(1, "0.4 0 0 0 1.5708 0.3", cylinder),
      collision(0, "-0.4 0 0 0 0 0", box) +
          collision(1, "0 0 0 0 1.5708 0.3", cylinder) +
          collision(2, "0.4 0 0 0.2 0.4 -0.1", capsule)};
  std::cout << std::setprecision(17);
  const std::vector<std::string> centers = {"0.03 -0.02 0.05", "0.1 -0.05 0.2", "0 0 0.5"};
  for (int count = 1; count <= 3; ++count) {
    for (int contact = 0; contact <= 1; ++contact) {
      const int scenario = (count - 1) * 2 + contact;
      const std::string floor = contact ?
          "<model name='floor'><static>true</static><link name='body'>"
          "<collision name='plane'><geometry><plane><normal>0 0 1</normal>"
          "<size>100 100</size></plane></geometry></collision></link></model>" : "";
      const std::string xml =
          "<sdf version='1.11'><world name='w'><gravity>0 0 " +
          std::string(contact ? "-9.81" : "0") + "</gravity>" + floor +
          "<model name='compound'><link name='body'><pose>0 0 " +
          std::string(contact ? "1" : "2") + " 0.2 0.1 -0.3</pose>"
          "<inertial><pose>" + centers[count - 1] + " 0.1 0.2 0.3</pose><mass>2</mass><inertia>"
          "<ixx>1</ixx><iyy>1.5</iyy><izz>2</izz></inertia></inertial>" +
          shapes[count - 1] + "</link></model></world></sdf>";
      std::ofstream fixture(output / (std::to_string(scenario) + ".sdf"));
      fixture << xml;
      fixture.close();
      if (!fixture) return 5;
      sdf::Root root;
      if (!root.LoadSdfString(xml).empty() || !root.WorldByIndex(0)) return 6;
      auto world = engine->ConstructWorld(*root.WorldByIndex(0));
      if (!world) return 7;
      auto model = world->GetModel("compound");
      if (!model) return 8;
      auto body = model->GetLink("body");
      if (!body) return 9;
      const auto *sdfModel = root.WorldByIndex(0)->ModelByName("compound");
      const auto *sdfLink = sdfModel ? sdfModel->LinkByName("body") : nullptr;
      if (!sdfLink) return 12;
      const auto center = sdfLink->Inertial().Pose().Pos();
      const Eigen::Vector3d localCOM(center.X(), center.Y(), center.Z());
      // Native FrameData linearVelocity is the link-frame origin velocity.
      // The adapter's BodyState instead returns the COM linear velocity.
      const auto initialFrame = body->FrameDataRelativeToWorld();
      const double initialCOMZ = (initialFrame.pose * localCOM).z();
      if (!contact) {
        auto group = body->FindFreeGroup();
        if (!group) return 10;
        group->SetWorldAngularVelocity(Eigen::Vector3d(0.1, 0.2, -0.3));
        group->SetWorldLinearVelocity(Eigen::Vector3d(0.2, -0.1, 0.3));
        const auto commanded = body->FrameDataRelativeToWorld();
        if (!commanded.linearVelocity.isApprox(Eigen::Vector3d(0.2, -0.1, 0.3), 1e-12) ||
            !commanded.angularVelocity.isApprox(Eigen::Vector3d(0.1, 0.2, -0.3), 1e-12)) return 13;
      }
      gz::physics::ForwardStep::Input input;
      gz::physics::ForwardStep::State state;
      gz::physics::ForwardStep::Output result;
      input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
      for (int tick = 0; tick <= 600; ++tick) {
        if (tick) world->Step(result, state, input);
        if (tick != 0 && tick != 1 && tick != 10 && tick != 100 && tick != 600) continue;
        auto value = body->FrameDataRelativeToWorld();
        const Eigen::Quaterniond q(value.pose.linear());
        const Eigen::Vector3d worldOffset = value.pose.linear() * localCOM;
        const Eigen::Vector3d comPosition = value.pose * localCOM;
        const Eigen::Vector3d comVelocity = value.linearVelocity + value.angularVelocity.cross(worldOffset);
        if (!value.pose.matrix().allFinite() || !value.linearVelocity.allFinite() ||
            !value.angularVelocity.allFinite() || !localCOM.allFinite() ||
            !comPosition.allFinite() || !comVelocity.allFinite() ||
            std::abs(q.squaredNorm() - 1.0) > 1e-12) return 14;
        if (contact && tick == 600) {
          const double ballisticZ = initialCOMZ - 9.81 * 0.001 * 0.001 * 600 * 601 / 2;
          if (!(comPosition.z() > ballisticZ + 0.01 &&
                comVelocity.z() > -9.81 * 0.6 + 0.01)) return 15;
        }
        std::cout << "{\"scenario\":" << scenario << ",\"count\":" << count
                  << ",\"contact\":" << contact << ",\"tick\":" << tick
                  << ",\"position\":";
        vector(value.pose.translation());
        std::cout << ",\"orientation\":[" << q.x() << ',' << q.y() << ','
                  << q.z() << ',' << q.w() << "],\"velocity\":";
        vector(comVelocity);
        std::cout << ",\"origin_velocity\":";
        vector(value.linearVelocity);
        std::cout << ",\"com_position\":";
        vector(comPosition);
        std::cout << ",\"local_com\":";
        vector(localCOM);
        std::cout << ",\"angular_velocity\":";
        vector(value.angularVelocity);
        std::cout << "}\n";
      }
    }
  }
  return maps(output / "after.maps") ? 0 : 11;
}
