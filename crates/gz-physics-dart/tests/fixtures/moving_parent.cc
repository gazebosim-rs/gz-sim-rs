// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Usage: moving-parent <pinned dartsim plugin> <loaded-maps output>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<gz::physics::ForwardStep,
    gz::physics::GetBasicJointState, gz::physics::SetBasicJointState,
    gz::physics::GetJointFromModel, gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel, gz::physics::LinkFrameSemantics,
    gz::physics::sdf::ConstructSdfWorld>;

static void vector(const Eigen::Vector3d &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
static std::string link(const std::string &name, const std::string &pose, bool tilted) {
  return "<link name='" + name + "'><pose>" + pose + "</pose><inertial><pose>" +
    (tilted ? "0.1 0.05 0.02 0 0 0" : "0 0 0 0 0 0") +
    "</pose><mass>1</mass><inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz>"
    "</inertia></inertial><collision name='c'><geometry><sphere><radius>0.1</radius>"
    "</sphere></geometry></collision></link>";
}
static std::string joint(const std::string &name, const std::string &parent,
                          const std::string &child, bool tilted) {
  return "<joint name='" + name + "' type='revolute'><parent>" + parent +
    "</parent><child>" + child + "</child><pose>" +
    (tilted ? "0.1 -0.2 0.8 0 0 0" : "0 0 1 0 0 0") +
    "</pose><axis><xyz>" + (tilted ? "0.3 0.8 0.5" : "0 1 0") +
    "</xyz></axis></joint>";
}

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  if (!engine) return 3;
  std::cout << std::setprecision(17);
  for (bool tilted : {false, true}) {
    sdf::Root root;
    const std::string text = "<sdf version='1.11'><world name='w'><gravity>0 0 -9.81</gravity>"
      "<model name='m'>" + link("parent", tilted ? "0.3 -0.2 -0.9 0.3 -0.2 0.5" : "0 0 -1 0 0 0", tilted) +
      link("child", tilted ? "0.4 0.1 -2.5 -0.2 0.4 -0.3" : "0 0 -3 0 0 0", tilted) +
      joint("root", "world", "parent", tilted) + joint("leaf", "parent", "child", tilted) +
      "</model></world></sdf>";
    if (!root.LoadSdfString(text).empty()) return 4;
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    if (!world) return 5;
    auto model = world->GetModel("m");
    auto upper = model->GetJoint("root");
    auto lower = model->GetJoint("leaf");
    upper->SetVelocity(0, tilted ? 0.7 : 1.0);
    lower->SetVelocity(0, tilted ? -1.1 : 2.0);
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
    for (int tick = 0; tick <= 1000; ++tick) {
      const double effort = tick == 1 ? 1.5 : tick == 3 ? -3.0 : tick == 5 ? 2.0 : 0.0;
      if (tick > 0) {
        lower->SetForce(0, effort);
        world->Step(output, state, input);
      }
      if (tick > 6 && tick % 100 != 0) continue;
      std::cout << "{\"tilted\":" << (tilted ? "true" : "false") << ",\"tick\":" << tick
                << ",\"effort\":" << effort << ",\"bodies\":[";
      int index = 0;
      for (const char *name : {"parent", "child"}) {
        auto body = model->GetLink(name);
        auto hinge = model->GetJoint(index == 0 ? "root" : "leaf");
        const auto *definition = root.WorldByIndex(0)->ModelByIndex(0)->LinkByName(name);
        const auto *axis = root.WorldByIndex(0)->ModelByIndex(0)->JointByName(index == 0 ? "root" : "leaf");
        const auto data = body->FrameDataRelativeToWorld();
        const Eigen::Quaterniond q(data.pose.linear());
        const Eigen::Vector3d center = gz::math::eigen3::convert(definition->Inertial().Pose().Pos());
        std::cout << (index++ ? "," : "") << "{\"position\":";
        vector(data.pose.translation());
        std::cout << ",\"orientation\":[" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
        std::cout << ",\"velocity\":";
        vector(data.linearVelocity + data.angularVelocity.cross(data.pose.linear() * center));
        std::cout << ",\"angular_velocity\":";
        vector(data.angularVelocity);
        std::cout << ",\"q\":" << hinge->GetPosition(0) << ",\"dq\":" << hinge->GetVelocity(0);
        if (tick == 0) {
          std::cout << ",\"center\":";
          vector(center);
          std::cout << ",\"anchor\":";
          vector(data.pose * gz::math::eigen3::convert(axis->RawPose().Pos()));
          std::cout << ",\"axis\":";
          vector(data.pose.linear() * gz::math::eigen3::convert(axis->Axis(0)->Xyz()));
        }
        std::cout << '}';
      }
      std::cout << "]}\n";
    }
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream recorded(argv[2]);
  recorded << maps.rdbuf();
  return recorded ? 0 : 6;
}
