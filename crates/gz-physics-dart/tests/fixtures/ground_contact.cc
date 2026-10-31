// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Usage: ground-contact <pinned dartsim plugin> <loaded-maps output>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
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
    gz::physics::SetFreeGroupWorldVelocity, gz::physics::GetBasicJointState, gz::physics::SetBasicJointState,
    gz::physics::GetJointFromModel, gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel, gz::physics::LinkFrameSemantics,
    gz::physics::sdf::ConstructSdfWorld>;

static void vector(const Eigen::Vector3d &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  if (!engine) return 3;
  std::cout << std::setprecision(17);
  for (double force : {0.001, -0.001}) {
    sdf::Root root;
    if (!root.Load((std::filesystem::path(__FILE__).parent_path() / "apply_joint_force.sdf").string()).empty()) return 4;
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    if (!world) return 5;
    auto model = world->GetModel("joint_force_test");
    auto lower = model->GetJoint("j1");
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
    for (int tick=0; tick<=800; ++tick) {
      const double effort = tick<=10 ? 0.0 : force;
      if (tick>0) { lower->SetForce(0,effort); world->Step(output,state,input); }
      if (!(tick<=6 || tick==10 || tick==11 || (tick>=100 && tick%100==0))) continue;
      std::cout << "{\"force\":" << force << ",\"tick\":" << tick << ",\"q\":" << lower->GetPosition(0) << ",\"dq\":" << lower->GetVelocity(0) << ",\"bodies\":[";
      int index=0;
      for (const char *name : {"base_link","rotor"}) {
        auto data=model->GetLink(name)->FrameDataRelativeToWorld();
        Eigen::Quaterniond q(data.pose.linear());
        std::cout << (index++ ? "," : "") << "{\"position\":";
        vector(data.pose.translation());
        std::cout << ",\"orientation\":[" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << "]";
        std::cout << ",\"velocity\":"; vector(data.linearVelocity);
        std::cout << ",\"angular_velocity\":"; vector(data.angularVelocity); std::cout << '}';
      }
      std::cout << "]}\n";
    }
  }
  std::ifstream maps("/proc/self/maps"); std::ofstream recorded(argv[2]); recorded << maps.rdbuf();
  return recorded ? 0 : 6;
}
