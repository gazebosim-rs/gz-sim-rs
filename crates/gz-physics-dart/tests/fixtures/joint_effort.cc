// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>

using Features = gz::physics::FeatureList<gz::physics::ForwardStep,
    gz::physics::GetBasicJointState, gz::physics::SetBasicJointState,
    gz::physics::GetJointFromModel, gz::physics::GetModelFromWorld,
    gz::physics::sdf::ConstructSdfWorld>;

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  if (!engine) return 3;
  sdf::Root root;
  const auto errors = root.LoadSdfString(
      "<sdf version='1.11'><world name='w'><gravity>0 0 0</gravity>"
      "<model name='m'><link name='l'><inertial><mass>1</mass><inertia>"
      "<ixx>0.016</ixx><iyy>0.016</iyy><izz>0.016</izz></inertia></inertial>"
      "<collision name='c'><geometry><sphere><radius>0.2</radius></sphere>"
      "</geometry></collision></link><joint name='j' type='revolute'>"
      "<parent>world</parent><child>l</child><axis><xyz>0 1 0</xyz></axis>"
      "</joint></model></world></sdf>");
  if (!errors.empty()) return 4;
  auto world = engine->ConstructWorld(*root.WorldByIndex(0));
  if (!world) return 5;
  auto joint = world->GetModel("m")->GetJoint("j");
  if (!joint) return 6;
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
  std::cout << std::setprecision(17);
  for (double effort : {1.5, 0.0, -3.0, 0.0, 2.0, 0.0}) {
    joint->SetForce(0, effort);
    world->Step(output, state, input);
    std::cout << "{\"effort\":" << effort << ",\"q\":" << joint->GetPosition(0)
              << ",\"dq\":" << joint->GetVelocity(0) << "}\n";
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream recorded(argv[2]);
  recorded << maps.rdbuf();
  return recorded ? 0 : 7;
}
