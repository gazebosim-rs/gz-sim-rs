// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//
// Checks whether the installed gz-physics dartsim plugin, with the packaged
// DART, survives a light 0.2 m cube resting on a static floor for 20 steps.
// Usage: upstream_light_cube <libgz-physics-dartsim-plugin.so> <mass in kg>
// prints "survived"; an aborting DART assertion exits with status 134.
// Build and run commands are in docs/DART.md.
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<gz::physics::sdf::ConstructSdfWorld, gz::physics::ForwardStep>;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <dartsim plugin> <mass>\n";
    return 2;
  }
  const std::string mass = argv[2];
  const double m = std::stod(mass);
  char text[64];
  std::snprintf(text, sizeof(text), "%.17g", m * 0.04 / 6.0);
  const std::string moment = text;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(loader.Instantiate("gz::physics::dartsim::Plugin"));
  const std::string world = R"(<?xml version="1.0"?><sdf version="1.11"><world name="w"><gravity>0 0 -9.81</gravity>
  <model name="ground"><static>true</static><link name="link"><pose>0 0 -0.5 0 0 0</pose>
    <collision name="c"><geometry><box><size>20 20 1</size></box></geometry></collision></link></model>
  <model name="cube"><link name="link"><pose>0 0 0.099 0 0 0</pose><inertial><mass>)" + mass +
      R"(</mass><inertia><ixx>)" + moment + "</ixx><iyy>" + moment + "</iyy><izz>" + moment +
      R"(</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model>
  </world></sdf>)";
  sdf::Root root;
  if (!root.LoadSdfString(world).empty()) { std::cerr << "bad sdf\n"; return 2; }
  auto w = engine->ConstructWorld(*root.WorldByIndex(0));
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
  for (int i = 0; i < 20; ++i) w->Step(output, state, input);
  std::cout << "survived\n";
  return 0;
}
