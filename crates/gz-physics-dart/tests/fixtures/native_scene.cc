// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//
// Runs a scene file (tests/fixtures/divergent_scene_*.txt: one body per line,
// shape, DART dimensions, mass, principal moments, position, quaternion xyzw,
// COM velocity, angular velocity) through the installed gz-physics dartsim
// plugin on a 100 m floor at 1 ms, printing every 100th step's largest body spin
// and first body position like tests/divergence.rs compares.
// Usage: native_scene <libgz-physics-dartsim-plugin.so> <scene file>
// Build and run commands are in docs/DART.md.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<
    gz::physics::sdf::ConstructSdfWorld, gz::physics::ForwardStep,
    gz::physics::GetModelFromWorld, gz::physics::GetLinkFromModel,
    gz::physics::LinkFrameSemantics, gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldVelocity>;

std::string num(double v) {
  char t[40];
  std::snprintf(t, sizeof(t), "%.17g", v);
  return t;
}

struct Body {
  std::string shape;
  double d[3], m, I[3], p[3], q[4], v[3], w[3];
};

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <dartsim plugin> <scene file>\n";
    return 2;
  }
  std::ifstream in(argv[2]);
  std::vector<Body> bodies;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream s(line);
    Body b;
    s >> b.shape >> b.d[0] >> b.d[1] >> b.d[2] >> b.m >> b.I[0] >> b.I[1] >> b.I[2] >> b.p[0] >> b.p[1] >> b.p[2] >> b.q[0] >> b.q[1] >> b.q[2] >> b.q[3] >> b.v[0] >> b.v[1] >> b.v[2] >> b.w[0] >> b.w[1] >> b.w[2];
    bodies.push_back(b);
  }
  std::string sdf = R"(<?xml version="1.0"?><sdf version="1.11"><world name="w"><gravity>0 0 -9.81</gravity>
  <model name="ground"><static>true</static><link name="link"><pose>0 0 -0.5 0 0 0</pose>
    <collision name="c"><geometry><box><size>100 100 1</size></box></geometry></collision></link></model>)";
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    const Body &b = bodies[i];
    std::string geometry;
    if (b.shape == "sphere") geometry = "<sphere><radius>" + num(b.d[0]) + "</radius></sphere>";
    if (b.shape == "box") geometry = "<box><size>" + num(b.d[0]) + " " + num(b.d[1]) + " " + num(b.d[2]) + "</size></box>";
    if (b.shape == "cylinder") geometry = "<cylinder><radius>" + num(b.d[0]) + "</radius><length>" + num(b.d[1]) + "</length></cylinder>";
    if (b.shape == "capsule") geometry = "<capsule><radius>" + num(b.d[0]) + "</radius><length>" + num(b.d[1]) + "</length></capsule>";
    sdf += "<model name=\"b" + std::to_string(i) + "\"><link name=\"link\"><pose rotation_format=\"quat_xyzw\">" +
           num(b.p[0]) + " " + num(b.p[1]) + " " + num(b.p[2]) + " " + num(b.q[0]) + " " + num(b.q[1]) + " " + num(b.q[2]) + " " + num(b.q[3]) +
           "</pose><inertial><mass>" + num(b.m) + "</mass><inertia><ixx>" + num(b.I[0]) + "</ixx><iyy>" + num(b.I[1]) + "</iyy><izz>" + num(b.I[2]) +
           "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial><collision name=\"c\"><geometry>" + geometry +
           "</geometry></collision></link></model>";
  }
  sdf += "</world></sdf>";
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(loader.Instantiate("gz::physics::dartsim::Plugin"));
  sdf::Root root;
  const auto errors = root.LoadSdfString(sdf);
  if (!errors.empty()) { std::cerr << errors[0].Message() << "\n"; return 2; }
  auto world = engine->ConstructWorld(*root.WorldByIndex(0));
  std::vector<gz::physics::Link3dPtr<Features>> links;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    auto model = world->GetModel("b" + std::to_string(i));
    auto group = model->FindFreeGroup();
    group->SetWorldAngularVelocity(Eigen::Vector3d(bodies[i].w[0], bodies[i].w[1], bodies[i].w[2]));
    group->SetWorldLinearVelocity(Eigen::Vector3d(bodies[i].v[0], bodies[i].v[1], bodies[i].v[2]));
    links.push_back(model->GetLink("link"));
  }
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
  for (int step = 1; step <= 3000; ++step) {
    world->Step(output, state, input);
    if (step % 100 == 0) {
      double mw = 0;
      for (auto &l : links) mw = std::max(mw, l->FrameDataRelativeToWorld().angularVelocity.norm());
      const auto p = links[0]->FrameDataRelativeToWorld().pose.translation();
      std::printf("step %d max|w| %.6e body0 %.9f %.9f %.9f\n", step, mw, p.x(), p.y(), p.z());
      if (!std::isfinite(mw) || mw > 1e60) break;
    }
  }
  return 0;
}
