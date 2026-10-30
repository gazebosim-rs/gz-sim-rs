// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Native reference engine only; the Rust simulator does not load system plugins.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
using Features = gz::physics::FeatureList<gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep, gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel, gz::physics::LinkFrameSemantics>;
struct Case { const char *name; double x, y, z, tilt, size, ballX; };
void vec(const Eigen::Vector3d &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
void quat(const Eigen::Quaterniond &q) {
  std::cout << '[' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
}
template <class Link> void state(const Link &link) {
  const auto value = link->FrameDataRelativeToWorld();
  std::cout << "{\"position\":"; vec(value.pose.translation());
  std::cout << ",\"orientation\":"; quat(Eigen::Quaterniond(value.pose.linear()));
  std::cout << ",\"velocity\":"; vec(value.linearVelocity);
  std::cout << ",\"angular_velocity\":"; vec(value.angularVelocity);
  std::cout << '}';
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::plugin::Loader loader;
  const auto names = loader.LoadLib(argv[1]);
  const std::string name = "gz::physics::dartsim::Plugin";
  if (!names.count(name)) throw std::runtime_error("reference engine absent");
  auto plugin = loader.Instantiate(name);
  bool first = true;
  std::cout << std::setprecision(17) << "[\n";
  for (const Case &c : {Case{"center", 0, 0, 1, 0, 2, 0},
       Case{"small_size", 0, 0, 1, 0, 1, 50},
       Case{"large_size", 0, 0, 1, 0, 100000, 50},
       Case{"outside_cube", 0, 0, 1, 0, 2, 1100},
       Case{"footprint_edge", 0, 0, 1, 0, 2, 1049.75},
       Case{"tilted", 0, 0, 1, 0.2, 2, 0},
       Case{"translated", 2, -3, 1, 0, 2, 2}}) {
    std::ostringstream xml;
    xml << std::setprecision(17)
      << "<sdf version='1.12'><world name='w'><gravity>0 0 -9.81</gravity>"
      << "<model name='ground'><static>true</static><link name='link'><pose>"
      << c.x << ' ' << c.y << ' ' << c.z << " 0 " << c.tilt << " 0</pose>"
      << "<collision name='collision'><geometry><plane><normal>0 0 1</normal><size>"
      << c.size << ' ' << c.size << "</size></plane></geometry></collision></link></model>"
      << "<model name='ball'><link name='link'><pose>" << c.ballX << ' ' << c.y << " 3 0 0 0</pose>"
      << "<inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>"
      << "<collision name='collision'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>"
      << "</link></model></world></sdf>";
    sdf::Root root;
    if (!root.LoadSdfString(xml.str()).empty()) throw std::runtime_error("invalid reference SDF");
    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto ground = world->GetModel("ground")->GetLink("link");
    auto ball = world->GetModel("ball")->GetLink("link");
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State cached;
    gz::physics::ForwardStep::Output output;
    input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
    for (int step = 0; step <= 1500; ++step) {
      if (step) world->Step(output, cached, input);
      if (!(step == 0 || step == 1 || step == 10 || step == 100 || step == 500 || step == 1000 || step == 1500)) continue;
      if (!first) std::cout << ",\n";
      first = false;
      std::cout << "{\"case\":\"" << c.name << "\",\"step\":" << step << ",\"input\":{\"position\":";
      vec(Eigen::Vector3d(c.x, c.y, c.z));
      std::cout << ",\"orientation\":"; quat(Eigen::Quaterniond(Eigen::AngleAxisd(c.tilt, Eigen::Vector3d::UnitY())));
      std::cout << ",\"size\":[" << c.size << ',' << c.size << "],\"ball\":";
      vec(Eigen::Vector3d(c.ballX, c.y, 3));
      std::cout << "},\"ground\":"; state(ground);
      std::cout << ",\"ball\":"; state(ball);
      std::cout << '}';
    }
  }
  std::cout << "\n]\n";
  std::ifstream maps("/proc/self/maps");
  std::ofstream out(argv[2]); out << maps.rdbuf();
  if (!out) return 3;
}
