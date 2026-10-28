/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Records trajectories from the installed gz-physics MuJoCo plugin for the
 * Rust parity fixture. See README.md beside this file for the exact command.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>

struct Features : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::GetJointFromModel,
  gz::physics::LinkFrameSemantics,
  gz::physics::GetBasicJointState,
  gz::physics::ForwardStep> {};

namespace {

const char *kSphereLink = R"(
      <link name="link">
        <inertial>
          <mass>%MASS%</mass>
          <inertia><ixx>%I%</ixx><iyy>%I%</iyy><izz>%I%</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>%R%</radius></sphere></geometry>
        </collision>
      </link>)";

std::string Replace(std::string _text, const std::string &_key,
                    const std::string &_value)
{
  for (auto at = _text.find(_key); at != std::string::npos;
       at = _text.find(_key, at + _value.size()))
  {
    _text.replace(at, _key.size(), _value);
  }
  return _text;
}

std::string Sphere(const std::string &_mass, const std::string &_inertia,
                   const std::string &_radius)
{
  return Replace(Replace(Replace(kSphereLink, "%MASS%", _mass), "%I%",
                         _inertia), "%R%", _radius);
}

std::string World(const std::string &_models)
{
  return "<?xml version='1.0'?><sdf version='1.11'><world name='w'>"
         "<gravity>0 0 -9.81</gravity>" + _models + "</world></sdf>";
}

const std::string kGround = R"(
    <model name="ground"><static>true</static><pose>0 0 -0.5 0 0 0</pose>
      <link name="link"><collision name="collision">
        <geometry><box><size>20 20 1</size></box></geometry>
      </collision></link>
    </model>)";

std::string FreeSphere(const std::string &_z)
{
  return "<model name='ball'><pose>0 0 " + _z + " 0 0 0</pose>" +
         Sphere("1", "0.1", "0.5") + "</model>";
}

// theta = 0.02 rad about +Y around the world origin, one meter below it.
const std::string kPendulum =
    "<model name='pendulum'>"
    "<pose>-0.019998666693333080 0 -0.99980000666657776 0 0.02 0</pose>" +
    Sphere("1", "0.004", "0.1") +
    R"(<joint name="hinge" type="revolute"><parent>world</parent>
        <child>link</child><pose>0 0 1 0 0 0</pose>
        <axis><xyz>0 1 0</xyz></axis></joint>
    </model>)";

template <typename Engine>
void Run(const Engine &_engine, const std::string &_scenario,
         const std::string &_sdf, const std::string &_model, bool _joint,
         int _steps, bool &_first)
{
  sdf::Root root;
  const auto errors = root.LoadSdfString(_sdf);
  if (!errors.empty())
  {
    std::cerr << errors << std::endl;
    std::exit(1);
  }
  auto world = _engine->ConstructWorld(*root.WorldByIndex(0));
  auto link = world->GetModel(_model)->GetLink("link");
  auto joint = _joint ? world->GetModel(_model)->GetJoint("hinge") : nullptr;
  gz::physics::ForwardStep::Output output;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input input;
  input.Get<std::chrono::steady_clock::duration>() =
      std::chrono::milliseconds(1);
  for (int step = 1; step <= _steps; ++step)
  {
    world->Step(output, state, input);
    if (step % 10 != 0)
      continue;
    const auto frame = link->FrameDataRelativeToWorld();
    const Eigen::Quaterniond q(frame.pose.linear());
    std::printf(
        "%s{\"scenario\":\"%s\",\"iteration\":%d,"
        "\"position\":[%.17g,%.17g,%.17g],"
        "\"orientation\":[%.17g,%.17g,%.17g,%.17g],"
        "\"velocity\":[%.17g,%.17g,%.17g],"
        "\"angular_velocity\":[%.17g,%.17g,%.17g]",
        _first ? "" : ",\n", _scenario.c_str(), step,
        frame.pose.translation().x(), frame.pose.translation().y(),
        frame.pose.translation().z(), q.x(), q.y(), q.z(), q.w(),
        frame.linearVelocity.x(), frame.linearVelocity.y(),
        frame.linearVelocity.z(), frame.angularVelocity.x(),
        frame.angularVelocity.y(), frame.angularVelocity.z());
    if (joint)
    {
      std::printf(",\"q\":%.17g,\"dq\":%.17g", joint->GetPosition(0),
                  joint->GetVelocity(0));
    }
    std::printf("}");
    _first = false;
  }
}

// Best of five: SDF parsing excluded; world construction plus one 1 ms step,
// matching examples/load_time.rs.
template <typename Engine>
void Load(const Engine &_engine, int _count, bool _hinged)
{
  // Same hinge as examples/load_time.rs: world anchor one meter above the link.
  const std::string hinge = _hinged ?
      "<joint name='hinge' type='revolute'><parent>world</parent>"
      "<child>link</child><pose>0 0 1 0 0 0</pose>"
      "<axis><xyz>0 1 0</xyz></axis></joint>" : "";
  std::string models;
  for (int i = 0; i < _count; ++i)
  {
    models += "<model name='m" + std::to_string(i) + "'><pose>" +
              std::to_string(i) + " 0 1 0 0 0</pose>" +
              Sphere("1", "0.004", "0.1") + hinge + "</model>";
  }
  sdf::Root root;
  if (!root.LoadSdfString(World(models)).empty())
    std::exit(1);
  double best = 1e300;
  for (int run = 0; run < 5; ++run)
  {
    const auto start = std::chrono::steady_clock::now();
    auto world = _engine->ConstructWorld(*root.WorldByIndex(0));
    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    input.Get<std::chrono::steady_clock::duration>() =
        std::chrono::milliseconds(1);
    world->Step(output, state, input);
    const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - start;
    best = std::min(best, elapsed.count());
  }
  std::printf("%d %s bodies: %.3f ms\n", _count, _hinged ? "hinged" : "free",
              best);
}

}  // namespace

int main(int _argc, char **_argv)
{
  if (_argc < 2 || (_argc > 2 && std::string(_argv[2]) != "--load"))
  {
    std::cerr << "usage: mujoco_reference <libgz-physics-mujoco-plugin.so> "
                 "[--load N...]\n";
    return 2;
  }
  gz::plugin::Loader loader;
  loader.LoadLib(_argv[1]);
  const auto names = gz::physics::FindFeatures3d<Features>::From(loader);
  if (names.empty())
  {
    std::cerr << "no MuJoCo plugin with the required features\n";
    return 1;
  }
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate(*names.begin()));
  if (_argc > 2)
  {
    for (int i = 3; i < _argc; ++i)
    {
      Load(engine, std::stoi(_argv[i]), false);
      Load(engine, std::stoi(_argv[i]), true);
    }
    return 0;
  }
  bool first = true;
  std::printf("[\n");
  Run(engine, "free_fall", World(FreeSphere("5")), "ball", false, 1000,
      first);
  Run(engine, "ground_contact", World(kGround + FreeSphere("1")), "ball",
      false, 2000, first);
  Run(engine, "pendulum", World(kPendulum), "pendulum", true, 4000, first);
  std::printf("\n]\n");
  return 0;
}
