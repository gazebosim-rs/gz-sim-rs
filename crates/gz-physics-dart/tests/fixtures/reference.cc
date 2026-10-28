// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//
// Records same-engine reference trajectories from the installed gz-physics
// dartsim plugin. See docs/DART.md for the exact build and run commands.
// Usage: reference <libgz-physics-dartsim-plugin.so>  (JSON on stdout)

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <set>
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

using Features = gz::physics::FeatureList<
    gz::physics::sdf::ConstructSdfWorld, gz::physics::ForwardStep,
    gz::physics::GetModelFromWorld, gz::physics::GetLinkFromModel,
    gz::physics::GetJointFromModel, gz::physics::GetBasicJointState,
    gz::physics::SetBasicJointState, gz::physics::LinkFrameSemantics>;

namespace {

std::string number(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.17g", value);
  return text;
}

const char *kBall = R"(
  <model name="ball">
    <link name="link">
      <pose>0 0 5 0 0 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.5</radius></sphere></geometry>
      </collision>
    </link>
  </model>)";

const char *kGround = R"(
  <model name="ground">
    <static>true</static>
    <link name="link">
      <pose>0 0 -0.5 0 0 0</pose>
      <collision name="collision">
        <geometry><box><size>20 20 1</size></box></geometry>
      </collision>
    </link>
  </model>)";

std::string world(const std::string &models) {
  return "<?xml version=\"1.0\"?><sdf version=\"1.11\"><world name=\"w\">"
         "<gravity>0 0 -9.81</gravity>" + models + "</world></sdf>";
}

std::string pendulum() {
  const double theta = 0.02;
  return world(R"(
  <model name="pendulum">
    <link name="link">
      <pose>)" + number(-std::sin(theta)) + " 0 " + number(-std::cos(theta)) +
               " 0 " + number(theta) + R"( 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
      </collision>
    </link>
    <joint name="hinge" type="revolute">
      <parent>world</parent>
      <child>link</child>
      <pose>0 0 1 0 0 0</pose>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>
  </model>)");
}

// A hinge whose axis, anchor, COM offset and inertial frame all differ from the
// link axes, so every frame conversion in the adapter matters.
const char *kTiltedLink = R"(
    <link name="link">
      <pose>0.3 -0.2 -0.9 0.3 -0.2 0.5</pose>
      <inertial>
        <pose>0.1 0.05 -0.2 0.4 0.1 -0.3</pose>
        <mass>2</mass>
        <inertia><ixx>0.03</ixx><iyy>0.05</iyy><izz>0.04</izz>
          <ixy>0.002</ixy><ixz>-0.001</ixz><iyz>0.003</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
      </collision>
    </link>
    <joint name="hinge" type="revolute">
      <parent>world</parent>
      <child>link</child>
      <pose>0.1 -0.2 0.8 0 0 0</pose>
      <axis><xyz>0.3 0.8 0.5</xyz></axis>
    </joint>)";

std::string tilted() {
  return world(std::string("<model name=\"tilted\">") + kTiltedLink +
               "</model>");
}

const double kTiltedRate = 0.7;

std::string vector(const Eigen::Vector3d &value) {
  return "[" + number(value.x()) + "," + number(value.y()) + "," +
         number(value.z()) + "]";
}

// The world-frame construction inputs the adapter's API takes for "tilted".
void tiltedInputs(const sdf::Model &model, bool &first) {
  const sdf::Link &link = *model.LinkByName("link");
  const sdf::Joint &joint = *model.JointByName("hinge");
  const Eigen::Isometry3d linkPose = gz::math::eigen3::convert(link.RawPose());
  const Eigen::Vector3d anchor =
      linkPose * gz::math::eigen3::convert(joint.RawPose().Pos());
  const Eigen::Vector3d axis =
      linkPose.linear() * gz::math::eigen3::convert(joint.Axis(0)->Xyz());
  const auto &inertial = link.Inertial();
  const auto &q = inertial.Pose().Rot();
  const auto &moi = inertial.MassMatrix().Moi();
  std::cout << (first ? "" : ",\n")
            << "{\"scenario\":\"tilted_hinge_inputs\",\"anchor\":"
            << vector(anchor) << ",\"axis\":" << vector(axis)
            << ",\"center\":"
            << vector(gz::math::eigen3::convert(inertial.Pose().Pos()))
            << ",\"inertial_orientation\":[" << number(q.X()) << ","
            << number(q.Y()) << "," << number(q.Z()) << "," << number(q.W())
            << "],\"tensor\":[" << number(moi(0, 0)) << ","
            << number(moi(1, 1)) << "," << number(moi(2, 2)) << ","
            << number(moi(0, 1)) << "," << number(moi(0, 2)) << ","
            << number(moi(1, 2)) << "],\"mass\":"
            << number(inertial.MassMatrix().Mass())
            << ",\"rate\":" << number(kTiltedRate) << "}";
  first = false;
}

bool recorded(std::size_t step) { return step <= 10 || step % 10 == 0; }

void run(const gz::plugin::PluginPtr &plugin, const std::string &name,
         const std::string &sdfText, const std::string &modelName,
         std::size_t steps, bool joint, bool &first,
         double rate = 0.0) {
  auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
  sdf::Root root;
  if (!root.LoadSdfString(sdfText).empty()) {
    throw std::runtime_error("invalid SDF for " + name);
  }
  auto physicsWorld = engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = physicsWorld->GetModel(modelName);
  auto link = model->GetLink("link");
  auto hinge = joint ? model->GetJoint("hinge") : nullptr;
  if (hinge) {
    hinge->SetVelocity(0, rate);
  }
  // FrameData reports the link origin's velocity; the adapter reports the COM's.
  const Eigen::Vector3d center = gz::math::eigen3::convert(
      root.WorldByIndex(0)->ModelByIndex(0)->LinkByIndex(0)->Inertial()
          .Pose().Pos());

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  input.Get<std::chrono::steady_clock::duration>() =
      std::chrono::milliseconds(1);
  for (std::size_t step = 0; step <= steps; ++step) {
    if (step > 0) {
      physicsWorld->Step(output, state, input);
    }
    if (!recorded(step)) {
      continue;
    }
    const auto data = link->FrameDataRelativeToWorld();
    const Eigen::Quaterniond q(data.pose.linear());
    const Eigen::Vector3d p = data.pose.translation();
    const Eigen::Vector3d w = data.angularVelocity;
    const Eigen::Vector3d v =
        data.linearVelocity + w.cross(data.pose.linear() * center);
    std::cout << (first ? "" : ",\n") << "{\"scenario\":\"" << name
              << "\",\"step\":" << step << ",\"position\":[" << number(p.x())
              << "," << number(p.y()) << "," << number(p.z())
              << "],\"orientation\":[" << number(q.x()) << ","
              << number(q.y()) << "," << number(q.z()) << ","
              << number(q.w()) << "],\"velocity\":[" << number(v.x()) << ","
              << number(v.y()) << "," << number(v.z())
              << "],\"angular_velocity\":[" << number(w.x()) << ","
              << number(w.y()) << "," << number(w.z()) << "]";
    if (hinge) {
      std::cout << ",\"joint_position\":" << number(hinge->GetPosition(0))
                << ",\"joint_velocity\":" << number(hinge->GetVelocity(0));
    }
    std::cout << "}";
    first = false;
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <dartsim plugin library>\n";
    return 2;
  }
  gz::plugin::Loader loader;
  const auto names = loader.LoadLib(argv[1]);
  const std::string pluginName = "gz::physics::dartsim::Plugin";
  if (names.count(pluginName) == 0) {
    std::cerr << "dartsim plugin not found in " << argv[1] << "\n";
    return 1;
  }
  auto plugin = loader.Instantiate(pluginName);
  bool first = true;
  std::cout << "[\n";
  run(plugin, "free_fall", world(kBall), "ball", 1000, false, first);
  run(plugin, "ground_contact", world(std::string(kGround) + kBall), "ball",
      2000, false, first);
  run(plugin, "world_revolute", pendulum(), "pendulum", 4000, true, first);
  sdf::Root root;
  if (!root.LoadSdfString(tilted()).empty()) {
    std::cerr << "invalid tilted SDF\n";
    return 1;
  }
  tiltedInputs(*root.WorldByIndex(0)->ModelByIndex(0), first);
  run(plugin, "tilted_hinge", tilted(), "tilted", 2000, true, first,
      kTiltedRate);
  std::cout << "\n]\n";
  return 0;
}
