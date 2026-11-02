// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Diagnostic SOURCE for the exact compound_com.rs origin-change experiment.
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<
    gz::physics::ForwardStep, gz::physics::SetFreeGroupWorldVelocity,
    gz::physics::SetFreeGroupWorldPose, gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel, gz::physics::GetShapeFromLink,
    gz::physics::GetShapeKinematicProperties, gz::physics::LinkFrameSemantics,
    gz::physics::AddLinkExternalForceTorque, gz::physics::sdf::ConstructSdfWorld>;

static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
static void maps(const std::filesystem::path &path) {
  std::ifstream source("/proc/self/maps");
  std::ofstream saved(path);
  saved << source.rdbuf();
  saved.flush();
  require(bool(source) && bool(saved), "maps capture failed");
}
static void vector(std::ostream &out, const Eigen::Vector3d &value) {
  require(value.allFinite(), "nonfinite vector");
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}
static void quaternion(std::ostream &out, const Eigen::Quaterniond &q) {
  require(q.coeffs().allFinite(), "nonfinite quaternion");
  require(std::abs(q.squaredNorm() - 1.0) <= 1e-12, "nonunit quaternion");
  out << '[' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
}
static void pose(std::ostream &out, const Eigen::Isometry3d &value) {
  require(value.matrix().allFinite(), "nonfinite pose");
  out << "{\"position\":";
  vector(out, value.translation());
  out << ",\"orientation\":";
  quaternion(out, Eigen::Quaterniond(value.linear()));
  out << '}';
}

// Same RPY convention and scalar expression as nalgebra0.35's
// UnitQuaternion::from_euler_angles used by the failing test. Actual rounded
// inputs are retained below; this is input construction, not a physics oracle.
static Eigen::Quaterniond inputQuaternion(double roll, double pitch, double yaw) {
  const double sr = std::sin(roll * 0.5), cr = std::cos(roll * 0.5);
  const double sp = std::sin(pitch * 0.5), cp = std::cos(pitch * 0.5);
  const double sy = std::sin(yaw * 0.5), cy = std::cos(yaw * 0.5);
  return {cr * cp * cy + sr * sp * sy,
          sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}
static std::string xmlPose(const Eigen::Vector3d &position,
                           const Eigen::Quaterniond &orientation) {
  std::ostringstream out;
  out << std::setprecision(17) << "<pose rotation_format='quat_xyzw'>"
      << position.x() << ' ' << position.y() << ' ' << position.z() << ' '
      << orientation.x() << ' ' << orientation.y() << ' '
      << orientation.z() << ' ' << orientation.w() << "</pose>";
  return out.str();
}
static std::string collision(unsigned index, const Eigen::Vector3d &position,
                             const Eigen::Quaterniond &orientation) {
  return "<collision name='c" + std::to_string(index) + "'>" +
      xmlPose(position, orientation) + "<geometry>" +
      (index == 1 ? "<box><size>0.6 0.4 0.8</size></box>" :
                    "<sphere><radius>0.2</radius></sphere>") +
      "</geometry></collision>";
}

int main(int argc, char **argv) {
  if (argc != 3) return 2; // pinned plugin path; NEW output directory
  const std::filesystem::path output(argv[2]);
  require(std::filesystem::create_directory(output), "output already exists");
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  require(bool(engine), "selected engine lacks required features");
  maps(output / "before.maps");

  const Eigen::Vector3d center(0.1, -0.05, 0.2), origin(1.0, 2.0, 3.0);
  const Eigen::Vector3d comVelocity(0.2, -0.1, 0.3), angular(0.1, 0.2, -0.3);
  const Eigen::Vector3d force(1.0, 2.0, 3.0), torque(0.2, -0.1, 0.3);
  const auto rawRotation = inputQuaternion(0.2, 0.1, -0.3);
  const auto rotation = rawRotation.normalized();
  const auto boxRotation = inputQuaternion(0.0, 1.5708, 0.3).normalized();
  const Eigen::Vector3d worldCOM = origin + rotation * center;
  const std::array<Eigen::Vector3d, 3> localShapes = {
      Eigen::Vector3d(-0.4, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.4, 0.0, 0.0)};
  std::array<Eigen::Isometry3d, 3> firstWorldShapes;
  Eigen::Vector3d firstCOM, firstCOMVelocity;
  Eigen::Matrix3d firstRotation;

  std::ofstream inputs(output / "inputs.json");
  inputs << std::setprecision(17)
      << "{\"source_test\":\"changing_body_origin_preserves_world_shapes_com_motion_and_pending_wrenches\","
      << "\"dt_ns\":1000000,\"steps\":10,\"gravity\":[0,0,0],\"mass\":1,"
      << "\"inertia\":[1,1,1,0,0,0],\"inertial_orientation\":[0,0,0,1],"
      << "\"raw_body_quaternion\":";
  quaternion(inputs, rawRotation);
  inputs << ",\"body_quaternion\":";
  quaternion(inputs, rotation);
  inputs << ",\"box_quaternion\":";
  quaternion(inputs, boxRotation);
  inputs << ",\"com_velocity\":";
  vector(inputs, comVelocity);
  inputs << ",\"angular_velocity\":";
  vector(inputs, angular);
  inputs << ",\"first_step_world_force\":";
  vector(inputs, force);
  inputs << ",\"first_step_world_torque\":";
  vector(inputs, torque);
  inputs << ",\"cases\":[";
  std::cout << std::setprecision(17);

  for (unsigned scenario = 0; scenario < 2; ++scenario) {
    const Eigen::Vector3d localCOM = scenario == 0 ? center : Eigen::Vector3d(0.0, 0.0, 0.0);
    const Eigen::Vector3d bodyOrigin = scenario == 0 ? origin : worldCOM;
    std::string shapes;
    for (unsigned index = 0; index < 3; ++index) {
      const Eigen::Vector3d local = localShapes[index] -
          (scenario == 0 ? Eigen::Vector3d(0.0, 0.0, 0.0) : center);
      shapes += collision(index, local,
          index == 1 ? boxRotation : Eigen::Quaterniond::Identity());
    }
    const std::string xml =
        "<sdf version='1.11'><world name='w'><gravity>0 0 0</gravity>"
        "<model name='compound'><link name='body'>" +
        xmlPose(bodyOrigin, rotation) + "<inertial>" +
        xmlPose(localCOM, Eigen::Quaterniond::Identity()) +
        "<mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz>"
        "</inertia></inertial>" + shapes + "</link></model></world></sdf>";
    std::ofstream fixture(output / (std::to_string(scenario) + ".sdf"));
    fixture << xml;
    fixture.close();
    require(bool(fixture), "XML save failed");
    sdf::Root root;
    require(root.LoadSdfString(xml).empty() && root.WorldByIndex(0), "SDF load failed");
    const auto *sdfModel = root.WorldByIndex(0)->ModelByName("compound");
    const auto *sdfLink = sdfModel ? sdfModel->LinkByName("body") : nullptr;
    require(sdfLink && sdfLink->CollisionCount() == 3, "SDF collision count");
    const auto observedCenter = sdfLink->Inertial().Pose().Pos();
    const Eigen::Vector3d loadedCOM(observedCenter.X(), observedCenter.Y(), observedCenter.Z());
    require((loadedCOM - localCOM).cwiseAbs().maxCoeff() <= 1e-12, "SDF COM changed");
    require(sdfLink->Inertial().MassMatrix().Mass() == 1.0, "mass changed");
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    require(bool(world), "world construction failed");
    auto model = world->GetModel("compound");
    require(bool(model), "model construction failed");
    auto body = model->GetLink("body");
    require(bool(body) && body->GetShapeCount() == 3, "link construction failed");
    auto group = body->FindFreeGroup();
    require(bool(group), "missing free group");
    // Override through the maintained public pose setter to avoid treating an
    // XML Euler conversion as the declared body transform.
    Eigen::Isometry3d commanded = Eigen::Isometry3d::Identity();
    commanded.linear() = rotation.toRotationMatrix();
    commanded.translation() = bodyOrigin;
    group->SetWorldPose(commanded);
    const auto initial = body->FrameDataRelativeToWorld();
    const Eigen::Vector3d originVelocity = comVelocity - angular.cross(initial.pose.linear() * loadedCOM);
    group->SetWorldAngularVelocity(angular);
    group->SetWorldLinearVelocity(originVelocity);

    if (scenario) inputs << ',';
    inputs << "{\"scenario\":" << scenario << ",\"position\":";
    vector(inputs, bodyOrigin);
    inputs << ",\"local_com\":";
    vector(inputs, loadedCOM);
    inputs << ",\"commanded_origin_velocity\":";
    vector(inputs, originVelocity);
    inputs << '}';
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output result;
    input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
    for (unsigned tick = 0; tick <= 10; ++tick) {
      if (tick) {
        if (tick == 1) {
          // Default position would be the ORIGIN. This call is explicitly at
          // the loaded local COM, matching World::step's pending-wrench path.
          body->AddExternalForce(force, gz::physics::FrameID::World(), loadedCOM);
          body->AddExternalTorque(torque, gz::physics::FrameID::World());
        }
        world->Step(result, state, input);
      }
      const auto value = body->FrameDataRelativeToWorld();
      const Eigen::Vector3d offset = value.pose.linear() * loadedCOM;
      const Eigen::Vector3d actualCOM = value.pose * loadedCOM;
      const Eigen::Vector3d actualCOMVelocity = value.linearVelocity + value.angularVelocity.cross(offset);
      require(value.pose.matrix().allFinite() && value.linearVelocity.allFinite() &&
              value.angularVelocity.allFinite() && actualCOM.allFinite() &&
              actualCOMVelocity.allFinite(), "nonfinite native observation");
      std::array<Eigen::Isometry3d, 3> worldShapes;
      for (unsigned index = 0; index < 3; ++index) {
        auto shape = body->GetShape("c" + std::to_string(index));
        require(bool(shape), "missing collision identity");
        worldShapes[index] = value.pose * shape->GetRelativeTransform();
        require(worldShapes[index].matrix().allFinite(), "nonfinite shape pose");
      }
      if (tick == 0) {
        require((actualCOM - worldCOM).cwiseAbs().maxCoeff() <= 1e-12, "initial COM position changed");
        require((actualCOMVelocity - comVelocity).cwiseAbs().maxCoeff() <= 1e-12, "initial COM velocity changed");
        require((value.angularVelocity - angular).cwiseAbs().maxCoeff() <= 1e-12, "initial angular velocity changed");
        if (scenario == 0) {
          firstWorldShapes = worldShapes;
          firstCOM = actualCOM;
          firstCOMVelocity = actualCOMVelocity;
          firstRotation = value.pose.linear();
        } else {
          require((actualCOM - firstCOM).cwiseAbs().maxCoeff() <= 1e-12, "representations differ initially");
          require((actualCOMVelocity - firstCOMVelocity).cwiseAbs().maxCoeff() <= 1e-12, "COM twists differ initially");
          require((value.pose.linear() - firstRotation).cwiseAbs().maxCoeff() <= 1e-12, "body rotations differ initially");
          for (unsigned index = 0; index < 3; ++index)
            require((worldShapes[index].matrix() - firstWorldShapes[index].matrix()).cwiseAbs().maxCoeff() <= 1e-12,
                    "world shapes differ initially");
        }
      }
      std::cout << "{\"scenario\":" << scenario << ",\"tick\":" << tick
          << ",\"time_ns\":" << tick * 1000000 << ",\"wrench_this_step\":"
          << (tick == 1 ? "true" : "false") << ",\"position\":";
      vector(std::cout, value.pose.translation());
      std::cout << ",\"orientation\":";
      quaternion(std::cout, Eigen::Quaterniond(value.pose.linear()));
      std::cout << ",\"velocity\":";
      vector(std::cout, actualCOMVelocity);
      std::cout << ",\"angular_velocity\":";
      vector(std::cout, value.angularVelocity);
      std::cout << ",\"origin_velocity\":";
      vector(std::cout, value.linearVelocity);
      std::cout << ",\"com_position\":";
      vector(std::cout, actualCOM);
      std::cout << ",\"local_com\":";
      vector(std::cout, loadedCOM);
      std::cout << ",\"world_shapes\":[";
      for (unsigned index = 0; index < 3; ++index) {
        if (index) std::cout << ',';
        pose(std::cout, worldShapes[index]);
      }
      std::cout << "]}\n";
    }
  }
  inputs << "]}\n";
  inputs.flush();
  require(bool(inputs), "input record failed");
  maps(output / "after.maps");
}
