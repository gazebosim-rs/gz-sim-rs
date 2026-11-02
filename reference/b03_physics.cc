// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Explicitly configured public-plugin physics workload; no Server or transport.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Geometry>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Error.hh>
#include <sdf/Link.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
using Features = gz::physics::FeatureList<gz::physics::ForwardStep,
    gz::physics::GetModelFromWorld, gz::physics::GetLinkFromModel,
    gz::physics::GetJointFromModel, gz::physics::GetShapeFromLink,
    gz::physics::LinkFrameSemantics, gz::physics::GetBasicJointState,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetRevoluteJointProperties, gz::physics::Gravity,
    gz::physics::CollisionDetector, gz::physics::Solver,
    gz::physics::CollisionPairMaxContacts, gz::physics::sdf::ConstructSdfWorld>;
static constexpr const char *contract = "b03-physics-unlimited-pair-v1";
static constexpr const char *role = "native-public-physics";
static constexpr std::array<const char *, 22> fields = {
    "px", "py", "pz", "qx", "qy", "qz", "qw", "origin_vx", "origin_vy",
    "origin_vz", "com_vx", "com_vy", "com_vz", "wx", "wy", "wz", "com_px",
    "com_py", "com_pz", "local_com_x", "local_com_y", "local_com_z"};
static void require(bool ok, const char *why) {
  if (!ok) throw std::runtime_error(why);
}
static std::ofstream create(const std::filesystem::path &path) {
  require(!std::filesystem::exists(path), "refuse to overwrite output");
  std::ofstream out(path); require(bool(out), "output open failed"); return out;
}
static void maps(const std::filesystem::path &path) {
  std::ifstream in("/proc/self/maps"); auto out = create(path);
  require(bool(in), "maps open failed"); out << in.rdbuf(); out.flush();
  require(bool(out), "maps save failed");
}
template <class Numbers> static void numbers(std::ostream &out, const Numbers &values) {
  out << '['; bool first = true;
  for (double value : values) {
    require(std::isfinite(value), "nonfinite observation");
    if (!first) out << ',';
    first = false; out << value;
  }
  out << ']';
}
struct Entity {
  std::string family, name;
  std::size_t id, parent;
  bool root = false;
};
struct Frame {
  std::array<std::array<double, 22>, 7> bodies{};
  std::array<std::array<double, 2>, 4> joints{};
};
int main(int argc, char **argv) try {
  require(argc == 5, "expected plugin XML new-prefix correctness|measure");
  const std::string mode(argv[4]);
  require(mode == "correctness" || mode == "measure", "unknown mode");
  const std::string prefix(argv[3]);
  for (const auto *suffix : {".input.sdf", ".state.json", ".metrics.json", ".maps"})
    require(!std::filesystem::exists(prefix + suffix), "output prefix already used");
  std::ifstream input(argv[2]); require(bool(input), "input absent");
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  auto saved = create(prefix + ".input.sdf"); saved << xml; saved.close();
  require(bool(saved), "input save failed");
  sdf::Root root;
  require(root.Load(argv[2]).empty() && root.WorldCount() == 1, "DOM load failed");
  const auto *dom = root.WorldByIndex(0);
  require(dom && dom->Name() == "default" && dom->ModelCount() == 3 && dom->LightCount() == 1,
          "world differs");
  require(dom->LightByIndex(0)->Name() == "sun", "input sun missing");
  gz::plugin::Loader loader; loader.LoadLib(argv[1]);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
  require(bool(engine), "public engine features absent");
  auto world = engine->ConstructWorld(*dom);
  require(bool(world) && world->GetModelCount() == 3, "world construction failed");
  // Bare ConstructSdfWorld ignores Physics DOM options. This is an explicit,
  // separate configuration; it does not qualify original Server defaults.
  world->SetCollisionDetector("ode"); world->SetSolver("dantzig");
  world->SetCollisionPairMaxContacts(std::numeric_limits<std::size_t>::max());
  const auto gravity = dom->Gravity();
  world->SetGravity(Eigen::Vector3d(gravity.X(), gravity.Y(), gravity.Z()));
  const auto actualGravity = world->GetGravity();
  const auto detector = world->GetCollisionDetector(), solver = world->GetSolver();
  const auto pairLimit = world->GetCollisionPairMaxContacts();
  require(detector == "ode" && solver == "DantzigBoxedLcpSolver" &&
          pairLimit == std::numeric_limits<std::size_t>::max(), "public settings differ");
  require(actualGravity == Eigen::Vector3d(0, 0, -9.8), "gravity differs");
  // Dantzig iterations are unsupported. Do not query a getter that warns and
  // returns a sentinel zero, or mislabel that sentinel as zero iterations.
  using Model = decltype(world->GetModel(std::string{}));
  using Link = decltype(std::declval<Model>()->GetLink(std::string{}));
  using Joint = decltype(std::declval<Model>()->GetJoint(std::string{}));
  struct Body { std::string name; Link link; Eigen::Vector3d localCOM; };
  struct Hinge { std::string name; Joint joint; };
  std::vector<Body> bodies; bodies.reserve(7);
  std::vector<Hinge> joints; joints.reserve(4);
  std::vector<Entity> entities; entities.reserve(30);
  std::set<std::size_t> ids;
  auto identity = [&](const char *family, const std::string &name, auto entity,
                      std::size_t parent, bool isRoot = false) {
    require(bool(entity) && ids.insert(entity->EntityID()).second, "identity collision");
    entities.push_back({family, name, entity->EntityID(), parent, isRoot});
  };
  identity("WORLD", "default", world, 0, true);
  for (std::size_t m = 0; m < dom->ModelCount(); ++m) {
    const auto *dm = dom->ModelByIndex(m); auto model = world->GetModel(dm->Name());
    require(bool(model) && model->GetName() == dm->Name() &&
            model->GetLinkCount() == dm->LinkCount() && model->GetJointCount() == dm->JointCount(),
            "model identity differs");
    const auto mn = "default::" + dm->Name(); identity("MODEL", mn, model, world->EntityID());
    for (std::size_t l = 0; l < dm->LinkCount(); ++l) {
      const auto *dl = dm->LinkByIndex(l); auto link = model->GetLink(dl->Name());
      require(bool(link) && link->GetName() == dl->Name() &&
              link->GetShapeCount() == dl->CollisionCount(), "link identity differs");
      const auto ln = mn + "::" + dl->Name(); identity("BODY", ln, link, model->EntityID());
      const auto center = dl->Inertial().Pose().Pos();
      bodies.push_back({ln, link, Eigen::Vector3d(center.X(), center.Y(), center.Z())});
      for (std::size_t s = 0; s < dl->CollisionCount(); ++s) {
        const auto *dc = dl->CollisionByIndex(s); auto shape = link->GetShape(dc->Name());
        require(bool(shape) && shape->GetName() == dc->Name() &&
                shape->GetLink()->EntityID() == link->EntityID(), "shape identity differs");
        identity("SHAPE", ln + "::" + dc->Name(), shape, link->EntityID());
      }
    }
    for (std::size_t j = 0; j < dm->JointCount(); ++j) {
      const auto *dj = dm->JointByIndex(j); auto joint = model->GetJoint(dj->Name());
      require(bool(joint) && joint->GetName() == dj->Name() &&
              joint->GetDegreesOfFreedom() == 1 && bool(joint->CastToRevoluteJoint()),
              "joint identity differs");
      const auto jn = mn + "::" + dj->Name(); identity("JOINT", jn, joint, model->EntityID());
      joints.push_back({jn, joint});
    }
  }
  require(bodies.size() == 7 && joints.size() == 4 && entities.size() == 30,
          "physical cardinality differs");
  std::array<Frame, 100> frames{};
  gz::physics::ForwardStep::Input stepInput;
  gz::physics::ForwardStep::State stepState;
  gz::physics::ForwardStep::Output stepOutput;
  stepInput.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
  std::uint64_t steps = 0, bodyQueries = 0, jointQueries = 0;
  const auto begin = std::chrono::steady_clock::now();
  for (auto &frame : frames) {
    world->Step(stepOutput, stepState, stepInput); ++steps;
    for (std::size_t b = 0; b < bodies.size(); ++b) {
      const auto &body = bodies[b];
      gz::physics::FrameData3d local; local.pose.translation() = body.localCOM;
      const auto origin = engine->Resolve(gz::physics::RelativeFrameData3d(
          body.link->GetFrameID(), gz::physics::FrameData3d()));
      const auto com = engine->Resolve(gz::physics::RelativeFrameData3d(body.link->GetFrameID(), local));
      const auto p = gz::math::eigen3::convert(origin.pose);
      frame.bodies[b] = {p.Pos().X(), p.Pos().Y(), p.Pos().Z(),
          p.Rot().X(), p.Rot().Y(), p.Rot().Z(), p.Rot().W(),
          origin.linearVelocity.x(), origin.linearVelocity.y(), origin.linearVelocity.z(),
          com.linearVelocity.x(), com.linearVelocity.y(), com.linearVelocity.z(),
          origin.angularVelocity.x(), origin.angularVelocity.y(), origin.angularVelocity.z(),
          com.pose.translation().x(), com.pose.translation().y(), com.pose.translation().z(),
          body.localCOM.x(), body.localCOM.y(), body.localCOM.z()};
      ++bodyQueries;
    }
    for (std::size_t j = 0; j < joints.size(); ++j) {
      frame.joints[j] = {joints[j].joint->GetPosition(0), joints[j].joint->GetVelocity(0)};
      ++jointQueries;
    }
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - begin).count();
  require(steps == 100 && bodyQueries == 700 && jointQueries == 400 && elapsed > 0,
          "incomplete measured work");
  // All validation, serialization and maps IO are outside the measured window.
  auto state = create(prefix + ".state.json"); state << std::setprecision(17);
  state << "{\"schema_version\":1,\"contract\":" << std::quoted(contract)
        << ",\"role\":" << std::quoted(role) << ",\"declared_uri\":\"b03.sdf\",\"body_fields\":[";
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i) state << ',';
    state << std::quoted(fields[i]);
  }
  state << "],\"settings\":{\"gravity\":{\"value\":";
  numbers(state, std::array<double,3>{actualGravity.x(), actualGravity.y(), actualGravity.z()});
  state << ",\"evidence\":\"public-readback\"},"
        << "\"detector\":{\"value\":\"ode\",\"evidence\":\"public-readback\"},"
        << "\"solver\":{\"value\":\"DantzigBoxedLcpSolver\",\"evidence\":\"public-readback\"},"
        << "\"pair_max_contacts\":{\"value\":\"unlimited\",\"evidence\":\"public-readback\",\"raw\":"
        << std::quoted(std::to_string(pairLimit)) << "},"
        << "\"global_max_contacts\":{\"value\":10000,\"evidence\":\"source-established\"},"
        << "\"dt_ns\":{\"value\":1000000,\"evidence\":\"forward-step-input\"},"
        << "\"solver_iterations\":{\"value\":null,\"evidence\":\"unsupported\"}},\"entities\":[";
  for (std::size_t i = 0; i < entities.size(); ++i) {
    if (i) state << ',';
    const auto &e = entities[i];
    state << "{\"family\":" << std::quoted(e.family) << ",\"name\":" << std::quoted(e.name)
          << ",\"id\":" << e.id << ",\"parent_id\":";
    if (e.root) state << "null"; else state << e.parent;
    state << '}';
  }
  state << "],\"frames\":[";
  for (std::size_t f = 0; f < frames.size(); ++f) {
    if (f) state << ',';
    state << "{\"tick\":" << f + 1 << ",\"time_ns\":" << (f + 1) * 1000000 << ",\"bodies\":[";
    for (std::size_t b = 0; b < bodies.size(); ++b) {
      if (b) state << ',';
      state << "{\"name\":" << std::quoted(bodies[b].name) << ",\"id\":"
            << bodies[b].link->EntityID() << ",\"values\":";
      numbers(state, frames[f].bodies[b]); state << '}';
    }
    state << "],\"joints\":[";
    for (std::size_t j = 0; j < joints.size(); ++j) {
      if (j) state << ',';
      state << "{\"name\":" << std::quoted(joints[j].name) << ",\"id\":"
            << joints[j].joint->EntityID() << ",\"values\":";
      numbers(state, frames[f].joints[j]); state << '}';
    }
    state << "]}";
  }
  state << "]}\n"; state.flush(); require(bool(state), "state save failed");
  auto metrics = create(prefix + ".metrics.json");
  metrics << "{\"schema_version\":1,\"contract\":" << std::quoted(contract)
          << ",\"role\":" << std::quoted(role) << ",\"mode\":" << std::quoted(mode)
          << ",\"steps\":" << steps << ",\"dt_ns\":1000000,\"body_queries\":" << bodyQueries
          << ",\"joint_queries\":" << jointQueries
          << ",\"worlds\":1,\"bodies\":7,\"joints\":4,\"colliders\":15,\"sensors\":0,"
          << "\"rendering\":false,\"network\":false,\"window_wall_ns\":" << elapsed << "}\n";
  metrics.flush(); require(bool(metrics), "metrics save failed");
  maps(prefix + ".maps");
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n'; return 1;
}
