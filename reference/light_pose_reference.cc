// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Reference instrumentation only; no Rust system or plugin contract.
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CastShadows.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Transparency.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>

namespace sim = gz::sim;
namespace comp = gz::sim::components;
struct Row
{
  std::string kind, name;
  sim::Entity id{0}, parent{0};
  gz::math::Pose3d local, world;
  bool on{false}, visualize{false}, shadows{false};
  double transparency{0};
  int type{0};
};

void poseJson(std::ostream &out, const gz::math::Pose3d &pose)
{
  out << '[' << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z()
      << ',' << pose.Rot().X() << ',' << pose.Rot().Y() << ',' << pose.Rot().Z()
      << ',' << pose.Rot().W() << ']';
}

bool near(const gz::math::Pose3d &a, const gz::math::Pose3d &b)
{
  const double values[] = {a.Pos().X()-b.Pos().X(), a.Pos().Y()-b.Pos().Y(),
      a.Pos().Z()-b.Pos().Z(), a.Rot().X()-b.Rot().X(), a.Rot().Y()-b.Rot().Y(),
      a.Rot().Z()-b.Rot().Z(), a.Rot().W()-b.Rot().W()};
  for (double value : values)
    if (!std::isfinite(value) || std::abs(value) >= 1e-12) return false;
  return true;
}

class Observe final : public sim::System, public sim::ISystemPostUpdate
{
 public:
  std::vector<Row> rows;
  sim::Entity world{0};
  bool failed{false};
  unsigned calls{0};
  void PostUpdate(const sim::UpdateInfo &info,
      const sim::EntityComponentManager &ecm) override
  {
    ++calls;
    if (calls != 1 || info.paused || info.iterations != 1 ||
        info.simTime != std::chrono::milliseconds(1)) failed = true;
    ecm.Each<comp::World>([&](auto id, const auto *)
        { if (world != 0) failed = true; world = id; return true; });
    // Match the broadcaster's exact component-query sets and family order.
    ecm.Each<comp::Visual, comp::Name, comp::Pose>(
        [&](auto id, const auto *, const auto *name, const auto *pose)
        {
          Row row;
          row.kind = "visual"; row.id = id; row.name = name->Data();
          row.local = pose->Data(); row.world = sim::worldPose(id, ecm);
          const auto *parent = ecm.Component<comp::ParentEntity>(id);
          const auto *shadows = ecm.Component<comp::CastShadows>(id);
          const auto *transparency = ecm.Component<comp::Transparency>(id);
          if (!parent || !shadows || !transparency) failed = true;
          else { row.parent = parent->Data(); row.shadows = shadows->Data();
            row.transparency = transparency->Data(); }
          rows.push_back(row);
          return true;
        });
    ecm.Each<comp::Light, comp::Name, comp::Pose>(
        [&](auto id, const auto *light, const auto *name, const auto *pose)
        {
          Row row;
          row.kind = "light"; row.id = id; row.name = name->Data();
          row.local = pose->Data(); row.world = sim::worldPose(id, ecm);
          const auto *parent = ecm.Component<comp::ParentEntity>(id);
          if (!parent) failed = true; else row.parent = parent->Data();
          row.on = light->Data().LightOn(); row.visualize = light->Data().Visualize();
          row.shadows = light->Data().CastShadows();
          row.type = static_cast<int>(light->Data().Type());
          rows.push_back(row);
          return true;
        });
  }
};

struct Receipt
{
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<gz::msgs::Pose_V> messages;
};

int main(int argc, char **argv)
{
  if (argc != 4) return 2;
  const auto *backend = std::getenv("GZ_TRANSPORT_IMPLEMENTATION");
  if (!backend || std::string(backend) != "zenoh") return 2;
  std::ifstream input(argv[1]);
  if (!input) return 2;
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  const bool empty = std::string(argv[3]) == "empty";
  if (!empty && std::string(argv[3]) != "lights") return 2;
  const std::string output = argv[2];
  auto receipt = std::make_shared<Receipt>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Pose_V &)> receive = [receipt](const auto &message)
  {
    std::lock_guard<std::mutex> guard(receipt->mutex);
    receipt->messages.push_back(message);
    receipt->changed.notify_all();
  };
  if (!node.Subscribe("/world/default/pose/info", receive)) return 3;
  sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 4;
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  for (const auto &plugin : std::vector<std::pair<std::string, std::string>>{
      {"gz-sim-physics-system", "gz::sim::systems::Physics"},
      {"gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"}})
    config.AddPlugin(sim::ServerConfig::PluginInfo("default", "world",
        sdf::Plugin(plugin.first, plugin.second)));
  sim::Server server(config);
  auto observer = std::make_shared<Observe>();
  const auto added = server.AddSystem(observer);
  if (!added || !*added) return 5;
  if (!server.Run(true, 1, false)) return 5;
  bool seen = false;
  gz::msgs::Pose_V wire;
  {
    std::unique_lock<std::mutex> guard(receipt->mutex);
    seen = receipt->changed.wait_for(guard, std::chrono::seconds(5), [&]
    {
      for (const auto &message : receipt->messages)
        if (message.has_header() && message.header().has_stamp() &&
            message.header().stamp().sec() == 0 &&
            message.header().stamp().nsec() == 1000000) { wire = message; return true; }
      return false;
    });
  }
  // Retain real observations even when a receipt or semantic assertion fails.
  std::ofstream observations(output + "/observations.jsonl");
  observations << std::setprecision(17);
  observations << "{\"kind\":\"world\",\"id\":" << observer->world << "}\n";
  for (const auto &row : observer->rows)
  {
    observations << "{\"kind\":" << std::quoted(row.kind) << ",\"id\":" << row.id
        << ",\"name\":" << std::quoted(row.name) << ",\"parent\":" << row.parent
        << ",\"local_pose\":"; poseJson(observations, row.local);
    observations << ",\"world_pose\":"; poseJson(observations, row.world);
    observations << ",\"on\":" << row.on << ",\"visualize\":" << row.visualize
        << ",\"shadows\":" << row.shadows << ",\"transparency\":" << row.transparency
        << ",\"type\":" << row.type << "}\n";
  }
  std::ofstream all(output + "/received.txt");
  { std::lock_guard<std::mutex> guard(receipt->mutex);
    for (const auto &message : receipt->messages) all << message.DebugString() << "\n"; }
  std::ifstream maps("/proc/self/maps");
  std::ofstream mapped(output + "/maps"); mapped << maps.rdbuf();
  if (!observations || !all || !maps || !mapped) return 7;
  if (!seen) { std::cerr << "no first-step Pose_V receipt within five seconds\n"; return 6; }
  std::ofstream bytes(output + "/pose_v.pb", std::ios::binary);
  if (!wire.SerializeToOstream(&bytes)) return 7;
  std::ofstream text(output + "/pose_v.txt"); text << wire.DebugString();
  if (!bytes || !text || observer->failed || observer->calls != 1 || observer->world == 0)
    return 8;
  if (observer->rows.size() != (empty ? 0u : 24u) ||
      wire.pose_size() != static_cast<int>(observer->rows.size())) return 8;
  const gz::math::Pose3d expected = gz::math::Pose3d(4, -2, 1, .2, -.3, .4) *
      gz::math::Pose3d(1, 2, 3, -.1, .25, -.35);
  std::set<sim::Entity> ids;
  std::set<std::string> names, observedLights, expectedLights;
  for (const std::string type : {"directional", "point", "spot"})
    for (const std::string on : {"_on_", "_off_"})
      for (const std::string visibility : {"visible", "hidden"})
        expectedLights.insert(type + on + visibility);
  for (std::size_t i = 0; i < observer->rows.size(); ++i)
  {
    const auto &row = observer->rows[i];
    const auto &pose = wire.pose(static_cast<int>(i));
    const gz::math::Pose3d received(
        gz::math::Vector3d(pose.position().x(), pose.position().y(), pose.position().z()),
        gz::math::Quaterniond(pose.orientation().w(), pose.orientation().x(),
            pose.orientation().y(), pose.orientation().z()));
    if (row.id == 0 || !ids.insert(row.id).second || !names.insert(row.name).second ||
        !pose.has_position() || !pose.has_orientation() ||
        pose.id() != row.id || pose.name() != row.name || !near(received, row.local)) return 8;
    if (i < 12)
    {
      const auto lightName = row.name.substr(0, row.name.size() - 6);
      const Row *parent = nullptr;
      for (const auto &candidate : observer->rows)
        if (candidate.kind == "light" && candidate.id == row.parent) parent = &candidate;
      if (row.kind != "visual" || !parent || row.name != parent->name + "Visual" ||
          lightName != parent->name || !near(row.local, gz::math::Pose3d::Zero) ||
          row.shadows || row.transparency != 0 || !near(row.world, parent->world)) return 8;
    }
    else
    {
      observedLights.insert(row.name);
      if (row.kind != "light" || row.parent != observer->world ||
          !near(row.local, expected) || !near(row.world, expected) || row.shadows ||
          row.on != (row.name.find("_on_") != std::string::npos) ||
          row.visualize != (row.name.find("_visible") != std::string::npos)) return 8;
      const std::string prefix = row.name.substr(0, row.name.find('_'));
      const auto expectedType = prefix == "directional" ? sdf::LightType::DIRECTIONAL :
          prefix == "point" ? sdf::LightType::POINT : sdf::LightType::SPOT;
      if (row.type != static_cast<int>(expectedType)) return 8;
    }
  }
  if (observedLights != (empty ? std::set<std::string>{} : expectedLights)) return 8;
  std::cout << "native light wire ready lights=" << (empty ? 0 : 12)
      << " visuals=" << (empty ? 0 : 12) << " sim_ns=1000000\n";
}
