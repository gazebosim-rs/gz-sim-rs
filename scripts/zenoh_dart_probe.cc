// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Untimed REF-01 prerequisite. The complete upstream B03 world is not edited.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <exception>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <gz/msgs/clock.pb.h>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>

class Trace final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
 public:
  std::vector<gz::sim::Entity> joints;
  std::vector<gz::sim::Entity> links;
  std::array<double, 4> first{};
  std::array<double, 4> last{};
  std::uint64_t iterations{0};
  bool failed{false};

  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (info.iterations != 1) return;
    ecm.Each<gz::sim::components::Joint>([&](auto id, const auto *)
        { joints.push_back(id); return true; });
    ecm.Each<gz::sim::components::Link>([&](auto id, const auto *)
        { links.push_back(id); return true; });
    unsigned lights = 0;
    ecm.Each<gz::sim::components::Light, gz::sim::components::Name>(
        [&](auto, const auto *, const auto *name)
        { ++lights; failed = failed || name->Data() != "sun"; return true; });
    failed = failed || joints.size() != 4 || links.size() != 7 || lights != 1;
    std::sort(joints.begin(), joints.end());
    std::set<std::string> names;
    for (auto id : joints)
    {
      names.insert(gz::sim::scopedName(id, ecm, "::", false));
      gz::sim::Joint joint(id);
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
    }
    failed = failed || names != std::set<std::string>{
      "default::double_pendulum_with_base0::upper_joint",
      "default::double_pendulum_with_base0::lower_joint",
      "default::double_pendulum_with_base1::upper_joint",
      "default::double_pendulum_with_base1::lower_joint"};
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    ++iterations;
    failed = failed || info.paused || info.iterations != iterations ||
      info.dt != std::chrono::milliseconds(1) ||
      info.simTime != std::chrono::milliseconds(iterations);
    for (auto id : links)
    {
      const auto pose = gz::sim::Link(id).WorldPose(ecm);
      failed = failed || !pose || !pose->IsFinite();
    }
    for (std::size_t i = 0; i < joints.size() && i < 4; ++i)
    {
      const auto q = gz::sim::Joint(joints[i]).Position(ecm);
      const auto dq = gz::sim::Joint(joints[i]).Velocity(ecm);
      if (!q || !dq || q->size() != 1 || dq->size() != 1)
      {
        // Match the existing native articulation probe's acquisition warmup.
        failed = failed || iterations > 100;
        continue;
      }
      failed = failed || !std::isfinite(q->at(0)) || !std::isfinite(dq->at(0));
      if (iterations == 101) first[i] = q->at(0);
      last[i] = q->at(0);
    }
  }
};

struct ClockState
{
  std::mutex mutex;
  std::condition_variable changed;
  bool final{false};
  bool failed{false};
};

int main(int argc, char **argv)
try
{
  const char *transport = std::getenv("GZ_TRANSPORT_IMPLEMENTATION");
  if (argc != 3 || !transport || std::string(transport) != "zenoh") return 2;
  std::ifstream input(argv[1]);
  if (!input) return 3;
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  // Callback ownership can outlive native unsubscribe.
  auto clock = std::make_shared<ClockState>();
  std::function<void(const gz::msgs::Clock &)> receive = [clock](const auto &value)
  {
    std::lock_guard<std::mutex> lock(clock->mutex);
    clock->failed = clock->failed || !value.has_sim() ||
      value.sim().sec() != 0 || value.sim().nsec() < 0 ||
      value.sim().nsec() > 200000000 || value.sim().nsec() % 1000000 != 0;
    clock->final = clock->final || value.sim().nsec() == 200000000;
    clock->changed.notify_all();
  };
  gz::transport::Node node;
  if (!node.Subscribe("/world/default/clock", receive)) return 4;
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 5;
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  gz::sim::Server server(config);
  auto trace = std::make_shared<Trace>();
  if (server.AddSystem(trace) != std::optional<bool>(true)) return 6;
  if (!server.Run(true, 100, false) || trace->failed || trace->iterations != 100) return 7;
  if (!server.Run(true, 100, false) || trace->failed || trace->iterations != 200) return 7;
  double motion = 0;
  for (std::size_t i = 0; i < 4; ++i)
    motion = std::max(motion, std::abs(trace->last[i] - trace->first[i]));
  if (!(motion > 1e-6)) return 8;
  {
    std::unique_lock<std::mutex> lock(clock->mutex);
    if (!clock->changed.wait_for(lock, std::chrono::seconds(5), [&]
        { return clock->final || clock->failed; }) || clock->failed) return 9;
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream recorded(argv[2]);
  if (!maps || !recorded) return 10;
  recorded << maps.rdbuf();
  recorded.close();
  if (!recorded) return 10;
  std::cout << "zenoh dart ready warmup=100 steps=100 links=7 joints=4 light=1 moving=1\n";
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 11;
}
