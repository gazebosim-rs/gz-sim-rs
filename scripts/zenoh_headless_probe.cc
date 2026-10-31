// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>

namespace
{
bool firstStep(const gz::msgs::Time &stamp)
{
  return stamp.sec() == 0 && stamp.nsec() == 1000000;
}

bool valid(const gz::msgs::Clock &message)
{
  if (message.has_header() || !message.has_sim() || !message.has_real() ||
      !message.has_system() || !firstStep(message.sim())) return false;
  for (const auto *stamp : {&message.real(), &message.system()})
    if (stamp->sec() < 0 || stamp->nsec() < 0 || stamp->nsec() >= 1000000000)
      return false;
  return message.system().sec() >= 1000000000;
}

bool valid(const gz::msgs::Pose_V &message)
{
  if (!message.has_header() || !message.header().has_stamp() ||
      !firstStep(message.header().stamp()) || message.pose_size() != 4)
    return false;
  std::set<std::string> names;
  std::set<uint32_t> ids;
  for (const auto &pose : message.pose())
  {
    if (!names.insert(pose.name()).second || !ids.insert(pose.id()).second ||
        pose.id() == 0 || !pose.has_position() || !pose.has_orientation())
      return false;
    const double z = pose.name() == "ball" ? 5 : pose.name() == "base" ? -0.5 : 0;
    const auto &p = pose.position();
    const auto &q = pose.orientation();
    if (p.x() != 0 || p.y() != 0 || p.z() != z ||
        q.x() != 0 || q.y() != 0 || q.z() != 0 || q.w() != 1) return false;
  }
  return names == std::set<std::string>{"ground", "ball", "base", "body"};
}

bool valid(const gz::msgs::IMU &message)
{
  if (!message.has_header() || !message.header().has_stamp() ||
      !firstStep(message.header().stamp()) || message.entity_name() != "ball::body::imu" ||
      !message.has_orientation() || !message.has_linear_acceleration() ||
      !message.has_angular_velocity()) return false;
  const auto &q = message.orientation();
  const auto &a = message.linear_acceleration();
  const auto &w = message.angular_velocity();
  return q.x() == 0 && q.y() == 0 && q.z() == 0 && q.w() == 1 &&
      a.x() == 0 && a.y() == 0 && std::abs(a.z()-9.81) < 1e-9 &&
      w.x() == 0 && w.y() == 0 && w.z() == 0;
}

struct Observation
{
  std::mutex mutex;
  std::condition_variable changed;
  std::array<bool, 3> seen{false, false, false};
  bool failed{false};
};

template<class Message>
void receive(const std::shared_ptr<Observation> &state, std::size_t stream,
    const Message &message, const gz::msgs::Time &stamp)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  // Scene startup can publish time zero; it cannot qualify a committed step.
  if (stamp.sec() == 0 && stamp.nsec() == 0) return;
  state->failed = state->failed || !valid(message);
  state->seen[stream] = state->seen[stream] || !state->failed;
  state->changed.notify_all();
}
}

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const char *backend = std::getenv("GZ_TRANSPORT_IMPLEMENTATION");
  if (!backend || std::string(backend) != "zenoh") return 2;
  std::ifstream input(argv[1]);
  if (!input) return 2;
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  // Native dispatch may outlive unsubscribe; callbacks own their observations.
  auto observations = std::make_shared<Observation>();
  std::function<void(const gz::msgs::Clock &)> onClock = [observations](const auto &message)
      { receive(observations, 0, message, message.sim()); };
  std::function<void(const gz::msgs::Pose_V &)> onPose = [observations](const auto &message)
      { receive(observations, 1, message, message.header().stamp()); };
  std::function<void(const gz::msgs::IMU &)> onImu = [observations](const auto &message)
      { receive(observations, 2, message, message.header().stamp()); };
  gz::transport::Node node;
  if (!node.Subscribe("/world/default/clock", onClock) ||
      !node.Subscribe("/world/default/pose/info", onPose) ||
      !node.Subscribe("/p2/imu", onImu)) return 3;
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 4;
  config.SetPhysicsEngine("gz-physics-tpe-plugin");
  for (const auto &plugin : std::vector<std::pair<std::string, std::string>>{
      {"gz-sim-physics-system", "gz::sim::systems::Physics"},
      {"gz-sim-imu-system", "gz::sim::systems::Imu"},
      {"gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"}})
    config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
        sdf::Plugin(plugin.first, plugin.second)));
  gz::sim::Server server(config);
  if (!server.Run(true, 1, false)) return 5;
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    if (!observations->changed.wait_for(lock, std::chrono::seconds(5), [&]
        { return observations->failed || (observations->seen[0] &&
            observations->seen[1] && observations->seen[2]); }) || observations->failed)
      return 6;
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream output(argv[2]);
  if (!maps || !output) return 7;
  output << maps.rdbuf();
  output.close();
  if (!output) return 7;
  std::cout << "zenoh headless ready clock=1 pose=1 imu=1 sim_ns=1000000\n";
}
