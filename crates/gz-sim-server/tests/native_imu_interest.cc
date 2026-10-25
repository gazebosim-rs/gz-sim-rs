// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/imu.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#define REQUIRE(condition) do { \
  if (!(condition)) throw std::runtime_error("native IMU interest check at line " + std::to_string(__LINE__)); \
} while (false)

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const std::string mode = argv[1];
  if (mode != "late" && mode != "shared") return 2;
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<gz::msgs::IMU> slow, fast;
  gz::transport::Node node;
  std::function<void(const gz::msgs::IMU &)> onSlow = [&](const auto &message) {
    std::lock_guard<std::mutex> guard(mutex); slow.push_back(message); changed.notify_all();
  };
  std::function<void(const gz::msgs::IMU &)> onFast = [&](const auto &message) {
    std::lock_guard<std::mutex> guard(mutex); fast.push_back(message); changed.notify_all();
  };
  if (mode == "shared") REQUIRE(node.Subscribe("/activation/slow", onSlow));
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  gz::sim::ServerConfig config;
  REQUIRE(config.SetSdfString(xml));
  config.SetPhysicsEngine("gz-physics-tpe-plugin");
  for (const auto &plugin : std::vector<std::pair<std::string, std::string>>{
      {"gz-sim-physics-system", "gz::sim::systems::Physics"},
      {"gz-sim-imu-system", "gz::sim::systems::Imu"}})
    config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world", sdf::Plugin(plugin.first, plugin.second)));
  gz::sim::Server server(config);
  auto wait = [&](const std::vector<gz::msgs::IMU> &frames, std::size_t count) {
    std::unique_lock<std::mutex> guard(mutex);
    return changed.wait_for(guard, std::chrono::seconds(5), [&] { return frames.size() >= count; });
  };
  auto stamp = [](const gz::msgs::IMU &message) {
    return message.header().stamp().sec() * 1000000000LL + message.header().stamp().nsec();
  };
  auto sequence = [&](const gz::msgs::IMU &message) {
    for (const auto &data : message.header().data())
      if (data.key() == "seq" && data.value_size() == 1) return std::stoull(data.value(0));
    throw std::runtime_error("missing native IMU sequence");
  };
  const unsigned initial = mode == "shared" ? 100 : 50;
  REQUIRE(server.Run(true, initial, false));
  if (mode == "shared")
  {
    REQUIRE(wait(slow, 2));
    REQUIRE(slow.size() == 2 && stamp(slow[0]) == 1000000 && stamp(slow[1]) == 100000000);
    REQUIRE(sequence(slow[0]) == 0 && sequence(slow[1]) == 1);
  }
  REQUIRE(node.Subscribe("/activation/fast", onFast));
  REQUIRE(server.Run(true, 1, false) && wait(fast, 1));
  REQUIRE(fast.size() == 1 && stamp(fast[0]) == (initial + 1) * 1000000LL);
  REQUIRE(sequence(fast[0]) == (mode == "shared" ? 2u : 0u));
  if (mode == "shared") REQUIRE(node.Unsubscribe("/activation/slow"));
  REQUIRE(node.Unsubscribe("/activation/fast"));
  REQUIRE(server.Run(true, 10, false));
  REQUIRE(node.Subscribe("/activation/fast", onFast));
  REQUIRE(server.Run(true, 1, false) && wait(fast, 2));
  REQUIRE(fast.size() == 2 && stamp(fast[1]) == (initial + 12) * 1000000LL);
  REQUIRE(sequence(fast[1]) == (mode == "shared" ? 3u : 1u));
  std::ofstream maps(argv[2]); maps << std::ifstream("/proc/self/maps").rdbuf(); REQUIRE(maps.good());
  std::cout << "{\"mode\":\"" << mode << "\",\"slow_count\":" << slow.size() << ",\"fast\":[";
  for (std::size_t index = 0; index < fast.size(); ++index)
  {
    if (index) std::cout << ',';
    std::cout << "{\"stamp_ns\":" << stamp(fast[index]) << ",\"sequence\":" << sequence(fast[index]) << '}';
  }
  std::cout << "]}\n";
}
