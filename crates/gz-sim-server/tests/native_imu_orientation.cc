// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/imu.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <chrono>
#include <cmath>
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
  if (!(condition)) throw std::runtime_error("native IMU orientation check at line " + std::to_string(__LINE__)); \
} while (false)

int main(int argc, char **argv)
{
  if (argc != 4) return 2;
  const bool orientation = std::string(argv[1]) == "enabled";
  const bool noisy = std::string(argv[2]) == "noisy";
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<gz::msgs::IMU> frames;
  gz::transport::Node node;
  std::function<void(const gz::msgs::IMU &)> callback = [&](const auto &message) {
    std::lock_guard<std::mutex> guard(mutex); frames.push_back(message); changed.notify_all();
  };
  REQUIRE(node.Subscribe("/world/default/model/ball/link/body/sensor/imu/imu", callback));
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  gz::sim::ServerConfig config;
  REQUIRE(config.SetSdfString(xml));
  config.SetPhysicsEngine("gz-physics-tpe-plugin");
  for (const auto &plugin : std::vector<std::pair<std::string, std::string>>{
      {"gz-sim-physics-system", "gz::sim::systems::Physics"},
      {"gz-sim-imu-system", "gz::sim::systems::Imu"}})
    config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world", sdf::Plugin(plugin.first, plugin.second)));
  gz::sim::Server server(config);
  REQUIRE(server.Run(true, 2, false));
  std::unique_lock<std::mutex> guard(mutex);
  REQUIRE(changed.wait_for(guard, std::chrono::seconds(5), [&] { return frames.size() >= 2; }));
  REQUIRE(frames.size() == 2);
  for (std::size_t index = 0; index < frames.size(); ++index)
  {
    const auto &frame = frames[index];
    REQUIRE(frame.has_orientation() == orientation);
    if (orientation)
      REQUIRE(frame.orientation().x() == 0 && frame.orientation().y() == 0 &&
          frame.orientation().z() == 0 && frame.orientation().w() == 1);
    REQUIRE(frame.has_header() && frame.header().has_stamp());
    REQUIRE(frame.header().stamp().sec() == 0 && frame.header().stamp().nsec() == static_cast<int64_t>(index + 1) * 1000000LL);
    REQUIRE(frame.entity_name() == "ball::body::imu");
    REQUIRE(frame.header().data_size() == 2 && frame.header().data(0).key() == "frame_id" &&
        frame.header().data(0).value_size() == 1 && frame.header().data(0).value(0) == "ball::body::imu" &&
        frame.header().data(1).key() == "seq" && frame.header().data(1).value_size() == 1 &&
        frame.header().data(1).value(0) == std::to_string(index));
    REQUIRE(frame.has_angular_velocity() && frame.has_linear_acceleration());
    REQUIRE(frame.angular_velocity().x() == 0 && frame.angular_velocity().y() == 0);
    REQUIRE(std::isfinite(frame.angular_velocity().z()));
    REQUIRE(noisy ? frame.angular_velocity().z() != 0 : frame.angular_velocity().z() == 0);
    REQUIRE(frame.linear_acceleration().x() == 0 && frame.linear_acceleration().y() == 0 &&
        std::abs(frame.linear_acceleration().z() - 9.81) < 1e-12);
    REQUIRE(frame.has_orientation_covariance() && frame.has_angular_velocity_covariance() &&
        frame.has_linear_acceleration_covariance());
    for (const auto *cov : {&frame.orientation_covariance(), &frame.angular_velocity_covariance(), &frame.linear_acceleration_covariance()})
    {
      REQUIRE(!cov->has_header() && cov->data_size() == 9);
      for (int axis = 0; axis < 9; ++axis)
        REQUIRE(cov->data(axis) == (noisy && cov == &frame.angular_velocity_covariance() && axis == 8 ? 0.01f : 0.0f));
    }
  }
  std::ofstream maps(argv[3]); maps << std::ifstream("/proc/self/maps").rdbuf(); REQUIRE(maps.good());
  std::cout << "{\"orientation\":" << (orientation ? "true" : "false")
      << ",\"noisy\":" << (noisy ? "true" : "false") << ",\"samples\":2,\"stamps_ns\":[1000000,2000000],\"sequences\":[0,1]}\n";
}
