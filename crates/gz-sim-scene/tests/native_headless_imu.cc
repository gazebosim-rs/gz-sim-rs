// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <sdf/Plugin.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
int main(int argc, char **argv)
{
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<gz::msgs::IMU> samples;
  gz::transport::Node node;
  if (!node.Subscribe("/p2/imu", std::function<void(const gz::msgs::IMU &)>(
      [&](const gz::msgs::IMU &message)
      {
        std::lock_guard<std::mutex> guard(mutex);
        samples.push_back(message);
        ready.notify_all();
      }))) return 2;
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 3;
  if (argc != 3) return 2;
  const std::string engine = argv[2];
  if (engine != "tpe" && engine != "dartsim") return 2;
  config.SetPhysicsEngine("gz-physics-" + engine + "-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-imu-system", "gz::sim::systems::Imu")));
  gz::sim::Server server(config);
  // Native finite Run counts advancing iterations, not paused polls.
  if (!server.Run(false, 100, true)) return 4;
  // Subscribe registered a process-local callback before Server construction.
  // Native Publisher::HasConnections checks that same local subscriber registry.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::vector<gz::transport::MessagePublisher> publishers, subscribers;
    node.TopicInfo("/p2/imu", publishers, subscribers);
    if (publishers.size() == 1 && publishers[0].MsgTypeName() == "gz.msgs.IMU")
    {
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!found) return 5;
  // Queue playback between simulation iterations. Direct SetPaused can change
  // the current PostUpdate's pause flag before its zero-time sensor pass finishes.
  gz::msgs::WorldControl request;
  gz::msgs::Boolean response;
  bool accepted = false;
  request.set_pause(false);
  if (!node.Request("/world/default/control", request, 5000, response, accepted)
      || !accepted || !response.data()) return 5;
  {
    std::unique_lock<std::mutex> guard(mutex);
    if (!ready.wait_for(guard, std::chrono::seconds(5), [&]{return samples.size() >= 100;}))
    {
      server.Stop();
      std::cerr << "received " << samples.size() << " samples\n";
      return 6;
    }
  }
  server.Stop();
  std::lock_guard<std::mutex> guard(mutex);
  if (samples.size() != 100) return 7;
  std::ifstream maps("/proc/self/maps");
  std::ofstream output(argv[1]);
  output << maps.rdbuf();
  if (!output) return 8;
  std::cout << std::setprecision(17);
  for (const auto &message : samples)
  {
    const auto &stamp = message.header().stamp();
    const auto &a = message.linear_acceleration();
    const auto &w = message.angular_velocity();
    const auto &q = message.orientation();
    std::cout << "{\"time_ns\":" << stamp.sec()*1000000000LL+stamp.nsec()
              << ",\"specific_force\":[" << a.x() << "," << a.y() << "," << a.z()
              << "],\"angular_velocity\":[" << w.x() << "," << w.y() << "," << w.z()
              << "],\"orientation\":[" << q.x() << "," << q.y() << "," << q.z() << "," << q.w()
              << "]}\n";
  }
}
