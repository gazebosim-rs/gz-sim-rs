// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <gz/msgs/imu.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>

bool valid(const gz::msgs::IMU &message, uint64_t sequence)
{
  const auto &header = message.header();
  const auto &stamp = header.stamp();
  const auto &q = message.orientation();
  const auto &a = message.linear_acceleration();
  const auto &w = message.angular_velocity();
  if (stamp.sec()*1000000000LL + stamp.nsec() != static_cast<int64_t>((sequence+1)*1000000) ||
      message.entity_name() != "ball::body::imu" || header.data_size() != 2 ||
      q.x() != 0 || q.y() != 0 || q.z() != 0 || q.w() != 1 ||
      a.x() != 0 || a.y() != 0 || !(std::abs(a.z()-9.81) < 1e-9) ||
      w.x() != 0 || w.y() != 0 || w.z() != 0)
    return false;
  bool frame = false, seq = false;
  for (const auto &entry : header.data())
  {
    if (entry.value_size() != 1) return false;
    if (entry.key() == "frame_id") frame = entry.value(0) == "ball::body::imu";
    if (entry.key() == "seq") seq = entry.value(0) == std::to_string(sequence);
  }
  for (const auto *covariance : {&message.orientation_covariance(),
       &message.angular_velocity_covariance(), &message.linear_acceleration_covariance()})
  {
    if (covariance->data_size() != 9 || !std::all_of(covariance->data().begin(),
        covariance->data().end(), [](float value){return value == 0;})) return false;
  }
  return frame && seq;
}

class MotionCheck final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  public: gz::sim::Entity body{gz::sim::kNullEntity};
  public: bool failed{false};
  public: double finalX{0};
  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (info.iterations == 1)
    {
      body = ecm.EntityByComponents(gz::sim::components::Link(),
          gz::sim::components::Name("body"));
      gz::sim::Link(body).EnableVelocityChecks(ecm);
      ecm.CreateComponent(body, gz::sim::components::WorldLinearVelocityReset({1, 0, 0}));
    }
  }
  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    const auto pose = gz::sim::Link(body).WorldPose(ecm);
    if (!pose) { failed = true; return; }
    finalX = pose->Pos().X();
    failed = failed || !(std::abs(finalX-info.iterations*0.001) < 1e-9) ||
        pose->Pos().Y() != 0 || pose->Pos().Z() != 5;
  }
};

int main(int argc, char **argv)
{
  if (argc != 4) return 2;
  const unsigned steps = std::stoul(argv[3]);
  if (steps == 0 || steps > 1000) return 2;
  std::ifstream input(argv[1]);
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  std::mutex mutex;
  std::condition_variable changed;
  uint64_t count = 0, timeSum = 0, logicalBytes = 0;
  double forceSum = 0, warmupForceX = 0;
  bool failed = false;
  gz::transport::Node node;
  std::function<void(const gz::msgs::IMU &)> callback = [&](const gz::msgs::IMU &message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (!valid(message, count))
    {
      if (!failed) std::cerr << "invalid sample " << count << "\n" << message.DebugString();
      failed = true;
    }
    if (count == 0) warmupForceX = message.linear_acceleration().x();
    if (count > 0)
    {
      const auto &stamp = message.header().stamp();
      timeSum += stamp.sec()*1000000000LL + stamp.nsec();
      forceSum += message.linear_acceleration().z();
      logicalBytes += message.ByteSizeLong();
    }
    ++count;
    changed.notify_all();
  };
  if (!node.Subscribe("/p2/imu", callback)) return 3;
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 4;
  config.SetPhysicsEngine("gz-physics-tpe-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-imu-system", "gz::sim::systems::Imu")));
  gz::sim::Server server(config);
  auto motion = std::make_shared<MotionCheck>();
  if (server.AddSystem(motion) != std::optional<bool>(true)) return 4;
  auto wait = [&](uint64_t target)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, std::chrono::seconds(5),
        [&]{return failed || count >= target;}) && !failed && count == target;
  };
  // Actual receipt proves both sensor initialization and local subscription readiness.
  if (!server.Run(true, 1, false) || !wait(1)) return 5;
  const auto start = std::chrono::steady_clock::now();
  if (!server.Run(true, steps, false) || !wait(steps+1) || motion->failed) return 6;
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-start).count();
  std::ifstream maps("/proc/self/maps");
  std::ofstream output(argv[2]);
  output << maps.rdbuf();
  if (!output) return 7;
  std::string fixtureHex;
  const char *digits = "0123456789abcdef";
  for (unsigned char byte : xml) { fixtureHex += digits[byte >> 4]; fixtureHex += digits[byte & 15]; }
  std::lock_guard<std::mutex> guard(mutex);
  std::cout << std::setprecision(17) << "PIPELINE {\"steps\":" << steps
            << ",\"nanos\":" << nanos << ",\"logical_bytes\":" << logicalBytes
            << ",\"time_sum\":" << timeSum << ",\"force_sum\":" << forceSum
            << ",\"final_x\":" << motion->finalX << ",\"warmup_force_x\":" << warmupForceX << ",\"fixture_hex\":\"" << fixtureHex << "\"}\n";
}
