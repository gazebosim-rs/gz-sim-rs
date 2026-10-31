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
#include <map>
#include <thread>
#include <cstdlib>
#include <vector>
#include <utility>
#include <memory>
#include <set>
#include <google/protobuf/io/coded_stream.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/pose_v.pb.h>
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

bool validPose(const gz::msgs::Pose_V &message, uint64_t sequence,
    std::map<std::string, uint32_t> &identities)
{
  const auto expected = (sequence+1)*1000000;
  if (message.header().stamp().nsec() < 0 || message.header().stamp().nsec() >= 1000000000 || message.header().stamp().sec() < 0) return false;
  if (!message.has_header() || !message.header().has_stamp() ||
      message.header().data_size() != 0 || message.pose_size() != 4 ||
      message.header().stamp().sec() != static_cast<int64_t>(expected/1000000000) ||
      message.header().stamp().nsec() != static_cast<int32_t>(expected%1000000000)) return false;
  std::set<std::string> names;
  std::set<uint32_t> ids;
  for (int index = 0; index < message.pose_size(); ++index)
  {
    const auto &pose = message.pose(index);
    const auto &name = pose.name();
    if (!names.insert(name).second || !ids.insert(pose.id()).second || pose.id() == 0 ||
        pose.has_header() || !pose.has_position() || !pose.has_orientation() ||
        pose.position().has_header() || pose.orientation().has_header() ||
        pose.ByteSizeLong() >= 128) return false;
    const auto inserted = identities.emplace(name, pose.id());
    if (inserted.first->second != pose.id()) return false;
    const bool model = name == "ball" || name == "ground";
    if ((index < 2) != model) return false;
    if (name != "ball" && name != "ground" && name != "base" && name != "body") return false;
    const double x = name == "ball" ? (sequence+1)*0.001 : 0;
    const double z = name == "ball" ? 5 : name == "base" ? -0.5 : 0;
    if (!(std::abs(pose.position().x()-x) < 1e-9) ||
        pose.position().y() != 0 || pose.position().z() != z ||
        pose.orientation().x() != 0 || pose.orientation().y() != 0 ||
        pose.orientation().z() != 0 || pose.orientation().w() != 1) return false;
  }
  return names == std::set<std::string>{"ground", "ball", "base", "body"};
}

bool validClock(const gz::msgs::Clock &message, uint64_t sequence,
    std::pair<int64_t, int32_t> &lastReal)
{
  const auto expected = (sequence+1)*1000000;
  if (message.has_header() || !message.has_sim() || !message.has_real() ||
      !message.has_system()) return false;
  for (const auto *stamp : {&message.sim(), &message.real(), &message.system()})
    if (stamp->sec() < 0 || stamp->nsec() < 0 || stamp->nsec() >= 1000000000) return false;
  if (message.sim().sec() != static_cast<int64_t>(expected/1000000000) ||
      message.sim().nsec() != static_cast<int32_t>(expected%1000000000) ||
      message.system().sec() < 1000000000) return false;
  const auto observed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  if (message.system().sec() < observed-1 || message.system().sec() > observed+1) return false;
  const auto real = std::make_pair(message.real().sec(), message.real().nsec());
  if (real < lastReal) return false;
  lastReal = real;
  return true;
}

int main(int argc, char **argv)
{
  if (argc != 4) return 2;
  const unsigned steps = std::stoul(argv[3]);
  if (steps == 0 || steps > 1000) return 2;
  std::ifstream input(argv[1]);
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  std::mutex mutex;
  std::condition_variable changed;
  uint64_t count = 0, clocks = 0, poses = 0, timeSum = 0, clockTimeSum = 0, poseTimeSum = 0;
  uint64_t logicalBytes = 0, clockBytes = 0, poseBytes = 0, normalizedPoseBytes = 0;
  double forceSum = 0, warmupForceX = 0;
  bool failed = false;
  std::map<std::string, uint32_t> identities;
  std::pair<int64_t, int32_t> lastReal{0, 0};
  gz::transport::Node node;
  std::function<void(const gz::msgs::IMU &)> imu = [&](const auto &message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    failed = failed || !valid(message, count) || !message.has_angular_velocity() ||
        !message.has_linear_acceleration() || !message.has_orientation() ||
        message.orientation().has_header() || message.angular_velocity().has_header() ||
        message.linear_acceleration().has_header() || message.header().stamp().sec() < 0 ||
        message.header().stamp().nsec() < 0 || message.header().stamp().nsec() >= 1000000000;
    for (const auto *covariance : {&message.orientation_covariance(),
        &message.angular_velocity_covariance(), &message.linear_acceleration_covariance()})
      failed = failed || covariance->has_header();
    if (failed) { std::cerr << "IMU sequence " << count << "\n" << message.DebugString(); changed.notify_all(); return; }
    if (count == 0) warmupForceX = message.linear_acceleration().x();
    else
    {
      timeSum += message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
      forceSum += message.linear_acceleration().z();
      logicalBytes += message.ByteSizeLong();
    }
    ++count;
    changed.notify_all();
  };
  std::function<void(const gz::msgs::Clock &)> clock = [&](const auto &message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    failed = failed || !validClock(message, clocks, lastReal);
    if (failed) { std::cerr << "Clock sequence " << clocks << " last real " << lastReal.first << ":" << lastReal.second << "\n" << message.DebugString(); changed.notify_all(); return; }
    if (clocks > 0)
    {
      clockTimeSum += message.sim().sec()*1000000000LL + message.sim().nsec();
      clockBytes += message.ByteSizeLong();
    }
    ++clocks;
    changed.notify_all();
  };
  std::function<void(const gz::msgs::Pose_V &)> pose = [&](const auto &message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    failed = failed || !validPose(message, poses, identities);
    if (failed) { std::cerr << "Pose sequence " << poses << "\n" << message.DebugString(); changed.notify_all(); return; }
    if (poses > 0)
    {
      poseTimeSum += message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
      poseBytes += message.ByteSizeLong();
      uint64_t idBytes = 0;
      for (const auto &item : message.pose())
        idBytes += google::protobuf::io::CodedOutputStream::VarintSize32(item.id());
      normalizedPoseBytes += message.ByteSizeLong()-idBytes;
    }
    ++poses;
    changed.notify_all();
  };
  if (!node.Subscribe("/p2/imu", imu) || !node.Subscribe("/world/default/clock", clock) ||
      !node.Subscribe("/world/default/pose/info", pose)) return 3;
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
  auto motion = std::make_shared<MotionCheck>();
  if (server.AddSystem(motion) != std::optional<bool>(true)) return 4;
  auto wait = [&](uint64_t target)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, std::chrono::seconds(5), [&]
        {return failed || (count >= target && clocks >= target && poses >= target);}) &&
        !failed && count == target && clocks == target && poses == target;
  };
  if (!server.Run(true, 1, false) || !wait(1)) return 5;
  const char *periodValue = std::getenv("GZ_STREAM_PERIOD_US");
  const uint64_t periodUs = periodValue ? std::stoull(periodValue) : 20000;
  if (periodUs < 20000 || periodUs > 1000000) return 2;
  const auto period = std::chrono::microseconds(periodUs);
  const auto wallStart = std::chrono::steady_clock::now();
  auto nextRequest = wallStart+period;
  uint64_t nanos = 0, pacingWait = 0;
  for (unsigned sequence = 1; sequence <= steps; ++sequence)
  {
    const auto waiting = std::chrono::steady_clock::now();
    std::this_thread::sleep_until(nextRequest);
    const auto start = std::chrono::steady_clock::now();
    pacingWait += std::chrono::duration_cast<std::chrono::nanoseconds>(start-waiting).count();
    if (!server.Run(true, 1, false) || !wait(sequence+1) || motion->failed)
    {
      std::cerr << "Incomplete step " << sequence << " counts=" << count << "," << clocks << "," << poses << " motion=" << motion->failed << "\n";
      return 6;
    }
    const auto end = std::chrono::steady_clock::now();
    nanos += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
    nextRequest = end+period;
  }
  const auto wallNanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-wallStart).count();
  std::ifstream maps("/proc/self/maps");
  std::ofstream output(argv[2]);
  output << maps.rdbuf();
  if (!output) return 7;
  std::string fixtureHex;
  const char *digits = "0123456789abcdef";
  for (unsigned char byte : xml) { fixtureHex += digits[byte >> 4]; fixtureHex += digits[byte & 15]; }
  std::lock_guard<std::mutex> guard(mutex);
  std::cout << std::setprecision(17) << "PIPELINE {\"steps\":" << steps
      << ",\"nanos\":" << nanos << ",\"wall_nanos\":" << wallNanos
      << ",\"pacing_wait_nanos\":" << pacingWait << ",\"request_period_us\":" << periodUs << ",\"logical_bytes\":" << logicalBytes
      << ",\"time_sum\":" << timeSum << ",\"force_sum\":" << forceSum
      << ",\"final_x\":" << motion->finalX << ",\"warmup_force_x\":" << warmupForceX
      << ",\"clock_count\":" << clocks-1 << ",\"pose_count\":" << poses-1
      << ",\"clock_time_sum\":" << clockTimeSum << ",\"pose_time_sum\":" << poseTimeSum
      << ",\"clock_bytes\":" << clockBytes << ",\"pose_bytes\":" << poseBytes
      << ",\"pose_bytes_without_id_values\":" << normalizedPoseBytes
      << ",\"fixture_hex\":\"" << fixtureHex << "\",\"pose_ids\":{";
  bool comma = false;
  for (const auto &entry : identities)
  {
    if (comma) std::cout << ',';
    comma = true;
    std::cout << '"' << entry.first << "\":" << entry.second;
  }
  std::cout << "}}\n";
}
