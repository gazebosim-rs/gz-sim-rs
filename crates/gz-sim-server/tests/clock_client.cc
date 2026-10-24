// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <condition_variable>
#include <charconv>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

struct Observations
{
  std::mutex mutex;
  std::condition_variable changed;
  bool initial = false;
  bool final = false;
  bool invalid = false;
  int64_t lastClock = 0;
  int64_t lastImu = 0;
  int64_t target = 3000000;
};

int main(int argc, char **)
{
  const bool withImu = argc > 1;
  auto observations = std::make_shared<Observations>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Clock &)> callback =
    [observations](const auto &clock)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      if (!clock.has_sim() || !clock.has_real() || !clock.has_system() ||
          clock.sim().nsec() < 0 || clock.sim().nsec() >= 1000000000 ||
          clock.real().nsec() < 0 || clock.real().nsec() >= 1000000000 ||
          clock.system().sec() < 1000000000 || clock.sim().sec() != 0)
        observations->invalid = true;
      if (clock.sim().sec() == 0 && clock.sim().nsec() == 0)
        observations->initial = true;
      observations->lastClock = clock.sim().nsec();
      if (clock.sim().sec() == 0 && clock.sim().nsec() == observations->target)
        observations->final = true;
      observations->changed.notify_all();
    };
  if (!node.Subscribe("/world/default/clock", callback)) return 2;
  std::function<void(const gz::msgs::IMU &)> imuCallback =
    [observations](const auto &imu)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      bool valid = imu.entity_name() == "ball::body::imu" && imu.has_header() &&
        imu.header().has_stamp() && imu.header().stamp().sec() == 0 &&
        imu.header().stamp().nsec() > 0 && imu.has_orientation() &&
        imu.has_angular_velocity() && imu.has_linear_acceleration() &&
        imu.has_orientation_covariance() && imu.has_angular_velocity_covariance() &&
        imu.has_linear_acceleration_covariance();
      const auto &header = imu.header();
      valid = valid && header.data_size() == 2 &&
        header.data(0).key() == "frame_id" && header.data(0).value_size() == 1 &&
        header.data(0).value(0) == "ball::body::imu" &&
        header.data(1).key() == "seq" && header.data(1).value_size() == 1;
      if (valid)
      {
        const auto &text = header.data(1).value(0);
        uint64_t sequence = 0;
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), sequence);
        valid = parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() &&
          sequence + 1 == static_cast<uint64_t>(header.stamp().nsec() / 1000000);
      }
      for (const auto *covariance : {&imu.orientation_covariance(),
          &imu.angular_velocity_covariance(), &imu.linear_acceleration_covariance()})
      {
        valid = valid && !covariance->has_header() && covariance->data_size() == 9;
        for (float value : covariance->data()) valid = valid && value == 0;
      }
      for (double value : {imu.orientation().x(), imu.orientation().y(),
          imu.orientation().z(), imu.orientation().w() - 1,
          imu.angular_velocity().x(), imu.angular_velocity().y(), imu.angular_velocity().z(),
          imu.linear_acceleration().x(), imu.linear_acceleration().y(), imu.linear_acceleration().z()})
        valid = valid && std::isfinite(value) && std::abs(value) < 1e-9;
      observations->invalid = observations->invalid || !valid;
      observations->lastImu = header.stamp().nsec();
      observations->changed.notify_all();
    };
  if (withImu && !node.Subscribe("/world/default/model/ball/link/body/sensor/imu/imu", imuCallback)) return 7;
  auto waitFor = [&](bool Observations::*field)
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    return observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
      { return observations.get()->*field || observations->invalid; }) &&
      !observations->invalid;
  };
  // A received packet proves the subscription works; registration alone does not.
  if (!waitFor(&Observations::initial)) return 3;
  gz::msgs::WorldControl request;
  gz::msgs::Boolean response;
  bool result = false;
  unsigned int warmup = 0;
  if (withImu)
  {
    // Establish sensor readiness using real sampled steps, never fabricated packets.
    for (; warmup < 100;)
    {
      ++warmup;
      request.set_pause(true);
      request.set_step(true);
      if (!node.Request("/world/default/control", request, 7000, response, result) ||
          !result || !response.data()) return 8;
      std::unique_lock<std::mutex> lock(observations->mutex);
      const int64_t stamp = warmup * 1000000;
      if (!observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
          { return observations->lastClock >= stamp || observations->invalid; }) ||
          observations->invalid) return 9;
      if (observations->changed.wait_for(lock, std::chrono::milliseconds(50), [&]
          { return observations->lastImu >= stamp || observations->invalid; })) break;
    }
    std::lock_guard<std::mutex> lock(observations->mutex);
    if (observations->lastImu == 0 || observations->invalid) return 10;
    observations->target = (warmup + 3) * 1000000;
    observations->final = false;
  }
  request.set_pause(true);
  request.set_step(true);
  request.set_multi_step(3);
  if (!node.Request("/world/default/control", request, 7000, response, result) ||
      !result || !response.data()) return 4;
  if (!waitFor(&Observations::final)) return 5;
  if (withImu)
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    if (!observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
        { return observations->lastImu >= observations->target || observations->invalid; }) ||
        observations->invalid) return 11;
  }
  request.Clear();
  request.set_seed(42);
  if (!node.Request("/world/default/control", request, 7000, response, result) ||
      !result || response.data()) return 6;
  std::cout << "PASS native clock/control: baseline=" << warmup
    << " final=" << warmup + 3 << " imu=" << withImu << "\n";
}
