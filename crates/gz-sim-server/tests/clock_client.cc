// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <condition_variable>
#include <charconv>
#include <cmath>
#include <iostream>
#include <memory>
#include <map>
#include <set>
#include <mutex>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

struct Observations
{
  std::mutex mutex;
  std::condition_variable changed;
  bool initial = false;
  bool final = false;
  bool initialPose = false;
  bool allowRemoved = false;
  bool removed = false;
  int64_t lastPose = 0;
  std::map<std::string, uint32_t> poseIds;
  bool invalid = false;
  int64_t lastClock = 0;
  int64_t lastImu = 0;
  std::map<std::string, int64_t> imuTimes;
  int64_t target = 3000000;
};

int main(int argc, char **argv)
{
  const bool sharedImu = argc > 1 && std::string(argv[1]) == "--shared";
  const bool withImu = sharedImu || (argc > 1 && std::string(argv[1]) == "--imu");
  const bool remove = argc > 1 && std::string(argv[1]) == "--remove";
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
  std::function<void(const gz::msgs::Pose_V &)> poseCallback =
    [observations](const auto &poses)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      bool valid = poses.has_header() && poses.header().has_stamp() &&
        poses.header().data_size() == 0 && poses.header().stamp().sec() == 0 &&
        poses.header().stamp().nsec() >= 0 && (poses.pose_size() == 4 ||
          (observations->allowRemoved && poses.pose_size() == 2));
      std::set<uint32_t> ids;
      std::set<std::string> names;
      for (const auto &pose : poses.pose())
      {
        valid = valid && !pose.has_header() && pose.id() != 0 &&
          ids.insert(pose.id()).second && names.insert(pose.name()).second &&
          pose.has_position() && pose.has_orientation() &&
          !pose.position().has_header() && !pose.orientation().has_header();
        const auto inserted = observations->poseIds.emplace(pose.name(), pose.id());
        valid = valid && inserted.first->second == pose.id();
        for (double value : {pose.position().x(), pose.position().y(),
            pose.orientation().x(), pose.orientation().y(), pose.orientation().z(),
            pose.orientation().w() - 1})
          valid = valid && std::isfinite(value) && std::abs(value) < 1e-12;
        const double z = pose.position().z();
        valid = valid && std::isfinite(z);
        if (pose.name() == "ball") valid = valid && z <= 5 && z > 4;
        else if (pose.name() == "base") valid = valid && z == -0.5;
        else if (pose.name() == "ground" || pose.name() == "body")
          valid = valid && std::abs(z) < 1e-12;
        else valid = false;
      }
      const auto expected = poses.pose_size() == 2 ?
        std::set<std::string>{"ground", "base"} :
        std::set<std::string>{"ground", "ball", "base", "body"};
      valid = valid && names == expected;
      if (valid && poses.pose_size() == 2) observations->removed = true;
      observations->invalid = observations->invalid || !valid;
      observations->lastPose = poses.header().stamp().nsec();
      if (observations->lastPose == 0) observations->initialPose = true;
      observations->changed.notify_all();
    };
  if (!node.Subscribe("/world/default/pose/info", poseCallback)) return 12;
  std::function<void(const gz::msgs::IMU &)> imuCallback =
    [observations, sharedImu, frame = std::string(argc > 3 ? argv[3] : "ball::body::imu")](const auto &imu)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      const bool second = sharedImu && imu.entity_name() == "ball::body::imu2";
      const auto expectedFrame = sharedImu ? (second ? "second" : "first") : frame;
      bool valid = (imu.entity_name() == "ball::body::imu" || second) && imu.has_header() &&
        imu.header().has_stamp() && imu.header().stamp().sec() == 0 &&
        imu.header().stamp().nsec() > 0 && imu.has_orientation() &&
        imu.has_angular_velocity() && imu.has_linear_acceleration() &&
        imu.has_orientation_covariance() && imu.has_angular_velocity_covariance() &&
        imu.has_linear_acceleration_covariance();
      const auto &header = imu.header();
      valid = valid && header.data_size() == 2 &&
        header.data(0).key() == "frame_id" && header.data(0).value_size() == 1 &&
        header.data(0).value(0) == expectedFrame &&
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
      observations->imuTimes[imu.entity_name()] = header.stamp().nsec();
      observations->changed.notify_all();
    };
  if (withImu && !node.Subscribe(sharedImu ? "/robot/shared" : argc > 2 ? gz::transport::TopicUtils::AsValidTopic(argv[2]) :
      "/world/default/model/ball/link/body/sensor/imu/imu", imuCallback)) return 7;
  auto waitFor = [&](bool Observations::*field)
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    return observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
      { return observations.get()->*field || observations->invalid; }) &&
      !observations->invalid;
  };
  // A received packet proves the subscription works; registration alone does not.
  if (!waitFor(&Observations::initial) || !waitFor(&Observations::initialPose)) return 3;
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
          { return observations->invalid || (observations->lastImu >= stamp &&
              (!sharedImu || (observations->imuTimes.size() == 2 &&
                observations->imuTimes.at("ball::body::imu") >= stamp &&
                observations->imuTimes.at("ball::body::imu2") >= stamp))); })) break;
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
        { return observations->invalid || (observations->lastImu >= observations->target &&
            (!sharedImu || (observations->imuTimes.size() == 2 &&
              observations->imuTimes.at("ball::body::imu") >= observations->target &&
              observations->imuTimes.at("ball::body::imu2") >= observations->target))); }) ||
        observations->invalid) return 11;
  }
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    if (!observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
        { return observations->lastPose >= observations->target || observations->invalid; }) ||
        observations->invalid) return 13;
  }
  request.Clear();
  request.set_seed(42);
  if (!node.Request("/world/default/control", request, 7000, response, result) ||
      !result || response.data()) return 6;
  if (remove)
  {
    gz::msgs::Entity entity;
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      entity.set_id(observations->poseIds.at("body"));
    }
    if (!node.Request("/world/default/remove", entity, 7000, response, result) ||
        !result || !response.data()) return 14;
    if (!node.Request("/world/default/remove/blocking", entity, 7000, response, result) ||
        !result || response.data()) return 15;
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      observations->allowRemoved = true;
      entity.set_id(observations->poseIds.at("ball"));
    }
    // A nonzero ID takes precedence over a contradictory name and type.
    entity.set_name("ground");
    entity.set_type(gz::msgs::Entity::LINK);
    if (!node.Request("/world/default/remove/blocking", entity, 7000, response, result) ||
        !result || !response.data()) return 16;
    if (!waitFor(&Observations::removed)) return 17;
  }
  std::cout << "PASS native clock/control: baseline=" << warmup
    << " final=" << warmup + 3 << " imu=" << withImu << " poses=4 removed=" << remove
    << " shared=" << sharedImu << '\n';
}
