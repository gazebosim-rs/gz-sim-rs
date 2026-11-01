// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <condition_variable>
#include <fstream>
#include <algorithm>
#include <functional>
#include <future>
#include <utility>
#include <charconv>
#include <cmath>
#include <iostream>
#include <memory>
#include <map>
#include <set>
#include <vector>
#include <mutex>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

// Native blocking Request races its waiter-side removal against response cleanup.
// Async Request leaves removal to the response thread; shared state survives timeout.
template<class Request, class Reply>
bool request_with_reply(gz::transport::Node &node, const std::string &topic,
    const Request &request, unsigned int timeout, Reply &response, bool &result)
{
  using Outcome = std::pair<Reply, bool>;
  auto completion = std::make_shared<std::promise<Outcome>>();
  auto pending = completion->get_future();
  std::function<void(const Reply &, bool)> callback =
      [completion](const Reply &reply, bool accepted)
      {
        completion->set_value(Outcome{reply, accepted});
      };
  if (!node.Request(topic, request, callback) ||
      pending.wait_for(std::chrono::milliseconds(timeout)) != std::future_status::ready)
    return false;
  auto outcome = pending.get();
  response = std::move(outcome.first);
  result = outcome.second;
  return true;
}

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

struct SamplingObservations
{
  std::mutex mutex;
  std::condition_variable changed;
  int64_t clock = -1;
  int64_t imu = -1;
  bool collecting = false;
  bool overflow = false;
  std::vector<gz::msgs::IMU> frames;
};

int RunSamplingCase(bool tpe, bool noisy = false, bool orientation = true)
{
  auto observed = std::make_shared<SamplingObservations>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Clock &)> clockCallback = [observed](const auto &value)
      {
        std::lock_guard<std::mutex> lock(observed->mutex);
        observed->clock = value.sim().sec() * 1000000000LL + value.sim().nsec();
        observed->changed.notify_all();
      };
  if (!node.Subscribe("/world/default/clock", clockCallback)) return 31;
  std::function<void(const gz::msgs::IMU &)> imuCallback = [observed](const auto &value)
      {
        std::lock_guard<std::mutex> lock(observed->mutex);
        observed->imu = value.header().stamp().sec() * 1000000000LL + value.header().stamp().nsec();
        if (observed->collecting)
        {
          if (observed->frames.size() == 256) observed->overflow = true;
          else observed->frames.push_back(value);
        }
        observed->changed.notify_all();
      };
  if (!node.Subscribe("/world/default/model/ball/link/body/sensor/imu/imu", imuCallback)) return 32;
  auto wait = [&](auto condition, std::chrono::milliseconds timeout = std::chrono::seconds(8))
  {
    std::unique_lock<std::mutex> lock(observed->mutex);
    return observed->changed.wait_for(lock, timeout, [&] { return condition(*observed); });
  };
  auto control = [&](uint64_t steps, bool reset)
  {
    gz::msgs::WorldControl request;
    request.set_pause(true);
    if (reset) request.mutable_reset()->set_all(true);
    else request.set_multi_step(steps);
    gz::msgs::Boolean response;
    bool result = false;
    return request_with_reply(node, "/world/default/control", request, 7000, response, result) && result && response.data();
  };
  if (!wait([](const auto &v) { return v.clock >= 0; })) return 33;
  unsigned int warmup = 0;
  bool ready = false;
  for (; warmup < 20 && !ready;)
  {
    ++warmup;
    const int64_t target = warmup * 100000000LL;
    if (!control(100, false) || !wait([&](const auto &v) { return v.clock == target; })) return 34;
    ready = wait([&](const auto &v) { return v.imu == target; }, std::chrono::milliseconds(50));
  }
  if (!ready) return 35;
  auto reset = [&]
  {
    {
      std::lock_guard<std::mutex> lock(observed->mutex);
      observed->collecting = false;
      observed->frames.clear();
      observed->overflow = false;
      observed->imu = -1;
    }
    if (!control(0, true) || !wait([](const auto &v) { return v.clock == 0; })) return false;
    std::lock_guard<std::mutex> lock(observed->mutex);
    observed->collecting = true;
    return true;
  };
  auto step = [&](uint64_t steps, int64_t target)
  {
    return control(steps, false) && wait([&](const auto &v) { return v.clock == target; });
  };
  if (!reset() || !step(1, 1000000) ||
      !wait([](const auto &v) { return v.imu == 1000000; }) ||
      !step(98, 99000000) || !step(1, 100000000) ||
      !wait([](const auto &v) { return v.imu == 100000000; })) return 36;
  auto valid = [&](const gz::msgs::IMU &frame, int64_t stamp, unsigned int sequence)
  {
    const auto &header = frame.header();
    if (frame.entity_name() != "ball::body::imu" || !header.has_stamp() ||
        header.stamp().sec() != 0 || header.stamp().nsec() != stamp || header.data_size() != 2 ||
        header.data(0).key() != "frame_id" || header.data(0).value_size() != 1 ||
        header.data(0).value(0) != "ball::body::imu" || header.data(1).key() != "seq" ||
        header.data(1).value_size() != 1 || header.data(1).value(0) != std::to_string(sequence) ||
        frame.has_orientation() != orientation || !frame.has_angular_velocity() || !frame.has_linear_acceleration() ||
        !frame.has_orientation_covariance() || !frame.has_angular_velocity_covariance() ||
        !frame.has_linear_acceleration_covariance()) return false;
    if (noisy)
    {
      for (const auto *covariance : {&frame.orientation_covariance(), &frame.angular_velocity_covariance(), &frame.linear_acceleration_covariance()})
      {
        if (covariance->has_header() || covariance->data_size() != 9) return false;
        for (int index = 0; index < 9; ++index)
        {
          const float expected = covariance == &frame.angular_velocity_covariance() && index == 8 ? 0.01f : 0.0f;
          if (covariance->data(index) != expected) return false;
        }
      }
      for (double value : {frame.orientation().x(), frame.orientation().y(), frame.orientation().z(),
          frame.orientation().w() - (orientation ? 1 : 0), frame.angular_velocity().x() - 0.25, frame.angular_velocity().y() + 2.5,
          frame.linear_acceleration().x(), frame.linear_acceleration().y(),
          frame.linear_acceleration().z() - (tpe ? 9.81 : 0.0)})
        if (!std::isfinite(value) || std::abs(value) >= 1e-9) return false;
      return std::isfinite(frame.angular_velocity().z()) && frame.angular_velocity().z() != 0;
    }
    for (const auto *covariance : {&frame.orientation_covariance(), &frame.angular_velocity_covariance(), &frame.linear_acceleration_covariance()})
    {
      if (covariance->has_header() || covariance->data_size() != 9) return false;
      for (float value : covariance->data()) if (value != 0) return false;
    }
    for (double value : {frame.orientation().x(), frame.orientation().y(), frame.orientation().z(),
        frame.orientation().w() - (orientation ? 1 : 0), frame.angular_velocity().x(), frame.angular_velocity().y(),
        frame.angular_velocity().z(), frame.linear_acceleration().x(), frame.linear_acceleration().y(),
        frame.linear_acceleration().z() - (tpe ? 9.81 : 0.0)})
      if (!std::isfinite(value) || std::abs(value) >= 1e-9) return false;
    return true;
  };
  double firstNoisyZ = 0;
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    // Receipt of the last frame drains earlier frames on this same IMU stream.
    if (observed->overflow || observed->frames.size() != 2 || !valid(observed->frames[0], 1000000, 0) ||
        !valid(observed->frames[1], 100000000, 1)) return 37;
    if (noisy) firstNoisyZ = observed->frames[0].angular_velocity().z();
  }
  if (!reset() || !step(1, 1000000) ||
      !wait([](const auto &v) { return v.imu == 1000000; })) return 38;
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    if (observed->overflow || observed->frames.size() != 1 || !valid(observed->frames[0], 1000000, 0)) return 39;
    if (noisy && observed->frames[0].angular_velocity().z() != firstNoisyZ) return 40;
  }
  if (noisy)
  {
    std::cout << "PASS native noisy schedule warmup=" << warmup << " final=1 samples=2 reset_sequence=0\n";
    return 0;
  }
  std::cout << "PASS native schedule warmup=" << warmup << " final=1 samples=2 reset_sequence=0\n";
  return 0;
}

int RunRootClockCase()
{
  struct ClockPairs
  {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::string> frames[2];
    int64_t target = 0;
    bool matched = false;
    bool invalid = false;
  };
  auto observed = std::make_shared<ClockPairs>();
  auto callback = [observed](unsigned int endpoint)
  {
    return std::function<void(const gz::msgs::Clock &)>([observed, endpoint](const auto &clock)
    {
      std::lock_guard<std::mutex> lock(observed->mutex);
      if (!clock.has_sim() || !clock.has_real() || !clock.has_system() ||
          clock.sim().sec() != 0 || clock.sim().nsec() < 0 || clock.sim().nsec() > 2000000)
        observed->invalid = true;
      else if (clock.sim().nsec() == observed->target)
      {
        const auto bytes = clock.SerializeAsString();
        auto &own = observed->frames[endpoint];
        const auto &other = observed->frames[1 - endpoint];
        // Keep bounded recent observations; this does not assert lossless delivery.
        if (own.size() == 64) own.erase(own.begin());
        own.push_back(bytes);
        observed->matched = observed->matched ||
            std::find(other.begin(), other.end(), bytes) != other.end();
      }
      observed->changed.notify_all();
    });
  };
  gz::transport::Node node;
  if (!node.Subscribe("/clock", callback(0)) ||
      !node.Subscribe("/world/default/clock", callback(1))) return 81;
  auto wait = [&]
  {
    std::unique_lock<std::mutex> lock(observed->mutex);
    return observed->changed.wait_for(lock, std::chrono::seconds(8), [&]
        { return observed->matched || observed->invalid; }) && !observed->invalid;
  };
  if (!wait()) return 82;
  for (int64_t target : {1000000LL, 2000000LL})
  {
    {
      std::lock_guard<std::mutex> lock(observed->mutex);
      observed->target = target;
      observed->matched = false;
      for (auto &frames : observed->frames) frames.clear();
    }
    gz::msgs::WorldControl request;
    request.set_pause(true);
    request.set_multi_step(1);
    gz::msgs::Boolean response;
    bool result = false;
    if (!request_with_reply(node, "/world/default/control", request, 7000, response, result)
        || !result || !response.data() || !wait()) return 83;
  }
  std::cout << "PASS native root clock pairs=3 final=2000000\n";
  return 0;
}

// Supplemental Light wire mode; existing clock/IMU modes remain unchanged.
int RunLightPosesCase(const std::string &mode, const std::string &output)
{
  const bool twelve = mode == "twelve";
  const bool empty = mode == "empty";
  if (!twelve && !empty && mode != "sun-off" && mode != "sun-default") return 101;
  struct LightFrames
  {
    std::mutex mutex;
    std::condition_variable changed;
    bool invalid = false, resetArmed = false;
    bool initialClock = false, stepClock = false, resetClock = false;
    bool initialPose = false, stepPose = false, resetPose = false;
    std::map<std::string, uint32_t> ids;
    gz::msgs::Pose_V frames[3];
  };
  const int count = twelve ? 24 : empty ? 0 : 2;
  std::set<std::string> expected;
  if (twelve)
    for (const std::string type : {"directional", "point", "spot"})
      for (const std::string on : {"_on_", "_off_"})
        for (const std::string visible : {"visible", "hidden"})
        {
          const auto name = type + on + visible;
          expected.insert(name); expected.insert(name + "Visual");
        }
  else if (!empty) expected = {"sun", "sunVisual"};
  auto observed = std::make_shared<LightFrames>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Clock &)> clockCallback = [observed](const auto &clock)
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    if (!clock.has_sim() || !clock.has_real() || !clock.has_system() ||
        clock.sim().sec() != 0 || (clock.sim().nsec() != 0 && clock.sim().nsec() != 1000000))
      observed->invalid = true;
    else if (clock.sim().nsec() == 1000000) observed->stepClock = true;
    else if (observed->resetArmed) observed->resetClock = true;
    else observed->initialClock = true;
    observed->changed.notify_all();
  };
  std::function<void(const gz::msgs::Pose_V &)> poseCallback =
      [observed, expected, count, twelve](const auto &poses)
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    bool valid = poses.has_header() && poses.header().has_stamp() &&
        poses.header().data_size() == 0 && poses.header().stamp().sec() == 0 &&
        (poses.header().stamp().nsec() == 0 || poses.header().stamp().nsec() == 1000000) &&
        poses.pose_size() == count;
    std::map<std::string, uint32_t> names;
    std::set<uint32_t> ids;
    bool lightSeen = false;
    const double native[] = {3.44025611409198, -0.7556191885311716, 4.483992419371312,
        0.09068129542746878, 0.018271207596863924, 0.05842711968717301, 0.9939966485444552};
    for (const auto &pose : poses.pose())
    {
      const auto &name = pose.name();
      const bool visual = name.size() >= 6 && name.compare(name.size() - 6, 6, "Visual") == 0;
      if (!visual) lightSeen = true;
      else if (lightSeen) valid = false;
      valid = valid && expected.count(name) == 1 && pose.id() != 0 &&
          names.emplace(name, pose.id()).second && ids.insert(pose.id()).second &&
          !pose.has_header() && pose.has_position() && pose.has_orientation() &&
          !pose.position().has_header() && !pose.orientation().has_header();
      const double actual[] = {pose.position().x(), pose.position().y(), pose.position().z(),
          pose.orientation().x(), pose.orientation().y(), pose.orientation().z(), pose.orientation().w()};
      for (unsigned int i = 0; i < 7; ++i)
      {
        const double target = visual || !twelve ? (i == 6 ? 1.0 : 0.0) : native[i];
        valid = valid && std::isfinite(actual[i]) &&
            (visual || !twelve ? actual[i] == target : std::abs(actual[i] - target) < 1e-12);
      }
    }
    valid = valid && names.size() == expected.size();
    if (!observed->initialPose) observed->ids = names;
    else valid = valid && names == observed->ids;
    observed->invalid = observed->invalid || !valid;
    if (valid)
    {
      if (poses.header().stamp().nsec() == 1000000)
      { observed->stepPose = true; observed->frames[1] = poses; }
      else if (observed->resetArmed)
      { observed->resetPose = true; observed->frames[2] = poses; }
      else { observed->initialPose = true; observed->frames[0] = poses; }
    }
    observed->changed.notify_all();
  };
  if (!node.Subscribe("/world/default/clock", clockCallback) ||
      !node.Subscribe("/world/default/pose/info", poseCallback)) return 102;
  auto wait = [&](auto ready)
  {
    std::unique_lock<std::mutex> lock(observed->mutex);
    return observed->changed.wait_for(lock, std::chrono::seconds(8), [&]
        { return observed->invalid || ready(*observed); }) && !observed->invalid;
  };
  if (!wait([](const auto &v) { return v.initialClock && v.initialPose; })) return 103;
  gz::msgs::WorldControl control;
  control.set_pause(true); control.set_multi_step(1);
  gz::msgs::Boolean response;
  bool result = false;
  if (!request_with_reply(node, "/world/default/control", control, 7000, response, result) ||
      !result || !response.data()) return 104;
  if (!wait([](const auto &v) { return v.stepClock && v.stepPose; })) return 105;
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    observed->resetArmed = true;
  }
  control.Clear(); control.set_pause(true); control.mutable_reset()->set_all(true);
  if (!request_with_reply(node, "/world/default/control", control, 7000, response, result) ||
      !result || !response.data()) return 106;
  if (!wait([](const auto &v) { return v.resetClock && v.resetPose; })) return 107;
  {
    std::lock_guard<std::mutex> lock(observed->mutex);
    const std::string names[] = {"initial", "step", "reset"};
    for (unsigned int i = 0; i < 3; ++i)
    {
      std::ofstream raw(output + "/" + names[i] + ".pose_v.pb", std::ios::binary);
      std::ofstream text(output + "/" + names[i] + ".pose_v.txt");
      if (!observed->frames[i].SerializeToOstream(&raw)) return 108;
      text << observed->frames[i].DebugString();
      if (!raw || !text) return 108;
    }
    for (const auto &entry : observed->ids) std::cout << "ID " << entry.first << ' ' << entry.second << '\n';
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream capture(output + "/maps"); capture << maps.rdbuf();
  if (!maps || !capture) return 108;
  std::cout << "PASS native light poses case=" << mode << " rows=" << count
      << " first=1000000 reset=0\n";
  return 0;
}

int main(int argc, char **argv)
{
  if (argc > 1)
  {
    const std::string mode = argv[1];
    if (mode == "--light-poses")
      return argc == 4 ? RunLightPosesCase(argv[2], argv[3]) : 101;
    if (mode == "--root-clock") return RunRootClockCase();
    if (mode == "--disabled-schedule" || mode == "--tpe-disabled-schedule" ||
        mode == "--disabled-noisy-schedule" || mode == "--tpe-disabled-noisy-schedule")
      return RunSamplingCase(mode.find("--tpe-") == 0, mode.find("noisy") != std::string::npos, false);
  }
  if (argc > 1 && (std::string(argv[1]) == "--noisy-schedule" || std::string(argv[1]) == "--tpe-noisy-schedule"))
    return RunSamplingCase(std::string(argv[1]) == "--tpe-noisy-schedule", true);
  if (argc > 1 && (std::string(argv[1]) == "--schedule" || std::string(argv[1]) == "--tpe-schedule"))
    return RunSamplingCase(std::string(argv[1]) == "--tpe-schedule");
  const bool sharedImu = argc > 1 && std::string(argv[1]) == "--shared";
  const bool tpe = argc > 1 && std::string(argv[1]) == "--tpe-reset";
  const bool reset = tpe || (argc > 1 && std::string(argv[1]) == "--reset");
  const bool withImu = reset || sharedImu || (argc > 1 && std::string(argv[1]) == "--imu");
  const bool remove = argc > 1 && std::string(argv[1]) == "--remove";
  auto observations = std::make_shared<Observations>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Clock &)> callback =
    [observations, reset](const auto &clock)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      if (!clock.has_sim() || !clock.has_real() || !clock.has_system() ||
          clock.sim().nsec() < 0 || clock.sim().nsec() >= 1000000000 ||
          clock.real().nsec() < 0 || clock.real().nsec() >= 1000000000 ||
          clock.system().sec() < 1000000000 || clock.sim().sec() != 0)
        observations->invalid = true;
      if (clock.sim().sec() == 0 && clock.sim().nsec() == 0)
      {
        if (reset && (clock.real().sec() != 0 || clock.real().nsec() != 0))
          observations->invalid = true;
        observations->initial = true;
      }
      observations->lastClock = clock.sim().nsec();
      if (clock.sim().sec() == 0 && clock.sim().nsec() == observations->target)
        observations->final = true;
      observations->changed.notify_all();
    };
  if (!node.Subscribe("/world/default/clock", callback)) return 2;
  std::function<void(const gz::msgs::Pose_V &)> poseCallback =
    [observations, tpe](const auto &poses)
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
        if (pose.name() == "ball") valid = valid && (tpe ? z == 5 : z <= 5 && z > 4);
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
    [observations, sharedImu, tpe, frame = std::string(argc > 3 ? argv[3] : "ball::body::imu")](const auto &imu)
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
          imu.linear_acceleration().x(), imu.linear_acceleration().y(), imu.linear_acceleration().z() - (tpe ? 9.81 : 0.0)})
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
  unsigned int reconnect = 0;
  if (withImu)
  {
    // Establish sensor readiness using real sampled steps, never fabricated packets.
    for (; warmup < 100;)
    {
      ++warmup;
      request.set_pause(true);
      request.set_step(true);
      if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
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
  if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
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
  if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
      !result || response.data()) return 6;
  if (remove || reset)
  {
    gz::msgs::Entity entity;
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      entity.set_id(observations->poseIds.at("body"));
    }
    if (!request_with_reply(node, "/world/default/remove", entity, 7000, response, result) ||
        !result || !response.data()) return 14;
    if (!request_with_reply(node, "/world/default/remove/blocking", entity, 7000, response, result) ||
        !result || response.data()) return 15;
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      observations->allowRemoved = true;
      entity.set_id(observations->poseIds.at("ball"));
    }
    // A nonzero ID takes precedence over a contradictory name and type.
    entity.set_name("ground");
    entity.set_type(gz::msgs::Entity::LINK);
    if (!request_with_reply(node, "/world/default/remove/blocking", entity, 7000, response, result) ||
        !result || !response.data()) return 16;
    if (!waitFor(&Observations::removed)) return 17;
  }
  if (reset)
  {
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      observations->initial = false;
      observations->initialPose = false;
      observations->lastImu = 0;
      observations->imuTimes.clear();
    }
    request.Clear();
    request.set_pause(true);
    request.mutable_reset()->set_all(true);
    if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
        !result || !response.data()) return 18;
    if (!waitFor(&Observations::initial) || !waitFor(&Observations::initialPose)) return 19;
    // Removal retired the endpoint. Prove the recreated sensor connection with
    // sampled steps, then reset its live lease to test the first sequence exactly.
    for (; reconnect < 100;)
    {
      ++reconnect;
      request.Clear();
      request.set_pause(true);
      request.set_step(true);
      if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
          !result || !response.data()) return 23;
      std::unique_lock<std::mutex> lock(observations->mutex);
      const int64_t stamp = reconnect * 1000000;
      if (!observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
          { return observations->lastClock == stamp || observations->invalid; }) ||
          observations->invalid) return 24;
      if (observations->changed.wait_for(lock, std::chrono::milliseconds(50), [&]
          { return observations->lastImu == stamp || observations->invalid; })) break;
    }
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      if (observations->lastImu == 0 || observations->invalid) return 25;
      observations->initial = false;
      observations->initialPose = false;
      observations->lastImu = 0;
      observations->target = 1000000;
      observations->final = false;
    }
    request.Clear();
    request.set_pause(true);
    request.mutable_reset()->set_time_only(true);
    if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
        !result || !response.data()) return 26;
    if (!waitFor(&Observations::initial) || !waitFor(&Observations::initialPose)) return 27;
    request.Clear();
    request.set_pause(true);
    request.set_step(true);
    if (!request_with_reply(node, "/world/default/control", request, 7000, response, result) ||
        !result || !response.data()) return 20;
    if (!waitFor(&Observations::final)) return 21;
    std::unique_lock<std::mutex> lock(observations->mutex);
    if (!observations->changed.wait_for(lock, std::chrono::seconds(8), [&]
        { return observations->invalid ||
            (observations->lastImu == 1000000 && observations->lastPose == 1000000); }) ||
        observations->invalid) return 22;
  }
  std::cout << "PASS native clock/control: baseline=" << warmup
    << " final=" << (reset ? 1 : warmup + 3) << " imu=" << withImu << " poses=4 removed=" << remove
    << " shared=" << sharedImu << " reset=" << reset << " reconnect=" << reconnect << '\n';
}
