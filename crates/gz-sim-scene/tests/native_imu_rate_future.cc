// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/sim/TestFixture.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <iterator>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
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

int main(int argc, char **argv)
try
{
  if (argc != 4) return 2;
  const std::string mode = argv[1], engine = argv[2];
  if ((mode != "zero" && mode != "nan" && mode != "zero-peer" && mode != "nan-peer") || (engine != "tpe" && engine != "dartsim")) return 2;
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  std::mutex mutex;
  std::atomic<int> phase{0};
  std::condition_variable fenceReady;
  int sentFence = 0, receivedFence = 0;
  struct Tick { int phase; long long time; unsigned long long iteration; bool paused; };
  std::vector<Tick> ticks;
  struct Boundary { std::string label; int phase; size_t samples; size_t ticks; };
  std::vector<Boundary> boundaries;
  bool overflow = false;
  struct Sample { int phase; std::string channel; gz::msgs::IMU message; };
  std::vector<Sample> samples;
  gz::transport::Node node;
  if (!node.Subscribe("/p2/imu", std::function<void(const gz::msgs::IMU &)>([&](const auto &message)
      { std::lock_guard<std::mutex> guard(mutex);
        if (samples.size() >= 256) { overflow = true; return; }
        samples.push_back({phase.load(), "target", message}); }))) return 3;
  if (!node.Subscribe("/rate_capture_fence", std::function<void(const gz::msgs::Int32 &)>([&](const auto &message)
      { std::lock_guard<std::mutex> guard(mutex); receivedFence = message.data(); fenceReady.notify_all(); }))) return 3;
  if (mode.find("-peer") != std::string::npos &&
      !node.Subscribe("/p2/other", std::function<void(const gz::msgs::IMU &)>([&](const auto &message)
      { std::lock_guard<std::mutex> guard(mutex);
        if (samples.size() >= 256) { overflow = true; return; }
        samples.push_back({phase.load(), "peer", message}); }))) return 3;
  auto fencePublisher = node.Advertise<gz::msgs::Int32>("/rate_capture_fence");
  if (!fencePublisher) return 3;
  auto checkpoint = [&](const std::string &label) {
    // All local sensor and fence callbacks share the pinned single FIFO pubQueue.
    gz::msgs::Int32 fence; fence.set_data(++sentFence);
    if (!fencePublisher.Publish(fence)) throw std::runtime_error("callback fence publish failed");
    std::unique_lock<std::mutex> guard(mutex);
    if (!fenceReady.wait_for(guard, std::chrono::seconds(7), [&]{return receivedFence == sentFence;}))
      throw std::runtime_error("callback drain fence timed out");
    boundaries.push_back({label, phase.load(), samples.size(), ticks.size()});
  };
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 4;
  config.SetPhysicsEngine("gz-physics-" + engine + "-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world", sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world", sdf::Plugin("gz-sim-imu-system", "gz::sim::systems::Imu")));
  unsigned resets = 0;
  gz::sim::TestFixture fixture(config);
  fixture.OnReset([&](const auto &, auto &) { ++resets; })
    .OnPostUpdate([&](const auto &info, const auto &) {
      ticks.push_back({phase.load(), std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(), info.iterations, info.paused});
    }).Finalize();
  auto server = fixture.Server();
  if (!server->RunOnce(true)) return 5;
  checkpoint("bootstrap_end");
  auto rate = [&](double value) {
    gz::msgs::Double request; request.set_data(value);
    gz::msgs::Empty reply; bool result = false;
    if (!request_with_reply(node, "/p2/imu/set_rate", request, 7000, reply, result) || !result || reply.ByteSizeLong())
      throw std::runtime_error("rate request failed");
  };
  rate(20.0);
  phase = 1; checkpoint("initial_start");
  if (!server->Run(true, 1, false)) return 6;
  checkpoint("initial_end");
  rate(mode.find("nan") == 0 ? std::numeric_limits<double>::quiet_NaN() : 0.0);
  phase = 2; checkpoint("changed_start");
  if (!server->Run(true, 10, false)) return 7;
  checkpoint("changed_end");
  phase = 3; checkpoint("before_deadline_start");
  if (!server->Run(true, 38, false)) return 8;
  checkpoint("before_deadline_end");
  phase = 4; checkpoint("deadline_start");
  if (!server->Run(true, 4, false)) return 9;
  checkpoint("deadline_end");
  std::ifstream maps("/proc/self/maps"); std::ofstream output(argv[3]); output << maps.rdbuf();
  if (!output) return 11;
  std::lock_guard<std::mutex> guard(mutex);
  if (overflow || samples.empty()) return 12;
  std::cout << "{\"mode\":\"" << mode << "\",\"engine\":\"" << engine << "\",\"resets\":" << resets << ",\"samples\":[";
  bool first = true;
  for (const auto &entry : samples)
  {
    if (!first) std::cout << ',';
    first = false;
    const auto &stamp = entry.message.header().stamp();
    std::cout << "{\"phase\":" << entry.phase << ",\"channel\":\"" << entry.channel << "\",\"time_ns\":" << stamp.sec()*1000000000LL+stamp.nsec() << ",\"wire_hex\":\"";
    const auto bytes = entry.message.SerializeAsString();
    constexpr char hex[] = "0123456789abcdef";
    for (unsigned char byte : bytes) std::cout << hex[byte >> 4] << hex[byte & 15];
    std::cout << "\"}";
  }
  std::cout << "],\"ticks\":[";
  first = true;
  for (const auto &tick : ticks) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"phase\":" << tick.phase << ",\"time_ns\":" << tick.time << ",\"iteration\":" << tick.iteration << ",\"paused\":" << std::boolalpha << tick.paused << '}';
  }
  std::cout << "],\"boundaries\":[";
  first = true;
  for (const auto &entry : boundaries) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"label\":\"" << entry.label << "\",\"phase\":" << entry.phase << ",\"samples\":" << entry.samples << ",\"ticks\":" << entry.ticks << '}';
  }
  std::cout << "]}\n";
  return 0;
}
catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 2; }
