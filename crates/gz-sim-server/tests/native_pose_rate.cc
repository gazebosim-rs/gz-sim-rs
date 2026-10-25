// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/int32.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  using Clock = std::chrono::steady_clock;
  using namespace std::chrono_literals;
  auto require = [](bool value) {
    if (!value) throw std::runtime_error("native pose-rate contract failed");
  };
  std::mutex mutex;
  std::condition_variable received;
  std::vector<int> values;
  gz::transport::Node node;
  std::function<void(const gz::msgs::Int32 &)> callback =
      [&](const gz::msgs::Int32 &message) {
        std::lock_guard<std::mutex> lock(mutex);
        values.push_back(message.data());
        received.notify_all();
      };
  require(node.Subscribe("/pose-rate-control", callback));
  gz::transport::AdvertiseMessageOptions options;
  options.SetMsgsPerSec(60);
  auto publisher = node.Advertise<gz::msgs::Int32>("/pose-rate-control", options);
  require(publisher.Valid() && publisher.ThrottledUpdateReady());
  auto copy = publisher;
  const auto start = Clock::now();
  gz::msgs::Int32 message;
  message.set_data(0);
  require(publisher.Publish(message));
  for (int value = 1; value <= 4; ++value)
  {
    require(!copy.ThrottledUpdateReady());
    message.set_data(value);
    // Native reports success for intentional suppression.
    require(copy.Publish(message));
  }
  const auto burst = Clock::now() - start;
  require(burst < 16666667ns);
  const auto deadline = start + 5s;
  while (!publisher.ThrottledUpdateReady())
  {
    require(Clock::now() < deadline);
    std::this_thread::yield();
  }
  const auto eligible = Clock::now() - start;
  require(eligible >= 16666667ns);
  message.set_data(99);
  require(publisher.Publish(message));
  {
    std::unique_lock<std::mutex> lock(mutex);
    require(received.wait_until(lock, deadline, [&] { return values.size() >= 2; }));
    require(values == std::vector<int>({0, 99}));
  }
  std::ofstream maps(argv[1]);
  maps << std::ifstream("/proc/self/maps").rdbuf();
  require(maps.good());
  std::cout << "{\"hz\":60,\"minimum_integer_period_ns\":16666667,"
      << "\"offers\":6,\"received\":[0,99],\"suppressed_publish_returns_true\":true,"
      << "\"shared_publisher_timer\":true,\"burst_ns\":"
      << std::chrono::duration_cast<std::chrono::nanoseconds>(burst).count()
      << ",\"next_ready_ns\":"
      << std::chrono::duration_cast<std::chrono::nanoseconds>(eligible).count()
      << "}\n";
}
