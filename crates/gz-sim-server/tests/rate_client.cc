// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/double.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// stdin/stdout synchronize the test harness, not the outputless Gazebo protocol.
int main(int argc, char **argv)
{
  if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--delay-first")) return 9;
  const bool delayFirst = argc == 2;
  struct State {
    std::mutex mutex;
    std::vector<gz::msgs::IMU> frames;
    bool overflow = false;
    std::atomic<bool> first{true};
  };
  auto observed = std::make_shared<State>();
  gz::transport::Node node;
  if (!node.Subscribe("/p2/imu", std::function<void(const gz::msgs::IMU &)>(
      [observed, delayFirst](const auto &message) {
        if (delayFirst && observed->first.exchange(false))
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> guard(observed->mutex);
        if (observed->frames.size() == 256) observed->overflow = true;
        else observed->frames.push_back(message);
      }))) return 2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(7);
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::vector<gz::transport::ServicePublisher> providers;
    node.ServiceInfo("/p2/imu/set_rate", providers);
    if (providers.size() == 1 && providers[0].ReqTypeName() == "gz.msgs.Double" &&
        providers[0].RepTypeName() == "gz.msgs.Empty")
    {
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!found) return 3;
  std::cout << "READY" << std::endl;
  std::string line;
  while (std::getline(std::cin, line))
  {
    std::istringstream input(line);
    std::string command;
    input >> command;
    if (command == "rate")
    {
      double value;
      if (!(input >> value)) return 4;
      gz::msgs::Double request;
      request.set_data(value);
      // Empty replies are one-way. This return value proves handoff only.
      if (!node.Request("/p2/imu/set_rate", request)) return 5;
      std::cout << "SENT" << std::endl;
    }
    else if (command == "arm")
    {
      std::lock_guard<std::mutex> guard(observed->mutex);
      observed->frames.clear();
      observed->overflow = false;
      std::cout << "ARMED" << std::endl;
    }
    else if (command == "snapshot")
    {
      std::lock_guard<std::mutex> guard(observed->mutex);
      if (observed->overflow) return 6;
      std::cout << "[";
      bool first = true;
      for (const auto &frame : observed->frames)
      {
        if (!first) std::cout << ',';
        first = false;
        std::cout << '"';
        constexpr char hex[] = "0123456789abcdef";
        for (unsigned char byte : frame.SerializeAsString())
          std::cout << hex[byte >> 4] << hex[byte & 15];
        std::cout << '"';
      }
      std::cout << "]" << std::endl;
    }
    else if (command == "quit")
    {
      std::cout << "BYE" << std::endl;
      return 0;
    }
    else if (command == "hang")
    {
      std::cerr << "intentional rate test hang" << std::endl;
      std::cout << "HANGING" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(60));
      return 10;
    }
    else
    {
      std::cerr << "invalid rate test command" << std::endl;
      return 7;
    }
  }
  return 8;
}
