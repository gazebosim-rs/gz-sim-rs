// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/double.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Test command acknowledgments are not application receipts from the service.
int main(int argc, char **)
{
  if (argc != 1) { return 9; }
  struct State {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::string> scans;
    std::vector<std::string> points;
    bool overflow = false;
  };
  auto state = std::make_shared<State>();
  auto record = [state](const auto &message, bool points) {
    std::lock_guard<std::mutex> lock(state->mutex);
    auto &frames = points ? state->points : state->scans;
    if (message.ByteSizeLong() > 1024 || frames.size() == 16) { state->overflow = true; }
    else { frames.push_back(message.SerializeAsString()); }
    state->changed.notify_all();
  };
  gz::transport::Node node;
  if (!node.Subscribe("/m115/scan", std::function<void(const gz::msgs::LaserScan &)>(
      [record](const auto &message) { record(message, false); }))) { return 2; }
  if (!node.Subscribe("/m115/scan/points", std::function<void(const gz::msgs::PointCloudPacked &)>(
      [record](const auto &message) { record(message, true); }))) { return 2; }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(7);
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<gz::transport::ServicePublisher> services;
    node.ServiceInfo("/m115/scan/set_rate", services);
    if (services.size() == 1 && services[0].ReqTypeName() == "gz.msgs.Double" &&
        services[0].RepTypeName() == "gz.msgs.Empty") { found = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!found) { return 3; }
  std::cout << "READY" << std::endl;
  std::string line;
  unsigned commands = 0;
  while (std::getline(std::cin, line)) {
    if (++commands > 32 || line.size() > 128) { return 10; }
    std::istringstream input(line);
    std::string command;
    input >> command;
    if (command == "rate") {
      double value;
      std::string extra;
      if (!(input >> value) || (input >> extra)) { return 4; }
      gz::msgs::Double request;
      request.set_data(value);
      if (!node.Request("/m115/scan/set_rate", request)) { return 5; }
      std::cout << "SENT" << std::endl;
    } else if (command == "captured") {
      std::unique_lock<std::mutex> lock(state->mutex);
      if (!state->changed.wait_for(lock, std::chrono::seconds(5), [&] {
          return state->overflow || (!state->scans.empty() && !state->points.empty());
        }) || state->overflow) { return 6; }
      std::cout << "CAPTURED" << std::endl;
    } else if (command == "snapshot") {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->overflow) { return 6; }
      auto output = [](const std::vector<std::string> &frames) {
        std::cout << '[';
        bool first = true;
        constexpr char hex[] = "0123456789abcdef";
        for (const auto &frame : frames) {
          if (!first) { std::cout << ','; }
          first = false;
          std::cout << '"';
          for (unsigned char byte : frame) { std::cout << hex[byte >> 4] << hex[byte & 15]; }
          std::cout << '"';
        }
        std::cout << ']';
      };
      std::cout << "{\"scan\":";
      output(state->scans);
      std::cout << ",\"points\":";
      output(state->points);
      std::cout << '}' << std::endl;
    } else if (command == "arm") {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->scans.clear(); state->points.clear(); state->overflow = false;
      std::cout << "ARMED" << std::endl;
    } else if (command == "quit") {
      std::cout << "BYE" << std::endl;
      return 0;
    } else { return 7; }
  }
  return 8;
}
