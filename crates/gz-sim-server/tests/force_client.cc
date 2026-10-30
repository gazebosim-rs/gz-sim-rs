// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

// stdin/stdout coordinate this fixture, not the Gazebo wire protocol.
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::Double>(argv[1]);
  if (!publisher) return 3;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(7);
  while (!publisher.HasConnections()) {
    if (std::chrono::steady_clock::now() >= deadline) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout << "READY" << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line == "quit") { std::cout << "DONE" << std::endl; return 0; }
    gz::msgs::Double message;
    if (line != "default") {
      try {
        size_t consumed = 0;
        const double value = std::stod(line, &consumed);
        if (consumed != line.size()) return 5;
        message.set_data(value);
      } catch (const std::exception &) { return 6; }
    }
    if (!publisher.Publish(message)) return 7;
    std::cout << "SENT" << std::endl;
  }
  return 8;
}
