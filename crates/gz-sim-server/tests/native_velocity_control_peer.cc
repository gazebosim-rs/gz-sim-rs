// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical transport-only peer; no native Sim/physics/system linkage.
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
int main() {
  try {
    gz::transport::Node node;
    auto publisher = node.Advertise<gz::msgs::Twist>("/m79/cmd");
    if (!publisher) throw std::runtime_error("canonical Twist publisher");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!publisher.HasConnections()) {
      if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("VelocityControl subscriber deadline");
      std::this_thread::yield();
    }
    gz::msgs::Twist value;
    value.mutable_linear()->set_x(1.0); value.mutable_linear()->set_y(-2.0); value.mutable_linear()->set_z(3.0);
    value.mutable_angular()->set_x(0.5); value.mutable_angular()->set_y(-0.75); value.mutable_angular()->set_z(1.25);
    if (!publisher.Publish(value)) throw std::runtime_error("six-axis canonical publication");
    value.mutable_linear()->set_x(std::numeric_limits<double>::infinity());
    if (!publisher.Publish(value)) throw std::runtime_error("nonfinite canonical control publication");
    std::cout << "SENT six-axis Twist and invalid control\n" << std::flush;
    // Keep this actual publisher alive until the harness observes both real
    // callback receipts. The owning Rust process enforces the35-second bound.
    std::string command;
    if (!std::getline(std::cin, command) || command != "finish") throw std::runtime_error("owned harness completion");
    std::cout << "PASS canonical six-axis VelocityControl\n" << std::flush;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "velocity-control peer: " << error.what() << '\n'; return 1;
  }
}
