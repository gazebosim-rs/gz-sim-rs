// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/transport/Node.hh>
#include <gz/transport/config.hh>
#include <gz/msgs/double.pb.h>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#if !defined(HAVE_ZENOH) || !HAVE_ZENOH
#error "The client reference must enable Zenoh"
#endif

int main()
{
  std::mutex mutex;
  std::condition_variable received;
  bool negativeZero = false;
  gz::transport::Node node;
  const std::function<void(const gz::msgs::Double &)> callback = [&](const gz::msgs::Double &value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    negativeZero = value.data() == 0.0 && std::signbit(value.data());
    received.notify_one();
  };
  if (!node.Subscribe("/bootstrap", callback))
    return 1;
  auto publisher = node.Advertise<gz::msgs::Double>("/bootstrap");
  if (!publisher)
    return 2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!publisher.HasConnections())
  {
    if (std::chrono::steady_clock::now() >= deadline)
      return 3;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  gz::msgs::Double message;
  message.set_data(-0.0);
  if (!publisher.Publish(message))
    return 4;
  std::unique_lock<std::mutex> lock(mutex);
  if (!received.wait_until(lock, deadline, [&] {return negativeZero;}))
    return 5;
  std::cout << "zenoh client ready\n";
}
