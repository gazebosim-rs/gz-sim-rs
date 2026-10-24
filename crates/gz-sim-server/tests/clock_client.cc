// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

struct Observations
{
  std::mutex mutex;
  std::condition_variable changed;
  bool initial = false;
  bool final = false;
  bool invalid = false;
};

int main()
{
  auto observations = std::make_shared<Observations>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Clock &)> callback =
    [observations](const auto &clock)
    {
      std::lock_guard<std::mutex> lock(observations->mutex);
      if (!clock.has_sim() || !clock.has_real() || !clock.has_system() ||
          clock.sim().nsec() < 0 || clock.sim().nsec() >= 1000000000 ||
          clock.real().nsec() < 0 || clock.real().nsec() >= 1000000000 ||
          clock.system().sec() < 1000000000)
        observations->invalid = true;
      if (clock.sim().sec() == 0 && clock.sim().nsec() == 0)
        observations->initial = true;
      if (clock.sim().sec() == 0 && clock.sim().nsec() == 3000000)
        observations->final = true;
      observations->changed.notify_all();
    };
  if (!node.Subscribe("/world/default/clock", callback)) return 2;
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
  request.set_pause(true);
  request.set_step(true);
  request.set_multi_step(3);
  gz::msgs::Boolean response;
  bool result = false;
  if (!node.Request("/world/default/control", request, 7000, response, result) ||
      !result || !response.data()) return 4;
  if (!waitFor(&Observations::final)) return 5;
  request.Clear();
  request.set_seed(42);
  if (!node.Request("/world/default/control", request, 7000, response, result) ||
      !result || response.data()) return 6;
  std::cout << "PASS native clock/control: 3 steps, 3000000 ns, rejected seed\n";
}
