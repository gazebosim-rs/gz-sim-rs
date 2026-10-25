// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <fstream>
#include <future>
#include <chrono>
#include <string>
#include <memory>
#include <stdexcept>
#include <utility>
#include <optional>
#include <iomanip>
#include <functional>
#include <iterator>
#include <sdf/Plugin.hh>
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

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  gz::sim::ServerConfig config;
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  if (!config.SetSdfString(xml)) return 2;
  const std::string engine = argv[2];
  if (engine != "dartsim" && engine != "tpe") return 2;
  config.SetPhysicsEngine("gz-physics-" + engine + "-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-user-commands-system", "gz::sim::systems::UserCommands")));
  gz::sim::TestFixture fixture(config);
  unsigned resets = 0;
  int64_t lastTime = 0;
  gz::sim::Entity ball = 0, original = 0;
  std::optional<gz::math::Pose3d> pose;
  auto inspect = [&](const gz::sim::EntityComponentManager &ecm) {
    ball = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("ball"));
    if (!original) original = ball;
    const auto *component = ecm.Component<gz::sim::components::Pose>(ball);
    if (ball && !component) throw std::runtime_error("model has no pose");
    pose = component ? std::optional(component->Data()) : std::nullopt;
  };
  fixture.OnPostUpdate([&](const auto &info, const auto &ecm) { lastTime = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(); inspect(ecm); })
    .OnReset([&](const auto &, auto &ecm) { ++resets; inspect(ecm); }).Finalize();
  auto server = fixture.Server();
  if (!server->RunOnce(true) || !original) return 3;
  gz::transport::Node node;
  gz::msgs::Boolean response;
  bool result = false;
  auto control = [&](bool rewind, bool modelOnly, unsigned steps = 0, bool timeOnly = false) {
    gz::msgs::WorldControl request;
    request.set_pause(true);
    if (rewind) { if (timeOnly) request.mutable_reset()->set_time_only(true); else request.mutable_reset()->set_all(true); }
    request.set_multi_step(steps);
    if (modelOnly) request.mutable_reset()->set_model_only(true);
    return request_with_reply(node, "/world/default/control", request, 7000, response, result) && result && response.data();
  };
  auto remove = [&] {
    gz::msgs::Entity request;
    request.set_name("ball"); request.set_type(gz::msgs::Entity::MODEL);
    return request_with_reply(node, "/world/default/remove", request, 7000, response, result) && result && response.data();
  };
  const std::string mode = argv[1];
  auto emit = [&] {
    std::ifstream maps("/proc/self/maps");
    std::ofstream out(argv[3]);
    out << maps.rdbuf();
    if (!out) throw std::runtime_error("cannot record native library mappings");
    std::cout << "{\"mode\":\"" << mode << "\",\"engine\":\"" << engine
        << "\",\"resets\":" << resets << ",\"ball\":" << ball
        << ",\"original\":" << original << ",\"time_ns\":" << lastTime << ",\"pose\":";
    if (pose) {
      std::cout << std::setprecision(17) << '[' << pose->Pos().X() << ','
          << pose->Pos().Y() << ',' << pose->Pos().Z() << ',' << pose->Rot().X()
          << ',' << pose->Rot().Y() << ',' << pose->Rot().Z() << ',' << pose->Rot().W() << ']';
    } else std::cout << "null";
    std::cout << "}\n";
  };
  if (mode == "ordinary") {
    if (!control(true, false) || !server->RunOnce(true)) return 4;
    if (!remove()) return 5;
    if (!(server->RunOnce(true) && server->RunOnce(true))) return 6;
    emit();
    return resets == 1 && ball == original ? 0 : 7;
  }
  if (mode == "overwrite") {
    if (!control(true, false) || !control(false, false) || !(server->RunOnce(true) && server->RunOnce(true))) return 8;
    emit();
    return resets == 0 && ball == original ? 0 : 9;
  }
  if (mode == "model-only") {
    if (!remove() || !(server->RunOnce(true) && server->RunOnce(true)) || ball) return 10;
    if (!control(false, true) || !(server->RunOnce(true) && server->RunOnce(true))) return 11;
    emit();
    return resets == 0 && !ball ? 0 : 12;
  }
  if (mode == "time-only") {
    if (!remove() || !server->RunOnce(true) || !server->RunOnce(true) || ball) return 14;
    if (!control(true, false, 0, true) || !server->RunOnce(true) || !server->RunOnce(true)) return 15;
    emit();
    return resets == 1 && ball == original ? 0 : 16;
  }
  if (mode == "step-reset") {
    if (!control(true, false, 3) || !server->RunOnce(true) || !server->Run(true, 3, false)) return 17;
    emit();
    return resets == 1 && lastTime == 2000000 ? 0 : 18;
  }
  return 13;
}
