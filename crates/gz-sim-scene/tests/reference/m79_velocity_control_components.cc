// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Test-only public-system component ledger; never steps physics or SystemManager.
#include "VelocityControl.hh"
#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

namespace m79 {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
namespace c = gz::sim::components;
using Six = std::array<double, 6>;
void need(bool value, const char *why) { if (!value) throw std::runtime_error(why); }
std::string quote(const std::string &s) {
  need(s.size() <= 32768, "string cap");
  constexpr char h[] = "0123456789abcdef";
  std::string out = "\"";
  for (unsigned char ch : s) {
    need(ch < 128, "ASCII ledger");
    if (ch == '"' || ch == '\\') { out += '\\'; out += static_cast<char>(ch); }
    else if (ch < 32) { out += "\\u00"; out += h[ch >> 4]; out += h[ch & 15]; }
    else out += static_cast<char>(ch);
  }
  return out + '"';
}
std::string bits(double value) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "f64");
  need(std::isfinite(value), "finite component");
  std::uint64_t word; std::memcpy(&word, &value, 8);
  std::ostringstream out; out << std::hex << std::setw(16) << std::setfill('0') << word;
  return quote(out.str());
}
std::string vector(const gz::math::Vector3d &v) {
  return "[" + bits(v.X()) + ',' + bits(v.Y()) + ',' + bits(v.Z()) + ']';
}
std::string wire(const gz::msgs::Twist &msg) {
  need(msg.ByteSizeLong() <= 512, "Twist cap");
  constexpr char h[] = "0123456789abcdef"; std::string out;
  for (unsigned char ch : msg.SerializeAsString()) { out += h[ch >> 4]; out += h[ch & 15]; }
  return quote(out);
}
struct Ledger {
  std::uint64_t rows = 0, bytes = 0;
  Clock::time_point began = Clock::now();
  void emit(const std::string &kind, const std::string &fields) {
    need(Clock::now() - began < 20s, "helper active deadline");
    const auto row = "{\"schema\":\"M79_NATIVE_COMPONENT_LEDGER_V1\",\"ordinal\":" +
      std::to_string(rows) + ",\"kind\":" + quote(kind) + fields + "}\n";
    need(row.size() <= 16384 && rows < 4096 && bytes + row.size() <= 16u*1024u*1024u, "ledger caps");
    std::cout << row; std::cout.flush(); need(bool(std::cout), "actual stdout write");
    ++rows; bytes += row.size();
  }
};
struct State {
  std::optional<gz::math::Vector3d> linear, angular;
  bool matches(const Six &v) const {
    return linear && angular && linear->X() == v[0] && linear->Y() == v[1] &&
      linear->Z() == v[2] && angular->X() == v[3] && angular->Y() == v[4] && angular->Z() == v[5];
  }
  std::string json() const {
    return "{\"linear\":" + (linear ? vector(*linear) : "null") +
      ",\"angular\":" + (angular ? vector(*angular) : "null") + '}';
  }
};
State state(const gz::sim::EntityComponentManager &ecm, gz::sim::Entity model) {
  State out;
  if (const auto *v = ecm.Component<c::LinearVelocityCmd>(model)) out.linear = v->Data();
  if (const auto *v = ecm.Component<c::AngularVelocityCmd>(model)) out.angular = v->Data();
  return out;
}
void maps(const std::filesystem::path &path) {
  std::ifstream input("/proc/self/maps"); need(bool(input), "maps open");
  std::string text, line;
  while (std::getline(input, line)) {
    need(line.find("libgz-sim-velocity-control-system.so") == std::string::npos,
      "expected linked original object, not VelocityControl DSO");
    need(text.size() + line.size() + 1 <= 2u*1024u*1024u, "maps cap"); text += line + '\n';
  }
  need(input.eof(), "complete maps");
  const int fd = ::open(path.c_str(), O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC, 0600);
  need(fd >= 0, "exclusive owned maps"); std::size_t n = 0;
  while (n < text.size()) { const auto w = ::write(fd, text.data()+n, text.size()-n);
    if (w <= 0) { ::close(fd); throw std::runtime_error("maps write"); } n += static_cast<std::size_t>(w); }
  const bool flushed = ::fsync(fd) == 0, closed = ::close(fd) == 0;
  need(flushed && closed, "durable maps");
}
struct Tap {
  std::mutex mutex; std::condition_variable changed;
  bool active = true, armed = false, bad = false;
  std::string expected; std::vector<std::string> wires;
  unsigned received = 0;
};
void run(const std::string &mode, const std::string &name, const std::filesystem::path &mapPath) {
  need(mode == "defaults" || mode == "explicit", "closed mode");
  need(name.size() == 28 && name.rfind("m79_", 0) == 0 &&
    name.substr(4).find_first_not_of("0123456789abcdef") == std::string::npos, "owned model name");
  need(mapPath.is_absolute() && !std::filesystem::exists(mapPath), "owned new maps path");
  need(gz::common::Console::Init(mapPath.parent_path().string(), mapPath.filename().string()+".console.log"), "owned Console");
  gz::common::Console::SetVerbosity(2);
  need(gz::common::Console::Directory() == mapPath.parent_path().string() && gz::common::Console::Verbosity() == 2, "Console getback");
  Ledger out; gz::sim::EntityComponentManager ecm; gz::sim::EventManager events;
  const auto world = ecm.CreateEntity(); ecm.CreateComponent(world, c::World());
  ecm.CreateComponent(world, c::Name("default"));
  const auto model = ecm.CreateEntity(); ecm.CreateComponent(model, c::Model());
  ecm.CreateComponent(model, c::Name(name)); need(ecm.SetParentEntity(model, world), "actual model graph parent");
  const gz::math::Pose3d pose(3, -2, 5, 0.1, -0.2, 0.3);
  ecm.CreateComponent(model, c::Pose(pose));
  const auto link = ecm.CreateEntity(); ecm.CreateComponent(link, c::Link());
  ecm.CreateComponent(link, c::Name("body")); need(ecm.SetParentEntity(link, model), "actual link graph parent");
  ecm.CreateComponent(link, c::Pose(gz::math::Pose3d::Zero));
  const std::string topic = mode == "defaults" ? "/model/"+name+"/cmd_vel" : "/m79/"+name+"/custom_velocity";
  const std::string content = mode == "defaults" ? "" :
    "<topic>"+topic+"</topic><initial_linear>0.25 -0.5 0.75</initial_linear><initial_angular>-0.125 0.375 -0.625</initial_angular>";
  sdf::Plugin plugin("gz-sim-velocity-control-system", "gz::sim::systems::VelocityControl", content);
  const auto element = plugin.ToElement(); need(bool(element), "parsed plugin");
  std::unique_ptr<gz::sim::System> system(new gz::sim::systems::VelocityControl());
  auto *configure = dynamic_cast<gz::sim::ISystemConfigure *>(system.get());
  auto *pre = dynamic_cast<gz::sim::ISystemPreUpdate *>(system.get());
  auto *post = dynamic_cast<gz::sim::ISystemPostUpdate *>(system.get());
  need(configure && pre && post, "public system interfaces");
  out.emit("configure_call", ",\"mode\":"+quote(mode)+",\"model_name\":"+quote(name)+
    ",\"world_id\":"+std::to_string(world)+",\"model_id\":"+std::to_string(model)+
    ",\"link_id\":"+std::to_string(link)+",\"topic\":"+quote(topic)+
    ",\"xml\":"+quote(element->ToString(""))+",\"commands_before\":"+state(ecm, model).json()+
    ",\"model_position\":"+vector(pose.Pos())+",\"model_orientation_xyzw\":["+
    bits(pose.Rot().X())+','+bits(pose.Rot().Y())+','+bits(pose.Rot().Z())+','+bits(pose.Rot().W())+']');
  configure->Configure(model, element, ecm, events);
  out.emit("configure_return", ",\"commands_after\":"+state(ecm, model).json());
  auto tapped = std::make_shared<Tap>(); gz::transport::Node node;
  std::function<void(const gz::msgs::Twist &)> callback = [tapped](const auto &message) {
    std::lock_guard<std::mutex> lock(tapped->mutex); if (!tapped->active) return;
    const auto body = message.SerializeAsString();
    if (!tapped->armed || body.size() > 512 || body != tapped->expected || !tapped->wires.empty() || tapped->received >= 3) tapped->bad = true;
    else { tapped->wires.push_back(body); ++tapped->received; }
    tapped->changed.notify_all();
  };
  need(node.Subscribe(topic, callback), "typed tap");
  auto publisher = node.Advertise<gz::msgs::Twist>(topic); need(bool(publisher), "typed publisher");
  const auto discoveryDeadline = Clock::now()+5s;
  std::size_t pubCount = 0, subCount = 0;
  for (;;) {
    std::vector<gz::transport::MessagePublisher> pubs, subs;
    if (node.TopicInfo(topic, pubs, subs) && pubs.size() == 1 && subs.size() == 2 && publisher.HasConnections()) {
      bool valid = true;
      for (const auto &p : pubs) valid = valid && p.MsgTypeName() == "gz.msgs.Twist";
      for (const auto &s : subs) valid = valid && s.MsgTypeName() == "gz.msgs.Twist";
      if (valid) { pubCount = pubs.size(); subCount = subs.size(); break; }
    }
    need(Clock::now() < discoveryDeadline, "discovery deadline"); std::this_thread::yield();
  }
  out.emit("discovery", ",\"publishers\":"+std::to_string(pubCount)+",\"subscribers\":"+std::to_string(subCount)+
    ",\"type\":\"gz.msgs.Twist\",\"native_callback_receipt\":\"UNKNOWN\"");
  std::uint64_t call = 0, iteration = 0; std::int64_t time = 0;
  std::optional<Clock::time_point> phaseDeadline;
  auto update = [&](bool isPre, bool paused, const std::string &reason) {
    if (phaseDeadline) need(Clock::now() < *phaseDeadline, "phase call deadline");
    gz::sim::UpdateInfo info; info.paused = paused; info.iterations = iteration;
    info.simTime = std::chrono::nanoseconds(time); info.realTime = info.simTime;
    info.dt = paused ? 0ns : 1ms;
    need(ecm.Component<c::Name>(model) && ecm.Component<c::ParentEntity>(model) &&
      ecm.Component<c::Pose>(model) && ecm.Component<c::Pose>(link) && ecm.Component<c::ParentEntity>(link), "graph input presence");
    const auto before = state(ecm, model).json();
    out.emit("update_call", ",\"call\":"+std::to_string(call)+",\"interface\":"+quote(isPre?"PreUpdate":"PostUpdate")+
      ",\"reason\":"+quote(reason)+",\"paused\":"+(paused?"true":"false")+
      ",\"iterations\":"+std::to_string(iteration)+",\"sim_time_ns\":"+std::to_string(time)+
      ",\"real_time_ns\":"+std::to_string(time)+
      ",\"dt_ns\":"+std::to_string(paused?0:1000000)+",\"commands_before\":"+before);
    if (isPre) pre->PreUpdate(info, ecm); else post->PostUpdate(info, ecm);
    if (phaseDeadline) need(Clock::now() < *phaseDeadline, "returned phase call deadline");
    const auto after = state(ecm, model);
    out.emit("update_return", ",\"call\":"+std::to_string(call)+",\"commands_after\":"+after.json()); ++call;
    need(ecm.Component<c::Pose>(model)->Data() == pose && ecm.Component<c::ParentEntity>(link)->Data() == model &&
      ecm.Component<c::Name>(model)->Data() == name && ecm.Component<c::ParentEntity>(model)->Data() == world &&
      ecm.Component<c::Pose>(link)->Data() == gz::math::Pose3d::Zero,
      "no model pose/graph mutation");
    if (paused) need(before == after.json(), "actual paused components held");
    return after;
  };
  const Six initial = mode == "defaults" ? Six{} : Six{0.25,-0.5,0.75,-0.125,0.375,-0.625};
  update(true, true, "initial_pause"); update(false, true, "initial_pause");
  need(update(true, false, "initial").matches(initial), "configured initial vectors"); update(false, false, "initial");
  auto publish = [&](const Six &value, bool omitLinear, unsigned phase) {
    gz::msgs::Twist message;
    if (!omitLinear) { auto *v = message.mutable_linear(); v->set_x(value[0]); v->set_y(value[1]); v->set_z(value[2]); }
    auto *v = message.mutable_angular(); v->set_x(value[3]); v->set_y(value[4]); v->set_z(value[5]);
    { std::lock_guard<std::mutex> lock(tapped->mutex); need(!tapped->bad && !tapped->armed && tapped->wires.empty(), "one publication");
      tapped->expected = message.SerializeAsString(); tapped->armed = true; }
    const auto deadline = Clock::now()+5s; phaseDeadline = deadline; need(publisher.Publish(message), "canonical Publish");
    out.emit("publication", ",\"phase\":"+std::to_string(phase)+",\"wire_hex\":"+wire(message)+
      ",\"linear_present\":"+(message.has_linear()?"true":"false")+",\"angular_present\":true,\"publish_returned_true\":true");
    { std::unique_lock<std::mutex> lock(tapped->mutex);
      need(tapped->changed.wait_until(lock, deadline, [&]{return tapped->bad || !tapped->wires.empty();}), "tap deadline");
      need(!tapped->bad && tapped->wires.size() == 1 && tapped->wires.front() == message.SerializeAsString(), "exact canonical tap");
      tapped->wires.clear(); }
    out.emit("tap", ",\"phase\":"+std::to_string(phase)+",\"wire_hex\":"+wire(message)+
      ",\"evidence_scope\":\"tap-is-not-native-private-callback-receipt\"");
    return deadline;
  };
  auto fence = [&](const Six &wanted, unsigned phase, Clock::time_point deadline) {
    unsigned probes = 0; bool observed = false;
    for (; probes < 32; ++probes) {
      need(Clock::now() < deadline, "component effect deadline");
      ++iteration; time += 1000000;
      const auto after = update(true, false, "component_effect_probe"); update(false, false, "component_effect_probe");
      { std::lock_guard<std::mutex> lock(tapped->mutex); need(!tapped->bad && tapped->wires.empty(), "no duplicate tap"); }
      if (after.matches(wanted)) { observed = true; ++probes; break; } std::this_thread::yield();
    }
    need(observed, "bounded original component effect not observed");
    { std::lock_guard<std::mutex> lock(tapped->mutex); tapped->armed = false; }
    out.emit("effect_fence", ",\"phase\":"+std::to_string(phase)+",\"actual_probe_rounds\":"+std::to_string(probes)+
      ",\"completion\":\"OBSERVED_PUBLIC_COMPONENT_EFFECT\",\"native_callback_receipt_time\":\"UNKNOWN\"");
    need(Clock::now() < deadline, "actual fence completion deadline"); phaseDeadline.reset();
  };
  const Six first{1.25,-2.5,3.75,-0.5,1.,-1.5}; const auto firstDeadline = publish(first, false, 0);
  need(update(true, false, "first_pre_before_post_latch").matches(initial), "previous latch before first PostUpdate");
  update(false, false, "first_post_after_publication"); fence(first, 0, firstDeadline);
  const Six zero{}; const auto zeroDeadline = publish(zero, false, 1);
  for (unsigned n=0; n<3; ++n) { update(true, true, "paused_after_publication"); update(false, true, "paused_after_publication"); }
  need(update(true, false, "first_resume_pre").matches(first), "old committed target at first resume PreUpdate");
  update(false, false, "first_resume_post"); fence(zero, 1, zeroDeadline);
  const Six partial{0.,0.,0.,0.25,-0.75,1.5}; const auto partialDeadline = publish(partial, true, 2);
  need(update(true, false, "partial_pre_before_post_latch").matches(zero), "old zero latch before PostUpdate");
  update(false, false, "partial_post"); fence(partial, 2, partialDeadline);
  for (unsigned n=0;n<3;++n) { ++iteration; time+=1000000;
    need(update(true, false, "persistent_target").matches(partial), "persistent target"); update(false, false, "persistent_target"); }
  maps(mapPath);
  { std::lock_guard<std::mutex> lock(tapped->mutex); need(!tapped->bad && tapped->received == 3 && tapped->wires.empty(), "complete tap ledger"); tapped->active = false; }
  need(node.Unsubscribe(topic), "retire owned tap");
  out.emit("summary", ",\"state\":\"NATIVE_PUBLIC_COMPONENT_OBSERVATIONS_PENDING_ACTUAL_EXIT_AND_DATA\",\"actual_update_calls\":"+
    std::to_string(call)+",\"publications\":3,\"taps\":3,\"physics_stepped\":false,\"SystemManager_reset_executed\":false,\"native_callback_quiescence\":\"UNKNOWN\"");
}
}  // namespace m79
int main(int argc, char **argv) {
  if (!std::getenv("M79_NATIVE_COMPONENT_CAPTURE_ROOT_RELEASED") ||
      std::string(std::getenv("M79_NATIVE_COMPONENT_CAPTURE_ROOT_RELEASED")) != "1") {
    std::cerr << "HELD: whole Source review and Root serial release required\n"; return 2;
  }
  try { m79::need(argc == 4, "expected MODE OWNED_MODEL_NAME NEW_MAPS_PATH");
    m79::run(argv[1], argv[2], argv[3]); return 0;
  } catch (const std::exception &error) { std::cerr << "M79 partial native component ledger: " << error.what() << '\n'; return 1; }
}
