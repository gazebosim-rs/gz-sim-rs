// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Reference conventions: gz-sensors/test/integration/altimeter.cc (OSRF 2019)
// and gz-sim/src/systems/altimeter/Altimeter.cc (OSRF, Apache-2.0).
// Source only until the owner releases a bounded existing-style native probe.
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/msgs/altimeter.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/sensors/AltimeterSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Altimeter.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <sdf/sdf.hh>

using namespace std::chrono_literals;
using Message = gz::msgs::Altimeter;
struct Received
{
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<Message> messages;
};
void subscribe(gz::transport::Node &node, Received &received)
{
  if (!node.Subscribe("/m13/altimeter", std::function<void(const Message &)>(
      [&](const Message &message)
      {
        std::lock_guard<std::mutex> guard(received.mutex);
        received.messages.push_back(message);
        received.ready.notify_all();
      }))) throw std::runtime_error("altimeter subscription failed");
}
long long stamp(const Message &message)
{
  return message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
}
void fields(const Message &message)
{
  std::string frame;
  std::string sequence;
  for (const auto &data : message.header().data())
  {
    if (data.key() == "frame_id" && data.value_size() == 1) frame = data.value(0);
    if (data.key() == "seq" && data.value_size() == 1) sequence = data.value(0);
  }
  if (frame != "rig/body/altimeter" || sequence.empty() ||
      sequence.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("unexpected native altimeter header");
  std::cout << "\"time_ns\":" << stamp(message) << ",\"frame_id\":\"" << frame
      << "\",\"sequence\":\"" << sequence << "\",\"vertical_position\":"
      << message.vertical_position() << ",\"vertical_velocity\":"
      << message.vertical_velocity() << ",\"vertical_reference\":" << message.vertical_reference();
}
void writeMaps(const std::string &path)
{
  std::ifstream maps("/proc/self/maps");
  std::ofstream output(path);
  if (!maps || !output) throw std::runtime_error("native live maps open failed");
  output << maps.rdbuf();
  output.flush();
  if (!output) throw std::runtime_error("native live maps write failed");
}
void model()
{
  // Exact native public SensorReadings cases, plus a negative reference.
  const std::string xml = "<sdf version='1.12'><model name='rig'><link name='body'>"
      "<sensor name='altimeter' type='altimeter'><topic>/m13/altimeter</topic>"
      "<frame_id>rig/body/altimeter</frame_id><altimeter/></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);
  sdf::init(parsed);
  if (!sdf::readString(xml, parsed)) throw std::runtime_error("native model SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::AltimeterSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor) throw std::runtime_error("native altimeter creation failed");
  Received received;
  gz::transport::Node node;
  subscribe(node, received);
  const auto discovery = std::chrono::steady_clock::now() + 5s;
  while (!sensor->HasConnections())
  {
    if (std::chrono::steady_clock::now() >= discovery)
      throw std::runtime_error("native model connection deadline");
    std::this_thread::sleep_for(1ms);
  }
  const double inputs[][3] = {{2, 3, 1}, {-6, 3, 1}, {3.5, 0, 3.5}, {-4, -2, -5}};
  for (unsigned int index = 0; index < 4; ++index)
  {
    const auto &input = inputs[index];
    sensor->SetVerticalReference(input[2]);
    sensor->SetPosition(input[0]);
    sensor->SetVerticalVelocity(input[1]);
    if (!sensor->Update(std::chrono::nanoseconds((index + 1)*1000000LL)))
      throw std::runtime_error("native model update failed");
    std::unique_lock<std::mutex> guard(received.mutex);
    if (!received.ready.wait_for(guard, 5s, [&]{return received.messages.size() >= index+1;}))
      throw std::runtime_error("native model message deadline");
    const auto &message = received.messages[index];
    if (message.vertical_position() != input[0]-input[2] ||
        message.vertical_velocity() != input[1] || message.vertical_reference() != input[2])
      throw std::runtime_error("native setter/output disagreement");
    std::cout << "{\"mode\":\"model\",\"world_z\":" << input[0]
        << ",\"world_velocity_z\":" << input[1] << ',';
    fields(message);
    std::cout << "}\n";
  }
}

struct Observation { double worldZ; double pointVz; double linkVz; };
struct CallbackState
{
  const char *phase = "not-entered";
  long long timeNs = 0;
  long long dtNs = 0;
  std::uint64_t iteration = 0;
  bool paused = false;
  gz::sim::Entity sensor = gz::sim::kNullEntity;
  gz::sim::Entity body = gz::sim::kNullEntity;
  std::size_t sensorMatches = 0;
  bool sensorExists = false;
  bool bodyExists = false;
  bool pointFieldsExamined = false;
  bool altimeter = false;
  bool worldPose = false;
  bool pointVelocity = false;
  bool linkVelocity = false;
  std::vector<std::uint64_t> sensorTypes;
  std::vector<std::uint64_t> bodyTypes;
};

class Observe final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  public: bool moving = false;
  public: bool applied = false;
  public: std::map<long long, Observation> observations;
  public: std::mutex mutex;
  public: std::uint64_t preEntries = 0;
  public: std::uint64_t postEntries = 0;
  public: std::string failure;
  public: CallbackState failedState;
  private: CallbackState enter(const char *phase, const gz::sim::UpdateInfo &info)
  {
    CallbackState state;
    state.phase = phase;
    state.timeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    state.dtNs = std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    state.iteration = info.iterations;
    state.paused = info.paused;
    return state;
  }
  private: void fail(const CallbackState &state, const std::string &error)
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (failure.empty())
    {
      failure = error.substr(0, 2048);
      failedState = state;
    }
  }
  private: static void types(const gz::sim::EntityComponentManager &ecm,
      gz::sim::Entity entity, std::vector<std::uint64_t> &result)
  {
    for (const auto type : ecm.ComponentTypes(entity)) result.push_back(type);
    std::sort(result.begin(), result.end());
  }
  public: bool diagnostic(std::ostream &output)
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (failure.empty()) return false;
    const auto &s = failedState;
    output << "NATIVE_OBSERVER_FAILURE {\"phase\":\"" << s.phase
        << "\",\"time_ns\":" << s.timeNs << ",\"dt_ns\":" << s.dtNs
        << ",\"iteration\":" << s.iteration << ",\"paused\":" << (s.paused ? "true" : "false")
        << ",\"pre_entries\":" << preEntries << ",\"post_entries\":" << postEntries
        << ",\"sensor_entity\":" << s.sensor << ",\"body_entity\":" << s.body
        << ",\"sensor_matches\":" << s.sensorMatches
        << ",\"sensor_exists\":" << (s.sensorExists ? "true" : "false")
        << ",\"body_exists\":" << (s.bodyExists ? "true" : "false")
        << ",\"point_fields_examined\":" << (s.pointFieldsExamined ? "true" : "false")
        << ",\"has_altimeter\":" << (s.altimeter ? "true" : "false")
        << ",\"has_world_pose\":" << (s.worldPose ? "true" : "false")
        << ",\"has_point_velocity\":" << (s.pointVelocity ? "true" : "false")
        << ",\"has_link_velocity\":" << (s.linkVelocity ? "true" : "false")
        << ",\"required_type_ids\":{\"altimeter\":" << gz::sim::components::Altimeter::typeId
        << ",\"name\":" << gz::sim::components::Name::typeId
        << ",\"world_pose\":" << gz::sim::components::WorldPose::typeId
        << ",\"world_linear_velocity\":" << gz::sim::components::WorldLinearVelocity::typeId
        << "},\"sensor_component_type_ids\":[";
    for (std::size_t i = 0; i < s.sensorTypes.size(); ++i)
    { if (i) output << ','; output << s.sensorTypes[i]; }
    output << "],\"body_component_type_ids\":[";
    for (std::size_t i = 0; i < s.bodyTypes.size(); ++i)
    { if (i) output << ','; output << s.bodyTypes[i]; }
    output << "],\"error\":\"";
    for (const unsigned char c : failure)
    {
      if (c == '\\' || c == '"') output << '\\' << static_cast<char>(c);
      else if (c < 32) output << '?';
      else output << static_cast<char>(c);
    }
    output << "\"}\n";
    return true;
  }
  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    CallbackState state = enter("PreUpdate", info);
    {
      std::lock_guard<std::mutex> guard(mutex);
      ++preEntries;
      if (!failure.empty()) return;
    }
    try
    {
      state.body = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
      state.bodyExists = ecm.HasEntity(state.body);
      types(ecm, state.body, state.bodyTypes);
      if (!state.bodyExists) throw std::runtime_error("missing native link");
      gz::sim::Link link(state.body);
      link.EnableVelocityChecks(ecm);
      if (moving && !applied && !info.paused)
      {
        ecm.CreateComponent(state.body, gz::sim::components::WorldLinearVelocityReset({0, 0, 3}));
        ecm.CreateComponent(state.body, gz::sim::components::WorldAngularVelocityReset({0, 2, 0}));
        applied = true;
      }
    }
    catch (const std::exception &error) { fail(state, error.what()); }
    catch (...) { fail(state, "unknown PreUpdate exception"); }
  }
  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    CallbackState state = enter("PostUpdate", info);
    {
      std::lock_guard<std::mutex> guard(mutex);
      ++postEntries;
      if (!failure.empty()) return;
    }
    if (info.paused) return;
    try
    {
      // EntityByComponents compares DATA, so Altimeter() would request an empty
      // sdf::Sensor. Select type presence and the exact name without replacing data.
      ecm.Each<gz::sim::components::Altimeter, gz::sim::components::Name>(
          [&](const gz::sim::Entity &entity, const gz::sim::components::Altimeter *,
              const gz::sim::components::Name *name)
          {
            if (name->Data() == "altimeter") { state.sensor = entity; ++state.sensorMatches; }
            return true;
          });
      state.body = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
      state.sensorExists = ecm.HasEntity(state.sensor);
      state.bodyExists = ecm.HasEntity(state.body);
      types(ecm, state.sensor, state.sensorTypes);
      types(ecm, state.body, state.bodyTypes);
      state.altimeter = ecm.Component<gz::sim::components::Altimeter>(state.sensor) != nullptr;
      const auto *pose = ecm.Component<gz::sim::components::WorldPose>(state.sensor);
      const auto *velocity = ecm.Component<gz::sim::components::WorldLinearVelocity>(state.sensor);
      auto linkVelocity = gz::sim::Link(state.body).WorldLinearVelocity(ecm);
      state.pointFieldsExamined = true;
      state.worldPose = pose != nullptr;
      state.pointVelocity = velocity != nullptr;
      state.linkVelocity = linkVelocity.has_value();
      if (state.sensorMatches != 1) throw std::runtime_error("native altimeter name/type selection is not unique");
      if (!state.sensorExists || !state.bodyExists || !state.altimeter || !pose || !velocity || !linkVelocity)
        throw std::runtime_error("missing native point observation; see exact presence flags and type IDs");
      bool inserted;
      {
        std::lock_guard<std::mutex> guard(mutex);
        inserted = observations.emplace(state.timeNs, Observation{pose->Data().Pos().Z(),
            velocity->Data().Z(), linkVelocity->Z()}).second;
      }
      if (!inserted) throw std::runtime_error("duplicate advancing native timestamp");
    }
    catch (const std::exception &error) { fail(state, error.what()); }
    catch (...) { fail(state, "unknown PostUpdate exception"); }
  }
};

void scene(const std::string &mode, const std::string &mapsPath)
{
  std::string xml;
  char value;
  while (std::cin.get(value))
  {
    if (xml.size() >= 256*1024) throw std::runtime_error("bounded fixture exceeded");
    xml += value;
  }
  Received received;
  gz::transport::Node node;
  subscribe(node, received);
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) throw std::runtime_error("native scene SDF refused");
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-physics-system", "gz::sim::systems::Physics")));
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default", "world",
      sdf::Plugin("gz-sim-altimeter-system", "gz::sim::systems::Altimeter")));
  gz::sim::Server server(config);
  auto observe = std::make_shared<Observe>();
  observe->moving = mode == "offset";
  bool mapsAttempted = false;
  bool diagnosticPrinted = false;
  const auto captureMaps = [&]
  {
    if (!mapsAttempted) { mapsAttempted = true; writeMaps(mapsPath); }
  };
  const auto diagnose = [&]
  {
    if (diagnosticPrinted) return true;
    diagnosticPrinted = observe->diagnostic(std::cerr);
    return diagnosticPrinted;
  };
  try
  {
  if (server.AddSystem(observe) != std::optional<bool>(true))
    throw std::runtime_error("native observer setup failed");
  // Match the explicit startup reference: create while paused, then advance100ms.
  if (!server.Run(false, 100, true)) throw std::runtime_error("native paused start failed");
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::vector<gz::transport::MessagePublisher> publishers, subscribers;
    node.TopicInfo("/m13/altimeter", publishers, subscribers);
    if (publishers.size() == 1 && publishers[0].MsgTypeName() == "gz.msgs.Altimeter")
    { found = true; break; }
    std::this_thread::sleep_for(1ms);
  }
  if (!found) throw std::runtime_error("native discovery deadline");
  gz::msgs::WorldControl request;
  gz::msgs::Boolean response;
  bool accepted = false;
  request.set_pause(false);
  if (!node.Request("/world/default/control", request, 5000, response, accepted)
      || !accepted || !response.data()) throw std::runtime_error("native resume failed");
  {
    std::unique_lock<std::mutex> guard(received.mutex);
    if (!received.ready.wait_for(guard, 5s, [&]{return received.messages.size() >= 11;}))
      throw std::runtime_error("native acquisition deadline");
  }
  const auto completion = std::chrono::steady_clock::now() + 5s;
  while (server.Running())
  {
    if (std::chrono::steady_clock::now() >= completion)
      throw std::runtime_error("native finite run deadline");
    std::this_thread::sleep_for(1ms);
  }
  server.Stop();
  // Capture while Server still owns the Altimeter/Physics plugin and DART engine.
  captureMaps();
  if (diagnose()) throw std::runtime_error("native observer failed; callback state retained above");
  std::lock_guard<std::mutex> messagesGuard(received.mutex);
  std::lock_guard<std::mutex> stateGuard(observe->mutex);
  if (received.messages.size() != 11 || observe->observations.size() != 100)
    throw std::runtime_error("unexpected native100ms acquisition/frame counts");
  for (const auto &message : received.messages)
  {
    const auto at = observe->observations.find(stamp(message));
    if (at == observe->observations.end() || message.vertical_reference() != 3.5
        || message.vertical_position() != at->second.worldZ-3.5
        || message.vertical_velocity() != at->second.pointVz)
      throw std::runtime_error("native sensor/world-point disagreement");
    std::cout << "{\"mode\":\"" << mode << "\",\"world_z\":" << at->second.worldZ
        << ",\"point_velocity_z\":" << at->second.pointVz << ",\"link_velocity_z\":" << at->second.linkVz << ',';
    fields(message);
    std::cout << "}\n";
  }
  }
  catch (...)
  {
    const auto original = std::current_exception();
    try { server.Stop(); }
    catch (const std::exception &error) { std::cerr << "native stop during failure: " << error.what() << '\n'; }
    catch (...) { std::cerr << "unknown native stop failure\n"; }
    // Diagnostics must survive a failed maps write; neither error replaces the
    // original callback/measurement exception, and no partial maps is retried.
    try { diagnose(); }
    catch (const std::exception &error) { std::cerr << "native diagnostic failure: " << error.what() << '\n'; }
    catch (...) { std::cerr << "unknown native diagnostic failure\n"; }
    try { captureMaps(); }
    catch (const std::exception &error) { std::cerr << "native live maps failure: " << error.what() << '\n'; }
    catch (...) { std::cerr << "unknown native live maps failure\n"; }
    std::rethrow_exception(original);
  }
}

int main(int argc, char **argv)
try
{
  if (argc != 3) return 2;
  std::cout << std::setprecision(17);
  const std::string mode = argv[1];
  if (mode == "model") { model(); writeMaps(argv[2]); }
  else if (mode == "offset" || mode == "fall") scene(mode, argv[2]);
  else return 2;
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
