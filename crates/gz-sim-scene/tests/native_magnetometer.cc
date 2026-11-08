// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Reference conventions: gz-sensors/test/integration/magnetometer.cc (OSRF 2019)
// and gz-sim/src/systems/magnetometer/Magnetometer.cc (OSRF, Apache-2.0).
// Source only until the owner releases a bounded existing-style native probe.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/sensors/MagnetometerSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Magnetometer.hh>
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
using Message = gz::msgs::Magnetometer;
struct Received
{
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<Message> messages;
};
void subscribe(gz::transport::Node &node, Received &received)
{
  if (!node.Subscribe("/m17/magnetometer", std::function<void(const Message &)>(
      [&](const Message &message)
      {
        std::lock_guard<std::mutex> guard(received.mutex);
        received.messages.push_back(message);
        received.ready.notify_all();
      }))) throw std::runtime_error("magnetometer subscription failed");
}
long long stamp(const Message &message)
{
  return message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
}
using Vector = gz::math::Vector3d;
using Rotation = gz::math::Quaterniond;
const Vector worldField(2e-5, -3e-5, 4e-5);
Vector ideal(const Rotation &q, const Vector &field)
{
  // Independent scalar transpose, matching MagnetometerSensor's inverse rotation.
  const double x=q.X(), y=q.Y(), z=q.Z(), w=q.W();
  return Vector((1-2*(y*y+z*z))*field.X()+2*(x*y+z*w)*field.Y()+2*(x*z-y*w)*field.Z(),
      2*(x*y-z*w)*field.X()+(1-2*(x*x+z*z))*field.Y()+2*(y*z+x*w)*field.Z(),
      2*(x*z+y*w)*field.X()+2*(y*z-x*w)*field.Y()+(1-2*(x*x+y*y))*field.Z());
}
bool finite(const Vector &v) { return std::isfinite(v.X()) && std::isfinite(v.Y()) && std::isfinite(v.Z()); }
bool finite(const Rotation &q) { return std::isfinite(q.X()) && std::isfinite(q.Y()) && std::isfinite(q.Z()) && std::isfinite(q.W()); }
void vector(const Vector &v) { std::cout << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']'; }
void rotation(const Rotation &q) { std::cout << '[' << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W() << ']'; }
Vector measured(const Message &m) { return Vector(m.field_tesla().x(),m.field_tesla().y(),m.field_tesla().z()); }
void fields(const Message &message, unsigned int expectedSequence, long long expectedTime)
{
  std::string frame, sequence;
  unsigned int frames=0, sequences=0;
  for (const auto &data : message.header().data())
  {
    if (data.key()=="frame_id" && data.value_size()==1) { frame=data.value(0); ++frames; }
    if (data.key()=="seq" && data.value_size()==1) { sequence=data.value(0); ++sequences; }
  }
  if (frames!=1 || sequences!=1 || frame!="rig/body/magnetometer" || sequence!=std::to_string(expectedSequence) ||
      stamp(message)!=expectedTime || !message.has_field_tesla() || !finite(measured(message)))
    throw std::runtime_error("unexpected native magnetometer header/measurement");
  std::cout << "\"time_ns\":" << stamp(message) << ",\"frame_id\":\"" << frame
      << "\",\"sequence\":\"" << sequence << "\",\"field_tesla\":";
  vector(measured(message));
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
void model(const std::string &mapsPath)
{
  const std::string xml = "<sdf version='1.12'><model name='rig'><link name='body'>"
      "<sensor name='magnetometer' type='magnetometer'><topic>/m17/magnetometer</topic>"
      "<frame_id>rig/body/magnetometer</frame_id><magnetometer/></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);
  sdf::init(parsed);
  if (!sdf::readString(xml, parsed)) throw std::runtime_error("native model SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::MagnetometerSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor) throw std::runtime_error("native magnetometer creation failed");
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
  struct Case { Vector position; Rotation orientation; Vector field; };
  const Rotation full(.2,-.3,.4);
  const Case cases[] = {
    {Vector::Zero, Rotation::Identity, worldField},
    {Vector(2,-3,4), Rotation(0,0,1.5707963267948966), worldField},
    {Vector(0,0,3), full, worldField},
    {Vector(100,-30,40), full, worldField},
    {Vector::Zero, full, Vector::Zero},
    {Vector::Zero, full, Vector(5.5645e-6,22.8758e-6,-42.3884e-6)},
  };
  for (unsigned int index=0; index<6; ++index)
  {
    const auto &at=cases[index];
    sensor->SetWorldPose(gz::math::Pose3d(at.position,at.orientation));
    sensor->SetWorldMagneticField(at.field);
    if (!sensor->Update(std::chrono::nanoseconds((index+1)*1000000LL)))
      throw std::runtime_error("native model update failed");
    std::unique_lock<std::mutex> guard(received.mutex);
    if (!received.ready.wait_for(guard,5s,[&]{return received.messages.size()>=index+1;}))
      throw std::runtime_error("native model message deadline");
    const auto &message=received.messages[index];
    const auto expected=ideal(at.orientation,at.field);
    if (!finite(expected) || (measured(message)-expected).Length()>1e-15)
      throw std::runtime_error("native model inverse-rotation disagreement");
    std::cout << "{\"mode\":\"model\",\"world_position\":"; vector(at.position);
    std::cout << ",\"world_orientation\":"; rotation(at.orientation);
    std::cout << ",\"field_world\":"; vector(at.field); std::cout << ',';
    fields(message,index,(index+1)*1000000LL);
    std::cout << "}\n";
  }
  std::lock_guard<std::mutex> finalGuard(received.mutex);
  if (received.messages.size()!=6) throw std::runtime_error("unexpected model message count");
  if ((measured(received.messages[2])-measured(received.messages[3])).Length()>1e-15)
    throw std::runtime_error("translation changed uniform magnetic field");
  // Capture while the actual sensor/transport objects remain alive.
  writeMaps(mapsPath);
}

struct Observation { gz::math::Pose3d pose; gz::math::Pose3d link; };
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
  bool magnetometer = false;
  bool worldPose = false;
  bool linkPose = false;
  std::vector<std::uint64_t> sensorTypes;
  std::vector<std::uint64_t> bodyTypes;
};

class Observe final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  public: bool rotating = false;
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
        << ",\"has_magnetometer\":" << (s.magnetometer ? "true" : "false")
        << ",\"has_world_pose\":" << (s.worldPose ? "true" : "false")
        << ",\"has_link_pose\":" << (s.linkPose ? "true" : "false")
        << ",\"required_type_ids\":{\"magnetometer\":" << gz::sim::components::Magnetometer::typeId
        << ",\"name\":" << gz::sim::components::Name::typeId
        << ",\"world_pose\":" << gz::sim::components::WorldPose::typeId

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
      if (!applied && !info.paused)
      {
        ecm.CreateComponent(state.body, gz::sim::components::WorldLinearVelocityReset({0, 0, 3}));
        ecm.CreateComponent(state.body, gz::sim::components::WorldAngularVelocityReset(rotating ? Vector(0,2,0) : Vector::Zero));
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
      // EntityByComponents compares DATA, so Magnetometer() would request an empty
      // sdf::Sensor. Select type presence and the exact name without replacing data.
      ecm.Each<gz::sim::components::Magnetometer, gz::sim::components::Name>(
          [&](const gz::sim::Entity &entity, const gz::sim::components::Magnetometer *,
              const gz::sim::components::Name *name)
          {
            if (name->Data() == "magnetometer") { state.sensor = entity; ++state.sensorMatches; }
            return true;
          });
      state.body = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
      state.sensorExists = ecm.HasEntity(state.sensor);
      state.bodyExists = ecm.HasEntity(state.body);
      types(ecm, state.sensor, state.sensorTypes);
      types(ecm, state.body, state.bodyTypes);
      state.magnetometer = ecm.Component<gz::sim::components::Magnetometer>(state.sensor) != nullptr;
      const auto *pose = ecm.Component<gz::sim::components::WorldPose>(state.sensor);
      auto linkPose = gz::sim::Link(state.body).WorldPose(ecm);
      state.pointFieldsExamined = true;
      state.worldPose = pose != nullptr;
      state.linkPose = linkPose.has_value();
      if (state.sensorMatches != 1) throw std::runtime_error("native magnetometer name/type selection is not unique");
      if (!state.sensorExists || !state.bodyExists || !state.magnetometer || !pose || !linkPose)
        throw std::runtime_error("missing native point observation; see exact presence flags and type IDs");
      bool inserted;
      {
        std::lock_guard<std::mutex> guard(mutex);
        inserted = observations.emplace(state.timeNs, Observation{pose->Data(), *linkPose}).second;
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
      sdf::Plugin("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer")));
  gz::sim::Server server(config);
  auto observe = std::make_shared<Observe>();
  observe->rotating = mode == "rotate";
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
    node.TopicInfo("/m17/magnetometer", publishers, subscribers);
    if (publishers.size() == 1 && publishers[0].MsgTypeName() == "gz.msgs.Magnetometer")
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
  // Capture while Server still owns the Magnetometer/Physics plugin and DART engine.
  captureMaps();
  if (diagnose()) throw std::runtime_error("native observer failed; callback state retained above");
  std::lock_guard<std::mutex> messagesGuard(received.mutex);
  std::lock_guard<std::mutex> stateGuard(observe->mutex);
  if (received.messages.size() != 11 || observe->observations.size() != 100)
    throw std::runtime_error("unexpected native100ms acquisition/frame counts");
  const auto &first = observe->observations.begin()->second;
  const auto &last = observe->observations.rbegin()->second;
  const Rotation initial = Rotation(.2,-.3,.4)*Rotation(.3,.4,-.2);
  const auto firstExpected=ideal(initial,worldField);
  if (!finite(first.pose.Pos()) || !finite(last.pose.Pos()) || !finite(first.pose.Rot()) ||
      (first.pose.Pos()-first.link.Pos()).Length()<1.0 ||
      (last.link.Pos()-first.link.Pos()).Length()<.1 ||
      (ideal(first.pose.Rot(),worldField)-firstExpected).Length()>2e-7 ||
      (mode=="rotate" && (ideal(last.pose.Rot(),worldField)-ideal(first.pose.Rot(),worldField)).Length()<1e-6) ||
      (mode=="translate" && (ideal(last.pose.Rot(),worldField)-ideal(first.pose.Rot(),worldField)).Length()>1e-15))
    throw std::runtime_error("native Physics pose/offset/rotation/translation witness missing");
  unsigned int index = 0;
  for (const auto &message : received.messages)
  {
    const auto at = observe->observations.find(stamp(message));
    if (at == observe->observations.end())
      throw std::runtime_error("native acquisition has no observed world pose");
    const auto &observed=at->second;
    const auto expected=ideal(observed.pose.Rot(),worldField);
    if (!finite(observed.pose.Pos()) || !finite(observed.link.Pos()) || !finite(observed.pose.Rot()) ||
        !finite(expected) || !finite(measured(message)) || (measured(message)-expected).Length()>1e-15)
      throw std::runtime_error("native magnetometer/world-pose disagreement");
    std::cout << "{\"mode\":\"" << mode << "\",\"world_position\":"; vector(observed.pose.Pos());
    std::cout << ",\"world_orientation\":"; rotation(observed.pose.Rot());
    std::cout << ",\"link_position\":"; vector(observed.link.Pos());
    std::cout << ",\"link_orientation\":"; rotation(observed.link.Rot());
    std::cout << ",\"field_world\":"; vector(worldField); std::cout << ',';
    fields(message, index, index == 0 ? 1000000LL : index*10000000LL);
    ++index;
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
  if (mode == "model") model(argv[2]);
  else if (mode == "rotate" || mode == "translate") scene(mode, argv[2]);
  else return 2;
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
