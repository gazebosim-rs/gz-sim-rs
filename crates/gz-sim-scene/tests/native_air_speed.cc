// Copyright (C) 2019, 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Reference conventions: gz-sensors/test/integration/air_speed.cc (OSRF 2023)
// and gz-sim/src/systems/air_speed/AirSpeed.cc (OSRF, Apache-2.0).
// Numeric formula follows pinned AirSpeedSensor.cc; its wire field comment
// says hPa but the actual final float *100 publishes a Pa-scale number.
// Source only until the owner releases a bounded existing-style native probe.
#include <algorithm>
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
#include <gz/msgs/air_speed.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/sensors/AirSpeedSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AirSpeedSensor.hh>
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
using Message = gz::msgs::AirSpeed;
struct Received
{
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<Message> messages;
};
void subscribe(gz::transport::Node &node, Received &received)
{
  if (!node.Subscribe("/m26/air_speed", std::function<void(const Message &)>(
      [&](const Message &message)
      {
        std::lock_guard<std::mutex> guard(received.mutex);
        received.messages.push_back(message);
        received.ready.notify_all();
      }))) throw std::runtime_error("air_speed subscription failed");
}
long long stamp(const Message &message)
{
  return message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
}
struct Reading { double pressure; double temperature; };
Reading ideal(double height, double sensorX)
{
  const float z = static_cast<float>(height);
  const float temperature = 288.15f - .0065f*z;
  const float density = 1.225f / ::powf(288.15f/temperature, 4.256f);
  const float coefficient = (sensorX > 0 ? 1.0f : sensorX < 0 ? -1.0f : 0.0f) * .005f * density;
  const float intermediate = static_cast<float>(static_cast<double>(coefficient)*sensorX*sensorX);
  return {static_cast<double>(intermediate*100.0f), static_cast<double>(temperature)};
}
void vector(std::ostream &out, const gz::math::Vector3d &v)
{ out << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']'; }
bool finite(const gz::math::Vector3d &v)
{ return std::isfinite(v.X()) && std::isfinite(v.Y()) && std::isfinite(v.Z()); }
void fields(const Message &message, unsigned int expectedSequence, long long expectedTime)
{
  std::string frame;
  std::string sequence;
  for (const auto &data : message.header().data())
  {
    if (data.key() == "frame_id" && data.value_size() == 1) frame = data.value(0);
    if (data.key() == "seq" && data.value_size() == 1) sequence = data.value(0);
  }
  if (frame != "rig/body/air_speed" || sequence != std::to_string(expectedSequence) ||
      stamp(message) != expectedTime ||
      !std::isfinite(message.diff_pressure()) || !std::isfinite(message.temperature()) || message.temperature() <= 0)
    throw std::runtime_error("unexpected native pressure header/measurement");
  std::cout << "\"time_ns\":" << stamp(message) << ",\"frame_id\":\"" << frame
      << "\",\"sequence\":\"" << sequence << "\",\"diff_pressure\":"
      << message.diff_pressure() << ",\"temperature\":" << message.temperature();
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
      "<sensor name='air_speed' type='air_speed'><topic>/m26/air_speed</topic>"
      "<frame_id>rig/body/air_speed</frame_id><update_rate>0</update_rate><air_speed/></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF); sdf::init(parsed);
  if (!sdf::readString(xml, parsed)) throw std::runtime_error("native model SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::AirSpeedSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  Received received; gz::transport::Node node; bool mapsAttempted = false;
  const auto capture = [&] { if (!mapsAttempted) { mapsAttempted = true; writeMaps(mapsPath); } };
  try
  {
    if (!sensor) throw std::runtime_error("native AirSpeed SensorFactory creation failed");
    subscribe(node, received);
    const auto discovery=std::chrono::steady_clock::now()+5s;
    while (!sensor->HasConnections()) {
      if (std::chrono::steady_clock::now()>=discovery) throw std::runtime_error("native model connection deadline");
      std::this_thread::sleep_for(1ms);
    }
    const double heights[]={0,0,0,0,1000,-100};
    const gz::math::Vector3d velocities[]={{0,0,0},{10,0,0},{-10,0,0},{0,10,0},{10,0,0},{3.125,2,-4}};
    sensor->SetWindVelocity({0,0,0});
    for (unsigned index=0;index<6;++index) {
      sensor->SetPose(gz::math::Pose3d(0,0,heights[index],.2,-.3,.4));
      sensor->SetVelocity(velocities[index]);
      const long long time=(index+1)*1000000LL;
      if (!sensor->Update(std::chrono::nanoseconds(time))) throw std::runtime_error("native model update refused");
      std::unique_lock<std::mutex> lock(received.mutex);
      if (!received.ready.wait_for(lock,5s,[&]{return received.messages.size()>=index+1;})) throw std::runtime_error("native model message deadline");
      const auto &message=received.messages[index];const auto expected=ideal(heights[index],velocities[index].X());
      if (std::abs(message.diff_pressure()-expected.pressure)>1e-6 || message.temperature()!=expected.temperature)
        throw std::runtime_error("native model numeric disagreement");
      std::cout << "{\"mode\":\"model\",\"world_z\":" << heights[index] << ",\"velocity_sensor\":";
      vector(std::cout,velocities[index]);std::cout << ',';fields(message,index,time);std::cout << "}\n";
    }
    { std::lock_guard<std::mutex> lock(received.mutex); if (received.messages.size()!=6) throw std::runtime_error("extra model frames"); }
    capture();
  }
  catch (...) {
    const auto original=std::current_exception();
    try {capture();} catch (const std::exception &e) {std::cerr << "live maps failure: " << e.what() << '\n';}
    std::rethrow_exception(original);
  }
}

struct Observation {
  double worldZ; double linkZ;
  gz::math::Quaterniond rotation;
  gz::math::Vector3d worldPointVelocity;
  gz::math::Vector3d linkVelocity;
  gz::math::Vector3d angularVelocity;
  gz::math::Vector3d sensorVelocity;
};
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
  bool air_speed = false;
  bool worldPose = false;
  bool linkPose = false;
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
        << ",\"has_air_speed\":" << (s.air_speed ? "true" : "false")
        << ",\"has_world_pose\":" << (s.worldPose ? "true" : "false")
        << ",\"has_link_pose\":" << (s.linkPose ? "true" : "false")
        << ",\"required_type_ids\":{\"air_speed\":" << gz::sim::components::AirSpeedSensor::typeId
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
      if (moving && !applied && !info.paused)
      {
        ecm.CreateComponent(state.body, gz::sim::components::WorldLinearVelocityReset({3, 0, 0}));
        ecm.CreateComponent(state.body, gz::sim::components::WorldAngularVelocityReset({0, 0, 2}));
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
      // EntityByComponents compares DATA, so AirSpeedSensor() would request an empty
      // sdf::Sensor. Select type presence and the exact name without replacing data.
      ecm.Each<gz::sim::components::AirSpeedSensor, gz::sim::components::Name>(
          [&](const gz::sim::Entity &entity, const gz::sim::components::AirSpeedSensor *,
              const gz::sim::components::Name *name)
          {
            if (name->Data() == "air_speed") { state.sensor = entity; ++state.sensorMatches; }
            return true;
          });
      state.body = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
      state.sensorExists = ecm.HasEntity(state.sensor);
      state.bodyExists = ecm.HasEntity(state.body);
      types(ecm, state.sensor, state.sensorTypes);
      types(ecm, state.body, state.bodyTypes);
      state.air_speed = ecm.Component<gz::sim::components::AirSpeedSensor>(state.sensor) != nullptr;
      const auto *pose = ecm.Component<gz::sim::components::WorldPose>(state.sensor);
      auto linkPose = gz::sim::Link(state.body).WorldPose(ecm);
      state.pointFieldsExamined = true;
      state.worldPose = pose != nullptr;
      state.linkPose = linkPose.has_value();
      if (state.sensorMatches != 1) throw std::runtime_error("native air_speed name/type selection is not unique");
      if (!state.sensorExists || !state.bodyExists || !state.air_speed || !pose || !linkPose)
        throw std::runtime_error("missing native point observation; see exact presence flags and type IDs");
      const auto *point=ecm.Component<gz::sim::components::WorldLinearVelocity>(state.sensor);
      const auto *local=ecm.Component<gz::sim::components::Pose>(state.sensor);
      auto linear=gz::sim::Link(state.body).WorldLinearVelocity(ecm);
      auto angular=gz::sim::Link(state.body).WorldAngularVelocity(ecm);
      if (!point || !local || !linear || !angular) throw std::runtime_error("missing native point/link velocity field");
      const auto pointVelocity=point->Data();
      const auto expectedPoint=*linear+angular->Cross(linkPose->Rot().RotateVector(local->Data().Pos()));
      if (!finite(pointVelocity) || !finite(*linear) || !finite(*angular) || (pointVelocity-expectedPoint).Length()>1e-9)
        throw std::runtime_error("native Physics sensor point velocity disagrees with link origin plus angular offset");
      const auto relative=pose->Data().Rot().RotateVectorReverse(pointVelocity);
      bool inserted;
      {
        std::lock_guard<std::mutex> guard(mutex);
        inserted=observations.emplace(state.timeNs,Observation{pose->Data().Pos().Z(),linkPose->Pos().Z(),
            pose->Data().Rot(),pointVelocity,*linear,*angular,relative}).second;
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
  // A paused Run(iterations) never consumes its iteration budget. RunOnce
  // creates the publisher in one finite paused discovery tick instead.
  if (!server.RunOnce(true)) throw std::runtime_error("native paused discovery tick failed");
  const auto deadline=std::chrono::steady_clock::now()+5s;bool found=false;
  while (std::chrono::steady_clock::now()<deadline) {
    std::vector<gz::transport::MessagePublisher> publishers,subscribers;
    node.TopicInfo("/m26/air_speed",publishers,subscribers);
    if (publishers.size()==1 && !subscribers.empty() && publishers[0].MsgTypeName()=="gz.msgs.AirSpeed") {found=true;break;}
    std::this_thread::sleep_for(1ms);
  }
  if (!found) throw std::runtime_error("native publisher/subscriber discovery deadline");
  if (!server.Run(true,100,false)) throw std::runtime_error("native100 advancing ticks failed");
  {
    std::unique_lock<std::mutex> guard(received.mutex);
    if (!received.ready.wait_for(guard,5s,[&]{return received.messages.size()>=11;})) throw std::runtime_error("native acquisition deadline");
  }
  server.Stop();
  // Capture while Server still owns the AirSpeed/Physics plugin and DART engine.
  captureMaps();
  if (diagnose()) throw std::runtime_error("native observer failed; callback state retained above");
  std::lock_guard<std::mutex> messagesGuard(received.mutex);
  std::lock_guard<std::mutex> stateGuard(observe->mutex);
  if (received.messages.size() != 11 || observe->observations.size() != 100)
    throw std::runtime_error("unexpected native100ms acquisition/frame counts");
  const auto &first=observe->observations.begin()->second;
  if (!std::isfinite(first.worldZ) || !std::isfinite(first.linkZ) || std::abs(first.worldZ-3.5)>.01 || first.worldZ-first.linkZ<.4)
    throw std::runtime_error("native nonzero Physics sensor-height witness missing");
  if (mode=="offset" && ((first.worldPointVelocity-first.linkVelocity).Length()<.5 ||
      std::abs(first.sensorVelocity.X()-first.linkVelocity.X())<.5))
    throw std::runtime_error("native angular-offset/composite-rotation witness missing");
  if (mode=="offset" && std::abs(observe->observations.rbegin()->second.rotation.Dot(first.rotation))>.999)
    throw std::runtime_error("native orientation motion witness missing");
  unsigned int index = 0;
  for (const auto &message : received.messages)
  {
    const auto at = observe->observations.find(stamp(message));
    if (at == observe->observations.end())
      throw std::runtime_error("native acquisition has no observed world pose");
    const auto &observation=at->second;
    const auto expected=ideal(observation.worldZ,observation.sensorVelocity.X());
    if (!std::isfinite(expected.pressure) || !std::isfinite(expected.temperature) || expected.temperature<=0 ||
        std::abs(message.diff_pressure()-expected.pressure)>1e-6 || message.temperature()!=expected.temperature)
      throw std::runtime_error("native point/frame/air-speed numeric disagreement");
    std::cout << "{\"mode\":\"" << mode << "\",\"world_z\":" << observation.worldZ
        << ",\"link_z\":" << observation.linkZ << ",\"rotation\":[" << observation.rotation.X() << ','
        << observation.rotation.Y() << ',' << observation.rotation.Z() << ',' << observation.rotation.W()
        << "],\"world_point_velocity\":";vector(std::cout,observation.worldPointVelocity);
    std::cout << ",\"link_velocity\":";vector(std::cout,observation.linkVelocity);
    std::cout << ",\"angular_velocity\":";vector(std::cout,observation.angularVelocity);
    std::cout << ",\"velocity_sensor\":";vector(std::cout,observation.sensorVelocity);std::cout << ',';
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
  else if (mode == "offset" || mode == "stationary") scene(mode, argv[2]);
  else return 2;
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
