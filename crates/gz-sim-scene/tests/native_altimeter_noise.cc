// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Conventions: gz-sensors/test/integration/altimeter.cc (2019 OSRF) and
// src/Noise_TEST.cc. Native and Rust RNG samples are intentionally not compared.
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/msgs/altimeter.pb.h>
#include <gz/sensors/AltimeterSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

using namespace std::chrono_literals;
using Message = gz::msgs::Altimeter;
struct Received
{
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<Message> messages;
  bool overflow = false;
};
void writeMaps(const std::string &path)
{
  std::ifstream source("/proc/self/maps");
  std::ofstream output(path);
  if (!source || !output) throw std::runtime_error("live maps open failed");
  output << source.rdbuf(); output.flush();
  if (!output) throw std::runtime_error("live maps write failed");
}
std::string config(const std::string &mode)
{
  if (mode == "deterministic")
    return "<vertical_position><noise type='gaussian'><mean>2.25</mean><precision>0.5</precision></noise></vertical_position>"
           "<vertical_velocity><noise type='gaussian_quantized'><mean>-2.25</mean><precision>0.5</precision></noise></vertical_velocity>";
  if (mode == "white")
    return "<vertical_position><noise type='gaussian'><mean>1</mean><stddev>0.2</stddev></noise></vertical_position>"
           "<vertical_velocity><noise type='gaussian'><mean>-2</mean><stddev>0.4</stddev></noise></vertical_velocity>";
  if (mode == "bias-zero-dt")
    return "<vertical_position><noise type='gaussian'><bias_mean>2</bias_mean><dynamic_bias_stddev>8</dynamic_bias_stddev><dynamic_bias_correlation_time>0.2</dynamic_bias_correlation_time></noise></vertical_position>"
           "<vertical_velocity><noise type='gaussian'><bias_mean>3</bias_mean><dynamic_bias_stddev>9</dynamic_bias_stddev><dynamic_bias_correlation_time>0.3</dynamic_bias_correlation_time></noise></vertical_velocity>";
  throw std::runtime_error("unknown native noise mode");
}
void moments(const std::vector<double> &values, double expectedMean, double stddev)
{
  if (values.size() != 100) throw std::runtime_error("native moment count differs");
  double mean = 0;
  for (double value : values) mean += value / 100;
  double variance = 0;
  for (double value : values) variance += (value-mean)*(value-mean) / 100;
  // Same 100-sample / five-sigma policy as retained upstream Noise_TEST.cc.
  if (std::abs(mean-expectedMean) > 5*stddev/std::sqrt(100.0) ||
      std::abs(variance-stddev*stddev) > 5*std::sqrt(2*std::pow(stddev,4)/99))
    throw std::runtime_error("native Gaussian moment bounds differ");
}
void run(const std::string &mode, const std::string &mapsPath)
{
  const std::string xml = "<sdf version='1.12'><model name='rig'><link name='body'>"
      "<sensor name='altimeter' type='altimeter'><topic>/m21/altimeter</topic>"
      "<frame_id>rig/body/altimeter</frame_id><altimeter>" + config(mode) +
      "</altimeter></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);
  sdf::init(parsed);
  if (!sdf::readString(xml, parsed)) throw std::runtime_error("native noisy SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::AltimeterSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor) throw std::runtime_error("native noisy altimeter creation failed");
  Received received;
  gz::transport::Node node;
  bool mapsAttempted = false;
  unsigned int current = 0;
  auto capture = [&]
  {
    if (!mapsAttempted) { mapsAttempted = true; writeMaps(mapsPath); }
  };
  try
  {
    if (!node.Subscribe("/m21/altimeter", std::function<void(const Message &)>(
        [&](const Message &message)
        {
          std::lock_guard<std::mutex> guard(received.mutex);
          if (received.messages.size() < 100) received.messages.push_back(message);
          else received.overflow = true;
          received.ready.notify_all();
        }))) throw std::runtime_error("native noise subscription failed");
    const auto deadline = std::chrono::steady_clock::now()+5s;
    while (!sensor->HasConnections())
    {
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("native noise connection deadline");
      std::this_thread::sleep_for(1ms);
    }
    std::vector<double> positionResiduals, velocityResiduals;
    for (current = 0; current < 100; ++current)
    {
      const double worldZ = (current % 2 == 0) ? 3.5 : -4.0;
      const double reference = (current % 2 == 0) ? 3.5 : -5.0;
      const double velocity = (current % 2 == 0) ? 0.0 : -2.0;
      // Update mutates native readings: refresh physical values before EVERY call.
      sensor->SetVerticalReference(reference);
      sensor->SetPosition(worldZ);
      sensor->SetVerticalVelocity(velocity);
      const long long timeNs = (current+1)*1000000LL;
      if (!sensor->Update(std::chrono::nanoseconds(timeNs)))
        throw std::runtime_error("native noisy update failed");
      std::unique_lock<std::mutex> guard(received.mutex);
      if (!received.ready.wait_for(guard, 5s, [&]{return received.messages.size() >= current+1 || received.overflow;}))
        throw std::runtime_error("native noisy message deadline");
      if (received.overflow || received.messages.size() != current+1)
        throw std::runtime_error("unexpected native noisy message count");
      const auto &message = received.messages[current];
      std::string frame, sequence;
      for (const auto &entry : message.header().data())
      {
        if (entry.key() == "frame_id" && entry.value_size() == 1) frame = entry.value(0);
        if (entry.key() == "seq" && entry.value_size() == 1) sequence = entry.value(0);
      }
      const auto actualTime = message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
      if (frame != "rig/body/altimeter" || sequence != std::to_string(current) ||
          actualTime != timeNs || message.vertical_reference() != reference ||
          !std::isfinite(message.vertical_position()) || !std::isfinite(message.vertical_velocity()))
        throw std::runtime_error("native noisy fields differ");
      const double positionResidual = message.vertical_position()-(worldZ-reference);
      const double velocityResidual = message.vertical_velocity()-velocity;
      positionResiduals.push_back(positionResidual); velocityResiduals.push_back(velocityResidual);
      if (mode == "deterministic" &&
          (message.vertical_position() != std::round((worldZ-reference+2.25)/0.5)*0.5 ||
           message.vertical_velocity() != std::round((velocity-2.25)/0.5)*0.5))
        throw std::runtime_error("native mean/rounding differs");
      if (mode == "bias-zero-dt" &&
          (std::abs(positionResidual) != 2 || std::abs(velocityResidual) != 3 ||
           positionResidual != positionResiduals.front() || velocityResidual != velocityResiduals.front()))
        throw std::runtime_error("native fixed bias unexpectedly evolved");
      std::cout << "{\"mode\":\"" << mode << "\",\"time_ns\":" << timeNs
          << ",\"sequence\":" << current << ",\"frame_id\":\"" << frame
          << "\",\"world_z\":" << worldZ << ",\"world_velocity_z\":" << velocity
          << ",\"vertical_reference\":" << reference << ",\"vertical_position\":"
          << message.vertical_position() << ",\"vertical_velocity\":" << message.vertical_velocity() << "}\n";
    }
    if (mode == "white") { moments(positionResiduals,1,0.2); moments(velocityResiduals,-2,0.4); }
    capture();
  }
  catch (...)
  {
    const auto original = std::current_exception();
    std::cerr << "native noisy mode=" << mode << " index=" << current << '\n';
    try { capture(); }
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
  run(argv[1], argv[2]);
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
