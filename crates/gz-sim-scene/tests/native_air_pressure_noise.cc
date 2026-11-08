// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Native conventions: gz-sensors8ddf4c17 AirPressureSensor.cc,
// test/integration/air_pressure.cc and Noise_TEST.cc (OSRF, Apache-2.0).
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/sensors/AirPressureSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

using namespace std::chrono_literals;
using Message = gz::msgs::FluidPressure;

double ideal(double height)
{
  // RotorS-attributed native geopotential model, independently kept scalar here.
  const double geopotential = 6356766.0 * height / (6356766.0 + height);
  const double temperature = 288.15 - .0065 * geopotential;
  return 101325.0 * std::exp((9.80665 * 28.9644 / (8314.32 * -.0065)) *
      std::log(288.15 / temperature));
}

void maps(const std::string &path)
{
  std::ifstream input("/proc/self/maps");
  std::ofstream output(path);
  if (!input || !output) throw std::runtime_error("native live maps open failed");
  output << input.rdbuf();
  output.flush();
  if (!output) throw std::runtime_error("native live maps write failed");
}

void run(const std::string &mode, const std::string &mapsPath)
{
  std::string noise;
  double mean = 0, stddev = 0, precision = 0;
  if (mode == "ideal") noise = "";
  else if (mode == "zero") noise = "<noise type='gaussian'/>";
  else if (mode == "mean")
  {
    mean = 2.25;
    noise = "<noise type='gaussian'><mean>2.25</mean></noise>";
  }
  else if (mode == "gaussian-round" || mode == "quantized")
  {
    mean = 1.23;
    precision = .25;
    noise = "<noise type='" + std::string(mode == "quantized" ?
        "gaussian_quantized" : "gaussian") +
        "'><mean>1.23</mean><precision>0.25</precision></noise>";
  }
  else if (mode == "white")
  {
    mean = 10;
    stddev = 5;
    noise = "<noise type='gaussian'><mean>10</mean><stddev>5</stddev></noise>";
  }
  else if (mode == "bias-zero-dt")
    noise = "<noise type='gaussian'><bias_mean>0.3</bias_mean>"
        "<dynamic_bias_stddev>100</dynamic_bias_stddev>"
        "<dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise>";
  else throw std::runtime_error("unknown native pressure noise mode");

  gz::math::Rand::Seed(42);
  const std::string xml = "<sdf version='1.12'><model name='rig'><link name='body'>"
      "<sensor name='air_pressure' type='air_pressure'><topic>/m22/air_pressure</topic>"
      "<frame_id>rig/body/air_pressure</frame_id><air_pressure><pressure>" + noise +
      "</pressure></air_pressure></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);
  sdf::init(parsed);
  if (!sdf::readString(xml, parsed)) throw std::runtime_error("native pressure SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::AirPressureSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor || sensor->ReferenceAltitude() != 0)
    throw std::runtime_error("native pressure creation/reference failed");
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<Message> messages;
  bool overflow = false;
  gz::transport::Node node;
  if (!node.Subscribe("/m22/air_pressure", std::function<void(const Message &)>(
      [&](const Message &message) {
        std::lock_guard<std::mutex> guard(mutex);
        if (messages.size() >= 100) overflow = true;
        else messages.push_back(message);
        ready.notify_all();
      }))) throw std::runtime_error("native pressure subscription failed");
  bool attemptedMaps = false;
  auto capture = [&] { attemptedMaps = true; maps(mapsPath); };
  try
  {
    const auto discovery = std::chrono::steady_clock::now() + 5s;
    while (!sensor->HasConnections())
    {
      if (std::chrono::steady_clock::now() >= discovery)
        throw std::runtime_error("native pressure discovery deadline");
      std::this_thread::sleep_for(1ms);
    }
    const unsigned int count = mode == "white" ? 100 : 5;
    const double heights[] = {0, 3.5, -3, 100, 1000};
    double bias = 0;
    std::vector<double> residuals;
    for (unsigned int index = 0; index < count; ++index)
    {
      const double height = mode == "white" || mode == "bias-zero-dt" ? 3.5 : heights[index];
      // Re-supply the physical input on every acquisition, not a noisy previous value.
      sensor->SetPose(gz::math::Pose3d(0, 0, height, 0, 0, 0));
      const long long time = (index + 1) * (mode == "bias-zero-dt" ? 1000000000LL : 1000000LL);
      if (!sensor->Update(std::chrono::nanoseconds(time)))
        throw std::runtime_error("native pressure update refused");
      Message message;
      {
        std::unique_lock<std::mutex> guard(mutex);
        if (!ready.wait_for(guard, 5s, [&] { return overflow || messages.size() >= index + 1; }) || overflow)
          throw std::runtime_error("native pressure message deadline/overflow");
        message = messages[index];
      }
      std::string frame, sequence;
      for (const auto &field : message.header().data())
      {
        if (field.key() == "frame_id" && field.value_size() == 1) frame = field.value(0);
        if (field.key() == "seq" && field.value_size() == 1) sequence = field.value(0);
      }
      const long long stamp = message.header().stamp().sec() * 1000000000LL + message.header().stamp().nsec();
      if (stamp != time || frame != "rig/body/air_pressure" || sequence != std::to_string(index) ||
          !std::isfinite(message.pressure()) || message.pressure() <= 0 ||
          message.variance() != std::sqrt(stddev))
        throw std::runtime_error("native canonical pressure fields disagree");
      double expected = ideal(height) + mean;
      if (mode == "bias-zero-dt")
      {
        if (index == 0) bias = message.pressure() - expected;
        if (std::abs(std::abs(bias) - .3) > 1e-9)
          throw std::runtime_error("native initial signed bias disagrees");
        expected += bias;
      }
      if (precision > 1e-6) expected = std::round(expected / precision) * precision;
      if (mode != "white" && std::abs(message.pressure() - expected) > 1e-9)
        throw std::runtime_error("native deterministic pressure/no-elapsed-bias disagreement");
      residuals.push_back(message.pressure() - ideal(height));
      std::cout << "{\"mode\":\"" << mode << "\",\"sample\":" << index <<
          ",\"world_z\":" << height << ",\"time_ns\":" << time <<
          ",\"sequence\":\"" << sequence << "\",\"frame_id\":\"" << frame <<
          "\",\"pressure\":" << message.pressure() << ",\"variance\":" << message.variance() << "}\n";
    }
    if (mode == "white")
    {
      double actualMean = 0, actualVariance = 0;
      for (double value : residuals) actualMean += value / 100.0;
      for (double value : residuals) actualVariance += (value - actualMean) * (value - actualMean) / 99.0;
      // Retained upstream NoiseTest policy: 100 samples, five-sigma bounds.
      if (std::abs(actualMean - mean) > 5 * stddev / std::sqrt(100.0) ||
          std::abs(actualVariance - stddev * stddev) > 5 * std::sqrt(2 * std::pow(stddev, 4) / 99.0))
        throw std::runtime_error("native white pressure moment bounds disagree");
    }
    capture(); // Sensor, transport subscription and provider are still alive.
  }
  catch (...)
  {
    const auto original = std::current_exception();
    if (!attemptedMaps)
    {
      try { capture(); }
      catch (const std::exception &error) { std::cerr << "native maps: " << error.what() << '\n'; }
    }
    std::rethrow_exception(original);
  }
}

int main(int argc, char **argv)
{
  try
  {
    if (argc != 3) throw std::runtime_error("expected MODE MAPS_PATH");
    std::cout << std::setprecision(17);
    run(argv[1], argv[2]);
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "native pressure noise: " << error.what() << '\n';
    return 1;
  }
}
