// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>

class RateProbe final : public gz::sensors::Sensor
{
  public: using Sensor::Update;
  public: bool Update(const std::chrono::steady_clock::duration &) override
  {
    ++this->count;
    return true;
  }
  public: unsigned int count = 0;
};

void number(double value)
{
  if (std::isnan(value)) std::cout << "\"nan\"";
  else if (!std::isfinite(value)) std::cout << (value < 0 ? "\"-inf\"" : "\"inf\"");
  else std::cout << value;
}

bool load(RateProbe &sensor, const std::string &name,
          const std::string &topic, double maximum)
{
  sdf::Sensor config;
  config.SetName(name);
  config.SetType(sdf::SensorType::ALTIMETER);
  config.SetTopic(topic);
  config.SetUpdateRate(maximum);
  return sensor.Load(config);
}

void ready(gz::transport::Node &node, const std::string &service)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::vector<gz::transport::ServicePublisher> publishers;
    if (node.ServiceInfo(service, publishers) && !publishers.empty())
    {
      if (publishers.size() != 1 ||
          publishers[0].ReqTypeName() != gz::msgs::Double::descriptor()->full_name() ||
          publishers[0].RepTypeName() != gz::msgs::Empty::descriptor()->full_name())
        throw std::runtime_error("unexpected rate service publisher/type: " + service);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("rate service discovery timed out: " + service);
}

void request(gz::transport::Node &node, const std::string &service, double rate)
{
  gz::msgs::Double input;
  input.set_data(rate);
  gz::msgs::Empty reply;
  bool result = false;
  if (!node.Request(service, input, 1000u, reply, result) || !result)
    throw std::runtime_error("rate service request failed: " + service);
  if (reply.ByteSizeLong() != 0u)
    throw std::runtime_error("rate service reply is not an empty message");
}

void state(const RateProbe &sensor)
{
  std::cout << "\"rate\":";
  number(sensor.UpdateRate());
  std::cout << ",\"rate_negative_zero\":" << (sensor.UpdateRate() == 0.0 && std::signbit(sensor.UpdateRate()));
  std::cout << ",\"next_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(
      sensor.NextDataUpdateTime()).count() << ",\"count\":" << sensor.count;
}

int main()
try
{
  std::cout << std::boolalpha << std::setprecision(17) << "{\"bounds\":[";
  const double maxima[] = {0.0, 10.0, 1e-7};
  const double rates[] = {5.0, 0.0, 20.0, -5.0, -0.0, 1e-7, 1e-6, 1e-5,
      10.0, 10.0 + 0.5e-6, 10.0 + 1e-6, 10.0 + 2e-6,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  for (unsigned int i = 0; i < 3; ++i)
  {
    RateProbe sensor;
    const auto topic = "/rate_bounds_" + std::to_string(i);
    if (!load(sensor, "bounds_" + std::to_string(i), topic, maxima[i])) return 2;
    gz::transport::Node node;
    ready(node, topic + "/set_rate");
    if (i) std::cout << ',';
    std::cout << "{\"sdf_max\":" << maxima[i] << ",\"rows\":[";
    for (unsigned int j = 0; j < sizeof(rates) / sizeof(rates[0]); ++j)
    {
      request(node, topic + "/set_rate", rates[j]);
      if (j) std::cout << ',';
      std::cout << "{\"request\":";
      number(rates[j]);
      std::cout << ",\"request_negative_zero\":" << (rates[j] == 0.0 && std::signbit(rates[j]));
      std::cout << ',';
      state(sensor);
      std::cout << '}';
    }
    std::cout << "]}";
  }
  RateProbe sensor;
  if (!load(sensor, "phase", "/rate_phase", 10.0)) return 2;
  gz::transport::Node node;
  ready(node, "/rate_phase/set_rate");
  std::cout << "],\"phase\":[";
  const long long times[] = {0, 50000000, 100000000, 200000000, 300000000};
  for (unsigned int i = 0; i < 5; ++i)
  {
    if (i == 1) request(node, "/rate_phase/set_rate", 5.0);
    if (i == 3) request(node, "/rate_phase/set_rate", 10.0);
    const bool sampled = sensor.Update(std::chrono::nanoseconds(times[i]), false);
    if (i) std::cout << ',';
    std::cout << "{\"now_ns\":" << times[i] << ",\"sampled\":" << sampled << ',';
    state(sensor);
    std::cout << '}';
  }
  std::cout << "],\"nonfinite_phase\":[";
  const double special[] = {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  const long long ticks[] = {0, 0, 1, 2};
  for (unsigned int i = 0; i < 3; ++i)
  {
    RateProbe specialSensor;
    const auto topic = "/rate_special_" + std::to_string(i);
    if (!load(specialSensor, "special_" + std::to_string(i), topic, 0.0)) return 2;
    gz::transport::Node specialNode;
    ready(specialNode, topic + "/set_rate");
    // NaN and clamped negative infinity bypass the positive-rate time gate.
    specialSensor.SetNextDataUpdateTime(std::chrono::nanoseconds(i == 1 ? 0 : 1000000000));
    request(specialNode, topic + "/set_rate", special[i]);
    if (i) std::cout << ',';
    std::cout << "{\"request\":";
    number(special[i]);
    std::cout << ",\"rows\":[";
    for (unsigned int j = 0; j < 4; ++j)
    {
      const bool sampled = specialSensor.Update(std::chrono::nanoseconds(ticks[j]), false);
      if (j) std::cout << ',';
      std::cout << "{\"now_ns\":" << ticks[j] << ",\"sampled\":" << sampled << ',';
      state(specialSensor);
      std::cout << '}';
    }
    std::cout << "]}";
  }
  RateProbe sharedA, sharedB;
  const bool loadedA = load(sharedA, "shared_a", "/rate_shared", 10.0);
  const bool loadedB = load(sharedB, "shared_b", "/rate_shared", 100.0);
  gz::transport::Node sharedNode;
  std::vector<gz::transport::ServicePublisher> sharedPublishers;
  const auto sharedDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do
  {
    sharedNode.ServiceInfo("/rate_shared/set_rate", sharedPublishers);
    if (!sharedPublishers.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < sharedDeadline);
  if (sharedPublishers.empty()) throw std::runtime_error("shared rate service absent");
  for (const auto &publisher : sharedPublishers)
    if (publisher.ReqTypeName() != gz::msgs::Double::descriptor()->full_name() ||
        publisher.RepTypeName() != gz::msgs::Empty::descriptor()->full_name())
      throw std::runtime_error("shared rate service type mismatch");
  std::cout << "],\"shared\":{\"load_a\":" << loadedA << ",\"load_b\":" << loadedB
            << ",\"publishers\":" << sharedPublishers.size() << ",\"rows\":[";
  const double sharedRates[] = {3.0, 4.0, 5.0, 20.0, 0.0, 6.0};
  for (unsigned int i = 0; i < 6; ++i)
  {
    request(sharedNode, "/rate_shared/set_rate", sharedRates[i]);
    if (i) std::cout << ',';
    std::cout << "{\"request\":" << sharedRates[i] << ",\"a\":";
    number(sharedA.UpdateRate());
    std::cout << ",\"b\":";
    number(sharedB.UpdateRate());
    std::cout << '}';
  }
  std::cout << "]}}\n";
  return 0;
}
catch (const std::exception &error)
{
  std::cerr << error.what() << '\n';
  return 2;
}
