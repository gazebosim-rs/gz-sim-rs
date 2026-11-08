// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Pinned CpuLidarSensor.cc and test/integration/cpu_lidar_sensor.cc,
// plus Noise_TEST.cc (OSRF, Apache-2.0) define these bounded controls.
#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/math/Rand.hh>
#include <gz/sensors/CpuLidarSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <sdf/sdf.hh>

void maps(const std::string &path)
{
  std::ifstream input("/proc/self/maps");
  std::ofstream output(path);
  if (!input || !output) throw std::runtime_error("CPU lidar maps open failed");
  output << input.rdbuf(); output.flush();
  if (!output) throw std::runtime_error("CPU lidar maps write failed");
}

void run(const std::string &mode, const std::string &mapsPath)
{
  double mean = 0, deviation = 0;
  if (mode == "mean") mean = 1;
  else if (mode == "lower") mean = -20;
  else if (mode == "upper") mean = 20;
  else if (mode == "inactive") { mean = 1e-6; deviation = 1e-6; }
  else if (mode == "active-boundary") mean = std::nextafter(1e-6, std::numeric_limits<double>::infinity());
  else if (mode == "white") { mean = .1; deviation = .2; }
  else if (mode != "none") throw std::runtime_error("unknown CPU lidar mode");
  std::ostringstream configuration;
  configuration << std::setprecision(17);
  configuration << "<sdf version='1.12'><model name='scanner'><link name='body'>"
      "<sensor name='laser' type='lidar'><topic>/m24/scan</topic><ray><scan><horizontal>"
      "<samples>5</samples><resolution>1</resolution><min_angle>-0.7853981633974483</min_angle>"
      "<max_angle>0.7853981633974483</max_angle></horizontal><vertical><samples>1</samples>"
      "<resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>"
      "<range><min>0</min><max>10</max><resolution>0</resolution></range>";
  if (mode != "none") configuration << "<noise><type>gaussian</type><mean>" << mean <<
      "</mean><stddev>" << deviation << "</stddev></noise>";
  configuration << "</ray></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF); sdf::init(parsed);
  if (!sdf::readString(configuration.str(), parsed)) throw std::runtime_error("native CPU lidar SDF refused");
  gz::math::Rand::Seed(42);
  gz::sensors::SensorFactory factory;
  auto sensor = factory.CreateSensor<gz::sensors::CpuLidarSensor>(
      parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor || sensor->RangeMin() != 0 || sensor->RangeMax() != 10)
    throw std::runtime_error("native CPU lidar construction/range failed");
  const auto rays = sensor->GenerateRays();
  if (rays.size() != 5) throw std::runtime_error("native CPU lidar beam count");
  for (const auto &ray : rays)
    if (ray.first.Length() > 1e-12 || std::abs(ray.second.Length() - 10) > 1e-12)
      throw std::runtime_error("native origin/minimum or maximum endpoint mismatch");
  bool attemptedMaps = false;
  auto capture = [&] { attemptedMaps = true; maps(mapsPath); };
  try
  {
    std::vector<std::vector<double>> residuals(5);
    const unsigned int scans = mode == "white" ? 100 : 2;
    for (unsigned int scan = 0; scan < scans; ++scan)
    {
      std::vector<gz::sensors::CpuLidarSensor::RayResult> input(5);
      const double raw[] = {0, 2.5, 0, 7.5, 10};
      for (unsigned int beam = 0; beam < 5; ++beam)
      {
        input[beam].fraction = beam == 2 ? std::numeric_limits<double>::infinity() :
            (mode == "white" ? .5 : raw[beam] / 10);
        if (beam != 2)
          input[beam].point = rays[beam].first + input[beam].fraction * (rays[beam].second - rays[beam].first);
      }
      sensor->SetRaycastResults(input);
      std::vector<double> ranges; sensor->Ranges(ranges);
      if (ranges.size() != 5) throw std::runtime_error("native CPU lidar output count");
      for (unsigned int beam = 0; beam < 5; ++beam)
      {
        if (beam == 2)
        {
          if (ranges[beam] != std::numeric_limits<double>::infinity()) throw std::runtime_error("native miss changed");
          std::cout << "{\"mode\":\"" << mode << "\",\"scan\":" << scan <<
              ",\"beam\":" << beam << ",\"raw_range\":null,\"range\":null,\"miss\":true}\n";
          continue;
        }
        const double physical = mode == "white" ? 5 : raw[beam];
        if (!std::isfinite(ranges[beam]) || ranges[beam] < 0 || ranges[beam] > 10)
          throw std::runtime_error("native finite result bounds");
        const double appliedMean = mode == "inactive" ? 0 : mean;
        const double expected = std::max(0.0, std::min(10.0, physical + appliedMean));
        if (mode != "white" && ranges[beam] != expected)
          throw std::runtime_error("native activation/mean/post-noise clipping disagreement");
        residuals[beam].push_back(ranges[beam] - physical);
        std::cout << "{\"mode\":\"" << mode << "\",\"scan\":" << scan << ",\"beam\":" << beam <<
            ",\"raw_range\":" << physical << ",\"range\":" << ranges[beam] << ",\"miss\":false}\n";
      }
    }
    if (mode == "white")
    {
      for (unsigned int beam : {0u, 1u, 3u, 4u})
      {
        double actualMean = 0, variance = 0;
        for (double value : residuals[beam]) actualMean += value / 100;
        for (double value : residuals[beam]) variance += (value - actualMean) * (value - actualMean) / 99;
        // Existing NoiseTest policy: one hundred samples and five-sigma bounds.
        if (residuals[beam].size() != 100 || std::abs(actualMean - mean) > 5 * deviation / std::sqrt(100.0) ||
            std::abs(variance - deviation * deviation) > 5 * std::sqrt(2 * std::pow(deviation, 4) / 99))
          throw std::runtime_error("native CPU lidar white moment bounds");
      }
    }
    capture();
  }
  catch (...)
  {
    const auto original = std::current_exception();
    if (!attemptedMaps)
    {
      try { capture(); }
      catch (const std::exception &error) { std::cerr << "native CPU maps: " << error.what() << '\n'; }
    }
    std::rethrow_exception(original);
  }
}

int main(int argc, char **argv)
{
  try
  {
    if (argc != 3) throw std::runtime_error("expected MODE MAPS_PATH");
    std::cout << std::setprecision(17); run(argv[1], argv[2]); return 0;
  }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
