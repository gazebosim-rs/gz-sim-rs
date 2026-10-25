// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/sensors/GaussianNoiseModel.hh>
#include <gz/sensors/Noise.hh>
#include <gz/math/Rand.hh>
#include <sdf/Noise.hh>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main()
{
  std::cout << std::setprecision(17);
  for (const std::string name : {"none", "mean", "precision", "white", "bias", "dynamic"})
  {
    gz::math::Rand::Seed(42);
    sdf::Noise config;
    config.SetType(name == "none" ? sdf::NoiseType::NONE : sdf::NoiseType::GAUSSIAN);
    if (name == "mean") config.SetMean(10);
    if (name == "precision") config.SetPrecision(0.3);
    if (name == "white") { config.SetMean(10); config.SetStdDev(5); }
    if (name == "bias") config.SetBiasMean(100);
    if (name == "dynamic")
    {
      config.SetDynamicBiasStdDev(0.7);
      config.SetDynamicBiasCorrelationTime(2.0);
    }
    auto noise = gz::sensors::NoiseFactory::NewNoiseModel(config);
    if (!noise) throw std::runtime_error("native noise creation failed");
    auto gaussian = std::dynamic_pointer_cast<gz::sensors::GaussianNoiseModel>(noise);
    std::cout << "{\"mode\":\"" << name << "\",\"seed\":42,\"initial_bias\":"
        << (gaussian ? gaussian->Bias() : 0) << ",\"samples\":[";
    for (int sample = 0; sample < 16; ++sample)
    {
      const double input = name == "precision" ? -12.92 + sample * 0.01 : 42.0;
      // ImuSensor converts acquisition elapsed through duration<float>.
      const double dt = sample == 0 ? 0 : static_cast<double>(static_cast<float>(0.001));
      const double value = noise->Apply(input, dt);
      if (!std::isfinite(value)) throw std::runtime_error("nonfinite native noise");
      if (sample) std::cout << ',';
      std::cout << "{\"input\":" << input << ",\"dt\":" << dt
          << ",\"value\":" << value << ",\"bias\":"
          << (gaussian ? gaussian->Bias() : 0) << '}';
    }
    std::cout << "]}\n";
  }
}
