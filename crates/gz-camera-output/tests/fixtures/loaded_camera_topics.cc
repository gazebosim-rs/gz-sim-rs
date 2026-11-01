// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/sensors/CameraSensor.hh>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Sensor.hh>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
int main(int argc, char **argv) {
  if (argc != 5) return 2;
  sdf::Root root;
  const auto errors = root.Load(argv[1]);
  if (!errors.empty() || !root.Model() || !root.Model()->LinkByIndex(0) ||
      !root.Model()->LinkByIndex(0)->SensorByIndex(0)) return 3;
  sdf::Sensor metadata(*root.Model()->LinkByIndex(0)->SensorByIndex(0));
  if (std::string(argv[2]) != "-") metadata.SetName(argv[2]);
  if (std::string(argv[3]) != "-") metadata.SetTopic(argv[3]);
  gz::sensors::CameraSensor sensor;
  const bool loaded = sensor.Load(metadata);
  std::ifstream maps("/proc/self/maps");
  std::ofstream saved(argv[4]);
  saved << maps.rdbuf();
  std::cout << "RESULT {\"loaded\":" << (loaded ? "true" : "false")
            << ",\"topic\":" << std::quoted(sensor.Topic())
            << ",\"info_topic\":" << std::quoted(sensor.InfoTopic())
            << ",\"frame\":" << std::quoted(sensor.FrameId())
            << ",\"active\":" << (sensor.IsActive() ? "true" : "false")
            << ",\"has_rendering_scene\":" << (sensor.Scene() ? "true" : "false")
            << "}\n";
}
