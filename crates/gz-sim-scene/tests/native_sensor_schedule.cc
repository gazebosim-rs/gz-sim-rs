// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <gz/sensors/Sensor.hh>
#include <iostream>
#include <string>
class ScheduleProbe final : public gz::sensors::Sensor
{
  public: using Sensor::Update;
  public: bool Update(const std::chrono::steady_clock::duration &) override
  {
    return true;
  }
};
int main(int argc, char **argv)
{
  if (argc != 5) return 2;
  ScheduleProbe sensor;
  sensor.SetUpdateRate(std::stod(argv[1]));
  sensor.SetNextDataUpdateTime(std::chrono::nanoseconds(std::stoll(argv[2])));
  const bool sampled = sensor.Update(
      std::chrono::nanoseconds(std::stoll(argv[3])), std::stoi(argv[4]) != 0);
  std::cout << "{\"sampled\":" << std::boolalpha << sampled << ",\"next_ns\":"
    << std::chrono::duration_cast<std::chrono::nanoseconds>(
        sensor.NextDataUpdateTime()).count() << "}\n";
}
