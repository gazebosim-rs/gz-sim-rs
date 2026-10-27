// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <iomanip>
#include <iostream>
#include <gz/sensors/Sensor.hh>

class ActivationProbe final : public gz::sensors::Sensor
{
  public: using Sensor::Update;
  public: bool Update(const std::chrono::steady_clock::duration &) override
  {
    ++this->count;
    return true;
  }
  public: unsigned int count = 0;
};

void state(const ActivationProbe &sensor)
{
  std::cout << "\"active\":" << sensor.IsActive()
            << ",\"rate_hz\":" << sensor.UpdateRate()
            << ",\"next_ns\":"
            << std::chrono::duration_cast<std::chrono::nanoseconds>(
                sensor.NextDataUpdateTime()).count()
            << ",\"count\":" << sensor.count;
}

int main()
{
  std::cout << std::boolalpha << std::setprecision(17) << "{\"groups\":[";
  const double rates[] = {0.0, 10.0};
  for (unsigned int group = 0; group < 2; ++group)
  {
    ActivationProbe sensor;
    sensor.SetUpdateRate(rates[group]);
    sensor.SetNextDataUpdateTime(std::chrono::milliseconds(100));
    if (group) std::cout << ',';
    std::cout << "{\"initial\":{";
    state(sensor);
    std::cout << "},\"rows\":[";
    bool first = true;
    auto update = [&](const char *label, long long now_ms, bool active, bool force)
    {
      sensor.SetActive(active);
      const bool sampled = sensor.Sensor::Update(std::chrono::milliseconds(now_ms), force);
      if (!first) std::cout << ',';
      first = false;
      std::cout << "{\"label\":\"" << label << "\",\"now_ms\":" << now_ms
                << ",\"force\":" << force << ",\"sampled\":" << sampled << ',';
      state(sensor);
      std::cout << '}';
    };
    update("active_before_deadline", 0, true, false);
    update("inactive_before_deadline", 50, false, false);
    update("forced_inactive_before_deadline", 50, false, true);
    update("reenabled_before_deadline", 50, true, false);
    update("inactive_at_deadline", 100, false, false);
    update("reenabled_at_deadline", 100, true, false);
    update("inactive_overdue", 250, false, false);
    update("forced_inactive_overdue", 250, false, true);
    update("reenabled_overdue", 250, true, false);
    std::cout << "]}";
  }
  std::cout << "]}\n";
  return 0;
}
