// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <iostream>
#include <iterator>
#include <string>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Types.hh>
int main()
{
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  gz::sim::ServerConfig config;
  if (!config.SetSdfString(xml)) return 2;
  gz::sim::Server server(config);
  if (!server.Run(true, 3, false)) return 3;
  const auto info = server.CurrentInfo();
  if (!info) return 4;
  std::cout << "{\"time_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(info->simTime).count()
    << ",\"dt_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(info->dt).count()
    << ",\"iterations\":" << info->iterations << "}\n";
}
