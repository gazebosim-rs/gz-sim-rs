// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <fstream>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::sim::ServerConfig config;
  config.SetSdfFile(argv[1]);
  gz::sim::Server server(config);
  const auto count = server.SystemCount();
  if (!count) return 3;
  std::ifstream maps("/proc/self/maps");
  std::ofstream saved(argv[2]);
  saved << maps.rdbuf();
  std::cout << "RESULT " << *count << '\n';
  // No Run, RunOnce, sensor update, rendering scene or GPU admission.
}
