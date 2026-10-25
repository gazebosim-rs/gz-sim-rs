// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  sdf::Root root;
  sdf::ParserConfig config;
  const auto errors = root.Load(argv[1], config);
  std::string name;
  if (root.WorldCount() && root.WorldByIndex(0)->ModelCount())
    name = root.WorldByIndex(0)->ModelByIndex(0)->Name();
  std::cout << "{\"name\":" << std::quoted(name) << ",\"errors\":[";
  for (std::size_t i = 0; i < errors.size(); ++i)
  {
    if (i) std::cout << ',';
    std::cout << static_cast<int>(errors[i].Code());
  }
  std::cout << "]}\n";
}
