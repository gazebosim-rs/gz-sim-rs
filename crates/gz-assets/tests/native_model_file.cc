// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include <sdf/parser.hh>
int main(int argc, char **argv)
{
  if (argc != 2 && argc != 3) return 2;
  sdf::Errors errors;
  const auto path = sdf::getModelFilePath(errors, argv[1]);
  std::cout << "{\"path\":" << std::quoted(path) << ",\"errors\":" << errors.size();
  if (argc == 3) {
    std::cout << ",\"codes\":[";
    for (std::size_t i = 0; i < errors.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << static_cast<int>(errors[i].Code());
    }
    std::cout << ']';
  }
  std::cout << "}\n";
}
