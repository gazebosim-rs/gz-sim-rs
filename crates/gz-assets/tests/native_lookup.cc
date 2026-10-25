// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include <sdf/SDFImpl.hh>
#include <sdf/ParserConfig.hh>
int main(int argc, char **argv)
{
  if (argc < 2) return 2;
  sdf::ParserConfig config;
  for (int i = 2; i < argc; ++i) config.AddURIPath("model://", argv[i]);
  sdf::Errors errors;
  const auto result = sdf::findFile(errors, argv[1], true, false, config);
  std::cout << "{\"path\":" << std::quoted(result)
    << ",\"errors\":" << errors.size() << "}\n";
}
