// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <gz/common/URI.hh>
#include <gz/fuel_tools/ClientConfig.hh>
#include <gz/fuel_tools/FuelClient.hh>
#include <gz/fuel_tools/ModelIdentifier.hh>
#include <iomanip>
#include <iostream>
#include <sdf/parser.hh>
int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  gz::fuel_tools::ClientConfig config;
  config.SetCacheLocation(argv[1]);
  gz::fuel_tools::FuelClient client(config);
  gz::fuel_tools::ModelIdentifier id;
  const bool parsed = client.ParseModelUrl(gz::common::URI(argv[2]), id);
  std::string path, file;
  bool cached = false;
  sdf::Errors errors;
  if (parsed) {
    cached = client.CachedModel(id, path);
    if (cached) file = sdf::getModelFilePath(errors, path);
  }
  std::cout << "{\"parsed\":" << std::boolalpha << parsed
    << ",\"cached\":" << cached << ",\"owner\":" << std::quoted(id.Owner())
    << ",\"name\":" << std::quoted(id.Name()) << ",\"version\":" << id.Version()
    << ",\"path\":" << std::quoted(path) << ",\"file\":" << std::quoted(file)
    << ",\"errors\":" << errors.size() << "}\n";
}
