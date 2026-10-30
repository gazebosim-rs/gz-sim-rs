// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <sdf/SDFImpl.hh>
#include <sdf/parser.hh>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string input{std::istreambuf_iterator<char>(std::cin), {}};
  sdf::SDFPtr document(new sdf::SDF);
  sdf::init(document);
  sdf::Errors errors;
  const bool loaded = sdf::readString(
      "<sdf version='1.12'><model name='m'><plugin "
      "name='gz::sim::systems::ApplyJointForce' "
      "filename='gz-sim-apply-joint-force-system'>" + input +
      "</plugin></model></sdf>", document, errors);
  std::cout << loaded << ' ' << errors.size() << '\n';
  if (loaded) {
    const auto plugin = document->Root()->GetElement("model")
        ->GetElement("plugin")->Clone();
    const auto name = plugin->GetElement("joint_name")->Get<std::string>();
    for (unsigned char c : name)
      std::cout << "0123456789abcdef"[c >> 4]
                << "0123456789abcdef"[c & 15];
    std::cout << '\n';
  }
  std::ofstream(argv[1]) << std::ifstream("/proc/self/maps").rdbuf();
}
