// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#pragma once
#ifdef NDEBUG
#error "Native articulation fixtures require assertions"
#endif
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

// The child checks its own actual mappings, not merely the linker's search path.
inline bool check_dart_library() {
  const char *expected = std::getenv("GZ_DART_PROBE_LIBDIR");
  if (!expected) return false;
  std::error_code error;
  const auto libdir = std::filesystem::canonical(expected, error);
  if (error) return false;
  std::ifstream maps("/proc/self/maps");
  if (!maps.good()) return false;
  bool found = false;
  std::string line;
  while (std::getline(maps, line)) {
    const auto start = line.find('/');
    if (start == std::string::npos) continue;
    const std::filesystem::path mapped = line.substr(start);
    const std::string name = mapped.filename().string();
    if (name != "libdart.so" && name.rfind("libdart.so.", 0) != 0) continue;
    const auto library = std::filesystem::canonical(mapped, error);
    if (error || library.parent_path() != libdir) return false;
    found = true;
  }
  return found;
}
