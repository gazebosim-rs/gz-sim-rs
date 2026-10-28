// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//
// Minimal owner of one DART world for the Rust adapter. Rust validates every
// value first. Every bridge function is declared fallible, and the trycatch
// below converts any C++ exception, including non-std ones, into a Rust error.
#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <unordered_map>
#include <vector>

#include <dart/collision/CollisionResult.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include "rust/cxx.h"

// cxx's default catches only std::exception; anything else would terminate.
namespace rust {
namespace behavior {
template <typename Try, typename Fail>
static void trycatch(Try &&func, Fail &&fail) noexcept try {
  func();
} catch (const std::exception &error) {
  fail(error.what());
} catch (...) {
  fail("unknown C++ exception");
}
}  // namespace behavior
}  // namespace rust

namespace gz_physics_dart {

struct BodyConfig;
struct Twist;
struct Wrench;
struct Limits;
struct Readout;
class GuardedSolver;

class World {
 public:
  World(const Twist &gravity, const Limits &limits);
  ~World();
  World(const World &) = delete;
  World &operator=(const World &) = delete;

  std::size_t add_body(const BodyConfig &config);
  void remove_body(std::size_t slot);
  bool set_velocity(std::size_t slot, const Twist &velocity);
  bool apply_impulse(std::size_t slot, const Twist &impulse);
  bool add_revolute(std::size_t slot, const Twist &anchor_axis);
  bool remove_revolute(std::size_t slot);
  std::uint8_t step(double dt, rust::Slice<const Wrench> wrenches);
  Readout read(std::size_t slot) const;
  std::size_t skeleton_count() const;
  bool contacts_conditioned();

  // Whether every contact in `result` keeps its inverse effective mass bound.
  bool contactsWithin(const dart::collision::CollisionResult &result) const;
  // Whether every generalized velocity is finite and within the bound.
  bool velocitiesWithin() const;
  rust::Vec<rust::String> library_paths() const;

 private:
  dart::dynamics::BodyNode &body(std::size_t slot) const;
  void build(dart::dynamics::Skeleton &skeleton, const BodyConfig &config);
  bool insideGrid(const dart::dynamics::Skeleton &skeleton) const;
  bool contactsConditioned();
  double inverseMassAt(const dart::dynamics::BodyNode &node,
                       const Eigen::Vector3d &point) const;

  // Copied at construction: cxx shared structs are complete only in the
  // generated header, which includes this one, so keep plain values here.
  struct Bounds {
    double magnitude;
    double position;
    double contact;
  } limits_;
  // Validated principal inertia of each dynamic body, in its body frame.
  struct Principal {
    double inverseMass;
    Eigen::Vector3d inverseMoments;
    Eigen::Matrix3d axes;
  };
  std::unordered_map<const dart::dynamics::BodyNode *, Principal> principal_;
  dart::simulation::WorldPtr world_;
  // Owned by world_.
  GuardedSolver *solver_ = nullptr;
  std::vector<dart::dynamics::SkeletonPtr> slots_;
};

std::unique_ptr<World> new_world(const Twist &gravity, const Limits &limits);

// Throws the requested exception kind so tests can check exception
// translation: 0 returns, 1 throws std::runtime_error, other values throw an int.
void exception_probe(std::uint8_t kind);

}  // namespace gz_physics_dart
