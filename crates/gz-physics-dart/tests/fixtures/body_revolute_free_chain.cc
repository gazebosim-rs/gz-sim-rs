// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Native-first control only: no adapter admission or original B03 claim.
#include <dart/config.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/external/odelcpsolver/lcp.h>
#include <dart/simulation/World.hpp>
#include <Eigen/Geometry>
#include <array>
#include <cassert>
#include <cmath>
#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include "body_revolute_identity.hh"
#include "../../src/joint_state.hh"

static_assert(DART_MAJOR_VERSION == 6 && DART_MINOR_VERSION == 13 &&
              DART_PATCH_VERSION == 2);
using namespace dart::dynamics;

static bool mapped_from(const std::string &prefix, const char *variable) {
  const char *configured = std::getenv(variable);
  if (!configured) return false;
  std::error_code error;
  const auto expected = std::filesystem::canonical(configured, error);
  if (error) return false;
  std::ifstream maps("/proc/self/maps");
  bool found = false;
  for (std::string line; std::getline(maps, line);) {
    const auto start = line.find('/');
    if (start == std::string::npos) continue;
    const std::filesystem::path path = line.substr(start);
    if (path.filename().string().rfind(prefix, 0) != 0) continue;
    const auto actual = std::filesystem::canonical(path, error);
    if (error || actual.parent_path() != expected) return false;
    found = true;
  }
  return found;
}

struct State {
  Eigen::Isometry3d pose;
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};
static State state(const BodyNode &body) {
  return {body.getWorldTransform(), body.getCOMLinearVelocity(),
          body.getAngularVelocity()};
}
static void same(const State &a, const State &b) {
  assert((a.pose.matrix() - b.pose.matrix()).norm() < 1e-10);
  assert((a.linear - b.linear).norm() < 1e-10);
  assert((a.angular - b.angular).norm() < 1e-10);
}
static void free_velocity(BodyNode &body, const State &value) {
  auto &joint = static_cast<FreeJoint &>(*body.getParentJoint());
  joint.setAngularVelocity(value.angular);
  joint.setLinearVelocity(value.linear - value.angular.cross(
      body.getWorldTransform().linear() * body.getLocalCOM()));
}
static void dirty(Skeleton &skeleton) {
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    skeleton.getBodyNode(i)->dirtyJacobian();
    skeleton.getBodyNode(i)->dirtyJacobianDeriv();
  }
}

struct ProbeChain {
  dart::simulation::WorldPtr world = dart::simulation::World::create("free-three");
  std::array<SkeletonPtr, 3> sources;
  std::array<BodyNodePtr, 3> bodies;
  std::array<Eigen::Vector3d, 2> anchors;
  std::array<Eigen::Vector3d, 2> axes;

  explicit ProbeChain(bool moving) {
    world->setConstraintSolver(
        std::make_unique<dart::constraint::BoxedLcpConstraintSolver>());
    auto *solver = world->getConstraintSolver();
    solver->setCollisionDetector(dart::collision::OdeCollisionDetector::create());
    solver->getCollisionOption().maxNumContacts = 10000;
    solver->getCollisionOption().collisionFilter =
        std::make_shared<dart::collision::BodyNodeCollisionFilter>();
    world->setTimeStep(0.001);
    world->setGravity(Eigen::Vector3d::Zero());
    anchors = {Eigen::Vector3d(0, 0, -1), Eigen::Vector3d::Zero()};
    axes = {Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()};
    if (moving) {
      anchors[1] = Eigen::Vector3d(0, 0, -3);
      axes[1] = Eigen::Vector3d(0.3, 0.8, 0.5).normalized();
    }
    for (std::size_t i = 0; i < 3; ++i) {
      sources[i] = Skeleton::create("body-" + std::to_string(i));
      sources[i]->setSelfCollisionCheck(false);
      BodyNode::Properties properties;
      properties.mName = "body-" + std::to_string(i);
      properties.mInertia.setMass(1.0);
      properties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.004);
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      if (moving) center = Eigen::Vector3d(0.03, -0.02, 0.01);
      properties.mInertia.setLocalCOM(center);
      FreeJoint::Properties free_properties;
      free_properties.mName = properties.mName + "_free";
      auto [joint, body] = sources[i]->createJointAndBodyNodePair<FreeJoint>(
          nullptr, free_properties, properties);
      bodies[i] = body;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(0, 0, -2.0 * i);
      if (!moving && i == 2) pose.translation() = Eigen::Vector3d(10, 0, 0);
      if (moving)
        pose.linear() = Eigen::AngleAxisd(0.2 * i, Eigen::Vector3d::UnitX())
                            .toRotationMatrix();
      joint->setTransform(pose);
      auto *shape = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          std::make_shared<SphereShape>(0.1));
      shape->getDynamicsAspect()->setFrictionCoeff(1.0);
      world->addSkeleton(sources[i]);
    }
    State root = state(*bodies[0]);
    root.linear.setZero();
    root.angular.setZero();
    if (moving) {
      root.linear = Eigen::Vector3d(0.2, -0.1, 0.05);
      root.angular = Eigen::Vector3d(0.1, 0.3, -0.2);
    }
    free_velocity(*bodies[0], root);
    for (std::size_t i = 1; i < 3; ++i) {
      const auto parent = state(*bodies[i - 1]);
      auto child = state(*bodies[i]);
      const double rate = moving ? (i == 1 ? 0.7 : -0.4) : 0.0;
      child.angular = parent.angular + axes[i - 1] * rate;
      child.linear = parent.linear + parent.angular.cross(
          bodies[i]->getCOM() - bodies[i - 1]->getCOM()) +
          (axes[i - 1] * rate).cross(bodies[i]->getCOM() - anchors[i - 1]);
      free_velocity(*bodies[i], child);
    }
  }

  void join(std::size_t child, bool commit) {
    const auto before = states();
    auto &node = *bodies[child];
    const auto pose = node.getWorldTransform();
    Eigen::Isometry3d joint_pose = pose;
    joint_pose.translation() = anchors[child - 1];
    RevoluteJoint::Properties properties;
    properties.mName = node.getName() + "_body_revolute";
    properties.mAxis = pose.linear().transpose() * axes[child - 1];
    properties.mPositionLowerLimits[0] = -1e16;
    properties.mPositionUpperLimits[0] = 1e16;
    properties.mIsPositionLimitEnforced = true;
    const double infinity = std::numeric_limits<double>::infinity();
    properties.mForceLowerLimits[0] = -infinity;
    properties.mForceUpperLimits[0] = infinity;
    properties.mVelocityLowerLimits[0] = -infinity;
    properties.mVelocityUpperLimits[0] = infinity;
    auto *joint = node.moveTo<RevoluteJoint>(bodies[child - 1].get(), properties);
    assert(joint);
    joint->setTransformFromParentBodyNode(
        bodies[child - 1]->getWorldTransform().inverse() * joint_pose);
    joint->setTransformFromChildBodyNode(pose.inverse() * joint_pose);
    Eigen::Vector6d twist;
    twist << before[child].angular, before[child].linear;
    gz_physics_dart::setJointTwist(node, twist);
    assert(sources[child]->getNumBodyNodes() == 0);
    for (std::size_t i = 0; i < 3; ++i) same(state(*bodies[i]), before[i]);
    if (commit) {
      world->removeSkeleton(sources[child]);
      sources[child].reset();
    }
  }

  std::array<State, 3> states() const {
    return {state(*bodies[0]), state(*bodies[1]), state(*bodies[2])};
  }
};

static void impulse_response(ProbeChain &chain) {
  const auto skeleton = chain.bodies[0]->getSkeleton();
  const Eigen::VectorXd initial = skeleton->getVelocities();
  assert(initial.size() == 8);
  for (std::size_t target = 0; target < 3; ++target) {
    auto &body = *chain.bodies[target];
    assert(body.getNumDependentGenCoords() == 6 + target);
    for (int axis = 0; axis < 6; ++axis) {
      const auto before = chain.states();
      Eigen::Vector6d impulse = Eigen::Vector6d::Zero();
      impulse[axis] = 1.0;
      const Eigen::Matrix3d inverse_rotation = body.getWorldTransform().linear().transpose();
      Eigen::Vector6d local;
      local.tail<3>() = inverse_rotation * impulse.tail<3>();
      local.head<3>() = inverse_rotation * impulse.head<3>() +
                       body.getLocalCOM().cross(local.tail<3>());
      skeleton->clearConstraintImpulses();
      skeleton->updateBiasImpulse(&body, local);
      skeleton->updateVelocityChange();
      std::array<Eigen::Vector6d, 3> observed;
      for (std::size_t i = 0; i < 3; ++i) {
        const auto &node = *chain.bodies[i];
        const auto delta = node.getBodyVelocityChange();
        const auto rotation = node.getWorldTransform().linear();
        observed[i].head<3>() = rotation * delta.head<3>();
        observed[i].tail<3>() = rotation *
            (delta.tail<3>() + delta.head<3>().cross(node.getLocalCOM()));
      }
      skeleton->setVelocities(gz_physics_dart::impulseVelocity(body, impulse));
      for (std::size_t i = 0; i < 3; ++i) {
        Eigen::Vector6d measured;
        const auto after = state(*chain.bodies[i]);
        measured << after.angular - before[i].angular, after.linear - before[i].linear;
        assert((measured - observed[i]).norm() < 1e-10 * (1.0 + observed[i].norm()));
      }
      skeleton->setVelocities(initial);
      skeleton->clearConstraintImpulses();
    }
  }
}

static void vector(std::ostream &out, const Eigen::Vector3d &v) {
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

static void record(std::ostream &out, const ProbeChain &chain, bool moving, int tick) {
  out << "{\"moving\":" << (moving ? "true" : "false") << ",\"tick\":" << tick
      << ",\"bodies\":[";
  for (std::size_t i = 0; i < 3; ++i) {
    if (i != 0) out << ',';
    const auto &body = *chain.bodies[i];
    const auto current = state(body);
    const Eigen::Quaterniond rotation(current.pose.linear());
    out << "{\"position\":";
    vector(out, current.pose.translation());
    out << ",\"orientation\":[" << rotation.x() << ',' << rotation.y() << ','
        << rotation.z() << ',' << rotation.w() << "],\"velocity\":";
    vector(out, current.linear);
    out << ",\"angular_velocity\":";
    vector(out, current.angular);
    out << ",\"center\":";
    vector(out, body.getLocalCOM());
    if (i != 0) {
      out << ",\"anchor\":";
      vector(out, chain.anchors[i - 1]);
      out << ",\"axis\":";
      vector(out, chain.axes[i - 1]);
      out << ",\"q\":" << body.getParentJoint()->getPosition(0)
          << ",\"dq\":" << body.getParentJoint()->getVelocity(0);
    }
    out << '}';
  }
  out << "]}\n";
  assert(out.good());
}

int main(int argc, char **argv) {
  if (argc < 1 || argc > 3) return 3;
  // Retain the actual external LCP DSO even with --as-needed linking.
  Dl_info lcp{};
  if (dladdr(reinterpret_cast<const void *>(&dart::external::ode::dSolveLCP),
             &lcp) == 0 || !lcp.dli_fname) return 2;
  if (!check_dart_library() ||
      !mapped_from("libdart-collision-ode.so", "GZ_DART_PROBE_LIBDIR") ||
      !mapped_from("libdart-external-odelcpsolver.so", "GZ_DART_PROBE_LIBDIR") ||
      !mapped_from("libode.so", "GZ_DART_PROBE_ODE_LIBDIR")) return 2;
  std::ofstream trace;
  if (argc == 3) {
    trace.open(argv[2]);
    assert(trace.good());
    trace << std::setprecision(17);
  }
  for (bool moving : {false, true}) {
    ProbeChain chain(moving);
    chain.join(1, true);
    const auto skeleton = chain.bodies[0]->getSkeleton();
    assert(skeleton->getNumDofs() == 7);
    // Warm survivor caches before a real third-body attach followed by rollback.
    for (const auto &body : chain.bodies) (void)body->getWorldJacobian(body->getLocalCOM());
    const auto before = chain.states();
    const Eigen::VectorXd positions = skeleton->getPositions();
    const Eigen::VectorXd velocities = skeleton->getVelocities();
    chain.join(2, false);
    assert(skeleton->getNumBodyNodes() == 3 && skeleton->getNumDofs() == 8);
    auto *restored = chain.bodies[2]->moveTo<FreeJoint>(chain.sources[2], nullptr);
    dirty(*skeleton);
    restored->setTransform(before[2].pose);
    free_velocity(*chain.bodies[2], before[2]);
    assert(skeleton->getPositions().isApprox(positions, 1e-12));
    assert(skeleton->getVelocities().isApprox(velocities, 1e-12));
    for (std::size_t i = 0; i < 3; ++i) same(state(*chain.bodies[i]), before[i]);
    assert(chain.world->getNumSkeletons() == 2);
    chain.join(2, true);
    assert(chain.world->getNumSkeletons() == 1);
    assert(skeleton->getNumDofs() == 8);
    impulse_response(chain);
    if (trace.is_open()) record(trace, chain, moving, 0);
    const Eigen::Vector3d a0 = chain.bodies[0]->getWorldTransform().inverse() * chain.anchors[0];
    const Eigen::Vector3d a1 = chain.bodies[1]->getWorldTransform().inverse() * chain.anchors[0];
    const Eigen::Vector3d b0 = chain.bodies[1]->getWorldTransform().inverse() * chain.anchors[1];
    const Eigen::Vector3d b1 = chain.bodies[2]->getWorldTransform().inverse() * chain.anchors[1];
    for (int step = 0; step < 1000; ++step) {
      if (step == 2) chain.bodies[2]->getParentJoint()->setForce(0, -0.7);
      chain.world->step();
      assert(chain.world->getConstraintSolver()->getLastCollisionResult().getNumContacts() == 0);
      assert((chain.bodies[0]->getWorldTransform() * a0 -
              chain.bodies[1]->getWorldTransform() * a1).norm() < 1e-10);
      assert((chain.bodies[1]->getWorldTransform() * b0 -
              chain.bodies[2]->getWorldTransform() * b1).norm() < 1e-10);
      for (const auto &body : chain.bodies) {
        const auto current = state(*body);
        assert(current.pose.matrix().allFinite() && current.linear.allFinite() &&
               current.angular.allFinite());
      }
      const int tick = step + 1;
      if (trace.is_open() && (tick <= 3 || tick == 10 || tick == 100 || tick == 1000))
        record(trace, chain, moving, tick);
    }
    // A middle split carries the leaf with it; both surviving joints/rates matter.
    const auto split_before = chain.states();
    const double leaf_rate = chain.bodies[2]->getParentJoint()->getVelocity(0);
    auto subtree = chain.bodies[1]->split<FreeJoint>("middle-subtree");
    dirty(*skeleton);
    auto *free = static_cast<FreeJoint *>(chain.bodies[1]->getParentJoint());
    free->setTransform(split_before[1].pose);
    free_velocity(*chain.bodies[1], split_before[1]);
    chain.world->addSkeleton(subtree);
    for (std::size_t i = 0; i < 3; ++i) same(state(*chain.bodies[i]), split_before[i]);
    assert(subtree->getNumBodyNodes() == 2 && subtree->getNumDofs() == 7);
    assert(chain.bodies[2]->getParentJoint()->getVelocity(0) == leaf_rate);
    assert(skeleton->getNumBodyNodes() == 1 && skeleton->getNumDofs() == 6);
    assert(chain.world->getNumSkeletons() == 2);
    std::weak_ptr<Skeleton> lifetime = subtree;
    chain.world->removeSkeleton(subtree);
    subtree.reset();
    assert(!lifetime.expired());
    chain.bodies[1] = nullptr;
    assert(!lifetime.expired());
    chain.bodies[2] = nullptr;
    assert(lifetime.expired());
    chain.world->step();
    assert(chain.bodies[0]->getWorldTransform().matrix().allFinite());
  }
  if (argc >= 2) {
    std::ifstream maps("/proc/self/maps");
    std::ofstream capture(argv[1]);
    assert(maps.good() && capture.good());
    capture << maps.rdbuf();
    assert(!capture.fail());
  }
  std::cout << "free three-body move/rollback/8-DOF impulse/step/subtree lifetime preserved\n";
}
