// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <iostream>
#include "body_revolute_identity.hh"

using namespace dart::dynamics;
using dart::simulation::World;

static bool near(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a-b).norm() < 1e-10;
}
static bool near(const Eigen::Isometry3d &a, const Eigen::Isometry3d &b) {
  return (a.matrix()-b.matrix()).norm() < 1e-10;
}
static void free_velocity(BodyNode &node, const Eigen::Vector3d &linear,
                          const Eigen::Vector3d &angular) {
  auto &joint = static_cast<FreeJoint &>(*node.getParentJoint());
  joint.setAngularVelocity(angular);
  joint.setLinearVelocity(linear);
}

int main() {
  auto world = World::create("body-joint-probe");
  if (!check_dart_library()) return 2;
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto parent_skeleton = Skeleton::create("parent");
  auto child_skeleton = Skeleton::create("child");
  auto [parent_joint, parent] = parent_skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto [child_joint, child] = child_skeleton->createJointAndBodyNodePair<FreeJoint>();
  BodyNodePtr tracked_child = child;
  world->addSkeleton(parent_skeleton);
  world->addSkeleton(child_skeleton);
  Eigen::Isometry3d parent_pose = Eigen::Isometry3d::Identity();
  parent_pose.translation() = Eigen::Vector3d(-1, 0, 0);
  parent_pose.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d child_pose = Eigen::Isometry3d::Identity();
  child_pose.translation() = Eigen::Vector3d(1, 0, 0);
  child_pose.linear() = Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  parent_joint->setTransform(parent_pose);
  child_joint->setTransform(child_pose);
  free_velocity(*parent, Eigen::Vector3d(0,-2,0), Eigen::Vector3d(0,0,2));
  free_velocity(*child, Eigen::Vector3d(0,3,0), Eigen::Vector3d(0,0,3));
  assert(near(parent->getCOMLinearVelocity(), Eigen::Vector3d(0,-2,0)));
  assert(near(child->getCOMLinearVelocity(), Eigen::Vector3d(0,3,0)));
  assert(world->getNumSkeletons() == 2);
  const auto parent_before = parent->getWorldTransform();
  const auto child_before = child->getWorldTransform();
  Eigen::Isometry3d anchor = child_before;
  anchor.translation().setZero();
  RevoluteJoint::Properties props;
  props.mAxis = child_before.linear().transpose() * Eigen::Vector3d::UnitZ();
  auto *hinge = child->moveTo<RevoluteJoint>(parent, props);
  assert(hinge != nullptr);
  hinge->setTransformFromParentBodyNode(parent_before.inverse() * anchor);
  hinge->setTransformFromChildBodyNode(child_before.inverse() * anchor);
  hinge->setVelocity(0, 1.0);
  assert(near(parent->getWorldTransform(), parent_before));
  assert(near(child->getWorldTransform(), child_before));
  assert(near(parent->getAngularVelocity(), Eigen::Vector3d(0,0,2)));
  assert(near(child->getAngularVelocity(), Eigen::Vector3d(0,0,3)));
  assert(near(parent->getCOMLinearVelocity(), Eigen::Vector3d(0,-2,0)));
  assert(near(child->getCOMLinearVelocity(), Eigen::Vector3d(0,3,0)));
  std::cout << "after merge world=" << world->getNumSkeletons()
            << " parent_bodies=" << parent_skeleton->getNumBodyNodes()
            << " child_bodies=" << child_skeleton->getNumBodyNodes()
            << " dofs=" << parent_skeleton->getNumDofs() << '\n';
  assert(child->getSkeleton() == parent_skeleton);
  assert(parent_skeleton->getNumBodyNodes() == 2);
  assert(child_skeleton->getNumBodyNodes() == 0);
  world->removeSkeleton(child_skeleton);
  assert(world->getNumSkeletons() == 1);
  child_skeleton.reset();
  assert(tracked_child.get() == child);
  assert(tracked_child->getSkeleton() == parent_skeleton);
  assert(child->getSkeleton() == parent_skeleton);
  const Eigen::VectorXd initial_positions = parent_skeleton->getPositions();
  const Eigen::VectorXd initial_velocities = parent_skeleton->getVelocities();
  const Eigen::Vector3d parent_anchor = parent_before.inverse() * anchor.translation();
  const Eigen::Vector3d child_anchor = child_before.inverse() * anchor.translation();
  for (int i=0; i<1000; ++i) {
    world->step();
    assert(near(parent->getWorldTransform() * parent_anchor,
                child->getWorldTransform() * child_anchor));
  }
  parent_skeleton->setPositions(initial_positions);
  parent_skeleton->setVelocities(initial_velocities);
  world->setTime(0.0);
  assert(world->getTime() == 0.0);
  assert(near(parent->getWorldTransform(), parent_before));
  assert(near(child->getWorldTransform(), child_before));
  assert(near(parent->getCOMLinearVelocity(), Eigen::Vector3d(0,-2,0)));
  assert(near(child->getCOMLinearVelocity(), Eigen::Vector3d(0,3,0)));
  assert(near(parent->getAngularVelocity(), Eigen::Vector3d(0,0,2)));
  assert(near(child->getAngularVelocity(), Eigen::Vector3d(0,0,3)));
  for (int i=0; i<10; ++i) world->step();
  const auto detached_pose = child->getWorldTransform();
  const auto detached_linear = child->getCOMLinearVelocity();
  const auto detached_angular = child->getAngularVelocity();
  auto detached = child->split<FreeJoint>("detached");
  auto *detached_joint = static_cast<FreeJoint *>(child->getParentJoint());
  detached_joint->setTransform(detached_pose);
  free_velocity(*child, detached_linear, detached_angular);
  world->addSkeleton(detached);
  assert(near(child->getWorldTransform(), detached_pose));
  assert(near(child->getCOMLinearVelocity(), detached_linear));
  assert(near(child->getAngularVelocity(), detached_angular));
  assert(parent_skeleton->getNumBodyNodes() == 1);
  assert(detached->getNumBodyNodes() == 1);
  assert(world->getNumSkeletons() == 2);
  assert(tracked_child.get() == child);
  assert(tracked_child->getSkeleton() == detached);
  std::weak_ptr<Skeleton> weak_detached = detached;
  world->removeSkeleton(detached);
  detached.reset();
  assert(!weak_detached.expired());
  tracked_child = nullptr;
  assert(weak_detached.expired());
  assert(world->getNumSkeletons() == 1);
  world->step();
  std::cout << "detach pose/velocity preserved; parent remains live\n";
}
