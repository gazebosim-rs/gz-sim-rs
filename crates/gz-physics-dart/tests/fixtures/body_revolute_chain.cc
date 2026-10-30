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

static bool near(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a-b).norm() < 1e-10;
}
int main() {
  using namespace dart::dynamics;
  auto world = dart::simulation::World::create("two-hinge-probe");
  if (!check_dart_library()) return 2;
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto upper_skeleton = Skeleton::create("upper");
  auto lower_skeleton = Skeleton::create("lower");
  RevoluteJoint::Properties upper_props;
  upper_props.mName = "upper_joint";
  upper_props.mAxis = Eigen::Vector3d::UnitY();
  BodyNode::Properties upper_body;
  upper_body.mName = "upper_body";
  auto [upper_joint, upper] = upper_skeleton->createJointAndBodyNodePair<RevoluteJoint>(nullptr, upper_props, upper_body);
  Eigen::Isometry3d upper_child = Eigen::Isometry3d::Identity();
  upper_child.translation().z() = 1.0;
  upper_joint->setTransformFromChildBodyNode(upper_child);
  upper_joint->setVelocity(0, 1.0);
  BodyNode::Properties lower_body;
  lower_body.mName = "lower_body";
  FreeJoint::Properties free_props;
  free_props.mName = "lower_free";
  auto [lower_free, lower] = lower_skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr, free_props, lower_body);
  Eigen::Isometry3d lower_pose = Eigen::Isometry3d::Identity();
  lower_pose.translation().z() = -3.0;
  lower_free->setTransform(lower_pose);
  lower_free->setAngularVelocity(Eigen::Vector3d(0,3,0));
  lower_free->setLinearVelocity(Eigen::Vector3d(-5,0,0));
  world->addSkeleton(upper_skeleton);
  world->addSkeleton(lower_skeleton);
  assert(near(upper->getCOM(), Eigen::Vector3d(0,0,-1)));
  assert(near(upper->getCOMLinearVelocity(), Eigen::Vector3d(-1,0,0)));
  assert(near(lower->getCOM(), Eigen::Vector3d(0,0,-3)));
  assert(near(lower->getCOMLinearVelocity(), Eigen::Vector3d(-5,0,0)));
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  anchor.translation().z() = -2.0;
  const auto before = lower->getWorldTransform();
  RevoluteJoint::Properties lower_props;
  lower_props.mName = "lower_joint";
  lower_props.mAxis = Eigen::Vector3d::UnitY();
  auto *lower_joint = lower->moveTo<RevoluteJoint>(upper, lower_props);
  assert(lower_joint);
  lower_joint->setTransformFromParentBodyNode(upper->getWorldTransform().inverse() * anchor);
  lower_joint->setTransformFromChildBodyNode(before.inverse() * anchor);
  lower_joint->setVelocity(0, 2.0);
  assert((lower->getWorldTransform().matrix() - before.matrix()).norm() < 1e-10);
  assert(near(lower->getAngularVelocity(), Eigen::Vector3d(0,3,0)));
  assert(near(lower->getCOMLinearVelocity(), Eigen::Vector3d(-5,0,0)));
  assert(upper_skeleton->getNumDofs() == 2);
  world->removeSkeleton(lower_skeleton);
  lower_skeleton.reset();
  const auto positions = upper_skeleton->getPositions();
  const auto velocities = upper_skeleton->getVelocities();
  assert(lower->moveTo<RevoluteJoint>(lower, lower_props) == nullptr);
  assert(upper->moveTo<RevoluteJoint>(lower, upper_props) == nullptr);
  assert(lower->getParentBodyNode() == upper);
  assert(upper->getParentBodyNode() == nullptr);
  assert(upper_skeleton->getPositions() == positions);
  assert(upper_skeleton->getVelocities() == velocities);
  const Eigen::Vector3d upper_local = upper->getWorldTransform().inverse() * anchor.translation();
  const Eigen::Vector3d lower_local = before.inverse() * anchor.translation();
  for (int i=0; i<1000; ++i) {
    world->step();
    assert(near(upper->getWorldTransform()*upper_local, lower->getWorldTransform()*lower_local));
  }
  upper_skeleton->setPositions(positions);
  upper_skeleton->setVelocities(velocities);
  assert(near(upper->getCOMLinearVelocity(), Eigen::Vector3d(-1,0,0)));
  assert(near(lower->getCOMLinearVelocity(), Eigen::Vector3d(-5,0,0)));
  assert(near(lower->getAngularVelocity(), Eigen::Vector3d(0,3,0)));
  std::cout << "two-DOF world-root chain preserved moving parent/child twists, anchor and reset\n";
}
