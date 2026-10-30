// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include "body_revolute_identity.hh"

int main() {
  using namespace dart::dynamics;
  auto world = dart::simulation::World::create("fixed-parent-probe");
  if (!check_dart_library()) return 2;
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto parent_skeleton = Skeleton::create("parent");
  auto child_skeleton = Skeleton::create("child");
  auto [parent_free, parent] = parent_skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto [child_free, child] = child_skeleton->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d parent_pose = Eigen::Isometry3d::Identity();
  parent_pose.translation() = Eigen::Vector3d(-1, 0, 0);
  Eigen::Isometry3d child_pose = Eigen::Isometry3d::Identity();
  child_pose.translation() = Eigen::Vector3d(1, 0, 0);
  parent_free->setTransform(parent_pose);
  child_free->setTransform(child_pose);
  parent_skeleton->setMobile(false);
  world->addSkeleton(parent_skeleton);
  world->addSkeleton(child_skeleton);
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  RevoluteJoint::Properties props;
  props.mAxis = Eigen::Vector3d::UnitY();
  auto *hinge = child->moveTo<RevoluteJoint>(parent, props);
  assert(hinge);
  hinge->setTransformFromParentBodyNode(parent_pose.inverse() * anchor);
  hinge->setTransformFromChildBodyNode(child_pose.inverse() * anchor);
  hinge->setVelocity(0, 1.0);
  world->removeSkeleton(child_skeleton);
  child_skeleton.reset();
  for (int i=0; i<100; ++i) world->step();
  assert(std::abs(hinge->getPosition(0)) < 1e-12);
  std::cout << "nonmobile parent skeleton freezes the child too\n";
  auto *weld = parent->changeParentJointType<WeldJoint>();
  assert(weld);
  weld->setTransformFromParentBodyNode(parent_pose);
  parent_skeleton->setMobile(true);
  hinge = static_cast<RevoluteJoint *>(child->getParentJoint());
  hinge->setVelocity(0, 1.0);
  for (int i=0; i<100; ++i) world->step();
  assert((parent->getWorldTransform().matrix() - parent_pose.matrix()).norm() < 1e-12);
  assert(hinge->getPosition(0) > 0.09);
  std::cout << "welded root keeps parent fixed and allows child motion\n";
  std::weak_ptr<Skeleton> weak = parent_skeleton;
  world->removeSkeleton(parent_skeleton);
  parent_skeleton.reset();
  assert(weak.expired());
  assert(world->getNumSkeletons() == 0);
  std::cout << "merged skeleton and both bodies retire with last owner\n";
}
