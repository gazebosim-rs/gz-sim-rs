// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <iostream>
#include "body_revolute_identity.hh"

int main() {
  using namespace dart::dynamics;
  auto skeleton = Skeleton::create("response-probe");
  if (!check_dart_library()) return 2;
  auto [root_joint, root] = skeleton->createJointAndBodyNodePair<FreeJoint>();
  RevoluteJoint::Properties first;
  first.mName = "first";
  first.mAxis = Eigen::Vector3d::UnitX();
  BodyNode::Properties upper_properties;
  upper_properties.mName = "upper";
  auto [upper_joint, upper] = skeleton->createJointAndBodyNodePair<RevoluteJoint>(root, first, upper_properties);
  RevoluteJoint::Properties second;
  second.mName = "second";
  second.mAxis = Eigen::Vector3d::UnitY();
  BodyNode::Properties lower_properties;
  lower_properties.mName = "lower";
  auto [lower_joint, lower] = skeleton->createJointAndBodyNodePair<RevoluteJoint>(upper, second, lower_properties);
  Eigen::Isometry3d root_pose = Eigen::Isometry3d::Identity();
  root_pose.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  root_joint->setTransform(root_pose);
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().z() = 1.0;
  upper_joint->setTransformFromParentBodyNode(offset);
  lower_joint->setTransformFromParentBodyNode(offset);
  upper_joint->setPosition(0, 0.3);
  lower_joint->setPosition(0, -0.5);
  assert(skeleton->getNumDofs() == 8);
  Eigen::VectorXd initial = Eigen::VectorXd::LinSpaced(8, -0.3, 0.4);
  skeleton->setVelocities(initial);
  const Eigen::MatrixXd mass = skeleton->getMassMatrix();
  Eigen::LDLT<Eigen::MatrixXd> factor(mass);
  assert(factor.info() == Eigen::Success && factor.isPositive());
  for (const auto &entry : {std::pair<BodyNode *, std::size_t>{root, 6}, {upper, 7}, {lower, 8}}) {
    auto *node = entry.first;
    const Eigen::Vector3d local_point(0.2, -0.3, 0.4);
    const auto local_jacobian = node->getWorldJacobian(local_point);
    const auto &indices = node->getDependentGenCoordIndices();
    assert(indices.size() == entry.second);
    assert(local_jacobian.cols() == static_cast<Eigen::Index>(indices.size()));
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 8);
    for (std::size_t i = 0; i < indices.size(); ++i) {
      assert(indices[i] < 8);
      jacobian.col(indices[i]) = local_jacobian.col(i);
    }
    const Eigen::MatrixXd linear = jacobian.bottomRows(3);
    const Eigen::Matrix3d response = linear * factor.solve(linear.transpose());
    assert(response.allFinite());
    const double bound = response.trace();
    assert(std::isfinite(bound) && bound > 0.0);
    const Eigen::Vector3d previous = node->getLinearVelocity(local_point);
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3d impulse = Eigen::Vector3d::Unit(axis);
      const Eigen::VectorXd delta = factor.solve(linear.transpose() * impulse);
      skeleton->setVelocities(initial + delta);
      const Eigen::Vector3d observed = node->getLinearVelocity(local_point) - previous;
      assert((observed - response * impulse).norm() < 1e-10);
      assert(impulse.dot(observed) <= bound + 1e-10);
      skeleton->setVelocities(initial);
    }
  }
  std::cout << "expanded dependent Jacobians match native point impulse response\n";
}
