// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

namespace gz_physics_dart {

// Set the revolute rate that best fits the requested COM twist, keeping the
// contribution of every ancestor coordinate fixed.
inline void setJointTwist(dart::dynamics::BodyNode &body,
                          const Eigen::Vector6d &twist) {
  auto &joint = *body.getParentJoint();
  const auto skeleton = body.getSkeleton();
  const dart::math::Jacobian jacobian = body.getWorldJacobian(body.getLocalCOM());
  Eigen::Vector6d remaining = twist;
  Eigen::Vector6d column = Eigen::Vector6d::Zero();
  const auto jointIndex = joint.getIndexInSkeleton(0);
  for (std::size_t i = 0; i < body.getNumDependentGenCoords(); ++i) {
    const auto index = body.getDependentGenCoordIndex(i);
    if (index == jointIndex) {
      column = jacobian.col(i);
    } else {
      remaining -= jacobian.col(i) * skeleton->getVelocity(index);
    }
  }
  joint.setVelocity(0, column.dot(remaining) / column.squaredNorm());
}

// Expand the body's dependent Jacobian into the skeleton's coordinate order.
inline Eigen::VectorXd impulseVelocity(const dart::dynamics::BodyNode &body,
                                       const Eigen::Vector6d &impulse) {
  const auto skeleton = body.getSkeleton();
  const dart::math::Jacobian jacobian = body.getWorldJacobian(body.getLocalCOM());
  const Eigen::VectorXd dependent = jacobian.transpose() * impulse;
  Eigen::VectorXd generalized = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  for (std::size_t i = 0; i < body.getNumDependentGenCoords(); ++i) {
    generalized[body.getDependentGenCoordIndex(i)] = dependent[i];
  }
  return skeleton->getVelocities() + skeleton->getInvMassMatrix() * generalized;
}

}  // namespace gz_physics_dart
