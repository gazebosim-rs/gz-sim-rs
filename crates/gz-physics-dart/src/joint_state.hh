// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

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

// All-coordinate response for the bounded two-body contact domain. The existing
// single-body bounds do not account for an articulated body's ancestor rates.
inline double articulatedContactResponse(const dart::dynamics::BodyNode &body,
                                         const Eigen::Vector3d &point) {
  const double refused = std::numeric_limits<double>::infinity();
  const auto skeleton = body.getSkeleton();
  const Eigen::MatrixXd mass = skeleton->getMassMatrix();
  if (!point.allFinite() || !mass.allFinite() ||
      !(mass.array() == mass.transpose().array()).all() ||
      !(mass.diagonal().array() > 0.0).all())
    return refused;
  const Eigen::VectorXd scale = mass.diagonal().cwiseSqrt();
  const Eigen::VectorXd inverseScale = scale.cwiseInverse();
  const Eigen::MatrixXd scaling = scale * scale.transpose();
  const Eigen::MatrixXd normalized = mass.array() / scaling.array();
  if (!normalized.allFinite()) return refused;
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
  // This is an additional articulated-contact restriction, independent of the
  // principal-inertia anisotropy bound on each admitted body.
  if (spectrum.info() != Eigen::Success ||
      !(spectrum.eigenvalues().minCoeff() > 0.0) ||
      !(spectrum.eigenvalues().maxCoeff() /
            spectrum.eigenvalues().minCoeff() <= 1e8))
    return refused;
  const Eigen::MatrixXd inverse = skeleton->getInvMassMatrix();
  if (!inverse.allFinite() ||
      !(inverse.array() == inverse.transpose().array()).all())
    return refused;
  const Eigen::MatrixXd normalizedInverse = inverse.array() * scaling.array();
  if (!normalizedInverse.allFinite()) return refused;
  const Eigen::LLT<Eigen::MatrixXd> factor(normalizedInverse);
  if (factor.info() != Eigen::Success) return refused;
  const double error = (Eigen::MatrixXd::Identity(mass.rows(), mass.cols()) -
                        normalized * normalizedInverse).norm();
  if (!(error < 1.0)) return refused;

  const Eigen::Vector3d local = body.getWorldTransform().inverse() * point;
  const auto dependent = body.getLinearJacobian(local);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, mass.cols());
  for (std::size_t i = 0; i < body.getNumDependentGenCoords(); ++i)
    jacobian.col(body.getDependentGenCoordIndex(i)) = dependent.col(i);
  // CK is similar to an SPD matrix. Its eigenvalues lie within `error` of one,
  // so C^-1 <= K/(1-error). This also bounds the native inverse response itself.
  const Eigen::MatrixXd response =
      jacobian * inverseScale.asDiagonal() * factor.matrixL();
  const double bound = response.squaredNorm() / (1.0 - error);
  return std::isfinite(bound) ? bound : refused;
}

}  // namespace gz_physics_dart
