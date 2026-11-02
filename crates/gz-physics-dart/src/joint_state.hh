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
#include <memory>
#include <optional>
#include <utility>
#include <vector>

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

// Skeleton mass and inverse mass depend on configuration/inertia, not the
// contact point. contactsWithin only reads them: no step, edit, collision query,
// solver call or callback occurs while its local pass object is alive.
class ArticulatedContactFactorization {
 public:
  explicit ArticulatedContactFactorization(
      const dart::dynamics::Skeleton &skeleton) : owner_(&skeleton) {
    const Eigen::MatrixXd mass = skeleton.getMassMatrix();
    if (!mass.allFinite() ||
        !(mass.array() == mass.transpose().array()).all() ||
        !(mass.diagonal().array() > 0.0).all())
      return;
    const Eigen::VectorXd scale = mass.diagonal().cwiseSqrt();
    inverseScale_ = scale.cwiseInverse();
    const Eigen::MatrixXd scaling = scale * scale.transpose();
    const Eigen::MatrixXd normalized = mass.array() / scaling.array();
    if (!normalized.allFinite()) return;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
    // This is an additional articulated-contact restriction, independent of the
    // principal-inertia anisotropy bound on each admitted body.
    if (spectrum.info() != Eigen::Success ||
        !(spectrum.eigenvalues().minCoeff() > 0.0) ||
        !(spectrum.eigenvalues().maxCoeff() /
              spectrum.eigenvalues().minCoeff() <= 1e8))
      return;
    const Eigen::MatrixXd inverse = skeleton.getInvMassMatrix();
    if (!inverse.allFinite() ||
        !(inverse.array() == inverse.transpose().array()).all())
      return;
    const Eigen::MatrixXd normalizedInverse = inverse.array() * scaling.array();
    if (!normalizedInverse.allFinite()) return;
    factor_.emplace(normalizedInverse);
    if (factor_->info() != Eigen::Success) return;
    error_ = (Eigen::MatrixXd::Identity(mass.rows(), mass.cols()) -
              normalized * normalizedInverse).norm();
    if (!(error_ < 1.0)) return;
    columns_ = mass.cols();
    valid_ = true;
  }

  double response(const dart::dynamics::BodyNode &body,
                  const Eigen::Vector3d &point) const {
    const double refused = std::numeric_limits<double>::infinity();
    if (!point.allFinite() || !valid_ || body.getSkeleton().get() != owner_)
      return refused;
    const Eigen::Vector3d local = body.getWorldTransform().inverse() * point;
    const auto dependent = body.getLinearJacobian(local);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, columns_);
    for (std::size_t i = 0; i < body.getNumDependentGenCoords(); ++i)
      jacobian.col(body.getDependentGenCoordIndex(i)) = dependent.col(i);
    // CK is similar to an SPD matrix. Its eigenvalues lie within `error` of one,
    // so C^-1 <= K/(1-error). This also bounds the native inverse response itself.
    const Eigen::MatrixXd response =
        jacobian * inverseScale_.asDiagonal() * factor_->matrixL();
    const double bound = response.squaredNorm() / (1.0 - error_);
    return std::isfinite(bound) ? bound : refused;
  }

 private:
  const dart::dynamics::Skeleton *owner_;
  Eigen::Index columns_ = 0;
  Eigen::VectorXd inverseScale_;
  std::optional<Eigen::LLT<Eigen::MatrixXd>> factor_;
  double error_ = std::numeric_limits<double>::infinity();
  bool valid_ = false;
};

// Standalone callers still calculate and validate their own fresh factorization.
inline double articulatedContactResponse(const dart::dynamics::BodyNode &body,
                                         const Eigen::Vector3d &point) {
  if (!point.allFinite()) return std::numeric_limits<double>::infinity();
  return ArticulatedContactFactorization(*body.getSkeleton()).response(body, point);
}

// Default construction owns no heap. Entries are created only for articulated
// mobile contacts that reach inverse-mass evaluation. Skeletons are kept alive
// by their world/bodies for the entire pass; this object never escapes the call.
class ArticulatedContactPass {
 public:
  double response(const dart::dynamics::BodyNode &body,
                  const Eigen::Vector3d &point) {
    if (!point.allFinite()) return std::numeric_limits<double>::infinity();
    const auto *owner = body.getSkeleton().get();
    for (const auto &entry : entries_)
      if (entry.first == owner) return entry.second->response(body, point);
    entries_.emplace_back(owner,
        std::make_unique<ArticulatedContactFactorization>(*owner));
    return entries_.back().second->response(body, point);
  }

 private:
  std::vector<std::pair<const dart::dynamics::Skeleton *,
                        std::unique_ptr<ArticulatedContactFactorization>>> entries_;
};

}  // namespace gz_physics_dart
