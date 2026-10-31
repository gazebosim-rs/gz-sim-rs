// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <cassert>
#include <iostream>
#include "body_revolute_identity.hh"
#include "../../src/joint_state.hh"

// Exercise both sides of the new normalized-mass condition ceiling with
// individually valid isotropic bodies, independently of shape anisotropy.
static void conditionBoundary() {
  using namespace dart::dynamics;
  for (double ratio : {2.49e7, 2.51e7}) {
    auto skeleton = Skeleton::create("condition-boundary");
    auto [rootJoint, root] = skeleton->createJointAndBodyNodePair<FreeJoint>();
    RevoluteJoint::Properties properties;
    properties.mName = "hinge";
    properties.mAxis = Eigen::Vector3d::UnitZ();
    BodyNode::Properties childProperties;
    childProperties.mName = "child";
    auto [joint, child] = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
        root, properties, childProperties);
    Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(Eigen::Matrix3d::Identity() * 1e-4);
    root->setInertia(inertia);
    inertia.setMoment(Eigen::Matrix3d::Identity() * 1e-4 * ratio);
    child->setInertia(inertia);
    const Eigen::MatrixXd mass = skeleton->getMassMatrix();
    const Eigen::VectorXd scale = mass.diagonal().cwiseSqrt();
    const Eigen::MatrixXd normalized =
        mass.array() / (scale * scale.transpose()).array();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
    assert(spectrum.info() == Eigen::Success);
    const double condition = spectrum.eigenvalues().maxCoeff() /
                             spectrum.eigenvalues().minCoeff();
    const bool admitted = ratio == 2.49e7;
    assert((condition <= 1e8) == admitted);
    const double bound = gz_physics_dart::articulatedContactResponse(
        *root, Eigen::Vector3d(0.01, 0.0, 0.0));
    assert(std::isfinite(bound) == admitted);
    if (admitted) assert(bound > 0.0 && bound <= 4e5);
  }
}

int main() {
  using namespace dart::dynamics;
  if (!check_dart_library()) return 2;
  conditionBoundary();
  std::size_t admitted = 0, refused = 0, excluded = 0;
  for (int sample = 0; sample < 126; ++sample) {
    const double length = std::pow(10.0, sample % 7 - 3);
    const double ratio = std::pow(10.0, sample / 7 % 9 * 5 - 20);
    const bool offset = sample >= 63;
    // Rust refuses these masses before any native body exists. Raw native
    // diagnostics below that boundary are not part of this admission domain.
    if (120.0 * ratio < 1e-5) {
      excluded += 8;
      continue;
    }
    auto skeleton = Skeleton::create("contact-response");
    auto [rootJoint, root] = skeleton->createJointAndBodyNodePair<FreeJoint>();
    RevoluteJoint::Properties properties;
    properties.mName = "hinge";
    properties.mAxis = Eigen::Vector3d::UnitZ();
    BodyNode::Properties childProperties;
    childProperties.mName = "child";
    auto [joint, child] = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
        root, properties, childProperties);
    Inertia inertia;
    inertia.setMass(120.0 * ratio);
    inertia.setMoment(Eigen::Vector3d(2.501, 2.501, 5.0).asDiagonal() * length * length * ratio);
    inertia.setLocalCOM(Eigen::Vector3d(0.03, -0.02, 0.01) * length * offset);
    root->setInertia(inertia);
    inertia.setMass(0.6);
    inertia.setMoment(Eigen::Vector3d(0.032, 0.032, 0.00012).asDiagonal() * length * length);
    child->setInertia(inertia);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(0.43, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    rootJoint->setTransform(pose);
    pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 0.5 * length;
    joint->setTransformFromParentBodyNode(pose);
    pose.translation().z() = -0.5 * length;
    joint->setTransformFromChildBodyNode(pose);
    joint->setPosition(0, 0.23);
    for (auto *body : {root, child}) {
      for (int corner = 0; corner < 4; ++corner) {
        const Eigen::Vector3d local = Eigen::Vector3d(
            corner % 2 ? 0.25 : -0.25, corner / 2 ? 0.25 : -0.25, -0.005) * length;
        const double bound = gz_physics_dart::articulatedContactResponse(
            *body, body->getWorldTransform() * local);
        if (!std::isfinite(bound) || bound > 4e5) {
          ++refused;
          continue;
        }
        ++admitted;
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, skeleton->getNumDofs());
        const auto dependent = body->getLinearJacobian(local);
        for (std::size_t i = 0; i < body->getNumDependentGenCoords(); ++i)
          jacobian.col(body->getDependentGenCoordIndex(i)) = dependent.col(i);
        const Eigen::Matrix3d expected = jacobian * skeleton->getInvMassMatrix() * jacobian.transpose();
        Eigen::Matrix3d observed;
        for (int axis = 0; axis < 3; ++axis) {
          const Eigen::Vector3d force = body->getWorldTransform().linear().transpose() * Eigen::Vector3d::Unit(axis);
          Eigen::Vector6d impulse;
          impulse << local.cross(force), force;
          skeleton->clearConstraintImpulses();
          skeleton->updateBiasImpulse(body, impulse);
          skeleton->updateVelocityChange();
          const Eigen::Vector6d delta = body->getBodyVelocityChange();
          observed.col(axis) = body->getWorldTransform().linear() *
              (delta.tail<3>() + delta.head<3>().cross(local));
        }
        skeleton->clearConstraintImpulses();
        assert(observed.allFinite());
        if (!((observed - expected).norm() < 1e-10 * (1.0 + expected.norm())))
          std::cerr << "sample=" << sample << " root_mass=" << 120*ratio << " offset=" << offset
                    << " body=" << body->getName() << " bound=" << bound
                    << " difference=" << (observed-expected).norm()
                    << " norm=" << expected.norm() << " asymmetry=" << (observed-observed.transpose()).norm() << '\n';
        assert((observed - expected).norm() < 1e-10 * (1.0 + expected.norm()));
        // This is DART's own absolute contact-block symmetry tolerance.
        assert((observed - observed.transpose()).cwiseAbs().maxCoeff() < 1e-6);
        assert((observed.diagonal().array() <= bound).all());
      }
    }
  }
  assert(admitted > 0 && excluded > 0);
  std::cout << "bounded full-coordinate contact response matches native impulse propagation: "
            << admitted << " admitted points, " << refused << " response refusals, "
            << excluded << " points below the body mass floor\n";
}
