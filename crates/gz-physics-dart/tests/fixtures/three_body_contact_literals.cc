// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Native references only; retained Rust contact refusals remain untouched.
#include <dart/config.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <ode/odeinit.h>
#include <array>
#include <cassert>
#include <iomanip>
#include <iostream>
#include "body_revolute_identity.hh"
#include "../../src/joint_state.hh"
static_assert(DART_MAJOR_VERSION == 6 && DART_MINOR_VERSION == 13 && DART_PATCH_VERSION == 2);
using namespace dart::dynamics;
static void number(double v) {
  if (std::isfinite(v)) std::cout << v;
  else std::cout << '"' << v << '"';
}
template<class T> static void values(const Eigen::MatrixBase<T> &v) {
  std::cout << '[';
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if(i) std::cout << ',';
    number(v(i));
  }
  std::cout << ']';
}
static void maps(const char *path) {
  std::ifstream in("/proc/self/maps");
  std::ofstream out(path);
  assert(in.good() && out.good());
  out << in.rdbuf();
  assert(!out.fail());
}
using State = Eigen::Matrix<double, 13, 1>;
static State state(const BodyNode &body) {
  State s;
  const Eigen::Quaterniond q(body.getWorldTransform().linear());
  s << body.getWorldTransform().translation(), q.coeffs(), body.getCOMLinearVelocity(), body.getAngularVelocity();
  return s;
}
static void free_velocity(BodyNode &body, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular) {
  auto &joint = static_cast<FreeJoint &>(*body.getParentJoint());
  joint.setAngularVelocity(angular);
  joint.setLinearVelocity(linear - angular.cross(body.getWorldTransform().linear() * body.getLocalCOM()));
}
static void dirty(Skeleton &s) {
  for (std::size_t i = 0; i < s.getNumBodyNodes(); ++i) {
    s.getBodyNode(i)->dirtyJacobian();
    s.getBodyNode(i)->dirtyJacobianDeriv();
  }
}
struct LiteralWorld {
  dart::simulation::WorldPtr world = dart::simulation::World::create("literal-reference");
  std::array<SkeletonPtr, 3> sources;
  std::array<BodyNode *, 3> bodies;
  std::array<RevoluteJoint *, 2> joints{};
  explicit LiteralWorld(bool sphere = true, bool footprint = false) {
    world->setConstraintSolver(std::make_unique<dart::constraint::BoxedLcpConstraintSolver>());
    auto *solver = world->getConstraintSolver();
    solver->setCollisionDetector(dart::collision::OdeCollisionDetector::create());
    solver->getCollisionOption().maxNumContacts = 10000;
    solver->getCollisionOption().collisionFilter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
    world->setGravity(footprint?Eigen::Vector3d(0, 0, -9.8):Eigen::Vector3d::Zero());
    world->setTimeStep(.001);
    for (int i = 0; i < 3; ++i) {
      sources[i] = Skeleton::create("body_" + std::to_string(i));
      sources[i]->setSelfCollisionCheck(false);
      BodyNode::Properties bp;
      bp.mName = sources[i]->getName();
      bp.mInertia.setMass(1.0);
      bp.mInertia.setMoment(Eigen::Matrix3d::Identity() * .004);
      FreeJoint::Properties jp;
      jp.mName = bp.mName + "_FreeJoint";
      auto [joint, body] = sources[i]->createJointAndBodyNodePair<FreeJoint>(nullptr, jp, bp);
      bodies[i] = body;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = footprint?Eigen::Vector3d(0, 0, (sphere?.095:.0045) + i * .5): (i == 2?Eigen::Vector3d(10, 0, 0):Eigen::Vector3d(0, 0, -2.0 * i));
      joint->setTransform(pose);
      ShapePtr shape = sphere?std::static_pointer_cast<Shape>(std::make_shared<SphereShape>(.1)):std::static_pointer_cast<Shape>(std::make_shared<BoxShape>(Eigen::Vector3d(.5, .5, .01)));
      auto *node = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape, bp.mName + ":collision");
      node->getDynamicsAspect()->setFrictionCoeff(1.0);
      world->addSkeleton(sources[i]);
    }
  }
  SkeletonPtr obstacle(const Eigen::Vector3d &position, const Eigen::Vector3d &half) {
    auto s = Skeleton::create("obstacle");
    s->setMobile(false);
    BodyNode::Properties bp;
    bp.mName = "obstacle";
    bp.mInertia.setMass(1.0);
    bp.mInertia.setMoment(Eigen::Vector3d((half.y() * half.y() + half.z() * half.z()) / 3., (half.x() * half.x() + half.z() * half.z()) / 3., (half.x() * half.x() + half.y() * half.y()) / 3.).asDiagonal());
    FreeJoint::Properties jp;
    jp.mName = "obstacle_FreeJoint";
    auto [joint, body] = s->createJointAndBodyNodePair<FreeJoint>(nullptr, jp, bp);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = position;
    joint->setTransform(pose);
    auto *node = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(std::make_shared<BoxShape>(half * 2.), "obstacle:collision");
    node->getDynamicsAspect()->setFrictionCoeff(1.0);
    world->addSkeleton(s);
    return s;
  }
  void join(int child, const Eigen::Vector3d &anchor, bool commit = true,
            const Eigen::Vector3d &axis = Eigen::Vector3d::UnitY()) {
    auto &node = *bodies[child];
    const auto pose = node.getWorldTransform();
    const auto twist = state(node);
    const auto source = node.getSkeleton();
    Eigen::Isometry3d frame = pose;
    frame.translation() = anchor;
    RevoluteJoint::Properties p;
    p.mName = node.getName() + "_body_revolute";
    p.mAxis = pose.linear().transpose() * axis;
    p.mPositionLowerLimits[0] = -1e16;
    p.mPositionUpperLimits[0] = 1e16;
    p.mIsPositionLimitEnforced = true;
    const double inf = std::numeric_limits<double>::infinity();
    p.mForceLowerLimits[0] = p.mVelocityLowerLimits[0] = -inf;
    p.mForceUpperLimits[0] = p.mVelocityUpperLimits[0] = inf;
    auto *joint = node.moveTo<RevoluteJoint>(bodies[child - 1], p);
    joint->setTransformFromParentBodyNode(bodies[child - 1]->getWorldTransform().inverse() * frame);
    joint->setTransformFromChildBodyNode(pose.inverse() * frame);
    Eigen::Vector6d target;
    target << twist.tail<3>(), twist.segment<3>(7);
    gz_physics_dart::setJointTwist(node, target);
    joints[child - 1] = joint;
    if(commit)world->removeSkeleton(source);
  }
  void rollback_leaf(const State &previous) {
    auto destination = bodies[0]->getSkeleton();
    auto *free = bodies[2]->moveTo<FreeJoint>(sources[2], nullptr);
    dirty(*destination);
    dirty(*sources[2]);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = previous.head<3>();
    pose.linear() = Eigen::Quaterniond(previous[6], previous[3], previous[4], previous[5]).toRotationMatrix();
    free->setTransform(pose);
    free_velocity(*bodies[2], previous.segment<3>(7), previous.tail<3>());
    joints[1] = nullptr;
  }
  void commands(const Eigen::Vector3d &force, const Eigen::Vector3d &torque, int hinge, double effort) {
    bodies[0]->addExtForce(force, bodies[0]->getLocalCOM(), false, true);
    bodies[0]->addExtTorque(torque, false);
    joints[hinge]->setForce(0, effort);
  }
  dart::collision::CollisionResult collision() {
    assert(dAllocateODEDataForThread(dAllocateMaskAll) != 0);
    dart::collision::CollisionResult result;
    auto *solver = world->getConstraintSolver();
    solver->getCollisionGroup()->collide(solver->getCollisionOption(), &result);
    return result;
  }
};
static void record(const std::string &stage, LiteralWorld &w) {
  const auto contacts = w.collision();
  std::cout << "{\"kind\":\"state\",\"stage\":\"" << stage << "\",\"time\":" << w.world->getTime() << ",\"contacts\":" << contacts.getNumContacts() << ",\"bodies\":[";
  for(int i = 0; i < 3; ++i){
    if(i) std::cout << ',';
    values(state(*w.bodies[i]));
    assert(state(*w.bodies[i]).allFinite());
  }
  std::cout << "],\"joints\":[";
  for(int i = 0; i < 2; ++i){
    if(i) std::cout << ',';
    if(w.joints[i])                                                                            std::cout << '[' << w.joints[i]->getPosition(0) << ',' << w.joints[i]->getVelocity(0) << ']';
    else std::cout << "null";
  }
  std::cout << "],\"contact_data\":[";
  for(std::size_t i = 0; i < contacts.getNumContacts(); ++i){
    if(i) std::cout << ',';
    const auto &c = contacts.getContact(i);
    auto *a = c.collisionObject1->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
    auto *b = c.collisionObject2->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
    const double response = (a->getSkeleton()->isMobile()?gz_physics_dart::articulatedContactResponse(*a, c.point):0) + (b->getSkeleton()->isMobile()?gz_physics_dart::articulatedContactResponse(*b, c.point):0);
    std::cout << "{\"point\":";
    values(c.point);
    std::cout << ",\"response\":";
    number(response);
    std::cout << '}';
  }
  std::cout << "]}\n";
}
static void near(LiteralWorld &a, LiteralWorld &b) {
  for(int i = 0; i < 3; ++i)assert((state(*a.bodies[i]) - state(*b.bodies[i])).cwiseAbs().maxCoeff() < 1e-12);
  for(int i = 0; i < 2; ++i)if(a.joints[i]&&b.joints[i]){
      assert(std::abs(a.joints[i]->getPosition(0) - b.joints[i]->getPosition(0)) < 1e-12);
      assert(std::abs(a.joints[i]->getVelocity(0) - b.joints[i]->getVelocity(0)) < 1e-12);
    }
}
static void attachment(bool rollback) {
  LiteralWorld candidate, clean;
  candidate.join(1, {0, 0, -1});
  clean.join(1, {0, 0, -1});
  auto floor = candidate.obstacle({10, 0, -.15}, {.25, .25, .1});
  auto clean_floor = clean.obstacle({10, 0, -.15}, {.25, .25, .1});
  for(auto *w:{&candidate, &clean}){
    auto s = w->bodies[1]->getSkeleton();
    s->setVelocities(gz_physics_dart::impulseVelocity(*w->bodies[1], Eigen::Vector6d::Zero()));
  }
  std::cout << "{\"kind\":\"held_commands\",\"force\":[0.2,0,0],\"torque\":[0,0.1,0],\"joint\":0,\"effort\":0.7}\n";
  record("warm-pair-with-isolated-leaf-contact", candidate);
  if(rollback){
    std::array<State, 3> before;
    for(int i = 0; i < 3; ++i) before[i] = state(*candidate.bodies[i]);
    const double q = candidate.joints[0]->getPosition(0), v = candidate.joints[0]->getVelocity(0);
    for(int trial = 0; trial < 2; ++trial){
      candidate.join(2, {0, 0, 0}, false);
      record("prospective-join-" + std::to_string(trial), candidate);
      candidate.rollback_leaf(before[2]);
      record("restored-pair-" + std::to_string(trial), candidate);
      for(int i = 0; i < 3; ++i) assert((state(*candidate.bodies[i]) - before[i]).cwiseAbs().maxCoeff() < 1e-12);
      assert(candidate.joints[0]->getPosition(0) == q&&candidate.joints[0]->getVelocity(0) == v);
    }
    candidate.world->removeSkeleton(floor);
    clean.world->removeSkeleton(clean_floor);
    candidate.join(2, {0, 0, 0});
    clean.join(2, {0, 0, 0});
    for(int tick = 1; tick <= 3; ++tick){
      for(auto *w:{&candidate, &clean}){
        if(tick == 1) w->commands({.2, 0, 0}, {0, .1, 0}, 0, .7);
        w->world->step();
      }
      near(candidate, clean);
      record("future-command-once-" + std::to_string(tick), candidate);
    }
  }
  else{
    candidate.join(2, {0, 0, 0});
    record("joined-at-existing-leaf-contact", candidate);
    for(int tick = 1; tick <= 100; ++tick){
      if(tick == 1) candidate.commands({.2, 0, 0}, {0, .1, 0}, 0, .7);
      candidate.world->step();
      record("native-contact-step-" + std::to_string(tick), candidate);
    }
  }
}
static void approach(bool remove) {
  LiteralWorld candidate, clean;
  for(auto *w:{&candidate, &clean}){
    w->join(1, {0, 0, -1});
    w->join(2, {0, 0, 0});
    free_velocity(*w->bodies[0], {0, 0, -3}, {0, 0, 0});
  }
  auto floor = candidate.obstacle({0, 0, -3.1}, {20, 20, .1});
  auto clean_floor = clean.obstacle({0, 0, -3.1}, {20, 20, .1});
  std::cout << "{\"kind\":\"per_step_commands\",\"force\":[0.01,0,0],\"torque\":[0,0,0],\"joint\":1,\"effort\":0.001}\n";
  bool encountered = false;
  for(int tick = 0; tick < 1000; ++tick){
    if(candidate.collision().getNumContacts() > 0&&!encountered){
      encountered = true;
      record("first-prestep-contact-" + std::to_string(tick), candidate);
      if(remove){
        near(candidate, clean);
        candidate.world->removeSkeleton(floor);
        clean.world->removeSkeleton(clean_floor);
        for(int future = 1; future <= 3; ++future){
          for(auto *w:{&candidate, &clean}){
            if(future == 1) w->commands({.01, 0, 0}, {0, 0, 0}, 1, .001);
            w->world->step();
          }
          near(candidate, clean);
          record("removed-contact-future-command-once-" + std::to_string(future), candidate);
        }
        return;
      }
    }
    for(auto *w:{&candidate, &clean}){
      w->commands({.01, 0, 0}, {0, 0, 0}, 1, .001);
      w->world->step();
    }
    record("native-approach-step-" + std::to_string(tick + 1), candidate);
  }
  std::cout << "{\"kind\":\"approach_result\",\"encountered_contact\":" << encountered << "}\n";
}
// Direct source-predicate audit for this diagonal, identity-inertial-frame box
// corpus only. This does not execute Rust config or admit a three-body contact.
static bool body_eligible(const BodyNode &body, const Eigen::Vector3d &half) {
  const double mass = body.getMass();
  const Eigen::Vector3d moment = body.getInertia().getMoment().diagonal();
  const auto center = body.getLocalCOM();
  const auto bounded = [](double x){
                         return x >= 1e-60&&x <= 1e60;
                       };
  if(!bounded(mass))return false;
  for(int i = 0; i < 3; ++i)if(!bounded(half[i])||!bounded(moment[i])) return false;
  const double total = moment.sum();
  if(moment.maxCoeff() > 1e8 * moment.minCoeff())return false;
  for(int i = 0; i < 3; ++i)if(moment[i] > total - moment[i] + total * 1e-12) return false;
  if(!center.allFinite()||center.norm() > 1e60||mass * center.squaredNorm() / moment.minCoeff() > 1e5)return false;
  const Eigen::Vector3d extent = half + center.cwiseAbs();
  double response = 1. / mass;
  for(int i = 0; i < 3; ++i)response += (extent.squaredNorm() - extent[i] * extent[i]) / moment[i];
  if(!(response <= 1e5))return false;
  const auto pose = body.getWorldTransform();
  const auto aabb = pose.linear().cwiseAbs() * half;
  const Eigen::Quaterniond q(pose.linear());
  if(std::abs(q.squaredNorm() - 1.) > 1e-12) return false;
  if(!pose.matrix().allFinite()||!moment.allFinite()||!std::isfinite(moment.squaredNorm()))return false;
  for(int i = 0; i < 3; ++i){
    const double low = pose.translation()[i] - aabb[i], high = pose.translation()[i] + aabb[i];
    if(std::abs(low) > 5e8||std::abs(high) > 5e8||!std::isnormal(high - low)||high <= low) return false;
  }
  return body.getCOMLinearVelocity().norm() <= 1e60&&body.getAngularVelocity().norm() <= 1e60;
}
static bool point_response(BodyNode &body, const Eigen::Vector3d &point) {
  const auto skeleton = body.getSkeleton();
  const Eigen::MatrixXd inverse = skeleton->getInvMassMatrix();
  const double bound = gz_physics_dart::articulatedContactResponse(body, body.getWorldTransform() * point);
  const bool measured = std::isfinite(bound)&&bound <= 4e5;
  std::cout << "{\"kind\":\"footprint_bound\",\"body\":\"" << body.getName() << "\",\"point\":";
  values(point);
  std::cout << ",\"bound\":";
  number(bound);
  std::cout << ",\"measured\":" << measured << "}\n";
  if(!measured)return false;
  const auto dependent = body.getLinearJacobian(point);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 8);
  for(std::size_t i = 0; i < body.getNumDependentGenCoords(); ++i)jacobian.col(body.getDependentGenCoordIndex(i)) = dependent.col(i);
  const Eigen::Matrix3d expected = jacobian * inverse * jacobian.transpose();
  Eigen::Matrix3d observed;
  for(int axis = 0; axis < 3; ++axis){
    skeleton->clearConstraintImpulses();
    assert(skeleton->getVelocityChanges().isZero(0));
    const Eigen::Vector3d force = body.getWorldTransform().linear().transpose() * Eigen::Vector3d::Unit(axis);
    Eigen::Vector6d impulse;
    impulse << point.cross(force), force;
    skeleton->updateBiasImpulse(&body, impulse);
    skeleton->updateVelocityChange();
    const auto actual = skeleton->getVelocityChanges();
    const Eigen::VectorXd predicted = inverse * jacobian.transpose().col(axis);
    const double error = (actual - predicted).norm();
    std::cout << "{\"kind\":\"coordinate_response\",\"body\":\"" << body.getName() << "\",\"axis\":" << axis << ",\"actual\":";
    values(actual);
    std::cout << ",\"expected\":";
    values(predicted);
    std::cout << ",\"error\":" << error << ",\"expected_norm\":" << predicted.norm() << "}\n";
    assert(actual.allFinite()&&error < 1e-10 * (1. + predicted.norm()));
    const auto delta = body.getBodyVelocityChange();
    observed.col(axis) = body.getWorldTransform().linear() * (delta.tail<3>() + delta.head<3>().cross(point));
    skeleton->clearConstraintImpulses();
    assert(skeleton->getVelocityChanges().isZero(0));
  }
  const double error = (observed - expected).norm(), symmetry = (observed - observed.transpose()).cwiseAbs().maxCoeff();
  std::cout << "{\"kind\":\"point_response\",\"body\":\"" << body.getName() << "\",\"actual\":";
  values(observed);
  std::cout << ",\"expected\":";
  values(expected);
  std::cout << ",\"error\":" << error << ",\"expected_norm\":" << expected.norm() << ",\"symmetry\":" << symmetry << "}\n";
  assert(observed.allFinite()&&error < 1e-10 * (1. + expected.norm()));
  assert(symmetry < 1e-6);
  assert((observed.diagonal().array() <= bound).all());
  return true;
}
static void corpus(int sample) {
  assert(sample >= 0&&sample < 126);
  const double length = std::pow(10., sample % 7 - 3), ratio = std::pow(10., sample / 7 % 9 * 5 - 20);
  const bool offset = sample >= 63;
  std::cout << "{\"kind\":\"corpus_input\",\"sample\":" << sample << ",\"length\":" << length << ",\"mass_ratio\":" << ratio << ",\"offset\":" << offset << "}\n";
  auto skeleton = Skeleton::create("corpus-three");
  std::array<BodyNode *, 3>bodies;
  FreeJoint::Properties fp;
  fp.mName = "root_free";
  BodyNode::Properties bp;
  bp.mName = "root";
  auto [free, root] = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr, fp, bp);
  bodies[0] = root;
  for(int i = 1; i < 3; ++i){
    RevoluteJoint::Properties jp;
    jp.mName = "hinge-" + std::to_string(i);
    jp.mAxis = Eigen::Vector3d::UnitZ();
    bp.mName = "child-" + std::to_string(i);
    auto [joint, body] = skeleton->createJointAndBodyNodePair<RevoluteJoint>(bodies[i - 1], jp, bp);
    bodies[i] = body;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = .5 * length;
    joint->setTransformFromParentBodyNode(pose);
    pose.translation().z() = -.5 * length;
    joint->setTransformFromChildBodyNode(pose);
    joint->setPosition(0, .23);
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(.43, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  free->setTransform(pose);
  bool eligible = true;
  for(int i = 0; i < 3; ++i){
    Inertia inertia;
    inertia.setMass(i == 0?120. * ratio:.6);
    Eigen::Vector3d moment = i == 0?Eigen::Vector3d(2.501, 2.501, 5.):Eigen::Vector3d(.032, .032, .00012);
    moment *= length * length * (i == 0?ratio:1.);
    inertia.setMoment(moment.asDiagonal());
    inertia.setLocalCOM(Eigen::Vector3d(.03, -.02, .01) * length * offset);
    bodies[i]->setInertia(inertia);
    eligible = body_eligible(*bodies[i], Eigen::Vector3d(.25, .25, .005) * length)&&eligible;
  }
  std::cout << "{\"kind\":\"body_predicate_audit\",\"eligible\":" << eligible << ",\"rust_config_executed\":false}\n";
  if(!eligible)return;
  assert(skeleton->getNumDofs() == 8);
  int measured = 0, refused = 0;
  for(auto *body:bodies)for(int corner = 0; corner < 4; ++corner){
      const Eigen::Vector3d point = Eigen::Vector3d(corner % 2?.25:-.25, corner / 2?.25:-.25, -.005) * length;
      if(point_response(*body, point)) ++measured;
      else ++refused;
    }
  std::cout << "{\"kind\":\"corpus_result\",\"measured_points\":" << measured << ",\"refused_points\":" << refused << "}\n";
}
static void footprint(bool sphere) {
  LiteralWorld w(sphere, true);
  w.join(1, {0, 0, .25});
  w.join(2, {0, 0, .75});
  auto floor = w.obstacle({0, 0, -.1}, {20, 20, .1});
  for(auto *body:w.bodies){
    if(sphere){
      for(int axis = 0; axis < 3; ++axis) for(double sign:{-1., 1.}) point_response(*body, Eigen::Vector3d::Unit(axis) * (.1 * sign));
    }
    else{
      for(int corner = 0; corner < 4; ++corner) point_response(*body, {corner % 2?.25:-.25, corner / 2?.25:-.25, -.005});
    }
  }
  for(int tick = 0; tick <= 100; ++tick){
    if(tick) w.world->step();
    record("footprint-contact-" + std::to_string(tick), w);
  }
}
// Match the bridge's immobile FreeJoint box, not the earlier WeldJoint ground.
static void above_condition() {
  LiteralWorld w(false, true);
  w.world->setGravity(Eigen::Vector3d::Zero());
  for (int i = 0; i < 3; ++i) {
    Inertia inertia;
    inertia.setMass(i == 0 ? 120.0 : 0.6);
    inertia.setMoment(Eigen::Matrix3d::Identity() * 1e-4 *
                      (i == 2 ? 74282549781.30127 : 1.0));
    w.bodies[i]->setInertia(inertia);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 0.0051 + 0.5 * i;
    static_cast<FreeJoint *>(w.bodies[i]->getParentJoint())->setTransform(pose);
    free_velocity(*w.bodies[i], {0, 0, -0.1}, {0, 0, 0});
  }
  w.join(1, {0, 0, 0.5051});
  w.join(2, {0, 0, 1.0051}, true, Eigen::Vector3d::UnitX());
  auto ground = w.obstacle({0, 0, -0.05}, {50, 50, 0.05});
  std::cout << "{\"kind\":\"per_step_commands\",\"force\":[0.00001,0,0],\"torque\":[0,0,0],\"joint\":1,\"effort\":0.00001}\n";
  for (int tick = 0; tick <= 100; ++tick) {
    if (tick) {
      w.commands({1e-5, 0, 0}, {0, 0, 0}, 1, 1e-5);
      w.world->step();
    }
    const Eigen::MatrixXd mass = w.bodies[0]->getSkeleton()->getMassMatrix();
    const Eigen::VectorXd scale = mass.diagonal().cwiseSqrt();
    const Eigen::MatrixXd normalized = mass.array() / (scale * scale.transpose()).array();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
    assert(spectrum.info() == Eigen::Success);
    std::cout << "{\"kind\":\"matrix_condition\",\"tick\":" << tick << ",\"condition\":";
    number(spectrum.eigenvalues().maxCoeff() / spectrum.eigenvalues().minCoeff());
    std::cout << "}\n";
    record("above-condition-approach-" + std::to_string(tick), w);
  }
}
int main(int argc, char **argv) {
  assert(argc == 5);
  if (!check_dart_library()) return 2;
  std::cout << std::setprecision(17) << std::boolalpha << std::unitbuf;
  const std::string mode = argv[1];
  maps(argv[3]);
  if (mode == "attach-live") attachment(false);
  else if (mode == "attach-rollback") attachment(true);
  else if (mode == "approach-live") approach(false);
  else if (mode == "approach-remove") approach(true);
  else if (mode == "footprint-sphere") footprint(true);
  else if (mode == "footprint-box") footprint(false);
  else if (mode == "corpus") corpus(std::stoi(argv[2]));
  else if (mode == "above-condition") above_condition();
  else return 3;
  maps(argv[4]);
}
