// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stationary prescribed pose for an unloaded robot with one fixed world mount.
//! Sleeping the complete native island keeps its constraints, finite collision mass,
//! and warm starts intact. Native collision and commanded loads wake it before solving.
use super::*;
use crate::ArticulationTier;

// Maximum velocity discarded by a stationary cut (SI). Both are checked on every link.
const LINEAR_REST: f64 = 0.00025;
const ANGULAR_REST: f64 = 0.001;

fn defer(reason: &'static str) -> PhysicsError {
    PhysicsError::TransitionDeferred { reason }
}

impl RapierBackend {
    fn robot_parent_joint(&self, link: BodyId) -> Option<(JointId, &Joint)> {
        self.joints
            .records
            .iter()
            .find(|(_, j)| j.initial.robot && j.initial.child == link)
            .map(|(id, j)| (*id, j))
    }
    pub(crate) fn mounted_robot_root(&self, root: BodyId) -> bool {
        self.bodies
            .get(&root)
            .is_some_and(|e| self.world.multibody_joints.rigid_body_link(e.0).is_none())
            && self
                .robot_parent_joint(root)
                .is_some_and(|(_, j)| j.initial.parent.is_none())
    }
    pub(crate) fn robot_member(&self, root: BodyId, mut body: BodyId) -> bool {
        // Impulse graphs can be made programmatically as well as through the SDF tree loader.
        for _ in 0..=self.joints.records.len() {
            if body == root {
                return true;
            }
            let Some(parent) = self
                .robot_parent_joint(body)
                .and_then(|(_, j)| j.initial.parent)
            else {
                return false;
            };
            body = parent;
        }
        false
    }
    pub(crate) fn mounted_members(&self, root: BodyId) -> Vec<BodyId> {
        self.joints
            .records
            .values()
            .filter(|j| j.initial.robot && self.robot_member(root, j.initial.child))
            .map(|j| j.initial.child)
            .collect()
    }
    pub(crate) fn mounted_tier(&self, root: BodyId) -> ArticulationTier {
        if self.parked_robots.get(&root).is_some_and(|hs| {
            hs.iter()
                .all(|h| self.world.bodies.get(*h).is_some_and(|b| b.is_sleeping()))
        }) {
            ArticulationTier::Prescribed
        } else {
            ArticulationTier::Dynamic
        }
    }
    pub(crate) fn retire_woken_robots(&mut self) {
        self.parked_robots.retain(|_, hs| {
            hs.iter()
                .all(|h| self.world.bodies.get(*h).is_some_and(|b| b.is_sleeping()))
        });
    }

    // Evaluate free directions on the authored joint manifold. Solver pivot drift
    // can tilt a nominally horizontal unloaded slide by microradians.
    fn mounted_axis(&self, joint: Construction) -> Vector {
        let mut axis = joint.frame1.rotation * Vector::X;
        let mut parent = joint.parent;
        while let Some(id) = parent {
            let (index, j) = self.robot_parent_joint(id).expect("mounted ancestor");
            let c = j.initial;
            axis = c.frame2.rotation.inverse() * axis;
            if matches!(c.motion, JointMotion::Angular) && !c.authored_weld() {
                axis = Rotation::from_rotation_x(
                    self.world_joint_state(index)
                        .expect("hinge coordinate")
                        .position,
                ) * axis;
            }
            axis = c.frame1.rotation * axis;
            parent = c.parent;
        }
        axis
    }

    pub(crate) fn validate_mounted_park(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<Vec<RigidBodyHandle>, PhysicsError> {
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let members = self.mounted_members(root);
        if members.iter().collect::<BTreeSet<_>>().len() != members.len()
            || self.joints.records.values().any(|j| {
                let c = j.initial;
                (members.contains(&c.child) || c.parent.is_some_and(|p| members.contains(&p)))
                    && (!c.robot
                        || !members.contains(&c.child)
                        || c.parent.is_some_and(|p| !members.contains(&p)))
            })
        {
            return Err(defer("mounted robot has an external or duplicate joint"));
        }
        let handles: Vec<_> = members.iter().map(|id| self.bodies[id].0).collect();
        for &h in &handles {
            let body = &self.world.bodies[h];
            for c in body.colliders() {
                if self.world.narrow_phase.contact_pairs_with(*c).any(|pair| {
                    let other = if pair.collider1 == *c {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    (pair.total_impulse_magnitude() > 0.
                        || pair
                            .solver_manifolds()
                            .iter()
                            .any(|m| m.data.solver_contacts.iter().any(|c| c.dist <= 0.)))
                        && !self.world.colliders[other]
                            .parent()
                            .is_some_and(|p| handles.contains(&p))
                }) {
                    return Err(defer("mounted robot has an external contact"));
                }
            }
        }
        for (&id, &h) in members.iter().zip(&handles) {
            let body = &self.world.bodies[h];
            if !body.is_dynamic()
                || body.linvel().length() > LINEAR_REST
                || body.angvel().length() > ANGULAR_REST
                || self.pending_wrenches.contains(&id)
            {
                return Err(defer("mounted robot has motion or a pending wrench"));
            }
            let (joint_id, joint) = self
                .robot_parent_joint(id)
                .expect("collected attached link");
            let c = joint.initial;
            let Handle::Impulse(handle) = joint.handle else {
                return Err(defer("expected an impulse joint"));
            };
            if !self
                .world
                .impulse_joints
                .get(handle)
                .expect("registered joint")
                .data
                .motor_axes
                .is_empty()
                || self.joints.efforts.contains_key(&joint_id)
                || self.joints.universal_efforts.contains_key(&joint_id)
            {
                return Err(defer("mounted robot has a pending input or active motor"));
            }
            if c.authored_weld() {
                continue;
            }
            let q = self
                .world_joint_state(joint_id)
                .ok_or_else(|| defer("unsupported mounted parking joint"))?
                .position;
            let frame = c.parent.map_or(Pose::IDENTITY, |p| {
                *self.world.bodies[self.bodies[&p].0].position()
            }) * c.frame1;
            let axis = self.mounted_axis(c);
            let spring = c.spring.map_or(0., |s| {
                s.effort(JointState {
                    position: q,
                    velocity: 0.,
                })
            });
            let mut generalized = spring;
            let mut scale = spring.abs();
            for (&descendant, &dh) in members.iter().zip(&handles) {
                if !self.robot_member(id, descendant) {
                    continue;
                }
                let rb = &self.world.bodies[dh];
                let force = self.world.gravity * rb.mass();
                let arm = rb.center_of_mass() - frame.translation;
                let load = match c.motion {
                    JointMotion::Angular => arm.cross(force),
                    JointMotion::Prismatic => force,
                    _ => return Err(defer("unsupported mounted parking joint")),
                };
                generalized += load.dot(axis);
                scale += load.length();
            }
            // A free coordinate must do zero gravitational virtual work. A stop
            // may support only the signed load directed into its active face.
            if generalized.abs() > 1e-8 * scale
                && !c.limits.is_some_and(|[lo, hi]| {
                    let stop = if generalized < 0. { lo } else { hi };
                    (q - stop).abs() <= 2e-5
                })
            {
                return Err(defer(
                    "gravity or spring preload can move an unrestrained mounted coordinate",
                ));
            }
        }
        // The first native solve establishes narrow-phase contact pairs and processes
        // new restraints. Until then an apparently unloaded mount can already touch a load.
        if self.joints.construction_open() {
            return Err(defer(
                "mounted parking needs an initial committed contact solve",
            ));
        }
        Ok(handles)
    }
    pub(crate) fn switch_mounted_robot(
        &mut self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        match target {
            ArticulationTier::Prescribed if self.mounted_tier(root) == target => {}
            ArticulationTier::Prescribed => {
                let handles = self.validate_mounted_park(root, dt)?;
                for &h in &handles {
                    self.world.bodies[h].sleep();
                }
                self.parked_robots.insert(root, handles);
            }
            ArticulationTier::Dynamic => {
                if let Some(handles) = self.parked_robots.remove(&root) {
                    for h in handles {
                        self.world.islands.wake_up(&mut self.world.bodies, h, true);
                    }
                }
            }
        }
        Ok(())
    }
}
