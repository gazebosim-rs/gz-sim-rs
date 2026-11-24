// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Preserve private signed speed references across a stationary mounted interval.
use super::*;

pub(crate) struct ParkedSpeed {
    pub(super) joints: Vec<JointId>,
    elapsed_s: f64,
    neighbors: Vec<(ColliderHandle, ColliderHandle, Pose)>,
    references: Vec<(RigidBodyHandle, Option<(MultibodyJointHandle, f64)>)>,
}

fn quiet(body: &RigidBody) -> bool {
    body.linvel().length() <= parked_robot::LINEAR_REST
        && body.angvel().length() <= parked_robot::ANGULAR_REST
}

impl RapierBackend {
    pub(super) fn park_speed_references(
        &mut self,
        root: BodyId,
        members: &[RigidBodyHandle],
    ) -> Result<(), PhysicsError> {
        let mut group = ParkedSpeed {
            neighbors: vec![],
            elapsed_s: 0.,
            joints: vec![],
            references: vec![],
        };
        for (id, caps) in &self.joints.speed_caps {
            if members.contains(&self.bodies[&self.joints.records[id].initial.child].0) {
                for (reference, drive) in caps.iter().flatten().flat_map(speed::AxisCap::references)
                {
                    let spin = drive.map(|drive| {
                        let (tree, idx) = self
                            .world
                            .multibody_joints
                            .get(drive)
                            .expect("registered private joint");
                        let offset = tree.link(idx).unwrap().assembly_id();
                        (drive, tree.generalized_velocity()[offset])
                    });
                    group.references.push((reference, spin));
                }
                group.joints.push(*id);
            }
        }
        if group.references.is_empty() {
            return Ok(());
        }
        self.visit_external_pairs(members, |a, b, body| {
            if quiet(body) {
                group.neighbors.push((a, b, *body.position()));
            }
            false
        });
        for &(h, drive) in &group.references {
            if let Some((drive, _)) = drive {
                let (tree, _) = self.world.multibody_joints.get(drive).unwrap();
                if tree.links().any(|l| {
                    !members.contains(&l.rigid_body_handle())
                        && !group
                            .references
                            .iter()
                            .any(|(r, _)| *r == l.rigid_body_handle())
                }) {
                    return Err(PhysicsError::TransitionDeferred {
                        reason: "speed reference shares an external native tree",
                    });
                }
            }
            debug_assert!(self.world.bodies[h].colliders().is_empty());
        }
        for &(h, drive) in &group.references {
            if let Some((drive, _)) = drive {
                let (tree, _) = self.world.multibody_joints.get_mut(drive).unwrap();
                tree.generalized_velocity_mut().fill(0.);
            }
            self.world.bodies[h].sleep();
        }
        self.parked_speed.insert(root, group);
        Ok(())
    }
    pub(super) fn resume_speed_references(&mut self, root: BodyId) {
        if let Some(group) = self.parked_speed.remove(&root) {
            for (h, drive) in group.references {
                if let Some((drive, spin)) = drive {
                    let (tree, idx) = self.world.multibody_joints.get_mut(drive).unwrap();
                    let offset = tree.link(idx).unwrap().assembly_id();
                    tree.generalized_velocity_mut()[offset] = spin;
                    tree.link_mut(idx)
                        .unwrap()
                        .joint
                        .integrate(group.elapsed_s, &[spin]);
                    tree.forward_kinematics(&self.world.bodies, true);
                    let link = tree.link(idx).expect("registered speed reference");
                    let velocity = tree.body_jacobian(idx) * tree.generalized_velocity();
                    let rb = &mut self.world.bodies[h];
                    rb.set_position(*link.local_to_world(), false);
                    rb.set_linvel(Vector::new(velocity[0], velocity[1], velocity[2]), false);
                    rb.set_angvel(Vector::new(velocity[3], velocity[4], velocity[5]), false);
                }
                self.world.islands.wake_up(&mut self.world.bodies, h, true);
            }
        }
    }
    pub(crate) fn resume_parked_speed_groups(&mut self) {
        while let Some(root) = self.parked_speed.keys().next().copied() {
            self.resume_speed_references(root);
            for h in self
                .parked_robots
                .remove(&root)
                .expect("parked speed island")
            {
                self.world.islands.wake_up(&mut self.world.bodies, h, true);
            }
        }
    }
    pub(crate) fn advance_parked_reference_time(&mut self) {
        for group in self.parked_speed.values_mut() {
            group.elapsed_s += self.world.integration_parameters.dt;
        }
    }
    pub(crate) fn has_woken_speed_group(&self) -> bool {
        self.parked_speed
            .keys()
            .any(|id| self.mounted_tier(*id) == crate::ArticulationTier::Dynamic)
    }
    fn visit_external_pairs(
        &self,
        group: &[RigidBodyHandle],
        mut visit: impl FnMut(ColliderHandle, ColliderHandle, &RigidBody) -> bool,
    ) -> bool {
        group
            .iter()
            .flat_map(|h| self.world.bodies[*h].colliders())
            .any(|c| {
                self.world.narrow_phase.contact_pairs_with(*c).any(|pair| {
                    let other = if pair.collider1 == *c {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    self.world.colliders[other].parent().is_some_and(|h| {
                        !group.contains(&h) && visit(*c, other, &self.world.bodies[h])
                    })
                })
            })
    }
    // Resume before Rapier creates a candidate pair against sleeping speed rows.
    // Existing neighbors may remain only while they retain the admitted quiet pose.
    pub(crate) fn approaching_speed_group(&self) -> bool {
        self.parked_speed.iter().any(|(root, group)| {
            self.visit_external_pairs(&self.parked_robots[root], |a, b, body| {
                !group.neighbors.iter().any(|&(x, y, pose)| {
                    x == a
                        && y == b
                        && pose.translation.distance(body.position().translation) <= 1e-6
                        && pose.rotation.angle_between(*body.rotation()) <= 1e-5
                        && quiet(body)
                })
            })
        })
    }
    // Rapier can batch disconnected awake islands into one integration chunk. Sleeping
    // a high-iteration robot can therefore change a flying payload's time integration
    // and the tick at which its broad-phase leaf is reinserted, long before contact.
    pub(super) fn foreign_body_moving(&self, members: &[RigidBodyHandle]) -> bool {
        self.world
            .bodies
            .iter()
            .any(|(h, b)| !b.colliders().is_empty() && !members.contains(&h) && !quiet(b))
    }

    pub(crate) fn speed_park_invalidated(&self) -> bool {
        self.has_woken_speed_group()
            || self.approaching_speed_group()
            || self
                .parked_speed
                .keys()
                .any(|root| self.foreign_body_moving(&self.parked_robots[root]))
    }
    pub(crate) fn parked_speed_trial(&mut self, ordinary: bool) -> Option<PhysicsWorld> {
        if !ordinary || self.speed_park_invalidated() {
            self.resume_parked_speed_groups();
            None
        } else {
            Some(Self::copy_native_world(&self.world))
        }
    }
}
