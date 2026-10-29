// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl Joints {
    pub(crate) fn finish_velocities(&self, world: &mut PhysicsWorld) {
        if world.multibody_joints.iter().next().is_none() {
            return;
        }
        let mut initialized = BTreeSet::new();
        let mut updates = Vec::new();
        for joint in self.records.values() {
            let Handle::Reduced(handle) = joint.handle else {
                continue;
            };
            let (multibody, _) = world
                .multibody_joints
                .get_mut(handle)
                .expect("registered articulation");
            let root = multibody
                .link(0)
                .expect("articulation root")
                .rigid_body_handle();
            if !initialized.insert(root.into_raw_parts()) {
                continue;
            }
            multibody.forward_kinematics(&world.bodies, true);
            for link in multibody.links() {
                let velocity =
                    multibody.body_jacobian(link.link_id()) * multibody.generalized_velocity();
                updates.push((link.rigid_body_handle(), velocity));
            }
        }
        // Rapier's body velocity cache precedes the last multibody solver update.
        // Publish the current generalized velocity through its maintained Jacobian.
        for (handle, velocity) in updates {
            world.bodies[handle]
                .set_linvel(Vector::new(velocity[0], velocity[1], velocity[2]), false);
            world.bodies[handle]
                .set_angvel(Vector::new(velocity[3], velocity[4], velocity[5]), false);
        }
    }
    pub(crate) fn initialize_velocities(&self, world: &mut PhysicsWorld) {
        if world.multibody_joints.iter().next().is_none() {
            return;
        }
        // Construction and tree splitting create free roots with zero generalized
        // velocity; the admitted body velocities remain authoritative here.
        let mut initialized = BTreeSet::new();
        for joint in self.records.values() {
            let Handle::Reduced(handle) = joint.handle else {
                continue;
            };
            let (multibody, _) = world
                .multibody_joints
                .get_mut(handle)
                .expect("registered articulation");
            let root = multibody
                .link(0)
                .expect("articulation root")
                .rigid_body_handle();
            if !initialized.insert(root.into_raw_parts()) {
                continue;
            }
            multibody.forward_kinematics(&world.bodies, true);
            // Rapier adds 0.1 angular damping to every multibody DoF by default.
            // These constructors specify no joint damping or root drag.
            multibody.damping_mut().fill(0.0);
            let mut velocities = vec![0.0; multibody.generalized_velocity().len()];
            for link in multibody.links() {
                let body = &world.bodies[link.rigid_body_handle()];
                let offset = link.assembly_id();
                if let Some(parent) = link.parent_id() {
                    let parent = &world.bodies[multibody
                        .link(parent)
                        .expect("link parent")
                        .rigid_body_handle()];
                    let axis =
                        *parent.rotation() * link.joint.data.local_frame1.rotation * Vector::X;
                    velocities[offset] = (body.angvel() - parent.angvel()).dot(axis);
                } else if link.joint.ndofs() == 6 {
                    velocities[offset..offset + 3].copy_from_slice(&body.linvel().to_array());
                    velocities[offset + 3..offset + 6].copy_from_slice(&body.angvel().to_array());
                }
            }
            multibody
                .generalized_velocity_mut()
                .copy_from_slice(&velocities);
        }
    }
}

impl RapierBackend {
    pub(super) fn preflight_velocities(
        &self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: Vector,
        axis: Vector,
    ) -> Result<(), PhysicsError> {
        let twist = |id: BodyId, initial: bool| {
            let (handle, spec, rotation, ..) = self.bodies[&id];
            let (linear, angular, center) = if initial {
                (
                    Vector::from(spec.velocity),
                    Vector::from(rotation.angular_velocity),
                    Vector::from(spec.position),
                )
            } else {
                let body = &self.world.bodies[handle];
                (body.linvel(), body.angvel(), body.center_of_mass())
            };
            (linear + angular.cross(anchor - center), angular)
        };
        // Reset uses spawn values even if a free body was edited before construction.
        for initial in [false, true] {
            let (child_velocity, child_angular) = twist(child, initial);
            let (parent_velocity, parent_angular) =
                parent.map_or((Vector::ZERO, Vector::ZERO), |id| twist(id, initial));
            let angular = child_angular - parent_angular;
            let residual = angular - axis * angular.dot(axis);
            let scale = 1.0_f64
                .max(child_velocity.length())
                .max(parent_velocity.length())
                .max(angular.length());
            if !scale.is_finite()
                || !child_velocity.is_finite()
                || !parent_velocity.is_finite()
                || (child_velocity - parent_velocity).length() > 1e-10 * scale
                || residual.length() > 1e-10 * scale
            {
                return Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "incompatible articulation construction velocities",
                });
            }
        }
        Ok(())
    }

    pub(super) fn preflight_solver(
        &self,
        parent: Option<BodyId>,
        child: BodyId,
        solver: Solver,
    ) -> Result<(), PhysicsError> {
        let reduced = matches!(solver, Solver::Reduced);
        for id in [parent, Some(child)].into_iter().flatten() {
            let (handle, _, _, inertia, _) = self.bodies[&id];
            let mixed = if reduced {
                self.world.impulse_joints_with(handle).next().is_some()
            } else {
                self.world.multibody_joints_with(handle).next().is_some()
            };
            if mixed {
                return Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "mixed impulse and reduced-coordinate constraints",
                });
            }
            if reduced
                && inertia.is_some_and(|inertia| inertia.center.into_iter().any(|v| v != 0.0))
            {
                return Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "offset articulation centers of mass",
                });
            }
        }
        if reduced
            && let Some(child_link) = self
                .world
                .multibody_joints
                .rigid_body_link(self.bodies[&child].0)
        {
            let same_tree = parent
                .and_then(|id| {
                    self.world
                        .multibody_joints
                        .rigid_body_link(self.bodies[&id].0)
                })
                .is_some_and(|parent_link| parent_link.multibody == child_link.multibody);
            // Reject before insert: Rapier can allocate roots before its own rejection.
            if child_link.id != 0 || same_tree {
                return Err(PhysicsError::InvalidJoint);
            }
        }
        Ok(())
    }
}
