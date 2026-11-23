// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl Joints {
    pub(crate) fn finish_velocities(&self, world: &mut PhysicsWorld, integrated: bool) {
        if world.multibody_joints.iter().next().is_none() {
            return;
        }
        // Link one has exactly one registered joint per articulation. Update
        // the complete tree when visiting that joint, without a deduplication
        // set or temporary per-link result buffer in the stepping hot path.
        let mut poses_changed = false;
        for joint in self.records.values() {
            let Handle::Reduced(handle) = joint.handle else {
                continue;
            };
            let (tree, link_id) = world
                .multibody_joints
                .get_mut(handle)
                .expect("registered articulation");
            if link_id != 1 {
                continue;
            }
            // Rapier integrates the free root by quaternion multiplication. A
            // revolute child derives its angle from a scalar, so only the root
            // accumulates norm drift. Correct ordinary time integration before
            // the 1e-12 admission bound; checkpoint reads must preserve bits.
            let root = tree.root();
            let handle = root.rigid_body_handle();
            let rotation = *world.bodies[handle].rotation();
            let correct = integrated
                && root.joint.ndofs() == 6
                && (rotation.length_squared() - 1.).abs() > 5e-13
                && rotation.is_normalized();
            if correct {
                world.bodies[handle].set_rotation(rotation.normalize(), false);
                poses_changed = true;
            }
            tree.forward_kinematics(&world.bodies, true);
            for link in tree.links() {
                let velocity = tree.body_jacobian(link.link_id()) * tree.generalized_velocity();
                let body = &mut world.bodies[link.rigid_body_handle()];
                if correct {
                    // Keep every link, its inertia and the published Jacobian
                    // in the same frame as the corrected generalized state.
                    body.set_position(*link.local_to_world(), false);
                }
                body.set_linvel(Vector::new(velocity[0], velocity[1], velocity[2]), false);
                body.set_angvel(Vector::new(velocity[3], velocity[4], velocity[5]), false);
            }
        }
        if poses_changed {
            world
                .bodies
                .propagate_modified_body_positions_to_colliders(&mut world.colliders);
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
            let velocities = initial_generalized_velocity(multibody, &world.bodies);
            multibody.generalized_velocity_mut().copy_from(&velocities);
        }
    }
}

/// Body twists are authoritative until the first native interval initializes
/// reduced velocities. A cold point-impulse read must see that same state.
pub(super) fn initial_generalized_velocity(
    multibody: &Multibody,
    bodies: &RigidBodySet,
) -> rapier3d_f64::na::DVector<f64> {
    let mut velocities = rapier3d_f64::na::DVector::<f64>::zeros(multibody.ndofs());
    for link in multibody.links() {
        let body = &bodies[link.rigid_body_handle()];
        let offset = link.assembly_id();
        if let Some(parent) = link.parent_id() {
            let parent = &bodies[multibody
                .link(parent)
                .expect("link parent")
                .rigid_body_handle()];
            let axis = *parent.rotation() * link.joint.data.local_frame1.rotation * Vector::X;
            if link.joint.data.user_data != EXACT_REVOLUTE_LOCK {
                velocities[offset] = (body.angvel() - parent.angvel()).dot(axis);
            }
        } else if link.joint.ndofs() == 6 {
            velocities.as_mut_slice()[offset..offset + 3]
                .copy_from_slice(&body.linvel().to_array());
            velocities.as_mut_slice()[offset + 3..offset + 6]
                .copy_from_slice(&body.angvel().to_array());
        }
    }
    velocities
}

impl RapierBackend {
    pub(super) fn preflight_velocities(
        &self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: Vector,
        free_axis: Option<Vector>,
    ) -> Result<(), PhysicsError> {
        let twist = |id: BodyId, initial: bool| {
            let (handle, spec, rotation, ..) = &self.bodies[&id];
            let (linear, angular, center) = if initial {
                (
                    Vector::from(spec.velocity),
                    Vector::from(rotation.angular_velocity),
                    Vector::from(spec.position),
                )
            } else {
                let body = &self.world.bodies[*handle];
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
            let residual = free_axis.map_or(angular, |axis| angular - axis * angular.dot(axis));
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
            let (handle, spec, _, inertia, _) = &self.bodies[&id];
            let mixed = if reduced {
                self.world.impulse_joints_with(*handle).next().is_some()
            } else {
                self.world.multibody_joints_with(*handle).next().is_some()
            };
            if mixed {
                return Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "mixed impulse and reduced-coordinate constraints",
                });
            }
            if reduced
                && spec.articulation.is_none()
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
