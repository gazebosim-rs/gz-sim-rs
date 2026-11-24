// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A stationary zero-width stop can eliminate its own dynamic response while
//! retaining the revolute coordinate, public handle, and free six-DoF root.
use super::*;

impl RapierBackend {
    /// Select exact zero-width stops on one floating tree during construction.
    /// Existing finite ranges, continuous hinges and the root stay dynamic.
    pub fn enable_exact_revolute_locks(&mut self, root: BodyId) -> Result<(), PhysicsError> {
        let tree_id = self.articulation_index(root)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(tree_id)
            .expect("validated tree");
        let mut selected = BTreeMap::new();
        let mut found = false;
        for (&id, joint) in &self.joints.records {
            let Handle::Reduced(_) = joint.handle else {
                continue;
            };
            if joint.initial.limits != Some([0., 0.]) {
                continue;
            }
            let body = self.bodies[&joint.initial.child].0;
            let Some(member) = self.world.multibody_joints.rigid_body_link(body) else {
                continue;
            };
            if member.multibody != tree_id {
                continue;
            }
            found = true;
            if joint.initial.exact_lock {
                continue;
            }
            let link = tree.link(member.id).expect("registered link");
            let parent = tree
                .link(link.parent_id().ok_or(PhysicsError::InvalidJoint)?)
                .expect("registered parent");
            let a = &self.world.bodies[parent.rigid_body_handle()];
            let b = &self.world.bodies[body];
            let axis = *a.rotation() * joint.initial.frame1.rotation * Vector::X;
            if !self.joints.construction_open()
                || tree.root().joint.kinematic
                || link.joint.coords()[3] != 0.
                || tree.joint_velocity(link)[0] != 0.
                || (b.angvel() - a.angvel()).dot(axis) != 0.
                || !link.joint.data.motor_axes.is_empty()
            {
                return Err(PhysicsError::TransitionDeferred {
                    reason: "exact stops require stationary zero coordinates without motors before stepping",
                });
            }
            selected.insert(member.id, id);
        }
        if !found {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "exact lock requires an authored zero-width stop at zero",
            });
        }
        if selected.is_empty() {
            return Ok(());
        }
        // Changing DoF count rebuilds the target tree's Jacobian layout.
        // Keep the registered parent-first order and the existing data on
        // every other joint, including already configured sibling motors.
        let root_handle = tree.root().rigid_body_handle();
        let mut edges = Vec::with_capacity(tree.num_links() - 1);
        for link in tree.links().skip(1) {
            let parent = tree
                .link(link.parent_id().expect("nonroot link"))
                .expect("registered parent")
                .rigid_body_handle();
            let data = if let Some(id) = selected.get(&link.link_id()) {
                let mut config = self.joints.records[id].initial;
                config.exact_lock = true;
                config.data()
            } else {
                link.joint.data
            };
            edges.push((parent, link.rigid_body_handle(), data));
        }
        // Each existing edge has one child in this tree. Reinstalling its
        // unchanged endpoints in parent-first order cannot close a cycle.
        // Native joint handles are keyed by child body; public IDs remain stable.
        let joints = &mut self.world.multibody_joints;
        joints.remove_multibody_articulations(root_handle, false);
        for (parent, child, data) in edges {
            let restored = joints
                .insert(parent, child, data, true)
                .expect("preflighted tree edge");
            debug_assert_eq!(restored, MultibodyJointHandle(child.0));
        }
        for id in selected.into_values() {
            self.joints
                .records
                .get_mut(&id)
                .expect("registered joint")
                .initial
                .exact_lock = true;
        }
        Ok(())
    }
    pub fn is_exact_revolute_lock(&self, id: JointId) -> Result<bool, PhysicsError> {
        self.joints
            .records
            .get(&id)
            .map(|joint| joint.initial.exact_lock && !joint.initial.authored_weld())
            .ok_or(PhysicsError::MissingJoint)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn robot() -> (RapierBackend, [BodyId; 3], [JointId; 2]) {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let links = std::array::from_fn(|i| {
            b.spawn_articulation_link(
                BodySpec {
                    position: [i as f64, 0., 0.],
                    mass: Some(1.),
                    velocity: [0.; 3],
                    shape: Shape::Sphere { radius: 0.15 },
                },
                RotationSpec::default(),
                None,
            )
            .unwrap()
        });
        let stops = std::array::from_fn(|i| {
            b.add_limited_revolute(
                Some(links[i]),
                links[i + 1],
                [i as f64 + 0.5, 0., 0.],
                [0., 0., 1.],
                if i == 0 { [-0., 0.] } else { [-0.4, 0.4] },
            )
            .unwrap()
        });
        (b, links, stops)
    }
    #[test]
    fn exact_axis_survives_reset_and_tier_changes_while_its_sibling_accepts_drive() {
        let (mut b, links, joints) = robot();
        let dt = Duration::from_millis(1);
        b.enable_exact_revolute_locks(links[0]).unwrap();
        b.switch_articulation(links[0], ArticulationTier::Prescribed, dt)
            .unwrap();
        b.switch_articulation(links[0], ArticulationTier::Dynamic, dt)
            .unwrap();
        for _ in 0..2 {
            assert!(b.is_exact_revolute_lock(joints[0]).unwrap());
            assert!(!b.is_exact_revolute_lock(joints[1]).unwrap());
            let motor = Some(JointVelocityMotor {
                target_velocity: 1.,
                max_force: 0.8,
            });
            assert!(b.set_joint_velocity_motor(joints[0], motor).is_err());
            b.set_joint_velocity_motor(joints[1], motor).unwrap();
            for _ in 0..40 {
                b.add_joint_effort(joints[0], 0.3).unwrap();
                b.step(dt).unwrap();
                let locked = b.joint_state(joints[0]).unwrap();
                assert_eq!((locked.position, locked.velocity), (0., 0.));
            }
            let sibling = b.joint_state(joints[1]).unwrap();
            assert!(
                sibling.position > 0. && sibling.velocity > 0.,
                "finite sibling state {sibling:?}"
            );
            b.reset().unwrap();
        }
    }
    #[test]
    fn selecting_an_active_or_already_integrated_soft_stop_leaves_it_unmodified() {
        let (mut b, links, joints) = robot();
        b.set_joint_velocity_motor(
            joints[0],
            Some(JointVelocityMotor {
                target_velocity: 0.1,
                max_force: 0.3,
            }),
        )
        .unwrap();
        assert!(b.enable_exact_revolute_locks(links[0]).is_err());
        assert!(!b.is_exact_revolute_lock(joints[0]).unwrap());
        b.set_joint_velocity_motor(joints[0], None).unwrap();
        b.step(Duration::from_millis(1)).unwrap();
        assert!(b.enable_exact_revolute_locks(links[0]).is_err());
        assert!(!b.is_exact_revolute_lock(joints[0]).unwrap());
    }
}
