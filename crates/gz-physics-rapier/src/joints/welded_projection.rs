// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A floating tree of fixed links has exactly one rigid velocity. Solve it with
//! Rapier's ordinary rigid contact rows, preserving the authored collider slots
//! and the complete native narrow-phase history across ordinary public intervals.
//!
//! The carrier owns one floating assembly and any independent dynamic bodies.
//! Prescribed motion, selected constraint rows and held-support solves use the
//! multibody solver. Switching representations cold-starts the destination
//! contact solver, since its cached rows use different responding body frames.
//! Enter only when no geometric manifold is live in the multibody world.
use super::*;

pub(crate) struct WeldedProjection {
    solver: PhysicsWorld,
    root: RigidBodyHandle,
    tree: MultibodyIndex,
    members: Vec<RigidBodyHandle>,
    peers: Vec<RigidBodyHandle>,
    published_pose: Pose,
    published_twist: [f64; 6],
}

impl WeldedProjection {
    fn build(b: &RapierBackend) -> Option<Box<Self>> {
        let first = b
            .joints
            .records
            .values()
            .find(|j| j.initial.authored_weld())?;
        let child = b.bodies.get(&first.initial.child)?.0;
        let w = &b.world;
        let tree = w.multibody_joints.rigid_body_link(child)?.multibody;
        let assembly = w.multibody_joints.get_multibody(tree)?;
        let root = assembly.root().rigid_body_handle();
        if assembly.ndofs() != 6
            || assembly.root().joint.kinematic
            || !w.bodies[root].is_dynamic()
            || b.joints.records.len() + 1 != assembly.num_links()
            || !b.joints.records.values().all(|j| j.initial.authored_weld())
        {
            return None;
        }
        let members: Vec<_> = assembly
            .links()
            .map(|link| link.rigid_body_handle())
            .collect();
        // The projection and each independent free body share a single native
        // contact solve, so both sides receive the same interaction impulse.
        // Prescribed peers can be advanced by separate trajectory owners.
        if w.bodies.iter().any(|(_, body)| body.is_kinematic()) {
            return None;
        }
        let peers = w
            .bodies
            .iter()
            .filter_map(|(h, body)| (body.is_dynamic() && !members.contains(&h)).then_some(h))
            .collect();
        let agg = b.welded_assembly_state(first.initial.child).ok()?;
        let pose = *w.bodies[root].position();
        let rot = Matrix::from_quat(pose.rotation);
        let tensor = rot.transpose() * Matrix::from_cols_array_2d(&agg.inertia_columns) * rot;
        let props = crate::inertia_frame::properties(
            pose.inverse() * Vector::from(agg.center),
            agg.mass,
            Rotation::IDENTITY,
            tensor,
        )
        .ok()?;
        let mut solver = PhysicsWorld {
            gravity: w.gravity,
            integration_parameters: w.integration_parameters,
            bodies: w.bodies.clone(),
            colliders: w.colliders.clone(),
            broad_phase: w.broad_phase.clone(),
            narrow_phase: w.narrow_phase.clone(),
            islands: w.islands.clone(),
            ..PhysicsWorld::new()
        };
        for &member in &members {
            let link = &w.bodies[member];
            for &ch in link.colliders() {
                // Rapier already suppresses colliders sharing a rigid parent.
                // An otherwise redundant filter forces a full manifold rebuild
                // every step, losing the rigid solver's ordinary frozen anchors.
                let group = w.colliders[ch].user_data;
                if group != 0
                    && !w.colliders.iter().any(|(_, c)| {
                        c.user_data == group && !c.parent().is_some_and(|h| members.contains(&h))
                    })
                {
                    let hooks =
                        solver.colliders[ch].active_hooks() - ActiveHooks::FILTER_CONTACT_PAIRS;
                    solver.colliders[ch].set_active_hooks(hooks);
                }
                let local =
                    pose.inverse() * (*link.position() * *w.colliders[ch].position_wrt_parent()?);
                if member != root {
                    solver
                        .colliders
                        .set_parent(ch, Some(root), &mut solver.bodies);
                    solver.colliders[ch].set_position_wrt_parent(local);
                }
                // A massless shape with offset needs an identity mass in the
                // parent frame to avoid rediagonalizing the aggregate inertia.
                let inv = solver.colliders[ch].position_wrt_parent()?.inverse();
                solver.colliders[ch].set_mass_properties(MassProperties {
                    local_com: inv.translation,
                    principal_inertia_local_frame: inv.rotation,
                    ..Default::default()
                });
            }
            if member != root {
                solver.bodies[member].set_enabled(false);
            }
        }
        let rb = &mut solver.bodies[root];
        rb.set_additional_mass_properties(props, true);
        rb.recompute_mass_properties_from_colliders(&solver.colliders);
        rb.set_linvel(Vector::from(agg.velocity), true);
        rb.set_angvel(Vector::from(agg.spin), true);
        Some(Box::new(Self {
            solver,
            root,
            tree,
            members,
            peers,
            published_pose: pose,
            published_twist: assembly.generalized_velocity().as_slice().try_into().ok()?,
        }))
    }

    pub(crate) fn duplicate(&self) -> Box<Self> {
        Box::new(Self {
            solver: RapierBackend::copy_native_world(&self.solver),
            root: self.root,
            tree: self.tree,
            members: self.members.clone(),
            peers: self.peers.clone(),
            published_pose: self.published_pose,
            published_twist: self.published_twist,
        })
    }

    pub(crate) fn quarantined(&self) -> bool {
        !self.solver.quarantine().is_empty()
    }

    #[cfg(feature = "landing-audit")]
    pub(crate) fn normal_mobility(&self, origin: Vector, arms: [[f64; 3]; 4]) -> [[f64; 4]; 4] {
        let rb = &self.solver.bodies[self.root];
        let torques =
            arms.map(|r| (Vector::from(r) + (origin - rb.center_of_mass())).cross(Vector::Z));
        let m = rb.mass_properties();
        std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                m.effective_inv_mass.z + torques[i].dot(m.effective_world_inv_inertia * torques[j])
            })
        })
    }

    fn compatible(&self, b: &RapierBackend) -> bool {
        let w = &b.world;
        w.bodies.len() == self.solver.bodies.len()
            && w.colliders.len() == self.solver.colliders.len()
            && w.multibody_joints
                .get_multibody(self.tree)
                .is_some_and(|t| {
                    t.root().rigid_body_handle() == self.root
                        && t.ndofs() == 6
                        && !t.root().joint.kinematic
                        && t.num_links() == self.members.len()
                })
            && w.bodies[self.root].is_dynamic()
    }

    fn exchange_queries(&mut self, w: &mut PhysicsWorld) {
        std::mem::swap(&mut w.broad_phase, &mut self.solver.broad_phase);
        std::mem::swap(&mut w.narrow_phase, &mut self.solver.narrow_phase);
    }

    fn step(&mut self, b: &mut RapierBackend) {
        let w = &mut b.world;
        let rigid = &mut self.solver.bodies[self.root];
        let root = &w.bodies[self.root];
        let pose = *root.position();
        let tree = w
            .multibody_joints
            .get_multibody(self.tree)
            .expect("active welded tree");
        let qdot = tree.generalized_velocity();
        // Native reduced-coordinate transactions may publish a new pose or twist
        // between ordinary steps. An untouched root retains its full warm state.
        if pose != self.published_pose || qdot.as_slice() != self.published_twist {
            rigid.set_position(pose, true);
            let v = Vector::new(qdot[0], qdot[1], qdot[2]);
            let spin = Vector::new(qdot[3], qdot[4], qdot[5]);
            rigid.set_linvel(
                v + spin.cross(rigid.center_of_mass() - root.center_of_mass()),
                true,
            );
            rigid.set_angvel(spin, true);
        }
        let center = rigid.center_of_mass();
        let mut force = Vector::ZERO;
        let mut torque = Vector::ZERO;
        for &h in &self.members {
            let link = &w.bodies[h];
            let f = link.user_force();
            force += f;
            torque += link.user_torque() + (link.center_of_mass() - center).cross(f);
        }
        rigid.reset_forces(false);
        rigid.reset_torques(false);
        if force != Vector::ZERO {
            rigid.add_force(force, true);
        }
        if torque != Vector::ZERO {
            rigid.add_torque(torque, true);
        }
        for &h in &self.peers {
            let public = &w.bodies[h];
            let peer = &mut self.solver.bodies[h];
            // Public commands can change a free peer's twist or accumulated
            // wrench while paused. Avoid invalidating an untouched warm body.
            if peer.activation() != public.activation() {
                *peer.activation_mut() = *public.activation();
            }
            if peer.is_ccd_enabled() != public.is_ccd_enabled() {
                peer.enable_ccd(public.is_ccd_enabled());
            }
            if peer.linvel() != public.linvel() {
                peer.set_linvel(public.linvel(), true);
            }
            if peer.angvel() != public.angvel() {
                peer.set_angvel(public.angvel(), true);
            }
            if peer.user_force() != public.user_force() {
                peer.reset_forces(false);
                peer.add_force(public.user_force(), true);
            }
            if peer.user_torque() != public.user_torque() {
                peer.reset_torques(false);
                peer.add_torque(public.user_torque(), true);
            }
        }
        self.solver.integration_parameters = w.integration_parameters;
        self.solver.gravity = w.gravity;
        self.solver.step_with_events(&crate::collision::Groups, &());
        let rigid = &self.solver.bodies[self.root];
        let root = &mut w.bodies[self.root];
        root.set_position(*rigid.position(), false);
        let spin = rigid.angvel();
        let velocity = rigid.linvel() + spin.cross(root.center_of_mass() - rigid.center_of_mass());
        let tree = w
            .multibody_joints
            .get_multibody_mut(self.tree)
            .expect("active welded tree");
        tree.forward_kinematics(&w.bodies, true);
        tree.update_rigid_bodies(&mut w.bodies, true);
        let mut qdot = tree.generalized_velocity_mut();
        qdot.as_mut_slice()[..3].copy_from_slice(&velocity.to_array());
        qdot.as_mut_slice()[3..6].copy_from_slice(&spin.to_array());
        self.published_pose = *w.bodies[self.root].position();
        self.published_twist.copy_from_slice(qdot.as_slice());
        for &h in &self.members {
            let rb = &mut w.bodies[h];
            rb.set_linvel(
                rigid.linvel() + spin.cross(rb.center_of_mass() - rigid.center_of_mass()),
                false,
            );
            rb.set_angvel(spin, false);
        }
        for &h in &self.peers {
            let peer = &self.solver.bodies[h];
            let public = &mut w.bodies[h];
            public.set_position(*peer.position(), false);
            public.set_linvel(peer.linvel(), false);
            public.set_angvel(peer.angvel(), false);
            *public.activation_mut() = *peer.activation();
        }
        w.bodies
            .propagate_modified_body_positions_to_colliders(&mut w.colliders);
    }
}

impl RapierBackend {
    pub(crate) fn release_welded_projection(&mut self) {
        self.welded_exclusion = None;
        if let Some(mut previous) = self.welded_projection.take() {
            previous.exchange_queries(&mut self.world);
        }
    }
    pub(crate) fn step_welded_projection(&mut self, eligible: bool) -> bool {
        let mut projection = self.welded_projection.take();
        if let Some(p) = &mut projection {
            p.exchange_queries(&mut self.world);
        }
        if !eligible {
            return false;
        }
        if projection.as_ref().is_some_and(|p| !p.compatible(self)) {
            projection = None;
        }
        let mut p = if let Some(p) = projection {
            p
        } else {
            let topology = (
                self.world.bodies.len(),
                self.world.colliders.len(),
                self.joints.records.len(),
            );
            if self.welded_exclusion == Some(topology) {
                return false;
            }
            if self
                .world
                .narrow_phase
                .contact_pairs()
                .any(|p| !p.manifolds().is_empty())
            {
                // Keep the existing contact owner until the next explicit
                // scene/state edit; it can have warm rows on several links.
                self.welded_exclusion = Some(topology);
                return false;
            }
            let Some(p) = WeldedProjection::build(self) else {
                self.welded_exclusion = Some(topology);
                return false;
            };
            self.welded_exclusion = None;
            p
        };
        p.step(self);
        // The observed broad/narrow phases always belong to the actual solve.
        // Their collider handles resolve through the original public link map.
        p.exchange_queries(&mut self.world);
        self.welded_projection = Some(p);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rig() -> (RapierBackend, BodyId, BodyId) {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let mut link = |z| {
            b.spawn_articulation_link(
                BodySpec {
                    position: [0., 0., z],
                    velocity: [0.; 3],
                    mass: Some(1.),
                    shape: Shape::Sphere { radius: 0.1 },
                },
                RotationSpec::default(),
                None,
            )
            .unwrap()
        };
        let root = link(0.);
        let child = link(0.4);
        b.add_fixed_joint(Some(root), child, [0., 0., 0.2]).unwrap();
        (b, root, child)
    }

    #[test]
    fn projected_tree_resumes_the_impulse_from_an_exclusive_contact_transaction() {
        let (mut b, root, child) = rig();
        let dt = Duration::from_micros(500);
        b.step(dt).unwrap();
        let mut interval = b.begin_articulated_material_interval(&[child]).unwrap();
        let mut branch = interval.branch().unwrap();
        branch
            .prepare(&[ArticulatedContactSite {
                body: child,
                local_point: [0.; 3],
            }])
            .unwrap()
            .commit(&[[0.1, 0., 0.]])
            .unwrap();
        branch.step(dt).unwrap();
        branch.commit();
        interval.commit();
        b.step(dt).unwrap();
        let state = b.welded_assembly_state(root).unwrap();
        assert!((Vector::from(state.linear_momentum) - 0.1 * Vector::X).length() < 1e-12);
        assert!((Vector::from(state.angular_momentum) - 0.02 * Vector::Y).length() < 1e-10);
        assert!(b.welded_projection.is_some());
    }

    #[test]
    fn free_peer_receives_public_edits_and_one_interval_of_wrench() {
        let (mut b, _, _) = rig();
        let peer = b
            .spawn(BodySpec {
                position: [0., 1., 0.],
                velocity: [0.; 3],
                mass: Some(2.),
                shape: Shape::Sphere { radius: 0.1 },
            })
            .unwrap();
        let dt = Duration::from_millis(1);
        b.step(dt).unwrap();
        b.enable_continuous_collision(peer).unwrap();
        assert!(b.welded_projection.is_some());
        b.set_world_velocity(peer, [0.5, 0., 0.], [0., 0., 0.1])
            .unwrap();
        b.add_world_wrench(peer, [4., 0., 0.], [0., 0., 0.008])
            .unwrap();
        b.step(dt).unwrap();
        let carrier = b.welded_projection.as_ref().unwrap();
        assert!(carrier.solver.bodies[b.bodies[&peer].0].is_ccd_enabled());
        let once = b.state(peer).unwrap();
        assert!((once.velocity[0] - 0.502).abs() < 1e-12);
        assert!((once.angular_velocity[2] - 0.101).abs() < 1e-12);
        b.step(dt).unwrap();
        assert_eq!(b.state(peer).unwrap().velocity, once.velocity);
        assert!((b.state(peer).unwrap().angular_velocity[2] - 0.101).abs() < 1e-12);
        let mut moved = b.state(peer).unwrap();
        moved.position = [2., 0., 1.];
        moved.velocity = [0.; 3];
        moved.angular_velocity = [0.; 3];
        b.set_body_state(peer, moved).unwrap();
        b.step(dt).unwrap();
        assert_eq!(b.state(peer).unwrap().position, moved.position);
        assert!(b.welded_projection.is_some());
    }

    #[test]
    fn prescribed_peer_keeps_its_original_motion_owner() {
        let (mut b, _, _) = rig();
        let peer = b
            .spawn_velocity_boundary(
                BodySpec {
                    position: [2., 0., 0.],
                    velocity: [0., 0.3, 0.],
                    mass: Some(1.),
                    shape: Shape::Box {
                        half_extents: [0.05; 3],
                    },
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        b.step(Duration::from_millis(1)).unwrap();
        assert!(b.welded_projection.is_none());
        assert!((b.state(peer).unwrap().position[1] - 0.0003).abs() < 1e-12);
    }

    #[test]
    fn a_shared_external_group_still_filters_the_fixed_support() {
        let (mut b, root, child) = rig();
        let spec = BodySpec {
            position: [0., 0., 0.598],
            velocity: [0.; 3],
            mass: None,
            shape: Shape::Sphere { radius: 0.1 },
        };
        let support = b.spawn(spec).unwrap();
        for id in [root, child, support] {
            b.set_self_collision_group(id, Some(0)).unwrap();
        }
        b.step(Duration::from_micros(500)).unwrap();
        assert!(b.welded_projection.is_some());
        assert_eq!(b.active_contact_pairs(), 0);
        assert_eq!(b.state(child).unwrap().velocity, [0.; 3]);
    }

    #[test]
    fn removing_a_dynamic_peer_keeps_the_rigid_solver_and_static_edits_refresh_contacts() {
        let (mut b, root, child) = rig();
        let mut spec = BodySpec {
            position: [0., 0., 4.],
            velocity: [0.; 3],
            mass: Some(1.),
            shape: Shape::Sphere { radius: 0.1 },
        };
        let peer = b.spawn(spec).unwrap();
        let dt = Duration::from_micros(500);
        b.step(dt).unwrap();
        assert!(b.welded_projection.is_some());
        b.apply_impulse(peer, [0.2, 0., 0.], [0.; 3]).unwrap();
        b.step(dt).unwrap();
        assert!((b.state(peer).unwrap().velocity[0] - 0.2).abs() < 1e-12);
        assert!(b.remove(peer));
        spec.mass = None;
        spec.position = [0., 0., -0.198];
        let support = b.spawn(spec).unwrap();
        b.step(dt).unwrap();
        assert!(b.welded_projection.is_some());
        assert_eq!(b.active_contact_pairs(), 1);
        assert!(b.set_self_collision_group(child, Some(14)).is_err());
        assert!(b.apply_impulse(root, [1., 0., 0.], [0.; 3]).is_err());
        assert!(b.set_world_velocity(child, [1., 0., 0.], [0.; 3]).is_err());
        assert!(b.welded_projection.is_some());
        let mut moved = b.state(support).unwrap();
        moved.position = [4., 0., 0.];
        b.set_body_state(support, moved).unwrap();
        b.step(dt).unwrap();
        assert_eq!(b.active_contact_pairs(), 0);
        assert!(b.remove(support));
        spec.position = [0., 0., 0.598];
        let replacement = b.spawn(spec).unwrap();
        b.step(dt).unwrap();
        let patches = b.rigid_contact_patches();
        assert!(
            patches
                .iter()
                .any(|p| (p.a == child && p.b == replacement)
                    || (p.a == replacement && p.b == child))
        );
    }
}
