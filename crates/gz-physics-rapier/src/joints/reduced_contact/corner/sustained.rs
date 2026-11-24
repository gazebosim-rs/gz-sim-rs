// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validated motion of a sliding finite sole already bearing on one deck edge.
use super::*;
use gz_physics::{DeckDirection, PlanarEdgeArc, PlanarEdgeContact};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SustainedEdgeReceipt {
    pub motion: gz_physics::EdgeMotion,
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub normal_impulse_on_foot: [f64; 3],
    pub tangent_impulse_on_foot: [f64; 3],
    pub compressive_impulse: f64,
    pub dissipated_energy: f64,
    pub start_x: f64,
    pub end_x: f64,
    pub quadrature_intervals: u32,
}
#[derive(Clone, Copy)]
pub(super) struct RigidTreeMotion {
    pub(super) owner: MultibodyIndex,
    pub(super) root: RigidBodyHandle,
    pub(super) root_pose: Pose,
    pub(super) center: Vector,
    pub(super) center_velocity: Vector,
    pub(super) angular: Vector,
}
impl RigidTreeMotion {
    pub(super) fn commit(self, backend: &mut RapierBackend) {
        self.commit_links(backend);
        backend.world.bodies.propagate_modified_body_positions_to_colliders(&mut backend.world.colliders);
    }
    /// Several disjoint trees can publish their collider poses together once
    /// their independent generalized coordinates and link twists are committed.
    pub(super) fn commit_links(self, backend:&mut RapierBackend) {
        let w = &mut backend.world;
        let tree = w
            .multibody_joints
            .get_multibody_mut(self.owner)
            .expect("certified tree");
        w.bodies[self.root].set_position(self.root_pose, false);
        tree.forward_kinematics(&w.bodies, true);
        for l in tree.links() {
            let rb = &mut w.bodies[l.rigid_body_handle()];
            rb.set_position(*l.local_to_world(), false);
            rb.set_linvel(
                self.center_velocity + self.angular.cross(rb.center_of_mass() - self.center),
                false,
            );
            rb.set_angvel(self.angular, false);
        }
        let mut v = tree.generalized_velocity_mut();
        v.rows_mut(0, 3)
            .copy_from_slice(&w.bodies[self.root].linvel().to_array());
        v.rows_mut(3, 3).copy_from_slice(&self.angular.to_array());
    }
}
pub(crate) struct PreparedEdge {
    motion: RigidTreeMotion,
    pub colliders: (ColliderHandle, ColliderHandle),
    pub receipt: SustainedEdgeReceipt,
}
impl PreparedEdge {
    pub(crate) fn commit(self, backend: &mut RapierBackend) -> SustainedEdgeReceipt {
        self.motion.commit(backend);
        self.receipt
    }
}
impl RapierBackend {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn swept_locked_neighborhood(
        &self,
        tree: &Multibody,
        index: MultibodyIndex,
        boxes: RegisteredBoxes,
        com: Vector,
        gravity: Vector,
        geom: f64,
        center_speed: f64,
        angular_speed: f64,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        self.swept_locked_supports(tree,index,boxes,None,com,gravity,geom,center_speed,angular_speed,dt,None)
    }
    #[allow(clippy::too_many_arguments)]
    pub(super) fn swept_locked_supports(
        &self,
        tree: &Multibody,
        index: MultibodyIndex,
        boxes: RegisteredBoxes,
        other:Option<RegisteredBoxes>,
        com: Vector,
        gravity: Vector,
        geom: f64,
        center_speed: f64,
        angular_speed: f64,
        dt: Duration,
        batch:Option<&crate::approach_clearance::SharedNeighborhood>,
    ) -> Result<bool, PhysicsError> {
        let w = &self.world;
        let deck = &w.bodies[boxes.pad];
        let allowed=[boxes.colliders,other.map_or(boxes.colliders,|b|b.colliders)];
        if let Err(e) = self.check_contact_tree_pairs(tree,&allowed[..1+usize::from(other.is_some())],true,true){
            return match e {
                PhysicsError::Unsupported { .. } => Ok(false),
                _ => Err(e),
            };
        }
        let mut sweeps = Vec::new();
        for l in tree.links() {
            let bh = l.rigid_body_handle();
            let b = &w.bodies[bh];
            if !b.is_dynamic()
                || b.gravity_scale() * w.gravity != gravity
                || b.linear_damping() != 0.
                || b.angular_damping() != 0.
                || !b.locked_axes().is_empty()
                || b.user_force() != Vector::ZERO
                || b.user_torque() != Vector::ZERO
                || w.impulse_joints_with(bh).next().is_some()
                || !b.is_enabled()
                || l.joint.kinematic
            {
                return Ok(false);
            }
            for ch in b.colliders() {
                let c = &w.colliders[*ch];
                let pose =
                    *b.position() * c.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
                let bound = c.shape().compute_local_bounding_sphere();
                let radius = (pose * bound.center - com).length() + bound.radius;
                let margin = w.integration_parameters.prediction_distance()
                    + geom
                    + (center_speed + angular_speed * radius) * dt.as_secs_f64();
                for target in std::iter::once(boxes).chain(other) {
                if *ch != target.colliders.0
                    && crate::collision::can_exchange_impulses(
                        c,
                        &w.colliders[target.colliders.1],
                        w.bodies[target.pad].body_type(),
                    )
                    && !query::distance(
                        &pose,
                        c.shape(),
                        &target.pad_pose,
                        w.colliders[target.colliders.1].shape(),
                    )
                    .is_ok_and(|q| q.distance > margin + (w.bodies[target.pad].linvel()-deck.linvel()).length()*dt.as_secs_f64() + c.contact_skin())
                {
                    return Ok(false);
                }

                }
                sweeps.push((
                    c,
                    crate::approach_clearance::ApproachSweep {
                        pose,
                        velocity: deck.linvel(),
                        dt,
                        margin,
                    },
                ));
            }
        }
        if !self.approach_tree_neighbors_in(&sweeps, |h| {
            h == boxes.pad || other.is_some_and(|b|h==b.pad)
                || w.multibody_joints
                    .rigid_body_link(h)
                    .is_some_and(|l| l.multibody == index)
        },batch)? {
            return Ok(false);
        }
        Ok(true)
    }
    pub(crate) fn prepare_sustained_edge(
        &self,
        foot: BodyId,
        pad: BodyId,
        start: Duration,
        dt: Duration,
    ) -> Result<Option<PreparedEdge>, PhysicsError> {
        if !self.held_wrenches.is_empty()
            || !self.joints.efforts.is_empty()
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || self.curves.authored()
            || self.conserved_rotor.is_some()
        {
            return Ok(None);
        }
        let Some(boxes) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        let w = &self.world;
        let shoe = &w.bodies[boxes.foot];
        let deck = &w.bodies[boxes.pad];
        if deck.angvel() != Vector::ZERO {
            return Ok(None);
        }
        let index = w
            .multibody_joints
            .rigid_body_link(boxes.foot)
            .unwrap()
            .multibody;
        let tree = w
            .multibody_joints
            .get_multibody(index)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.ndofs() != 6
            || tree.armature().iter().any(|x| *x != 0.)
            || tree.damping().iter().any(|x| *x != 0.)
            || !tree.couplings().is_empty()
        {
            return Ok(None);
        }
        let f = boxes.pad_pose;
        let [x, axis, z] = [Vector::X, Vector::Y, Vector::Z].map(|u| f.rotation * u);
        let Some((m, com, inertia)) = self.planar_mass_properties(tree, shoe, axis) else {
            return Ok(None);
        };
        let tol = 256. * f64::EPSILON;
        let bx = boxes.sole_pose.rotation * Vector::X;
        let bz = boxes.sole_pose.rotation * Vector::Z;
        if (boxes.sole_pose.rotation * Vector::Y - axis).length() > tol {
            return Ok(None);
        }
        let gravity = w.gravity * shoe.gravity_scale();
        if axis.dot(gravity).abs() > tol * gravity.length() {
            return Ok(None);
        }
        let th = bz.dot(x).atan2(bz.dot(z));
        let Some((vs, om)) = self.generalized_link_twist(boxes.foot) else {
            return Ok(None);
        };
        let pitch = om.dot(axis);
        if (pitch == 0. && th == 0.) || om.cross(axis).length() > tol * (1. + pitch.abs()) {
            return Ok(None);
        }
        let center_velocity = vs + om.cross(com - shoe.center_of_mass());
        let relative = center_velocity - deck.linvel();
        let u = bx.dot(relative);
        let (side, edge_x) = if (th == 0. && pitch < 0.) || th < 0. {
            (DeckDirection::Positive, -boxes.pad_half.x)
        } else {
            (DeckDirection::Negative, boxes.pad_half.x)
        };
        let cp = f.inverse_transform_point(com);
        let fs = f.inverse_transform_point(boxes.sole_pose.translation);
        let transverse = [
            (-boxes.pad_half.y).max(fs.y - boxes.sole_half.y),
            boxes.pad_half.y.min(fs.y + boxes.sole_half.y),
        ];
        let pt = f * Vector::new(edge_x, cp.y, boxes.pad_half.z);
        let h = boxes.sole_half.z + bz.dot(com - boxes.sole_pose.translation);
        let extent = com.abs().max_element()
            + pt.abs().max_element()
            + boxes.sole_half.max_element()
            + boxes.pad_half.max_element()
            + 1.;
        let geom = tol * extent;
        let s = bx.dot(pt - com);
        let vel_tol = tol * (1. + relative.length() + om.length() * (1. + (pt - com).length()));
        if transverse[1] - transverse[0] < geom
            || (0.5 * (transverse[1] + transverse[0]) - cp.y).abs() > geom
            || axis.dot(relative).abs() > vel_tol
            || (bz.dot(com - pt) - h).abs() > geom
            || (bz.dot(relative) - s * pitch).abs() > vel_tol
        {
            return Ok(None);
        }
        let offset = bx.dot(com - boxes.sole_pose.translation);
        let arc = PlanarEdgeArc {
            contact: PlanarEdgeContact {
                mass: m,
                pitch_inertia: inertia,
                height: h,
                contact_x: s,
                tangent_center_speed: u,
                pitch_rate: pitch,
                friction: boxes.friction,
                acceleration: [bx.dot(gravity), bz.dot(gravity)],
                external_torque: 0.,
            },
            angle: th,
            sole_bounds: [-boxes.sole_half.x - offset, boxes.sole_half.x - offset],
            deck: side,
        };
        let Some(step) = arc.advance(dt)? else {
            return Ok(None);
        };
        // Bounds in the inertial support frame apply to all material points.
        // Unrelated native dynamics can only be skipped when they cannot reach
        // this tree. The other links of the tree must also stay off the deck.
        let energy = 0.5 * (m * relative.length_squared() + inertia * pitch * pitch);
        let speed_norm = (2. * energy).sqrt() + m.sqrt() * gravity.length() * dt.as_secs_f64();
        let center_speed = speed_norm / m.sqrt();
        let angular_speed = speed_norm / inertia.sqrt();
        if !self.swept_locked_neighborhood(
            tree,
            index,
            boxes,
            com,
            gravity,
            geom,
            center_speed,
            angular_speed,
            dt,
        )? {
            return Ok(None);
        }
        let dir = |v: [f64; 2]| x * v[0] + z * v[1];
        let next_center = pt + deck.linvel() * dt.as_secs_f64() + dir(step.relative_center);
        let next_velocity = deck.linvel() + dir(step.relative_velocity);
        let omega = axis * step.next.contact.pitch_rate;
        let dq = Rotation::from_scaled_axis(axis * (step.next.angle - th));
        let root = tree.root().rigid_body_handle();
        let old = *w.bodies[root].position();
        let pose = Pose::from_parts(
            next_center + dq * (old.translation - com),
            (dq * old.rotation).normalize(),
        );
        Ok(Some(PreparedEdge {
            motion: RigidTreeMotion {
                owner: index,
                root,
                root_pose: pose,
                center: next_center,
                center_velocity: next_velocity,
                angular: omega,
            },
            colliders: boxes.colliders,
            receipt: SustainedEdgeReceipt {
                motion: step.motion,
                foot,
                support: pad,
                start,
                duration: dt,
                normal_impulse_on_foot: dir(step.normal_impulse).to_array(),
                tangent_impulse_on_foot: dir(step.tangent_impulse).to_array(),
                compressive_impulse: step.compressive_impulse,
                dissipated_energy: step.lost_energy,
                start_x: s,
                end_x: step.next.contact.contact_x,
                quadrature_intervals: step.subintervals,
            },
        }))
    }
}
