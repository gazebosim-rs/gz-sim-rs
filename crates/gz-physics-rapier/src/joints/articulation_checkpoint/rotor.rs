// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One spherical body on one passive world revolute. No external hard contact.
use super::*;

const MAGIC: &[u8; 8] = b"GZRTOI01";
#[derive(Serialize, Deserialize)]
struct Coordinate {
    construction: [u8; 32],
    last_ns: u64,
    q: f64,
    qdot: f64,
    body: LinkMotion,
}

impl RapierBackend {
    fn timed_rotor_layout(
        &self,
        last: Duration,
    ) -> Result<(BodyId, JointId, MultibodyJointHandle, [u8; 32]), PhysicsError> {
        if last.is_zero()
            || last > Duration::from_secs(1)
            || self.world.gravity != Vector::ZERO
            || self.bodies.len() != 1
            || self.joints.records.len() != 1
            || self.joints.world_roots.len() != 1
            || self.world.bodies.len() != 2
            || self.world.colliders.len() != 1
            || self.native_profile.is_some()
            || !self.joints.efforts.is_empty()
            || !self.pending_wrenches.is_empty()
            || !self.held_wrenches.is_empty()
            || self.world.narrow_phase.contact_pairs().next().is_some()
        {
            return Err(invalid());
        }
        let (&body, (h, initial, rotation, inertia, group)) =
            self.bodies.first_key_value().ok_or_else(invalid)?;
        let (&id, j) = self.joints.records.first_key_value().ok_or_else(invalid)?;
        let Handle::Reduced(handle) = j.handle else {
            return Err(invalid());
        };
        let c = j.initial;
        let radius = initial
            .collider
            .shape()
            .as_ball()
            .ok_or_else(invalid)?
            .radius;
        let inertia = inertia.as_ref().ok_or_else(invalid)?;
        let limits = c.limits.ok_or_else(invalid)?;
        let rb = &self.world.bodies[*h];
        let (tree, index) = self
            .world
            .multibody_joints
            .get(handle)
            .ok_or_else(invalid)?;
        let entry = tree.link(index).ok_or_else(invalid)?;
        if c.parent.is_some()
            || c.child != body
            || c.exact_lock
            || index != 1
            || !rb.is_dynamic()
            || rb.is_sleeping()
            || !rb.is_enabled()
            || rb.is_ccd_enabled()
            || rb.soft_ccd_prediction() != 0.
            || tree.num_links() != 2
            || entry.joint.kinematic
            || entry.joint.data != c.data()
            || !self.world.bodies[tree.root().rigid_body_handle()].is_fixed()
            || !tree.couplings().is_empty()
            || (!self.joints.open && (tree.ndofs() != 1 || tree.damping()[0] != 0.))
        {
            return Err(invalid());
        }
        let mut params = self.world.integration_parameters;
        params.dt = last.as_secs_f64();
        let mut hash = Sha256::new();
        hash.update(MAGIC);
        hash.update(
            codec()
                .serialize(&(
                    body.raw(),
                    id.raw(),
                    self.next_id,
                    self.joints.next,
                    *h,
                    handle,
                    params,
                    self.allow_isolation,
                ))
                .map_err(|_| invalid())?,
        );
        hash.update(
            codec()
                .serialize(&(
                    initial.position,
                    initial.velocity,
                    initial.mass,
                    initial.can_sleep,
                    initial.kinematic,
                    rotation.orientation,
                    rotation.angular_velocity,
                    group,
                    radius,
                    inertia.center,
                    inertia.orientation,
                    inertia.tensor,
                ))
                .map_err(|_| invalid())?,
        );
        hash.update(
            codec()
                .serialize(&(c.frame1, c.frame2, limits, &initial.collider))
                .map_err(|_| invalid())?,
        );
        Ok((body, id, handle, hash.finalize().into()))
    }
    pub fn checkpoint_timed_world_revolute(&self, last: Duration) -> Result<Vec<u8>, PhysicsError> {
        if self.joints.open
            || self.world.integration_parameters.dt.to_bits() != last.as_secs_f64().to_bits()
        {
            return Err(invalid());
        }
        let (body, id, handle, construction) = self.timed_rotor_layout(last)?;
        let (tree, idx) = self
            .world
            .multibody_joints
            .get(handle)
            .ok_or_else(invalid)?;
        let joint = &self.joints.records[&id];
        let s = Coordinate {
            construction,
            last_ns: last.as_nanos() as u64,
            q: tree.link(idx).ok_or_else(invalid)?.joint.coords()[3],
            qdot: tree.generalized_velocity()[0],
            body: LinkMotion::from_body(&self.world.bodies[self.bodies[&body].0]),
        };
        if joint.position.to_bits() != s.q.to_bits() {
            return Err(invalid());
        }
        s.body.pose()?;
        if ![s.q, s.qdot].into_iter().all(f64::is_finite) {
            return Err(invalid());
        }
        let bytes = codec().serialize(&s).map_err(|_| invalid())?;
        let mut result = MAGIC.to_vec();
        result.extend_from_slice(&Sha256::digest(&bytes));
        result.extend_from_slice(&bytes);
        Ok(result)
    }
    /// Reconstruct a fixed-world generalized coordinate and velocity into the identical native recipe.
    pub fn restore_timed_world_revolute(&mut self, image: &[u8]) -> Result<Duration, PhysicsError> {
        // Eight-byte duration and two generalized scalars plus a 13-component body witness.
        if image.len() != 40 + 32 + 8 + 2 * 8 + 13 * 8
            || &image[..8] != MAGIC
            || image[8..40] != Sha256::digest(&image[40..])[..]
        {
            return Err(invalid());
        }
        let s: Coordinate = codec().deserialize(&image[40..]).map_err(|_| invalid())?;
        let last = Duration::from_nanos(s.last_ns);
        if self.timed_rotor_layout(last)?.3 != s.construction
            || ![s.q, s.qdot].into_iter().all(f64::is_finite)
        {
            return Err(invalid());
        }
        s.body.pose()?;
        let mut staged = Self::new_multifidelity([0.; 3])?;
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        staged.apply_initial(self.prepare_initial(&self.initial_state())?)?;
        staged.step(last)?;
        let (_, id, handle, construction) = staged.timed_rotor_layout(last)?;
        if construction != s.construction {
            return Err(invalid());
        }
        let (tree, idx) = staged
            .world
            .multibody_joints
            .get_mut(handle)
            .ok_or_else(invalid)?;
        let link = tree.link_mut(idx).ok_or_else(invalid)?;
        let mut motion = MultibodyJoint::new(link.joint.data, false);
        motion.apply_displacement(&[s.q]);
        link.joint = motion;
        tree.generalized_velocity_mut()[0] = s.qdot;
        tree.forward_kinematics(&staged.world.bodies, true);
        for l in tree.links() {
            staged.world.bodies[l.rigid_body_handle()].set_position(*l.local_to_world(), false);
        }
        let row = staged.joints.records.get_mut(&id).ok_or_else(invalid)?;
        row.position = s.q;
        staged.joints.finish_velocities(&mut staged.world, false);
        staged
            .world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut staged.world.colliders);
        if staged.checkpoint_timed_world_revolute(last)? != image {
            return Err(invalid());
        }
        *self = staged;
        Ok(last)
    }
}
