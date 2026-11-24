// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-build warm transfer of one complete unjointed primitive graph with prescribed curves.
mod construction;
use super::*;
use bincode::Options;
use sha2::{Digest, Sha256};
const MAGIC: &[u8; 8] = b"GZCURV01";
pub const TRAJECTORY_IMAGE_LIMIT: usize = 4 * 1024 * 1024;
const BODIES: usize = 64;
fn invalid() -> PhysicsError {
    PhysicsError::InvalidValue
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(TRAJECTORY_IMAGE_LIMIT as u64)
        .reject_trailing_bytes()
}
fn append(out: &mut Vec<u8>, value: &impl serde::Serialize) -> Result<(), PhysicsError> {
    codec()
        .serialize_into(&mut *out, value)
        .map_err(|_| invalid())?;
    if out.len() > TRAJECTORY_IMAGE_LIMIT {
        return Err(invalid());
    }
    Ok(())
}
fn read<T: serde::de::DeserializeOwned>(input: &mut &[u8]) -> Result<T, PhysicsError> {
    codec().deserialize_from(input).map_err(|_| invalid())
}
impl RapierBackend {
    /// Materialize a pure translation fleet when a process takes the complete native graph.
    pub fn prepare_trajectory_owner(&mut self) -> Result<(), PhysicsError> {
        if !self.curves.authored() {
            return Err(invalid());
        }
        self.materialize_fleet();
        self.curve_recipe().map(|_| ())
    }
    fn curve_recipe(&self) -> Result<[u8; 32], PhysicsError> {
        if !self.curves.authored()
            || !(1..=BODIES).contains(&self.bodies.len())
            || self.fleet_clock != FleetClock::Coupled
            || !matches!(self.fleet_dispatch, fleet_isolation::Dispatch::Disabled)
            || self.bodies.values().any(|(_, construction, ..)| construction.continuous_collision)
            || self.pair_observer.is_some()
            || self.native_profile.is_some()
            || self.join_queries_pending
            || self.grasp_payload_observer.is_some()
            || self.free_coast.is_some()
            || self.conserved_rotor.is_some()
            || !self.held_wrenches.is_empty()
            || !self.fleet_resume_positions.is_empty()
            || !self.spinning_assemblies.is_empty()
        {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "curve checkpoint needs one coupled unjointed graph of up to 64 primitives",
            });
        }
        self.validate_curve_graph(&self.world)?;
        let mut hash = Sha256::new();
        hash.update(b"gz-native-trajectory-construction-01");
        let params = IntegrationParameters {
            dt: IntegrationParameters::default().dt,
            ..self.world.integration_parameters
        };
        hash.update(
            codec()
                .serialize(&(self.next_id, self.world.gravity, params))
                .map_err(|_| invalid())?,
        );
        for (&id, (handle, c, r, m, group)) in &self.bodies {
            if c.articulation.is_some() || self.has_joint(id) != Some(false) {
                return Err(invalid());
            }
            hash.update(
                codec()
                    .serialize(&(
                        id.raw(),
                        handle,
                        c.position,
                        c.velocity,
                        c.mass,
                        c.kinematic,
                        c.can_sleep,
                        r.orientation,
                        r.angular_velocity,
                        m.map(|s| (s.center, s.orientation, s.tensor)),
                        group,
                        &c.collider,
                    ))
                    .map_err(|_| invalid())?,
            );
            if let Some(t) = self.curves.tracks.get(&id) {
                hash.update([1, t.path.end_policy() as u8]);
                hash.update((t.path.waypoints().len() as u64).to_le_bytes());
                for k in t.path.waypoints() {
                    hash.update(k.time.as_nanos().to_le_bytes());
                    for x in k.displacement.into_iter().chain(k.velocity) {
                        hash.update(x.to_le_bytes());
                    }
                }
            } else {
                hash.update([0]);
            }
        }
        Ok(hash.finalize().into())
    }
    fn validate_curve_graph(&self, world: &PhysicsWorld) -> Result<(), PhysicsError> {
        if world.gravity != self.world.gravity
            || !world.quarantine().is_empty()
            || !world.impulse_joints.is_empty()
            || world.multibody_joints.iter().next().is_some()
            || !world.soft_bodies.is_empty()
            || world.bodies.len() != self.bodies.len()
            || world.colliders.len() != self.bodies.len()
            || !world.integration_parameters.dt.is_finite()
            || world.integration_parameters.dt <= 0.
            || world.integration_parameters
                != (IntegrationParameters {
                    dt: world.integration_parameters.dt,
                    ..self.world.integration_parameters
                })
        {
            return Err(invalid());
        }
        for (h, c, ..) in self.bodies.values() {
            let b = world.bodies.get(*h).ok_or_else(invalid)?;
            if !b.is_enabled()
                || b.is_kinematic() != c.kinematic
                || b.is_dynamic() != (c.mass.is_some() && !c.kinematic)
                || !b.translation().is_finite()
                || !b.linvel().is_finite()
                || !b.angvel().is_finite()
                || !b.user_force().is_finite()
                || !b.user_torque().is_finite()
                || rotation(b.rotation().to_array()).is_err()
            {
                return Err(invalid());
            }
            let [collider] = b.colliders() else {
                return Err(invalid());
            };
            let col = world.colliders.get(*collider).ok_or_else(invalid)?;
            let shape = col.shape();
            if col.parent() != Some(*h)
                || !col.is_enabled()
                || !(shape.as_cuboid().is_some()
                    || shape.as_ball().is_some()
                    || shape.as_capsule().is_some()
                    || shape.as_cylinder().is_some())
            {
                return Err(invalid());
            }
        }
        Ok(())
    }
    /// Capture native broad/narrow phases, rolling contact caches, pending wrenches, the integer
    /// curve clock and interval reactions. The receiver must have the same immutable construction.
    pub fn checkpoint_trajectory(&self) -> Result<Vec<u8>, PhysicsError> {
        let seed = self.curve_recipe()?;
        let mut out = Vec::with_capacity(8192);
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&[0; 32]);
        out.extend_from_slice(&seed);
        let mut active = 0u64;
        let mut pending = 0u64;
        for (i, id) in self.bodies.keys().enumerate() {
            if self.curves.tracks.get(id).is_some_and(|t| t.active) {
                active |= 1 << i;
            }
            if self.pending_wrenches.contains(id) {
                pending |= 1 << i;
            }
        }
        append(
            &mut out,
            &(
                self.curves.time,
                self.joints.construction_open(),
                active,
                pending,
                self.curves.steps as u32,
                self.curves.contacts.len() as u32,
            ),
        )?;
        for c in &self.curves.contacts {
            append(
                &mut out,
                &(c.a.raw(), c.b.raw(), c.active, c.normal_impulse),
            )?;
        }
        append(&mut out, &self.world)?;
        let digest = Sha256::digest(&out[40..]);
        out[8..40].copy_from_slice(&digest);
        Ok(out)
    }
    /// Restore an integrity-checked, trusted same-build capture onto fresh, matching bodies.
    /// Every fallible check precedes publication.
    pub fn restore_trajectory(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        if !self.joints.construction_open() || !self.pending_wrenches.is_empty() {
            return Err(invalid());
        }
        self.install_trajectory(bytes)
    }
    /// Replace a same-construction graph at a validated application barrier, retaining public IDs.
    pub fn install_trajectory(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        if !(72..=TRAJECTORY_IMAGE_LIMIT).contains(&bytes.len())
            || &bytes[..8] != MAGIC
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
            || bytes[40..72] != self.curve_recipe()?
        {
            return Err(invalid());
        }
        let mut input = &bytes[72..];
        let (time, open, active, pending, steps, count): (Duration, bool, u64, u64, u32, u32) =
            read(&mut input)?;
        let n = self.bodies.len();
        if steps as usize > MAX_INTERVALS
            || count as usize > n * (n - 1) / 2
            || (n < 64 && (active | pending) >> n != 0)
            || (open
                && (!self.joints.construction_open()
                    || !time.is_zero()
                    || steps != 0
                    || count != 0))
        {
            return Err(invalid());
        }
        let mut contacts = Vec::with_capacity(count as usize);
        let mut previous = None;
        for _ in 0..count {
            let (a, b, active, impulse): (u64, u64, bool, f64) = read(&mut input)?;
            let (a, b) = (BodyId::from_raw(a), BodyId::from_raw(b));
            if a >= b
                || !self.bodies.contains_key(&a)
                || !self.bodies.contains_key(&b)
                || previous.is_some_and(|p| p >= (a, b))
                || !impulse.is_finite()
                || impulse < 0.
            {
                return Err(invalid());
            }
            contacts.push(RigidContactObservation {
                a,
                b,
                active,
                normal_impulse: impulse,
            });
            previous = Some((a, b));
        }
        let world: PhysicsWorld = read(&mut input)?;
        if !input.is_empty() {
            return Err(invalid());
        }
        self.validate_curve_graph(&world)?;
        let mut queued = BTreeSet::new();
        for (i, (&id, (h, ..))) in self.bodies.iter().enumerate() {
            let body = &world.bodies[*h];
            if active & (1 << i) != 0 {
                let t = self.curves.tracks.get(&id).ok_or_else(invalid)?;
                let sample = t.path.sample(time);
                if body.translation() != t.origin + Vector::from(sample.displacement)
                    || body.linvel() != Vector::from(sample.velocity)
                    || body.angvel() != Vector::ZERO
                {
                    return Err(invalid());
                }
            }
            if pending & (1 << i) != 0 {
                if !body.is_dynamic() {
                    return Err(invalid());
                }
                queued.insert(id);
            } else if body.user_force() != Vector::ZERO || body.user_torque() != Vector::ZERO {
                return Err(invalid());
            }
        }
        self.curves.invalidate_layout();
        self.world = world;
        self.pending_wrenches = queued;
        for (i, id) in self.bodies.keys().enumerate() {
            if let Some(t) = self.curves.tracks.get_mut(id) {
                t.active = active & (1 << i) != 0;
            }
        }
        self.curves.time = time;
        self.curves.steps = steps as usize;
        self.curves.contacts = contacts;
        self.curves.normal_reactions = None;
        if !open {
            self.joints.close();
        }
        self.refresh_max_boundary_spin();
        Ok(())
    }
}
