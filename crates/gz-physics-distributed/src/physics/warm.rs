// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Complete dynamic-box solver migration between trusted peers running the same build.
use super::*;
use gz_physics_multifidelity::{PARTITION_CHECKPOINT_LIMIT, partition_linear_speed_limit};
use sha2::{Digest, Sha256};

/// Maximum number of ordered COM impulses submitted for one tick.
pub const MAX_PENDING_IMPULSES: usize = 32;
const SCHEMA: u32 = 2;
fn schema(profile: NativeSolverProfile) -> u32 {
    match profile {
        NativeSolverProfile::Standard32 => SCHEMA,
        NativeSolverProfile::Extended64 => 3,
        NativeSolverProfile::Standard32PerPoint => 4,
        NativeSolverProfile::Extended64PerPoint => 5,
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PendingImpulse {
    pub id: u64,
    pub impulse: [f64; 3],
}

impl PartialEq for PendingImpulse {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.impulse.map(f64::to_bits) == other.impulse.map(f64::to_bits)
    }
}

/// Opaque native state is bound to the public snapshot, local handles and ordered
/// commands. The SHA-256 detects corruption. The caller establishes peer provenance.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WarmCheckpoint {
    pub snapshot: Snapshot,
    pub pending: Vec<PendingImpulse>,
    pub manifold_count: usize,
    warmstart_bits: Vec<u64>,
    schema: u32,
    bindings: Vec<u64>,
    native: Vec<u8>,
    digest: [u8; 32],
    #[serde(default, skip_serializing_if = "ContactPrefix::is_empty")]
    prefix: ContactPrefix,
}

// Bincode 1 uses the same length prefix and bytes for both forms. In this
// checksum the native payload can enter SHA-256 in one write. The public
// checkpoint's derived Serialize and Deserialize implementations stay intact.
struct BinaryPayload<'a>(&'a [u8]);
impl Serialize for BinaryPayload<'_> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_bytes(self.0)
    }
}

impl WarmCheckpoint {
    /// Cheap compatibility gate before hashing or hydrating a trusted peer payload.
    pub(crate) fn matches_profile(&self, expected: NativeSolverProfile) -> bool {
        self.schema == schema(expected)
    }

    // The private negotiated transport separates this byte vector from the JSON
    // metadata. Receipt verification and restore always see the full checkpoint.
    pub(crate) fn native_wire_payload(&mut self) -> &mut Vec<u8> {
        &mut self.native
    }

    pub fn native_bytes(&self) -> usize {
        self.native.len()
    }

    pub fn warmstart_normal_impulses(&self) -> impl Iterator<Item = f64> + '_ {
        self.warmstart_bits.iter().copied().map(f64::from_bits)
    }

    fn checksum(&self) -> Result<[u8; 32], WorldError> {
        let mut hash = Sha256::new();
        // Keep the historical domain and bind the selected native profile through schema.
        hash.update(b"Gazebo dynamic-box checkpoint v2: rapier3d-f64=0.36.1;bincode=1.3.3");
        bincode::serialize_into(
            &mut hash,
            &(
                self.schema,
                &self.snapshot,
                &self.pending,
                &self.bindings,
                self.manifold_count,
                &self.warmstart_bits,
                BinaryPayload(&self.native),
            ),
        )
        .map_err(|_| WorldError::Checkpoint)?;
        // Legacy one-solve checkpoints keep their original bytes and checksum.
        if !self.prefix.is_empty() {
            hash.update(b"first nine native solves");
            bincode::serialize_into(&mut hash, &self.prefix).map_err(|_| WorldError::Checkpoint)?;
        }
        Ok(hash.finalize().into())
    }
}

/// Check ordered accumulation against the current public COM velocity and mass.
/// Every intermediate velocity must be representable, as each impulse is applied in turn.
pub fn validate_impulse(
    bodies: &[Body],
    pending: &[PendingImpulse],
    id: u64,
    impulse: [f64; 3],
) -> Result<(), WorldError> {
    if pending.len() >= MAX_PENDING_IMPULSES {
        return Err(WorldError::Pending);
    }
    let body = bodies
        .iter()
        .find(|b| b.id == id)
        .ok_or(WorldError::Missing(id))?;
    if body.tier != Tier::Dynamic {
        return Err(PhysicsError::Unsupported {
            engine: "multifidelity",
            capability: "queued impulse on prescribed body",
        }
        .into());
    }
    let mut linear = body.linear;
    for change in pending
        .iter()
        .filter(|c| c.id == id)
        .map(|c| c.impulse)
        .chain([impulse])
    {
        for (velocity, component) in linear.iter_mut().zip(change) {
            *velocity += component * body.mass.recip();
        }
        // The Rapier staged solver clamps COM speed per substep. An accepted impulse
        // must be representable and realizable before another pending input acts.
        let speed = linear.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !speed.is_finite()
            || speed > partition_linear_speed_limit()
            // At the admitted speed and maximum 10 ms interval, travel is at most
            // four metres. Adding it to any finite coordinate cannot overflow f64.
            || !body.position.into_iter().all(f64::is_finite)
        {
            return Err(PhysicsError::InvalidValue.into());
        }
    }
    Ok(())
}

/// Recheck previously accepted, ordered COM impulses against an edited twist.
pub(crate) fn validate_queued_for(
    body: &Body,
    pending: &[PendingImpulse],
) -> Result<(), WorldError> {
    let mut accepted = Vec::new();
    for p in pending.iter().filter(|p| p.id == body.id) {
        validate_impulse(std::slice::from_ref(body), &accepted, p.id, p.impulse)?;
        accepted.push(p.clone());
    }
    Ok(())
}

impl Island {
    /// Apply a world-frame impulse (N s) at the dynamic box's center of mass,
    /// in submission order, immediately before its next advancing physics solve.
    pub fn queue_impulse(&mut self, id: u64, impulse: [f64; 3]) -> Result<(), WorldError> {
        self.healthy()?;
        validate_impulse(&self.committed.bodies, &self.pending, id, impulse)?;
        self.pending.push(PendingImpulse { id, impulse });
        Ok(())
    }

    pub fn checkpoint(&self) -> Result<WarmCheckpoint, WorldError> {
        self.checkpoint_with_commands(&self.pending, self.profile)
    }

    pub(super) fn checkpoint_with_commands(
        &self,
        pending: &[PendingImpulse],
        profile: NativeSolverProfile,
    ) -> Result<WarmCheckpoint, WorldError> {
        self.healthy()?;
        let bodies = &self.committed.bodies;
        if !(2..=profile.max_bodies()).contains(&bodies.len())
            || bodies.iter().any(|b| b.tier != Tier::Dynamic)
        {
            return Err(WorldError::Checkpoint);
        }
        self.solver_image(pending, profile)
    }

    pub(super) fn solver_image(
        &self,
        pending: &[PendingImpulse],
        profile: NativeSolverProfile,
    ) -> Result<WarmCheckpoint, WorldError> {
        self.healthy()?;
        if profile != self.profile {
            return Err(WorldError::Checkpoint);
        }
        let bodies = &self.committed.bodies;
        let world = self.sim.world();
        let backend = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let (manifold_count, warmstart_bits) = backend.partition_warmstart();
        let mut checkpoint = WarmCheckpoint {
            manifold_count,
            warmstart_bits,
            schema: schema(profile),
            snapshot: self.snapshot(),
            pending: pending.to_vec(),
            bindings: bodies
                .iter()
                .map(|b| {
                    world
                        .get::<Binding>(self.index[&b.id])
                        .unwrap()
                        .native
                        .raw()
                })
                .collect(),
            native: backend.checkpoint_solver_with_profile(profile)?,
            digest: [0; 32],
            prefix: self.prefix.clone(),
        };
        checkpoint.digest = checkpoint.checksum()?;
        Ok(checkpoint)
    }

    /// Restore a complete, verified checkpoint from an authorized peer on the
    /// same native build. Solver-internal serialization is unsuitable for untrusted files.
    pub fn restore_warm(state: &WarmCheckpoint) -> Result<Self, WorldError> {
        Self::restore_warm_with_profile(state, NativeSolverProfile::Standard32)
    }
    /// Restore a whole dynamic island under the explicitly selected native identity.
    pub fn restore_warm_with_profile(
        state: &WarmCheckpoint,
        profile: NativeSolverProfile,
    ) -> Result<Self, WorldError> {
        let bodies = &state.snapshot.bodies;
        if bodies.len() < 2 || bodies.iter().any(|b| b.tier != Tier::Dynamic) {
            return Err(WorldError::Checkpoint);
        }
        Self::restore_solver_with_profile(state, profile)
    }

    pub(crate) fn restore_solver_with_profile(
        state: &WarmCheckpoint,
        profile: NativeSolverProfile,
    ) -> Result<Self, WorldError> {
        let bodies = &state.snapshot.bodies;
        let count = bodies.len();
        if !(1..=profile.max_bodies()).contains(&count) {
            return Err(WorldError::Checkpoint);
        }
        let max_pairs = count * (count - 1) / 2;
        let max_manifolds = max_pairs + usize::from(state.snapshot.ground.is_some()) * count;
        if !state.matches_profile(profile)
            || state.native.is_empty()
            || state.native.len() > PARTITION_CHECKPOINT_LIMIT
            || bodies.windows(2).any(|p| p[0].id >= p[1].id)
            || state.bindings.len() != count
            || state
                .bindings
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                != count
            || state.pending.len() > MAX_PENDING_IMPULSES
            || state.snapshot.supports.len() > count
            || (!state.snapshot.supports.is_empty()
                && (state.snapshot.ground.is_none() || !state.snapshot.contact_history))
            || state.manifold_count > max_manifolds
            || state.warmstart_bits.len() > 8 * state.manifold_count
            || !state.prefix.valid_for(&state.snapshot)
            || state.snapshot.contacts > max_pairs
            || state.snapshot.impulses.len() != state.snapshot.contacts
            || (state.snapshot.contacts > 0 && !state.snapshot.contact_history)
            || state.checksum()? != state.digest
        {
            return Err(WorldError::Checkpoint);
        }
        for (i, command) in state.pending.iter().enumerate() {
            validate_impulse(
                &state.snapshot.bodies,
                &state.pending[..i],
                command.id,
                command.impulse,
            )?;
        }
        let backend = MultiFidelityBackend::restore_solver_with_profile(&state.native, profile)?;
        let (manifold_count, warmstart_bits) = backend.partition_warmstart();
        if backend.partition_plane()?.map(|g| g.bits())
            != state.snapshot.ground.map(GroundPlane::bits)
            || !backend.partition_step_matches(state.snapshot.step.duration())
            || backend.partition_box_count() != count
            || manifold_count != state.manifold_count
            || warmstart_bits != state.warmstart_bits
        {
            return Err(WorldError::Checkpoint);
        }
        let mut handles = BTreeMap::new();
        for (body, &raw) in bodies.iter().zip(&state.bindings) {
            let handle = BodyId::from_raw(raw);
            let (shape, mass) = backend
                .partition_shape(handle)
                .ok_or(WorldError::Checkpoint)?;
            let (primitive, size) =
                Primitive::from_shape(shape).map_err(|_| WorldError::Checkpoint)?;
            if primitive != body.primitive
                || backend.partition_friction(handle).map(f64::to_bits)
                    != Some(body.friction.to_bits())
                || size.map(f64::to_bits) != body.size.map(f64::to_bits)
                || mass.to_bits() != body.mass.to_bits()
            {
                return Err(WorldError::Checkpoint);
            }
            handles.insert(body.id, handle);
        }
        let mut island = Self::install(profile, backend, handles, state.snapshot.clone())?;
        island.prefix.clone_from(&state.prefix);
        island.refresh(state.snapshot.tick);
        if island.committed != state.snapshot {
            return Err(WorldError::Checkpoint);
        }
        island.pending.clone_from(&state.pending);
        Ok(island)
    }
}

#[cfg(test)]
#[path = "../../tests/private/warm_checkpoint.rs"]
mod tests;


#[cfg(test)]
#[test]
fn binary_payload_preserves_the_historical_checkpoint_checksum_bytes() {
    for size in [0, 1, 127, 128, 255, 256, 8191, 8192, 65536, PARTITION_CHECKPOINT_LIMIT] {
        let bytes: Vec<_> = (0..size).map(|i| (i % 256) as u8).collect();
        assert_eq!(bincode::serialize(&bytes).unwrap(), bincode::serialize(&BinaryPayload(&bytes)).unwrap());
    }
}
