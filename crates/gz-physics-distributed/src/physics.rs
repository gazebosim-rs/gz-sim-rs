// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A worker's always-awake, ground-free box island on Gazebo's Bevy physics schedule.
use bevy_ecs::prelude::{Component, Entity};
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, PhysicsBackend, PhysicsError, RotationSpec, Shape,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{IdentityError, Simulation, StepError, assign_id};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, HashMap},
    time::Duration,
};
use thiserror::Error;

mod ground;
pub use ground::{GroundPlane, GroundSupport};
mod partition;
pub use partition::{FiniteContactPatch, GroundContactPatch};
mod step;
pub use step::SimulationStep;
mod swept;
pub(crate) use swept::{forced_contact_groups, forced_groups, forced_groups_with_profile};
mod wrench;
pub use wrench::{BodyWrench, StepField};
pub(crate) use wrench::{ForceMotion, field_motion, validate_wrenches};
mod refinement;
use refinement::{ContactPrefix, validate_substeps};
mod replace;
mod warm;
pub use gz_physics_multifidelity::{MAX_CHECKPOINT_BODIES, NativeFrictionModel, NativeSolverProfile};
/// Cold construction only. Native restarts carry the complete existing solver graphs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) enum NativeSeedLayout {
    Standard32,
    Extended64,
    Canonical64,
    Standard32PerPoint,
    Extended64PerPoint,
    Canonical64PerPoint,
}
impl NativeSeedLayout {
    pub(crate) const fn profile(self) -> NativeSolverProfile {
        match self {
            Self::Standard32 => NativeSolverProfile::Standard32,
            Self::Extended64 | Self::Canonical64 => NativeSolverProfile::Extended64,
            Self::Standard32PerPoint => NativeSolverProfile::Standard32PerPoint,
            Self::Extended64PerPoint | Self::Canonical64PerPoint => NativeSolverProfile::Extended64PerPoint,
        }
    }
    pub(crate) const fn is_canonical(self) -> bool {
        matches!(self, Self::Canonical64 | Self::Canonical64PerPoint)
    }
    pub(crate) fn with_friction(self, model: NativeFrictionModel) -> Self {
        if self.is_canonical() {
            match model {
                NativeFrictionModel::Grouped => Self::Canonical64,
                NativeFrictionModel::PerPoint => Self::Canonical64PerPoint,
            }
        } else {
            self.profile().with_friction(model).into()
        }
    }
}
impl From<NativeSolverProfile> for NativeSeedLayout {
    fn from(profile: NativeSolverProfile) -> Self {
        match profile {
            NativeSolverProfile::Standard32 => Self::Standard32,
            NativeSolverProfile::Extended64 => Self::Extended64,
            NativeSolverProfile::Standard32PerPoint => Self::Standard32PerPoint,
            NativeSolverProfile::Extended64PerPoint => Self::Extended64PerPoint,
        }
    }
}
/// Maximum distinct mobile identities retained by one worker, including ownership tombstones.
/// A connected native contact solver has its separate, smaller checkpoint bound.
pub const MAX_PARTITION_BODIES: usize = 256;
pub use partition::{ComponentInfo, ComponentWork, Partition, active_contact_group};
pub(crate) use partition::{PartitionImage, SolveSchedule, separated_groups};
pub(crate) use replace::admit_edit;
pub use swept::{
    partition_can_step, partitions_may_interact, potential_contact_groups,
    potential_contact_groups_in_field,
};
#[cfg(feature = "smooth-plane-diagnostics")]
pub use swept::potential_contact_groups_on_smooth_plane;
pub(crate) use warm::validate_queued_for;
pub use warm::{MAX_PENDING_IMPULSES, PendingImpulse, WarmCheckpoint, validate_impulse};

/// Default interval used by legacy worker seeds and callers.
pub const STEP: Duration = SimulationStep::Ms10.duration();

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum Tier {
    Prescribed,
    Dynamic,
    Fixed,
}

/// Shape of a centered homogeneous rigid link. Spheres use three equal diameters.
/// A capsule uses its diameter in X and Y and the length between cap centers in Z.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Primitive {
    #[default]
    Box,
    Sphere,
    Capsule,
    Cylinder,
}
impl Primitive {
    fn is_box(&self) -> bool {
        *self == Self::Box
    }
    /// Portable dimensions in meters, before the native conversion to half-extents.
    pub(crate) fn from_shape(shape: Shape) -> Result<(Self, [f64; 3]), PhysicsError> {
        let (kind, half) = match shape {
            Shape::Box { half_extents } => (Self::Box, half_extents),
            Shape::Sphere { radius } => (Self::Sphere, [radius; 3]),
            Shape::Capsule {
                radius,
                half_length,
            } => (Self::Capsule, [radius, radius, half_length]),
            Shape::Cylinder {
                radius,
                half_height,
            } => (Self::Cylinder, [radius, radius, half_height]),
        };
        let size = half.map(|s| 2. * s);
        kind.shape(size)?;
        Ok((kind, size))
    }
    pub(crate) fn shape(self, size: [f64; 3]) -> Result<Shape, PhysicsError> {
        if !size.into_iter().all(|s| s > 0. && s.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        match self {
            Self::Box => Ok(Shape::Box {
                half_extents: size.map(|s| s * 0.5),
            }),
            Self::Sphere if size == [size[0]; 3] => Ok(Shape::Sphere {
                radius: size[0] * 0.5,
            }),
            Self::Cylinder if size[0] == size[1] => Ok(Shape::Cylinder {
                radius: 0.5 * size[0],
                half_height: 0.5 * size[2],
            }),
            Self::Capsule if size[0] == size[1] => Ok(Shape::Capsule {
                radius: 0.5 * size[0],
                half_length: 0.5 * size[2],
            }),
            _ => Err(PhysicsError::InvalidValue),
        }
    }
}
/// Portable, world-frame state and construction values. Box sides are full
/// lengths in metres, mass is kg, and the unit quaternion is xyzw.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub id: u64,
    pub tier: Tier,
    pub size: [f64; 3],
    pub mass: f64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub linear: [f64; 3],
    pub angular: [f64; 3],
    /// Isotropic Coulomb coefficient. Positive zero retains the original wire and digest encoding.
    #[serde(default, skip_serializing_if = "positive_zero")]
    pub friction: f64,
    #[serde(default, skip_serializing_if = "Primitive::is_box")]
    pub primitive: Primitive,
}
fn positive_zero(value: &f64) -> bool {
    value.to_bits() == 0
}

// Portable snapshots are also protocol receipts. Preserve every floating-point
// bit, including the sign of zero, when checking identity across a transfer.
impl PartialEq for Body {
    fn eq(&self, other: &Self) -> bool {
        self.same_definition(other)
            && self.position.map(f64::to_bits) == other.position.map(f64::to_bits)
            && self.orientation.map(f64::to_bits) == other.orientation.map(f64::to_bits)
            && self.linear.map(f64::to_bits) == other.linear.map(f64::to_bits)
            && self.angular.map(f64::to_bits) == other.angular.map(f64::to_bits)
    }
}

impl Body {
    /// Geometry and mass properties are immutable throughout a body's lifetime.
    pub(crate) fn same_definition(&self, other: &Self) -> bool {
        self.id == other.id
            && self.tier == other.tier
            && self.primitive == other.primitive
            && self.friction.to_bits() == other.friction.to_bits()
            && self.mass.to_bits() == other.mass.to_bits()
            && self.size.map(f64::to_bits) == other.size.map(f64::to_bits)
    }

    pub fn shape(&self) -> Result<Shape, PhysicsError> {
        self.primitive.shape(self.size)
    }
    pub(crate) fn bounds(&self) -> Result<gz_physics_multifidelity::PartitionBounds, PhysicsError> {
        gz_physics_multifidelity::PartitionBounds::new(
            self.shape()?,
            self.position,
            self.orientation,
        )
    }
    /// Test whether two centered boxes approach contact during the solver interval
    /// at their current world twists. Forces and future impacts can alter that path.
    pub fn predicts_box_contact(
        &self,
        other: &Body,
        step: SimulationStep,
    ) -> Result<bool, PhysicsError> {
        if self.primitive != Primitive::Box || other.primitive != Primitive::Box {
            return Err(PhysicsError::Unsupported {
                engine: "distributed",
                capability: "box contact forecast requires two cuboids",
            });
        }
        gz_physics_multifidelity::predicts_box_contact(
            [self.size, other.size],
            [self.state(), other.state()],
            step.duration(),
        )
    }

    pub fn promoted(&self) -> Self {
        Self {
            tier: Tier::Dynamic,
            ..self.clone()
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactImpulse {
    pub a: u64,
    pub b: u64,
    pub normal_impulse: f64,
}

impl PartialEq for ContactImpulse {
    fn eq(&self, other: &Self) -> bool {
        self.a == other.a
            && self.b == other.b
            && self.normal_impulse.to_bits() == other.normal_impulse.to_bits()
    }
}

/// The last committed public barrier. Native contact manifolds are intentionally
/// absent, so restoring a snapshot with any contact history is refused.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Snapshot {
    pub tick: u64,
    pub bodies: Vec<Body>,
    pub contacts: usize,
    pub contact_history: bool,
    pub impulses: Vec<ContactImpulse>,
    #[serde(default, skip_serializing_if = "SimulationStep::is_default")]
    pub step: SimulationStep,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ground: Option<GroundPlane>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub supports: Vec<GroundSupport>,
}

#[derive(Debug, Error)]
pub enum WorldError {
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Clock(#[from] StepError),
    #[error(transparent)]
    Identity(#[from] IdentityError),
    #[error("native contact history prevents cache-free transfer or restoration")]
    ContactHistory,
    #[error("possible contacts require a solver merger outside the admitted dynamic-box profile")]
    CrossIsland,
    #[error("global body ID {0} already has a local owner")]
    Duplicate(u64),
    #[error("global body ID {0} has no local owner")]
    Missing(u64),
    #[error("invalid or unsupported dynamic-box warm checkpoint")]
    Checkpoint,
    #[error("checkpoint of solver {members:?} failed: {source}")]
    ImageCapture {
        members: Vec<u64>,
        source: Box<WorldError>,
    },
    #[error("pending impulses prevent a cold transfer or exceed the queue capacity")]
    Pending,
    #[error("refinement IDs must be sorted, locally owned and cover complete prospective native islands")]
    IslandResolution,
    #[error("distributed barrier tick overflow")]
    TimeOverflow,
    #[error("local physics world is faulted: {0}")]
    Fault(String),
}

// Neither local Bevy entity nor local backend IDs are serialized.
#[derive(Component)]
struct Binding {
    id: u64,
    native: BodyId,
}

/// Local simulation for migrating a contact-free prescribed or dynamic box.
/// Pending COM impulses and native contact caches require a whole-island checkpoint.
pub struct Island {
    profile: NativeSolverProfile,
    sim: Simulation,
    index: BTreeMap<u64, Entity>,
    committed: Snapshot,
    pending: Vec<PendingImpulse>,
    prefix: ContactPrefix,
}

impl Island {
    /// Start a fresh solver from a snapshot eligible for migration. At nonzero
    /// tick the Bevy clock measures time since hydration; committed ticks retain
    /// the global barrier number and drive every worker with the same fixed dt.
    pub fn restore(snapshot: &Snapshot) -> Result<Self, WorldError> {
        Self::restore_with_profile(snapshot, NativeSolverProfile::Standard32)
    }

    /// Cold construction from a contact-free snapshot using an explicit native identity.
    pub fn restore_with_profile(
        snapshot: &Snapshot,
        profile: NativeSolverProfile,
    ) -> Result<Self, WorldError> {
        if !snapshot.supports.is_empty()
            || snapshot.contact_history
            || snapshot.contacts != 0
            || !snapshot.impulses.is_empty()
        {
            return Err(WorldError::ContactHistory);
        }
        let mut bodies = snapshot.bodies.clone();
        bodies.sort_unstable_by_key(|b| b.id);
        let (backend, handles) = materialize(&bodies, snapshot.step, snapshot.ground, profile)?;
        Self::install(
            profile,
            backend,
            handles,
            Snapshot {
                tick: snapshot.tick,
                step: snapshot.step,
                ground: snapshot.ground,
                bodies,
                ..Snapshot::default()
            },
        )
    }

    fn install(
        profile: NativeSolverProfile,
        backend: MultiFidelityBackend,
        handles: BTreeMap<u64, BodyId>,
        committed: Snapshot,
    ) -> Result<Self, WorldError> {
        let mut sim = Simulation::new(committed.step.duration())?;
        Self::install_step(&mut sim, backend);
        let mut index = BTreeMap::new();
        for (id, native) in handles {
            let e = spawn_binding(&mut sim, id, native)?;
            index.insert(id, e);
        }
        Ok(Self {
            profile,
            sim,
            index,
            committed,
            pending: Vec::new(),
            prefix: ContactPrefix::default(),
        })
    }

    #[cfg(test)]
    pub(crate) fn inject_commit_fault(&mut self) {
        use bevy_ecs::prelude::Res;
        use gz_sim_core::{Phase, StepFault};
        self.sim
            .add_systems(Phase::PhysicsCommit, |fault: Res<StepFault>| {
                fault.report(std::io::Error::other("interrupted local commit"));
            });
    }

    pub fn snapshot(&self) -> Snapshot {
        self.committed.clone()
    }

    /// Advance exactly one public step. No partially integrated state is
    /// published if the Bevy schedule faults.
    pub fn step(&mut self) -> Result<(), WorldError> {
        let tick = self
            .committed
            .tick
            .checked_add(1)
            .ok_or(WorldError::TimeOverflow)?;
        self.healthy()?;
        if !self.pending.is_empty() {
            let world = self.sim.world_mut();
            for command in &self.pending {
                let id = world
                    .get::<Binding>(self.index[&command.id])
                    .unwrap()
                    .native;
                if let Err(error) = world.resource_mut::<PhysicsBackend>().apply_impulse(
                    id,
                    command.impulse,
                    [0.; 3],
                ) {
                    world.resource::<gz_sim_core::StepFault>().report(error);
                    return self.healthy();
                }
            }
            self.pending.clear();
        }
        if let Err(error) = self.sim.step_exact(1) {
            return Err(WorldError::Fault(
                self.sim
                    .fault()
                    .map_or_else(|| error.to_string(), ToString::to_string),
            ));
        }
        let observed = self.commit_refinement();
        self.refresh_with_contacts(tick, observed);
        Ok(())
    }

    fn refresh(&mut self, tick: u64) {
        self.refresh_with_contacts(tick, None);
    }

    fn refresh_with_contacts(&mut self, tick: u64, observed: Option<ContactPrefix>) {
        let world = self.sim.world();
        let backend = world.resource::<PhysicsBackend>();
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .expect("island backend");
        // Refined execution has already validated the final native contacts
        // against the prefix. Hydration and same-tick edits query them afresh.
        let pairs = if observed.is_none() {
            native.rigid_contact_observations()
        } else {
            Vec::new()
        };
        let ground_contacts = if observed.is_none() && self.committed.ground.is_some() {
            native.partition_ground_contacts()
        } else {
            Vec::new()
        };
        let next = &mut self.committed;
        next.tick = tick;
        if let Some(observed) = observed {
            next.impulses = observed.pairs;
            next.supports = observed.supports;
        } else {
            next.impulses.clear();
            next.supports.clear();
        }
        next.contacts = next.impulses.len();
        let mut ids = HashMap::new();
        for body in &mut next.bodies {
            let binding = world
                .get::<Binding>(self.index[&body.id])
                .expect("registered entity");
            assert_eq!(binding.id, body.id);
            let state = backend
                .state(binding.native)
                .expect("registered native body");
            body.position = state.position;
            body.orientation = state.orientation;
            body.linear = state.velocity;
            body.angular = state.angular_velocity;
            body.tier = if backend.is_kinematic(binding.native) == Some(true) {
                Tier::Prescribed
            } else if backend.is_dynamic(binding.native) == Some(true) {
                Tier::Dynamic
            } else {
                Tier::Fixed
            };
            if !ground_contacts.is_empty() || !pairs.is_empty() {
                ids.insert(binding.native, body.id);
            }
        }
        for p in ground_contacts {
            next.contact_history = true;
            if p.active {
                next.supports.push(GroundSupport {
                    body: ids[&p.body],
                    normal_impulse: p.normal_impulse,
                });
            }
        }
        next.supports.sort_unstable_by_key(|p| p.body);
        for pair in pairs {
            // Even an inactive speculative manifold can have solver history.
            next.contact_history = true;
            if pair.active {
                next.contacts += 1;
                let (a, b) = (ids[&pair.a], ids[&pair.b]);
                next.impulses.push(ContactImpulse {
                    a: a.min(b),
                    b: a.max(b),
                    normal_impulse: pair.normal_impulse,
                });
            }
        }
        next.impulses.sort_unstable_by_key(|p| (p.a, p.b));
        self.prefix.add_to(next);
    }

    fn healthy(&self) -> Result<(), WorldError> {
        if let Some(error) = self.sim.fault() {
            return Err(WorldError::Fault(error.to_string()));
        }
        Ok(())
    }

    fn transferable(&self) -> Result<(), WorldError> {
        self.healthy()?;
        if !self.pending.is_empty() {
            return Err(WorldError::Pending);
        }
        if self.committed.contact_history {
            return Err(WorldError::ContactHistory);
        }
        Ok(())
    }

    pub fn export(&self, id: u64) -> Result<Body, WorldError> {
        self.transferable()?;
        let pos = self
            .committed
            .bodies
            .binary_search_by_key(&id, |b| b.id)
            .map_err(|_| WorldError::Missing(id))?;
        Ok(self.committed.bodies[pos].clone())
    }

    fn with_body(&self, body: Body) -> Result<Vec<Body>, WorldError> {
        self.transferable()?;
        let mut bodies = self.committed.bodies.clone();
        match bodies.binary_search_by_key(&body.id, |b| b.id) {
            Ok(_) => Err(WorldError::Duplicate(body.id)),
            Err(pos) => {
                bodies.insert(pos, body);
                Ok(bodies)
            }
        }
    }

    pub fn validate_import(&self, body: &Body) -> Result<(), WorldError> {
        materialize(
            &self.with_body(body.clone())?,
            self.committed.step,
            self.committed.ground,
            self.profile,
        )?;
        Ok(())
    }

    /// Build a full replacement solver in stable-ID order before committing to
    /// preserve native contact ordering irrespective of the order of arrivals.
    /// Existing Bevy entities and their world-local identities stay in place.
    pub fn import(&mut self, body: Body) -> Result<(), WorldError> {
        let id = body.id;
        let bodies = self.with_body(body)?;
        let (backend, handles) = materialize(
            &bodies,
            self.committed.step,
            self.committed.ground,
            self.profile,
        )?;
        let entity = spawn_binding(&mut self.sim, id, handles[&id])?;
        self.index.insert(id, entity);
        let world = self.sim.world_mut();
        for (id, native) in handles {
            world.get_mut::<Binding>(self.index[&id]).unwrap().native = native;
        }
        world.insert_resource(PhysicsBackend::from(backend));
        self.committed.bodies = bodies;
        Ok(())
    }

    pub fn remove(&mut self, id: u64) -> Result<(), WorldError> {
        self.transferable()?;
        let &entity = self.index.get(&id).ok_or(WorldError::Missing(id))?;
        let world = self.sim.world_mut();
        let native = world.get::<Binding>(entity).unwrap().native;
        if !world.resource_mut::<PhysicsBackend>().remove(native) {
            return Err(WorldError::Missing(id));
        }
        world.despawn(entity);
        self.index.remove(&id);
        self.committed.bodies.retain(|b| b.id != id);
        Ok(())
    }
}

fn spawn_binding(sim: &mut Simulation, id: u64, native: BodyId) -> Result<Entity, WorldError> {
    let world = sim.world_mut();
    let e = world.spawn(Binding { id, native }).id();
    if let Err(error) = assign_id(world, e) {
        world.despawn(e);
        return Err(error.into());
    }
    Ok(e)
}

fn materialize(
    bodies: &[Body],
    step: SimulationStep,
    ground: Option<GroundPlane>,
    profile: NativeSolverProfile,
) -> Result<(MultiFidelityBackend, BTreeMap<u64, BodyId>), WorldError> {
    let mut backend = MultiFidelityBackend::new_with_profile([0.; 3], profile)?;
    if !step.is_default() {
        backend.set_partition_step(step.duration())?;
    }
    if let Some(ground) = ground {
        backend.set_partition_plane(ground.point(), ground.normal(), ground.friction())?;
    }
    let mut handles = BTreeMap::new();
    for b in bodies {
        if handles.contains_key(&b.id) {
            return Err(WorldError::Duplicate(b.id));
        }
        let handle = spawn_box(&mut backend, b, step)?;
        handles.insert(b.id, handle);
    }
    Ok((backend, handles))
}

fn spawn_box(
    backend: &mut MultiFidelityBackend,
    b: &Body,
    step: SimulationStep,
) -> Result<BodyId, WorldError> {
    if b.tier == Tier::Fixed && b.mass.to_bits() != 0 {
        return Err(PhysicsError::InvalidValue.into());
    }
    if b.tier == Tier::Prescribed {
        backend.validate_kinematic_velocity(b.linear, b.angular, step.duration())?;
    }
    backend
        .spawn_partition_primitive(
            BodySpec {
                shape: b.shape()?,
                position: b.position,
                velocity: b.linear,
                mass: (b.tier != Tier::Fixed).then_some(b.mass),
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            b.tier == Tier::Prescribed,
            b.friction,
        )
        .map_err(Into::into)
}

/// Two 0.6 m boxes approach in free space. Body 1 starts prescribed and reaches
/// x=0 at one second, before its first contact with the 2 kg dynamic box.
pub fn fixture() -> Vec<Body> {
    let a = Body {
        id: 1,
        primitive: Primitive::Box,
        friction: 0.,
        tier: Tier::Prescribed,
        size: [0.6; 3],
        mass: 1.,
        position: [-2., 0., 0.],
        orientation: [0., 0., 0., 1.],
        linear: [2., 0., 0.],
        angular: [0.; 3],
    };
    let b = Body {
        id: 2,
        tier: Tier::Dynamic,
        mass: 2.,
        position: [1.2, 0., 0.],
        linear: [-0.2, 0., 0.],
        ..a.clone()
    };
    vec![a, b]
}

#[cfg(test)]
#[path = "../tests/private/physics.rs"]
mod tests;
