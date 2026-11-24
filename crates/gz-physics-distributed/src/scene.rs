// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two local authoritative workers behind the simulator's normal physics schedule.
use crate::physics::{BodyWrench, validate_wrenches};
use crate::{
    coordinator::{Coordinator, Frame},
    physics::{
        Body, Island, MAX_PARTITION_BODIES, NativeSolverProfile, PendingImpulse, Primitive, SimulationStep, Snapshot,
        Tier, validate_impulse,
    },
    pool::{PoolFrame, WorkerPool},
    transport::WorkerProcess,
};
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, InitialPhysicsState, JointId, JointState,
    PhysicsError, PlaneSpec, PreparedPhysicsReset, RotationSpec, Shape,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{
    any::Any,
    collections::BTreeMap,
    path::{Path, PathBuf},
    sync::Arc,
    time::Duration,
};

#[path = "scene_balance.rs"]
mod balance;
#[path = "scene_checkpoint.rs"]
mod checkpoint;
#[path = "scene_ground.rs"]
mod ground;
#[path = "scene_placement.rs"]
mod placement;
#[path = "scene_replay.rs"]
mod replay;
#[path = "scene_ray.rs"]
mod ray;

const ENGINE: &str = "distributed";
const JOURNAL_LIMIT: usize = 1024;
type Bodies = BTreeMap<BodyId, Body>;
fn unsupported(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: ENGINE,
        capability,
    }
}
fn native(error: impl std::fmt::Display) -> PhysicsError {
    PhysicsError::Native {
        engine: ENGINE,
        message: error.to_string(),
    }
}
fn pool_edit_error(error: crate::coordinator::Error) -> PhysicsError {
    match error {
        crate::coordinator::Error::EditDeferred(reason) => PhysicsError::TransitionDeferred { reason },
        other => native(other),
    }
}
fn supervised_pool_edit(
    pool: &mut WorkerPool,
    mut edit: impl FnMut(&mut WorkerPool) -> Result<(), crate::coordinator::Error>,
) -> Result<(), PhysicsError> {
    match edit(pool) {
        Err(crate::coordinator::Error::Transport(
            crate::transport::TransportError::Unavailable(_) | crate::transport::TransportError::Io(_),
        )) => edit(pool).map_err(pool_edit_error),
        result => result.map_err(pool_edit_error),
    }
}
type SceneSeed = (Bodies, Option<ground::SceneGround>);
enum Started {
    Pair(Box<Coordinator<WorkerProcess>>),
    Pool(Box<WorkerPool>),
}
struct Start {
    bodies: Bodies,
    ground: Option<ground::SceneGround>,
    live: Started,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum InitialPlacement {
    Spatial,
    ContactAware,
    CanonicalReference,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Refinement {
    Fixed,
    Scene,
    Island,
}

/// Centered boxes, spheres, capsules and finite cylinders in uniform gravity at a fixed
/// 1, 2, 5 or 10 ms step. The ECS owns scene identity. Workers own all advancing dynamics.
pub struct SceneBackend {
    executable: PathBuf,
    packed_observations: bool,
    packed_handoffs: bool,
    native_profile: NativeSolverProfile,
    placement: InitialPlacement,
    local_helper_limit: Option<usize>,
    periodic_placement: Option<crate::pool::PeriodicPlacement>,
    refinement: Refinement,
    // The advancing call owns the only strong reference, including during error unwinding.
    scoped_placement: Option<std::sync::Weak<crate::pool::PlacementRecord>>,
    step: SimulationStep,
    gravity: [f64; 3],
    identity: Arc<()>,
    next: u64,
    ground: Option<ground::SceneGround>,
    initial: Bodies,
    loading: Bodies,
    before_start: Vec<PendingImpulse>,
    wrenches: Vec<BodyWrench>,
    live: Option<Coordinator<WorkerProcess>>,
    width: u8,
    pool: Option<WorkerPool>,
}

impl SceneBackend {
    pub fn new(worker: &Path, gravity: [f64; 3], step: Duration) -> Result<Self, PhysicsError> {
        if !gravity.into_iter().all(f64::is_finite) {
            return Err(PhysicsError::InvalidValue);
        }
        let step = SimulationStep::from_duration(step)
            .ok_or_else(|| unsupported("distributed public steps are 1, 2, 5 and 10 ms"))?;
        Ok(Self {
            executable: worker.to_owned(),
            packed_observations: true,
            packed_handoffs: false,
            native_profile: NativeSolverProfile::Standard32,
            placement: InitialPlacement::Spatial,
            local_helper_limit: None,
            periodic_placement: None,
            refinement: Refinement::Fixed,
            scoped_placement: None,
            step,
            gravity,
            identity: Arc::new(()),
            next: 0,
            ground: None,
            initial: BTreeMap::new(),
            loading: BTreeMap::new(),
            before_start: Vec::new(),
            wrenches: Vec::new(),
            live: None,
            width: 2,
            pool: None,
        })
    }
    /// Choose the number of local authoritative workers before scene construction.
    pub fn with_worker_count(mut self, width: u8) -> Result<Self, PhysicsError> {
        if !(2..=8).contains(&width)
            || (width == 2 && !self.native_profile.is_standard())
            || self.live.is_some()
            || self.pool.is_some()
            || !self.loading.is_empty()
            || self.ground.is_some()
        {
            return Err(unsupported(
                "select two through eight workers before loading bodies",
            ));
        }
        self.width = width;
        Ok(self)
    }
    /// Select native capacity and contact friction before seeding a three- to eight-worker pool.
    pub fn with_native_solver_profile(
        mut self,
        profile: NativeSolverProfile,
    ) -> Result<Self, PhysicsError> {
        if (self.width == 2 && !profile.is_standard())
            || (self.placement == InitialPlacement::CanonicalReference
                && profile.capacity_profile() != NativeSolverProfile::Extended64)
            || self.live.is_some()
            || self.pool.is_some()
            || !self.loading.is_empty()
            || self.ground.is_some()
        {
            return Err(unsupported(
                "select a compatible native solver profile before loading bodies",
            ));
        }
        self.native_profile = profile;
        Ok(self)
    }
    /// Seed workers with intact prospective contact groups before the first public step.
    /// Applies to cold starts and scene resets. Live edits still use predictive routing.
    pub fn with_contact_placement(mut self) -> Result<Self, PhysicsError> {
        if self.live.is_some()
            || self.pool.is_some()
            || !self.loading.is_empty()
            || self.ground.is_some()
        {
            return Err(unsupported("select contact placement before loading bodies"));
        }
        if self.placement == InitialPlacement::CanonicalReference {
            return Err(unsupported("select one distributed cold placement policy"));
        }
        self.placement = InitialPlacement::ContactAware;
        Ok(self)
    }

    /// Seed a single native solver in ascending public-ID order on cold start and reset.
    /// Uniform steps and whole-solver migrations preserve native ordering for a closed
    /// scene of at most 64 finite bodies. Later population edits can change this ordering.
    /// Explicitly rebinding an external warm archive preserves the graph stored in that archive.
    pub fn with_canonical_reference(mut self) -> Result<Self, PhysicsError> {
        if self.native_profile.capacity_profile() != NativeSolverProfile::Extended64
            || self.placement == InitialPlacement::ContactAware
            || self.refinement == Refinement::Island
            || self.live.is_some()
            || self.pool.is_some()
            || !self.loading.is_empty()
            || self.ground.is_some()
        {
            return Err(unsupported(
                "canonical reference placement requires an empty extended pool with uniform stepping",
            ));
        }
        self.placement = InitialPlacement::CanonicalReference;
        Ok(self)
    }

    /// Forecast primitive encounters and contacts with shared ground across workers at 10 ms public barriers.
    /// Each selected barrier advances in ten 1 ms solves on every worker.
    pub fn with_adaptive_contacts(self) -> Result<Self, PhysicsError> {
        self.with_refinement(Refinement::Scene)
    }

    /// Resolve prospective contact islands independently inside each public physics tick.
    pub fn with_island_refinement(self) -> Result<Self, PhysicsError> {
        self.with_refinement(Refinement::Island)
    }

    fn with_refinement(mut self, refinement: Refinement) -> Result<Self, PhysicsError> {
        if self.step != SimulationStep::Ms10 {
            return Err(unsupported("adaptive contacts require a 10 ms public step"));
        }
        if self.placement == InitialPlacement::CanonicalReference
            && refinement == Refinement::Island
        {
            return Err(unsupported(
                "canonical reference placement requires uniform stepping",
            ));
        }
        self.refinement = refinement;
        Ok(self)
    }

    /// Maximum native solves per island in the most recently committed public interval.
    /// Tick zero has no subdivision decision.
    pub fn last_substeps(&self) -> Option<u8> {
        if let Some(pool) = &self.pool {
            pool.last_substeps()
        } else {
            self.live.as_ref().and_then(Coordinator::last_substeps)
        }
    }

    /// Body IDs that actually ran at 1 ms in the last island-selective barrier.
    /// Before the first step, and after a scene-wide or fixed step, there is no island receipt.
    pub fn last_island_refined_bodies(&self) -> Option<&[u64]> {
        if let Some(pool) = &self.pool {
            pool.last_island_refined_bodies()
        } else {
            self.live.as_ref().and_then(Coordinator::last_island_refined_bodies)
        }
    }

    pub fn pool_frame(&self) -> Option<&PoolFrame> {
        self.pool.as_ref().map(WorkerPool::frame)
    }

    /// Reject only a pooled velocity replacement that cannot become admissible
    /// before its next Bevy PhysicsPrepare. Every warm fragment keeps its native
    /// history across splitting, retirement of peers, handoff and recovery.
    /// A cold group may lose another member while paused, so its topology and
    /// the eventual COM twist remain the application-time setter's decision.
    pub fn validate_deferred_velocity_target(&self, id: BodyId) -> Result<(), PhysicsError> {
        let Some(frame) = self.pool_frame() else {
            return Ok(());
        };
        let (owner, body) = frame.body(id.raw()).ok_or(PhysicsError::MissingBody)?;
        let component = frame.workers[usize::from(owner)]
            .components
            .iter()
            .find(|group| group.ids.binary_search(&id.raw()).is_ok())
            .ok_or(PhysicsError::MissingBody)?;
        if component.contact_history {
            component.admit_free_edit(body.tier)?;
        }
        Ok(())
    }

    pub fn local_worker_pids(&self) -> Option<Vec<u32>> {
        if let Some(pool) = &self.pool {
            return (0..self.width).map(|w| pool.worker_pid(w)).collect();
        }
        self.worker_pids().map(|p| p.to_vec())
    }

    fn start(
        &self,
        bodies: &Bodies,
        plane: Option<ground::SceneGround>,
        pending: &[PendingImpulse],
        wrenches: &[BodyWrench],
    ) -> Result<Started, PhysicsError> {
        if self.periodic_placement.is_some() && (self.width < 3 || self.placement == InitialPlacement::CanonicalReference) {
            return Err(unsupported("periodic placement requires an ordinary three- to eight-worker pool"));
        }
        let run = uuid::Uuid::new_v4().as_u128() as u64;
        let ground = plane.map(|p| p.surface);
        let width = usize::from(self.width);
        // Every candidate still passes the ordinary worker seed admission and
        // coordinator routing checks. A failed forecast uses the original split.
        let prepare_seeds = || {
            if self.placement == InitialPlacement::ContactAware {
                let snapshot = Snapshot {
                    bodies: bodies.values().cloned().collect(),
                    step: self.step,
                    ground,
                    ..Snapshot::default()
                };
                placement::contact_groups(
                    &snapshot,
                    width,
                    self.refinement,
                    self.native_profile,
                    self.gravity,
                    pending,
                    wrenches,
                )
            } else {
                None
            }
            .unwrap_or_else(|| placement::spatial(bodies, width))
        };
        if self.width > 2 {
            let mut pool = if self.placement == InitialPlacement::CanonicalReference {
                WorkerPool::spawn_canonical_scene_with_friction(
                    &self.executable,
                    run,
                    bodies.values().cloned().collect(),
                    self.width,
                    self.step,
                    ground,
                    JOURNAL_LIMIT,
                    self.native_profile.friction(),
                )
            } else {
                WorkerPool::spawn_with_profile(
                    &self.executable,
                    run,
                    prepare_seeds(),
                    self.step,
                    ground,
                    JOURNAL_LIMIT,
                    self.native_profile,
                )
            }
            .map_err(native)?;
            pool.enable_scene_supervision();
            pool.set_packed_observations(self.packed_observations);
            pool.set_packed_handoffs(self.packed_handoffs);
            pool.set_local_helper_limit(self.local_helper_limit);
            pool.set_periodic_placement(self.periodic_placement).map_err(native)?;
            pool.validate_next(self.gravity).map_err(native)?;
            return Ok(Started::Pool(Box::new(pool)));
        }
        let seeds = prepare_seeds();
        let mut peers = [
            WorkerProcess::spawn_with_ground(&self.executable, run, 0, &seeds[0], self.step, ground)
                .map_err(native)?,
            WorkerProcess::spawn_with_ground(&self.executable, run, 1, &seeds[1], self.step, ground)
                .map_err(native)?,
        ];
        for peer in &mut peers {
            peer.set_packed_observations(self.packed_observations);
            peer.set_packed_handoffs(self.packed_handoffs);
        }
        let mut live = Coordinator::from_seeds(run, peers, JOURNAL_LIMIT).map_err(native)?;
        live.field_routes(self.gravity, &[]).map_err(native)?;
        live.enable_rolling_journals();
        Ok(Started::Pair(Box::new(live)))
    }

    /// Spawn and verify the selected workers without advancing physics. Safe to call
    /// again. Scene loaders use this before returning a constructed world.
    pub fn initialize(&mut self) -> Result<(), PhysicsError> {
        if self.live.is_some() || self.pool.is_some() {
            return Ok(());
        }
        let mut live = match self.start(&self.loading, self.ground, &self.before_start, &self.wrenches)? {
            Started::Pair(pair) => *pair,
            Started::Pool(mut pool) => {
                for input in &self.before_start {
                    pool.queue_impulse_in_field(
                        input.id,
                        input.impulse,
                        self.gravity,
                        &self.wrenches,
                    )
                    .map_err(native)?;
                }
                pool.validate_field(self.gravity, &self.wrenches)
                    .map_err(native)?;
                self.pool = Some(*pool);
                self.loading.clear();
                self.before_start.clear();
                return Ok(());
            }
        };
        let forced = live
            .force_motion(self.gravity, &self.wrenches)
            .map_err(native)?;
        for p in &self.before_start {
            live.supervised(|c| {
                c.queue_impulse_routed_forced(p.id, p.impulse, &forced)
                    .map(|_| ())
            })
            .map_err(native)?;
        }
        live.field_routes(self.gravity, &self.wrenches)
            .map_err(native)?;
        self.live = Some(live);
        self.loading.clear();
        self.before_start.clear();
        Ok(())
    }

    /// Choose lossless packed or JSON replies for live and subsequently reset workers.
    pub fn set_packed_observations(&mut self, enabled: bool) {
        self.packed_observations = enabled;
        if let Some(pool) = &mut self.pool {
            pool.set_packed_observations(enabled);
        }
        if let Some(c) = &mut self.live {
            c.set_packed_observations(enabled);
        }
    }
    /// Select negotiated native handoff payloads on live and subsequently created workers.
    pub fn set_packed_handoffs(&mut self, enabled: bool) {
        self.packed_handoffs = enabled;
        if let Some(pool) = &mut self.pool {
            pool.set_packed_handoffs(enabled);
        }
        if let Some(pair) = &mut self.live {
            pair.set_packed_handoffs(enabled);
        }
    }

    /// Number of current connections using the negotiated native handoff encoding.
    pub fn packed_handoff_workers(&self) -> Option<usize> {
        self.pool.as_ref().map(WorkerPool::packed_handoff_workers)
            .or_else(||self.live.as_ref().map(Coordinator::packed_handoff_workers))
    }

    /// Fault-injection hook for a locally owned worker. Its next sequenced command
    /// runs normally and the reply is withheld. Requires an initialized scene.
    pub fn drop_next_acknowledgement(&mut self, worker: u8) -> Result<(), PhysicsError> {
        if let Some(pool) = &mut self.pool {
            return pool.drop_next_acknowledgement(worker).map_err(native);
        }
        self.live.as_mut().ok_or_else(|| unsupported("worker is not initialized"))?
            .drop_next_acknowledgement(worker).map_err(native)
    }
    /// Cumulative serialized application traffic at each current worker, including restarts.
    pub fn worker_traffic(&self) -> Option<[crate::transport::Traffic; 2]> {
        self.live.as_ref().map(Coordinator::traffic)
    }

    pub fn local_worker_traffic(&self) -> Option<Vec<crate::transport::Traffic>> {
        if let Some(pool) = &self.pool {
            return Some(pool.traffic());
        }
        self.worker_traffic().map(|p| p.to_vec())
    }

    pub fn frame(&self) -> Option<&Frame> {
        self.live.as_ref().map(Coordinator::frame)
    }

    /// Local worker process IDs in partition order, available after initialization.
    pub fn worker_pids(&self) -> Option<[u32; 2]> {
        self.live.as_ref().map(Coordinator::worker_pids)
    }

    fn current(&self, id: BodyId) -> Option<&Body> {
        if let Some(pool) = &self.pool {
            return pool.frame().body(id.raw()).map(|(_, b)| b);
        }
        if let Some(c) = &self.live {
            c.frame().body(id.raw()).map(|(_, b)| b)
        } else {
            self.loading.get(&id)
        }
    }

    /// Forecast contact between two distinct boxes from their committed world poses
    /// and twists during the next configured interval. This read does not advance workers.
    pub fn predicts_box_contact(&self, a: BodyId, b: BodyId) -> Result<bool, PhysicsError> {
        if a == b {
            return Err(PhysicsError::InvalidValue);
        }
        let lookup = |id| {
            if self.is_shared_ground(id) {
                return Err(unsupported("box contact forecast requires two cuboids"));
            }
            self.current(id).ok_or(PhysicsError::MissingBody)
        };
        let left = lookup(a)?;
        let right = lookup(b)?;
        left.predicts_box_contact(right, self.step)
    }

    fn editable(&self, id: BodyId) -> Result<&Body, PhysicsError> {
        if self.is_shared_ground(id) {
            return Err(unsupported("immutable shared ground"));
        }
        self.current(id).ok_or(PhysicsError::MissingBody)
    }

    /// Spawn a centered finite box with its immutable surface coefficient.
    pub fn spawn_box_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
        tier: Tier,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        if !matches!(spec.shape, Shape::Box { .. }) {
            return Err(unsupported("box shape required"));
        }
        self.spawn_primitive_link(spec, rotation, inertia, tier, friction)
    }

    /// Spawn a centered uniform-density primitive with immutable Coulomb friction.
    pub fn spawn_primitive_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
        tier: Tier,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        let limit = if self.placement == InitialPlacement::CanonicalReference {
            self.native_profile.max_bodies()
        } else {
            MAX_PARTITION_BODIES
        };
        if self.initial.len() >= limit {
            return Err(unsupported("distributed scene body limit"));
        }
        let (primitive, size) = Primitive::from_shape(spec.shape)?;
        let (tier, mass) = match (tier, spec.mass) {
            (Tier::Dynamic, None) => (Tier::Fixed, 0.),
            (_, Some(mass)) => (tier, mass),
            _ => return Err(PhysicsError::InvalidValue),
        };
        if let Some(i) = inertia {
            let [x, y, z] = size.map(|s| s * 0.5);
            let diagonal = match primitive {
                Primitive::Box => [
                    mass * (y * y + z * z) / 3.,
                    mass * (x * x + z * z) / 3.,
                    mass * (x * x + y * y) / 3.,
                ],
                Primitive::Sphere => [0.4 * mass * x * x; 3],
                Primitive::Cylinder => {
                    let transverse=mass*(3.*x*x+4.*z*z)/12.;
                    [transverse,transverse,mass*x*x*0.5]
                }
                Primitive::Capsule => {
                    // A cylinder and two solid hemispheres of equal density.
                    let caps_mass = mass * (2. * x / (3. * z + 2. * x));
                    let cylinder_mass = mass - caps_mass;
                    let transverse = cylinder_mass * (3. * x * x + 4. * z * z) / 12.
                        + caps_mass * (0.4 * x * x + z * z + 0.75 * z * x);
                    let axial = cylinder_mass * x * x * 0.5 + caps_mass * x * x * 0.4;
                    [transverse, transverse, axial]
                }
            };
            if i.center != [0.; 3]
                || i.orientation[..3] != [0.; 3]
                || i.orientation[3].abs() != 1.
                || i.tensor[3..] != [0.; 3]
                || i.tensor[..3].iter().zip(diagonal).any(|(&v, d)| {
                    !v.is_finite()
                        || v <= 0.
                        || (tier != Tier::Fixed
                            && (!(d > 0. && d.is_finite())
                                || (v - d).abs() > 32. * f64::EPSILON * d))
                })
            {
                return Err(unsupported(
                    "distributed primitives require centered homogeneous inertia",
                ));
            }
        }
        let id = BodyId::from_raw(self.next);
        let next = self.next.checked_add(1).ok_or(PhysicsError::IdOverflow)?;
        let b = Body {
            id: id.raw(),
            primitive,
            tier,
            friction,
            size,
            mass,
            position: spec.position,
            orientation: rotation.orientation,
            linear: spec.velocity,
            angular: rotation.angular_velocity,
        };
        if let Some(pool) = &mut self.pool {
            supervised_pool_edit(pool, |p| {
                p.insert_body_in_field(b.clone(), self.gravity, &self.wrenches).map(|_| ())
            })?;
        } else if let Some(live) = &mut self.live {
            live.supervised(|c| {
                c.insert_checked(b.clone(), Some((self.gravity, &self.wrenches)))
                    .map(|_| ())
            })
            .map_err(native)?;
        } else {
            Island::restore_with_profile(&Snapshot {
                step: self.step,
                ground: self.ground.map(|p| p.surface),
                bodies: vec![b.clone()],
                ..Default::default()
            }, self.native_profile)
            .map_err(native)?;
            let incoming = b.bounds()?;
            for resident in self.loading.values() {
                if incoming.predicts_overlap(&resident.bounds()?) {
                    return Err(PhysicsError::TransitionDeferred {
                        reason: "distributed boxes start inside contact prediction bounds",
                    });
                }
            }
            self.loading.insert(id, b.clone());
        }
        self.next = next;
        self.initial.insert(id, b);
        Ok(id)
    }

    /// Check a new contribution against the ordered world-force accumulator.
    pub fn validate_world_wrench(
        &self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if self.is_shared_ground(id) {
            return Err(unsupported("immutable shared ground"));
        }
        let b = self.current(id).ok_or(PhysicsError::MissingBody)?;
        if self.wrenches.len() >= crate::physics::MAX_PENDING_IMPULSES {
            return Err(unsupported(
                "more than 32 world wrenches per distributed step",
            ));
        }
        let mut inputs = self.wrenches.clone();
        inputs.push(BodyWrench {
            id: id.raw(),
            force,
            torque,
        });
        if let Some(pool) = &self.pool {
            pool.validate_field(self.gravity, &inputs).map_err(native)?;
        } else if let Some(c) = &self.live {
            c.field_routes(self.gravity, &inputs).map_err(native)?;
        } else {
            inputs.retain(|w| w.id == id.raw());
            validate_wrenches(self.step, std::slice::from_ref(b), &inputs).map_err(native)?;
        }
        Ok(())
    }

    /// Queue a COM impulse for the next advancing native solve.
    pub fn queue_linear_impulse(&mut self, id: BodyId, j: [f64; 3]) -> Result<(), PhysicsError> {
        if let Some(pool) = &mut self.pool {
            let first = pool
                .queue_impulse_in_field(id.raw(), j, self.gravity, &self.wrenches)
                .map(|_| ());
            return match first {
                Err(crate::coordinator::Error::Transport(_)) => pool
                    .queue_impulse_in_field(id.raw(), j, self.gravity, &self.wrenches)
                    .map(|_| ())
                    .map_err(native),
                other => other.map_err(native),
            };
        }
        if self.current(id).is_none() {
            return Err(PhysicsError::MissingBody);
        }
        if let Some(c) = &mut self.live {
            let forced = c
                .force_motion(self.gravity, &self.wrenches)
                .map_err(native)?;
            c.supervised(|c| {
                c.queue_impulse_routed_forced(id.raw(), j, &forced)
                    .map(|_| ())
            })
            .map_err(native)?;
        } else {
            let bodies: Vec<_> = self.loading.values().cloned().collect();
            validate_impulse(&bodies, &self.before_start, id.raw(), j).map_err(native)?;
            self.before_start.push(PendingImpulse {
                id: id.raw(),
                impulse: j,
            });
        }
        Ok(())
    }
}

impl DynamicBackend for SceneBackend {
    fn engine(&self) -> &'static str {
        if self.width > 2 {
            "distributed-pool"
        } else {
            ENGINE
        }
    }
    fn spawn_link(
        &mut self,
        s: BodySpec,
        r: RotationSpec,
        i: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive_link(s, r, i, Tier::Dynamic, 0.)
    }
    fn spawn_kinematic_link(
        &mut self,
        s: BodySpec,
        r: RotationSpec,
        i: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive_link(s, r, i, Tier::Prescribed, 0.)
    }
    fn validate_kinematic_velocity(
        &self,
        v: [f64; 3],
        w: [f64; 3],
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        MultiFidelityBackend::new([0.; 3])?.validate_kinematic_velocity(v, w, dt)
    }
    fn spawn_static_plane(&mut self, spec: PlaneSpec) -> Result<BodyId, PhysicsError> {
        self.spawn_ground_surface(spec, 0.)
    }
    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        if self.is_shared_ground(id) {
            return Some(false);
        }
        self.current(id).map(|b| b.tier == Tier::Dynamic)
    }
    fn is_kinematic(&self, id: BodyId) -> Option<bool> {
        if self.is_shared_ground(id) {
            return Some(false);
        }
        self.current(id).map(|b| b.tier == Tier::Prescribed)
    }
    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        if self.is_shared_ground(id) {
            return Some(false);
        }
        self.current(id).map(|_| false)
    }
    fn state(&self, id: BodyId) -> Option<BodyState> {
        if let Some(plane) = self.ground.filter(|_| id == ground::handle()) {
            return Some(BodyState {
                position: plane.spec.position,
                orientation: plane.spec.orientation,
                velocity: [0.; 3],
                angular_velocity: [0.; 3],
            });
        }
        self.current(id).map(Body::state)
    }
    fn validate_body_state(&self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        let old = self.editable(id)?;
        if let Some(pool) = &self.pool {
            return pool.validate_state_in_field(id.raw(), state, self.gravity, &self.wrenches)
                .map_err(pool_edit_error);
        }
        let body = old.with_state(state);
        if let Some(c) = &self.live {
            c.validate_replacement(&body).map_err(native)?;
            c.validate_edited_body(&body, self.gravity, &self.wrenches)
                .map_err(native)
        } else {
            crate::physics::admit_edit(
                old,
                &body,
                self.loading.values(),
                &self.before_start,
                self.step.duration(),
                self.ground.map(|p| p.surface),
            )
            .map_err(native)
        }
    }
    fn set_body_state(&mut self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        self.editable(id)?;
        if let Some(pool) = &mut self.pool {
            return supervised_pool_edit(pool, |p| {
                p.replace_state_in_field(id.raw(), state, self.gravity, &self.wrenches).map(|_| ())
            });
        }
        self.validate_body_state(id, state)?;
        if let Some(c) = &mut self.live {
            c.supervised(|c| c.replace_state(id.raw(), state).map(|_| ()))
                .map_err(native)?;
        } else {
            let b = self.loading.get_mut(&id).unwrap();
            *b = b.with_state(state);
        }
        Ok(())
    }
    fn cast_ray(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        limit: f64,
        exclude: Option<BodyId>,
    ) -> Result<Option<f64>, PhysicsError> {
        ray::cast(self, origin, direction, limit, exclude)
    }
    fn remove(&mut self, id: BodyId) -> bool {
        if !self.initial.contains_key(&id) {
            return false;
        }
        if let Some(pool) = &mut self.pool {
            if supervised_pool_edit(pool, |p| p.retire(id.raw()).map(|_| ())).is_err() {
                return false;
            }
        } else if let Some(c) = &mut self.live {
            if c.supervised(|c| c.retire(id.raw()).map(|_| ())).is_err() {
                return false;
            }
        } else {
            self.loading.remove(&id);
            self.before_start.retain(|p| p.id != id.raw());
        }
        self.wrenches.retain(|w| w.id != id.raw());
        self.initial.remove(&id);
        true
    }
    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let b = self.editable(id)?;
        if linear.into_iter().chain(angular).any(|v| !v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        if b.tier == Tier::Fixed {
            return Err(unsupported("velocity command on a fixed body"));
        }
        if b.tier == Tier::Prescribed {
            self.validate_kinematic_velocity(linear, angular, self.step.duration())?;
        }
        let mut projected = b.clone();
        projected.linear = linear;
        projected.angular = angular;
        if let Some(pool) = &mut self.pool {
            return supervised_pool_edit(pool, |p| {
                p.set_velocity_in_field(id.raw(), linear, angular, self.gravity, &self.wrenches).map(|_| ())
            });
        }
        if let Some(c) = &mut self.live {
            c.validate_edited_body(&projected, self.gravity, &self.wrenches)
                .map_err(native)?;
            c.supervised(|c| c.set_velocity(id.raw(), linear, angular).map(|_| ()))
                .map_err(native)?;
        } else {
            let current = self.loading.get_mut(&id).unwrap();
            crate::physics::validate_queued_for(&projected, &self.before_start).map_err(native)?;
            *current = projected;
        }
        Ok(())
    }
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        self.validate_world_wrench(id, force, torque)?;
        self.wrenches.push(BodyWrench {
            id: id.raw(),
            force,
            torque,
        });
        Ok(())
    }
    fn apply_impulse(
        &mut self,
        id: BodyId,
        j: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if self.is_shared_ground(id) {
            return Err(unsupported("immutable shared ground"));
        }
        let body = self.current(id).ok_or(PhysicsError::MissingBody)?;
        if body.tier != Tier::Dynamic {
            return Err(unsupported("spatial impulse on a nondynamic body"));
        }
        let body = body.clone();
        if let Some(pool) = &mut self.pool {
            supervised_pool_edit(pool, |p| {
                p.apply_impulse_in_field(id.raw(), j, angular, self.gravity, &self.wrenches)
                    .map(|_| ())
            })?;
        } else if let Some(live) = &mut self.live {
            live.supervised(|c| {
                c.impulse_checked(id.raw(), j, angular, Some((self.gravity, &self.wrenches)))
                    .map(|_| ())
            })
            .map_err(native)?;
        } else {
            let mut solver = Island::restore_with_profile(&Snapshot {
                step: self.step,
                ground: self.ground.map(|p| p.surface),
                bodies: vec![body],
                ..Default::default()
            }, self.native_profile)
            .map_err(native)?;
            solver
                .impulse_now(id.raw(), j, angular, &self.before_start)
                .map_err(native)?;
            let updated = solver.snapshot().bodies.remove(0);
            let b = self.loading.get_mut(&id).unwrap();
            b.linear = updated.linear;
            b.angular = updated.angular;
        }
        Ok(())
    }
    fn add_world_revolute(
        &mut self,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(unsupported("distributed joints"))
    }
    fn add_revolute(
        &mut self,
        _: BodyId,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(unsupported("distributed joints"))
    }
    fn add_reduced_revolute(
        &mut self,
        _: Option<BodyId>,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(unsupported("distributed joints"))
    }
    fn set_self_collision_group(
        &mut self,
        id: BodyId,
        group: Option<u64>,
    ) -> Result<(), PhysicsError> {
        if self.current(id).is_none() {
            return Err(PhysicsError::MissingBody);
        }
        if group.is_some() {
            return Err(unsupported("distributed self-collision groups"));
        }
        Ok(())
    }
    fn add_joint_effort(&mut self, _: JointId, _: f64) -> Result<(), PhysicsError> {
        Err(unsupported("distributed joints"))
    }
    fn joint_child(&self, _: JointId) -> Option<BodyId> {
        None
    }
    fn joint_state(&self, _: JointId) -> Option<JointState> {
        None
    }
    fn remove_joint(&mut self, _: JointId) -> bool {
        false
    }
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        if dt != self.step.duration() {
            return Err(unsupported(
                "distributed interval differs from the configured public step",
            ));
        }
        self.initialize()?;
        let gravity = self.gravity;
        let inputs = &self.wrenches;
        if let Some(pool) = &mut self.pool {
            let replay = self.scoped_placement.as_ref().and_then(std::sync::Weak::upgrade);
            let advance = |pool: &mut WorkerPool| {
                match self.refinement {
                    Refinement::Fixed => pool.advance_in_field(gravity, inputs),
                    Refinement::Scene => pool.advance_adaptive_in_field(gravity, inputs),
                    Refinement::Island => match &replay {
                        Some(record) => pool.advance_islands_with_placement(gravity, inputs, record),
                        None => pool.advance_islands_in_field(gravity, inputs),
                    },
                }.map(|_| ())
            };
            match advance(pool) {
                Err(crate::coordinator::Error::Transport(_)) => advance(pool).map_err(native),
                result => result.map_err(native),
            }?;
        } else {
            let pair = self.live.as_mut().expect("initialized pair");
            pair.supervised(|pair| {
                match self.refinement {
                    Refinement::Fixed => pair.advance_in_field_overlapped(gravity, inputs),
                    Refinement::Scene => pair.advance_adaptive_in_field(gravity, inputs),
                    Refinement::Island => pair.advance_islands_in_field(gravity, inputs),
                }.map(|_| ())
            }).map_err(native)?;
        }
        self.wrenches.clear();
        Ok(())
    }
    fn reset(&mut self) -> Result<(), PhysicsError> {
        let prepared = self.prepare_initial(&self.initial_state())?;
        self.apply_initial(prepared)
    }
    fn initial_state(&self) -> InitialPhysicsState {
        InitialPhysicsState::new(&self.identity, (self.initial.clone(), self.ground))
    }
    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        initial.data::<SceneSeed>(&self.identity).is_ok()
    }
    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        let (bodies, ground) = initial.data::<SceneSeed>(&self.identity)?;
        let live = self.start(bodies, *ground, &[], &[])?;
        Ok(PreparedPhysicsReset::new(
            &self.identity,
            Start {
                bodies: bodies.clone(),
                ground: *ground,
                live,
            },
        ))
    }
    fn apply_initial(&mut self, p: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        let start = p.into_data::<Start>(&self.identity)?;
        self.wrenches.clear();
        self.initial = start.bodies;
        self.ground = start.ground;
        match start.live {
            Started::Pair(pair) => {
                self.live = Some(*pair);
                self.pool = None;
            }
            Started::Pool(pool) => {
                self.pool = Some(*pool);
                self.live = None;
            }
        }
        self.before_start.clear();
        self.loading.clear();
        Ok(())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
