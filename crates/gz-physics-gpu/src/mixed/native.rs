// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Guarded exchange of complete, independent sphere contacts with pinned f64 Rapier.
use super::*;
use rapier3d_f64::prelude::{
    ColliderBuilder, ColliderHandle, IntegrationParameters, MassProperties, PhysicsWorld, Pose,
    RigidBodyBuilder, RigidBodyHandle, Rotation, SharedShape, Vector,
};
use std::collections::{BTreeMap, BTreeSet};
pub const MAX_NATIVE_PAIRS: usize = 128;
pub const MAX_NATIVE_WORKERS: usize = 4;
const SUBSTEPS: usize = 16;
const H: f64 = 0.001 / SUBSTEPS as f64;
// Match the independent fine-step Rapier reference and the documented SI resolution.
const PREDICTION: f64 = 1e-6;
const ALLOWED_ERROR: f64 = 1e-7;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairSolver {
    NativeRapier,
    SpecializedGpu,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum NativeHandoffRefusal {
    #[error("use the guarded native pair solver selection API")]
    GuardRequired,
    #[error("only marked independent sphere-pair or sphere-plane environments are eligible")]
    Unregistered,
    #[error("the native pair and its collider contact history must be fully separated")]
    ContactHistory,
    #[error("next-step swept surface clearance is inside the native prediction margin")]
    SweptClearance,
    #[error(
        "sphere coordinates, speed, spin or inertia exceed the resolved native substep envelope"
    )]
    MotionEnvelope,
    #[error("native body cannot exactly represent the transferred frame or inertia")]
    NativeRepresentation,
    #[error("native registration requires a fresh fixed scene and at most 128 marked pairs")]
    Registry,
}
/// A numerical contact impulse sampled at the native substep cadence. The
/// interval identifies the first solver substep carrying an impulse; a
/// continuous physical time of impact is unavailable from this discrete solver.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NativeContact {
    pub first_impulse_interval: Option<[f64; 2]>,
    /// Normal component on sphere B for a free pair or on the supported sphere A.
    pub normal_impulse_vector: [f64; 3],
    /// Impulse on the free pair's first sphere, or the sphere in a sphere-plane contact.
    pub impulse_on_a: [f64; 3],
    /// Equal and opposite impulse on the second sphere or the fixed plane.
    pub impulse_on_b: [f64; 3],
    pub retained_points: usize,
    pub active_solver_contact: bool,
    pub warmstart_normal_impulse: f64,
    /// Sum of magnitudes of world-frame tangent impulses retained by Rapier for warmstarting.
    pub warmstart_tangent_impulse: f64,
}
fn refused(id: u64, reason: NativeHandoffRefusal) -> GpuError {
    GpuError::NativeHandoff { id, reason }
}
fn plane_sphere(s: SpherePlaneLane) -> SolidSphere {
    SolidSphere {
        position: s.position,
        velocity: s.velocity,
        angular_velocity: s.angular_velocity,
        orientation: s.orientation,
        radius: s.radius,
        inverse_mass: s.inverse_mass,
    }
}
/// Recover a Rapier halfspace anchor that reproduces the registered plane
/// equation in f64. The representation stores only the signed offset.
fn plane_anchor(id: u64, p: SpherePlaneLane) -> Result<Vector, GpuError> {
    let n = Vector::from(p.plane_normal);
    if (n.length_squared() - 1.).abs() > 64. * f64::EPSILON {
        return Err(refused(id, NativeHandoffRefusal::NativeRepresentation));
    }
    let candidate = n * p.plane_offset;
    if n.dot(candidate) == p.plane_offset {
        return Ok(candidate);
    }
    for axis in 0..3 {
        if n[axis] == 0. {
            continue;
        }
        let mut point = Vector::ZERO;
        point[axis] = p.plane_offset / n[axis];
        if point.is_finite() && n.dot(point) == p.plane_offset {
            return Ok(point);
        }
    }
    Err(refused(id, NativeHandoffRefusal::NativeRepresentation))
}
fn resolution(s: EnvironmentState) -> f64 {
    let scale = match s {
        EnvironmentState::SpherePair(p) => Vector::from(p.a.position)
            .abs()
            .max_element()
            .max(Vector::from(p.b.position).abs().max_element()),
        EnvironmentState::SpherePlane(p) => Vector::from(p.position)
            .abs()
            .max_element()
            .max(p.plane_offset.abs()),
        _ => 0.,
    };
    64. * f64::EPSILON * scale.max(1.)
}
fn resolved_body(b: SolidSphere, gravity: f64, tol: f64) -> bool {
    let v = Vector::from(b.velocity).length();
    let w = Vector::from(b.angular_velocity).length();
    let mass = b.inverse_mass.recip();
    let moment = 0.4 * mass * b.radius * b.radius;
    b.radius > ALLOWED_ERROR + tol
        && b.inverse_mass > 0.
        && mass.is_finite()
        && moment.is_finite()
        && moment > 0.
        && v + gravity * STEP.as_secs_f64() < IntegrationParameters::default().max_linear_velocity()
        && (v + gravity * STEP.as_secs_f64()) * H <= 0.125 * b.radius
        && w * H <= 0.125
}
pub(super) fn motion_envelope(id: u64, state: EnvironmentState) -> Result<(), GpuError> {
    let tol = resolution(state);
    let resolved = match state {
        EnvironmentState::SpherePair(p) => [p.a, p.b]
            .into_iter()
            .all(|b| resolved_body(b, Vector::from(p.gravity).length(), tol)),
        EnvironmentState::SpherePlane(p) => {
            resolved_body(plane_sphere(p), Vector::from(p.gravity).length(), tol)
        }
        _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
    };
    if resolved && state.values().iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(refused(id, NativeHandoffRefusal::MotionEnvelope))
    }
}
fn guard(
    id: u64,
    state: EnvironmentState,
    candidate: EnvironmentState,
    history: Option<&NativePair>,
) -> Result<(), GpuError> {
    let contact = match state {
        EnvironmentState::SpherePair(p) => {
            p.normal_impulse != 0. || p.tangential_impulse != [0.; 3] || p.impact_time >= 0.
        }
        EnvironmentState::SpherePlane(p) => {
            p.normal_impulse != 0. || p.tangential_impulse != [0.; 3]
        }
        _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
    };
    if contact
        || history.is_some_and(|n| {
            n.contact.retained_points > 0
                || n.contact.active_solver_contact
                || n.contact.first_impulse_interval.is_some()
        })
    {
        return Err(refused(id, NativeHandoffRefusal::ContactHistory));
    }
    motion_envelope(id, state)?;
    motion_envelope(id, candidate)?;
    let h = STEP.as_secs_f64();
    let margin = match (state, candidate) {
        (EnvironmentState::SpherePair(p), EnvironmentState::SpherePair(v)) => {
            let r = Vector::from(p.b.position) - Vector::from(p.a.position);
            let u = Vector::from(v.b.velocity) - Vector::from(v.a.velocity);
            let u2 = u.length_squared();
            let t = if u2 > 0. {
                (-r.dot(u) / u2).clamp(0., h)
            } else {
                0.
            };
            (r + u * t).length() - p.a.radius - p.b.radius
        }
        (EnvironmentState::SpherePlane(p), EnvironmentState::SpherePlane(v)) => {
            let n = Vector::from(p.plane_normal);
            plane_anchor(id, p)?;
            let start = n.dot(Vector::from(p.position)) - p.plane_offset - p.radius;
            // A conservative lower bound across both integrators for the entire interval.
            // Separate minima account for a descending sphere under outward acceleration.
            start
                + (h * n.dot(Vector::from(v.velocity))).min(0.)
                + (h * h * n.dot(Vector::from(p.gravity))).min(0.)
        }
        _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
    };
    if !margin.is_finite() || margin <= PREDICTION + resolution(state) {
        return Err(refused(id, NativeHandoffRefusal::SweptClearance));
    }
    if !state.current_valid() || !candidate.current_valid() {
        return Err(refused(id, NativeHandoffRefusal::NativeRepresentation));
    }
    Ok(())
}
pub(super) struct NativePair {
    world: PhysicsWorld,
    handles: [RigidBodyHandle; 2],
    colliders: [ColliderHandle; 2],
    contact: NativeContact,
}
impl NativePair {
    fn new(id: u64, state: EnvironmentState) -> Result<Self, GpuError> {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.dt = H;
        world.integration_parameters.normalized_allowed_linear_error = ALLOWED_ERROR;
        world.integration_parameters.normalized_prediction_distance = PREDICTION;
        world.integration_parameters.contact_recycling = false;
        let handles = match state {
            EnvironmentState::SpherePair(p) => {
                world.gravity = p.gravity.into();
                [
                    Self::sphere(&mut world, id, p.a, p.friction, p.restitution)?,
                    Self::sphere(&mut world, id, p.b, p.friction, p.restitution)?,
                ]
            }
            EnvironmentState::SpherePlane(p) => {
                world.gravity = p.gravity.into();
                let n = Vector::from(p.plane_normal);
                let fixed = world.insert(
                    RigidBodyBuilder::fixed().translation(plane_anchor(id, p)?),
                    ColliderBuilder::new(SharedShape::halfspace(n))
                        .friction(p.friction)
                        .restitution(0.),
                );
                [
                    fixed,
                    Self::sphere(&mut world, id, plane_sphere(p), p.friction, 0.)?,
                ]
            }
            _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
        };
        Ok(Self {
            world,
            handles: handles.map(|h| h.0),
            colliders: handles.map(|h| h.1),
            contact: NativeContact::default(),
        })
    }
    fn sphere(
        world: &mut PhysicsWorld,
        id: u64,
        b: SolidSphere,
        friction: f64,
        restitution: f64,
    ) -> Result<(RigidBodyHandle, ColliderHandle), GpuError> {
        let inv_i = 2.5 * b.inverse_mass / (b.radius * b.radius);
        let handles = world.insert(
            RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .pose(Pose::from_parts(
                    Vector::from(b.position),
                    Rotation::from_array(b.orientation),
                ))
                .linvel(Vector::from(b.velocity))
                .angvel(Vector::from(b.angular_velocity)),
            ColliderBuilder::ball(b.radius)
                .mass_properties(MassProperties {
                    local_com: Vector::ZERO,
                    inv_mass: b.inverse_mass,
                    inv_principal_inertia: Vector::splat(inv_i),
                    principal_inertia_local_frame: Rotation::IDENTITY,
                })
                .friction(friction)
                .restitution(restitution),
        );
        let rb = &mut world.bodies[handles.0];
        rb.recompute_mass_properties_from_colliders(&world.colliders);
        let mp = rb.mass_properties().local_mprops;
        if mp.local_com != Vector::ZERO
            || mp.inv_mass != b.inverse_mass
            || mp.inv_principal_inertia.to_array() != [inv_i; 3]
            || rb.translation().to_array().map(f64::to_bits) != b.position.map(f64::to_bits)
            || rb.rotation().to_array().map(f64::to_bits) != b.orientation.map(f64::to_bits)
            || rb.linvel().to_array().map(f64::to_bits) != b.velocity.map(f64::to_bits)
            || rb.angvel().to_array().map(f64::to_bits) != b.angular_velocity.map(f64::to_bits)
        {
            return Err(refused(id, NativeHandoffRefusal::NativeRepresentation));
        }
        Ok(handles)
    }
    /// All persistent state in this two-body world; pipelines and CCD are
    /// Rapier scratch workspaces. Joints, soft bodies and sleep are excluded
    /// by construction. Physical contacts and native warmstart data are cloned.
    fn fork(&self) -> Self {
        let w = &self.world;
        Self {
            world: PhysicsWorld {
                gravity: w.gravity,
                integration_parameters: w.integration_parameters,
                islands: w.islands.clone(),
                broad_phase: w.broad_phase.clone(),
                narrow_phase: w.narrow_phase.clone(),
                bodies: w.bodies.clone(),
                colliders: w.colliders.clone(),
                ..PhysicsWorld::default()
            },
            handles: self.handles,
            colliders: self.colliders,
            contact: self.contact,
        }
    }
    pub(super) fn advance(
        &self,
        id: u64,
        input: EnvironmentState,
    ) -> Result<(Self, EnvironmentState), GpuError> {
        let mut next = self.fork();
        let value = next.advance_in_place(id, input)?;
        Ok((next, value))
    }
    fn install_velocity(&mut self, slot: usize, velocity: [f64; 3]) {
        let body = &mut self.world.bodies[self.handles[slot]];
        if body.linvel().to_array().map(f64::to_bits) != velocity.map(f64::to_bits) {
            body.set_linvel(Vector::from(velocity), true);
        }
    }
    fn read_body(&self, slot: usize, b: &mut SolidSphere) {
        let rb = &self.world.bodies[self.handles[slot]];
        b.position = rb.translation().to_array();
        b.orientation = rb.rotation().to_array();
        b.velocity = rb.linvel().to_array();
        b.angular_velocity = rb.angvel().to_array();
    }
    fn advance_in_place(
        &mut self,
        id: u64,
        input: EnvironmentState,
    ) -> Result<EnvironmentState, GpuError> {
        motion_envelope(id, input)?;
        let (receiver_inverse_mass, plane_normal) = match input {
            EnvironmentState::SpherePair(p) => {
                self.install_velocity(0, p.a.velocity);
                self.install_velocity(1, p.b.velocity);
                (p.b.inverse_mass, None)
            }
            EnvironmentState::SpherePlane(p) => {
                self.install_velocity(1, p.velocity);
                (p.inverse_mass, Some(Vector::from(p.plane_normal)))
            }
            _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
        };
        let mut contact = NativeContact::default();
        let mut normal = Vector::ZERO;
        let mut impulse = Vector::ZERO;
        for k in 0..SUBSTEPS {
            let previous = self.world.bodies[self.handles[1]].linvel();
            self.world.step();
            if let Some(pair) = self
                .world
                .narrow_phase
                .contact_pair(self.colliders[0], self.colliders[1])
            {
                let mut n = pair.total_impulse();
                if n.length_squared() > 0. {
                    let delta = plane_normal.unwrap_or_else(|| {
                        self.world.bodies[self.handles[1]].translation()
                            - self.world.bodies[self.handles[0]].translation()
                    });
                    if n.dot(delta) < 0. {
                        n = -n;
                    }
                    normal += n;
                    let v = self.world.bodies[self.handles[1]].linvel();
                    impulse += (v - previous - H * self.world.gravity) / receiver_inverse_mass;
                    contact
                        .first_impulse_interval
                        .get_or_insert([k as f64 * H, (k + 1) as f64 * H]);
                }
            }
        }
        if let Some(pair) = self
            .world
            .narrow_phase
            .contact_pair(self.colliders[0], self.colliders[1])
        {
            contact.active_solver_contact = pair.has_any_active_contact();
            for m in pair.manifolds() {
                contact.retained_points += m.points.len();
                contact.warmstart_tangent_impulse += m
                    .points
                    .iter()
                    .map(|c| c.data.warmstart_tangent_world.length())
                    .sum::<f64>();
                contact.warmstart_normal_impulse += m
                    .points
                    .iter()
                    .map(|c| c.data.warmstart_impulse)
                    .sum::<f64>();
            }
        }
        contact.normal_impulse_vector = normal.to_array();
        let value = match input {
            EnvironmentState::SpherePair(mut p) => {
                contact.impulse_on_b = impulse.to_array();
                contact.impulse_on_a = (-impulse).to_array();
                self.read_body(0, &mut p.a);
                self.read_body(1, &mut p.b);
                p.normal_impulse = normal.length();
                p.contact_normal = if p.normal_impulse > 0. {
                    (normal / p.normal_impulse).to_array()
                } else {
                    [0.; 3]
                };
                p.tangential_impulse = (impulse - normal).to_array();
                p.impact_time = -1.;
                EnvironmentState::SpherePair(p)
            }
            EnvironmentState::SpherePlane(mut p) => {
                let mut b = plane_sphere(p);
                self.read_body(1, &mut b);
                p.position = b.position;
                p.orientation = b.orientation;
                p.velocity = b.velocity;
                p.angular_velocity = b.angular_velocity;
                contact.impulse_on_a = impulse.to_array();
                contact.impulse_on_b = (-impulse).to_array();
                p.normal_impulse = normal.dot(Vector::from(p.plane_normal));
                p.tangential_impulse = (impulse - normal).to_array();
                EnvironmentState::SpherePlane(p)
            }
            _ => return Err(refused(id, NativeHandoffRefusal::Unregistered)),
        };
        motion_envelope(id, value)?;
        self.contact = contact;
        Ok(value)
    }
}
type StagedWorlds = BTreeMap<usize, Result<(NativePair, EnvironmentState), GpuError>>;
pub(super) struct Registry {
    pub(super) eligible: BTreeSet<usize>,
    pub(super) active: BTreeMap<usize, NativePair>,
    initial: BTreeMap<usize, NativePair>,
    pub(super) overlap_gpu: bool,
    pool: Option<bevy_tasks::TaskPool>,
}
impl Registry {
    /// Chunks borrow immutable committed worlds. Each job stages independent clones;
    /// errors and observations are consumed by stable index on the schedule thread.
    pub(super) fn stage_parallel(
        &self,
        committed: &[Environment],
        pending: &BTreeMap<usize, (CentralImpulse, EnvironmentState)>,
    ) -> Option<StagedWorlds> {
        let pool = self.pool.as_ref().filter(|_| self.active.len() > 1)?;
        let rows: Vec<_> = self.active.iter().collect();
        let workers = pool.thread_num() + 1;
        let chunk = rows.len().div_ceil(workers * 2);
        Some(
            pool.scope(|scope| {
                for rows in rows.chunks(chunk) {
                    scope.spawn(async move {
                        rows.iter()
                            .map(|&(&i, native)| {
                                let state = pending.get(&i).map_or(committed[i].state, |(_, p)| *p);
                                (i, native.advance(committed[i].id, state))
                            })
                            .collect::<Vec<_>>()
                    });
                }
            })
            .into_iter()
            .flatten()
            .collect(),
        )
    }
    pub(super) fn reset(&mut self) {
        self.active = self.initial.iter().map(|(&i, p)| (i, p.fork())).collect();
    }
}
impl Environment {
    /// Validate an authored solver before opening an adapter. A new sphere can begin
    /// at rest on its plane with a cold cache. Subsequent owner changes must clear
    /// the swept migration guard, including at tick zero.
    pub fn validate_native_pair(&self) -> Result<(), GpuError> {
        if let EnvironmentState::SpherePlane(p) = self.state {
            motion_envelope(self.id, self.state)?;
            plane_anchor(self.id, p)?;
            if !self.state.valid() {
                return Err(refused(self.id, NativeHandoffRefusal::NativeRepresentation));
            }
            return Ok(());
        }
        guard(self.id, self.state, self.state, None)
    }
}
impl MixedBatch {
    /// At construction, configure the maximum workers for the native CPU stage.
    /// The calling thread is included. The default serial path creates no threads.
    /// The persistent pool survives full resets and performs no work for GPU-only ticks.
    pub fn with_native_staging(self, workers: usize) -> Result<Self, GpuError> {
        self.configure_native_staging(workers, false)
    }
    /// Submit independent GPU work before staging native worlds. Both stages
    /// must finish successfully before the public frame advances.
    pub fn with_overlapped_native_staging(self, workers: usize) -> Result<Self, GpuError> {
        self.configure_native_staging(workers, true)
    }
    /// Whether early submission is enabled. Ticks without simultaneous native
    /// and GPU residents still use the ordinary single-domain path.
    pub fn overlaps_native_gpu(&self) -> bool {
        self.native.as_ref().is_some_and(|s| s.overlap_gpu)
    }
    fn configure_native_staging(
        mut self,
        workers: usize,
        overlap_gpu: bool,
    ) -> Result<Self, GpuError> {
        if !(1..=MAX_NATIVE_WORKERS).contains(&workers)
            || self.tick != 0
            || !self.pending.is_empty()
        {
            return Err(GpuError::NativeStaging);
        }
        let set = self.native.as_mut().ok_or(GpuError::NativeStaging)?;
        set.overlap_gpu = overlap_gpu;
        set.pool = if workers == 1 {
            None
        } else {
            Some(
                std::panic::catch_unwind(|| {
                    bevy_tasks::TaskPoolBuilder::new()
                        .num_threads(workers - 1)
                        .thread_name("gz-native-physics".into())
                        .build()
                })
                .map_err(|_| GpuError::NativeStagingUnavailable)?,
            )
        };
        Ok(self)
    }
    /// Includes the calling thread. Serial staging does not allocate a task pool.
    pub fn native_staging_workers(&self) -> usize {
        self.native
            .as_ref()
            .and_then(|s| s.pool.as_ref())
            .map_or(1, |p| p.thread_num() + 1)
    }
    pub fn has_guarded_native_pairs(&self) -> bool {
        self.native.is_some()
    }
    /// Register independent two-sphere or sphere/plane contacts in a fixed scene, including their
    /// authored full-reset owners. Other registered groups keep their ordinary solver.
    pub fn with_guarded_native_pairs(
        mut self,
        marked: &[(u64, PairSolver)],
    ) -> Result<Self, GpuError> {
        if marked.is_empty() {
            return Ok(self);
        }
        if !self.fixed_scene
            || self.native.is_some()
            || self.tick != 0
            || !self.pending.is_empty()
            || marked.len() > MAX_NATIVE_PAIRS
        {
            return Err(refused(marked[0].0, NativeHandoffRefusal::Registry));
        }
        let mut eligible = BTreeSet::new();
        let mut active = BTreeMap::new();
        let mut ids = vec![];
        for &(id, solver) in marked {
            let i = self.index(id)?;
            let state = self.committed[i].state;
            if !eligible.insert(i) {
                return Err(GpuError::DuplicateEnvironment(id));
            }
            if self.execution_placement(id)? != ExecutionPlacement::Gpu {
                return Err(refused(id, NativeHandoffRefusal::Registry));
            }
            self.committed[i].validate_native_pair()?;
            if solver == PairSolver::NativeRapier {
                active.insert(i, NativePair::new(id, state)?);
                ids.push(id);
            }
        }
        self.place(0, &ids, ExecutionPlacement::Cpu)?;
        self.initial_placement = self.placement.clone();
        let initial = active.iter().map(|(&i, p)| (i, p.fork())).collect();
        self.native = Some(Registry {
            eligible,
            active,
            initial,
            pool: None,
            overlap_gpu: false,
        });
        Ok(self)
    }
    fn registered_pair(&self, id: u64) -> Result<(usize, &Registry), GpuError> {
        let i = self.index(id)?;
        let set = self
            .native
            .as_ref()
            .filter(|s| s.eligible.contains(&i))
            .ok_or_else(|| refused(id, NativeHandoffRefusal::Unregistered))?;
        Ok((i, set))
    }
    pub fn pair_solver(&self, id: u64) -> Result<PairSolver, GpuError> {
        let (i, set) = self.registered_pair(id)?;
        Ok(if set.active.contains_key(&i) {
            PairSolver::NativeRapier
        } else {
            PairSolver::SpecializedGpu
        })
    }
    pub fn native_contact(&self, id: u64) -> Result<Option<NativeContact>, GpuError> {
        let (i, set) = self.registered_pair(id)?;
        Ok(set.active.get(&i).map(|p| p.contact))
    }
    /// Validate every member and pending command before transferring any owner.
    pub fn set_pair_solver(
        &mut self,
        tick: u64,
        ids: &[u64],
        target: PairSolver,
    ) -> Result<PlacementChange, GpuError> {
        if tick != self.tick {
            return Err(GpuError::StaleBoundary);
        }
        let mut seen = BTreeSet::new();
        let mut updates = vec![];
        let mut additions = vec![];
        let mut removals = vec![];
        for &id in ids {
            let (i, set) = self.registered_pair(id)?;
            let native = set.active.get(&i);
            if !seen.insert(i) {
                return Err(GpuError::DuplicateEnvironment(id));
            }
            if native.is_some() == (target == PairSolver::NativeRapier) {
                continue;
            }
            let state = self.committed[i].state;
            let candidate = self.pending.get(&i).map_or(state, |(_, s)| *s);
            guard(id, state, candidate, native)?;
            if target == PairSolver::NativeRapier {
                additions.push((i, NativePair::new(id, state)?));
            } else {
                removals.push(i);
            }
            updates.push(id);
        }
        let target_owner = match target {
            PairSolver::NativeRapier => ExecutionPlacement::Cpu,
            PairSolver::SpecializedGpu => ExecutionPlacement::Gpu,
        };
        let report = self.place(tick, &updates, target_owner)?;
        if let Some(set) = &mut self.native {
            for i in removals {
                set.active.remove(&i);
            }
            set.active.extend(additions);
        }
        Ok(report)
    }
}

#[cfg(test)]
#[path = "../../tests/private/native.rs"]
mod tests;
