// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Evaluate prescribed curves at native contact substeps on the same clock as the rigid scene.
#[cfg(feature = "partition-checkpoint")]
mod checkpoint;
#[cfg(feature = "partition-checkpoint")]
pub use checkpoint::TRAJECTORY_IMAGE_LIMIT;
mod layout;
use super::*;
use gz_physics::trajectory::{TranslationSample, TranslationSpline};
const MAX_SLICE_NS: u128 = 500_000;
const MAX_INTERVALS: usize = 16_384;
#[derive(Clone)]
struct Track {
    handle: RigidBodyHandle,
    path: TranslationSpline,
    origin: Vector,
    active: bool,
}
#[derive(Clone, Default)]
pub(super) struct Curves {
    tracks: BTreeMap<BodyId, Track>,
    layout: std::sync::OnceLock<std::sync::Arc<layout::Layout>>,
    samples: std::sync::OnceLock<Vec<TranslationSample>>,
    // Rays and broad-phase queries address native handles rather than public identities.
    handles: std::collections::HashMap<RigidBodyHandle, BodyId>,
    time: Duration,
    /// Positive and speculative native pairs over the last complete public step.
    pub contacts: Vec<RigidContactObservation>,
    pub steps: usize,
}
impl Curves {
    pub(super) fn authored(&self) -> bool {
        !self.tracks.is_empty()
    }
    pub(super) fn drives(&self, handle: RigidBodyHandle) -> bool {
        self.handles
            .get(&handle)
            .and_then(|id| self.tracks.get(id))
            .is_some_and(|track| track.active)
    }
    pub(super) fn active(&self) -> bool {
        self.tracks.values().any(|t| t.active)
    }
    pub(super) fn remove(&mut self, id: BodyId, handle: RigidBodyHandle) {
        self.invalidate_layout();
        self.handles.remove(&handle);
        if self.tracks.remove(&id).is_some() && self.tracks.is_empty() {
            self.contacts.clear();
            self.steps = 0;
        }
    }
    pub(super) fn override_link(&mut self, id: BodyId) {
        if let Some(t) = self.tracks.get_mut(&id) {
            t.active = false;
            self.invalidate_layout();
        }
    }
    pub(super) fn reset(&mut self, bodies: &BTreeMap<BodyId, BodyEntry>) {
        self.invalidate_layout();
        for (id, t) in &mut self.tracks {
            t.handle = bodies[id].0;
        }
        self.handles.clear();
        self.handles
            .extend(self.tracks.keys().map(|id| (bodies[id].0, *id)));
        self.time = Duration::ZERO;
        self.steps = 0;
        self.contacts.clear();
        for t in self.tracks.values_mut() {
            t.active = true;
        }
    }
    pub(super) fn next_time(&self, dt: Duration) -> Result<Duration, PhysicsError> {
        self.time.checked_add(dt).ok_or(PhysicsError::InvalidValue)
    }
    pub(super) fn advance_unloaded(&mut self, time: Duration) {
        self.samples.take();
        self.time = time;
    }
    pub(super) fn at(&self, id: BodyId) -> Option<(Vector, Vector)> {
        let track = self.tracks.get(&id).filter(|t| t.active)?;
        let s = track.path.sample(self.time);
        Some((
            track.origin + Vector::from(s.displacement),
            Vector::from(s.velocity),
        ))
    }
    pub(super) fn at_handle(&self, handle: RigidBodyHandle) -> Option<(Vector, Vector)> {
        self.at(*self.handles.get(&handle)?)
    }
    /// Validate both the bounded schedule and overflow before installing any physical substep.
    pub(super) fn plan(
        &self,
        dt: Duration,
        iterations: usize,
    ) -> Result<Vec<Duration>, PhysicsError> {
        if !self.active() {
            return Ok(Vec::new());
        }
        let end = self
            .time
            .checked_add(dt)
            .ok_or(PhysicsError::InvalidValue)?;
        let ns = dt.as_nanos();
        let count = ns.div_ceil(MAX_SLICE_NS).max((iterations as u128).min(ns));
        if count > MAX_INTERVALS as u128 {
            return Err(PhysicsError::InvalidValue);
        }
        let layout = self.layout();
        let mut deadlines = Vec::with_capacity(count as usize);
        let mut at = self.time;
        for i in 1..=count {
            let ns = (i * ns / count) as u64;
            let grid = self.time + Duration::from_nanos(ns);
            while at < grid {
                let mut slice = grid - at;
                for path in &layout.paths {
                    if let Some(knot) = path.until_knot(at) {
                        slice = slice.min(knot);
                    }
                }
                if deadlines.len() == MAX_INTERVALS {
                    return Err(PhysicsError::InvalidValue);
                }
                at += slice;
                deadlines.push(at);
            }
        }
        debug_assert_eq!(at, end);
        Ok(deadlines)
    }
    pub(super) fn chord(
        &self,
        id: BodyId,
        dt: Duration,
    ) -> Result<Option<(Vector, f64)>, PhysicsError> {
        let Some(t) = self.tracks.get(&id).filter(|t| t.active) else {
            return Ok(None);
        };
        let end = self
            .time
            .checked_add(dt)
            .ok_or(PhysicsError::InvalidValue)?;
        let diff = Vector::from(t.path.sample(end).displacement)
            - Vector::from(t.path.sample(self.time).displacement);
        let h = dt.as_secs_f64();
        let margin = t.path.acceleration_bound() * h * h / 8.;
        let velocity = diff / h;
        if !velocity.is_finite() || !margin.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Some((velocity, margin)))
    }
}
impl RapierBackend {
    /// Prescribe the translation of a newly authored independent, centered kinematic primitive.
    /// Its immutable construction pose and first tangent remain the loaded reset frame.
    pub fn set_initial_translation_path(
        &mut self,
        id: BodyId,
        path: TranslationSpline,
    ) -> Result<(), PhysicsError> {
        let (handle, construction, rotation, inertia, _) =
            self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        if !self.joints.construction_open()
            || !construction.kinematic
            || construction.articulation.is_some()
            || self.has_joint(id) != Some(false)
            || rotation.angular_velocity != [0.; 3]
            || inertia.is_some_and(|i| i.center != [0.; 3])
            || !self.world.bodies[*handle].is_kinematic()
            || self.curves.tracks.contains_key(&id)
        {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "translation path needs a fresh centered free kinematic link",
            });
        }
        if construction.velocity != path.sample(Duration::ZERO).velocity
            || path.speed_bound() > self.world.integration_parameters.max_linear_velocity()
            || !std::array::from_fn::<_, 3, _>(|i| {
                construction.position[i].abs() + path.extent()[i]
            })
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let origin = Vector::from(construction.position);
        let handle = *handle;
        self.curves.invalidate_layout();
        self.curves.handles.insert(handle, id);
        self.rejoin_fleet();
        self.fleet_dispatch = fleet_isolation::Dispatch::Disabled;
        self.curves.tracks.insert(
            id,
            Track {
                handle,
                path,
                origin,
                active: true,
            },
        );
        Ok(())
    }
    /// Displacement and its derivatives at the last physical publication.
    pub fn translation_sample(&self, id: BodyId) -> Option<TranslationSample> {
        self.curves
            .tracks
            .get(&id)
            .filter(|t| t.active)
            .map(|t| t.path.sample(self.curves.time))
    }
    /// Immutable curve, world origin and authoritative clock at the last publication.
    pub fn translation_path(&self, id: BodyId) -> Option<(TranslationSpline, [f64; 3], Duration)> {
        let t = self.curves.tracks.get(&id).filter(|t| t.active)?;
        Some((t.path.clone(), t.origin.to_array(), self.curves.time))
    }
    /// Native substep count and normal impulses integrated across all native substeps of the last curve interval.
    pub fn translation_interval(&self) -> (usize, &[RigidContactObservation]) {
        (self.curves.steps, &self.curves.contacts)
    }
    pub(super) fn advance_curve_world(&mut self, deadlines: &[Duration]) {
        let iterations = self.world.integration_parameters.num_solver_iterations;
        let original = self.world.integration_parameters.dt;
        self.world.integration_parameters.num_solver_iterations = 1;
        let mut reactions = BTreeMap::<(BodyId, BodyId), RigidContactObservation>::new();
        let layout = self.curves.layout().clone();
        let mut means = Vec::with_capacity(layout.paths.len());
        let mut ends = Vec::with_capacity(layout.paths.len());
        for &end in deadlines {
            let at = self.curves.time;
            let dt = end - at;
            self.world.integration_parameters.dt = dt.as_secs_f64();
            self.transport_velocities(Some(dt.as_secs_f64()));
            means.clear();
            ends.clear();
            for p in &layout.paths {
                means.push(Vector::from(
                    p.mean_velocity(at, dt)
                        .expect("preflight divides every polynomial boundary"),
                ));
                ends.push(p.sample(end));
            }
            for row in &layout.rows {
                self.world.bodies[row.handle].set_linvel(means[row.group], true);
            }
            self.step_with_held_supports();
            if self.last_step_quarantined() {
                break;
            }
            for hit in self.rigid_contact_observations() {
                reactions
                    .entry((hit.a, hit.b))
                    .and_modify(|h| {
                        h.active |= hit.active;
                        h.normal_impulse += hit.normal_impulse;
                    })
                    .or_insert(hit);
            }
            for row in &layout.rows {
                let s = ends[row.group];
                let body = &mut self.world.bodies[row.handle];
                body.set_translation(row.origin + Vector::from(s.displacement), true);
                body.set_linvel(Vector::from(s.velocity), true);
            }
            self.curves.advance_unloaded(end);
            self.curves.steps += 1;
        }
        self.world.integration_parameters.num_solver_iterations = iterations;
        self.world.integration_parameters.dt = original;
        self.curves.contacts = reactions.into_values().collect();
    }
}
