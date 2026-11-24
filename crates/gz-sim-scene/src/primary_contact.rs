// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prospective refinement at a released box foot’s touchdown or a nominated box face strike.
//! The authored world plugin selects pairs. All bodies share the native clock.

mod early_face;
use early_face::EarlyFace;
mod geometry;
mod returning_pad;
pub use returning_pad::PadImpact;
use returning_pad::{CornerHistory, PadMotion};
pub(crate) mod sdf;
use crate::{BodyHandle, Gravity, JointHandle, LinkCenterOfMass, LoadError, Name};
use bevy_ecs::prelude::*;
use geometry::BoxShape;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, DynamicBackend, JointId, PhysicsBackend, PhysicsReset};
use gz_physics_multifidelity::{
    ContactIntervalReceipt, ContactSegment, ContactSolverIterations, MultiFidelityBackend,
};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::time::Duration;

impl sdf::Projection {
    fn closing_now_or_within(
        self,
        native: &MultiFidelityBackend,
        (foot, pad): (BodyId, BodyId),
        dt: Duration,
        certify_face_vertex: bool,
        certify_skew_edge: bool,
        resolve_fractional_points: bool,
    ) -> Result<bool, gz_physics::PhysicsError> {
        let now = match self {
            Self::Corner => native.corner_at_support_face(foot, pad),
            Self::Planar => native.planar_at_support_face(foot, pad),
            Self::SlidingEdge => native.sliding_edge_at_support_face(foot, pad),
            Self::FiniteSegment => native.finite_segment_closing(foot, pad),
            Self::Finite => native.finite_point_closing(foot, pad),
            Self::MixedEdge => native.mixed_edge_at_support_face(foot, pad),
            Self::FrictionlessSole => native.frictionless_sole_closing(foot, pad),
            Self::Edge => native.edge_at_support_face(foot, pad),
        }?;
        if now {
            return Ok(true);
        }
        let flight = match self {
            Self::Corner => native.corner_arrival(foot, pad, dt),
            Self::Planar => native.planar_arrival(foot, pad, dt),
            Self::SlidingEdge => native.sliding_edge_arrival(foot, pad, dt),
            Self::FiniteSegment => return Ok(false),
            Self::Finite if resolve_fractional_points => {
                native.fractional_point_arrival(foot, pad, dt)
            }
            Self::Finite if certify_skew_edge => {
                native.certified_skew_arrival(foot, pad, dt, certify_face_vertex)
            }
            Self::Finite if certify_face_vertex => native.certified_face_arrival(foot, pad, dt),
            Self::Finite => native.finite_arrival(foot, pad, dt),
            Self::MixedEdge => native.mixed_edge_arrival(foot, pad, dt),
            Self::FrictionlessSole => native.frictionless_sole_arrival(foot, pad, dt),
            Self::Edge => native.edge_arrival(foot, pad, dt),
        }?;
        Ok(flight.is_some())
    }
}

/// An authored pair that requested refinement at the beginning of the tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RequestedPair {
    pub foot_link: u64,
    pub support_link: u64,
    pub foot: BodyId,
    pub support: BodyId,
    /// Total delivered impulse across this public interval, in N·s.
    pub normal_impulse: f64,
    /// Integrated signed normal reaction on the foot, in world axes and N·s.
    pub normal_impulse_on_foot: [f64; 3],
    /// Signed world-space friction impulse on the foot where the native model reports it.
    pub tangent_impulse_on_foot: Option<[f64; 3]>,
    pub first_positive_end: Option<Duration>,
    pub active_at_end: bool,
    /// Impulse from the last native solve alone, in N·s.
    pub last_native_normal_impulse: f64,
}
/// Results from the most recent successfully refined public tick.
#[derive(Clone, Debug, PartialEq)]
pub struct Refinement {
    /// Minimum native calls requested by a resident registered screw speed stop.
    /// One leaves the ordinary step available. A predicted impact may request more.
    pub speed_stop_floor: u32,
    pub epoch: u64,
    pub iteration: u64,
    pub start_time: Duration,
    pub sim_time: Duration,
    pub requested_pairs: Vec<RequestedPair>,
    /// Enrolled pad pairs sampled during a returning-corner refinement.
    pub pad_impacts: Vec<PadImpact>,
    /// Actual fine prefix and remaining impact grid, present only on an admitted separated face.
    pub early_face_schedule: Option<[ContactSegment; 2]>,
    /// The actually used three-part schedule when a positive reaction and separating
    /// cuboid surfaces allowed a longer middle step after impact.
    pub released_face_schedule: Option<[ContactSegment; 3]>,
    /// Solver settings requested for this interval's ordinary impact grid. Absent fields inherit the world’s settings.
    pub solver: ContactSolverIterations,
    /// Includes contacts elsewhere in the same collision world.
    pub interval: ContactIntervalReceipt,
}
impl Refinement {
    /// Exact native segments used for an admitted separated-face refinement.
    pub fn realized_face_schedule(&self) -> Option<&[ContactSegment]> {
        self.released_face_schedule
            .as_ref()
            .map(|s| s.as_slice())
            .or_else(|| self.early_face_schedule.as_ref().map(|s| s.as_slice()))
    }
}
/// Returns the latest committed receipt in this epoch. A pause retains it.
pub fn last_refinement(sim: &Simulation) -> Option<&Refinement> {
    let receipt = sim.world().get_resource::<Policy>()?.last.as_ref()?;
    (!sim.is_faulted() && receipt.epoch == sim.epoch() && receipt.sim_time <= sim.elapsed())
        .then_some(receipt)
}
/// The current committed public step only. Sensor reads must not reuse an older impact.
pub(crate) fn current_interval<'w>(
    world: &'w World,
    clock: &UpdateInfo,
) -> Option<&'w ContactIntervalReceipt> {
    let r = world.get_resource::<Policy>()?.last.as_ref()?;
    (r.epoch == clock.epoch && r.iteration == clock.iteration && r.sim_time == clock.sim_time)
        .then_some(&r.interval)
}
/// Geometry requests on the last advancing tick that used selected box features.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoxFeatureRefresh {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub native_steps: u32,
    /// Foot then support, in authored pair order after stable scoped-link sorting.
    pub pairs: Vec<(BodyId, BodyId)>,
    /// Number of existing populated native manifolds recomputed across all substeps.
    pub rebuilt_manifolds: u64,
}
pub fn last_box_feature_refresh(sim: &Simulation) -> Option<&BoxFeatureRefresh> {
    let r = sim
        .world()
        .get_resource::<Policy>()?
        .last_geometry
        .as_ref()?;
    (!sim.is_faulted() && r.epoch == sim.epoch() && r.sim_time <= sim.elapsed()).then_some(r)
}
/// The latest nominated landing that checked one or more authored returning-box pairs.
#[derive(Clone, Debug, PartialEq)]
pub struct ReturningBoxFeatures {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub pairs: Vec<gz_physics_multifidelity::ReturningBoxFeatureReceipt>,
}
pub fn last_returning_box_features(sim: &Simulation) -> Option<&ReturningBoxFeatures> {
    let r = sim
        .world()
        .get_resource::<Policy>()?
        .last_returning
        .as_ref()?;
    (!sim.is_faulted() && r.epoch == sim.epoch() && r.sim_time <= sim.elapsed()).then_some(r)
}
/// The most recent public tick on which authored finite-box pairs requested live timing.
#[derive(Clone, Debug, PartialEq)]
pub struct BoxTimingInterval {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub pairs: Vec<(BodyId, BodyId)>,
    pub timing: gz_physics_multifidelity::BoxTimingReceipt,
    pub interval: ContactIntervalReceipt,
}
pub fn last_box_timing(sim: &Simulation) -> Option<&BoxTimingInterval> {
    let r = sim.world().get_resource::<Policy>()?.last_timing.as_ref()?;
    (!sim.is_faulted() && r.epoch == sim.epoch() && r.sim_time <= sim.elapsed()).then_some(r)
}
/// Last committed selected-pair separation check, including ordinary one-call ticks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoxFlightInterval {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub pairs: Vec<(BodyId, BodyId)>,
    pub flight: gz_physics_multifidelity::BoxFlightReceipt,
}
pub fn last_box_flight(sim: &Simulation) -> Option<&BoxFlightInterval> {
    let r = sim.world().get_resource::<Policy>()?.last_flight.as_ref()?;
    (!sim.is_faulted() && r.epoch == sim.epoch() && r.sim_time <= sim.elapsed()).then_some(r)
}
#[derive(Clone, Copy)]
struct Link {
    id: u64,
    entity: Option<Entity>,
    body: BodyId,
    shape: BoxShape,
}
impl Link {
    fn present(self, links: &Query<(&EntityId, &BodyHandle)>) -> bool {
        self.entity
            .and_then(|e| links.get(e).ok())
            .is_some_and(|(id, body)| id.get() == self.id && body.id() == self.body)
    }
    fn rebind(&mut self, world: &World) {
        self.entity = entity_by_id(world, self.id);
        if let Some(h) = self.entity.and_then(|e| world.get::<BodyHandle>(e)) {
            self.body = h.id();
        }
    }
}
/// Prospective decisions taken at the public pre-solve boundary.
#[derive(Clone, Debug)]
pub struct BoxSweepInterval {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub pairs: Vec<(BodyId, BodyId, gz_physics_multifidelity::RigidBoxSweep)>,
}
pub fn last_box_sweep(sim: &Simulation) -> Option<&BoxSweepInterval> {
    sim.world().get_resource::<Policy>()?.last_sweeps.as_ref()
}
/// Committed two-sided native interval for a registered upper-corner request.
#[derive(Clone, Debug, PartialEq)]
pub struct OpposingCornerInterval {
    pub epoch: u64,
    pub iteration: u64,
    pub sim_time: Duration,
    pub interval: gz_physics_multifidelity::ContactIntervalReceipt,
}
pub fn last_opposing_corner(sim: &Simulation) -> Option<&OpposingCornerInterval> {
    sim.world().get_resource::<Policy>()?.last_upper.as_ref()
}
/// Force distribution is indeterminate. Carry a previously admissible selection
/// only across a continuous material boundary, including a stationary tier hold.
struct SupportPressure {
    fraction: f64,
    foot: BodyState,
    pad: BodyState,
}
struct Pair {
    returning_pad: Option<JointId>,
    corners: CornerHistory,
    incoming_face: Option<DVec3>,
    strike_latched: bool,
    follow_planar_edge: bool,
    follow_sliding_vertex: bool,
    follow_sticking_point: bool,
    resolve_point_transfers: bool,
    subdivide_point_bounds: bool,
    compact_point_tail: bool,
    refine_active_point: bool,
    detect_initial_point: bool,
    point_at_previous_end: bool,
    target_point_bounds: bool,
    follow_sliding_skew_edge: bool,
    follow_skew_slip_launch: bool,
    follow_frictionless_edge: bool,
    resolve_rocking_limit: bool,
    follow_frictionless_face: bool,
    follow_frictionless_deck_edge: bool,
    share_coplanar_face: bool,
    share_uniform_sliding: bool,
    settle_comoving_supports: bool,
    pressure_history: Option<Box<SupportPressure>>,
    edge_active: bool,
    inspect_initial_bearing: bool,
    guard: bool,
    rotate: bool,
    ballistic: bool,
    follow_release: bool,
    detect_crossings: bool,
    detect_finite_arrival: bool,
    compact_budget: bool,
    rigid_corner: bool,
    rigid_corner_neighbors: bool,
    opponent: Option<Link>,
    project_opponent_corner: bool,
    project_opponent_return: bool,
    project_opposing_normals: bool,
    rigid_departure: bool,
    retime_finite_transfer: bool,
    certify_face_vertex: bool,
    certify_skew_edge: bool,
    resolve_fractional_points: bool,
    certify_segment_friction_work: bool,
    release_active: bool,
    refine_rotation_timing: bool,
    rotation_pulse: Option<gz_physics_multifidelity::RotationContactPulse>,
    projection: Option<sdf::Projection>,
    refresh: bool,
    refresh_returning: bool,
    timing: bool,
    tracking_geometry: bool,
    foot: Link,
    pad: Link,
    armed: bool,
}
impl Pair {
    fn key(&self) -> (BodyId, BodyId) {
        (
            self.foot.body.min(self.pad.body),
            self.foot.body.max(self.pad.body),
        )
    }
}
struct SpeedStop {
    id: u64,
    entity: Option<Entity>,
    rate: f64,
}
#[derive(Resource)]
struct Policy {
    pad_rocking_steps: Option<u32>,
    early_face: Option<EarlyFace>,
    touching_solver: Option<(u32, ContactSolverIterations)>,
    speed_stop: Option<SpeedStop>,
    enabled: bool,
    follows_release: bool,
    shares_faces: bool,
    pairs: Vec<Pair>,
    calls: u32,
    iterations: ContactSolverIterations,
    last: Option<Refinement>,
    last_geometry: Option<BoxFeatureRefresh>,
    last_returning: Option<ReturningBoxFeatures>,
    last_timing: Option<BoxTimingInterval>,
    last_flight: Option<BoxFlightInterval>,
    last_sweeps: Option<BoxSweepInterval>,
    last_upper: Option<OpposingCornerInterval>,
}

/// Errors when changing an explicitly authored refinement policy at a committed boundary.
#[derive(Debug, thiserror::Error)]
pub enum PolicyError {
    #[error("this world has no authored primary contact refinement policy")]
    Unconfigured,
    #[error(transparent)]
    Clock(#[from] gz_sim_core::StepError),
}
/// Receives native substeps of a *selected* interval. The UpdateInfo identifies
/// the outer tick. Measurements are provisional until that simulation tick
/// commits. The callback has read-only native access and runs synchronously.
#[derive(Resource)]
struct RefinedObserver(
    Box<dyn for<'a> FnMut(UpdateInfo, gz_physics_multifidelity::ContactSubstep<'a>) + Send + Sync>,
);
impl RefinedObserver {
    fn sample(&mut self, time: UpdateInfo, step: gz_physics_multifidelity::ContactSubstep<'_>) {
        (self.0)(time, step);
    }
}
/// Install or replace an interval observer on an already-authored local policy.
pub fn set_refined_observer<F>(sim: &mut Simulation, observer: F) -> Result<(), PolicyError>
where
    F: for<'a> FnMut(UpdateInfo, gz_physics_multifidelity::ContactSubstep<'a>)
        + Send
        + Sync
        + 'static,
{
    sim.validate_steps(0)?;
    if !installed(sim.world()) {
        return Err(PolicyError::Unconfigured);
    }
    sim.world_mut()
        .insert_resource(RefinedObserver(Box::new(observer)));
    Ok(())
}
/// Stop observing selected native solves. Registration persists across reset
/// until explicitly cleared, so callbacks can distinguish epochs.
pub fn clear_refined_observer(sim: &mut Simulation) -> Result<(), PolicyError> {
    sim.validate_steps(0)?;
    if !installed(sim.world()) {
        return Err(PolicyError::Unconfigured);
    }
    sim.world_mut().remove_resource::<RefinedObserver>();
    Ok(())
}

fn selected_interval(
    native: &mut MultiFidelityBackend,
    time: UpdateInfo,
    ordinary: ContactSegment,
    early: Option<&[ContactSegment; 2]>,
    fresh: &[(BodyId, BodyId)],
    observer: Option<&mut dyn FnMut(gz_physics_multifidelity::ContactSubstep<'_>)>,
) -> Result<(ContactIntervalReceipt, u64), gz_physics::PhysicsError> {
    let segments = early.map_or(std::slice::from_ref(&ordinary), |pair| pair.as_slice());
    if let Some(probe) = observer {
        native.observe_contact_schedule(time.dt, segments, fresh, probe)
    } else if early.is_some() {
        native.step_contact_schedule(time.dt, segments, fresh)
    } else if fresh.is_empty() {
        native
            .step_contact_interval(time.dt, ordinary.native_steps, ordinary.iterations)
            .map(|r| (r, 0))
    } else {
        native.step_contact_interval_with_box_features(
            time.dt,
            ordinary.native_steps,
            ordinary.iterations,
            fresh,
        )
    }
}

/// Current switch for an authored local primary contact policy.
pub fn is_enabled(sim: &Simulation) -> Option<bool> {
    sim.world().get_resource::<Policy>().map(|p| p.enabled)
}

/// Toggle future selections without advancing physics or changing the current
/// native contact history. A changed setting clears release and feature-tracking
/// witnesses. Foot touchdown requires a new measured release, and incoming faces may detect a cold strike. Reset restores the
/// authored enabled state. Historical receipts remain readable in their epoch.
pub fn set_enabled(sim: &mut Simulation, enabled: bool) -> Result<(), PolicyError> {
    sim.validate_steps(0)?;
    let mut p = sim
        .world_mut()
        .get_resource_mut::<Policy>()
        .ok_or(PolicyError::Unconfigured)?;
    if p.enabled != enabled {
        p.enabled = enabled;
        for binding in &mut p.pairs {
            binding.corners = CornerHistory::default();
            binding.strike_latched = false;
            binding.armed = false;
            binding.tracking_geometry = false;
            binding.edge_active = false;
            binding.point_at_previous_end = false;
            binding.pressure_history = None;
            binding.inspect_initial_bearing = true;
            binding.release_active = false;
        }
    }
    Ok(())
}
pub(crate) fn installed(world: &World) -> bool {
    world.contains_resource::<Policy>()
}

pub(crate) fn invalid(entity: &str, reason: &str) -> LoadError {
    LoadError::PrimaryContactRefinement {
        entity: entity.into(),
        reason: reason.into(),
    }
}
pub(crate) fn install(
    sim: &mut Simulation,
    backend: PhysicsBackend,
    plan: sdf::Plan,
) -> Result<(), LoadError> {
    let native = backend
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or_else(|| invalid(&plan.world, "requires the local multifidelity backend"))?;
    native
        .validate_contact_interval(sim.step_duration(), plan.calls, plan.iterations)
        .map_err(|e| invalid(&plan.world, &e.to_string()))?;
    if let Some((calls, solver)) = plan.touching_solver {
        native
            .validate_contact_interval(sim.step_duration(), calls, solver)
            .map_err(|e| invalid(&plan.world, &e.to_string()))?;
    }
    let speed_stop = if let Some(name) = &plan.speed_stop_joint {
        #[allow(deprecated)]
        let binding = sim.world().iter_entities().find_map(|e| {
            if e.get::<Name>()?.0 != *name {
                return None;
            }
            Some(SpeedStop {
                id: e.get::<EntityId>()?.get(),
                entity: Some(e.id()),
                rate: native.screw_stop_resolution_rate(e.get::<JointHandle>()?.id())?,
            })
        });
        Some(binding.ok_or_else(|| {
            invalid(
                name,
                "speed_stop_joint requires a screw with positive finite effort and speed limits",
            )
        })?)
    } else {
        None
    };
    let resolve = |name: &str, half: DVec3| {
        #[allow(deprecated)]
        sim.world()
            .iter_entities()
            .find_map(|e| {
                if e.get::<Name>()?.0 != name {
                    return None;
                }
                Some(Link {
                    id: e.get::<EntityId>()?.get(),
                    entity: Some(e.id()),
                    body: e.get::<BodyHandle>()?.id(),
                    shape: BoxShape {
                        half,
                        local_com: e
                            .get::<LinkCenterOfMass>()
                            .map_or(DVec3::ZERO, |v| v.0.into()),
                    },
                })
            })
            .ok_or_else(|| invalid(name, "missing local rigid body"))
    };
    let mut pairs = Vec::with_capacity(plan.pairs.len());
    for p in plan.pairs {
        let foot = resolve(&p.foot, p.foot_half)?;
        let pad = resolve(&p.support, p.pad_half)?;
        let returning_pad = if p.returning_pad {
            Some(native.pad_joint(pad.body).ok_or_else(|| {
                invalid(
                    &p.support,
                    "returning_pad_face requires an authored reduced pad stiffness",
                )
            })?)
        } else {
            None
        };
        if native.is_dynamic(foot.body) != Some(true) {
            return Err(invalid(&p.foot, "foot must be a dynamic rigid body"));
        }
        let instantaneous_edge = matches!(
            p.projection,
            Some(sdf::Projection::SlidingEdge | sdf::Projection::MixedEdge | sdf::Projection::Edge)
        ) && !p.refresh_box_features
            && !p.refresh_returning_box_features
            && !p.prospective_timing;
        if p.incoming_face.is_none()
            && (native.is_dynamic(pad.body) != Some(false)
                || !native.state(pad.body).is_some_and(|s| {
                    if instantaneous_edge {
                        // Only current tangency can qualify on a rotating support.
                        // The shared free-flight planner requires a constant support orientation.
                        true
                    } else if p.refresh_box_features
                        || p.refresh_returning_box_features
                        || p.prospective_timing
                        || p.projection.is_some()
                        || p.detect_finite_arrival
                        || p.follow_frictionless_face
                    {
                        s.angular_velocity == [0.; 3]
                    } else {
                        geometry::horizontal(s)
                    }
                }))
        {
            return Err(invalid(
                &p.support,
                if instantaneous_edge {
                    "support must be a fixed or prescribed box"
                } else if p.refresh_box_features
                    || p.refresh_returning_box_features
                    || p.prospective_timing
                    || p.projection.is_some()
                    || p.detect_finite_arrival
                    || p.follow_frictionless_face
                {
                    "support must be a fixed or translating box without spin"
                } else {
                    "support must be a fixed or translating horizontal box"
                },
            ));
        }
        let opponent = p
            .opponent
            .as_ref()
            .map(|(name, half)| {
                let link = resolve(name, *half)?;
                if native.is_dynamic(link.body) != Some(false)
                    || !native
                        .state(link.body)
                        .is_some_and(|s| s.angular_velocity == [0.; 3])
                {
                    return Err(invalid(
                        name,
                        "guard_opponent requires a fixed or translating box",
                    ));
                }
                Ok(link)
            })
            .transpose()?;
        pairs.push(Pair {
            returning_pad,
            corners: CornerHistory::default(),
            incoming_face: p.incoming_face,
            strike_latched: false,
            opponent,
            project_opponent_corner: p.project_opponent_corner,
            project_opponent_return: p.project_opponent_return,
            project_opposing_normals: p.project_opposing_normals,
            follow_planar_edge: p.follow_planar_edge,
            follow_sliding_vertex: p.follow_sliding_vertex,
            follow_sticking_point: p.follow_sticking_point,
            resolve_point_transfers: p.resolve_point_transfers,
            subdivide_point_bounds: p.subdivide_point_bounds,
            compact_point_tail: p.compact_point_tail,
            refine_active_point: p.refine_active_point,
            detect_initial_point: p.detect_initial_point,
            point_at_previous_end: false,
            target_point_bounds: p.target_point_bounds,
            follow_sliding_skew_edge: p.follow_sliding_skew_edge,
            follow_skew_slip_launch: p.follow_skew_slip_launch,
            follow_frictionless_edge: p.follow_frictionless_edge,
            resolve_rocking_limit: p.resolve_rocking_limit,
            follow_frictionless_face: p.follow_frictionless_face,
            follow_frictionless_deck_edge: p.follow_frictionless_deck_edge,
            share_coplanar_face: p.share_coplanar_face,
            share_uniform_sliding: p.share_uniform_sliding,
            settle_comoving_supports: p.settle_comoving_supports,
            pressure_history: None,
            edge_active: false,
            inspect_initial_bearing: true,
            projection: p.projection,
            guard: p.guard_separated_flight,
            rotate: p.integrate_free_rotation,
            ballistic: p.ballistic_translation,
            follow_release: p.integrate_released_rotation,
            detect_crossings: p.detect_separated_crossings,
            detect_finite_arrival: p.detect_finite_arrival,
            compact_budget: p.compact_budget,
            rigid_corner: p.rigid_corner,
            rigid_corner_neighbors: p.rigid_corner_neighbors,
            rigid_departure: p.rigid_departure,
            retime_finite_transfer: p.retime_finite_transfer,
            certify_face_vertex: p.certify_face_vertex,
            certify_skew_edge: p.certify_skew_edge,
            resolve_fractional_points: p.resolve_fractional_points,
            certify_segment_friction_work: p.certify_segment_friction_work,
            release_active: false,
            refine_rotation_timing: p.refine_rotation_timing,
            rotation_pulse: p.rotation_pulse,
            refresh: p.refresh_box_features,
            refresh_returning: p.refresh_returning_box_features,
            timing: p.prospective_timing,
            tracking_geometry: false,
            foot,
            pad,
            armed: false,
        });
    }
    pairs.sort_by_key(|p| (p.foot.id, p.pad.id));
    gz_physics::install_with_step(sim, backend, advance)?;
    sim.world_mut().insert_resource(Policy {
        speed_stop,
        shares_faces: pairs.iter().any(|p| p.share_coplanar_face),
        follows_release: pairs.iter().any(|p| p.follow_release),
        enabled: true,
        pairs,
        early_face: plan.early_face,
        touching_solver: plan.touching_solver,
        pad_rocking_steps: plan.pad_rocking_steps,
        calls: plan.calls,
        iterations: plan.iterations,
        last: None,
        last_geometry: None,
        last_returning: None,
        last_timing: None,
        last_flight: None,
        last_sweeps: None,
        last_upper: None,
    });
    sim.add_systems(Phase::Reset, reset.after(PhysicsReset));
    Ok(())
}
fn reset(world: &mut World) {
    world.resource_scope(|world, mut policy: Mut<Policy>| {
        policy.enabled = true;
        policy.last = None;
        policy.last_geometry = None;
        policy.last_returning = None;
        policy.last_timing = None;
        policy.last_flight = None;
        policy.last_sweeps = None;
        policy.last_upper = None;
        if let Some(stop) = &mut policy.speed_stop {
            stop.entity = entity_by_id(world, stop.id);
        }
        for p in &mut policy.pairs {
            p.armed = false;
            p.tracking_geometry = false;
            p.edge_active = false;
            p.point_at_previous_end = false;
            p.pressure_history = None;
            p.inspect_initial_bearing = true;
            p.release_active = false;
            p.corners = CornerHistory::default();
            p.strike_latched = false;
            p.foot.rebind(world);
            p.pad.rebind(world);
            if let Some(id) = &mut p.returning_pad
                && let Some(native) = world
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                && let Some(new_id) = native.pad_joint(p.pad.body)
            {
                *id = new_id;
            }
            if let Some(link) = &mut p.opponent {
                link.rebind(world);
            }
        }
    });
}
#[derive(bevy_ecs::system::SystemParam)]
struct Bindings<'w, 's> {
    links: Query<'w, 's, (&'static EntityId, &'static BodyHandle)>,
    joints: Query<'w, 's, &'static EntityId, With<JointHandle>>,
}
fn advance(
    time: Res<UpdateInfo>,
    gravity: Res<Gravity>,
    fault: Res<StepFault>,
    bindings: Bindings,
    mut policy: ResMut<Policy>,
    mut backend: ResMut<PhysicsBackend>,
    mut observer: Option<ResMut<RefinedObserver>>,
) {
    if time.dt.is_zero() {
        return;
    }
    let Some(native) = backend.downcast_mut::<MultiFidelityBackend>() else {
        fault.report(std::io::Error::other(
            "primary contact refinement lost its local multifidelity backend",
        ));
        return;
    };
    if !policy.enabled {
        if let Err(e) = native.step(time.dt) {
            fault.report(e);
        }
        return;
    }
    let Bindings { links, joints } = bindings;
    let floor = policy
        .speed_stop
        .as_ref()
        .filter(|s| {
            s.entity
                .and_then(|e| joints.get(e).ok())
                .is_some_and(|id| id.get() == s.id)
        })
        .map_or(1, |s| {
            (s.rate * time.dt.as_secs_f64())
                .ceil()
                .clamp(1., f64::from(policy.calls)) as u32
        });
    let mut requests = Vec::new();
    let pad_rocking_steps = policy.pad_rocking_steps;
    let mut rocking_requests = 0;
    let mut initial_face = None;
    let mut separated_face = None;
    let mut cold_request = None;
    let mut fresh = Vec::new();
    let mut returning_features = Vec::new();
    let mut timing_pairs = Vec::new();
    let mut refined_timing = Vec::new();
    let mut compact_budget = Vec::new();
    let mut rigid_corners = Vec::new();
    let mut corner_neighbors = Vec::new();
    let mut rigid_departures = Vec::new();
    let mut pulses = Vec::new();
    let mut guards = Vec::new();
    let mut box_opponents = Vec::new();
    let mut upper_corners = Vec::new();
    let mut upper_returns = Vec::new();
    let mut opposing_normals = Vec::new();
    let mut rotations = Vec::new();
    let mut ballistic = Vec::new();
    let mut releases = Vec::new();
    let mut sweeps = Vec::new();
    let mut sweep_bounds = Vec::new();
    let mut continued = false;
    let mut projections = Vec::new();
    let mut sustained = Vec::new();
    let mut vertices = Vec::new();
    let mut sticking_points = Vec::new();
    let mut point_transfers = Vec::new();
    let mut skew_slides = Vec::new();
    let mut slip_launches = Vec::new();
    let mut subdivided = Vec::new();
    let mut compact_points = Vec::new();
    let mut directed = Vec::new();
    let mut two_bearing = Vec::new();
    let mut rocking_limits = Vec::new();
    let mut face_bearings = Vec::new();
    let mut deck_edge_bearings = Vec::new();
    let mut shared_faces = Vec::new();
    let mut patches = Vec::new();
    let mut edges = Vec::new();
    let mut mixed = Vec::new();
    let mut all_edges = Vec::new();
    let mut finite = Vec::new();
    let mut certified_vertices = Vec::new();
    let mut certified_edges = Vec::new();
    let mut fractional_points = Vec::new();
    let mut segments = Vec::new();
    let mut segment_work = Vec::new();
    let mut soles = Vec::new();
    let mut transfer = Vec::new();
    // Pairs are sorted by public foot ID. A foot's state cannot change between
    // prospective tests at this barrier, even if many supports are registered.
    let mut offset = 0;
    for group in policy.pairs.chunk_by_mut(|a, b| a.foot.id == b.foot.id) {
        let start = offset;
        offset += group.len();
        let foot_link = group[0].foot;
        if !foot_link.present(&links) || native.is_dynamic(foot_link.body) != Some(true) {
            for p in group {
                p.corners = CornerHistory::default();
                p.strike_latched = false;
                p.armed = false;
                p.tracking_geometry = false;
                p.edge_active = false;
                p.point_at_previous_end = false;
                if p.detect_initial_point {
                    p.inspect_initial_bearing = true;
                }
                if p.pressure_history.as_ref().is_some_and(|w| {
                    !foot_link.present(&links)
                        || !p.pad.present(&links)
                        || native.state(foot_link.body) != Some(w.foot)
                        || native.state(p.pad.body) != Some(w.pad)
                }) {
                    p.pressure_history = None;
                }
                p.release_active = false;
            }
            continue;
        }
        let Some(foot) = native.state(foot_link.body) else {
            fault.report(gz_physics::PhysicsError::MissingBody);
            return;
        };
        for (j, p) in group.iter_mut().enumerate() {
            if !p.pad.present(&links)
                || p.incoming_face.is_none() && native.is_dynamic(p.pad.body) != Some(false)
            {
                p.corners = CornerHistory::default();
                p.strike_latched = false;
                p.armed = false;
                p.tracking_geometry = false;
                p.edge_active = false;
                p.point_at_previous_end = false;
                if p.detect_initial_point {
                    p.inspect_initial_bearing = true;
                }
                p.pressure_history = None;
                p.release_active = false;
                continue;
            }
            let Some(pad) = native.state(p.pad.body) else {
                fault.report(gz_physics::PhysicsError::MissingBody);
                return;
            };
            if let Some(face) = p.incoming_face {
                if let Some(joint) = p.returning_pad {
                    if native.joint_child(joint) != Some(p.pad.body) {
                        p.corners = CornerHistory::default();
                        continue;
                    }
                    // A new corner can strike while its opposite edge is carrying load.
                    let reaction = match native.normal_reaction_on(p.foot.body, p.pad.body) {
                        Ok(reaction) => reaction,
                        Err(e) => {
                            fault.report(e);
                            return;
                        }
                    };
                    let normal = DQuat::from_array(pad.orientation) * face;
                    let loaded = reaction.is_some_and(|r| {
                        r.loaded_at_end && DVec3::from(r.impulse).dot(normal) > 0.
                    });
                    let motion = p.corners.select(
                        loaded,
                        face,
                        (foot, p.foot.shape),
                        (pad, p.pad.shape),
                        time.dt.as_secs_f64(),
                        pad_rocking_steps.is_some(),
                    );
                    // The applied couple can start a rock before the current
                    // velocities expose it. Resolve this entire input tick.
                    let forced = loaded
                        && pad_rocking_steps.is_some()
                        && native
                            .pending_world_wrench(p.foot.body)
                            .is_some_and(|(_, torque)| torque != [0.; 3]);
                    if forced || motion != PadMotion::Ordinary {
                        rocking_requests += usize::from(motion == PadMotion::Rocking && !forced);
                        requests.push(start + j);
                    }
                    continue;
                }
                if let Some(arrival) = geometry::incoming_face(
                    &mut p.strike_latched,
                    face,
                    (foot, p.foot.shape),
                    (pad, p.pad.shape),
                    time.dt.as_secs_f64(),
                ) {
                    requests.push(start + j);
                    match arrival {
                        geometry::FaceArrival::TouchingParallelFaces => {
                            initial_face = Some(start + j)
                        }
                        geometry::FaceArrival::SeparatedParallelFaces(t) => {
                            separated_face = Some(t)
                        }
                        geometry::FaceArrival::Predicted => {}
                    }
                }
                continue;
            }
            if p.pressure_history
                .as_ref()
                .is_some_and(|w| w.foot != foot || w.pad != pad)
            {
                p.pressure_history = None;
            }
            let pair = (p.foot.body, p.pad.body);
            let opponent = p
                .opponent
                .filter(|link| link.present(&links) && native.is_dynamic(link.body) == Some(false))
                .map(|link| link.body);
            let upper_reach = if p.project_opponent_corner
                && let Some(gate) = opponent
            {
                match native.opposing_corner_reach(pair.0, pair.1, gate, time.dt) {
                    Ok(r) => r,
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                }
            } else {
                gz_physics_multifidelity::OpposingCornerReach::Unavailable
            };
            let upper_possible =
                upper_reach == gz_physics_multifidelity::OpposingCornerReach::Possible;
            let upper_near = upper_possible
                || upper_reach == gz_physics_multifidelity::OpposingCornerReach::Nearby;
            let event = p.projection.map_or(Ok(false), |geometry| {
                geometry.closing_now_or_within(
                    native,
                    pair,
                    time.dt,
                    p.certify_face_vertex,
                    p.certify_skew_edge,
                    p.resolve_fractional_points,
                )
            });
            let project = match event {
                Ok(value) => value,
                Err(err) => {
                    fault.report(err);
                    return;
                }
            };
            let face_bearing = p.follow_frictionless_face
                && match native.coplanar_face_now(pair.0, pair.1, p.share_uniform_sliding) {
                    Ok(now) => now || project || p.edge_active,
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                };
            if face_bearing {
                face_bearings.push(pair);
                if p.follow_frictionless_deck_edge {
                    deck_edge_bearings.push(pair);
                }
            }
            let mut bearing = p.follow_frictionless_edge && (project || p.edge_active);
            let inspect = std::mem::take(&mut p.inspect_initial_bearing);
            if inspect
                && !bearing
                && p.follow_frictionless_edge
                && p.projection == Some(sdf::Projection::FrictionlessSole)
            {
                match native.frictionless_bearing_now(pair.0, pair.1, p.resolve_rocking_limit) {
                    Ok(active) => bearing = active,
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                }
            }
            if bearing {
                two_bearing.push(pair);
                if p.resolve_rocking_limit {
                    rocking_limits.push(pair);
                }
            }
            if p.follow_planar_edge && (project || p.edge_active) {
                sustained.push(pair);
            }
            if project
                || gravity.0 != [0.; 3]
                || p.projection == Some(sdf::Projection::Finite)
                || bearing
                || ((p.follow_sliding_vertex
                    || p.follow_sticking_point
                    || p.resolve_point_transfers
                    || p.follow_sliding_skew_edge
                    || p.follow_skew_slip_launch)
                    && p.edge_active)
            {
                if p.follow_sliding_vertex {
                    vertices.push(pair)
                }
                if p.follow_sticking_point {
                    sticking_points.push(pair)
                }
                if p.resolve_point_transfers {
                    point_transfers.push(pair)
                }
                if p.target_point_bounds {
                    directed.push(pair)
                }
                if p.subdivide_point_bounds {
                    subdivided.push(pair)
                }
                if p.compact_point_tail {
                    compact_points.push(pair)
                }
                if p.follow_sliding_skew_edge {
                    skew_slides.push(pair)
                }
                if p.follow_skew_slip_launch {
                    slip_launches.push(pair)
                }
                match p.projection {
                    Some(sdf::Projection::Corner) => projections.push(pair),
                    Some(sdf::Projection::Planar) => patches.push(pair),
                    Some(sdf::Projection::SlidingEdge) => edges.push(pair),
                    Some(sdf::Projection::FiniteSegment) => {
                        segments.push(pair);
                        if p.certify_segment_friction_work {
                            segment_work.push(pair)
                        }
                        if p.retime_finite_transfer {
                            transfer.push(pair)
                        }
                    }
                    Some(sdf::Projection::Finite) => {
                        finite.push(pair);
                        if p.certify_face_vertex {
                            certified_vertices.push(pair);
                        }
                        if p.certify_skew_edge {
                            certified_edges.push(pair);
                        }
                        if p.resolve_fractional_points {
                            fractional_points.push(pair);
                        }
                        if p.retime_finite_transfer {
                            transfer.push(pair);
                        }
                    }
                    Some(sdf::Projection::MixedEdge) => mixed.push(pair),
                    Some(sdf::Projection::Edge) => all_edges.push(pair),
                    Some(sdf::Projection::FrictionlessSole) => soles.push(pair),
                    None => {}
                }
            }
            let predict = if p.refresh || p.refresh_returning || p.timing || p.projection.is_some()
            {
                geometry::predict_oriented(
                    &mut p.armed,
                    (foot, p.foot.shape),
                    (pad, p.pad.shape),
                    gravity.0.into(),
                    time.dt.as_secs_f64(),
                    true,
                )
            } else {
                geometry::predict(
                    &mut p.armed,
                    (foot, p.foot.shape),
                    (pad, p.pad.shape),
                    gravity.0.into(),
                    time.dt.as_secs_f64(),
                )
            };
            let mut swept = false;
            let mut clear = false;
            if p.detect_crossings && !predict && !project {
                let sweep = match opponent {
                    Some(id) => native.rigid_box_sweep_with_opponent(pair.0, pair.1, id, time.dt),
                    None => native.rigid_box_sweep(pair.0, pair.1, time.dt),
                };
                match sweep {
                    Ok(s) => {
                        swept =
                            matches!(s, gz_physics_multifidelity::RigidBoxSweep::Possible { .. });
                        clear = matches!(s, gz_physics_multifidelity::RigidBoxSweep::Clear);
                        sweeps.push((pair.0, pair.1, s));
                    }
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                }
            }
            // Enroll the ordinary uniform grid at an isolated finite arrival,
            // including a cold start inside the release detector's hysteresis.
            let finite_arrival = if p.detect_finite_arrival && !predict && !swept && !project {
                match native.finite_arrival(pair.0, pair.1, time.dt) {
                    Ok(event) => event.is_some(),
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                }
            } else {
                false
            };
            let predict = predict || swept || finite_arrival;
            // Prior pressure only allocates solver work. Each new interval
            // re-reads both current shapes, poses, twists and material.
            let continuing_point = std::mem::take(&mut p.point_at_previous_end);
            let cold_point = if inspect
                && p.detect_initial_point
                && !predict
                && !project
                && !upper_possible
                && !continuing_point
            {
                match native.tangent_material_point(pair.0, pair.1) {
                    Ok(found) => found,
                    Err(e) => {
                        fault.report(e);
                        return;
                    }
                }
            } else {
                false
            };
            if cold_point {
                cold_request = Some((start + j, pair));
            }
            if predict || project || upper_possible || continuing_point || cold_point {
                requests.push(start + j);
                if p.refresh_returning && (predict || project) {
                    returning_features.push(pair);
                }
            }
            // Rotation can turn the cached normal, and sliding can change the
            // supporting features. Keep their geometry current at the original
            // public cadence while the named pair is in relative motion.
            if p.refresh || p.timing {
                p.tracking_geometry = pad.angular_velocity == [0.; 3]
                    && (predict
                        || p.tracking_geometry && native.has_box_contact(p.foot.body, p.pad.body));
                let moving = foot.angular_velocity != [0.; 3] || foot.velocity != pad.velocity;
                let pair = (p.foot.body, p.pad.body);
                if p.release_active
                    && (p.armed || predict || !p.ballistic && foot.angular_velocity == [0.; 3])
                {
                    p.release_active = false;
                }
                if p.release_active {
                    releases.push(pair);
                }
                if p.guard
                    && (p.armed
                        || upper_near
                        || predict
                        || clear
                        || p.tracking_geometry && moving
                        || p.release_active)
                {
                    guards.push(pair);
                    if let Some(id) = opponent {
                        let triple = (pair.0, pair.1, id);
                        box_opponents.push(triple);
                        if upper_near {
                            upper_corners.push(triple);
                            if p.project_opponent_return {
                                upper_returns.push(triple);
                            }
                            if p.project_opposing_normals {
                                opposing_normals.push(triple);
                            }
                        }
                    }
                    if p.detect_crossings {
                        sweep_bounds.push(pair);
                    }
                    if p.rotate {
                        rotations.push(pair);
                        if p.ballistic {
                            ballistic.push(pair);
                        }
                    }
                }
                if p.tracking_geometry && p.refresh && (predict || moving) {
                    fresh.push(pair);
                }
                if p.tracking_geometry && p.timing {
                    // After the first impact a different vertex can approach while the
                    // ordinary release detector is disarmed. Project from the live twist.
                    let mut continuation = true;
                    if predict
                        || moving
                            && geometry::predict_oriented(
                                &mut continuation,
                                (foot, p.foot.shape),
                                (pad, p.pad.shape),
                                gravity.0.into(),
                                time.dt.as_secs_f64(),
                                true,
                            )
                    {
                        timing_pairs.push(pair);
                        if p.refine_rotation_timing && rotations.contains(&pair) {
                            refined_timing.push(pair);
                            if p.rigid_corner {
                                rigid_corners.push(pair);
                                if p.rigid_corner_neighbors {
                                    corner_neighbors.push(pair);
                                }
                                if p.rigid_departure {
                                    rigid_departures.push(pair);
                                }
                            }
                            if p.compact_budget {
                                compact_budget.push(pair);
                            }
                            if let Some(mode) = p.rotation_pulse {
                                pulses.push((pair, mode));
                            }
                        }
                    }
                }
            }
        }
    }
    if policy.shares_faces {
        for group in policy.pairs.chunk_by(|a, b| a.foot.body == b.foot.body) {
            let mut active = group.iter().filter(|p| {
                p.share_coplanar_face && face_bearings.contains(&(p.foot.body, p.pad.body))
            });
            if let (Some(a), Some(b), None) = (active.next(), active.next(), active.next()) {
                // If only one material support changed, carry the surviving
                // site's previous share. The current pressure solve clamps it to
                // the *new* finite footprints before applying any reaction.
                let recent = b
                    .pressure_history
                    .as_ref()
                    .map(|s| s.fraction)
                    .or_else(|| a.pressure_history.as_ref().map(|s| 1. - s.fraction));
                shared_faces.push(gz_physics_multifidelity::SharedSupportRequest {
                    foot: a.foot.body,
                    supports: [a.pad.body, b.pad.body],
                    preferred_fraction_b: recent,
                    uniform_kinetic: a.share_uniform_sliding && b.share_uniform_sliding,
                    settle_comoving: a.settle_comoving_supports && b.settle_comoving_supports,
                });
            }
        }
    }
    let rebuilt;
    // A cold point alone can raise the ordinary one-call budget. Try the
    // ordinary whole tick first, after construction has installed the true
    // link twists. The native integrator commits that very trial only when
    // the swept, isolated tree qualifies. Otherwise it spends the requested
    // grid. Other refinement requests keep their chosen clock.
    if let Some((i, pair)) = cold_request
        && requests == [i]
        && timing_pairs.is_empty()
        && compact_points.is_empty()
    {
        compact_points.push(pair);
    }
    let selected = !requests.is_empty() || !timing_pairs.is_empty();
    // A simultaneous arrival, an existing native contact or a screw that needs
    // a faster clock keeps the ordinary authored impact budget.
    let calls = pad_rocking_steps
        .filter(|_| {
            rocking_requests > 0 && rocking_requests == requests.len() && timing_pairs.is_empty()
        })
        .unwrap_or(policy.calls);
    let (impact_calls, impact_solver) = policy
        .touching_solver
        .filter(|(calls, _)| {
            *calls >= floor
                && requests.len() == 1
                && requests.first().copied() == initial_face
                && native.active_contact_pairs() == 0
        })
        .unwrap_or((calls, policy.iterations));
    let early_face_schedule = if let (Some(window), Some(arrival)) =
        (policy.early_face, separated_face)
        && requests.len() == 1
        && native.active_contact_pairs() == 0
    {
        window.schedule(
            time.dt,
            arrival,
            impact_calls,
            impact_solver,
            native.ordinary_contact_iterations(),
        )
    } else {
        None
    };
    let mut pad_impacts = Vec::new();
    if requests
        .iter()
        .any(|&i| policy.pairs[i].returning_pad.is_some())
    {
        pad_impacts.extend(
            policy
                .pairs
                .iter()
                .filter(|p| {
                    p.returning_pad
                        .is_some_and(|id| native.joint_child(id) == Some(p.pad.body))
                        && p.foot.present(&links)
                        && native.is_dynamic(p.foot.body) == Some(true)
                        && p.pad.present(&links)
                })
                .map(|p| PadImpact::new(p.foot.body, p.pad.body)),
        );
    }
    let observe = observer.is_some() || !pad_impacts.is_empty();
    let mut sample = |step: gz_physics_multifidelity::ContactSubstep<'_>| {
        PadImpact::sample_all(&mut pad_impacts, &step);
        if let Some(external) = observer.as_deref_mut() {
            external.sample(*time, step);
        }
    };
    let mut released_face_schedule = None;
    let receipt = if !returning_features.is_empty()
        || !timing_pairs.is_empty()
        || !projections.is_empty()
        || !patches.is_empty()
        || !edges.is_empty()
        || !mixed.is_empty()
        || !all_edges.is_empty()
        || !finite.is_empty()
        || !segments.is_empty()
        || !soles.is_empty()
        || !guards.is_empty()
        || !sustained.is_empty()
        || !two_bearing.is_empty()
        || !face_bearings.is_empty()
    {
        if requests.iter().any(|&i| {
            !compact_budget.contains(&(policy.pairs[i].foot.body, policy.pairs[i].pad.body))
        }) {
            compact_budget.clear();
        }
        if !upper_corners.is_empty() {
            compact_budget.clear();
        }
        let actions = gz_physics_multifidelity::BoxContactActions {
            fresh_features: &fresh,
            returning_features: &returning_features,
            prospective_timing: &timing_pairs,
            corner_impacts: &projections,
            planar_impacts: &patches,
            sustained_edges: &sustained,
            sliding_vertices: &vertices,
            sticking_points: &sticking_points,
            point_transfers: &point_transfers,
            sliding_skew_edges: &skew_slides,
            slip_launches: &slip_launches,
            subdivide_point_bounds: &subdivided,
            compact_point_tails: &compact_points,
            target_point_bounds: &directed,
            two_bearing_edges: &two_bearing,
            rocking_limits: &rocking_limits,
            face_bearings: &face_bearings,
            deck_edge_bearings: &deck_edge_bearings,
            shared_faces: &shared_faces,
            sliding_edge_impacts: &edges,
            mixed_edge_impacts: &mixed,
            edge_impacts: &all_edges,
            finite_point_impacts: &finite,
            certify_face_vertices: &certified_vertices,
            certify_skew_edges: &certified_edges,
            fractional_point_paths: &fractional_points,
            finite_segment_impacts: &segments,
            certify_segment_friction_work: &segment_work,
            frictionless_sole_impacts: &soles,
            retime_finite_transfer: &transfer,
            opposing_box_guards: &box_opponents,
            project_opposing_corners: &upper_corners,
            project_opposing_returns: &upper_returns,
            project_opposing_normals: &opposing_normals,
            guard_separated_flight: &guards,
            integrate_free_rotation: &rotations,
            integrate_ballistic_translation: &ballistic,
            integrate_released_rotation: &releases,
            swept_box_bounds: &sweep_bounds,
            compact_impact_budget: &compact_budget,
            project_rigid_corner: &rigid_corners,
            rigid_corner_with_neighbors: &corner_neighbors,
            follow_rigid_departure: &rigid_departures,
            refine_rotation_timing: &refined_timing,
            rotation_contact_pulses: &pulses,
        };
        let (calls, iterations) = if selected {
            (impact_calls, impact_solver)
        } else {
            (floor, native.ordinary_contact_iterations())
        };
        let result = if observe && selected {
            native.advance_box_contacts(time.dt, calls, iterations, actions, Some(&mut sample))
        } else {
            native.advance_box_contacts(time.dt, calls, iterations, actions, None)
        };
        match result {
            Ok(r) => {
                if !upper_corners.is_empty() {
                    policy.last_upper = Some(OpposingCornerInterval {
                        epoch: time.epoch,
                        iteration: time.iteration,
                        sim_time: time.sim_time,
                        interval: r.interval.clone(),
                    });
                }
                let returning_rebuilds = r
                    .returning_features
                    .iter()
                    .map(|r| r.counts.requery)
                    .sum::<u64>();
                rebuilt = r.rebuilt_manifolds - returning_rebuilds;
                if !r.returning_features.is_empty() {
                    policy.last_returning = Some(ReturningBoxFeatures {
                        epoch: time.epoch,
                        iteration: time.iteration,
                        sim_time: time.sim_time,
                        pairs: r.returning_features,
                    });
                }
                if let Some(flight) = r.flight {
                    continued = flight
                        .released
                        .as_ref()
                        .and_then(|v| v.last())
                        .is_some_and(|s| s.integrated);
                    policy.last_flight = Some(BoxFlightInterval {
                        epoch: time.epoch,
                        iteration: time.iteration,
                        sim_time: time.sim_time,
                        pairs: guards,
                        flight,
                    });
                }

                if let Some(timing) = r.timing {
                    policy.last_timing = Some(BoxTimingInterval {
                        epoch: time.epoch,
                        iteration: time.iteration,
                        sim_time: time.sim_time,
                        pairs: timing_pairs,
                        timing,
                        interval: r.interval.clone(),
                    });
                }
                Some(r.interval)
            }
            Err(e) => {
                fault.report(e);
                return;
            }
        }
    } else if requests.is_empty() && floor == 1 {
        let result = if fresh.is_empty() {
            native.step(time.dt).map(|_| 0)
        } else {
            native.step_box_features(time.dt, &fresh)
        };
        match result {
            Ok(n) => rebuilt = n,
            Err(e) => {
                fault.report(e);
                return;
            }
        }
        None
    } else {
        let (calls, iterations) = if requests.is_empty() {
            (floor, native.ordinary_contact_iterations())
        } else {
            (impact_calls, impact_solver)
        };
        let staged =
            early_face_schedule.and_then(|s| policy.early_face?.after_release(s, separated_face?));
        let outcome = if let Some((segments, fallback)) = staged {
            let pair = policy.pairs[requests[0]].key();
            let result = if observe {
                native.step_after_box_release(
                    time.dt,
                    &segments,
                    pair,
                    fallback,
                    &fresh,
                    Some(&mut sample),
                )
            } else {
                native.step_after_box_release(time.dt, &segments, pair, fallback, &fresh, None)
            };
            result.map(|(r, n)| {
                if r.native_steps == segments.iter().map(|s| s.native_steps).sum::<u32>() {
                    released_face_schedule = Some(segments);
                }
                (r, n)
            })
        } else {
            selected_interval(
                native,
                *time,
                ContactSegment {
                    duration: time.dt,
                    native_steps: calls,
                    iterations,
                },
                early_face_schedule.as_ref(),
                &fresh,
                observe.then_some(&mut sample),
            )
        };
        match outcome {
            Ok((r, n)) => {
                rebuilt = n;
                Some(r)
            }
            Err(e) => {
                fault.report(e);
                return;
            }
        }
    };
    if !sweeps.is_empty() {
        policy.last_sweeps = Some(BoxSweepInterval {
            epoch: time.epoch,
            iteration: time.iteration,
            sim_time: time.sim_time,
            pairs: sweeps,
        });
    }
    if policy.follows_release
        && let Some(r) = &receipt
    {
        for (i, p) in policy
            .pairs
            .iter_mut()
            .enumerate()
            .filter(|(_, p)| p.follow_release)
        {
            if p.release_active && !continued {
                p.release_active = false;
            }
            if requests.contains(&i)
                && r.pairs
                    .iter()
                    .any(|c| (c.a, c.b) == p.key() && c.normal_impulse > 0.)
            {
                p.release_active = true;
            }
        }
    }
    if receipt.is_none() && policy.shares_faces {
        for p in policy
            .pairs
            .iter_mut()
            .filter(|p| p.pressure_history.is_some())
        {
            if native.is_dynamic(p.foot.body) == Some(true) {
                p.pressure_history = None;
            }
        }
    }
    if let Some(r) = &receipt {
        for (i, p) in policy.pairs.iter_mut().enumerate().filter(|(_, p)| {
            p.follow_planar_edge
                || p.follow_frictionless_edge
                || p.follow_sliding_vertex
                || p.follow_sticking_point
                || p.resolve_point_transfers
                || p.follow_sliding_skew_edge
                || p.follow_skew_slip_launch
                || p.follow_frictionless_face
        }) {
            let pair = (p.foot.body, p.pad.body);
            let end = r
                .sustained_edges
                .iter()
                .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = r.planar_impacts.iter().any(|s| {
                (s.resultant.foot, s.resultant.support) == pair
                    && s.bearing != gz_physics::Bearing::Face
                    && s.resultant.elapsed == time.dt
            });
            let end = end
                || r.two_bearing_edges
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.follow_frictionless_edge
                    && r.edge_impacts.iter().any(|hit| {
                        use gz_physics_multifidelity::EdgeImpactReceipt;
                        let sites = match hit {
                            EdgeImpactReceipt::BothSlide(s) => Some(&s.sites),
                            EdgeImpactReceipt::Recontact(s)
                                if s.state == [gz_physics::NormalSiteState::Loaded; 2] =>
                            {
                                Some(&s.sites)
                            }
                            _ => None,
                        };
                        sites.is_some_and(|s| {
                            (s[0].foot, s[0].support) == pair
                                && s[0].friction == 0.
                                && s[0].elapsed == time.dt
                        })
                    }));
            let end = end
                || r.sole_transfers
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.follow_frictionless_edge
                    && r.face_pressure_impacts.iter().any(|s| {
                        (s.foot, s.support) == pair
                            && s.elapsed == time.dt
                            && s.after_normal.iter().filter(|v| **v == 0.).count() == 2
                    }));
            let end = end
                || r.rocking_limits
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let end = end
                || r.sliding_skew_edges
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.follow_skew_slip_launch
                    && r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && s.resultant.elapsed == time.dt
                    }));
            let late = late
                || (p.follow_sliding_skew_edge
                    && r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && !s.resultant.sticking
                            && s.resultant.elapsed == time.dt
                    }));
            let end = end
                || r.point_transfers
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.resolve_point_transfers
                    && r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && s.resultant.elapsed == time.dt
                    }));
            let end = end
                || r.sticking_points
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.follow_sticking_point
                    && r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && s.resultant.sticking
                            && s.resultant.elapsed == time.dt
                    }));
            let end = end
                || r.sliding_vertices
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let late = late
                || (p.follow_sliding_vertex
                    && r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && !s.resultant.sticking
                            && s.resultant.elapsed == time.dt
                    }));
            let end = end
                || r.face_bearings
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let end = end
                || r.deck_edge_bearings
                    .iter()
                    .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt);
            let shared = r
                .shared_faces
                .iter()
                .rev()
                .filter(|s| s.foot == p.foot.body && s.start + s.duration == time.dt)
                .find_map(|s| {
                    s.loads
                        .iter()
                        .position(|v| v.support == p.pad.body)
                        .map(|i| {
                            if i == 1 {
                                s.selected_fraction_b
                            } else {
                                1. - s.selected_fraction_b
                            }
                        })
                });
            let single = r
                .face_bearings
                .iter()
                .rev()
                .find(|v| (v.foot, v.support) == pair && v.start + v.duration == time.dt);
            if p.share_coplanar_face && native.is_dynamic(p.foot.body) == Some(true) {
                let fraction = shared.or_else(|| single.map(|_| 1.));
                if let (Some(fraction), Some(foot), Some(pad)) = (
                    fraction,
                    native.state(p.foot.body),
                    native.state(p.pad.body),
                ) {
                    let pressure = SupportPressure {
                        fraction,
                        foot,
                        pad,
                    };
                    match &mut p.pressure_history {
                        Some(stored) => **stored = pressure,
                        None => p.pressure_history = Some(Box::new(pressure)),
                    }
                } else {
                    p.pressure_history = None;
                }
            }
            let fractional_end = r
                .fractional_point_paths
                .iter()
                .any(|s| (s.foot, s.support) == pair && s.native_boundary == time.dt);
            let end = end || shared.is_some() || fractional_end;
            p.point_at_previous_end = p.refine_active_point
                && (fractional_end
                    || r.sliding_vertices
                        .iter()
                        .chain(&r.sliding_skew_edges)
                        .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt)
                    || r.sticking_points
                        .iter()
                        .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt)
                    || r.point_transfers
                        .iter()
                        .any(|s| (s.foot, s.support) == pair && s.start + s.duration == time.dt)
                    || r.finite_point_impacts.iter().any(|s| {
                        (s.resultant.foot, s.resultant.support) == pair
                            && s.resultant.elapsed == time.dt
                    }));
            p.edge_active = end || late;
            if end && !requests.contains(&i) {
                requests.push(i);
            }
        }
    }
    if !fresh.is_empty() {
        policy.last_geometry = Some(BoxFeatureRefresh {
            epoch: time.epoch,
            iteration: time.iteration,
            sim_time: time.sim_time,
            native_steps: receipt.as_ref().map_or(1, |r| r.native_steps),
            pairs: fresh,
            rebuilt_manifolds: rebuilt,
        });
    }
    if receipt.is_none() && !policy.pairs.iter().any(|p| p.armed) {
        return;
    }
    let last = native.rigid_contact_observations();
    for p in policy.pairs.iter_mut().filter(|p| p.armed) {
        let key = p.key();
        let hit = match &receipt {
            Some(r) => r
                .pairs
                .iter()
                .any(|c| (c.a, c.b) == key && c.normal_impulse > 0.),
            None => last
                .iter()
                .any(|c| (c.a, c.b) == key && c.normal_impulse > 0.),
        };
        if hit {
            p.armed = false;
        }
    }
    if let Some(interval) = receipt
        && (!requests.is_empty() || floor > 1)
    {
        let solver = if requests.is_empty() {
            native.ordinary_contact_iterations()
        } else {
            impact_solver
        };
        let requested_pairs = requests
            .into_iter()
            .map(|i| {
                let p = &mut policy.pairs[i];
                let key = p.key();
                let integrated = interval.pairs.iter().find(|c| (c.a, c.b) == key);
                // Acceleration inside the interval may turn a predicted arrival
                // into free flight. A later strike can still start at tangency.
                if p.incoming_face.is_some() {
                    p.strike_latched = integrated.is_some_and(|c| c.normal_impulse > 0.);
                }
                RequestedPair {
                    foot_link: p.foot.id,
                    support_link: p.pad.id,
                    foot: p.foot.body,
                    support: p.pad.body,
                    normal_impulse: integrated.map_or(0., |c| c.normal_impulse),
                    normal_impulse_on_foot: integrated.map_or([0.; 3], |c| {
                        c.normal_impulse_on_a
                            .map(|v| if c.a == p.foot.body { v } else { -v })
                    }),
                    tangent_impulse_on_foot: integrated.map_or(Some([0.; 3]), |c| {
                        c.tangent_impulse_on_a
                            .map(|t| t.map(|v| if c.a == p.foot.body { v } else { -v }))
                    }),
                    first_positive_end: integrated.and_then(|c| c.first_positive_end),
                    active_at_end: integrated.is_some_and(|c| c.active_at_end),
                    last_native_normal_impulse: last
                        .iter()
                        .filter(|c| (c.a, c.b) == key)
                        .map(|c| c.normal_impulse)
                        .sum(),
                }
            })
            .collect();
        policy.last = Some(Refinement {
            speed_stop_floor: floor,
            epoch: time.epoch,
            iteration: time.iteration,
            start_time: time.start_time,
            sim_time: time.sim_time,
            requested_pairs,
            pad_impacts,
            early_face_schedule: early_face_schedule.filter(|_| released_face_schedule.is_none()),
            released_face_schedule,
            solver,
            interval,
        });
    }
}
