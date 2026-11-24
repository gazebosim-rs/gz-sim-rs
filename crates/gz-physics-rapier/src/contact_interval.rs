// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Integrate public-tick forces once while refreshing native rigid contacts multiple times.
use super::*;
use crate::box_features::BoxFeatures;
use crate::box_timing::{BoxTiming, FlightGuard};
use crate::joints::{CornerFlight, ProjectionGeometry};
use gz_physics::NativeStepGrid;

/// Native solver work for a single public contact interval. The temporal
/// iterations divide integration time. Biased PGS and unbiased relaxation
/// passes solve constraints at fixed times. Unspecified values inherit the
/// world's settings. Changes are scoped to the requested interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactSolverIterations {
    pub temporal: usize,
    pub pgs: Option<usize>,
    pub relaxation: Option<usize>,
}

impl From<usize> for ContactSolverIterations {
    fn from(temporal: usize) -> Self {
        Self {
            temporal,
            pgs: None,
            relaxation: None,
        }
    }
}

impl ContactSolverIterations {
    fn valid(self) -> bool {
        (1..=32).contains(&self.temporal)
            && self.pgs.is_none_or(|n| (1..=32).contains(&n))
            && self.relaxation.is_none_or(|n| n <= 32)
    }
}

/// A contiguous portion of one public tick with its own native solve budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactSegment {
    pub duration: Duration,
    pub native_steps: u32,
    pub iterations: ContactSolverIterations,
}

#[derive(Clone, Copy)]
struct PreparedSegment {
    grid: NativeStepGrid,
    iterations: ContactSolverIterations,
}

/// Optional finite-box operations on one advancing public interval. Pair order
/// is dynamic sole followed by support. Every other body shares this time grid.
#[derive(Clone, Copy, Debug, Default)]
pub struct BoxContactActions<'a> {
    /// At most one native requery on each named pair with a geometrically disproved, fully unloaded manifold.
    pub returning_features: &'a [(BodyId, BodyId)],
    pub fresh_features: &'a [(BodyId, BodyId)],
    pub prospective_timing: &'a [(BodyId, BodyId)],
    /// During proven separation, suppress only this sole/pad pair until the native boundary.
    pub guard_separated_flight: &'a [(BodyId, BodyId)],
    /// Integrate the torque-free rotation of this subset of separated rigid pairs.
    pub integrate_free_rotation: &'a [(BodyId, BodyId)],
    /// On a truly separated regular body, advance the inertial COM under uniform gravity.
    pub integrate_ballistic_translation: &'a [(BodyId, BodyId)],
    /// Request strict torque-free continuation after an observed release.
    pub integrate_released_rotation: &'a [(BodyId, BodyId)],
    /// Carry the public sweep's vertex bounds into the native footprint and separated-flight checks.
    pub swept_box_bounds: &'a [(BodyId, BodyId)],
    /// End the native contact budget once the measured remaining flight is proven.
    pub compact_impact_budget: &'a [(BodyId, BodyId)],
    /// Apply a single cold spatial corner law in a verified isolated rigid-body pulse.
    pub project_rigid_corner: &'a [(BodyId, BodyId)],
    /// Admit other scene geometry only after its entire swept interval is excluded.
    pub rigid_corner_with_neighbors: &'a [(BodyId, BodyId)],
    /// Registered (foot, primary pad, opposing pad). Only proved intervals filter both pairs.
    pub opposing_box_guards: &'a [(BodyId, BodyId, BodyId)],
    /// Explicit opt-in to isolated upper-corner impacts for this (foot, lower, upper) triple.
    pub project_opposing_corners: &'a [(BodyId, BodyId, BodyId)],
    /// Permit an isolated lower return after a witnessed upper strike during this public tick.
    pub project_opposing_returns: &'a [(BodyId, BodyId, BodyId)],
    /// Apply an enclosed frictionless two-site impulse after a measured lower hit.
    pub project_opposing_normals: &'a [(BodyId, BodyId, BodyId)],
    /// Permit one newly certified outgoing interval anchored at the preceding native projection.
    pub follow_rigid_departure: &'a [(BodyId, BodyId)],
    /// Bound crossing time using the measured Euler derivatives of each sole vertex.
    pub refine_rotation_timing: &'a [(BodyId, BodyId)],
    /// Post-bracket pulse policy, specific to its registered cubic-timing pair.
    pub rotation_contact_pulses: &'a [((BodyId, BodyId), RotationContactPulse)],
    pub corner_impacts: &'a [(BodyId, BodyId)],
    /// Reflection-symmetric locked sole patches on registered flat pads.
    pub planar_impacts: &'a [(BodyId, BodyId)],
    /// An isolated sliding pad edge can carry load between public updates.
    pub sustained_edges: &'a [(BodyId, BodyId)],
    /// Sustain a strictly sliding vertex on an isolated finite support face.
    pub sliding_vertices: &'a [(BodyId, BodyId)],
    /// Sustain a strictly interior static friction force at one material point.
    pub sticking_points: &'a [(BodyId, BodyId)],
    /// Carry a finite material point through isolated, pressure-certified neighbor impacts.
    pub point_transfers: &'a [(BodyId, BodyId)],
    /// Stage shorter certified spans when a whole transfer interval cannot be proved.
    /// Try proof horizons near separately predicted material-bearing changes.
    pub target_point_bounds: &'a [(BodyId, BodyId)],
    pub compact_point_tails: &'a [(BodyId, BodyId)],
    /// Retry whole-span bounds on successively shorter ranges.
    pub subdivide_point_bounds: &'a [(BodyId, BodyId)],
    /// Follow a separating slip velocity from zero on two moving material edges.
    pub slip_launches: &'a [(BodyId, BodyId)],
    /// Sustain the migrating point on two nonparallel finite sliding edges.
    pub sliding_skew_edges: &'a [(BodyId, BodyId)],
    /// Follow a positively loaded frictionless two-site edge until its first boundary.
    pub two_bearing_edges: &'a [(BodyId, BodyId)],
    /// Resolve a symmetric frictionless sole's repeated rocking and finite settling time.
    pub rocking_limits: &'a [(BodyId, BodyId)],
    /// Smooth support by an interior pressure centroid on a clipped two-box face.
    pub face_bearings: &'a [(BodyId, BodyId)],
    /// Continue the same enrolled face along a finite pad-owned line when pressure exits.
    pub deck_edge_bearings: &'a [(BodyId, BodyId)],
    pub shared_faces: &'a [SharedSupportRequest],
    /// Frictionless sole edge or complete bottom face on the interior of the pad.
    pub frictionless_sole_impacts: &'a [(BodyId, BodyId)],
    /// Two co-level sliding vertices on a cold finite pad.
    pub sliding_edge_impacts: &'a [(BodyId, BodyId)],
    /// An isolated material vertex/face or skew edge/edge pair on finite boxes.
    pub finite_point_impacts: &'a [(BodyId, BodyId)],
    /// Enclose each competing material vertex throughout the final nanosecond of a finite face arrival.
    pub certify_face_vertices: &'a [(BodyId, BodyId)],
    /// Require a bounded single-site outgoing skew-edge response through the integer bracket.
    /// Stage subnanosecond point impacts and sustained reactions before committing their integer endpoint.
    pub fractional_point_paths: &'a [(BodyId, BodyId)],
    pub certify_skew_edges: &'a [(BodyId, BodyId)],
    /// Exactly incident finite face and edge with a certified single carrying endpoint.
    /// Resolve uncertain cold line pressure using a bounded Coulomb-work exclusion.
    pub certify_segment_friction_work: &'a [(BodyId, BodyId)],
    pub finite_segment_impacts: &'a [(BodyId, BodyId)],
    /// Retime native calls at finite feature changes. With directed point
    /// bounds and compact tails, a proved sliding prefix can hand off an
    /// approaching line to the native solver within the existing budget.
    pub retime_finite_transfer: &'a [(BodyId, BodyId)],
    /// Two simultaneous loaded vertices with at least one sticking endpoint.
    pub mixed_edge_impacts: &'a [(BodyId, BodyId)],
    /// A certified two-vertex active set on a cold cuboid edge.
    pub edge_impacts: &'a [(BodyId, BodyId)],
}
impl<'a> BoxContactActions<'a> {
    fn point_modes(
        &self,
        pair: (BodyId, BodyId),
        remember_endpoint: bool,
    ) -> crate::joints::PointFollowModes {
        let transfer = self.point_transfers.contains(&pair);
        crate::joints::PointFollowModes {
            remember_endpoint,
            before_line: false,
            vertex: transfer || self.sliding_vertices.contains(&pair),
            sticking: transfer || self.sticking_points.contains(&pair),
            transfer,
            skew: self.sliding_skew_edges.contains(&pair),
            launch: self.slip_launches.contains(&pair),
            subdivide: self.subdivide_point_bounds.contains(&pair),
            directed: self.target_point_bounds.contains(&pair),
        }
    }

    fn projections(self) -> [(ProjectionGeometry, &'a [(BodyId, BodyId)]); 8] {
        [
            (ProjectionGeometry::Corner, self.corner_impacts),
            (ProjectionGeometry::Planar, self.planar_impacts),
            (ProjectionGeometry::SlidingEdge, self.sliding_edge_impacts),
            (ProjectionGeometry::MixedEdge, self.mixed_edge_impacts),
            (ProjectionGeometry::Edge, self.edge_impacts),
            (
                ProjectionGeometry::FrictionlessSole,
                self.frictionless_sole_impacts,
            ),
            (ProjectionGeometry::Finite, self.finite_point_impacts),
            (
                ProjectionGeometry::FiniteSegment,
                self.finite_segment_impacts,
            ),
        ]
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoxContactOutcome {
    pub interval: ContactIntervalReceipt,
    pub rebuilt_manifolds: u64,
    pub returning_features: Vec<crate::ReturningBoxFeatureReceipt>,
    pub timing: Option<BoxTimingReceipt>,
    pub flight: Option<BoxFlightReceipt>,
}

/// Contact history for one pair over a single advancing public tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactIntervalPair {
    pub a: BodyId,
    pub b: BodyId,
    /// Sum of delivered compressive impulses across the native solves, in N·s.
    pub normal_impulse: f64,
    /// Vector sum of compressive reactions on body a, in world axes and N·s.
    pub normal_impulse_on_a: [f64; 3],
    /// Applied world-space friction impulse on a in N·s. A grouped rigid
    /// manifold with unavailable applied tangent data makes this None.
    pub tangent_impulse_on_a: Option<[f64; 3]>,
    /// Time from tick start to the first positive reaction. A projected
    /// impulse has its application time, a native reaction its first loaded solve end.
    pub first_positive_end: Option<Duration>,
    /// Whether this pair had active contacts in the final native solve.
    pub active_at_end: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CompactPointTail {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub omitted_native_steps: u32,
}
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactIntervalReceipt {
    pub native_steps: u32,
    /// A fully staged isolated suffix replaced the unused part of the native grid.
    pub compact_point_tail: Option<CompactPointTail>,
    /// Isolated impact impulses, with their application times inside this public tick.
    pub corner_impacts: Vec<CornerImpactReceipt>,
    pub rigid_point_impacts: Vec<RigidPointImpactReceipt>,
    pub opposing_corner_impacts: Vec<OpposingCornerReceipt>,
    pub opposing_return_impacts: Vec<OpposingReturnReceipt>,
    pub opposing_normal_series: Vec<OpposingNormalReceipt>,
    pub opposing_normal_declines: Vec<OpposingNormalDecline>,
    pub planar_impacts: Vec<PlanarImpactReceipt>,
    pub sustained_edges: Vec<SustainedEdgeReceipt>,
    pub sliding_vertices: Vec<SlidingVertexReceipt>,
    pub sticking_points: Vec<StickingPointReceipt>,
    pub point_transfers: Vec<BoundTransferReceipt>,
    /// Sustain the migrating point on two nonparallel finite sliding edges.
    pub sliding_skew_edges: Vec<SlidingPointReceipt>,
    pub two_bearing_edges: Vec<TwoBearingReceipt>,
    pub sole_transfers: Vec<SoleTransferReceipt>,
    pub rocking_limits: Vec<RockingLimitReceipt>,
    pub face_bearings: Vec<FaceBearingReceipt>,
    pub deck_edge_bearings: Vec<DeckEdgeBearingReceipt>,
    pub shared_faces: Vec<SharedFaceReceipt>,
    pub face_pressure_impacts: Vec<FacePressureReceipt>,
    pub sliding_edge_impacts: Vec<SlidingEdgeImpactReceipt>,
    pub finite_point_impacts: Vec<FiniteImpactReceipt>,
    pub fractional_point_paths: Vec<FractionalPointReceipt>,
    pub mixed_edge_impacts: Vec<MixedEdgeImpactReceipt>,
    pub edge_impacts: Vec<EdgeImpactReceipt>,
    /// Sorted by public body identifier pair, including speculative pairs.
    pub pairs: Vec<ContactIntervalPair>,
}

/// Read-only state immediately after one native contact solve.
pub struct ContactSubstep<'a> {
    pub elapsed: Duration,
    pub duration: Duration,
    solver: &'a RapierBackend,
    /// Newly committed finite-feature impacts, including a time-zero impact on
    /// the first callback. Subsequent callbacks never repeat a boundary impact.
    pub finite_point_impacts: &'a [FiniteImpactReceipt],
    pub fractional_point_path: Option<&'a FractionalPointReceipt>,
    /// A sustained material load committed during this solve, if present.
    pub point_span: Option<&'a crate::PointSpanLoad>,
    /// Instantaneous regular-rigid reaction inside this native interval.
    pub rigid_point_impact: Option<RigidPointImpactReceipt>,
    pub opposing_corner_impact: Option<OpposingCornerReceipt>,
    pub opposing_return_impact: Option<&'a OpposingReturnReceipt>,
    pub opposing_normal_series: Option<&'a OpposingNormalReceipt>,
}
impl ContactSubstep<'_> {
    pub fn state(&self, id: BodyId) -> Option<BodyState> {
        let entry = self.solver.bodies.get(&id)?;
        let mut state = self.solver.state_from_entry(entry);
        if let Some((linear, angular)) = self.solver.generalized_link_twist(entry.0) {
            state.velocity = linear.to_array();
            state.angular_velocity = angular.to_array();
        }
        Some(state)
    }
    #[cfg(feature = "landing-audit")]
    pub fn landing_solver_points(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Vec<crate::LandingSolverPoint>, PhysicsError> {
        self.solver.landing_solver_points(foot, pad)
    }

    pub fn contact_patches(&self) -> Vec<RigidContactPatch> {
        self.solver.rigid_contact_patches()
    }
}

#[derive(Default)]
struct CornerPlan<'a> {
    actions: BoxContactActions<'a>,
    flight: Option<CornerFlight>,
    fractional: Option<Box<crate::joints::PreparedFractional>>,
    guard: Option<&'a mut FlightGuard>,
    upper: Option<crate::box_timing::UpperPair>,
}

struct Integration<'plan, 'observer> {
    observer: Option<&'observer mut dyn FnMut(ContactSubstep<'_>)>,
    segments: &'plan [PreparedSegment],
    live: Option<&'plan mut BoxTiming>,
    project: CornerPlan<'plan>,
    projected: Vec<CornerImpactReceipt>,
    rigid_points: Vec<RigidPointImpactReceipt>,
    upper_points: Vec<OpposingCornerReceipt>,
    upper_returns: Vec<OpposingReturnReceipt>,
    normal_series: Vec<OpposingNormalReceipt>,
    normal_declines: Vec<OpposingNormalDecline>,
    planar: Vec<PlanarImpactReceipt>,
    sustained: Vec<SustainedEdgeReceipt>,
    vertices: Vec<SlidingVertexReceipt>,
    sticking_points: Vec<StickingPointReceipt>,
    point_transfers: Vec<BoundTransferReceipt>,
    sliding_skew_edges: Vec<SlidingPointReceipt>,
    two_bearing: Vec<TwoBearingReceipt>,
    sole_transfers: Vec<SoleTransferReceipt>,
    rocking_limits: Vec<RockingLimitReceipt>,
    face_bearings: Vec<FaceBearingReceipt>,
    deck_edge_bearings: Vec<DeckEdgeBearingReceipt>,
    shared_faces: Vec<SharedFaceReceipt>,
    pressure: Vec<FacePressureReceipt>,
    edges: Vec<SlidingEdgeImpactReceipt>,
    mixed: Vec<MixedEdgeImpactReceipt>,
    all_edges: Vec<EdgeImpactReceipt>,
    finite: Vec<FiniteImpactReceipt>,
    fractional: Vec<FractionalPointReceipt>,
    observed_finite: usize,
    steps: u32,
    compact_point_tail: Option<CompactPointTail>,
    body_index: partition::ContactBodyIndex,
    pairs: BTreeMap<(BodyId, BodyId), (ContactIntervalPair, u32)>,
}
struct SubstepReactions<'a> {
    rigid: Option<RigidPointImpactReceipt>,
    corner: Option<OpposingCornerReceipt>,
    returned: Option<&'a OpposingReturnReceipt>,
    normals: Option<&'a OpposingNormalReceipt>,
    span: Option<&'a crate::PointSpanLoad>,
}
impl Integration<'_, '_> {
    fn observe(
        &mut self,
        solver: &RapierBackend,
        elapsed: Duration,
        duration: Duration,
        reactions: SubstepReactions<'_>,
    ) {
        if let Some(observer) = &mut self.observer {
            observer(ContactSubstep {
                fractional_point_path: self
                    .fractional
                    .last()
                    .filter(|r| r.native_boundary == elapsed),
                finite_point_impacts: &self.finite[self.observed_finite..],
                point_span: reactions.span,
                rigid_point_impact: reactions.rigid,
                opposing_corner_impact: reactions.corner,
                opposing_return_impact: reactions.returned,
                opposing_normal_series: reactions.normals,
                solver,
                elapsed,
                duration,
            });
            self.observed_finite = self.finite.len();
        }
        self.steps += 1;
        let mut measured = Vec::new();
        for pair in solver.world.narrow_phase.contact_pairs() {
            if let Some((reaction, normal, tangent)) =
                solver.interval_impulse(pair, &self.body_index)
            {
                self.add_vectors(reaction, normal, tangent);
                measured.push(reaction);
            }
        }
        // Use the same observation type and sort as the existing public scalar
        // path. Equal keys on compound bodies retain its floating-point order.
        measured.sort_unstable_by_key(|p| (p.a, p.b));
        for p in measured {
            self.add_scalar(p, elapsed);
        }
    }
    fn add_vectors(&mut self, p: RigidContactObservation, normal: Vector, tangent: Option<Vector>) {
        let (entry, _) = self.pairs.entry((p.a, p.b)).or_insert((
            ContactIntervalPair {
                a: p.a,
                b: p.b,
                normal_impulse: 0.,
                normal_impulse_on_a: [0.; 3],
                tangent_impulse_on_a: Some([0.; 3]),
                first_positive_end: None,
                active_at_end: false,
            },
            0,
        ));
        entry.normal_impulse_on_a =
            (Vector::from_array(entry.normal_impulse_on_a) + normal).to_array();
        entry.tangent_impulse_on_a = entry
            .tangent_impulse_on_a
            .zip(tangent)
            .map(|(a, b)| (Vector::from_array(a) + b).to_array());
    }
    fn add_scalar(&mut self, p: RigidContactObservation, elapsed: Duration) {
        let (entry, last) = self
            .pairs
            .get_mut(&(p.a, p.b))
            .expect("contact vectors registered the native pair");
        if p.normal_impulse > 0. {
            entry.first_positive_end.get_or_insert(elapsed);
        }
        entry.normal_impulse += p.normal_impulse;
        entry.active_at_end = p.active || (*last == self.steps && entry.active_at_end);
        *last = self.steps;
    }
    fn add_projected(&mut self, hit: CornerImpactReceipt) {
        self.add_resultant(hit);
        self.projected.push(hit);
    }
    fn add_planar(&mut self, hit: PlanarImpactReceipt) {
        self.add_resultant(hit.resultant);
        self.planar.push(hit);
    }
    fn add_geometry(
        &mut self,
        solver: &mut RapierBackend,
        geometry: ProjectionGeometry,
        pair: (BodyId, BodyId),
        elapsed: Duration,
        allowance: f64,
        finite_witness: Option<gz_physics::FiniteContactWitness>,
    ) -> Result<(), PhysicsError> {
        let (foot, pad) = pair;
        match geometry {
            ProjectionGeometry::Corner => {
                if let Some(h) = solver.apply_corner_at(foot, pad, elapsed, allowance)? {
                    self.add_projected(h);
                }
            }
            ProjectionGeometry::Planar => {
                if let Some(h) = solver.apply_planar_at(foot, pad, elapsed, allowance)? {
                    self.add_planar(h);
                }
            }
            ProjectionGeometry::FiniteSegment => {
                let work = self
                    .project
                    .actions
                    .certify_segment_friction_work
                    .contains(&(foot, pad));
                if let Some(h) = solver.apply_finite_segment(foot, pad, elapsed, work)? {
                    self.add_resultant(h.resultant);
                    self.finite.push(h);
                }
            }
            ProjectionGeometry::Finite => {
                if let Some(h) =
                    solver.apply_finite_at(foot, pad, elapsed, allowance, finite_witness)?
                {
                    self.add_resultant(h.resultant);
                    self.finite.push(h);
                }
            }
            ProjectionGeometry::FrictionlessSole | ProjectionGeometry::Edge => {
                if geometry == ProjectionGeometry::FrictionlessSole {
                    if !solver.frictionless_registered(foot, pad)? {
                        return Ok(());
                    }
                    if let Some(h) = solver.apply_normal_face_at(foot, pad, elapsed, allowance)? {
                        let n = Vector::from(h.normal);
                        self.add_impact(
                            foot,
                            pad,
                            elapsed,
                            n,
                            h.normal_impulse * n,
                            h.normal_impulse,
                        );
                        self.pressure.push(h);
                        return Ok(());
                    }
                }
                if let Some(h) = solver.apply_edge_at(foot, pad, elapsed, allowance)? {
                    match h {
                        EdgeImpactReceipt::Recontact(s) => {
                            for site in s.sites {
                                self.add_resultant(site);
                            }
                        }
                        EdgeImpactReceipt::OneLoaded(s) => self.add_resultant(s.loaded),
                        EdgeImpactReceipt::Sticking(s) => self.add_mixed_resultant(s),
                        EdgeImpactReceipt::BothSlide(s) => {
                            for site in s.sites {
                                self.add_resultant(site);
                            }
                        }
                    }
                    self.all_edges.push(h);
                }
            }
            ProjectionGeometry::MixedEdge => {
                if let Some(h) = solver.apply_mixed_edge_at(foot, pad, elapsed, allowance)? {
                    self.add_mixed_resultant(h);
                    self.mixed.push(h);
                }
            }
            ProjectionGeometry::SlidingEdge => {
                if let Some(h) = solver.apply_sliding_edge_at(foot, pad, elapsed, allowance)? {
                    for site in h.sites {
                        self.add_resultant(site);
                    }
                    self.edges.push(h);
                }
            }
        }
        Ok(())
    }
    fn add_mixed_resultant(&mut self, h: MixedEdgeImpactReceipt) {
        self.add_impact(
            h.foot,
            h.support,
            h.elapsed,
            Vector::from_array(h.normal),
            Vector::from_array(h.total_impulse_on_foot()),
            h.impulses.normal.iter().sum(),
        );
    }
    fn add_resultant(&mut self, hit: CornerImpactReceipt) {
        let j = Vector::from_array(hit.impulse_on_foot);
        let n = Vector::from_array(hit.normal);
        self.add_impact(hit.foot, hit.support, hit.elapsed, n, j, j.dot(n));
    }
    fn add_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        n: Vector,
        j: Vector,
        scalar: f64,
    ) {
        let p = RigidContactObservation {
            a: foot.min(support),
            b: foot.max(support),
            active: false,
            normal_impulse: scalar,
        };
        let sign = if p.a == foot { 1. } else { -1. };
        self.add_vectors(p, sign * n * scalar, Some(sign * (j - n * scalar)));
        self.add_scalar(p, elapsed);
    }
    fn add_shared(&mut self, load: SharedFaceReceipt) {
        for site in load.loads {
            let foot = load.foot;
            let p = RigidContactObservation {
                a: foot.min(site.support),
                b: foot.max(site.support),
                active: site.contact_at_end,
                normal_impulse: site.selected_normal_impulse,
            };
            let sign = if p.a == foot { 1. } else { -1. };
            self.add_vectors(
                p,
                sign * p.normal_impulse * Vector::from(load.normal),
                Some(sign * Vector::from(site.tangent_impulse_on_foot)),
            );
            self.add_scalar(p, load.start);
        }
        self.shared_faces.push(load);
    }
    fn add_sustained(&mut self, load: SustainedEdgeReceipt) {
        let a = load.foot.min(load.support);
        let p = RigidContactObservation {
            a,
            b: load.foot.max(load.support),
            active: true,
            normal_impulse: load.compressive_impulse,
        };
        let sign = if a == load.foot { 1. } else { -1. };
        self.add_vectors(
            p,
            sign * Vector::from_array(load.normal_impulse_on_foot),
            Some(sign * Vector::from_array(load.tangent_impulse_on_foot)),
        );
        self.add_scalar(p, load.start);
        self.sustained.push(load);
    }
    fn finish(self) -> ContactIntervalReceipt {
        ContactIntervalReceipt {
            native_steps: self.steps,
            compact_point_tail: self.compact_point_tail,
            corner_impacts: self.projected,
            rigid_point_impacts: self.rigid_points,
            opposing_corner_impacts: self.upper_points,
            opposing_return_impacts: self.upper_returns,
            opposing_normal_series: self.normal_series,
            opposing_normal_declines: self.normal_declines,
            planar_impacts: self.planar,
            sustained_edges: self.sustained,
            sliding_vertices: self.vertices,
            sticking_points: self.sticking_points,
            point_transfers: self.point_transfers,
            sliding_skew_edges: self.sliding_skew_edges,
            two_bearing_edges: self.two_bearing,
            sole_transfers: self.sole_transfers,
            rocking_limits: self.rocking_limits,
            face_bearings: self.face_bearings,
            deck_edge_bearings: self.deck_edge_bearings,
            shared_faces: self.shared_faces,
            face_pressure_impacts: self.pressure,
            sliding_edge_impacts: self.edges,
            mixed_edge_impacts: self.mixed,
            edge_impacts: self.all_edges,
            finite_point_impacts: self.finite,
            fractional_point_paths: self.fractional,
            pairs: self
                .pairs
                .into_values()
                .map(|(mut entry, last)| {
                    entry.active_at_end &= last == self.steps;
                    entry
                })
                .collect(),
        }
    }
}

impl RapierBackend {
    fn interval_impulse(
        &self,
        pair: &rapier3d_f64::geometry::ContactPair,
        index: &partition::ContactBodyIndex,
    ) -> Option<(RigidContactObservation, Vector, Option<Vector>)> {
        let first = index.get(self, pair.collider1)?;
        let second = index.get(self, pair.collider2)?;
        let ordered = first <= second;
        let mut scalar = 0.;
        let mut normal = Vector::ZERO;
        let mut tangent = Some(Vector::ZERO);
        for m in pair.solver_manifolds() {
            if m.data.solver_contacts.is_empty() {
                continue;
            }
            let basis = self.applied_tangent_basis(m);
            let mut man_scalar = 0.;
            let mut man_tangent = Vector::ZERO;
            for i in partition::solved_point_indices(m) {
                let point = &m.points[i].data;
                man_scalar += point.impulse;
                if let Some((u, v)) = basis {
                    man_tangent += u * point.tangent_impulse.x + v * point.tangent_impulse.y;
                }
            }
            // The native manifold normal points towards collider two.
            scalar += man_scalar;
            normal -= m.data.normal * man_scalar;
            // The grouped model clamps friction to mu times the sum of its
            // nonnegative normal impulses on every solved interval. An unloaded
            // speculative manifold therefore contributes exactly zero to an
            // otherwise known registered tangent impulse.
            if basis.is_some() || man_scalar != 0. {
                tangent = tangent.zip(basis).map(|(sum, _)| sum + man_tangent);
            }
        }
        Some((
            RigidContactObservation {
                a: first.min(second),
                b: first.max(second),
                active: pair.has_any_active_contact(),
                normal_impulse: scalar,
            },
            if ordered { normal } else { -normal },
            tangent.map(|t| if ordered { t } else { -t }),
        ))
    }
}
impl RapierBackend {
    /// Select the native projected Gauss–Seidel sweeps at each internal
    /// integration boundary. This does not alter the timestep, public actuator
    /// cadence, or native contact refresh rate. Direct reset retains this setting.
    pub fn set_contact_pgs_passes(&mut self, passes: usize) -> Result<(), PhysicsError> {
        if !(1..=32).contains(&passes) {
            return Err(PhysicsError::InvalidValue);
        }
        self.world
            .integration_parameters
            .num_internal_pgs_iterations = passes;
        Ok(())
    }
    pub fn contact_pgs_passes(&self) -> usize {
        self.world
            .integration_parameters
            .num_internal_pgs_iterations
    }

    /// Preflight the native contact grid and its compatibility with current
    /// clock ownership without advancing the world or consuming queued commands.
    pub fn validate_contact_interval(
        &self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
    ) -> Result<(), PhysicsError> {
        self.contact_grid(dt, calls, iterations.into()).map(|_| ())
    }
    fn contact_grid(
        &self,
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
    ) -> Result<Option<NativeStepGrid>, PhysicsError> {
        if calls == 0 || !iterations.valid() {
            return Err(PhysicsError::InvalidValue);
        }
        if dt.is_zero() {
            return Ok(None);
        }
        let grid = NativeStepGrid::new(dt, calls)?;
        // These modes have their own clocks, query dispatchers or interval
        // reports. Refuse before consuming the caller's pending inputs.
        if self.curves.authored()
            || self.free_coast.is_some()
            || self.conserved_rotor.is_some()
            || matches!(self.fleet_clock, FleetClock::Translating(_))
            || matches!(self.fleet_dispatch, fleet_isolation::Dispatch::Active(_))
        {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "contact subdivision on an independently clocked motion or fleet",
            });
        }
        #[cfg(feature = "partition-checkpoint")]
        if self.native_profile.is_some()
            || self.pair_observer.is_some()
            || self.grasp_payload_observer.is_some()
            || self.join_queries_pending
        {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "contact subdivision on a partition-owned native profile",
            });
        }
        Ok(Some(grid))
    }
    /// Advance a rigid contact world on a nested native grid. Body wrenches and
    /// joint efforts apply for the entire public interval. One-time impulses
    /// retain their original application time. The next ordinary step returns
    /// to its original iteration count. A paused tick consumes no input.
    pub fn step_contact_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        self.run_contact_interval(dt, calls, iterations.into(), None, None)
    }
    /// Refine one public tick and request fresh native convex features on explicitly
    /// named box pairs. The count measures previously populated manifolds that
    /// Parry re-queries. Cold first encounters already obtain full fresh geometry.
    pub fn step_contact_interval_with_box_features(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        let iterations = iterations.into();
        if dt.is_zero() {
            return self
                .step_contact_interval(dt, calls, iterations)
                .map(|r| (r, 0));
        }
        let fresh = BoxFeatures::new(self, pairs)?;
        let receipt = self.run_contact_interval(dt, calls, iterations, None, fresh.as_ref())?;
        Ok((receipt, fresh.as_ref().map_or(0, BoxFeatures::rebuilt)))
    }

    /// Keep the ordinary public timestep and engine iteration settings while
    /// updating selected finite-box contact features during an ongoing contact.
    pub fn step_box_features(
        &mut self,
        dt: Duration,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<u64, PhysicsError> {
        if dt.is_zero() || pairs.is_empty() {
            self.step(dt)?;
            return Ok(0);
        }
        self.validate_contact_interval(
            dt,
            1,
            self.world.integration_parameters.num_solver_iterations,
        )?;
        let fresh = BoxFeatures::new(self, pairs)?;
        self.advance_public_tick(dt, None, fresh.as_ref(), None)?;
        if self.last_step_quarantined() {
            return Err(PhysicsError::NonFiniteStep { engine: "rapier" });
        }
        Ok(fresh.as_ref().map_or(0, BoxFeatures::rebuilt))
    }
    /// Observe committed native solves without advancing Bevy's public clock.
    /// A paused interval calls the observer zero times.
    pub fn observe_contact_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        mut observe: impl FnMut(ContactSubstep<'_>),
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        self.run_contact_interval(dt, calls, iterations.into(), Some(&mut observe), None)
    }
    #[cfg(feature = "landing-audit")]
    pub fn step_returning_box_features(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        mode: crate::ReturnFeatureMode,
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<(ContactIntervalReceipt, crate::ReturnFeatureCounts), PhysicsError> {
        self.validate_contact_interval(dt, calls, iterations)?;
        if dt.is_zero() {
            return Ok(Default::default());
        }
        let Some(fresh) = BoxFeatures::returning(self, pair, mode, dt, calls)? else {
            return self
                .run_contact_interval(dt, calls, iterations, observer, None)
                .map(|r| (r, Default::default()));
        };
        let receipt = self.run_contact_interval(dt, calls, iterations, observer, Some(&fresh))?;
        Ok((receipt, fresh.return_counts()))
    }
    fn run_contact_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
        fresh: Option<&BoxFeatures>,
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        let Some(grid) = self.contact_grid(dt, calls, iterations)? else {
            return Ok(ContactIntervalReceipt::default());
        };
        self.integrate_prepared(
            dt,
            &[PreparedSegment { grid, iterations }],
            observer,
            fresh,
            None,
            CornerPlan::default(),
        )
    }

    /// Integrate an exact schedule inside one public tick. Optional box pairs
    /// request fresh convex features. The returned count excludes cold encounters.
    pub fn step_contact_schedule(
        &mut self,
        dt: Duration,
        segments: &[ContactSegment],
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        self.run_schedule(dt, segments, pairs, None)
    }

    /// Observe every committed native solve on the same public elapsed clock.
    pub fn observe_contact_schedule(
        &mut self,
        dt: Duration,
        segments: &[ContactSegment],
        pairs: &[(BodyId, BodyId)],
        mut observe: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        self.run_schedule(dt, segments, pairs, Some(&mut observe))
    }

    fn run_schedule(
        &mut self,
        dt: Duration,
        segments: &[ContactSegment],
        pairs: &[(BodyId, BodyId)],
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        let mut total = Duration::ZERO;
        let mut steps = 0_u32;
        let mut prepared = Vec::with_capacity(segments.len());
        for segment in segments {
            total = total
                .checked_add(segment.duration)
                .ok_or(PhysicsError::InvalidValue)?;
            steps = steps
                .checked_add(segment.native_steps)
                .ok_or(PhysicsError::InvalidValue)?;
            let grid = self
                .contact_grid(segment.duration, segment.native_steps, segment.iterations)?
                .ok_or(PhysicsError::InvalidValue)?;
            prepared.push(PreparedSegment {
                grid,
                iterations: segment.iterations,
            });
        }
        if total != dt {
            return Err(PhysicsError::InvalidValue);
        }
        if dt.is_zero() {
            return Ok((ContactIntervalReceipt::default(), 0));
        }
        let fresh = BoxFeatures::new(self, pairs)?;
        let receipt = self.integrate_prepared(
            dt,
            &prepared,
            observer,
            fresh.as_ref(),
            None,
            CornerPlan::default(),
        )?;
        Ok((receipt, fresh.as_ref().map_or(0, BoxFeatures::rebuilt)))
    }

    /// Ordinary temporal work, retaining the world's existing fixed-time sweeps.
    pub fn ordinary_contact_iterations(&self) -> ContactSolverIterations {
        self.world
            .integration_parameters
            .num_solver_iterations
            .into()
    }
    /// Divide a public tick into a fixed count of native calls, placing an upcoming
    /// boundary from current cuboid motion. Declined candidates use a uniform grid
    /// for the remaining duration. Fresh geometry and time registration are independent.
    pub fn step_prospective_box_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        fresh_pairs: &[(BodyId, BodyId)],
        timing_pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64, BoxTimingReceipt), PhysicsError> {
        self.run_prospective_box_interval(
            dt,
            calls,
            iterations.into(),
            fresh_pairs,
            timing_pairs,
            None,
        )
    }

    /// Observe live-retimed native solves at their committed physical timestamps.
    pub fn observe_prospective_box_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        fresh_pairs: &[(BodyId, BodyId)],
        timing_pairs: &[(BodyId, BodyId)],
        mut observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, u64, BoxTimingReceipt), PhysicsError> {
        self.run_prospective_box_interval(
            dt,
            calls,
            iterations.into(),
            fresh_pairs,
            timing_pairs,
            Some(&mut observer),
        )
    }
    fn run_prospective_box_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        fresh_pairs: &[(BodyId, BodyId)],
        timing_pairs: &[(BodyId, BodyId)],
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<(ContactIntervalReceipt, u64, BoxTimingReceipt), PhysicsError> {
        let result = self.advance_box_contacts(
            dt,
            calls,
            iterations,
            BoxContactActions {
                fresh_features: fresh_pairs,
                prospective_timing: timing_pairs,
                ..Default::default()
            },
            observer,
        )?;
        let timing = match result.timing {
            Some(t) => t,
            None if !dt.is_zero() && timing_pairs.is_empty() => BoxTimingReceipt {
                durations: NativeStepGrid::new(dt, calls)?.intervals().collect(),
                decline: Some(BoxTimingDecline::MultipleCandidates),
                decline_after: Some(0),
                ..Default::default()
            },
            None => BoxTimingReceipt::default(),
        };
        Ok((result.interval, result.rebuilt_manifolds, timing))
    }

    /// Advance all bodies with one native grid, enabling only the named contact
    /// operations. The interval includes projected and native signed reactions.
    pub fn advance_box_contacts(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        actions: BoxContactActions<'_>,
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<BoxContactOutcome, PhysicsError> {
        let Some(grid) = self.contact_grid(dt, calls, iterations)? else {
            return Ok(BoxContactOutcome::default());
        };
        let fresh = BoxFeatures::enroll(
            self,
            actions.fresh_features,
            actions.returning_features,
            crate::box_features::ReturnFeatureMode::FirstStale,
            dt,
            calls,
        )?;
        for &(a, b) in actions
            .sustained_edges
            .iter()
            .chain(actions.two_bearing_edges)
            .chain(actions.face_bearings)
        {
            if a == b {
                return Err(PhysicsError::InvalidValue);
            }
            if !self.bodies.contains_key(&a) || !self.bodies.contains_key(&b) {
                return Err(PhysicsError::MissingBody);
            }
        }
        for (i, group) in actions.shared_faces.iter().enumerate() {
            if (group.settle_comoving && !group.uniform_kinetic)
                || group.supports[0] == group.supports[1]
                || !group
                    .supports
                    .iter()
                    .all(|p| actions.face_bearings.contains(&(group.foot, *p)))
                || group
                    .preferred_fraction_b
                    .is_some_and(|v| !v.is_finite() || !(0.0..=1.).contains(&v))
                || actions.shared_faces[..i]
                    .iter()
                    .any(|prev| prev.foot == group.foot)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        if actions.deck_edge_bearings.iter().enumerate().any(|(i, k)| {
            !actions.face_bearings.contains(k) || actions.deck_edge_bearings[..i].contains(k)
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        for (i, pair) in actions.face_bearings.iter().enumerate() {
            if actions.face_bearings[..i].contains(pair) || actions.sustained_edges.contains(pair) {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (i, pair) in actions.rocking_limits.iter().enumerate() {
            if !actions.frictionless_sole_impacts.contains(pair)
                || !actions.two_bearing_edges.contains(pair)
                || actions.rocking_limits[..i].contains(pair)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (i, pair) in actions.two_bearing_edges.iter().enumerate() {
            if !(actions.edge_impacts.contains(pair)
                || actions.frictionless_sole_impacts.contains(pair))
                || actions.two_bearing_edges[..i].contains(pair)
                || actions.sustained_edges.contains(pair)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        if actions.retime_finite_transfer.iter().any(|p| {
            !actions.finite_point_impacts.contains(p) && !actions.finite_segment_impacts.contains(p)
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .fractional_point_paths
            .iter()
            .enumerate()
            .any(|(i, p)| {
                !actions.finite_point_impacts.contains(p)
                    || actions.fractional_point_paths[..i].contains(p)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions.certify_skew_edges.iter().enumerate().any(|(i, p)| {
            !actions.finite_point_impacts.contains(p) || actions.certify_skew_edges[..i].contains(p)
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .certify_face_vertices
            .iter()
            .enumerate()
            .any(|(i, p)| {
                !actions.finite_point_impacts.contains(p)
                    || actions.certify_face_vertices[..i].contains(p)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .certify_segment_friction_work
            .iter()
            .any(|p| !actions.finite_segment_impacts.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .target_point_bounds
            .iter()
            .enumerate()
            .any(|(i, p)| {
                !actions.subdivide_point_bounds.contains(p)
                    || actions.target_point_bounds[..i].contains(p)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .compact_point_tails
            .iter()
            .enumerate()
            .any(|(i, p)| {
                !actions.point_transfers.contains(p) || actions.compact_point_tails[..i].contains(p)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .subdivide_point_bounds
            .iter()
            .enumerate()
            .any(|(i, p)| {
                !actions.point_transfers.contains(p)
                    || actions.subdivide_point_bounds[..i].contains(p)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        for pairs in [
            actions.sliding_vertices,
            actions.sticking_points,
            actions.point_transfers,
            actions.sliding_skew_edges,
            actions.slip_launches,
        ] {
            for (i, pair) in pairs.iter().enumerate() {
                if !(actions.finite_point_impacts.contains(pair)
                    || actions.finite_segment_impacts.contains(pair))
                    || pairs[..i].contains(pair)
                {
                    return Err(PhysicsError::InvalidValue);
                }
            }
        }
        let geometries = actions.projections();
        let mut one = None;
        let count = geometries.iter().map(|(_, p)| p.len()).sum::<usize>();
        let mut seen = std::collections::BTreeSet::new();
        for (geometry, pairs) in geometries {
            for &(foot, pad) in pairs {
                if foot == pad || count > 1 && !seen.insert((foot, pad)) {
                    return Err(PhysicsError::InvalidValue);
                }
                if !self.bodies.contains_key(&foot) || !self.bodies.contains_key(&pad) {
                    return Err(PhysicsError::MissingBody);
                }
                if count == 1 {
                    one = Some((foot, pad, geometry));
                }
            }
        }
        let mut fractional = None;
        let flight = if let Some((foot, pad, geometry)) = one {
            // A stationary, already supported sole can have a tiny positive
            // reconstructed height at the next public boundary. Its active normal
            // constraint and gravity are resolved by the continued bearing.
            if actions.rocking_limits.contains(&(foot, pad))
                && self.frictionless_bearing_now(foot, pad, true)?
                || actions.face_bearings.contains(&(foot, pad))
                    && self.frictionless_face_now(foot, pad)?
            {
                None
            } else if actions.fractional_point_paths.contains(&(foot, pad)) {
                self.finite_flight_mode(foot, pad, dt, false, false, Some(&mut fractional))?
            } else if actions.certify_face_vertices.contains(&(foot, pad))
                || actions.certify_skew_edges.contains(&(foot, pad))
            {
                self.finite_flight(
                    foot,
                    pad,
                    dt,
                    actions.certify_face_vertices.contains(&(foot, pad)),
                    actions.certify_skew_edges.contains(&(foot, pad)),
                )?
            } else {
                self.registered_flight(foot, pad, dt, geometry)?
            }
            .filter(|f| {
                (calls > 1 || f.at == dt)
                    && actions.prospective_timing.iter().all(|&p| p == (foot, pad))
            })
        } else {
            None
        };
        let mut timing = (flight.is_none() && !actions.prospective_timing.is_empty())
            .then(|| BoxTiming::new(self, dt, calls, actions.prospective_timing))
            .transpose()?;
        if actions
            .integrate_free_rotation
            .iter()
            .any(|p| !actions.guard_separated_flight.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions.refine_rotation_timing.iter().any(|p| {
            !actions.integrate_free_rotation.contains(p) || !actions.prospective_timing.contains(p)
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        for (i, (pair, mode)) in actions.rotation_contact_pulses.iter().enumerate() {
            if !actions.refine_rotation_timing.contains(pair)
                || actions.rotation_contact_pulses[..i]
                    .iter()
                    .any(|(p, _)| p == pair)
                || matches!(mode,RotationContactPulse::Fixed(ns) if !(1..=4000).contains(ns))
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (i, p) in actions.compact_impact_budget.iter().enumerate() {
            if actions.compact_impact_budget[..i].contains(p)
                || !actions.swept_box_bounds.contains(p)
                || !actions
                    .rotation_contact_pulses
                    .contains(&(*p, RotationContactPulse::Measured))
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        if actions
            .integrate_ballistic_translation
            .iter()
            .any(|p| !actions.integrate_free_rotation.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        for (ix, p) in actions.project_rigid_corner.iter().enumerate() {
            if actions.project_rigid_corner[..ix].contains(p)
                || !actions.swept_box_bounds.contains(p)
                || !actions
                    .rotation_contact_pulses
                    .contains(&(*p, RotationContactPulse::Measured))
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (i, &(foot, pad, opponent)) in actions.opposing_box_guards.iter().enumerate() {
            if foot == pad
                || opponent == foot
                || opponent == pad
                || !actions.guard_separated_flight.contains(&(foot, pad))
                || actions.opposing_box_guards[..i]
                    .iter()
                    .any(|&(a, b, _)| (a, b) == (foot, pad))
            {
                return Err(PhysicsError::InvalidValue);
            }
            if !self.bodies.contains_key(&opponent) {
                return Err(PhysicsError::MissingBody);
            }
        }
        for (i, triple) in actions.project_opposing_corners.iter().enumerate() {
            if !actions.opposing_box_guards.contains(triple)
                || actions.project_opposing_corners[..i].contains(triple)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (index, triple) in actions.project_opposing_returns.iter().enumerate() {
            if !actions.project_opposing_corners.contains(triple)
                || actions.project_opposing_returns[..index].contains(triple)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        if actions.project_opposing_normals.len() > 1
            || actions
                .project_opposing_normals
                .iter()
                .any(|p| !actions.project_opposing_corners.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let named = match actions.opposing_box_guards {
            &[(foot, pad, opponent)]
                if actions.guard_separated_flight == [(foot, pad)]
                    && (actions.prospective_timing.is_empty()
                        || actions.prospective_timing == [(foot, pad)])
                    && actions.integrate_free_rotation.contains(&(foot, pad))
                    && count == 0
                    && actions.sustained_edges.is_empty()
                    && actions.two_bearing_edges.is_empty()
                    && actions.face_bearings.is_empty() =>
            {
                Some(opponent)
            }
            _ => None,
        };
        for (i, p) in actions.rigid_corner_with_neighbors.iter().enumerate() {
            if !actions.project_rigid_corner.contains(p)
                || actions.rigid_corner_with_neighbors[..i].contains(p)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (i, p) in actions.follow_rigid_departure.iter().enumerate() {
            if !actions.project_rigid_corner.contains(p)
                || actions.follow_rigid_departure[..i].contains(p)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        if let Some(live) = timing.as_mut() {
            if let Some(id) = named {
                live.enroll_opponent(self, id);
            }
            if let [pair] = actions.project_rigid_corner
                && actions.prospective_timing == [*pair]
                && actions.guard_separated_flight == [*pair]
                && count == 0
                && actions.sustained_edges.is_empty()
                && actions.two_bearing_edges.is_empty()
                && actions.face_bearings.is_empty()
            {
                live.point_pair = Some(*pair);
                live.point_neighbors = actions.rigid_corner_with_neighbors.contains(pair);
                live.receipt.point_attempts = Some(Vec::new());
                if actions.follow_rigid_departure.contains(pair) {
                    live.receipt.departure_calls = Some(Vec::new());
                }
            }
            live.ballistic = actions.prospective_timing.len() == 1
                && actions
                    .integrate_ballistic_translation
                    .contains(&actions.prospective_timing[0]);
            live.cubic = !actions.refine_rotation_timing.is_empty();
            live.swept = actions.prospective_timing.len() == 1
                && actions
                    .swept_box_bounds
                    .contains(&actions.prospective_timing[0]);
            if let [(_, mode)] = actions.rotation_contact_pulses {
                live.pulse = Some((*mode, iterations.temporal));
            }
            if !actions.compact_impact_budget.is_empty() {
                let shared = actions.prospective_timing != actions.guard_separated_flight
                    || actions.prospective_timing.len() != 1
                    || count > 0
                    || !actions.sustained_edges.is_empty()
                    || !actions.two_bearing_edges.is_empty()
                    || !actions.face_bearings.is_empty()
                    || !actions.integrate_released_rotation.is_empty();
                live.enroll_compact(self, shared);
            }
        }
        if actions
            .integrate_released_rotation
            .iter()
            .any(|p| !actions.integrate_free_rotation.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        if actions
            .swept_box_bounds
            .iter()
            .any(|p| !actions.integrate_free_rotation.contains(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let shared = (actions.prospective_timing == actions.guard_separated_flight)
            .then_some(timing.as_ref())
            .flatten();
        let mut guard = (!actions.guard_separated_flight.is_empty())
            .then(|| {
                FlightGuard::new(
                    self,
                    actions.guard_separated_flight,
                    shared,
                    !actions.integrate_free_rotation.is_empty(),
                    !actions.integrate_released_rotation.is_empty(),
                    !actions.swept_box_bounds.is_empty(),
                    !actions.integrate_ballistic_translation.is_empty(),
                )
            })
            .transpose()?;
        if let (Some(guard), Some(id)) = (guard.as_mut(), named) {
            guard.enroll_opponent(self, id);
        }
        let upper = match actions.project_opposing_corners {
            &[triple]
                if named == Some(triple.2)
                    && actions.compact_impact_budget.is_empty()
                    && actions.face_bearings.is_empty()
                    && actions.sticking_points.is_empty()
                    && actions.sliding_vertices.is_empty() =>
            {
                crate::box_timing::UpperPair::new(self, triple)
            }
            _ => None,
        };
        if let Some(g) = guard.as_mut()
            && !actions.project_opposing_corners.is_empty()
        {
            g.receipt.opposing_corner_calls = Some(Vec::new());
        }
        let uniform = [PreparedSegment { grid, iterations }];
        let phased;
        let segments = if let Some(f) = flight
            && f.at < dt
        {
            let tail = dt - f.at;
            let after = (u128::from(calls - 1)).min(tail.as_nanos()) as u32;
            phased = [
                PreparedSegment {
                    grid: NativeStepGrid::new(f.at, calls - after)?,
                    iterations,
                },
                PreparedSegment {
                    grid: NativeStepGrid::new(tail, after)?,
                    iterations,
                },
            ];
            &phased[..]
        } else {
            &uniform[..]
        };
        let interval = self.integrate_prepared(
            dt,
            segments,
            observer,
            fresh.as_ref(),
            timing.as_mut(),
            CornerPlan {
                actions,
                fractional,
                flight,
                guard: guard.as_mut(),
                upper,
            },
        )?;
        Ok(BoxContactOutcome {
            interval,
            rebuilt_manifolds: fresh.as_ref().map_or(0, BoxFeatures::rebuilt),
            returning_features: fresh
                .as_ref()
                .map_or_else(Vec::new, BoxFeatures::return_receipts),
            timing: timing.map(|t| t.receipt),
            flight: guard.map(|g| g.receipt),
        })
    }
    fn integrate_prepared(
        &mut self,
        dt: Duration,
        segments: &[PreparedSegment],
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
        fresh: Option<&BoxFeatures>,
        live: Option<&mut BoxTiming>,
        project: CornerPlan<'_>,
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        let mut work = Integration {
            observer,
            segments,
            live,
            project,
            projected: Vec::new(),
            rigid_points: Vec::new(),
            upper_points: Vec::new(),
            upper_returns: Vec::new(),
            normal_series: Vec::new(),
            normal_declines: Vec::new(),
            planar: Vec::new(),
            sustained: Vec::new(),
            vertices: Vec::new(),
            sticking_points: Vec::new(),
            point_transfers: Vec::new(),
            sliding_skew_edges: Vec::new(),
            two_bearing: Vec::new(),
            sole_transfers: Vec::new(),
            rocking_limits: Vec::new(),
            face_bearings: Vec::new(),
            deck_edge_bearings: Vec::new(),
            shared_faces: Vec::new(),
            pressure: Vec::new(),
            edges: Vec::new(),
            mixed: Vec::new(),
            all_edges: Vec::new(),
            finite: Vec::new(),
            fractional: Vec::new(),
            observed_finite: 0,
            steps: 0,
            compact_point_tail: None,
            body_index: Default::default(),
            pairs: BTreeMap::new(),
        };
        self.advance_public_tick(dt, Some(&mut work), fresh, None)?;
        if self.last_step_quarantined() {
            return Err(PhysicsError::NonFiniteStep { engine: "rapier" });
        }
        Ok(work.finish())
    }
    /// A first-order finite-feature hint only moves a native call earlier.
    /// The entire remaining duration and the original native budget are spent.
    fn finite_suffix_grid(
        &self,
        hit: &mut FiniteImpactReceipt,
        tail: Duration,
        rest: u32,
    ) -> Result<Option<(Duration, NativeStepGrid)>, PhysicsError> {
        if rest < 2 {
            return Ok(None);
        }
        let Some((limit, seconds)) = self.finite_transfer_hint(*hit)? else {
            return Ok(None);
        };
        let candidate = Self::short_native_suffix(seconds, tail, rest)?;
        if let Some((first, _)) = &candidate {
            hit.next_feature_boundary = Some((limit, *first));
        }
        Ok(candidate)
    }
    fn short_native_suffix(
        seconds: f64,
        tail: Duration,
        rest: u32,
    ) -> Result<Option<(Duration, NativeStepGrid)>, PhysicsError> {
        if rest < 2
            || !seconds.is_finite()
            || seconds <= 0.
            || seconds >= tail.as_secs_f64() / f64::from(rest)
        {
            return Ok(None);
        }
        let first = Duration::from_nanos((seconds * 1e9).ceil().max(1.) as u64);
        if first.as_nanos() >= tail.as_nanos() / u128::from(rest) {
            return Ok(None);
        }
        let grid = NativeStepGrid::new(tail - first, rest - 1)?;
        Ok(Some((first, grid)))
    }
    fn solve_subdivisions(
        &mut self,
        work: &mut Integration<'_, '_>,
        fresh: Option<&BoxFeatures>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        let previous = self.world.integration_parameters;
        let mut elapsed = Duration::ZERO;
        let mut previous_upper = None;
        let mut previous_material = None;
        let mut transfer: Option<(u32, Duration, NativeStepGrid)> = None;
        // A cold, single registered line can already have a projected impulse at
        // time zero. Its right-hand feature determines the first native interval.
        if work.live.is_none()
            && work.project.flight.is_none()
            && let [segment] = work.segments
            && let [hit] = work.finite.as_mut_slice()
            && hit.segment.is_some()
            && work.project.actions.finite_segment_impacts
                == [(hit.resultant.foot, hit.resultant.support)]
            && work
                .project
                .actions
                .projections()
                .iter()
                .map(|(_, p)| p.len())
                .sum::<usize>()
                == 1
            && work
                .project
                .actions
                .retime_finite_transfer
                .contains(&(hit.resultant.foot, hit.resultant.support))
            && let Some((first, grid)) =
                self.finite_suffix_grid(hit, dt, segment.grid.intervals().len() as u32)?
        {
            transfer = Some((0, first, grid));
        }
        let result = (|| {
            'schedule: for segment in work.segments {
                let parameters = &mut self.world.integration_parameters;
                parameters.num_solver_iterations = segment.iterations.temporal;
                parameters.num_internal_pgs_iterations = segment
                    .iterations
                    .pgs
                    .unwrap_or(previous.num_internal_pgs_iterations);
                parameters.num_internal_stabilization_iterations = segment
                    .iterations
                    .relaxation
                    .unwrap_or(previous.num_internal_stabilization_iterations);
                for ordinary in segment.grid.intervals() {
                    if work
                        .live
                        .as_ref()
                        .is_some_and(|p| p.receipt.compact.is_some_and(|r| r.complete))
                    {
                        break;
                    }
                    let mut h = match work.live.as_deref_mut() {
                        Some(planner) => {
                            let guarded = planner.pulse.is_some()
                                && work
                                    .project
                                    .guard
                                    .as_ref()
                                    .is_some_and(|g| g.receipt.guarded.last() == Some(&true));
                            planner.next(self, elapsed, work.steps, guarded)
                        }
                        None => match transfer {
                            Some((start, first, _)) if work.steps == start => first,
                            Some((start, _, grid)) => grid
                                .interval(work.steps - start - 1)
                                .expect("postimpact suffix grid"),
                            None => ordinary,
                        },
                    };
                    self.world.integration_parameters.dt = h.as_secs_f64();
                    self.transport_velocities(Some(h.as_secs_f64()));
                    let mut prepared_ordered =
                        if work.project.actions.project_opposing_normals.is_empty() {
                            None
                        } else {
                            let mut failure = OpposingNormalDecline::new(elapsed, h);
                            let hit = work
                                .project
                                .upper
                                .as_ref()
                                .and_then(|p| p.normal_series(self, elapsed, h, &mut failure));
                            if hit.is_none() {
                                work.normal_declines.push(failure);
                            }
                            hit
                        };
                    let compound = prepared_ordered.is_some();
                    let mut attempted_ordered = compound;
                    if !compound
                        && work.live.is_none()
                        && !work.project.actions.project_opposing_returns.is_empty()
                        && let [segment] = work.segments
                        && let Some(pair) = work.project.upper.as_ref()
                    {
                        attempted_ordered = true;
                        prepared_ordered = pair.ordered(self, elapsed, h, true, previous_upper);
                        let remaining = segment.grid.intervals().len() as u32 - work.steps - 1;
                        if prepared_ordered.is_none()
                            && remaining > 0
                            && let Some((short, step)) =
                                pair.shorten(self, elapsed, h, previous_upper)
                        {
                            let suffix = NativeStepGrid::new(dt - elapsed - short, remaining)?;
                            transfer = Some((work.steps, short, suffix));
                            h = short;
                            self.world.integration_parameters.dt = h.as_secs_f64();
                            self.transport_velocities(Some(h.as_secs_f64()));
                            prepared_ordered = Some(step);
                        }
                    }
                    let flight = work.project.flight.filter(|f| elapsed < f.at);
                    let sustained = match work.project.actions.sustained_edges {
                        &[(foot, pad)]
                            if flight.is_none()
                                && work.live.is_none()
                                && work.project.guard.is_none() =>
                        {
                            self.prepare_sustained_edge(foot, pad, elapsed, h)?
                        }
                        _ => None,
                    };
                    let point_actions = work.project.actions;
                    let mut bound_pairs = [
                        point_actions.sliding_vertices,
                        point_actions.sticking_points,
                        point_actions.point_transfers,
                        point_actions.sliding_skew_edges,
                        point_actions.slip_launches,
                    ]
                    .into_iter()
                    .flatten();
                    let bound_pair = bound_pairs
                        .next()
                        .copied()
                        .filter(|p| bound_pairs.all(|q| q == p));
                    let vertex = match bound_pair {
                        Some((foot, pad))
                            if flight.is_none()
                                && work.live.is_none()
                                && work.project.guard.is_none()
                                && work.project.actions.two_bearing_edges.is_empty()
                                && work.project.actions.sustained_edges.is_empty()
                                && work
                                    .project
                                    .actions
                                    .projections()
                                    .iter()
                                    .map(|(_, p)| p.len())
                                    .sum::<usize>()
                                    == 1 =>
                        {
                            let hit = work.finite.last().copied().filter(|h| {
                                h.resultant.elapsed == elapsed
                                    && (h.resultant.foot, h.resultant.support) == (foot, pad)
                            });
                            let fractional = work.fractional.last().filter(|r| {
                                r.native_boundary == elapsed && (r.foot, r.support) == (foot, pad)
                            });
                            let onset = fractional.map(crate::joints::MaterialEndpoint::from);
                            let pair = (foot, pad);
                            let track_fractional =
                                point_actions.retime_finite_transfer.contains(&pair)
                                    && work
                                        .fractional
                                        .last()
                                        .is_some_and(|r| (r.foot, r.support) == pair);
                            let modes = point_actions.point_modes(pair, track_fractional);
                            let left = dt - elapsed;
                            let try_full = left > h
                                && (hit.is_some()
                                    || fractional.is_some()
                                    || elapsed.is_zero() && work.project.flight.is_none())
                                && point_actions.compact_point_tails == [pair]
                                && point_actions.face_bearings.is_empty()
                                && work.project.upper.is_none()
                                && self.isolated_point_world(foot, pad);
                            let full = if try_full {
                                self.prepare_bound_point(pair, elapsed, left, hit, onset, modes)?
                            } else {
                                None
                            };
                            if let Some(prepared) = full {
                                let total: u32 = work
                                    .segments
                                    .iter()
                                    .map(|s| s.grid.intervals().len() as u32)
                                    .sum();
                                work.compact_point_tail = Some(CompactPointTail {
                                    foot,
                                    support: pad,
                                    start: elapsed,
                                    duration: left,
                                    omitted_native_steps: total - work.steps - 1,
                                });
                                h = left;
                                self.world.integration_parameters.dt = h.as_secs_f64();
                                self.transport_velocities(Some(h.as_secs_f64()));
                                Some(prepared)
                            } else {
                                // Reuse the first native call to carry a verified material
                                // point up to its next unresolved line. Spend every remaining
                                // call on the suffix, including the actual line impulse.
                                let mut step = None;
                                if try_full
                                    && point_actions.retime_finite_transfer.contains(&pair)
                                    && modes.directed
                                    && let [segment] = work.segments
                                    && let Some(prefix) = self.prepare_bound_point(
                                        pair,
                                        elapsed,
                                        left,
                                        hit,
                                        onset,
                                        crate::joints::PointFollowModes {
                                            before_line: true,
                                            ..modes
                                        },
                                    )?
                                {
                                    let short = prefix.duration();
                                    let remaining =
                                        segment.grid.intervals().len() as u32 - work.steps - 1;
                                    if remaining > 0
                                        && short < left
                                        && (left - short).as_nanos() >= u128::from(remaining)
                                    {
                                        let grid = NativeStepGrid::new(left - short, remaining)?;
                                        transfer = Some((work.steps, short, grid));
                                        h = short;
                                        self.world.integration_parameters.dt = h.as_secs_f64();
                                        self.transport_velocities(Some(h.as_secs_f64()));
                                        step = Some(prefix);
                                    }
                                }
                                if step.is_none() {
                                    step = self
                                        .prepare_bound_point(pair, elapsed, h, hit, onset, modes)?;
                                }
                                let carried = previous_material
                                    .filter(|(at, p, _)| *at == elapsed && *p == pair)
                                    .map(|(_, _, q)| q);
                                if track_fractional
                                    && step.is_none()
                                    && let Some(site) = carried
                                {
                                    step = self.prepare_bound_point(
                                        pair,
                                        elapsed,
                                        h,
                                        None,
                                        Some(site),
                                        modes,
                                    )?;
                                }
                                if track_fractional
                                    && step.is_none()
                                    && let Some(endpoint) = onset.or(carried)
                                    && let Some((_, seconds)) = endpoint.limit
                                {
                                    let total: u32 = work
                                        .segments
                                        .iter()
                                        .map(|s| s.grid.intervals().len() as u32)
                                        .sum();
                                    // The linearized first margin can lie one native
                                    // clock quantum from the certified crossing. Only
                                    // move a boundary when its whole step is qualified.
                                    for shift in [0., 1e-9, -1e-9] {
                                        let Some((short, grid)) = Self::short_native_suffix(
                                            seconds + shift,
                                            left,
                                            total - work.steps,
                                        )?
                                        else {
                                            continue;
                                        };
                                        let Some(prepared) = self.prepare_bound_point(
                                            pair,
                                            elapsed,
                                            short,
                                            None,
                                            Some(endpoint),
                                            modes,
                                        )?
                                        else {
                                            continue;
                                        };
                                        transfer = Some((work.steps, short, grid));
                                        h = short;
                                        self.world.integration_parameters.dt = h.as_secs_f64();
                                        self.transport_velocities(Some(h.as_secs_f64()));
                                        let r = work
                                            .fractional
                                            .last_mut()
                                            .expect("verified fractional endpoint");
                                        if onset.is_some() {
                                            r.next_native_boundary = Some(h);
                                        }
                                        r.continued_boundaries.push(elapsed + h);
                                        step = Some(prepared);
                                        break;
                                    }
                                }
                                step
                            }
                        }
                        _ => None,
                    };
                    let actions = work.project.actions;
                    let bearing_pair = match (actions.two_bearing_edges, actions.face_bearings) {
                        ([e], []) | ([], [e]) => Some(*e),
                        ([e], [f]) if e == f => Some(*e),
                        _ => None,
                    };
                    let bearing = if let Some((foot, pad)) = bearing_pair
                        && flight.is_none()
                        && work.live.is_none()
                        && work.project.guard.is_none()
                        && actions.sustained_edges.is_empty()
                    {
                        let pair = (foot, pad);
                        self.prepare_two_bearing(
                            foot,
                            pad,
                            elapsed,
                            h,
                            BearingModes {
                                batch: None,
                                edge: actions.two_bearing_edges.contains(&pair),
                                sole: actions.frictionless_sole_impacts.contains(&pair),
                                rocking: actions.rocking_limits.contains(&pair),
                                face: actions.face_bearings.contains(&pair),
                                deck: actions.deck_edge_bearings.contains(&pair),
                                shared: None,
                            },
                        )?
                    } else if let [group] = actions.shared_faces
                        && actions.face_bearings.len() == 2
                        && actions.two_bearing_edges.is_empty()
                        && actions.projections().iter().all(|(_, ps)| ps.is_empty())
                        && flight.is_none()
                        && work.live.is_none()
                        && work.project.guard.is_none()
                        && actions.sustained_edges.is_empty()
                    {
                        let modes = BearingModes {
                            batch: None,
                            edge: false,
                            sole: false,
                            rocking: false,
                            face: true,
                            deck: false,
                            shared: Some(*group),
                        };
                        let mut prepared = self.prepare_two_bearing(
                            group.foot,
                            group.supports[0],
                            elapsed,
                            h,
                            modes,
                        )?;
                        if prepared.is_none() {
                            for &pad in &group.supports {
                                prepared = self.prepare_two_bearing(
                                    group.foot,
                                    pad,
                                    elapsed,
                                    h,
                                    BearingModes {
                                        shared: None,
                                        ..modes
                                    },
                                )?;
                                if prepared.is_some() {
                                    break;
                                }
                            }
                        }
                        prepared
                    } else {
                        None
                    };
                    let batched = if actions.shared_faces.len() > 1
                        && actions.face_bearings.len() == actions.shared_faces.len() * 2
                        && actions.two_bearing_edges.is_empty()
                        && actions.projections().iter().all(|(_, ps)| ps.is_empty())
                        && flight.is_none()
                        && work.live.is_none()
                        && work.project.guard.is_none()
                        && work.project.upper.is_none()
                        && actions.sustained_edges.is_empty()
                    {
                        self.prepare_shared_batch(actions.shared_faces, elapsed, h)?
                    } else {
                        Vec::new()
                    };
                    let point = if compound {
                        None
                    } else {
                        work.live
                            .as_deref_mut()
                            .and_then(|p| p.rigid_point(self, elapsed, h))
                    };
                    let upper = if point.is_none()
                        && flight.is_none()
                        && vertex.is_none()
                        && sustained.is_none()
                        && bearing.is_none()
                    {
                        if attempted_ordered {
                            prepared_ordered
                        } else {
                            work.project.upper.as_ref().and_then(|p| {
                                if work.project.actions.project_opposing_returns.is_empty() {
                                    p.prepare(self, elapsed, h, work.live.is_none())
                                } else {
                                    p.ordered(self, elapsed, h, work.live.is_none(), previous_upper)
                                }
                            })
                        }
                    } else {
                        None
                    };
                    let upper_enrolled = upper.is_some();
                    let cached = work.live.as_deref_mut().and_then(|p| {
                        p.cached
                            .take()
                            .or_else(|| p.departure_call(self, elapsed, h))
                    });
                    let departed = cached.as_ref().is_some_and(|c| c.from_point);
                    let (separated, spin) =
                        work.project.guard.as_deref_mut().map_or((None, None), |g| {
                            if point.is_some() || upper.is_some() {
                                g.projected_contact(h);
                                (None, None)
                            } else if let Some(call) = cached {
                                g.prepared(h, call)
                            } else {
                                g.during(self, h, work.live.as_deref(), flight.is_some())
                            }
                        });
                    let opposing = point
                        .as_ref()
                        .and_then(|p| p.opposing)
                        .or_else(|| work.project.guard.as_deref().and_then(|g| g.auxiliary));
                    if batched.is_empty() {
                        self.step_with_registered_features(
                            fresh,
                            flight
                                .map(|f| f.colliders)
                                .or_else(|| point.as_ref().map(|p| p.pair))
                                .or_else(|| upper.as_ref().map(|p| p.pair))
                                .or(separated)
                                .or_else(|| vertex.as_ref().map(|s| s.colliders))
                                .or_else(|| sustained.as_ref().map(|s| s.colliders))
                                .or_else(|| bearing.as_ref().map(|s| s.colliders)),
                            upper
                                .as_ref()
                                .map(|p| p.ceiling)
                                .or_else(|| opposing.map(|p| p.collider))
                                .or_else(|| bearing.as_ref().and_then(|b| b.second_pad())),
                        );
                    } else {
                        let rows: Vec<_> = batched
                            .iter()
                            .map(|p| crate::collision::SuppressedRows {
                                foot: p.colliders.0,
                                pad: p.colliders.1,
                                second: p.second_pad(),
                            })
                            .collect();
                        self.step_with_suppressed_rows(fresh, &rows);
                    }
                    if self.last_step_quarantined() {
                        break 'schedule;
                    }
                    if let Some(samples) = work
                        .project
                        .guard
                        .as_deref_mut()
                        .and_then(|g| g.receipt.opposing_corner_calls.as_mut())
                    {
                        samples.push(upper_enrolled);
                    }
                    if let Some(samples) = work
                        .project
                        .guard
                        .as_deref_mut()
                        .and_then(|g| g.receipt.opposing.as_mut())
                    {
                        samples.push(opposing.map(|p| p.record));
                    }
                    if let Some(spin) = spin {
                        spin.commit(self);
                    }
                    previous_material = vertex
                        .as_ref()
                        .and_then(|s| s.endpoint.zip(bound_pair))
                        .map(|(site, pair)| (elapsed + h, pair, site));
                    let vertex_load = vertex.map(|s| s.commit(self));
                    let load = sustained.map(|s| s.commit(self));
                    let bearing_load = bearing.map(|s| s.commit(self));
                    if !batched.is_empty() {
                        self.solved_shared_groups
                            .extend_from_slice(actions.shared_faces);
                    }
                    let batch_loads = self.commit_shared_batch(batched);
                    elapsed += h;
                    if let Some(f) = flight {
                        f.finish_free_step(self, elapsed);
                    }
                    if let Some(f) = flight.filter(|f| f.arrives && f.at == elapsed) {
                        if let Some(staged) = work.project.fractional.take() {
                            let r = staged.commit(self);
                            let a = f.foot.min(f.support);
                            let sign = if a == f.foot { 1. } else { -1. };
                            let contact = RigidContactObservation {
                                a,
                                b: f.foot.max(f.support),
                                active: true,
                                normal_impulse: r.normal_impulse,
                            };
                            let n = Vector::from(r.normal_impulse_on_foot);
                            work.add_vectors(
                                contact,
                                sign * n,
                                Some(sign * (Vector::from(r.impulse_on_foot) - n)),
                            );
                            work.add_scalar(contact, elapsed);
                            work.fractional.push(r);
                        } else {
                            work.add_geometry(
                                self,
                                f.geometry,
                                (f.foot, f.support),
                                elapsed,
                                f.allowance,
                                f.finite_witness,
                            )?;
                        }
                        if work
                            .project
                            .actions
                            .retime_finite_transfer
                            .contains(&(f.foot, f.support))
                            && let Some(hit) = work
                                .finite
                                .last_mut()
                                .filter(|r| r.resultant.elapsed == elapsed)
                        {
                            let all_calls: u32 = work
                                .segments
                                .iter()
                                .map(|s| s.grid.intervals().len() as u32)
                                .sum();
                            let ran = work.steps + 1;
                            let rest = all_calls - ran;
                            let tail = dt - elapsed;
                            if let Some((first, grid)) = self.finite_suffix_grid(hit, tail, rest)? {
                                transfer = Some((ran, first, grid));
                            }
                        }
                    }
                    let rigid_point = point.map(|p| p.commit(self));
                    if let Some(p) = work.live.as_deref_mut()
                        && let Some(calls) = p.receipt.departure_calls.as_mut()
                    {
                        if departed {
                            calls.push((elapsed - h, h));
                        }
                        p.departure_witness = rigid_point.map(|hit| (elapsed, hit.corner));
                    }
                    let mut upper = upper;
                    let normals = upper.as_mut().and_then(|s| s.normals.take());
                    let returned = upper.as_ref().and_then(|p| p.returned.as_deref().copied());
                    let upper_hit = upper.and_then(|p| p.commit(self));
                    if normals.is_some() || returned.is_some() || !upper_enrolled {
                        previous_upper = None;
                    } else if let Some(hit) = upper_hit {
                        previous_upper = Some(crate::box_timing::PriorUpper::from(hit));
                    }
                    work.observe(
                        self,
                        elapsed,
                        h,
                        SubstepReactions {
                            rigid: rigid_point,
                            corner: upper_hit,
                            returned: returned.as_ref(),
                            normals: normals.as_deref(),
                            span: vertex_load.as_ref(),
                        },
                    );
                    if let Some(normals) = normals {
                        let r = *normals;
                        work.add_resultant(r.first_lower.point.impact);
                        let mut add = |k: usize, t: Duration, j: f64| {
                            let n = Vector::from_array(r.normals[k]);
                            work.add_impact(
                                r.first_lower.point.impact.foot,
                                r.supports[k],
                                t,
                                n,
                                n * j,
                                j,
                            );
                        };
                        for h in &r.finite {
                            add(
                                usize::from(h.site),
                                Duration::from_secs_f64(h.time_s),
                                h.impulse,
                            );
                        }
                        let limit = Duration::from_secs_f64(
                            (r.terminal_time_s[0] + r.terminal_time_s[1]) * 0.5,
                        );
                        for (k, &j) in r.terminal_normal.iter().enumerate() {
                            add(k, limit, j);
                        }
                        work.normal_series.push(r);
                    }
                    if let Some(hit) = upper_hit {
                        work.add_resultant(hit.point.impact);
                        work.upper_points.push(hit);
                    }
                    if let Some(r) = returned {
                        work.add_resultant(r.lower.point.impact);
                        work.upper_returns.push(r);
                    }
                    if let Some(hit) = rigid_point {
                        work.add_resultant(hit.impact);
                        work.rigid_points.push(hit);
                        if let Some(mark) = work
                            .live
                            .as_deref_mut()
                            .and_then(|p| p.receipt.compact.as_mut())
                        {
                            mark.pulse_loaded = true;
                            mark.projected_load = true;
                        }
                    }
                    for bearing in bearing_load.into_iter().chain(batch_loads) {
                        let mut signed_vector = None;
                        let fields = match bearing {
                            BearingResult::Shared(load, tail, _) => {
                                work.add_shared(*load);
                                if let Some(next) = tail {
                                    work.add_shared(*next);
                                }
                                None
                            }
                            BearingResult::Smooth(load) => {
                                let fields = (
                                    load.foot,
                                    load.support,
                                    load.normal,
                                    load.start,
                                    load.normal_impulses.iter().sum(),
                                );
                                work.two_bearing.push(load);
                                Some(fields)
                            }
                            BearingResult::Deck(load) => {
                                let fields = (
                                    load.foot,
                                    load.support,
                                    load.end_normal,
                                    load.start,
                                    load.normal_impulse(),
                                );
                                signed_vector = Some(Vector::from(load.impulse_on_foot));
                                work.deck_edge_bearings.push(*load);
                                Some(fields)
                            }
                            BearingResult::Face(load) => {
                                let fields = (
                                    load.foot,
                                    load.support,
                                    load.normal,
                                    load.start,
                                    load.normal_impulse,
                                );
                                work.face_bearings.push(*load);
                                Some(fields)
                            }
                            BearingResult::Rocking(load) => {
                                let fields = (
                                    load.foot,
                                    load.support,
                                    load.normal,
                                    load.start,
                                    load.response.integrated_normal_impulse,
                                );
                                work.rocking_limits.push(*load);
                                Some(fields)
                            }
                            BearingResult::Transfer(load) => {
                                let fields = (
                                    load.foot,
                                    load.support,
                                    load.pressure.normal,
                                    load.start,
                                    load.integrated_normal_impulse(),
                                );
                                work.pressure.push(load.pressure);
                                work.sole_transfers.push(*load);
                                Some(fields)
                            }
                        };
                        if let Some((foot, support, normal, start, net)) = fields {
                            let p = RigidContactObservation {
                                a: foot.min(support),
                                b: foot.max(support),
                                active: true,
                                normal_impulse: net,
                            };
                            let sign = if p.a == foot { 1. } else { -1. };
                            work.add_vectors(
                                p,
                                sign * signed_vector.unwrap_or_else(|| net * Vector::from(normal)),
                                Some(Vector::ZERO),
                            );
                            work.add_scalar(p, start);
                        }
                    }
                    if let Some(bound) = vertex_load {
                        let (foot, pad, start, j, jn, normal) = match bound {
                            crate::joints::PointSpanLoad::Sliding(s) => {
                                let j = Vector::from(s.impulse_on_foot);
                                let n = Vector::from(s.normal);
                                if matches!(s.feature, gz_physics::SeparatingFeature::EdgePair(..))
                                {
                                    work.sliding_skew_edges.push(s);
                                    (
                                        s.foot,
                                        s.support,
                                        s.start,
                                        j,
                                        Vector::from(s.normal_impulse_on_foot),
                                        s.normal_impulse,
                                    )
                                } else {
                                    let amount = j.dot(n);
                                    work.vertices.push(s);
                                    (s.foot, s.support, s.start, j, amount * n, amount)
                                }
                            }
                            crate::joints::PointSpanLoad::Transfer(s) => {
                                let observation = (
                                    s.foot,
                                    s.support,
                                    s.start,
                                    Vector::from(s.impulse_on_foot),
                                    Vector::from(s.normal_impulse_on_foot),
                                    s.normal_impulse,
                                );
                                work.point_transfers.push(s);
                                observation
                            }
                            crate::joints::PointSpanLoad::Sticking(s) => {
                                work.sticking_points.push(s);
                                (
                                    s.foot,
                                    s.support,
                                    s.start,
                                    Vector::from(s.impulse_on_foot),
                                    Vector::from(s.normal_impulse_on_foot),
                                    s.normal_impulse,
                                )
                            }
                        };
                        let p = RigidContactObservation {
                            a: foot.min(pad),
                            b: foot.max(pad),
                            active: true,
                            normal_impulse: normal,
                        };
                        let sign = if foot == p.a { 1. } else { -1. };
                        work.add_vectors(p, sign * jn, Some(sign * (j - jn)));
                        work.add_scalar(p, start);
                    }
                    if let Some(load) = load {
                        work.add_sustained(load);
                    }
                    if work.compact_point_tail.is_some() {
                        debug_assert_eq!(elapsed, dt);
                        break 'schedule;
                    }
                }
            }
            Ok(())
        })();
        self.world.integration_parameters = previous;
        result
    }
    fn advance_public_tick(
        &mut self,
        dt: Duration,
        refinement: Option<&mut Integration<'_, '_>>,
        fresh: Option<&BoxFeatures>,
        coating: Option<collision::SuppressedRows>,
    ) -> Result<(), PhysicsError> {
        if !dt.is_zero() {
            if let FleetClock::Translating(elapsed) = self.fleet_clock {
                let now = elapsed.checked_add(dt).ok_or(PhysicsError::InvalidValue)?;
                self.fleet_clock = FleetClock::Translating(now);
                if self.curves.authored() {
                    self.curves.advance_unloaded(now);
                }
                self.joints.close();
                return Ok(());
            }
            // Rapier 0.36.1 caps angular speed at pi/4 divided by the public
            // step duration in the staged island solver. Refuse before changing
            // any state, including pending wrenches and joint construction.
            if self.max_boundary_spin > boundary_spin_limit(dt) {
                self.refresh_max_boundary_spin();
                if self.max_boundary_spin > boundary_spin_limit(dt) {
                    return Err(PhysicsError::InvalidValue);
                }
            }
            let curve_end = if self.curves.authored() {
                Some(self.curves.next_time(dt)?)
            } else {
                None
            };
            let curve_cuts = self
                .curves
                .plan(dt, self.world.integration_parameters.num_solver_iterations)?;
            let rotation_update = self.free_rotation_step(dt)?;
            self.joints
                .validate_motor_step(&self.world, dt.as_secs_f64())?;
            #[cfg(feature = "partition-checkpoint")]
            self.apply_sphere_island_topology();
            let joint_loads = self.joints.prepare_efforts(&self.world, &self.bodies)?;
            self.validate_spinning_step(dt)?;
            self.resume_disturbed_coast(dt)?;
            if let Some(coast) = &mut self.free_coast {
                coast.advance(&mut self.world, dt)?;
                self.joints.update(&self.world, &self.bodies);
                return Ok(());
            }
            if self.joints.construction_open() {
                self.joints.initialize_velocities(&mut self.world);
            }
            self.joints.close();
            for (id, (force, torque)) in joint_loads {
                let body = &mut self.world.bodies[self.bodies[&id].0];
                body.add_force(force, true);
                body.add_torque(torque, true);
                self.pending_wrenches.insert(id);
            }
            self.world.integration_parameters.dt = dt.as_secs_f64();
            self.cover_fleet_step(dt);
            self.apply_deck_resumes();
            self.transport_velocities(Some(dt.as_secs_f64()));
            #[cfg(feature = "partition-checkpoint")]
            if let Some(observer) = &mut self.pair_observer {
                let body = &self.world.bodies[self.bodies[&observer.spheres[0]].0];
                observer.initial_velocity = body.linvel().to_array();
            }
            #[cfg(feature = "partition-checkpoint")]
            if let Some(observer) = &mut self.grasp_payload_observer {
                let rb = &self.world.bodies[self.bodies[&observer.parts[0]].0];
                observer.incoming_velocity = rb.linvel().to_array();
                observer.external_impulse = (rb.user_force() * dt.as_secs_f64()).to_array();
            }
            if self.curves.authored() {
                self.curves.steps = 0;
                self.curves.contacts.clear();
            }
            if let Some(plan) = refinement {
                for (geometry, pairs) in plan.project.actions.projections() {
                    for &(foot, pad) in pairs {
                        plan.add_geometry(self, geometry, (foot, pad), Duration::ZERO, 0., None)?;
                    }
                }
                self.solve_subdivisions(plan, fresh, dt)?;
            } else if curve_cuts.is_empty() {
                self.step_with_suppressed_rows(fresh, coating.as_slice());
                if let Some(end) = curve_end {
                    self.curves.advance_unloaded(end);
                }
            } else {
                self.advance_curve_world(&curve_cuts);
            }
            if let Some(update) = rotation_update {
                update.commit(&mut self.world);
            }
            #[cfg(feature = "partition-checkpoint")]
            if std::mem::take(&mut self.join_queries_pending) {
                self.world
                    .narrow_phase
                    .set_query_dispatcher(rapier3d_f64::parry::query::DefaultQueryDispatcher);
            }
            self.transport_velocities(None);
            self.joints.finish_velocities(&mut self.world, true);
            self.joints.update(&self.world, &self.bodies);
            self.joints.consume_efforts();
            self.retire_woken_robots();
            for id in std::mem::take(&mut self.pending_wrenches) {
                if let Some((handle, ..)) = self.bodies.get(&id) {
                    let body = &mut self.world.bodies[*handle];
                    body.reset_forces(false);
                    body.reset_torques(false);
                }
            }
        }
        Ok(())
    }
    pub(super) fn advance_coated_pair(
        &mut self,
        dt: Duration,
        row: collision::SuppressedRows,
    ) -> Result<(), PhysicsError> {
        self.advance_public_tick(dt, None, None, Some(row))
    }
    pub(super) fn advance_ordinary(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        self.advance_public_tick(dt, None, None, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duplicate_collider_pairs_share_one_scalar_and_vector_and_active_flag() {
        let dt = Duration::from_micros(500);
        let mut run = Integration {
            compact_point_tail: None,
            observer: None,
            segments: &[],
            live: None,
            project: CornerPlan::default(),
            projected: Vec::new(),
            rigid_points: Vec::new(),
            upper_points: Vec::new(),
            upper_returns: Vec::new(),
            normal_series: Vec::new(),
            normal_declines: Vec::new(),
            planar: Vec::new(),
            sustained: Vec::new(),
            two_bearing: Vec::new(),
            sole_transfers: Vec::new(),
            rocking_limits: Vec::new(),
            face_bearings: Vec::new(),
            deck_edge_bearings: Vec::new(),
            shared_faces: Vec::new(),
            pressure: Vec::new(),
            edges: Vec::new(),
            mixed: Vec::new(),
            all_edges: Vec::new(),
            finite: Vec::new(),
            fractional: Vec::new(),
            observed_finite: 0,
            vertices: Vec::new(),
            sticking_points: Vec::new(),
            point_transfers: Vec::new(),
            sliding_skew_edges: Vec::new(),
            steps: 1,
            body_index: Default::default(),
            pairs: BTreeMap::new(),
        };
        let a = BodyId::from_raw(4);
        let b = BodyId::from_raw(9);
        let add = |r: &mut Integration<'_, '_>, active, scalar, normal, tangent, time| {
            let observation = RigidContactObservation {
                a,
                b,
                active,
                normal_impulse: scalar,
            };
            r.add_vectors(observation, normal, tangent);
            r.add_scalar(observation, time);
        };
        add(&mut run, true, 2., Vector::Z * 2., Some(Vector::X), dt / 2);
        add(
            &mut run,
            false,
            0.,
            Vector::ZERO,
            Some(Vector::ZERO),
            dt / 2,
        );
        assert!(run.pairs[&(a, b)].0.active_at_end);
        run.steps += 1;
        add(&mut run, false, 0., Vector::ZERO, Some(Vector::ZERO), dt);
        assert!(!run.pairs[&(a, b)].0.active_at_end);
        add(&mut run, true, 3., Vector::Y * 3., None, dt);
        let q = run.finish().pairs[0];
        assert_eq!(q.first_positive_end, Some(dt / 2));
        assert!(q.active_at_end);
        assert_eq!(q.normal_impulse, 5.);
        assert_eq!(q.normal_impulse_on_a, [0., 3., 2.]);
        assert_eq!(q.tangent_impulse_on_a, None);
    }

    #[test]
    fn signed_normal_and_tangent_recover_oblique_impact_momentum_in_both_id_orders() {
        let dt = Duration::from_micros(500);
        let angle = 0.25_f64;
        let n = Vector::new(angle.sin(), 0., angle.cos());
        let t = Vector::new(angle.cos(), 0., -angle.sin());
        for foot_first in [false, true] {
            let mut s = RapierBackend::new_multifidelity_with_friction(
                [0.; 3],
                NativeFrictionModel::PerPoint,
            )
            .unwrap();
            let sphere = |s: &mut RapierBackend| {
                s.spawn_link(
                    BodySpec {
                        position: (n * 0.1002).to_array(),
                        velocity: (t * 0.65 - n).to_array(),
                        shape: Shape::Sphere { radius: 0.1 },
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    None,
                )
                .unwrap()
            };
            let floor = |s: &mut RapierBackend| {
                s.spawn_link(
                    BodySpec {
                        position: (-n * 0.05).to_array(),
                        velocity: [0.; 3],
                        shape: Shape::Box {
                            half_extents: [0.4, 0.4, 0.05],
                        },
                        mass: None,
                    },
                    RotationSpec {
                        orientation: [0., (angle / 2.).sin(), 0., (angle / 2.).cos()],
                        angular_velocity: [0.; 3],
                    },
                    None,
                )
                .unwrap()
            };
            let (foot, pad) = if foot_first {
                (sphere(&mut s), floor(&mut s))
            } else {
                let pad = floor(&mut s);
                (sphere(&mut s), pad)
            };
            let before = Vector::from_array(s.state(foot).unwrap().velocity);
            let r = s.step_contact_interval(dt, 8, 4).unwrap();
            let after = Vector::from_array(s.state(foot).unwrap().velocity);
            let pair = &r.pairs[0];
            assert_eq!((pair.a, pair.b), (foot.min(pad), foot.max(pad)));
            let sign = if foot_first { 1. } else { -1. };
            let normal = Vector::from_array(pair.normal_impulse_on_a) * sign;
            let tangent = Vector::from_array(pair.tangent_impulse_on_a.unwrap()) * sign;
            assert!(tangent.dot(t) < -0.01 && normal.dot(n) > 0.8);
            assert!((normal - n * pair.normal_impulse).length() < 1e-9);
            assert!((tangent + normal - (after - before)).length() < 1e-8);
        }
    }

    #[test]
    fn compound_deck_receipt_matches_every_native_patch_and_validates_recycled_handles() {
        let dt = Duration::from_micros(500);
        for per_point in [false, true] {
            let material = if per_point {
                NativeFrictionModel::PerPoint
            } else {
                NativeFrictionModel::Grouped
            };
            let mut s =
                RapierBackend::new_multifidelity_with_friction([0., 0., -9.81], material).unwrap();
            let spawn = |s: &mut RapierBackend, position, mass, half_extents| {
                s.spawn_link(
                    BodySpec {
                        shape: Shape::Box { half_extents },
                        position,
                        mass,
                        velocity: [0.; 3],
                    },
                    RotationSpec::default(),
                    None,
                )
                .unwrap()
            };
            let dead = spawn(&mut s, [0., 3., 0.], None, [0.1; 3]);
            assert!(s.remove(dead));
            let deck = spawn(&mut s, [-0.14, 0., -0.05], None, [0.12, 0.22, 0.05]);
            let other = ColliderBuilder::cuboid(0.12, 0.22, 0.05).translation(Vector::X * 0.28);
            let handle = s.bodies[&deck].0;
            s.world
                .colliders
                .insert_with_parent(other, handle, &mut s.world.bodies);
            let foot = spawn(&mut s, [0., 0., 0.051], Some(1.), [0.25, 0.14, 0.05]);
            for _ in 0..100 {
                s.step(dt).unwrap();
            }

            let mut sum = 0.;
            let mut vector = Vector::ZERO;
            let mut friction = Vector::ZERO;
            let mut known = true;
            let mut count = 0;
            let interval = s
                .observe_contact_interval(dt, 8, 4, |native| {
                    let patches = native.contact_patches();
                    assert_eq!(patches.len(), 2);
                    for patch in patches {
                        assert_eq!((patch.a, patch.b), (deck, foot));
                        let n = Vector::from_array(patch.normal_on_a);
                        for p in patch.points {
                            sum += p.normal_impulse;
                            vector += n * p.normal_impulse;
                            if let Some(v) = p.tangent_impulse_on_a {
                                friction += Vector::from_array(v);
                            } else {
                                known = false;
                            }
                        }
                    }
                    count += 1;
                })
                .unwrap();
            assert_eq!(count, 8);
            assert_eq!(interval.pairs.len(), 1);
            let p = interval.pairs[0];
            assert!(p.active_at_end && p.first_positive_end.is_some());
            assert!(sum > 0.);
            assert!((p.normal_impulse - sum).abs() < 1e-14);
            assert!((Vector::from_array(p.normal_impulse_on_a) - vector).length() < 1e-14);
            assert!(vector.z < 0.); // force on the fixed lower ID, opposite to the foot
            assert_eq!(p.tangent_impulse_on_a.is_some(), per_point);
            assert_eq!(known, per_point);
            if let Some(t) = p.tangent_impulse_on_a {
                assert!((Vector::from_array(t) - friction).length() < 1e-14);
            }
        }
    }
}
