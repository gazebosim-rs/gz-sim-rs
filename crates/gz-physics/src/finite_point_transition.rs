// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve internal changes of a loaded finite material point while the
//! geometry, Coulomb mode, and the next impact are independently identifiable.
use super::*;
use crate::{
    FeatureMargin, FiniteContactSegment, SegmentPointImpact, generalized_segment_work_impact,
};
#[derive(Clone, Copy, Debug)]
pub struct BoundPointImpact {
    pub segment: FiniteContactSegment,
    pub law: SegmentPointImpact,
    pub work_boxes: u32,
}
#[derive(Clone, Copy, Debug)]
pub struct BoundPointPiece {
    /// Offset of the sustained contact in the original interval, in seconds.
    pub start: f64,
    pub motion: FiniteArcResult,
    pub sticking: bool,
    pub slip_growth: Option<SlipGrowth>,
    /// Instantaneous impact at the end of this piece, if a neighbor arrives.
    pub impact: Option<BoundPointImpact>,
}
#[derive(Debug)]
pub struct BoundPointPath {
    pub pieces: Vec<BoundPointPiece>,
    pub center_speed_bound: f64,
    pub angular_speed_bound: f64,
    /// Sum of the numerical corrections to coincident material points.
    pub translation_allowance: f64,
}
// With |d''|<=C, a potentially incident margin can have at most one zero
// on [0,h] if d'(0)+Ch<0. Require every other feature to remain strictly
// interior (or to open immediately from the initial contact line).
pub(super) fn unique_closing(
    candidate: &mut Option<FeatureLimit>,
    d: &FeatureMargin,
    h: f64,
    c: f64,
    tol: f64,
) -> bool {
    if candidate.is_some()
        || d.distance <= tol
        || d.speed + c * h >= -2e-12 * (1. + d.speed.abs() + c * h)
    {
        return false;
    }
    *candidate = Some(d.limit);
    true
}
pub(super) fn same_event(
    a: &FiniteArcResult,
    b: &FiniteArcResult,
    candidate: Option<FeatureLimit>,
    tol: f64,
) -> bool {
    if a.end != b.end {
        return false;
    }
    match b.end {
        FiniteArcEnd::Horizon => candidate.is_none_or(|c| {
            b.site
                .margins()
                .iter()
                .any(|m| m.limit == c && m.distance > tol)
        }),
        FiniteArcEnd::Feature(l) => {
            Some(l) == candidate
                && b.bracket[0] > 0.
                && (a.bracket[0] - b.bracket[0]).abs() <= 2e-13
                && b.bracket[1] - b.bracket[0] <= 2e-17
                && b.site
                    .margins()
                    .iter()
                    .any(|m| m.limit == l && m.distance.abs() <= tol)
        }
        _ => false,
    }
}
impl FiniteContactSegment {
    fn neither_endpoint_stops_closure(
        self,
        motion: FiniteArcResult,
        m: SphericalPointMaterial,
    ) -> bool {
        let n = self.normal;
        let x = n.any_orthonormal_vector();
        let basis = [x, n.cross(x), n];
        let arms = self.moving_points.map(|p| p - motion.center);
        let before = arms.map(|r| motion.velocity + motion.angular_velocity.cross(r));
        let tolerance = 1e-11 * (1. + before[0].length() + before[1].length());
        (0..2).all(|i| {
            let other = 1 - i;
            let response =
                |k: usize, f: DVec3| f / m.mass + arms[i].cross(f).cross(arms[k]) / m.inertia;
            let vn = before[i].dot(n);
            if vn >= -tolerance {
                // Even the greatest admissible normal impulse at the nearly
                // stationary port must leave the opposite port closing.
                let row = |k| DVec3::from_array(basis.map(|a| n.dot(response(k, a))));
                let own = row(i);
                let cross = row(other);
                let minimum = own.z - m.friction * own.truncate().length();
                let maximum = (cross.z + m.friction * cross.truncate().length()).max(0.);
                return minimum > 0.
                    && before[other].dot(n) + (tolerance - vn).max(0.) * maximum / minimum
                        < -tolerance;
            }
            let point = crate::PointImpact {
                inverse_mass: std::array::from_fn(|a| basis.map(|b| basis[a].dot(response(i, b)))),
                relative_velocity: basis.map(|a| a.dot(before[i])),
                friction: m.friction,
            };
            let Ok(impact) = point.solve() else {
                return false;
            };
            let [a, b, c] = impact.impulse;
            let opposite = before[other] + response(other, a * x + b * basis[1] + c * n);
            opposite.dot(n) < -tolerance
        })
    }
    /// Evaluate both world-space contact ports for a free body with spherical
    /// principal moments. Includes the distributed-pressure work certificate.
    pub fn spherical_endpoint_with_work(
        self,
        center: DVec3,
        v: DVec3,
        omega: DVec3,
        m: SphericalPointMaterial,
        max_boxes: u32,
    ) -> Result<Option<(SegmentPointImpact, u32)>, PhysicsError> {
        if ![m.mass, m.inertia, m.friction]
            .into_iter()
            .all(f64::is_finite)
            || m.mass <= 0.
            || m.inertia <= 0.
            || m.friction < 0.
            || ![center, v, omega].into_iter().all(DVec3::is_finite)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let a = [DVec3::X, DVec3::Y, DVec3::Z];
        let r = self.moving_points.map(|p| p - center);
        let mobility = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                std::array::from_fn(|u| {
                    std::array::from_fn(|w| {
                        a[u].dot(a[w] / m.mass + r[j].cross(a[w]).cross(r[i]) / m.inertia)
                    })
                })
            })
        });
        generalized_segment_work_impact(
            mobility,
            r.map(|r| v + omega.cross(r)),
            self.normal,
            r[1] - r[0],
            m.friction,
            max_boxes,
        )
    }
}
/// Result of the ordinary full-span and (when needed) dyadic first trial.
#[derive(Clone, Copy)]
struct PointTrial {
    span: f64,
    result: Option<(FiniteArcResult, Option<SlipGrowth>)>,
}
impl FiniteFeatureArc {
    /// Integrate a native interval across at least one change of bearing
    /// feature. Preflight every piece; ambiguity aborts the whole trial.
    /// The caller still proves that no other body can reach this pair.
    pub fn resolved_point_transfers(self, h: f64) -> Result<Option<BoundPointPath>, PhysicsError> {
        self.transfers(h, false, false, false, false, None)
    }
    /// Also admit sustained sliding between two nonparallel finite edges.
    pub fn resolved_spatial_transfers(
        self,
        h: f64,
    ) -> Result<Option<BoundPointPath>, PhysicsError> {
        self.transfers(h, true, false, false, false, None)
    }
    /// Permit a certified increasing slip component on a skew outgoing edge.
    pub fn resolved_launch_transfers(self, h: f64) -> Result<Option<BoundPointPath>, PhysicsError> {
        self.transfers(h, true, true, false, false, None)
    }
    /// Retry conservative whole-span bounds on shorter consecutive intervals.
    /// A bounded search stages all geometry and impulses before any commit.
    pub fn subdivided_point_transfers(
        self,
        h: f64,
        skew: bool,
        launch: bool,
    ) -> Result<Option<BoundPointPath>, PhysicsError> {
        self.transfers(h, skew, launch, true, false, None)
    }
    /// Use current material separations and rates to nominate a short interval for the
    /// same verified transfer calculation. Failed qualifications leave the state untouched.
    pub fn directed_point_transfers(
        self,
        h: f64,
        skew: bool,
        launch: bool,
    ) -> Result<Option<BoundPointPath>, PhysicsError> {
        // Preserve the full previously qualified path, including its material
        // event order. A locally valid directed prefix can otherwise consume
        // the restart budget and make a known complete interval unreachable.
        let first = self.ordinary_trial(h, skew, launch, true)?;
        if let Some(path) = self.transfers(h, skew, launch, true, false, Some(first))? {
            return Ok(Some(path));
        }
        self.transfers(h, skew, launch, true, true, Some(first))
    }
    /// Propose a whole-nanosecond handoff before a sliding vertex reaches a
    /// line where neither endpoint alone can prevent penetration. The two
    /// meshes only select a clock. The caller must independently qualify the
    /// shortened span, and leave the impact itself to the receiving solver.
    pub fn vertex_handoff_hint(self, h: f64) -> Result<Option<std::time::Duration>, PhysicsError> {
        if !h.is_finite() || h <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        if self.locked || self.initial_slip_force.is_some() {
            return Ok(None);
        }
        let Some((_, sample)) = self.frame(self.start()) else {
            return Ok(None);
        };
        if !matches!(
            sample.site.feature,
            crate::SeparatingFeature::SupportFace(_)
        ) {
            return Ok(None);
        }
        let candidate = |mesh| -> Result<Option<f64>, PhysicsError> {
            let motion = self.measure_to(h, mesh, 1e-15)?;
            let FiniteArcEnd::Feature(limit @ FeatureLimit::MovingNeighbor(_)) = motion.end else {
                return Ok(None);
            };
            let tol = 1024.
                * f64::EPSILON
                * (1.
                    + motion.center.length()
                    + self.flight.support.center.length()
                    + motion.moving.half_extent.length());
            let Some(line) = self
                .track
                .incident_segment(motion.moving, motion.site, limit, tol)
            else {
                return Ok(None);
            };
            Ok(line
                .neither_endpoint_stops_closure(motion, self.material)
                .then_some(motion.bracket[0]))
        };
        let Some(coarse) = candidate(8)? else {
            return Ok(None);
        };
        let Some(fine) = candidate(16)? else {
            return Ok(None);
        };
        let ns = (fine * 1e9).floor();
        Ok(
            (ns >= 1. && ns < u64::MAX as f64 && ns == (coarse * 1e9).floor())
                .then(|| std::time::Duration::from_nanos(ns as u64)),
        )
    }
    fn feature_trial(self, span: f64) -> Option<f64> {
        let (_, sample) = self.frame(self.start())?;
        let mut first = f64::INFINITY;
        let mut second = f64::INFINITY;
        for margin in sample.margins() {
            if margin.distance <= 0. || margin.speed >= 0. {
                continue;
            }
            let t = -margin.distance / margin.speed;
            if t < first {
                second = first;
                first = t;
            } else if t < second {
                second = t;
            }
        }
        // Stop after the estimated first crossing, before the next estimate. The
        // estimate decides only where to retry; the native time grid never changes.
        let target = (1.25 * first).min(first + 0.5 * (second - first));
        (target > 0. && target.is_finite() && target < span).then_some(target)
    }
    fn bound_piece(
        self,
        h: f64,
        skew: bool,
        launch: bool,
    ) -> Result<Option<(FiniteArcResult, Option<SlipGrowth>)>, PhysicsError> {
        if self.locked {
            return Ok(self.sticking_bound(h, true)?.map(|s| (s.motion, None)));
        }
        if let Some(s) = self.sliding_bound(h, true)? {
            return Ok(Some((s.motion, None)));
        }
        if skew {
            return Ok(self
                .sliding_skew_bound(h, true, launch)?
                .map(|s| (s.motion, s.slip_growth)));
        }
        Ok(None)
    }
    fn ordinary_trial(
        self,
        h: f64,
        skew: bool,
        launch: bool,
        subdivide: bool,
    ) -> Result<PointTrial, PhysicsError> {
        let result = self.bound_piece(h, skew, launch)?;
        if result.is_none() && subdivide {
            self.short_trial(h, skew, launch)
        } else {
            Ok(PointTrial { span: h, result })
        }
    }
    fn restart_piece(
        self,
        h: f64,
        skew: bool,
        launch: bool,
    ) -> Result<Option<(FiniteArcResult, Option<SlipGrowth>)>, PhysicsError> {
        Self {
            require_restart_precision: true,
            ..self
        }
        .bound_piece(h, skew, launch)
    }
    fn short_trial(self, h: f64, skew: bool, launch: bool) -> Result<PointTrial, PhysicsError> {
        let mut span = h;
        for _ in 0..4 {
            span *= 0.5;
            if let Some(motion) = self.restart_piece(span, skew, launch)? {
                return Ok(PointTrial {
                    span,
                    result: Some(motion),
                });
            }
        }
        Ok(PointTrial { span, result: None })
    }
    fn transfers(
        self,
        h: f64,
        skew: bool,
        launch: bool,
        subdivide: bool,
        directed: bool,
        mut first: Option<PointTrial>,
    ) -> Result<Option<BoundPointPath>, PhysicsError> {
        if !h.is_finite() || h <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let mut arc = self;
        let m = self.material;
        let initial = self.start();
        let speed = (initial
            .v
            .length()
            .hypot((m.inertia / m.mass).sqrt() * initial.w.length())
            + h * self.flight.acceleration.length())
            * (1. + 3e-12)
            + 1e-13;
        let angular = speed * (m.mass / m.inertia).sqrt() * (1. + 3e-12) + 1e-13;
        let mut pieces = Vec::new();
        let mut t = 0.;
        let mut translations = 0.;
        let mut impacts = 0;
        let mut subintervals = 0;
        loop {
            let left = h - t;
            let shorten = subdivide && subintervals < 16;
            let PointTrial { span, result } = if let Some(known) = first.take() {
                // The first native state, material and duration are identical on
                // retry. A shorter ordinary result implies that the full trial
                // already failed. Only the new material-directed proposal can
                // change this decision. Never reuse it after advancing the arc.
                if directed
                    && known.span != left
                    && let Some(span) = arc.feature_trial(left)
                    && span != known.span
                    && let Some(motion) = arc.restart_piece(span, skew, launch)?
                {
                    PointTrial {
                        span,
                        result: Some(motion),
                    }
                } else {
                    known
                }
            } else if directed && shorten {
                let full = arc.bound_piece(left, skew, launch)?;
                if full.is_some() {
                    PointTrial {
                        span: left,
                        result: full,
                    }
                } else if let Some(span) = arc.feature_trial(left)
                    && let Some(motion) = arc.restart_piece(span, skew, launch)?
                {
                    PointTrial {
                        span,
                        result: Some(motion),
                    }
                } else {
                    arc.short_trial(left, skew, launch)?
                }
            } else {
                arc.ordinary_trial(left, skew, launch, shorten)?
            };
            let Some((motion, growth)) = result else {
                return Ok(None);
            };
            translations += motion.initial_translation.abs();
            let mut piece = BoundPointPiece {
                start: t,
                sticking: arc.locked,
                slip_growth: growth,
                motion,
                impact: None,
            };
            match motion.end {
                FiniteArcEnd::Horizon if span < left => {
                    let end = t + span;
                    if end <= t || end >= h || motion.bracket != [span, span] {
                        return Ok(None);
                    }
                    let tol = 1024.
                        * f64::EPSILON
                        * (1.
                            + motion.center.length()
                            + arc.flight.support.center.length()
                            + motion.moving.half_extent.length());
                    let flight = FiniteBoxFlight {
                        moving: motion.moving,
                        center_of_mass: motion.center,
                        relative_velocity: motion.velocity,
                        angular_velocity: motion.angular_velocity,
                        ..arc.flight
                    };
                    let Some(next) = Self::with_track(
                        flight,
                        motion.site.site,
                        arc.track,
                        m,
                        arc.locked,
                        tol,
                        false,
                    )?
                    else {
                        return Ok(None);
                    };
                    // A smooth subdivision may not introduce a new contact mode.
                    if next.locked != arc.locked {
                        return Ok(None);
                    }
                    pieces.push(piece);
                    subintervals += 1;
                    arc = next;
                    t = end;
                }
                FiniteArcEnd::Horizon => {
                    if pieces.is_empty() {
                        return Ok(None);
                    }
                    pieces.push(piece);
                    return Ok(Some(BoundPointPath {
                        pieces,
                        center_speed_bound: speed,
                        angular_speed_bound: angular,
                        translation_allowance: translations,
                    }));
                }
                FiniteArcEnd::Feature(limit) if impacts < 4 => {
                    let tol = 1024.
                        * f64::EPSILON
                        * (1.
                            + motion.center.length()
                            + arc.flight.support.center.length()
                            + motion.moving.half_extent.length());
                    let Some(segment) =
                        arc.track
                            .incident_segment(motion.moving, motion.site, limit, tol)
                    else {
                        return Ok(None);
                    };
                    let Some((law, boxes)) = segment.spherical_endpoint_with_work(
                        motion.center,
                        motion.velocity,
                        motion.angular_velocity,
                        m,
                        128,
                    )?
                    else {
                        return Ok(None);
                    };
                    let r = segment.moving_points[law.loaded_end] - motion.center;
                    let next = FiniteBoxFlight {
                        moving: motion.moving,
                        center_of_mass: motion.center,
                        relative_velocity: motion.velocity + law.impulse / m.mass,
                        angular_velocity: motion.angular_velocity
                            + r.cross(law.impulse) / m.inertia,
                        ..arc.flight
                    };
                    let Some(next) = Self::after_line_endpoint(
                        next,
                        segment,
                        law.loaded_end,
                        m,
                        law.sticking,
                        tol,
                    )?
                    else {
                        return Ok(None);
                    };
                    let end = t + motion.bracket[0];
                    if end <= t + 2e-14 * h || end >= h {
                        return Ok(None);
                    }
                    impacts += 1;
                    piece.impact = Some(BoundPointImpact {
                        segment,
                        law,
                        work_boxes: boxes,
                    });
                    pieces.push(piece);
                    arc = next;
                    t = end;
                }
                _ => return Ok(None),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Two measured cuts exercise opposite orders: sliding vertex to sliding
    // vertex, and a sticking skew pair to a sliding vertex. The reference times
    // and impulses were integrated on a separate 128-subdivision event history.
    struct Case {
        flight: FiniteBoxFlight,
        time: f64,
        impulse: DVec3,
        test_end: f64,
    }
    fn cuts() -> [Case; 2] {
        [
            Case {
                flight: FiniteBoxFlight {
                    moving: FlightBox {
                        center: DVec3::from_array([
                            2.0188724268320794e-05,
                            -5.056411527575514e-05,
                            0.042447108629315194,
                        ]),
                        half_extent: DVec3::new(0.18, 0.10, 0.06),
                        orientation: DQuat::from_array([
                            1.5543291518739182e-05,
                            -6.990103397446323e-10,
                            -4.4971834682407515e-05,
                            0.9999999988679702,
                        ]),
                    },
                    support: FlightBox {
                        center: DVec3::from_array([0.165, 0.0, -0.07755600000000001]),
                        half_extent: DVec3::new(0.33, 0.18, 0.06),
                        orientation: DQuat::IDENTITY,
                    },
                    center_of_mass: DVec3::from_array([
                        2.0188528545426214e-05,
                        -5.2740176081514935e-05,
                        0.11244710859551396,
                    ]),
                    relative_velocity: DVec3::from_array([
                        0.17429228627610183,
                        0.10560006937176643,
                        -0.4024603466911702,
                    ]),
                    angular_velocity: DVec3::from_array([
                        -1.0808739431654377,
                        1.7839839209817547,
                        -7.078736788337457e-07,
                    ]),
                    acceleration: -9.81 * DVec3::Z,
                },
                time: 1.7382974869375666e-05,
                impulse: DVec3::from_array([
                    -0.1491537658220493,
                    -0.07166354410051572,
                    0.33095322577032105,
                ]),
                test_end: 2.1e-05,
            },
            Case {
                flight: FiniteBoxFlight {
                    moving: FlightBox {
                        center: DVec3::from_array([
                            9.93065012964176e-05,
                            1.656484608933943e-05,
                            0.0406027019771692,
                        ]),
                        half_extent: DVec3::new(0.18, 0.10, 0.06),
                        orientation: DQuat::from_array([
                            -2.135352557839242e-12,
                            -3.1883441453003736e-05,
                            6.697362190324367e-08,
                            0.9999999994917209,
                        ]),
                    },
                    support: FlightBox {
                        center: DVec3::from_array([0.225, 0.0, -0.07940399999999864]),
                        half_extent: DVec3::new(0.33, 0.18, 0.06),
                        orientation: DQuat::IDENTITY,
                    },
                    center_of_mass: DVec3::from_array([
                        9.48428194952652e-05,
                        1.6564846089339855e-05,
                        0.11060270183486186,
                    ]),
                    relative_velocity: DVec3::from_array([
                        0.19051386850411894,
                        -0.1812782116284144,
                        -0.293445208113893,
                    ]),
                    angular_velocity: DVec3::from_array([
                        1.3943758983106842,
                        1.4654157500168217,
                        2.7573676852329153e-10,
                    ]),
                    acceleration: -9.81 * DVec3::Z,
                },
                time: 3.301789600100724e-05,
                impulse: DVec3::from_array([
                    -0.22280580573723827,
                    0.12377837511982075,
                    0.5097588183348005,
                ]),
                test_end: 4.5e-05,
            },
        ]
    }
    #[test]
    fn resolves_both_orders_of_material_contact_without_changing_the_external_clock() {
        for c in cuts() {
            for scale in [0.3, 1., 8.] {
                let m = SphericalPointMaterial {
                    mass: 2. * scale,
                    inertia: 0.05 * scale,
                    friction: 0.5,
                };
                let f = c.flight;
                let (_, s) = f.incident_segment_now(0.).unwrap();
                let (h, _) = s
                    .spherical_endpoint_with_work(
                        f.center_of_mass,
                        f.relative_velocity,
                        f.angular_velocity,
                        m,
                        128,
                    )
                    .unwrap()
                    .unwrap();
                let after = FiniteBoxFlight {
                    relative_velocity: f.relative_velocity + h.impulse / m.mass,
                    angular_velocity: f.angular_velocity
                        + (s.moving_points[h.loaded_end] - f.center_of_mass).cross(h.impulse)
                            / m.inertia,
                    ..f
                };
                let arc = FiniteFeatureArc::after_line_endpoint(
                    after,
                    s,
                    h.loaded_end,
                    m,
                    h.sticking,
                    2e-13,
                )
                .unwrap()
                .unwrap();
                assert!(
                    arc.resolved_point_transfers(c.time * 0.9)
                        .unwrap()
                        .is_none()
                );
                assert!(arc.vertex_handoff_hint(c.test_end).unwrap().is_none());
                let path = arc
                    .resolved_point_transfers(c.test_end)
                    .unwrap()
                    .expect("finite isolated transition");
                assert_eq!(path.pieces.len(), 2);
                let first = path.pieces[0];
                let next = path.pieces[1];
                let hit = first.impact.unwrap();
                assert!((next.start - c.time).abs() < 1e-12);
                assert!(hit.law.impulse.distance(c.impulse * scale) < 1e-10 * scale);
                assert!(
                    hit.law.velocity_after[1 - hit.law.loaded_end].dot(hit.segment.normal) > 0.
                );
                assert!(
                    next.impact.is_none()
                        && (next.start + next.motion.bracket[0] - c.test_end).abs() < 2e-16
                );
                let j = first.motion.impulse + hit.law.impulse + next.motion.impulse;
                let tau = first.motion.angular_impulse
                    + (hit.segment.moving_points[hit.law.loaded_end] - first.motion.center)
                        .cross(hit.law.impulse)
                    + next.motion.angular_impulse;
                assert!(
                    j.distance(
                        m.mass
                            * (next.motion.velocity
                                - after.relative_velocity
                                - c.test_end * f.acceleration)
                    ) < 1e-11 * scale
                );
                assert!(
                    tau.distance(
                        m.inertia * (next.motion.angular_velocity - after.angular_velocity)
                    ) < 1e-11 * scale
                );
            }
        }
    }
    #[test]
    fn declines_reversal_and_simultaneous_or_initially_ambiguous_limits() {
        let m = |i, d, v| FeatureMargin {
            limit: FeatureLimit::MovingNeighbor(i),
            distance: d,
            speed: v,
        };
        let mut a = None;
        assert!(!unique_closing(&mut a, &m(0, 0., -1.), 0.1, 1., 1e-12));
        assert!(!unique_closing(&mut a, &m(0, 0.01, -0.1), 0.1, 2., 1e-12));
        assert!(unique_closing(&mut a, &m(0, 0.01, -1.), 0.1, 2., 1e-12));
        assert!(!unique_closing(&mut a, &m(1, 0.01, -1.), 0.1, 2., 1e-12));
    }

    #[test]
    fn a_sticking_endpoint_impulse_launches_a_sliding_edge_across_native_grids() {
        // Matched incoming state of fixture 80, stroke 2, in the translating
        // deck frame. Reference event and impulse use 512 integration divisions.
        let flight = FiniteBoxFlight {
            moving: FlightBox {
                center: DVec3::from_array([
                    0.00026136224182831894,
                    -1.5172064143227696e-05,
                    0.026084343211038297,
                ]),
                orientation: DQuat::from_array([
                    6.488351584286034e-05,
                    -4.899938303607086e-06,
                    -0.00039294368289203186,
                    0.9999999206806879,
                ]),
                half_extent: DVec3::new(0.18, 0.10, 0.06),
            },
            support: FlightBox {
                center: DVec3::from_array([0.19, 0.0, -0.09392999999999536]),
                orientation: DQuat::IDENTITY,
                half_extent: DVec3::new(0.33, 0.18, 0.06),
            },
            center_of_mass: DVec3::from_array([
                0.0002606726811407517,
                -2.425548608474602e-05,
                0.0960843426183144,
            ]),
            relative_velocity: DVec3::from_array([
                0.3130994928514472,
                0.22286142864046832,
                -0.045412532940603656,
            ]) - 0.522 * DVec3::Z,
            angular_velocity: DVec3::from_array([
                -1.911000333698728,
                2.6846796864809988,
                -1.173853060623508e-06,
            ]),
            acceleration: -9.81 * DVec3::Z,
        };
        let (feature, normal, [point, _], _) = flight.contact_geometry(4e-13).unwrap();
        let site = FinitePointSite {
            feature,
            normal,
            point,
        };
        let m = SphericalPointMaterial {
            mass: 2.,
            inertia: 0.05,
            friction: 0.5,
        };
        let arc = FiniteFeatureArc::new(flight, site, m, false, 4e-13)
            .unwrap()
            .unwrap();
        let bin = 62.5e-6;
        let first = arc
            .directed_point_transfers(bin, true, true)
            .unwrap()
            .unwrap();
        let end = first.pieces.last().unwrap().motion;
        let restarted = FiniteFeatureArc::new(
            FiniteBoxFlight {
                moving: end.moving,
                center_of_mass: end.center,
                relative_velocity: end.velocity,
                angular_velocity: end.angular_velocity,
                ..flight
            },
            end.site.site,
            m,
            false,
            4e-13,
        )
        .unwrap()
        .unwrap();
        for (start, h, arc) in [(0., 500e-6, arc), (bin, bin, restarted)] {
            let path = arc
                .directed_point_transfers(h, true, true)
                .unwrap()
                .unwrap();
            let (i, hit) = path
                .pieces
                .iter()
                .enumerate()
                .filter_map(|(i, p)| p.impact.map(|hit| (i, hit)))
                .next_back()
                .unwrap();
            assert!(hit.law.sticking && hit.law.opening_lower_bound > 0.);
            assert!(
                hit.law.impulse.distance(DVec3::new(
                    -0.00991499524410628,
                    -0.010068370032446523,
                    0.18015849297037806
                )) < 2e-11
            );
            let launch = path.pieces[i + 1];
            assert!((start + launch.start - 92.0702635585503e-6).abs() < 2e-13);
            assert_eq!(
                launch.motion.site.site.feature,
                crate::SeparatingFeature::EdgePair(0, 1)
            );
            assert!(!launch.sticking);
            let witness = launch.slip_growth.unwrap();
            assert_eq!(witness.initial_component, 0.);
            assert!(witness.acceleration > 0.);
            let port = launch.motion.velocity
                + launch
                    .motion
                    .angular_velocity
                    .cross(launch.motion.site.site.point - launch.motion.center);
            assert!(port.dot(witness.direction) >= witness.acceleration * launch.motion.bracket[0]);
            let last = path.pieces.last().unwrap();
            assert!((last.start + last.motion.bracket[0] - h).abs() < 1e-16);
            if start == 0. {
                let end = last.motion;
                let next = FiniteFeatureArc::new(
                    FiniteBoxFlight {
                        moving: end.moving,
                        center_of_mass: end.center,
                        relative_velocity: end.velocity,
                        angular_velocity: end.angular_velocity,
                        ..flight
                    },
                    end.site.site,
                    m,
                    false,
                    4e-13,
                )
                .unwrap()
                .expect("the public endpoint supplies a valid sliding start for the next tick");
                assert!(
                    next.sliding_skew_launch_span(h).unwrap().is_some()
                        || next
                            .directed_point_transfers(h, true, true)
                            .unwrap()
                            .is_some()
                );
            }
            assert!(
                path.pieces
                    .iter()
                    .all(|p| p.motion.normal_impulse > 0. && p.motion.friction_work <= 0.)
            );
            let mut impulse = DVec3::ZERO;
            let mut torque = DVec3::ZERO;
            for piece in &path.pieces {
                impulse += piece.motion.impulse;
                torque += piece.motion.angular_impulse;
                if let Some(hit) = piece.impact {
                    impulse += hit.law.impulse;
                    torque += (hit.segment.moving_points[hit.law.loaded_end] - piece.motion.center)
                        .cross(hit.law.impulse);
                }
            }
            assert!(
                impulse.distance(
                    m.mass
                        * (last.motion.velocity
                            - arc.flight.relative_velocity
                            - h * flight.acceleration)
                ) < 2e-12
            );
            assert!(
                torque.distance(
                    m.inertia * (last.motion.angular_velocity - arc.flight.angular_velocity)
                ) < 2e-12
            );
        }
    }

}
