// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Track the finite extents of an already identified vertex/face or edge/edge pair.
use crate::{FiniteBoxFlight, FinitePointSite, FlightBox, PhysicsError, SeparatingFeature};
use glam::DVec3;

/// A material feature stops being isolated when a neighbor reaches the separating
/// plane or the closest point reaches the end of one of the finite features.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureLimit {
    MovingNeighbor(usize),
    SupportNeighbor(usize),
    MovingExtent(usize, i8),
    SupportExtent(usize, i8),
}
#[derive(Debug, Clone, Copy)]
pub struct FeatureMargin {
    pub limit: FeatureLimit,
    /// Positive in the interior of the incident feature, in metres.
    pub distance: f64,
    pub speed: f64,
}
#[derive(Debug, Clone, Copy)]
pub struct FiniteFeatureSample {
    pub site: FinitePointSite,
    pub support_point: DVec3,
    pub normal_rate: DVec3,
    pub gap: f64,
    /// Holds at most eight distinct faces or endpoints.
    margins: [FeatureMargin; 8],
    count: usize,
}
impl FiniteFeatureSample {
    pub fn margins(&self) -> &[FeatureMargin] {
        &self.margins[..self.count]
    }
    fn add(&mut self, limit: FeatureLimit, distance: f64, speed: f64) {
        self.margins[self.count] = FeatureMargin {
            limit,
            distance,
            speed,
        };
        self.count += 1
    }
    /// First-order timing estimate only. A numerical schedule using this hint
    /// must keep the ordinary collision solve active at every boundary.
    pub fn approaching_limit(&self) -> Option<(FeatureLimit, f64)> {
        self.margins()
            .iter()
            .filter(|m| m.distance > 0. && m.speed < 0.)
            .map(|m| (m.limit, -m.distance / m.speed))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
}
#[derive(Debug, Clone, Copy)]
pub struct FiniteFeatureTrack {
    feature: SeparatingFeature,
    orientation_sign: f64,
    moving_local: DVec3,
    support_local: DVec3,
    support: FlightBox,
}
const AXES: [DVec3; 3] = [DVec3::X, DVec3::Y, DVec3::Z];
impl FiniteFeatureTrack {
    pub fn new(f: FiniteBoxFlight, site: FinitePointSite) -> Result<Self, PhysicsError> {
        use SeparatingFeature::*;
        let axes = |b: FlightBox| AXES.map(|x| b.orientation * x);
        let (a, b) = (axes(f.moving), axes(f.support));
        let get = |v: [DVec3; 3], i| v.get(i).copied().ok_or(PhysicsError::InvalidValue);
        let raw = match site.feature {
            SupportFace(j) => get(b, j)?,
            MovingFace(i) => get(a, i)?,
            EdgePair(i, j) => get(a, i)?.cross(get(b, j)?),
        };
        let n = site.normal;
        if !n.is_finite()
            || (n.length_squared() - 1.).abs() > 1e-10
            || raw.length() < 1e-7
            || (raw.normalize().dot(n).abs() - 1.).abs() > 1e-10
        {
            return Err(PhysicsError::InvalidValue);
        }
        let corner =
            |b: FlightBox, d: DVec3| b.half_extent * (b.orientation.inverse() * d).signum();
        let mut moving_local = corner(f.moving, -n);
        let mut support_local = corner(f.support, n);
        match site.feature {
            SupportFace(j) => {
                for i in 0..3 {
                    if i != j {
                        support_local[i] = 0.
                    }
                }
            }
            MovingFace(i) => {
                for j in 0..3 {
                    if j != i {
                        moving_local[j] = 0.
                    }
                }
            }
            EdgePair(i, j) => {
                moving_local[i] = 0.;
                support_local[j] = 0.;
            }
        }
        Ok(Self {
            feature: site.feature,
            orientation_sign: raw.dot(n).signum(),
            moving_local,
            support_local,
            support: f.support,
        })
    }
    /// Instantaneous signed distances and derivatives for the same incident
    /// material features, even after their finite bounds have been crossed.
    pub fn sample(
        self,
        moving: FlightBox,
        center: DVec3,
        velocity: DVec3,
        w: DVec3,
    ) -> Option<FiniteFeatureSample> {
        use SeparatingFeature::*;
        let a = AXES.map(|e| moving.orientation * e);
        let b = AXES.map(|e| self.support.orientation * e);
        let (p, q) = (
            moving.center + moving.orientation * self.moving_local,
            self.support.center + self.support.orientation * self.support_local,
        );
        let dp = velocity + w.cross(p - center);
        let sign = self.orientation_sign;
        let mut endpoints = None;
        let (n, dn, point, support_point, point_rate, support_rate) = match self.feature {
            SupportFace(j) => {
                let n = sign * b[j];
                let gap = (p - q).dot(n);
                (n, DVec3::ZERO, p, p - gap * n, dp, dp - n * dp.dot(n))
            }
            MovingFace(i) => {
                let n = sign * a[i];
                let dn = w.cross(n);
                let gap = (p - q).dot(n);
                let dg = dp.dot(n) + (p - q).dot(dn);
                (n, dn, q + gap * n, q, dg * n + gap * dn, DVec3::ZERO)
            }
            EdgePair(i, j) => {
                let (u, v) = (a[i], b[j]);
                let du = w.cross(u);
                let line = u.cross(v);
                let dline = du.cross(v);
                let den = line.length_squared();
                if den < 1e-14 {
                    return None;
                }
                let n = sign * line / den.sqrt();
                let dn = sign * (dline - n * n.dot(dline)) / den.sqrt();
                let delta = q - p;
                let h = delta.cross(v);
                let s = h.dot(line) / den;
                let t = u.dot(v) * s - v.dot(delta);
                let ds = ((-dp).cross(v).dot(line) + h.dot(dline) - s * 2. * line.dot(dline)) / den;
                let dt = du.dot(v) * s + u.dot(v) * ds + v.dot(dp);
                endpoints = Some((i, j, s, t, ds, dt));
                (n, dn, p + s * u, q + t * v, dp + ds * u + s * du, dt * v)
            }
        };
        let mut out = FiniteFeatureSample {
            site: FinitePointSite {
                normal: n,
                point,
                feature: self.feature,
            },
            normal_rate: dn,
            support_point,
            gap: n.dot(point - support_point),
            margins: [FeatureMargin {
                limit: FeatureLimit::MovingNeighbor(0),
                distance: 0.,
                speed: 0.,
            }; 8],
            count: 0,
        };
        let foot_center_rate = velocity + w.cross(moving.center - center);
        for k in 0..3 {
            if self.moving_local[k] != 0. && !matches!(self.feature, MovingFace(_)) {
                let arc = -2. * self.moving_local[k] * a[k];
                out.add(
                    FeatureLimit::MovingNeighbor(k),
                    arc.dot(n),
                    w.cross(arc).dot(n) + arc.dot(dn),
                );
            }
            if self.support_local[k] != 0. && !matches!(self.feature, SupportFace(_)) {
                let arc = 2. * self.support_local[k] * b[k];
                out.add(FeatureLimit::SupportNeighbor(k), arc.dot(n), arc.dot(dn));
            }
            match self.feature {
                SupportFace(j) if j != k => {
                    let s = b[k].dot(support_point - self.support.center);
                    let ds = b[k].dot(support_rate);
                    for side in [-1_i8, 1] {
                        let e = f64::from(side);
                        out.add(
                            FeatureLimit::SupportExtent(k, side),
                            self.support.half_extent[k] - e * s,
                            -e * ds,
                        );
                    }
                }
                MovingFace(i) if i != k => {
                    let delta = point - moving.center;
                    let s = a[k].dot(delta);
                    let ds = a[k].dot(point_rate - foot_center_rate) + w.cross(a[k]).dot(delta);
                    for side in [-1_i8, 1] {
                        let e = f64::from(side);
                        out.add(
                            FeatureLimit::MovingExtent(k, side),
                            moving.half_extent[k] - e * s,
                            -e * ds,
                        );
                    }
                }
                _ => {}
            }
        }
        if let Some((i, j, s, t, ds, dt)) = endpoints {
            for side in [-1_i8, 1] {
                let e = f64::from(side);
                out.add(
                    FeatureLimit::MovingExtent(i, side),
                    moving.half_extent[i] - e * s,
                    -e * ds,
                );
                out.add(
                    FeatureLimit::SupportExtent(j, side),
                    self.support.half_extent[j] - e * t,
                    -e * dt,
                );
            }
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    #[test]
    fn a_releasing_line_selects_the_outgoing_material_feature() {
        // The inclined shoe first meets the horizontal deck along its local X
        // edge. Wide decks retain the shoe vertex. A narrow deck clips that
        // vertex, leaving a genuine crossing between two finite edges.
        for half_width in [0.07, 0.5] {
            for spin in [-1.1, 1.1] {
                for yaw in [-0.42, 0.63] {
                    let world = DQuat::from_rotation_z(yaw);
                    let q = world * DQuat::from_rotation_x(0.29);
                    let h = DVec3::new(0.16, 0.09, 0.05);
                    let c = -(q * DVec3::new(0., -h.y, -h.z));
                    let deck = FlightBox {
                        center: -0.1 * DVec3::Z,
                        orientation: world,
                        half_extent: DVec3::new(half_width, 0.5, 0.1),
                    };
                    let mut f = FiniteBoxFlight {
                        moving: FlightBox {
                            center: c,
                            orientation: q,
                            half_extent: h,
                        },
                        support: deck,
                        center_of_mass: c,
                        relative_velocity: DVec3::ZERO,
                        angular_velocity: world * (spin * DVec3::Y),
                        acceleration: -9.81 * DVec3::Z,
                    };
                    let (_, seg) = f.incident_segment_now(0.).unwrap();
                    let end = usize::from(
                        (seg.moving_points[1] - seg.moving_points[0]).dot(world * DVec3::X) * spin
                            > 0.,
                    );
                    f.relative_velocity = -f.angular_velocity.cross(seg.moving_points[end] - c);
                    let (track, s) = FiniteFeatureTrack::outgoing_endpoint(f, seg, end).unwrap();
                    assert_eq!(
                        s.site.feature,
                        if half_width < h.x {
                            SeparatingFeature::EdgePair(0, 1)
                        } else {
                            SeparatingFeature::SupportFace(2)
                        }
                    );
                    assert!(s.site.point.distance(seg.moving_points[end]) < 1e-13);
                    let t = 1e-7;
                    let next = track
                        .sample(
                            f.at(t),
                            f.com_at(t),
                            f.relative_velocity + t * f.acceleration,
                            f.angular_velocity,
                        )
                        .unwrap();
                    for (a, b) in s.margins().iter().zip(next.margins()) {
                        assert!(b.distance > 0.);
                        assert!(((b.distance - a.distance) / t - a.speed).abs() < 1e-5);
                    }
                    assert!(FiniteFeatureTrack::outgoing_endpoint(f, seg, 1 - end).is_none());
                    f.angular_velocity = DVec3::ZERO;
                    f.relative_velocity = DVec3::ZERO;
                    assert!(FiniteFeatureTrack::outgoing_endpoint(f, seg, end).is_none());
                }
            }
        }
    }
    #[test]
    fn all_finite_margins_have_the_measured_material_derivative() {
        use SeparatingFeature::*;
        let a = FlightBox {
            center: DVec3::new(-0.11, 0.05, 0.23),
            orientation: DQuat::from_rotation_y(0.25) * DQuat::from_rotation_x(-0.18),
            half_extent: DVec3::new(0.18, 0.1, 0.06),
        };
        let b = FlightBox {
            center: DVec3::new(0.05, -0.02, -0.08),
            orientation: DQuat::from_rotation_z(0.3),
            half_extent: DVec3::new(0.25, 0.12, 0.05),
        };
        let f = FiniteBoxFlight {
            moving: a,
            support: b,
            center_of_mass: a.center + 0.07 * DVec3::Z,
            angular_velocity: DVec3::new(3.1, -2.4, 1.87),
            relative_velocity: DVec3::new(-0.4, 1.2, -0.85),
            acceleration: -9.81 * DVec3::Z,
        };
        for (which, n, count) in [
            (SupportFace(2), b.orientation * DVec3::Z, 7),
            (MovingFace(2), a.orientation * DVec3::Z, 7),
            (
                EdgePair(0, 1),
                (a.orientation * DVec3::X)
                    .cross(b.orientation * DVec3::Y)
                    .normalize(),
                8,
            ),
        ] {
            for sign in [-1., 1.] {
                let track = FiniteFeatureTrack::new(
                    f,
                    FinitePointSite {
                        feature: which,
                        normal: sign * n,
                        point: DVec3::ZERO,
                    },
                )
                .unwrap();
                let sample = |t| {
                    track
                        .sample(
                            f.at(t),
                            f.com_at(t),
                            f.relative_velocity + t * f.acceleration,
                            f.angular_velocity,
                        )
                        .unwrap()
                };
                let h = 2e-6;
                let (left, mid, right) = (sample(-h), sample(0.), sample(h));
                assert_eq!(mid.margins().len(), count);
                assert!(
                    ((right.site.normal - left.site.normal) / (2. * h)).distance(mid.normal_rate)
                        < 1e-8
                );
                for ((a, b), c) in left
                    .margins()
                    .iter()
                    .zip(mid.margins())
                    .zip(right.margins())
                {
                    assert_eq!(a.limit, c.limit);
                    assert!(((c.distance - a.distance) / (2. * h) - b.speed).abs() < 1e-8)
                }
            }
        }
    }
}

/// Endpoints of the line of geometrically incident material after a neighbor
/// becomes coplanar. Both extreme points are needed to check nonpenetration.
#[derive(Debug, Clone, Copy)]
pub struct FiniteContactSegment {
    pub moving_points: [DVec3; 2],
    pub support_points: [DVec3; 2],
    pub normal: DVec3,
}
impl FiniteFeatureTrack {
    /// Clip the new material edge against the finite incident face. A second
    /// simultaneous change, unresolved tangent/coincident line, or endpoint
    /// leaving a face has no isolated edge/face segment certificate.
    pub fn incident_segment(
        self,
        moving: FlightBox,
        sample: FiniteFeatureSample,
        limit: FeatureLimit,
        tolerance: f64,
    ) -> Option<FiniteContactSegment> {
        use FeatureLimit::*;
        use SeparatingFeature::*;
        if !tolerance.is_finite() || tolerance < 0. || sample.site.feature != self.feature {
            return None;
        }
        let mut active = sample.margins().iter().filter(|m| m.distance <= tolerance);
        if active.next()?.limit != limit || active.next().is_some() {
            return None;
        }
        let (edge_moves, e, f) = match (self.feature, limit) {
            (SupportFace(j), MovingNeighbor(i)) => (true, i, j),
            (MovingFace(i), SupportNeighbor(j)) => (false, j, i),
            (EdgePair(i, j), SupportNeighbor(k)) if k != j => (true, i, 3 - j - k),
            (EdgePair(i, j), MovingNeighbor(k)) if k != i => (false, j, 3 - i - k),
            _ => return None,
        };
        let (edge, face, le, lf) = if edge_moves {
            (moving, self.support, self.moving_local, self.support_local)
        } else {
            (self.support, moving, self.support_local, self.moving_local)
        };
        clip_edge_face(
            EdgeFace {
                edge,
                face,
                edge_axis: e,
                face_axis: f,
                edge_local: le,
                face_local: lf,
                edge_moves,
            },
            sample.site.normal,
            tolerance,
        )
    }
}
struct EdgeFace {
    edge: FlightBox,
    face: FlightBox,
    edge_axis: usize,
    face_axis: usize,
    edge_local: DVec3,
    face_local: DVec3,
    edge_moves: bool,
}
fn clip_edge_face(pair: EdgeFace, previous: DVec3, tolerance: f64) -> Option<FiniteContactSegment> {
    let EdgeFace {
        edge,
        face,
        edge_axis: e,
        face_axis: f,
        edge_local: mut le,
        face_local: lf,
        edge_moves,
    } = pair;
    le[e] = 0.;
    let origin = edge.center + edge.orientation * le;
    let u = edge.orientation * AXES[e];
    let center = face.center + face.orientation * (AXES[f] * lf[f]);
    let nf = face.orientation * AXES[f];
    let n = nf * nf.dot(previous).signum();
    let mut bounds = [-edge.half_extent[e], edge.half_extent[e]];
    for (i, axis) in AXES.iter().enumerate() {
        if i != f {
            let a = face.orientation * *axis;
            let slope = a.dot(u);
            let offset = a.dot(origin - center);
            let h = face.half_extent[i];
            if slope.abs() * edge.half_extent[e] <= 64. * f64::EPSILON * (1. + h) {
                if offset.abs() > h + tolerance {
                    return None;
                }
            } else {
                let x = (-h - offset) / slope;
                let y = (h - offset) / slope;
                bounds[0] = bounds[0].max(x.min(y));
                bounds[1] = bounds[1].min(x.max(y));
            }
        }
    }
    if bounds[1] - bounds[0] <= 2. * tolerance {
        return None;
    }
    let edge_points = bounds.map(|s| origin + s * u);
    if edge_points
        .iter()
        .any(|p| n.dot(*p - center).abs() > 2. * tolerance)
    {
        return None;
    }
    let face_points = edge_points.map(|p| p - n * n.dot(p - center));
    let (moving_points, support_points) = if edge_moves {
        (edge_points, face_points)
    } else {
        (face_points, edge_points)
    };
    Some(FiniteContactSegment {
        moving_points,
        support_points,
        normal: n,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeFaceFeature {
    MovingEdge { edge: usize, support_face: usize },
    SupportEdge { edge: usize, moving_face: usize },
}

impl FiniteBoxFlight {
    /// Detect a geometrically isolated, nonzero-length line of incidence on
    /// the measured boxes at a public boundary. Tied faces or two distinct
    /// admissible normals decline. No contact skin participates in this query.
    pub fn incident_segment_now(
        self,
        allowance: f64,
    ) -> Option<(EdgeFaceFeature, FiniteContactSegment)> {
        if !allowance.is_finite() || allowance < 0. || !self.moving.valid() || !self.support.valid()
        {
            return None;
        }
        let scale = 1.
            + self.moving.center.abs().max_element()
            + self.support.center.abs().max_element()
            + self.moving.half_extent.length()
            + self.support.half_extent.length();
        let tol = allowance + 128. * f64::EPSILON * scale;
        if !tol.is_finite() {
            return None;
        }
        let mut result = None;
        for edge_moves in [true, false] {
            let (edge, face) = if edge_moves {
                (self.moving, self.support)
            } else {
                (self.support, self.moving)
            };
            let a = AXES.map(|i| edge.orientation * i);
            let b = AXES.map(|i| face.orientation * i);
            for f in 0..3 {
                // Outward normal from the face through the candidate incident edge.
                let n = b[f] * (edge.center - face.center).dot(b[f]).signum();
                let height = (edge.center - face.center).dot(n)
                    - face.half_extent[f]
                    - a.iter()
                        .enumerate()
                        .map(|(i, u)| edge.half_extent[i] * u.dot(n).abs())
                        .sum::<f64>();
                if height.abs() > tol {
                    continue;
                }
                let mut parallel = None;
                let mut ambiguous = false;
                let mut le = DVec3::ZERO;
                for (i, u) in a.iter().enumerate() {
                    let d = u.dot(n);
                    let span = 2. * edge.half_extent[i] * d.abs();
                    if span <= tol {
                        if parallel.replace(i).is_some() {
                            ambiguous = true
                        }
                    } else if span <= 8. * tol {
                        ambiguous = true
                    } else {
                        le[i] = -d.signum() * edge.half_extent[i]
                    }
                }
                let Some(e) = parallel else { continue };
                if ambiguous {
                    continue;
                }
                let nf = if edge_moves { n } else { -n };
                let lf = AXES[f] * (n.dot(b[f]).signum() * face.half_extent[f]);
                let Some(segment) = clip_edge_face(
                    EdgeFace {
                        edge,
                        face,
                        edge_axis: e,
                        face_axis: f,
                        edge_local: le,
                        face_local: lf,
                        edge_moves,
                    },
                    nf,
                    tol,
                ) else {
                    continue;
                };
                // Ensure the segment has interior on the selected finite face.
                // An entire segment on a face rim can admit a normal cone.
                let midpoint = 0.5 * (segment.moving_points[0] + segment.support_points[1]);
                if (0..3).any(|i| {
                    i != f
                        && (midpoint - face.center).dot(b[i]).abs()
                            >= face.half_extent[i] - 8. * tol
                }) {
                    return None;
                }
                if result.is_some() {
                    return None;
                }
                let feature = if edge_moves {
                    EdgeFaceFeature::MovingEdge {
                        edge: e,
                        support_face: f,
                    }
                } else {
                    EdgeFaceFeature::SupportEdge {
                        edge: e,
                        moving_face: f,
                    }
                };
                result = Some((feature, segment));
            }
        }
        result
    }
}

impl FiniteFeatureTrack {
    /// Identify the single outgoing finite point after an endpoint impulse
    /// opens the rest of an incident line. At zero distance a material
    /// neighbor must have strictly positive separating speed. A tie declines.
    pub fn outgoing_endpoint(
        f: FiniteBoxFlight,
        line: FiniteContactSegment,
        loaded: usize,
    ) -> Option<(Self, FiniteFeatureSample)> {
        let p = *line.moving_points.get(loaded)?;
        let q = *line.support_points.get(loaded)?;
        let n = line.normal;
        if !f.moving.valid()
            || !f.support.valid()
            || ![
                p,
                q,
                n,
                f.relative_velocity,
                f.angular_velocity,
                f.center_of_mass,
            ]
            .iter()
            .all(|v| v.is_finite())
            || (n.length_squared() - 1.).abs() > 1e-12
        {
            return None;
        }
        let tol = 512.
            * f64::EPSILON
            * (1.
                + f.moving.center.length()
                + f.support.center.length()
                + f.moving.half_extent.length()
                + f.support.half_extent.length());
        let vt = 512.
            * f64::EPSILON
            * (1.
                + f.relative_velocity.length()
                + f.angular_velocity.length()
                    * (1. + f.moving.half_extent.length() + f.support.half_extent.length()));
        if p.distance(q) > 4. * tol {
            return None;
        }
        let ap = f.moving.orientation.inverse() * (p - f.moving.center);
        let bq = f.support.orientation.inverse() * (q - f.support.center);
        let anchor = |v: DVec3, h: DVec3, fixed: [bool; 3]| {
            let mut output = DVec3::ZERO;
            for i in 0..3 {
                if fixed[i] {
                    if (v[i].abs() - h[i]).abs() > 4. * tol {
                        return None;
                    }
                    output[i] = v[i].signum() * h[i];
                } else if v[i].abs() > h[i] + 4. * tol {
                    return None;
                }
            }
            Some(output)
        };
        let a = AXES.map(|x| f.moving.orientation * x);
        let b = AXES.map(|x| f.support.orientation * x);
        let mut outgoing = None;
        for feature in (0..3)
            .map(SeparatingFeature::MovingFace)
            .chain((0..3).map(SeparatingFeature::SupportFace))
            .chain((0..3).flat_map(|i| (0..3).map(move |j| SeparatingFeature::EdgePair(i, j))))
        {
            let (dir, am, bm) = match feature {
                SeparatingFeature::MovingFace(i) => {
                    (a[i], std::array::from_fn(|k| k == i), [true; 3])
                }
                SeparatingFeature::SupportFace(j) => {
                    (b[j], [true; 3], std::array::from_fn(|k| k == j))
                }
                SeparatingFeature::EdgePair(i, j) => (
                    a[i].cross(b[j]),
                    std::array::from_fn(|k| k != i),
                    std::array::from_fn(|k| k != j),
                ),
            };
            if dir.length() < 1e-7 || dir.cross(n).length() > 1e-10 * dir.length() {
                continue;
            }
            let (Some(moving_local), Some(support_local)) = (
                anchor(ap, f.moving.half_extent, am),
                anchor(bq, f.support.half_extent, bm),
            ) else {
                continue;
            };
            let track = Self {
                feature,
                moving_local,
                support_local,
                support: f.support,
                orientation_sign: dir.dot(n).signum(),
            };
            let Some(s) = track.sample(
                f.moving,
                f.center_of_mass,
                f.relative_velocity,
                f.angular_velocity,
            ) else {
                continue;
            };
            if s.site.point.distance(p) > 4. * tol
                || s.support_point.distance(q) > 4. * tol
                || s.margins()
                    .iter()
                    .any(|m| m.distance < -tol || (m.distance <= tol && m.speed <= vt))
            {
                continue;
            }
            if outgoing.replace((track, s)).is_some() {
                return None;
            }
        }
        outgoing
    }
}
