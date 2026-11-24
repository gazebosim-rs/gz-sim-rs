// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous intersection of two finite oriented boxes. The moving box
//! rotates at constant world angular velocity about its ballistic COM.
//! All coordinates are in the inertial frame translating with the support.
use crate::PhysicsError;
use glam::{DQuat, DVec3};
use std::time::Duration;

#[derive(Clone, Copy, Debug)]
pub struct FlightBox {
    pub center: DVec3,
    pub orientation: DQuat,
    pub half_extent: DVec3,
}

impl FlightBox {
    pub(crate) fn valid(self) -> bool {
        self.center.is_finite()
            && self.half_extent.is_finite()
            && self.half_extent.min_element() > 0.
            && self.orientation.is_finite()
            && (self.orientation.length_squared() - 1.).abs() < 1e-12
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FiniteBoxFlight {
    pub moving: FlightBox,
    pub support: FlightBox,
    pub center_of_mass: DVec3,
    pub relative_velocity: DVec3,
    pub angular_velocity: DVec3,
    pub acceleration: DVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeparatingFeature {
    MovingFace(usize),
    SupportFace(usize),
    EdgePair(usize, usize),
}

#[derive(Clone, Copy, Debug)]
pub struct BoxCrossing {
    pub earliest: Duration,
    pub latest: Duration,
    /// Unit separating normal pointing from support toward moving body.
    pub normal: DVec3,
    pub feature: SeparatingFeature,
    /// Matched points on the two boxes at the first overlapping nanosecond.
    /// Parallel faces, tied vertices and ill-conditioned edge pairs can have
    /// a finite crossing without a unique point.
    pub points: Option<[DVec3; 2]>,
}

#[derive(Clone, Copy, Debug)]
pub enum FiniteBoxArrival {
    Separated,
    InitiallyTouching,
    Crossing(BoxCrossing),
    /// Earliest unresolved interval. Tangency with no certified overlap
    /// and exhausted subdivision both take this path.
    Unresolved {
        earliest: Duration,
        latest: Duration,
    },
}
#[derive(Clone, Copy)]
struct Obb {
    center: DVec3,
    half: DVec3,
    axes: [DVec3; 3],
}
impl From<FlightBox> for Obb {
    fn from(b: FlightBox) -> Self {
        Self {
            center: b.center,
            half: b.half_extent,
            axes: [DVec3::X, DVec3::Y, DVec3::Z].map(|v| b.orientation * v),
        }
    }
}
impl Obb {
    fn extent(self, n: DVec3) -> f64 {
        self.axes
            .iter()
            .enumerate()
            .map(|(i, q)| self.half[i] * q.dot(n).abs())
            .sum()
    }
    fn face_point(self, p: DVec3, n: DVec3, axis: usize, strict: f64) -> Option<DVec3> {
        let q = p - self.center;
        for i in 0..3 {
            if i != axis && q.dot(self.axes[i]).abs() >= self.half[i] - strict {
                return None;
            }
        }
        Some(p + n * (self.extent(n) - q.dot(n)))
    }
    fn support(self, n: DVec3, edge: Option<usize>, tol: f64) -> Option<DVec3> {
        let mut p = self.center;
        for (i, a) in self.axes.iter().enumerate() {
            if Some(i) == edge {
                continue;
            }
            let d = n.dot(*a);
            if 2. * self.half[i] * d.abs() <= tol {
                return None;
            }
            p += self.half[i] * d.signum() * *a;
        }
        Some(p)
    }
}
#[derive(Clone, Copy)]
struct Witness {
    gap: f64,
    n: DVec3,
    feature: SeparatingFeature,
}
fn separation(a: Obb, b: Obb) -> Witness {
    let d = a.center - b.center;
    let mut best = Witness {
        gap: f64::NEG_INFINITY,
        n: DVec3::ZERO,
        feature: SeparatingFeature::SupportFace(0),
    };
    let mut test = |n: DVec3, feature| {
        let signed = d.dot(n);
        let gap = signed.abs() - a.extent(n) - b.extent(n);
        if gap > best.gap {
            best = Witness {
                gap,
                n: if signed < 0. { -n } else { n },
                feature,
            };
        }
    };
    for i in 0..3 {
        test(b.axes[i], SeparatingFeature::SupportFace(i));
        test(a.axes[i], SeparatingFeature::MovingFace(i));
    }
    for i in 0..3 {
        for j in 0..3 {
            if let Some(n) = a.axes[i].cross(b.axes[j]).try_normalize() {
                test(n, SeparatingFeature::EdgePair(i, j));
            }
        }
    }
    best
}
fn points(a: Obb, b: Obb, w: Witness, tol: f64) -> Option<[DVec3; 2]> {
    let n = w.n;
    match w.feature {
        SeparatingFeature::SupportFace(j) => {
            let x = a.support(-n, None, tol)?;
            Some([x, b.face_point(x, n, j, tol)?])
        }
        SeparatingFeature::MovingFace(i) => {
            let x = b.support(n, None, tol)?;
            Some([a.face_point(x, -n, i, tol)?, x])
        }
        SeparatingFeature::EdgePair(i, j) => {
            let p = a.support(-n, Some(i), tol)?;
            let q = b.support(n, Some(j), tol)?;
            let (u, v) = (a.axes[i], b.axes[j]);
            let uv = u.dot(v);
            let cross = u.cross(v);
            let den = cross.length_squared();
            if den < 1e-14 {
                return None;
            }
            let delta = q - p;
            let s = delta.cross(v).dot(cross) / den;
            let t = uv * s - v.dot(delta);
            if s.abs() >= a.half[i] - tol || t.abs() >= b.half[j] - tol {
                return None;
            }
            Some([p + s * u, q + t * v])
        }
    }
}
struct Search {
    flight: FiniteBoxFlight,
    b: Obb,
    radius: f64,
    tol: f64,
    remaining: u32,
}
impl Search {
    fn interval(&mut self, lo: u64, hi: u64) -> FiniteBoxArrival {
        use FiniteBoxArrival::*;
        let ns = Duration::from_nanos;
        if self.remaining == 0 {
            return Unresolved {
                earliest: ns(lo),
                latest: ns(hi),
            };
        }
        self.remaining -= 1;
        let f = self.flight;
        let mid = lo + (hi - lo) / 2;
        let t = ns(mid).as_secs_f64();
        let h = ns(hi - mid).as_secs_f64();
        let w = separation(f.at(t).into(), self.b);
        let v = f.relative_velocity + t * f.acceleration;
        // This particular axis remains fixed in the inertial frame for the
        // proof. No angular derivative of the SAT maximum is assumed.
        let rotational = f.angular_velocity.length() * self.radius;
        let reach =
            (v.dot(w.n).abs() + rotational) * h + 0.5 * f.acceleration.dot(w.n).abs() * h * h;
        if w.gap > reach + self.tol {
            return Separated;
        }
        let start: Obb = f.at(ns(lo).as_secs_f64()).into();
        let end: Obb = f.at(ns(hi).as_secs_f64()).into();
        let chord_clear = |n: DVec3| {
            // Each material vertex deviates from the straight chord between
            // its two projections by at most max|d²height/dt²| Δt²/8.
            let span = ns(hi - lo).as_secs_f64();
            let curvature =
                f.acceleration.dot(n).abs() + f.angular_velocity.length_squared() * self.radius;
            let gap = |a: Obb| (a.center - self.b.center).dot(n) - a.extent(n) - self.b.extent(n);
            gap(start).min(gap(end)) > self.tol + 0.125 * curvature * span * span
        };
        if chord_clear(w.n) {
            return Separated;
        }
        if hi - lo == 1 {
            let last = separation(end, self.b);
            if last.gap > self.tol && chord_clear(last.n) {
                return Separated;
            }
            if last.gap < -self.tol {
                let tol = (reach + 8. * self.tol).max(1e-10);
                let sites = points(end, self.b, last, tol).filter(|p| p[0].distance(p[1]) <= tol);
                return Crossing(BoxCrossing {
                    earliest: ns(lo),
                    latest: ns(hi),
                    normal: last.n,
                    feature: last.feature,
                    points: sites,
                });
            }
            return Unresolved {
                earliest: ns(lo),
                latest: ns(hi),
            };
        }
        match self.interval(lo, mid) {
            Separated => self.interval(mid, hi),
            r => r,
        }
    }
}
impl FiniteBoxFlight {
    pub(crate) fn separation_at(self,t:f64)->(f64,DVec3,SeparatingFeature) {
        let w=separation(self.at(t).into(),self.support.into());
        (w.gap,w.n,w.feature)
    }
    /// Instantaneous contact sites on the measured boxes. The gap allowance
    /// comes from the spatial reach of the enclosed arrival bracket.
    pub fn contact_geometry(
        self,
        allowance: f64,
    ) -> Option<(SeparatingFeature, DVec3, [DVec3; 2], f64)> {
        if !self.moving.valid() || !self.support.valid() {
            return None;
        }
        let a = self.moving.into();
        let b = self.support.into();
        let w = separation(a, b);
        let scale = 1.
            + self.moving.center.abs().max_element()
            + self.support.center.abs().max_element()
            + self.moving.half_extent.length()
            + self.support.half_extent.length();
        let tol = allowance + 128. * f64::EPSILON * scale;
        if !tol.is_finite() || allowance < 0. || w.gap.abs() > tol {
            return None;
        }
        let pair = points(a, b, w, (8. * tol).max(1e-10))?;
        (pair[0].distance(pair[1]) <= tol).then_some((w.feature, w.n, pair, w.gap))
    }
    pub fn com_at(self, t: f64) -> DVec3 {
        self.center_of_mass + (self.relative_velocity + 0.5 * t * self.acceleration) * t
    }
    pub fn at(self, t: f64) -> FlightBox {
        let delta = DQuat::from_scaled_axis(self.angular_velocity * t);
        FlightBox {
            center: self.com_at(t) + delta * (self.moving.center - self.center_of_mass),
            orientation: (delta * self.moving.orientation).normalize(),
            ..self.moving
        }
    }
    pub fn first_crossing(self, dt: Duration) -> Result<FiniteBoxArrival, PhysicsError> {
        let limit = u64::try_from(dt.as_nanos()).map_err(|_| PhysicsError::InvalidValue)?;
        if limit == 0
            || !self.moving.valid()
            || !self.support.valid()
            || ![
                self.center_of_mass,
                self.relative_velocity,
                self.angular_velocity,
                self.acceleration,
            ]
            .into_iter()
            .all(|v| v.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let b = self.support.into();
        let radius =
            self.moving.center.distance(self.center_of_mass) + self.moving.half_extent.length();
        let scale = 1.
            + radius
            + self.support.half_extent.length()
            + self.support.center.abs().max_element()
            + self.center_of_mass.abs().max_element()
            + self.com_at(dt.as_secs_f64()).abs().max_element();
        let tol = 128. * f64::EPSILON * scale;
        if !scale.is_finite() || !(self.angular_velocity.length() * dt.as_secs_f64()).is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let start = separation(self.moving.into(), b);
        if start.gap <= tol {
            return Ok(FiniteBoxArrival::InitiallyTouching);
        }
        let r = Search {
            flight: self,
            b,
            radius,
            tol,
            remaining: 512,
        }
        .interval(0, limit);
        // A mathematical contact may fall exactly on an integer-nanosecond
        // endpoint. Require a strictly overlapping witness on the following
        // nanosecond before widening the original bracket by one.
        if let FiniteBoxArrival::Unresolved { earliest, latest } = r
            && latest - earliest == Duration::from_nanos(1)
            && latest < dt
        {
            let end = latest + Duration::from_nanos(1);
            let a = self.at(end.as_secs_f64()).into();
            let w = separation(a, b);
            if w.gap < -tol {
                let speed = self.relative_velocity.length()
                    + self.acceleration.length() * end.as_secs_f64()
                    + self.angular_velocity.length() * radius;
                let delta = 2e-9 * speed + 8. * tol;
                let sites =
                    points(a, b, w, delta.max(1e-10)).filter(|p| p[0].distance(p[1]) <= delta);
                return Ok(FiniteBoxArrival::Crossing(BoxCrossing {
                    earliest,
                    latest: end,
                    normal: w.n,
                    feature: w.feature,
                    points: sites,
                }));
            }
        }
        Ok(r)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(q: DQuat, gap: f64) -> FiniteBoxFlight {
        let half = DVec3::new(0.13, 0.08, 0.04);
        let z = Obb::from(FlightBox {
            center: DVec3::ZERO,
            orientation: q,
            half_extent: half,
        })
        .extent(DVec3::Z);
        let moving = FlightBox {
            center: DVec3::Z * (z + gap),
            half_extent: half,
            orientation: q,
        };
        FiniteBoxFlight {
            moving,
            support: FlightBox {
                center: DVec3::Z * (-0.06),
                half_extent: DVec3::new(0.33, 0.18, 0.06),
                orientation: DQuat::IDENTITY,
            },
            relative_velocity: -DVec3::Z,
            center_of_mass: moving.center + q * (DVec3::Z * 0.07),
            angular_velocity: DVec3::ZERO,
            acceleration: DVec3::ZERO,
        }
    }
    #[test]
    fn finite_translating_support_matches_analytic_vertex_or_reports_tied_face() {
        for t in [1., 887.2, 83531., 225055.7, 499997.8] {
            for q in [
                DQuat::IDENTITY,
                DQuat::from_rotation_x(0.14) * DQuat::from_rotation_y(-0.21),
            ] {
                let f = fixture(q, 1e-9 * t);
                let FiniteBoxArrival::Crossing(c) =
                    f.first_crossing(Duration::from_micros(500)).unwrap()
                else {
                    panic!("missing {t}")
                };
                assert!(
                    c.earliest.as_nanos() as f64 <= t + 1e-6
                        && c.latest.as_nanos() as f64 >= t - 1e-6
                );
                assert!((c.latest - c.earliest).as_nanos() <= 2);
                assert_eq!(c.points.is_some(), q != DQuat::IDENTITY);
                let f = FiniteBoxFlight {
                    support: FlightBox {
                        center: f.support.center + DVec3::X * 0.9,
                        ..f.support
                    },
                    ..f
                };
                assert!(matches!(
                    f.first_crossing(Duration::from_micros(500)).unwrap(),
                    FiniteBoxArrival::Separated
                ));
            }
        }
    }
    #[test]
    fn rotates_about_ballistic_com_and_obeys_global_frame_rotation() {
        let q = DQuat::from_rotation_y(0.22) * DQuat::from_rotation_x(0.03);
        let mut f = fixture(q, 0.00045);
        f.angular_velocity = DVec3::new(9., 17., 0.1);
        f.acceleration = DVec3::new(0.3, -2., -9.81);
        let mut times = vec![];
        for a in [
            DQuat::IDENTITY,
            DQuat::from_rotation_arc(DVec3::Z, DVec3::new(1., 2., 3.).normalize()),
        ] {
            let rot = |s: FlightBox| FlightBox {
                center: a * s.center,
                orientation: a * s.orientation,
                ..s
            };
            let h = FiniteBoxFlight {
                moving: rot(f.moving),
                support: rot(f.support),
                relative_velocity: a * f.relative_velocity,
                angular_velocity: a * f.angular_velocity,
                center_of_mass: a * f.center_of_mass,
                acceleration: a * f.acceleration,
            };
            let FiniteBoxArrival::Crossing(c) =
                h.first_crossing(Duration::from_micros(500)).unwrap()
            else {
                panic!("rotating crossing")
            };
            let points = c.points.unwrap();
            assert!(points[0].distance(points[1]) < 1e-8);
            times.push((c.earliest, c.latest));
        }
        assert_eq!(times[0], times[1]);
    }
}
