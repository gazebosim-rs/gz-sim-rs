// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two moving supports with an identifiable resultant and a set of feasible
//! individual loads. Material pressure witnesses certify the full time span.
use super::*;
use crate::{SupportLoadRange, coplanar_load_range};
#[derive(Clone, Copy, Debug)]
pub struct CoplanarPad {
    /// Center of the top face in the common tangent-plane axes.
    pub center: DVec2,
    pub half: DVec2,
    pub yaw: f64,
    pub velocity: DVec2,
}
#[derive(Clone, Copy, Debug)]
pub struct BoundSharedFace {
    pub(super) flat: BoundFaceSupport,
    pads: [CoplanarPad; 2],
}
#[derive(Clone, Copy, Debug)]
pub struct SharedSiteLoad {
    pub contact_at_end: bool,
    /// A selected material pressure centroid for this specific representative.
    pub material_center: Option<DVec3>,
    pub selected_normal_impulse: f64,
    /// Bounds enclosing the integral over all feasible pressure histories.
    pub normal_impulse_bounds: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub struct SharedFaceStep {
    pub state: EdgeSupportState,
    pub angular_impulse_about_com: DVec3,
    pub normal_force: f64,
    pub normal_impulse: f64,
    pub selected_fraction_b: f64,
    /// Any constant fraction in this interval can be sustained by fixed
    /// material pressure points over the entire span.
    pub inner_fraction_b: [f64; 2],
    /// All feasible instantaneous fractions lie in this interval.
    pub outer_fraction_b: [f64; 2],
    pub sites: [SharedSiteLoad; 2],
}
#[derive(Clone, Copy)]
struct Poly {
    p: [DVec2; 16],
    n: usize,
}
impl Poly {
    fn rect(w: f64, h: f64) -> Self {
        let mut p = [DVec2::ZERO; 16];
        p[..4].copy_from_slice(&[
            DVec2::ZERO,
            DVec2::new(w, 0.),
            DVec2::new(w, h),
            DVec2::new(0., h),
        ]);
        Self { p, n: 4 }
    }
    fn points(&self) -> &[DVec2] {
        &self.p[..self.n]
    }
    fn within(&self, v: DVec2, tol: f64) -> bool {
        match self.points() {
            [] => false,
            [point] => point.distance(v) <= tol,
            [a, b] => {
                let edge = *b - *a;
                let n = edge.length();
                if n == 0. {
                    return a.distance(v) <= tol;
                }
                let along = (v - *a).dot(edge / n).clamp(0., n);
                v.distance(*a + along * (edge / n)) <= tol
            }
            points => (0..points.len()).all(|j| {
                let d = points[(j + 1) % points.len()] - points[j];
                d.perp_dot(v - points[j]) >= -tol * d.length()
            }),
        }
    }
    fn clip(&mut self, n: DVec2, offset: f64) -> bool {
        if self.n == 0 {
            return true;
        }
        let old = *self;
        self.n = 0;
        let mut add = |v: DVec2| {
            if self.n >= 16 {
                return false;
            }
            if self.n == 0 || v != self.p[self.n - 1] {
                self.p[self.n] = v;
                self.n += 1;
            }
            true
        };
        let mut last = old.p[old.n - 1];
        let mut prev = offset - n.dot(last);
        for &p in old.points() {
            let gap = offset - n.dot(p);
            if (gap < 0.) != (prev < 0.) && !add(last + (p - last) * (prev / (prev - gap))) {
                return false;
            }
            if gap >= 0. && !add(p) {
                return false;
            }
            prev = gap;
            last = p;
        }
        if self.n > 1 && self.p[0] == self.p[self.n - 1] {
            self.n -= 1;
        }
        true
    }
}
impl FrictionlessFaceSupport {
    pub fn bind_coplanar(
        self,
        start: EdgeSupportState,
        pads: [CoplanarPad; 2],
    ) -> Result<Option<BoundSharedFace>, PhysicsError> {
        if pads.iter().any(|p| {
            !p.center.is_finite()
                || !p.half.is_finite()
                || p.half.min_element() <= 0.
                || !p.velocity.is_finite()
                || !p.yaw.is_finite()
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(self
            .bind_flat(start, false)?
            .map(|flat| BoundSharedFace { flat, pads }))
    }
}
impl BoundSharedFace {
    pub(super) fn axes(self) -> ([DVec3; 2], DVec2) {
        let low = self.flat.body.corners[0];
        let x = self.flat.body.corners[2] - low;
        let y = self.flat.body.corners[1] - low;
        (
            [x.normalize(), y.normalize()],
            DVec2::new(x.length(), y.length()),
        )
    }
    fn polygons(self, t: f64, swept: bool) -> Option<[Poly; 2]> {
        let f = self.flat;
        let (axes, size) = self.axes();
        let r = f.body.corners.iter().map(|c| c.length()).fold(0., f64::max);
        let mut result = [Poly::rect(size.x, size.y); 2];
        let tol = 128.
            * f64::EPSILON
            * (1.
                + size.length()
                + f.start.center.length()
                + self
                    .pads
                    .iter()
                    .map(|p| p.center.length())
                    .fold(0., f64::max));
        // The material axes and origin at each endpoint are common to every
        // pad half-plane, including when the sole yaws during this interval.
        let pose = |time| {
            let q = (f.rotation(time) * f.start.orientation).normalize();
            (
                f.pad_point(f.center(time) + q * f.body.corners[0]),
                axes.map(|a| q * a),
            )
        };
        let start = pose(0.);
        let end = if f.spin != 0. && t > 0. {
            Some(pose(t))
        } else {
            None
        };
        for (poly, pad) in result.iter_mut().zip(self.pads) {
            let ax = DVec2::from_angle(pad.yaw);
            let ay = ax.perp();
            for (side, half) in [(ax, pad.half.x), (ay, pad.half.y)] {
                let world = f.body.pad_axes[0] * side.x + f.body.pad_axes[1] * side.y;
                let acceleration = f.body.gravity.dot(world);
                let relative = (f.pad_point(f.start.velocity) - pad.velocity).dot(side);
                if f.spin == 0. || t == 0. {
                    // The sole material half-plane normal is fixed. Relative
                    // offset is quadratic, so its endpoint and any interior
                    // vertex give its exact extrema over the entire interval.
                    let offset = (start.0 - pad.center).dot(side);
                    let linear = relative * t;
                    let second = 0.5 * acceleration * t * t;
                    let delta = linear + second;
                    let mut lo = 0f64.min(delta);
                    let mut hi = 0f64.max(delta);
                    let stationary = -relative / acceleration;
                    if stationary > 0. && stationary < t {
                        let x = 0.5 * relative * stationary;
                        lo = lo.min(x);
                        hi = hi.max(x);
                    }
                    let e =
                        tol + 128. * f64::EPSILON * (offset.abs() + linear.abs() + second.abs());
                    if !e.is_finite() || !lo.is_finite() || !hi.is_finite() {
                        return None;
                    }
                    let (positive, negative) = if swept {
                        (half - offset - hi - e, half + offset + lo - e)
                    } else {
                        (half - offset - lo + e, half + offset + hi + e)
                    };
                    let direction = DVec2::new(world.dot(start.1[0]), world.dot(start.1[1]));
                    if !poly.clip(direction, positive) || !poly.clip(-direction, negative) {
                        return None;
                    }
                    continue;
                }
                let gn = acceleration.abs();
                let curvature = 0.125 * t * t * (gn + f.spin.powi(2) * r);
                let reach = t
                    * ((f.pad_point(f.start.velocity) - pad.velocity)
                        .dot(side)
                        .abs()
                        + f.spin.abs() * r)
                    + 0.5 * t * t * gn;
                for (time, geometry) in [(0., start), (t, end?)] {
                    if !swept && time > 0. {
                        continue;
                    }
                    let h = if swept {
                        half - curvature - tol
                    } else {
                        half + reach + tol
                    };
                    if h <= 0. {
                        poly.n = 0;
                        break;
                    }
                    let offset = (geometry.0 - (pad.center + pad.velocity * time)).dot(side);
                    let n = DVec2::new(world.dot(geometry.1[0]), world.dot(geometry.1[1]));
                    if !poly.clip(n, h - offset) || !poly.clip(-n, h + offset) {
                        return None;
                    }
                }
            }
        }
        Some(result)
    }
    fn target(self) -> DVec2 {
        let (_, size) = self.axes();
        (self.flat.normalized_center + DVec2::ONE) * 0.5 * size
    }
    /// The strict interior of the convex hull rules out a gravitationally
    /// descending tilt. All retained hull vertices belong to actual eroded
    /// material patches. Nearly collinear hulls fail the clearance test.
    fn interior(self, p: [Poly; 2]) -> bool {
        let mut points = [DVec2::ZERO; 32];
        let mut m = 0;
        for patch in &p {
            for &point in patch.points() {
                points[m] = point;
                m += 1;
            }
        }
        points[..m].sort_unstable_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
        let mut hull = [DVec2::ZERO; 64];
        let mut len = 0;
        for reverse in [false, true] {
            let bottom = len;
            for i in 0..m {
                let v = points[if reverse { m - 1 - i } else { i }];
                while len >= bottom + 2
                    && (hull[len - 1] - hull[len - 2]).perp_dot(v - hull[len - 1]) <= 0.
                {
                    len -= 1;
                }
                hull[len] = v;
                len += 1;
            }
            len = len.saturating_sub(1);
        }
        let c = self.target();
        let radius = (self.flat.tolerance * 64.).max(2048. * f64::EPSILON * self.axes().1.length());
        len >= 3
            && (0..len).all(|i| {
                let edge = hull[(i + 1) % len] - hull[i];
                edge.perp_dot(c - hull[i]) > radius * edge.length()
            })
    }
    pub(super) fn stationary(
        self,
        seconds: f64,
        preferred: Option<f64>,
    ) -> Result<Option<SharedFaceStep>, PhysicsError> {
        let Some(inner) = self.polygons(0., true) else {
            return Ok(None);
        };
        if !self.interior(inner) {
            return Ok(None);
        }
        let Some(outer) = self.polygons(0., false) else {
            return Ok(None);
        };
        // Position and velocity are exactly constant in the co-moving material
        // frame, including for tilted gravity. Projecting two nearly parallel
        // gravity vectors can otherwise invent a roundoff-sized fresh slip.
        let mut fixed = self;
        fixed.flat.body.gravity = DVec3::ZERO;
        let Some(mut value) = fixed.advance_with(seconds, preferred, inner, outer)? else {
            return Ok(None);
        };
        let belt = self.pads[0].velocity;
        let v = self.flat.body.pad_axes[0] * belt.x + self.flat.body.pad_axes[1] * belt.y;
        value.state.center = self.flat.start.center + v * seconds;
        value.state.velocity = v;
        value.state.angular_velocity = DVec3::ZERO;
        Ok(Some(value))
    }
    /// Report the pressure split without selecting an artificial unique force.
    pub fn instantaneous(self) -> Result<Option<SupportLoadRange>, PhysicsError> {
        let Some(p) = self.polygons(0., false) else {
            return Ok(None);
        };
        coplanar_load_range(self.target(), [p[0].points(), p[1].points()])
    }
    /// Select a physically supported representative closest to the supplied
    /// previous fraction. With no previous load use the interior midpoint.
    pub fn advance(
        self,
        seconds: f64,
        preferred_fraction_b: Option<f64>,
    ) -> Result<Option<SharedFaceStep>, PhysicsError> {
        if !seconds.is_finite()
            || seconds <= 0.
            || preferred_fraction_b.is_some_and(|v| !v.is_finite() || !(0.0..=1.).contains(&v))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(inner) = self.polygons(seconds, true) else {
            return Ok(None);
        };
        let Some(outer) = self.polygons(seconds, false) else {
            return Ok(None);
        };
        self.advance_with(seconds, preferred_fraction_b, inner, outer)
    }
    fn advance_with(
        self,
        seconds: f64,
        preferred_fraction_b: Option<f64>,
        inner: [Poly; 2],
        outer: [Poly; 2],
    ) -> Result<Option<SharedFaceStep>, PhysicsError> {
        let c = self.target();
        let Some(feasible) = coplanar_load_range(c, [inner[0].points(), inner[1].points()])? else {
            return Ok(None);
        };
        let Some(all) = coplanar_load_range(c, [outer[0].points(), outer[1].points()])? else {
            return Ok(None);
        };
        let [lo, hi] = feasible.fraction_b;
        let target = preferred_fraction_b
            .unwrap_or(0.5 * lo + 0.5 * hi)
            .clamp(lo, hi);
        let (axes, size) = self.axes();
        let tol = 512. * f64::EPSILON * (1. + size.length());
        let Some(points) = witness(inner, c, target, tol) else {
            return Ok(None);
        };
        let (state, angular_impulse_about_com) = self.flat.motion(seconds)?;
        let normal_impulse = self.flat.normal_force * seconds;
        let bounds = all.force_bounds(normal_impulse)?;
        let corners = [0, 2, 3, 1].map(|i| {
            self.flat
                .pad_point(state.center + state.orientation * self.flat.body.corners[i])
        });
        let touching = self.pads.map(|p| {
            let inv = DVec2::from_angle(-p.yaw);
            let poly = corners.map(|x| inv.rotate(x - p.center - p.velocity * seconds));
            FaceOverlap::clip(poly, p.half).is_some_and(|v| v.count > 0)
        });
        let sites = std::array::from_fn(|k| {
            let share = if k == 0 { 1. - target } else { target };
            SharedSiteLoad {
                contact_at_end: touching[k],
                material_center: points[k]
                    .map(|p| self.flat.body.corners[0] + axes[0] * p.x + axes[1] * p.y),
                selected_normal_impulse: share * normal_impulse,
                normal_impulse_bounds: bounds[k],
            }
        });
        Ok(Some(SharedFaceStep {
            state,
            normal_force: self.flat.normal_force,
            normal_impulse,
            angular_impulse_about_com,
            selected_fraction_b: target,
            inner_fraction_b: feasible.fraction_b,
            outer_fraction_b: all.fraction_b,
            sites,
        }))
    }
}
pub(crate) fn contact_centers(
    patches: [&[DVec2]; 2],
    c: DVec2,
    b: f64,
) -> Result<Option<[Option<DVec2>; 2]>, PhysicsError> {
    if patches.iter().any(|p| p.len() > 16) {
        return Err(PhysicsError::InvalidValue);
    }
    let ps = patches.map(|p| {
        let mut v = Poly {
            p: [DVec2::ZERO; 16],
            n: p.len(),
        };
        v.p[..p.len()].copy_from_slice(p);
        v
    });
    let scale = patches
        .iter()
        .flat_map(|p| p.iter())
        .map(|p| p.distance(c))
        .fold(0., f64::max);
    Ok(witness(ps, c, b, 256. * f64::EPSILON * scale))
}
fn witness(p: [Poly; 2], c: DVec2, b: f64, tol: f64) -> Option<[Option<DVec2>; 2]> {
    let a = 1. - b;
    if b == 0. {
        return p[0].within(c, tol).then_some([Some(c), None]);
    }
    if a == 0. {
        return p[1].within(c, tol).then_some([None, Some(c)]);
    }
    // Intersection in moment coordinates has a convex solution set. The
    // scaled polygons share the origin when both centroids coincide with c.
    let mut sets = p;
    for (shape, weight) in sets.iter_mut().zip([a, -b]) {
        for v in &mut shape.p[..shape.n] {
            *v = (*v - c) * weight;
        }
    }
    let mut sum = DVec2::ZERO;
    let mut count = 0;
    let mut consider = |v: DVec2| {
        let pa = c + v / a;
        let pb = c - v / b;
        if p[0].within(pa, tol) && p[1].within(pb, tol) {
            sum += v;
            count += 1;
        }
    };
    for s in sets {
        for &v in s.points() {
            consider(v);
        }
    }
    let [pa, pb] = sets;
    for i in 0..pa.n {
        for j in 0..pb.n {
            let x = pa.p[i];
            let dx = pa.p[(i + 1) % pa.n] - x;
            let y = pb.p[j];
            let dy = pb.p[(j + 1) % pb.n] - y;
            let det = dx.perp_dot(dy);
            if det.abs() <= 32. * f64::EPSILON * dx.length() * dy.length() {
                continue;
            }
            let u = (y - x).perp_dot(dy) / det;
            let v = (y - x).perp_dot(dx) / det;
            if (-1e-13..=1. + 1e-13).contains(&u) && (-1e-13..=1. + 1e-13).contains(&v) {
                consider(x + dx * u);
            }
        }
    }
    if count == 0 {
        return None;
    }
    let v = sum / (count as f64);
    Some([Some(c + v / a), Some(c - v / b)])
}

#[cfg(test)]
mod witness_tests {
    use super::*;
    #[test]
    fn degenerate_supports_preserve_certified_line_and_point_centers() {
        let edge = [DVec2::new(-1., -0.3), DVec2::new(-1., 0.3)];
        let point = [DVec2::new(1., 0.1)];
        let c = DVec2::new(0., 0.15);
        let r = contact_centers([&edge, &point], c, 0.5).unwrap().unwrap();
        assert!(r[0].unwrap().distance(DVec2::new(-1., 0.2)) < 1e-14);
        assert!(r[1].unwrap().distance(point[0]) < 1e-14);
        assert!(
            contact_centers([&edge, &point], DVec2::new(0., 0.3), 0.5)
                .unwrap()
                .is_none()
        );
        assert!(
            contact_centers([&[], &point], point[0], 1.)
                .unwrap()
                .is_some()
        );
    }
}
