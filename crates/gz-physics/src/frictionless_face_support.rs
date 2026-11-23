// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sustained frictionless support of a yawing flat sole on a finite rectangular
//! pad. A strictly interior center of pressure is enough for a compressive
//! reaction even when the two boxes overlap only partially.
use crate::{EdgeSupportState, PhysicsError};
use glam::{DMat3, DQuat, DVec2, DVec3};
#[derive(Clone, Copy, Debug)]
pub struct FrictionlessFaceSupport {
    pub mass: f64,
    pub inertia_body: DMat3,
    /// Rectangle vertices relative to COM, ordered (−x,−y),(−x,+y),(+x,−y),(+x,+y).
    pub corners: [DVec3; 4],
    /// Right-handed orthonormal tangents of the finite pad. The top plane
    /// passes through the origin in the uniformly translating support frame.
    pub pad_axes: [DVec3; 2],
    pub pad_half: DVec2,
    pub gravity: DVec3,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceOverlap {
    /// Counter-clockwise vertices in pad coordinates.
    pub vertices: [DVec2; 8],
    pub count: u8,
    pub area: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct BoundFaceSupport {
    body: FrictionlessFaceSupport,
    start: EdgeSupportState,
    normal: DVec3,
    normal_force: f64,
    spin: f64,
    center_offset: DVec3,
    normalized_center: DVec2,
    tolerance: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct FaceBearingSample {
    pub state: EdgeSupportState,
    pub center_of_pressure: DVec3,
    pub pad_center: DVec2,
    pub sole_center: DVec2,
    pub overlap: FaceOverlap,
    pub normal_force: f64,
    pub normal_impulse: f64,
    pub angular_impulse_about_com: DVec3,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FacePressureExit {
    pub seconds: f64,
    pub numerical_bracket_seconds: [f64; 2],
    /// Bit 0 = −x, bit 1 = +x, bit 2 = −y, bit 3 = +y on the pad.
    pub pad_edges: u8,
}
#[derive(Clone, Copy, Debug)]
// The accepted hot path returns the overlap geometry in-place, avoiding a
// heap allocation on every supported step.
#[allow(clippy::large_enum_variant)]
pub enum FaceBearingAdvance {
    Supported(FaceBearingSample),
    /// The unconstrained continuation's pressure leaves the feasible footprint.
    /// A different unilateral solution must be used on the far side.
    Exits(FacePressureExit),
    Unresolved,
}
impl FrictionlessFaceSupport {
    pub fn bind(self, s: EdgeSupportState) -> Result<Option<BoundFaceSupport>, PhysicsError> {
        self.bind_flat(s, true)
    }
    fn bind_flat(
        self,
        s: EdgeSupportState,
        require_pad: bool,
    ) -> Result<Option<BoundFaceSupport>, PhysicsError> {
        let x = self.corners[2] - self.corners[0];
        let y = self.corners[1] - self.corners[0];
        let (w, h) = (x.length(), y.length());
        let [a, b] = self.pad_axes;
        let eps = 512. * f64::EPSILON;
        let i = self.inertia_body;
        if !self.mass.is_finite()
            || self.mass <= 0.
            || !self.pad_half.is_finite()
            || self.pad_half.min_element() <= 0.
            || !self.gravity.is_finite()
            || !i.is_finite()
            || (a.length_squared() - 1.).abs() > eps
            || !a.is_finite()
            || (b.length_squared() - 1.).abs() > eps
            || !b.is_finite()
            || a.dot(b).abs() > eps
            || self.corners.iter().any(|v| !v.is_finite())
            || !w.is_finite()
            || !h.is_finite()
            || w == 0.
            || h == 0.
            || x.dot(y).abs() > eps * w * h
            || (self.corners[3] - self.corners[0] - x - y).length() > eps * (w + h)
            || !s.center.is_finite()
            || !s.velocity.is_finite()
            || !s.angular_velocity.is_finite()
            || !s.orientation.is_finite()
            || (s.orientation.length_squared() - 1.).abs() > eps
        {
            return Err(PhysicsError::InvalidValue);
        }
        let scale = i
            .to_cols_array()
            .into_iter()
            .map(f64::abs)
            .fold(0., f64::max);
        if scale == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let m = i * (1. / scale);
        if (m - m.transpose())
            .to_cols_array()
            .into_iter()
            .any(|v| v.abs() > eps)
            || m.x_axis.x <= 0.
            || m.x_axis.x.mul_add(m.y_axis.y, -m.x_axis.y * m.y_axis.x) <= 0.
            || m.determinant() <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let n = a.cross(b);
        let q = s.orientation;
        let spin = s.angular_velocity.dot(n);
        let tol = eps * (1. + w + h + s.center.length() + self.pad_half.length());
        if (q * (x.cross(y) / (w * h)) - n).length() > eps
            || (s.angular_velocity - spin * n).length() > eps * (1. + spin.abs())
            || s.velocity.dot(n).abs() > eps * (1. + s.velocity.length())
            || self
                .corners
                .iter()
                .any(|r| (s.center + q * (*r)).dot(n).abs() > tol)
        {
            return Ok(None);
        }
        let force = -self.mass * self.gravity.dot(n);
        if force <= 0. || !force.is_finite() {
            return Ok(None);
        }
        let inormal = q * (i * (q.conjugate() * n));
        let offset = (inormal - n * inormal.dot(n)) * (-spin.powi(2) / force);
        let world = s.center + offset;
        let cp = world - n * world.dot(n);
        let local = q.conjugate() * (cp - s.center) - self.corners[0];
        let coeff = DVec2::new(
            2. * local.dot(x) / (w * w) - 1.,
            2. * local.dot(y) / (h * h) - 1.,
        );
        if !coeff.is_finite()
            || coeff.x.abs() >= 1. - tol / w
            || coeff.y.abs() >= 1. - tol / h
            || !offset.is_finite()
        {
            return Ok(None);
        }
        let result = BoundFaceSupport {
            body: self,
            start: s,
            normal: n,
            normal_force: force,
            spin,
            center_offset: offset,
            normalized_center: coeff,
            tolerance: tol,
        };
        if require_pad && (self.pad_half - result.pad_point(cp).abs()).min_element() <= tol {
            return Ok(None);
        }
        Ok(Some(result))
    }
}
impl BoundFaceSupport {
    fn pad_point(self, p: DVec3) -> DVec2 {
        DVec2::new(p.dot(self.body.pad_axes[0]), p.dot(self.body.pad_axes[1]))
    }
    fn tangent(self, x: DVec3) -> DVec3 {
        x - self.normal * x.dot(self.normal)
    }
    fn rotation(self, t: f64) -> DQuat {
        DQuat::from_axis_angle(self.normal, self.spin * t)
    }
    fn center(self, t: f64) -> DVec3 {
        self.start.center
            + self.tangent(self.start.velocity * t + self.body.gravity * (0.5 * t * t))
    }
    fn cp(self, t: f64) -> DVec3 {
        self.tangent(self.center(t)) + self.rotation(t) * self.center_offset
    }
    fn margins(self, t: f64) -> [f64; 4] {
        let p = self.pad_point(self.cp(t));
        let h = self.body.pad_half;
        [h.x + p.x, h.x - p.x, h.y + p.y, h.y - p.y]
    }
    fn accelerations(self) -> [f64; 4] {
        let v = self.pad_point(self.body.gravity).abs()
            + DVec2::splat(self.spin.powi(2) * self.center_offset.length());
        [v.x, v.x, v.y, v.y]
    }
    fn speeds(self, t: f64) -> [f64; 4] {
        let v = self.pad_point(
            self.start.velocity
                + self.body.gravity * t
                + self.spin * self.normal.cross(self.rotation(t) * self.center_offset),
        );
        [v.x, -v.x, v.y, -v.y]
    }
    fn motion(self, t: f64) -> Result<(EdgeSupportState, DVec3), PhysicsError> {
        if !t.is_finite() || t < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let rot = self.rotation(t);
        let q = (rot * self.start.orientation).normalize();
        let c = self.center(t);
        let state = EdgeSupportState {
            center: c,
            orientation: q,
            velocity: self.tangent(self.start.velocity + self.body.gravity * t),
            angular_velocity: self.spin * self.normal,
        };
        let h = self.start.orientation
            * (self.body.inertia_body
                * (self.start.orientation.conjugate() * state.angular_velocity));
        if !q.is_finite()
            || !c.is_finite()
            || !state.velocity.is_finite()
            || !h.is_finite()
            || !(self.normal_force * t).is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok((state, rot * h - h))
    }
    pub fn sample(self, t: f64) -> Result<FaceBearingSample, PhysicsError> {
        let (state, angular_impulse_about_com) = self.motion(t)?;
        let corners = [0, 2, 3, 1]
            .map(|i| self.pad_point(state.center + state.orientation * self.body.corners[i]));
        let overlap =
            FaceOverlap::clip(corners, self.body.pad_half).ok_or(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "resolved convex sole/pad intersection",
            })?;
        let cp = self.cp(t);
        if !cp.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(FaceBearingSample {
            state,
            center_of_pressure: cp,
            pad_center: self.pad_point(cp),
            sole_center: self.normalized_center,
            overlap,
            normal_force: self.normal_force,
            normal_impulse: self.normal_force * t,
            angular_impulse_about_com,
        })
    }
    /// Bind the pad-owned line at the first unique transverse pressure exit.
    /// At this boundary the flat and tipping solutions have the same normal
    /// force and zero tilt acceleration. The post-exit enclosure additionally
    /// requires strictly outward jerk to continue from this equality.
    pub fn departing_deck_edge(
        self,
        horizon: f64,
    ) -> Result<Option<(FacePressureExit, crate::BoundDeckEdge)>, PhysicsError> {
        let FaceBearingAdvance::Exits(exit) = self.advance(horizon)? else {
            return Ok(None);
        };
        if exit.pad_edges.count_ones() != 1 {
            return Ok(None);
        }
        let id = exit.pad_edges.trailing_zeros() as usize;
        let Some(mut edge) = self
            .body
            .bind_deck_edge(self.sample(exit.seconds)?.state, id)?
        else {
            return Ok(None);
        };
        edge.at_face_exit();
        Ok(Some((exit, edge)))
    }
    fn interior(self, a: f64, b: f64, ma: [f64; 4], mb: [f64; 4]) -> bool {
        let h = (b - a).abs();
        let acc = self.accelerations();
        (0..4).all(|k| ma[k].min(mb[k]) - acc[k] * (h * h / 8.) > self.tolerance)
    }
    /// Prove continuous support over a whole interval or locate its first
    /// transverse exit. Ambiguous grazing and nonseparated roots are declined.
    pub fn advance(self, t: f64) -> Result<FaceBearingAdvance, PhysicsError> {
        if !t.is_finite() || t <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let ma = self.margins(0.);
        let mb = self.margins(t);
        if mb.iter().any(|v| !v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        if self.interior(0., t, ma, mb) {
            return Ok(FaceBearingAdvance::Supported(self.sample(t)?));
        }
        let mut work = [(0., 0., 0u8); 32];
        work[0] = (0., t, 0);
        let mut count = 1;
        let mut visited = 0;
        while count > 0 {
            count -= 1;
            let (a, b, depth) = work[count];
            visited += 1;
            if visited > 128 {
                return Ok(FaceBearingAdvance::Unresolved);
            }
            let ma = self.margins(a);
            let mb = self.margins(b);
            if self.interior(a, b, ma, mb) {
                continue;
            }
            let speed = self.speeds(a);
            let acc = self.accelerations();
            let dt = b - a;
            let mut roots = [f64::INFINITY; 4];
            let mut brackets = [[0.; 2]; 4];
            let mut certified = true;
            for k in 0..4 {
                if ma[k].min(mb[k]) - acc[k] * dt * dt / 8. > self.tolerance {
                    continue;
                }
                if speed[k] + acc[k] * dt >= 0. || ma[k] < 0. || mb[k] > 0. {
                    certified = false;
                    break;
                }
                let (mut lo, mut hi) = (a, b);
                for _ in 0..56 {
                    let m = 0.5 * (lo + hi);
                    if m == lo || m == hi {
                        break;
                    }
                    if self.margins(m)[k] > 0. {
                        lo = m;
                    } else {
                        hi = m;
                    }
                }
                roots[k] = 0.5 * (lo + hi);
                brackets[k] = [lo, hi];
            }
            if certified {
                let i = (0..4)
                    .min_by(|i, j| roots[*i].total_cmp(&roots[*j]))
                    .unwrap();
                if roots[i].is_finite() {
                    let threshold = 4. * f64::EPSILON * (1. + roots[i]);
                    let bits = (0..4)
                        .filter(|&k| (roots[i] - roots[k]).abs() <= threshold)
                        .fold(0, |mask, k| mask | (1 << k));
                    return Ok(FaceBearingAdvance::Exits(FacePressureExit {
                        seconds: roots[i],
                        numerical_bracket_seconds: brackets[i],
                        pad_edges: bits,
                    }));
                }
            }
            if depth >= 25 || count + 2 > work.len() {
                return Ok(FaceBearingAdvance::Unresolved);
            }
            let mid = 0.5 * (a + b);
            work[count] = (mid, b, depth + 1);
            work[count + 1] = (a, mid, depth + 1);
            count += 2;
        }
        Ok(FaceBearingAdvance::Supported(self.sample(t)?))
    }
}
impl FaceOverlap {
    /// Intersect a counter-clockwise convex sole with a centered, axis-aligned
    /// pad rectangle. Coordinates and area are in the pad tangent plane.
    pub fn clip(quad: [DVec2; 4], h: DVec2) -> Option<Self> {
        if quad.iter().any(|p| !p.is_finite()) {
            return None;
        }
        let mut a = [DVec2::ZERO; 12];
        a[..4].copy_from_slice(&quad);
        let mut count = 4;
        for (axis, sign, radius) in [(0, 1., h.x), (0, -1., h.x), (1, 1., h.y), (1, -1., h.y)] {
            if count == 0 {
                break;
            }
            let mut v = [DVec2::ZERO; 12];
            let mut n = 0;
            let mut overflow = false;
            let mut push = |p: DVec2| {
                if n == v.len() {
                    overflow = true;
                    return;
                }
                if n == 0 || p != v[n - 1] {
                    v[n] = p;
                    n += 1;
                }
            };
            let mut prev = a[count - 1];
            let mut gp = radius - sign * prev[axis];
            for &p in &a[..count] {
                let g = radius - sign * p[axis];
                if (gp < 0.) != (g < 0.) {
                    let mut x = prev + (p - prev) * (gp / (gp - g));
                    x[axis] = sign * radius;
                    push(x);
                }
                if g >= 0. {
                    push(p);
                }
                prev = p;
                gp = g;
            }
            if overflow {
                return None;
            }
            if n > 1 && v[0] == v[n - 1] {
                n -= 1;
            }
            a = v;
            count = n;
        }
        if count > 8 {
            return None;
        }
        let mut result = Self {
            vertices: [DVec2::ZERO; 8],
            count: count as u8,
            area: 0.,
        };
        // Intersection of two convex quadrilaterals has at most eight vertices.
        result.vertices[..count].copy_from_slice(&a[..count]);
        if count >= 3 {
            let orig = a[0];
            result.area = (1..count - 1)
                .map(|i| (a[i] - orig).perp_dot(a[i + 1] - orig))
                .sum::<f64>()
                .abs()
                * 0.5;
        }
        Some(result)
    }
}
mod uniform_slide;
pub use uniform_slide::{ComovingSupportStop, UniformSlidingStep};
mod shared;
pub(crate) use shared::contact_centers;
pub use shared::{BoundSharedFace, CoplanarPad, SharedFaceStep, SharedSiteLoad};
#[cfg(test)]
mod tests;
