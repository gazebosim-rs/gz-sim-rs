// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Swept contact in a translating and shortest-arc rotating local frame.
use super::{XpbdError, capsule::Capsule, cuboid};
use glam::{DQuat, DVec3};

#[derive(Clone, Copy)]
pub(super) enum OrientedShape {
    Cuboid(DVec3),
    Capsule(Capsule),
}
impl OrientedShape {
    /// Radius of a conservative enclosing sphere.
    pub fn bound(self) -> f64 {
        match self {
            Self::Cuboid(h) => h.length(),
            Self::Capsule(c) => c.half_length + c.radius,
        }
    }
    pub fn extent(self) -> f64 {
        match self {
            Self::Cuboid(h) => h.max_element(),
            Self::Capsule(c) => c.half_length + c.radius,
        }
    }
    pub fn witness(self, p: DVec3) -> (f64, DVec3, DVec3) {
        match self {
            Self::Cuboid(h) => cuboid::witness(p, h),
            Self::Capsule(c) => c.signed_witness(p),
        }
    }
    fn linear_entry(self, a: DVec3, b: DVec3, skin: f64) -> Option<f64> {
        match self {
            Self::Cuboid(h) => cuboid::linear_entry(a, b, h, skin),
            Self::Capsule(c) => c.linear_entry(a, b, skin),
        }
    }
    pub fn rounded(self, local_normal: DVec3) -> bool {
        match self {
            Self::Cuboid(_) => {
                local_normal
                    .abs()
                    .cmpgt(DVec3::splat(1e-9))
                    .bitmask()
                    .count_ones()
                    > 1
            }
            Self::Capsule(_) => true,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Hit {
    pub time: f64,
    pub local_normal: DVec3,
    pub local_surface: DVec3,
    pub initial_overlap: f64,
}

#[derive(Clone, Copy)]
pub(super) struct Sweep {
    pub geometry: OrientedShape,
    pub radius: f64,
    /// World particle position relative to the moving receiver center.
    pub start: DVec3,
    pub end: DVec3,
    pub q0: DQuat,
    pub q1: DQuat,
}
impl Sweep {
    pub fn angular_travel(&self) -> DVec3 {
        if self.q0 == self.q1 || self.q0 == -self.q1 {
            return DVec3::ZERO;
        }
        let q = self.q1 * self.q0.conjugate();
        let q = if q.w < 0. { -q } else { q };
        let vector = DVec3::new(q.x, q.y, q.z);
        let norm = vector.length();
        if norm == 0. {
            DVec3::ZERO
        } else {
            vector * (2. * norm.atan2(q.w) / norm)
        }
    }
    pub fn shape_rotates(&self, angular: DVec3) -> bool {
        match self.geometry {
            OrientedShape::Cuboid(_) => angular != DVec3::ZERO,
            OrientedShape::Capsule(_) => {
                angular.cross(self.q0 * DVec3::Z).length() > 64. * f64::EPSILON * angular.length()
            }
        }
    }
    pub fn end_witness(&self, hit: &Hit) -> (DVec3, DVec3) {
        let transport = match self.geometry {
            OrientedShape::Cuboid(_) => self.q1,
            OrientedShape::Capsule(_) => {
                // Axial spin changes the material velocity but leaves the capsule
                // invariant. Transport the geometric tangent by axis tilt only.
                let at_hit = self.rotation_at(hit.time);
                DQuat::from_rotation_arc(at_hit * DVec3::Z, self.q1 * DVec3::Z) * at_hit
            }
        };
        (transport * hit.local_normal, transport * hit.local_surface)
    }
    pub fn rotation_at(&self, t: f64) -> DQuat {
        DQuat::from_scaled_axis(t * self.angular_travel()) * self.q0
    }
    fn local(&self, t: f64) -> DVec3 {
        self.rotation_at(t).conjugate() * self.start.lerp(self.end, t)
    }
    /// A touching grazing orbit has zero impulse. When the computed entry
    /// speed is within the CCD spatial uncertainty, require a crossing of a
    /// slightly smaller offset surface to distinguish impact from tangency.
    pub fn impact(&self) -> Result<Option<Hit>, XpbdError> {
        let Some(hit) = self.cast()? else {
            return Ok(None);
        };
        let w = self.angular_travel();
        if !self.shape_rotates(w) {
            return Ok(Some(hit));
        }
        let tol = 1e-10 * self.geometry.extent().max(self.radius)
            + 64. * f64::EPSILON * self.start.length().max(self.end.length());
        if hit.initial_overlap > tol {
            return Ok(Some(hit));
        }
        let q = self.rotation_at(hit.time);
        let relative = self.end - self.start - w.cross(q * hit.local_surface);
        let closing = -relative.dot(q * hit.local_normal);
        let speed =
            (self.end - self.start).length() + w.length() * (self.geometry.bound() + self.radius);
        let rounding = match self.geometry {
            OrientedShape::Cuboid(_) => self.radius,
            OrientedShape::Capsule(c) => c.radius + self.radius,
        };
        if closing > 8. * speed * (tol / rounding).sqrt() {
            return Ok(Some(hit));
        }
        if rounding <= 8. * tol {
            return Err(XpbdError::UnresolvedSweep);
        }
        let shrink = 8. * tol;
        let mut core = *self;
        core.radius = (self.radius - shrink).max(0.);
        if let OrientedShape::Capsule(c) = &mut core.geometry {
            c.radius -= (shrink - self.radius).max(0.);
        }
        Ok(core.cast()?.map(|mut inner| {
            let (_, normal, surface) = self.geometry.witness(self.local(inner.time));
            inner.local_normal = normal;
            inner.local_surface = surface;
            inner.initial_overlap = hit.initial_overlap;
            inner
        }))
    }
    pub fn cast(&self) -> Result<Option<Hit>, XpbdError> {
        let scale = self
            .start
            .length()
            .max(self.end.length())
            .max(self.geometry.bound())
            .max(self.radius);
        if !scale.is_finite() || !(self.end - self.start).length_squared().is_finite() {
            return Err(XpbdError::Numerical);
        }
        let tol = 1e-10 * self.geometry.extent().max(self.radius) + 64. * f64::EPSILON * scale;
        let local_a = self.q0.conjugate() * self.start;
        let local_b = self.q1.conjugate() * self.end;
        let (initial_gap, _, _) = self.geometry.witness(local_a);
        let initial_overlap = (self.radius - initial_gap).max(0.);
        let angular = self.angular_travel();
        let angle = angular.length();
        let hit_at = |time: f64| {
            let (_, normal, surface) = self.geometry.witness(self.local(time));
            Hit {
                time,
                local_normal: normal,
                local_surface: surface,
                initial_overlap,
            }
        };
        if !self.shape_rotates(angular) {
            let local_b = self.q0.conjugate() * self.end;
            return Ok(self
                .geometry
                .linear_entry(local_a, local_b, self.radius)
                .map(hit_at));
        }
        // For x(t) = R(t)^T [a+t(b-a)], |x''| <= w² max|x| + 2w|b-a|.
        // Its chord is within |x''| Δt² / 8 of every point on the true path.
        let curvature = angle * angle * self.start.length().max(self.end.length())
            + 2. * angle * (self.end - self.start).length();
        if !curvature.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let mut search = RotationSearch {
            sweep: self,
            curvature,
            tolerance: tol,
            remaining: 4096,
        };
        Ok(search.entry(0., 1., local_a, local_b, 0)?.map(hit_at))
    }
}

/// Search left first so the first certified candidate is the incident side.
/// Bounded depth and work avoid allocating a new stack for each particle.
struct RotationSearch<'a> {
    sweep: &'a Sweep,
    curvature: f64,
    tolerance: f64,
    remaining: u32,
}
impl RotationSearch<'_> {
    fn entry(
        &mut self,
        lo: f64,
        hi: f64,
        a: DVec3,
        b: DVec3,
        depth: u32,
    ) -> Result<Option<f64>, XpbdError> {
        self.remaining = self
            .remaining
            .checked_sub(1)
            .ok_or(XpbdError::UnresolvedSweep)?;
        let error = self.curvature * (hi - lo) * (hi - lo) / 8.;
        let radius = self.sweep.radius;
        let Some(fraction) =
            self.sweep
                .geometry
                .linear_entry(a, b, radius + error + self.tolerance)
        else {
            return Ok(None);
        };
        if error <= self.tolerance
            || fraction == 0. && self.sweep.geometry.witness(a).0 <= radius + self.tolerance
        {
            return Ok(Some(lo + (hi - lo) * fraction));
        }
        let mid = lo + 0.5 * (hi - lo);
        if depth == 64 || mid == lo || mid == hi {
            return Err(XpbdError::UnresolvedSweep);
        }
        let p = self.sweep.local(mid);
        if let Some(first) = self.entry(lo, mid, a, p, depth + 1)? {
            return Ok(Some(first));
        }
        self.entry(mid, hi, p, b, depth + 1)
    }
}

#[cfg(test)]
#[path = "../tests/private/cuboid_geometry.rs"]
mod tests;
