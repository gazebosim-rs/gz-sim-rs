// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Particle centers against a translated finite capsule of fixed axial direction.
use super::XpbdError;
use glam::DVec3;

#[derive(Clone, Copy)]
pub(super) struct Capsule {
    pub axis: DVec3,
    pub half_length: f64,
    pub radius: f64,
    pub angular_speed: f64,
}

impl Capsule {
    pub fn new(
        axis: [f64; 3],
        half_length: f64,
        radius: f64,
        angular_speed: f64,
    ) -> Result<Self, XpbdError> {
        let axis = DVec3::from(axis)
            .try_normalize()
            .ok_or(XpbdError::InvalidBoundary)?;
        if !(half_length.is_finite()
            && half_length > 0.
            && radius.is_finite()
            && radius > 0.
            && (half_length + radius).is_finite()
            && angular_speed.is_finite()
            && (angular_speed * radius).is_finite())
        {
            return Err(XpbdError::InvalidBoundary);
        }
        Ok(Self {
            axis,
            half_length,
            radius,
            angular_speed,
        })
    }
    fn witness(self, p: DVec3) -> DVec3 {
        p - self.axis * p.dot(self.axis).clamp(-self.half_length, self.half_length)
    }

    fn trace(
        self,
        a: DVec3,
        delta: DVec3,
        r: f64,
        eps: f64,
        grazing: bool,
    ) -> Option<(f64, DVec3)> {
        let z = a.dot(self.axis);
        let vz = delta.dot(self.axis);
        let mut first = None;
        let mut accept = |t: f64, normal: DVec3| {
            if first.is_none_or(|(earlier, _)| t < earlier) {
                first = Some((t, normal));
            }
        };
        // Every accepted root is clamped to this time interval. Axial coordinates of
        // the chord stay between its endpoints, including when axial travel reverses sign.
        let end_z = z + vz;
        let low = z.min(end_z);
        let high = z.max(end_z);
        if low <= self.half_length + eps
            && high >= -self.half_length - eps
            && let Some((t, n)) = entry(a - z * self.axis, delta - vz * self.axis, r, eps, grazing)
            && (z + t * vz).abs() <= self.half_length + eps
        {
            accept(t, n);
        }
        for sign in [-1., 1.] {
            if (sign < 0. && low > -self.half_length + eps)
                || (sign > 0. && high < self.half_length - eps)
            {
                continue;
            }
            let cap = self.axis * (sign * self.half_length);
            if let Some((t, n)) = entry(a - cap, delta, r, eps, grazing)
                && sign * (z + t * vz) >= self.half_length - eps
            {
                accept(t, n);
            }
        }
        first
    }

    pub(super) fn signed_witness(self, p: DVec3) -> (f64, DVec3, DVec3) {
        let radial = self.witness(p);
        let n = radial
            .try_normalize()
            .unwrap_or_else(|| self.axis.any_orthonormal_vector());
        (
            radial.length() - self.radius,
            n,
            p - radial + self.radius * n,
        )
    }

    pub(super) fn linear_entry(self, a: DVec3, b: DVec3, skin: f64) -> Option<f64> {
        let r = self.radius + skin;
        if self.witness(a).length() <= r {
            return Some(0.);
        }
        let eps = 64. * f64::EPSILON * (a.length() + b.length() + self.half_length + r);
        self.trace(a, b - a, r, eps, true).map(|(time, _)| time)
    }

    /// Return the first incident tangent plane (normal, penetration at the end,
    /// initial overlap). Coordinates have the center's linear motion removed.
    pub fn project(
        self,
        a: DVec3,
        b: DVec3,
        particle_radius: f64,
    ) -> Result<Option<(DVec3, f64, f64)>, XpbdError> {
        let r = self.radius + particle_radius;
        let w = self.witness(a);
        let d0 = w.length();
        let delta = b - a;
        let scale = a.length() + b.length() + self.half_length + r;
        let travel = delta.length();
        if !scale.is_finite() || !d0.is_finite() || !travel.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let eps = 64. * f64::EPSILON * scale;
        // Distance to a segment is 1-Lipschitz. Leave several numerical tolerances
        // on the clear side before skipping the exact strip and hemisphere tests.
        if d0 - travel > r + 4. * eps {
            return Ok(None);
        }
        let first = (d0 >= r - eps)
            .then(|| self.trace(a, delta, r, eps, false))
            .flatten();
        let normal = if let Some((_, n)) = first {
            n
        } else if d0 < r - eps
            && let Some(n) = w.try_normalize()
            && delta.dot(n) < -eps
        {
            // An already overlapping particle can cross the whole solid within
            // one interval. Resolve its inward motion on its initial side.
            n
        } else {
            let nearest = self.witness(b);
            let distance = nearest.length();
            if distance >= r - eps {
                return Ok(None);
            }
            nearest
                .try_normalize()
                .or_else(|| w.try_normalize())
                .unwrap_or_else(|| self.axis.any_orthonormal_vector())
        };
        // Support function of the inflated segment. The incident plane transports
        // a crossed particle to the correct side even when both endpoints are clear.
        let depth = r + self.half_length * normal.dot(self.axis).abs() - b.dot(normal);
        if !depth.is_finite() {
            return Err(XpbdError::Numerical);
        }
        if depth > eps || (first.is_some() && depth >= -eps && delta.dot(normal) < -eps) {
            Ok(Some((normal, depth.max(0.), (r - d0).max(0.))))
        } else {
            Ok(None)
        }
    }
}

// This calculation also serves the 2D radial section of the cylindrical strip.
fn entry(p: DVec3, v: DVec3, radius: f64, eps: f64, grazing: bool) -> Option<(f64, DVec3)> {
    let distance = v.length();
    if distance == 0. {
        return None;
    }
    let direction = v / distance;
    let along = -p.dot(direction);
    if along < 0. {
        return None;
    }
    let perpendicular = (p + along * direction).length();
    // At exact tangency there is no inward motion or normal impulse.
    if perpendicular > radius + eps || (!grazing && perpendicular >= radius - eps) {
        return None;
    }
    let chord = ((radius - perpendicular).max(0.).sqrt()) * ((radius + perpendicular).sqrt());
    let enter = along - chord;
    if enter < -eps || enter > distance + eps {
        return None;
    }
    let t = (enter / distance).clamp(0., 1.);
    let n = (p + t * v).try_normalize()?;
    if !grazing && v.dot(n) >= -eps {
        return None;
    }
    Some((t, n))
}

#[cfg(test)]
#[path = "capsule_trace_tests.rs"]
mod trace_tests;
