// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! All flat-contact acceleration responses at the instant a translating sole
//! slides across two supports. Its yaw acceleration is free.
use super::*;
use glam::DVec3;
#[derive(Clone, Copy, Debug)]
pub struct SlidingAccelerationVertex {
    pub fraction_b: f64,
    pub yaw_acceleration: f64,
    pub planar_acceleration: DVec2,
    pub site_centers: [Option<DVec2>; 2],
}
#[derive(Clone, Debug)]
pub struct SlidingAccelerationSet {
    /// Convex hull, counterclockwise in (load fraction, angular acceleration).
    /// Every convex mixture of these responses is mechanically attainable.
    pub vertices: Vec<SlidingAccelerationVertex>,
}
impl SlidingAccelerationSet {
    pub fn fraction_at_yaw(&self, beta: f64) -> Result<Option<[f64; 2]>, PhysicsError> {
        if !beta.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let mut range = [f64::INFINITY, f64::NEG_INFINITY];
        let scale = self
            .vertices
            .iter()
            .map(|v| v.yaw_acceleration.abs())
            .fold(0., f64::max);
        let tol = 128. * f64::EPSILON * (1. + scale);
        let mut add = |x: f64| {
            range[0] = range[0].min(x);
            range[1] = range[1].max(x);
        };
        for (i, v) in self.vertices.iter().enumerate() {
            let w = self.vertices[(i + 1) % self.vertices.len()];
            let a = v.yaw_acceleration - beta;
            let b = w.yaw_acceleration - beta;
            if a.abs() <= tol {
                add(v.fraction_b);
            }
            if (a < -tol && b > tol) || (a > tol && b < -tol) {
                add(v.fraction_b + (w.fraction_b - v.fraction_b) * (a / (a - b)));
            }
        }
        Ok((range[0] <= range[1]).then_some(range))
    }
}
#[derive(Clone, Copy)]
struct Column {
    moment: DVec2,
    yaw: f64,
    site: usize,
    point: DVec2,
}
impl CoplanarSlidingContact {
    /// `inertia_normal` is I*n in an orthonormal (tangent, tangent, normal)
    /// frame. The sole must initially have zero angular velocity. Euler's
    /// equations allow yaw acceleration while both tilt accelerations vanish.
    pub fn free_yaw(
        self,
        patches: [&[DVec2]; 2],
        inertia_normal: DVec3,
    ) -> Result<Option<SlidingAccelerationSet>, PhysicsError> {
        if !inertia_normal.is_finite() || inertia_normal.z <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(k) = self.tractions(patches)? else {
            return Ok(None);
        };
        let nf = -self.mass * self.gravity_normal;
        let mut cols = Vec::new();
        for (site, patch) in patches.iter().enumerate() {
            let scale = patch
                .iter()
                .map(|v| v.distance(self.center))
                .fold(0., f64::max)
                .max(f64::MIN_POSITIVE);
            Patch::from(patch, self.center, scale)?;
            for &point in *patch {
                let torque = (point - self.center)
                    .extend(-self.height)
                    .cross(k[site].extend(1.));
                let moment =
                    torque.truncate() - inertia_normal.truncate() * (torque.z / inertia_normal.z);
                cols.push(Column {
                    moment,
                    yaw: torque.z * nf / inertia_normal.z,
                    site,
                    point,
                });
            }
        }
        let scale = cols
            .iter()
            .map(|v| v.moment.abs().max_element())
            .fold(0., f64::max);
        if !scale.is_finite() || cols.iter().any(|v| !v.yaw.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        if scale > 0. {
            for c in &mut cols {
                c.moment /= scale;
            }
        }
        let mut candidates = Vec::new();
        let tol = 256. * f64::EPSILON;
        let mut add = |w: &[(usize, f64)]| {
            if w.iter().any(|(_, v)| *v < -tol) {
                return;
            }
            let sum = w.iter().map(|(_, v)| v.max(0.)).sum::<f64>();
            let mut moment = DVec2::ZERO;
            let mut share = [0.; 2];
            let mut cp = [DVec2::ZERO; 2];
            let mut yaw = 0.;
            for &(j, v) in w {
                let c = cols[j];
                let f = v.max(0.) / sum;
                moment += f * c.moment;
                yaw += f * c.yaw;
                share[c.site] += f;
                cp[c.site] += f * c.point;
            }
            if moment.length() > 2. * tol {
                return;
            }
            candidates.push(SlidingAccelerationVertex {
                fraction_b: share[1].clamp(0., 1.),
                yaw_acceleration: yaw,
                planar_acceleration: self.gravity_tangent
                    - self.gravity_normal * (k[0] * share[0] + k[1] * share[1]),
                site_centers: std::array::from_fn(|j| (share[j] > 0.).then(|| cp[j] / share[j])),
            });
        };
        // Unit total pressure and its two horizontal Euler equations have
        // rank at most three. Every extremal pressure uses at most three sites.
        for i in 0..cols.len() {
            add(&[(i, 1.)]);
            for j in 0..i {
                let origin = cols[i].moment;
                let d = cols[j].moment - origin;
                if d.length_squared() > 0. {
                    let u = -origin.dot(d) / d.length_squared();
                    add(&[(i, 1. - u), (j, u)]);
                }
                for (h, earlier) in cols[..j].iter().enumerate() {
                    let e = earlier.moment - origin;
                    let det = d.perp_dot(e);
                    if det.abs() <= 64. * f64::EPSILON * d.length() * e.length() {
                        continue;
                    }
                    let u = (-origin).perp_dot(e) / det;
                    let v = d.perp_dot(-origin) / det;
                    add(&[(i, 1. - u - v), (j, u), (h, v)]);
                }
            }
        }
        if candidates.is_empty() {
            return Ok(None);
        }
        candidates.sort_by(|a, b| {
            a.fraction_b
                .total_cmp(&b.fraction_b)
                .then(a.yaw_acceleration.total_cmp(&b.yaw_acceleration))
        });
        let yscale = candidates
            .iter()
            .map(|v| v.yaw_acceleration.abs())
            .fold(0., f64::max)
            .max(f64::MIN_POSITIVE);
        let coords =
            |v: SlidingAccelerationVertex| DVec2::new(v.fraction_b, v.yaw_acceleration / yscale);
        candidates.dedup_by(|a, b| coords(*a).distance(coords(*b)) <= 32. * f64::EPSILON);
        let mut hull = Vec::new();
        let mut chain = |list: &[SlidingAccelerationVertex]| {
            let mut boundary = Vec::new();
            for &x in list {
                while boundary.len() >= 2 {
                    let len = boundary.len();
                    let a = coords(boundary[len - 2]);
                    let b = coords(boundary[len - 1]);
                    let c = coords(x);
                    if (b - a).perp_dot(c - a)
                        > 128. * f64::EPSILON * (b - a).length() * (c - a).length()
                    {
                        break;
                    }
                    boundary.pop();
                }
                boundary.push(x);
            }
            boundary.pop();
            hull.extend(boundary);
        };
        if candidates.len() == 1 {
            return Ok(Some(SlidingAccelerationSet {
                vertices: candidates,
            }));
        }
        chain(&candidates);
        candidates.reverse();
        chain(&candidates);
        Ok(Some(SlidingAccelerationSet { vertices: hull }))
    }
}
