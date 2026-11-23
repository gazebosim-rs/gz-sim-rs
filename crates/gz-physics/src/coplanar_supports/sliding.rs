// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Coulomb sliding on two independently translating coplanar pads.
mod yaw;
use super::*;
pub use yaw::{SlidingAccelerationSet, SlidingAccelerationVertex};
#[derive(Clone, Copy, Debug)]
pub struct CoplanarSlidingContact {
    pub mass: f64,
    pub height: f64,
    pub gravity_tangent: DVec2,
    pub gravity_normal: f64,
    pub center: DVec2,
    pub velocity: DVec2,
    pub pad_velocities: [DVec2; 2],
    pub kinetic_friction: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub struct SlidingLoadSet {
    pub load: SupportLoadRange,
    pub tangent_per_normal: [DVec2; 2],
    body: CoplanarSlidingContact,
    sections: Option<(DVec2, [Option<[f64; 2]>; 2])>,
}
#[derive(Clone, Copy, Debug)]
pub struct SlidingLoadChoice {
    pub normal_force: [f64; 2],
    pub tangential_force: [DVec2; 2],
    pub acceleration: DVec2,
    pub net_pressure_center: DVec2,
    pub dissipated_power: f64,
    pub prescribed_support_power: f64,
    pub site_centers: [Option<DVec2>; 2],
}
fn section(poly: &[DVec2], origin: DVec2, axis: DVec2) -> Option<[f64; 2]> {
    if poly.is_empty() {
        return None;
    }
    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    let mut add = |v: DVec2| {
        let t = (v - origin).dot(axis);
        range[0] = range[0].min(t);
        range[1] = range[1].max(t);
    };
    let tol = 128. * f64::EPSILON * poly.iter().map(|v| v.distance(origin)).fold(0., f64::max);
    for (i, &p) in poly.iter().enumerate() {
        let q = poly[(i + 1) % poly.len()];
        let a = axis.perp_dot(p - origin);
        let b = axis.perp_dot(q - origin);
        if a.abs() <= tol {
            add(p);
        }
        if (a < -tol && b > tol) || (b < -tol && a > tol) {
            add(p + (q - p) * (a / (a - b)));
        }
    }
    (range[0] <= range[1]).then_some(range)
}
impl CoplanarSlidingContact {
    pub(super) fn tractions(
        self,
        patches: [&[DVec2]; 2],
    ) -> Result<Option<[DVec2; 2]>, PhysicsError> {
        if !self.mass.is_finite()
            || self.mass <= 0.
            || !self.height.is_finite()
            || self.height <= 0.
            || !self.gravity_normal.is_finite()
            || !(self.gravity_normal * self.mass).is_finite()
            || !self.gravity_tangent.is_finite()
            || !self.center.is_finite()
            || !self.velocity.is_finite()
            || self.pad_velocities.iter().any(|v| !v.is_finite())
            || self
                .kinetic_friction
                .iter()
                .any(|u| !u.is_finite() || *u < 0.)
            || patches
                .iter()
                .any(|p| p.len() > 16 || p.iter().any(|v| !v.is_finite()))
        {
            return Err(PhysicsError::InvalidValue);
        }
        if self.gravity_normal >= 0. {
            return Ok(None);
        }
        let mut k = [DVec2::ZERO; 2];
        for i in 0..2 {
            if self.kinetic_friction[i] == 0. || patches[i].is_empty() {
                continue;
            }
            let rel = self.velocity - self.pad_velocities[i];
            let speed = rel.length();
            if speed
                <= 128.
                    * f64::EPSILON
                    * (1. + self.velocity.length() + self.pad_velocities[i].length())
            {
                return Ok(None);
            }
            k[i] = rel * (-self.kinetic_friction[i] / speed);
        }
        Ok(Some(k))
    }
    /// Solve the instantaneous nonspinning mode. A loaded positive-friction pad
    /// must be sliding. Static or poorly conditioned traction directions decline.
    pub fn solve(self, patches: [&[DVec2]; 2]) -> Result<Option<SlidingLoadSet>, PhysicsError> {
        let Some(k) = self.tractions(patches)? else {
            return Ok(None);
        };
        let delta = k[0] - k[1];
        if delta == DVec2::ZERO {
            let Some(load) = coplanar_load_range(self.center - self.height * k[0], patches)? else {
                return Ok(None);
            };
            return Ok(Some(SlidingLoadSet {
                load,
                tangent_per_normal: k,
                body: self,
                sections: None,
            }));
        }
        if delta.length() <= 128. * f64::EPSILON * k[0].length().max(k[1].length()) {
            return Ok(None);
        }
        // Zero pitch/roll moment gives sum w_i(p_i + d k_i) = c.
        // Zero yaw moment gives (p_A-p_B) × (k_A-k_B) = 0. The two shifted
        // pressure points consequently lie on one line through c.
        let axis = delta.normalize();
        let mut ranges = [None; 2];
        for i in 0..2 {
            let scale = patches[i]
                .iter()
                .map(|v| v.distance(self.center))
                .fold(0., f64::max)
                .max(f64::MIN_POSITIVE);
            Patch::from(patches[i], self.center, scale)?;
            ranges[i] = section(patches[i], self.center - self.height * k[i], axis);
        }
        let ends = ranges.map(|r| r.map(|p| p.map(|x| DVec2::X * x)));
        let empty: &[DVec2] = &[];
        let Some(load) = coplanar_load_range(
            DVec2::ZERO,
            [
                ends[0].as_ref().map_or(empty, |p| p.as_slice()),
                ends[1].as_ref().map_or(empty, |p| p.as_slice()),
            ],
        )?
        else {
            return Ok(None);
        };
        Ok(Some(SlidingLoadSet {
            load,
            tangent_per_normal: k,
            body: self,
            sections: Some((axis, ranges)),
        }))
    }
}
impl SlidingLoadSet {
    pub fn accelerations(self) -> [DVec2; 2] {
        self.load.fraction_b.map(|b| {
            self.body.gravity_tangent
                - self.body.gravity_normal
                    * ((1. - b) * self.tangent_per_normal[0] + b * self.tangent_per_normal[1])
        })
    }
    /// Select an admissible fraction and exhibit both physical pressure centers.
    pub fn choose(
        self,
        patches: [&[DVec2]; 2],
        fraction_b: f64,
    ) -> Result<Option<SlidingLoadChoice>, PhysicsError> {
        if !fraction_b.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let [lo, hi] = self.load.fraction_b;
        if !(lo..=hi).contains(&fraction_b) {
            return Ok(None);
        }
        let b = fraction_b;
        let a = 1. - b;
        let c = self.body.center;
        let depth = self.body.height;
        let k = self.tangent_per_normal;
        let points = if let Some((axis, ranges)) = self.sections {
            if b == 0. {
                [Some(c - depth * k[0]), None]
            } else if a == 0. {
                [None, Some(c - depth * k[1])]
            } else {
                let [Some(ra), Some(rb)] = ranges else {
                    return Ok(None);
                };
                let w = 0.5 * ((a * ra[0]).max(-b * rb[1]) + (a * ra[1]).min(-b * rb[0]));
                [
                    Some(c + axis * (w / a) - depth * k[0]),
                    Some(c - axis * (w / b) - depth * k[1]),
                ]
            }
        } else {
            let Some(w) =
                crate::frictionless_face_support::contact_centers(patches, c - depth * k[0], b)?
            else {
                return Ok(None);
            };
            w
        };
        // Revalidate against the caller's actual footprint before publishing.
        for i in 0..2 {
            if let Some(p) = points[i]
                && coplanar_load_range(p, [patches[i], &[]])?.is_none()
            {
                return Ok(None);
            }
        }
        let nf = -self.body.mass * self.body.gravity_normal;
        let normal = [a * nf, b * nf];
        let tang = [normal[0] * k[0], normal[1] * k[1]];
        let net = tang[0] + tang[1];
        let support =
            tang[0].dot(self.body.pad_velocities[0]) + tang[1].dot(self.body.pad_velocities[1]);
        Ok(Some(SlidingLoadChoice {
            normal_force: normal,
            tangential_force: tang,
            acceleration: self.body.gravity_tangent + net / self.body.mass,
            net_pressure_center: c - depth * net / nf,
            dissipated_power: -(tang[0].dot(self.body.velocity - self.body.pad_velocities[0])
                + tang[1].dot(self.body.velocity - self.body.pad_velocities[1])),
            prescribed_support_power: support,
            site_centers: points,
        }))
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;
    fn contact() -> CoplanarSlidingContact {
        CoplanarSlidingContact {
            mass: 2.,
            height: 0.13,
            gravity_tangent: DVec2::new(0.1, -0.02),
            gravity_normal: -9.81,
            center: DVec2::ZERO,
            velocity: DVec2::new(0.6, 0.14),
            pad_velocities: [DVec2::new(-0.3, -0.04), DVec2::new(1.4, 0.38)],
            kinetic_friction: [0.2, 0.58],
        }
    }
    fn quad(x: f64) -> [DVec2; 4] {
        let h = DVec2::new(0.105, 0.16);
        [
            DVec2::new(-h.x, -h.y),
            DVec2::new(h.x, -h.y),
            h,
            DVec2::new(-h.x, h.y),
        ]
        .map(|p| p + DVec2::X * x)
    }
    #[test]
    fn unequal_kinetic_directions_allow_multiple_accelerations_but_every_choice_balances_torque() {
        let c = contact();
        let a = quad(-0.13);
        let b = quad(0.13);
        let patches = [a.as_slice(), b.as_slice()];
        let set = c.solve(patches).unwrap().unwrap();
        let [lo, hi] = set.load.fraction_b;
        assert!(hi - lo > 0.25);
        assert!(set.accelerations()[0].distance(set.accelerations()[1]) > 1.);
        for w in [lo, 0.25 * lo + 0.75 * hi, hi] {
            let s = set.choose(patches, w).unwrap().unwrap();
            let mut torque = DVec3::ZERO;
            for (k, cp) in s.site_centers.into_iter().enumerate() {
                if let Some(p) = cp {
                    let r = (p - c.center).extend(-c.height);
                    let f = s.tangential_force[k].extend(s.normal_force[k]);
                    torque += r.cross(f);
                }
            }
            assert!(torque.length() < 2e-13 && s.dissipated_power > 0.);
            let belt = s.prescribed_support_power - s.dissipated_power;
            assert!(
                (belt
                    - c.velocity
                        .dot(s.tangential_force[0] + s.tangential_force[1]))
                .abs()
                    < 1e-12
            );
        }
    }
    #[test]
    fn identical_tractions_fix_free_translation_even_though_pad_loads_are_not_unique() {
        let mut c = contact();
        c.pad_velocities = [DVec2::ZERO; 2];
        c.kinetic_friction = [0.18; 2];
        let a = quad(-0.13);
        let b = quad(0.13);
        let p = [a.as_slice(), b.as_slice()];
        let set = c.solve(p).unwrap().unwrap();
        assert!(set.load.fraction_b[1] > set.load.fraction_b[0] + 0.2);
        assert!(set.accelerations()[0].distance(set.accelerations()[1]) < 1e-13);
        assert!(
            set.choose(p, set.load.fraction_b.iter().sum::<f64>() * 0.5)
                .unwrap()
                .is_some()
        );
        c.pad_velocities[0] = c.velocity;
        assert!(c.solve(p).unwrap().is_none());
    }
}
#[cfg(test)]
mod free_yaw_tests {
    use super::*;
    use glam::DVec3;
    #[test]
    fn the_complete_coupled_yaw_hull_contains_the_zero_angular_acceleration_family() {
        let c = CoplanarSlidingContact {
            mass: 2.,
            height: 0.13,
            gravity_tangent: DVec2::new(0.05, -0.09),
            gravity_normal: -9.81,
            center: DVec2::new(0.014, 0.02),
            velocity: DVec2::new(0.76, 0.14),
            pad_velocities: [DVec2::new(-0.4, 0.19), DVec2::new(1.7, -0.31)],
            kinetic_friction: [0.22, 0.44],
        };
        let patch = |x: f64| {
            [
                (x - 0.10, -0.12),
                (x + 0.10, -0.12),
                (x + 0.10, 0.13),
                (x - 0.10, 0.13),
            ]
            .map(DVec2::from)
        };
        let a = patch(-0.13);
        let b = patch(0.14);
        let pads = [a.as_slice(), b.as_slice()];
        let no_yaw = c.solve(pads).unwrap().unwrap().load.fraction_b;
        for inertia in [DVec3::new(0., 0., 0.059), DVec3::new(-0.008, 0.006, 0.059)] {
            let set = c.free_yaw(pads, inertia).unwrap().unwrap();
            assert!(set.vertices.len() >= 3);
            let cut = set.fraction_at_yaw(0.).unwrap().unwrap();
            assert!((cut[0] - no_yaw[0]).abs().max((cut[1] - no_yaw[1]).abs()) < 1e-12);
            let width = set
                .vertices
                .iter()
                .map(|v| v.yaw_acceleration)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                    (a.min(v), b.max(v))
                });
            assert!(width.0 < -0.1 && width.1 > 0.1);
            for v in set.vertices {
                let mut torque = DVec3::ZERO;
                for (j, p) in v.site_centers.into_iter().enumerate() {
                    if let Some(p) = p {
                        let w = if j == 0 {
                            1. - v.fraction_b
                        } else {
                            v.fraction_b
                        };
                        let traction =
                            -(c.velocity - c.pad_velocities[j]).normalize() * c.kinetic_friction[j];
                        torque += (p - c.center)
                            .extend(-c.height)
                            .cross(-w * c.mass * c.gravity_normal * traction.extend(1.));
                    }
                }
                assert!(torque.distance(inertia * v.yaw_acceleration) < 1e-13);
            }
        }
    }
}
