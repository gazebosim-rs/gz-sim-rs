// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full sticking on an incident straight line of a free rigid body.
//!
//! Angular momentum about the contact line cannot change under any pressure
//! there. The minimizer of post-impact kinetic energy at fixed axial angular
//! momentum is rotation about the line. A feasible Coulomb impulse field
//! attains this lower bound precisely when its barycentric endpoint resultants
//! can supply the required linear and angular impulses.
use crate::{EdgeImpulseSet, FiniteContactSegment, PhysicsError};
use glam::{DMat3, DVec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StickingLine {
    /// No incoming endpoint approaches at a resolved normal speed.
    NoApproach,
    /// The required endpoint-equivalent normal impulses include tension.
    Tensile,
    /// A required tangential resultant lies outside the available disks.
    OutsideFriction,
    /// Roundoff prevents separating a physical boundary from an infeasible case.
    Marginal,
    /// Attainable global kinetic-energy minimum, for a zero-restitution line.
    Attainable(LineStickingResponse),
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineStickingResponse {
    pub velocity: DVec3,
    pub angular_velocity: DVec3,
    pub impulse: DVec3,
    pub angular_impulse: DVec3,
    pub dissipated_energy: f64,
    /// Equivalent endpoint resultants in the line, cross-line and normal
    /// directions. A distributed pressure has the same zeroth/first moments.
    /// These are not measurements of endpoint point loads.
    pub equivalent: EdgeImpulseSet,
}

impl FiniteContactSegment {
    /// `velocity` is the velocity of the COM in the nonrotating translating
    /// support frame. `inertia` is the world-space tensor about the COM.
    /// Feasibility certifies a maximum-dissipation sticking response. It does
    /// not select this solution for other simultaneous impact closures.
    pub fn maximum_dissipation_stick(
        self,
        center: DVec3,
        velocity: DVec3,
        spin: DVec3,
        mass: f64,
        inertia: DMat3,
        friction: f64,
    ) -> Result<StickingLine, PhysicsError> {
        let [p, q] = self.moving_points;
        let n = self.normal;
        if ![p, q, center, velocity, spin, n]
            .into_iter()
            .all(|x| x.is_finite())
            || !inertia.is_finite()
            || !mass.is_finite()
            || mass <= 0.
            || !friction.is_finite()
            || friction < 0.
            || (n.length_squared() - 1.).abs() > 1e-10
        {
            return Err(PhysicsError::InvalidValue);
        }
        let scale = inertia
            .to_cols_array()
            .into_iter()
            .map(f64::abs)
            .fold(0., f64::max);
        if scale == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let scaled = inertia * (1. / scale);
        if (scaled - scaled.transpose())
            .to_cols_array()
            .into_iter()
            .any(|x| x.abs() > 1e-12)
            || scaled.x_axis.x <= 0.
            || scaled.x_axis.x * scaled.y_axis.y - scaled.x_axis.y * scaled.y_axis.x <= 0.
            || scaled.determinant() <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let delta = q - p;
        let length = delta.length();
        if !length.is_finite() || length <= 128. * f64::EPSILON * (1. + p.length() + q.length()) {
            return Err(PhysicsError::InvalidValue);
        }
        let axis = delta / length;
        if n.dot(axis).abs() > 1e-10 {
            return Err(PhysicsError::InvalidValue);
        }
        let transverse = n.cross(axis).normalize();
        let radius = center - p;
        let before = [p, q].map(|s| velocity + spin.cross(s - center));
        let vt = 256. * f64::EPSILON * (1. + before[0].length() + before[1].length());
        if before.iter().all(|v| v.dot(n) >= -vt) {
            return Ok(StickingLine::NoApproach);
        }
        let angular = axis.dot(inertia * spin + radius.cross(mass * velocity));
        let rotational = axis.dot(inertia * axis) + mass * axis.cross(radius).length_squared();
        let omega = axis * (angular / rotational);
        let v = omega.cross(radius);
        let impulse = mass * (v - velocity);
        let torque = inertia * (omega - spin);
        // d × j1 = torque - (p-C) × J. Only its component along
        // the line is unknown, because it does not affect the moment.
        let j1 = (torque + radius.cross(impulse)).cross(axis) / length;
        let j0 = impulse - j1;
        let normals = [j0.dot(n), j1.dot(n)];
        let across = [j0.dot(transverse), j1.dot(transverse)];
        let longitudinal = impulse.dot(axis);
        let tol = 512.
            * f64::EPSILON
            * (mass * (velocity.length() + v.length())
                + (torque.length() + radius.length() * impulse.length()) / length);
        if !tol.is_finite()
            || !v.is_finite()
            || !omega.is_finite()
            || !normals.iter().chain(&across).all(|v| v.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        if normals.iter().any(|v| *v < -tol) {
            return Ok(StickingLine::Tensile);
        }
        if normals.iter().any(|v| *v <= tol) {
            return Ok(StickingLine::Marginal);
        }
        let mut disks = [0.; 2];
        for i in 0..2 {
            let cap = friction * normals[i];
            let tang = across[i].abs();
            if tang > cap + tol {
                return Ok(StickingLine::OutsideFriction);
            }
            if tang + tol >= cap {
                return Ok(StickingLine::Marginal);
            }
            disks[i] = (cap - tang).sqrt() * (cap + tang).sqrt();
        }
        let lo = (-disks[0]).max(longitudinal - disks[1]);
        let hi = disks[0].min(longitudinal + disks[1]);
        if lo > hi + tol {
            return Ok(StickingLine::OutsideFriction);
        }
        if lo + tol >= hi {
            return Ok(StickingLine::Marginal);
        }
        let lost = 0.5
            * (mass * (v - velocity).length_squared()
                + (omega - spin).dot(inertia * (omega - spin)));
        if !lost.is_finite() || !(lo.is_finite() && hi.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(StickingLine::Attainable(LineStickingResponse {
            velocity: v,
            angular_velocity: omega,
            impulse,
            angular_impulse: torque,
            dissipated_energy: lost,
            equivalent: EdgeImpulseSet {
                longitudinal_total: longitudinal,
                transverse: across,
                normal: normals,
                first_longitudinal_range: [lo, hi],
            },
        }))
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn every_admissible_split_has_identical_wrench_and_attains_the_kinetic_energy_floor() {
        let quat = glam::DQuat::from_euler(glam::EulerRot::XYZ, 0.5, 0.8, 0.13);
        let rotation = DMat3::from_quat(quat);
        let inertia =
            rotation * DMat3::from_diagonal(DVec3::new(0.03, 0.08, 0.07)) * rotation.transpose();
        let axis = quat * DVec3::X;
        let y = quat * DVec3::Y;
        let n = quat * DVec3::Z;
        let center = DVec3::new(2.5, -0.8, 0.7);
        let p = center - 0.14 * axis - 0.12 * n;
        let q = p + 0.40 * axis;
        let segment = FiniteContactSegment {
            moving_points: [p, q],
            support_points: [p, q],
            normal: n,
        };
        let v = quat * DVec3::new(0.13, 0.05, -0.9);
        let spin = quat * DVec3::new(0.48, 0.3, -0.1);
        let mass = 2.3;
        let mu = 0.8;
        let StickingLine::Attainable(h) = segment
            .maximum_dissipation_stick(center, v, spin, mass, inertia, mu)
            .unwrap()
        else {
            panic!("expected sticking")
        };
        let energy = |v: DVec3, w: DVec3| 0.5 * (mass * v.length_squared() + w.dot(inertia * w));
        assert!(
            (energy(v, spin) - energy(h.velocity, h.angular_velocity) - h.dissipated_energy).abs()
                < 1e-12
        );
        for i in 0..=10 {
            let [lo, hi] = h.equivalent.first_longitudinal_range;
            let x = if i == 10 {
                hi
            } else {
                lo + (hi - lo) * f64::from(i) / 10.
            };
            let j = h
                .equivalent
                .split(x)
                .unwrap()
                .map(|t| axis * t[0] + y * t[1] + n * t[2]);
            assert!((j[0] + j[1]).distance(h.impulse) < 1e-12);
            assert!(
                ((p - center).cross(j[0]) + (q - center).cross(j[1])).distance(h.angular_impulse)
                    < 1e-12
            );
            for (x, j) in [(p, j[0]), (q, j[1])] {
                assert!((h.velocity + h.angular_velocity.cross(x - center)).length() < 1e-12);
                assert!(j.reject_from_normalized(n).length() <= mu * j.dot(n) + 1e-12);
            }
            // Any localized reaction preserves axial angular momentum. The
            // excess kinetic energy is exactly the squared twist difference.
            let at = p.lerp(q, f64::from(i) / 10.);
            let u = v + h.impulse / mass;
            let w = spin + inertia.inverse() * (at - center).cross(h.impulse);
            let extra = energy(u, w) - energy(h.velocity, h.angular_velocity);
            assert!((extra - energy(u - h.velocity, w - h.angular_velocity)).abs() < 1e-12);
        }
        assert!(matches!(
            segment
                .maximum_dissipation_stick(center, v, spin, mass, inertia, 0.001)
                .unwrap(),
            StickingLine::OutsideFriction
        ));
    }
}
