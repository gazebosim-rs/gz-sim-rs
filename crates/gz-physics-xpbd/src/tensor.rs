// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rotation and world-space contact response about a free rigid body’s COM.
use super::XpbdError;
use glam::{DQuat, DVec2, DVec3};

/// A native body's principal mass frame and uncoupled pose across one public tick.
/// Quaternions are xyzw; angular velocities and contact impulses are world-frame.
/// The native orientations define a shortest-arc path. Supply a tick whose
/// endpoint angular speeds imply less than a half-turn over that interval.
#[derive(Clone, Copy, Debug)]
pub struct PrincipalRigidMotion {
    pub inverse_mass: f64,
    pub inverse_principal_inertia: [f64; 3],
    /// Rotation from principal coordinates into the link's local frame.
    pub principal_orientation: [f64; 4],
    pub start_orientation: [f64; 4],
    pub end_orientation: [f64; 4],
    pub start_angular_velocity: [f64; 3],
}

#[derive(Clone)]
pub(super) struct TensorMotion {
    inverse: DVec3,
    local_axes: DQuat,
    start: DQuat,
    end: DQuat,
    initial_spin: DVec3,
    final_spin: DVec3,
    dt: f64,
    base_pose: DQuat,
    pose: DQuat,
    previous_pose: DQuat,
    base_spin: DVec3,
    added_momentum: DVec3,
    spin: DVec3,
}

impl TensorMotion {
    pub fn new(spec: PrincipalRigidMotion, final_spin: DVec3, dt: f64) -> Result<Self, XpbdError> {
        let inverse = DVec3::from(spec.inverse_principal_inertia);
        if !inverse.is_finite() || inverse.min_element() <= 0. || !inverse.recip().is_finite() {
            return Err(XpbdError::InvalidBoundary);
        }
        let moments = inverse.recip();
        let scaled = moments / moments.max_element();
        // A physical inertia tensor obeys all three triangle inequalities.
        if scaled.element_sum() < 2. - 1e-12 {
            return Err(XpbdError::InvalidBoundary);
        }
        Self::trajectory(spec, final_spin, dt)
    }

    /// The enclosing native mechanism supplies the complete path of this
    /// receiver. Zero inverse mobility keeps XPBD impulses in its load report.
    pub fn prescribed(
        start: [f64; 4],
        end: [f64; 4],
        spin: [f64; 3],
        final_spin: DVec3,
        dt: f64,
    ) -> Result<Self, XpbdError> {
        Self::trajectory(
            PrincipalRigidMotion {
                inverse_mass: 0.,
                inverse_principal_inertia: [0.; 3],
                principal_orientation: DQuat::IDENTITY.to_array(),
                start_orientation: start,
                end_orientation: end,
                start_angular_velocity: spin,
            },
            final_spin,
            dt,
        )
    }

    fn trajectory(
        spec: PrincipalRigidMotion,
        final_spin: DVec3,
        dt: f64,
    ) -> Result<Self, XpbdError> {
        let inverse = DVec3::from(spec.inverse_principal_inertia);
        let local_axes = DQuat::from_array(spec.principal_orientation);
        let start = DQuat::from_array(spec.start_orientation);
        let end = DQuat::from_array(spec.end_orientation);
        let initial_spin = DVec3::from(spec.start_angular_velocity);
        if !initial_spin.is_finite()
            || !final_spin.is_finite()
            || dt * initial_spin.length().max(final_spin.length()) >= std::f64::consts::PI
            || [local_axes, start, end]
                .iter()
                .any(|q| (q.length_squared() - 1.).abs() > 1e-10 || !q.is_finite())
        {
            return Err(XpbdError::InvalidBoundary);
        }
        Ok(Self {
            inverse,
            local_axes,
            start,
            end,
            initial_spin,
            final_spin,
            dt,
            base_pose: start,
            pose: start,
            previous_pose: start,
            base_spin: initial_spin,
            added_momentum: DVec3::ZERO,
            spin: initial_spin,
        })
    }

    fn tensor(&self, pose: DQuat, diagonal: DVec3, v: DVec3) -> DVec3 {
        let axes = pose * self.local_axes;
        axes * (diagonal * (axes.conjugate() * v))
    }
    pub fn inverse_at(&self, pose: DQuat, v: DVec3) -> DVec3 {
        self.tensor(pose, self.inverse, v)
    }
    pub fn inverse_inertia(&self, v: DVec3) -> DVec3 {
        self.tensor(self.pose, self.inverse, v)
    }
    fn spin_at(&self, pose: DQuat, base: DQuat, spin: DVec3) -> DVec3 {
        if self.added_momentum == DVec3::ZERO && pose == base {
            return spin;
        }
        let free_momentum = self.tensor(base, self.inverse.recip(), spin);
        self.tensor(pose, self.inverse, free_momentum + self.added_momentum)
    }
    pub fn can_rotate(&self) -> bool {
        self.spin != DVec3::ZERO || self.final_spin != DVec3::ZERO || self.start != self.end
    }

    pub fn spin(&self) -> DVec3 {
        self.spin
    }
    pub fn add_torque_impulse(&mut self, impulse: DVec3) {
        self.added_momentum += impulse;
        self.spin = self.spin_at(self.pose, self.base_pose, self.base_spin);
    }

    // Transport against the actual native start/end trajectory, adding the
    // world angular momentum deposited by all contacts. Use midpoint spin for
    // the correction; endpoint impulses first affect the following pose step.
    pub fn advance(&mut self, h: f64, end_time: f64) {
        self.previous_pose = self.pose;
        let fraction = (end_time / self.dt).clamp(0., 1.);
        let base = if fraction >= 1. - 4. * f64::EPSILON {
            self.end
        } else {
            self.start.slerp(self.end, fraction)
        };
        let spin = self.initial_spin.lerp(self.final_spin, fraction);
        if self.added_momentum == DVec3::ZERO && self.pose == self.base_pose {
            self.pose = base;
        } else {
            let half_base = self.base_pose.slerp(base, 0.5);
            let half_spin = self.base_spin.lerp(spin, 0.5);
            let offset = self.base_pose.conjugate() * self.pose;
            let spin_delta = self.spin() - self.base_spin;
            let half_pose =
                (DQuat::from_scaled_axis(spin_delta * (0.5 * h)) * half_base * offset).normalize();
            let mid_delta = self.spin_at(half_pose, half_base, half_spin) - half_spin;
            self.pose = (DQuat::from_scaled_axis(mid_delta * h) * base * offset).normalize();
        }
        self.base_pose = base;
        self.base_spin = spin;
        self.spin = self.spin_at(self.pose, base, spin);
    }
    pub fn orientations(&self) -> (DQuat, DQuat) {
        (self.previous_pose, self.pose)
    }
    /// Project the pose with the same world angular impulse that corrected the
    /// velocities. Re-evaluate spin from its stored world angular momentum.
    pub fn rotate_projection(&mut self, delta: DVec3) -> Result<(), XpbdError> {
        if !delta.is_finite() || delta.length() > std::f64::consts::FRAC_PI_4 {
            return Err(XpbdError::UnresolvedSweep);
        }
        self.pose = (DQuat::from_scaled_axis(delta) * self.pose).normalize();
        self.spin = self.spin_at(self.pose, self.base_pose, self.base_spin);
        Ok(())
    }
    pub fn rotation_correction(&self) -> DQuat {
        if self.pose == self.base_pose {
            DQuat::IDENTITY
        } else {
            (self.pose * self.base_pose.conjugate()).normalize()
        }
    }
    pub fn spin_correction(&self) -> DVec3 {
        self.spin() - self.base_spin
    }

    /// Solve the two tangent components jointly. With an anisotropic tensor
    /// slip direction and impulse direction can differ even in sticking contact.
    /// For sliding, the multiplier enforces the circular Coulomb impulse disk.
    pub fn friction(
        &self,
        arm: DVec3,
        normal: DVec3,
        tangent: DVec3,
        inverse_mass: f64,
        cap: f64,
    ) -> Result<DVec3, XpbdError> {
        if cap == 0. {
            return Ok(DVec3::ZERO);
        }
        let slip = tangent.length();
        let u = tangent / slip;
        let v = normal.cross(u);
        let a = arm.cross(u);
        let b = arm.cross(v);
        let aa = inverse_mass + a.dot(self.inverse_inertia(a));
        let ab = a.dot(self.inverse_inertia(b));
        let bb = inverse_mass + b.dot(self.inverse_inertia(b));
        // Scale before forming the determinant to avoid overflow in K*K.
        let scale = aa.max(bb);
        let (aa, ab, bb, rhs) = (aa / scale, ab / scale, bb / scale, -slip / scale);
        let solve = |lambda: f64| {
            let d = (aa + lambda) * (bb + lambda) - ab * ab;
            if d.is_finite() {
                DVec2::new((bb + lambda) * rhs / d, -ab * rhs / d)
            } else {
                // At tiny Coulomb radii the multiplier can swamp K. Eliminate
                // the second component without forming its squared magnitude.
                let ratio = ab / (bb + lambda);
                let first = rhs / (aa + lambda - ab * ratio);
                DVec2::new(first, -ratio * first)
            }
        };
        let mut j = solve(0.);
        if !j.is_finite() {
            return Err(XpbdError::Numerical);
        }
        // Squaring can underflow for tiny but valid Coulomb coefficients.
        let inside = |j: DVec2| j.x.hypot(j.y) <= cap;
        if !inside(j) {
            let (mut lo, mut hi) = (0., rhs.abs() / cap);
            if !hi.is_finite() {
                return Err(XpbdError::Numerical);
            }
            for _ in 0..52 {
                let mid = lo + (hi - lo) * 0.5;
                if !inside(solve(mid)) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            j = solve(hi);
        }
        Ok(j.x * u + j.y * v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tiny_positive_coulomb_radius_still_limits_the_contact_impulse() {
        let q = DQuat::IDENTITY.to_array();
        let t = TensorMotion::new(
            PrincipalRigidMotion {
                inverse_mass: 1.,
                inverse_principal_inertia: [6., 8., 10.],
                principal_orientation: q,
                start_orientation: q,
                end_orientation: q,
                start_angular_velocity: [0.; 3],
            },
            DVec3::ZERO,
            0.001,
        )
        .unwrap();
        for cap in [0., 1e-300, 1e-200, 1e-100] {
            let j = t
                .friction(DVec3::X, DVec3::X, DVec3::Y + 0.4 * DVec3::Z, 2., cap)
                .unwrap();
            let magnitude = j.y.hypot(j.z);
            assert!(
                (magnitude - cap).abs() <= cap * 1e-12,
                "cap={cap:e} |J|={magnitude:e}"
            );
        }
    }
}
