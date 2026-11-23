// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A certified kinetic Coulomb branch for two coplanar translating supports.
use super::*;
#[derive(Clone, Copy, Debug)]
pub struct UniformSlidingStep {
    pub shared: SharedFaceStep,
    pub tangent_per_normal: DVec3,
    pub slip_start: [f64; 2],
    pub slip_end: [f64; 2],
    pub dissipated_energy: f64,
    /// Work done by the tangential forces at the pads' prescribed velocities
    /// in the common translating reference frame.
    pub support_tangent_work: f64,
}
impl FrictionlessFaceSupport {
    fn monotone_kinetic_mobility(self, q: DQuat, n: DVec3, k: DVec3, depth: f64) -> bool {
        // In coordinates (normal force, normal torque about e1,e2), common
        // kinetic shear determines all remaining force and torque components.
        // Strong monotonicity of this three-dimensional map makes the total
        // pressure and its first moments unique across unilateral solutions.
        let [x, y] = self.pad_axes;
        let rot = DMat3::from_quat(q);
        let inv = rot * self.inertia_body.inverse() * rot.transpose();
        let kx = k.dot(x);
        let ky = k.dot(y);
        let torque = [depth * (x * ky - y * kx), x - n * kx, y - n * ky];
        let cols: [DVec3; 3] = std::array::from_fn(|j| {
            let a = inv * torque[j];
            DVec3::new(if j == 0 { 1. / self.mass } else { 0. }, a.dot(x), a.dot(y))
        });
        let l = DMat3::from_cols(cols[0], cols[1], cols[2]);
        let m = (l + l.transpose()) * 0.5;
        let diagonal = DVec3::new(m.x_axis.x, m.y_axis.y, m.z_axis.z);
        if !m.is_finite() || diagonal.min_element() <= 0. {
            return false;
        }
        let scale = DMat3::from_diagonal(DVec3::new(
            1. / diagonal.x.sqrt(),
            1. / diagonal.y.sqrt(),
            1. / diagonal.z.sqrt(),
        ));
        let symmetric = scale * m * scale;
        let tol = 1024. * f64::EPSILON;
        symmetric.x_axis.x * symmetric.y_axis.y - symmetric.x_axis.y * symmetric.x_axis.y > tol
            && symmetric.determinant() > tol
    }
    /// Advance an initially nonspinning sole if both Coulomb tractions have
    /// the same constant world direction and magnitude throughout the step.
    /// A stopping port, incompatible motion, or a nonmonotone contact operator
    /// delegates to regular native contact.
    pub fn advance_uniform_slide(
        self,
        start: EdgeSupportState,
        pads: [CoplanarPad; 2],
        friction: [f64; 2],
        seconds: f64,
        preferred: Option<f64>,
    ) -> Result<Option<UniformSlidingStep>, PhysicsError> {
        self.kinetic_interval(start, pads, friction, seconds, preferred, false)
    }
    fn kinetic_interval(
        self,
        start: EdgeSupportState,
        pads: [CoplanarPad; 2],
        friction: [f64; 2],
        seconds: f64,
        preferred: Option<f64>,
        arrest_at_end: bool,
    ) -> Result<Option<UniformSlidingStep>, PhysicsError> {
        if friction.iter().any(|v| !v.is_finite() || *v < 0.)
            || preferred.is_some_and(|v| !v.is_finite() || !(0. ..=1.).contains(&v))
            || !start.angular_velocity.is_finite()
            || !seconds.is_finite()
            || seconds <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(mut bound) = self.bind_coplanar(start, pads)? else {
            return Ok(None);
        };
        if friction[0] != friction[1] || friction[0] == 0. || start.angular_velocity != DVec3::ZERO
        {
            return Ok(None);
        }
        let n = bound.flat.normal;
        let rel = pads.map(|p| bound.flat.pad_point(start.velocity) - p.velocity);
        let speeds = rel.map(DVec2::length);
        let eps = 256. * f64::EPSILON;
        let vt = eps
            * (1.
                + start.velocity.length()
                + pads[0].velocity.length()
                + pads[1].velocity.length());
        if speeds
            .into_iter()
            .any(|v| v == 0. || !v.is_finite() || (!arrest_at_end && v <= vt))
        {
            return Ok(None);
        }
        let axis = rel[0] / speeds[0];
        if axis.dot(rel[1]) <= 0.
            || axis.perp_dot(rel[1]).abs() > vt.min(1e-12 * speeds[1]).max(eps * speeds[1])
        {
            return Ok(None);
        }
        let gravity = bound.flat.pad_point(self.gravity);
        if axis.perp_dot(gravity).abs() > eps * (1. + self.gravity.length()) {
            return Ok(None);
        }
        let k = -(self.pad_axes[0] * axis.x + self.pad_axes[1] * axis.y) * friction[0];
        let acceleration = self.gravity + bound.flat.normal_force / self.mass * k;
        let slope = axis.dot(bound.flat.pad_point(acceleration));
        let mut end = speeds.map(|v| v + slope * seconds);
        if arrest_at_end {
            if end.iter().any(|v| v.abs() > vt) {
                return Ok(None);
            }
            end = [0.; 2];
        } else if end.iter().any(|v| *v <= vt) {
            return Ok(None);
        }
        let depth = (start.orientation * self.corners[0]).dot(-n);
        if depth <= 0. || !self.monotone_kinetic_mobility(start.orientation, n, k, depth) {
            return Ok(None);
        }
        let offset = -depth * k;
        let pressure = bound.flat.tangent(start.center) + offset;
        let arm = start.orientation.conjugate() * (pressure - start.center) - self.corners[0];
        let (axes, size) = bound.axes();
        let material = DVec2::new(arm.dot(axes[0]), arm.dot(axes[1]));
        let tol = bound.flat.tolerance;
        if material.min_element() <= tol || (size - material).min_element() <= tol {
            return Ok(None);
        }
        bound.flat.center_offset = offset;
        bound.flat.normalized_center = 2. * material / size - DVec2::ONE;
        bound.flat.body.gravity = acceleration;
        let Some(shared) = bound.advance(seconds, preferred)? else {
            return Ok(None);
        };
        let mut dissipated = 0.;
        let mut applied = 0.;
        for j in 0..2 {
            let impulse = shared.sites[j].selected_normal_impulse;
            dissipated += friction[0] * impulse * 0.5 * (speeds[j] + end[j]);
            applied += impulse * bound.flat.pad_point(k).dot(pads[j].velocity);
        }
        Ok(Some(UniformSlidingStep {
            shared,
            tangent_per_normal: k,
            slip_start: speeds,
            slip_end: end,
            dissipated_energy: dissipated,
            support_tangent_work: applied,
        }))
    }
}

/// Exact public-interval solution when a sole reaches the common belt velocity.
/// Physical phase lengths remain floating-point when the stop is between
/// nanoseconds of the actuator clock. At most two nonempty phases are present.
#[derive(Clone, Copy, Debug)]
pub struct ComovingSupportStop {
    pub stop_seconds: f64,
    pub sliding: Option<UniformSlidingStep>,
    pub resting: Option<SharedFaceStep>,
}
impl FrictionlessFaceSupport {
    /// In the common translating frame and under strictly normal gravity, the
    /// arrested state has zero relative kinetic energy. If the projected COM is
    /// in the support hull interior then every small tilt must lift the COM.
    /// A passive dry contact can consequently maintain rest with zero traction.
    pub fn advance_comoving_stop(
        self,
        start: EdgeSupportState,
        pads: [CoplanarPad; 2],
        friction: [f64; 2],
        seconds: f64,
        preferred: Option<f64>,
    ) -> Result<Option<ComovingSupportStop>, PhysicsError> {
        if !seconds.is_finite()
            || seconds <= 0.
            || friction.iter().any(|v| !v.is_finite() || *v < 0.)
            || preferred.is_some_and(|v| !v.is_finite() || !(0.0..=1.).contains(&v))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(bound) = self.bind_coplanar(start, pads)? else {
            return Ok(None);
        };
        let n = bound.flat.normal;
        let g = self.gravity.dot(n);
        if start.angular_velocity != DVec3::ZERO
            || pads[0].velocity != pads[1].velocity
            || friction[0] != friction[1]
            || friction[0] == 0.
            || bound.flat.pad_point(self.gravity).length()
                > 256. * f64::EPSILON * self.gravity.length()
        {
            return Ok(None);
        }
        let speed = (bound.flat.pad_point(start.velocity) - pads[0].velocity).length();
        let stop = speed / (-g * friction[0]);
        if !stop.is_finite() || stop > seconds {
            return Ok(None);
        }
        let model = Self {
            gravity: n * g,
            ..self
        };
        let sliding = if stop > 0. {
            let Some(mut s) =
                model.kinetic_interval(start, pads, friction, stop, preferred, true)?
            else {
                return Ok(None);
            };
            s.shared.state.velocity =
                self.pad_axes[0] * pads[0].velocity.x + self.pad_axes[1] * pads[0].velocity.y;
            Some(s)
        } else {
            None
        };
        let stopped = sliding.map_or(start, |s| s.shared.state);
        let pads = pads.map(|p| CoplanarPad {
            center: p.center + p.velocity * stop,
            ..p
        });
        let remaining = seconds - stop;
        let resting = if remaining > 0. {
            let rest = if stop == 0. {
                let mut b = bound;
                b.flat.body.gravity = model.gravity;
                b
            } else {
                let Some(b) = model.bind_coplanar(stopped, pads)? else {
                    return Ok(None);
                };
                b
            };
            let Some(value) = rest.stationary(
                remaining,
                sliding.map(|s| s.shared.selected_fraction_b).or(preferred),
            )?
            else {
                return Ok(None);
            };
            Some(value)
        } else {
            None
        };
        Ok(Some(ComovingSupportStop {
            stop_seconds: stop,
            sliding,
            resting,
        }))
    }
}
