// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounds for a torque-free rigid body with a positive definite body-frame tensor.
use super::*;
#[derive(Clone, Copy)]
pub(super) struct InertiaBound {
    /// Expressed in the enrolled foot frame and fixed for this native interval.
    tensor: Matrix,
    inverse: Matrix,
    lower: f64,
    spread: f64,
}
#[derive(Clone, Copy)]
pub(super) struct RotationBound {
    pub speed: f64,
    pub acceleration: f64,
}
impl InertiaBound {
    pub(super) fn from_world(
        cols: [Vector; 3],
        foot: Rotation,
        member_lower: f64,
        member_upper: f64,
    ) -> Result<Option<Self>, BoxTimingDecline> {
        let scalar = (cols[0].x + cols[1].y + cols[2].z) / 3.;
        let axes = [Vector::X, Vector::Y, Vector::Z];
        let delta = cols
            .iter()
            .zip(axes)
            .map(|(c, a)| (*c - scalar * a).length_squared())
            .sum::<f64>()
            .sqrt();
        if !scalar.is_finite()
            || scalar <= 0.
            || !delta.is_finite()
            || !member_lower.is_finite()
            || !member_upper.is_finite()
        {
            return Err(BoxTimingDecline::InvalidInertia);
        }
        // A truly spherical aggregate has round-off from rotated member frames and parallel axes.
        if delta <= 128. * f64::EPSILON * scalar {
            return Ok(None);
        }
        // Eigenvalue bounds follow both from the Frobenius enclosure and from
        // summing each member's principal extrema and PSD parallel-axis terms.
        let roundoff = 128. * f64::EPSILON * member_upper;
        let lower = member_lower.max(scalar - delta) - roundoff;
        let upper = member_upper.min(scalar + delta) + roundoff;
        if !(lower > 0. && upper >= lower) {
            return Err(BoxTimingDecline::InvalidInertia);
        }
        let world = Matrix::from_cols(cols[0], cols[1], cols[2]);
        let rotation = Matrix::from_quat(foot);
        let tensor = rotation.transpose() * world * rotation;
        Ok(Some(Self {
            tensor,
            inverse: tensor.inverse(),
            lower,
            spread: upper - lower,
        }))
    }
    /// World-frame angular acceleration at the measured boundary and a bound on its
    /// time derivative over the entire torque-free horizon.
    pub(super) fn changing_acceleration(
        self,
        frame: Rotation,
        spin: Vector,
        motion: RotationBound,
    ) -> (Vector, f64) {
        let local = frame.inverse() * spin;
        let alpha = frame * (self.inverse * ((self.tensor * local).cross(local)));
        // α_world = R F(ω_body). Differentiate once. |DF| <= spread/I_min * |ω|.
        let beta = (1. + self.spread / self.lower) * motion.speed * motion.acceleration;
        (alpha, beta * (1. + 128. * f64::EPSILON))
    }
    /// Derivatives at the current native cut and a uniform fourth-order bound
    /// for a rotating vertex over the entire proposed horizon.
    pub(super) fn jet(self, frame: Rotation, spin: Vector, h: f64) -> taylor::AngularJet {
        let u = frame.inverse() * spin;
        let u_momentum = self.tensor * u;
        let f = self.inverse * u_momentum.cross(u);
        let df_f = self.inverse * ((self.tensor * f).cross(u) + u_momentum.cross(f));
        let motion = self.motion(u, h);
        let ratio = self.spread / self.lower;
        let a = motion.acceleration;
        let w = motion
            .speed
            .min((spin.length() + a * h) * (1. + 128. * f64::EPSILON));
        // F(u)=I^-1[(Iu)×u], ||DF|| <= ratio*w, ||D²F|| <= ratio.
        // Differentiate R*F and then R*(u×F+DF*F) to bound the next two derivatives.
        let beta = (1. + ratio) * w * a;
        let gamma = (1. + ratio).powi(2) * w * w * a + ratio * a * a;
        taylor::AngularJet {
            alpha: frame * f,
            beta: frame * (u.cross(f) + df_f),
            snap: (gamma + 4. * w * beta + 3. * a * a + 6. * w * w * a + w.powi(4))
                * (1. + 256. * f64::EPSILON),
        }
    }
    pub(super) fn motion(self, body_spin: Vector, horizon: f64) -> RotationBound {
        let momentum = self.tensor * body_spin;
        // Twice the kinetic energy and the norm of angular momentum are
        // conserved in the torque-free flight before the next contact.
        let speed = (body_spin.dot(momentum) / self.lower)
            .max(0.)
            .sqrt()
            .min(momentum.length() / self.lower)
            * (1. + 128. * f64::EPSILON);
        let ratio = self.spread / self.lower;
        let global = 0.5 * ratio * speed * speed;
        // For body-frame Euler flow F(w), |DF| <= spread * |w| / I_min.
        // Gronwall bounds |F(w(t))| by the current slope times exp(L t).
        // The near-principal case can have a much smaller certified slope.
        let current = (body_spin.cross(momentum).length()
            + 128. * f64::EPSILON * body_spin.length() * momentum.length())
            / self.lower;
        let local = current * (ratio * speed * horizon).exp();
        let acceleration = global.min(local) * (1. + 128. * f64::EPSILON);
        RotationBound {
            speed,
            acceleration,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Integrate the free Euler equations directly in body components, without
    // using the production motion bound or its world-momentum attitude path.
    fn euler(tensor: Matrix, q: Rotation, w: Vector, h: f64) -> (Rotation, Vector) {
        let inverse = tensor.inverse();
        let rate = |q: Rotation, w: Vector| {
            let pure = Rotation::from_xyzw(w.x, w.y, w.z, 0.);
            (
                (q.normalize() * pure) * 0.5,
                inverse * ((tensor * w).cross(w)),
            )
        };
        let (a, x) = rate(q, w);
        let (b, y) = rate(q + a * (0.5 * h), w + x * (0.5 * h));
        let (c, z) = rate(q + b * (0.5 * h), w + y * (0.5 * h));
        let (d, t) = rate(q + c * h, w + z * h);
        (
            (q + (a + (b + c) * 2. + d) * (h / 6.)).normalize(),
            w + (x + 2. * (y + z) + t) * (h / 6.),
        )
    }
    #[test]
    fn euler_flight_and_all_corner_heights_stay_inside_the_frozen_spin_envelope() {
        let mut cases = 0;
        for principal in [
            Vector::new(0.021, 0.025, 0.027),
            Vector::new(0.003, 0.030, 0.031),
            Vector::new(0.0014, 0.044, 0.045),
        ] {
            for f in [0.17, 0.83, 1.45] {
                let axes = Rotation::from_scaled_axis(Vector::new(f, 0.4, 0.8));
                let frame = Matrix::from_quat(axes);
                let tensor = frame * Matrix::from_diagonal(principal) * frame.transpose();
                for magnitude in [0., 0.8, 8., 40., 120.] {
                    for axis in [Vector::Y, Vector::new(0.1, 0.8, -0.3).normalize()] {
                        let w0 = axes * (axis * magnitude);
                        let start = Rotation::from_scaled_axis(Vector::new(0.23, -0.41, f));
                        let world = Matrix::from_quat(start)
                            * tensor
                            * Matrix::from_quat(start).transpose();
                        let body = InertiaBound::from_world(
                            [world.x_axis, world.y_axis, world.z_axis],
                            start,
                            principal.min_element(),
                            principal.max_element(),
                        )
                        .unwrap()
                        .unwrap();
                        let horizon = 0.0005;
                        let bound = body.motion(w0, horizon);
                        let mut q = start;
                        let mut w = w0;
                        let l0 = start * (tensor * w0);
                        let e0 = w0.dot(tensor * w0);
                        for i in 1..=128 {
                            let h = horizon / 128.;
                            (q, w) = euler(tensor, q, w, h);
                            let t = i as f64 * h;
                            let frozen = Rotation::from_scaled_axis(start * w0 * t) * start;
                            assert!(w.length() <= bound.speed + 1e-10);
                            assert!(
                                (tensor.inverse() * ((tensor * w).cross(w))).length()
                                    <= bound.acceleration + 1e-9
                            );
                            let angle = (q * frozen.inverse()).to_scaled_axis().length();
                            assert!(angle <= 0.5 * bound.acceleration * t * t + 1e-12);
                            for sign in [-1., 1.] {
                                let arm = Vector::new(0.18, sign * 0.1, -0.13);
                                assert!(
                                    (q * arm - frozen * arm).length()
                                        <= 0.5 * bound.acceleration * t * t * arm.length() + 1e-12
                                );
                            }
                            assert!((q * (tensor * w)).distance(l0) < 1e-10);
                            assert!((w.dot(tensor * w) - e0).abs() < 1e-8);
                        }
                        cases += 1;
                    }
                }
            }
        }
        assert_eq!(cases, 90);
    }
}
