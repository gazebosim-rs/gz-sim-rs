// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Interval bounds for the normal gap of a torque-free rotating cuboid vertex.
use super::*;

#[derive(Clone, Copy)]
pub(super) struct AngularJet {
    pub alpha: Vector,
    pub beta: Vector,
    /// Uniform bound on the fourth derivative of a rotating unit vector.
    pub snap: f64,
}

#[derive(Clone, Copy)]
pub(super) struct Height {
    gap: f64,
    speed: f64,
    acceleration: f64,
    jerk: f64,
    snap: f64,
    epsilon: f64,
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Crossing {
    Clear,
    Contact,
    Root {
        lo: f64,
        hi: f64,
        gap_error: f64,
    },
    /// A crossing might lie on either side of the inspected horizon.
    Horizon {
        lo: f64,
    },
    Uncertain,
}

pub(super) struct MovingVertex {
    pub point: Vector,
    pub com: Vector,
    pub v: Vector,
    pub gravity: Vector,
    pub spin: Vector,
}
impl AngularJet {
    /// Keep a finite top face when a radius-times-speed estimate reaches an edge.
    /// Bound each signed lateral coordinate through the same free motion as the normal gap.
    pub(super) fn over_pad(self, cut: MovingVertex, pad: Pose, extent: Vector, h: f64) -> bool {
        let local = pad.inverse_transform_point(cut.point);
        let scale = cut.point.length() + cut.com.length() + pad.translation.length();
        for (axis, size, coordinate) in [
            (Vector::X, extent.x, local.x),
            (Vector::Y, extent.y, local.y),
        ] {
            for sign in [-1., 1.] {
                let n = pad.rotation * (axis * -sign);
                if self
                    .height(
                        cut.point - cut.com,
                        n,
                        cut.spin,
                        (cut.v, cut.gravity),
                        size - sign * coordinate,
                        scale,
                    )
                    .released_span(h, cut.gravity.dot(n))
                    .is_none()
                {
                    return false;
                }
            }
        }
        true
    }
    pub fn height(
        self,
        arm: Vector,
        normal: Vector,
        spin: Vector,
        linear: (Vector, Vector),
        gap: f64,
        position_scale: f64,
    ) -> Height {
        let (v, gravity) = linear;
        let wr = spin.cross(arm);
        let ar = self.alpha.cross(arm);
        Height {
            gap,
            speed: (v + wr).dot(normal),
            acceleration: (gravity + ar + spin.cross(wr)).dot(normal),
            jerk: (self.beta.cross(arm)
                + self.alpha.cross(wr) * 2.
                + spin.cross(ar + spin.cross(wr)))
            .dot(normal),
            snap: arm.length() * self.snap,
            epsilon: 512. * f64::EPSILON * (1. + position_scale + arm.length()),
        }
    }
}

fn quadratic_extrema(v: f64, a: f64, b: f64, h: f64) -> (f64, f64) {
    let end = v + h * (a + h * b);
    let (mut lo, mut hi) = (v.min(end), v.max(end));
    if b != 0. {
        let t = -0.5 * a / b;
        if t > 0. && t < h {
            let at = v + t * (a + t * b);
            lo = lo.min(at);
            hi = hi.max(at);
        }
    }
    (lo, hi)
}

impl Height {
    fn polynomial(self, t: f64) -> f64 {
        self.gap + t * (self.speed + t * (0.5 * self.acceleration + t * self.jerk / 6.))
    }
    fn remainder(self, t: f64) -> f64 {
        self.epsilon + self.snap * t.powi(4) / 24.
    }
    fn zero(self, h: f64, sign: f64) -> (f64, f64) {
        let (mut lo, mut hi) = (0., h);
        for _ in 0..34 {
            let mid = (lo + hi) * 0.5;
            if self.polynomial(mid) + sign * self.remainder(mid) > 0. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo, hi)
    }
    /// Maximum displacement along this normal in one clock quantum near any
    /// point of the interval. The angular derivative bounds apply to h+quantum.
    pub(super) fn clock_margin(self, h: f64, quantum: f64) -> f64 {
        let t = h + quantum;
        quantum
            * (self.speed.abs()
                + t * (self.acceleration.abs() + t * (0.5 * self.jerk.abs() + t * self.snap / 6.)))
    }
    /// Lower bound over the *entire* proposed interval used by the pair filter.
    /// The least gap is at the terminal boundary for monotone approach.
    pub fn strictly_separated(self, h: f64) -> bool {
        let (vmin, vmax) = quadratic_extrema(self.speed, self.acceleration, 0.5 * self.jerk, h);
        let margin = self.snap * h.powi(3) / 6.;
        if ![
            self.gap,
            self.speed,
            self.acceleration,
            self.jerk,
            self.epsilon,
            margin,
            h,
        ]
        .iter()
        .all(|x| x.is_finite())
            || h <= 0.
        {
            return false;
        }
        let least = if vmax + margin <= 0. {
            self.polynomial(h) - self.remainder(h)
        } else {
            self.gap + (vmin - margin).min(0.) * h - self.epsilon
        };
        least.is_finite() && least > 0.
    }
    /// Positive lower bound on approach speed throughout a proposed root bracket.
    pub(super) fn closing_speed(self, h: f64) -> Option<f64> {
        let (_, vmax) = quadratic_extrema(self.speed, self.acceleration, 0.5 * self.jerk, h);
        let closing = -vmax - self.snap * h.powi(3) / 6.;
        (closing.is_finite() && closing > 0.05).then_some(closing)
    }
    /// Strict post-impact separation from tangency with zero normal speed.
    pub(super) fn outward_acceleration(self, h: f64) -> Option<f64> {
        let lower = self.acceleration - self.jerk.abs() * h - 0.5 * self.snap * h * h;
        let epsilon = 1024.
            * f64::EPSILON
            * (1. + self.acceleration.abs() + self.jerk.abs() * h + self.snap * h * h);
        (lower.is_finite() && lower > epsilon).then_some(lower)
    }
    /// Lower bound on every gap in a released native interval. Include the
    /// displacement difference between ballistic and semi-implicit uniform
    /// gravity. Bisect only the inconclusive proof, without adding native work.
    pub(super) fn released_span(mut self, h: f64, normal_gravity: f64) -> Option<(f64, u16)> {
        self.acceleration -= normal_gravity.abs();
        if self.gap <= self.epsilon
            || ![
                self.gap,
                self.acceleration,
                self.speed,
                self.snap,
                self.jerk,
                h,
            ]
            .iter()
            .all(|x| x.is_finite())
        {
            return None;
        }
        for n in [1_u16, 2, 4, 8, 16] {
            let mut minimum = f64::INFINITY;
            for index in 0..n {
                let a = f64::from(index) * h / f64::from(n);
                let b = f64::from(index + 1) * h / f64::from(n);
                let span = b - a;
                let v = self.speed + a * (self.acceleration + 0.5 * a * self.jerk);
                let (lo, hi) =
                    quadratic_extrema(v, self.acceleration + a * self.jerk, 0.5 * self.jerk, span);
                let rate_error = self.snap * b.powi(3) / 6.;
                // Intersect forward and backward linear enclosures throughout this span.
                let lower = (self.polynomial(a) - self.remainder(a)
                    + (lo - rate_error).min(0.) * span)
                    .max(self.polynomial(b) - self.remainder(b) - (hi + rate_error).max(0.) * span);
                if !lower.is_finite() || lower <= 0. {
                    minimum = 0.;
                    break;
                }
                minimum = minimum.min(lower);
            }
            if minimum > 0. {
                return Some((minimum, n));
            }
        }
        None
    }
    /// Independently recheck the named corner from the actual post-approach cut.
    /// No positive lower-gap threshold is used at this second measurement.
    pub(super) fn arrives_by(self, limit_ns: u64) -> Option<u64> {
        let h = limit_ns as f64 * 1e-9;
        let (_, vmax) = quadratic_extrema(self.speed, self.acceleration, 0.5 * self.jerk, h);
        let uncertainty = self.remainder(h);
        let endpoint = self.polynomial(h) + uncertainty;
        if !endpoint.is_finite()
            || !vmax.is_finite()
            || !uncertainty.is_finite()
            || self.gap < -self.epsilon
            || self.gap.is_nan()
            || vmax + self.snap * h.powi(3) / 6. >= -0.01
            || endpoint >= 0.
        {
            return None;
        }
        Some((self.zero(h, 1.).1 * 1e9).ceil() as u64)
    }
    pub fn classify(self, h: f64) -> Crossing {
        use Crossing::*;
        let remainder = self.remainder(h);
        let rate_margin = self.snap * h.powi(3) / 6.;
        let (vmin, vmax) = quadratic_extrema(self.speed, self.acceleration, self.jerk * 0.5, h);
        if ![
            self.gap,
            self.speed,
            self.acceleration,
            self.jerk,
            remainder,
            rate_margin,
            h,
        ]
        .iter()
        .all(|x| x.is_finite())
            || h <= 0.
        {
            return Uncertain;
        }
        // A positive lower bound over the entire interval proves no impact,
        // including when a receding vertex reverses its normal velocity.
        if self.gap > self.epsilon
            && (vmin >= rate_margin || self.gap + (vmin - rate_margin).min(0.) * h > self.epsilon)
        {
            return Clear;
        }
        if self.gap < -3e-4 {
            return Uncertain;
        }
        if vmax + rate_margin >= -0.01 {
            // A departing vertex already on the plane cannot be the next approaching one.
            if vmin - rate_margin > 0. && self.gap >= -self.epsilon {
                return Clear;
            }
            return Uncertain;
        }
        if self.gap <= 2e-8 {
            return Contact;
        }
        if self.polynomial(h) - remainder > 0. {
            return Clear;
        }
        let lo = self.zero(h, -1.).0;
        if self.polynomial(h) + remainder > 0. {
            return Horizon { lo };
        }
        let hi = self.zero(h, 1.).1;
        Root {
            lo,
            hi,
            gap_error: self.remainder(hi),
        }
    }
}

/// Compare all four vertices before claiming a first corner. A horizon that
/// straddles zero can exclude an earlier event but cannot establish an impact.
pub(super) fn first(
    corners: [Crossing; 4],
    share: u64,
    elapsed: u64,
    receipt: &mut BoxTimingReceipt,
) -> Result<u64, BoxTimingDecline> {
    use Crossing::*;
    let mut earliest = None;
    for (index, c) in corners.iter().enumerate() {
        if let Root { hi, .. } = *c
            && earliest.is_none_or(|(_, end)| hi < end)
        {
            earliest = Some((index, hi));
        }
    }
    if corners.iter().any(|c| matches!(c, Uncertain)) {
        return Err(BoxTimingDecline::AngularTrajectoryUncertain);
    }
    let touching = corners.iter().filter(|c| matches!(c, Contact)).count();
    if touching > 1 {
        return Err(BoxTimingDecline::AmbiguousCornerTiming);
    }
    if touching == 1 {
        return Ok(share.clamp(1, 4_000));
    }
    let Some((index, hi)) = earliest else {
        if corners.iter().any(|c| matches!(c, Horizon { .. })) {
            receipt.uncertain_horizons += 1;
        }
        return Ok(share);
    };
    if corners
        .iter()
        .enumerate()
        .any(|(i, c)| i != index && matches!(c, Root {lo,..} | Horizon {lo} if *lo <= hi))
    {
        return Err(BoxTimingDecline::AmbiguousCornerTiming);
    }
    let Root { lo, gap_error, .. } = corners[index] else {
        unreachable!()
    };
    let low_ns = (lo * 1e9).floor() as u64;
    let high_ns = (hi * 1e9).ceil() as u64;
    receipt.max_gap_uncertainty_nm = receipt
        .max_gap_uncertainty_nm
        .max((gap_error * 1e9).ceil() as u64);
    receipt.max_time_uncertainty_ns = receipt.max_time_uncertainty_ns.max(high_ns - low_ns);
    if gap_error > 0.25e-6 || hi - lo > 0.5e-6 {
        return Err(BoxTimingDecline::AngularTrajectoryUncertain);
    }
    receipt.crossings.push(BoxCrossing {
        corner: index as u8,
        measured_ns: elapsed,
        earliest_ns: elapsed + low_ns,
        latest_ns: elapsed + high_ns,
    });
    Ok(low_ns.max(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn linear(gap: f64, speed: f64) -> Height {
        Height {
            gap,
            speed,
            acceleration: 0.,
            jerk: 0.,
            snap: 0.,
            epsilon: 1e-13,
        }
    }
    #[test]
    fn a_straddled_horizon_and_overlapping_vertex_brackets_do_not_certify_an_impact() {
        let h = 125e-6;
        let root = linear(h, -1.).classify(h);
        assert!(matches!(root, Crossing::Horizon { .. }));
        let mut receipt = BoxTimingReceipt::default();
        assert_eq!(
            first(
                [root, Crossing::Clear, Crossing::Clear, Crossing::Clear],
                62_500,
                0,
                &mut receipt
            ),
            Ok(62_500)
        );
        assert_eq!(receipt.uncertain_horizons, 1);
        assert!(receipt.crossings.is_empty());
        let root = linear(100e-6, -1.).classify(h);
        assert_eq!(
            first(
                [root, root, Crossing::Clear, Crossing::Clear],
                62_500,
                0,
                &mut receipt
            ),
            Err(BoxTimingDecline::AmbiguousCornerTiming)
        );
        assert!(receipt.crossings.is_empty());
    }
    #[test]
    fn grazing_acceleration_reversals_and_nonfinite_motion_decline() {
        let mut g = linear(0.000_000_061, -0.002);
        g.acceleration = 4.;
        assert!(matches!(g.classify(0.0005), Crossing::Uncertain));
        g.gap = 0.02;
        assert!(matches!(g.classify(0.0005), Crossing::Clear));
        g.snap = f64::INFINITY;
        assert!(matches!(g.classify(0.0005), Crossing::Uncertain));
    }

    #[test]
    fn independent_euler_rk_corners_and_crossings_respect_the_whole_envelope() {
        // Independent attitude solution at held fixed world angular momentum.
        fn advance(tensor: Matrix, q: Rotation, l: Vector, h: f64) -> Rotation {
            let inv = tensor.inverse();
            let rate = |q: Rotation| {
                let q = q.normalize();
                let w = q * (inv * (q.inverse() * l));
                (Rotation::from_xyzw(w.x, w.y, w.z, 0.) * q) * 0.5
            };
            let a = rate(q);
            let b = rate(q + a * (0.5 * h));
            let c = rate(q + b * (0.5 * h));
            let d = rate(q + c * h);
            (q + (a + (b + c) * 2. + d) * (h / 6.)).normalize()
        }
        let mut enclosed = 0;
        let mut roots = 0;
        for k in 0..96 {
            let moments = [
                Vector::new(0.021, 0.025, 0.027),
                Vector::new(0.003, 0.030, 0.031),
                Vector::new(0.0014, 0.044, 0.045),
            ][k % 3];
            let kf = k as f64;
            let local =
                Rotation::from_scaled_axis(Vector::new(0.83, (kf * 0.2).sin(), (kf * 0.9).cos()));
            let a = Matrix::from_quat(local);
            let tensor = a * Matrix::from_diagonal(moments) * a.transpose();
            let q = Rotation::from_scaled_axis(Vector::new(0.24, kf * 0.07, -0.44));
            let aq = Matrix::from_quat(q);
            let world = aq * tensor * aq.transpose();
            let bound = InertiaBound::from_world(
                [world.x_axis, world.y_axis, world.z_axis],
                q,
                moments.min_element(),
                moments.max_element(),
            )
            .unwrap()
            .unwrap();
            let local_spin = Vector::new((kf * 0.4).cos(), (kf * 0.91).sin(), 0.5).normalize()
                * [0.0, 8., 45., 115.][k % 4];
            let omega = q * local_spin;
            let momentum = q * (tensor * local_spin);
            let g = Vector::new(0., 0., -9.81);
            let n = Vector::new(0.03, -0.2, 1.).normalize();
            let arm = Vector::new(0.18, if k % 2 == 0 { -0.1 } else { 0.1 }, -0.13);
            let r = q * arm;
            let v = n * (-4. - omega.cross(r).dot(n)) + Vector::new(0.13, 0.01, 0.);
            let h = [40e-6, 125e-6, 500e-6][(k / 4) % 3];
            let initial = 4. * h * 0.6;
            let height = bound
                .jet(q, omega, h)
                .height(r, n, omega, (v, g), initial, 2.);
            let exact =
                |qt: Rotation, t: f64| initial + (qt * arm - r + v * t + 0.5 * g * t * t).dot(n);
            let mut state = q;
            let mut observed = None;
            let step = h / 128.;
            for j in 1..=128 {
                let t = j as f64 * step;
                let next = advance(tensor, state, momentum, step);
                let gap = exact(next, t);
                assert!(
                    (gap - height.polynomial(t)).abs() <= height.remainder(t),
                    "case {k} at {t}"
                );
                if observed.is_none() && gap < 0. {
                    let (mut lo, mut hi) = (0., step);
                    let start = (j - 1) as f64 * step;
                    for _ in 0..36 {
                        let mid = (lo + hi) * 0.5;
                        if exact(advance(tensor, state, momentum, mid), start + mid) > 0. {
                            lo = mid
                        } else {
                            hi = mid
                        }
                    }
                    observed = Some(start + (lo + hi) * 0.5);
                }
                state = next;
                enclosed += 1;
            }
            if let Crossing::Root { lo, hi, .. } = height.classify(h) {
                let crossing = observed.expect("verified sign change");
                assert!(
                    lo <= crossing && crossing <= hi,
                    "{k}: {lo} .. {crossing} .. {hi}"
                );
                roots += 1;
            }
        }
        assert_eq!(enclosed, 96 * 128);
        assert!(roots >= 80, "{roots}");
    }
}
