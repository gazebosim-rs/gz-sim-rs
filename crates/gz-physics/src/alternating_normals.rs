// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Normal impulse envelopes for alternating perfectly inelastic contacts.
/// Sufficient bounds on a two-site impact sequence, beginning immediately
/// after the previous site acquired zero normal velocity. The other site is
/// closing and separated. Indices identify the two real material contacts.
///
/// Callers must enclose the supplied response coefficients and accelerations
/// over all states with at most `impulse_budget` total world impulse and the
/// supplied time period. They must independently verify spatial contact
/// isolation and both finite footprints throughout this period.
#[derive(Clone, Copy, Debug)]
pub struct AlternatingNormalInput {
    pub next: usize,
    /// Positive clearance of the closing site, in metres, upper bound.
    pub clearance_upper: f64,
    /// Positive closing speed along the next normal, lower/upper, in m/s.
    pub closing_speed: [f64; 2],
    /// Each entry bounds the normal velocity change at site i from a unit
    /// normal impulse at site j, including any coupled tangential reaction.
    pub response: [[[f64; 2]; 2]; 2],
    /// Both points have strictly positive free normal acceleration.
    pub outward_min: [f64; 2],
    /// Upper bound on the magnitude of either point's normal acceleration.
    pub acceleration_max: f64,
    /// Bound on total scalar/world impulse ratio for each site. E.g.
    /// sqrt(1+mu^2) bounds world impulse norm per normal impulse.
    pub world_per_normal: [f64; 2],
    pub impulse_budget: f64,
    /// Maximum admissible time from the input cut to accumulation, seconds.
    pub time_budget: f64,
    /// Absolute normal velocity residual, in addition to the closing-speed
    /// interval, e.g. the arithmetic uncertainty in evaluating the geometry.
    pub normal_roundoff: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct AlternatingNormalLimit {
    pub initial_time_cap: f64,
    pub time_cap: f64,
    pub time_ratio_cap: f64,
    pub transfer_lower: [f64; 2],
    pub transfer_upper: [f64; 2],
    pub acceleration_ratio: f64,
    pub pair_speed_ratio_cap: f64,
    pub world_impulse_bound: f64,
    pub normal_impulse_bounds: [[f64; 2]; 2],
}
impl AlternatingNormalInput {
    /// Return the finite limiting time and both normal impulse sums. The free
    /// velocities tend to zero along both normals, and positive free normal
    /// acceleration permits them to separate immediately at the limit.
    pub fn bound(self) -> Option<AlternatingNormalLimit> {
        let x = self;
        if x.next >= 2
            || ![
                x.clearance_upper,
                x.acceleration_max,
                x.impulse_budget,
                x.time_budget,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.)
            || !x.normal_roundoff.is_finite()
            || x.normal_roundoff < 0.
            || !x
                .outward_min
                .iter()
                .all(|v| v.is_finite() && *v > 0. && *v <= x.acceleration_max)
            || !x.world_per_normal.iter().all(|v| v.is_finite() && *v >= 1.)
            || x.closing_speed[0] <= 0.
            || x.closing_speed[1] < x.closing_speed[0]
            || !x.closing_speed.iter().all(|v| v.is_finite())
            || !x
                .response
                .iter()
                .flatten()
                .all(|v| v.iter().all(|s| s.is_finite()) && v[0] <= v[1])
        {
            return None;
        }
        let h = 2. * x.clearance_upper / x.closing_speed[0];
        let t = 2. * h;
        if h <= 0. || !t.is_finite() || t > x.time_budget {
            return None;
        }
        let mut beta = [0.; 2];
        let mut gamma = [0.; 2];
        let mut norm = [0.; 2];
        for j in 0..2 {
            let [dlo, dhi] = x.response[j][j];
            let [clo, chi] = x.response[1 - j][j];
            if dlo <= 0. || chi >= 0. {
                return None;
            }
            beta[j] = -chi / dhi;
            gamma[j] = -clo / dlo;
            norm[j] = x.world_per_normal[j] / dlo;
        }
        // After an impact the other vertex is closing by at least beta*c
        // minus curvature. Its small gap grew quadratically while departing.
        // These inequalities inductively bound the next duration by q times
        // the preceding one and its arrival speed by at least beta*c/2.
        let b = beta[0].min(beta[1]);
        let q = (0.25 * b).min(0.25);
        let eta = x.acceleration_max * h / (x.closing_speed[0] - x.acceleration_max * h);
        if !eta.is_finite() || eta < 0. || eta >= 0.25 * b * q {
            return None;
        }
        let grow = gamma.map(|v| v + q * eta);
        let rho = grow[0] * grow[1];
        if rho >= 1. {
            return None;
        }
        let total = (x.closing_speed[1] + x.acceleration_max * h)
            * (norm[x.next] + grow[x.next] * norm[1 - x.next])
            / (1. - rho);
        if !total.is_finite() || total >= x.impulse_budget {
            return None;
        }
        let extra = x.normal_roundoff + x.acceleration_max * t;
        let rhs = [0, 1].map(|k| {
            if k == x.next {
                [x.closing_speed[0] - extra, x.closing_speed[1] + extra]
            } else {
                [-extra, extra]
            }
        });
        let mut bounds = [[f64::INFINITY, 0f64]; 2];
        // Integrated zero normal velocity gives two linear equations. Each
        // coefficient is an impulse-weighted average within its interval.
        // Positive determinant makes each component linear-fractional in any
        // one coefficient or RHS entry, so extrema occur at box vertices.
        for bits in 0..64 {
            let get = |v: [f64; 2], shift: usize| v[(bits >> shift) & 1usize];
            let a = get(x.response[0][0], 0);
            let b = get(x.response[0][1], 1);
            let c = get(x.response[1][0], 2);
            let d = get(x.response[1][1], 3);
            // Normalize to avoid overflow/underflow at very large/small masses.
            let scale = a.abs().max(b.abs()).max(c.abs()).max(d.abs());
            let (a, b, c, d) = (a / scale, b / scale, c / scale, d / scale);
            let area = a * d - b * c;
            if area <= 64. * f64::EPSILON * (a * d + (b * c).abs()) {
                return None;
            }
            let f = get(rhs[0], 4) / scale;
            let g = get(rhs[1], 5) / scale;
            let vals = [(d * f - b * g) / area, (a * g - c * f) / area];
            let round = 128.
                * f64::EPSILON
                * (1. + ((a * d).abs() + (b * c).abs()) / area)
                * (f.abs() + g.abs())
                * (a.abs() + b.abs() + c.abs() + d.abs())
                / area;
            if !round.is_finite() || !vals.iter().all(|v| v.is_finite()) {
                return None;
            }
            for (k, v) in vals.into_iter().enumerate() {
                bounds[k][0] = bounds[k][0].min(v - round);
                bounds[k][1] = bounds[k][1].max(v + round);
            }
        }
        for bound in &mut bounds {
            bound[0] = bound[0].max(0.);
        }
        Some(AlternatingNormalLimit {
            initial_time_cap: h,
            time_cap: t,
            time_ratio_cap: q,
            transfer_lower: beta,
            transfer_upper: gamma,
            acceleration_ratio: eta,
            pair_speed_ratio_cap: rho,
            world_impulse_bound: total,
            normal_impulse_bounds: bounds,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn input() -> AlternatingNormalInput {
        AlternatingNormalInput {
            next: 1,
            clearance_upper: 1e-8,
            closing_speed: [1.2, 1.2],
            response: [[[1.4, 1.4], [-0.45, -0.45]], [[-0.45, -0.45], [0.9, 0.9]]],
            outward_min: [0.7, 0.1],
            acceleration_max: 1.,
            world_per_normal: [1., 1.],
            impulse_budget: 3.,
            time_budget: 1e-6,
            normal_roundoff: 1e-14,
        }
    }
    #[test]
    fn constant_acceleration_alternating_events_close_both_normals() {
        let x = input();
        let p = x.bound().unwrap();
        let mut gap = x.clearance_upper;
        let mut closing = 1.2;
        let mut time = 0.;
        let mut j = [0.; 2];
        let mut site = x.next;
        let acc = [0.7, 0.1];
        let mut prev_h = f64::INFINITY;
        for _ in 0..256 {
            let other = 1 - site;
            let h = 2. * gap / (closing + (closing * closing - 2. * acc[site] * gap).sqrt());
            assert!(h <= p.time_ratio_cap * prev_h);
            prev_h = h;
            time += h;
            let impulse = (closing - acc[site] * h) / x.response[site][site][0];
            j[site] += impulse;
            closing = -x.response[other][site][0] * impulse - acc[other] * h;
            gap = 0.5 * acc[other] * h * h;
            site = other;
            if closing < 1e-16 {
                break;
            }
        }
        assert!(time <= p.time_cap && j.iter().sum::<f64>() < p.world_impulse_bound);
        for (k, x) in j.into_iter().enumerate() {
            assert!(x >= p.normal_impulse_bounds[k][0] && x <= p.normal_impulse_bounds[k][1]);
        }
    }
    #[test]
    fn rejects_open_coupling_singular_mass_inward_acceleration_and_short_budget() {
        let x = input();
        let mut y = x;
        y.response[0][1] = [0., 0.1];
        assert!(y.bound().is_none());
        let mut y = x;
        y.response[0][0] = [0., 1.4];
        assert!(y.bound().is_none());
        let mut y = x;
        y.outward_min[1] = 0.;
        assert!(y.bound().is_none());
        let mut y = x;
        y.time_budget = 1e-10;
        assert!(y.bound().is_none());
        let mut y = x;
        y.impulse_budget = 0.1;
        assert!(y.bound().is_none());
    }
}
