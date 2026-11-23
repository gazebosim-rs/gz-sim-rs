// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the already-proved ordered impacts with their fixed-pose impulse map.
//! Track the two normal closing speeds separately from the full twist.
use super::*;
fn number(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn data(v: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
impl Local<'_> {
    pub(super) fn track(&self, g: Geometry, p: &Value) -> Option<([f64; 2], usize)> {
        let mut events = vec![];
        self.frozen_with(g, |site, j, u, w| events.push((site, j, u, w)))?;
        let totals = [0, 1].map(|k| data(&p["frozen"]["impulses"][k]));
        let c = number(&p["incoming_speed"]);
        let sum = number(&p["trial_impulse_limit_ns"]);
        let horizon = number(&p["remaining_time_cap_s"]);
        let cap = number(&p["acceleration_cap"]);
        let ratio = number(&p["interarrival_ratio_cap"]);
        let mut flight = number(&p["initial_time_cap_s"]);
        let rho = number(&p["normal_speed_pair_ratio_cap"]);
        let eta = number(&p["relative_acceleration"]);
        let gamma = [0, 1].map(|k| number(&p["closure_ratio_upper"][k]) + ratio * eta);
        let diag = [0, 1].map(|k| number(&p["normal_coefficients"][k][k][0]));
        let full = [0, 1].map(|k| (1. + self.mu[k].powi(2)).sqrt());
        let rr = g.arm.map(|r| r.length());
        let max_r = rr[0].max(rr[1]);
        let eps = self.uncertainty;
        let min_i = self.r.body.lower;
        let maxw = (self.x.l.length() + eps.momentum + max_r * sum) / min_i;
        let ang = eps.angle + maxw * horizon + 256. * f64::EPSILON;
        let initial_v = eps.velocity + self.r.gravity.length() * horizon;
        let slip_min = [0, 1].map(|k| {
            let u = g.u[k];
            let tangent = u - g.n[k] * u.dot(g.n[k]);
            let du = initial_v
                + sum / M
                + rr[k] * ((eps.momentum + max_r * sum) / min_i + 3. * maxw * ang);
            tangent.length() - du - 1024. * f64::EPSILON * (1. + u.length())
        });
        if (0..2).any(|k| self.mu[k] > 0. && slip_min[k] <= 0.) {
            return None;
        }
        let round = 2048.
            * f64::EPSILON
            * (1. + g.u[0].length() + g.u[1].length() + sum * (1. / M + max_r * max_r / min_i));
        let mut dv = initial_v;
        let mut dl = eps.momentum;
        let mut dc = eps.velocity(self.r, self.x, rr[1 - self.fired]) + cap * flight + round;
        let mut upper = c + dc;
        let mut measured = [DVec3::ZERO; 2];
        let mut err = [0.; 2];
        let mut best = [f64::INFINITY; 2];
        let mut best_step = 0;
        for (idx, (k, j, _, post)) in events.into_iter().enumerate() {
            let next = 1 - k;
            let n = g.n[k];
            let jn = j.dot(n);
            if jn <= 0. {
                break;
            }
            let slope = j / jn - n;
            let slip = post - n * post.dot(n);
            let kn = 1. / M + rr[k] * rr[k] / min_i;
            let dr = 4. * rr[k] * rr[k] * ang / min_i + 256. * f64::EPSILON * kn;
            let cross = g.rows[k][k];
            let nt = (cross - n * cross.dot(n)).length();
            let du = dv + rr[k] * (dl / min_i + 3. * maxw * ang) + round;
            let angle_gain = if self.mu[k] == 0. {
                0.
            } else {
                self.mu[k] / (slip_min[k] * slip.length()).sqrt()
            };
            let slope_def = slope.distance(-self.mu[k] * slip.normalize_or_zero())
                + 256. * f64::EPSILON * (1. + self.mu[k]);
            let base = (dc + post.dot(n).abs() + round + jn * dr * full[k]) / diag[k];
            let weight = jn * (1. + full[k] * nt / diag[k]);
            let feedback = weight * angle_gain * kn;
            let dj = if feedback < 0.98 {
                (base * full[k] + weight * (slope_def + angle_gain * (du + dr * j.length())))
                    / (1. - feedback)
                    + round / diag[k]
            } else {
                // A frozen pre-impact tangent can be close to zero even when
                // the actual branch remains sliding. The global disk response
                // bound handles this case without normalizing that velocity.
                let row = |a: DVec3, b: DVec3| {
                    a.dot(
                        b / M
                            + self
                                .r
                                .body
                                .omega(self.x.q, g.arm[k].cross(b))
                                .cross(g.arm[k]),
                    )
                };
                let z = n;
                let x = self.planes[k].frame * DVec3::X;
                let y = z.cross(x);
                let basis = [x, y, z];
                let matrix = basis.map(|a| basis.map(|b| row(a, b)));
                gz_physics::PointImpactSensitivity::new(matrix, self.mu[k])?.impulse_error(
                    j.length(),
                    du + round,
                    dr,
                )?
            };
            if !dj.is_finite() {
                break;
            }
            let ds = (slope_def + angle_gain * (du + dr * j.length() + kn * dj))
                .min(2. * self.mu[k] + slope_def);
            let dn = dj.min(base + jn * nt * ds / diag[k]);
            let w = g.rows[next][k];
            let wtan = (w - n * w.dot(n)).length();
            let coupling = -number(&p["normal_coefficients"][next][k][0]);
            let wk = 4. * rr[0] * rr[1] * ang / min_i + 256. * f64::EPSILON * (1. + w.length());
            dc = coupling * dn
                + jn * (wk * full[k] + wtan * ds)
                + cap * flight * (1. + ratio)
                + round;
            measured[k] += j;
            err[k] += dj;
            dv += dj / M + round;
            dl += rr[k] * (dj + ang * (j.length() + dj)) + round * rr[k];
            upper *= gamma[k];
            flight *= ratio;
            let mut bound = [0.; 2];
            // Bound the total actual impulse in all following odd/even events
            // directly using the previously certified two-step contraction.
            for side in 0..2 {
                let next_amp = if side == next {
                    upper
                } else {
                    upper * gamma[next]
                };
                let remaining = next_amp * full[side] / (diag[side] * (1. - rho));
                bound[side] = err[side]
                    + remaining
                    + totals[side].distance(measured[side])
                    + round / diag[side];
            }
            if bound[0].max(bound[1]) < best[0].max(best[1]) {
                best = bound;
                best_step = idx + 1;
            }
            if bound[0].max(bound[1]) < 1e-10 {
                break;
            }
        }
        (best_step > 0).then_some((best, best_step))
    }
}
