// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sufficient bounds for a finite accumulation of increasingly fast alternating impacts.
//! Only the two named vertices, strict sliding (or zero friction), and positive
//! free normal acceleration on both vertices are admitted.
use super::*;
#[path = "inertia_gap_tight.rs"]
mod tight;
struct Local<'a> {
    r: &'a Physical,
    x: Flight,
    planes: [Face; 2],
    sites: [Site; 2],
    mu: [f64; 2],
    fired: usize,
    gap: Interval,
    time: f64,
    uncertainty: uncertainty::Error,
}
#[derive(Clone, Copy, Default)]
struct Coeff {
    lo: f64,
    hi: f64,
}
#[derive(Clone, Copy)]
struct Geometry {
    n: [DVec3; 2],
    arm: [DVec3; 2],
    u: [DVec3; 2],
    rows: [[DVec3; 2]; 2],
}
impl Local<'_> {
    fn geometry(&self) -> Geometry {
        let n = self.planes.map(|p| p.frame * DVec3::Z);
        let arm = self.sites.map(|s| self.x.q * vertices()[s.k]);
        let w = self.r.body.omega(self.x.q, self.x.l);
        let u = [0, 1].map(|k| self.x.v - self.planes[k].v + w.cross(arm[k]));
        let rows = [0, 1].map(|i| {
            [0, 1].map(|k| {
                n[i] / M
                    + self
                        .r
                        .body
                        .omega(self.x.q, arm[i].cross(n[i]))
                        .cross(arm[k])
            })
        });
        Geometry { n, arm, u, rows }
    }
    fn trial(&self, g: Geometry, limit: f64, c: f64, h: f64, free_after: f64) -> Option<Value> {
        let r = self.r;
        let x = self.x;
        let first = 1 - self.fired;
        let accumulation_time = 2. * h;
        let bound_time = accumulation_time + free_after;
        let max_r = g.arm.iter().map(|r| r.length()).fold(0., f64::max);
        let min_i = r.body.lower;
        let diff = (r.body.upper - min_i) / min_i;
        let w0 = (x.l.length() + self.uncertainty.momentum) / min_i;
        let wcap = w0 + max_r * limit / min_i;
        let angle = self.uncertainty.angle + wcap * bound_time + 256. * f64::EPSILON;
        let dw_body = (max_r * limit + self.uncertainty.momentum) / min_i + angle * w0;
        let dw = dw_body + wcap * angle;
        let dv = self.uncertainty.velocity + limit / M + r.gravity.length() * bound_time;
        let acap = 0.5 * diff * wcap * wcap;
        let accel = r.gravity.length() + (acap + wcap * wcap) * max_r;
        let mut outward = [0.; 2];
        let mut slopes = [DVec3::ZERO; 2];
        let mut tilt = [0.; 2];
        let mut coeff = [[Coeff::default(); 2]; 2];
        let mut uniq = [0.; 2];
        for j in 0..2 {
            let rr = g.arm[j].length();
            let tj = jet(r, x, self.sites[j], self.planes[j]);
            let vary = rr * ((diff + 2.) * wcap * dw_body + (acap + wcap * wcap) * angle);
            outward[j] = tj.a - vary - tj.aeps;
            if outward[j] <= 0. {
                return None;
            }
            let t = g.u[j] - g.n[j] * g.u[j].dot(g.n[j]);
            let slip = t.length();
            let du = dv + rr * (dw + wcap * angle) + tj.veps;
            if self.mu[j] > 0. {
                if slip <= du {
                    return None;
                }
                slopes[j] = -self.mu[j] * t / slip;
                tilt[j] = self.mu[j] * (2. * du / (slip - du)).min(2.);
            }
            let kmax = 1. + M * rr * rr / min_i;
            let nj = g.rows[j][j].dot(g.n[j]);
            let nt = (g.rows[j][j] - g.n[j] * nj).length();
            let drift = 4. * rr * rr * angle / min_i + tj.veps;
            uniq[j] = 2. * kmax.sqrt() / (1. + kmax) - self.mu[j] * (nt + drift) / (nj - drift);
            if uniq[j] <= 0. || nj <= drift {
                return None;
            }
            for (i, coefficients) in coeff.iter_mut().enumerate() {
                let w = g.rows[i][j];
                let proj = w.dot(g.n[j] + slopes[j]);
                let terr = (w - g.n[j] * w.dot(g.n[j])).length() * tilt[j];
                let geo = (4. * g.arm[i].length() * rr * angle / min_i
                    + 1024. * f64::EPSILON * (1. + w.length()))
                    * (1. + self.mu[j]);
                coefficients[j] = Coeff {
                    lo: proj - terr - geo,
                    hi: proj + terr + geo,
                };
            }
        }
        let speed_error = self.uncertainty.velocity(r, x, g.arm[first].length());
        let normal_roundoff = 2048. * f64::EPSILON * (1. + g.u[0].length() + g.u[1].length());
        let series = gz_physics::AlternatingNormalInput {
            next: first,
            clearance_upper: self.gap.hi,
            closing_speed: [c - speed_error, c + speed_error],
            response: coeff.map(|row| row.map(|v| [v.lo, v.hi])),
            outward_min: outward,
            acceleration_max: accel,
            world_per_normal: self.mu.map(|m| (1. + m * m).sqrt()),
            impulse_budget: limit,
            time_budget: accumulation_time * (1. + 8. * f64::EPSILON),
            normal_roundoff,
        }
        .bound()?;
        let (beta, gamma, q, eta, rho, sum) = (
            series.transfer_lower,
            series.transfer_upper,
            series.time_ratio_cap,
            series.acceleration_ratio,
            series.pair_speed_ratio_cap,
            series.world_impulse_bound,
        );
        let mut other = f64::INFINITY;
        let mut margin = f64::INFINITY;
        for (i, plane) in self.planes.iter().copied().enumerate() {
            for (k, local) in vertices().iter().enumerate() {
                let p = x.com + x.q * (*local);
                let coords = plane.coordinates(p, self.time);
                let travel = self.uncertainty.position
                    + local.length() * self.uncertainty.angle
                    + plane.v.length() * self.uncertainty.time
                    + ((x.v - plane.v).length() + dv + wcap * local.length()) * bound_time;
                margin = margin.min((plane.half - coords.truncate().abs()).min_element() - travel);
                if !self.sites.iter().any(|s| s.side == i && s.k == k) {
                    other = other.min(coords.z - travel);
                }
            }
        }
        if other <= 1e-11 || margin <= 1e-10 {
            return None;
        }
        let low = series.normal_impulse_bounds.map(|v| v[0]);
        let high = series.normal_impulse_bounds.map(|v| v[1]);
        let mut initial = [0.; 2];
        initial[first] = -c;
        let rhs_err = speed_error + normal_roundoff + accel * series.time_cap;
        let center =
            [0, 1].map(|k| (0.5 * (high[k] + low[k].max(0.)) * (g.n[k] + slopes[k])).to_array());
        let radius = [0, 1].map(|k| {
            0.5 * (high[k] - low[k].max(0.)) * (1. + self.mu[k].powi(2)).sqrt() + high[k] * tilt[k]
        });
        Some(
            json!({"first":first,"prior_gap_m":self.gap.data(),"incoming_speed":c,"initial_time_cap_s":h,"remaining_time_cap_s":accumulation_time,"free_after_s":free_after,"interarrival_ratio_cap":q,"closure_ratio_lower":beta,"closure_ratio_upper":gamma,"normal_speed_pair_ratio_cap":rho,"acceleration_cap":accel,"relative_acceleration":eta,"angular_travel_cap_rad":angle,"trial_impulse_limit_ns":limit,"total_impulse_bound_ns":sum,"normal_coefficients":coeff.map(|row|row.map(|a|[a.lo,a.hi])),"normal_initial":initial,"normal_rhs_error":rhs_err,"normal_sum_bounds_ns":[[low[0].max(0.),high[0]],[low[1].max(0.),high[1]]],"world_vector_centers":center,"world_vector_radii":radius,"slip_cone_radius":tilt,"both_release_accel_floor":outward,"unique_friction_margin":uniq,"other_gap_m":other,"footprint_margin_m":margin,"frozen":self.frozen(g)}),
        )
    }
    fn frozen(&self, g: Geometry) -> Option<Value> {
        self.frozen_with(g, |_, _, _, _| {})
    }
    fn frozen_with(
        &self,
        g: Geometry,
        mut trace: impl FnMut(usize, DVec3, DVec3, DVec3),
    ) -> Option<Value> {
        let mut x = self.x;
        let mut site = 1 - self.fired;
        let mut j = [DVec3::ZERO; 2];
        let mut total_loss = 0.;
        let mut count = 0;
        let response = |site: usize, v: DVec3| {
            v / M
                + self
                    .r
                    .body
                    .omega(x.q, g.arm[site].cross(v))
                    .cross(g.arm[site])
        };
        for _ in 0..2048 {
            let w = self.r.body.omega(x.q, x.l);
            let u = x.v - self.planes[site].v + w.cross(g.arm[site]);
            let n = g.n[site];
            let c = -u.dot(n);
            if c <= 4e-13 * (1. + u.length()) {
                break;
            }
            let mu = self.mu[site];
            let tangent = |v: DVec3| v - n * v.dot(n);
            let mut slope = -mu * tangent(u).normalize_or_zero();
            let mut value = None;
            for _ in 0..64 {
                let label = n + slope;
                let d = n.dot(response(site, label));
                if d <= 0. {
                    return None;
                }
                let current = c / d * label;
                let up = u + response(site, current);
                let actual = -mu * tangent(up).normalize_or_zero();
                if actual.distance(slope) < 2e-13 * (1. + mu) {
                    value = Some((current, up));
                    break;
                }
                slope = actual;
            }
            let (jj, up) = value?;
            trace(site, jj, u, up);
            total_loss += -0.5 * jj.dot(u + up);
            j[site] += jj;
            x.v += jj / M;
            x.l += g.arm[site].cross(jj);
            count += 1;
            site = 1 - site;
        }
        let w = self.r.body.omega(x.q, x.l);
        Some(
            json!({"steps":count,"impulses":j.map(|v|v.to_array()),"loss_j":total_loss,"velocity":x.v.to_array(),"momentum":x.l.to_array(),"normal_velocity":([0,1].map(|k|(x.v-self.planes[k].v+w.cross(g.arm[k])).dot(g.n[k])))}),
        )
    }
}
pub(super) fn classify(
    r: &Physical,
    context: ([Face; 2], [Site; 2], [f64; 2]),
    x: Flight,
    fired: usize,
    gap: Interval,
    global: f64,
) -> Value {
    let (planes, sites, mu) = context;
    let s = Local {
        r,
        x,
        planes,
        sites,
        mu,
        fired,
        gap,
        time: global,
        uncertainty: Default::default(),
    };
    let g = s.geometry();
    let other = 1 - fired;
    let v = g.u[other].dot(g.n[other]);
    if let Some(after) = open_from(r, gap, x, (sites, planes), fired, global, 8e-9) {
        return json!({"kind":"departing","free":after});
    }
    if !gap.positive() || v >= -1e-12 {
        return json!({"kind":"unresolved","speed":v});
    }
    let h = 2. * gap.hi / (-v);
    if !(h > 0. && h < 1e-9) {
        return json!({"kind":"unresolved","time_cap_s":h});
    }
    for k in 0..40 {
        let limit = (-v) * 0.125 * (1.5f64).powi(k);
        if let Some(proof) = s.trial(g, limit, -v, h, 0.) {
            let extended = [1e-4, 8e-6, 1e-6, 128e-9, 8e-9]
                .into_iter()
                .find_map(|duration| s.trial(g, limit, -v, h, duration));
            return json!({"kind":"accumulation_then_free_release","at_ns":global*1e9,"proof":proof,"extended":extended});
        }
    }
    json!({"kind":"unresolved","time_cap_s":h,"speed":v,"frozen_control":s.frozen(g)})
}
fn open_from(
    r: &Physical,
    gap: Interval,
    x: Flight,
    geometry: ([Site; 2], [Face; 2]),
    fired: usize,
    at: f64,
    h: f64,
) -> Option<Value> {
    let (sites, planes) = geometry;
    let (_, margin) = protect(r, x, planes, sites, at, h)?;
    let (_, a) = released(r, x, sites[fired], planes[fired], h, 32)?;
    let other = 1 - fired;
    let j = jet(r, x, sites[other], planes[other]);
    let (_, amin) = curvature(r, x, sites[other], planes[other], h, 32)?;
    let v = j.v - j.veps + amin.min(0.) * h;
    let min = gap.lo + v.min(0.) * h;
    if min <= 0. {
        return None;
    }
    Some(json!({"h_s":h,"departing_accel_floor":a,"other_gap_floor_m":min,"margin_m":margin}))
}
#[derive(Clone, Copy)]
pub(super) struct Goal {
    /// Per-pair world-vector error allowed on the terminal nominal solution.
    pub vector_radius: f64,
    /// Minimum certified free interval after the limiting double tangency.
    pub release_s: f64,
    /// Refuse a closure whose entire remaining event interval exceeds this time.
    pub time_s: f64,
}
pub(super) fn early(
    r: &Physical,
    context: ([Face; 2], [Site; 2], [f64; 2]),
    x: Flight,
    fired: usize,
    gap: Interval,
    time: f64,
    settings: (Goal, uncertainty::Error),
) -> Option<Value> {
    let (goal, uncertainty) = settings;
    if !gap.positive() {
        return None;
    }
    let (planes, sites, mu) = context;
    let s = Local {
        r,
        x,
        planes,
        sites,
        mu,
        fired,
        gap,
        time,
        uncertainty,
    };
    let g = s.geometry();
    let c = -g.u[1 - fired].dot(g.n[1 - fired]);
    if c <= 1e-12 {
        return None;
    }
    let lower = c - uncertainty.velocity(r, x, g.arm[1 - fired].length());
    if lower <= 0. {
        return None;
    }
    let h = 2. * gap.hi / lower;
    if h <= 0. || 2. * h > goal.time_s {
        return None;
    }
    for k in 0..40 {
        let limit = c * 0.125 * (1.5f64).powi(k);
        let Some(proof) = s.trial(g, limit, c, h, goal.release_s) else {
            continue;
        };
        let q = &proof["frozen"];
        if q.is_null() {
            return None;
        }
        let radii = [0, 1].map(|j| {
            let v = |v: &Value| DVec3::from_array(serde_json::from_value(v.clone()).unwrap());
            proof["world_vector_radii"][j].as_f64().unwrap()
                + v(&proof["world_vector_centers"][j]).distance(v(&q["impulses"][j]))
                + 2e-12
        });
        let fits = |r: [f64; 2]| {
            r.iter()
                .zip(uncertainty.impulses)
                .all(|(v, e)| v + e <= goal.vector_radius)
        };
        let (radii, tracked) = if fits(radii) {
            (radii, 0)
        } else if let Some((alternate, steps)) = s.track(g, &proof) {
            let tight = [0, 1].map(|j| alternate[j].min(radii[j]));
            (tight, steps)
        } else {
            (radii, 0)
        };
        if fits(radii) {
            return Some(
                json!({"kind":"early_accumulation","at_s":time,"radii":radii,"tracked_impulses":tracked,"propagated":uncertainty.data(),"proof":proof}),
            );
        }
        // Increasing the bootstrap ball only enlarges the admissible variation.
        return None;
    }
    None
}
