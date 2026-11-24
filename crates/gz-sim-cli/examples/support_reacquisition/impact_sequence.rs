// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve successive frictionless rigid impacts only while their order and separating flights are certified.
use super::*;
#[path = "impact_sequence_native.rs"]
mod native;
#[path = "impact_production.rs"]
pub(crate) mod production;
const GAP_EPS: f64 = 2e-12;
const TIME_WIDTH: f64 = 2e-13;
const SPEED_EPS: f64 = 1e-11;
const LIMIT: usize = 64;

#[derive(Clone, Copy)]
struct Flight {
    time: f64,
    com: DVec3,
    q: DQuat,
    velocity: DVec3,
    spin: DVec3,
    touching: [bool; 4],
}
impl Flight {
    fn advance(self, t: f64) -> Self {
        Self {
            time: self.time + t,
            com: self.com + self.velocity * t,
            q: DQuat::from_scaled_axis(self.spin * t) * self.q,
            ..self
        }
    }
    fn state(self, r: Reference, t: f64) -> State {
        let f = self.advance(t - self.time);
        let c = r.case;
        let v = f.velocity + c.pad_v;
        let link = |arm: DVec3| Body {
            position: (f.com + c.pad_v * t + arm).to_array(),
            orientation: f.q.to_array(),
            velocity: (v + f.spin.cross(arm)).to_array(),
            angular_velocity: f.spin.to_array(),
        };
        State {
            foot: link(-(f.q * ARM)),
            upper: link(f.q * ARM),
            pad: Body {
                position: (c.pad_v * t - r.normal * PAD_HALF[2]).into(),
                orientation: c.surface.to_array(),
                velocity: c.pad_v.to_array(),
                angular_velocity: [0.; 3],
            },
        }
    }
    fn bounded(self, r: Reference, t: f64) -> bool {
        let c = r.case;
        let change =
            |radius: f64| (self.velocity.length() + radius * self.spin.length()) * t + GAP_EPS;
        let inv = c.surface.conjugate();
        // Prove the lower sole remains below the top corners over this branch.
        let local_up = self.q.conjugate() * r.normal;
        if local_up.z - self.spin.length() * t <= 0. {
            return false;
        }
        for p in corners() {
            let arm = self.q * (p - ARM);
            let b = inv * (self.com + arm);
            let bound = change(arm.length());
            if b.x.abs() + bound >= PAD_HALF[0] || b.y.abs() + bound >= PAD_HALF[1] {
                return false;
            }
        }
        let height =
            (self.com + self.q * ARM).dot(r.normal) - local_up.abs().dot(UPPER_HALF.into());
        height > change(ARM.length() + DVec3::from(UPPER_HALF).length())
    }
}
#[derive(Clone, Copy)]
struct Gap {
    height: f64,
    v: f64,
    omega: f64,
    s: f64,
    c: f64,
    accel_bound: f64,
    jerk_bound: f64,
}
impl Gap {
    fn new(f: Flight, n: DVec3, k: usize) -> Self {
        let arm = f.q * (corners()[k] - ARM);
        let omega = f.spin.length();
        let axis = if omega > 0. {
            f.spin / omega
        } else {
            DVec3::ZERO
        };
        let s = axis.cross(arm).dot(n);
        let c = axis.cross(axis.cross(arm)).dot(n);
        Self {
            height: if f.touching[k] {
                0.
            } else {
                (f.com + arm).dot(n)
            },
            v: f.velocity.dot(n),
            omega,
            s,
            c,
            accel_bound: omega * omega * (s * s + c * c).sqrt(),
            jerk_bound: omega.powi(3) * (s * s + c * c).sqrt(),
        }
    }
    // Rodrigues' formula in the pad frame. 2*sin(theta/2)^2 retains the
    // quadratic opening of a plastically arrested corner near t=0.
    fn at(self, t: f64) -> f64 {
        let theta = self.omega * t;
        self.height + self.v * t + self.s * theta.sin() + 2. * self.c * (0.5 * theta).sin().powi(2)
    }
    fn velocity(self, t: f64) -> f64 {
        let (s, c) = (self.omega * t).sin_cos();
        self.v + self.omega * (self.s * c + self.c * s)
    }
    fn acceleration(self, t: f64) -> f64 {
        let (s, c) = (self.omega * t).sin_cos();
        self.omega * self.omega * (-self.s * s + self.c * c)
    }
    fn remains_on_plane(self) -> bool {
        self.height.abs() < GAP_EPS
            && self.velocity(0.).abs() < SPEED_EPS
            && self.accel_bound < 1e-12
    }
    fn first(self, l: f64, h: f64) -> std::result::Result<Option<(f64, f64)>, &'static str> {
        let width = h - l;
        let mid = (l + h) * 0.5;
        let gl = self.at(l);
        let gh = self.at(h);
        let vm = self.velocity(mid);
        let far = self.accel_bound * width * 0.5;
        if self.remains_on_plane() {
            return Ok(None);
        }
        if l == 0.
            && self.height.abs() < GAP_EPS
            && self.velocity(0.) >= -SPEED_EPS
            && self.acceleration(mid) - self.jerk_bound * width * 0.5 > 0.
        {
            return Ok(None);
        }
        if vm - far >= 0. && gl >= -GAP_EPS {
            return Ok(None);
        }
        if vm + far < 0. {
            if gh > 0. {
                return Ok(None);
            }
            if gl < -GAP_EPS {
                return Err("entering branch already penetrates");
            }
            let (mut lo, mut hi) = (l, h);
            while hi - lo > TIME_WIDTH {
                let m = 0.5 * (lo + hi);
                if self.at(m) > 0. {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            return Ok(Some((lo, hi)));
        }
        // A twice differentiable scalar is within A*h²/8 of its chord.
        if gl.min(gh) - self.accel_bound * width * width / 8. > GAP_EPS {
            return Ok(None);
        }
        if width <= TIME_WIDTH {
            return Err("grazing or sustained contact unresolved");
        }
        if let Some(t) = self.first(l, mid)? {
            return Ok(Some(t));
        }
        self.first(mid, h)
    }
}
#[derive(Clone, Serialize)]
struct Impact {
    time_s: f64,
    bracket_s: [f64; 2],
    corner_ns: [f64; 4],
    active: [bool; 4],
    before_speed_m_s: [f64; 4],
    after_speed_m_s: [f64; 4],
    min_inactive_gap_m: Option<f64>,
}
#[derive(Clone)]
struct Sequence {
    r: Reference,
    impacts: Vec<Impact>,
    branches: Vec<Flight>,
    certified_until: f64,
    reason: &'static str,
}
impl Sequence {
    fn state(&self, t: f64) -> State {
        if t < self.impacts[0].time_s {
            return self.r.state((t * 1e9).round() as u64);
        }
        let index = self
            .branches
            .partition_point(|f| f.time <= t)
            .saturating_sub(1);
        self.branches[index].state(self.r, t)
    }
    fn solve(r: Reference) -> Result<Self> {
        let c = r.case;
        let initial_t = c.arrival_ns as f64 * 1e-9;
        let pre = Flight {
            time: initial_t,
            com: c.at_com - c.pad_v * initial_t,
            q: c.orientation,
            velocity: c.incoming_v - c.pad_v,
            spin: c.incoming_spin,
            touching: r.active,
        };
        let mut f = Flight {
            velocity: r.outgoing_v - c.pad_v,
            spin: r.outgoing_spin,
            ..pre
        };
        let speeds = |f: Flight| std::array::from_fn(|k| Gap::new(f, r.normal, k).velocity(0.));
        let mut out = Self {
            r,
            impacts: vec![Impact {
                time_s: initial_t,
                bracket_s: [initial_t; 2],
                corner_ns: r.j,
                active: r.active,
                before_speed_m_s: speeds(pre),
                after_speed_m_s: speeds(f),
                min_inactive_gap_m: minimum_inactive(pre, r.normal),
            }],
            branches: vec![f],
            certified_until: 0.,
            reason: "certified",
        };
        let from_start = Flight {
            time: 0.,
            com: pre.com - pre.velocity * initial_t,
            q: DQuat::from_scaled_axis(-pre.spin * initial_t) * pre.q,
            ..pre
        };
        if !from_start.bounded(r, initial_t)
            || (0..4).any(|k| {
                let g = Gap::new(pre, r.normal, k);
                g.velocity(0.) + g.accel_bound * initial_t >= 0.
            })
        {
            out.reason = "incoming flight or finite pad not certified";
            return Ok(out);
        }
        if r.j.iter().zip(r.active).any(|(j, a)| a && *j <= 0.) {
            out.reason = "inadmissible primary load";
            return Ok(out);
        }
        let end = LEN_NS as f64 * 1e-9;
        out.certified_until = f.time;
        'sequence: while f.time < end {
            if out.impacts.len() >= LIMIT {
                out.reason = "event count limit";
                break;
            }
            let left = end - f.time;
            let mut times = vec![];
            for k in 0..4 {
                match Gap::new(f, r.normal, k).first(0., left) {
                    Ok(Some((lo, hi))) => times.push((lo, hi, k)),
                    Ok(None) => {}
                    Err(reason) => {
                        out.reason = reason;
                        break 'sequence;
                    }
                }
            }
            times.sort_by(|a, b| a.0.total_cmp(&b.0));
            if times.is_empty() {
                if f.bounded(r, left) {
                    out.certified_until = end;
                } else {
                    out.reason = "finite pad or upper clearance";
                }
                break;
            }
            let (lo, hi, k) = times[0];
            let t = 0.5 * (lo + hi);
            if !f.bounded(r, hi) {
                out.reason = "finite pad or upper clearance";
                break;
            }
            out.certified_until = f.time + lo;
            if lo < 2. * TIME_WIDTH || times.get(1).is_some_and(|e| e.0 <= hi + TIME_WIDTH) {
                out.reason = "unresolved event separation";
                break;
            }
            let mut before = f.advance(t);
            let mut touching = [false; 4];
            touching[k] = true;
            for (i, slot) in touching.iter_mut().enumerate() {
                if i != k {
                    let g = Gap::new(f, r.normal, i);
                    if g.remains_on_plane() {
                        *slot = true;
                    } else if g.at(t) <= GAP_EPS + g.velocity(t).abs() * (hi - lo) {
                        out.reason = "inactive gap below certification tolerance";
                        break 'sequence;
                    }
                }
            }
            before.touching = touching;
            let Some((post, impulses)) = solve_impulse(before, r.normal) else {
                out.reason = "unilateral impact unresolved";
                break;
            };
            out.impacts.push(Impact {
                time_s: post.time,
                bracket_s: [f.time + lo, f.time + hi],
                active: touching,
                corner_ns: impulses,
                before_speed_m_s: speeds(before),
                after_speed_m_s: speeds(post),
                min_inactive_gap_m: minimum_inactive(before, r.normal),
            });
            out.branches.push(post);
            f = post;
        }
        Ok(out)
    }
}
fn minimum_inactive(f: Flight, n: DVec3) -> Option<f64> {
    (0..4)
        .filter(|k| !f.touching[*k])
        .map(|k| Gap::new(f, n, k).height)
        .min_by(f64::total_cmp)
}
fn solve_impulse(f: Flight, n: DVec3) -> Option<(Flight, [f64; 4])> {
    let arm = corners().map(|p| f.q * (p - ARM));
    let torque = arm.map(|r| r.cross(n));
    let speed = arm.map(|r| (f.velocity + f.spin.cross(r)).dot(n));
    // Enumerate independent active rows. Every returned candidate satisfies
    // all touching inequalities and complementarity, also for unloaded rows.
    for mask in 1_u8..16 {
        let indices: Vec<_> = (0..4).filter(|k| mask & (1 << k) != 0).collect();
        if indices.len() > 3 || indices.iter().any(|k| !f.touching[*k]) {
            continue;
        }
        let m = indices.len();
        let mut aug = [[0.; 4]; 3];
        for (a, &i) in indices.iter().enumerate() {
            for (b, &j) in indices.iter().enumerate() {
                aug[a][b] = 0.5 + torque[i].dot(torque[j]) / I;
            }
            aug[a][m] = -speed[i];
        }
        let mut invertible = true;
        for i in 0..m {
            let max = (i..m).max_by(|&a, &b| f64::total_cmp(&aug[a][i].abs(), &aug[b][i].abs()))?;
            aug.swap(i, max);
            let pivot = aug[i][i];
            if pivot.abs() < 1e-12 {
                invertible = false;
                break;
            }
            for x in &mut aug[i][i..=m] {
                *x /= pivot;
            }
            let source = aug[i];
            for (j, row) in aug.iter_mut().enumerate().take(m) {
                if i != j {
                    let factor = row[i];
                    for k in i..=m {
                        row[k] -= factor * source[k];
                    }
                }
            }
        }
        if !invertible {
            continue;
        }
        let mut load = [0.; 4];
        for (i, &k) in indices.iter().enumerate() {
            load[k] = aug[i][m];
        }
        if load.iter().any(|x| *x < 0.) {
            continue;
        }
        let post = Flight {
            velocity: f.velocity + n * load.iter().sum::<f64>() / M,
            spin: f.spin + (0..4).map(|k| torque[k] * load[k]).sum::<DVec3>() / I,
            ..f
        };
        let valid = (0..4).all(|k| {
            !f.touching[k] || {
                let v = (post.velocity + post.spin.cross(arm[k])).dot(n);
                v >= -SPEED_EPS && (load[k] == 0. || v.abs() < SPEED_EPS)
            }
        });
        if valid {
            return Some((post, load));
        }
    }
    None
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    native::cost(path)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    let mut sequences = vec![];
    for id in 0..N_CASES {
        let data = Sequence::solve(Reference::solve(build(id))?)?;
        let until = data.certified_until;
        rows.push(json!({"id":id,"kind":data.r.case.kind,"certified_until_s":until,"reason":data.reason,"impacts":data.impacts,
            "final_certified_state":data.state(until)}));
        sequences.push(data);
    }
    native::observe(path, &sequences)?;
    write_json(
        &path.join("sequence.json"),
        &json!({"cases":rows,"inactive_clearance_tolerance_m":GAP_EPS,"event_time_width_s":TIME_WIDTH}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn monotone_crossing_is_bracketed_and_separating_flight_stays_clear() {
        let base = Gap {
            height: 1e-4,
            v: -0.5,
            omega: 0.,
            s: 0.,
            c: 0.,
            accel_bound: 0.,
            jerk_bound: 0.,
        };
        let (lo, hi) = base.first(0., 0.001).unwrap().unwrap();
        assert!(lo <= 2e-4 && hi >= 2e-4 && hi - lo <= TIME_WIDTH);
        assert!(Gap { v: 0.5, ..base }.first(0., 0.001).unwrap().is_none());
    }
    #[test]
    fn piecewise_impacts_close_momentum_and_exclude_a_hidden_corner() {
        let mut many = 0;
        let mut five = 0;
        for id in 0..N_CASES {
            let seq = Sequence::solve(Reference::solve(build(id)).unwrap()).unwrap();
            if seq.reason != "certified" || seq.impacts.len() < 2 {
                continue;
            }
            many += 1;
            five += usize::from(seq.impacts.len() == 5);
            let n = seq.r.normal;
            for (i, hit) in seq.impacts.iter().enumerate().skip(1) {
                assert!(hit.time_s > seq.impacts[i - 1].time_s);
                assert!(hit.bracket_s[1] - hit.bracket_s[0] <= TIME_WIDTH * 1.0001);
                let prev = seq.branches[i - 1];
                let pre = prev.advance(hit.time_s - prev.time);
                let post = seq.branches[i];
                let lambda = hit.corner_ns.iter().sum::<f64>();
                assert!(lambda > 0.);
                assert!((post.velocity - pre.velocity).distance(n * lambda / M) < 1e-11);
                let mut torque = DVec3::ZERO;
                for (k, point) in corners().into_iter().enumerate() {
                    assert!(hit.corner_ns[k] >= 0.);
                    let arm = pre.q * (point - ARM);
                    let height = (pre.com + arm).dot(n);
                    if hit.active[k] {
                        assert!(height.abs() < 3e-12);
                        assert!(hit.after_speed_m_s[k] >= -SPEED_EPS);
                    } else {
                        assert!(height > GAP_EPS);
                    }
                    torque += arm.cross(n * hit.corner_ns[k]);
                    for t in [0.125, 0.5, 0.875] {
                        let h = prev.advance(t * (pre.time - prev.time));
                        assert!((h.com + h.q * (point - ARM)).dot(n) > -GAP_EPS);
                    }
                }
                assert!(((post.spin - pre.spin) * I).distance(torque) < 1e-11);
            }
        }
        assert_eq!(many, 35);
        assert_eq!(five, 2);
    }
}
