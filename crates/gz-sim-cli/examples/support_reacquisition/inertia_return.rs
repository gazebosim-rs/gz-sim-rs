// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently integrate triaxial flights and successive isolated, plastic corner impacts.
use super::*;
use gz_sim_scene::primary_contact::{
    last_box_flight, last_box_timing, last_refinement, set_refined_observer,
};
use std::sync::{Arc, Mutex};
#[path = "inertia_sweep.rs"]
pub(crate) mod sweep;
const END: f64 = 0.04;
const DONORS: usize = 640;
const EPS: f64 = 2e-12;

fn donor(id: usize) -> Physical {
    let mut r = Physical::new(id);
    if id < 320 {
        return r;
    }
    // Additional low-tilt arrivals place successive vertices closer together.
    let h = |p| radical(id + 951, p);
    let sign = |p| if h(p) > 0.5 { 1. } else { -1. };
    let w_body = r.q.conjugate() * r.body.omega(r.q, r.pre.l);
    r.q = r.pad_q
        * DQuat::from_euler(
            glam::EulerRot::ZYX,
            0.2 * (2. * h(71) - 1.),
            sign(23) * (0.005 + 0.085 * h(19)),
            sign(31) * (0.006 + 0.10 * h(29)),
        );
    let n = r.n;
    let w = r.q * w_body;
    let arms = corners().map(|p| r.q * (p - ARM));
    r.corner = (0..4)
        .min_by(|&i, &j| arms[i].dot(n).total_cmp(&arms[j].dot(n)))
        .unwrap();
    let arm = arms[r.corner];
    let axis = arm.cross(n);
    let relative = r.pre.v - r.pad_v;
    let v = r.pad_v + relative - n * relative.dot(n) - n * (r.closing + w.cross(arm).dot(n));
    r.at = r.pad_v * r.arrival - n * arm.dot(n);
    r.j = r.closing / (1. / M + axis.dot(r.body.omega(r.q, axis)));
    r.pre = Branch {
        v,
        l: r.body.momentum(r.q, w),
    };
    r.post = Branch {
        v: v + n * r.j / M,
        l: r.pre.l + axis * r.j,
    };
    r
}
#[derive(Clone, Copy)]
struct Flight {
    t: f64,
    com: DVec3,
    q: DQuat,
    v: DVec3,
    l: DVec3,
}
impl Flight {
    fn advance(self, body: ReferenceBody, dt: f64) -> Self {
        Self {
            t: self.t + dt,
            com: self.com + self.v * dt,
            q: advance_q(body, self.q, self.l, dt),
            ..self
        }
    }
    fn state(self, r: &Physical, t: f64) -> State {
        let dt = t - self.t;
        let n = (dt.abs() / RK_H).ceil().max(1.) as usize;
        let mut f = self;
        for _ in 0..n {
            f = f.advance(r.body, dt / n as f64);
        }
        let w = r.body.omega(f.q, f.l);
        let v = f.v + r.pad_v;
        let com = f.com + r.pad_v * t;
        let link = |arm: DVec3| Body {
            position: (com + arm).into(),
            velocity: (v + w.cross(arm)).into(),
            orientation: f.q.into(),
            angular_velocity: w.into(),
        };
        State {
            foot: link(-(f.q * ARM)),
            upper: link(f.q * ARM),
            pad: Body {
                position: (r.pad_v * t - r.n * PAD_HALF[2]).into(),
                orientation: r.pad_q.into(),
                velocity: r.pad_v.into(),
                angular_velocity: [0.; 3],
            },
        }
    }
}
#[derive(Serialize, Clone, Debug)]
struct Event {
    time_s: f64,
    bracket_s: [f64; 2],
    corner: usize,
    impulse_ns: f64,
    closing: f64,
    neighbor_gap_m: f64,
    post_speed: f64,
    energy_residue_j: f64,
}
struct History {
    events: Vec<Event>,
    branches: Vec<Flight>,
    end: f64,
    reason: &'static str,
    orientation_convergence: f64,
}
struct Bounds {
    omega: f64,
    alpha: f64,
    jerk: f64,
}
impl Bounds {
    fn new(r: &Physical, f: Flight) -> Self {
        let w = r.body.omega(f.q, f.l);
        let omega = (w.dot(f.l) / r.body.lower)
            .sqrt()
            .min(f.l.length() / r.body.lower)
            * (1. + 1e-13);
        let ratio = (r.body.upper - r.body.lower) / r.body.lower;
        let alpha = 0.5 * ratio * omega * omega;
        let jerk = (1. + ratio) * omega * alpha + 3. * omega * alpha + omega.powi(3);
        Self { omega, alpha, jerk }
    }
}
#[derive(Clone, Copy)]
enum Next {
    Free,
    Hit { index: usize, lo: f64, hi: f64 },
    Split(&'static str),
}
fn inspect(
    r: &Physical,
    f: Flight,
    end: Flight,
    h: f64,
    fresh: Option<usize>,
    bound: &Bounds,
) -> Next {
    if (f.q.conjugate() * r.n).z <= bound.omega * h {
        return Next::Split("sole orientation or footprint");
    }
    let omega = r.body.omega(f.q, f.l);
    let alpha = r.body.accel(f.q, omega);
    let mut events = vec![];
    for (i, p) in corners().into_iter().enumerate() {
        let local = p - ARM;
        let arm = f.q * local;
        let point = r.pad_q.conjugate() * (f.com + arm);
        let travel = (f.v.length() + bound.omega * arm.length()) * h + EPS;
        if point.x.abs() + travel >= PAD_HALF[0] || point.y.abs() + travel >= PAD_HALF[1] {
            return Next::Split("sole orientation or footprint");
        }
        let z = point.z;
        let zend = (end.com + end.q * local).dot(r.n);
        let speed = (f.v + omega.cross(arm)).dot(r.n);
        let acc = (alpha.cross(arm) + omega.cross(omega.cross(arm))).dot(r.n);
        let amax = arm.length() * (bound.alpha + bound.omega * bound.omega);
        let jerk = arm.length() * bound.jerk;
        if fresh == Some(i) {
            if z.abs() > EPS || speed.abs() > 1e-9 {
                return Next::Split("impact reconstruction");
            }
            if acc - jerk * h > 0. {
                continue;
            }
            return Next::Split("persistent or grazing contact");
        }
        if z < -EPS {
            return Next::Split("unresolved penetration");
        }
        let mono = speed + amax * h;
        let ramp = if acc > 0. {
            (-speed / acc).clamp(0., h)
        } else if speed + 0.5 * acc * h < 0. {
            h
        } else {
            0.
        };
        let lower = z + ramp * (speed + 0.5 * acc * ramp) - jerk * h.powi(3) / 6.;
        if lower > EPS
            || z >= -EPS && (speed - amax * h > 1e-10 || speed >= 0. && acc - jerk * h > 0.)
            || mono < -1e-8 && zend > EPS
        {
            continue;
        }
        if mono >= -1e-8 || z <= 0. || zend > 0. {
            return Next::Split("uncertain monotone crossing");
        }
        let (mut lo, mut hi) = (0., h);
        for _ in 0..36 {
            let mid = 0.5 * (lo + hi);
            let at = f.advance(r.body, mid);
            if (at.com + at.q * local).dot(r.n) > 0. {
                lo = mid
            } else {
                hi = mid
            }
        }
        events.push((lo, hi, i, EPS / (-mono)));
    }
    events.sort_by(|x, y| x.0.total_cmp(&y.0));
    if let Some(&(lo, hi, i, error)) = events.first() {
        if events.get(1).is_some_and(|b| b.0 - b.3 <= hi + error) {
            Next::Split("multiple simultaneous vertices")
        } else {
            Next::Hit { index: i, lo, hi }
        }
    } else {
        Next::Free
    }
}
impl History {
    fn solve(r: &Physical, max_h: f64) -> Self {
        let mut hist = Self {
            events: vec![],
            branches: vec![],
            end: 0.,
            reason: "complete",
            orientation_convergence: 0.,
        };
        if let Some(s) = r.certify_branches(&[(0., r.pre)]).reason {
            hist.reason = s;
            return hist;
        }
        let mut f = Flight {
            t: r.arrival,
            com: r.at - r.pad_v * r.arrival,
            q: r.q,
            v: r.post.v - r.pad_v,
            l: r.post.l,
        };
        let mut fresh = Some(r.corner);
        let mut bound = Bounds::new(r, f);
        let ng = |f: Flight, k| {
            corners()
                .into_iter()
                .enumerate()
                .filter(|(i, _)| *i != k)
                .map(|(_, v)| (f.com + f.q * (v - ARM)).dot(r.n))
                .fold(f64::INFINITY, f64::min)
        };
        hist.events.push(Event {
            time_s: r.arrival,
            bracket_s: [r.arrival; 2],
            corner: r.corner,
            impulse_ns: r.j,
            closing: r.closing,
            neighbor_gap_m: ng(f, r.corner),
            post_speed: 0.,
            energy_residue_j: 0.,
        });
        hist.branches.push(f);
        hist.end = f.t;
        let mut check = f.q;
        let mut cw = check.conjugate() * r.body.omega(check, f.l);
        while f.t < END - 2e-14 {
            let mut h = (END - f.t).min(max_h);
            let (new, step) = loop {
                let new = f.advance(r.body, h);
                let decision = inspect(r, f, new, h, fresh, &bound);
                if let Next::Split(why) = decision {
                    if h <= 1e-9 {
                        hist.reason = why;
                        return hist;
                    }
                    h *= 0.5;
                } else {
                    break (new, decision);
                }
            };
            let (next, dt) = match step {
                Next::Free => (new, h),
                Next::Hit { lo, hi, .. } => {
                    let dt = 0.5 * (lo + hi);
                    (f.advance(r.body, dt), dt)
                }
                Next::Split(_) => unreachable!(),
            };
            for _ in 0..2 {
                (check, cw) = advance_euler(r.body, check, cw, 0.5 * dt);
            }
            hist.orientation_convergence = hist
                .orientation_convergence
                .max(angular_distance(next.q, check));
            if hist.orientation_convergence > 1e-9
                || (check * (r.body.tensor * cw)).distance(f.l) > 3e-9
            {
                hist.reason = "independent Euler convergence";
                return hist;
            }
            hist.end = next.t;
            if let Next::Hit { index, lo, hi } = step {
                let q = next.q;
                let arm = q * (corners()[index] - ARM);
                let axis = arm.cross(r.n);
                let w = r.body.omega(q, f.l);
                let speed = (f.v + w.cross(arm)).dot(r.n);
                let gap = (next.com + arm).dot(r.n);
                let neighbor = ng(next, index);
                if speed >= -1e-4 || neighbor <= 2e-9 || gap.abs() > EPS || dt < 2e-12 {
                    hist.reason = "unresolved isolated strike";
                    return hist;
                }
                let j = -speed / (1. / M + axis.dot(r.body.omega(q, axis)));
                let after = Flight {
                    com: next.com - r.n * gap,
                    v: f.v + r.n * j / M,
                    l: f.l + axis * j,
                    ..next
                };
                let spin = r.body.omega(q, after.l);
                let closing = (after.v + spin.cross(arm)).dot(r.n);
                let loss = 0.5
                    * (M * (f.v.length_squared() - after.v.length_squared()) + w.dot(f.l)
                        - spin.dot(after.l)
                        + speed * j);
                if closing.abs() > 1e-9 || loss.abs() > 1e-9 {
                    hist.reason = "momentum jump closure";
                    return hist;
                }
                let err = (EPS + angular_distance(next.q, check) * arm.length()) / speed.abs();
                hist.events.push(Event {
                    time_s: next.t,
                    bracket_s: [f.t + lo - err, f.t + hi + err],
                    corner: index,
                    impulse_ns: j,
                    closing: -speed,
                    post_speed: closing,
                    neighbor_gap_m: neighbor,
                    energy_residue_j: loss,
                });
                hist.branches.push(after);
                f = after;
                fresh = Some(index);
                check = q;
                cw = q.conjugate() * spin;
                bound = Bounds::new(r, f);
                if hist.events.len() >= 32 {
                    hist.reason = "event limit";
                    return hist;
                }
            } else {
                f = next;
                fresh = None;
            }
        }
        hist.end = END;
        hist
    }
    fn state(&self, r: &Physical, t: f64) -> State {
        if t < r.arrival {
            return r.state_branch(t, r.pre);
        }
        let i = self.branches.partition_point(|x| x.t <= t);
        self.branches[i.saturating_sub(1)].state(r, t)
    }
    fn trace(&self, r: &Physical, n: u64) -> Vec<Snapshot> {
        (0..=n)
            .map(|ix| {
                let t = ix as f64 * 0.0005;
                let mut points = [0.; 4];
                for e in &self.events {
                    if e.time_s > t - 0.0005 && e.time_s <= t {
                        points[e.corner] += e.impulse_ns;
                    }
                }
                frame(ix, self.state(r, t), 0, false, points)
            })
            .collect()
    }
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    let mut featured = false;
    let mut capture = serde_json::Map::new();
    for id in 0..DONORS {
        let r = donor(id);
        let hist = History::solve(&r, RK_H);
        let n = ((hist.end + 1e-13) * 2000.).floor() as u64;
        let events = hist
            .events
            .iter()
            .filter(|e| e.time_s <= n as f64 * 0.0005)
            .count();
        let mut row = json!({"id":id,"events":hist.events,"ticks":n,"censored":hist.reason,"until_s":hist.end,"orientation_convergence_rad":hist.orientation_convergence});
        if events >= 2 {
            let small = History::solve(&r, 0.5 * RK_H);
            let mut max_time = 0.0_f64;
            let mut max_j = 0.0_f64;
            for (i, event) in hist.events.iter().take(events).enumerate() {
                let other = small
                    .events
                    .get(i)
                    .ok_or("finer integrator censored a certified event")?;
                if event.corner != other.corner {
                    return Err(format!("event order differs in {id}/{i}").into());
                }
                max_time = max_time.max((event.time_s - other.time_s).abs());
                max_j = max_j.max((event.impulse_ns - other.impulse_ns).abs());
            }
            if max_time > 0.5e-9 || max_j > 1e-6 {
                return Err(
                    format!("return reference convergence {id}: {max_time} {max_j}").into(),
                );
            }
            row["half_step_root_convergence_ns"] = json!(max_time * 1e9);
            row["half_step_impulse_convergence_ns"] = json!(max_j);
            let view = !featured && events >= 3 && n >= 12;
            let mut trials = vec![];
            for budget in [4, 8, 16] {
                for release in [false, true] {
                    let (_, xml) = released::configured(&r, budget, release);
                    let (trial, frames) = run(&r, &hist, &xml, n)?;
                    trials.push(json!({"budget":budget,"release":release,"trial":trial}));
                    if view && budget == 8 {
                        let k = if release { "released" } else { "baseline" };
                        separated::single_rigid_trace(path, k, &frames)?;
                        fs::write(path.join(format!("{k}.sdf")), xml)?;
                    }
                }
            }
            if view {
                featured = true;
                for (label, e) in ["first_strike", "second_strike", "third_strike"]
                    .into_iter()
                    .zip(&hist.events)
                {
                    capture.insert(label.into(), json!((e.time_s * 2000.).ceil() as u64));
                }
                separated::single_rigid_trace(path, "analytic", &hist.trace(&r, n))?;
                row["view"] = true.into();
            }
            row["trials"] = json!(trials);
        }
        cases.push(row);
    }
    write_json(
        &path.join("returns.json"),
        &json!({"horizon_s":END,"cases":cases}),
    )?;
    if !featured {
        return Err("no certified multistrike replay".into());
    }
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"capture_ticks":capture,"title":"Repeated triaxial corner impacts on a moving pad","lanes":[
        {"label":"baseline","name":"Ordinary rotation between impacts","file":"traces/baseline.json"},
        {"label":"released","name":"Certified rotation between impacts","file":"traces/released.json"},
        {"label":"analytic","name":"Independent successive rigid impacts","file":"traces/analytic.json","analytical":true}]}),
    )
}
fn run(r: &Physical, h: &History, xml: &str, n: u64) -> Result<(Value, Vec<Snapshot>)> {
    let (mut sim, ids) = sdf_features::load_finite_box(xml, false)?;
    let mut frames = vec![frame(0, state(backend(&sim), ids)?, 0, false, [0.; 4])];
    let observed = Arc::new(Mutex::new(Vec::new()));
    let hook = observed.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let ns = s.elapsed.as_nanos() as u64;
        let cut = gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns,
        };
        let mut o = Observed::default();
        o.record(s, ids);
        hook.lock().unwrap().push((cut, o.corners));
    })?;
    let mut receipts = vec![];
    let mut released_intervals = vec![];
    let mut sweep_candidates = vec![];
    let mut sweep_queries = 0;
    let mut nominated = vec![];
    let mut count = 0;
    let mut maximum_departure_spin_residual = 0.0_f64;
    let mut minimum_departure_gap = f64::INFINITY;
    for tick in 1..=n {
        let mut before = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let stamp = sim.elapsed();
        if let Some(log) =
            gz_sim_scene::primary_contact::last_box_sweep(&sim).filter(|x| x.sim_time == stamp)
        {
            sweep_queries += log.pairs.len();
            for (_, _, status) in &log.pairs {
                if let gz_physics_multifidelity::RigidBoxSweep::Possible { vertices } = status {
                    sweep_candidates.push((tick, *vertices));
                }
            }
        }
        let mut measures = std::mem::take(&mut *observed.lock().unwrap());
        if measures.is_empty() {
            measures.push((
                gyro::Cut::bodies(backend(&sim), ids, 500_000),
                sdf::unrefined(backend(&sim), ids).0,
            ));
        }
        let total = measures.len() as u32;
        count += total;
        let mut sum = [0.; 4];
        let flight = last_box_flight(&sim).filter(|x| x.sim_time == stamp);
        let timing = last_box_timing(&sim).filter(|x| x.sim_time == stamp);
        let compact = timing.and_then(|t| t.timing.compact);
        if total > 1 {
            nominated.push(
                json!({"tick":tick,"calls":total,"compact":compact.map(scene::compact_record)}),
            );
        }

        for (index, (cut, points)) in measures.into_iter().enumerate() {
            let ns = cut.ns;
            if index + 1 == total as usize && compact.is_some_and(|m| m.complete) {
                assert!(total >= 3);
                assert_eq!(cut.ns, 500_000);
                assert_eq!(points.iter().sum::<f64>(), 0.);
                let independent = released::checked_departure(r.body, before, cut)?;
                let lower = independent["quadratic_lower_m"].as_f64().unwrap();
                let sampled = independent["sample_min_m"].as_f64().unwrap();
                let witness = compact.unwrap().tail_gap_pm.unwrap() as f64 * 1e-12;
                assert!(
                    lower > 0. && sampled + 2e-12 >= witness,
                    "invalid compact tail {} {independent} {witness}",
                    r.id
                );
                nominated.last_mut().unwrap()["tail_control"] = independent;
            }

            if let Some(mark) = flight
                .and_then(|g| g.flight.released.as_ref())
                .and_then(|g| g.get(index))
                .filter(|s| s.integrated)
            {
                assert_eq!(points.iter().sum::<f64>(), 0.);
                let independent = released::oracle(r.body, before, cut)?;
                let min = independent["sample_min_m"].as_f64().unwrap();
                let lower = independent["quadratic_lower_m"].as_f64().unwrap();
                assert!(
                    min + 2e-12 >= mark.minimum_gap_pm.unwrap() as f64 * 1e-12 && lower > 0.,
                    "return departure {} {independent}",
                    r.id
                );
                minimum_departure_gap = minimum_departure_gap.min(lower);
                maximum_departure_spin_residual = maximum_departure_spin_residual
                    .max(independent["end_spin_error_rad_s"].as_f64().unwrap());
            }
            before = cut;
            for (s, v) in sum.iter_mut().zip(points) {
                *s += v
            }
            if points.iter().sum::<f64>() > 1e-12 {
                receipts.push(json!({"end_ns":(tick-1)*500000+ns,"corners":points}));
            }
        }
        if let Some(g) = last_box_flight(&sim).filter(|x| x.sim_time == stamp) {
            let mut start = (tick - 1) * 500000;
            for (&dt, &guard) in g.flight.durations.iter().zip(&g.flight.guarded) {
                let end = start + dt.as_nanos() as u64;
                if guard && g.flight.released.is_some() {
                    released_intervals.push([start, end]);
                }
                start = end;
            }
        }
        let selected = last_refinement(&sim).is_some_and(|x| x.sim_time == stamp)
            || last_box_timing(&sim).is_some_and(|x| x.sim_time == stamp);
        frames.push(frame(
            tick,
            state(backend(&sim), ids)?,
            total,
            selected,
            sum,
        ));
    }
    let actual = frames.last().unwrap().state;
    let exact = h.state(r, n as f64 * 0.0005);
    Ok((
        json!({"nominated":nominated,"sweep_queries":sweep_queries,"sweep_candidates":sweep_candidates,"minimum_departure_gap_m":minimum_departure_gap.is_finite().then_some(minimum_departure_gap),"maximum_departure_spin_residual_rad_s":maximum_departure_spin_residual,"native_calls":count,"receipts":receipts,"release_intervals_ns":released_intervals,"final_com_m":actual.com().distance(exact.com()),"final_spin_rad_s":DVec3::from(actual.foot.angular_velocity).distance(exact.foot.angular_velocity.into())}),
        frames,
    ))
}

fn clocked(xml: &str, n: u64, end: State) -> Result<Vec<u64>> {
    let (mut sim, ids) = sdf_features::load_finite_box(xml, false)?;
    let mut costs = Vec::with_capacity(n as usize);
    for _ in 0..n {
        let now = Instant::now();
        sim.step_exact(1)?;
        costs.push(now.elapsed().as_nanos() as u64);
    }
    if state(backend(&sim), ids)? != end {
        return Err("unobserved triaxial return differs from verified run".into());
    }
    Ok(costs)
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    let base = path.parent().ok_or("expected a timing subfolder")?;
    let summary: Value = serde_json::from_slice(&fs::read(base.join("returns.json"))?)?;
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for c in summary["cases"].as_array().ok_or("missing returns")? {
        if c["trials"].is_null() {
            continue;
        }
        let id = c["id"].as_u64().unwrap() as usize;
        let n = c["ticks"].as_u64().unwrap();
        let r = donor(id);
        let mut fixtures = vec![];
        for release in [false, true] {
            let (_, xml) = released::configured(&r, 8, release);
            let (mut sim, ids) = sdf_features::load_finite_box(&xml, false)?;
            sim.step_exact(n)?;
            let end = state(backend(&sim), ids)?;
            clocked(&xml, n, end)?;
            fixtures.push((release, xml, end));
        }
        let mut rows_ns: [Vec<Vec<u64>>; 2] = Default::default();
        for rep in 0..4 {
            for position in 0..2 {
                let i = (id + position + rep) % 2;
                rows_ns[i].push(clocked(&fixtures[i].1, n, fixtures[i].2)?);
            }
        }
        for ((release, _, _), raw) in fixtures.into_iter().zip(rows_ns) {
            rows.push(json!({"id":id,"release":release,"steps_ns":raw}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}
