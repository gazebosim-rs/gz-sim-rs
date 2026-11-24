// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Register native intervals from measured incoming states and compare with equal-work uniform grids.
use super::*;
#[derive(Clone, Copy, Eq, PartialEq, Serialize, Debug)]
#[serde(rename_all = "snake_case")]
pub(super) enum Profile {
    Default8,
    Uniform4,
    Uniform8,
    Uniform16,
    Prospective4,
    Prospective8,
    Scoped8,
    Oracle8,
}
const PROFILES: [Profile; 8] = [
    Profile::Default8,
    Profile::Uniform4,
    Profile::Uniform8,
    Profile::Uniform16,
    Profile::Prospective4,
    Profile::Prospective8,
    Profile::Scoped8,
    Profile::Oracle8,
];
impl Profile {
    fn calls(self) -> u64 {
        match self {
            Self::Uniform4 | Self::Prospective4 => 4,
            Self::Uniform16 => 16,
            _ => 8,
        }
    }
    fn online(self) -> bool {
        matches!(
            self,
            Self::Prospective4 | Self::Prospective8 | Self::Scoped8
        )
    }
}
fn effective(s: State) -> (Flight, DVec3) {
    let normal = DQuat::from_array(s.pad.orientation) * DVec3::Z;
    (
        Flight {
            time: 0.,
            com: s.com() - DVec3::from(s.pad.position) - normal * PAD_HALF[2],
            q: DQuat::from_array(s.foot.orientation).normalize(),
            velocity: s.velocity() - DVec3::from(s.pad.velocity),
            spin: DVec3::from(s.foot.angular_velocity),
            touching: [false; 4],
        },
        normal,
    )
}
/// Predict only while the measured corner is closing. The bound on acceleration
/// excludes a turning point over the short interval before accepting its root.
fn online_h(f: Flight, n: DVec3, ns: u64, left: u64) -> u64 {
    if left <= 1 {
        return ns;
    }
    let share = ns / left;
    let horizon = ns.saturating_sub(left - 1).min(2 * share) as f64 * 1e-9;
    let mut first = f64::INFINITY;
    for k in 0..4 {
        let g = Gap::new(f, n, k);
        if g.velocity(0.) + g.accel_bound * horizon >= -0.01 {
            continue;
        }
        if g.height <= 2e-8 && g.height >= -3e-4 {
            return share.clamp(1, 4000);
        }
        if g.height > 0. && g.at(horizon) <= 0. {
            let mut lo = 0.;
            let mut hi = horizon;
            for _ in 0..22 {
                let mid = 0.5 * (lo + hi);
                if g.at(mid) > 0. {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            first = first.min(lo);
        }
    }
    if first.is_finite() {
        return ((first * 1e9).floor() as u64).clamp(500.min(share), ns - left + 1);
    }
    share
}
fn oracle_nodes(seq: &Sequence, mut nodes: Vec<u64>, calls: u64) -> Vec<u64> {
    let (a, b) = (nodes[0], nodes[1]);
    for event in &seq.impacts {
        let t = (event.time_s * 1e9).floor() as u64;
        if t > a + 1 && t < b && nodes.len() <= calls as usize {
            nodes.push(t);
        }
        let after = t + 4000;
        if after > a && after < b && nodes.len() <= calls as usize {
            nodes.push(after);
        }
    }
    nodes.sort_unstable();
    nodes.dedup();
    while nodes.len() < calls as usize + 1 {
        let i = nodes
            .windows(2)
            .enumerate()
            .max_by_key(|(_, v)| v[1] - v[0])
            .unwrap()
            .0;
        let mid = nodes[i] + (nodes[i + 1] - nodes[i]) / 2;
        nodes.insert(i + 1, mid);
    }
    nodes
}
#[derive(Default)]
struct Measurements {
    j: DVec3,
    corner: [f64; 4],
    pre_second: Option<(u64, DVec3, [f64; 4])>,
    events: Vec<[f64; 4]>,
    calls: usize,
    fresh: usize,
    relaxed: usize,
    spans: usize,
    ticks: Vec<Snapshot>,
}
struct Runner<'a> {
    seq: &'a Sequence,
    ids: Bodies,
    profile: Profile,
    start: u64,
    t: u64,
}
impl Runner<'_> {
    fn run_tick(
        &mut self,
        engine: &mut MultiFidelityBackend,
        end: u64,
        measure: Option<&mut Measurements>,
    ) -> Result<()> {
        self.run_until(engine, end, end, measure)
    }
    fn run_until(
        &mut self,
        engine: &mut MultiFidelityBackend,
        end: u64,
        stop: u64,
        mut measure: Option<&mut Measurements>,
    ) -> Result<()> {
        let count = ((end - self.t) * self.profile.calls())
            .div_ceil(500_000)
            .max(1);
        let first = self.t;
        let nodes = if self.profile == Profile::Oracle8 {
            oracle_nodes(self.seq, vec![first, end], count)
        } else {
            vec![]
        };
        let mut outer = [0.; 4];
        let mut changed = false;
        for i in 0..count {
            if self.t == stop {
                break;
            }
            let ref_state = if self.profile.online() || measure.is_some() {
                Some(state(engine, self.ids)?)
            } else {
                None
            };
            let left = count - i;
            let h = if self.profile.online() {
                let (f, n) = effective(ref_state.unwrap());
                online_h(f, n, end - self.t, left)
            } else if self.profile == Profile::Oracle8 {
                nodes[i as usize + 1] - self.t
            } else {
                first + (i + 1) * (end - first) / count - self.t
            };
            let h = h.min(stop - self.t);
            if h == 0 || self.t + h > end {
                return Err("invalid native event grid".into());
            }
            let (fresh, relax) = match self.profile {
                Profile::Default8 => (false, 1),
                Profile::Scoped8 => schedule(ref_state.unwrap(), Policy::Scoped),
                _ => (true, 16),
            };
            let dt = Duration::from_nanos(h);
            if fresh {
                engine.step_landing_with_fresh_box_pairs(
                    dt,
                    relax,
                    &[(self.ids.foot, self.ids.pad)],
                )?;
            } else if relax == 1 {
                engine.step(dt)?;
            } else {
                engine.step_landing_with_relaxation(dt, relax)?;
            }
            let prev = self.t;
            self.t += h;
            changed |= self.profile.online() && h != (end - first) / count;
            let Some(data) = measure.as_deref_mut() else {
                continue;
            };
            data.calls += 1;
            data.fresh += usize::from(fresh);
            data.relaxed += usize::from(relax > 1);
            let q = DQuat::from_array(ref_state.unwrap().foot.orientation);
            let mut point = [0.; 4];
            let mut vector = DVec3::ZERO;
            for patch in engine.rigid_contact_patches() {
                let direct = (patch.a, patch.b) == (self.ids.foot, self.ids.pad);
                if !direct && (patch.b, patch.a) != (self.ids.foot, self.ids.pad) {
                    if patch.points.iter().any(|p| p.normal_impulse > 1e-8) {
                        return Err("unrelated sequence impulse".into());
                    }
                    continue;
                }
                for p in patch.points {
                    let local =
                        q.conjugate() * DVec3::from(if direct { p.lever_a } else { p.lever_b });
                    let k = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
                    point[k] += p.normal_impulse;
                    vector += DVec3::from(patch.normal_on_a)
                        * (if direct { 1. } else { -1. })
                        * p.normal_impulse;
                }
            }
            for k in 0..4 {
                outer[k] += point[k];
                data.corner[k] += point[k];
            }
            data.j += vector;
            if let Some(secondary) = self.seq.impacts.get(1)
                && (self.t as f64) * 1e-9 < secondary.bracket_s[0]
                && (self.t as f64) * 1e-9 > self.seq.impacts[0].bracket_s[1]
            {
                data.pre_second = Some((self.t, data.j, data.corner));
            }
            let active_events = self
                .seq
                .impacts
                .iter()
                .enumerate()
                .filter(|(_, e)| e.time_s > self.start as f64 * 1e-9);
            if let Some((bin, _)) = active_events.min_by(|(_, a), (_, b)| {
                (a.time_s - self.t as f64 * 1e-9)
                    .abs()
                    .total_cmp(&(b.time_s - self.t as f64 * 1e-9).abs())
            }) {
                for (dst, v) in data.events[bin].iter_mut().zip(point) {
                    *dst += v;
                }
            }
            if self
                .seq
                .impacts
                .iter()
                .filter(|e| e.time_s > prev as f64 * 1e-9 && e.time_s <= self.t as f64 * 1e-9)
                .count()
                > 1
            {
                data.spans += 1;
            }
        }
        if self.t != stop {
            return Err("native sequence grid missed actuator boundary".into());
        }
        if let Some(data) = measure
            && stop == end
        {
            data.ticks.push(Snapshot {
                state: state(engine, self.ids)?,
                tick: end / 500_000,
                time_us: end / 1000,
                native_steps: count as u32,
                selected: changed,
                secondary_start: false,
                normal_impulse_ns: outer.iter().sum(),
                corner_ns: outer,
            });
        }
        Ok(())
    }
}
fn run(
    seq: &Sequence,
    p: Profile,
    order: bool,
    matched: bool,
) -> Result<(Value, Vec<Snapshot>, State)> {
    let start = if matched {
        (seq.impacts[0].time_s * 1e9).ceil() as u64 + 1
    } else {
        0
    };
    let begin = seq.state(start as f64 * 1e-9);
    let (mut native, ids) = fresh_in_order(begin, true, 1, order)?;
    let mut runner = Runner {
        seq,
        ids,
        profile: p,
        start,
        t: start,
    };
    let mut m = Measurements {
        events: vec![[0.; 4]; seq.impacts.len()],
        ..Default::default()
    };
    m.ticks.push(Snapshot {
        state: state(&native, ids)?,
        tick: 0,
        time_us: start / 1000,
        native_steps: 0,
        selected: false,
        secondary_start: false,
        normal_impulse_ns: 0.,
        corner_ns: [0.; 4],
    });
    for end in (1..=8).map(|i| i * 500_000).filter(|v| *v > start) {
        runner.run_tick(&mut native, end, Some(&mut m))?;
    }
    let expected = seq.state(LEN_NS as f64 * 1e-9);
    let mut target = [0.; 4];
    for e in seq.impacts.iter().skip(usize::from(matched)) {
        for (k, v) in e.corner_ns.iter().enumerate() {
            target[k] += v;
        }
    }
    let end = state(&native, ids)?;
    let pre=m.pre_second.map(|(t,v,js)|{
        let expected=if matched{0.}else{seq.impacts[0].corner_ns.iter().sum()};
        json!({"end_ns":t,"normal_ns":v.dot(seq.r.normal),"expected_ns":expected,"vector_error_ns":v.distance(seq.r.normal*expected),"corner_ns":js})
    });
    let total = target.iter().sum::<f64>();
    let body_errors = relative_state_error(end, expected);
    Ok((
        json!({"profile":p,"pad_first":order,"matched_after_primary":matched,"begin_ns":start,"actual_calls":m.calls,
        "refreshed_calls":m.fresh,"relaxed_calls":m.relaxed,"multiple_events_in_one_native_call":m.spans,
        "impulse_vector_ns":m.j.to_array(),"impulse_vector_error_ns":m.j.distance(seq.r.normal*total),"corner_ns":m.corner,
        "corner_l1_error_ns":m.corner.iter().zip(target).map(|(v,w)|(v-w).abs()).sum::<f64>(),"spin_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(expected.foot.angular_velocity.into()),
        "com_error_m":end.com().distance(expected.com()),"link_velocity_error_m_s":body_errors.1,"prior_to_secondary":pre,"nearest_event_receipts_ns":m.events}),
        m.ticks,
        end,
    ))
}
// The ordinary run remains unmodified. This independent prefix repeats its
// decisions through the last complete solve, then clips the crossing native
// solve to the common midpoint of free flight between impacts 1 and 2.
fn matched_midpoint(seq: &Sequence, p: Profile, order: bool, matched: bool) -> Result<Value> {
    let start = if matched {
        (seq.impacts[0].time_s * 1e9).ceil() as u64 + 1
    } else {
        0
    };
    let cut = ((seq.impacts[0].bracket_s[1] + seq.impacts[1].bracket_s[0]) * 0.5e9).floor() as u64;
    let (mut native, ids) = fresh_in_order(seq.state(start as f64 * 1e-9), true, 1, order)?;
    let mut runner = Runner {
        seq,
        profile: p,
        ids,
        start,
        t: start,
    };
    let mut m = Measurements {
        events: vec![[0.; 4]; seq.impacts.len()],
        ..Default::default()
    };
    for end in (1..=8).map(|i| i * 500_000).filter(|v| *v > start) {
        runner.run_until(&mut native, end, end.min(cut), Some(&mut m))?;
        if runner.t == cut {
            break;
        }
    }
    let j = if matched {
        0.
    } else {
        seq.impacts[0].corner_ns.iter().sum()
    };
    Ok(
        json!({"time_ns":cut,"calls":m.calls,"corner_ns":m.corner,"normal_ns":m.j.dot(seq.r.normal),"exact_ns":j,
        "vector_error_ns":m.j.distance(seq.r.normal*j),"spin_error_rad_s":DVec3::from(state(&native,ids)?.foot.angular_velocity).distance(seq.branches[0].spin)}),
    )
}
pub(super) fn trace_reference(seq: &Sequence) -> Vec<Snapshot> {
    (0..=8)
        .map(|tick| {
            let end = tick * 500_000;
            let mut corner = [0.; 4];
            for e in &seq.impacts {
                if tick > 0
                    && e.time_s > (tick - 1) as f64 * 0.0005
                    && e.time_s <= tick as f64 * 0.0005
                {
                    for (k, j) in e.corner_ns.iter().enumerate() {
                        corner[k] += j;
                    }
                }
            }
            Snapshot {
                state: seq.state(end as f64 * 1e-9),
                tick,
                time_us: end / 1000,
                native_steps: 0,
                selected: false,
                secondary_start: false,
                normal_impulse_ns: corner.iter().sum(),
                corner_ns: corner,
            }
        })
        .collect()
}
fn view(path: &Path, seq: &Sequence) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    for (p, name) in [
        (Profile::Uniform8, "uniform"),
        (Profile::Prospective8, "prospective"),
    ] {
        trace(path, name, &run(seq, p, false, false)?.1)?;
    }
    trace(path, "analytic", &trace_reference(seq))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"title":format!("{} certified impacts of a rocking sole",seq.impacts.len()),"lanes":[
        {"label":"uniform","name":"Uniform eight native calls","file":"traces/uniform.json"},
        {"label":"prospective","name":"State-timed eight native calls","file":"traces/prospective.json"},
        {"label":"analytic","name":"Certified event sequence","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(super) fn observe(path: &Path, sequences: &[Sequence]) -> Result<()> {
    let mut cases = vec![];
    let mut multi = false;
    for seq in sequences.iter().filter(|s| {
        s.reason == "certified" && (s.impacts.len() > 1 || s.r.case.kind == Feature::Corner)
    }) {
        let mut rows = vec![];
        for order in [false, true] {
            for profile in PROFILES {
                let mut result = run(seq, profile, order, false)?.0;
                if seq.impacts.len() > 1 {
                    result["common_midpoint"] = matched_midpoint(seq, profile, order, false)?;
                }
                rows.push(result);
                if seq.impacts.len() > 1
                    && matches!(
                        profile,
                        Profile::Uniform8 | Profile::Prospective8 | Profile::Oracle8
                    )
                {
                    let mut result = run(seq, profile, order, true)?.0;
                    result["common_midpoint"] = matched_midpoint(seq, profile, order, true)?;
                    rows.push(result);
                }
            }
        }
        if seq.impacts.len() == 5 && !multi {
            view(path, seq)?;
            multi = true;
        }
        cases.push(json!({"id":seq.r.case.id,"event_count":seq.impacts.len(),"reference":seq.impacts,"runs":rows}));
    }
    write_json(
        &path.join("native.json"),
        &json!({"cases":cases,"comparison":"full trajectory; separate cold restarts from the matched analytical state 1 ns after primary","binning":"per-point impulses assigned to the closest certified event by native solve end, separate from complete pre-secondary window and integrated trajectory metrics"}),
    )
}

#[derive(Resource)]
struct Timed {
    seq: std::sync::Arc<Sequence>,
    ids: Bodies,
    profile: Profile,
    now: u64,
}
fn timed(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut driver: ResMut<Timed>,
    mut native: ResMut<PhysicsBackend>,
) {
    if time.dt.is_zero() {
        return;
    }
    let end = driver.now + 500_000;
    let mut runner = Runner {
        seq: &driver.seq,
        profile: driver.profile,
        ids: driver.ids,
        start: 0,
        t: driver.now,
    };
    if let Err(e) = runner.run_tick(
        native.downcast_mut::<MultiFidelityBackend>().unwrap(),
        end,
        None,
    ) {
        fault.report(std::io::Error::other(e.to_string()));
        return;
    }
    driver.now = end;
}
fn measure(
    seq: std::sync::Arc<Sequence>,
    p: Profile,
    order: bool,
    expected: State,
) -> Result<[u64; 8]> {
    let (engine, ids) = fresh_in_order(seq.state(0.), true, 1, order)?;
    let mut sim = Simulation::new(OUTER)?;
    gz_physics::install_with_step(&mut sim, engine, timed)?;
    sim.world_mut().insert_resource(Timed {
        seq,
        ids,
        profile: p,
        now: 0,
    });
    let mut ticks = [0; 8];
    for dt in &mut ticks {
        let now = std::time::Instant::now();
        sim.step_exact(1)?;
        *dt = now.elapsed().as_nanos() as u64;
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    if state(native, ids)? != expected {
        return Err("timed contact-sequence endpoint differs".into());
    }
    Ok(ticks)
}
pub(super) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut samples = vec![];
    let profiles = [
        Profile::Default8,
        Profile::Uniform4,
        Profile::Uniform8,
        Profile::Uniform16,
        Profile::Prospective4,
        Profile::Prospective8,
        Profile::Scoped8,
    ];
    for id in 0..N_CASES {
        let seq = std::sync::Arc::new(Sequence::solve(Reference::solve(build(id))?)?);
        if seq.reason != "certified" || seq.impacts.len() == 1 && seq.r.case.kind != Feature::Corner
        {
            continue;
        }
        let primary = seq.r.case.arrival_ns.div_ceil(500_000) as usize - 1;
        for order in [false, true] {
            let mut endpoints = vec![];
            for p in profiles {
                let end = run(&seq, p, order, false)?.2;
                measure(seq.clone(), p, order, end)?;
                endpoints.push(end);
            }
            let mut records: [Vec<[u64; 8]>; 7] = Default::default();
            for rep in 0usize..14 {
                for pos in 0..7 {
                    let i = (rep + pos) % 7;
                    let i = if (rep / 7 + usize::from(order)).is_multiple_of(2) {
                        i
                    } else {
                        6 - i
                    };
                    records[i].push(measure(seq.clone(), profiles[i], order, endpoints[i])?);
                }
            }
            for (p, times) in profiles.into_iter().zip(records) {
                samples.push(json!({"id":id,"event_count":seq.impacts.len(),"pad_first":order,"profile":p,"primary_tick":primary+1,"tick_ns":times}));
            }
        }
    }
    write_json(
        &path.join("cost.json"),
        &json!({"scope":"complete Bevy public tick, native work and online timing decisions, no contact diagnostics","repetitions":14,"samples":samples}),
    )
}
