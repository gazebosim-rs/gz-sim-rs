// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Production native interval API driven at the public Bevy clock.
use super::*;
use gz_physics_multifidelity::{
    BoxTimingReceipt, ContactIntervalReceipt, ContactSegment, ContactSolverIterations,
    ContactSubstep,
};
use std::time::Instant;
#[path = "inertia_impact.rs"]
pub(crate) mod inertia_impact;
#[path = "impact_production_sdf.rs"]
mod sdf;

#[derive(Clone, Copy, Serialize)]
struct Profile {
    calls: u32,
    prospective: bool,
}
const PROFILES: [Profile; 4] = [
    Profile {
        calls: 4,
        prospective: false,
    },
    Profile {
        calls: 4,
        prospective: true,
    },
    Profile {
        calls: 8,
        prospective: false,
    },
    Profile {
        calls: 8,
        prospective: true,
    },
];
impl Profile {
    fn work(self) -> ContactSegment {
        ContactSegment {
            duration: OUTER,
            native_steps: self.calls,
            iterations: ContactSolverIterations {
                temporal: 4,
                pgs: Some(1),
                relaxation: Some(16),
            },
        }
    }
}
/// Signed height of the lowest native sole vertex above the native pad plane.
fn lowest_gap(foot: gz_physics::BodyState, pad: gz_physics::BodyState) -> f64 {
    let n = DQuat::from_array(pad.orientation) * DVec3::Z;
    let delta = DVec3::from(foot.position) - DVec3::from(pad.position);
    let rotation = DQuat::from_array(foot.orientation);
    corners()
        .into_iter()
        .map(|r| (delta + rotation * r).dot(n) - PAD_HALF[2])
        .fold(f64::INFINITY, f64::min)
}
#[derive(Default)]
struct Observed {
    corners: [f64; 4],
    elapsed: Vec<u64>,
    normal_by_end: Vec<f64>,
    minimum_gap_by_end: Vec<f64>,
    first_load: Option<u64>,
}
impl Observed {
    fn record(&mut self, s: ContactSubstep<'_>, ids: Bodies) {
        let end = s.elapsed.as_nanos() as u64;
        self.elapsed.push(end);
        let mut total = 0.;
        let foot = s.state(ids.foot).unwrap();
        let q = DQuat::from_array(foot.orientation);
        let pad = s.state(ids.pad).unwrap();
        self.minimum_gap_by_end.push(lowest_gap(foot, pad));
        for p in s.contact_patches() {
            let sign = if (p.a, p.b) == (ids.foot, ids.pad) {
                true
            } else if (p.b, p.a) == (ids.foot, ids.pad) {
                false
            } else {
                continue;
            };
            for x in p.points {
                let local = q.conjugate() * DVec3::from(if sign { x.lever_a } else { x.lever_b });
                self.corners[2 * usize::from(local.x > 0.) + usize::from(local.y > 0.)] +=
                    x.normal_impulse;
                total += x.normal_impulse;
                if x.normal_impulse > 0. {
                    self.first_load.get_or_insert(end);
                }
            }
        }
        self.normal_by_end.push(total);
    }
}
#[derive(Resource)]
struct Drive {
    ids: Bodies,
    profile: Profile,
    capture: bool,
    results: Option<(ContactIntervalReceipt, Option<BoxTimingReceipt>, Observed)>,
}
fn advance(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut d: ResMut<Drive>,
    mut backend: ResMut<PhysicsBackend>,
) {
    if time.dt.is_zero() {
        return;
    }
    let e = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    let ids = d.ids;
    let pair = [(ids.foot, ids.pad)];
    let w = d.profile.work();
    let mut data = Observed::default();
    let mut read = |s: ContactSubstep<'_>| data.record(s, ids);
    let result = match (d.profile.prospective, d.capture) {
        (true, true) => e
            .observe_prospective_box_interval(
                time.dt,
                w.native_steps,
                w.iterations,
                &pair,
                &pair,
                &mut read,
            )
            .map(|(r, _, t)| (r, Some(t))),
        (true, false) => e
            .step_prospective_box_interval(time.dt, w.native_steps, w.iterations, &pair, &pair)
            .map(|(r, _, t)| (r, Some(t))),
        (false, true) => e
            .observe_contact_schedule(time.dt, &[w], &pair, &mut read)
            .map(|(r, _)| (r, None)),
        (false, false) => e
            .step_contact_interval_with_box_features(time.dt, w.native_steps, w.iterations, &pair)
            .map(|(r, _)| (r, None)),
    };
    match result {
        Ok((r, t)) => d.results = d.capture.then_some((r, t, data)),
        Err(e) => fault.report(e),
    }
}
fn setup(
    seq: &Sequence,
    profile: Profile,
    order: bool,
    capture: bool,
) -> Result<(Simulation, Bodies)> {
    let (engine, ids) = fresh_in_order(seq.state(0.), true, 1, order)?;
    let mut sim = Simulation::new(OUTER)?;
    gz_physics::install_with_step(&mut sim, engine, advance)?;
    sim.world_mut().insert_resource(Drive {
        ids,
        profile,
        capture,
        results: None,
    });
    Ok((sim, ids))
}
fn backend(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn frame(tick: u64, state: State, calls: u32, changed: bool, corner: [f64; 4]) -> Snapshot {
    Snapshot {
        tick,
        time_us: tick * 500,
        native_steps: calls,
        selected: changed,
        secondary_start: false,
        normal_impulse_ns: corner.iter().sum(),
        corner_ns: corner,
        state,
    }
}
fn score(seq: &Sequence, end: State, j: DVec3) -> Value {
    let reference = seq.state(0.004);
    let total = seq.impacts.iter().flat_map(|e| e.corner_ns).sum::<f64>();
    json!({"impulse_ns":j.to_array(),"impulse_vector_error_ns":j.distance(total*seq.r.normal),"com_error_m":end.com().distance(reference.com()),"spin_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(reference.foot.angular_velocity.into())})
}
fn trial(seq: &Sequence, p: Profile, order: bool) -> Result<(Value, Vec<Snapshot>, State)> {
    let (mut sim, ids) = setup(seq, p, order, true)?;
    let mut frames = vec![frame(0, state(backend(&sim), ids)?, 0, false, [0.; 4])];
    let mut ticks = vec![];
    let mut vector = DVec3::ZERO;
    let mut first = None;
    let mut retimed = 0;
    let mut declined = 0;
    for tick in 1..=8 {
        sim.step_exact(1)?;
        let (r, t, observed) = sim
            .world_mut()
            .resource_mut::<Drive>()
            .results
            .take()
            .ok_or("missing committed native interval")?;
        if r.native_steps != p.calls
            || observed.elapsed.len() != p.calls as usize
            || observed.elapsed.last() != Some(&500000)
        {
            return Err("public boundary or native budget mismatch".into());
        }
        let reaction = r
            .pairs
            .iter()
            .find(|c| (c.a, c.b) == (ids.foot, ids.pad) || (c.b, c.a) == (ids.foot, ids.pad));
        if let Some(c) = reaction {
            vector += DVec3::from(c.normal_impulse_on_a) * if c.a == ids.foot { 1. } else { -1. };
            if (c.normal_impulse - observed.corners.iter().sum::<f64>()).abs() > 1e-10
                || c.first_positive_end.map(|x| x.as_nanos() as u64) != observed.first_load
            {
                return Err(
                    "native point totals or onset differ from integrated pair receipt".into(),
                );
            }
        }
        if let Some(us) = observed.first_load {
            first.get_or_insert((tick - 1) * 500000 + us);
        }
        let mut changed = 0;
        if let Some(t) = &t {
            let mut end = 0;
            let boundaries: Vec<_> = t
                .durations
                .iter()
                .map(|d| {
                    end += d.as_nanos() as u64;
                    end
                })
                .collect();
            if boundaries != observed.elapsed {
                return Err("planned duration differs from observed end".into());
            }
            changed = t.retimed;
            retimed += changed;
            declined += usize::from(t.decline.is_some());
        }
        ticks.push(json!({"end_ns":observed.elapsed,"first_positive_end_ns":observed.first_load,"retimed":changed,"decline":t.as_ref().and_then(|t|t.decline.map(|r|format!("{r:?}"))),"decline_after":t.as_ref().and_then(|t|t.decline_after)}));
        frames.push(frame(
            tick,
            state(backend(&sim), ids)?,
            p.calls,
            changed > 0,
            observed.corners,
        ));
    }
    let end = frames.last().unwrap().state;
    Ok((
        json!({"profile":p,"pad_first":order,"first_positive_end_ns":first,"onset_end_error_ns":first.map(|t|t as i64-seq.r.case.arrival_ns as i64),"retimed_calls":retimed,"declined_ticks":declined,"end_state":end,"metrics":score(seq,end,vector),"ticks":ticks}),
        frames,
        end,
    ))
}
fn cases() -> Result<Vec<Sequence>> {
    let mut v = vec![];
    for id in 0..N_CASES {
        let seq = Sequence::solve(Reference::solve(build(id))?)?;
        if seq.reason == "certified"
            && (seq.impacts.len() > 1 || seq.r.case.kind == Feature::Corner)
        {
            v.push(seq);
        }
    }
    Ok(v)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut rows = vec![];
    for seq in cases()? {
        let mut trials = vec![];
        for order in [false, true] {
            for p in PROFILES {
                let (r, frames, _) = trial(&seq, p, order)?;
                if seq.r.case.id == 84 && !order && p.calls == 8 {
                    trace(
                        path,
                        if p.prospective {
                            "prospective"
                        } else {
                            "uniform"
                        },
                        &frames,
                    )?;
                }
                trials.push(r);
            }
        }
        if seq.r.case.id == 84 {
            trace(path, "analytic", &native::trace_reference(&seq))?;
        }
        rows.push(json!({"id":seq.r.case.id,"reference":seq.impacts,"trials":trials}));
    }
    write_json(&path.join("native.json"), &json!({"cases":rows}))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"title":"Live native box timing through five successive impacts","lanes":[
        {"label":"uniform","name":"Uniform eight native calls","file":"traces/uniform.json"},
        {"label":"prospective","name":"Production live eight calls","file":"traces/prospective.json"},
        {"label":"analytic","name":"Certified event sequence","file":"traces/analytic.json","analytical":true}]}),
    )?;
    sdf::write(path)
}
fn measure(seq: &Sequence, p: Profile, order: bool, expected: State) -> Result<[u64; 8]> {
    let (mut sim, ids) = setup(seq, p, order, false)?;
    let mut out = [0; 8];
    for v in &mut out {
        let start = Instant::now();
        sim.step_exact(1)?;
        *v = start.elapsed().as_nanos() as u64;
    }
    if state(backend(&sim), ids)? != expected {
        return Err("unobserved native timing endpoint differs from recorded state".into());
    }
    Ok(out)
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for seq in cases()? {
        for order in [false, true] {
            let mut ends = vec![];
            for p in PROFILES {
                let end = trial(&seq, p, order)?.2;
                measure(&seq, p, order, end)?;
                ends.push(end);
            }
            let mut samples: [Vec<[u64; 8]>; 4] = Default::default();
            for repeat in 0..12 {
                for pos in 0..4 {
                    let i = (repeat + pos) % 4;
                    let i = if (repeat / 4 + usize::from(order)) % 2 == 0 {
                        i
                    } else {
                        3 - i
                    };
                    samples[i].push(measure(&seq, PROFILES[i], order, ends[i])?);
                }
            }
            for (i, s) in samples.into_iter().enumerate() {
                rows.push(json!({"id":seq.r.case.id,"event_count":seq.impacts.len(),"primary_tick":seq.r.case.arrival_ns.div_ceil(500000),"pad_first":order,"profile":PROFILES[i],"tick_ns":s}));
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"samples":rows}))?;
    sdf::cost(path)
}
