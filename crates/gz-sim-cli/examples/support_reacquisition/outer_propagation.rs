// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Follow a matched first landing through the remaining public actuator protocol.
#[path = "outer_convergence.rs"]
pub(crate) mod convergence;
#[path = "secondary_long.rs"]
pub(crate) mod secondary;
#[path = "retimed_propagation.rs"]
pub(crate) mod retimed;
#[path = "finite_propagation.rs"]
pub(crate) mod finite;
#[path = "landing_restart.rs"]
pub(crate) mod landing_restart;
#[path = "landing_persistence.rs"]
pub(crate) mod persistence;
use super::*;
use gz_physics::DynamicBackend;
use gz_physics_multifidelity::ContactSolverIterations;
const FORMAT: &str = "first-impact-propagation-v1";
const COUNT: usize = 250;
const DT: Duration = Duration::from_micros(STEP_US);
#[derive(Clone, Copy)]
struct Budget {
    n: u32,
    r: usize,
}
const fn b(n: u32, r: usize) -> Budget {
    Budget { n, r }
}
#[derive(Clone, Copy)]
struct Profile {
    label: &'static str,
    onset: Budget,
    suffix: Budget,
}
const fn p(label: &'static str, onset: Budget, suffix: Budget) -> Profile {
    Profile {
        label,
        onset,
        suffix,
    }
}
// Three branches have an identical 512-call continuation. Two more share a
// 64-call continuation. The two continuous 4,096 paths bound reference sensitivity.
const PATHS: [Profile; 8] = [
    p("8r1_then512", b(8, 1), b(512, 1)),
    p("8r8_then512", b(8, 8), b(512, 1)),
    p("4096r1_then512", b(4096, 1), b(512, 1)),
    p("4096r8_then512", b(4096, 8), b(512, 1)),
    p("8r1_then64", b(8, 1), b(64, 1)),
    p("4096r1_then64", b(4096, 1), b(64, 1)),
    p("continuous4096r1", b(4096, 1), b(4096, 1)),
    p("continuous4096r8", b(4096, 8), b(4096, 8)),
];
fn fixture(k: usize) -> Fixture {
    let i = k * 117 % COUNT;
    Fixture::new(
        "long-run",
        (165 + 5 * (i / 10)) as f64 / 1000.,
        (76 + 12 * ((i / 2) % 5)) as f64 / 100.,
        i % 2 == 1,
    )
}
#[derive(Clone, Copy)]
struct Tick {
    foot: Body,
    upper: Body,
    pad: Body,
    tick: u64,
    normal: f64,
    extra: f64,
    tangent: DVec3,
    first: Option<Duration>,
    gap: Option<f64>,
    calls: u32,
}
impl Tick {
    fn com(&self) -> DVec3 {
        0.5 * (DVec3::from(self.foot.position) + DVec3::from(self.upper.position))
    }
    fn as_json(&self) -> Value {
        json!({"tick":self.tick,"time_us":self.tick*STEP_US,"foot":self.foot,"upper":self.upper,"pad":self.pad,"native_steps":self.calls,"solver_iterations":4,
        "normal_impulse_ns":self.normal,"other_pair_normal_ns":self.extra,"tangent_on_foot_ns":self.tangent.to_array(),"first_local_ns":self.first.map(|t|t.as_nanos() as u64),"footprint_gap_m":self.gap})
    }
}
fn path(f: Fixture, cut: &ReboundFrame, profile: Profile) -> Result<Vec<Tick>> {
    Ok(advance_path(f, cut, profile, ProjectionSpan::Native)?.0)
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum ProjectionSpan {
    Native,
    FirstTick,
    AllTicks,
    RetimeFirst,
}
fn advance_path(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    scope: ProjectionSpan,
) -> Result<(Vec<Tick>, Vec<Value>)> {
    advance_path_until(f, cut, profile, scope, DURATION)
}

fn advance_path_until(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    scope: ProjectionSpan,
    last: u64,
) -> Result<(Vec<Tick>, Vec<Value>)> {
    observe_path_until(f, cut, profile, scope, last, |_, _, _| Ok(()))
}

fn observe_path_until(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    scope: ProjectionSpan,
    last: u64,
    mut observe: impl FnMut(&MultiFidelityBackend, [BodyId; 3], &Tick) -> Result<()>,
) -> Result<(Vec<Tick>, Vec<Value>)> {
    if !(cut.tick + 1..=DURATION).contains(&last) {
        return Err("propagation endpoint outside the measured protocol".into());
    }
    let mut impacts = vec![];
    let mut rig = restart(f, ADAPTIVE, cut)?;
    let mut speed = rig.last_speed;
    let lane = &mut rig.case.lane;
    if lane.tier()? != Dynamic {
        return Err("first released cut must use dynamic solver".into());
    }
    let (a, b, d) = (lane.foot, lane.upper, lane.pad);
    let mut res = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("expected local physics")?;
    let mut ticks = vec![];
    for tick in cut.tick + 1..=last {
        let next = command(f, tick - 1);
        let q = if tick == cut.tick + 1 {
            profile.onset
        } else {
            profile.suffix
        };
        let project = scope == ProjectionSpan::AllTicks
            || (matches!(
                scope,
                ProjectionSpan::FirstTick | ProjectionSpan::RetimeFirst
            ) && tick == cut.tick + 1);
        let sample = propagate_one(
            native,
            [a, b, d],
            tick,
            q,
            (next != speed).then_some(next),
            project.then_some(scope),
            &mut impacts,
        )?;
        speed = next;
        observe(native, [a, b, d], &sample)?;
        ticks.push(sample);
    }
    Ok((ticks, impacts))
}

/// One public clock interval. A changed support command may be installed only at its start.
fn propagate_one(
    native: &mut MultiFidelityBackend,
    [a, b, d]: [BodyId; 3],
    tick: u64,
    q: Budget,
    new_speed: Option<f64>,
    projection: Option<ProjectionSpan>,
    impacts: &mut Vec<Value>,
) -> Result<Tick> {
    if let Some(v) = new_speed {
        native.set_world_velocity(d, [0., 0., v], [0.; 3])?;
    }
    let iterations = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(q.r),
    };
    let registered = (a, d);
    let receipt = if let Some(scope) = projection {
        native
            .advance_box_contacts(
                DT,
                q.n,
                iterations,
                gz_physics_multifidelity::BoxContactActions {
                    finite_point_impacts: &[(a, d)],
                    retime_finite_transfer: if scope == ProjectionSpan::RetimeFirst {
                        std::slice::from_ref(&registered)
                    } else {
                        &[]
                    },
                    ..Default::default()
                },
                None,
            )?
            .interval
    } else {
        native.step_contact_interval(DT, q.n, iterations)?
    };
    for impact in &receipt.finite_point_impacts {
        let h = impact.resultant;
        impacts.push(json!({"tick":tick,"local_ns":h.elapsed.as_nanos() as u64,"feature":format!("{:?}",impact.feature),
                "point":h.point,"normal":h.normal,"impulse_ns":h.impulse_on_foot,"sticking":h.sticking}));
        if let Some((limit, dt)) = impact.next_feature_boundary {
            impacts.last_mut().unwrap()["retimed"] =
                json!({"next":format!("{limit:?}"),"after_impact_ns":dt.as_nanos() as u64});
        }
    }
    let mut normal = 0.;
    let mut extra = 0.;
    let mut first = None;
    let mut tangent = DVec3::ZERO;
    for pair in receipt.pairs {
        if (pair.a, pair.b) == (a.min(d), a.max(d)) {
            normal += pair.normal_impulse;
            first = pair.first_positive_end;
            let t = DVec3::from(pair.tangent_impulse_on_a.ok_or("unknown foot friction")?);
            tangent += if pair.a == a { t } else { -t };
        } else {
            extra += pair.normal_impulse;
        }
    }
    let foot = native.state(a).ok_or("missing foot")?;
    let upper = native.state(b).ok_or("missing upper")?;
    let pad = native.state(d).ok_or("missing support")?;
    let gap = supported_gap(foot, pad);
    Ok(Tick {
        foot: Body::from(foot),
        upper: Body::from(upper),
        pad: Body::from(pad),
        tick,
        normal,
        extra,
        tangent,
        first,
        gap,
        calls: q.n,
    })
}
fn events(path: &[Tick]) -> Value {
    let mut strokes = vec![];
    let mut all_normal = 0.;
    let mut other = 0.;
    let mut tan = DVec3::ZERO;
    let mut disjoint = None;
    let mut reentry = None;
    for t in path {
        all_normal += t.normal;
        other += t.extra;
        tan += t.tangent;
        if t.gap.is_none() {
            disjoint.get_or_insert(t.tick);
        } else if disjoint.is_some() {
            reentry.get_or_insert(t.tick);
        }
    }
    for stroke in 0..3 {
        let start = sweep::DEPART_US[stroke] / STEP_US;
        let end = sweep::DEPART_US
            .get(stroke + 1)
            .map_or(DURATION, |v| v / STEP_US);
        let mut separated = stroke == 0;
        let mut landing = None;
        let mut reaction = 0.;
        let mut window_ticks = 0;
        let mut sum = 0.;
        for t in path.iter().filter(|t| t.tick > start && t.tick <= end) {
            sum += t.normal;
            if t.normal == 0. && t.gap.is_some_and(|v| v > 1e-3) {
                separated = true;
            }
            if separated
                && landing.is_none()
                && let Some(h) = t.first
            {
                landing = Some((t.tick - 1) * STEP_US * 1000 + h.as_nanos() as u64);
            }
            if landing.is_some() && window_ticks < 10 {
                reaction += t.normal;
                window_ticks += 1;
            }
        }
        strokes.push(json!({"stroke":stroke+1,"released":separated,"first_native_end_ns":landing,"sum_first_ten_loaded_tick_window_ns":reaction,"window_public_ticks":window_ticks,"stroke_normal_ns":sum}));
    }
    json!({"strokes":strokes,"first_disjoint_tick":disjoint,"first_reentered_tick":reentry,"foot_normal_ns":all_normal,"other_pair_normal_ns":other,"foot_tangent_ns":tan.to_array()})
}
fn compare(a: &[Tick], b: &[Tick]) -> Result<Value> {
    if a.len() != b.len() || a.is_empty() {
        return Err("unmatched trajectory clocks".into());
    }
    let mut first = [None; 4];
    let mut curve = vec![];
    let mut max = 0.0_f64;
    let mut worst_tick = 0;
    let mut sq = 0.;
    let mut spin_sq = 0.;
    let mut max_loaded_diff = 0.0_f64;
    let mut presence_mismatch = 0;
    let mut com_end = 0.;
    let mut spin_end = 0.;
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        if x.tick != y.tick || x.pad != y.pad {
            // Different native grids can round prescribed positions by a few ulps.
            if x.tick != y.tick
                || DVec3::from(x.pad.position).distance(y.pad.position.into()) > 1e-9
                || x.pad.velocity != y.pad.velocity
            {
                return Err("actuator trajectory mismatch".into());
            }
        }
        let d = x.com().distance(y.com());
        let w = DVec3::from(x.foot.angular_velocity).distance(y.foot.angular_velocity.into());
        if d > max {
            max = d;
            worst_tick = x.tick;
        }
        for (idx, lim) in [1e-6, 1e-4, 1e-3, 1e-2].iter().enumerate() {
            if d > *lim {
                first[idx].get_or_insert(x.tick);
            }
        }
        sq += d * d;
        spin_sq += w * w;
        com_end = d;
        spin_end = w;
        max_loaded_diff = max_loaded_diff.max((x.normal - y.normal).abs());
        presence_mismatch += usize::from(x.gap.is_some() != y.gap.is_some());
        if matches!(i, 0 | 1 | 9 | 25 | 99 | 199 | 399 | 799 | 1199) || i + 1 == a.len() {
            curve.push(json!({"public_ticks_after_cut":i+1,"com_m":d,"spin_rad_s":w}));
        }
    }
    Ok(
        json!({"rms_com_m":(sq/a.len() as f64).sqrt(),"rms_spin_rad_s":(spin_sq/a.len() as f64).sqrt(),"max_com_m":max,"max_tick":worst_tick,"final_com_m":com_end,"final_spin_rad_s":spin_end,"max_public_reaction_difference_ns":max_loaded_diff,"footprint_availability_mismatch_ticks":presence_mismatch,"first_above_1um_100um_1mm_10mm_ticks":first,"curve":curve}),
    )
}
type Trajectories = (ReboundFrame, Vec<Vec<Tick>>);
fn collect(f: Fixture) -> Result<(Value, Option<Trajectories>)> {
    let (cuts, censored) = survey::cuts(f)?;
    let Some(cut) = cuts[0] else {
        return Ok((
            json!({"fixture":f,"donor_disjoint_tick":censored,"cut":null}),
            None,
        ));
    };
    let paths = PATHS
        .iter()
        .map(|&p| path(f, &cut, p))
        .collect::<Result<Vec<_>>>()?;
    let mut data = vec![];
    for (i, p) in PATHS.iter().enumerate() {
        let series = &paths[i];
        let last = series.last().ok_or("empty propagation")?;
        let baseline = if p.suffix.n == 64 { 5 } else { 2 };
        data.push(json!({"profile":p.label,"native_calls":series.iter().map(|s|u64::from(s.calls)).sum::<u64>(),"outcomes":events(series),"first":series[0].as_json(),"end":last.as_json(),
            "common_suffix_onset_reference":([0,1,2,3,4,5].contains(&i)).then(||compare(series,&paths[baseline])).transpose()?,
            "vs_uniform4096r1":compare(series,&paths[6])?,"vs_uniform4096r8":compare(series,&paths[7])?}));
    }
    Ok((
        json!({"fixture":f,"cut":snapshot(&cut),"cut_tick":cut.tick,"end_tick":DURATION,"donor_disjoint_tick":censored,"paths":data}),
        Some((cut, paths)),
    ))
}
pub(crate) fn survey(dir: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|&n| n <= COUNT && count > 0)
        .ok_or("long-horizon lattice: 250 cases")?;
    fs::create_dir_all(dir.join("cases"))?;
    for k in first..end {
        let path = dir.join("cases").join(format!("case-{k:03}.json"));
        if path.exists() {
            let v: Value = serde_json::from_reader(fs::File::open(path)?)?;
            if v["format"] != FORMAT
                || v["case"] != k
                || v["result"]["fixture"] != serde_json::to_value(fixture(k))?
            {
                return Err("invalid long-horizon case".into());
            }
            continue;
        }
        let started = std::time::Instant::now();
        let (result, _) = collect(fixture(k))?;
        let tmp = path.with_extension(format!("partial-{}", std::process::id()));
        write_json(&tmp, &json!({"format":FORMAT,"case":k,"result":result}))?;
        fs::rename(tmp, path)?;
        eprintln!(
            "completed propagation {k} in {:.2}s",
            started.elapsed().as_secs_f64()
        );
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    if k >= COUNT {
        return Err("invalid long-horizon case".into());
    }
    let saved: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let (actual, run) = collect(fixture(k))?;
    let canonical: Value = serde_json::from_slice(&serde_json::to_vec(&actual)?)?;
    if saved["result"] != canonical {
        return Err("long-horizon reproduction mismatch".into());
    }
    let (cut, paths) = run.ok_or("censored before primary landing")?;
    let out = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    // Include the same dynamic clocks around every lane's independently observed return.
    let mut times = std::collections::BTreeSet::from([cut.tick, DURATION]);
    for i in [0, 2, 6] {
        let p = &paths[i];
        for t in p {
            if t.tick % 16 == 0 || t.tick <= cut.tick + 10 {
                times.insert(t.tick);
            }
        }
        for r in events(p)["strokes"].as_array().unwrap() {
            if let Some(t) = r["first_native_end_ns"].as_u64() {
                let tick = t.div_ceil(STEP_US * 1000);
                times.extend(tick.saturating_sub(4).max(cut.tick)..=(tick + 8).min(DURATION));
            }
        }
    }
    for (i, label) in [
        (0, "Coarse first impact, 512 thereafter"),
        (2, "Fine first impact, 512 thereafter"),
        (6, "Continuous 4,096 calls"),
    ] {
        let p = PATHS[i];
        let mut initial = serde_json::to_value(cut.with_effort())?;
        initial
            .as_object_mut()
            .ok_or("invalid donor frame")?
            .remove("corner_ns");
        let mut frames = vec![initial];
        frames.extend(
            paths[i]
                .iter()
                .filter(|s| times.contains(&s.tick))
                .map(Tick::as_json),
        );
        write_json(
            &out.join("traces").join(format!("{}.json", p.label)),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"name":label,"label":p.label,"file":format!("traces/{}.json",p.label)}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Following the first impact through subsequent returns","case":k,"sampled_ticks":true,"pair_totals_only":true,"lanes":lanes}),
    )
}
