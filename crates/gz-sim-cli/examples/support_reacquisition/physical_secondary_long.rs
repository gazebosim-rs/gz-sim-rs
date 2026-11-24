// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare whole secondary landing intervals. Re-import every branch at the
//! same following public boundary, so inherited native row history is controlled.
use super::super::super::finite::load::transfer::second::{after::sequence, boundary_xml};
use super::*;
use gz_physics::FlightBox;
const TAG: &str = "cold-boundary-secondary-mechanics-v1";
const NAMES: [&str; 5] = [
    "native8_cold512",
    "endpoint8_cold512",
    "native4096_cold512",
    "native8192_cold512",
    "mechanics_cold512",
];
fn vector(a: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(a.clone())?))
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn state(
    b: Body,
) -> BodyState {
    BodyState {
        position: b.position,
        velocity: b.velocity,
        angular_velocity: b.angular_velocity,
        orientation: b.orientation,
    }
}
fn bx(b: Body, half: [f64; 3]) -> FlightBox {
    FlightBox {
        center: b.position.into(),
        half_extent: half.into(),
        orientation: DQuat::from_array(b.orientation),
    }
}
pub(super) fn mechanics_at(
    d: &LaboratoryCut,
    f: Fixture,
    c: &ReboundFrame,
    n: u32,
    h: f64,
) -> Result<(Value, Option<Tick>)> {
    let r = sequence::path(d.relative_flight, d.relative_end, d.line, h, n)?;
    if r["end"] != "original_clock" {
        return Ok((r, None));
    }
    let deck_v = DVec3::Z * command(f, c.tick);
    let translation = deck_v * (d.geometric_time_s + h);
    let center = vector(&r["com_m"])? + translation;
    let foot = vector(&r["foot_m"])? + translation;
    let speed = vector(&r["velocity_m_s"])? + deck_v;
    let w = vector(&r["spin_rad_s"])?;
    let orientation: [f64; 4] = serde_json::from_value(r["orientation_xyzw"].clone())?;
    let rigid = |p: DVec3| Body {
        position: p.to_array(),
        velocity: (speed + w.cross(p - center)).to_array(),
        angular_velocity: w.to_array(),
        orientation,
    };
    let a = rigid(foot);
    let u = rigid(2. * center - foot);
    let pad = Body {
        position: (d.deck.center + deck_v * h).to_array(),
        velocity: deck_v.to_array(),
        angular_velocity: [0.; 3],
        orientation: d.deck.orientation.to_array(),
    };
    let t = Tick {
        foot: a,
        upper: u,
        pad,
        tick: c.tick + 1,
        normal: r["normal_impulse_ns"]
            .as_f64()
            .ok_or("missing mechanical force integral")?,
        extra: 0.,
        tangent: vector(&r["impulse_ns"])? - vector(&r["normal_impulse_vector_ns"])?,
        first: Some(d.clock_offset),
        gap: supported_gap(state(a), state(pad)),
        calls: 0,
    };
    Ok((r, Some(t)))
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn thaw(
    f: Fixture,
    t: Tick,
) -> Result<Vec<Tick>> {
    thaw_with_grid(f, t, 512)
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn thaw_with_grid(
    f: Fixture,
    t: Tick,
    calls: u32,
) -> Result<Vec<Tick>> {
    let (mut sim, ids) = cold_boundary(t)?;
    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("requires local world")?;
    let [a, u, d] = ids;
    let mut speed = t.pad.velocity[2];
    let mut ticks = vec![t];
    let mut unexpected = vec![];
    for tick in t.tick + 1..=DURATION {
        let next = command(f, tick - 1);
        ticks.push(propagate_one(
            native,
            [a, u, d],
            tick,
            b(calls, 1),
            (next != speed).then_some(next),
            None,
            &mut unexpected,
        )?);
        speed = next;
    }
    if !unexpected.is_empty() {
        return Err("unexpected continued impact intervention".into());
    }
    Ok(ticks)
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn cold_xml(
    t: Tick,
) -> Result<String> {
    let u = (DVec3::from(t.foot.velocity) + DVec3::from(t.upper.velocity)) * 0.5;
    boundary_xml(
        bx(t.foot, FOOT_HALF),
        u,
        t.foot.angular_velocity.into(),
        bx(t.pad, PAD_HALF),
        t.pad.velocity.into(),
    )
}
/// Rebuild an identical cold registered world including every link pose and twist.
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn cold_boundary(
    t: Tick,
) -> Result<(Simulation, [BodyId; 3])> {
    let xml = cold_xml(t)?;
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "post-secondary.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let ids = [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::lift::pad",
    ]
    .map(|name| {
        sim.world().iter_entities().find_map(|e| {
            if e.get::<Name>()?.0 == name {
                Some(e.get::<BodyHandle>()?.id())
            } else {
                None
            }
        })
    });
    let [Some(a), Some(u), Some(d)] = ids else {
        return Err("missing second-boundary links".into());
    };
    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("requires local world")?;
    for (id, want) in [(a, t.foot), (u, t.upper), (d, t.pad)] {
        let now = native.state(id).ok_or("missing re-imported body")?;
        let dq = DQuat::from_array(now.orientation) * DQuat::from_array(want.orientation).inverse();
        if DVec3::from(now.position).distance(want.position.into()) > 2e-11
            || dq.xyz().length() > 2e-11
            || DVec3::from(now.velocity).distance(want.velocity.into()) > 2e-11
            || DVec3::from(now.angular_velocity).distance(want.angular_velocity.into()) > 2e-11
        {
            return Err("post-impact re-import failed to preserve a link pose or twist".into());
        }
    }
    Ok((sim, [a, u, d]))
}
struct Study {
    report: Value,
    lanes: Vec<Vec<Tick>>,
}
fn run(k: usize) -> Result<Study> {
    let f = fixture(source(k));
    let c = survey::cuts(f)?.0[0].ok_or("no primary donor")?;
    let mut report = json!({"format":TAG,"case":k,"source":source(k),"cut":snapshot(&c),"cut_tick":c.tick,"end_tick":DURATION,"fixture":f});
    let Some(d) = laboratory(f, &c)? else {
        return Ok(Study {
            report,
            lanes: vec![],
        });
    };
    let (fine, physical) = mechanics_at(&d, f, &c, 512, (DT - d.clock_offset).as_secs_f64())?;
    let (coarse, comp) = mechanics_at(&d, f, &c, 128, (DT - d.clock_offset).as_secs_f64())?;
    report["offset_ns"] = json!(d.clock_offset.as_nanos() as u64);
    report["mechanics"] = fine;
    report["mechanics128"] = coarse;
    let (Some(physical), Some(comp)) = (physical, comp) else {
        return Ok(Study {
            report,
            lanes: vec![],
        });
    };
    // Report every refusal. Mesh agreement is an empirical accuracy check on
    // the propagated state, separate from the proof at each instantaneous impact.
    let cmp = |t: Tick| {
        (
            t.com(),
            DVec3::from(t.foot.angular_velocity),
            t.normal,
            t.tangent,
        )
    };
    let (x, w, j, ft) = cmp(physical);
    let (y, z, kj, kt) = cmp(comp);
    if x.distance(y) > 1e-7
        || w.distance(z) > 1e-4
        || (j - kj).abs() > 1e-5
        || ft.distance(kt) > 1e-5
    {
        report["unresolved_mesh"] = json!(true);
        return Ok(Study {
            report,
            lanes: vec![],
        });
    }
    let mut lanes = vec![];
    for (i, (n, ac)) in [
        (8, Action::Native),
        (8, Action::Endpoint),
        (4096, Action::Native),
        (8192, Action::Native),
    ]
    .into_iter()
    .enumerate()
    {
        let p = path(f, &c, &d, p(NAMES[i], b(n, 1), b(512, 1)), ac, c.tick + 1)?;
        if i == 0 {
            report["initial"] = p.initial.clone()
        } else if report["initial"] != p.initial {
            return Err("unequal secondary starts".into());
        }
        lanes.push(thaw(f, p.ticks[0])?);
    }
    lanes.push(thaw(f, physical)?);
    let mut paths = vec![];
    for (i, lane) in lanes.iter().enumerate() {
        let diffs = [0, 2, 3, 4]
            .map(|j| compare(lane, &lanes[j]))
            .into_iter()
            .collect::<Result<Vec<_>>>()?;
        paths.push(json!({"profile":NAMES[i],"first":lane[0].as_json(),"end":lane.last().unwrap().as_json(),"outcomes":events(lane),
            "native_calls":lane.iter().map(|t|u64::from(t.calls)).sum::<u64>(),"comparisons":diffs}));
    }
    report["paths"] = json!(paths);
    Ok(Study { report, lanes })
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|n| count > 0 && *n <= 50)
        .ok_or("invalid nominal case range")?;
    fs::create_dir_all(dir.join("cases"))?;
    for k in first..end {
        let file = dir.join("cases").join(format!("case-{k:03}.json"));
        if file.exists() {
            let v: Value = serde_json::from_reader(fs::File::open(&file)?)?;
            if v["format"] != TAG || v["case"] != k {
                return Err("wrong post-impact archive".into());
            }
            continue;
        }
        let data = run(k)?;
        let tmp = file.with_extension(format!("partial-{}", std::process::id()));
        write_json(&tmp, &data.report)?;
        fs::rename(tmp, file)?;
        eprintln!("verified cold-boundary fixture {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let d: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let trial = run(k)?;
    if serde_json::from_slice::<Value>(&serde_json::to_vec(&trial.report)?)? != d {
        return Err("mechanical replay changed".into());
    }
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let cut = d["cut_tick"].as_u64().ok_or("no clock")?;
    let mut times = std::collections::BTreeSet::from([DURATION]);
    times.extend(cut + 1..=cut + 12);
    for &i in &[0, 3, 4] {
        for r in d["paths"][i]["outcomes"]["strokes"]
            .as_array()
            .ok_or("no returns")?
        {
            if let Some(t) = r["first_native_end_ns"].as_u64() {
                let t = t.div_ceil(STEP_US * 1000);
                times.extend(t.saturating_sub(4)..=t + 8)
            }
        }
    }
    let mut names = vec![];
    for (i, label) in [
        (0, "Eight ordinary landing calls"),
        (4, "Piecewise rigid landing solution"),
        (3, "8,192 ordinary landing calls"),
    ] {
        let lane = trial.lanes.get(i).ok_or("no complete reference")?;
        let mut frames = vec![d["initial"].clone()];
        frames.extend(
            lane.iter()
                .filter(|s| s.tick % 16 == 0 || times.contains(&s.tick))
                .map(Tick::as_json),
        );
        let id = NAMES[i];
        let f = format!("traces/{id}.json");
        write_json(
            &folder.join(&f),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        let mut meta = json!({"name":label,"label":id,"file":f});
        if i == 4 {
            meta["analytical_ticks"] = json!([cut + 1]);
        }
        names.push(meta);
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Whole landing response with identical cold restarts","pair_totals_only":true,"sampled_ticks":true,"lanes":names}),
    )
}
