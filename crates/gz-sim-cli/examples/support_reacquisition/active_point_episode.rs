// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the original public actuator clock while an isolated bearing spans
//! multiple commands. Replay the authored world and observe native reactions.
use super::super::super::finite::{
    geometry, kinematics,
    load::{complete, transfer::second::boundary_xml},
};
use super::whole_cost::swizzle;
use super::*;
const HORIZON: u64 = 26;
const LABELS: [&str; 5] = [
    "onset_budget",
    "retain_budget",
    "compact_active",
    "native8192",
    "compact_onset",
];
#[derive(Default)]
struct Load {
    j: DVec3,
    normal: f64,
    tangent: DVec3,
    other: f64,
    first: Option<Duration>,
}
fn pairs(
    r: &gz_physics_multifidelity::ContactIntervalReceipt,
    a: BodyId,
    d: BodyId,
) -> Result<Load> {
    let mut z = Load::default();
    for p in &r.pairs {
        if (p.a, p.b) != (a.min(d), a.max(d)) {
            z.other += p.normal_impulse;
            continue;
        }
        let sign = if p.a == a { 1. } else { -1. };
        let tan = sign * DVec3::from(p.tangent_impulse_on_a.ok_or("no signed native friction")?);
        z.tangent += tan;
        z.j += tan + sign * DVec3::from(p.normal_impulse_on_a);
        z.normal += p.normal_impulse;
        z.first = p.first_positive_end;
    }
    Ok(z)
}
fn fallback(native: &MultiFidelityBackend, a: BodyId, d: BodyId) -> Result<Load> {
    let mut z = Load::default();
    for p in native.rigid_contact_patches() {
        for s in p.points {
            if s.normal_impulse <= 0. {
                continue;
            }
            if (p.a, p.b) != (a.min(d), a.max(d)) {
                z.other += s.normal_impulse;
                continue;
            }
            let sign = if p.a == a { 1. } else { -1. };
            let tan = sign
                * DVec3::from(
                    s.tangent_impulse_on_a
                        .ok_or("no ordinary native friction")?,
                );
            z.normal += s.normal_impulse;
            z.j += tan + sign * s.normal_impulse * DVec3::from(p.normal_on_a);
            z.tangent += tan;
            z.first = Some(DT);
        }
    }
    Ok(z)
}
pub(super) fn episode(
    base: &str,
    f: Fixture,
    start_tick: u64,
    steps: u64,
    lane: usize,
    mut timing: Option<&mut Vec<(u64, Option<u64>)>>,
) -> Result<Vec<Value>> {
    let mut xml = swizzle(
        base,
        if lane == 2 || lane == 4 {
            "compact8"
        } else {
            "subdivided8"
        },
        8,
    )?;
    if lane != 3 {
        xml = xml.replace(
            "</pair>",
            "<target_point_bounds>true</target_point_bounds></pair>",
        )
    }
    if lane == 1 || lane == 2 {
        xml = xml.replace(
            "</pair>",
            "<refine_active_point>true</refine_active_point></pair>",
        )
    }
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "active-point-episode.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let links = [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::lift::pad",
    ]
    .map(|name| {
        sim.world().iter_entities().find_map(|e| {
            if e.get::<Name>()?.0 != name {
                return None;
            }
            Some((e.get::<BodyHandle>()?.id(), e.get::<EntityId>()?.get()))
        })
    });
    let [Some((a, _)), Some((u, _)), Some((d, pad_link))] = links else {
        return Err("missing episode links".into());
    };
    let mut speed = command(f, start_tick);
    let mut rows = vec![];
    for i in 1..=steps {
        let tick = start_tick + i;
        let next = command(f, tick - 1);
        let (z, calls, cover, compacted, hits, active) = if lane == 3 {
            let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
            let native = res.downcast_mut::<MultiFidelityBackend>().unwrap();
            if next != speed {
                native.set_world_velocity(d, [0., 0., next], [0.; 3])?
            }
            let r = native.step_contact_interval(
                DT,
                8192,
                ContactSolverIterations {
                    temporal: 4,
                    pgs: None,
                    relaxation: Some(1),
                },
            )?;
            (pairs(&r, a, d)?, r.native_steps, 0, 0, 0, false)
        } else {
            if next != speed {
                queue_link_world_velocity(&mut sim, pad_link, [0., 0., next], [0.; 3])?;
            }
            if let Some(samples) = timing.as_deref_mut() {
                let wall = std::time::Instant::now();
                let cpu = super::super::secondary::cost::cpu_ns();
                sim.step_exact(1)?;
                let cost = cpu
                    .zip(super::super::secondary::cost::cpu_ns())
                    .map(|(a, b)| b - a);
                samples.push((wall.elapsed().as_nanos() as u64, cost));
            } else {
                sim.step_exact(1)?;
            }

            if sim.elapsed() != DT * i as u32 {
                return Err("changed public actuator clock".into());
            }
            if let Some(r) = gz_sim_scene::primary_contact::last_refinement(&sim)
                .filter(|r| r.sim_time == sim.elapsed())
            {
                let z = &r.interval;
                let durations = z
                    .sliding_vertices
                    .iter()
                    .chain(&z.sliding_skew_edges)
                    .map(|s| (s.start, s.duration))
                    .chain(z.sticking_points.iter().map(|s| (s.start, s.duration)))
                    .chain(z.point_transfers.iter().map(|s| (s.start, s.duration)))
                    .collect::<Vec<_>>();
                let cover = durations
                    .iter()
                    .map(|(_, s)| s.as_nanos() as u64)
                    .sum::<u64>();
                let active = durations.iter().any(|(s, t)| *s + *t == DT)
                    || z.finite_point_impacts
                        .iter()
                        .any(|p| p.resultant.elapsed == DT);
                (
                    pairs(z, a, d)?,
                    z.native_steps,
                    cover,
                    z.compact_point_tail.map_or(0, |c| c.omitted_native_steps),
                    z.finite_point_impacts.len()
                        + z.point_transfers
                            .iter()
                            .flat_map(|s| &s.intervals)
                            .filter(|s| s.impact.is_some())
                            .count(),
                    active,
                )
            } else {
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                (fallback(native, a, d)?, 1, 0, 0, 0, false)
            }
        };
        speed = next;
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let foot = native.state(a).ok_or("foot")?;
        let pad = native.state(d).ok_or("pad")?;
        let t = Tick {
            tick,
            foot: foot.into(),
            upper: native.state(u).ok_or("upper")?.into(),
            pad: pad.into(),
            normal: z.normal,
            extra: z.other,
            tangent: z.tangent,
            first: z.first,
            gap: supported_gap(foot, pad),
            calls,
        };
        let mut row = t.as_json();
        row["j"] = json!(z.j.to_array());
        row["covered_ns"] = json!(cover);
        row["omitted"] = json!(compacted);
        row["hits"] = json!(hits);
        row["point_at_end"] = json!(active);
        row["drive"] = json!(next);
        rows.push(row);
    }
    Ok(rows)
}
pub(crate) fn write(dir: &Path, start: usize, count: usize) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|end| end > 50) {
        return Err("invalid nominal range".into());
    }
    fs::create_dir_all(dir.join("cases"))?;
    for k in start..start + count {
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("missing initial landing")?;
        let (flight, defect) = kinematics(f, &c)?;
        if defect > 1e-8 {
            return Err("nonrigid primary".into());
        }
        let geo = geometry(flight)?;
        let drive = DVec3::Z * command(f, c.tick);
        let xml = boundary_xml(
            flight.moving,
            flight.relative_velocity + drive,
            flight.angular_velocity,
            flight.support,
            drive,
        )?;
        let mut refs = vec![];
        for i in 1..=HORIZON {
            if command(f, c.tick + i - 1) != drive.z {
                break;
            }
            let h = DT.as_secs_f64() * i as f64;
            let a = complete::at(flight, &geo, h, 128)?;
            let b = complete::at(flight, &geo, h, 512)?;
            let done = a["end"] == "original_clock" && b["end"] == "original_clock";
            refs.push(json!({"at":i,"coarse":a,"fine":b}));
            if !done {
                break;
            }
        }
        let mut paths = vec![];
        for i in 0..LABELS.len() {
            paths.push(episode(&xml, f, c.tick, HORIZON, i, None)?)
        }
        let initial = json!({"tick":c.tick,"time_us":c.tick*STEP_US,"foot":c.foot,"upper":c.upper,"pad":c.pad,"native_steps":0,"normal_impulse_ns":0.});
        let file = dir.join("cases").join(format!("case-{k:03}.json"));
        let temp = file.with_extension("partial");
        write_json(
            &temp,
            &json!({"format":"factorial-multi-command-material-point-v1","case":k,"source":source(k,true),"cut":snapshot(&c),"initial":initial,"drive":drive.to_array(),"profiles":LABELS,"references":refs,"paths":paths}),
        )?;
        fs::rename(temp, file)?;
        eprintln!("completed 13 ms of actual material-point commands for {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let a: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut lanes = vec![];
    for (i, label) in [
        (0, "Refine only on prospective touchdown"),
        (2, "Keep proven bearing active and coalesce complete spans"),
        (3, "Continuous 8,192 native calls"),
    ] {
        let mut frames = vec![a["initial"].clone()];
        frames.extend(
            a["paths"][i]
                .as_array()
                .ok_or("missing episode")?
                .iter()
                .cloned(),
        );
        let file = format!("traces/{}.json", LABELS[i]);
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":LABELS[i],"name":label,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"The loaded material point across 26 public commands","pair_totals_only":true,"lanes":lanes}),
    )
}

/// Counterbalance all four SDF policies on every nominal donor. State
/// verification and SDF creation are outside each real public-step timer.
pub(crate) fn cost(source_dir: &Path, output: &Path) -> Result<()> {
    use std::collections::BTreeMap;
    let mut groups = BTreeMap::<(&str, &str), (Vec<u64>, Vec<u64>)>::new();
    let mut records = vec![];
    for k in 0..50 {
        let raw: Value = serde_json::from_reader(fs::File::open(
            source_dir.join("cases").join(format!("case-{k:03}.json")),
        )?)?;
        if raw["profiles"] != json!(LABELS) {
            return Err("wrong verified active episode archive".into());
        }
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("missing fixture")?;
        let flight = kinematics(f, &c)?.0;
        let drive = DVec3::Z * command(f, c.tick);
        let xml = boundary_xml(
            flight.moving,
            flight.relative_velocity + drive,
            flight.angular_velocity,
            flight.support,
            drive,
        )?;
        for rep in 0..4 {
            for j in 0..4 {
                let offset = (rep + k + j) % 4;
                let lane = [0, 1, 2, 4][if rep % 2 == 0 { offset } else { 3 - offset }];
                let mut measured = vec![];
                let p = episode(&xml, f, c.tick, HORIZON, lane, Some(&mut measured))?;
                if serde_json::from_slice::<Value>(&serde_json::to_vec(&p)?)? != raw["paths"][lane]
                    || measured.len() != 26
                {
                    return Err(format!("changed timed episode {k}/{}", LABELS[lane]).into());
                }
                if rep == 0 {
                    continue;
                }
                for (i, (wall, cpu)) in measured.into_iter().enumerate() {
                    // Membership is fixed to the previous state of the onset-only control.
                    let class = if i == 0 {
                        "primary"
                    } else if raw["paths"][0][i - 1]["point_at_end"] == true {
                        "previously_supported"
                    } else {
                        "other"
                    };
                    for group in [class, "all"] {
                        let g = groups.entry((group, LABELS[lane])).or_default();
                        g.0.push(wall);
                        if let Some(t) = cpu {
                            g.1.push(t)
                        }
                    }
                    records.push(json!({"case":k,"local_tick":i+1,"class":class,"policy":LABELS[lane],"rep":rep-1,"wall_ns":wall,"thread_ns":cpu,"native_steps":p[i]["native_steps"]}));
                }
            }
        }
    }
    let summary=groups.into_iter().map(|((class,policy),(wall,cpu))|json!({"class":class,"policy":policy,"wall":super::super::secondary::cost::quant(wall),"thread":super::super::secondary::cost::quant(cpu)})).collect::<Vec<_>>();
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({"unit":"one Bevy update; 50 donors, 26 commands, one warmup and three measured episodes per treatment","summary":summary,"records":records}),
    )
}
