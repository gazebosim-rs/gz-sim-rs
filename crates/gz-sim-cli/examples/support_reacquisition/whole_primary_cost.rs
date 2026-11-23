// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time the real Bevy update on an authored pre-primary fixture, including its
//! prospective contact check. Compare the cold SDF and retained native graph.
use super::super::super::finite::{
    kinematics,
    load::transfer::second::{boundary_xml, substituted},
};
use super::super::secondary::cost::{cpu_ns, quant};
use super::*;
use std::collections::BTreeMap;
const COMPACT: [(&str, u32, usize); 4] = [
    ("subdivided8", 8, 5),
    ("compact8", 8, 5),
    ("subdivided64", 64, 7),
    ("compact64", 64, 7),
];
const DIRECTED: [(&str, u32, usize); 8] = [
    ("subdivided8", 8, 5),
    ("subdivided8_directed", 8, 5),
    ("compact8", 8, 5),
    ("compact8_directed", 8, 5),
    ("subdivided64", 64, 7),
    ("subdivided64_directed", 64, 7),
    ("compact64", 64, 7),
    ("compact64_directed", 64, 7),
];
const PROFILES: [(&str, u32, usize); 7] = [
    ("native8", 8, 2),
    ("endpoint8", 8, 3),
    ("launch2", 2, 1),
    ("launch8", 8, 5),
    ("subdivided8", 8, 5),
    ("subdivided_retimed8", 8, 6),
    ("native8192", 8192, 9),
];
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
pub(super) fn swizzle(xml: &str, name: &str, n: u32) -> Result<String> {
    let mut s = substituted(xml, "native_steps", &n.to_string())?;
    if name.starts_with("native") {
        return Ok(substituted(&s, "project_finite_point", "false")?.replace(
            "</pair>",
            "<detect_finite_arrival>true</detect_finite_arrival></pair>",
        ));
    }
    if name != "endpoint8" {
        let mut options = String::from(
            "<resolve_point_transfers>true</resolve_point_transfers><follow_sliding_skew_edge>true</follow_sliding_skew_edge><follow_skew_slip_launch>true</follow_skew_slip_launch>",
        );
        if name.starts_with("subdivided") || name.starts_with("compact") {
            options.push_str("<subdivide_point_bounds>true</subdivide_point_bounds>")
        }
        if name.starts_with("compact") {
            options.push_str("<compact_point_tail>true</compact_point_tail>")
        }
        if name.ends_with("_directed") {
            options.push_str("<target_point_bounds>true</target_point_bounds>");
        }
        if name.contains("retimed") {
            options.push_str("<retime_finite_transfer>true</retime_finite_transfer>")
        }
        options.push_str("</pair>");
        s = s.replace("</pair>", &options);
    }
    Ok(s)
}
fn update(xml: &str, n: u32, c: &ReboundFrame, drive: DVec3) -> Result<(Value, u64, Option<u64>)> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        "matched-primary.sdf",
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
            (e.get::<Name>()?.0 == name)
                .then(|| e.get::<BodyHandle>().map(|h| h.id()))
                .flatten()
        })
    });
    let [Some(a), Some(u), Some(d)] = ids else {
        return Err("missing primary links".into());
    };
    let mut deck = c.pad;
    deck.velocity = drive.to_array();
    for (id, want) in [(a, c.foot), (u, c.upper), (d, deck)] {
        let backend = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing solver")?;
        let got = backend.state(id).ok_or("missing body")?;
        let dq = DQuat::from_array(want.orientation) * DQuat::from_array(got.orientation).inverse();
        if DVec3::from(want.position).distance(got.position.into()) > 2e-11
            || DVec3::from(want.velocity).distance(got.velocity.into()) > 2e-11
            || dq.xyz().length() > 2e-11
            || DVec3::from(want.angular_velocity).distance(got.angular_velocity.into()) > 2e-11
        {
            return Err("authored pre-primary donor mismatch".into());
        }
    }
    let timer = std::time::Instant::now();
    let cpu = cpu_ns();
    sim.step_exact(1)?;
    let elapsed = cpu.zip(cpu_ns()).map(|(a, b)| b - a);
    let wall = timer.elapsed().as_nanos() as u64;
    let receipt = gz_sim_scene::primary_contact::last_refinement(&sim)
        .ok_or("missing registered contact step")?;
    let z = &receipt.interval;
    let [pair] = receipt.requested_pairs.as_slice() else {
        return Err("unexpected registered pairs".into());
    };
    if pair.foot != a
        || pair.support != d
        || z.native_steps
            .checked_add(z.compact_point_tail.map_or(0, |s| s.omitted_native_steps))
            != Some(n)
    {
        return Err("wrong original clock or pair".into());
    }
    let j = DVec3::from(pair.normal_impulse_on_foot)
        + DVec3::from(
            pair.tangent_impulse_on_foot
                .ok_or("missing delivered friction")?,
        );
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let mut spans = z
        .sliding_vertices
        .iter()
        .chain(&z.sliding_skew_edges)
        .map(|s| (s.start, s.duration))
        .chain(z.sticking_points.iter().map(|s| (s.start, s.duration)))
        .chain(z.point_transfers.iter().map(|s| (s.start, s.duration)))
        .collect::<Vec<_>>();
    spans.sort_unstable();
    let mut at = z.finite_point_impacts.first().map(|s| s.resultant.elapsed);
    for (start, len) in &spans {
        if at != Some(*start) {
            at = None;
            break;
        }
        at = Some(*start + *len);
    }
    let first = z
        .finite_point_impacts
        .first()
        .map(|h| json!({"ns":h.resultant.elapsed.as_nanos(),"j":h.resultant.impulse_on_foot}));
    let bodies = [a, u, d].map(|id| Body::from(backend.state(id).expect("known live body")));
    let center = (DVec3::from(bodies[0].position) + DVec3::from(bodies[1].position)) * 0.5;
    Ok((
        json!({"calls":z.native_steps,"compact":z.compact_point_tail.map(|t|(t.start.as_nanos(),t.duration.as_nanos(),t.omitted_native_steps)),"bodies":bodies,"com":center.to_array(),"j":j.to_array(),"normal":pair.normal_impulse,"first":first,"spans":spans.len(),"whole":at==Some(DT),"later_hits":z.point_transfers.iter().flat_map(|r|&r.intervals).filter(|p|p.impact.is_some()).count()}),
        wall,
        elapsed,
    ))
}
pub(crate) enum CostField {
    Primary,
    Compact,
    Directed,
}
pub(crate) fn write(source: &Path, ordinary: &Path, output: &Path, study: CostField) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut groups = BTreeMap::<(&str, &str), (Vec<u64>, Vec<u64>)>::new();
    let mut records = vec![];
    let mut states = vec![];
    for k in 0..1250 {
        let a: Value = serde_json::from_reader(fs::File::open(
            source.join("cases").join(format!("case-{k:04}.json")),
        )?)?;
        if a["format"] != "subdivided-original-primary-v1" {
            return Err("expected complete subdivided primary field".into());
        }
        let prior: Value = serde_json::from_reader(fs::File::open(
            ordinary.join("cases").join(format!("case-{k:04}.json")),
        )?)?;
        let f = survey::fixture(k);
        for t in a["result"]["trials"]
            .as_array()
            .ok_or("no primary trials")?
        {
            let reference = &t["mechanics"][1];
            if reference["end"] != "original_clock" || t["mechanics"][0]["end"] != "original_clock"
            {
                continue;
            }
            let stroke = t["stroke"].as_u64().unwrap() as usize;
            let warm = &t["paths"][5];
            let spans = warm["spans"].as_array().ok_or("no spans")?;
            let Some(first) = warm["hits"].as_array().and_then(|h| h.first()) else {
                continue;
            };
            let mut at = Some((first["at_s"].as_f64().unwrap() * 1e9).round() as u64);
            for p in spans {
                if at != p[0].as_u64() {
                    at = None;
                    break;
                }
                at = Some(p[0].as_u64().unwrap() + p[1].as_u64().unwrap())
            }
            let class = if at == Some(500000) {
                "whole"
            } else if spans.is_empty() {
                "native_remainder"
            } else {
                "partial"
            };
            let quota = counts.entry((class, stroke, f.switched)).or_insert(0);
            if *quota >= 2 {
                continue;
            }
            *quota += 1;
            let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing donor")?;
            let flight = kinematics(f, &c)?.0;
            let drive = DVec3::Z * command(f, c.tick);
            let xml = boundary_xml(
                flight.moving,
                flight.relative_velocity + drive,
                flight.angular_velocity,
                flight.support,
                drive,
            )?;
            let mut paths = vec![];
            let profiles = match study {
                CostField::Primary => PROFILES.as_slice(),
                CostField::Compact => COMPACT.as_slice(),
                CostField::Directed => DIRECTED.as_slice(),
            };
            for &(name, n, ix) in profiles {
                let cfg = swizzle(&xml, name, n)?;
                let (state, _, _) = update(&cfg, n, &c, drive)
                    .map_err(|e| format!("fixture {k}/{stroke} {name}: {e}"))?;
                let warm = if (name.starts_with("subdivided") || name.starts_with("compact"))
                    && !name.ends_with("_directed")
                {
                    t
                } else {
                    prior["result"]["trials"]
                        .as_array()
                        .ok_or("missing original cuts")?
                        .iter()
                        .find(|s| s["stroke"] == stroke)
                        .ok_or("wrong primary")?
                };
                if warm["cut"] != t["cut"] || warm["geometry"] != t["geometry"] {
                    return Err("different warm originals".into());
                }
                let want = &warm["paths"][ix];
                if !name.starts_with("native") {
                    let h = &state["first"];
                    if h["ns"] != t["geometry"]["hi_ns"]
                        || vector(&h["j"])?
                            .distance(vector(&t["geometry"]["instantaneous_impulse_ns"])?)
                            > 1e-8
                    {
                        return Err(
                            "authored initial projection differs from physical contact".into()
                        );
                    }
                }
                let target = vector(&reference["com"])? + DT.as_secs_f64() * drive;
                let wc = (vector(&want["first"]["foot"]["position"])?
                    + vector(&want["first"]["upper"]["position"])?)
                    * 0.5;
                states.push(json!({"case":k,"stroke":stroke,"warm_class":class,"policy":name,"state":state,"cold_com_error_m":vector(&state["com"])?.distance(target),"cold_j_error_ns":vector(&state["j"])?.distance(vector(&reference["j"])?),"warm_com_difference_m":vector(&state["com"])?.distance(wc),"warm_j_difference_ns":vector(&state["j"])?.distance(vector(&want["j"])?),"warm_profile":want["profile"]}));
                paths.push((name, n, cfg, state));
            }
            for rep in 0..10 {
                for offset in 0..paths.len() {
                    let i = (k + rep + offset) % paths.len();
                    let (name, n, cfg, expected) =
                        &paths[if rep % 2 == 0 { i } else { paths.len() - 1 - i }];
                    let (v, wall, cpu) = update(cfg, *n, &c, drive)?;
                    if v != *expected {
                        return Err("a repeated SDF gave different physical results".into());
                    }
                    if rep >= 2 {
                        let g = groups.entry((class, name)).or_default();
                        g.0.push(wall);
                        if let Some(n) = cpu {
                            g.1.push(n)
                        }
                        records.push(json!({"case":k,"stroke":stroke,"warm_class":class,"policy":name,"rep":rep-2,"wall_ns":wall,"thread_ns":cpu}));
                    }
                }
            }
            if counts.values().sum::<usize>() == 36 {
                break;
            }
        }
        if counts.values().sum::<usize>() == 36 {
            break;
        }
    }
    let summary=groups.into_iter().map(|((class,name),(wall,cpu))|json!({"warm_class":class,"policy":name,"wall":quant(wall),"thread":quant(cpu)})).collect::<Vec<_>>();
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({"sample_unit":"one full authored Bevy update; two warmups and eight timed fresh worlds per donor and policy","summary":summary,"states":states,"records":records}),
    )
}

pub(crate) fn fixture_sdf(output: &Path, k: usize, stroke: usize) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid primary fixture index".into());
    }
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("no donor")?;
    let flight = kinematics(f, &c)?.0;
    let drive = DVec3::Z * command(f, c.tick);
    let base = boundary_xml(
        flight.moving,
        flight.relative_velocity + drive,
        flight.angular_velocity,
        flight.support,
        drive,
    )?;
    let xml=swizzle(&base,"compact8",8)?.replace("Matched first outer-support landing, fixture 0. The skew edges of the foot and deck meet during the first public tick.",&format!("Matched pre-primary state, fixture {k}, stroke {stroke}. The actuator advances once over the 500 microsecond landing."));
    fs::create_dir_all(output)?;
    fs::write(output.join(format!("case-{k:04}-stroke-{stroke}.sdf")), xml)?;
    Ok(())
}
