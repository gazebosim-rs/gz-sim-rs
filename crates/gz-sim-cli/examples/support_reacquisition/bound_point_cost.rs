// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the full Bevy public update on repeated cold, independently
//! archived laboratory states. World creation and verification are untimed.
use super::super::super::finite::load::transfer::second::substituted;
use super::*;
use std::collections::BTreeMap;
#[cfg(target_os = "linux")]
pub(crate) fn cpu_ns() -> Option<u64> {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
    Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
}
#[cfg(not(target_os = "linux"))]
pub(crate) fn cpu_ns() -> Option<u64> {
    None
}
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
fn check_body(s: BodyState, v: &Value) -> Result<()> {
    let q = DQuat::from_array(serde_json::from_value(v["orientation"].clone())?);
    if vector(&v["position"])?.distance(s.position.into()) > 3e-12
        || vector(&v["velocity"])?.distance(s.velocity.into()) > 3e-11
        || vector(&v["angular_velocity"])?.distance(s.angular_velocity.into()) > 3e-10
        || (q * DQuat::from_array(s.orientation).inverse())
            .xyz()
            .length()
            > 3e-12
    {
        return Err("timed SDF changed archived link pose or twist".into());
    }
    Ok(())
}
fn measure(xml: &str, required: &Value, n: u32) -> Result<(u64, Option<u64>)> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        "bound-point-cost.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut ids = vec![];
    for name in [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::lift::pad",
    ] {
        ids.push(
            sim.world()
                .iter_entities()
                .find_map(|e| {
                    (e.get::<Name>()?.0 == name)
                        .then(|| e.get::<BodyHandle>().map(|h| h.id()))
                        .flatten()
                })
                .ok_or("missing body")?,
        );
    }
    let start = std::time::Instant::now();
    let service = cpu_ns();
    sim.step_exact(1)?;
    let service = service.zip(cpu_ns()).map(|(a, b)| b - a);
    let wall = start.elapsed().as_nanos() as u64;
    let r = gz_sim_scene::primary_contact::last_refinement(&sim)
        .ok_or("SDF did not schedule this landing")?;
    let z = &r.interval;
    if z.native_steps != n
        || z.finite_point_impacts.len() != 1
        || z.sliding_vertices.len() + z.sliding_skew_edges.len() + z.sticking_points.len()
            != required["spans"].as_array().map_or(0, Vec::len)
        || z.point_transfers.len() != required["transfers"].as_array().map_or(0, Vec::len)
    {
        return Err(format!("timed SDF changed contact enrollment at {n}").into());
    }
    let solver = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing solver")?;
    for (id, key) in ids.iter().zip(["foot", "upper", "pad"]) {
        check_body(
            solver.state(*id).ok_or("missing body")?,
            &required["first"][key],
        )?;
    }
    let (a, b) = (ids[0].min(ids[2]), ids[0].max(ids[2]));
    let pair = z
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (a, b))
        .ok_or("no reaction")?;
    let sign = if ids[0] == a { 1. } else { -1. };
    let j = sign
        * (DVec3::from(pair.normal_impulse_on_a)
            + DVec3::from(pair.tangent_impulse_on_a.ok_or("missing signed friction")?));
    if j.distance(vector(&required["world_impulse_ns"])?) > 3e-11
        || (pair.normal_impulse
            - required["first"]["normal_impulse_ns"]
                .as_f64()
                .ok_or("no expected normal")?)
        .abs()
            > 3e-11
    {
        return Err("timing changed delivered impulse".into());
    }
    Ok((wall, service))
}
pub(crate) fn quant(mut values: Vec<u64>) -> Value {
    if values.is_empty() {
        return Value::Null;
    }
    values.sort_unstable();
    let at = |p: usize| values[(p * values.len()).div_ceil(100) - 1];
    json!({"n":values.len(),"median_ns":at(50),"p95_ns":at(95),"p99_ns":at(99)})
}
pub(crate) fn write(source: &Path, output: &Path) -> Result<()> {
    let mut by = BTreeMap::new();
    let mut panel = vec![];
    // First two canonical donors in each combination of mechanism, stroke
    // and tier history. Mechanism is classified at eight calls before timing.
    for k in 0..1250 {
        let file = source.join("cases").join(format!("case-{k:04}.json"));
        let a: Value = serde_json::from_reader(fs::File::open(file)?)?;
        if a["format"] != "finite-secondary-internal-transfer-v1" {
            return Err("wrong bound transfer archive".into());
        }
        let f = survey::fixture(k);
        for t in a["result"]["trials"].as_array().ok_or("no trials")? {
            let Some(rows) = t["paths"].as_array() else {
                continue;
            };
            let stroke = t["stroke"].as_u64().ok_or("no stroke")? as usize;
            let class = if !rows[5]["transfers"]
                .as_array()
                .ok_or("no transfer receipts")?
                .is_empty()
            {
                "internal_transfer"
            } else if !rows[5]["spans"]
                .as_array()
                .ok_or("no span receipts")?
                .is_empty()
            {
                "sustained"
            } else {
                "native_remainder"
            };
            let got = by.entry((class, f.switched, stroke)).or_insert(0);
            if *got < 2 {
                *got += 1;
                panel.push((k, stroke, class, rows.clone()))
            }
        }
        if panel.len() == 36 {
            break;
        }
    }
    let mut records = vec![];
    let mut groups = BTreeMap::<(&str, u32, &str), (Vec<u64>, Vec<u64>)>::new();
    for (idx, (k, stroke, class, rows)) in panel.into_iter().enumerate() {
        let f = survey::fixture(k);
        let c = survey::cuts(f)?.0[stroke - 1].ok_or("no primary")?;
        let d = laboratory(f, &c)?.ok_or("no secondary")?;
        let ns = (DT - d.clock_offset).as_nanos();
        let base = substituted(
            &d.xml,
            "max_step_size",
            &format!("{}.{:09}", ns / 1_000_000_000, ns % 1_000_000_000),
        )?;
        let mut configs = vec![];
        for (i, n) in [2, 8, 64].into_iter().enumerate() {
            let endpoint = path(
                f,
                &c,
                &d,
                p("benchmark", b(n, 1), b(8, 1)),
                Action::Endpoint,
                c.tick + 1,
            )?;
            let ordinary = json!({"first":endpoint.ticks[0].as_json(),"world_impulse_ns":endpoint.first_world_impulse.to_array()});
            for (label, params, q) in [
                ("endpoint", "", ordinary),
                (
                    "bound",
                    "<follow_sliding_vertex>true</follow_sliding_vertex><follow_sticking_point>true</follow_sticking_point>",
                    rows[4 * i].clone(),
                ),
                (
                    "transfer",
                    "<resolve_point_transfers>true</resolve_point_transfers>",
                    rows[4 * i + 1].clone(),
                ),
            ] {
                let xml = substituted(&base, "native_steps", &n.to_string())?
                    .replace("</pair>", &format!("{params}</pair>"));
                configs.push((n, label, xml, q));
            }
        }
        for rep in 0..14 {
            for step in 0..configs.len() {
                let offset = (rep + idx + step) % configs.len();
                let i = if rep % 2 == 0 {
                    offset
                } else {
                    configs.len() - 1 - offset
                };
                let (n, label, text, expected) = &configs[i];
                let (wall, cpu) = measure(text, expected, *n)?;
                if rep >= 2 {
                    let g = groups.entry((class, *n, label)).or_default();
                    g.0.push(wall);
                    if let Some(x) = cpu {
                        g.1.push(x)
                    }
                    records.push(json!({"case":k,"stroke":stroke,"class":class,"rep":rep-2,"calls":n,"policy":label,"wall_ns":wall,"thread_ns":cpu}));
                }
            }
        }
    }
    let summary=groups.into_iter().map(|((class,n,label),(wall,cpu))|json!({"class":class,"calls":n,"policy":label,"wall":quant(wall),"thread":quant(cpu)})).collect::<Vec<_>>();
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({"measured":"one public Bevy step from a fresh laboratory SDF","summary":summary,"records":records}),
    )
}

/// Equal work, compared from the same original SDF. Two donors per stroke
/// and tier history for both positive launch and unchanged controls. The rare
/// small-positive-speed category is kept as a separate measured cohort.
pub(crate) fn launch_cost(source: &Path, output: &Path) -> Result<()> {
    let mut by = BTreeMap::new();
    let mut panel = vec![];
    for k in 0..1250 {
        let a: Value = serde_json::from_reader(fs::File::open(
            source.join("cases").join(format!("case-{k:04}.json")),
        )?)?;
        if a["format"] != "finite-secondary-skew-slip-launch-v1" {
            return Err("wrong slip launch archive".into());
        }
        let f = survey::fixture(k);
        for t in a["result"]["trials"].as_array().ok_or("missing cuts")? {
            let Some(paths) = t["paths"].as_array() else {
                continue;
            };
            let r = &paths[5];
            let mut any = false;
            let mut rest = false;
            for s in r["spans"].as_array().ok_or("no spans")?.iter().chain(
                r["transfers"]
                    .as_array()
                    .ok_or("no transfers")?
                    .iter()
                    .flat_map(|t| t["pieces"].as_array().unwrap()),
            ) {
                if let Some(v) = s["slip_growth"]["initial_m_s"].as_f64() {
                    any = true;
                    rest |= v < 1e-12;
                }
            }
            let class = if rest {
                "launch"
            } else if any {
                "small_positive_slip"
            } else {
                "unchanged"
            };
            let stroke = t["stroke"].as_u64().ok_or("no stroke")? as usize;
            let n = by.entry((class, f.switched, stroke)).or_insert(0);
            if *n < 2 || class == "small_positive_slip" {
                *n += 1;
                panel.push((k, stroke, class, paths.clone()));
            }
        }
    }
    let mut records = vec![];
    let mut groups = BTreeMap::<(&str, u32, &str), (Vec<u64>, Vec<u64>)>::new();
    for (idx, (k, stroke, class, rows)) in panel.into_iter().enumerate() {
        let f = survey::fixture(k);
        let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing primary")?;
        let d = laboratory(f, &c)?.ok_or("missing secondary")?;
        let h = (DT - d.clock_offset).as_nanos();
        let base = substituted(
            &d.xml,
            "max_step_size",
            &format!("{}.{:09}", h / 1_000_000_000, h % 1_000_000_000),
        )?;
        let mut configs = vec![];
        for (i, n) in [2, 8, 64].into_iter().enumerate() {
            for (j, id) in ["spatial", "launch"].into_iter().enumerate() {
                let mut params="<resolve_point_transfers>true</resolve_point_transfers><follow_sliding_skew_edge>true</follow_sliding_skew_edge>".to_string();
                if j == 1 {
                    params.push_str("<follow_skew_slip_launch>true</follow_skew_slip_launch>");
                }
                let xml = substituted(&base, "native_steps", &n.to_string())?
                    .replace("</pair>", &format!("{params}</pair>"));
                configs.push((n, id, xml, &rows[4 * i + j]));
            }
        }
        for rep in 0..14 {
            for step in 0..configs.len() {
                let offset = (rep + idx + step) % configs.len();
                let i = if rep % 2 == 0 {
                    offset
                } else {
                    configs.len() - 1 - offset
                };
                let (n, label, xml, z) = &configs[i];
                let (wall, cpu) = measure(xml, z, *n)?;
                if rep >= 2 {
                    let g = groups.entry((class, *n, label)).or_default();
                    g.0.push(wall);
                    if let Some(x) = cpu {
                        g.1.push(x);
                    }
                    records.push(json!({"case":k,"stroke":stroke,"class":class,"rep":rep-2,"calls":n,"policy":label,"wall_ns":wall,"thread_ns":cpu}));
                }
            }
        }
    }
    let summary=groups.into_iter().map(|((class,calls,policy),(wall,cpu))|json!({"class":class,"calls":calls,"policy":policy,"wall":quant(wall),"thread":quant(cpu)})).collect::<Vec<_>>();
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({"measured":"same-state public SDF step with positive slip growth","records":records,"summary":summary}),
    )
}
