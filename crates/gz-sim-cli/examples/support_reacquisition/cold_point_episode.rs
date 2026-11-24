// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Re-discover a bearing from its present material kinematics after import.
use super::{super::secondary::physical::cold_xml, episode::episode, handoff::prefix, *};
pub(crate) fn write(dir: &Path, episodes: &Path, start: usize, count: usize) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|n| n > 50) {
        return Err("invalid donor range".into());
    }
    fs::create_dir_all(dir.join("cases"))?;
    for k in start..start + count {
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("missing nominal donor")?;
        let file = format!("case-{k:03}.json");
        let e: Value =
            serde_json::from_reader(fs::File::open(episodes.join("cases").join(&file))?)?;
        let prefix = prefix(f, &c, &e)?;
        let mut trials = Vec::new();
        for n in [1, 2, 8] {
            let t = prefix[n - 1];
            let xml = cold_xml(t)?;
            let with = xml.replace(
                "</pair>",
                "<detect_initial_point>true</detect_initial_point></pair>",
            );
            let mut paths = Vec::new();
            for (scene, lane) in [(&xml, 1), (&with, 1), (&xml, 2), (&with, 2), (&xml, 3)] {
                let p = episode(scene, f, t.tick, 26, lane, None)?;
                paths.push(p);
            }
            trials.push(json!({"prefix":n,"boundary":t.as_json(),"paths":paths}));
        }
        let dst = dir.join("cases").join(file);
        let tmp = dst.with_extension("partial");
        write_json(
            &tmp,
            &json!({"format":"cold-material-bearing-v1","case":k,"source":source(k,true),"cut":e["cut"],"policies":["fixed_history","fixed_detect","compact_history","compact_detect","native8192"],"trials":trials}),
        )?;
        fs::rename(tmp, dst)?;
        eprintln!("checked cold point at three public cuts {k}");
    }
    Ok(())
}

pub(crate) fn replay(dir: &Path, episodes: &Path, k: usize, n: usize) -> Result<()> {
    if k >= 50 || ![1, 2, 8].contains(&n) {
        return Err("invalid cold restart".into());
    }
    let name = format!("case-{k:03}.json");
    let e: Value = serde_json::from_reader(fs::File::open(episodes.join("cases").join(&name))?)?;
    let d: Value = serde_json::from_reader(fs::File::open(dir.join("cases").join(&name))?)?;
    let f = fixture(source(k, true));
    let c = survey::cuts(f)?.0[0].ok_or("missing cut")?;
    let t = prefix(f, &c, &e)?[n - 1];
    let folder = dir.join(format!("case-{k:03}-cut-{n}"));
    let xml=super::whole_cost::swizzle(&cold_xml(t)?,"compact8",8)?.replace("</pair>","<target_point_bounds>true</target_point_bounds><refine_active_point>true</refine_active_point><detect_initial_point>true</detect_initial_point></pair>");
    let xml=xml.replace("Matched first outer-support landing, fixture 0. The skew edges of the foot and deck meet during the first public tick.",&format!("A supported foot is imported after {n} physical contact ticks from nominal donor {k}. Both material points have matching normal speed."));
    fs::create_dir_all(folder.join("traces"))?;
    fs::write(folder.join("world.sdf"), xml)?;
    let trials = d["trials"].as_array().ok_or("missing cold trials")?;
    let trial = trials
        .iter()
        .find(|r| r["prefix"] == n)
        .ok_or("missing boundary")?;
    let mut initial = trial["boundary"].clone();
    initial["normal_impulse_ns"] = json!(0.);
    initial["tangent_on_foot_ns"] = json!([0., 0., 0.]);
    initial["first_local_ns"] = Value::Null;
    let mut lanes = Vec::new();
    for (i, label) in [
        (2, "Ordinary selection"),
        (3, "Inspect initial bearing"),
        (4, "8192 native calls"),
    ] {
        let mut frames = vec![initial.clone()];
        frames.extend(
            trial["paths"][i]
                .as_array()
                .ok_or("missing public trajectory")?
                .iter()
                .cloned(),
        );
        let file = format!("traces/native-{i}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":format!("policy{i}"),"name":label,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Cold local restart on an already loaded material point","pair_totals_only":true,"lanes":lanes}),
    )
}

fn field_xml(t: Tick, retime: bool) -> Result<String> {
    let base = cold_xml(t)?;
    Ok(if retime {
        base.replace(
            "</pair>",
            "<retime_finite_transfer>true</retime_finite_transfer></pair>",
        )
    } else {
        base
    })
}

/// Reuse original actuator phases and the two independently converged meshes;
/// exclusions retain the reason in the denominator.
pub(crate) fn field(
    dir: &Path,
    primary: &Path,
    start: usize,
    count: usize,
    retime: bool,
) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|n| n > 1250) {
        return Err("invalid cold point field range".into());
    }
    fs::create_dir_all(dir.join("cases"))?;
    for k in start..start + count {
        let f = survey::fixture(k);
        let file = format!("case-{k:04}.json");
        let d: Value = serde_json::from_reader(fs::File::open(primary.join("cases").join(&file))?)?;
        let cuts = survey::cuts(f)?.0;
        let mut trials = Vec::new();
        for r in d["result"]["trials"]
            .as_array()
            .ok_or("missing primary landings")?
        {
            let stroke = r["stroke"].as_u64().ok_or("missing stroke")? as usize;
            let c = cuts[stroke - 1].ok_or("missing primary cut")?;
            let cutsame: Value = serde_json::from_slice(&serde_json::to_vec(&snapshot(&c))?)?;
            if cutsame != r["cut"] {
                return Err("matched primary differs".into());
            }
            let reference = &r["mechanics"][1];
            if !super::handoff::qualified(&r["mechanics"][0], reference)? {
                trials.push(json!({"stroke":stroke,"status":"unqualified primary"}));
                continue;
            }
            let (mechanics, t) = super::whole_long::mechanical(f, &c)?;
            let m: Value = serde_json::from_slice(&serde_json::to_vec(&mechanics)?)?;
            if m != *reference {
                return Err("independent primary did not repeat".into());
            }
            if command(f, c.tick + 1) != command(f, c.tick) {
                trials.push(json!({"stroke":stroke,"status":"changed drive"}));
                continue;
            }
            let flight = super::super::super::finite::kinematics(f, &c)?.0;
            let g = &r["geometry"];
            let controls = [128, 512]
                .map(|n| super::super::super::finite::load::complete::at(flight, g, 0.001, n))
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
            let ok = super::handoff::qualified(&controls[0], &controls[1])?;
            let xml = field_xml(t, retime)?;
            let detected = xml.replace(
                "</pair>",
                "<detect_initial_point>true</detect_initial_point></pair>",
            );
            let mut paths = Vec::new();
            for (scene, lane) in [
                (&xml, 1),
                (&detected, 1),
                (&xml, 2),
                (&detected, 2),
                (&xml, 3),
            ] {
                paths.push(episode(scene, f, t.tick, 1, lane, None)?.remove(0));
            }
            trials.push(json!({"stroke":stroke,"status":if ok{"qualified"}else{"unqualified successor"},"boundary":t.as_json(),"controls":controls,"paths":paths}));
        }
        let dst = dir.join("cases").join(file);
        let tmp = dst.with_extension("partial");
        write_json(
            &tmp,
            &json!({"format":"cold-material-successor-field-v1","case":k,"fixture":f,"trials":trials,"retime_finite_transfer":retime}),
        )?;
        fs::rename(tmp, dst)?;
        if k % 25 == 0 {
            eprintln!("cold material field {k}");
        }
    }
    Ok(())
}

fn field_boundary(k: usize, stroke: usize) -> Result<(Fixture, Tick)> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid field landing".into());
    }
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing primary donor")?;
    Ok((f, super::whole_long::mechanical(f, &c)?.1))
}
pub(crate) fn field_replay(dir: &Path, k: usize, stroke: usize) -> Result<()> {
    let (f, t) = field_boundary(k, stroke)?;
    let raw: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let trial = &raw["trials"][stroke - 1];
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    fs::create_dir_all(folder.join("traces"))?;
    let xml = field_xml(t, raw["retime_finite_transfer"] == true)?;
    let with = xml.replace(
        "</pair>",
        "<detect_initial_point>true</detect_initial_point></pair>",
    );
    let authored=super::whole_cost::swizzle(&with,"compact8",8)?.replace("</pair>","<target_point_bounds>true</target_point_bounds><refine_active_point>true</refine_active_point></pair>")
       .replace("Matched first outer-support landing, fixture 0. The skew edges of the foot and deck meet during the first public tick.",&format!("Cold material bearing at the public boundary following fixture {k}, stroke {stroke}."));
    fs::write(folder.join("world.sdf"), authored)?;
    let mut start = t.as_json();
    start["normal_impulse_ns"] = json!(0.);
    start["tangent_on_foot_ns"] = json!([0., 0., 0.]);
    start["first_local_ns"] = Value::Null;
    let mut lanes = Vec::new();
    for (scene, mode, ix, name) in [
        (&xml, 2, 2, "Ordinary selection"),
        (&with, 2, 3, "Inspect initial bearing"),
        (&xml, 3, 4, "8192 native calls"),
    ] {
        let p = episode(scene, f, t.tick, 26, mode, None)?;
        let actual: Value = serde_json::from_slice(&serde_json::to_vec(&p[0])?)?;
        if actual != trial["paths"][ix] {
            return Err("playback disagrees with field solve".into());
        }
        let mut frames = vec![start.clone()];
        frames.extend(p);
        let file = format!("traces/lane-{ix}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":format!("lane{ix}"),"name":name,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Resuming the outer-support trajectory at a loaded material point","pair_totals_only":true,"lanes":lanes}),
    )
}

pub(crate) fn cost(source: &Path, output: &Path) -> Result<()> {
    use std::collections::BTreeMap;
    let mut quotas = BTreeMap::new();
    let mut records = Vec::new();
    for k in 0..1250 {
        let raw: Value = serde_json::from_reader(fs::File::open(
            source.join("cases").join(format!("case-{k:04}.json")),
        )?)?;
        for trial in raw["trials"].as_array().ok_or("missing field landings")? {
            if trial["status"] != "qualified" {
                continue;
            }
            let paths = &trial["paths"];
            let class = if paths[0]["covered_ns"] == 500000 {
                "complete"
            } else if paths[1]["covered_ns"] == 500000 {
                "recovered"
            } else {
                "partial_or_native"
            };
            let stroke = trial["stroke"].as_u64().ok_or("missing stroke")? as usize;
            let switched = raw["fixture"]["switched"].as_bool().ok_or("missing tier")?;
            let quota = quotas.entry((class, stroke, switched)).or_insert(0);
            if *quota >= 2 {
                continue;
            }
            *quota += 1;
            let (f, t) = field_boundary(k, stroke)?;
            let xml = field_xml(t, raw["retime_finite_transfer"] == true)?;
            let with = xml.replace(
                "</pair>",
                "<detect_initial_point>true</detect_initial_point></pair>",
            );
            for rep in 0..10 {
                for shift in 0..4 {
                    let ix = (shift + rep + k) % 4;
                    let ix = if rep % 2 == 0 { ix } else { 3 - ix };
                    let mut times = Vec::new();
                    let p = episode(
                        if ix % 2 == 0 { &xml } else { &with },
                        f,
                        t.tick,
                        1,
                        if ix < 2 { 1 } else { 2 },
                        Some(&mut times),
                    )?;
                    let norm: Value = serde_json::from_slice(&serde_json::to_vec(&p[0])?)?;
                    if norm != paths[ix] || times.len() != 1 {
                        return Err("changed cold timing treatment".into());
                    }
                    if rep >= 2 {
                        records.push(json!({"case":k,"stroke":stroke,"tier_switched":switched,"class":class,"policy":(["fixed_history","fixed_detect","compact_history","compact_detect"][ix]),"rep":rep-2,"wall_ns":times[0].0,"thread_ns":times[0].1}));
                    }
                }
            }
        }
        if quotas.values().sum::<usize>() == 36 {
            break;
        }
    }
    let mut groups = BTreeMap::<(&str, &str), (Vec<u64>, Vec<u64>)>::new();
    for r in &records {
        let policy = r["policy"].as_str().ok_or("missing treatment")?;
        for class in [r["class"].as_str().unwrap(), "all"] {
            let (wall, cpu) = groups.entry((class, policy)).or_default();
            wall.push(r["wall_ns"].as_u64().unwrap());
            if let Some(t) = r["thread_ns"].as_u64() {
                cpu.push(t);
            }
        }
    }
    let summary=groups.into_iter().map(|((class,policy),(wall,cpu))|json!({"class":class,"policy":policy,"wall":super::super::secondary::cost::quant(wall),"thread":super::super::secondary::cost::quant(cpu)})).collect::<Vec<_>>();
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({"unit":"one complete cold Bevy update","donors":quotas.values().sum::<usize>(),"summary":summary,"records":records}),
    )
}

/// Recreate the same cold world for each timed step. Validate the full result
/// (including pose, impulse, solver coverage and actuator phase) after timing.
pub(crate) fn fixture_cost(
    source: &Path,
    output: &Path,
    k: usize,
    stroke: usize,
    step: u32,
) -> Result<()> {
    const WARMUP: usize = 32;
    const SAMPLES: usize = 256;
    let (f, t) = field_boundary(k, stroke)?;
    let raw: Value = serde_json::from_reader(fs::File::open(
        source.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let trial = &raw["trials"][stroke - 1];
    if trial["stroke"] != stroke || trial["status"] != "qualified" {
        return Err("missing qualified fixture/stroke for timing".into());
    }
    let canonical =
        |v: &Value| -> Result<Value> { Ok(serde_json::from_slice(&serde_json::to_vec(v)?)?) };
    if canonical(&t.as_json())? != trial["boundary"] {
        return Err("timing input differs from the archived boundary".into());
    }
    let with = field_xml(t, raw["retime_finite_transfer"] == true)?.replace(
        "</pair>",
        "<detect_initial_point>true</detect_initial_point></pair>",
    );
    let expected = if step == 1 {
        vec![trial["paths"][3].clone()]
    } else {
        let replay = source.join(format!("case-{k:04}-stroke-{stroke}/traces/lane-3.json"));
        let trace: Value = serde_json::from_reader(fs::File::open(replay)?)?;
        let frames = trace["frames"]
            .as_array()
            .ok_or("missing compact trajectory")?;
        if frames.get(1) != Some(&trial["paths"][3]) {
            return Err("trajectory differs from first resumed step".into());
        }
        frames
            .get(1..=step as usize)
            .ok_or("step outside trace")?
            .to_vec()
    };
    let mut times = Vec::with_capacity(WARMUP + SAMPLES);
    for _ in 0..WARMUP + SAMPLES {
        let mut sample = vec![];
        let rows = episode(&with, f, t.tick, step.into(), 2, Some(&mut sample))?;
        if serde_json::from_slice::<Vec<Value>>(&serde_json::to_vec(&rows)?)? != expected {
            return Err("timing changed the archived compact detection result".into());
        }
        times.push(*sample.last().ok_or("missing full-step timing")?);
    }
    let times = &times[WARMUP..];
    let quant = super::super::secondary::cost::quant;
    let wall = quant(times.iter().map(|p| p.0).collect());
    let thread = quant(times.iter().filter_map(|p| p.1).collect());
    fs::create_dir_all(output)?;
    write_json(
        &output.join("cost.json"),
        &json!({
            "case": k, "stroke": stroke, "public_tick": t.tick + u64::from(step),
            "unit": "one complete cold Bevy update", "policy": "compact_detect",
            "warmups": WARMUP, "wall": wall, "thread": thread,
            "samples_wall_thread_ns": times,
        }),
    )
}
