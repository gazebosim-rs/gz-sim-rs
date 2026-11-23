// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Three-stroke continuations from a matched secondary landing. Keep the
//! live native graph on four sparse interventions, then isolate graph effects
//! with fresh native copies of two of the same first-boundary states.
use super::*;
const TAG: &str = "internal-contact-transfer-three-strokes-v1";
const NAMES: [&str; 9] = [
    "native8_warm512",
    "bound8_warm512",
    "transfer8_warm512",
    "transfer2_warm512",
    "continuous4096",
    "continuous8192",
    "native8_cold512",
    "transfer8_cold512",
    "mechanics_cold512",
];
struct Run {
    data: Value,
    paths: Vec<Vec<Tick>>,
}
fn run(k: usize) -> Result<Run> {
    let f = fixture(source(k));
    let c = survey::cuts(f)?.0[0].ok_or("missing primary")?;
    let mut data = json!({"format":TAG,"case":k,"source":source(k),"fixture":f,"cut":snapshot(&c),"cut_tick":c.tick,"end_tick":DURATION});
    let Some(d) = laboratory(f, &c)? else {
        return Ok(Run {
            data,
            paths: vec![],
        });
    };
    let (diagnostic, mechanical) =
        physical::mechanics_at(&d, f, &c, 512, (DT - d.clock_offset).as_secs_f64())?;
    let (_, coarse) = physical::mechanics_at(&d, f, &c, 128, (DT - d.clock_offset).as_secs_f64())?;
    data["mechanics"] = diagnostic;
    data["offset_ns"] = json!(d.clock_offset.as_nanos() as u64);
    let (Some(mechanical), Some(coarse)) = (mechanical, coarse) else {
        return Ok(Run {
            data,
            paths: vec![],
        });
    };
    if mechanical.com().distance(coarse.com()) > 1e-7
        || DVec3::from(mechanical.foot.angular_velocity)
            .distance(coarse.foot.angular_velocity.into())
            > 1e-4
        || (mechanical.normal - coarse.normal).abs() > 1e-5
        || mechanical.tangent.distance(coarse.tangent) > 1e-5
    {
        data["unresolved_mesh"] = json!(true);
        return Ok(Run {
            data,
            paths: vec![],
        });
    }
    let mut paths = vec![];
    let mut receipts = vec![];
    for (i, (n, tail, action)) in [
        (8, 512, Action::Native),
        (8, 512, Action::Both),
        (8, 512, Action::Transfer),
        (2, 512, Action::Transfer),
        (4096, 4096, Action::Native),
        (8192, 8192, Action::Native),
    ]
    .into_iter()
    .enumerate()
    {
        let b = path(
            f,
            &c,
            &d,
            p(NAMES[i], b(n, 1), b(tail, 1)),
            action,
            DURATION,
        )?;
        if i == 0 {
            data["initial"] = b.initial.clone()
        } else if data["initial"] != b.initial {
            return Err("unequal native starting snapshot".into());
        }
        receipts.push(json!({"impulse":b.impulse,"spans":b.vertices,"transfers":b.transfers,"world_impulse_ns":b.first_world_impulse.to_array()}));
        paths.push(b.ticks);
    }
    for i in [0, 2] {
        paths.push(physical::thaw(f, paths[i][0])?);
    }
    paths.push(physical::thaw(f, mechanical)?);
    let mut rows = vec![];
    for (i, a) in paths.iter().enumerate() {
        let diffs = [0, 1, 4, 5, 8]
            .map(|j| compare(a, &paths[j]))
            .into_iter()
            .collect::<Result<Vec<_>>>()?;
        rows.push(json!({"profile":NAMES[i],"first":a[0].as_json(),"end":a.last().unwrap().as_json(),"native_calls":a.iter().map(|t|u64::from(t.calls)).sum::<u64>(),"outcomes":events(a),"comparisons":diffs,
            "delivered":receipts.get(i)}));
    }
    data["paths"] = json!(rows);
    Ok(Run { data, paths })
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|v| count > 0 && *v <= 50)
        .ok_or("invalid nominal fixture range")?;
    for sub in ["cases", "trajectories", "frames"] {
        fs::create_dir_all(dir.join(sub))?;
    }
    for k in first..end {
        let name = format!("case-{k:03}.json");
        let file = dir.join("cases").join(&name);
        let frames = dir.join("frames").join(&name);
        let traces = dir.join("trajectories").join(&name);
        if file.exists() {
            let x: Value = serde_json::from_reader(fs::File::open(&file)?)?;
            if x["format"] != TAG
                || x["case"] != k
                || (!x["paths"].is_null() && !(frames.exists() && traces.exists()))
            {
                return Err("incomplete long transfer archive".into());
            }
            continue;
        }
        let r = run(k)?;
        let save = |p: &Path, obj: &Value| -> Result<()> {
            let tmp = p.with_extension(format!("partial-{}", std::process::id()));
            write_json(&tmp, obj)?;
            fs::rename(tmp, p)?;
            Ok(())
        };
        if !r.paths.is_empty() {
            let mut times = std::collections::BTreeSet::from([DURATION]);
            let tick = r.paths[0][0].tick;
            times.extend(tick..=tick + 10);
            for i in [0, 2, 3, 4, 5, 8] {
                for e in r.data["paths"][i]["outcomes"]["strokes"]
                    .as_array()
                    .ok_or("missing strokes")?
                {
                    if let Some(at) = e["first_native_end_ns"].as_u64() {
                        let at = at.div_ceil(500000);
                        times.extend(at.saturating_sub(4)..=at + 8)
                    }
                }
            }
            let mut saved = vec![];
            let mut series = vec![];
            for lane in &r.paths {
                let mut f = vec![r.data["initial"].clone()];
                f.extend(
                    lane.iter()
                        .filter(|x| x.tick % 16 == 0 || times.contains(&x.tick))
                        .map(Tick::as_json),
                );
                saved.push(f);
                series.push(
                    lane.iter()
                        .map(|x| {
                            (
                                x.com().to_array(),
                                x.foot.angular_velocity,
                                x.normal,
                                x.tangent.to_array(),
                            )
                        })
                        .collect::<Vec<_>>(),
                );
            }
            save(&frames, &json!(saved))?;
            save(
                &traces,
                &json!({"first_tick":tick,"profiles":NAMES,"series":series}),
            )?;
        }
        save(&file, &r.data)?;
        eprintln!("integrated all internal-transfer continuations {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let name = format!("case-{k:03}.json");
    let raw: Value = serde_json::from_reader(fs::File::open(dir.join("cases").join(&name))?)?;
    let frames: Value = serde_json::from_reader(fs::File::open(dir.join("frames").join(name))?)?;
    if raw["case"] != k || raw["format"] != TAG {
        return Err("wrong saved clock".into());
    }
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut lanes = vec![];
    for (i, label) in [
        (1, "Eight calls following each individual material point"),
        (2, "Eight calls resolving internal bearing changes"),
        (5, "Continuous 8,192-call reference"),
    ] {
        let f = frames[i].as_array().ok_or("missing replay")?;
        if f.first() != Some(&raw["initial"])
            || f.get(1) != Some(&raw["paths"][i]["first"])
            || f.last() != Some(&raw["paths"][i]["end"])
        {
            return Err("saved poses do not match simulation".into());
        }
        let id = NAMES[i];
        let file = format!("traces/{id}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":f}),
        )?;
        lanes.push(json!({"label":id,"name":label,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Continuing after a resolved internal change of bearing","sampled_ticks":true,"pair_totals_only":true,"lanes":lanes}),
    )
}
