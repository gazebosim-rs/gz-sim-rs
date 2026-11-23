// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Perturb only the first true impact, or admit finite points throughout three strokes.
//! A separate 50-cell nominal-drive campaign measures three finer continuous references.
use super::*;
#[path = "cold_point_episode.rs"]
pub(crate) mod cold_point;
#[path = "cold_reference.rs"]
pub(crate) mod cold_reference;
#[path = "compact_primary.rs"]
pub(crate) mod compact;
#[path = "active_point_episode.rs"]
pub(crate) mod episode;
#[path = "matched_fine_suffix.rs"]
pub(crate) mod fine_suffix;
#[path = "physical_handoff.rs"]
pub(crate) mod handoff;
#[path = "deep_primary_onset.rs"]
pub(crate) mod onset;
#[path = "whole_primary_policy.rs"]
pub(crate) mod whole;
#[path = "whole_primary_cost.rs"]
pub(crate) mod whole_cost;
#[path = "whole_primary_long.rs"]
pub(crate) mod whole_long;
const LONG: &str = "finite-three-stroke-v1";
const DEEP: &str = "nominal-three-stroke-deep-v1";
const COMPARE: [(Profile, ProjectionSpan); 7] = [
    (PATHS[0], ProjectionSpan::Native),
    (
        p("finite8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::FirstTick,
    ),
    (
        p("finite2_then512", b(2, 1), b(512, 1)),
        ProjectionSpan::FirstTick,
    ),
    (p("native8", b(8, 1), b(8, 1)), ProjectionSpan::Native),
    (p("finite8", b(8, 1), b(8, 1)), ProjectionSpan::AllTicks),
    (p("native64", b(64, 1), b(64, 1)), ProjectionSpan::Native),
    (p("finite64", b(64, 1), b(64, 1)), ProjectionSpan::AllTicks),
];
const GRIDS: [(Profile, ProjectionSpan); 5] = [
    (p("8192", b(8192, 1), b(8192, 1)), ProjectionSpan::Native),
    (p("16384", b(16384, 1), b(16384, 1)), ProjectionSpan::Native),
    (p("32768", b(32768, 1), b(32768, 1)), ProjectionSpan::Native),
    COMPARE[0],
    COMPARE[1],
];
fn source(k: usize, deep: bool) -> usize {
    if !deep {
        return k;
    }
    let i = k * 17 % 50;
    let target = 10 * (i / 2) + 4 + i % 2;
    (0..COUNT).find(|x| x * 117 % COUNT == target).unwrap()
}
struct Study {
    record: Value,
    traces: Vec<Vec<Tick>>,
    cut: Option<ReboundFrame>,
}
fn measured(k: usize, deep: bool) -> Result<Study> {
    let f = fixture(source(k, deep));
    let (cuts, disjoint) = survey::cuts(f)?;
    let Some(cut) = cuts[0] else {
        return Ok(Study {
            record: json!({"fixture":f,"cut":null,"disjoint":disjoint}),
            traces: vec![],
            cut: None,
        });
    };
    let modes = if deep { &GRIDS[..] } else { &COMPARE[..] };
    let mut traces = vec![];
    let mut records = vec![];
    for &(mode, span) in modes {
        let (run, impacts) = advance_path(f, &cut, mode, span)?;
        records.push(json!({"profile":mode.label,"native_calls":run.iter().map(|t|u64::from(t.calls)).sum::<u64>(),
            "outcomes":events(&run),"end":run.last().unwrap().as_json(),"first":run[0].as_json(),"impacts":impacts}));
        traces.push(run);
    }
    for (i, r) in records.iter_mut().enumerate() {
        if deep {
            r["against"] = json!(
                (0..3)
                    .map(|j| compare(&traces[i], &traces[j]))
                    .collect::<Result<Vec<_>>>()?
            );
        } else {
            let peer = [0, 0, 0, 3, 3, 5, 5][i];
            r["same_native_budget_control"] = json!(modes[peer].0.label);
            r["trajectory_change"] = compare(&traces[i], &traces[peer])?;
        }
    }
    Ok(Study {
        record: json!({"fixture":f,"cut":snapshot(&cut),"cut_tick":cut.tick,"paths":records,"end_tick":DURATION}),
        traces,
        cut: Some(cut),
    })
}
pub(crate) fn survey(dir: &Path, first: usize, count: usize, deep: bool) -> Result<()> {
    let (tag, limit) = if deep { (DEEP, 50) } else { (LONG, COUNT) };
    let end = first
        .checked_add(count)
        .filter(|n| count > 0 && *n <= limit)
        .ok_or("invalid campaign range")?;
    fs::create_dir_all(dir.join("cases"))?;
    if deep {
        fs::create_dir_all(dir.join("trajectories"))?;
    }
    for k in first..end {
        let dst = dir.join("cases").join(format!("case-{k:03}.json"));
        let src = source(k, deep);
        let trajectory = dir.join("trajectories").join(format!("case-{k:03}.json"));
        if dst.exists() {
            let d: Value = serde_json::from_reader(fs::File::open(&dst)?)?;
            if d["format"] != tag
                || d["case"] != k
                || d["source"] != src
                || d["result"]["fixture"] != serde_json::to_value(fixture(src))?
                || (deep && !d["result"]["cut"].is_null() && !trajectory.is_file())
            {
                return Err("invalid existing long-horizon case".into());
            }
            continue;
        }
        let now = std::time::Instant::now();
        let study = measured(k, deep)?;
        if deep && study.cut.is_some() {
            // Preserve public COM, spin and signed reactions. Later endpoint
            // comparisons can reuse this high-cost continuous integration.
            let paths=study.traces.iter().zip(GRIDS).map(|(trace,(mode,_))|json!({"profile":mode.label,"ticks":trace.iter().map(|t|json!([t.tick,t.com().to_array(),t.foot.angular_velocity,t.normal,t.tangent.to_array(),t.first.map(|v|v.as_nanos()as u64)])).collect::<Vec<_>>() })).collect::<Vec<_>>();
            let tmp = trajectory.with_extension(format!("partial-{}", std::process::id()));
            write_json(&tmp, &json!({"case":k,"source":src,"paths":paths}))?;
            fs::rename(tmp, trajectory)?;
        }
        let tmp = dst.with_extension(format!("partial-{}", std::process::id()));
        write_json(
            &tmp,
            &json!({"format":tag,"case":k,"source":src,"result":study.record}),
        )?;
        fs::rename(tmp, dst)?;
        eprintln!("completed {tag} {k} in {:.2}s", now.elapsed().as_secs_f64());
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize, deep: bool) -> Result<()> {
    let saved: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let study = measured(k, deep)?;
    if serde_json::from_slice::<Value>(&serde_json::to_vec(&study.record)?)? != saved["result"] {
        return Err("long-horizon replay disagrees with archive".into());
    }
    let cut = study.cut.ok_or("missing primary donor")?;
    let indexes = if deep { [0, 1, 2] } else { [0, 1, 4] };
    let modes = if deep { &GRIDS[..] } else { &COMPARE[..] };
    let names = if deep {
        indexes.map(|i| format!("Continuous {} native calls", modes[i].0.label))
    } else {
        [
            "Native impact then 512 calls",
            "Finite point impact then 512 calls",
            "Finite points throughout, eight calls",
        ]
        .map(str::to_owned)
    };
    let lanes = std::array::from_fn(|j| {
        (
            modes[indexes[j]].0.label,
            names[j].as_str(),
            study.traces[indexes[j]].as_slice(),
        )
    });
    let title = if deep {
        "Reference sensitivity at nominal drive"
    } else {
        "Finite point impact propagated through three strokes"
    };
    write_comparison(dir, k, &cut, lanes, title, &[])
}

pub(super) fn write_comparison(
    dir: &Path,
    k: usize,
    cut: &ReboundFrame,
    paths: [(&str, &str, &[Tick]); 3],
    title: &str,
    captures: &[(&str, u64)],
) -> Result<()> {
    let mut capture_ticks: std::collections::BTreeMap<_, _> = captures.iter().copied().collect();
    let mut times: std::collections::BTreeSet<_> = capture_ticks.values().copied().collect();
    for (i, (_, _, run)) in paths.iter().enumerate() {
        times.extend(
            run.iter()
                .filter(|t| t.tick % 16 == 0 || t.tick <= cut.tick + 12 || t.tick == DURATION)
                .map(|t| t.tick),
        );
        for (stroke, label) in events(run)["strokes"].as_array().unwrap().iter().zip([
            "first_return",
            "second_return",
            "third_return",
        ]) {
            if let Some(ns) = stroke["first_native_end_ns"].as_u64() {
                let n = ns.div_ceil(STEP_US * 1000);
                if i == 0 {
                    capture_ticks.insert(label, n);
                }
                times.extend(n.saturating_sub(4)..=n + 8);
            }
        }
    }
    capture_ticks.insert("final_endpoint", DURATION);
    if let Some(sample) = paths[1]
        .2
        .iter()
        .find(|t| times.contains(&t.tick) && t.gap.is_none())
    {
        capture_ticks.insert("first_sample_without_middle_footprint", sample.tick);
    }
    let out = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    for (label, name, run) in paths {
        let mut initial = serde_json::to_value(cut.with_effort())?;
        initial.as_object_mut().unwrap().remove("corner_ns");
        let mut frames = vec![initial];
        frames.extend(
            run.iter()
                .filter(|t| times.contains(&t.tick))
                .map(Tick::as_json),
        );
        let file = format!("traces/{label}.json");
        write_json(
            &out.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":label,"name":name,"file":file}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"pair_totals_only":true,"sampled_ticks":true,"capture_ticks":capture_ticks,"case":k,"title":title,"lanes":lanes}),
    )
}
