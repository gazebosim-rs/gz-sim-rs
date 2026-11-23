// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reexecute the first 13 ms of the three long-horizon branches on their original 512-call suffix.
use super::*;

const FORMAT: &str = "common-512-landing-prefix-v2";
const ENDPOINTS: [u64; 4] = [1, 2, 10, 26];
const TREATMENTS: [(Profile, ProjectionSpan); 5] = [
    (PATHS[0], ProjectionSpan::Native),
    (PATHS[1], ProjectionSpan::Native),
    (PATHS[6], ProjectionSpan::Native),
    (PATHS[7], ProjectionSpan::Native),
    (
        p("finite8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::FirstTick,
    ),
];

fn measured(f: Fixture) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let Some(cut) = cuts[0] else {
        return Ok(
            json!({"fixture": f, "cut": null, "donor_disjoint_tick": disjoint, "paths": []}),
        );
    };
    let mut records = Vec::with_capacity(TREATMENTS.len());
    let last = cut.tick + ENDPOINTS[3];
    for (profile, scope) in TREATMENTS {
        let (ticks, hits) = advance_path_until(f, &cut, profile, scope, last)?;
        if ticks.len() != 26
            || ticks.first().is_none_or(|s| s.calls != profile.onset.n)
            || ticks[1..].iter().any(|s| s.calls != profile.suffix.n)
        {
            return Err("incorrect propagation prefix budget".into());
        }
        let states = ENDPOINTS.map(|dt| ticks[(dt - 1) as usize].as_json());
        let cumulative_load: f64 = ticks.iter().map(|s| s.normal).sum();
        let cumulative_tangent: DVec3 = ticks.iter().map(|s| s.tangent).sum();
        records.push(json!({
            "profile": profile.label,
            "samples": states,
            "impacts": hits,
            "native_calls": ticks.iter().map(|s| u64::from(s.calls)).sum::<u64>(),
            "cumulative_normal_ns": cumulative_load,
            "cumulative_tangent_ns": cumulative_tangent.to_array()
        }));
    }
    Ok(json!({"fixture":f, "cut":snapshot(&cut), "donor_disjoint_tick":disjoint, "paths":records}))
}

pub(crate) fn survey(dir: &Path, start: usize, count: usize) -> Result<()> {
    let end = start
        .checked_add(count)
        .filter(|&end| count > 0 && end <= COUNT)
        .ok_or("landing persistence uses at most 250 propagation fixtures")?;
    let cases = dir.join("cases");
    fs::create_dir_all(&cases)?;
    for index in start..end {
        let file = cases.join(format!("case-{index:03}.json"));
        if file.is_file() {
            let record: Value = serde_json::from_reader(fs::File::open(&file)?)?;
            if record["format"] != FORMAT
                || record["case"] != index
                || record["result"]["fixture"] != serde_json::to_value(fixture(index))?
            {
                return Err("existing propagation prefix has another fixture or schema".into());
            }
            continue;
        }
        let result = measured(fixture(index))?;
        let tmp = file.with_extension(format!("partial-{}", std::process::id()));
        write_json(&tmp, &json!({"format":FORMAT,"case":index,"result":result}))?;
        fs::rename(tmp, file)?;
    }
    Ok(())
}

/// Reconstruct all three native trajectories and check their four early states,
/// last state, reaction sums and independently located returns against the two source campaigns.
pub(crate) fn replay(dir: &Path, index: usize) -> Result<()> {
    if index >= COUNT {
        return Err("invalid three-stroke fixture".into());
    }
    let f = fixture(index);
    let (cuts, _) = survey::cuts(f)?;
    let cut = cuts[0].ok_or("no landing donor for replay")?;
    let name = format!("cases/case-{index:03}.json");
    let root = dir
        .parent()
        .ok_or("the persistence archive must have a parent directory")?;
    let load = |p: &Path| -> Result<Value> { Ok(serde_json::from_reader(fs::File::open(p)?)?) };
    let prefix = load(&dir.join(&name))?;
    let native = load(&root.join("multifidelity-primary-propagation").join(&name))?;
    let finite = load(&root.join("multifidelity-finite-propagation").join(&name))?;
    let source_cut = serde_json::to_value(snapshot(&cut))?;
    if [&prefix, &native, &finite]
        .into_iter()
        .any(|r| r["case"] != index || r["result"]["cut"] != source_cut)
    {
        return Err("replay donor differs from measured source".into());
    }
    let mut traces = Vec::new();
    let selected = [TREATMENTS[0], TREATMENTS[4], TREATMENTS[2]];
    let archives = [&native, &finite, &native];
    for ((profile, scope), archive) in selected.into_iter().zip(archives) {
        let (run, _) = advance_path(f, &cut, profile, scope)?;
        let lane = |r: &Value| -> Result<Value> {
            Ok(r["result"]["paths"]
                .as_array()
                .and_then(|p| p.iter().find(|v| v["profile"] == profile.label))
                .ok_or("missing archived trace")?
                .clone())
        };
        let early = lane(&prefix)?;
        let full = lane(archive)?;
        let normalized =
            |v: &Value| -> Result<Value> { Ok(serde_json::from_slice(&serde_json::to_vec(v)?)?) };
        let checkpoints = ENDPOINTS.map(|s| run[(s - 1) as usize].as_json());
        if normalized(&json!(checkpoints))? != early["samples"]
            || normalized(&run[0].as_json())? != full["first"]
            || normalized(&run.last().ok_or("empty native trace")?.as_json())? != full["end"]
            || normalized(&events(&run))? != full["outcomes"]
        {
            return Err(
                format!("reproduced {} differs from archived physics", profile.label).into(),
            );
        }
        traces.push(run);
    }
    // Preserve every public endpoint for diagnosing the ordering change between the sparse display frames.
    let mut delta_mm = Vec::with_capacity(traces[0].len());
    let mut tracking = Vec::with_capacity(traces[0].len());
    for ((&a, &b), &c) in traces[0].iter().zip(&traces[1]).zip(&traces[2]) {
        let tick = a.tick;
        let states = [a, b, c];
        if states.iter().any(|state| state.tick != tick) {
            return Err("persistence replay public clocks diverged".into());
        }
        let position = states.map(|s| s.com());
        delta_mm
            .push(1000. * (position[1].distance(position[2]) - position[0].distance(position[2])));
        tracking.push(json!({
            "tick":tick, "com_m":position.map(|v|v.to_array()),
            "spin_rad_s":states.map(|s|s.foot.angular_velocity),
            "normal_ns":states.map(|s|s.normal), "tangent_ns":states.map(|s|s.tangent.to_array()),
            "projected_gap_m":states.map(|s|s.gap), "first_local_ns":states.map(|s|s.first.map(|arrival|arrival.as_nanos() as u64))
        }));
    }
    let mut captures = vec![
        ("at_thirteen_ms", cut.tick + 26),
        ("final_endpoint", DURATION),
    ];
    for (label, minimum_mm, consecutive) in [
        ("first_three_ticks_over_ten_um", 0.01, 3),
        ("first_ten_ticks_over_one_mm", 1., 10),
    ] {
        let start = 26;
        if let Some(j) = delta_mm[start..]
            .windows(consecutive)
            .position(|w| w.iter().all(|d| *d >= minimum_mm))
        {
            captures.push((label, traces[0][start + j].tick));
        }
    }
    let out = dir.join(format!("case-{index:03}"));
    fs::create_dir_all(&out)?;
    write_json(
        &out.join("tracking.json"),
        &json!({"case":index,"cut_tick":cut.tick,"label":["8r1_then512","finite8_then512","continuous4096r1"],"frames":tracking}),
    )?;
    let names = [
        "Ordinary first impact, then 512 calls",
        "Geometric first impact, then 512 calls",
        "Continuous 4,096 native calls",
    ];
    let selected = std::array::from_fn(|i| (selected[i].0.label, names[i], traces[i].as_slice()));
    finite::write_comparison(
        dir,
        index,
        &cut,
        selected,
        "First landing and third return on the same numerical continuation",
        &captures,
    )
}
