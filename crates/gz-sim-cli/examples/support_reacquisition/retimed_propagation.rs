// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate moving the next native update after the first finite-point impulse.
//! The remaining three-stroke sequence uses identical 512-call intervals.
use super::*;
const FORMAT: &str = "retimed-first-three-strokes-v1";
const MODES: [(Profile, ProjectionSpan); 3] = [
    (
        p("finite8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::FirstTick,
    ),
    (
        p("retimed8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::RetimeFirst,
    ),
    (
        p("native8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::Native,
    ),
];
struct Study {
    summary: Value,
    trajectories: [Vec<Tick>; 3],
    cut: ReboundFrame,
}
fn run(k: usize) -> Result<Study> {
    let f = fixture(k);
    let cut = survey::cuts(f)?.0[0].ok_or("missing primary cut")?;
    let mut traces = Vec::new();
    let mut paths = Vec::new();
    for (mode, scope) in MODES {
        let (ticks, impacts) = advance_path(f, &cut, mode, scope)?;
        let first = &ticks[0];
        let last = ticks.last().ok_or("empty interval")?;
        let calls = ticks.iter().map(|t| u64::from(t.calls)).sum::<u64>();
        paths.push(json!({"profile":mode.label,"native_calls":calls,"first":first.as_json(),"end":last.as_json(),
            "outcomes":events(&ticks),"impacts":impacts}));
        traces.push(ticks);
    }
    let trajectories: [Vec<Tick>; 3] = traces.try_into().map_err(|_| "missing control")?;
    let summary = json!({"case":k,"format":FORMAT,"fixture":f,"cut":snapshot(&cut),"end_tick":DURATION,
        "change":compare(&trajectories[1],&trajectories[0])?,"paths":paths});
    Ok(Study {
        cut,
        summary,
        trajectories,
    })
}
fn paths(dir: &Path, k: usize) -> (PathBuf, PathBuf) {
    let name = format!("case-{k:03}.json");
    (
        dir.join("cases").join(&name),
        dir.join("trajectories").join(name),
    )
}
pub(crate) fn survey(dir: &Path, first: usize, count: usize) -> Result<()> {
    let stop = first
        .checked_add(count)
        .filter(|v| count > 0 && *v <= COUNT)
        .ok_or("invalid fixture range")?;
    for sub in ["cases", "trajectories"] {
        fs::create_dir_all(dir.join(sub))?
    }
    for k in first..stop {
        let (dst, raw) = paths(dir, k);
        if dst.exists() {
            let v: Value = serde_json::from_reader(fs::File::open(&dst)?)?;
            if v["format"] != FORMAT || v["case"] != k || !raw.exists() {
                return Err("incomplete saved three-stroke trial".into());
            }
            continue;
        }
        let study = run(k)?;
        let series=study.trajectories.iter().zip(MODES).map(|(ticks,(mode,_))|json!({"profile":mode.label,
            "ticks":ticks.iter().map(|t|json!([t.tick,t.com().to_array(),t.foot.angular_velocity,t.normal,t.tangent.to_array(),t.first.map(|v|v.as_nanos()as u64)])).collect::<Vec<_>>() })).collect::<Vec<_>>();
        for (file, value) in [
            (raw, json!({"case":k,"paths":series})),
            (dst, study.summary),
        ] {
            let temp = file.with_extension(format!("partial-{}", std::process::id()));
            write_json(&temp, &value)?;
            fs::rename(temp, file)?;
        }
        eprintln!("completed retimed three-stroke fixture {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    if k >= COUNT {
        return Err("invalid fixture".into());
    }
    let study = run(k)?;
    let v: Value = serde_json::from_reader(fs::File::open(paths(dir, k).0)?)?;
    if serde_json::from_slice::<Value>(&serde_json::to_vec(&study.summary)?)? != v {
        return Err("three-stroke replay disagrees with archive".into());
    }
    let names = [
        "Finite first point then 512",
        "Retimed second feature then 512",
        "Ordinary onset then 512",
    ];
    let lanes =
        std::array::from_fn(|i| (MODES[i].0.label, names[i], study.trajectories[i].as_slice()));
    finite::write_comparison(
        dir,
        k,
        &study.cut,
        lanes,
        "Timing the first transfer through three support strokes",
        &[],
    )
}
