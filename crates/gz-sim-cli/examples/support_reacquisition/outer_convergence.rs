// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Long-horizon numerical-reference sensitivity at slow, nominal and fast drive.
use super::*;
const TAG: &str = "continuous-three-stroke-grid-v1";
const CELLS: usize = 150;
const MODES: [Profile; 5] = [
    p("2048", b(2048, 1), b(2048, 1)),
    p("4096", b(4096, 1), b(4096, 1)),
    p("8192", b(8192, 1), b(8192, 1)),
    PATHS[0],
    PATHS[1],
];
fn source(k: usize) -> usize {
    // Invert the bijection used by the parent lattice. Speed indices 0,2,4
    // give exactly 25 × 3 × 2 distinct, paired propagation fixtures.
    let i = k * 71 % CELLS;
    let physical = 10 * (i / 6) + 4 * ((i / 2) % 3) + i % 2;
    (0..COUNT).find(|v| v * 117 % COUNT == physical).unwrap()
}
fn measure(k: usize) -> Result<(Value, Option<Trajectories>)> {
    let src = source(k);
    let f = fixture(src);
    let (cuts, censor) = survey::cuts(f)?;
    let Some(cut) = cuts[0] else {
        return Ok((json!({"fixture":f,"cut":null,"disjoint":censor}), None));
    };
    let mut paths = vec![];
    for mode in MODES {
        paths.push(path(f, &cut, mode)?);
    }
    let mut rows = vec![];
    for (idx, p) in paths.iter().enumerate() {
        rows.push(json!({"profile":MODES[idx].label,"calls":p.iter().map(|t|u64::from(t.calls)).sum::<u64>(),"end":p.last().unwrap().as_json(),"events":events(p),
        "against":(0..3).map(|j|compare(p,&paths[j])).collect::<Result<Vec<_>>>()?}));
    }
    Ok((
        json!({"fixture":f,"cut":snapshot(&cut),"cut_tick":cut.tick,"end_tick":DURATION,"paths":rows}),
        Some((cut, paths)),
    ))
}
pub(crate) fn survey(dir: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|v| *v <= CELLS && count > 0)
        .ok_or("150 convergence cases")?;
    fs::create_dir_all(dir.join("cases"))?;
    for k in first..end {
        let dest = dir.join("cases").join(format!("case-{k:03}.json"));
        if dest.exists() {
            let v: Value = serde_json::from_reader(fs::File::open(&dest)?)?;
            if v["format"] != TAG
                || v["case"] != k
                || v["source"] != source(k)
                || v["result"]["fixture"] != serde_json::to_value(fixture(source(k)))?
            {
                return Err("stored reference fixture mismatch".into());
            }
            continue;
        }
        let now = std::time::Instant::now();
        let (r, _) = measure(k)?;
        let tmp = dest.with_extension(format!("partial-{}", std::process::id()));
        write_json(
            &tmp,
            &json!({"format":TAG,"case":k,"source":source(k),"result":r}),
        )?;
        fs::rename(tmp, dest)?;
        eprintln!(
            "completed convergence {k} in {:.2}s",
            now.elapsed().as_secs_f64()
        );
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    if k >= CELLS {
        return Err("unknown convergence case".into());
    }
    let saved: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let (r, data) = measure(k)?;
    if serde_json::from_slice::<Value>(&serde_json::to_vec(&r)?)? != saved["result"] {
        return Err("reference replay mismatch".into());
    }
    let (cut, paths) = data.ok_or("no matched landing")?;
    let out = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(out.join("traces"))?;
    let mut times = std::collections::BTreeSet::new();
    for path in paths.iter().take(3) {
        times.extend(
            path.iter()
                .filter(|s| s.tick % 16 == 0 || s.tick < cut.tick + 10 || s.tick == DURATION)
                .map(|s| s.tick),
        );
        for row in events(path)["strokes"].as_array().unwrap() {
            if let Some(t) = row["first_native_end_ns"].as_u64() {
                let n = t.div_ceil(STEP_US * 1000);
                times.extend(n.saturating_sub(4)..=n + 8);
            }
        }
    }
    let mut lanes = vec![];
    for (p, profile) in paths.iter().zip(&MODES).take(3) {
        let mut init = serde_json::to_value(cut.with_effort())?;
        init.as_object_mut().unwrap().remove("corner_ns");
        let mut frames = vec![init];
        frames.extend(
            p.iter()
                .filter(|q| times.contains(&q.tick))
                .map(Tick::as_json),
        );
        let id = profile.label;
        write_json(
            &out.join("traces").join(format!("{id}.json")),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":id,"name":format!("Continuous {id} calls"),"file":format!("traces/{id}.json")}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Three continuous native grids through three landings","sampled_ticks":true,"pair_totals_only":true,"case":k,"lanes":lanes}),
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn subset_has_balanced_unique_source_fixtures() {
        let s: std::collections::BTreeSet<_> = (0..CELLS).map(source).collect();
        assert_eq!(s.len(), CELLS);
        for speed in [0, 2, 4] {
            assert_eq!(
                s.iter()
                    .filter(|r| ((**r * 117 % COUNT) / 2) % 5 == speed)
                    .count(),
                50
            );
        }
    }
}
