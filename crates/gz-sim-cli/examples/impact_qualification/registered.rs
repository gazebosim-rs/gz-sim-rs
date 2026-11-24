// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
type Stats = BTreeMap<(String, u64, String), Series>;
fn report(stats: Stats) -> Vec<Value> {
    stats.into_iter().map(|((group,calls,field),series)|json!({"group":group,"calls":calls,"field":field,"statistics":series.stats()})).collect()
}
pub(super) fn patch(dir: &Path) -> Result<()> {
    let mut stats = Stats::new();
    let mut seen = BTreeSet::new();
    let mut missing = vec![];
    let mut counts = BTreeMap::new();
    let phases = [
        0u64, 1, 15_625, 62_499, 62_501, 138_197, 250_000, 312_501, 437_499, 484_375, 499_999,
    ];
    for e in fs::read_dir(dir.join("batches"))? {
        let p = e?.path();
        if p.extension().is_none_or(|e| e != "json") {
            continue;
        }
        let doc = read(&p)?;
        let lo = uint(&doc["start"])? as usize;
        let hi = uint(&doc["end"])? as usize;
        let rows = array(&doc["rows"])?;
        if doc["format"] != "projected-planar-patch-v1"
            || hi <= lo
            || hi > 243100
            || rows.len() != 2 * (hi - lo)
        {
            return Err("invalid patch batch".into());
        }
        for (off, chunk) in rows.chunks_exact(2).enumerate() {
            let i = lo + off;
            let k = i / 11;
            let phase = phases[(i % 11 + k) % 11];
            if !seen.insert(i) {
                return Err("duplicated physical patch".into());
            }
            for (r, n) in chunk.iter().zip([2u64, 8]) {
                if r["case"] != k || r["phase_ns"] != phase || r["calls"] != n {
                    return Err("patch identity or actual budget mismatch".into());
                }
                if uint(&r["projections"])? != 1 {
                    missing.push(json!([i, n]));
                    continue;
                }
                let bearing = r["bearing"].as_str().ok_or("missing bearing")?;
                *counts.entry(format!("{n}_{bearing}")).or_insert(0u64) += 1;
                for key in [
                    "clock_error_ns",
                    "impulse_error_ns",
                    "pressure_error_m",
                    "work_error_j",
                    "cone_excess_ns",
                    "total_impulse_error_ns",
                    "spin_error_rad_s",
                    "com_error_m",
                    "native_extra_ns",
                ] {
                    let x = num(&r[key])?;
                    for group in ["all".to_owned(), bearing.into(), format!("phase_{phase}")] {
                        stats.entry((group, n, key.into())).or_default().push(x, i);
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"cases":seen.len(),"complete":seen.len()==243100,"projected_counts":counts,"missing":missing,"rows":report(stats)}),
    )
}
pub(super) fn ballistic(dir: &Path) -> Result<()> {
    let v = read(&dir.join("study.json"))?;
    let mut ids = BTreeSet::new();
    let mut stats = Stats::new();
    let mut counts = BTreeMap::new();
    for (k, row) in array(&v["rows"])?.iter().enumerate() {
        let prefix = uint(&row["prefix"])?;
        let id = serde_json::to_string(&[
            &row["gravity_index"],
            &row["orientation_index"],
            &row["pad_first"],
            &row["prefix"],
            &row["phase_ns"],
            &row["mu"],
            &row["closing"],
        ])?;
        if !ids.insert(id) {
            return Err("duplicate ballistic fixture".into());
        }
        let paths = array(&row["paths"])?;
        if paths.len() != 3 {
            return Err("missing ballistic control".into());
        }
        for (r, n) in paths.iter().zip([0u64, 2, 8]) {
            let projects = uint(&r["projected"])?;
            if projects > 1 || n == 0 && projects > 0 {
                return Err("spurious or duplicate projected impulse".into());
            }
            *counts
                .entry(format!("calls_{n}_projected_{projects}"))
                .or_insert(0u64) += 1;
            if !r["first_report_ns"].is_null() {
                let delta = (uint(&r["first_report_ns"])? as f64
                    - (prefix as f64 * 500000. + num(&row["phase_ns"])?))
                .abs();
                if (delta - num(&r["released_time_error_ns"])?).abs() > 1e-6 {
                    return Err("reported physical time mismatch".into());
                }
            }
            for key in [
                "released_time_error_ns",
                "matched_time_error_ns",
                "released_velocity_error_m_s",
                "matched_velocity_error_m_s",
                "released_impulse_error_ns",
                "matched_impulse_error_ns",
                "end_impulse_error_ns",
                "max_prior_spin_rad_s",
                "max_unloaded_position_error_m",
            ] {
                if r[key].is_null() {
                    continue;
                }
                for group in ["all".to_owned(), format!("prefix_{prefix}")] {
                    stats
                        .entry((group, n, key.into()))
                        .or_default()
                        .push(num(&r[key])?, k);
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"cases":ids.len(),"complete":ids.len()==2880,"counts":counts,"rows":report(stats)}),
    )
}
