// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Summarize matched arm timings. Standby boundaries are simulation timestamps in milliseconds.
use serde_json::{Value, json};
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn percentiles(values: &[u64]) -> [u64; 2] {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    [95, 99].map(|p| sorted[(p * sorted.len()).div_ceil(100) - 1])
}

fn read(path: &Path, start_ms: u64, end_ms: u64) -> Result<(usize, u64, [u64; 5])> {
    let timing: Value = serde_json::from_slice(&std::fs::read(path)?)?;
    summarize_run(&timing, start_ms, end_ms)
}

fn summarize_run(timing: &Value, start_ms: u64, end_ms: u64) -> Result<(usize, u64, [u64; 5])> {
    let dt = timing["physics_step_ns"]
        .as_u64()
        .ok_or("missing step duration")?;
    let samples: Vec<u64> = serde_json::from_value(timing["public_step_wall_ns"].clone())?;
    let index = |ms: u64| -> Result<usize> {
        let ns = ms.checked_mul(1_000_000).ok_or("clock overflow")?;
        if dt == 0 || !ns.is_multiple_of(dt) {
            return Err("standby boundary falls between solves".into());
        }
        Ok(usize::try_from(ns / dt)?)
    };
    let window = samples
        .get(index(start_ms)?..index(end_ms)?)
        .filter(|s| !s.is_empty())
        .ok_or("empty or out-of-range standby interval")?;
    let [p95, p99] = percentiles(&samples);
    if timing["p95_ns"] != p95 || timing["p99_ns"] != p99 || timing["steps"] != samples.len() {
        return Err("timing summary and public steps disagree".into());
    }
    let [idle95, idle99] = percentiles(window);
    Ok((
        samples.len(),
        dt,
        [
            timing["elapsed_ns"]
                .as_u64()
                .ok_or("missing elapsed time")?,
            p95,
            p99,
            idle95,
            idle99,
        ],
    ))
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let start = args.next().ok_or("expected standby start in ms")?.parse()?;
    let end = args.next().ok_or("expected standby end in ms")?.parse()?;
    let pairs: Vec<_> = args.collect();
    if pairs.is_empty() || pairs.len().is_multiple_of(2) {
        return Err("supply an odd number of paired timing directories".into());
    }
    let mut lanes = [Vec::new(), Vec::new()];
    let mut clock = None;
    for dir in &pairs {
        for (name, rows) in ["dynamic", "adaptive"].into_iter().zip(&mut lanes) {
            let (steps, dt, cost) = read(&Path::new(dir).join(format!("{name}.json")), start, end)?;
            if clock.replace((steps, dt)).is_some_and(|v| v != (steps, dt)) {
                return Err("public clocks or solve counts differ".into());
            }
            rows.push(cost);
        }
    }
    let summarize = |runs: Vec<[u64; 5]>| {
        let medians: [u64; 5] = std::array::from_fn(|i| {
            let mut column = runs.iter().map(|r| r[i]).collect::<Vec<_>>();
            column.sort_unstable();
            column[column.len() / 2]
        });
        json!({"runs":runs,"median":medians})
    };
    let [dynamic, adaptive] = lanes.map(summarize);
    let (steps, dt) = clock.unwrap();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "columns":["elapsed_ns","step_p95_ns","step_p99_ns","standby_p95_ns","standby_p99_ns"],
            "steps":steps,"dt_ns":dt,"standby_start_ms":start,"standby_end_ms":end,"dynamic":dynamic,"adaptive":adaptive
        }))?
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn nearest_rank_keeps_the_slowest_step_in_a_short_window() {
        assert_eq!(percentiles(&[9, 2, 1, 7]), [9, 9]);
        let reverse: Vec<_> = (1..=100).rev().collect();
        assert_eq!(percentiles(&reverse), [95, 99]);
    }

    #[test]
    fn window_begins_with_the_first_solve_after_parking() {
        let base = json!({"physics_step_ns": 2_000_000, "steps": 4,
            "public_step_wall_ns":[999,12,20,888],"p95_ns":999,"p99_ns":999,"elapsed_ns":5020});
        assert_eq!(
            summarize_run(&base, 2, 6).unwrap(),
            (4, 2_000_000, [5020, 999, 999, 20, 20])
        );
        for (start, end) in [(0, 0), (0, 10), (1, 6), (6, 2), (u64::MAX, 8)] {
            assert!(summarize_run(&base, start, end).is_err());
        }
        for (field, bad) in [
            ("steps", json!(5)),
            ("p95_ns", json!(12)),
            ("p99_ns", json!(11)),
            ("physics_step_ns", json!(0)),
            ("elapsed_ns", Value::Null),
            ("public_step_wall_ns", json!([])),
            ("physics_step_ns", Value::Null),
        ] {
            let mut corrupted = base.clone();
            corrupted[field] = bad;
            assert!(summarize_run(&corrupted, 2, 6).is_err());
        }
    }
}
