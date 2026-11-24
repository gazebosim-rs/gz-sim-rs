// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render measured weight support and guard decisions over the complete recovery trial.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, io::Write, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const FINGERS: RGBColor = RGBColor(13, 115, 113);
const BENCH: RGBColor = RGBColor(216, 154, 70);
const OTHER: RGBColor = RGBColor(194, 202, 206);
fn n(v: &Value) -> Result<f64> {
    v.as_f64().ok_or_else(|| "missing measurement".into())
}
fn u(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing integer".into())
}
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Support {
    Fingers,
    Bench,
    Other,
}
fn support(row: &Value) -> Result<Support> {
    let weight = 9.81 * n(&row["mass_kg"])?;
    if weight <= 0. {
        return Err("expected positive sheet mass".into());
    }
    let bench = n(&row["bench_up_n"])?;
    let jaws = [n(&row["jaw_up_n"][0])?, n(&row["jaw_up_n"][1])?];
    let clearance = n(&row["min_bench_clearance_m"])?;
    Ok(if bench >= 0.8 * weight {
        Support::Bench
    } else if bench.abs() < 1e-6
        && jaws.iter().all(|v| *v > 0.)
        && ((jaws[0] + jaws[1]) / weight - 1.).abs() < 0.025
        && clearance > 0.01
    {
        Support::Fingers
    } else {
        Support::Other
    })
}

struct Row {
    columns: u64,
    load_n: f64,
    role: &'static str,
    states: Vec<Support>,
    events: Vec<(u64, &'static str)>,
    motion_mm: Option<f64>,
}
fn make_row(trace: &Value, case: &Value, role: &'static str, dt: u64, horizon: u64) -> Result<Row> {
    let baseline = u(&case["baseline_ns"])?;
    let summary = &case[role];
    let mut states = Vec::new();
    let rows = trace.as_array().ok_or("expected trace samples")?;
    for r in rows {
        let now = u(&r["time_ns"])?;
        if now > baseline {
            if u128::from(now) != u128::from(baseline) + (states.len() as u128 + 1) * u128::from(dt)
            {
                return Err("missing public sample".into());
            }
            states.push(support(r)?);
        }
    }
    let observed = u(&summary["observed_postgrip_ticks"])?;
    let escape = summary
        .get("escaped_after_tick")
        .ok_or("missing escape decision")?;
    let end = if escape.is_null() {
        horizon
    } else {
        u(escape)?
    };
    if observed != end || observed > horizon || states.len() as u64 != observed || states.len() < 20
    {
        return Err("incomplete recovery recording".into());
    }
    let tail = &states[states.len() - 20..];
    for (state, field) in [
        (Support::Fingers, "final_bilateral_ticks"),
        (Support::Bench, "final_bench_supported_ticks"),
    ] {
        if tail.iter().filter(|s| **s == state).count() as u64 != u(&summary[field])? {
            return Err("reported terminal support disagrees with forces".into());
        }
    }
    let mut events = Vec::new();
    for (key, label) in [
        ("tighten_after_tick", "T"),
        ("lower_after_tick", "L"),
        ("release_after_tick", "R"),
    ] {
        let event = summary.get(key).ok_or("missing guard decision")?;
        if !event.is_null() {
            let tick = u(event)?;
            if role == "control"
                || tick == 0
                || tick > observed
                || events.last().is_some_and(|(prev, _)| tick <= *prev)
            {
                return Err("invalid action time".into());
            }
            events.push((tick, label));
        }
    }
    if !escape.is_null() {
        events.push((end, "E"));
    }
    let motion = summary
        .get("patch_motion_at_400ms_m")
        .ok_or("missing 400 ms observation")?;
    let motion_mm = if u128::from(dt) * u128::from(observed) < 400_000_000 {
        if !motion.is_null() {
            return Err("400 ms travel reported after the episode ended".into());
        }
        None
    } else {
        Some(1000. * n(&motion[0])?.abs().max(n(&motion[1])?.abs()))
    };
    Ok(Row {
        columns: u(&case["columns"])?,
        load_n: n(&case["peak_mn"])? / 1000.,
        role,
        states,
        events,
        motion_mm,
    })
}
fn motion(mm: f64) -> String {
    if mm < 0.001 {
        format!("{:.1} nm", mm * 1e6)
    } else if mm < 1. {
        format!("{:.1} µm", mm * 1e3)
    } else {
        format!("{mm:.2} mm")
    }
}
fn draw(rows: &[Row], dt: u64, horizon: usize) -> Result<String> {
    let span_ms = horizon as f64 * dt as f64 * 1e-6;
    let mut svg = String::new();
    {
        let height = 190 + rows.len() as u32 * 46;
        let page = SVGBackend::with_string(&mut svg, (1120, height)).into_drawing_area();
        page.fill(&WHITE)?;
        let mut labels = Vec::new();
        let mut text = |s: String, x: i32, y: i32, size: i32| labels.push((s, (x, y), size));
        text(
            "Can the robot recover a slipping folded sheet?".into(),
            30,
            17,
            29,
        );
        text(
            "Each band follows the measured weight support after the sideways force starts.".into(),
            32,
            58,
            16,
        );
        for (x, color, label) in [
            (32, FINGERS, "Both fingers carry weight"),
            (255, BENCH, "Bench carries ≥80%"),
            (440, OTHER, "Other or changing support"),
        ] {
            page.draw(&Rectangle::new([(x, 83), (x + 15, 94)], color.filled()))?;
            text(label.into(), x + 22, 78, 15);
        }
        text(
            "T  tighten   ·   L  lower   ·   R  open   ·   E  escaped".into(),
            704,
            78,
            15,
        );
        let left = 250;
        let right = 927;
        let top = 160;
        let bottom = top + 46 * rows.len() as i32 - 15;
        text("Sheet / load".into(), 32, 126, 15);
        text("Branch".into(), 166, 126, 15);
        text("Sheet travel".into(), 952, 110, 15);
        text("in grip at 400 ms".into(), 952, 130, 15);
        let x = |tick: usize| {
            left + ((right - left) as f64 * tick as f64 / horizon as f64).round() as i32
        };
        for i in 0..=6 {
            let pos = x(i * horizon / 6);
            page.draw(&PathElement::new(
                [(pos, 145), (pos, bottom)],
                RGBColor(224, 227, 228),
            ))?;
            text(format!("{:.0}", span_ms * i as f64 / 6.), pos - 6, 126, 13);
        }
        text("Time after grip (ms)".into(), left + 250, 106, 15);
        for (i, row) in rows.iter().enumerate() {
            let y = top + i as i32 * 46;
            if i % 2 == 0 {
                text(
                    format!("{} points  ·  {:.2} N", row.columns, row.load_n),
                    32,
                    y - 2,
                    16,
                );
            }
            text(
                if row.role == "guard" {
                    "Guard".into()
                } else {
                    "Control".into()
                },
                166,
                y - 2,
                15,
            );
            let mut start = 0;
            while start < row.states.len() {
                let state = row.states[start];
                let next = start
                    + row.states[start..]
                        .iter()
                        .take_while(|s| **s == state)
                        .count();
                let color = match state {
                    Support::Fingers => FINGERS,
                    Support::Bench => BENCH,
                    Support::Other => OTHER,
                };
                page.draw(&Rectangle::new(
                    [(x(start), y), (x(next), y + 12)],
                    color.filled(),
                ))?;
                start = next;
            }
            for (tick, label) in &row.events {
                let marker = x(*tick as usize);
                page.draw(&PathElement::new(
                    [(marker, y - 1), (marker, y + 14)],
                    BLACK.stroke_width(2),
                ))?;
                text((*label).into(), marker - 4, y + 15, 13);
            }
            text(
                row.motion_mm
                    .map_or_else(|| "Ended earlier".to_owned(), motion),
                952,
                y - 2,
                16,
            );
        }
        for (label, position, size) in labels {
            page.draw(&Text::new(label, position, ("sans-serif", size)))?;
        }
        page.present()?;
    }
    Ok(svg)
}
fn render(root: &Path, output: &Path) -> Result<()> {
    let summary = read(&root.join("comparison.json"))?;
    let dt = u(&summary["dt_ns"])?;
    let horizon = u(&summary["horizon_ticks"])?;
    if dt == 0 {
        return Err("zero public clock".into());
    }
    let cases = summary["cases"]
        .as_array()
        .filter(|s| !s.is_empty())
        .ok_or("missing recovery trials")?;
    let mut rows = Vec::new();
    let mut unique = std::collections::BTreeSet::new();
    for case in cases {
        let c = u(&case["columns"])?;
        let load = u(&case["peak_mn"])?;
        if !unique.insert((c, load)) {
            return Err("duplicate recovery load".into());
        }
        for role in ["guard", "control"] {
            let trace = read(&root.join(format!("mesh-{c}-{load}-{role}/trace.json")))?;
            rows.push(make_row(&trace, case, role, dt, horizon)?);
        }
    }
    let svg = draw(&rows, dt, horizon.try_into()?)?;
    fs::File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [root, out] = &args[..] else {
        return Err("usage: fold_recovery_timeline STUDY_DIR NEW_FIGURE.svg".into());
    };
    render(Path::new(root), Path::new(out))
}
#[cfg(test)]
#[path = "../../tests/support/fold_recovery_timeline.rs"]
mod tests;
