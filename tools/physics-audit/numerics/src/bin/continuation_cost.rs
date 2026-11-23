// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read paired native timings at two CPU placements and rescore a held-out continuation policy.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::Path,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
type Key = (u64, u64);
const LABELS: [&str; 3] = ["stop", "continue", "selected"];
fn value(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn n(x: &Value) -> Result<u64> {
    x.as_u64().ok_or_else(|| "invalid timing integer".into())
}
fn vec_u(x: &Value) -> Result<Vec<u64>> {
    x.as_array().ok_or("tick array")?.iter().map(n).collect()
}
fn vec_b(x: &Value) -> Result<Vec<bool>> {
    x.as_array()
        .ok_or("load array")?
        .iter()
        .map(|v| v.as_bool().ok_or_else(|| "load flag".into()))
        .collect()
}
fn stats(mut v: Vec<i64>) -> Value {
    if v.is_empty() {
        return json!({"n":0,"mean":null,"p50":null,"p95":null,"p99":null,"max":null});
    }
    v.sort_unstable();
    let q = |k: usize| v[(v.len() * k).div_ceil(100) - 1] as f64 / 1000.;
    json!({"n":v.len(),"mean":v.iter().map(|e|*e as f64).sum::<f64>()/v.len() as f64/1000.,"p50":q(50),"p95":q(95),"p99":q(99),"max":q(100)})
}
#[derive(Default)]
struct Times {
    first: Vec<i64>,
    tick: Vec<i64>,
    sums: Vec<i64>,
    loaded: Vec<i64>,
    empty: Vec<i64>,
    both_loaded: Vec<i64>,
    both_empty: Vec<i64>,
}
impl Times {
    fn add(&mut self, ns: &[u64], load: &[bool], both: &[bool]) {
        self.first.push(ns[0] as i64);
        self.sums.push(ns[1..].iter().sum::<u64>() as i64);
        for ((&t, &active), &other) in ns[1..].iter().zip(&load[1..]).zip(&both[1..]) {
            let t = t as i64;
            self.tick.push(t);
            if active {
                self.loaded.push(t);
            } else {
                self.empty.push(t);
            }
            if active && other {
                self.both_loaded.push(t);
            }
            if !active && !other {
                self.both_empty.push(t);
            }
        }
    }
    fn summary(self) -> Value {
        json!({"common_first":stats(self.first),"suffix_step":stats(self.tick),"suffix_total":stats(self.sums),"own_loaded_step":stats(self.loaded),"own_unloaded_step":stats(self.empty),"both_loaded_step":stats(self.both_loaded),"both_unloaded_step":stats(self.both_empty)})
    }
}

fn profile(
    dir: &Path,
    policy: &BTreeMap<Key, bool>,
    physical: &mut BTreeMap<(Key, u64), Vec<bool>>,
) -> Result<Value> {
    let design = value(&dir.join("design.json"))?;
    let repeats = n(&design["repetitions"])?;
    let recorded = n(&design["matched_pairs"])?;
    let mut durations: [Times; 3] = Default::default();
    let mut total_delta = Vec::new();
    let mut step_delta = Vec::new();
    let mut seen = BTreeMap::<Key, BTreeSet<u64>>::new();
    let mut selected_repeats = 0;
    let mut position_order = [0; 2];
    for line in fs::read_to_string(dir.join("suffix-cost.jsonl"))?.lines() {
        let raw: Value = serde_json::from_str(line)?;
        let id = n(&raw["index"])?;
        let stroke = n(&raw["stroke"])?;
        let key = (id, stroke);
        let repeat = n(&raw["repeat"])?;
        if repeat >= repeats || !seen.entry(key).or_default().insert(repeat) {
            return Err("duplicate or bad repeat".into());
        }
        let k = id * 613 % 1250;
        assert_eq!(raw["offset_mm"], 165 + 5 * (k / 50));
        assert!(n(&raw["offset_mm"])?.is_multiple_of(25));
        let take = *policy
            .get(&key)
            .ok_or("no held-out policy decision for timing cohort")?;
        let mut ns: [Vec<u64>; 2] = Default::default();
        let mut load: [Vec<bool>; 2] = Default::default();
        let first_stop = (id + stroke + repeat).is_multiple_of(2);
        for (i, obj) in raw["runs"]
            .as_array()
            .ok_or("timed pair")?
            .iter()
            .enumerate()
        {
            let passes = n(&obj["suffix_passes"])?;
            assert_eq!(passes, if first_stop == (i == 0) { 1 } else { 8 });
            let j = usize::from(passes == 8);
            if i == 0 {
                position_order[j] += 1;
            }
            ns[j] = vec_u(&obj["tick_ns"])?;
            load[j] = vec_b(&obj["loaded"])?;
            assert_eq!((ns[j].len(), load[j].len()), (26, 26));
            assert!(ns[j].iter().all(|&v| v > 0));
            let template = physical
                .entry((key, passes))
                .or_insert_with(|| load[j].clone());
            assert_eq!(*template, load[j]);
        }
        assert_eq!(load[0][0], load[1][0]);
        for (i, j) in [0, 1, usize::from(take)].into_iter().enumerate() {
            durations[i].add(&ns[j], &load[j], &load[1 - j]);
        }
        selected_repeats += usize::from(take);
        let mut delta = 0;
        for (&a, &b) in ns[0][1..].iter().zip(&ns[1][1..]) {
            let t = b as i64 - a as i64;
            delta += t;
            step_delta.push(t);
        }
        total_delta.push(delta);
    }
    assert_eq!(position_order, [recorded / 2; 2]);
    assert_eq!(seen.len() as u64 * repeats, recorded);
    assert!(seen.values().all(|x| x.len() as u64 == repeats));
    let rows: BTreeMap<_, _> = LABELS
        .into_iter()
        .zip(durations.map(Times::summary))
        .collect();
    Ok(
        json!({"placement":dir.file_name().unwrap().to_string_lossy(),"stat_units":"microseconds","rows":rows,
        "paired_continue_minus_stop_suffix_total":stats(total_delta),"paired_continue_minus_stop_suffix_step":stats(step_delta),"matched_donors":seen.len(),
        "repeats":repeats,"matched_pairs":recorded,"selected_repeats_with_eight":selected_repeats,"first_positions_stop_continue":position_order,
        "sibling_cpu_ids":fs::read_to_string(dir.join("sibling-cpus"))?.trim(),"sibling_jobs_at_start":fs::read_to_string(dir.join("sibling-jobs"))?.trim()}),
    )
}
fn chart(path: &Path, data: &[Value]) -> Result<()> {
    let root = SVGBackend::new(path, (1000, 610)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "Measured cost after the first shared landing interval",
        ("sans-serif", 24),
    )?;
    let (header, area) = body.split_vertically(63);
    header.draw(&Text::new("Independent pinned runs on two hardware threads. The third bar uses the measured treatment chosen for each donor.",(20,15),("sans-serif",15)))?;
    header.draw(&Text::new("All step and whole-suffix costs exclude the common first 500 µs and use the ordinary native API.",(20,39),("sans-serif",15)))?;
    let colors = [
        RGBColor(24, 127, 131),
        RGBColor(187, 96, 49),
        RGBColor(64, 89, 157),
    ];
    for (panel, (field, title)) in area.split_evenly((2, 1)).iter().zip([
        ("suffix_step", "p95 cost of one subsequent public step"),
        ("suffix_total", "p95 cost of all 25 subsequent public steps"),
    ]) {
        let max = data
            .iter()
            .flat_map(|r| LABELS.map(|label| r["rows"][label][field]["p95"].as_f64().unwrap()))
            .fold(0., f64::max);
        let mut p = ChartBuilder::on(panel)
            .margin(10)
            .caption(title, ("sans-serif", 17))
            .x_label_area_size(33)
            .y_label_area_size(77)
            .build_cartesian_2d(0.0..2.0, 0.0..max * 1.30)?;
        p.configure_mesh()
            .disable_x_mesh()
            .max_light_lines(1)
            .x_labels(0)
            .y_desc("Wall time (µs)")
            .draw()?;
        for (idx, r) in data.iter().enumerate() {
            for (k, label) in LABELS.iter().enumerate() {
                let val = r["rows"][label][field]["p95"].as_f64().unwrap();
                let left = idx as f64 + 0.12 + k as f64 * 0.23;
                p.draw_series([Rectangle::new(
                    [(left, 0.), (left + 0.19, val)],
                    colors[k].filled(),
                )])?;
                p.draw_series([Text::new(
                    format!("{val:.1}"),
                    (left + 0.02, val + 0.045 * max),
                    ("sans-serif", 13),
                )])?;
                let (px, py) = p.plotting_area().map_coordinate(&(left + 0.095, 0.));
                let text_style = ("sans-serif", 13).into_text_style(&root).pos(
                    plotters::style::text_anchor::Pos::new(
                        plotters::style::text_anchor::HPos::Center,
                        plotters::style::text_anchor::VPos::Top,
                    ),
                );
                root.draw(&Text::new(*label, (px, py + 6), text_style))?;
            }
            p.draw_series([Text::new(
                r["placement"].as_str().unwrap().to_owned(),
                (idx as f64 + 0.12, 1.18 * max),
                ("sans-serif", 14),
            )])?;
        }
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let a: Vec<_> = env::args_os().skip(1).collect();
    if a.len() != 3 {
        return Err("continuation_cost TIMING_DIR CONTACT_RECEIPTS_JSONL OUTPUT_DIR".into());
    }
    let source = Path::new(&a[0]);
    let out = Path::new(&a[2]);
    fs::create_dir_all(out)?;
    let mut policy = BTreeMap::new();
    for line in fs::read_to_string(&a[1])?.lines() {
        let r: Value = serde_json::from_str(line)?;
        let s = &r["site"];
        let key = (n(&s["index"])?, n(&s["stroke"])?);
        assert!(
            policy
                .insert(
                    key,
                    r["continue_eight"].as_bool().ok_or("missing decision")?
                )
                .is_none()
        );
    }
    let mut physical = BTreeMap::new();
    let rows = [
        profile(&source.join("cpu-7"), &policy, &mut physical)?,
        profile(&source.join("cpu-8"), &policy, &mut physical)?,
    ];
    chart(&out.join("continuation-cost.svg"), &rows)?;
    fs::write(
        out.join("continuation-cost.json"),
        serde_json::to_vec_pretty(
            &json!({"runs":rows,"same_reaction_masks_across_repeats_and_cpus":physical.len(),"timed_api":"MultiFidelityBackend::step_contact_interval without a native substep observer","selected_scope":"reuses each measured candidate suffix for the archived offline decision; decision overhead excluded"}),
        )?,
    )?;
    Ok(())
}
