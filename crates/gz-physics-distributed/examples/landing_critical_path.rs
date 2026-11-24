// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join coordinator, Zenoh callback and worker monotonic-clock events at public landing ticks.
#[path = "support/join_report.rs"]
mod join_report;
#[path = "support/wake_report.rs"]
mod wake_report;
use clap::Parser;
use plotters::prelude::*;
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const STAGES: [&str; 7] = [
    "Coordinator before submit",
    "Request delivery",
    "Worker preparation",
    "Native batch",
    "Worker after batch",
    "Reply delivery",
    "Coordinator after ready",
];

#[derive(Parser)]
struct Args {
    /// Trace root with adjacent and separated subdirectories, then one numbered directory per run.
    traces: PathBuf,
    /// Output directory for summaries, the chart and one compact row per measured tick.
    out: PathBuf,
    /// Include per-thread runqueue and CPU attribution from GZ_SIM_SCHED_TRACE captures.
    #[arg(long)]
    scheduler: bool,
    /// Analyze both task completions and the calling worker thread through scope return.
    #[arg(long)]
    join: bool,
}
#[derive(Deserialize)]
struct Event {
    name: String,
    ns: u64,
    peer: Option<u8>,
    world: Option<u64>,
    helper: Option<bool>,
    cpu_ns: Option<u64>,
    sched: Option<wake_report::Stat>,
}
#[derive(Deserialize)]
struct Row {
    tick: u64,
    events: Vec<Event>,
}
#[derive(Deserialize)]
struct Load {
    tick: u64,
    pad_pairs: [usize; 2],
}
#[derive(Serialize)]
struct Tick {
    run: usize,
    tick: u64,
    class: &'static str,
    gate_worker: u8,
    total_us: f64,
    stage_us: [f64; 7],
    /// Independent overlapping observations and detail within the batch on the populated owner.
    owner_batch_us: f64,
    owner_critical_lane_cpu_us: f64,
    owner_critical_lane_non_cpu_us: f64,
    owner_lane_gaps_us: f64,
    owner_critical_lane_launch_us: f64,
    owner_critical_lane_interworld_us: f64,
    owner_lane_join_us: f64,
    owner_lane_finish_gap_us: Option<f64>,
    owner_other_lane_worlds_us: f64,
    owner_critical_lane_worlds_us: f64,
    owner_longest_world_us: f64,
    owner_critical_count: usize,
    coordinator_collect_wait_us: f64,
    owner_reply_to_last_ready_us: f64,
}
fn read<T: DeserializeOwned>(p: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|s| Ok(serde_json::from_str(&s?)?))
        .collect()
}
fn stamp(row: &Row, name: &str, peer: Option<u8>) -> Result<u64> {
    let mut found = row
        .events
        .iter()
        .filter(|e| e.name == name && e.peer == peer);
    let value = found
        .next()
        .ok_or_else(|| format!("missing {name}/{peer:?} at {}", row.tick))?;
    if found.next().is_some() {
        return Err(format!("duplicate {name}/{peer:?} at {}", row.tick).into());
    }
    Ok(value.ns)
}
fn duration(a: u64, b: u64) -> Result<f64> {
    Ok(b.checked_sub(a)
        .ok_or_else(|| format!("clock order {a} > {b}"))? as f64
        / 1_000.)
}
fn tables(p: &Path) -> Result<BTreeMap<u64, Row>> {
    let mut out = BTreeMap::new();
    for r in read::<Row>(p)? {
        let tick = r.tick;
        if out.insert(tick, r).is_some() {
            return Err(format!("duplicate tick {tick} in {}", p.display()).into());
        }
    }
    Ok(out)
}

#[derive(Default)]
struct Lane {
    wall_us: f64,
    cpu_us: f64,
    last_ns: u64,
    worlds: usize,
}
struct Batch {
    wall_us: f64,
    cpu_us: f64,
    off_cpu_us: f64,
    gaps_us: f64,
    launch_us: f64,
    interworld_us: f64,
    join_us: f64,
    finish_gap_us: Option<f64>,
    other_worlds_us: f64,
    critical_worlds_us: f64,
    largest_us: f64,
    critical_count: usize,
}
fn owner_batch(row: &Row) -> Result<Batch> {
    let begin = stamp(row, "batch_start", None)?;
    let end = stamp(row, "batch_end", None)?;
    let mut lanes: [Lane; 2] = Default::default();
    let mut ranges = [Vec::new(), Vec::new()];
    let starts: Vec<_> = row
        .events
        .iter()
        .filter(|e| e.name == "native_start")
        .collect();
    let ends: Vec<_> = row
        .events
        .iter()
        .filter(|e| e.name == "native_end")
        .collect();
    if starts.len() != 8 || ends.len() != 8 {
        return Err(format!("expected eight native worlds at {}", row.tick).into());
    }
    let mut ids = BTreeSet::new();
    let mut largest_us = 0.0_f64;
    for s in starts {
        let world = s.world.ok_or("missing native world")?;
        if !ids.insert(world) {
            return Err("duplicate native world".into());
        }
        let finish = ends
            .iter()
            .find(|e| e.world == Some(world))
            .ok_or("missing native end")?;
        let lane = usize::from(s.helper.ok_or("missing executing lane")?);
        if finish.helper != s.helper || s.ns < begin || finish.ns > end {
            return Err("native world outside its batch or lane".into());
        }
        let wall = duration(s.ns, finish.ns)?;
        let cpu = duration(
            s.cpu_ns.ok_or("missing native CPU start")?,
            finish.cpu_ns.ok_or("missing native CPU end")?,
        )?;
        // Clock reads are separate operations. A sub-microsecond difference can put thread CPU just over wall time.
        lanes[lane].wall_us += wall;
        lanes[lane].cpu_us += cpu.min(wall);
        lanes[lane].last_ns = lanes[lane].last_ns.max(finish.ns);
        lanes[lane].worlds += 1;
        largest_us = largest_us.max(wall);
        ranges[lane].push((s.ns, finish.ns));
    }
    for r in &mut ranges {
        r.sort_unstable();
        if r.windows(2).any(|w| w[0].1 > w[1].0) {
            return Err("overlapping native worlds on one lane".into());
        }
    }
    let i = usize::from(lanes[1].last_ns > lanes[0].last_ns);
    let longest = &lanes[i];
    let wall_us = duration(begin, end)?;
    Ok(Batch {
        wall_us,
        cpu_us: longest.cpu_us,
        off_cpu_us: longest.wall_us - longest.cpu_us,
        gaps_us: (wall_us - longest.wall_us).max(0.),
        launch_us: duration(begin, ranges[i][0].0)?,
        interworld_us: ranges[i]
            .windows(2)
            .map(|p| (p[1].0 - p[0].1) as f64 / 1_000.)
            .sum(),
        join_us: duration(longest.last_ns, end)?,
        finish_gap_us: (lanes[1 - i].worlds > 0)
            .then(|| duration(lanes[1 - i].last_ns, longest.last_ns))
            .transpose()?,
        other_worlds_us: lanes[1 - i].wall_us,
        critical_worlds_us: longest.wall_us,
        largest_us,
        critical_count: longest.worlds,
    })
}
fn parse_tick(
    run: usize,
    tick: u64,
    class: &'static str,
    c: &Row,
    workers: &[BTreeMap<u64, Row>],
) -> Result<Tick> {
    let mut ready = [0; 4];
    let mut waits = 0.;
    for (n, ns) in ready.iter_mut().enumerate() {
        let peer = Some(n as u8);
        *ns = stamp(c, "ready", peer)?;
        let received = stamp(c, "received", peer)?;
        if *ns > received {
            return Err("reply read before callback".into());
        }
        waits += duration(stamp(c, "collect_start", peer)?, received)?;
    }
    let (id, &last) = ready
        .iter()
        .enumerate()
        .max_by_key(|(_, t)| *t)
        .ok_or("missing ready event")?;
    let w = workers[id].get(&tick).ok_or("missing gating worker")?;
    let start = stamp(c, "api_start", None)?;
    let end = stamp(c, "api_end", None)?;
    let cuts = [
        start,
        stamp(c, "submit", Some(id as u8))?,
        stamp(w, "request", None)?,
        stamp(w, "batch_start", None)?,
        stamp(w, "batch_end", None)?,
        stamp(w, "reply_submit", None)?,
        last,
        end,
    ];
    let mut stage_us = [0.; 7];
    for (out, p) in stage_us.iter_mut().zip(cuts.windows(2)) {
        *out = duration(p[0], p[1])?;
    }
    let b = owner_batch(workers[0].get(&tick).ok_or("missing populated owner")?)?;
    Ok(Tick {
        run,
        tick,
        class,
        gate_worker: id as u8,
        total_us: duration(start, end)?,
        stage_us,
        owner_batch_us: b.wall_us,
        owner_critical_lane_cpu_us: b.cpu_us,
        owner_critical_lane_non_cpu_us: b.off_cpu_us,
        owner_lane_gaps_us: b.gaps_us,
        owner_critical_lane_launch_us: b.launch_us,
        owner_critical_lane_interworld_us: b.interworld_us,
        owner_lane_join_us: b.join_us,
        owner_lane_finish_gap_us: b.finish_gap_us,
        owner_other_lane_worlds_us: b.other_worlds_us,
        owner_critical_lane_worlds_us: b.critical_worlds_us,
        owner_longest_world_us: b.largest_us,
        owner_critical_count: b.critical_count,
        coordinator_collect_wait_us: waits,
        owner_reply_to_last_ready_us: duration(ready[0], last)?,
    })
}
fn read_run(
    path: &Path,
    run: usize,
    mut wake: Option<&mut Vec<wake_report::Observation>>,
    mut join: Option<&mut Vec<join_report::Observation>>,
) -> Result<(Vec<Tick>, String)> {
    let mut coord = None;
    let mut peers = BTreeMap::new();
    for v in fs::read_dir(path)? {
        let p = v?.path();
        let name = p.file_name().unwrap().to_string_lossy();
        if name.starts_with("coord-") && name.ends_with(".jsonl") {
            if coord.replace(tables(&p)?).is_some() {
                return Err("duplicate coordinator".into());
            }
        } else if name.starts_with("worker-") && name.ends_with(".jsonl") {
            let id: usize = name
                .split('-')
                .nth(1)
                .ok_or("worker name missing index")?
                .parse()?;
            if peers.insert(id, tables(&p)?).is_some() {
                return Err("duplicate worker file".into());
            }
        }
    }
    let coord = coord.ok_or("missing coord file")?;
    if peers.keys().copied().ne(0..4) {
        return Err("missing peer trace".into());
    }
    let workers: Vec<_> = peers.into_values().collect();
    let loads: Vec<Load> = read(&path.join("loads.jsonl"))?;
    let summary: Value = serde_json::from_reader(File::open(path.join("summary.json"))?)?;
    let digest = summary["physics_sha256"]
        .as_str()
        .ok_or("missing physics digest")?
        .to_owned();
    if loads.len() != 280
        || loads.first().map(|l| l.tick) != Some(71)
        || loads.last().map(|l| l.tick) != Some(350)
        || loads.windows(2).any(|p| p[1].tick != p[0].tick + 1)
    {
        return Err("incomplete measured tick sequence".into());
    }
    let mut last_landing = None;
    let mut last_release = None;
    let mut landings = 0;
    let mut ticks = Vec::new();
    for (i, l) in loads.iter().enumerate() {
        let landing = i > 0
            && loads[i - 1]
                .pad_pairs
                .iter()
                .zip(l.pad_pairs)
                .any(|(before, after)| *before == 0 && after == 24);
        let class = if landing {
            landings += 1;
            last_landing = Some(l.tick);
            "landing"
        } else if last_landing.is_some_and(|s| l.tick - s <= 3) {
            "after landing"
        } else {
            "other"
        };
        let parsed = parse_tick(
            run,
            l.tick,
            class,
            coord.get(&l.tick).ok_or("missing coordinator tick")?,
            &workers,
        )?;
        if landing && let Some(rows) = wake.as_deref_mut() {
            rows.push(wake_report::observe(&parsed, &coord[&l.tick], &workers)?);
        }
        if let Some(rows) = join.as_deref_mut() {
            let release = i > 0
                && loads[i - 1]
                    .pad_pairs
                    .iter()
                    .zip(l.pad_pairs)
                    .any(|(before, after)| *before == 24 && after == 0);
            if release {
                last_release = Some(l.tick);
            }
            let event = if class != "other" {
                class
            } else if release {
                "release"
            } else if last_release.is_some_and(|s| l.tick - s <= 3) {
                "after release"
            } else {
                "steady"
            };
            rows.push(join_report::observe(&parsed, &workers[0][&l.tick], event)?);
        }
        ticks.push(parsed);
    }
    if landings != 6 {
        return Err(format!("expected six observed landings, found {landings}").into());
    }
    Ok((ticks, digest))
}

fn nearest_rank(values: &[f64], pct: usize) -> f64 {
    let mut v = values.to_vec();
    v.sort_unstable_by(f64::total_cmp);
    v[(v.len() * pct).div_ceil(100) - 1]
}
fn summary(ticks: &[&Tick]) -> Value {
    let n = ticks.len() as f64;
    let two_lane_gaps: Vec<_> = ticks
        .iter()
        .filter_map(|t| t.owner_lane_finish_gap_us)
        .collect();
    let average = |f: fn(&Tick) -> f64| ticks.iter().map(|t| f(t)).sum::<f64>() / n;
    let mean_stages: [f64; 7] =
        std::array::from_fn(|i| ticks.iter().map(|t| t.stage_us[i]).sum::<f64>() / n);
    let total: Vec<_> = ticks.iter().map(|t| t.total_us).collect();
    json!({"samples":ticks.len(),"mean_us":average(|t|t.total_us),"p50_us":nearest_rank(&total,50),"p95_us":nearest_rank(&total,95),"max_us":nearest_rank(&total,100),
        "mean_stage_us":mean_stages,"populated_owner_last_ready":ticks.iter().filter(|t|t.gate_worker==0).count(),
        "mean_owner_batch_us":average(|t|t.owner_batch_us),"mean_owner_critical_lane_cpu_us":average(|t|t.owner_critical_lane_cpu_us),
        "mean_owner_critical_lane_non_cpu_us":average(|t|t.owner_critical_lane_non_cpu_us),"mean_owner_lane_gaps_us":average(|t|t.owner_lane_gaps_us),
        "mean_owner_critical_lane_worlds_us":average(|t|t.owner_critical_lane_worlds_us),"mean_owner_other_lane_worlds_us":average(|t|t.owner_other_lane_worlds_us),
        "mean_owner_critical_lane_launch_us":average(|t|t.owner_critical_lane_launch_us),
        "mean_owner_critical_lane_interworld_us":average(|t|t.owner_critical_lane_interworld_us),
        "mean_owner_lane_join_us":average(|t|t.owner_lane_join_us),
        "both_lanes_used":two_lane_gaps.len(),
        "mean_owner_lane_finish_gap_us":(!two_lane_gaps.is_empty()).then(||two_lane_gaps.iter().sum::<f64>()/two_lane_gaps.len() as f64),
        "mean_owner_longest_world_us":average(|t|t.owner_longest_world_us),
        "mean_coordinator_collect_wait_us":average(|t|t.coordinator_collect_wait_us),"mean_owner_reply_to_last_ready_us":average(|t|t.owner_reply_to_last_ready_us)})
}

fn plot(out: &Path, bars: &[(String, [f64; 7])]) -> Result<()> {
    let root = SVGBackend::new(out, (1050, 370)).into_drawing_area();
    root.fill(&WHITE)?;
    let max = bars
        .iter()
        .map(|(_, v)| v.iter().sum::<f64>())
        .fold(0., f64::max)
        * 1.04;
    let colors = [
        RGBColor(100, 124, 144),
        RGBColor(240, 182, 80),
        RGBColor(144, 187, 158),
        RGBColor(17, 116, 136),
        RGBColor(103, 158, 177),
        RGBColor(207, 134, 84),
        RGBColor(159, 159, 181),
    ];
    let (body, legend) = root.split_vertically(295);
    let mut chart = ChartBuilder::on(&body)
        .margin(15)
        .caption("Where landing ticks spent wall time", ("sans-serif", 24))
        .x_label_area_size(40)
        .y_label_area_size(170)
        .build_cartesian_2d(0.0..max, 0.5..(bars.len() as f64 + 0.5))?;
    chart
        .configure_mesh()
        .x_desc("Instrumented time (µs)")
        .y_labels(bars.len())
        .y_label_formatter(&|v| {
            let i = v.round() as usize;
            if (v - i as f64).abs() < 1e-6 && i > 0 && i <= bars.len() {
                bars[i - 1].0.clone()
            } else {
                String::new()
            }
        })
        .disable_y_mesh()
        .draw()?;
    for (i, (_, segments)) in bars.iter().enumerate() {
        let mut x = 0.;
        for (&length, color) in segments.iter().zip(colors) {
            chart.draw_series([Rectangle::new(
                [(x, i as f64 + 0.72), (x + length, i as f64 + 1.28)],
                color.filled(),
            )])?;
            x += length;
        }
    }
    for (i, (&name, color)) in STAGES.iter().zip(colors).enumerate() {
        let x = 27 + (i % 4) as i32 * 250;
        let y = 16 + (i / 4) as i32 * 28;
        legend.draw(&Rectangle::new(
            [(x, y - 5), (x + 16, y + 5)],
            color.filled(),
        ))?;
        legend.draw(&Text::new(name, (x + 23, y - 9), ("sans-serif", 15)))?;
    }
    root.present()?;
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir_all(&args.out)?;
    let mut layouts = BTreeMap::new();
    let mut worst = BTreeMap::new();
    let mut wake = BTreeMap::new();
    let mut join = BTreeMap::new();
    let mut bars = Vec::new();
    let mut series = BufWriter::new(File::create(args.out.join("ticks.jsonl"))?);
    for layout in ["adjacent", "separated"] {
        let dir = args.traces.join(layout);
        let mut runs = BTreeMap::new();
        for entry in fs::read_dir(&dir)? {
            let entry = entry?;
            if entry.file_type()?.is_dir() {
                let n: usize = entry.file_name().to_string_lossy().parse()?;
                runs.insert(n, entry.path());
            }
        }
        if runs.is_empty() {
            return Err("no runs".into());
        }
        let mut ticks = vec![];
        let mut digest = None;
        for (i, p) in &runs {
            let (new, hash) = read_run(
                p,
                *i,
                args.scheduler.then(|| wake.entry(layout).or_default()),
                args.join.then(|| join.entry(layout).or_default()),
            )
            .map_err(|e| format!("{}: {e}", p.display()))?;
            if digest.as_ref().is_some_and(|d| d != &hash) {
                return Err("physics digest changed across repetitions".into());
            }
            digest = Some(hash);
            ticks.extend(new);
        }
        for t in &ticks {
            serde_json::to_writer(&mut series, &json!({"layout":layout,"observation":t}))?;
            writeln!(series)?;
        }
        let all: Vec<_> = ticks.iter().collect();
        let mut landing: Vec<_> = ticks.iter().filter(|t| t.class == "landing").collect();
        landing.sort_by(|a, b| b.total_us.total_cmp(&a.total_us));
        let follow: Vec<_> = ticks
            .iter()
            .filter(|t| t.class == "after landing")
            .collect();
        let top = &landing[..landing.len().div_ceil(10)];
        let last = landing[0];
        let spans: Vec<_> = STAGES
            .iter()
            .zip(last.stage_us)
            .map(|(name, us)| json!({"name":name,"us":us}))
            .collect();
        worst.insert(layout,json!({"tick":last.tick,"run":last.run,"spans":spans,"total_us":last.total_us,"gate_worker":last.gate_worker}));
        for (label, sample) in [("all", &landing[..]), ("slowest 10%", top)] {
            let means = std::array::from_fn(|i| {
                sample.iter().map(|t| t.stage_us[i]).sum::<f64>() / sample.len() as f64
            });
            bars.push((format!("{layout} · {label}"), means));
        }
        layouts.insert(layout,json!({"runs":runs.len(),"physics_sha256":digest,"all_ticks":summary(&all),"impact_ticks":summary(&landing),
            "slowest_impact_decile":summary(top),"three_following_ticks":summary(&follow),"worst_impact":last}));
    }
    series.flush()?;
    fs::write(
        args.out.join("worst-landings.json"),
        serde_json::to_vec_pretty(&worst)?,
    )?;
    fs::write(
        args.out.join("summary.json"),
        serde_json::to_vec_pretty(&json!({"stages":STAGES,"layouts":layouts}))?,
    )?;
    if args.join {
        join_report::write(&args.out, &mut join)?;
    }
    if args.scheduler {
        wake_report::write(&args.out, &mut wake)?;
    }
    plot(&args.out.join("landing-breakdown.svg"), &bars)
}

#[cfg(test)]
mod tests {
    use super::*;
    // The first lane does most of the CPU work, but the other lane finishes last after waking late.
    #[test]
    fn decomposes_the_last_finishing_lane_and_allows_a_single_executing_lane() -> Result<()> {
        for late_helper in [true, false] {
            let mut events = vec![
                json!({"name":"batch_start","ns":1000}),
                json!({"name":"batch_end","ns":101000}),
            ];
            for world in 0..8u64 {
                let helper = world == 7 && late_helper;
                let start = if helper { 90000 } else { 2000 + world * 10000 };
                let length = if helper { 6000 } else { 9000 };
                for (name, t, cpu) in [
                    ("native_start", start, 1000),
                    ("native_end", start + length, 1000 + length - 1000),
                ] {
                    events.push(
                        json!({"name":name,"ns":t,"cpu_ns":cpu,"world":world,"helper":helper}),
                    );
                }
            }
            let r: Row = serde_json::from_value(json!({"tick":1,"events":events}))?;
            let b = owner_batch(&r)?;
            assert_eq!(b.wall_us, b.cpu_us + b.off_cpu_us + b.gaps_us);
            assert_eq!(b.gaps_us, b.launch_us + b.interworld_us + b.join_us);
            assert_eq!(b.critical_count, if late_helper { 1 } else { 8 });
            assert_eq!(b.finish_gap_us, late_helper.then_some(25.));
        }
        Ok(())
    }
}
