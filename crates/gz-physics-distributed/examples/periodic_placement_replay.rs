// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show apples-to-apples periodic placement measurements with the actual migrating scene.
#[path = "support/cadence_replay.rs"]
mod cadence_replay;
#[path = "support/worker_scene_display.rs"]
mod display;
#[path = "support/changing_worlds.rs"]
#[allow(dead_code, unused_imports)]
mod fixture;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::{Parser, ValueEnum};
use display::{BodyLayer, Frame, OWNER_COLORS};
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long)]
    evidence: PathBuf,
    #[arg(long, required_unless_present = "cadence", conflicts_with = "cadence")]
    trials: Option<PathBuf>,
    #[arg(long)]
    cadence: bool,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Row {
    #[serde(flatten)]
    physical: Frame,
    decision: Option<Decision>,
    window_counters: Vec<u64>,
    ineligible_windows: Option<u64>,
    deferred_windows: Option<u64>,
    elapsed_us: f64,
    events: Vec<String>,
    pids: [u32; 4],
}
#[derive(Clone, Deserialize, Serialize)]
struct Decision {
    tick: u64,
    source: u8,
    destination: u8,
    members: Vec<u64>,
    before_lower_ns: u64,
    after_serial_peak_ns: u64,
    estimated_payback_ticks: u64,
}
fn read<T: serde::de::DeserializeOwned>(p: &Path) -> Result<T> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
fn lines<T: serde::de::DeserializeOwned>(p: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|s| Ok(serde_json::from_str(&s?)?))
        .collect()
}
fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    let i = v.len() / 2;
    if v.len().is_multiple_of(2) {
        (v[i - 1] + v[i]) / 2.
    } else {
        v[i]
    }
}
fn bench(p: &Path, source: &Value) -> Result<Vec<Value>> {
    let proof: Value = read(&p.join("proof.json"))?;
    for k in ["native_sha256", "helpers", "initial", "layout", "policy"] {
        if proof[k] != source[k] {
            return Err(format!("benchmark and recording differ in {k}").into());
        }
    }
    let rounds = proof["rounds"].as_u64().ok_or("missing completed rounds")? as usize;
    if rounds < 2 {
        return Err("requires replicated matched trials".into());
    }
    let rows: Vec<Value> = lines(&p.join("trials.jsonl"))?;
    let mut stats = vec![];
    for mode in ["fixed", "observe", "automatic"] {
        let mut times = BTreeMap::new();
        for r in &rows {
            let d = &r["data"];
            if d["mode"] != mode {
                continue;
            }
            let pair = (
                d["full_us"].as_f64().ok_or("bad elapsed")?,
                d["steady_p95_us"].as_f64().ok_or("bad steady p95")?,
            );
            if times.insert(r["round"].as_u64().unwrap(), pair).is_some() {
                return Err("duplicate timing".into());
            }
        }
        if times.keys().copied().ne(0..rounds as u64) {
            return Err("incomplete matched rounds".into());
        }
        let mut elapsed = times.values().map(|x| x.0).collect::<Vec<_>>();
        let mut steady = times.values().map(|x| x.1).collect::<Vec<_>>();
        stats.push(json!({"mode":mode,"rounds":rounds,"full_median_ms":median(&mut elapsed)/1000.,"median_steady_p95_ms":median(&mut steady)/1000.}));
    }
    Ok(stats)
}
fn main() -> Result<()> {
    let a = Args::parse();
    let proof: Value = read(&a.evidence.join("proof.json"))?;
    if proof["layout"] != a.layout.to_possible_value().unwrap().get_name() {
        return Err("unexpected layout evidence".into());
    }
    let stats = a
        .trials
        .as_deref()
        .map(|p| bench(p, &proof))
        .transpose()?
        .unwrap_or_default();
    let rows: Vec<Row> = lines(&a.evidence.join("frames.jsonl"))?;
    if rows.len() != 351 {
        return Err("missing physical observations".into());
    }
    let cadence = a
        .cadence
        .then(|| cadence_replay::Cadence::read(&a.evidence, &proof, &rows))
        .transpose()?;
    let groups = fixture::seeds(a.layout);
    let pads: Vec<u64> = groups
        .iter()
        .filter(|g| g.len() == 25)
        .map(|g| g[0].id)
        .collect();
    let mut layer = BodyLayer::new(&groups, 0, "periodic", 0.);
    let mut hash = Sha256::new();
    let mut stream = vec![];
    let mut timeline = vec![];
    let mut last_move: Option<&Decision> = None;
    let mut last_event: Option<(u64, &str)> = None;
    let mut expected = rows[0].physical.owners.clone();
    let mut totals = [0usize; 2];
    for (i, r) in rows.iter().enumerate() {
        let f = &r.physical;
        let t = f.tick;
        if t != i as u64
            || f.bodies.len() != 62
            || f.owners.len() != 62
            || f.bodies
                .iter()
                .enumerate()
                .any(|(k, b)| b.id != k as u64 + 1)
        {
            return Err("bad body or time order".into());
        }
        if i > 0 {
            hash.update(bincode::serialize(&(t, &f.bodies, &f.pairs, &f.floors))?);
            totals[0] += f.pairs.len();
            totals[1] += f.floors.len();
        }
        if let Some(d) = &r.decision {
            if d.tick != t
                || d.destination == d.source
                || d.members.is_empty()
                || d.members.iter().collect::<BTreeSet<_>>().len() != d.members.len()
            {
                return Err("invalid scheduler move".into());
            }
            for &id in &d.members {
                let w = expected
                    .get_mut(id as usize - 1)
                    .ok_or("unknown moved member")?;
                if *w != d.source {
                    return Err("source ownership mismatch".into());
                }
                *w = d.destination;
            }
            last_move = Some(d);
        }
        if expected != f.owners {
            return Err("real owner map inconsistent with move history".into());
        }
        if i > 0 {
            let previous = rows[i - 1].pids;
            let changed = previous
                .iter()
                .zip(r.pids)
                .filter(|(x, y)| **x != *y)
                .count();
            let should_change = if r.events.iter().any(|e| e == "fresh_pool") {
                if r.window_counters.iter().any(|&n| n != 0)
                    || previous.iter().any(|p| r.pids.contains(p))
                {
                    return Err("fresh checkpoint failed".into());
                }
                4
            } else if r.events.iter().any(|e| e == "worker_kill") {
                1
            } else {
                0
            };
            if changed != should_change {
                return Err("recovery PIDs mismatch".into());
            }
        }
        if let Some(e) = r.events.last() {
            last_event = Some((t, e));
        }
        let mut v = View::default();
        v.cuboid(
            2,
            "shared_ground_display",
            [0., 0., -0.07],
            [25., 21., 0.14],
            [0.28, 0.34, 0.40, 1.],
        );
        layer.draw(&mut v, f, &f.owners)?;
        let base = 2_000_000 + t * 80;
        for (w, pid) in r.pids.iter().enumerate() {
            let n = f.owners.iter().filter(|&&o| o as usize == w).count();
            let len = (4.0 * n as f64 / 62.).max(0.06);
            v.cuboid(
                base + 2 * w as u64,
                &format!("tick_{t}_worker_{w}_pid_{pid}_owns_{n}_bodies"),
                [-10.8 + w as f64 * 5.5 + len / 2., 5.8, 0.22],
                [len, 0.38, 0.35],
                OWNER_COLORS[w],
            );
        }
        for (j, &pad) in pads.iter().enumerate() {
            let n = f
                .pairs
                .iter()
                .filter(|p| p.a == pad && p.normal_impulse > 0.)
                .count();
            if f.loaded_pairs[j] != n {
                return Err("wrong positive finite pair count".into());
            }
            let len = (3.5 * n as f64 / 24.).max(0.045);
            v.cuboid(
                base + 12 + 2 * j as u64,
                &format!("tick_{t}_pad_{pad}_positive_contacts_{n}"),
                [-10.8 + 5.5 * j as f64 + len / 2., 6.65, 0.22],
                [len, 0.34, 0.35],
                [0.94, 0.74, 0.26, 1.],
            );
        }
        let [started, done, interrupted, unavailable, extra @ ..] = r.window_counters.as_slice()
        else {
            return Err("missing periodic placement counters".into());
        };
        let mut suffix = match extra {
            [] => String::new(),
            [n] => format!("_ineligible_windows_{n}"),
            _ => return Err("unknown periodic counter schema".into()),
        };
        if let (Some(ineligible), Some(deferred)) = (r.ineligible_windows, r.deferred_windows) {
            suffix = format!("_ineligible_windows_{ineligible}_deferred_windows_{deferred}");
        }
        let open = *started > done + interrupted;
        v.cuboid(base+18,&format!("tick_{t}_windows_started_{started}_completed_{done}_interrupted_{interrupted}_unavailable_probes_{unavailable}{suffix}"),[1.1,6.65,0.21],[0.8,0.34,0.36],if open{[0.19,0.72,0.51,1.]}else{[0.48,0.55,0.64,1.]});
        let label = last_event
            .map(|(at, e)| format!("{e}_at_tick_{at}"))
            .unwrap_or_else(|| "before_faults".into());
        v.cuboid(
            base + 20,
            &label,
            [3.0, 6.65, 0.21],
            [0.6, 0.34, 0.36],
            if !r.events.is_empty() {
                [0.98, 0.42, 0.18, 1.]
            } else {
                [0.50, 0.58, 0.67, 1.]
            },
        );
        if let Some(d) = last_move {
            let name = format!(
                "last_move_tick_{}_w{}_to_w{}_{}_members_{}_tick_estimated_payback",
                d.tick,
                d.source,
                d.destination,
                d.members.len(),
                d.estimated_payback_ticks
            );
            v.cuboid(
                base + 22,
                &name,
                [6.1, 6.65, 0.21],
                [0.22 * d.estimated_payback_ticks as f64, 0.34, 0.36],
                OWNER_COLORS[d.destination as usize],
            );
        }
        if let Some(c) = &cadence {
            c.draw(&mut v, t);
        }
        for (j, s) in stats.iter().enumerate() {
            let mode = s["mode"].as_str().unwrap();
            let full = s["full_median_ms"].as_f64().unwrap();
            let p95 = s["median_steady_p95_ms"].as_f64().unwrap();
            let len = full / 80.;
            v.cuboid(
                base + 30 + 2 * j as u64,
                &format!(
                    "{}_matched_{mode}_median_full_{full:.3}_ms_median_steady_p95_{p95:.3}_ms",
                    s["rounds"]
                ),
                [-10.8 + j as f64 * 7.2 + len / 2., -6.30, 0.22],
                [len, 0.40, 0.36],
                [
                    [0.54, 0.60, 0.68, 1.],
                    [0.66, 0.44, 0.78, 1.],
                    [0.22, 0.70, 0.51, 1.],
                ][j],
            );
        }
        let elapsed = r.elapsed_us / 1000.;
        // The bounded scale keeps exceptional migration costs visible without hiding the scene.
        let len = (2.0 * elapsed.ln_1p()).max(0.05);
        v.cuboid(
            base + 40,
            &format!("native_tick_{t}_measured_step_{elapsed:.6}_ms_log_scale"),
            [-10.8 + len / 2., -7.15, 0.2],
            [len, 0.35, 0.34],
            [0.32, 0.66, 0.83, 1.],
        );
        timeline.push(json!({"tick":t,"window_open":open,"last_move":last_move,"decision":r.decision,"events":r.events,"event_label":label}));
        stream.push(v.finish(t));
    }
    if proof["native_sha256"] != format!("{:x}", hash.finalize())
        || proof["native_pairs_per_trial"] != totals[0]
        || proof["native_floors_per_trial"] != totals[1]
    {
        return Err("independent native physics proof mismatch".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"proof":proof,"benchmark":stats,"cadence":cadence,"samples":timeline,"pad_ids":pads}),
        )?,
    )?;
    let qualification = if a.cadence {
        "the sampling and contact-transition timeline"
    } else {
        "the matched three-arm timing benchmark"
    };
    println!(
        "Verified and exported 351 frames, all four process IDs, contact receipts, native physics SHA, actual scheduler-selected moves and {qualification}."
    );

    Ok(())
}
