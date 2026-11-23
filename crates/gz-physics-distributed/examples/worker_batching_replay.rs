// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay dense qualification and the settled-stack benchmark with measured release timings.
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, ContactImpulse, GroundSupport, Primitive},
};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured};
use plotters::prelude::*;
use primitive_replay_view::View;
use sdformat::NoResources;
use serde::{Deserialize, de::DeserializeOwned};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// Contains dense-benchmark.jsonl, stacks-benchmark.jsonl and batched/distributed.jsonl.
    directory: PathBuf,
    #[arg(long)]
    worker: PathBuf,
}
#[derive(Deserialize)]
struct Trial {
    round: usize,
    build: String,
    elapsed_ms: f64,
    tick_p95_us: f64,
    ticks: u64,
    bodies: usize,
    warmup_ticks: Option<u64>,
    physics_sha256: Option<String>,
}
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<u8>,
    fine_ids: Vec<u64>,
    impulses: Vec<ContactImpulse>,
    supports: Vec<GroundSupport>,
}
fn read<T: DeserializeOwned>(p: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect()
}
const BUILDS: [&str; 2] = ["baseline", "batched"];
const DENSE_END: u64 = 230;
const STACK_WARMUP: u64 = 110;
const STACK_END: u64 = 210;
fn timing(p: &Path, ticks: u64, bodies: usize) -> Result<[Vec<Trial>; 2]> {
    let mut paired: [Vec<Trial>; 2] = Default::default();
    for trial in read::<Trial>(p)? {
        let build = BUILDS
            .iter()
            .position(|&s| s == trial.build)
            .ok_or("unknown benchmark label")?;
        if trial.ticks != ticks
            || trial.bodies != bodies
            || ![trial.elapsed_ms, trial.tick_p95_us]
                .into_iter()
                .all(|v| v.is_finite() && v > 0.)
        {
            return Err("unmatched workload or invalid timing".into());
        }
        paired[build].push(trial);
    }
    for list in &mut paired {
        list.sort_by_key(|t| t.round);
        if list.len() < 2 || list.iter().enumerate().any(|(i, t)| i + 1 != t.round) {
            return Err("missing or duplicate round".into());
        }
    }
    if paired[0].len() != paired[1].len() {
        return Err("unpaired benchmark".into());
    }
    Ok(paired)
}
fn median(mut samples: Vec<f64>) -> f64 {
    samples.sort_by(f64::total_cmp);
    let i = samples.len() / 2;
    (samples[(samples.len() - 1) / 2] + samples[i]) * 0.5
}
fn stacks(worker: &Path, trials: &[Vec<Trial>; 2]) -> Result<Vec<Sample>> {
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_diagonal_stacks.sdf"),
        "distributed_diagonal_stacks.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedIslands(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let mut digest = Sha256::new();
    let mut frames = Vec::new();
    for tick in 0..=STACK_END {
        if tick != 0 {
            sim.step_exact(1)?;
        }
        let scene = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .ok_or("missing distributed SDF backend")?;
        let frame = scene.pool_frame().ok_or("missing four-worker frame")?;
        if frame.tick != tick || frame.workers.len() != 4 {
            return Err("incomplete SDF publication".into());
        }
        let mut bodies: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.bodies.iter().cloned())
            .collect();
        bodies.sort_by_key(|b| b.id);
        let mut impulses: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter().cloned())
            .collect();
        impulses.sort_by_key(|p| (p.a, p.b));
        let mut supports: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.supports.iter().cloned())
            .collect();
        supports.sort_by_key(|p| p.body);
        if tick > STACK_WARMUP {
            // Match the loaded benchmark's canonical serialization at every measured barrier.
            digest.update(bincode::serialize(&(tick, &bodies, &impulses, &supports))?);
        }
        frames.push(Sample {
            tick,
            owners: bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect(),
            bodies,
            impulses,
            supports,
            fine_ids: scene
                .last_island_refined_bodies()
                .unwrap_or_default()
                .to_vec(),
        });
    }
    let digest = format!("{:x}", digest.finalize());
    if trials
        .iter()
        .flatten()
        .any(|t| t.warmup_ticks != Some(STACK_WARMUP) || t.physics_sha256.as_ref() != Some(&digest))
    {
        return Err(format!("stack capture failed measured-window physics digest {digest}").into());
    }
    println!("Both stack builds and the replay have the same measured physics digest {digest}.");
    Ok(frames)
}
const GOLD: [f64; 4] = [0.93, 0.52, 0.15, 1.];
const TEAL: [f64; 4] = [0.08, 0.69, 0.7, 1.];
fn gauges(view: &mut View, median_us: [f64; 2], ground: bool) {
    let span = 8.;
    let max_us = (median_us.into_iter().fold(0., f64::max) / 100.).ceil() * 100.;
    let y = if ground { -5. } else { -1. };
    let base = if ground { 0.5 } else { -0.9 };
    for (i, value) in median_us.into_iter().enumerate() {
        let start = -span * 0.5;
        let width = span * value / max_us;
        let z = base - i as f64 * 0.46;
        let id = 8 + i as u64 * 4;
        view.cuboid(
            id,
            &format!("scale_{}_0_to_{max_us:.0}_us", BUILDS[i]),
            [0., y, z],
            [span, 0.12, 0.08],
            [0.42, 0.47, 0.52, 1.],
        );
        view.cuboid(
            id + 2,
            &format!("{}_median_tick_p95_{value:.1}_us", BUILDS[i]),
            [start + 0.5 * width, y, z + 0.11],
            [width, 0.23, 0.17],
            [GOLD, TEAL][i],
        );
    }
}
#[derive(Deserialize)]
struct Recovery {
    migration_tick: u64,
    lost_ack_retried_tick: u64,
    worker_replacement_barrier: u64,
    archive_restore_barrier: u64,
}
fn export(
    output: &Path,
    frames: &[Sample],
    times: &[Vec<Trial>; 2],
    recovery: Option<&Recovery>,
) -> Result<()> {
    let ground = recovery.is_none();
    let end = if ground { STACK_END } else { DENSE_END };
    if frames.len() as u64 != end + 1 {
        return Err("expected complete physics trace".into());
    }
    let mut geometry: BTreeMap<u64, [f64; 3]> = BTreeMap::new();
    let center = if ground {
        [0., 0., 0.]
    } else {
        let bounds = frames.iter().flat_map(|f| &f.bodies).fold(
            ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
            |(mut lo, mut hi), b| {
                for i in 0..3 {
                    lo[i] = lo[i].min(b.position[i]);
                    hi[i] = hi[i].max(b.position[i]);
                }
                (lo, hi)
            },
        );
        std::array::from_fn(|i| 0.5 * (bounds.0[i] + bounds.1[i]))
    };
    let medians = times
        .each_ref()
        .map(|v| median(v.iter().map(|t| t.tick_p95_us).collect()));
    let colors = [
        [0.07, 0.42, 0.85, 1.],
        [0.57, 0.37, 0.76, 1.],
        [0.16, 0.66, 0.50, 1.],
        [0.83, 0.44, 0.20, 1.],
    ];
    let mut next_id = 40;
    let mut identities = BTreeMap::new();
    let mut stream = Vec::new();
    for (tick, f) in frames.iter().enumerate() {
        let ids: BTreeSet<_> = f.bodies.iter().map(|b| b.id).collect();
        if f.tick != tick as u64
            || ids.len() != if ground { 8 } else { 26 }
            || f.bodies.len() != ids.len()
            || f.owners.len() != ids.len()
            || !f.fine_ids.iter().all(|id| ids.contains(id))
        {
            return Err("incomplete or out-of-order frame".into());
        }
        if tick == 0 {
            geometry.extend(f.bodies.iter().map(|b| (b.id, b.size)));
        }
        let mut view = View::default();
        if ground {
            view.cuboid(
                2,
                "ground",
                [0., 0., -0.03],
                [16., 16., 0.06],
                [0.32, 0.35, 0.40, 1.],
            );
        }
        if !ground || f.tick > STACK_WARMUP {
            gauges(&mut view, medians, ground);
        }
        if !ground
            && tick > 0
            && !(f.impulses.len() == 1
                && f.impulses[0].a == 1
                && f.impulses[0].b == 2
                && f.impulses[0].normal_impulse > 0.
                && f.supports.is_empty())
        {
            return Err("missing dense loaded contact".into());
        }
        if let Some(r) = recovery {
            // Publication clocks progress only once after a successful retry. A
            // recovered worker or pool first contributes to the *next* commit.
            let events = [
                (r.migration_tick, "loaded_migration"),
                (r.lost_ack_retried_tick, "acknowledgement_retry"),
                (
                    r.worker_replacement_barrier
                        .checked_add(1)
                        .ok_or("worker clock overflow")?,
                    "first_replacement_commit",
                ),
                (
                    r.archive_restore_barrier
                        .checked_add(1)
                        .ok_or("pool clock overflow")?,
                    "first_restored_pool_commit",
                ),
            ];
            for (i, (at, name)) in events.iter().enumerate() {
                if !(1..=end).contains(at) {
                    return Err("recovery event outside trace".into());
                }
                if f.tick >= *at {
                    view.cuboid(
                        20 + 2 * i as u64,
                        &format!("{name}_tick_{at}"),
                        [-3.8 + i as f64 * 0.55, -1., -1.75],
                        [0.34, 0.27, 0.24],
                        [0.18, 0.75, 0.48, 1.],
                    );
                }
            }
        }
        for (b, &owner) in f.bodies.iter().zip(&f.owners) {
            if owner >= 4
                || b.primitive != Primitive::Sphere
                || b.shape().is_err()
                || geometry.get(&b.id) != Some(&b.size)
            {
                return Err("unknown owner, body or sphere size".into());
            }
            let fine = f.fine_ids.contains(&b.id);
            let (label, color) = if ground {
                ("stack", colors[owner as usize])
            } else if b.id <= 2 {
                ("loaded_pair", colors[owner as usize])
            } else if fine {
                ("fine_fleet", [0.88, 0.42, 0.12, 1.])
            } else {
                ("coarse_fleet", [0.44, 0.62, 0.78, 1.])
            };
            let state = (owner, label);
            let identity = identities.entry(b.id).or_insert((state, 0));
            if identity.1 == 0 || identity.0 != state {
                *identity = (state, next_id);
                next_id += 2;
            }
            let pos = std::array::from_fn(|i| b.position[i] - center[i]);
            view.add(
                identity.1,
                &format!("{label}_{}_worker_{owner}", b.id),
                pos,
                b.orientation,
                json!({"kind":"sphere","radius":0.5*b.size[0]}),
                color,
            );
        }
        stream.push(view.finish(f.tick));
    }
    gz_sim_cli::export_archive(output, Default::default(), stream)?;
    println!(
        "{} frames recorded at {}. Baseline and batched median tick p95s {:.2} and {:.2} us.",
        frames.len(),
        output.display(),
        medians[0],
        medians[1]
    );
    Ok(())
}

fn plot(path: &Path, dense: &[Vec<Trial>; 2], stacks: &[Vec<Trial>; 2]) -> Result<()> {
    let page = SVGBackend::new(path, (1040, 430)).into_drawing_area();
    page.fill(&WHITE)?;
    let panels = page.split_evenly((1, 2));
    for ((panel, trials), caption) in panels.into_iter().zip([dense, stacks]).zip([
        "26-body encounter · 230 ticks",
        "Four loaded stacks · 100 ticks",
    ]) {
        let n = trials[0].len();
        let ymax = trials
            .iter()
            .flatten()
            .map(|t| t.elapsed_ms)
            .fold(0., f64::max);
        let mut chart = ChartBuilder::on(&panel)
            .margin(20)
            .x_label_area_size(45)
            .y_label_area_size(50)
            .caption(caption, ("sans-serif", 22))
            .build_cartesian_2d(0..n + 1, 0.0..(ymax / 5.).ceil() * 5. + 5.)?;
        chart
            .configure_mesh()
            .x_desc("Paired release trial")
            .y_desc("Total step time (ms)")
            .disable_x_mesh()
            .max_light_lines(1)
            .bold_line_style(RGBColor(228, 232, 235))
            .label_style(("sans-serif", 13))
            .axis_desc_style(("sans-serif", 15))
            .draw()?;
        for (before, after) in trials[0].iter().zip(&trials[1]) {
            chart.draw_series([PathElement::new(
                [
                    (before.round, before.elapsed_ms),
                    (after.round, after.elapsed_ms),
                ],
                RGBColor(150, 161, 169).stroke_width(2),
            )])?;
        }
        for (i, color) in [RGBColor(199, 117, 20), RGBColor(0, 143, 150)]
            .into_iter()
            .enumerate()
        {
            let elapsed = median(trials[i].iter().map(|t| t.elapsed_ms).collect());
            chart
                .draw_series(
                    trials[i]
                        .iter()
                        .map(|t| Circle::new((t.round, t.elapsed_ms), 5, color.filled())),
                )?
                .label(format!("{}   {elapsed:.2} ms median", BUILDS[i]))
                .legend(move |(x, y)| Circle::new((x + 8, y), 5, color.filled()));
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::LowerLeft)
            .background_style(WHITE.mix(0.94))
            .draw()?;
    }
    page.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    let dir = &args.directory;
    let dense_times = timing(&dir.join("dense-benchmark.jsonl"), DENSE_END, 26)?;
    let stacks_times = timing(
        &dir.join("stacks-benchmark.jsonl"),
        STACK_END - STACK_WARMUP,
        8,
    )?;
    let dense = read(&dir.join("batched/distributed.jsonl"))?;
    let stack_samples = stacks(&args.worker, &stacks_times)?;
    let recovery: Recovery =
        serde_json::from_reader(File::open(dir.join("batched/qualification.json"))?)?;
    export(
        &dir.join("dense-replay"),
        &dense,
        &dense_times,
        Some(&recovery),
    )?;
    export(
        &dir.join("stacks-replay"),
        &stack_samples,
        &stacks_times,
        None,
    )?;
    plot(&dir.join("paired-timings.svg"), &dense_times, &stacks_times)
}
