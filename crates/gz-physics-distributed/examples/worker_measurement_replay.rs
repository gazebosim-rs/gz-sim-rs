// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Overlay paired worker measurements on a verified physical encounter in a gz-web-rs recording.
#[path = "support/changing_worlds.rs"]
#[allow(dead_code, unused_imports)]
mod fixture;
#[path = "support/worker_measurements.rs"]
mod measurements;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::{Parser, ValueEnum};
#[path = "support/worker_scene_display.rs"]
mod display;
use display::{BodyLayer, Frame};
use primitive_replay_view::View;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
#[derive(Clone, Copy, ValueEnum)]
enum Experiment {
    Wire,
    Digest,
    Sparse,
    Scheduled,
    Checkpoint,
    Batch,
    Balance,
    Retry,
    SceneRetry,
    Cold,
}
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    experiment: Experiment,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    /// Qualified distributed.jsonl; all fields are preserved for trajectory verification.
    #[arg(long)]
    trace: PathBuf,
    /// Run directory containing the paired benchmark and observed ownership sidecars, when applicable.
    #[arg(long)]
    evidence: PathBuf,
    /// Number of complete interleaved retry pairs to read from each four-CPU campaign.
    #[arg(long,default_value_t=16,value_parser=clap::value_parser!(u32).range(2..))]
    retry_prefix: u32,
    /// Use the shared-host placement trials in place of the independent physical-core campaign.
    #[arg(long)]
    shared_balance: bool,
    /// Write comparison.json and a new replay/ below this directory.
    directory: PathBuf,
}
const ARM_COLORS: [[f64; 4]; 2] = [[0.37, 0.68, 0.95, 1.], [0.82, 0.71, 0.28, 1.]];
fn main() -> Result<()> {
    let args = Args::parse();
    let layout = args.layout.to_possible_value().unwrap();
    let layout = layout.get_name();
    let (cmp, owners) = match args.experiment {
        Experiment::Checkpoint => {
            let (c, m) = measurements::checkpoint(&args.evidence, layout)?;
            (c, Some(m))
        }
        Experiment::Scheduled => {
            let (c, m) = measurements::scheduled(&args.evidence, layout)?;
            (c, Some(m))
        }
        Experiment::Sparse => {
            let (c, m) = measurements::sparse(&args.evidence, layout)?;
            (c, Some(m))
        }
        Experiment::Digest => (
            measurements::digest(&args.evidence, layout, &args.trace)?,
            None,
        ),
        Experiment::Wire => (
            measurements::wire(&args.evidence, layout, &args.trace)?,
            None,
        ),
        Experiment::Batch => (
            measurements::batch(&args.evidence, layout, &args.trace)?,
            None,
        ),
        Experiment::Retry => (
            measurements::retry(&args.evidence, layout, args.retry_prefix as usize)?,
            None,
        ),
        Experiment::SceneRetry => {
            let (c, map) = measurements::scene_retry(&args.evidence, layout)?;
            (c, Some(map))
        }
        Experiment::Balance => {
            let (c, map) = measurements::balance(&args.evidence, layout, args.shared_balance)?;
            (c, Some(map))
        }
        Experiment::Cold => {
            let (c, map) = measurements::cold(&args.evidence, layout)?;
            (c, Some(map))
        }
    };
    let groups = fixture::seeds(args.layout);
    let dual = owners.is_some()
        && !matches!(
            args.experiment,
            Experiment::Sparse | Experiment::Scheduled | Experiment::Checkpoint
        );
    let mut stream = vec![];
    let mut physics = Sha256::new();
    let mut source = Sha256::new();
    let mut layers = [0, 1].map(|arm| {
        BodyLayer::new(
            &groups,
            arm,
            if dual { cmp.arms[arm] } else { "common" },
            if dual { [6.4, -6.4][arm] } else { 0. },
        )
    });
    for (tick, line) in BufReader::new(File::open(&args.trace)?).lines().enumerate() {
        let line = line?;
        source.update(line.as_bytes());
        source.update(b"\n");
        let f: Frame = serde_json::from_str(&line)?;
        if f.tick != tick as u64
            || f.bodies.len() != 62
            || f.owners.len() != 62
            || f.bodies
                .iter()
                .enumerate()
                .any(|(i, b)| b.id != i as u64 + 1)
        {
            return Err("invalid fixture frame".into());
        }
        if tick > 0 {
            physics.update(bincode::serialize(&(
                f.tick, &f.bodies, &f.pairs, &f.floors,
            ))?)
        }
        let mut view = View::default();
        view.cuboid(
            2,
            "display_floor",
            [0., 0., -0.07],
            [24., if dual { 24. } else { 17. }, 0.14],
            [0.31, 0.36, 0.42, 1.],
        );
        for (arm, layer) in layers.iter_mut().enumerate().take(if dual { 2 } else { 1 }) {
            let map = owners
                .as_ref()
                .map_or(f.owners.as_slice(), |m| m[arm].at(f.tick));
            layer.draw(&mut view, &f, map)?;
        }
        for (i, g) in cmp.gauges.iter().enumerate() {
            let visible = g.window.map_or(
                !dual || i == tick * cmp.gauges.len() / 351,
                |[from, to, _]| (from..to).contains(&f.tick),
            );
            if !visible {
                continue;
            }
            for (arm, &value) in g.values.iter().enumerate() {
                let width = 8. * value / g.values[0].max(g.values[1]);
                let x = if dual { -4. } else { [-9., 1.][arm] };
                let y = if dual {
                    [1., -11.8][arm]
                } else {
                    -5. - g.window.map_or(i as f64, |w| w[2] as f64) * 0.75
                };
                // A square cap at the origin marks a measured zero. Positive bars share a linear scale.
                view.cuboid(
                    10000 + (i * 4 + arm * 2) as u64,
                    &format!(
                        "{}_{}_{value:.3}_{}_n{}",
                        g.labels.unwrap_or(cmp.arms)[arm],
                        g.name,
                        g.unit,
                        g.samples
                    ),
                    [x + width / 2., y, 0.24],
                    [
                        if value == 0. { 0.11 } else { width },
                        0.32,
                        if value == 0. { 0.44 } else { 0.25 },
                    ],
                    ARM_COLORS[arm],
                );
            }
        }
        if !dual {
            for (i, n) in f.loaded_pairs.iter().enumerate() {
                view.cuboid(
                    12000 + 2 * i as u64,
                    &format!("slab_{}_loaded_contacts_out_of_24", i + 1),
                    [-1. + i as f64 * 2., 0., *n as f64 / 12.],
                    [0.5, 0.35, 0.15],
                    [0.1, 0.74, 0.62, 1.],
                );
            }
        }
        if let Some((i, (tick, name))) = cmp
            .events
            .iter()
            .enumerate()
            .rev()
            .find(|(_, (t, _))| *t <= f.tick)
        {
            view.cuboid(
                13000 + 2 * i as u64,
                &format!("tick_{tick}_{name}"),
                [0., 1.8, 0.29],
                [0.8, 0.55, 0.58],
                [0.84, 0.41, 0.21, 1.],
            );
        }
        stream.push(view.finish(f.tick));
    }
    let digest = format!("{:x}", physics.finalize());
    if stream.len() != 351 || cmp.expected_physics.as_ref().is_some_and(|x| x != &digest) {
        return Err(format!("incomplete or divergent measured trajectory: {digest}").into());
    }
    fs::create_dir_all(&args.directory)?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        args.directory.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"layout":layout,"trajectory_sha256":digest,"trace_sha256":format!("{:x}",source.finalize()),"arms":cmp.arms,"paired_measurements":cmp.gauges,"events":cmp.events,"evidence":cmp.evidence,"separate_owners":dual,"owners":owners}),
        )?,
    )?;
    println!(
        "Verified 351 physical frames and {} paired measures: {digest}",
        cmp.gauges.len()
    );
    Ok(())
}
