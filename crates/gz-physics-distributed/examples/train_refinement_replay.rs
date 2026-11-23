// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display two native warm-train traces at identical simulation times.
use clap::Parser;
use gz_physics_distributed::physics::{Body, Primitive};
use serde::Deserialize;
use serde_json::json;
use std::{
    collections::BTreeMap,
    error::Error,
    f64::consts::FRAC_1_SQRT_2,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
    time::Duration,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Coarser warm_join_sweep JSONL trace.
    coarse: PathBuf,
    /// Fixed fine-step or native adaptive trace for the same seed.
    comparison: PathBuf,
    /// The second trace uses prospective refinement on the same public clock.
    #[arg(long)]
    adaptive: bool,
    /// Display a four-worker adaptive trace, coloring the comparison boxes by their committed owner.
    #[arg(long, requires = "adaptive")]
    worker_owners: bool,
    /// New gzweb archive directory.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Sample {
    tick: u32,
    substeps: Option<u32>,
    owners: Option<[usize; 4]>,
    time_s: f64,
    gap_m: f64,
    bodies: Vec<Body>,
}

struct Trace {
    step_ns: u64,
    frames: BTreeMap<u64, Sample>,
}

fn read(path: &Path) -> Result<Trace> {
    let mut frames = BTreeMap::new();
    let mut previous: Option<(u32, u64)> = None;
    let mut step_ns = None;
    for line in BufReader::new(File::open(path)?).lines() {
        let mut sample: Sample = serde_json::from_str(&line?)?;
        let ns = u64::try_from(Duration::try_from_secs_f64(sample.time_s)?.as_nanos())?;
        sample.bodies.sort_unstable_by_key(|b| b.id);
        if !sample.gap_m.is_finite()
            || sample.bodies.iter().map(|b| b.id).ne(1..=4)
            || sample.bodies.iter().any(|b| b.primitive != Primitive::Box)
        {
            return Err("expected a finite gap and four identified train boxes".into());
        }
        if let Some((tick, time)) = previous {
            let step = ns.checked_sub(time).ok_or("time reversed")?;
            if tick.checked_add(1) != Some(sample.tick)
                || step == 0
                || step_ns.replace(step).is_some_and(|dt| dt != step)
            {
                return Err("trace skipped a sample or changed its timestep".into());
            }
        }
        previous = Some((sample.tick, ns));
        frames.insert(ns, sample);
    }
    Ok(Trace {
        step_ns: step_ns.ok_or("expected at least two samples")?,
        frames,
    })
}

fn main() -> Result<()> {
    let args = Args::parse();
    let coarse = read(&args.coarse)?;
    let comparison = read(&args.comparison)?;
    if args.adaptive {
        if comparison.step_ns != coarse.step_ns
            || !comparison
                .frames
                .values()
                .all(|frame| matches!(frame.substeps, Some(n) if n > 0))
        {
            return Err(
                "adaptive comparison requires matching public steps and native solve counts".into(),
            );
        }
    } else if coarse.step_ns <= comparison.step_ns
        || !coarse.step_ns.is_multiple_of(comparison.step_ns)
    {
        return Err("expected a nested pair of simulation steps".into());
    }
    let coarse_label = format!("{}ms", coarse.step_ns as f64 / 1e6);
    let comparison_label = if args.worker_owners {
        if comparison
            .frames
            .values()
            .any(|s| s.owners.is_none_or(|o| o.iter().any(|&w| w >= 4)))
        {
            return Err(
                "worker comparison requires four-worker ownership metadata for every sample".into(),
            );
        }
        format!("distributed_adaptive_{coarse_label}")
    } else if args.adaptive {
        format!("adaptive_{coarse_label}")
    } else {
        format!("{}ms", comparison.step_ns as f64 / 1e6)
    };
    let (&center, closest) = coarse
        .frames
        .iter()
        .min_by(|a, b| a.1.gap_m.total_cmp(&b.1.gap_m))
        .ok_or("empty coarse trace")?;
    // A common origin and rigid display transforms preserve each train's relative geometry.
    let origin: [f64; 3] =
        std::array::from_fn(|k| closest.bodies.iter().map(|b| b.position[k]).sum::<f64>() / 4.);
    let mut frames = Vec::new();
    let window_ns = 550_000_000;
    let range = center.saturating_sub(window_ns)..=center.saturating_add(window_ns);
    let first_ns = *coarse
        .frames
        .range(range.clone())
        .next()
        .ok_or("empty encounter window")?
        .0;
    let mut previous_owners = [None; 4];
    let mut owner_models = [0u64; 4];
    let mut next_owner_model = 20;
    for (&time, upper) in coarse.frames.range(range) {
        let lower = comparison
            .frames
            .get(&time)
            .ok_or("missing corresponding comparison sample")?;
        let sequence = frames.len() + 1;
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (lane, (sample, label)) in [(upper, &coarse_label), (lower, &comparison_label)]
            .into_iter()
            .enumerate()
        {
            let offset = if lane == 0 { 0.85 } else { -0.85 };
            for b in &sample.bodies {
                if b.size != closest.bodies[(b.id - 1) as usize].size {
                    return Err("box dimensions differ between comparison samples".into());
                }
                let body_index = (b.id - 1) as usize;
                let owner =
                    (lane == 1 && args.worker_owners).then(|| sample.owners.unwrap()[body_index]);
                // Archive identities have immutable geometry and material. Start a new
                // display identity when the same physical body changes owner/color.
                let model = if let Some(worker) = owner {
                    if previous_owners[body_index] != Some(worker) {
                        owner_models[body_index] = next_owner_model;
                        next_owner_model += 2;
                        previous_owners[body_index] = Some(worker);
                    }
                    owner_models[body_index]
                } else {
                    2 + 2 * (lane as u64 * 4 + b.id)
                };
                let visual = model + 1;
                let name = format!(
                    "{label}_train_{}_box_{}",
                    if b.id <= 2 { "left" } else { "right" },
                    b.id
                );
                let [x, y, z]: [f64; 3] = std::array::from_fn(|k| b.position[k] - origin[k]);
                let [qx, qy, qz, qw] = b.orientation;
                let s = FRAC_1_SQRT_2;
                let color = if let Some(owner) = owner {
                    // One stable color per native process slot; a handoff changes the whole component.
                    [
                        [0.12, 0.55, 0.98, 1.],
                        [1., 0.45, 0.08, 1.],
                        [0.20, 0.78, 0.47, 1.],
                        [0.70, 0.36, 0.93, 1.],
                    ][owner]
                } else if b.id <= 2 {
                    [0.045, 0.67, 0.86, 1.]
                } else {
                    [1., 0.36, 0.075, 1.]
                };
                visuals.push(json!({"id":visual,"name":format!("default::{name}::box"),
                    "translation":[y+offset,-x,z+offset], "rotation":[s*(qx+qy),s*(qy-qx),s*(qz-qw),s*(qw+qz)],
                    "shape":{"kind":"box","size":b.size},"color":color}));
                models.push(json!({"model_id":model,"name":name,"visual_ids":[visual],
                    "can_remove":false,"can_set_state":false}));
            }
        }
        let elapsed = time - first_ns;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,
            "seconds":elapsed/1_000_000_000,"nanoseconds":elapsed%1_000_000_000,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,
            "models":models,"templates":[]});
        frames.push((scene.to_string(), catalog.to_string()));
    }
    let count = frames.len();
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    println!(
        "{count} synchronized samples from {:.3} s, centered on {:.3} s; upper {coarse_label}, lower {comparison_label}.",
        first_ns as f64 / 1e9,
        center as f64 / 1e9,
    );
    Ok(())
}
