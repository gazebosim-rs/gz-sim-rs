// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Counterbalanced full Bevy step timing for the qualified authored friction scene.
//! Usage: rapier_point_friction_timing OUTPUT_DIR REFERENCE_FRAMES ROUNDS
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/rapier_per_point_friction.sdf");
const STEPS: usize = 1200;
const NAMES: [&str; 2] = ["grouped", "pointwise"];

fn load(point: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        SDF,
        "rapier_per_point_friction.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Rapier,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: point,
            ..Default::default()
        },
    )?)
}

#[derive(Clone, Copy)]
struct Timing {
    full: u64,
    tail: u64,
}
fn percentile(values: &[u64], pct: usize) -> u64 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted[(sorted.len() * pct).div_ceil(100) - 1]
}
fn one(point: bool, golden: &str) -> Result<Timing> {
    let mut sim = load(point)?;
    let links = ["default::table::deck", "default::block::body"].map(|name| {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap();
        (
            e.get::<EntityId>().unwrap().get(),
            e.get::<BodyHandle>().unwrap().id(),
        )
    });
    // Keep timer setup, SDF loading, initial bindings, golden readback and allocation outside the loop.
    let mut durations = Vec::with_capacity(STEPS);
    let start = Instant::now();
    for step in 1..=STEPS {
        let tick = Instant::now();
        if step == 301 {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .apply_impulse(links[1].1, [4.8, 0., 0.], [0., 0., 0.96])?;
        }
        sim.step_exact(1)?;
        durations.push(tick.elapsed().as_nanos() as u64);
    }
    let full = start.elapsed().as_nanos() as u64;
    assert_eq!(sim.committed_update_info()?.sim_time.as_millis(), 1200);
    let backend = sim.world().resource::<PhysicsBackend>();
    let bodies: Vec<_> = links.iter().zip([[8.,3.,0.4],[0.8,0.4,0.2]]).map(|((public,id),size)| {
        let s=backend.state(*id).unwrap();
        json!({"id":public,"size":size,"position":s.position,"orientation":s.orientation,"velocity":s.velocity,"angular_velocity":s.angular_velocity})
    }).collect();
    // Check the serialized floats directly, preserving their original decimal representation.
    assert!(
        golden.starts_with(&format!(
            "{{\"bodies\":{},",
            serde_json::to_string(&bodies)?
        )),
        "terminal body state differs from qualified playback"
    );
    Ok(Timing {
        full,
        tail: percentile(&durations, 95),
    })
}
fn golden(dir: &Path, profile: &str) -> Result<String> {
    Ok(fs::read_to_string(dir.join(format!("{profile}.jsonl")))?
        .lines()
        .last()
        .ok_or("empty reference")?
        .to_owned())
}
fn stats(values: &[u64]) -> Value {
    json!({"p50_ns":percentile(values,50),"p95_ns":percentile(values,95)})
}
fn main() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 3 {
        return Err(
            "usage: rapier_point_friction_timing OUTPUT_DIR REFERENCE_DIR EVEN_ROUNDS".into(),
        );
    }
    let dir = PathBuf::from(&args[0]);
    let rounds = args[2].parse::<usize>()?;
    if !(2..=4096).contains(&rounds) || !rounds.is_multiple_of(2) {
        return Err("choose an even round count in 2..4096".into());
    }
    fs::create_dir_all(&dir)?;
    let refs = [
        golden(Path::new(&args[1]), NAMES[0])?,
        golden(Path::new(&args[1]), NAMES[1])?,
    ];
    let mut rows = BufWriter::new(fs::File::create(dir.join("pairs.jsonl"))?);
    let mut values: [Vec<Timing>; 2] = [Vec::new(), Vec::new()];
    for round in 0..(rounds + 4) {
        let order = if round.is_multiple_of(2) {
            [0, 1]
        } else {
            [1, 0]
        };
        let mut result = [Timing { full: 0, tail: 0 }; 2];
        for arm in order {
            result[arm] = one(arm == 1, &refs[arm])?;
        }
        if round < 4 {
            continue;
        }
        for arm in 0..2 {
            values[arm].push(result[arm]);
        }
        let row = json!({"round":round-4,"first":NAMES[order[0]],"grouped":{"run_ns":result[0].full,"tick_p95_ns":result[0].tail},"pointwise":{"run_ns":result[1].full,"tick_p95_ns":result[1].tail}});
        serde_json::to_writer(&mut rows, &row)?;
        writeln!(rows)?;
    }
    rows.flush()?;
    let report = |arm: usize| json!({"run":stats(&values[arm].iter().map(|v|v.full).collect::<Vec<_>>()),"per_run_tick_p95":stats(&values[arm].iter().map(|v|v.tail).collect::<Vec<_>>())});
    let mut ratio: Vec<_> = values[0]
        .iter()
        .zip(&values[1])
        .map(|(a, b)| b.full as f64 / a.full as f64)
        .collect();
    ratio.sort_by(f64::total_cmp);
    let summary = json!({"rounds":rounds,"warmup_pairs":4,"ticks_per_run":STEPS,"matched_terminal_worlds":2*(rounds+4),"grouped":report(0),"pointwise":report(1),"paired_run_ratio_p50":ratio[(rounds*50).div_ceil(100)-1],"paired_run_ratio_p95":ratio[(rounds*95).div_ceil(100)-1], "pointwise_faster_pairs":ratio.iter().filter(|x|**x<1.).count()});
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
