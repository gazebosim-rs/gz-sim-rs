// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure equivalent 10 ms windows of a four-worker pusher/cargo or sphere/tabletop SDF.
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured};
use sdformat::NoResources;
use serde::Serialize;
use std::{
    error::Error,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Measure a sphere landing on a cylinder and then on the shared floor.
    #[arg(long)]
    tabletop: bool,
}

#[derive(Clone, Copy, Serialize)]
enum Profile {
    Coarse,
    Adaptive,
    Fine,
}

#[derive(Serialize)]
struct Run {
    round: usize,
    profile: Profile,
    windows: usize,
    refined_windows: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    cargo_x_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    sphere_x_m: Option<f64>,
    total_ms: f64,
    window_p50_us: f64,
    window_p95_us: f64,
    window_p99_us: f64,
}

fn run(
    worker: &Path,
    round: usize,
    profile: Profile,
    tabletop: bool,
) -> Result<Run, Box<dyn Error>> {
    let (engine, step, ticks) = match profile {
        Profile::Coarse => (PhysicsEngine::DistributedPool(4), 10, 1),
        Profile::Adaptive => (PhysicsEngine::DistributedAdaptive(4), 10, 1),
        Profile::Fine => (PhysicsEngine::DistributedPool(4), 1, 10),
    };
    let (world, file, windows) = if tabletop {
        (
            include_str!("../../../examples/distributed_cylinder_tabletop.sdf"),
            "tabletop.sdf",
            220,
        )
    } else {
        (
            include_str!("../../../examples/distributed_boxes.sdf"),
            "boxes.sdf",
            132,
        )
    };
    let mut sim = load_world_configured(
        world,
        file,
        "default",
        &NoResources,
        WorldLoadOptions {
            engine,
            step: Some(Duration::from_millis(step)),
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let mut samples = Vec::with_capacity(windows);
    let mut refined = 0;
    for _ in 0..windows {
        let start = Instant::now();
        sim.step_exact(ticks)?;
        samples.push(start.elapsed().as_secs_f64());
        let backend = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap();
        refined += usize::from(backend.last_substeps() == Some(10));
    }
    assert_eq!(sim.elapsed(), Duration::from_millis(windows as u64 * 10));
    let scene = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    let mover = scene.pool_frame().unwrap().body(1).unwrap().1;
    if tabletop {
        assert!((mover.position[2] - 0.12).abs() < 0.001 && mover.position[0] > 1.);
    } else {
        assert!((mover.linear[0] - 1.5).abs() < 1e-6);
    }
    if matches!(profile, Profile::Adaptive) {
        assert!(refined > 0 && refined < samples.len());
    }
    let total_ms = samples.iter().sum::<f64>() * 1e3;
    samples.sort_unstable_by(f64::total_cmp);
    let percentile = |p: usize| samples[(samples.len() * p).div_ceil(100) - 1] * 1e6;
    Ok(Run {
        round,
        profile,
        windows: samples.len(),
        refined_windows: refined,
        cargo_x_m: (!tabletop).then_some(mover.position[0]),
        sphere_x_m: tabletop.then_some(mover.position[0]),
        total_ms,
        window_p50_us: percentile(50),
        window_p95_us: percentile(95),
        window_p99_us: percentile(99),
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    use Profile::*;
    let args = Args::parse();
    for (round, order) in [
        [Coarse, Adaptive, Fine],
        [Adaptive, Fine, Coarse],
        [Fine, Coarse, Adaptive],
        [Fine, Adaptive, Coarse],
        [Coarse, Fine, Adaptive],
        [Adaptive, Coarse, Fine],
    ]
    .iter()
    .enumerate()
    {
        for &profile in order {
            println!(
                "{}",
                serde_json::to_string(&run(&args.worker, round, profile, args.tabletop)?)?
            );
        }
    }
    Ok(())
}
