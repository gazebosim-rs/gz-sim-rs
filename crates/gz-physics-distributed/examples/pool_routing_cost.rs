// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time the global prospective field admission at a committed, physically loaded pool barrier.
use clap::Parser;
use gz_physics_distributed::{
    physics::{BodyWrench, GroundPlane, SimulationStep, fixture},
    pool::WorkerPool,
};
use std::{error::Error, hint::black_box, path::PathBuf, time::Instant};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 4)]
    width: usize,
    #[arg(long, default_value_t = 256)]
    bodies: usize,
    #[arg(long, default_value_t = 500)]
    samples: usize,
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    if !(3..=8).contains(&a.width)
        || !(2..=256).contains(&a.bodies)
        || !a.bodies.is_multiple_of(2)
        || a.samples == 0
    {
        return Err("use 3–8 workers, 2–256 even bodies and positive samples".into());
    }
    let mut shards = vec![vec![]; a.width];
    for i in 0..a.bodies {
        let mut b = fixture()[1].clone();
        b.id = i as u64;
        let cell = i / 2;
        // Keep independently supported stacks outside the possible redirection
        // radius, including under the maximum admissible contact speed.
        b.position = [
            (cell % 16) as f64 * 8.,
            (cell / 16) as f64 * 8.,
            0.7 + (i % 2) as f64 * 0.9,
        ];
        b.linear = [0.; 3];
        shards[(i / 2) * a.width / (a.bodies / 2)].push(b);
    }
    let gravity = [0., 0., -9.81];
    let mut pool = WorkerPool::spawn(
        &a.worker,
        17950,
        shards,
        SimulationStep::Ms5,
        Some(GroundPlane::new(0.)?),
        1024,
    )?;
    for _ in 0..300 {
        pool.advance_in_gravity(gravity)?;
    }
    let contacts = pool
        .frame()
        .workers
        .iter()
        .map(|w| w.snapshot.impulses.len() + w.snapshot.supports.len())
        .sum::<usize>();
    assert_eq!(contacts, a.bodies);
    let inputs = [BodyWrench {
        id: 0,
        force: [0.05, 0., 0.],
        torque: [0., 0., 0.001],
    }];
    for field in [&[][..], &inputs[..]] {
        let mut timings = Vec::with_capacity(a.samples);
        for _ in 0..a.samples {
            let start = Instant::now();
            pool.validate_field(black_box(gravity), black_box(field))?;
            timings.push(start.elapsed().as_nanos() as u64);
        }
        timings.sort_unstable();
        let q = |p: usize| timings[(p * timings.len()).div_ceil(100) - 1] as f64 * 0.001;
        println!(
            "{},{},{},{},{},{:.3},{:.3},{:.3}",
            a.bodies,
            a.width,
            contacts,
            field.len(),
            a.samples,
            q(50),
            q(95),
            q(99)
        );
    }
    Ok(())
}
