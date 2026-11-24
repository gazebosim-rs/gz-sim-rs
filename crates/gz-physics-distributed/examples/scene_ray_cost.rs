// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-frame scalar lidar casts through a mixed four-worker world, checked against native Rapier.
use clap::Parser;
use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{
    error::Error,
    hint::black_box,
    path::PathBuf,
    time::{Duration, Instant},
};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 256)]
    bodies: usize,
    #[arg(long, default_value_t = 9)]
    trials: usize,
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    if !(4..=256).contains(&a.bodies) || a.trials == 0 {
        return Err("expected 4–256 bodies and at least one trial".into());
    }
    let mut world =
        SceneBackend::new(&a.worker, [0.; 3], Duration::from_millis(5))?.with_worker_count(4)?;
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    let mut targets = Vec::new();
    for i in 0..a.bodies {
        let position = [
            (i % 16) as f64 * 3.7,
            (i / 16) as f64 * 4.1,
            (i % 3) as f64 * 0.14,
        ];
        targets.push(position);
        let shape = match i % 4 {
            0 => Shape::Box {
                half_extents: [0.42, 0.31, 0.26],
            },
            1 => Shape::Sphere { radius: 0.36 },
            2 => Shape::Capsule {
                half_length: 0.43,
                radius: 0.21,
            },
            _ => Shape::Cylinder {
                half_height: 0.39,
                radius: 0.24,
            },
        };
        let spec = BodySpec {
            shape,
            position,
            mass: Some(1.),
            velocity: [0.; 3],
        };
        let angle = 0.009 * (i + 1) as f64;
        let rot = RotationSpec {
            orientation: [0., angle.sin(), 0., angle.cos()],
            angular_velocity: [0.; 3],
        };
        world.spawn_primitive_link(spec, rot, None, Tier::Dynamic, 0.)?;
        native.spawn_partition_primitive(spec, rot, false, 0.)?;
    }
    world.initialize()?;
    let mut rays = Vec::new();
    for (kind, _) in ["target", "miss", "longitudinal", "oblique"]
        .iter()
        .enumerate()
    {
        for k in 0..1024usize {
            let [x, y, z] = targets[k % a.bodies];
            let origin = match kind {
                0 => [x + 0.10 * (k as f64).sin(), y, z + 2.],
                1 => [x + 1.04, y + 1.24, z + 3.],
                2 => [x - 3.15, y, z],
                _ => [x + 2.5, y - 1.3, z + 2.2],
            };
            let direction: [f64; 3] = match kind {
                0 | 1 => [0., 0., -1.],
                2 => [1., 0., 0.],
                _ => [-2.5, 1.3, -2.2],
            };
            let norm = direction[0].hypot(direction[1]).hypot(direction[2]);
            let direction = direction.map(|d| d / norm);
            let limit = if kind == 2 { 70. } else { 8. };
            let reference = native.cast_ray(origin, direction, limit, None)?;
            let actual = world.cast_ray(origin, direction, limit, None)?;
            if reference.map(f64::to_bits) != actual.map(f64::to_bits) {
                return Err(format!(
                    "ray {kind}/{k} differs from native: {actual:?} vs {reference:?}"
                )
                .into());
            }
            rays.push((origin, direction, limit, actual));
        }
    }
    let hits = rays.iter().filter(|r| r.3.is_some()).count();
    if hits != 3072 {
        return Err(format!("unexpected reference hit count {hits}").into());
    }
    println!("bodies,trial,rays,hits,elapsed_us,checksum");
    for trial in 0..a.trials {
        let start = Instant::now();
        let mut bits = 0u64;
        for (o, d, l, _) in &rays {
            let r = world.cast_ray(*o, *d, *l, None)?;
            bits = bits.rotate_left(1) ^ black_box(r.map_or(u64::MAX, f64::to_bits));
        }
        println!(
            "{},{trial},4096,{hits},{:.3},{bits:016x}",
            a.bodies,
            start.elapsed().as_secs_f64() * 1e6
        );
    }
    Ok(())
}
