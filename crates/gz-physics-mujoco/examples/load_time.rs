// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Load-time measurement matching tests/fixtures/reference/mujoco_reference.cc
//! `--load`: N free spheres one meter apart, then one 1 ms step. The hinged case
//! also adds a world revolute per sphere and queries its state right away, as
//! the scene does after constructing each joint.
//!
//! cargo run --release -p gz-physics-mujoco --example load_time -- 100 200 400
use gz_physics::DynamicBackend;
use gz_physics_mujoco::{BodySpec, Inertia, MujocoBackend, RotationSpec, Shape};
use std::time::{Duration, Instant};

fn load(count: usize, hinged: bool) -> Result<Duration, Box<dyn std::error::Error>> {
    let start = Instant::now();
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81])?;
    for i in 0..count {
        let body = mujoco.spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [i as f64, 0.0, 1.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
            }),
        )?;
        if hinged {
            let joint = mujoco.add_world_revolute(body, [i as f64, 0.0, 2.0], [0.0, 1.0, 0.0])?;
            mujoco.joint_state(joint).ok_or("joint state")?;
        }
    }
    mujoco.step(Duration::from_millis(1))?;
    Ok(start.elapsed())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    for count in std::env::args().skip(1) {
        let count: usize = count.parse()?;
        for hinged in [false, true] {
            let mut best = Duration::MAX;
            for _ in 0..5 {
                best = best.min(load(count, hinged)?);
            }
            let kind = if hinged { "hinged" } else { "free" };
            println!("{count} {kind} bodies: {:.3} ms", best.as_secs_f64() * 1e3);
        }
    }
    Ok(())
}
