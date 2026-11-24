// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Torque-limited bilateral reach with a floating root in zero gravity. The arms
//! reach forward and inward, while the legs flex and the trunk pitches.
//! Run `cargo run --release --locked -p gz-sim-scene --example humanoid_reach`.
//! Append `-- --sdf` to emit the neutral model and scene for other clients.
#[path = "support/humanoid.rs"]
pub mod humanoid;

use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use humanoid::Humanoid;
use sdformat::NoResources;
use std::{error::Error, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let model = Humanoid::new();
    let source = model.world_sdf("");
    match std::env::args().skip(1).collect::<Vec<_>>().as_slice() {
        [flag] if flag == "--sdf" => {
            println!("{source}");
            return Ok(());
        }
        [] => {}
        _ => return Err("usage: humanoid_reach [--sdf]".into()),
    }
    let mut sim = load_world_with_sdf_step(
        &source,
        "humanoid.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let control = model.bind(&sim);
    let initial_center = control.center_of_mass(&sim);
    let initial = control.hands_in_pelvis(&sim);
    let mut max_error = 0_f64;
    let mut max_drift = 0_f64;
    let start = Instant::now();
    for _ in 0..2000 {
        max_error = max_error.max(control.drive(&mut sim)?);
        sim.step_exact(1)?;
        let drift = control.center_of_mass(&sim).distance(initial_center);
        assert!(drift.is_finite());
        max_drift = max_drift.max(drift);
    }
    let elapsed = start.elapsed().as_secs_f64();
    let (residual, speed) = control.hold_residual(&sim)?;
    let hands = control.hands_in_pelvis(&sim);
    let forward = [hands[0].x - initial[0].x, hands[1].x - initial[1].x];
    let narrowing = initial[0].y - initial[1].y - (hands[0].y - hands[1].y);
    println!("31 links, 30 actuated revolute coordinates, floating base, 1 ms step");
    println!(
        "peak tracking error {max_error:.6} rad; hold residual {residual:.6} rad at {speed:.6} rad/s"
    );
    println!(
        "left/right forward reach {forward:.4?} m; hand separation reduced {narrowing:.4} m; peak COM drift {max_drift:.6} m"
    );
    println!(
        "2 s simulated in {elapsed:.3} s (real-time factor {:.3})",
        2. / elapsed
    );
    assert!(max_error < 0.04 && residual < 0.015 && speed < 0.04);
    assert!(forward.iter().all(|&x| x > 0.3) && narrowing > 0.10);
    assert!(max_drift < 0.005);
    Ok(())
}
