// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, Primitive, SimulationStep, Tier},
    pool::WorkerPool,
};
use std::{error::Error, path::Path};

#[test]
fn glancing_spheres_track_the_frictionless_impact_solution() -> Result<(), Box<dyn Error>> {
    let a = Body {
        id: 1,
        tier: Tier::Dynamic,
        size: [0.2; 3],
        mass: 1.,
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
        friction: 0.,
        primitive: Primitive::Sphere,
    };
    let b = Body {
        id: 2,
        position: [-0.48, 0.12, 0.],
        linear: [80., 0., 0.],
        ..a.clone()
    };
    let seed = vec![vec![a], vec![b], vec![], vec![]];
    // First touch is at t=4 ms, with center separation (-0.16, 0.12, 0).
    // The equal-mass, zero-restitution normal impulse has magnitude
    // -(v_b-v_a)·n/(1/m_a+1/m_b) = 32 N·s, n=(-0.8,0.6,0).
    let exact_impulse = 32.;
    let exact_velocity_a = [25.6, -19.2, 0.];
    let exact_position_a = exact_velocity_a.map(|v| v * 0.006);
    let distance = |a: [f64; 3], b: [f64; 3]| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]);
    let mut results = Vec::new();
    for (index, (mode, step, ticks)) in [
        ("coarse", SimulationStep::Ms10, 1),
        ("adaptive", SimulationStep::Ms10, 1),
        ("fine", SimulationStep::Ms1, 10),
    ]
    .into_iter()
    .enumerate()
    {
        let mut pool = WorkerPool::spawn(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            0xcc40 + index as u64,
            seed.clone(),
            step,
            None,
            64,
        )?;
        let mut impulse = 0.;
        for _ in 0..ticks {
            let frame = if mode == "adaptive" {
                pool.advance_adaptive_in_field([0.; 3], &[])?
            } else {
                pool.advance()?
            };
            for pair in frame.workers.iter().flat_map(|w| &w.snapshot.impulses) {
                assert_eq!((pair.a, pair.b), (1, 2));
                impulse += pair.normal_impulse;
            }
        }
        if mode == "adaptive" {
            assert_eq!(pool.last_substeps(), Some(10));
        }
        let first = pool.frame().body(1).unwrap().1.clone();
        let second = pool.frame().body(2).unwrap().1.clone();
        println!(
            "{}",
            serde_json::json!({"mode":mode,"normal_impulse_ns":impulse,
            "position_a_m":first.position,"velocity_a_m_s":first.linear,
            "position_b_m":second.position,"velocity_b_m_s":second.linear,
            "exact_impulse_ns":exact_impulse,"exact_position_a_m":exact_position_a,
            "exact_velocity_a_m_s":exact_velocity_a,
            "position_a_error_m":distance(first.position, exact_position_a)})
        );
        for axis in 0..3 {
            assert!((first.linear[axis] + second.linear[axis] - [80., 0., 0.][axis]).abs() < 1e-10);
        }
        results.push((first, second, impulse));
    }
    let (first, _, impulse) = &results[1];
    assert!((impulse - exact_impulse).abs() < 1e-10);
    assert!(distance(first.linear, exact_velocity_a) < 1e-10);
    // Native contact stabilization can shift the position even when momentum is exact.
    let position_error = distance(first.position, exact_position_a);
    assert!(position_error < 0.005);
    assert!(position_error <= distance(results[0].0.position, exact_position_a) + 1e-12);
    assert_eq!(results[1], results[2]);
    Ok(())
}
