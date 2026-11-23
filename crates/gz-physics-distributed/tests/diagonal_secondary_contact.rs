// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, Island, Primitive, SimulationStep, Snapshot, Tier},
    pool::WorkerPool,
};
use std::{error::Error, path::Path};

#[test]
fn oblique_impact_reaches_a_stationary_third_worker_within_the_same_tick()
-> Result<(), Box<dyn Error>> {
    let sphere = |id, x, y| Body {
        id,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        mass: 1.,
        tier: Tier::Dynamic,
        friction: 0.,
        position: [0.6 * x, y, 0.8 * x],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    };
    // In the impact plane the incoming ball follows y=-0.12. Its swept
    // sphere misses ball 3 at y=0.54. Ball 2 acquires its entire velocity
    // during the first collision and hits ball 3 before the 10 ms barrier.
    let bodies = [
        sphere(1, -0.46, -0.12),
        sphere(2, 0., 0.),
        sphere(3, 0.72, 0.54),
        sphere(4, 20., 0.),
    ];
    let mut reference = Island::restore(&Snapshot {
        bodies: bodies.to_vec(),
        ..Snapshot::default()
    })?;
    let mut pool = WorkerPool::spawn(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        0xd1a6,
        bodies.map(|b| vec![b]).into(),
        SimulationStep::Ms10,
        None,
        128,
    )?;
    let impulse = [180., 0., 240.];
    reference.queue_impulse(1, impulse)?;
    reference.step_refined_in_field([0.; 3], &[], 10)?;
    let expected = reference.snapshot();
    for edge in [(1, 2), (2, 3)] {
        assert!(
            expected
                .impulses
                .iter()
                .any(|i| { (i.a, i.b) == edge && i.normal_impulse > 0. }),
            "missing native contact {edge:?}: {:?}",
            expected.impulses
        );
    }
    pool.queue_impulse(1, impulse)?;
    assert_eq!(pool.refined_island_bodies([0.; 3], &[])?, [1, 2, 3]);
    let frame = pool.advance_islands_in_field([0.; 3], &[])?;
    for body in &expected.bodies {
        assert_eq!(frame.body(body.id).unwrap().1, body);
    }
    let owner = usize::from(frame.body(2).unwrap().0);
    assert_eq!(frame.workers[owner].snapshot.impulses, expected.impulses);
    Ok(())
}

#[test]
fn opposing_field_and_wrench_preserve_the_native_residual_acceleration()
-> Result<(), Box<dyn Error>> {
    use gz_physics_distributed::physics::BodyWrench;

    let moving = Body {
        id: 1,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        mass: 1.,
        tier: Tier::Dynamic,
        friction: 0.,
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    };
    let wall = Body {
        id: 2,
        tier: Tier::Fixed,
        mass: 0.,
        position: [0.235, 0., 0.],
        ..moving.clone()
    };
    // The adjacent x-component inputs round to the same magnitude if each
    // is multiplied by dt before summing. Their native residual is 1024 N.
    let gravity = [7.609281930405191e18, 0., 0.];
    let loads = [BodyWrench {
        id: 1,
        force: [-7.60928193040519e18, 0., 0.],
        torque: [0.; 3],
    }];
    let mut reference = Island::restore(&Snapshot {
        bodies: vec![moving.clone(), wall.clone()],
        ..Snapshot::default()
    })?;
    reference.step_refined_in_field(gravity, &loads, 10)?;
    let expected = reference.snapshot();
    assert!(expected.impulses.iter().any(|p| p.normal_impulse > 0.));
    let mut pool = WorkerPool::spawn(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        0xcace,
        vec![vec![moving], vec![wall], vec![]],
        SimulationStep::Ms10,
        None,
        128,
    )?;
    assert_eq!(pool.refined_island_bodies(gravity, &loads)?, [1, 2]);
    let frame = pool.advance_islands_in_field(gravity, &loads)?;
    for body in &expected.bodies {
        assert_eq!(frame.body(body.id).unwrap().1, body);
    }
    let owner = usize::from(frame.body(1).unwrap().0);
    assert_eq!(frame.workers[owner].snapshot.impulses, expected.impulses);
    Ok(())
}
