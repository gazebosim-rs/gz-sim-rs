// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public pool edit admission, exact native execution and process recovery.
use gz_physics::{BodySpec, BodyState, DynamicBackend, RotationSpec};
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, BodyWrench, GroundPlane, Primitive, SimulationStep, Tier, fixture},
    pool::WorkerPool,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::path::Path;

fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn body(id: u64, x: f64) -> Body {
    Body {
        id,
        tier: Tier::Dynamic,
        primitive: Primitive::Sphere,
        size: [0.5; 3],
        mass: 1.,
        position: [x, 0., 2.],
        linear: [0.; 3],
        angular: [0.; 3],
        ..fixture()[1].clone()
    }
}
fn state(b: &Body) -> BodyState {
    BodyState {
        position: b.position,
        orientation: b.orientation,
        velocity: b.linear,
        angular_velocity: b.angular,
    }
}
fn requests(p: &WorkerPool) -> Vec<u64> {
    p.traffic().iter().map(|w| w.requests).collect()
}
fn assert_bits(actual: BodyState, expected: BodyState) {
    assert_eq!(
        actual.position.map(f64::to_bits),
        expected.position.map(f64::to_bits)
    );
    assert_eq!(
        actual.orientation.map(f64::to_bits),
        expected.orientation.map(f64::to_bits)
    );
    assert_eq!(
        actual.velocity.map(f64::to_bits),
        expected.velocity.map(f64::to_bits)
    );
    assert_eq!(
        actual.angular_velocity.map(f64::to_bits),
        expected.angular_velocity.map(f64::to_bits)
    );
}

#[test]
fn foreign_geometry_shared_ground_pending_impulses_and_held_forces_are_checked_before_edit_rpc() {
    let a = body(1, 0.);
    let b = body(2, 0.85);
    let fixed = Body {
        tier: Tier::Fixed,
        mass: 0.,
        ..body(3, 12.)
    };
    let mut pool = WorkerPool::spawn(
        worker(),
        18021,
        vec![vec![b.clone()], vec![fixed], vec![a.clone()]],
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.).unwrap()),
        2,
    )
    .unwrap();
    let current = state(&a);
    let frame = pool.frame().clone();
    let traffic = requests(&pool);
    for target in [
        BodyState {
            position: b.position,
            ..current
        },
        BodyState {
            position: [0., 0., 0.1],
            ..current
        },
    ] {
        assert!(matches!(
            pool.validate_state_in_field(1, target, [0.; 3], &[]),
            Err(Error::EditDeferred(_))
        ));
        assert!(matches!(
            pool.replace_state_in_field(1, target, [0.; 3], &[]),
            Err(Error::EditDeferred(_))
        ));
    }
    assert!(
        pool.replace_state_in_field(
            1,
            BodyState {
                orientation: [0.; 4],
                ..current
            },
            [0.; 3],
            &[]
        )
        .is_err()
    );
    assert!(
        pool.set_velocity_in_field(1, [f64::NAN; 3], [0.; 3], [0.; 3], &[])
            .is_err()
    );
    assert!(
        pool.set_velocity_in_field(22, [0.; 3], [0.; 3], [0.; 3], &[])
            .is_err()
    );
    assert!(
        pool.replace_state_in_field(3, current, [0.; 3], &[])
            .is_err()
    );
    assert_eq!(requests(&pool), traffic);
    assert_eq!(pool.frame(), &frame);

    // Three separate effects can cover 0.39 m together. The 0.35 m gap
    // admits each pair but the complete next-step sweep requires migration.
    let loads = [BodyWrench {
        id: 1,
        force: [1300., 0., 0.],
        torque: [0.; 3],
    }];
    pool.queue_impulse_in_field(1, [13., 0., 0.], [0.; 3], &loads)
        .unwrap();
    let proposed = BodyState {
        velocity: [13., 0., 0.],
        ..current
    };
    assert!(
        pool.validate_state_in_field(1, proposed, [0.; 3], &[])
            .is_ok()
    );
    let traffic = requests(&pool);
    assert!(matches!(
        pool.set_velocity_in_field(1, proposed.velocity, [0.; 3], [0.; 3], &loads),
        Err(Error::Capacity)
    ));
    assert_eq!(requests(&pool), traffic);
    assert_eq!(pool.frame(), &frame);
    pool.advance_in_field([0.; 3], &loads).unwrap();
    assert!((pool.frame().body(1).unwrap().1.linear[0] - 26.).abs() < 1e-12);
}

#[test]
fn journal_replay_and_native_image_restore_pose_and_twist_with_pending_input_exactly_once() {
    let a = Body {
        primitive: Primitive::Box,
        size: [0.35, 0.6, 0.9],
        ..body(1, 0.)
    };
    let step = SimulationStep::Ms5;
    let gravity = [0., 0., -1.8];
    let mut native = MultiFidelityBackend::new(gravity).unwrap();
    native.set_partition_step(step.duration()).unwrap();
    let local = native
        .spawn_partition_primitive(
            BodySpec {
                shape: a.shape().unwrap(),
                position: a.position,
                mass: Some(a.mass),
                velocity: a.linear,
            },
            RotationSpec {
                orientation: a.orientation,
                angular_velocity: a.angular,
            },
            false,
            a.friction,
        )
        .unwrap();
    let mut pool = WorkerPool::spawn(
        worker(),
        18022,
        vec![vec![], vec![a.clone()], vec![body(2, 15.)], vec![]],
        step,
        None,
        8,
    )
    .unwrap();
    let seed = pool.frame().clone();
    // Rapier keeps existing axis bits when a pose or a complete velocity vector
    // compares equal. The acknowledgement still authenticates every unrelated bit.
    let signed_zero = BodyState {
        position: [-0., 0., 2.],
        orientation: [-0., 0., 0., 1.],
        velocity: [-0., 0., 0.],
        angular_velocity: [0., -0., 0.],
    };
    pool.replace_state_in_field(1, signed_zero, gravity, &[])
        .unwrap();
    native.set_body_state(local, signed_zero).unwrap();
    pool.set_velocity_in_field(
        1,
        signed_zero.velocity,
        signed_zero.angular_velocity,
        gravity,
        &[],
    )
    .unwrap();
    native
        .set_world_velocity(local, signed_zero.velocity, signed_zero.angular_velocity)
        .unwrap();
    assert_bits(
        state(pool.frame().body(1).unwrap().1),
        native.state(local).unwrap(),
    );
    let input = [0.4, -0.15, 0.2];
    pool.queue_impulse(1, input).unwrap();
    let angle = 0.235_f64;
    let edited = BodyState {
        position: [1.75, -0.45, 2.35],
        orientation: [0., angle.sin(), 0., angle.cos()],
        velocity: [-0.24, 0.35, 0.],
        angular_velocity: [0.25, -0.21, 0.16],
    };
    pool.drop_next_acknowledgement(1).unwrap();
    assert!(
        pool.replace_state_in_field(1, edited, gravity, &[])
            .is_err()
    );
    assert_eq!(pool.frame(), &seed);
    pool.replace_state_in_field(1, edited, gravity, &[])
        .unwrap();
    native.set_body_state(local, edited).unwrap();
    assert_bits(
        state(pool.frame().body(1).unwrap().1),
        native.state(local).unwrap(),
    );
    assert_eq!(
        pool.frame().workers[1].generations,
        seed.workers[1].generations
    );

    let (linear, angular) = ([0.15, 0.45, -0.2], [-0.3, 0.37, 0.29]);
    pool.drop_next_acknowledgement(1).unwrap();
    let before = pool.frame().clone();
    assert!(
        pool.set_velocity_in_field(1, linear, angular, gravity, &[])
            .is_err()
    );
    assert_eq!(pool.frame(), &before);
    pool.set_velocity_in_field(1, linear, angular, gravity, &[])
        .unwrap();
    native.set_world_velocity(local, linear, angular).unwrap();
    let before_journal = pool.worker_pid(1).unwrap();
    pool.recover(1).unwrap(); // Original seed plus hashed edit journal.
    let after_journal = pool.worker_pid(1).unwrap();
    assert_ne!(before_journal, after_journal);
    assert_bits(
        state(pool.frame().body(1).unwrap().1),
        native.state(local).unwrap(),
    );
    pool.compact().unwrap();
    pool.recover(1).unwrap(); // Native image already includes the replacement and pending input.
    assert_ne!(pool.worker_pid(1).unwrap(), after_journal);
    native.apply_impulse(local, input, [0.; 3]).unwrap();
    for _ in 0..75 {
        let frame = pool.advance_in_gravity(gravity).unwrap();
        native.step(step.duration()).unwrap();
        assert_bits(
            state(frame.body(1).unwrap().1),
            native.state(local).unwrap(),
        );
    }
}

#[test]
fn a_warm_component_defers_edits_while_an_independent_prescribed_member_remains_editable() {
    let mut a = body(1, 0.);
    a.linear[0] = 1.;
    let mut b = body(2, 0.55);
    b.linear[0] = -1.;
    let prescribed = Body {
        tier: Tier::Prescribed,
        ..body(3, 20.)
    };
    let mut p = WorkerPool::spawn(
        worker(),
        18023,
        vec![vec![a], vec![b], vec![prescribed.clone()]],
        SimulationStep::Ms5,
        None,
        24,
    )
    .unwrap();
    let mut contact = false;
    for _ in 0..10 {
        if p.advance()
            .unwrap()
            .workers
            .iter()
            .any(|w| !w.snapshot.impulses.is_empty())
        {
            contact = true;
            break;
        }
    }
    assert!(contact);
    let frame = p.frame().clone();
    let previous = requests(&p);
    let changed = BodyState {
        position: [12., 0., 2.],
        ..state(frame.body(1).unwrap().1)
    };
    assert!(matches!(
        p.replace_state_in_field(1, changed, [0.; 3], &[]),
        Err(Error::EditDeferred(_))
    ));
    assert!(matches!(
        p.set_velocity_in_field(2, [1.; 3], [0.; 3], [0.; 3], &[]),
        Err(Error::EditDeferred(_))
    ));
    assert!(
        p.set_velocity_in_field(3, [1e307; 3], [0.; 3], [0.; 3], &[])
            .is_err()
    );
    assert_eq!(p.frame(), &frame);
    assert_eq!(requests(&p), previous);
    let next = BodyState {
        position: [21., 0., 2.],
        velocity: [0., 1., 0.],
        ..state(&prescribed)
    };
    p.replace_state_in_field(3, next, [0.; 3], &[]).unwrap();
    p.set_velocity_in_field(3, [0.2, 0., 0.], [0.; 3], [0.; 3], &[])
        .unwrap();
    p.advance().unwrap();
    assert!((p.frame().body(3).unwrap().1.position[0] - 21.001).abs() < 1e-12);
}

#[test]
fn a_new_twist_routes_the_next_impact_before_native_contact() {
    let a = body(1, 0.);
    let b = body(2, 0.9);
    let dt = SimulationStep::Ms10;
    let mut p = WorkerPool::spawn(
        worker(),
        18024,
        vec![vec![b.clone()], vec![], vec![a.clone()]],
        dt,
        None,
        12,
    )
    .unwrap();
    let mut native = MultiFidelityBackend::new([0.; 3]).unwrap();
    native.set_partition_step(dt.duration()).unwrap();
    let ids = [&a, &b].map(|b| {
        native
            .spawn_partition_primitive(
                BodySpec {
                    shape: b.shape().unwrap(),
                    position: b.position,
                    mass: Some(b.mass),
                    velocity: b.linear,
                },
                RotationSpec::default(),
                false,
                0.,
            )
            .unwrap()
    });
    let speed = [18., 0., 0.];
    p.set_velocity_in_field(1, speed, [0.; 3], [0.; 3], &[])
        .unwrap();
    native.set_world_velocity(ids[0], speed, [0.; 3]).unwrap();
    let mut colocated = None;
    let mut first_contact = None;
    let mut peak = 0_f64;
    for tick in 1..=12 {
        let f = p.advance().unwrap();
        native.step(dt.duration()).unwrap();
        for (id, local) in [(1, ids[0]), (2, ids[1])] {
            assert_bits(state(f.body(id).unwrap().1), native.state(local).unwrap());
        }
        if f.body(1).unwrap().0 == f.body(2).unwrap().0 {
            colocated.get_or_insert(tick);
        }
        let expected = native
            .rigid_contact_observations()
            .into_iter()
            .find(|c| c.active);
        let actual = f.workers.iter().flat_map(|w| &w.snapshot.impulses).next();
        assert_eq!(
            actual.map(|c| c.normal_impulse.to_bits()),
            expected.map(|c| c.normal_impulse.to_bits())
        );
        if let Some(c) = actual {
            first_contact.get_or_insert(tick);
            peak = peak.max(c.normal_impulse);
        }
    }
    assert!(colocated.unwrap() <= first_contact.unwrap());
    assert!(peak > 8.);
    println!(
        "velocity edit routed at tick {colocated:?}; first native contact at {first_contact:?}, peak {peak:.12} N s"
    );
}

#[test]
fn an_airborne_singleton_retains_its_ground_cache_edit_guard_after_image_restoration() {
    let a = Body {
        position: [0., 0., 0.35],
        ..body(1, 0.)
    };
    let gravity = [0., 0., -9.81];
    let mut p = WorkerPool::spawn(
        worker(),
        18025,
        vec![vec![], vec![a], vec![]],
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.).unwrap()),
        30,
    )
    .unwrap();
    for _ in 0..40 {
        p.advance_in_gravity(gravity).unwrap();
    }
    assert!(!p.frame().workers[1].snapshot.supports.is_empty());
    p.apply_impulse_in_field(1, [0., 0., 4.], [0.; 3], gravity, &[])
        .unwrap();
    for _ in 0..15 {
        p.advance_in_gravity(gravity).unwrap();
    }
    let f = &p.frame().workers[1];
    assert!(f.snapshot.supports.is_empty());
    assert!(f.components[0].contact_history);
    assert_eq!(f.components[0].ids, [1]);
    assert!(f.snapshot.bodies[0].position[2] > 0.5);
    p.compact().unwrap();
    p.recover(1).unwrap();
    let current = p.frame().clone();
    let traffic = requests(&p);
    let replacement = BodyState {
        position: [4., 0., 1.],
        ..state(current.body(1).unwrap().1)
    };
    assert!(matches!(
        p.replace_state_in_field(1, replacement, gravity, &[]),
        Err(Error::EditDeferred(_))
    ));
    assert_eq!(p.frame(), &current);
    assert_eq!(requests(&p), traffic);
}

#[test]
fn rotating_a_long_axial_primitive_into_a_foreign_sphere_defers_before_publication() {
    let worker_exe = worker();
    for (run, primitive) in [(18026, Primitive::Capsule), (18027, Primitive::Cylinder)] {
        let axial = Body {
            primitive,
            size: [0.3, 0.3, 1.9],
            ..body(1, 0.)
        };
        let obstacle = body(2, 0.8);
        let dt = SimulationStep::Ms5;
        let mut p = WorkerPool::spawn(
            worker_exe,
            run,
            vec![vec![obstacle], vec![], vec![axial.clone()]],
            dt,
            None,
            12,
        )
        .unwrap();
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let horizontal = BodyState {
            orientation: [0., half, 0., half],
            ..state(&axial)
        };
        let original = p.frame().clone();
        let traffic = requests(&p);
        assert!(matches!(
            p.replace_state_in_field(1, horizontal, [0.; 3], &[]),
            Err(Error::EditDeferred(_))
        ));
        assert_eq!(p.frame(), &original);
        assert_eq!(requests(&p), traffic);
        let away = BodyState {
            position: [0., 2.2, 3.],
            velocity: [0.1, 0.2, -0.3],
            angular_velocity: [0.13, -0.25, 0.09],
            ..horizontal
        };
        p.replace_state_in_field(1, away, [0.; 3], &[]).unwrap();
        let mut reference = MultiFidelityBackend::new([0.; 3]).unwrap();
        reference.set_partition_step(dt.duration()).unwrap();
        let b = reference
            .spawn_partition_primitive(
                BodySpec {
                    shape: axial.shape().unwrap(),
                    position: axial.position,
                    mass: Some(axial.mass),
                    velocity: axial.linear,
                },
                RotationSpec::default(),
                false,
                axial.friction,
            )
            .unwrap();
        reference.set_body_state(b, away).unwrap();
        for _ in 0..24 {
            p.advance().unwrap();
            reference.step(dt.duration()).unwrap();
            assert_bits(
                state(p.frame().body(1).unwrap().1),
                reference.state(b).unwrap(),
            );
        }
    }
}
