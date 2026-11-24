// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn initial(bodies: Vec<Body>) -> Island {
    Island::restore(&Snapshot {
        bodies,
        ..Snapshot::default()
    })
    .unwrap()
}

#[test]
fn promoted_handoff_then_collision_matches_single_worker_and_normal_impulse() {
    let specs = fixture();
    let mut monolithic = initial(specs.clone());
    let mut left = initial(vec![specs[0].clone()]);
    let mut right = initial(vec![specs[1].clone()]);
    for _ in 0..100 {
        left.step().unwrap();
        right.step().unwrap();
        monolithic.step().unwrap();
    }
    let incoming = left.export(1).unwrap();
    assert_eq!(incoming.tier, Tier::Prescribed);
    assert!(incoming.position[0].abs() < 1e-12);
    right.import(incoming.promoted()).unwrap();
    left.remove(1).unwrap();
    let own = monolithic.export(1).unwrap().promoted();
    monolithic.remove(1).unwrap();
    monolithic.import(own).unwrap();
    assert!(left.snapshot().bodies.is_empty());
    let mut max_state_error = 0_f64;
    let mut max_impulse_error = 0_f64;
    let mut net_normal_impulse = 0.;
    let mut impact = None;
    for tick in 101..=180 {
        left.step().unwrap();
        right.step().unwrap();
        monolithic.step().unwrap();
        let actual = right.snapshot();
        let expected = monolithic.snapshot();
        assert_eq!(actual.tick, tick);
        assert_eq!(actual.contacts, expected.contacts);
        assert_eq!(actual.impulses.len(), expected.impulses.len());
        for (a, b) in actual.bodies.iter().zip(expected.bodies) {
            assert_eq!(a.id, b.id);
            for (x, y) in a
                .position
                .into_iter()
                .chain(a.orientation)
                .chain(a.linear)
                .chain(a.angular)
                .zip(
                    b.position
                        .into_iter()
                        .chain(b.orientation)
                        .chain(b.linear)
                        .chain(b.angular),
                )
            {
                max_state_error = max_state_error.max((x - y).abs());
            }
        }
        for (a, b) in actual.impulses.iter().zip(expected.impulses) {
            assert_eq!((a.a, a.b), (1, 2));
            assert_eq!((b.a, b.b), (1, 2));
            max_impulse_error = max_impulse_error.max((a.normal_impulse - b.normal_impulse).abs());
            net_normal_impulse += a.normal_impulse;
            if a.normal_impulse > 1e-5 {
                impact.get_or_insert(tick);
            }
        }
    }
    let end = right.snapshot();
    assert!(end.contact_history);
    assert!((115..=121).contains(&impact.unwrap()));
    assert!(
        max_state_error < 1e-9 && max_impulse_error < 1e-9,
        "max state {max_state_error:e} / impulse {max_impulse_error:e}"
    );
    let [a, b] = end.bodies.as_slice() else {
        panic!("lost body")
    };
    let momentum = a.mass * a.linear[0] + b.mass * b.linear[0];
    assert!((momentum - 1.6).abs() < 1e-10);
    let impulse_change = b.mass * (b.linear[0] - specs[1].linear[0]);
    eprintln!(
        "impact tick {impact:?}, final vx [{},{}], normal impulse {net_normal_impulse}, measured dP {impulse_change}, max state {max_state_error:e}, max impulse {max_impulse_error:e}",
        a.linear[0], b.linear[0]
    );
    assert!(impulse_change > 1. && (net_normal_impulse - impulse_change).abs() < 1e-8);
    assert!(right.export(1).is_err() && right.remove(2).is_err());
    assert!(Island::restore(&right.snapshot()).is_err());
}

#[test]
fn refused_import_preserves_backend_state_and_local_bevy_identity() {
    let mut body = fixture()[1].clone();
    body.linear = [0.; 3];
    let mut world = initial(vec![body.clone()]);
    for _ in 0..100 {
        world.step().unwrap();
    }
    let before = world.snapshot();
    let local = world.index[&2];
    assert!(matches!(
        world.import(body.clone()),
        Err(WorldError::Duplicate(2))
    ));
    let foreign = Body {
        id: 17,
        ..body.clone()
    };
    assert!(world.import(foreign.clone()).is_err());
    assert_eq!(world.snapshot(), before);
    world
        .import(Body {
            position: [2.2, 0., 0.],
            ..foreign
        })
        .unwrap();
    assert_eq!(world.index[&2], local);
    let native = world.sim.world().get::<Binding>(local).unwrap().native;
    assert!(
        world
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .is_dynamic(native)
            == Some(true)
    );
    let restored = Island::restore(&world.snapshot()).unwrap();
    assert_eq!(world.snapshot(), restored.snapshot());
}

#[test]
fn free_flight_after_glancing_contact_retains_nonportable_history() {
    let mut bodies = fixture();
    bodies[0].tier = Tier::Dynamic;
    bodies[0].position = [-0.35, 0., 0.];
    bodies[1].position = [0.35, 0.12, 0.];
    bodies[1].linear[1] = 0.8;
    // Each cube's inertia about its center is isotropic. Compare total
    // angular momentum about the same fixed world origin before and after impact.
    let angular_momentum_z = |bodies: &[Body]| {
        bodies
            .iter()
            .map(|b| {
                b.mass * (b.position[0] * b.linear[1] - b.position[1] * b.linear[0])
                    + b.mass * b.size[0].powi(2) / 6. * b.angular[2]
            })
            .sum::<f64>()
    };
    let initial_lz = angular_momentum_z(&bodies);
    let original = bodies.clone();
    let mut reference = initial(bodies.clone());
    let mut source = initial(vec![bodies[0].clone()]);
    let mut world = initial(vec![bodies[1].clone()]);
    let mut impulse = 0.;
    let mut max_state_error = 0_f64;
    let mut max_impulse_error = 0_f64;
    for tick in 0..300 {
        if tick == 2 {
            world.import(source.export(1).unwrap()).unwrap();
            source.remove(1).unwrap();
        }
        reference.step().unwrap();
        source.step().unwrap();
        world.step().unwrap();
        if tick < 2 {
            continue;
        }
        assert_eq!(world.committed.contacts, reference.committed.contacts);
        assert_eq!(
            world.committed.impulses.len(),
            reference.committed.impulses.len()
        );
        for (a, b) in world
            .committed
            .impulses
            .iter()
            .zip(&reference.committed.impulses)
        {
            assert_eq!((a.a, a.b), (b.a, b.b));
            impulse += a.normal_impulse;
            max_impulse_error = max_impulse_error.max((a.normal_impulse - b.normal_impulse).abs());
        }
        for (a, b) in world
            .committed
            .bodies
            .iter()
            .zip(&reference.committed.bodies)
        {
            assert_eq!(a.id, b.id);
            for (x, y) in a
                .position
                .into_iter()
                .chain(a.orientation)
                .chain(a.linear)
                .chain(a.angular)
                .zip(
                    b.position
                        .into_iter()
                        .chain(b.orientation)
                        .chain(b.linear)
                        .chain(b.angular),
                )
            {
                max_state_error = max_state_error.max((x - y).abs());
            }
        }
    }
    let final_lz = angular_momentum_z(&world.committed.bodies);
    let max_spin = world
        .committed
        .bodies
        .iter()
        .map(|b| b.angular[2].abs())
        .fold(0.0_f64, f64::max);
    eprintln!(
        "glancing cuboids: angular momentum z {initial_lz:.9} -> {final_lz:.9}, peak final |omega_z| {max_spin:.9}"
    );
    let native_lz = angular_momentum_z(&reference.committed.bodies);
    eprintln!(
        "10 ms uninterrupted native Lz {native_lz:.12}, migrated Lz {final_lz:.12}, peak state difference {max_state_error:e}, peak contact impulse difference {max_impulse_error:e}"
    );
    assert!(max_state_error < 1e-9 && max_impulse_error < 1e-9);
    assert!((final_lz - native_lz).abs() < 1e-10);
    assert!(max_spin > 0.1);
    // Compare the fixed public step against the same native collision at a
    // finer step. A position correction made within a contact tick can change
    // the discrete orbital moment, even when the impulses conserve momentum.
    for dt_ms in [5, 2, 1] {
        let (mut fine, handles) = materialize(&original, SimulationStep::default(), None, NativeSolverProfile::Standard32).unwrap();
        let dt = Duration::from_millis(dt_ms);
        for _ in 0..3000 / dt_ms {
            fine.step(dt).unwrap();
        }
        let mut observed = original.clone();
        for body in &mut observed {
            let state = fine.state(handles[&body.id]).unwrap();
            body.position = state.position;
            body.linear = state.velocity;
            body.angular = state.angular_velocity;
        }
        let finer_error = (angular_momentum_z(&observed) - initial_lz).abs();
        eprintln!("fine dt {dt_ms} ms angular error {finer_error:.9}");
    }
    assert!(impulse > 0.1);
    assert_eq!(world.committed.contacts, 0);
    assert!(world.committed.contact_history);
    assert!(matches!(world.export(1), Err(WorldError::ContactHistory)));
    assert!(matches!(
        Island::restore(&world.snapshot()),
        Err(WorldError::ContactHistory)
    ));
}

#[test]
fn invalid_barriers_and_ineligible_native_motion_refuse_before_any_mutation() {
    let a = fixture()[0].clone();
    let mut world = initial(vec![a.clone()]);
    let before = world.snapshot();
    assert!(matches!(world.export(173), Err(WorldError::Missing(173))));
    assert!(matches!(world.remove(173), Err(WorldError::Missing(173))));
    let candidate = Body {
        id: 22,
        position: [10., 0., 0.],
        ..a.clone()
    };
    world.validate_import(&candidate).unwrap();
    let excessive_spin = Body {
        angular: [0., 0., 1000.],
        ..candidate
    };
    assert!(
        world.validate_import(&excessive_spin).is_err() && world.import(excessive_spin).is_err()
    );
    assert_eq!(world.snapshot(), before);
    let duplicate = Snapshot {
        bodies: vec![a.clone(), a],
        ..Snapshot::default()
    };
    assert!(matches!(
        Island::restore(&duplicate),
        Err(WorldError::Duplicate(1))
    ));
    let mut end = Island::restore(&Snapshot {
        tick: u64::MAX,
        ..before
    })
    .unwrap();
    assert!(matches!(end.step(), Err(WorldError::TimeOverflow)));
    assert_eq!(end.snapshot().tick, u64::MAX);
}

#[test]
fn failed_bevy_step_preserves_committed_state_and_prevents_transfer() {
    let mut world = initial(fixture());
    let before = world.snapshot();
    world.inject_commit_fault();
    assert!(matches!(world.step(), Err(WorldError::Fault(_))));
    assert_eq!(world.snapshot(), before);
    assert!(matches!(world.export(1), Err(WorldError::Fault(_))));
}

#[test]
fn simultaneous_impacts_are_reported_per_pair_and_refuse_fresh_worker_ownership() {
    use crate::protocol::{Command, Outcome, ProtocolError, Request, Ticket, Worker};
    let cube = fixture()[0].promoted();
    let body = |id, x, v| Body {
        id,
        position: [x, 0., 0.],
        linear: [v, 0., 0.],
        ..cube.clone()
    };
    let seed = Snapshot {
        bodies: vec![body(77, 0., 0.), body(300, -0.7, 1.5), body(9, 0.7, -1.5)],
        ..Snapshot::default()
    };
    let mut worker = Worker::new(1, 0, seed.clone()).unwrap();
    assert!(matches!(
        Worker::new(
            1,
            0,
            Snapshot {
                contact_history: true,
                ..seed.clone()
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    let mut local = Island::restore(&seed).unwrap();
    let mut seq = 0;
    let request = |sequence, command| Request {
        run_id: 1,
        worker_id: 0,
        sequence,
        command,
    };
    for tick in 0..20 {
        seq += 1;
        let reply = worker.handle(request(seq, Command::Step { tick })).unwrap();
        let Outcome::State(obs) = reply.outcome else {
            panic!("missing committed state")
        };
        local.step().unwrap();
        assert_eq!(obs.snapshot, local.snapshot());
        let impulses = &obs.snapshot.impulses;
        if impulses.len() != 2 || impulses.iter().any(|p| p.normal_impulse < 0.1) {
            continue;
        }
        assert_eq!(
            [
                (impulses[0].a, impulses[0].b),
                (impulses[1].a, impulses[1].b)
            ],
            [(9, 77), (77, 300)]
        );
        let candidate = body(17, 5., 0.);
        assert!(matches!(
            local.validate_import(&candidate),
            Err(WorldError::ContactHistory)
        ));
        assert!(matches!(
            local.import(candidate),
            Err(WorldError::ContactHistory)
        ));
        let ticket = Ticket {
            run_id: 1,
            serial: 1,
            source: 0,
            destination: 1,
            body_id: 77,
            generation: 1,
            tick: tick + 1,
            promote: false,
        };
        assert!(matches!(
            worker.handle(request(seq + 1, Command::Export { ticket })),
            Err(ProtocolError::Physics(_))
        ));
        let retry = worker.handle(request(seq + 1, Command::Observe)).unwrap();
        assert_eq!(retry.outcome, Outcome::State(obs));
        return;
    }
    panic!("did not resolve two simultaneous opposing impacts");
}

#[test]
fn warm_active_contact_and_ordered_pending_impulses_replay_bit_for_bit() {
    let boxes: Vec<_> = fixture().iter().map(Body::promoted).collect();
    let mut control = initial(boxes.clone());
    let mut traveller = initial(boxes);
    while control.snapshot().contacts == 0 {
        control.step().unwrap();
        traveller.step().unwrap();
        assert!(control.snapshot().tick < 150);
    }
    let impact = control.snapshot();
    assert!(impact.impulses[0].normal_impulse > 1.);
    let momentum = |s: &Snapshot| s.bodies.iter().map(|b| b.mass * b.linear[0]).sum::<f64>();
    let mut expected_momentum = momentum(&impact);
    let mut active_resumes = 0;
    for tick in 0..25 {
        for island in [&mut control, &mut traveller] {
            island.queue_impulse(1, [0.04, 0., -0.]).unwrap();
            island.queue_impulse(2, [-0.02, 0., 0.]).unwrap();
        }
        expected_momentum += 0.02;
        let committed = traveller.snapshot();
        let archive = traveller.checkpoint().unwrap();
        assert!(archive.native_bytes() > 500);
        assert_eq!(archive.pending.len(), 2);
        let json = serde_json::to_vec(&archive).unwrap();
        let wire = serde_json::from_slice(&json).unwrap();
        assert_eq!(archive, wire);
        active_resumes += usize::from(committed.contacts != 0);
        traveller = Island::restore_warm(&wire).unwrap();
        assert_eq!(traveller.snapshot(), committed);
        for island in [&mut control, &mut traveller] {
            island.step().unwrap();
        }
        assert_eq!(
            traveller.snapshot(),
            control.snapshot(),
            "resumed step {tick}"
        );
        assert!((momentum(&traveller.snapshot()) - expected_momentum).abs() < 1e-11);
        assert!(traveller.checkpoint().unwrap().pending.is_empty());
    }
    eprintln!(
        "warm checkpoint tick {}, impulse {} N s, {} active-contact resumes",
        impact.tick, impact.impulses[0].normal_impulse, active_resumes
    );
    assert!(active_resumes > 10);
}

#[test]
fn pending_impulses_refuse_cold_moves_and_warm_restoration_detects_corruption() {
    let boxes = fixture();
    let mut mixed = initial(boxes.clone());
    assert!(mixed.checkpoint().is_err());
    assert!(mixed.queue_impulse(1, [0.; 3]).is_err());
    assert!(mixed.queue_impulse(99, [0.; 3]).is_err());
    assert!(mixed.queue_impulse(2, [f64::NAN; 3]).is_err());
    let mut island = initial(boxes.iter().map(Body::promoted).collect());
    for _ in 0..MAX_PENDING_IMPULSES {
        island.queue_impulse(1, [0.01, -0., 0.]).unwrap();
    }
    assert!(island.queue_impulse(2, [1.; 3]).is_err());
    assert!(matches!(island.export(2), Err(WorldError::Pending)));
    assert!(matches!(island.remove(1), Err(WorldError::Pending)));
    let before = island.checkpoint().unwrap();
    for field in [
        "native", "bindings", "snapshot", "pending", "schema", "digest",
    ] {
        let mut corrupt = serde_json::to_value(&before).unwrap();
        match field {
            "native" => {
                corrupt[field].as_array_mut().unwrap().pop();
            }
            "bindings" => corrupt[field][1] = 999.into(),
            "snapshot" => corrupt[field]["tick"] = 42.into(),
            "pending" => corrupt[field][0]["impulse"][1] = 0.0.into(),
            "schema" => corrupt[field] = 42.into(),
            _ => corrupt[field][0] = (255 - corrupt[field][0].as_u64().unwrap()).into(),
        }
        let changed = serde_json::from_value(corrupt).unwrap();
        assert_ne!(before, changed);
        assert!(
            Island::restore_warm(&changed).is_err(),
            "corruption in {field}"
        );
    }
    let mut restored = Island::restore_warm(&before).unwrap();
    restored.step().unwrap();
    assert!((restored.snapshot().bodies[0].linear[0] - 2.32).abs() < 1e-13);
    restored.step().unwrap();
    assert!((restored.snapshot().bodies[0].linear[0] - 2.32).abs() < 1e-13);
}

#[test]
fn warm_glancing_contact_retains_solver_impulses_and_rotational_state() {
    let mut boxes = fixture();
    boxes[0].tier = Tier::Dynamic;
    boxes[0].position = [-0.35, 0., 0.];
    boxes[1].position = [0.35, 0.12, 0.];
    boxes[1].linear[1] = 0.8;
    let mut control = initial(boxes.clone());
    let mut travelled = initial(boxes);
    let mut migrations = 0;
    let mut peak_warm = 0.0_f64;
    let mut peak_spin = 0.0_f64;
    for _ in 0..150 {
        travelled.step().unwrap();
        control.step().unwrap();
        assert_eq!(travelled.snapshot(), control.snapshot());
        let barrier = travelled.snapshot();
        if barrier.contacts == 0 {
            continue;
        }
        let wire = travelled.checkpoint().unwrap();
        let inputs: Vec<_> = wire.warmstart_normal_impulses().map(f64::to_bits).collect();
        let warm = wire.warmstart_normal_impulses().fold(0.0_f64, f64::max);
        let spin = barrier
            .bodies
            .iter()
            .map(|b| b.angular[2].abs())
            .fold(0.0_f64, f64::max);
        travelled = Island::restore_warm(&wire).unwrap();
        let restored = travelled.checkpoint().unwrap();
        assert_eq!(wire.manifold_count, 1);
        assert_eq!(restored.manifold_count, wire.manifold_count);
        assert_eq!(
            restored
                .warmstart_normal_impulses()
                .map(f64::to_bits)
                .collect::<Vec<_>>(),
            inputs
        );
        migrations += 1;
        peak_warm = peak_warm.max(warm);
        peak_spin = peak_spin.max(spin);
    }
    eprintln!(
        "glancing 150 public ticks, {migrations} active checkpoints, maximum retained native normal warm-start {peak_warm:.12} N s, maximum contact spin {peak_spin:.9} rad/s"
    );
    assert!(migrations >= 1 && peak_warm > 0. && peak_spin > 0.1);
}
