// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, BodyWrench, Island, Primitive, SimulationStep, Snapshot, StepField, Tier},
    protocol::{Command, Outcome, ProtocolError, Reply, Request, Worker},
};
use std::error::Error;

const RUN: u64 = 79;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn request(sequence: u64, command: Command) -> Request {
    Request {
        run_id: RUN,
        worker_id: 0,
        sequence,
        command,
    }
}

fn state(reply: &Reply) -> &Snapshot {
    match &reply.outcome {
        Outcome::State(s) => &s.snapshot,
        _ => panic!("expected worker state"),
    }
}

fn seed(bodies: &[Body]) -> Snapshot {
    Snapshot {
        bodies: bodies.to_vec(),
        ..Snapshot::default()
    }
}

fn command(tick: u64, field: &StepField, refined: &[u64]) -> Command {
    Command::StepIslands {
        tick,
        field: field.clone(),
        refined: refined.to_vec(),
    }
}

#[test]
fn one_worker_solves_only_selected_islands_and_replays_the_mixed_barrier() -> Result<()> {
    let resting = Body {
        id: 1,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        mass: 1.,
        tier: Tier::Dynamic,
        friction: 0.,
        position: [0., 0., 3.],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    };
    let hitting = Body {
        id: 2,
        position: [-0.48, 0.12, 3.],
        ..resting.clone()
    };
    let free = Body {
        id: 3,
        position: [20., 0., 6.],
        ..resting.clone()
    };
    let bodies = [resting, hitting, free];
    let field = StepField {
        gravity: [0., 0., -9.81],
        wrenches: vec![BodyWrench {
            id: 3,
            force: [1., 0.5, 0.],
            torque: [0., 0., 0.1],
        }],
    };
    let mut worker = Worker::new(RUN, 0, seed(&bodies))?;
    let mut pair_control = Island::restore(&seed(&bodies[..2]))?;
    let mut free_control = Island::restore(&seed(&bodies[2..]))?;
    let mut all_fine_free = Island::restore(&seed(&bodies[2..]))?;
    let impulse = [80., 0., 0.];
    worker.handle(request(
        1,
        Command::QueueImpulse {
            tick: 0,
            id: 2,
            impulse,
        },
    ))?;
    pair_control.queue_impulse(2, impulse)?;
    let image = worker.recovery_image(1, 0)?;

    // A queued input will merge the first two cold spheres on the next step.
    // A partial group is inadmissible even though they start in separate solvers.
    for invalid in [&[1][..], &[2, 1], &[1, 1], &[4], &[1, 2, 3, 4]] {
        assert!(matches!(
            worker.handle(request(2, command(0, &field, invalid))),
            Err(ProtocolError::Physics(_))
        ));
        assert_eq!(worker.recovery_image(1, 0)?, image);
    }
    let first = request(2, command(0, &field, &[1, 2]));
    let wire: Request = bincode::deserialize(&bincode::serialize(&first)?)?;
    assert_eq!(wire, first);
    let ack = worker.handle(wire)?;
    assert_eq!(worker.handle(first.clone())?, ack);
    assert_eq!(
        worker.handle(request(2, command(0, &field, &[3]))),
        Err(ProtocolError::ConflictingRetry)
    );

    pair_control.step_refined_in_field(field.gravity, &[], 10)?;
    free_control.step_refined_in_field(field.gravity, &field.wrenches, 1)?;
    all_fine_free.step_refined_in_field(field.gravity, &field.wrenches, 10)?;
    assert_ne!(
        all_fine_free.snapshot().bodies,
        free_control.snapshot().bodies
    );
    let check = |s: &Snapshot, p: &Island, f: &Island| {
        let pair = p.snapshot();
        let free = f.snapshot();
        assert_eq!(s.bodies, [pair.bodies, free.bodies].concat());
        assert_eq!(s.impulses, pair.impulses);
        assert_eq!(s.tick, pair.tick);
        assert_eq!(s.tick, free.tick);
    };
    check(state(&ack), &pair_control, &free_control);
    assert!((state(&ack).impulses[0].normal_impulse - 32.).abs() < 1e-10);

    // A replay from the pre-step image executes the kick exactly once, while
    // a post-step image preserves the transient impulse and both solve widths'
    // native end states. Switch widths on both islands at the following tick.
    let mut replay = Worker::from_trusted_image(RUN, 0, &image)?;
    assert_eq!(replay.handle(first)?, ack);
    let mut restored = Worker::from_trusted_image(RUN, 0, &worker.recovery_image(2, 1)?)?;
    let next = request(3, command(1, &field, &[3]));
    let second = worker.handle(next.clone())?;
    assert_eq!(replay.handle(next.clone())?, second);
    assert_eq!(restored.handle(next)?, second);
    pair_control.step_refined_in_field(field.gravity, &[], 1)?;
    free_control.step_refined_in_field(field.gravity, &field.wrenches, 10)?;
    check(state(&second), &pair_control, &free_control);
    Ok(())
}

#[test]
fn empty_worker_requires_the_same_public_clock_for_a_mixed_schedule() -> Result<()> {
    let field = StepField {
        gravity: [0.; 3],
        wrenches: vec![],
    };
    let mut worker = Worker::new(RUN, 0, seed(&[]))?;
    let ack = worker.handle(request(1, command(0, &field, &[])))?;
    assert_eq!(state(&ack).tick, 1);
    let mut other = Worker::new(
        RUN,
        0,
        Snapshot {
            step: SimulationStep::Ms2,
            ..seed(&[])
        },
    )?;
    assert!(matches!(
        other.handle(request(1, command(0, &field, &[]))),
        Err(ProtocolError::Physics(_))
    ));
    assert_eq!(state(&other.handle(request(1, Command::Observe))?).tick, 0);
    Ok(())
}

#[test]
fn separated_former_contact_partners_can_resume_different_resolutions() -> Result<()> {
    let a = Body {
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
    let b = Body {
        id: 2,
        position: [-0.48, 0.12, 0.],
        linear: [80., 0., 0.],
        ..a.clone()
    };
    let mut worker = Worker::new(RUN, 0, seed(&[a, b]))?;
    let field = StepField {
        gravity: [0., 0., -9.81],
        wrenches: vec![],
    };
    let mut separated = None;
    for tick in 0..40 {
        let ack = worker.handle(request(tick + 1, command(tick, &field, &[1, 2])))?;
        let Outcome::State(obs) = ack.outcome else {
            panic!("expected state")
        };
        if tick == 0 {
            assert_eq!(obs.components.len(), 1);
            assert!((obs.snapshot.impulses[0].normal_impulse - 32.).abs() < 1e-10);
        }
        if obs.components.len() == 2 {
            assert!(obs.snapshot.impulses.is_empty());
            separated = Some(tick + 1);
            break;
        }
    }
    let tick = separated.expect("missing fission after the glancing impact");
    assert!(tick <= 4, "glancing spheres first separated at tick {tick}");
    let image = worker.recovery_image(tick, tick)?;
    let mut coarse = Worker::from_trusted_image(RUN, 0, &image)?;
    let mut fine = Worker::from_trusted_image(RUN, 0, &image)?;
    let seq = tick + 1;
    let mixed = worker.handle(request(seq, command(tick, &field, &[1])))?;
    let one = coarse.handle(request(
        seq,
        Command::StepInField {
            tick,
            field: field.clone(),
        },
    ))?;
    let ten = fine.handle(request(seq, Command::StepRefined { tick, field }))?;
    assert_eq!(state(&mixed).bodies[0], state(&ten).bodies[0]);
    assert_eq!(state(&mixed).bodies[1], state(&one).bodies[1]);
    assert_ne!(state(&mixed).bodies[0], state(&one).bodies[0]);
    assert_ne!(state(&mixed).bodies[1], state(&ten).bodies[1]);
    Ok(())
}

#[test]
fn fission_preserves_loaded_neighbors_when_the_third_body_leaves() -> Result<()> {
    let a = Body {
        id: 1,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        mass: 1.,
        tier: Tier::Dynamic,
        friction: 0.,
        position: [-0.125, 0., 3.],
        orientation: [0., 0., 0., 1.],
        linear: [4., 0., 0.],
        angular: [0.; 3],
    };
    let b = Body {
        id: 2,
        position: [0.125, 0., 3.],
        linear: [-4., 0., 0.],
        ..a.clone()
    };
    let departing = Body {
        id: 3,
        position: [0.8, 0., 3.],
        linear: [30., 0., 0.],
        ..a.clone()
    };
    let initial = seed(&[a, b, departing]);
    let mut native = Island::restore(&initial)?;
    let mut worker = Worker::new(RUN, 0, initial)?;
    let field = StepField {
        gravity: [0., 0., -9.81],
        wrenches: [(1, 5.), (2, -5.)]
            .map(|(id, fx)| BodyWrench {
                id,
                force: [fx, 0., 0.],
                torque: [0.; 3],
            })
            .into(),
    };
    let mut split = None;
    for tick in 0..40 {
        let step = command(
            tick,
            &field,
            if split.is_some() { &[1, 2] } else { &[1, 2, 3] },
        );
        let ack = worker.handle(request(tick + 1, step))?;
        let Outcome::State(obs) = ack.outcome else {
            panic!("expected state")
        };
        native.step_refined_in_field(field.gravity, &field.wrenches, 10)?;
        let reference = native.snapshot();
        assert_eq!(obs.snapshot.bodies[..2], reference.bodies[..2]);
        assert_eq!(obs.snapshot.impulses, reference.impulses);
        assert!(
            obs.snapshot
                .impulses
                .iter()
                .any(|p| (p.a, p.b) == (1, 2) && p.normal_impulse > 0.)
        );
        if tick == 0 {
            assert_eq!(obs.components.len(), 1);
        }
        if obs.components.len() == 2 {
            assert_eq!(obs.components[0].ids, [1, 2]);
            assert_eq!(obs.components[1].ids, [3]);
            split.get_or_insert(tick + 1);
        }
    }
    let tick = split.expect("departing body stayed with the loaded pair");
    assert!(tick <= 13, "loaded pair first separated at tick {tick}");
    Ok(())
}
