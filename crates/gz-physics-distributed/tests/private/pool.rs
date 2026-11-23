// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{
    physics::{Primitive, Tier, fixture},
    protocol::{Handoff, Prepared, Ticket, Worker},
};
fn published() -> PoolFrame {
    let mut workers = Vec::new();
    for w in 0..3u8 {
        let mut body = fixture()[1].clone();
        body.id = w.into();
        let partition = Worker::with_width(
            220,
            w,
            Snapshot {
                bodies: vec![body],
                ..Default::default()
            },
            3,
        )
        .unwrap();
        workers.push(partition.observation());
    }
    PoolFrame { tick: 0, workers }
}
#[test]
fn a_partial_epoch_or_duplicate_authority_never_forms_a_publishable_frame() {
    let exact = published();
    exact.check(NativeSolverProfile::Standard32).unwrap();
    let mut corrupt = exact.clone();
    corrupt.workers[1].snapshot.tick += 1;
    assert!(corrupt.check(NativeSolverProfile::Standard32).is_err());
    let mut corrupt = exact.clone();
    corrupt.workers[1].snapshot.step = SimulationStep::Ms1;
    assert!(corrupt.check(NativeSolverProfile::Standard32).is_err());
    let mut corrupt = exact.clone();
    corrupt.workers[2] = corrupt.workers[0].clone();
    assert!(corrupt.check(NativeSolverProfile::Standard32).is_err());
    let mut corrupt = exact.clone();
    corrupt.workers[2].generations.clear();
    assert!(corrupt.check(NativeSolverProfile::Standard32).is_err());
}
#[test]
fn a_successful_step_cannot_smuggle_new_geometry_material_or_ownership_into_the_next_frame() {
    let mut frame = published();
    let old = frame.clone();
    for n in 0..9 {
        let mut obs = old.workers[0].clone();
        let body = &mut obs.snapshot.bodies[0];
        match n {
            0 => body.size[1] *= 2.,
            1 => body.primitive = Primitive::Sphere,
            2 => body.friction = -0.,
            3 => body.mass *= 0.5,
            4 => body.tier = Tier::Fixed,
            5 => body.id += 17,
            6 => {
                obs.snapshot.bodies.clear();
            }
            7 => {
                obs.generations.insert(0, 23);
            }
            _ => obs.snapshot.step = SimulationStep::Ms2,
        }
        assert!(frame.accept_step(0, Outcome::State(obs)).is_err());
        assert_eq!(frame, old);
    }
    let handoff = Handoff {
        ticket: Ticket {
            run_id: 220,
            serial: 1,
            source: 1,
            destination: 2,
            body_id: 0,
            generation: 1,
            tick: 0,
            promote: false,
        },
        body: old.workers[0].snapshot.bodies[0].clone(),
    };
    assert!(
        frame
            .accept_step(0, Outcome::Prepared(Prepared { handoff }))
            .is_err()
    );
    assert_eq!(frame, old);
}
#[test]
fn a_replayed_reply_with_only_a_changed_signed_zero_diverges_at_its_recorded_sequence() {
    let obs = published().workers.remove(2);
    let command = Request {
        run_id: 220,
        worker_id: 2,
        sequence: 42,
        command: Command::Observe,
    };
    let mut reply = Reply {
        run_id: 220,
        worker_id: 2,
        sequence: 42,
        outcome: Outcome::State(obs),
    };
    let mut journal = Journal::default();
    journal.push(command, &reply).unwrap();
    journal.entries[0].verify(&reply).unwrap();
    let Outcome::State(changed) = &mut reply.outcome else {
        unreachable!()
    };
    changed.snapshot.bodies[0].angular[1] = -0.;
    assert!(matches!(
        journal.entries[0].verify(&reply),
        Err(Error::Diverged(2, 42))
    ));
}

#[test]
fn a_state_edit_ack_must_preserve_the_exact_requested_body_and_the_rest_of_its_owner() {
    let mut frame = published();
    let original = frame.clone();
    let mut body = frame.workers[1].snapshot.bodies[0].clone();
    body.position = [1., -0., 3.];
    body.angular = [0.2, -0., 0.1];
    let mut edited = frame.workers[1].clone();
    edited.snapshot.bodies[0] = body.clone();

    let wrong_phase = Outcome::WarmActivated(edited.clone());
    assert!(frame.accept_edit(1, body.clone(), wrong_phase).is_err());
    for corrupt in 0..6 {
        let mut obs = edited.clone();
        match corrupt {
            0 => obs.snapshot.bodies.clear(),
            1 => obs.snapshot.bodies[0].position[1] = 0.,
            2 => obs.snapshot.bodies[0].angular[1] = 0.,
            3 => obs.snapshot.bodies[0].friction = -0.,
            4 => obs.components[0].contact_history = true,
            _ => {
                obs.generations.insert(1, 4);
            }
        }
        assert!(
            frame
                .accept_edit(1, body.clone(), Outcome::State(obs))
                .is_err()
        );
        assert_eq!(frame, original);
    }
    frame
        .accept_edit(1, body, Outcome::State(edited.clone()))
        .unwrap();
    assert_eq!(frame.workers[1], edited);
}

#[test]
fn helper_grants_require_distinct_fine_worlds_and_spare_lanes() {
    let mut frame = published();
    let mut requests = Vec::new();
    for (w, o) in frame.workers.iter_mut().enumerate() {
        let id = o.snapshot.bodies[0].id;
        let second = id + 10;
        o.components.push(crate::physics::ComponentInfo {
            ids: vec![second],
            contact_history: false,
        });
        requests.push(Request {
            run_id: 220,
            worker_id: w as u8,
            sequence: 2,
            command: Command::StepIslands {
                tick: 0,
                field: StepField {
                    gravity: [0.; 3],
                    wrenches: vec![],
                },
                refined: vec![id, second],
            },
        });
    }
    assert_eq!(local_helpers(&frame, &requests, 3, None), 0);
    assert_eq!(local_helpers(&frame, &requests, 4, None), 1);
    assert_eq!(local_helpers(&frame, &requests, 8, None), 7);
    // Identical fine counts, but one loaded peer also has ground work.
    frame.workers[2]
        .snapshot
        .supports
        .push(crate::physics::GroundSupport {
            body: 2,
            normal_impulse: 0.1,
        });
    assert_eq!(local_helpers(&frame, &requests, 4, None), 4);
    assert_eq!(local_helpers(&frame, &requests, 5, None), 5);
    // A configured cap uses the same work ranking and stable worker ID tie-break
    // as CPU contention. Excess allowance still respects the available CPUs.
    assert_eq!(local_helpers(&frame, &requests, 8, Some(0)), 0);
    assert_eq!(local_helpers(&frame, &requests, 8, Some(1)), 4);
    assert_eq!(local_helpers(&frame, &requests, 8, Some(2)), 5);
    assert_eq!(local_helpers(&frame, &requests, 4, Some(usize::MAX)), 4);
    // A coarse world adds one unit per body/solver and per contact. Check
    // that the heavy fine peer still takes the only grant.
    frame.workers[1]
        .components
        .push(crate::physics::ComponentInfo {
            ids: vec![42, 43],
            contact_history: true,
        });
    frame.workers[1]
        .snapshot
        .impulses
        .push(crate::physics::ContactImpulse {
            a: 42,
            b: 43,
            normal_impulse: 0.1,
        });
    assert_eq!(owner_work(&frame.workers[1], &[1, 11]), 44);
    assert_eq!(local_helpers(&frame, &requests, 4, None), 4);
    frame.workers[1].components.pop();
    requests[0].command = Command::Step { tick: 0 };
    frame.workers[1].components.pop();
    assert_eq!(local_helpers(&frame, &requests, 4, None), 4);
}
