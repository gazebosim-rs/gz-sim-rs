// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn pair() -> Vec<Body> {
    vec![body(1, Tier::Dynamic), body(2, Tier::Dynamic)]
}
fn warm_ticket() -> WarmTicket {
    WarmTicket {
        run_id: RUN,
        source: 0,
        destination: 1,
        serial: 1,
        tick: 1,
        ids: vec![1, 2],
        generations: vec![1, 1],
    }
}
fn after_one(id: u8, seed: Vec<Body>) -> Worker {
    let mut worker = worker(id, seed);
    send(&mut worker, 1, Command::Step { tick: 0 }).unwrap();
    worker
}
fn queue() -> Command {
    Command::QueueImpulse {
        tick: 1,
        id: 2,
        impulse: [0.125, 0., 0.],
    }
}
fn export(worker: &mut Worker, seq: u64, ticket: WarmTicket) -> Box<WarmHandoff> {
    let Outcome::WarmExported(h) = send(worker, seq, Command::ExportWarm { ticket }).unwrap()
    else {
        panic!("expected exported native checkpoint")
    };
    h
}

#[test]
fn complete_native_swap_has_phase_specific_receipts_and_consumes_the_queued_input_once() {
    let mut a = after_one(0, pair());
    let mut b = after_one(1, vec![]);
    let mut control = after_one(0, pair());
    let before = a.observation();
    let empty = b.observation();
    let input = queue();
    let ack = send(&mut a, 2, input.clone()).unwrap();
    assert_eq!(ack, Outcome::State(before.clone()));
    assert_eq!(send(&mut a, 2, input.clone()).unwrap(), ack);
    send(&mut control, 2, input).unwrap();
    let signed_zero = Command::QueueImpulse {
        tick: 1,
        id: 2,
        impulse: [0.125, -0., 0.],
    };
    assert_eq!(
        send(&mut a, 2, signed_zero),
        Err(ProtocolError::ConflictingRetry)
    );
    let t = warm_ticket();
    let h = export(&mut a, 3, t.clone());
    assert_eq!(export(&mut a, 3, t), h);
    assert_eq!(h.checkpoint.snapshot, before.snapshot);
    assert_eq!(h.checkpoint.pending.len(), 1);
    h.validate_digest().unwrap();
    let r = h.receipt.clone();
    let prepare = Command::PrepareWarm { handoff: h };
    let prepared = send(&mut b, 2, prepare.clone()).unwrap();
    assert_eq!(prepared, Outcome::WarmPrepared(r.prepared()));
    assert_eq!(send(&mut b, 2, prepare.clone()).unwrap(), prepared);
    let Command::PrepareWarm { mut handoff } = prepare else {
        unreachable!()
    };
    handoff.checkpoint.pending[0].impulse[2] = -0.;
    assert_eq!(
        send(&mut b, 2, Command::PrepareWarm { handoff }),
        Err(ProtocolError::ConflictingRetry)
    );
    assert_eq!(b.observation(), empty);
    assert_eq!(a.observation(), before);
    for w in [&mut a, &mut b] {
        let seq = w.last.as_ref().unwrap().0.sequence + 1;
        assert_eq!(
            send(w, seq, Command::Step { tick: 1 }),
            Err(ProtocolError::Pending)
        );
        assert_eq!(send(w, seq, queue()), Err(ProtocolError::Pending));
    }
    assert_eq!(
        send(&mut a, 4, Command::ReleaseWarm { receipt: r.clone() }),
        Err(ProtocolError::Receipt)
    );
    assert_eq!(
        send(
            &mut b,
            3,
            Command::ActivateWarm {
                receipt: r.prepared()
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let mut wrong_release = r.released();
    wrong_release.digest[0] ^= 1;
    assert_eq!(
        send(
            &mut b,
            3,
            Command::ActivateWarm {
                receipt: wrong_release
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let mut wrong_prepare = r.prepared();
    wrong_prepare.ticket.generations[0] += 1;
    assert_eq!(
        send(
            &mut a,
            4,
            Command::ReleaseWarm {
                receipt: wrong_prepare
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let release = Command::ReleaseWarm {
        receipt: r.prepared(),
    };
    assert_eq!(
        send(&mut a, 4, release.clone()).unwrap(),
        Outcome::WarmReleased(r.released())
    );
    assert_eq!(
        send(&mut a, 4, release.clone()).unwrap(),
        Outcome::WarmReleased(r.released())
    );
    assert_eq!(a.observation(), empty);
    assert_eq!(b.observation(), empty);
    assert_eq!(send(&mut a, 5, release), Err(ProtocolError::Receipt));
    let activate = Command::ActivateWarm {
        receipt: r.released(),
    };
    let activated = send(&mut b, 3, activate.clone()).unwrap();
    assert_eq!(send(&mut b, 3, activate.clone()).unwrap(), activated);
    assert_eq!(send(&mut b, 4, activate), Err(ProtocolError::Receipt));
    assert_eq!(
        b.observation().generations,
        BTreeMap::from([(1, 1), (2, 1)])
    );
    assert_eq!(b.observation().snapshot, before.snapshot);
    for tick in 1..=2 {
        let expected = observed(send(&mut control, tick + 2, Command::Step { tick }).unwrap());
        let actual = observed(send(&mut b, tick + 3, Command::Step { tick }).unwrap());
        assert_eq!(actual.snapshot, expected.snapshot);
        assert!((actual.snapshot.bodies[1].linear[0] - 0.275).abs() < 1.0e-12);
    }
    for msg in [Outcome::WarmReleased(r.released()), prepared] {
        assert!(serde_json::to_vec(&msg).unwrap().len() < 512);
    }
}

#[test]
fn refused_warm_transfers_leave_sequence_tick_and_all_owners_reusable() {
    let mut a = after_one(0, pair());
    let mut b = after_one(1, vec![]);
    let ticket = warm_ticket();
    for (t, expected) in [
        (
            WarmTicket {
                ids: vec![2, 1],
                ..ticket.clone()
            },
            ProtocolError::Ticket,
        ),
        (
            WarmTicket {
                ids: vec![1, 3],
                ..ticket.clone()
            },
            ProtocolError::Ownership,
        ),
        (
            WarmTicket {
                generations: vec![1, 2],
                ..ticket.clone()
            },
            ProtocolError::Ownership,
        ),
        (
            WarmTicket {
                run_id: RUN + 1,
                ..ticket.clone()
            },
            ProtocolError::Address,
        ),
        (
            WarmTicket {
                serial: 0,
                ..ticket.clone()
            },
            ProtocolError::Ticket,
        ),
    ] {
        assert_eq!(
            send(&mut a, 2, Command::ExportWarm { ticket: t }),
            Err(expected)
        );
    }
    let h = export(&mut a, 2, ticket.clone());
    let mut occupied = after_one(1, vec![body(7, Tier::Dynamic)]);
    let prepare = |h: &WarmHandoff| Command::PrepareWarm {
        handoff: Box::new(h.clone()),
    };
    // The legacy free pair may transfer intact to an empty worker. In a
    // populated partition the migration must move a connected contact pair.
    assert!(matches!(
        send(&mut occupied, 2, prepare(&h)),
        Err(ProtocolError::Physics(_))
    ));
    let mut edited = h.clone();
    edited.receipt.ticket.generations[0] = 7;
    assert_eq!(
        send(&mut b, 2, prepare(&edited)),
        Err(ProtocolError::Ownership)
    );
    let mut edited = h.clone();
    edited.checkpoint.snapshot.tick = 0;
    assert_eq!(
        send(&mut b, 2, prepare(&edited)),
        Err(ProtocolError::Ownership)
    );
    let mut edited = h.clone();
    edited.checkpoint.snapshot.bodies[0].tier = Tier::Prescribed;
    assert_eq!(
        send(&mut b, 2, prepare(&edited)),
        Err(ProtocolError::Ownership)
    );
    let mut edited = h.clone();
    edited.checkpoint.snapshot.bodies[1].position[1] = -0.;
    assert_eq!(
        send(&mut b, 2, prepare(&edited)),
        Err(ProtocolError::Receipt)
    );
    // A checkpoint can arrive under a consistent transfer envelope yet fail
    // the solver's own integrity/continuity checks. Never stage it or retire
    // the source on such a refusal.
    edited = h.clone();
    edited.checkpoint.snapshot.bodies[1].mass *= 2.;
    use sha2::{Digest, Sha256};
    let mut digest = Sha256::new();
    digest.update(b"gz-physics-distributed/warm-handoff/v1\0");
    digest.update(serde_json::to_vec(&(&ticket, &edited.checkpoint)).unwrap());
    edited.receipt.digest = digest.finalize().into();
    edited.validate_digest().unwrap();
    assert!(matches!(
        send(&mut b, 2, prepare(&edited)),
        Err(ProtocolError::Physics(_))
    ));
    assert!(b.observation().snapshot.bodies.is_empty());
    assert_eq!(a.observation().snapshot, h.checkpoint.snapshot);
    let mut oversized = h.clone();
    let filler = body(17, Tier::Dynamic);
    let count =
        crate::transport::MAX_CHECKPOINT_PAYLOAD / serde_json::to_vec(&filler).unwrap().len() + 1;
    oversized.checkpoint.snapshot.bodies.resize(count, filler);
    assert_eq!(oversized.validate_digest(), Err(ProtocolError::Capacity));
    // Fill tombstones for former owners on this worker; neither incoming ID is known.
    b.retired
        .extend((10..10 + MAX_BODY_IDS as u64 - 1).map(|id| (id, 1)));
    assert_eq!(send(&mut b, 2, prepare(&h)), Err(ProtocolError::Capacity));
    b.retired.remove(&40);
    assert_eq!(
        send(&mut b, 2, prepare(&h)),
        Ok(Outcome::WarmPrepared(h.receipt.prepared()))
    );
    let r = h.receipt.clone();
    send(
        &mut a,
        3,
        Command::ReleaseWarm {
            receipt: r.prepared(),
        },
    )
    .unwrap();
    send(
        &mut b,
        3,
        Command::ActivateWarm {
            receipt: r.released(),
        },
    )
    .unwrap();
    let back = WarmTicket {
        source: 1,
        destination: 0,
        serial: 2,
        generations: vec![2, 2],
        ..ticket.clone()
    };
    let returning = export(&mut b, 4, back);
    let returned = returning.receipt.clone();
    send(&mut a, 4, prepare(&returning)).unwrap();
    send(
        &mut b,
        5,
        Command::ReleaseWarm {
            receipt: returned.prepared(),
        },
    )
    .unwrap();
    send(
        &mut a,
        5,
        Command::ActivateWarm {
            receipt: returned.released(),
        },
    )
    .unwrap();
    assert_eq!(
        a.observation().generations,
        BTreeMap::from([(1, 2), (2, 2)])
    );
    assert_eq!(send(&mut b, 6, prepare(&h)), Err(ProtocolError::Ticket));
}

#[test]
fn impulse_scope_and_pending_input_preservation_on_cold_export() {
    for seed in [
        vec![body(2, Tier::Dynamic)],
        vec![body(1, Tier::Prescribed), body(2, Tier::Dynamic)],
    ] {
        let mut w = after_one(0, seed);
        send(&mut w, 2, queue()).unwrap();
        let state = observed(send(&mut w, 3, Command::Step { tick: 1 }).unwrap());
        let dynamic = state
            .snapshot
            .bodies
            .iter()
            .find(|body| body.id == 2)
            .unwrap();
        assert!((dynamic.linear[0] - 0.275).abs() < 1.0e-12);
    }
    let mut mixed = after_one(0, vec![body(1, Tier::Prescribed), body(2, Tier::Dynamic)]);
    assert_eq!(
        send(
            &mut mixed,
            2,
            Command::QueueImpulse {
                tick: 1,
                id: 1,
                impulse: [0.1, 0., 0.]
            }
        ),
        Err(ProtocolError::Ownership)
    );
    let mut w = after_one(0, pair());
    for input in [
        Command::QueueImpulse {
            tick: 1,
            id: 7,
            impulse: [0.; 3],
        },
        Command::QueueImpulse {
            tick: 0,
            id: 2,
            impulse: [0.; 3],
        },
        Command::QueueImpulse {
            tick: 1,
            id: 2,
            impulse: [f64::INFINITY, 0., 0.],
        },
    ] {
        assert!(send(&mut w, 2, input).is_err());
    }
    send(&mut w, 2, queue()).unwrap();
    let single = Ticket {
        tick: 1,
        body_id: 2,
        promote: false,
        ..ticket(1)
    };
    assert!(matches!(
        send(&mut w, 3, Command::Export { ticket: single }),
        Err(ProtocolError::Physics(_))
    ));
    let advanced = observed(send(&mut w, 3, Command::Step { tick: 1 }).unwrap());
    assert!((advanced.snapshot.bodies[1].linear[0] - 0.275).abs() < 1.0e-12);
}

fn contacting_pair_and_bystander() -> Vec<Body> {
    let mut seed = pair();
    seed[0].position = [0., 0., 0.];
    seed[1].position = [0.65, 0., 0.];
    seed[0].linear = [8.0, 0., 0.];
    seed[1].linear = [-8.0, 0., 0.];
    seed.push(body(3, Tier::Dynamic));
    seed
}

fn after_two(id: u8, seed: Vec<Body>) -> Worker {
    let mut w = after_one(id, seed);
    send(&mut w, 2, Command::Step { tick: 1 }).unwrap();
    w
}

fn impulse(id: u64, x: f64) -> Command {
    Command::QueueImpulse {
        tick: 2,
        id,
        impulse: [x, 0., 0.],
    }
}

#[test]
fn selective_transfer_preserves_component_history_and_resident_inputs_on_both_workers() {
    let seed = contacting_pair_and_bystander();
    let far = vec![body(4, Tier::Dynamic)];
    let mut a = after_two(0, seed.clone());
    let mut b = after_two(1, far.clone());
    let mut a_control = after_two(0, seed);
    let mut b_control = after_two(1, far);
    let original = a.observation();
    assert_eq!(original.snapshot.contacts, 1);
    assert_eq!(
        original.components,
        vec![
            ComponentInfo {
                ids: vec![1, 2],
                contact_history: true
            },
            ComponentInfo {
                ids: vec![3],
                contact_history: false
            }
        ]
    );
    let resident = b.observation();
    for w in [&mut a, &mut a_control] {
        send(w, 3, impulse(1, 0.025)).unwrap();
        send(w, 4, impulse(3, -0.1)).unwrap();
    }
    for w in [&mut b, &mut b_control] {
        send(w, 3, impulse(4, 0.15)).unwrap();
    }
    let t = WarmTicket {
        tick: 2,
        ..warm_ticket()
    };
    assert!(matches!(
        send(
            &mut a,
            5,
            Command::ExportWarm {
                ticket: WarmTicket {
                    ids: vec![1, 3],
                    ..t.clone()
                }
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    let moved = export(&mut a, 5, t);
    let receipt = moved.receipt.clone();
    assert_eq!(moved.checkpoint.snapshot.bodies.len(), 2);
    assert_eq!(moved.checkpoint.pending.len(), 1);
    assert_eq!(moved.checkpoint.pending[0].id, 1);
    assert_eq!(
        send(&mut b, 4, Command::PrepareWarm { handoff: moved }).unwrap(),
        Outcome::WarmPrepared(receipt.prepared())
    );
    assert_eq!(b.observation(), resident);
    assert_eq!(a.observation(), original);
    assert_eq!(
        send(&mut b, 5, impulse(4, 0.2)),
        Err(ProtocolError::Pending)
    );
    assert_eq!(
        send(&mut a, 6, impulse(3, 0.2)),
        Err(ProtocolError::Pending)
    );
    let mut bad = receipt.released();
    bad.ticket.tick += 1;
    assert_eq!(
        send(&mut b, 5, Command::ActivateWarm { receipt: bad }),
        Err(ProtocolError::Receipt)
    );
    assert_eq!(b.observation(), resident);
    send(
        &mut a,
        6,
        Command::ReleaseWarm {
            receipt: receipt.prepared(),
        },
    )
    .unwrap();
    let left = a.observation();
    assert_eq!(
        left.components,
        vec![ComponentInfo {
            ids: vec![3],
            contact_history: false
        }]
    );
    assert!(!left.snapshot.contact_history && left.snapshot.impulses.is_empty());
    assert_eq!(left.generations, BTreeMap::from([(3, 0)]));
    assert_eq!(b.observation(), resident);
    send(
        &mut b,
        5,
        Command::ActivateWarm {
            receipt: receipt.released(),
        },
    )
    .unwrap();
    let right = b.observation();
    assert_eq!(
        right.components,
        vec![
            original.components[0].clone(),
            resident.components[0].clone()
        ]
    );
    assert_eq!(right.snapshot.impulses, original.snapshot.impulses);
    assert_eq!(right.generations, BTreeMap::from([(1, 1), (2, 1), (4, 0)]));
    for tick in 2..=3 {
        let a_next = observed(send(&mut a, tick + 5, Command::Step { tick }).unwrap());
        let b_next = observed(send(&mut b, tick + 4, Command::Step { tick }).unwrap());
        let source_control =
            observed(send(&mut a_control, tick + 3, Command::Step { tick }).unwrap());
        let dest_control =
            observed(send(&mut b_control, tick + 2, Command::Step { tick }).unwrap());
        assert_eq!(a_next.snapshot.bodies, source_control.snapshot.bodies[2..]);
        assert_eq!(
            b_next.snapshot.bodies[..2],
            source_control.snapshot.bodies[..2]
        );
        assert_eq!(b_next.snapshot.bodies[2..], dest_control.snapshot.bodies);
        assert_eq!(b_next.snapshot.impulses, source_control.snapshot.impulses);
    }
}

#[test]
fn overfull_combined_pending_inputs_refuse_prepare_without_touching_resident_queue() {
    use crate::physics::MAX_PENDING_IMPULSES;
    let mut a = after_two(0, contacting_pair_and_bystander());
    let mut b = after_two(1, vec![body(4, Tier::Dynamic)]);
    send(&mut a, 3, impulse(2, 0.125)).unwrap();
    let moved = export(
        &mut a,
        4,
        WarmTicket {
            tick: 2,
            ..warm_ticket()
        },
    );
    for i in 0..MAX_PENDING_IMPULSES {
        send(&mut b, i as u64 + 3, impulse(4, 0.125)).unwrap();
    }
    let before = b.observation();
    let seq = MAX_PENDING_IMPULSES as u64 + 3;
    assert!(matches!(
        send(&mut b, seq, Command::PrepareWarm { handoff: moved }),
        Err(ProtocolError::Physics(_))
    ));
    assert_eq!(b.observation(), before);
    let advanced = observed(send(&mut b, seq, Command::Step { tick: 2 }).unwrap());
    assert_eq!(advanced.snapshot.bodies.len(), 1);
    assert!(
        (advanced.snapshot.bodies[0].linear[0] - (0.15 + MAX_PENDING_IMPULSES as f64 * 0.125))
            .abs()
            < 1.0e-12
    );
}

#[test]
fn high_speed_arrival_bridges_independent_loaded_solvers_at_one_atomic_step() {
    let mut seed = contacting_pair_and_bystander();
    // A high-speed bystander can bridge two distinct loaded solvers.
    // Either potential deflection must stay in the common native solve.
    for mut b in contacting_pair_and_bystander().into_iter().take(2) {
        b.id += 3;
        b.position[0] += 17.3;
        seed.push(b);
    }
    let mut w = after_two(0, seed);
    let before = w.observation();
    assert_eq!(before.snapshot.contacts, 2);
    send(&mut w, 3, impulse(3, -375.)).unwrap();
    let step = Command::Step { tick: 2 };
    let advanced = observed(send(&mut w, 4, step.clone()).unwrap());
    assert_eq!(advanced.snapshot.tick, 3);
    assert_eq!(advanced.components.len(), 1);
    assert_eq!(advanced.components[0].ids, vec![1, 2, 3, 4, 5]);
    assert_eq!(observed(send(&mut w, 4, step).unwrap()), advanced);
    assert_eq!(w.observation(), advanced);
}

#[test]
fn malformed_group_tickets_are_refused_at_both_ends_before_freezing_or_staging() {
    let mut a = after_one(0, pair());
    let mut b = after_one(1, vec![]);
    let initial = a.observation();
    let empty = b.observation();
    let invalid = [
        (vec![], vec![]),
        (vec![1], vec![1]),
        (vec![1, 2], vec![1]),
        (vec![1, 2], vec![1, 1, 1]),
        (vec![1, 1], vec![1, 1]),
        (vec![1, 3, 2], vec![1, 1, 1]),
        ((1..=33).collect(), vec![1; 33]),
    ];
    for (ids, generations) in &invalid {
        let ticket = WarmTicket {
            ids: ids.clone(),
            generations: generations.clone(),
            ..warm_ticket()
        };
        assert_eq!(
            send(&mut a, 2, Command::ExportWarm { ticket }),
            Err(ProtocolError::Ticket)
        );
        assert_eq!(a.observation(), initial);
    }
    let h = export(&mut a, 2, warm_ticket());
    for (ids, generations) in invalid {
        let mut altered = h.clone();
        altered.receipt.ticket.ids = ids;
        altered.receipt.ticket.generations = generations;
        assert_eq!(
            send(&mut b, 2, Command::PrepareWarm { handoff: altered }),
            Err(ProtocolError::Ticket)
        );
        assert_eq!(b.observation(), empty);
    }
    assert_eq!(
        send(&mut b, 2, Command::PrepareWarm { handoff: h.clone() }).unwrap(),
        Outcome::WarmPrepared(h.receipt.prepared())
    );
}

#[test]
fn connected_three_member_handoff_is_atomic_and_preserves_all_contact_and_input_state() {
    let mut members = vec![
        body(1, Tier::Dynamic),
        body(2, Tier::Dynamic),
        body(3, Tier::Dynamic),
    ];
    for ((b, x), v) in members.iter_mut().zip([-0.65, 0., 0.65]).zip([8., 0., -8.]) {
        b.position = [x, 0., 0.];
        b.linear = [v, 0., 0.];
    }
    let bystander = body(4, Tier::Dynamic);
    let mut seed = members;
    seed.push(bystander);
    let mut a = after_two(0, seed.clone());
    let mut control = after_two(0, seed);
    let mut b = after_two(1, vec![body(5, Tier::Dynamic)]);
    let before = a.observation();
    let original_dest = b.observation();
    assert_eq!(
        before
            .snapshot
            .impulses
            .iter()
            .map(|p| (p.a, p.b))
            .collect::<Vec<_>>(),
        vec![(1, 2), (2, 3)]
    );
    for w in [&mut a, &mut control] {
        send(w, 3, impulse(3, -0.25)).unwrap();
        send(w, 4, impulse(1, 0.05)).unwrap();
        send(w, 5, impulse(4, 0.125)).unwrap();
    }
    let t = WarmTicket {
        tick: 2,
        ids: vec![1, 2, 3],
        generations: vec![1; 3],
        ..warm_ticket()
    };
    let h = export(&mut a, 6, t);
    let r = h.receipt.clone();
    assert_eq!(
        h.checkpoint
            .pending
            .iter()
            .map(|p| p.id)
            .collect::<Vec<_>>(),
        vec![3, 1]
    );
    send(&mut b, 3, Command::PrepareWarm { handoff: h }).unwrap();
    assert_eq!(a.observation(), before);
    assert_eq!(b.observation(), original_dest);
    let mut invalid = r.prepared();
    invalid.ticket.generations[2] += 1;
    assert_eq!(
        send(&mut a, 7, Command::ReleaseWarm { receipt: invalid }),
        Err(ProtocolError::Receipt)
    );
    assert_eq!(a.observation(), before);
    send(
        &mut a,
        7,
        Command::ReleaseWarm {
            receipt: r.prepared(),
        },
    )
    .unwrap();
    assert_eq!(a.observation().generations, BTreeMap::from([(4, 0)]));
    assert_eq!(b.observation(), original_dest);
    let mut bad = r.released();
    bad.ticket.ids.pop();
    assert_eq!(
        send(&mut b, 4, Command::ActivateWarm { receipt: bad }),
        Err(ProtocolError::Receipt)
    );
    assert_eq!(b.observation(), original_dest);
    let activate = Command::ActivateWarm {
        receipt: r.released(),
    };
    let ack = send(&mut b, 4, activate.clone()).unwrap();
    assert_eq!(send(&mut b, 4, activate).unwrap(), ack);
    assert_eq!(
        b.observation().generations,
        BTreeMap::from([(1, 1), (2, 1), (3, 1), (5, 0)])
    );
    for tick in 2..=4 {
        let reference = observed(send(&mut control, tick + 4, Command::Step { tick }).unwrap());
        let source = observed(send(&mut a, tick + 6, Command::Step { tick }).unwrap());
        let dest = observed(send(&mut b, tick + 3, Command::Step { tick }).unwrap());
        assert_eq!(dest.snapshot.bodies[..3], reference.snapshot.bodies[..3]);
        assert_eq!(source.snapshot.bodies, reference.snapshot.bodies[3..]);
        assert_eq!(dest.snapshot.impulses, reference.snapshot.impulses);
    }
}

#[test]
fn two_member_ticket_retains_its_wire_representation() {
    let old = serde_json::json!({"run_id":RUN,"serial":1,"source":0,"destination":1,"tick":1,"ids":[1,2],"generations":[1,1]});
    assert_eq!(
        serde_json::from_value::<WarmTicket>(old.clone()).unwrap(),
        warm_ticket()
    );
    assert_eq!(serde_json::to_value(warm_ticket()).unwrap(), old);
}

#[test]
fn whole_solver_receipts_cannot_authorize_a_different_transfer_profile() {
    let mut a = after_one(0, pair());
    let mut b = after_one(1, vec![]);
    let h = export(&mut a, 2, warm_ticket());
    assert_eq!(
        send(&mut b, 2, Command::PrepareSolver { handoff: h.clone() }),
        Err(ProtocolError::Receipt)
    );
    let mut recipient = after_one(1, vec![]);
    let mut donor = after_one(0, pair());
    // A single solver is required. Far-apart cold seeds may be independent.
    let ids = donor.observation().components[0].ids.clone();
    let ticket = WarmTicket {
        generations: vec![1; ids.len()],
        ids,
        ..warm_ticket()
    };
    let Outcome::WarmExported(h) = send(&mut donor, 2, Command::ExportSolver { ticket }).unwrap()
    else {
        panic!("solver image")
    };
    assert!(
        send(
            &mut recipient,
            2,
            Command::PrepareWarm { handoff: h.clone() }
        )
        .is_err()
    );
    assert!(matches!(
        send(&mut recipient, 2, Command::PrepareSolver { handoff: h }),
        Ok(Outcome::WarmPrepared(_))
    ));
}

#[test]
fn invalid_input_interleavings_never_stage_a_receipt_or_consume_either_fifo() {
    let mut src = after_two(0, contacting_pair_and_bystander());
    let mut dst = after_two(1, vec![body(4, Tier::Dynamic)]);
    send(&mut src, 3, impulse(1, 0.125)).unwrap();
    send(&mut dst, 3, impulse(4, 0.25)).unwrap();
    let before = dst.observation();
    let h = export(
        &mut src,
        4,
        WarmTicket {
            tick: 2,
            ..warm_ticket()
        },
    );
    for (mask, complete) in [(0, false), (3, false), (4, false), (1, true)] {
        assert!(
            send(
                &mut dst,
                4,
                Command::PrepareOrdered {
                    handoff: h.clone(),
                    complete,
                    incoming_slots: mask
                }
            )
            .is_err()
        );
        assert_eq!(dst.observation(), before);
    }
    let prepare = Command::PrepareOrdered {
        handoff: h.clone(),
        complete: false,
        incoming_slots: 1,
    };
    assert_eq!(
        send(&mut dst, 4, prepare).unwrap(),
        Outcome::WarmPrepared(h.receipt.prepared())
    );
    assert_eq!(
        send(
            &mut dst,
            4,
            Command::PrepareOrdered {
                handoff: h,
                complete: false,
                incoming_slots: 2
            }
        ),
        Err(ProtocolError::ConflictingRetry)
    );
}
