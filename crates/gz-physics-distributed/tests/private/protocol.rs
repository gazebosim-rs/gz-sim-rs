// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::Tier;

const RUN: u64 = 87;
fn body(id: u64, tier: Tier) -> Body {
    Body {
        primitive: Default::default(),
            friction: 0.,
        id,
        tier,
        size: [0.6; 3],
        mass: 1.0,
        position: [id as f64 * 3., 0., 0.],
        orientation: [0., 0., 0., 1.],
        linear: [0.15, 0., 0.],
        angular: [0.; 3],
    }
}
fn snapshot(bodies: Vec<Body>) -> Snapshot {
    Snapshot {
        step: Default::default(),
        ground: None,
        supports: Vec::new(),
        tick: 0,
        bodies,
        contacts: 0,
        contact_history: false,
        impulses: vec![],
    }
}
fn worker(id: u8, bodies: Vec<Body>) -> Worker {
    Worker::new(RUN, id, snapshot(bodies)).unwrap()
}
fn request(worker: &Worker, seq: u64, command: Command) -> Request {
    Request {
        run_id: RUN,
        worker_id: worker.worker_id,
        sequence: seq,
        command,
    }
}
fn send(worker: &mut Worker, seq: u64, command: Command) -> Result<Outcome, ProtocolError> {
    worker
        .handle(request(worker, seq, command))
        .map(|reply| reply.outcome)
}
fn ticket(generation: u64) -> Ticket {
    Ticket {
        run_id: RUN,
        serial: generation,
        source: 0,
        destination: 1,
        body_id: 1,
        generation,
        tick: 0,
        promote: true,
    }
}
fn observed(outcome: Outcome) -> Observation {
    match outcome {
        Outcome::State(s) | Outcome::Activated(s) => s,
        _ => panic!("expected committed observation"),
    }
}

#[test]
fn addressed_contiguous_requests_are_idempotent_and_refusal_does_not_consume_sequence() {
    let mut worker = worker(0, vec![body(1, Tier::Dynamic)]);
    assert!(matches!(
        Worker::new(RUN, 2, snapshot(vec![])),
        Err(ProtocolError::Address)
    ));
    let first = request(&worker, 1, Command::Step { tick: 0 });
    for altered in [
        Request {
            run_id: RUN + 1,
            ..first.clone()
        },
        Request {
            worker_id: 1,
            ..first.clone()
        },
    ] {
        assert_eq!(worker.handle(altered), Err(ProtocolError::Address));
    }
    assert_eq!(
        send(&mut worker, 0, Command::Observe),
        Err(ProtocolError::Sequence {
            expected: 1,
            received: 0
        })
    );
    assert_eq!(
        send(&mut worker, 2, Command::Observe),
        Err(ProtocolError::Sequence {
            expected: 1,
            received: 2
        })
    );
    assert_eq!(
        send(&mut worker, 1, Command::Step { tick: 5 }),
        Err(ProtocolError::Tick {
            expected: 0,
            received: 5
        })
    );
    let reply = worker.handle(first.clone()).unwrap();
    assert_eq!((reply.run_id, reply.worker_id, reply.sequence), (RUN, 0, 1));
    assert_eq!(worker.handle(first).unwrap(), reply);
    assert_eq!(worker.tick, 1);
    assert_eq!(
        send(&mut worker, 1, Command::Observe),
        Err(ProtocolError::ConflictingRetry)
    );
    assert_eq!(
        send(&mut worker, 2, Command::Step { tick: 0 }),
        Err(ProtocolError::Tick {
            expected: 1,
            received: 0
        })
    );
    let state = observed(send(&mut worker, 2, Command::Observe).unwrap());
    assert_eq!(state.snapshot.tick, 1);
    assert_eq!(state.generations, BTreeMap::from([(1, 0)]));
    assert_eq!(
        send(&mut worker, 1, Command::Step { tick: 0 }),
        Err(ProtocolError::Sequence {
            expected: 3,
            received: 1
        })
    );
}

#[test]
fn transfer_freezes_both_islands_hides_staging_and_binds_every_receipt_to_source_bits() {
    let source_body = body(1, Tier::Prescribed);
    let mut a = worker(0, vec![source_body.clone()]);
    let mut b = worker(1, vec![body(2, Tier::Dynamic)]);
    let t = ticket(1);
    let Outcome::Exported(h) = send(&mut a, 1, Command::Export { ticket: t }).unwrap() else {
        panic!("export")
    };
    assert_eq!(h.body, source_body);
    assert_eq!(
        send(&mut a, 1, Command::Export { ticket: t }).unwrap(),
        Outcome::Exported(h.clone())
    );
    assert_eq!(
        send(&mut a, 2, Command::Step { tick: 0 }),
        Err(ProtocolError::Pending)
    );
    let owned = observed(send(&mut a, 2, Command::Observe).unwrap());
    assert_eq!(owned.snapshot.bodies, vec![source_body.clone()]);
    assert_eq!(owned.generations[&1], 0);

    let premature = Released { handoff: h.clone() };
    assert_eq!(
        send(
            &mut b,
            1,
            Command::Activate {
                released: premature
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let Outcome::Prepared(prepared) =
        send(&mut b, 1, Command::Prepare { handoff: h.clone() }).unwrap()
    else {
        panic!("prepare")
    };
    let mut altered = h.clone();
    altered.body.angular[1] = -0.0;
    assert_eq!(
        send(
            &mut b,
            1,
            Command::Prepare {
                handoff: altered.clone()
            }
        ),
        Err(ProtocolError::ConflictingRetry)
    );
    assert_eq!(
        send(&mut b, 1, Command::Prepare { handoff: h.clone() }).unwrap(),
        Outcome::Prepared(prepared.clone())
    );
    assert_eq!(
        send(&mut b, 2, Command::Step { tick: 0 }),
        Err(ProtocolError::Pending)
    );
    let staged = observed(send(&mut b, 2, Command::Observe).unwrap());
    assert_eq!(staged.snapshot.bodies.len(), 1);
    assert_eq!(staged.snapshot.bodies[0].id, 2);
    assert_eq!(staged.generations, BTreeMap::from([(2, 0)]));
    assert_eq!(
        send(
            &mut a,
            3,
            Command::Release {
                prepared: Prepared {
                    handoff: altered.clone()
                }
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let mut wrong = t;
    wrong.promote = false;
    assert_eq!(
        send(
            &mut a,
            3,
            Command::Release {
                prepared: Prepared {
                    handoff: Handoff {
                        ticket: wrong,
                        body: source_body
                    }
                }
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let Outcome::Released(released) = send(
        &mut a,
        3,
        Command::Release {
            prepared: prepared.clone(),
        },
    )
    .unwrap() else {
        panic!("release")
    };
    assert_eq!(
        send(&mut a, 3, Command::Release { prepared }).unwrap(),
        Outcome::Released(released.clone())
    );
    let retired = observed(send(&mut a, 4, Command::Observe).unwrap());
    assert!(retired.generations.is_empty() && retired.snapshot.bodies.is_empty());
    assert_eq!(
        send(
            &mut b,
            3,
            Command::Activate {
                released: Released { handoff: altered }
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let committed = observed(
        send(
            &mut b,
            3,
            Command::Activate {
                released: released.clone(),
            },
        )
        .unwrap(),
    );
    assert_eq!(committed.generations, BTreeMap::from([(1, 1), (2, 0)]));
    assert_eq!(committed.snapshot.bodies[0], h.body.promoted());
    assert_eq!(
        observed(send(&mut b, 3, Command::Activate { released }).unwrap()),
        committed
    );

    // The peer returning this body cannot reuse an origin generation or skip ahead.
    let back = Ticket {
        source: 1,
        destination: 0,
        promote: false,
        ..ticket(2)
    };
    for generation in [1, 3] {
        assert_eq!(
            send(
                &mut b,
                4,
                Command::Export {
                    ticket: Ticket { generation, ..back }
                }
            ),
            Err(ProtocolError::Ownership)
        );
        let stale = Handoff {
            ticket: Ticket { generation, ..back },
            body: h.body.promoted(),
        };
        assert_eq!(
            send(&mut a, 5, Command::Prepare { handoff: stale }),
            Err(ProtocolError::Ownership)
        );
    }
    let Outcome::Exported(returned) = send(&mut b, 4, Command::Export { ticket: back }).unwrap()
    else {
        panic!("return export")
    };
    let Outcome::Prepared(proof) = send(&mut a, 5, Command::Prepare { handoff: returned }).unwrap()
    else {
        panic!("return prepare")
    };
    let Outcome::Released(proof) = send(&mut b, 5, Command::Release { prepared: proof }).unwrap()
    else {
        panic!("return release")
    };
    let restored = observed(send(&mut a, 6, Command::Activate { released: proof }).unwrap());
    assert_eq!(restored.generations, BTreeMap::from([(1, 2)]));
    assert_eq!(restored.snapshot.bodies[0].tier, Tier::Dynamic);
    assert_eq!(
        observed(send(&mut a, 7, Command::Step { tick: 0 }).unwrap())
            .snapshot
            .tick,
        1
    );
    assert_eq!(
        observed(send(&mut b, 6, Command::Step { tick: 0 }).unwrap())
            .snapshot
            .tick,
        1
    );
}

#[test]
fn malformed_destination_import_or_stale_barrier_leaves_ownership_and_ticket_reusable() {
    let mut a = worker(0, vec![body(1, Tier::Dynamic)]);
    let mut b = worker(1, vec![body(2, Tier::Dynamic)]);
    let original = b.observation();
    let t = ticket(1);
    for invalid in [
        Ticket { run_id: 0, ..t },
        Ticket { source: 1, ..t },
        Ticket {
            destination: 2,
            ..t
        },
    ] {
        assert_eq!(
            send(&mut a, 1, Command::Export { ticket: invalid }),
            Err(ProtocolError::Address)
        );
    }
    assert_eq!(
        send(
            &mut a,
            1,
            Command::Export {
                ticket: Ticket { serial: 0, ..t }
            }
        ),
        Err(ProtocolError::Ticket)
    );
    assert_eq!(
        send(
            &mut a,
            1,
            Command::Export {
                ticket: Ticket { body_id: 3, ..t }
            }
        ),
        Err(ProtocolError::Ownership)
    );
    assert_eq!(
        send(
            &mut b,
            1,
            Command::Release {
                prepared: Prepared {
                    handoff: Handoff {
                        ticket: t,
                        body: body(1, Tier::Dynamic)
                    }
                }
            }
        ),
        Err(ProtocolError::Receipt)
    );
    let Outcome::Exported(h) = send(&mut a, 1, Command::Export { ticket: t }).unwrap() else {
        panic!("export")
    };
    assert_eq!(
        send(&mut a, 2, Command::Export { ticket: t }),
        Err(ProtocolError::Pending)
    );
    let mut invalid = h.clone();
    invalid.body.mass = -1.0;
    assert!(matches!(
        send(&mut b, 1, Command::Prepare { handoff: invalid }),
        Err(ProtocolError::Physics(_))
    ));
    let mut stale = h.clone();
    stale.ticket.tick = 1;
    assert_eq!(
        send(&mut b, 1, Command::Prepare { handoff: stale }),
        Err(ProtocolError::Tick {
            expected: 0,
            received: 1
        })
    );
    let wrong = Handoff {
        ticket: t,
        body: body(2, Tier::Dynamic),
    };
    assert_eq!(
        send(&mut b, 1, Command::Prepare { handoff: wrong }),
        Err(ProtocolError::Ownership)
    );
    assert_eq!(b.observation(), original);
    let wire = serde_json::to_string(&request(&b, 1, Command::Prepare { handoff: h })).unwrap();
    let decoded: Request = serde_json::from_str(&wire).unwrap();
    assert!(matches!(
        b.handle(decoded).unwrap().outcome,
        Outcome::Prepared(_)
    ));
}

#[test]
fn native_terminal_step_fault_never_advances_the_protocol_or_reuses_its_reply_cache() {
    let mut w = worker(0, vec![body(1, Tier::Prescribed)]);
    let before = observed(send(&mut w, 1, Command::Observe).unwrap());
    // A native Bevy commit-phase fault is terminal even when older replies
    // are cached. Recovery starts from the seed and acknowledged log.
    w.partition.inject_commit_fault();
    let Err(fault @ ProtocolError::Terminal(_)) = send(&mut w, 2, Command::Step { tick: 0 }) else {
        panic!("expected native terminal fault")
    };
    assert_eq!(w.tick, 0);
    assert_eq!(w.partition.snapshot(), before.snapshot);
    for (seq, command) in [
        (1, Command::Observe),
        (2, Command::Step { tick: 0 }),
        (2, Command::Export { ticket: ticket(1) }),
    ] {
        assert_eq!(send(&mut w, seq, command), Err(fault.clone()));
    }
}

#[test]
fn an_exhausted_public_clock_refuses_before_integrating_or_consuming_sequence() {
    let initial = Snapshot {
        tick: u64::MAX,
        ..snapshot(vec![body(1, Tier::Dynamic)])
    };
    let mut w = Worker::new(RUN, 0, initial.clone()).unwrap();
    assert_eq!(
        send(&mut w, 1, Command::Step { tick: u64::MAX }),
        Err(ProtocolError::Exhausted)
    );
    assert_eq!(
        observed(send(&mut w, 1, Command::Observe).unwrap()).snapshot,
        initial
    );
}

#[test]
fn distinct_id_capacity_bounds_seed_import_and_retired_ownership_history() {
    let full: Vec<_> = (1..=MAX_BODY_IDS as u64)
        .map(|i| body(i, Tier::Dynamic))
        .collect();
    let mut overflow = full.clone();
    overflow.push(body(MAX_BODY_IDS as u64 + 1, Tier::Dynamic));
    assert!(matches!(
        Worker::new(RUN, 1, snapshot(overflow)),
        Err(ProtocolError::Capacity)
    ));
    let mut at_limit = worker(1, full);
    let inbound = Handoff {
        ticket: Ticket {
            body_id: MAX_BODY_IDS as u64 + 1,
            ..ticket(1)
        },
        body: body(MAX_BODY_IDS as u64 + 1, Tier::Dynamic),
    };
    let initial = at_limit.observation();
    assert_eq!(
        send(
            &mut at_limit,
            1,
            Command::Prepare {
                handoff: inbound.clone()
            }
        ),
        Err(ProtocolError::Capacity)
    );
    assert_eq!(at_limit.observation(), initial);
    let outgoing = Ticket {
        source: 1,
        destination: 0,
        promote: false,
        ..ticket(1)
    };
    let Outcome::Exported(h) =
        send(&mut at_limit, 1, Command::Export { ticket: outgoing }).unwrap()
    else {
        panic!()
    };
    send(
        &mut at_limit,
        2,
        Command::Release {
            prepared: Prepared { handoff: h.clone() },
        },
    )
    .unwrap();
    let new_id = Handoff {
        ticket: Ticket {
            serial: 2,
            ..inbound.ticket
        },
        ..inbound
    };
    assert_eq!(
        send(&mut at_limit, 3, Command::Prepare { handoff: new_id }),
        Err(ProtocolError::Capacity)
    );
    let back = Handoff {
        ticket: Ticket {
            promote: false,
            ..ticket(2)
        },
        body: h.body,
    };
    assert!(matches!(
        send(&mut at_limit, 3, Command::Prepare { handoff: back }),
        Ok(Outcome::Prepared(_))
    ));
}

#[path = "warm_protocol.rs"]
mod warm_protocol;

#[test]
fn high_index_pool_worker_compares_pending_impulse_retries_and_destination_fences_bitwise() {
    let mut w = Worker::with_width(RUN, 5, snapshot(vec![body(7, Tier::Dynamic)]), 8).unwrap();
    let queue = Command::QueueImpulse {
        tick: 0,
        id: 7,
        impulse: [0.3, 0., -0.],
    };
    let success = send(&mut w, 1, queue.clone()).unwrap();
    assert_eq!(send(&mut w, 1, queue), Ok(success));
    assert_eq!(
        send(
            &mut w,
            1,
            Command::QueueImpulse {
                tick: 0,
                id: 7,
                impulse: [0.3, -0., -0.]
            }
        ),
        Err(ProtocolError::ConflictingRetry)
    );
    let left = w.recovery_image(1, 0).unwrap();
    assert_eq!(left.pending().len(), 1);
    assert_eq!(
        left.pending()[0].impulse.map(f64::to_bits),
        [0.3, 0., -0.].map(f64::to_bits)
    );
    w.retired.insert(18, 6);
    assert!(!w.accepts_generation(18, 6));
    assert!(!w.accepts_generation(18, 5));
    assert!(w.accepts_generation(18, 9));
    let t = Ticket {
        run_id: RUN,
        serial: 1,
        source: 5,
        destination: 8,
        body_id: 7,
        generation: 1,
        tick: 0,
        promote: false,
    };
    assert_eq!(
        send(&mut w, 2, Command::Export { ticket: t }),
        Err(ProtocolError::Address)
    );
}

#[test]
fn retrying_an_accepted_pool_field_checks_force_and_torque_bits_and_never_advances_twice() {
    use crate::physics::{BodyWrench, StepField};
    let mut w = Worker::with_width(RUN, 7, snapshot(vec![body(4, Tier::Dynamic)]), 8).unwrap();
    let field = StepField {
        gravity: [0., -0., -2.],
        wrenches: vec![BodyWrench {
            id: 4,
            force: [-0., 0.25, 0.],
            torque: [-0.15, 0., 0.],
        }],
    };
    let command = Command::StepInField {
        tick: 0,
        field: field.clone(),
    };
    let ack = send(&mut w, 1, command.clone()).unwrap();
    assert_eq!(send(&mut w, 1, command), Ok(ack));
    for changed in 0..3 {
        let mut value = field.clone();
        match changed {
            0 => value.gravity[1] = 0.,
            1 => value.wrenches[0].force[0] = 0.,
            _ => value.wrenches[0].torque[1] = -0.,
        }
        assert_eq!(
            send(
                &mut w,
                1,
                Command::StepInField {
                    tick: 0,
                    field: value
                }
            ),
            Err(ProtocolError::ConflictingRetry)
        );
    }
    assert_eq!(w.observation().snapshot.tick, 1);
}

#[test]
fn native_impulse_preview_is_read_only_and_requires_the_current_healthy_unfrozen_epoch() {
    let mut w = Worker::with_width(RUN, 4, snapshot(vec![body(1, Tier::Dynamic)]), 6).unwrap();
    let ack = send(&mut w, 1, Command::Observe).unwrap();
    let j = SpatialImpulse {
        id: 1,
        linear: [0.5, -0.1, 0.2],
        angular: [0.023, -0.032, 0.015],
    };
    assert!(matches!(
        w.preview_impulse(0, 0, &j),
        Err(ProtocolError::Sequence { .. })
    ));
    assert!(matches!(
        w.preview_impulse(1, 1, &j),
        Err(ProtocolError::Tick { .. })
    ));
    let before = w.recovery_image(1, 0).unwrap();
    let preview = w.preview_impulse(1, 0, &j).unwrap();
    assert_eq!(preview, w.preview_impulse(1, 0, &j).unwrap());
    assert_eq!(before, w.recovery_image(1, 0).unwrap());
    assert_eq!(send(&mut w, 1, Command::Observe).unwrap(), ack);
    let Outcome::State(edited) = send(
        &mut w,
        2,
        Command::ImpulseNow {
            tick: 0,
            impulse: j.clone(),
        },
    )
    .unwrap() else {
        panic!("native impulse state")
    };
    assert_eq!(
        edited.snapshot.bodies[0].linear.map(f64::to_bits),
        preview.linear.map(f64::to_bits)
    );
    assert_eq!(
        edited.snapshot.bodies[0].angular.map(f64::to_bits),
        preview.angular.map(f64::to_bits)
    );
    let t = WarmTicket {
        run_id: RUN,
        serial: 1,
        source: 4,
        destination: 3,
        ids: vec![1],
        generations: vec![1],
        tick: 0,
    };
    send(&mut w, 3, Command::ExportSolver { ticket: t }).unwrap();
    assert_eq!(w.preview_impulse(3, 0, &j), Err(ProtocolError::Pending));
    w.terminal = Some("native fault".into());
    assert!(matches!(
        w.preview_impulse(3, 0, &j),
        Err(ProtocolError::Terminal(_))
    ));
}

#[test]
fn frozen_and_staged_native_owners_refuse_all_body_edits_before_consuming_a_sequence() {
    use crate::physics::StepField;
    let b = body(3, Tier::Dynamic);
    let mut s = worker(0, vec![b.clone()]);
    let mut d = worker(1, vec![]);
    let t = WarmTicket {
        run_id: RUN,
        serial: 1,
        source: 0,
        destination: 1,
        tick: 0,
        ids: vec![3],
        generations: vec![1],
    };
    let Outcome::WarmExported(h) = send(&mut s, 1, Command::ExportSolver { ticket: t }).unwrap()
    else {
        panic!("native export")
    };
    let receipt = h.receipt.clone();
    send(&mut d, 1, Command::PrepareSolver { handoff: h }).unwrap();
    for peer in [&mut s, &mut d] {
        let old = peer.observation();
        let owner = peer.worker_id;
        for tick in [0, 1] {
            let edits = [
                Command::InsertBody {
                    tick,
                    body: body(5, Tier::Dynamic),
                },
                Command::ReplaceBody {
                    tick,
                    body: b.clone(),
                },
                Command::SetVelocity {
                    tick,
                    edit: VelocityEdit {
                        id: 3,
                        linear: [1.; 3],
                        angular: [0.; 3],
                    },
                },
                Command::ImpulseNow {
                    tick,
                    impulse: SpatialImpulse {
                        id: 3,
                        linear: [0.; 3],
                        angular: [1.; 3],
                    },
                },
                Command::RetireBody {
                    tick,
                    id: 3,
                    owner,
                    generation: 0,
                },
                Command::SplitComponent { tick, member: 3 },
                Command::QueueImpulse {
                    tick,
                    id: 3,
                    impulse: [0.3, 0., 0.],
                },
                Command::StepWithWrenches {
                    tick,
                    wrenches: vec![],
                },
                Command::StepInField {
                    tick,
                    field: StepField {
                        gravity: [0., 0., -7.1],
                        wrenches: vec![],
                    },
                },
            ];
            for edit in edits {
                let result = send(peer, 2, edit);
                if tick == 0 {
                    assert_eq!(result, Err(ProtocolError::Pending));
                } else {
                    assert!(matches!(result, Err(ProtocolError::Tick { .. })));
                }
                assert_eq!(peer.observation(), old);
            }
        }
        assert!(matches!(
            peer.start_probe(0, 4),
            Err(ProtocolError::Pending)
        ));
        assert!(peer.read_probe(0, 4).is_none());
    }
    send(
        &mut s,
        2,
        Command::ReleaseWarm {
            receipt: receipt.prepared(),
        },
    )
    .unwrap();
    let Outcome::WarmActivated(active) = send(
        &mut d,
        2,
        Command::ActivateWarm {
            receipt: receipt.released(),
        },
    )
    .unwrap() else {
        panic!("native activation")
    };
    assert_eq!(active.snapshot.bodies, vec![b]);
    assert_eq!(active.generations[&3], 1);
}

#[test]
fn invalid_geometry_and_ownership_commands_cannot_reuse_a_retired_body_id() {
    let b = body(3, Tier::Dynamic);
    let mut s = worker(0, vec![b.clone()]);
    let mut d = worker(1, vec![]);
    let old = s.observation();
    let mut overlap = body(4, Tier::Dynamic);
    overlap.position = b.position;
    assert!(matches!(
        send(
            &mut s,
            1,
            Command::InsertBody {
                tick: 0,
                body: overlap
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    assert_eq!(
        send(
            &mut s,
            1,
            Command::InsertBody {
                tick: 0,
                body: b.clone()
            }
        ),
        Err(ProtocolError::Ownership)
    );
    for (owner, generation) in [(0, 1), (1, 0), (7, 0)] {
        assert!(
            send(
                &mut s,
                1,
                Command::RetireBody {
                    tick: 0,
                    id: 3,
                    owner,
                    generation
                }
            )
            .is_err()
        );
    }
    let mut invalid = b.clone();
    invalid.mass *= 2.;
    assert!(matches!(
        send(
            &mut s,
            1,
            Command::ReplaceBody {
                tick: 0,
                body: invalid
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    assert!(matches!(
        send(
            &mut s,
            1,
            Command::SetVelocity {
                tick: 0,
                edit: VelocityEdit {
                    id: 13,
                    linear: [0.; 3],
                    angular: [0.; 3]
                }
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    assert!(matches!(
        send(
            &mut s,
            1,
            Command::ImpulseNow {
                tick: 0,
                impulse: SpatialImpulse {
                    id: 13,
                    linear: [0.; 3],
                    angular: [0.; 3]
                }
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    assert!(matches!(
        send(
            &mut s,
            1,
            Command::SplitComponent {
                tick: 0,
                member: 13
            }
        ),
        Err(ProtocolError::Physics(_))
    ));
    assert_eq!(s.observation(), old);
    let t = WarmTicket {
        run_id: RUN,
        serial: 1,
        source: 0,
        destination: 1,
        tick: 0,
        ids: vec![3],
        generations: vec![1],
    };
    let Outcome::WarmExported(h) = send(&mut s, 1, Command::ExportSolver { ticket: t }).unwrap()
    else {
        panic!("native export")
    };
    let r = h.receipt.clone();
    send(&mut d, 1, Command::PrepareSolver { handoff: h }).unwrap();
    send(
        &mut s,
        2,
        Command::ReleaseWarm {
            receipt: r.prepared(),
        },
    )
    .unwrap();
    send(
        &mut d,
        2,
        Command::ActivateWarm {
            receipt: r.released(),
        },
    )
    .unwrap();
    assert_eq!(
        send(&mut s, 3, Command::InsertBody { tick: 0, body: b }),
        Err(ProtocolError::Ownership)
    );
    assert!(s.observation().generations.is_empty());
    assert_eq!(d.observation().generations[&3], 1);
}

#[test]
fn pooled_worker_rechecks_native_edit_eligibility_without_consuming_a_sequence() {
    let mut a = body(1, Tier::Dynamic);
    let mut b = body(2, Tier::Dynamic);
    b.position[0] = a.position[0] + 0.65;
    a.linear = [5., 0., 0.];
    b.linear = [-5., 0., 0.];
    let mut peer = Worker::with_width(RUN, 2, snapshot(vec![a, b]), 4).unwrap();
    let mut tick = 0;
    while tick < 8 && peer.observation().snapshot.impulses.is_empty() {
        send(&mut peer, tick + 1, Command::Step { tick }).unwrap();
        tick += 1;
    }
    let committed = peer.observation();
    assert!(!committed.snapshot.impulses.is_empty());
    let mut changed = committed.snapshot.bodies[0].clone();
    changed.position[2] += 3.;
    for command in [
        Command::ReplaceBody { tick, body: changed },
        Command::SetVelocity {
            tick, edit: VelocityEdit { id: 2, linear: [0.; 3], angular: [0.; 3] },
        },
    ] {
        assert!(matches!(send(&mut peer, tick + 1, command), Err(ProtocolError::Physics(_))));
        assert_eq!(peer.observation(), committed);
    }
    assert_eq!(observed(send(&mut peer, tick + 1, Command::Observe).unwrap()), committed);
}
