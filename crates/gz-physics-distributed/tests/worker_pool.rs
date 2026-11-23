// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/worker_pool.rs"]
mod demo;
use gz_physics_distributed::{
    physics::{Body, SimulationStep, fixture},
    pool::WorkerPool,
};
use std::path::Path;
fn executable() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
#[test]
fn four_process_native_contacts_survive_multi_hop_generation_and_failed_partial_barrier() {
    let metrics = demo::qualify(executable()).unwrap();
    println!("{metrics}");
}
#[test]
fn pool_width_and_global_cold_shape_separation_are_checked_before_startup() {
    let b = fixture()[0].clone();
    let args = |seeds: Vec<Vec<Body>>| {
        WorkerPool::spawn(
            Path::new("/unused"),
            8,
            seeds,
            SimulationStep::Ms1,
            None,
            32,
        )
    };
    for width in [0, 2, 9] {
        assert!(matches!(
            args(vec![vec![]; width]),
            Err(gz_physics_distributed::coordinator::Error::Invalid(_))
        ));
    }
    assert!(matches!(
        args(vec![vec![b.clone()], vec![], vec![b.clone()]]),
        Err(gz_physics_distributed::coordinator::Error::Invalid(
            "duplicate pool seed ID"
        ))
    ));
    let mut other = b.clone();
    other.id += 100;
    assert!(matches!(
        args(vec![vec![b], vec![], vec![other]]),
        Err(gz_physics_distributed::coordinator::Error::Invalid(
            "pool seeds intersect across worker boundaries"
        ))
    ));
}

#[test]
fn eight_peers_keep_separated_bodies_on_their_owners_and_apply_the_same_gravity() {
    let mut base = fixture()[1].clone();
    base.linear = [0.; 3];
    base.angular = [0.; 3];
    let seeds = (0..8)
        .map(|i| {
            let mut b = base.clone();
            b.id = 100 + i;
            b.position = [i as f64 * 20., 0., 1.2];
            vec![b]
        })
        .collect();
    let mut p =
        WorkerPool::spawn(executable(), 71476, seeds, SimulationStep::Ms10, None, 4).unwrap();
    p.set_packed_observations(false);
    let original = p.frame().clone();
    assert!(p.validate_next([f64::NAN, 0., 0.]).is_err());
    assert!(p.advance_in_gravity([0., f64::INFINITY, 0.]).is_err());
    assert_eq!(p.frame(), &original);
    assert!(p.recover(8).is_err() && p.worker_pid(8).is_none());
    for tick in 1..=18 {
        if tick == 10 {
            p.set_packed_observations(true);
        }
        p.advance_in_gravity([0., 0., -4.5]).unwrap();
        let mut first = None;
        for w in 0..8 {
            let (owner, b) = p.frame().body(100 + w).unwrap();
            assert_eq!(owner as u64, w);
            assert_eq!(p.frame().workers[w as usize].generations[&b.id], 0);
            assert!((b.linear[2] + 0.045 * tick as f64).abs() < 1e-13);
            let state = (b.position[2].to_bits(), b.linear[2].to_bits());
            assert_eq!(*first.get_or_insert(state), state);
        }
        assert!(p.retained_entries().iter().all(|&n| n <= 4));
    }
}

#[test]
fn live_frozen_source_and_missing_destination_recover_to_the_same_committed_frame() {
    let mut b = fixture()[1].clone();
    b.linear = [0.; 3];
    let mut p = WorkerPool::spawn(
        executable(),
        71477,
        vec![vec![b.clone()], vec![], vec![]],
        SimulationStep::Ms5,
        None,
        16,
    )
    .unwrap();
    let old = p.frame().clone();
    let from_pid = p.worker_pid(0).unwrap();
    p.terminate_worker(2).unwrap();
    assert!(p.transfer_component(b.id, 2).is_err());
    assert_eq!(p.frame(), &old);
    p.transfer_component(b.id, 2).unwrap();
    assert_ne!(from_pid, p.worker_pid(0).unwrap());
    assert_eq!(p.frame().body(b.id), Some((2, &b)));
    assert_eq!(p.frame().workers[2].generations[&b.id], 1);
    p.compact().unwrap();
    // A third destination missed the first ownership transfer.
    p.transfer_component(b.id, 1).unwrap();
    p.transfer_component(b.id, 0).unwrap();
    assert_eq!(p.frame().workers[0].generations[&b.id], 3);
    assert!(p.transfer_component(b.id, 0).is_err());
    assert!(p.transfer_component(9999, 1).is_err());
}

#[test]
fn prospective_chain_and_barrier_journal_limits_refuse_before_publishing_an_owner() {
    use gz_physics_distributed::coordinator::Error;
    let make = |i: usize| {
        let mut b = fixture()[1].clone();
        b.id = i as u64;
        b.size = [0.5; 3];
        b.position = [i as f64 * 0.54, 0., 0.];
        b.linear = if i.is_multiple_of(2) {
            [3., 0., 0.]
        } else {
            [-3., 0., 0.]
        };
        b
    };
    let separate = |n: usize| {
        let mut seeds = vec![vec![]; 3];
        for i in 0..n {
            seeds[i % 3].push(make(i));
        }
        seeds
    };
    let mut small = WorkerPool::spawn(
        executable(),
        71478,
        separate(2),
        SimulationStep::Ms10,
        None,
        2,
    )
    .unwrap();
    let saved = small.frame().clone();
    assert!(matches!(small.validate_next([0.; 3]), Err(Error::Capacity)));
    assert!(matches!(small.advance(), Err(Error::Capacity)));
    assert_eq!(small.frame(), &saved);
    let mut crowded = WorkerPool::spawn(
        executable(),
        71479,
        separate(33),
        SimulationStep::Ms10,
        None,
        128,
    )
    .unwrap();
    let saved = crowded.frame().clone();
    assert!(matches!(
        crowded.validate_next([0.; 3]),
        Err(Error::Colocation)
    ));
    assert!(matches!(crowded.advance(), Err(Error::Colocation)));
    assert_eq!(crowded.frame(), &saved);
}

#[test]
fn acknowledged_impulse_fifo_survives_interleaved_migration_compaction_and_real_reply_loss() {
    use gz_physics_distributed::physics::{Island, Snapshot};
    let mut a = fixture()[1].clone();
    a.id = 100;
    a.position = [0.; 3];
    a.linear = [0.; 3];
    a.mass = 1.;
    let mut b = a.clone();
    b.id = 200;
    b.position[0] = 3.;
    let initial = vec![a.clone(), b.clone()];
    let mut reference = Island::restore(&Snapshot {
        step: SimulationStep::Ms5,
        bodies: initial.clone(),
        ..Default::default()
    })
    .unwrap();
    let mut pool = WorkerPool::spawn(
        executable(),
        72005,
        vec![vec![a.clone()], vec![b.clone()], vec![], vec![]],
        SimulationStep::Ms5,
        None,
        16,
    )
    .unwrap();
    let original = pool.frame().clone();
    assert!(pool.queue_impulse(3, [0.; 3]).is_err());
    assert!(pool.queue_impulse(100, [f64::NAN; 3]).is_err());
    let inputs = [
        (100, [1., -0., 0.]),
        (200, [-0.011, 0.555, -0.]),
        (100, [-0.4, 0., -0.]),
        (200, [0.011, 0., 0.]),
        (100, [0.2, 0., 0.]),
        (100, [0.2, 0., 0.]),
    ];
    // The worker accepts the first input and withholds its reply. The caller
    // sees the old committed ledger. Retrying contributes exactly once.
    pool.drop_next_acknowledgement(0).unwrap();
    assert!(pool.queue_impulse(inputs[0].0, inputs[0].1).is_err());
    assert_eq!(pool.frame(), &original);
    for (id, impulse) in inputs {
        reference.queue_impulse(id, impulse).unwrap();
        pool.queue_impulse(id, impulse).unwrap();
        assert_eq!(pool.frame(), &original);
    }
    // Moving the first body to the second owner requires a stable merge of
    // two nonempty, alternating input queues. Compaction checks the exact
    // per-worker native FIFO against the coordinator including the sign of zero.
    pool.transfer_component(100, 1).unwrap();
    pool.compact().unwrap();
    pool.recover(1).unwrap();
    let before = pool.frame().clone();
    pool.drop_next_acknowledgement(1).unwrap();
    assert!(pool.advance().is_err());
    assert_eq!(pool.frame(), &before);
    pool.advance().unwrap();
    reference.step().unwrap();
    for b in reference.snapshot().bodies {
        assert_eq!(pool.frame().body(b.id).unwrap().1, &b);
    }
    for _ in 0..5 {
        pool.advance().unwrap();
        reference.step().unwrap();
    }
    for b in reference.snapshot().bodies {
        assert_eq!(pool.frame().body(b.id).unwrap().1, &b);
    }
    assert_eq!(pool.frame().body(100).unwrap().1.linear, [1., 0., 0.]);
    assert!(
        pool.traffic()
            .iter()
            .all(|t| t.requests > 0 && t.reply_bytes > 0)
    );
}

#[test]
fn queued_momentum_routes_before_collision_and_per_worker_input_capacity_preserves_the_fifo() {
    let mut a = fixture()[1].clone();
    a.id = 1;
    a.mass = 1.;
    a.position = [0.; 3];
    a.linear = [0.; 3];
    let mut b = a.clone();
    b.id = 2;
    b.position[0] = 1.5;
    let mut p = WorkerPool::spawn(
        executable(),
        72006,
        vec![vec![a], vec![b], vec![]],
        SimulationStep::Ms10,
        None,
        96,
    )
    .unwrap();
    p.queue_impulse(1, [100., 0., 0.]).unwrap();
    p.advance().unwrap();
    assert_eq!(p.frame().body(1).unwrap().0, p.frame().body(2).unwrap().0);
    for _ in 0..gz_physics_distributed::physics::MAX_PENDING_IMPULSES {
        p.queue_impulse(1, [0.001, 0., -0.]).unwrap();
    }
    let prior = p.frame().clone();
    assert!(p.queue_impulse(2, [0.; 3]).is_err());
    assert_eq!(p.frame(), &prior);
    p.compact().unwrap();
    p.advance().unwrap();
}

#[test]
fn destination_input_capacity_refuses_whole_solver_transfer_without_freezing_anybody() {
    let a = fixture()[1].clone();
    let mut b = a.clone();
    b.id += 2;
    b.position[1] = 20.;
    let mut p = WorkerPool::spawn(
        executable(),
        72008,
        vec![vec![a.clone()], vec![b.clone()], vec![]],
        SimulationStep::Ms5,
        None,
        128,
    )
    .unwrap();
    for _ in 0..17 {
        p.queue_impulse(a.id, [0.; 3]).unwrap();
        p.queue_impulse(b.id, [0.; 3]).unwrap();
    }
    let saved = p.frame().clone();
    assert!(matches!(
        p.transfer_component(a.id, 1),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    assert_eq!(p.frame(), &saved);
    p.compact().unwrap();
    p.advance().unwrap();
}

#[test]
fn failed_native_baseline_keeps_every_old_restart_point_and_pending_command() {
    let mut b = fixture()[1].clone();
    b.id = 91;
    b.mass = 1.;
    b.linear = [0.; 3];
    let mut p = WorkerPool::spawn(
        executable(),
        72012,
        vec![vec![], vec![b.clone()], vec![]],
        SimulationStep::Ms2,
        None,
        8,
    )
    .unwrap();
    p.queue_impulse(b.id, [0.375, 0., -0.]).unwrap();
    let old = p.frame().clone();
    let lengths = p.retained_entries();
    p.terminate_worker(2).unwrap();
    assert!(p.compact().is_err());
    assert_eq!(p.retained_entries(), lengths);
    assert_eq!(p.frame(), &old);
    p.compact().unwrap();
    assert_eq!(p.retained_entries(), vec![0; 3]);
    p.recover(1).unwrap();
    p.advance().unwrap();
    assert_eq!(p.frame().body(b.id).unwrap().1.linear, [0.375, 0., 0.]);
}

#[test]
fn commands_that_would_exceed_a_future_barriers_input_or_ownership_budget_are_deferred_at_enqueue()
{
    let mut a = fixture()[1].clone();
    a.mass = 1.;
    a.id = 10;
    a.linear = [0.; 3];
    a.position = [0.; 3];
    let mut b = a.clone();
    b.id = 11;
    b.position[0] = 4.;
    let mut p = WorkerPool::spawn(
        executable(),
        72013,
        vec![vec![a.clone()], vec![], vec![b.clone()]],
        SimulationStep::Ms10,
        None,
        2,
    )
    .unwrap();
    let initial = p.frame().clone();
    // The bodies can advance independently. This impulse would require a
    // three-command source handoff in a worker journal of only two entries.
    assert!(p.validate_next([0.; 3]).is_ok());
    assert!(matches!(
        p.queue_impulse(a.id, [399., 0., 0.]),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    assert_eq!(p.frame(), &initial);
    p.advance().unwrap();
    assert_eq!(p.frame().body(a.id).unwrap().1.linear, [0.; 3]);
    assert!(matches!(
        WorkerPool::spawn(
            Path::new("/unused"),
            0,
            vec![vec![]; 3],
            SimulationStep::Ms1,
            None,
            0
        ),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
}

#[test]
fn every_lost_real_handoff_ack_preserves_the_warm_source_its_inputs_and_its_authority() {
    for packed in [false, true] {
        check_handoff_ack_losses(packed);
    }
}

fn check_handoff_ack_losses(packed: bool) {
    use gz_physics_distributed::physics::{Island, Primitive, Snapshot};
    let mut a = fixture()[1].clone();
    a.id = 11;
    a.size = [0.5; 3];
    a.primitive = Primitive::Sphere;
    a.position = [-0.28, 0., 0.];
    a.linear = [1., 0., 0.];
    let mut b = a.clone();
    b.id = 19;
    b.position[0] = 0.28;
    b.linear[0] = -1.;
    let mut bystander = b.clone();
    bystander.id = 31;
    bystander.position[1] = 20.;
    let mut solo = Island::restore(&Snapshot {
        bodies: vec![a.clone(), b.clone(), bystander.clone()],
        ..Default::default()
    })
    .unwrap();
    let mut p = WorkerPool::spawn(
        executable(),
        72521,
        vec![vec![a, b], vec![], vec![bystander], vec![]],
        SimulationStep::Ms10,
        None,
        64,
    )
    .unwrap();
    assert_eq!(p.set_packed_handoffs(packed), if packed { 4 } else { 0 });
    for _ in 0..8 {
        solo.step().unwrap();
        p.advance().unwrap();
    }
    assert_eq!(p.frame().workers[0].snapshot.impulses.len(), 1);
    for (id, j) in [
        (11, [0.02, -0., 0.]),
        (31, [0.03, 0., -0.]),
        (19, [-0.015, 0.01, 0.]),
    ] {
        p.queue_impulse(id, j).unwrap();
        solo.queue_impulse(id, j).unwrap();
    }
    p.compact().unwrap();
    let original = p.frame().clone();
    // Export, prepare, release, activate, then the final source observation:
    // the last two losses occur after the old authority has actually retired.
    for (worker, skip, counts) in [
        (0, 0, [1, 0]),
        (2, 0, [1, 1]),
        (0, 1, [2, 1]),
        (2, 1, [2, 2]),
        (0, 2, [3, 2]),
    ] {
        let before = p.traffic();
        p.drop_acknowledgement_after(worker, skip).unwrap();
        assert!(p.transfer_component(11, 2).is_err());
        assert_eq!(p.frame(), &original);
        let after = p.traffic();
        for (w, expected) in [(0, counts[0]), (2, counts[1])] {
            assert_eq!(after[w].requests - before[w].requests, expected);
        }
        p.compact().unwrap();
        assert_eq!(p.frame(), &original);
    }
    p.transfer_component(11, 2).unwrap();
    p.compact().unwrap();
    assert_eq!(p.frame().workers[2].generations[&11], 1);
    for _ in 0..6 {
        solo.step().unwrap();
        p.advance().unwrap();
        let native = solo.snapshot();
        for b in native.bodies {
            assert_eq!(p.frame().body(b.id).unwrap().1, &b);
        }
        assert_eq!(p.frame().workers[2].snapshot.impulses, native.impulses);
    }
}

#[test]
fn a_shard_at_its_stable_id_limit_reserves_retired_owners_and_accepts_a_known_return() {
    use gz_physics_distributed::physics::MAX_PARTITION_BODIES;
    let mut base = fixture()[1].clone();
    base.linear = [0.; 3];
    base.position = [0.; 3];
    let mut full = Vec::new();
    for i in 0..MAX_PARTITION_BODIES {
        let mut b = base.clone();
        b.id = i as u64;
        b.position[1] = 10. * i as f64;
        full.push(b);
    }
    let fresh = Body {
        id: 5000,
        position: [100., 0., 0.],
        ..base
    };
    let mut p = WorkerPool::spawn(
        executable(),
        72522,
        vec![full, vec![fresh], vec![]],
        SimulationStep::Ms2,
        None,
        64,
    )
    .unwrap();
    assert!(matches!(
        p.transfer_component(5000, 0),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    p.transfer_component(0, 2).unwrap();
    let before = p.frame().clone();
    assert_eq!(
        before.workers[0].generations.len(),
        MAX_PARTITION_BODIES - 1
    );
    let traffic = p.traffic();
    assert!(matches!(
        p.transfer_component(5000, 0),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    assert_eq!(p.traffic()[1].requests, traffic[1].requests);
    assert_eq!(p.frame(), &before);
    p.transfer_component(0, 0).unwrap();
    assert_eq!(p.frame().workers[0].generations.len(), MAX_PARTITION_BODIES);
    assert_eq!(p.frame().workers[0].generations[&0], 2);
    p.advance().unwrap();
}

#[test]
fn real_worker_receipts_are_bound_to_width_even_for_the_same_run_epoch_serial_and_endpoints() {
    use gz_physics_distributed::{
        protocol::{Command, Outcome, ProtocolError, Request, WarmTicket},
        transport::{TransportError, WorkerProcess},
    };
    use sha2::{Digest, Sha256};
    let run = 72931;
    let step = SimulationStep::Ms5;
    let b = fixture()[1].clone();
    let id = b.id;
    let mut source = WorkerProcess::spawn_in_pool(
        executable(),
        run,
        0,
        4,
        std::slice::from_ref(&b),
        step,
        None,
    )
    .unwrap();
    let mut correct =
        WorkerProcess::spawn_in_pool(executable(), run, 1, 4, &[], step, None).unwrap();
    let mut other = WorkerProcess::spawn_in_pool(executable(), run, 1, 3, &[], step, None).unwrap();
    let mut legacy = WorkerProcess::spawn_with_step(executable(), run, 1, &[], step).unwrap();
    let ticket = WarmTicket {
        run_id: run,
        source: 0,
        destination: 1,
        serial: 1,
        tick: 0,
        ids: vec![id],
        generations: vec![1],
    };
    let cmd = |worker_id, sequence, command| Request {
        run_id: run,
        worker_id,
        sequence,
        command,
    };
    let export = cmd(
        0,
        1,
        Command::ExportSolver {
            ticket: ticket.clone(),
        },
    );
    let Outcome::WarmExported(h) = source.request(&export).unwrap().outcome else {
        panic!("expected native image")
    };
    h.validate_digest_for_width(4).unwrap();
    for width in [1, 2, 3, 5, 8, 9] {
        assert!(h.validate_digest_for_width(width).is_err());
    }
    let receipt = h.receipt.clone();
    let prepare = cmd(1, 1, Command::PrepareSolver { handoff: h.clone() });
    for wrong in [&mut other, &mut legacy] {
        assert!(matches!(
            wrong.request(&prepare),
            Err(TransportError::Protocol(ProtocolError::Receipt))
        ));
        let Outcome::State(o) = wrong.request(&cmd(1, 1, Command::Observe)).unwrap().outcome else {
            panic!("expected state")
        };
        assert!(o.generations.is_empty());
    }
    assert_eq!(
        correct.request(&prepare).unwrap().outcome,
        Outcome::WarmPrepared(receipt.prepared())
    );
    assert_eq!(
        source
            .request(&cmd(
                0,
                2,
                Command::ReleaseWarm {
                    receipt: receipt.prepared()
                }
            ))
            .unwrap()
            .outcome,
        Outcome::WarmReleased(receipt.released())
    );
    let Outcome::WarmActivated(active) = correct
        .request(&cmd(
            1,
            2,
            Command::ActivateWarm {
                receipt: receipt.released(),
            },
        ))
        .unwrap()
        .outcome
    else {
        panic!("expected activation")
    };
    assert_eq!(active.generations[&id], 1);
    let mut pair_source =
        WorkerProcess::spawn_with_step(executable(), run, 0, std::slice::from_ref(&b), step)
            .unwrap();
    let Outcome::WarmExported(old) = pair_source.request(&export).unwrap().outcome else {
        panic!("pair export")
    };
    old.validate_digest().unwrap();
    let mut hash = Sha256::new();
    hash.update(b"gz-physics-distributed/solver-handoff/v1\0");
    hash.update(serde_json::to_vec(&(&ticket, &old.checkpoint)).unwrap());
    let legacy_digest: [u8; 32] = hash.finalize().into();
    assert_eq!(legacy_digest, old.receipt.digest);
    assert_ne!(h.receipt.digest, legacy_digest);
    assert_eq!(h.checkpoint, old.checkpoint);
    // The pair's older cold-body protocol has no width field. A wide worker
    // must require a width-bound native checkpoint even for a free singleton.
    let mut cold =
        WorkerProcess::spawn_with_step(executable(), run, 0, std::slice::from_ref(&b), step)
            .unwrap();
    let t = gz_physics_distributed::protocol::Ticket {
        run_id: run,
        serial: 1,
        source: 0,
        destination: 1,
        body_id: id,
        generation: 1,
        tick: 0,
        promote: false,
    };
    let Outcome::Exported(portable) = cold
        .request(&cmd(0, 1, Command::Export { ticket: t }))
        .unwrap()
        .outcome
    else {
        panic!("legacy cold receipt")
    };
    assert!(matches!(
        other.request(&cmd(1, 2, Command::Prepare { handoff: portable })),
        Err(TransportError::Protocol(ProtocolError::Address))
    ));
    let Outcome::State(empty) = other.request(&cmd(1, 2, Command::Observe)).unwrap().outcome else {
        panic!("expected unchanged state")
    };
    assert!(empty.generations.is_empty());
}

#[test]
fn native_request_deadline_is_local_and_survives_worker_recovery() {
    use gz_physics_distributed::pool::PoolNativeRecovery;
    use std::time::Duration;
    let seeds = vec![fixture(), vec![], vec![]];
    let mut p =
        WorkerPool::spawn(executable(), 71572, seeds, SimulationStep::Ms1, None, 8).unwrap();
    assert_eq!(p.native_work_timeout(), Duration::from_secs(1));
    p.set_native_work_timeout(Duration::from_secs(5)).unwrap();
    for invalid in [
        Duration::ZERO,
        Duration::from_secs(5) + Duration::from_nanos(1),
    ] {
        assert!(p.set_native_work_timeout(invalid).is_err());
        assert_eq!(p.native_work_timeout(), Duration::from_secs(5));
    }
    p.terminate_worker(0).unwrap();
    p.recover(0).unwrap();
    assert_eq!(p.native_work_timeout(), Duration::from_secs(5));
    p.advance_in_gravity([0., 0., 0.]).unwrap();
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("cut");
    p.save_native_recovery(&path).unwrap();
    drop(p);
    let p = PoolNativeRecovery::open(path)
        .unwrap()
        .resume(executable())
        .unwrap();
    assert_eq!(p.native_work_timeout(), Duration::from_secs(1));
}
