// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, Primitive, SimulationStep, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::{fs, path::Path};

fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}

fn seeds() -> Vec<Vec<Body>> {
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
    let far = |id| Body {
        id,
        position: [10. * id as f64, 0., 0.],
        linear: [0.; 3],
        ..a.clone()
    };
    let (x, y, z) = (far(31), far(51), far(71));
    vec![vec![a, b, x], vec![y], vec![z], vec![]]
}

fn spawn(limit: usize) -> WorkerPool {
    WorkerPool::spawn(
        worker(),
        0x42415443,
        seeds(),
        SimulationStep::Ms10,
        None,
        limit,
    )
    .unwrap()
}

const MOVES: [(u64, u8); 4] = [(19, 1), (31, 1), (51, 0), (71, 3)];

#[test]
fn invalid_or_over_budget_batch_preserves_the_committed_owners_without_dispatch() {
    let mut p = spawn(4);
    assert!(
        p.set_handoff_confirmation_window(Some(std::time::Duration::ZERO))
            .is_err()
    );
    assert!(
        p.set_handoff_confirmation_window(Some(std::time::Duration::from_millis(1001)))
            .is_err()
    );
    for _ in 0..8 {
        p.advance().unwrap();
    }
    let before = p.frame().clone();
    let traffic: Vec<_> = p.traffic().iter().map(|t| t.requests).collect();
    assert_eq!(p.transfer_components(&[]).unwrap(), &before);
    for invalid in [
        &[(11, 2), (19, 3)][..],
        &[(11, 2), (50, 2)],
        &[(19, 0)],
        &[(19, 4)],
        &[(11, 2); 6],
    ] {
        assert!(matches!(
            p.transfer_components(invalid),
            Err(Error::Invalid(_))
        ));
    }
    assert!(matches!(
        p.transfer_components(&MOVES),
        Err(Error::Capacity)
    ));
    assert_eq!(p.frame(), &before);
    assert_eq!(
        p.traffic().iter().map(|t| t.requests).collect::<Vec<_>>(),
        traffic
    );
}

#[test]
fn every_partial_batch_recovers_the_complete_pre_transfer_barrier_and_input_fifo() {
    let dir = tempfile::tempdir().unwrap();
    let seed = dir.path().join("before.native");
    let mut p = spawn(256);
    for _ in 0..8 {
        p.advance().unwrap();
    }
    assert_eq!(p.frame().workers[0].snapshot.impulses.len(), 1);
    for (id, x) in [
        (31, 0.07),
        (51, 0.005),
        (19, -0.02),
        (71, 0.125),
        (11, 0.01),
        (31, -0.04),
    ] {
        p.queue_impulse(id, [x, -0., 0.]).unwrap();
    }
    p.compact().unwrap();
    p.save_native_recovery(&seed).unwrap();
    let original = p.frame().clone();
    let logs = p.retained_entries();
    assert_eq!(p.set_packed_handoffs(true), 4);
    // All 16 commands through activation and all three final source observations.
    let fault_order = [0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 2, 3, 2, 3, 0, 1, 2];
    let mut offsets = [0; 4];
    for (ordinal, owner) in fault_order.into_iter().enumerate() {
        let before = p.traffic().iter().map(|t| t.requests).sum::<u64>();
        p.drop_acknowledgement_after(owner, offsets[usize::from(owner)])
            .unwrap();
        offsets[usize::from(owner)] += 1;
        assert!(p.transfer_components(&MOVES).is_err());
        assert_eq!(p.frame(), &original);
        assert_eq!(p.retained_entries(), logs);
        assert_eq!(
            p.traffic().iter().map(|t| t.requests).sum::<u64>() - before,
            ordinal as u64 + 1
        );
        let checkpoint = dir.path().join(format!("fenced-{ordinal}.native"));
        p.save_native_recovery(&checkpoint).unwrap();
        assert_eq!(fs::read(&checkpoint).unwrap(), fs::read(&seed).unwrap());
    }
    let first = dir.path().join("after.native");
    p.transfer_components(&MOVES).unwrap();
    for (id, owner) in [(11, 1), (19, 1), (31, 1), (51, 0), (71, 3)] {
        assert_eq!(p.frame().body(id).unwrap().0, owner);
        assert_eq!(p.frame().workers[usize::from(owner)].generations[&id], 1);
    }
    p.save_native_recovery(&first).unwrap();
    let mut reverse = PoolNativeRecovery::open(&seed)
        .unwrap()
        .resume(worker())
        .unwrap();
    // Different input order, alternate alias and default JSON encoding yield identical durable state.
    reverse
        .transfer_components(&[(71, 3), (51, 0), (31, 1), (11, 1)])
        .unwrap();
    let permuted = dir.path().join("permuted.native");
    reverse.save_native_recovery(&permuted).unwrap();
    assert_eq!(fs::read(&first).unwrap(), fs::read(&permuted).unwrap());
    let mut sequential = PoolNativeRecovery::open(&seed)
        .unwrap()
        .resume(worker())
        .unwrap();
    for (id, dest) in MOVES {
        sequential.transfer_component(id, dest).unwrap();
    }
    let mut resumed = PoolNativeRecovery::open(&first)
        .unwrap()
        .resume(worker())
        .unwrap();
    assert_eq!(resumed.frame(), sequential.frame());
    for _ in 0..24 {
        assert_eq!(resumed.advance().unwrap(), sequential.advance().unwrap());
        assert_eq!(p.advance().unwrap(), resumed.frame());
    }
}

#[test]
fn confirmed_batches_retain_other_workers_at_every_receipt_and_replay_each_staged_prefix() {
    let dir = tempfile::tempdir().unwrap();
    let seed = dir.path().join("seed.native");
    let mut control = spawn(256);
    for _ in 0..8 {
        control.advance().unwrap();
    }
    for (id, x) in [(31, 0.1), (51, 0.02), (19, -0.01), (71, 0.3), (11, 0.07)] {
        control.queue_impulse(id, [x, -0., 0.]).unwrap();
    }
    control.save_native_recovery(&seed).unwrap();
    control.transfer_components(&MOVES).unwrap();
    let expected = dir.path().join("expected.native");
    control.save_native_recovery(&expected).unwrap();
    let expected_bytes = fs::read(expected).unwrap();
    let mut future = Vec::new();
    for _ in 0..5 {
        future.push(control.advance().unwrap().clone());
    }
    for kill in [false, true] {
        for packed in [false, true] {
            let mut earlier = [0; 4];
            for (ordinal, w) in [0u8, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 2, 3, 2, 3, 0, 1, 2]
                .into_iter()
                .enumerate()
            {
                let mut p = PoolNativeRecovery::open(&seed)
                    .unwrap()
                    .resume(worker())
                    .unwrap();
                p.set_handoff_confirmation(true);
                p.set_packed_handoffs(packed);
                let pids = |p: &WorkerPool| {
                    (0..4)
                        .map(|id| p.worker_pid(id).unwrap())
                        .collect::<Vec<_>>()
                };
                let before = pids(&p);
                let offset = &mut earlier[usize::from(w)];
                if kill {
                    p.terminate_after_command(w, *offset).unwrap();
                } else {
                    p.drop_acknowledgement_after(w, *offset).unwrap();
                }
                *offset += 1;
                p.transfer_components(&MOVES).unwrap();
                for (i, (&a, b)) in before.iter().zip(pids(&p)).enumerate() {
                    assert_eq!(a != b, kill && usize::from(w) == i);
                }
                let name = dir
                    .path()
                    .join(format!("ack-{kill}-{packed}-{ordinal}.native"));
                p.save_native_recovery(&name).unwrap();
                assert_eq!(fs::read(name).unwrap(), expected_bytes);
                for next in &future {
                    assert_eq!(p.advance().unwrap(), next);
                }
            }
        }
    }
}
