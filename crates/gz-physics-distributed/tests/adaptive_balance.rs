// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{BalancePolicy, Coordinator},
    physics::{Body, fixture},
    transport::WorkerProcess,
};
use std::path::Path;

fn group(a: &[Body], b: &[Body]) -> Coordinator<WorkerProcess> {
    let bin = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    Coordinator::new_recorded(
        71,
        [
            WorkerProcess::spawn(bin, 71, 0, a).unwrap(),
            WorkerProcess::spawn(bin, 71, 1, b).unwrap(),
        ],
        256,
    )
    .unwrap()
}
#[test]
fn samples_require_exact_barriers_and_stable_native_membership_and_replacement_resets_only_diagnostics()
 {
    let seeds = fixture();
    let mut c = group(&seeds[..1], &seeds[1..]);
    assert!(c.start_load_sample(0).is_err());
    let token = c.start_load_sample(4).unwrap();
    assert_eq!(c.start_load_sample(4).unwrap(), token);
    assert!(c.finish_load_sample(token).is_err());
    let dir = tempfile::tempdir().unwrap();
    c.save_recovery(dir.path().join("start")).unwrap();
    for _ in 0..4 {
        c.advance_overlapped().unwrap();
    }
    let sample = c.finish_load_sample(token).unwrap();
    assert_eq!(sample.measured_steps(), 4);
    for (owner, rows) in sample.workers().iter().enumerate() {
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].members, [owner as u64 + 1]);
        assert!(rows[0].elapsed_ns > 0);
    }
    let expected = c.frame().clone();
    c.recover(0).unwrap();
    assert_eq!(c.frame(), &expected);
    assert!(c.finish_load_sample(token).is_err());
    // Starting and reading a sample never modifies the deterministic proof.
    let mut unprofiled = group(&seeds[..1], &seeds[1..]);
    for _ in 0..4 {
        unprofiled.advance().unwrap();
    }
    c.save_recovery(dir.path().join("profiled")).unwrap();
    unprofiled
        .save_recovery(dir.path().join("unprofiled"))
        .unwrap();
    assert_eq!(
        std::fs::read(dir.path().join("profiled")).unwrap(),
        std::fs::read(dir.path().join("unprofiled")).unwrap()
    );
    let transient = c.start_load_sample(4).unwrap();
    c.transfer(1, true).unwrap();
    assert!(c.plan_rebalance(&sample, BalancePolicy::default()).is_err());
    for _ in 0..4 {
        c.advance().unwrap();
    }
    assert!(c.finish_load_sample(transient).is_err());
}

#[test]
fn sampling_a_cold_native_union_requires_a_new_window_after_the_join() {
    let a = Body {
        id: 1,
        position: [-1.5, 0., 0.],
        linear: [2., 0., 0.],
        ..fixture()[1].clone()
    };
    let b = Body {
        id: 2,
        position: [1.5, 0., 0.],
        linear: [-2., 0., 0.],
        ..a.clone()
    };
    let mut c = group(&[a, b], &[]);
    assert_eq!(c.frame().workers[0].components.len(), 2);
    let old = c.start_load_sample(65).unwrap();
    for _ in 0..65 {
        c.advance_overlapped().unwrap();
    }
    assert_eq!(c.frame().workers[0].components.len(), 1);
    assert!(c.finish_load_sample(old).is_err());
    let new = c.start_load_sample(3).unwrap();
    for _ in 0..3 {
        c.advance().unwrap();
    }
    let sample = c.finish_load_sample(new).unwrap();
    assert_eq!(sample.workers()[0][0].members, [1, 2]);
    assert!(sample.workers()[1].is_empty());
}
