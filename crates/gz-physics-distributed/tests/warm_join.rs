// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/train_run.rs"]
mod train_run;

#[test]
fn two_loaded_box_trains_join_with_native_manifolds_before_their_first_cross_contact() {
    train_run::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        |_| {},
    )
    .unwrap();
}

#[test]
fn complete_transfers_keep_global_input_order_through_compaction_and_fresh_workers() {
    use gz_physics_distributed::{
        coordinator::Coordinator,
        physics::{Body, Island, Snapshot, fixture},
        transport::WorkerProcess,
    };
    let bodies: Vec<_> = [-10.35, -9.70, 9.70, 10.35]
        .into_iter()
        .enumerate()
        .map(|(i, x)| Body {
            id: 1 + i as u64,
            position: [x, 0., 0.],
            linear: [if i % 2 == 0 { 8. } else { -8. }, 0., 0.],
            mass: 1.,
            ..fixture()[0].promoted()
        })
        .collect();
    let worker = std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let run = 0x577a11;
    let peers = [
        WorkerProcess::spawn(worker, run, 0, &bodies[..2]).unwrap(),
        WorkerProcess::spawn(worker, run, 1, &bodies[2..]).unwrap(),
    ];
    let mut c = Coordinator::new_recorded(run, peers, 384).unwrap();
    let mut native = Island::restore(&Snapshot {
        bodies,
        ..Default::default()
    })
    .unwrap();
    for _ in 0..2 {
        c.advance().unwrap();
        native.step().unwrap();
    }
    assert!(c.frame().workers.iter().all(|w| w.snapshot.contacts == 1));
    for i in 0..32 {
        let id = [1, 4, 3, 2][i % 4];
        let input = [
            0.0001 * (i + 1) as f64,
            if i % 3 == 0 { -0. } else { 0. },
            0.,
        ];
        c.queue_impulse(id, input).unwrap();
        native.queue_impulse(id, input).unwrap();
    }
    for member in [1, 1, 3] {
        c.transfer_component(member).unwrap();
        c.compact().unwrap();
        c.recover(0).unwrap();
        c.recover(1).unwrap();
    }
    for _ in 0..50 {
        c.advance_routed_overlapped().unwrap();
        native.step().unwrap();
        let expected = native.snapshot();
        for b in &expected.bodies {
            assert_eq!(c.frame().body(b.id).unwrap().1, b);
        }
        let mut observed = c
            .frame()
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.clone())
            .collect::<Vec<_>>();
        observed.sort_by_key(|p| (p.a, p.b));
        assert_eq!(observed, expected.impulses);
    }
    for mut p in c.into_peers() {
        p.shutdown().unwrap();
    }
}
