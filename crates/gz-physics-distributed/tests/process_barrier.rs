// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Coordinator, physics::fixture, transport::WorkerProcess,
};
use std::path::Path;

#[test]
fn production_peers_fence_and_recover_a_transferred_owner() {
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let seed = fixture();
    let run = u64::from(std::process::id());
    let peers = [
        WorkerProcess::spawn(path, run, 0, &seed[..1]).unwrap(),
        WorkerProcess::spawn(path, run, 1, &seed[1..]).unwrap(),
    ];
    let original_session = peers[1].session_id().to_owned();
    let mut group = Coordinator::new(run, peers, 16).unwrap();
    let transfer = group.transfer(1, true).unwrap().clone();
    group.recover(1).unwrap();
    assert_eq!(group.frame(), &transfer);
    let tick = group.advance().unwrap();
    assert_eq!(tick.tick, 1);
    assert!(tick.body(1).unwrap().1.position[0] > seed[0].position[0]);
    assert_eq!(tick.workers[1].generations[&1], 1);
    let [mut left, mut right] = group.into_peers();
    assert_ne!(right.session_id(), original_session);
    left.shutdown().unwrap();
    right.shutdown().unwrap();
}

#[test]
fn warm_migration_after_cold_promotion_replays_on_both_replacement_processes() {
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut seed = fixture();
    seed[0].position[0] = -0.42;
    seed[1].position[0] = 0.42;
    let run = u64::from(std::process::id());
    let make = || {
        [
            WorkerProcess::spawn(path, run, 0, &seed[..1]).unwrap(),
            WorkerProcess::spawn(path, run, 1, &seed[1..]).unwrap(),
        ]
    };
    let mut moved = Coordinator::new(run, make(), 100).unwrap();
    let mut control = Coordinator::new(run, make(), 100).unwrap();
    for world in [&mut moved, &mut control] {
        world.transfer(1, true).unwrap();
        while world.frame().workers[1].snapshot.contacts == 0 {
            assert!(world.frame().tick < 20);
            world.advance().unwrap();
        }
        world.queue_impulse(1, [0.65, -0.2, 0.1]).unwrap();
    }
    // Body 1 has already crossed a worker boundary. Body 2 is crossing for
    // the first time, so the members of one island have different generations.
    moved.transfer_island(1).unwrap();
    assert_eq!(moved.frame().workers[0].generations[&1], 2);
    assert_eq!(moved.frame().workers[0].generations[&2], 1);
    moved.recover(1).unwrap(); // Recompute the exported checkpoint in a fresh process.
    moved.recover(0).unwrap(); // Restore it and its queued input in another.
    for _ in 0..10 {
        let a = moved.advance().unwrap();
        let b = control.advance().unwrap();
        assert_eq!(a.workers[0].snapshot, b.workers[1].snapshot);
    }
}
