// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan},
    physics::{
        Body, GroundPlane, MAX_CHECKPOINT_BODIES, MAX_PARTITION_BODIES, SimulationStep, Snapshot,
        fixture,
    },
    protocol::Worker,
    transport::WorkerProcess,
};
use std::path::Path;

#[test]
fn maximum_worker_population_can_restore_and_transfer_a_supported_body_with_pending_input() {
    let run = 0x9924;
    let step = SimulationStep::Ms5;
    let g = Some(GroundPlane::with_friction(0., 0.4).unwrap());
    let boxes: Vec<_> = (0..MAX_PARTITION_BODIES)
        .map(|i| Body {
            id: i as u64,
            position: [8. * (i % 16) as f64, 8. * (i / 16) as f64, 0.32],
            size: [0.4; 3],
            linear: [0.16, 0., 0.],
            angular: [0.; 3],
            friction: 0.3,
            ..fixture()[0].promoted()
        })
        .collect();
    let executable = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spawn = || {
        [
            WorkerProcess::spawn_with_ground(executable, run, 0, &boxes, step, g).unwrap(),
            WorkerProcess::spawn_with_ground(executable, run, 1, &[], step, g).unwrap(),
        ]
    };
    let mut distributed = Coordinator::new(run, spawn(), 1024).unwrap();
    let gravity = [0., 0., -9.81];
    for _ in 0..58 {
        distributed
            .advance_in_field_overlapped(gravity, &[])
            .unwrap();
    }
    assert_eq!(
        distributed.frame().workers[0].snapshot.supports.len(),
        MAX_PARTITION_BODIES
    );
    let last = (MAX_PARTITION_BODIES - 1) as u64;
    distributed.queue_impulse(last, [0.2, -0., 0.015]).unwrap();
    let tmp = tempfile::tempdir().unwrap();
    let path = tmp.path().join("partition-image");
    distributed.save_native_recovery(&path).unwrap();
    let bytes = std::fs::metadata(&path).unwrap().len();
    let mut control = NativeRecoveryPlan::open(&path)
        .unwrap()
        .resume(executable)
        .unwrap();
    assert_eq!(distributed.frame(), control.frame());
    distributed.transfer_component(last).unwrap();
    assert_eq!(distributed.frame().workers[1].snapshot.supports.len(), 1);
    let stats = distributed.compact().unwrap();
    distributed.recover(0).unwrap();
    distributed.recover(1).unwrap();
    for _ in 0..35 {
        let actual = distributed
            .advance_in_field_overlapped(gravity, &[])
            .unwrap();
        let expected = control.advance_in_field(gravity, &[]).unwrap();
        let mut supports = actual
            .workers
            .iter()
            .flat_map(|w| w.snapshot.supports.clone())
            .collect::<Vec<_>>();
        supports.sort_by_key(|s| s.body);
        assert_eq!(supports, expected.workers[0].snapshot.supports);
        for b in &expected.workers[0].snapshot.bodies {
            assert_eq!(actual.body(b.id).unwrap().1, b);
        }
    }
    eprintln!(
        "{MAX_PARTITION_BODIES} supported native singletons: native file {bytes} bytes, bootstrap on compaction {} bytes; 35 advancing frames exact after fenced restart and transfer",
        stats.bootstrap_bytes
    );
    for sim in [distributed, control] {
        for mut peer in sim.into_peers() {
            peer.shutdown().unwrap();
        }
    }
}

#[test]
fn cold_contacts_that_would_form_an_oversized_native_component_are_refused_before_stepping() {
    let template = fixture()[0].promoted();
    let chain: Vec<_> = (0..=MAX_CHECKPOINT_BODIES)
        .map(|i| Body {
            id: i as u64,
            position: [0.65 * i as f64, 0., 0.],
            linear: [0.; 3],
            ..template.clone()
        })
        .collect();
    assert!(
        Worker::new(
            120,
            0,
            Snapshot {
                bodies: chain.clone(),
                ..Default::default()
            }
        )
        .is_err()
    );
    assert!(
        Worker::new(
            120,
            0,
            Snapshot {
                bodies: chain[..MAX_CHECKPOINT_BODIES].to_vec(),
                ..Default::default()
            }
        )
        .is_ok()
    );
}

#[test]
fn rolling_journal_reserves_an_entire_sixty_four_route_barrier_before_migration() {
    let step = SimulationStep::Ms10;
    let seed = fixture()[0].promoted();
    let columns = [0, 1].map(|w| {
        (0..64)
            .map(|i| Body {
                id: 2 * i + w as u64,
                position: [if w == 0 { -0.7 } else { 0.7 }, 12. * i as f64, 0.],
                linear: [0.; 3],
                ..seed.clone()
            })
            .collect::<Vec<_>>()
    });
    let binary = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let start = |run, limit| {
        Coordinator::new(
            run,
            [0, 1].map(|w| {
                WorkerProcess::spawn_with_step(binary, run, w as u8, &columns[w], step).unwrap()
            }),
            limit,
        )
        .unwrap()
    };
    let mut rolling = start(791, 220);
    rolling.enable_rolling_journals();
    let mut reference = start(792, 2048);
    for c in [&mut rolling, &mut reference] {
        for _ in 0..82 {
            c.advance_overlapped().unwrap();
        }
        for b in &columns[0] {
            c.set_velocity(b.id, [50., 0., 0.], [0.; 3]).unwrap();
        }
        let routes = c.contact_routes().unwrap();
        assert_eq!(routes.len(), 64);
        assert!(routes.iter().all(|r| r.destination == 1));
    }
    let before = rolling.retained_entries();
    assert!(before[0] + 3 * 64 + 1 > 220);
    for tick in 83..=91 {
        let a = rolling.advance_routed_overlapped().unwrap();
        let b = reference.advance_routed_overlapped().unwrap();
        assert_eq!(a, b);
        if tick == 83 {
            assert_eq!(rolling.retained_entries(), [193, 129]);
        }
        if tick == 86 {
            rolling.recover(0).unwrap();
            rolling.recover(1).unwrap();
        }
    }
    for sim in [rolling, reference] {
        for mut peer in sim.into_peers() {
            peer.shutdown().unwrap();
        }
    }
}
