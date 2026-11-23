// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{Body, fixture},
    transport::WorkerProcess,
};
use std::{fs, path::Path};

const RUN: u64 = 20261119;
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn group(run: u64, seeds: &[Vec<Body>; 2], limit: usize) -> Coordinator<WorkerProcess> {
    let [a, b] = seeds;
    let peers = [
        WorkerProcess::spawn(worker(), run, 0, a).unwrap(),
        WorkerProcess::spawn(worker(), run, 1, b).unwrap(),
    ];
    Coordinator::new_recorded(run, peers, limit).unwrap()
}

#[test]
fn native_images_resume_mixed_singletons_loaded_caches_inputs_and_generation_fences() {
    let [a, b]: [Body; 2] = fixture().try_into().unwrap();
    let spectator = Body {
        id: 3,
        position: [4., 7., 2.],
        angular: [0.1, 0.2, 1.],
        ..a.clone()
    };
    let seeds = [vec![a, spectator], vec![b]];
    let dir = tempfile::tempdir().unwrap();
    let mut c = group(RUN, &seeds, 16);
    let mut control = group(RUN + 1, &seeds, 512);
    let mut max_images = 0;
    for target in [0, 11, 125, 140] {
        while c.frame().tick < target {
            if c.retained_entries().into_iter().max().unwrap() >= 12 {
                max_images = max_images.max(c.compact().unwrap().bootstrap_bytes);
            }
            assert_eq!(c.advance_overlapped().unwrap(), control.advance().unwrap());
        }
        if target == 11 {
            assert_eq!(
                c.transfer(1, true).unwrap(),
                control.transfer(1, true).unwrap()
            );
        }
        if target == 125 {
            assert!(c.frame().workers[1].snapshot.contacts > 0);
            c.compact().unwrap();
            assert_eq!(
                c.transfer_group(1).unwrap(),
                control.transfer_group(1).unwrap()
            );
            assert_eq!(
                c.queue_impulse(1, [1.2, 0., -0.]).unwrap(),
                control.queue_impulse(1, [1.2, 0., -0.]).unwrap()
            );
        }
        let path = dir.path().join(format!("{target}.gznative"));
        let previous = c.retained_entries();
        c.save_native_recovery(&path).unwrap();
        assert_eq!(previous, c.retained_entries());
        assert!(c.save_native_recovery(&path).is_err());
        assert!(RecoveryPlan::open(&path).is_err());
        let next = NativeRecoveryPlan::open(&path).unwrap();
        assert_eq!(next.frame(), control.frame());
        drop(c);
        c = next.resume(worker()).unwrap();
        assert_eq!(c.retained_entries(), [0, 0]);
        assert_eq!(c.compact().unwrap().released_entries, [0, 0]);
        c.recover(0).unwrap();
        c.recover(1).unwrap();
        if target == 125 {
            // Return the whole warm component, carrying the still-pending impulse.
            assert_eq!(
                c.transfer_group(2).unwrap(),
                control.transfer_group(2).unwrap()
            );
        }
        let mut damaged = fs::read(&path).unwrap();
        *damaged.last_mut().unwrap() ^= 1;
        let path = dir.path().join("bad");
        fs::write(&path, damaged).unwrap();
        assert!(NativeRecoveryPlan::open(path).is_err());
    }
    assert!(max_images > 0);
    assert_eq!(c.advance().unwrap(), control.advance().unwrap());
}

#[test]
fn seed_file_after_many_compactions_rebuilds_the_current_window_and_rolling_proof() {
    let b = fixture();
    let seeds = [vec![b[0].clone()], vec![b[1].clone()]];
    let mut c = group(RUN, &seeds, 6);
    for _ in 0..14 {
        for _ in 0..4 {
            c.advance().unwrap();
        }
        c.compact().unwrap();
    }
    c.advance().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("portable.gzrec");
    c.save_recovery(&path).unwrap();
    assert!(NativeRecoveryPlan::open(&path).is_err());
    let published = c.frame().clone();
    let suffix = c.retained_entries();
    let expected = c.advance().unwrap().clone();
    drop(c);
    let mut rebuilt = RecoveryPlan::open(path)
        .unwrap()
        .resume(|run, id, bodies| WorkerProcess::spawn(worker(), run, id, bodies))
        .unwrap();
    assert_eq!(rebuilt.frame(), &published);
    assert_eq!(rebuilt.retained_entries(), suffix);
    assert_eq!(rebuilt.advance().unwrap(), &expected);
}

#[test]
fn legacy_seed_recovery_replays_the_original_published_digest() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("legacy.gzrec");
    fs::write(&path, include_bytes!("data/recovery-v1.gzrec")).unwrap();
    let c = RecoveryPlan::open(path)
        .unwrap()
        .resume(|run, id, bodies| WorkerProcess::spawn(worker(), run, id, bodies))
        .unwrap();
    assert_eq!(c.frame().tick, 119);
}

#[test]
fn immediate_spatial_impulses_held_torques_velocity_and_deletion_replay_from_either_format() {
    use gz_physics_distributed::physics::BodyWrench;
    let [a, b]: [Body; 2] = fixture().try_into().unwrap();
    let seeds = [vec![a.promoted()], vec![b]];
    let mut c = group(RUN, &seeds, 512);
    for _ in 0..140 {
        c.advance_routed().unwrap();
    }
    assert!(c.frame().workers.iter().any(|w| w.snapshot.contacts > 0));
    let pending = [0.015, 0., -0.0];
    c.queue_impulse_routed(2, pending).unwrap();
    c.impulse_now(2, [-0.02, 0., 0.], [0.03, -0.011, 0.045])
        .unwrap();
    let wrench = BodyWrench {
        id: 1,
        force: [1.25, -0.1, 0.],
        torque: [0.003, 0., 0.],
    };
    c.advance_routed_with_wrenches(&[wrench]).unwrap();
    c.set_velocity(2, [0.25, -0.11, 0.3], [0.6, 0., -0.1])
        .unwrap();
    // Remove a contacting body at the same tick. Its accepted command expires.
    c.queue_impulse_routed(1, [0.01; 3]).unwrap();
    c.queue_impulse_routed(2, pending).unwrap();
    c.retire(1).unwrap();
    c.compact().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let native_path = dir.path().join("survivor.gznative");
    let seed_path = dir.path().join("survivor.gzrec");
    c.save_native_recovery(&native_path).unwrap();
    c.save_recovery(&seed_path).unwrap();
    let mut native = NativeRecoveryPlan::open(&native_path)
        .unwrap()
        .resume(worker())
        .unwrap();
    let mut seed = RecoveryPlan::open(&seed_path)
        .unwrap()
        .resume(|r, w, b| WorkerProcess::spawn(worker(), r, w, b))
        .unwrap();
    for _ in 0..85 {
        let expected = c.advance_routed_overlapped().unwrap();
        assert_eq!(native.advance_routed_overlapped().unwrap(), expected);
        assert_eq!(seed.advance_routed_overlapped().unwrap(), expected);
    }
    for world in [&mut native, &mut seed] {
        world.retire(2).unwrap();
        world.compact().unwrap();
        world.recover(0).unwrap();
        world.recover(1).unwrap();
        assert!(
            world
                .advance()
                .unwrap()
                .workers
                .iter()
                .all(|o| o.generations.is_empty())
        );
    }
}

#[test]
fn a_whole_mixed_solver_and_its_retired_warm_singleton_move_without_reconstruction() {
    let seeds = [
        fixture(),
        vec![Body {
            id: 3,
            position: [0., 25., 0.],
            ..fixture()[0].clone()
        }],
    ];
    let mut c = group(RUN, &seeds, 512);
    let mut reference = group(RUN + 1, &seeds, 512);
    for _ in 0..135 {
        c.advance_routed().unwrap();
        reference.advance_routed().unwrap();
    }
    assert_eq!(c.frame().workers[0].components.len(), 1);
    assert!(c.frame().workers[0].snapshot.contacts > 0);
    assert!(c.transfer_group(1).is_err());
    c.queue_impulse(2, [0., 0.02, 0.]).unwrap();
    reference.queue_impulse(2, [0., 0.02, 0.]).unwrap();
    c.transfer_component(1).unwrap();
    assert_eq!(c.frame().workers[1].generations[&2], 1);
    for i in 0..96 {
        if i == 16 {
            c.retire(1).unwrap();
            reference.retire(1).unwrap();
            assert!(
                c.frame().workers[1]
                    .components
                    .iter()
                    .any(|g| g.ids == [2] && g.contact_history)
            );
            c.transfer_component(2).unwrap();
            assert_eq!(c.frame().workers[0].generations[&2], 2);
        }
        if i == 19 {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("mixed.gzrec");
            c.save_recovery(&path).unwrap();
            let snapshot = c.frame().clone();
            drop(c);
            c = RecoveryPlan::open(&path)
                .unwrap()
                .resume(|r, w, s| WorkerProcess::spawn(worker(), r, w, s))
                .unwrap();
            assert_eq!(c.frame(), &snapshot);
            c.compact().unwrap();
            c.recover(0).unwrap();
        }
        c.advance_routed().unwrap();
        reference.advance_routed().unwrap();
        for id in [1, 2, 3] {
            assert_eq!(
                c.frame().body(id).map(|(_, b)| b),
                reference.frame().body(id).map(|(_, b)| b)
            );
        }
        let contacts = |c: &Coordinator<WorkerProcess>| {
            c.frame()
                .workers
                .iter()
                .flat_map(|w| w.snapshot.impulses.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(contacts(&c), contacts(&reference));
    }
}

#[test]
fn fixed_and_dynamic_solver_is_fenced_transferred_and_restored_in_uniform_gravity() {
    use gz_physics_distributed::physics::Tier;
    let mut floor = fixture()[0].clone();
    floor.tier = Tier::Fixed;
    floor.mass = 0.;
    floor.position = [0.; 3];
    floor.size = [2., 2., 0.2];
    floor.linear = [0.; 3];
    let mut load = fixture()[1].clone();
    load.position = [0., 0., 1.1];
    load.linear = [0.; 3];
    let seeds = [vec![floor], vec![load]];
    let mut c = group(RUN, &seeds, 1024);
    let g = [0., 0., -9.81];
    assert!(c.transfer(1, true).is_err());
    assert!(c.impulse_now(1, [0.; 3], [1., 0., 0.]).is_err());
    assert!(c.set_velocity(1, [1., 0., 0.], [0.; 3]).is_err());
    for _ in 0..90 {
        c.advance_in_field(g, &[]).unwrap();
    }
    let (owner, _) = c.frame().body(1).unwrap();
    assert!(c.frame().workers[usize::from(owner)].snapshot.contacts > 0);
    c.queue_impulse(2, [0.03, 0., 0.]).unwrap();
    let dir = tempfile::tempdir().unwrap();
    c.transfer_component(1).unwrap();
    c.compact().unwrap();
    let path = dir.path().join("stack.gzrec");
    c.save_recovery(&path).unwrap();
    let mut seed = RecoveryPlan::open(path)
        .unwrap()
        .resume(|r, w, b| WorkerProcess::spawn(worker(), r, w, b))
        .unwrap();
    let path = dir.path().join("stack.gznative");
    c.save_native_recovery(&path).unwrap();
    let mut restored = NativeRecoveryPlan::open(path)
        .unwrap()
        .resume(worker())
        .unwrap();
    assert_eq!(restored.frame(), c.frame());
    assert_eq!(seed.frame(), c.frame());
    for _ in 0..90 {
        let reference = c.advance_in_field(g, &[]).unwrap();
        assert_eq!(restored.advance_in_field(g, &[]).unwrap(), reference);
        assert_eq!(seed.advance_in_field(g, &[]).unwrap(), reference);
    }
}

#[test]
fn erased_highest_runtime_id_remains_fenced_after_both_durable_restart_formats() {
    let [a, b]: [Body; 2] = fixture().try_into().unwrap();
    let seeds = [vec![a], vec![b.clone()]];
    let mut c = group(RUN, &seeds, 128);
    let new = Body {
        id: 100,
        position: [9., 10., 0.],
        ..b
    };
    c.insert_body(new.clone()).unwrap();
    let state = gz_physics::BodyState {
        position: [2., 12., 0.],
        orientation: [0., 0., 0., 1.],
        velocity: [0.1, -0., 0.],
        angular_velocity: [0., 0., 0.25],
    };
    c.replace_state(100, state).unwrap();
    c.queue_impulse(100, [1., 0., 0.]).unwrap();
    c.advance_routed().unwrap();
    c.retire(100).unwrap();
    c.compact().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let seed = dir.path().join("birth.gzrec");
    let native = dir.path().join("birth.gznative");
    c.save_recovery(&seed).unwrap();
    c.save_native_recovery(&native).unwrap();
    let saved = c.frame().clone();
    drop(c);
    for via_seed in [true, false] {
        let mut restored = if via_seed {
            RecoveryPlan::open(&seed)
                .unwrap()
                .resume(|run, id, b| WorkerProcess::spawn(worker(), run, id, b))
                .unwrap()
        } else {
            NativeRecoveryPlan::open(&native)
                .unwrap()
                .resume(worker())
                .unwrap()
        };
        for id in [2, 39, 100] {
            assert!(restored.insert_body(Body { id, ..new.clone() }).is_err());
            assert_eq!(restored.frame(), &saved);
        }
        restored
            .insert_body(Body {
                id: 101,
                ..new.clone()
            })
            .unwrap();
        restored
            .recover(restored.frame().body(101).unwrap().0)
            .unwrap();
        restored.advance_routed().unwrap();
        for mut p in restored.into_peers() {
            p.shutdown().unwrap();
        }
    }
}
