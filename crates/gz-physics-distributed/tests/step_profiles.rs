// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{BodyWrench, Island, SimulationStep, Snapshot, fixture},
    transport::WorkerProcess,
};
use std::{path::Path, time::Duration};

const STEPS: [SimulationStep; 4] = [
    SimulationStep::Ms1,
    SimulationStep::Ms2,
    SimulationStep::Ms5,
    SimulationStep::Ms10,
];

#[test]
fn supported_intervals_bind_native_caches_for_a_glancing_contact_and_queued_input() {
    for step in STEPS {
        let h = step.duration().as_secs_f64();
        let mut boxes = fixture();
        boxes[0] = boxes[0].promoted();
        boxes[0].position = [0., 0.09, 0.];
        boxes[1].position = [0.63, 0., 0.];
        let seed = Snapshot {
            step,
            bodies: boxes,
            ..Snapshot::default()
        };
        let mut local = Island::restore(&seed).unwrap();
        // Before the first native solve a warm copy must already know its physical interval.
        assert_eq!(
            Island::restore_warm(&local.checkpoint().unwrap())
                .unwrap()
                .snapshot(),
            seed
        );
        let force = BodyWrench {
            id: 1,
            force: [0., 2., 0.],
            torque: [0.; 3],
        };
        local.step_with_wrenches(&[force]).unwrap();
        let b = &local.snapshot().bodies[0];
        assert!((b.linear[1] - 2. * h).abs() < 1e-14);
        assert!((b.position[1] - (0.09 + 0.625 * 2. * h * h)).abs() < 1e-14);
        for _ in 0..100 {
            if local
                .snapshot()
                .impulses
                .iter()
                .any(|p| p.normal_impulse > 0.)
            {
                break;
            }
            local.step().unwrap();
        }
        assert!(
            local
                .snapshot()
                .impulses
                .iter()
                .any(|p| p.normal_impulse > 0.)
        );
        local.queue_impulse(2, [0., 0., 0.125]).unwrap();
        let captured = local.checkpoint().unwrap();
        assert!(captured.warmstart_normal_impulses().any(|j| j > 0.));
        let mut resumed = Island::restore_warm(&captured).unwrap();
        for _ in 0..12 {
            local.step().unwrap();
            resumed.step().unwrap();
            assert_eq!(resumed.snapshot(), local.snapshot());
        }
    }
    for invalid in [
        Duration::ZERO,
        Duration::from_micros(1001),
        Duration::from_millis(3),
    ] {
        assert_eq!(SimulationStep::from_duration(invalid), None);
    }
}

#[test]
fn empty_peer_journals_and_native_recovery_preserve_each_profile() {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spawn =
        |run, id, bodies: &[_], step| WorkerProcess::spawn_with_step(worker, run, id, bodies, step);
    let run = u64::from(std::process::id());
    let bodies = fixture();
    let wrong = [
        spawn(run, 0, &bodies, SimulationStep::Ms2).unwrap(),
        spawn(run, 1, &[], SimulationStep::Ms5).unwrap(),
    ];
    assert!(matches!(
        Coordinator::new(run, wrong, 64),
        Err(gz_physics_distributed::coordinator::Error::Invalid(
            "worker solver intervals disagree"
        ))
    ));
    for step in STEPS {
        let peers = [
            spawn(run, 0, &bodies, step).unwrap(),
            spawn(run, 1, &[], step).unwrap(),
        ];
        let mut world = Coordinator::new_recorded(run, peers, 64).unwrap();
        for _ in 0..3 {
            world.advance().unwrap();
        }
        let tmp = tempfile::tempdir().unwrap();
        let journal = tmp.path().join("actions");
        let native = tmp.path().join("native");
        world.save_recovery(&journal).unwrap();
        world.save_native_recovery(&native).unwrap();
        let expected = world.advance().unwrap().clone();
        for mut p in world.into_peers() {
            p.shutdown().unwrap();
        }
        let saved = NativeRecoveryPlan::open(&native).unwrap();
        assert_eq!(saved.frame().step(), step);
        let mut fast = saved.resume(worker).unwrap();
        assert_eq!(fast.advance().unwrap(), &expected);
        for mut p in fast.into_peers() {
            p.shutdown().unwrap();
        }
        if step != SimulationStep::Ms10 {
            let wrong = RecoveryPlan::open(&journal)
                .unwrap()
                .resume(|run, id, bodies| WorkerProcess::spawn(worker, run, id, bodies));
            assert!(matches!(
                wrong,
                Err(gz_physics_distributed::coordinator::RecoveryError::Format(
                    "replacement workers use a different solver interval"
                ))
            ));
        }
        let plan = RecoveryPlan::open(&journal).unwrap();
        assert_eq!(plan.step(), step);
        let mut replay = plan.resume_with_step(spawn).unwrap();
        assert_eq!(replay.advance().unwrap(), &expected);
    }
}
