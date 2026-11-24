// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{Body, GroundPlane, SimulationStep, Tier, fixture},
    transport::WorkerProcess,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::path::Path;

#[test]
fn independent_ground_reactions_survive_loaded_transfer_and_native_restart() {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let run = u64::from(std::process::id());
    let ground = GroundPlane::new(0.25).unwrap();
    let step = SimulationStep::Ms5;
    let h = step.duration();
    let gravity = [0., 0., -9.81];
    let prototype = fixture()[0].promoted();
    let boxes: Vec<_> = (0..3)
        .map(|id| Body {
            id,
            tier: Tier::Dynamic,
            mass: (1 << id) as f64,
            size: [0.4; 3],
            position: [(id as f64 - 1.) * 12., 0., 1.05],
            linear: [0.3, 0., 0.],
            ..prototype.clone()
        })
        .collect();
    let mut native = MultiFidelityBackend::new(gravity).unwrap();
    native.set_partition_step(h).unwrap();
    native.set_partition_ground(ground.height()).unwrap();
    let handles: Vec<_> = boxes
        .iter()
        .map(|b| {
            native
                .spawn_awake_frictionless_box(
                    BodySpec {
                        position: b.position,
                        velocity: b.linear,
                        mass: Some(b.mass),
                        shape: Shape::Box {
                            half_extents: [0.2; 3],
                        },
                    },
                    RotationSpec::default(),
                    false,
                )
                .unwrap()
        })
        .collect();
    let peers = [
        WorkerProcess::spawn_with_ground(worker, run, 0, &boxes[..2], step, Some(ground)).unwrap(),
        WorkerProcess::spawn_with_ground(worker, run, 1, &boxes[2..], step, Some(ground)).unwrap(),
    ];
    let mut solver = Coordinator::new_recorded(run, peers, 1024).unwrap();
    let temp = tempfile::tempdir().unwrap();
    let mut first = None;
    let mut max_rest_error = [0.0_f64; 3];
    for tick in 1..=240 {
        if tick == 151 {
            let before = solver.frame().clone();
            assert_eq!(
                before
                    .workers
                    .iter()
                    .map(|w| w.snapshot.supports.len())
                    .collect::<Vec<_>>(),
                [2, 1]
            );
            solver.transfer_component(0).unwrap();
            let after = solver.frame();
            assert_eq!(
                after
                    .workers
                    .iter()
                    .map(|w| w.snapshot.supports.len())
                    .collect::<Vec<_>>(),
                [1, 2]
            );
            for b in &boxes {
                let contact = |f: &gz_physics_distributed::coordinator::Frame| {
                    f.workers
                        .iter()
                        .flat_map(|w| &w.snapshot.supports)
                        .find(|p| p.body == b.id)
                        .unwrap()
                        .normal_impulse
                };
                assert_eq!(contact(&before).to_bits(), contact(after).to_bits());
                assert!(contact(after) > 0.);
            }
            assert_ne!(before.body(0).unwrap().0, after.body(0).unwrap().0);
        }
        if tick == 181 {
            let checkpoint = temp.path().join("supported");
            solver.save_native_recovery(&checkpoint).unwrap();
            let journal = temp.path().join("supported-actions");
            solver.save_recovery(&journal).unwrap();
            let before = solver.frame().clone();
            for mut w in solver.into_peers() {
                w.shutdown().unwrap();
            }
            let replay = RecoveryPlan::open(&journal).unwrap();
            assert_eq!(replay.ground(), Some(ground));
            let replay = replay
                .resume_with_ground(|run, id, bodies, step, ground| {
                    WorkerProcess::spawn_with_ground(worker, run, id, bodies, step, ground)
                })
                .unwrap();
            assert_eq!(replay.frame(), &before);
            for mut w in replay.into_peers() {
                w.shutdown().unwrap();
            }
            solver = NativeRecoveryPlan::open(checkpoint)
                .unwrap()
                .resume(worker)
                .unwrap();
            assert_eq!(solver.frame(), &before);
        }
        let f = solver.advance_in_field(gravity, &[]).unwrap();
        native.step(h).unwrap();
        let supports = native.partition_ground_contacts();
        for (b, id) in boxes.iter().zip(&handles) {
            let actual = f.body(b.id).unwrap().1;
            let reference = native.state(*id).unwrap();
            assert_eq!(
                actual.position.map(f64::to_bits),
                reference.position.map(f64::to_bits),
                "position for {} at tick {tick}",
                b.id
            );
            assert_eq!(
                actual.linear.map(f64::to_bits),
                reference.velocity.map(f64::to_bits)
            );
            assert_eq!(
                actual.orientation.map(f64::to_bits),
                reference.orientation.map(f64::to_bits)
            );
            assert_eq!(
                actual.angular.map(f64::to_bits),
                reference.angular_velocity.map(f64::to_bits)
            );
            let measured = f
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.supports)
                .find(|p| p.body == b.id);
            let expected = supports.iter().find(|p| p.body == *id && p.active);
            assert_eq!(
                measured.map(|p| p.normal_impulse.to_bits()),
                expected.map(|p| p.normal_impulse.to_bits())
            );
            if measured.is_some_and(|p| p.normal_impulse > 0.) {
                first.get_or_insert(tick);
            }
            if tick >= 145 {
                assert!(
                    (actual.position[2] - 0.45).abs() < 0.002,
                    "z at {tick}: {:?}",
                    actual.position
                );
                assert!((actual.linear[0] - 0.3).abs() < 1e-12);
                let impulse = measured.unwrap().normal_impulse;
                let errors = [
                    (actual.position[2] - 0.45).abs(),
                    (actual.linear[0] - 0.3).abs(),
                    (impulse - b.mass * 9.81 * h.as_secs_f64()).abs(),
                ];
                for (max, value) in max_rest_error.iter_mut().zip(errors) {
                    *max = max.max(value);
                }
                assert!(
                    (impulse - b.mass * 9.81 * h.as_secs_f64()).abs() < 1e-5,
                    "support {} at {tick}, mass {}",
                    impulse,
                    b.mass
                );
            }
        }
        assert!(f.workers.iter().all(|w| w.snapshot.impulses.is_empty()));
    }
    println!("settled maximum height, tangential speed and support errors {max_rest_error:?}");
    let impact = first.unwrap() as f64 * h.as_secs_f64();
    let ideal = (2_f64 * (1.05 - 0.2 - 0.25) / 9.81).sqrt();
    assert!((impact - ideal).abs() < 2. * h.as_secs_f64());
    println!(
        "support onset {impact}s, analytic {ideal}s, final {:?}",
        solver
            .frame()
            .workers
            .each_ref()
            .map(|w| &w.snapshot.supports)
    );
}

#[test]
fn two_grounded_components_join_before_a_real_box_collision() {
    let step = SimulationStep::Ms5;
    let ground = GroundPlane::new(0.1).unwrap();
    let gravity = [0., 0., -9.81];
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let run = u64::from(std::process::id()) + 1;
    let bodies: Vec<_> = [-1., 1.]
        .into_iter()
        .enumerate()
        .map(|(i, sign)| Body {
            id: i as u64,
            mass: (i + 1) as f64,
            size: [0.4; 3],
            position: [3.2 * sign, 0., 0.35],
            linear: [-1.5 * sign, 0., 0.],
            ..fixture()[0].promoted()
        })
        .collect();
    let mut native = MultiFidelityBackend::new(gravity).unwrap();
    native.set_partition_step(step.duration()).unwrap();
    native.set_partition_ground(ground.height()).unwrap();
    let ids = bodies
        .iter()
        .map(|b| {
            native
                .spawn_awake_frictionless_box(
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: [0.2; 3],
                        },
                        position: b.position,
                        velocity: b.linear,
                        mass: Some(b.mass),
                    },
                    RotationSpec::default(),
                    false,
                )
                .unwrap()
        })
        .collect::<Vec<_>>();
    let peers = [0, 1].map(|i| {
        WorkerProcess::spawn_with_ground(worker, run, i as u8, &bodies[i..=i], step, Some(ground))
            .unwrap()
    });
    let mut sim = Coordinator::new(run, peers, 2048).unwrap();
    let mut merge = None;
    let mut impact = None;
    for tick in 1..=470 {
        if tick == 51 {
            let f = sim.frame();
            assert!(
                f.workers.iter().all(|w| w.snapshot.supports.len() == 1
                    && w.snapshot.supports[0].normal_impulse > 0.)
            );
            sim.transfer_component(1).unwrap();
            assert_eq!(sim.frame().workers[0].components.len(), 2);
        }
        let f = sim.advance_in_field(gravity, &[]).unwrap();
        native.step(step.duration()).unwrap();
        let supports = native.partition_ground_contacts();
        let expected_contacts = native.rigid_contact_observations();
        for (b, id) in bodies.iter().zip(&ids) {
            let a = f.body(b.id).unwrap().1;
            let r = native.state(*id).unwrap();
            assert_eq!(
                a.position.map(f64::to_bits),
                r.position.map(f64::to_bits),
                "position tick {tick} body {}",
                b.id
            );
            assert_eq!(
                a.orientation.map(f64::to_bits),
                r.orientation.map(f64::to_bits),
                "orientation tick {tick}"
            );
            assert_eq!(
                a.linear.map(f64::to_bits),
                r.velocity.map(f64::to_bits),
                "velocity tick {tick}"
            );
            assert_eq!(
                a.angular.map(f64::to_bits),
                r.angular_velocity.map(f64::to_bits),
                "spin tick {tick}"
            );
            let observed = f
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.supports)
                .find(|p| p.body == b.id);
            let expected = supports.iter().find(|p| p.body == *id && p.active);
            assert_eq!(
                observed.map(|p| p.normal_impulse.to_bits()),
                expected.map(|p| p.normal_impulse.to_bits()),
                "support tick {tick}"
            );
        }
        let j = f.workers.iter().flat_map(|w| &w.snapshot.impulses).next();
        assert_eq!(
            j.map(|p| p.normal_impulse.to_bits()),
            expected_contacts
                .iter()
                .find(|p| p.active)
                .map(|p| p.normal_impulse.to_bits()),
            "mobile impulse tick {tick}"
        );
        if f.workers[0].components.iter().any(|c| c.ids == [0, 1]) {
            merge.get_or_insert(tick);
        }
        if j.is_some_and(|p| p.normal_impulse > 0.) {
            impact.get_or_insert(tick);
        }
    }
    assert!(merge.unwrap() > 51 && merge.unwrap() < impact.unwrap());
    let momentum: f64 = sim
        .frame()
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .map(|b| b.mass * b.linear[0])
        .sum();
    assert!((momentum + 1.5).abs() < 1e-10);
    println!(
        "shared-plane warm join at tick {}, first box/box impulse tick {}, final horizontal momentum {momentum}",
        merge.unwrap(),
        impact.unwrap()
    );
}

#[test]
fn heterogeneous_surface_digests_and_ordered_inputs_survive_both_durable_recovery_paths() {
    let run = 987300;
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let step = SimulationStep::Ms5;
    let ground = GroundPlane::with_friction(0.2, 0.25).unwrap();
    let boxes = [
        Body {
            id: 4,
            position: [-7., 0., 0.48],
            size: [0.4; 3],
            linear: [2., 0.2, 0.],
            mass: 1.,
            friction: 0.5,
            ..fixture()[0].promoted()
        },
        Body {
            id: 8,
            position: [7., 0., 0.48],
            size: [0.4; 3],
            linear: [-1., 0.1, 0.],
            angular: [0., 0., 0.1],
            mass: 2.,
            friction: 0.09,
            ..fixture()[0].promoted()
        },
    ];
    let spawn = |ground| {
        [0, 1].map(|i| {
            WorkerProcess::spawn_with_ground(
                worker,
                run,
                i as u8,
                &boxes[i..=i],
                step,
                Some(ground),
            )
            .unwrap()
        })
    };
    let mut sim = Coordinator::new_recorded(run, spawn(ground), 1024).unwrap();
    let gravity = [0., 0., -9.81];
    for _ in 0..65 {
        sim.advance_in_field_overlapped(gravity, &[]).unwrap();
    }
    for id in [8, 4, 4, 8] {
        sim.queue_impulse(id, [0.02, -0., 0.01]).unwrap();
    }
    sim.transfer_component(4).unwrap();
    assert_eq!(sim.frame().workers[1].snapshot.supports.len(), 2);
    let tmp = tempfile::tempdir().unwrap();
    let actions = tmp.path().join("actions");
    let image = tmp.path().join("native");
    sim.save_recovery(&actions).unwrap();
    sim.save_native_recovery(&image).unwrap();
    let plan = RecoveryPlan::open(actions).unwrap();
    assert_eq!(plan.ground(), Some(ground));
    let mut replay = plan
        .resume_with_ground(|run, id, bodies, dt, g| {
            WorkerProcess::spawn_with_ground(worker, run, id, bodies, dt, g)
        })
        .unwrap();
    let mut restored = NativeRecoveryPlan::open(image)
        .unwrap()
        .resume(worker)
        .unwrap();
    assert_eq!(sim.frame(), restored.frame());
    assert_eq!(sim.frame(), replay.frame());
    for _ in 0..115 {
        let f = sim.advance_in_field_overlapped(gravity, &[]).unwrap();
        assert_eq!(f, restored.advance_in_field(gravity, &[]).unwrap());
        assert_eq!(f, replay.advance_in_field_overlapped(gravity, &[]).unwrap());
    }
    for c in [sim, restored, replay] {
        for mut p in c.into_peers() {
            p.shutdown().unwrap();
        }
    }
}
