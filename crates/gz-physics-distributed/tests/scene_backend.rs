// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, Island, STEP, Snapshot, Tier, fixture},
};
use std::{path::Path, time::Duration};
fn engine() -> SceneBackend {
    SceneBackend::new(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        [0.; 3],
        STEP,
    )
    .unwrap()
}
fn box_at(e: &mut SceneBackend, b: &Body) -> BodyId {
    let s = BodySpec {
        shape: Shape::Box {
            half_extents: b.size.map(|v| v / 2.),
        },
        mass: Some(b.mass),
        position: b.position,
        velocity: b.linear,
    };
    let r = RotationSpec {
        orientation: b.orientation,
        angular_velocity: b.angular,
    };
    if b.tier == Tier::Prescribed {
        e.spawn_kinematic_link(s, r, None).unwrap()
    } else {
        e.spawn_link(s, r, None).unwrap()
    }
}
#[test]
fn normal_backend_steps_two_real_workers_matches_contact_reset_and_surviving_edits() {
    let mut bodies = fixture();
    bodies[0].id = 0;
    bodies[0].tier = Tier::Dynamic;
    bodies[1].id = 1;
    let mut e = engine();
    let a = box_at(&mut e, &bodies[0]);
    let b = box_at(&mut e, &bodies[1]);
    let mut native = Island::restore(&Snapshot {
        bodies: bodies.clone(),
        ..Default::default()
    })
    .unwrap();
    let initial = e.initial_state();
    assert!(e.frame().is_none());
    e.step(Duration::ZERO).unwrap();
    assert!(e.frame().is_none());
    e.initialize().unwrap();
    assert_ne!(
        e.frame().unwrap().body(0).unwrap().0,
        e.frame().unwrap().body(1).unwrap().0
    );
    let mut first = None;
    for tick in 1..=145 {
        if tick == 90 {
            e.add_world_wrench(b, [2., 0., 0.], [0.; 3]).unwrap();

            e.step(Duration::ZERO).unwrap();
        }
        e.step(STEP).unwrap();
        if tick == 90 {
            native
                .step_with_wrenches(&[gz_physics_distributed::physics::BodyWrench {
                    id: 1,
                    force: [2., 0., 0.],
                    torque: [0.; 3],
                }])
                .unwrap();
        } else {
            native.step().unwrap();
        }
        for body in &native.snapshot().bodies {
            assert_eq!(e.frame().unwrap().body(body.id).unwrap().1, body);
        }
        if native.snapshot().contacts > 0 {
            first.get_or_insert(tick);
        }
    }
    assert!(first.is_some());
    let prepared = e.prepare_initial(&initial).unwrap();
    assert!(e.remove(a));
    assert!(e.frame().unwrap().body(0).is_none());
    assert_eq!(
        e.frame().unwrap().body(1).unwrap().1,
        &native.snapshot().bodies[1]
    );
    e.set_world_velocity(b, [-0.1, 0.2, 0.], [0., 0., 0.3])
        .unwrap();
    assert_eq!(e.state(b).unwrap().velocity, [-0.1, 0.2, 0.]);
    e.step(STEP).unwrap();
    assert!(e.remove(b));
    e.step(STEP).unwrap();
    assert!(!e.remove(b));
    e.apply_initial(prepared).unwrap();
    assert_eq!(e.frame().unwrap().tick, 0);
    assert_eq!(e.frame().unwrap().body(0).unwrap().1, &bodies[0]);
    assert_eq!(e.frame().unwrap().body(1).unwrap().1, &bodies[1]);
    assert!(e.step(Duration::from_millis(1)).is_err());
    assert_eq!(e.frame().unwrap().tick, 0);
}
#[test]
fn prestart_inputs_survive_initialization_but_reset_restores_construction_and_rejects_foreign_snapshots()
 {
    let mut e = engine();
    let mut b = fixture()[1].clone();
    b.linear = [0.; 3];
    b.mass = 2.;
    let id = box_at(&mut e, &b);
    let captured = e.initial_state();
    e.set_world_velocity(id, [1., 0., 0.], [0.; 3]).unwrap();
    e.apply_impulse(id, [2., 0., 0.], [0.; 3]).unwrap();
    assert_eq!(e.state(id).unwrap().velocity, [2., 0., 0.]);
    e.add_world_wrench(id, [4., 0., 0.], [0.; 3]).unwrap();
    e.initialize().unwrap();
    assert_eq!(e.state(id).unwrap().velocity, [2., 0., 0.]);
    e.step(STEP).unwrap();
    assert!((e.state(id).unwrap().velocity[0] - 2.02).abs() < 1e-14);
    e.step(STEP).unwrap();
    assert!((e.state(id).unwrap().velocity[0] - 2.02).abs() < 1e-14);
    e.reset().unwrap();
    assert_eq!(e.state(id).unwrap().velocity, [0.; 3]);
    let mut other = engine();
    assert!(!other.owns_initial_state(&captured));
    assert!(matches!(
        other.prepare_initial(&captured),
        Err(PhysicsError::ForeignInitialState)
    ));
    assert!(matches!(
        other.apply_initial(e.prepare_initial(&captured).unwrap()),
        Err(PhysicsError::ForeignInitialState)
    ));
}
#[test]
fn scene_admission_rejects_unrepresented_physics_before_allocating_an_id() {
    let bin = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    assert!(SceneBackend::new(bin, [0., 0., f64::NAN], STEP).is_err());
    assert!(SceneBackend::new(bin, [0.; 3], Duration::ZERO).is_err());
    let mut e = engine();
    let s = BodySpec {
        shape: Shape::Box {
            half_extents: [0.5; 3],
        },
        mass: Some(1.),
        position: [0.; 3],
        velocity: [0.; 3],
    };
    let r = RotationSpec::default();
    let uniform = Inertia {
        center: [0.; 3],
        orientation: [0., 0., 0., 1.],
        tensor: [1. / 6., 1. / 6., 1. / 6., 0., 0., 0.],
    };
    assert!(
        e.spawn_link(
            s,
            r,
            Some(Inertia {
                center: [0.1, 0., 0.],
                ..uniform
            })
        )
        .is_err()
    );
    assert!(
        e.spawn_link(
            BodySpec {
                shape: Shape::Cylinder {
                    radius: 0.5,
                    half_height: 0.
                },
                ..s
            },
            r,
            None
        )
        .is_err()
    );
    assert!(
        e.spawn_link(
            BodySpec {
                mass: None,
                velocity: [1., 0., 0.],
                ..s
            },
            r,
            None
        )
        .is_err()
    );
    let id = e.spawn_link(s, r, Some(uniform)).unwrap();
    assert_eq!(id.raw(), 0);
    assert!(e.spawn_link(s, r, None).is_err());
    e.apply_impulse(id, [0.; 3], [0., 0., 1.]).unwrap();
    assert!((e.state(id).unwrap().angular_velocity[2] - 6.).abs() < 1e-14);
    assert!(e.add_world_wrench(id, [0.; 3], [f64::NAN, 0., 0.]).is_err());
    assert!(e.remove(id));
    let next = box_at(&mut e, &fixture()[0]);
    assert_eq!(next.raw(), 1);
    assert!(e.apply_impulse(next, [1., 0., 0.], [0.; 3]).is_err());
    assert!(e.set_world_velocity(next, [0.; 3], [f64::MAX; 3]).is_err());
}

#[cfg(unix)]
#[test]
fn scene_replaces_exited_workers_across_routes_loaded_contact_held_force_and_compaction() {
    use gz_physics_distributed::physics::{BodyWrench, PendingImpulse};
    use std::process::Command;
    let mut bodies = fixture();
    for (i, b) in bodies.iter_mut().enumerate() {
        b.id = i as u64;
        b.tier = Tier::Dynamic;
    }
    let mut native = Island::restore(&Snapshot {
        bodies: bodies.clone(),
        ..Default::default()
    })
    .unwrap();
    let mut e = engine();
    let ids: Vec<_> = bodies.iter().map(|b| box_at(&mut e, b)).collect();
    assert!(e.worker_pids().is_none());
    e.initialize().unwrap();
    let kill = |pid: u32| {
        assert!(
            Command::new("kill")
                .args(["-KILL", &pid.to_string()])
                .status()
                .unwrap()
                .success()
        );
    };
    let mut contact = false;
    for tick in 1..=940 {
        let owner = usize::from(e.frame().unwrap().body(0).unwrap().0);
        let previous = e.worker_pids().unwrap();
        let replace = match tick {
            1 => Some(1),             // both steps dispatched at the first barrier
            105 | 936 => Some(owner), // a loaded solver, then a compacted one
            180 => Some(1 - owner),   // the empty peer dies after both inputs are accepted
            _ => None,
        };
        let inputs = if tick == 105 || tick == 180 {
            let p = PendingImpulse {
                id: 0,
                impulse: [0.02, 0., 0.],
            };
            e.queue_linear_impulse(ids[0], p.impulse).unwrap();
            native.queue_impulse(p.id, p.impulse).unwrap();
            let w = BodyWrench {
                id: 1,
                force: [-0.3, 0.1, 0.],
                torque: [0., 0., 0.013],
            };
            e.add_world_wrench(ids[1], w.force, w.torque).unwrap();
            vec![w]
        } else {
            Vec::new()
        };
        if let Some(i) = replace {
            kill(previous[i]);
        }
        e.step(STEP).unwrap();
        native.step_with_wrenches(&inputs).unwrap();
        if let Some(i) = replace {
            let now = e.worker_pids().unwrap();
            assert_ne!(now[i], previous[i]);
            assert_eq!(now[1 - i], previous[1 - i]);
        }
        assert_eq!(e.frame().unwrap().tick, tick);
        let expected = native.snapshot();
        for b in &expected.bodies {
            assert_eq!(e.frame().unwrap().body(b.id).unwrap().1, b, "tick {tick}");
        }
        let impulses: Vec<_> = e
            .frame()
            .unwrap()
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter().cloned())
            .collect();
        assert_eq!(impulses, expected.impulses);
        contact |= !impulses.is_empty();
    }
    assert!(contact);
    // Retirement reaches both peers, including a previous owner's generation fence.
    let previous = e.worker_pids().unwrap();
    kill(previous[1]);
    assert!(e.remove(ids[0]));
    assert!(e.state(ids[0]).is_none());
    assert_eq!(e.worker_pids().unwrap()[0], previous[0]);
    e.step(STEP).unwrap();
}

#[test]
fn immediate_angular_impulse_uses_rotated_native_inertia_at_each_barrier() {
    use gz_physics_multifidelity::MultiFidelityBackend;
    let mut b = fixture()[1].clone();
    b.size = [0.3, 0.9, 0.6];
    b.position = [0.; 3];
    b.linear = [0.22, -0.04, 0.1];
    b.angular = [1.4, -0.9, 3.1];
    let half = 0.43_f64;
    b.orientation = [0., half.sin(), 0., half.cos()];
    let mut e = engine();
    let scene = box_at(&mut e, &b);
    let mut native = MultiFidelityBackend::new([0.; 3]).unwrap();
    let reference = native
        .spawn_awake_frictionless_box(
            BodySpec {
                position: b.position,
                velocity: b.linear,
                mass: Some(b.mass),
                shape: Shape::Box {
                    half_extents: b.size.map(|s| s * 0.5),
                },
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            false,
        )
        .unwrap();
    for tick in 0..60 {
        if tick % 13 == 0 {
            let (j, l) = ([0.14, 0.19, -0.21], [-0.043, 0.026, 0.081]);
            e.apply_impulse(scene, j, l).unwrap();
            native.apply_impulse(reference, j, l).unwrap();
            assert_eq!(e.state(scene), native.state(reference), "edit at {tick}");
        }
        e.step(STEP).unwrap();
        native.step(STEP).unwrap();
        assert_eq!(e.state(scene), native.state(reference), "step {tick}");
    }
    let last = e.state(scene);
    assert!(e.apply_impulse(scene, [0.; 3], [f64::MAX; 3]).is_err());
    assert_eq!(e.state(scene), last);
    e.step(STEP).unwrap();
    native.step(STEP).unwrap();
    assert_eq!(e.state(scene), native.state(reference));
}

#[test]
fn creation_and_motion_edits_cannot_strand_an_oversized_domain_at_the_next_barrier() {
    let seed = fixture()[0].promoted();
    let build = || {
        let mut e = engine();
        for i in 0..32 {
            box_at(
                &mut e,
                &Body {
                    position: [i as f64 * 0.70, 0., 0.],
                    linear: [0.; 3],
                    ..seed.clone()
                },
            );
        }
        e.initialize().unwrap();
        e.step(STEP).unwrap();
        e
    };
    let mut e = build();
    let mut control = build();
    let last = 31. * 0.70;
    let spec = |x| BodySpec {
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Box {
            half_extents: [0.3; 3],
        },
    };
    let before = e.frame().unwrap().clone();
    assert!(
        e.spawn_link(spec(last + 0.70), RotationSpec::default(), None)
            .is_err()
    );
    assert_eq!(e.frame().unwrap(), &before);
    let id = e
        .spawn_link(spec(last + 6.2), RotationSpec::default(), None)
        .unwrap();
    assert_eq!(id.raw(), 32);
    assert_eq!(
        id,
        control
            .spawn_link(spec(last + 6.2), RotationSpec::default(), None)
            .unwrap()
    );
    let state = e.state(id).unwrap();
    for s in [&mut e, &mut control] {
        s.add_world_wrench(id, [0., 0.3, 0.], [0., 0.01, 0.])
            .unwrap();
        s.queue_linear_impulse(id, [0., 0., 0.007]).unwrap();
    }
    let before = e.frame().unwrap().clone();
    let edit = gz_physics::BodyState {
        position: [last + 0.70, 0., 0.],
        ..state
    };
    assert!(e.validate_body_state(id, edit).is_err());
    assert!(e.set_body_state(id, edit).is_err());
    assert!(e.set_world_velocity(id, [-200., 0., 0.], [0.; 3]).is_err());
    let pids = e.worker_pids().unwrap();
    assert!(e.apply_impulse(id, [-200., 0., 0.], [0., 0., 1.]).is_err());
    assert_ne!(e.worker_pids().unwrap(), pids);
    assert_eq!(e.frame().unwrap(), &before);
    for _ in 0..5 {
        e.step(STEP).unwrap();
        control.step(STEP).unwrap();
        assert_eq!(e.frame(), control.frame());
    }
}

#[test]
fn loading_refuses_a_cross_worker_oversized_chain_before_publication() {
    let mut e = engine();
    let seed = fixture()[0].promoted();
    for i in 0..33 {
        box_at(
            &mut e,
            &Body {
                position: [i as f64 * 0.70, 0., 0.],
                linear: [0.; 3],
                ..seed.clone()
            },
        );
    }
    assert!(e.initialize().is_err());
    assert!(e.frame().is_none());
    assert!(e.remove(BodyId::from_raw(32)));
    e.initialize().unwrap();
    e.step(STEP).unwrap();
}

#[test]
fn box_contact_forecast_reads_committed_scene_state_across_worker_widths() {
    for width in [2, 4] {
        let mut scene = engine().with_worker_count(width).unwrap();
        let mut bar = fixture()[0].promoted();
        bar.size = [2., 0.1, 0.1];
        bar.position = [0.; 3];
        bar.linear = [0.; 3];
        bar.angular = [0., 0., 8.];
        let mut other = bar.clone();
        other.size = [0.1; 3];
        other.position = [0.55, 0.13, 0.];
        other.angular = [0.; 3];
        let ground = scene
            .spawn_static_plane(gz_physics::PlaneSpec {
                position: [0., 0., -2.],
                orientation: [0., 0., 0., 1.],
                size: [5., 5.],
            })
            .unwrap();
        let a = box_at(&mut scene, &bar);
        let b = box_at(&mut scene, &other);
        assert!(scene.predicts_box_contact(a, b).unwrap());
        assert!(scene.predicts_box_contact(b, a).unwrap());
        assert!(matches!(
            scene.predicts_box_contact(a, a),
            Err(PhysicsError::InvalidValue)
        ));
        assert!(matches!(
            scene.predicts_box_contact(a, BodyId::from_raw(234)),
            Err(PhysicsError::MissingBody)
        ));
        assert!(matches!(
            scene.predicts_box_contact(a, ground),
            Err(PhysicsError::Unsupported { .. })
        ));
        scene.initialize().unwrap();
        let original = scene.state(a).unwrap();
        assert!(scene.predicts_box_contact(a, b).unwrap());
        let mut stopped = original;
        stopped.angular_velocity = [0.; 3];
        scene.set_body_state(a, stopped).unwrap();
        assert!(!scene.predicts_box_contact(a, b).unwrap());
        scene.reset().unwrap();
        assert!(scene.predicts_box_contact(a, b).unwrap());
    }
}

#[test]
fn scene_step_confirms_an_ambiguous_reply_with_held_input_exactly_once() {
    let dt = Duration::from_millis(10);
    for width in [2, 4] {
        for policy in 0..3 {
            let make = || {
                let engine = SceneBackend::new(
                    Path::new(env!("CARGO_BIN_EXE_physics_worker")),
                    [0., 0., -1.],
                    dt,
                )
                .unwrap()
                .with_worker_count(width)
                .unwrap();
                match policy {
                    0 => engine,
                    1 => engine.with_adaptive_contacts().unwrap(),
                    _ => engine.with_island_refinement().unwrap(),
                }
            };
            let mut actual = make();
            let mut control = make();
            assert!(actual.drop_next_acknowledgement(0).is_err());
            let mut input = None;
            for scene in [&mut actual, &mut control] {
                let mut b = fixture()[1].clone();
                b.tier = Tier::Dynamic;
                b.position = [0., 0., 3.];
                let first = box_at(scene, &b);
                b.position[0] = 10.;
                box_at(scene, &b);
                scene.initialize().unwrap();
                scene
                    .apply_impulse(first, [0.1, -0.4, 0.], [0.; 3])
                    .unwrap();
                scene
                    .add_world_wrench(first, [8., 1., 3.], [0., 0., 0.2])
                    .unwrap();
                input = Some(first);
            }
            assert!(actual.drop_next_acknowledgement(width).is_err());
            let pids = actual.local_worker_pids().unwrap();
            actual.drop_next_acknowledgement(0).unwrap();
            actual.drop_next_acknowledgement(1).unwrap();
            for _ in 0..3 {
                actual.step(dt).unwrap();
                control.step(dt).unwrap();
                let id = input.unwrap();
                assert_eq!(actual.state(id), control.state(id));
                assert_eq!(actual.frame(), control.frame());
                assert_eq!(actual.pool_frame(), control.pool_frame());
            }
            assert_eq!(actual.local_worker_pids().unwrap(), pids);
        }
    }
}
