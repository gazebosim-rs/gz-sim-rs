// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ports of gz-physics-rapier/tests/private/{ids,initial,joints}.rs.
use super::*;
use gz_physics::{PhysicsBackend, RestoredPhysics, Shape, install};
use gz_sim_core::Simulation;

fn sphere(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [1.0, 0.0, 0.0],
        mass: Some(2.0),
    }
}

fn dart(sim: &mut Simulation) -> &mut DartBackend {
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .into_inner()
        .downcast_mut()
        .unwrap()
}

fn spawn(backend: &mut DartBackend, spec: BodySpec) -> BodyId {
    backend
        .spawn_link(spec, RotationSpec::default(), None)
        .unwrap()
}

#[test]
fn exhausted_ids_do_not_insert_a_body() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    backend.next_body = u64::MAX;
    assert_eq!(
        backend.spawn_link(sphere([0.0; 3]), RotationSpec::default(), None),
        Err(PhysicsError::IdOverflow)
    );
    assert!(backend.world.bodies.is_empty());
}

#[test]
fn prepared_reset_restores_removed_initial_bodies_and_retires_later_allocations() {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let initial_body = backend
        .spawn_link(
            sphere([0.0, 0.0, 5.0]),
            RotationSpec {
                orientation: [0.0, 0.0, 0.5_f64.sin(), 0.5_f64.cos()],
                angular_velocity: [0.0, 0.0, 2.0],
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.2, 0.3, 0.4, 0.02, 0.0, 0.0],
            }),
        )
        .unwrap();
    let state = backend.state(initial_body).unwrap();
    let initial = backend.initial_state();
    backend.step(Duration::from_millis(10)).unwrap();
    assert_ne!(backend.state(initial_body), Some(state));
    assert!(backend.remove(initial_body));
    let runtime = spawn(&mut backend, sphere([0.0, 0.0, 10.0]));
    let runtime_state = backend.state(runtime);
    let prepared = backend.prepare_initial(&initial).unwrap();
    assert_eq!(backend.state(initial_body), None);
    assert_eq!(backend.state(runtime), runtime_state);
    // Allocations made after preparation must also remain retired after commit.
    let later = spawn(&mut backend, sphere([0.0, 0.0, 20.0]));
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(initial_body), Some(state));
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(later), None);
    let fresh = spawn(&mut backend, sphere([0.0, 0.0, 30.0]));
    assert!(fresh > later);
    backend.step(Duration::from_millis(1)).unwrap();
    let next = backend.state(initial_body).unwrap();
    assert!(next.position[0] > state.position[0]);
    assert!(next.position[2] < state.position[2]);
    assert_ne!(next.orientation, state.orientation);
    backend
        .apply_initial(backend.prepare_initial(&initial.clone()).unwrap())
        .unwrap();
    assert_eq!(backend.state(initial_body), Some(state));
    assert_eq!(backend.state(fresh), None);
    assert!(spawn(&mut backend, sphere([0.0, 0.0, 40.0])) > fresh);
}

#[test]
fn foreign_initial_and_prepared_states_leave_both_backends_unchanged() {
    let mut first = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let original = spawn(&mut first, sphere([0.0, 0.0, 5.0]));
    let first_state = first.state(original);
    let initial = first.initial_state();
    let mut second = DartBackend::new([0.0, 1.0, 0.0]).unwrap();
    let other = spawn(&mut second, sphere([10.0, 0.0, 0.0]));
    let second_state = second.state(other);
    assert!(matches!(
        second.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
    let prepared = first.prepare_initial(&initial).unwrap();
    assert_eq!(
        second.apply_initial(prepared),
        Err(PhysicsError::ForeignInitialState)
    );
    assert_eq!(first.state(original), first_state);
    assert_eq!(second.state(other), second_state);
    assert_eq!(second.gravity, [0.0, 1.0, 0.0]);
    drop(first);
    assert!(matches!(
        second.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
}

#[test]
fn empty_and_sparse_initial_states_preserve_body_ids_and_high_water_mark() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let empty = backend.initial_state();
    let removed = spawn(&mut backend, sphere([0.0; 3]));
    let retained = spawn(&mut backend, sphere([0.0, 2.0, 0.0]));
    assert!(backend.remove(removed));
    let sparse = backend.initial_state();
    let retained_state = backend.state(retained);
    assert!(backend.remove(retained));
    backend
        .apply_initial(backend.prepare_initial(&sparse).unwrap())
        .unwrap();
    assert_eq!(backend.state(removed), None);
    assert_eq!(backend.state(retained), retained_state);
    backend
        .apply_initial(backend.prepare_initial(&empty).unwrap())
        .unwrap();
    assert!(backend.world.bodies.is_empty());
    assert_eq!(backend.state(retained), None);
    assert!(spawn(&mut backend, sphere([0.0; 3])) > retained);
}

#[test]
fn initial_and_retained_reset_keep_construction_gravity() {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = spawn(&mut backend, sphere([0.0; 3]));
    let initial = backend.initial_state();
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    backend.reset().unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    let velocity = backend.state(body).unwrap().velocity;
    assert!((velocity[0] - 1.0).abs() < 1e-12);
    assert!((velocity[2] + 0.00981).abs() < 1e-12);
}

#[test]
fn prepared_epoch_avoids_rebuilding_and_stale_epoch_does_not_skip_retained_reset() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let body = spawn(&mut backend, sphere([0.0; 3]));
    let initial = backend.initial_state();
    backend.step(Duration::from_millis(1)).unwrap();
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, backend).unwrap();
    let epoch = sim.validate_reset().unwrap();
    sim.world_mut().insert_resource(RestoredPhysics { epoch });
    dart(&mut sim)
        .apply_impulse(body, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let prepared = dart(&mut sim).state(body);
    sim.reset().unwrap();
    assert_eq!(dart(&mut sim).state(body), prepared);
    // Keeping an old marker cannot suppress the next epoch's retained-body reset.
    sim.reset().unwrap();
    assert_eq!(
        dart(&mut sim).state(body).unwrap().velocity,
        [1.0, 0.0, 0.0]
    );
}

#[test]
fn invalid_joint_construction_is_atomic_including_exhaustion_and_large_offsets() {
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 0.1 },
        position: [0.0, 0.0, -1.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let body = spawn(&mut backend, spec);
    let before = backend.state(body).unwrap();
    for (anchor, axis) in [
        ([f64::INFINITY, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0; 3], [0.0; 3]),
        ([0.0; 3], [f64::NAN, 0.0, 0.0]),
        ([0.0; 3], [1e308, 1e308, 0.0]),
        ([0.0; 3], [1e-300, 0.0, 0.0]),
    ] {
        assert_eq!(
            backend.add_world_revolute(body, anchor, axis),
            Err(PhysicsError::InvalidJoint)
        );
        assert!(backend.world.joints.is_empty());
        assert_eq!(backend.state(body).unwrap(), before);
    }
    // A finite anchor outside the collision grid would make an imprecise,
    // far-reaching joint frame.
    assert_eq!(
        backend.add_world_revolute(body, [5.0000001e8, 0.0, 0.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    let fixed = spawn(
        &mut backend,
        BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..spec
        },
    );
    assert_eq!(
        backend.add_world_revolute(fixed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    let removed = spawn(&mut backend, spec);
    assert!(backend.remove(removed));
    assert_eq!(
        backend.add_world_revolute(removed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::MissingBody)
    );
    backend.next_joint = u64::MAX;
    assert_eq!(
        backend.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointIdOverflow)
    );
    assert!(backend.world.joints.is_empty());
    assert_eq!(backend.is_jointed(body), Some(false));
    assert_eq!(backend.next_joint, u64::MAX);
}

#[test]
fn removing_a_jointed_body_retires_its_joint_and_pending_wrench() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let body = spawn(&mut backend, sphere([0.0, 0.0, -1.0]));
    let joint = backend
        .add_world_revolute(body, [0.0; 3], [1.0, 0.0, 0.0])
        .unwrap();
    backend.add_world_wrench(body, [1.0; 3], [1.0; 3]).unwrap();
    assert!(backend.remove(body));
    assert!(backend.joint_state(joint).is_none());
    assert!(backend.pending.is_empty());
    assert!(backend.world.joints.is_empty());
    backend.step(Duration::from_millis(1)).unwrap();
}

#[test]
fn native_refusals_surface_as_typed_errors_and_leave_records_intact() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let body = spawn(&mut backend, sphere([0.0; 3]));
    let slot = backend.world.bodies[&body].slot;
    // Desynchronize on purpose: the native body vanishes behind the records.
    backend.world.native.remove_body(slot).unwrap();
    assert!(!backend.remove(body));
    assert_eq!(backend.state(body).map(|s| s.position), Some([0.0; 3]));
    let error = backend
        .set_world_velocity(body, [1.0; 3], [0.0; 3])
        .unwrap_err();
    assert!(error.to_string().contains("removed body slot"), "{error}");
    assert!(matches!(
        backend.add_world_revolute(body, [0.0; 3], [1.0, 0.0, 0.0]),
        Err(PhysicsError::Native { engine: "dart", .. })
    ));
    backend.add_world_wrench(body, [1.0; 3], [0.0; 3]).unwrap();
    assert!(matches!(
        backend.step(Duration::from_millis(1)),
        Err(PhysicsError::Native { .. })
    ));
    // A joint record whose native joint is gone cannot be removed either.
    backend.world.joints.insert(
        JointId::from_raw(0),
        Joint {
            child: body,
            anchor_axis: Twist {
                linear: [0.0; 3],
                angular: [1.0, 0.0, 0.0],
            },
        },
    );
    assert!(!backend.remove_joint(JointId::from_raw(0)));
    // Records still rebuild a consistent native world.
    backend.pending.clear();
    backend.world.joints.clear();
    backend.reset().unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.state(body).unwrap().position[0] > 0.0);
}

#[test]
fn a_failed_step_applies_no_wrench_and_the_next_step_applies_each_once() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let healthy = spawn(&mut backend, sphere([0.0; 3]));
    let broken = spawn(&mut backend, sphere([5.0, 0.0, 0.0]));
    // The second target's native body vanishes behind its record.
    let slot = backend.world.bodies[&broken].slot;
    backend.world.native.remove_body(slot).unwrap();
    for id in [healthy, broken] {
        backend
            .add_world_wrench(id, [0.0, 2.0, 0.0], [0.0; 3])
            .unwrap();
    }
    assert!(matches!(
        backend.step(Duration::from_millis(10)),
        Err(PhysicsError::Native { .. })
    ));
    assert_eq!(backend.pending.len(), 2);
    backend.world.bodies.remove(&broken);
    backend.pending.remove(&broken);
    backend.step(Duration::from_millis(10)).unwrap();
    // 2 N on 2 kg for 10 ms, applied exactly once.
    let velocity = backend.state(healthy).unwrap().velocity;
    assert!((velocity[1] - 0.01).abs() < 1e-15, "{velocity:?}");
    backend.step(Duration::from_millis(10)).unwrap();
    let velocity = backend.state(healthy).unwrap().velocity;
    assert!((velocity[1] - 0.01).abs() < 1e-15, "{velocity:?}");
}

#[test]
fn native_rejections_during_rebuild_leave_the_backend_unchanged() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let body = spawn(&mut backend, sphere([0.0, 0.0, -1.0]));
    let joint = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    let state = backend.state(body);
    // A second record for the same child cannot be replayed into a DART tree.
    let anchor_axis = backend.world.joints[&joint].anchor_axis;
    backend.world.joints.insert(
        JointId::from_raw(7),
        Joint {
            child: body,
            anchor_axis,
        },
    );
    assert!(matches!(backend.reset(), Err(PhysicsError::Native { .. })));
    let initial = backend.initial_state();
    assert!(matches!(
        backend.prepare_initial(&initial),
        Err(PhysicsError::Native { .. })
    ));
    backend.world.joints.remove(&JointId::from_raw(7));
    // A body record the native side rejects fails the rebuild the same way.
    backend.world.bodies.get_mut(&body).unwrap().config.shape = 9;
    assert!(matches!(backend.reset(), Err(PhysicsError::Native { .. })));
    assert_eq!(backend.state(body), state);
    assert!(!backend.joints_open);
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.joint_state(joint).is_some());
}

#[test]
fn loaded_libraries_must_come_from_the_build_libdir() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let loaded = backend.world.native.library_paths().unwrap();
    assert!(loaded[0].contains("libdart.so"), "{loaded:?}");
    assert!(loaded[1].contains("libdart-collision-ode"), "{loaded:?}");
    assert!(
        loaded[2].contains("libdart-external-odelcpsolver"),
        "{loaded:?}"
    );
    assert!(loaded[3].contains("libode"), "{loaded:?}");
    check_libraries(loaded.clone(), BUILT_LIBDIRS).unwrap();
    let elsewhere = std::env::temp_dir();
    let elsewhere = elsewhere.to_str().unwrap();
    // Each library is checked against its own directory, ODE included.
    for index in 0..4 {
        let mut expected = BUILT_LIBDIRS;
        expected[index] = elsewhere;
        let error = check_libraries(loaded.clone(), expected).unwrap_err();
        assert!(error.to_string().contains(&loaded[index]), "{error}");
    }
    // A vanished or relative file, or a missing library, is a mismatch too.
    for path in ["/nonexistent/libdart.so.6.13", "libdart.so.6.13"] {
        let mut paths = loaded.clone();
        paths[0] = path.to_owned();
        assert!(check_libraries(paths, BUILT_LIBDIRS).is_err());
    }
    let error = check_libraries(loaded[..3].to_vec(), BUILT_LIBDIRS).unwrap_err();
    assert!(error.to_string().contains("expected 4"), "{error}");
}
