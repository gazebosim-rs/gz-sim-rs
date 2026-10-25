// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodySpec, PhysicsError, RapierBackend, Shape, install};
use gz_sim_core::Simulation;
use std::time::Duration;

fn ball() -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}

#[test]
fn exact_steps_free_fall_pause_and_reset_use_z_up_and_f64() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let id = backend.spawn(ball()).unwrap();
    install(&mut sim, backend).unwrap();
    assert_eq!(
        install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .state(id)
            .unwrap()
            .position,
        ball().position
    );
    sim.step_exact(100).unwrap();
    let body = sim.world().resource::<RapierBackend>().state(id).unwrap();
    assert!((body.position[2] - (5.0 - 0.5 * 9.81 * 0.1_f64.powi(2))).abs() < 0.001);
    assert!((body.velocity[2] + 0.981).abs() < 1e-10);
    assert_eq!(body.position[0..2], [0.0, 0.0]);
    sim.reset().unwrap();
    let body = sim.world().resource::<RapierBackend>().state(id).unwrap();
    assert_eq!(body.position, ball().position);
    assert_eq!(body.velocity, [0.0; 3]);
}

#[test]
fn large_coordinate_motion_keeps_submillimeter_precision() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn(BodySpec {
            position: [1e7, 0.0, 0.0],
            velocity: [0.001, 0.0, 0.0],
            ..ball()
        })
        .unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(100).unwrap();
    let displacement = sim
        .world()
        .resource::<RapierBackend>()
        .state(id)
        .unwrap()
        .position[0]
        - 1e7;
    assert!((displacement - 0.0001).abs() < 1e-6, "{displacement}");
}

#[test]
fn bodies_contact_fixed_ground_and_removed_handles_stay_invalid() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = backend
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let id = backend.spawn(ball()).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(2000).unwrap();
    let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
    assert!((backend.state(id).unwrap().position[2] - 0.5).abs() < 0.01);
    assert_eq!(backend.state(ground).unwrap().position, [0.0, 0.0, -0.5]);
    assert!(backend.remove(id));
    assert!(!backend.remove(id));
    assert!(backend.state(id).is_none());
    backend.reset();
    assert!(backend.state(id).is_none());
    assert_ne!(backend.spawn(ball()).unwrap(), id);
}

#[test]
fn invalid_input_never_reaches_solver() {
    assert!(matches!(
        RapierBackend::new([f64::NAN, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    ));
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    for spec in [
        BodySpec {
            shape: Shape::Sphere { radius: f64::MAX },
            ..ball()
        },
        BodySpec {
            shape: Shape::Sphere {
                radius: f64::MIN_POSITIVE,
            },
            ..ball()
        },
        BodySpec {
            shape: Shape::Sphere {
                radius: f64::from_bits(1),
            },
            ..ball()
        },
        BodySpec {
            mass: Some(f64::MAX),
            ..ball()
        },
        BodySpec {
            mass: Some(f64::from_bits(1)),
            ..ball()
        },
        BodySpec {
            position: [f64::MAX; 3],
            ..ball()
        },
        BodySpec {
            position: [f64::INFINITY; 3],
            ..ball()
        },
        BodySpec {
            velocity: [f64::NAN; 3],
            ..ball()
        },
        BodySpec {
            mass: Some(-1.0),
            ..ball()
        },
        BodySpec {
            mass: Some(f64::NAN),
            ..ball()
        },
        BodySpec {
            shape: Shape::Sphere { radius: 0.0 },
            ..ball()
        },
        BodySpec {
            shape: Shape::Sphere { radius: f64::NAN },
            ..ball()
        },
        BodySpec {
            shape: Shape::Box {
                half_extents: [1.0, -1.0, 1.0],
            },
            ..ball()
        },
        BodySpec {
            shape: Shape::Box {
                half_extents: [1.0, f64::INFINITY, 1.0],
            },
            ..ball()
        },
        BodySpec {
            mass: None,
            velocity: [1.0, 0.0, 0.0],
            ..ball()
        },
    ] {
        assert_eq!(backend.spawn(spec), Err(PhysicsError::InvalidValue));
    }
    for error in [
        PhysicsError::InvalidValue,
        PhysicsError::IdOverflow,
        PhysicsError::AlreadyInstalled,
    ] {
        assert!(!error.to_string().is_empty());
    }
}

#[test]
fn removing_backend_resource_does_not_install_duplicate_systems() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()).unwrap();
    sim.world_mut().remove_resource::<RapierBackend>();
    assert_eq!(
        install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
}

#[test]
fn explicit_inertia_and_rotation_control_impulse_response_and_reset() {
    use gz_physics_rapier::{Inertia, RotationSpec};
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let inertia = Inertia {
        center: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    };
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotational = RotationSpec {
        orientation: [0.0, 0.0, s, s],
        angular_velocity: [0.0; 3],
    };
    let id = backend
        .spawn_link(
            BodySpec {
                mass: Some(2.0),
                ..ball()
            },
            rotational,
            Some(inertia),
        )
        .unwrap();
    backend
        .apply_impulse(id, [2.0, 0.0, 0.0], [3.0, 0.0, 0.0])
        .unwrap();
    let state = backend.state(id).unwrap();
    assert_eq!(state.velocity, [1.0, 0.0, 0.0]);
    assert!((state.angular_velocity[0] - 1.0).abs() < 1e-12);
    assert!(state.angular_velocity[1].abs() < 1e-12);
    assert_eq!(state.orientation, rotational.orientation);
    backend.reset();
    assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
    assert_eq!(
        backend.state(id).unwrap().orientation,
        rotational.orientation
    );
    // Rotating the inertial frame swaps the body's X/Y principal moments too.
    let id2 = backend
        .spawn_link(
            ball(),
            RotationSpec::default(),
            Some(Inertia {
                orientation: rotational.orientation,
                ..inertia
            }),
        )
        .unwrap();
    backend
        .apply_impulse(id2, [0.0; 3], [3.0, 0.0, 0.0])
        .unwrap();
    assert!((backend.state(id2).unwrap().angular_velocity[0] - 1.0).abs() < 1e-12);
    assert!(backend.remove(id));
    assert_eq!(
        backend.apply_impulse(id, [0.0; 3], [0.0; 3]),
        Err(PhysicsError::MissingBody)
    );
    assert!(!PhysicsError::MissingBody.to_string().is_empty());
    assert_eq!(
        backend.apply_impulse(id2, [f64::NAN; 3], [0.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        backend.apply_impulse(id2, [0.0; 3], [f64::INFINITY; 3]),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn invalid_inertia_and_rotations_are_rejected_before_insertion() {
    use gz_physics_rapier::{Inertia, RotationSpec};
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    for rotation in [
        RotationSpec {
            orientation: [0.0; 4],
            ..RotationSpec::default()
        },
        RotationSpec {
            orientation: [f64::NAN; 4],
            ..RotationSpec::default()
        },
        RotationSpec {
            angular_velocity: [f64::NAN; 3],
            ..RotationSpec::default()
        },
    ] {
        assert_eq!(
            backend.spawn_link(ball(), rotation, None),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(
        backend.spawn_link(
            BodySpec {
                mass: None,
                ..ball()
            },
            RotationSpec {
                angular_velocity: [1.0; 3],
                ..RotationSpec::default()
            },
            None
        ),
        Err(PhysicsError::InvalidValue)
    );
    let inertia = Inertia {
        center: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    };
    for invalid in [
        Inertia {
            center: [f64::NAN; 3],
            ..inertia
        },
        Inertia {
            orientation: [0.0; 4],
            ..inertia
        },
        Inertia {
            tensor: [f64::MAX; 6],
            ..inertia
        },
        Inertia {
            tensor: [f64::NAN; 6],
            ..inertia
        },
        Inertia {
            tensor: [-1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..inertia
        },
        Inertia {
            tensor: [10.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..inertia
        },
    ] {
        assert_eq!(
            backend.spawn_link(ball(), RotationSpec::default(), Some(invalid)),
            Err(PhysicsError::InvalidValue)
        );
    }
}

#[test]
fn overflow_impulses_leave_both_velocities_unchanged() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball()).unwrap();
    backend
        .apply_impulse(id, [f64::MAX, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let before = backend.state(id).unwrap();
    assert_eq!(
        backend.apply_impulse(id, [f64::MAX, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id).unwrap(), before);
    assert_eq!(
        backend.apply_impulse(id, [0.0; 3], [f64::MAX, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id).unwrap(), before);
}

#[test]
fn world_velocity_is_atomic_wakes_sleeping_bodies_and_preserves_reset_values() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball()).unwrap();
    let fixed = backend
        .spawn(BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..ball()
        })
        .unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(1000).unwrap();
    {
        let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
        backend
            .set_world_velocity(id, [1.0, 2.0, 3.0], [0.0, 0.0, 2.0])
            .unwrap();
        for (linear, angular) in [
            ([f64::NAN, 0.0, 0.0], [0.0; 3]),
            ([9.0; 3], [0.0, f64::INFINITY, 0.0]),
        ] {
            assert_eq!(
                backend.set_world_velocity(id, linear, angular),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(backend.state(id).unwrap().velocity, [1.0, 2.0, 3.0]);
            assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0, 0.0, 2.0]);
        }
        backend
            .set_world_velocity(fixed, [1.0; 3], [2.0; 3])
            .unwrap();
        assert_eq!(backend.state(fixed).unwrap().velocity, [0.0; 3]);
        assert_eq!(backend.state(fixed).unwrap().angular_velocity, [0.0; 3]);
    }
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .state(id)
            .unwrap()
            .position,
        ball().position
    );
    sim.step_exact(1).unwrap();
    let state = sim.world().resource::<RapierBackend>().state(id).unwrap();
    for (actual, expected) in state.position.into_iter().zip([0.01, 0.02, 5.03]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert!(state.orientation[2] > 0.0);
    sim.reset().unwrap();
    let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
    assert_eq!(backend.state(id).unwrap().velocity, [0.0; 3]);
    assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
    assert!(backend.remove(id));
    assert_eq!(
        backend.set_world_velocity(id, [1.0; 3], [2.0; 3]),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn world_wrenches_accumulate_for_one_advancing_step_and_cancel_on_reset_or_removal() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn(BodySpec {
            mass: Some(2.0),
            ..ball()
        })
        .unwrap();
    install(&mut sim, backend).unwrap();
    {
        let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
        backend
            .add_world_wrench(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        backend
            .add_world_wrench(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        assert_eq!(
            backend.add_world_wrench(id, [99.0; 3], [f64::NAN; 3]),
            Err(PhysicsError::InvalidValue)
        );
    }
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .state(id)
            .unwrap()
            .velocity,
        [0.0; 3]
    );
    for _ in 0..3 {
        sim.step_exact(1).unwrap();
        let state = sim.world().resource::<RapierBackend>().state(id).unwrap();
        assert!((state.velocity[0] - 0.01).abs() < 1e-12);
        assert_eq!(state.velocity[1..], [0.0; 2]);
    }
    {
        let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
        backend
            .set_world_velocity(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        backend
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
    }
    sim.step_exact(1).unwrap();
    assert!(
        (sim.world()
            .resource::<RapierBackend>()
            .state(id)
            .unwrap()
            .velocity[0]
            - 1.01)
            .abs()
            < 1e-12
    );
    sim.world_mut()
        .resource_mut::<RapierBackend>()
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .state(id)
            .unwrap()
            .velocity,
        [0.0; 3]
    );
    let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
    backend
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert!(backend.remove(id));
    assert_eq!(
        backend.add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3]),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn invalid_and_overflowing_wrenches_preserve_admitted_force_and_torque() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball()).unwrap();
    for (force, torque) in [
        ([f64::INFINITY; 3], [0.0; 3]),
        ([0.0; 3], [f64::NEG_INFINITY; 3]),
    ] {
        assert_eq!(
            backend.add_world_wrench(id, force, torque),
            Err(PhysicsError::InvalidValue)
        );
    }
    backend
        .add_world_wrench(id, [1e308, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert_eq!(
        backend.add_world_wrench(id, [1e308, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    backend
        .add_world_wrench(id, [-1e308, 0.0, 0.0], [0.0; 3])
        .unwrap();
    backend
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, 1e308])
        .unwrap();
    assert_eq!(
        backend.add_world_wrench(id, [1.0; 3], [0.0, 0.0, 1e308]),
        Err(PhysicsError::InvalidValue)
    );
    backend
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, -1e308])
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(1).unwrap();
    {
        let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
        assert_eq!(backend.state(id).unwrap().velocity, [0.0; 3]);
        assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
        let initial = backend.initial_state();
        let prepared = backend.prepare_initial(&initial).unwrap();
        backend
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0, 0.0, 0.1])
            .unwrap();
        backend.apply_initial(prepared).unwrap();
    }
    sim.step_exact(1).unwrap();
    let backend = sim.world().resource::<RapierBackend>();
    assert_eq!(backend.state(id).unwrap().velocity, [0.0; 3]);
    assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
}
