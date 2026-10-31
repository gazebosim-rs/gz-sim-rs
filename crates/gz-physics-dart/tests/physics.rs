// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ports of gz-physics-rapier/tests/physics.rs. DART stores rotations as matrices
//! and free-body velocities in body coordinates, so values that pass through a
//! rotation compare within 1e-12 (a few ULP) where Rapier's compare exactly.

use gz_physics::{
    BodySpec, DynamicBackend, Inertia, PhysicsBackend, PhysicsError, RotationSpec, Shape, install,
};
use gz_physics_dart::DartBackend;
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

fn spawn(backend: &mut DartBackend, spec: BodySpec) -> Result<gz_physics::BodyId, PhysicsError> {
    backend.spawn_link(spec, RotationSpec::default(), None)
}

fn dart(sim: &Simulation) -> &DartBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}

fn dart_mut(sim: &mut Simulation) -> &mut DartBackend {
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .into_inner()
        .downcast_mut()
        .unwrap()
}

fn close(actual: [f64; 3], expected: [f64; 3]) -> bool {
    actual
        .into_iter()
        .zip(expected)
        .all(|(a, e)| (a - e).abs() < 1e-12)
}

#[test]
fn exact_steps_free_fall_pause_and_reset_use_z_up_and_f64() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    assert_eq!(backend.engine(), "dart");
    let id = spawn(&mut backend, ball()).unwrap();
    install(&mut sim, backend).unwrap();
    assert_eq!(
        install(&mut sim, DartBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
    sim.tick().unwrap();
    assert_eq!(dart(&sim).state(id).unwrap().position, ball().position);
    sim.step_exact(100).unwrap();
    let body = dart(&sim).state(id).unwrap();
    // Semi-implicit Euler: z = 5 - g*dt^2*n(n+1)/2 exactly up to rounding.
    assert!((body.position[2] - (5.0 - 9.81 * 1e-6 * 5050.0)).abs() < 1e-12);
    assert!((body.velocity[2] + 0.981).abs() < 1e-10);
    assert_eq!(body.position[0..2], [0.0, 0.0]);
    sim.reset().unwrap();
    let body = dart(&sim).state(id).unwrap();
    assert_eq!(body.position, ball().position);
    assert_eq!(body.velocity, [0.0; 3]);
    assert!(sim.fault().is_none());
}

#[test]
fn large_coordinate_motion_keeps_submillimeter_precision() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = spawn(
        &mut backend,
        BodySpec {
            position: [1e7, 0.0, 0.0],
            velocity: [0.001, 0.0, 0.0],
            ..ball()
        },
    )
    .unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(100).unwrap();
    let displacement = dart(&sim).state(id).unwrap().position[0] - 1e7;
    assert!((displacement - 0.0001).abs() < 1e-6, "{displacement}");
}

#[test]
fn bodies_contact_fixed_ground_and_removed_handles_stay_invalid() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = spawn(
        &mut backend,
        BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        },
    )
    .unwrap();
    let id = spawn(&mut backend, ball()).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(2000).unwrap();
    let backend = dart_mut(&mut sim);
    assert!((backend.state(id).unwrap().position[2] - 0.5).abs() < 0.01);
    assert_eq!(backend.state(ground).unwrap().position, [0.0, 0.0, -0.5]);
    assert_eq!(backend.is_dynamic(ground), Some(false));
    assert_eq!(backend.is_dynamic(id), Some(true));
    assert!(backend.remove(id));
    assert!(!backend.remove(id));
    assert!(backend.state(id).is_none());
    assert_eq!(backend.is_dynamic(id), None);
    backend.reset().unwrap();
    assert!(backend.state(id).is_none());
    assert_ne!(spawn(backend, ball()).unwrap(), id);
}

#[test]
fn invalid_input_never_reaches_solver() {
    assert!(matches!(
        DartBackend::new([f64::NAN, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    ));
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
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
        assert_eq!(spawn(&mut backend, spec), Err(PhysicsError::InvalidValue));
    }
}

#[test]
fn removing_backend_resource_does_not_install_duplicate_systems() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, DartBackend::new([0.0; 3]).unwrap()).unwrap();
    sim.world_mut().remove_resource::<PhysicsBackend>();
    assert_eq!(
        install(&mut sim, DartBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
}

#[test]
fn explicit_inertia_and_rotation_control_impulse_response_and_reset() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
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
    assert!(close(state.velocity, [1.0, 0.0, 0.0]), "{state:?}");
    assert!(close(state.angular_velocity, [1.0, 0.0, 0.0]), "{state:?}");
    for (actual, expected) in state.orientation.into_iter().zip(rotational.orientation) {
        assert!((actual - expected).abs() < 1e-15);
    }
    backend.reset().unwrap();
    assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
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
fn offset_center_of_mass_reports_com_velocity_and_wrenches_act_at_the_com() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_link(
            BodySpec {
                velocity: [1.0, 0.0, 0.0],
                ..ball()
            },
            RotationSpec {
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0, 0.0, 2.0],
            },
            Some(Inertia {
                center: [0.5, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let state = backend.state(id).unwrap();
    assert!(close(state.velocity, [1.0, 0.0, 0.0]), "{state:?}");
    assert!(close(state.angular_velocity, [0.0, 0.0, 2.0]));
    // A force through the COM produces no angular acceleration.
    backend.set_world_velocity(id, [0.0; 3], [0.0; 3]).unwrap();
    backend
        .add_world_wrench(id, [0.0, 10.0, 0.0], [0.0; 3])
        .unwrap();
    backend.step(Duration::from_millis(10)).unwrap();
    let state = backend.state(id).unwrap();
    assert!(close(state.velocity, [0.0, 0.1, 0.0]), "{state:?}");
    assert!(close(state.angular_velocity, [0.0; 3]), "{state:?}");
}

#[test]
fn invalid_inertia_and_rotations_are_rejected_before_insertion() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
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
    // Rapier's f64::MAX cases scaled to DART's 1e60 velocity bound (docs/DART.md).
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = spawn(&mut backend, ball()).unwrap();
    backend
        .apply_impulse(id, [1e60, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let before = backend.state(id).unwrap();
    assert_eq!(
        backend.apply_impulse(id, [1e60, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id).unwrap(), before);
    assert_eq!(
        backend.apply_impulse(id, [0.0; 3], [1e60, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id).unwrap(), before);
}

#[test]
fn world_velocity_is_atomic_and_preserves_reset_values() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = spawn(&mut backend, ball()).unwrap();
    let fixed = spawn(
        &mut backend,
        BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..ball()
        },
    )
    .unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(1000).unwrap();
    {
        let backend = dart_mut(&mut sim);
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
        backend.apply_impulse(fixed, [1.0; 3], [2.0; 3]).unwrap();
        assert_eq!(backend.state(fixed).unwrap().velocity, [0.0; 3]);
    }
    sim.tick().unwrap();
    assert_eq!(dart(&sim).state(id).unwrap().position, ball().position);
    sim.step_exact(1).unwrap();
    let state = dart(&sim).state(id).unwrap();
    assert!(close(state.position, [0.01, 0.02, 5.03]), "{state:?}");
    assert!(state.orientation[2] > 0.0);
    sim.reset().unwrap();
    let backend = dart_mut(&mut sim);
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
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = spawn(
        &mut backend,
        BodySpec {
            mass: Some(2.0),
            ..ball()
        },
    )
    .unwrap();
    let fixed = spawn(
        &mut backend,
        BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..ball()
        },
    )
    .unwrap();
    install(&mut sim, backend).unwrap();
    {
        let backend = dart_mut(&mut sim);
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
        backend.add_world_wrench(fixed, [1.0; 3], [1.0; 3]).unwrap();
    }
    sim.tick().unwrap();
    assert_eq!(dart(&sim).state(id).unwrap().velocity, [0.0; 3]);
    for _ in 0..3 {
        sim.step_exact(1).unwrap();
        let state = dart(&sim).state(id).unwrap();
        assert!((state.velocity[0] - 0.01).abs() < 1e-12);
        assert_eq!(state.velocity[1..], [0.0; 2]);
        assert_eq!(dart(&sim).state(fixed).unwrap().position, [100.0, 0.0, 0.0]);
    }
    {
        let backend = dart_mut(&mut sim);
        backend
            .set_world_velocity(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        backend
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
    }
    sim.step_exact(1).unwrap();
    assert!((dart(&sim).state(id).unwrap().velocity[0] - 1.01).abs() < 1e-12);
    dart_mut(&mut sim)
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(dart(&sim).state(id).unwrap().velocity, [0.0; 3]);
    let backend = dart_mut(&mut sim);
    backend
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert!(backend.remove(id));
    assert_eq!(
        backend.add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3]),
        Err(PhysicsError::MissingBody)
    );
    backend.step(Duration::from_millis(1)).unwrap();
}

#[test]
fn invalid_and_overflowing_wrenches_preserve_admitted_force_and_torque() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = spawn(&mut backend, ball()).unwrap();
    for (force, torque) in [
        ([f64::INFINITY; 3], [0.0; 3]),
        ([0.0; 3], [f64::NEG_INFINITY; 3]),
    ] {
        assert_eq!(
            backend.add_world_wrench(id, force, torque),
            Err(PhysicsError::InvalidValue)
        );
    }
    // Rapier's 1e308 overflow cases scaled to DART's 1e60 accumulation bound.
    backend
        .add_world_wrench(id, [1e60, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert_eq!(
        backend.add_world_wrench(id, [1e60, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    backend
        .add_world_wrench(id, [-1e60, 0.0, 0.0], [0.0; 3])
        .unwrap();
    backend
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, 1e60])
        .unwrap();
    assert_eq!(
        backend.add_world_wrench(id, [1.0; 3], [0.0, 0.0, 1e60]),
        Err(PhysicsError::InvalidValue)
    );
    backend
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, -1e60])
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(1).unwrap();
    {
        let backend = dart_mut(&mut sim);
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
    let backend = dart(&sim);
    assert_eq!(backend.state(id).unwrap().velocity, [0.0; 3]);
    assert_eq!(backend.state(id).unwrap().angular_velocity, [0.0; 3]);
}

/// A tumbling body with a rotated pose, offset COM and non-diagonal inertia, and
/// a light body, which the largest admitted force accelerates past the velocity
/// bound in one step.
fn nonfinite_scene() -> (DartBackend, gz_physics::BodyId, gz_physics::BodyId) {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let s = 0.3_f64.sin();
    let tumbling = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.2, 0.3, 0.4],
                },
                position: [0.0, 0.0, 5.0],
                velocity: [0.3, -0.2, 0.5],
                mass: Some(2.0),
            },
            RotationSpec {
                orientation: [s, 0.0, 0.0, 0.3_f64.cos()],
                angular_velocity: [1.0, -2.0, 0.5],
            },
            Some(Inertia {
                center: [0.05, -0.02, 0.1],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.2, 0.3, 0.25, 0.01, -0.02, 0.015],
            }),
        )
        .unwrap();
    let light = backend
        .spawn_link(
            BodySpec {
                position: [10.0, 0.0, 5.0],
                mass: Some(2e-5),
                ..ball()
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    (backend, tumbling, light)
}

#[test]
fn out_of_bound_steps_fault_the_simulation_and_restore_the_previous_state() {
    let (mut failed, tumbling, light) = nonfinite_scene();
    let (mut clean, ..) = nonfinite_scene();
    for backend in [&mut failed, &mut clean] {
        for _ in 0..10 {
            backend.step(Duration::from_millis(1)).unwrap();
        }
    }
    // 1e60 N on 2e-5 kg reaches 5e61 m/s in 1 ms, beyond the 1e60 bound the
    // next step needs to stay finite inside DART.
    failed
        .add_world_wrench(light, [1e60, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let error = failed.step(Duration::from_millis(1)).unwrap_err();
    assert!(
        matches!(error, PhysicsError::Native { engine: "dart", .. }),
        "{error}"
    );
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, failed).unwrap();
    let _ = sim.step_exact(1);
    assert!(sim.fault().is_some());
    // After removing the offender, the native state continues exactly as in a
    // world that never failed, not only the cached readback.
    let mut failed = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    let failed: &mut DartBackend = failed.downcast_mut().unwrap();
    assert!(failed.remove(light));
    assert!(clean.remove(light));
    for _ in 0..100 {
        failed.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        assert_eq!(failed.state(tumbling), clean.state(tumbling));
    }
    assert!(failed.state(tumbling).unwrap().position[2] < 5.0);
}

#[test]
fn body_revolute_retirement_restores_endpoints_and_keeps_unsupported_input_errors() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let parent = backend
        .spawn_link(ball(), RotationSpec::default(), None)
        .unwrap();
    let child = backend
        .spawn_link(
            BodySpec {
                position: [2.0, 0.0, 5.0],
                ..ball()
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let before = [backend.state(parent), backend.state(child)];
    let joint = backend
        .add_revolute(parent, child, [0.0, 0.0, 5.0], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(backend.joint_child(joint), Some(child));
    assert_eq!([backend.state(parent), backend.state(child)], before);
    assert_eq!(backend.is_jointed(parent), Some(true));
    assert_eq!(backend.is_jointed(child), Some(true));
    assert!(backend.remove_joint(joint));
    for anchor in [[0.0; 3], [f64::NAN; 3]] {
        assert_eq!(
            backend.add_reduced_revolute(Some(parent), child, anchor, [0.0, 1.0, 0.0]),
            Err(PhysicsError::Unsupported {
                engine: "dart",
                capability: "reduced-coordinate revolute joint"
            })
        );
    }
    assert_eq!(
        backend.set_self_collision_group(parent, Some(u64::MAX)),
        Err(PhysicsError::Unsupported {
            engine: "dart",
            capability: "self-collision groups"
        })
    );
    assert_eq!([backend.state(parent), backend.state(child)], before);
    assert_eq!(backend.is_jointed(parent), Some(false));
    assert_eq!(backend.is_jointed(child), Some(false));
}
