// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Engine-independent behavior ported from crates/gz-physics-rapier/tests/physics.rs.
//! Exact equalities that depend on Rapier's arithmetic become 1e-12 bounds where
//! MuJoCo maps velocities through its mass matrix (a few ulps of rounding).
use gz_physics::{DynamicBackend, PhysicsBackend, PhysicsError, install};
use gz_physics_mujoco::{BodySpec, Inertia, MujocoBackend, RotationSpec, Shape};
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

fn backend(sim: &Simulation) -> &MujocoBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}

fn backend_mut(sim: &mut Simulation) -> &mut MujocoBackend {
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .into_inner()
        .downcast_mut()
        .unwrap()
}

fn close(actual: [f64; 3], expected: [f64; 3], tolerance: f64) -> bool {
    actual
        .into_iter()
        .zip(expected)
        .all(|(a, e)| (a - e).abs() <= tolerance)
}

#[test]
fn exact_steps_free_fall_pause_and_reset_use_z_up_and_f64() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let id = mujoco.spawn(ball()).unwrap();
    assert_eq!(mujoco.engine(), "mujoco");
    install(&mut sim, mujoco).unwrap();
    assert_eq!(
        install(&mut sim, MujocoBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
    sim.tick().unwrap();
    assert_eq!(backend(&sim).state(id).unwrap().position, ball().position);
    sim.step_exact(100).unwrap();
    let body = backend(&sim).state(id).unwrap();
    assert_eq!(backend(&sim).timestep(), 0.001);
    // Semi-implicit Euler: z(n) = z0 - g dt^2 n(n+1)/2, v(n) = -g dt n.
    assert!((body.position[2] - (5.0 - 9.81 * 1e-6 * 5050.0)).abs() < 1e-12);
    assert!((body.velocity[2] + 0.981).abs() < 1e-12);
    assert_eq!(body.position[0..2], [0.0, 0.0]);
    sim.reset().unwrap();
    let body = backend(&sim).state(id).unwrap();
    assert_eq!(body.position, ball().position);
    assert_eq!(body.velocity, [0.0; 3]);
}

#[test]
fn large_coordinate_motion_keeps_submillimeter_precision() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco
        .spawn(BodySpec {
            position: [1e7, 0.0, 0.0],
            velocity: [0.001, 0.0, 0.0],
            ..ball()
        })
        .unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(100).unwrap();
    let displacement = backend(&sim).state(id).unwrap().position[0] - 1e7;
    assert!((displacement - 0.0001).abs() < 1e-6, "{displacement}");
}

#[test]
fn bodies_contact_fixed_ground_and_removed_handles_stay_invalid() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = mujoco
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let id = mujoco.spawn(ball()).unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(2000).unwrap();
    let mujoco = backend_mut(&mut sim);
    assert!((mujoco.state(id).unwrap().position[2] - 0.5).abs() < 0.01);
    assert_eq!(mujoco.state(ground).unwrap().position, [0.0, 0.0, -0.5]);
    assert_eq!(mujoco.is_dynamic(ground), Some(false));
    assert_eq!(mujoco.is_dynamic(id), Some(true));
    assert!(mujoco.remove(id));
    assert!(!mujoco.remove(id));
    assert!(mujoco.state(id).is_none());
    assert_eq!(mujoco.is_dynamic(id), None);
    mujoco.reset().unwrap();
    assert!(mujoco.state(id).is_none());
    assert_ne!(mujoco.spawn(ball()).unwrap(), id);
}

#[test]
fn invalid_input_never_reaches_solver() {
    assert!(matches!(
        MujocoBackend::new([f64::NAN, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    ));
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
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
            shape: Shape::Cylinder {
                radius: 1.0,
                half_height: 0.0,
            },
            ..ball()
        },
        BodySpec {
            shape: Shape::Capsule {
                radius: f64::NAN,
                half_length: 1.0,
            },
            ..ball()
        },
        BodySpec {
            mass: None,
            velocity: [1.0, 0.0, 0.0],
            ..ball()
        },
    ] {
        assert_eq!(mujoco.spawn(spec), Err(PhysicsError::InvalidValue));
    }
    // Rejected bodies never consumed an identifier.
    assert_eq!(mujoco.spawn(ball()).unwrap().raw(), 0);
}

#[test]
fn removing_backend_resource_does_not_install_duplicate_systems() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, MujocoBackend::new([0.0; 3]).unwrap()).unwrap();
    sim.world_mut().remove_resource::<PhysicsBackend>();
    assert_eq!(
        install(&mut sim, MujocoBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
}

#[test]
fn explicit_inertia_and_rotation_control_impulse_response_and_reset() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
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
    let id = mujoco
        .spawn_link(
            BodySpec {
                mass: Some(2.0),
                ..ball()
            },
            rotational,
            Some(inertia),
        )
        .unwrap();
    mujoco
        .apply_impulse(id, [2.0, 0.0, 0.0], [3.0, 0.0, 0.0])
        .unwrap();
    let state = mujoco.state(id).unwrap();
    assert!(close(state.velocity, [1.0, 0.0, 0.0], 1e-12));
    assert!((state.angular_velocity[0] - 1.0).abs() < 1e-12);
    assert!(state.angular_velocity[1].abs() < 1e-12);
    assert!(
        state
            .orientation
            .into_iter()
            .zip(rotational.orientation)
            .all(|(a, e)| (a - e).abs() < 1e-15)
    );
    mujoco.reset().unwrap();
    assert_eq!(mujoco.state(id).unwrap().angular_velocity, [0.0; 3]);
    // Rotating the inertial frame swaps the body's X/Y principal moments too.
    let id2 = mujoco
        .spawn_link(
            ball(),
            RotationSpec::default(),
            Some(Inertia {
                orientation: rotational.orientation,
                ..inertia
            }),
        )
        .unwrap();
    mujoco
        .apply_impulse(id2, [0.0; 3], [3.0, 0.0, 0.0])
        .unwrap();
    assert!((mujoco.state(id2).unwrap().angular_velocity[0] - 1.0).abs() < 1e-12);
    assert!(mujoco.remove(id));
    assert_eq!(
        mujoco.apply_impulse(id, [0.0; 3], [0.0; 3]),
        Err(PhysicsError::MissingBody)
    );
    assert_eq!(
        mujoco.apply_impulse(id2, [f64::NAN; 3], [0.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        mujoco.apply_impulse(id2, [0.0; 3], [f64::INFINITY; 3]),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn invalid_inertia_and_rotations_are_rejected_before_insertion() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
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
            mujoco.spawn_link(ball(), rotation, None),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(
        mujoco.spawn_link(
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
        Inertia {
            tensor: [1.0, 1.0, 1.0, 2.0, 0.0, 0.0],
            ..inertia
        },
    ] {
        assert_eq!(
            mujoco.spawn_link(ball(), RotationSpec::default(), Some(invalid)),
            Err(PhysicsError::InvalidValue)
        );
    }
}

#[test]
fn overflow_impulses_leave_both_velocities_unchanged() {
    // MuJoCo resets beyond mjMAXVAL (1e10), so that bound replaces f64 overflow:
    // the first impulse reaches it exactly and any further push exceeds it.
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco.spawn(ball()).unwrap();
    mujoco
        .apply_impulse(id, [1e10, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let before = mujoco.state(id).unwrap();
    assert_eq!(before.velocity, [1e10, 0.0, 0.0]);
    assert_eq!(
        mujoco.apply_impulse(id, [1.0, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(mujoco.state(id).unwrap(), before);
    assert_eq!(
        mujoco.apply_impulse(id, [0.0; 3], [f64::MAX, 0.0, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(mujoco.state(id).unwrap(), before);
}

#[test]
fn world_velocity_is_atomic_and_preserves_reset_values() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco.spawn(ball()).unwrap();
    let fixed = mujoco
        .spawn(BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..ball()
        })
        .unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(1000).unwrap();
    {
        let mujoco = backend_mut(&mut sim);
        mujoco
            .set_world_velocity(id, [1.0, 2.0, 3.0], [0.0, 0.0, 2.0])
            .unwrap();
        let set = mujoco.state(id).unwrap();
        for (linear, angular) in [
            ([f64::NAN, 0.0, 0.0], [0.0; 3]),
            ([9.0; 3], [0.0, f64::INFINITY, 0.0]),
        ] {
            assert_eq!(
                mujoco.set_world_velocity(id, linear, angular),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(mujoco.state(id).unwrap(), set);
        }
        assert!(close(set.velocity, [1.0, 2.0, 3.0], 1e-12));
        assert!(close(set.angular_velocity, [0.0, 0.0, 2.0], 1e-12));
        mujoco
            .set_world_velocity(fixed, [1.0; 3], [2.0; 3])
            .unwrap();
        assert_eq!(mujoco.state(fixed).unwrap().velocity, [0.0; 3]);
        assert_eq!(mujoco.state(fixed).unwrap().angular_velocity, [0.0; 3]);
    }
    sim.tick().unwrap();
    assert_eq!(backend(&sim).state(id).unwrap().position, ball().position);
    sim.step_exact(1).unwrap();
    let state = backend(&sim).state(id).unwrap();
    assert!(close(state.position, [0.01, 0.02, 5.03], 1e-12));
    // A positive world-Z spin turns the body counterclockwise about Z.
    let expected = 0.01_f64; // half of omega*dt
    assert!((state.orientation[2] - expected.sin()).abs() < 1e-6);
    sim.reset().unwrap();
    let mujoco = backend_mut(&mut sim);
    assert_eq!(mujoco.state(id).unwrap().velocity, [0.0; 3]);
    assert_eq!(mujoco.state(id).unwrap().angular_velocity, [0.0; 3]);
    assert!(mujoco.remove(id));
    assert_eq!(
        mujoco.set_world_velocity(id, [1.0; 3], [2.0; 3]),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn world_wrenches_accumulate_for_one_advancing_step_and_cancel_on_reset_or_removal() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco
        .spawn(BodySpec {
            mass: Some(2.0),
            ..ball()
        })
        .unwrap();
    install(&mut sim, mujoco).unwrap();
    {
        let mujoco = backend_mut(&mut sim);
        mujoco
            .add_world_wrench(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        mujoco
            .add_world_wrench(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        assert_eq!(
            mujoco.add_world_wrench(id, [99.0; 3], [f64::NAN; 3]),
            Err(PhysicsError::InvalidValue)
        );
    }
    sim.tick().unwrap();
    assert_eq!(backend(&sim).state(id).unwrap().velocity, [0.0; 3]);
    for _ in 0..3 {
        sim.step_exact(1).unwrap();
        let state = backend(&sim).state(id).unwrap();
        assert!((state.velocity[0] - 0.01).abs() < 1e-12);
        assert_eq!(state.velocity[1..], [0.0; 2]);
    }
    {
        let mujoco = backend_mut(&mut sim);
        mujoco
            .set_world_velocity(id, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        mujoco
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
    }
    sim.step_exact(1).unwrap();
    assert!((backend(&sim).state(id).unwrap().velocity[0] - 1.01).abs() < 1e-12);
    backend_mut(&mut sim)
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(backend(&sim).state(id).unwrap().velocity, [0.0; 3]);
    let mujoco = backend_mut(&mut sim);
    mujoco
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert!(mujoco.remove(id));
    assert_eq!(
        mujoco.add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3]),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn invalid_and_overflowing_wrenches_preserve_admitted_force_and_torque() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco.spawn(ball()).unwrap();
    for (force, torque) in [
        ([f64::INFINITY; 3], [0.0; 3]),
        ([0.0; 3], [f64::NEG_INFINITY; 3]),
    ] {
        assert_eq!(
            mujoco.add_world_wrench(id, force, torque),
            Err(PhysicsError::InvalidValue)
        );
    }
    mujoco
        .add_world_wrench(id, [1e308, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert_eq!(
        mujoco.add_world_wrench(id, [1e308, 0.0, 0.0], [1.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    mujoco
        .add_world_wrench(id, [-1e308, 0.0, 0.0], [0.0; 3])
        .unwrap();
    mujoco
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, 1e308])
        .unwrap();
    assert_eq!(
        mujoco.add_world_wrench(id, [1.0; 3], [0.0, 0.0, 1e308]),
        Err(PhysicsError::InvalidValue)
    );
    mujoco
        .add_world_wrench(id, [0.0; 3], [0.0, 0.0, -1e308])
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(1).unwrap();
    {
        let mujoco = backend_mut(&mut sim);
        assert_eq!(mujoco.state(id).unwrap().velocity, [0.0; 3]);
        assert_eq!(mujoco.state(id).unwrap().angular_velocity, [0.0; 3]);
        let initial = mujoco.initial_state();
        let prepared = mujoco.prepare_initial(&initial).unwrap();
        mujoco
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0, 0.0, 0.1])
            .unwrap();
        mujoco.apply_initial(prepared).unwrap();
    }
    sim.step_exact(1).unwrap();
    let mujoco = backend(&sim);
    assert_eq!(mujoco.state(id).unwrap().velocity, [0.0; 3]);
    assert_eq!(mujoco.state(id).unwrap().angular_velocity, [0.0; 3]);
}

#[test]
fn body_revolutes_are_rejected_explicitly_without_changing_endpoints() {
    let mut backend = MujocoBackend::new([0.0; 3]).unwrap();
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
    assert_eq!(
        backend.add_revolute(parent, child, [0.0, 0.0, 5.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::Unsupported {
            engine: "mujoco",
            capability: "body-to-body revolute joint"
        })
    );
    for anchor in [[0.0; 3], [f64::NAN; 3]] {
        assert_eq!(
            backend.add_reduced_revolute(Some(parent), child, anchor, [0.0, 1.0, 0.0]),
            Err(PhysicsError::Unsupported {
                engine: "mujoco",
                capability: "reduced-coordinate revolute joint"
            })
        );
    }
    assert_eq!(
        backend.set_self_collision_group(parent, Some(u64::MAX)),
        Err(PhysicsError::Unsupported {
            engine: "mujoco",
            capability: "self-collision groups"
        })
    );
    assert_eq!([backend.state(parent), backend.state(child)], before);
    assert_eq!(backend.is_jointed(parent), Some(false));
    assert_eq!(backend.is_jointed(child), Some(false));
}
