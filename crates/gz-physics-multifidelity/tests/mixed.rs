// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

fn sphere() -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.2 },
        position: [0.0; 3],
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}

fn backend() -> MultiFidelityBackend {
    MultiFidelityBackend::new([0.0; 3]).unwrap()
}

#[test]
fn kinematic_offset_com_rotates_about_moving_center_and_preserves_world_twist() {
    let mut world = backend();
    let rotation = RotationSpec {
        angular_velocity: [0.0, 0.0, 1.0],
        ..Default::default()
    };
    let inertia = Inertia {
        center: [0.5, 0.0, 0.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    };
    let spec = BodySpec {
        velocity: [1.0, 0.0, 0.0],
        ..sphere()
    };
    let id = world
        .spawn_kinematic_link(spec, rotation, Some(inertia))
        .unwrap();
    assert_eq!(world.is_dynamic(id), Some(false));
    assert_eq!(world.is_kinematic(id), Some(true));
    let initial = world.state(id).unwrap();
    world.step(Duration::ZERO).unwrap();
    assert_eq!(world.state(id), Some(initial));

    for _ in 0..100 {
        world.step(Duration::from_millis(5)).unwrap();
    }
    let moved = world.state(id).unwrap();
    let theta = 0.5_f64;
    // Rapier normalizes its quaternion Euler update. Over this interval the
    // analytic angle error is bounded by theta * (omega * dt)^2 / 12.
    let angle_bound = theta * 0.005_f64.powi(2) / 12.0 + 1e-12;
    assert!(
        (moved.position[0] - (1.0 - 0.5 * theta.cos())).abs() < angle_bound,
        "{moved:?}"
    );
    assert!((moved.position[1] + 0.5 * theta.sin()).abs() < angle_bound);
    assert!((moved.orientation[2] - (theta / 2.0).sin()).abs() < angle_bound);
    assert_eq!(moved.velocity, spec.velocity);
    assert_eq!(moved.angular_velocity, rotation.angular_velocity);

    let replacement = gz_physics::BodyState {
        position: [4.0, 0.0, 0.0],
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
        ..initial
    };
    world.validate_body_state(id, replacement).unwrap();
    world.set_body_state(id, replacement).unwrap();
    assert_eq!(world.state(id), Some(replacement));
    let distance = world
        .cast_ray([0.0; 3], [1.0, 0.0, 0.0], 5.0, None)
        .unwrap()
        .unwrap();
    assert!((distance - 3.8).abs() < 1e-12);
    world
        .set_world_velocity(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    world.step(Duration::from_millis(10)).unwrap();
    assert!((world.state(id).unwrap().position[0] - 4.02).abs() < 1e-12);
    world.reset().unwrap();
    assert_eq!(world.state(id), Some(initial));
}

#[test]
fn unsupported_kinematic_inputs_preserve_pending_dynamic_forces_and_joint_ids() {
    let mut world = backend();
    let kin = world
        .spawn_kinematic_link(sphere(), Default::default(), None)
        .unwrap();
    let body = world
        .spawn_link(
            BodySpec {
                position: [20.0, 0.0, 0.0],
                ..sphere()
            },
            Default::default(),
            None,
        )
        .unwrap();
    let initial = world.state(kin);
    world
        .add_world_wrench(body, [5.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let axis = [0.0, 0.0, 1.0];
    for error in [
        world
            .add_world_wrench(kin, [10.0; 3], [0.0; 3])
            .unwrap_err(),
        world.apply_impulse(kin, [0.0; 3], [10.0; 3]).unwrap_err(),
        world.add_world_revolute(kin, [0.0; 3], axis).unwrap_err(),
        world.add_revolute(kin, body, [0.0; 3], axis).unwrap_err(),
        world
            .add_reduced_revolute(Some(body), kin, [0.0; 3], axis)
            .unwrap_err(),
    ] {
        assert!(matches!(
            error,
            PhysicsError::Unsupported {
                engine: "multifidelity",
                ..
            }
        ));
    }
    assert_eq!(world.is_jointed(kin), Some(false));
    world.step(Duration::ZERO).unwrap();
    for _ in 0..2 {
        world.step(Duration::from_millis(100)).unwrap();
        assert!((world.state(body).unwrap().velocity[0] - 0.5).abs() < 1e-12);
        assert_eq!(world.state(kin), initial);
    }
    world.reset().unwrap();
    let joint = world
        .add_world_revolute(body, [20.0, 0.0, 0.0], axis)
        .unwrap();
    assert_eq!(joint.raw(), 0);
}

#[test]
fn rejected_construction_and_foreign_resets_preserve_ids_and_motion() {
    let mut world = backend();
    let mut other = backend();
    let speed = BodySpec {
        velocity: [1.0, 2.0, 0.0],
        mass: None,
        ..sphere()
    };
    assert_eq!(
        world.spawn_kinematic_link(
            BodySpec {
                mass: Some(-1.0),
                ..speed
            },
            Default::default(),
            None
        ),
        Err(PhysicsError::InvalidValue)
    );
    let kin = world
        .spawn_kinematic_link(speed, Default::default(), None)
        .unwrap();
    assert_eq!(kin.raw(), 0);
    let initial = world.initial_state();
    assert!(world.owns_initial_state(&initial));
    assert!(!other.owns_initial_state(&initial));
    assert!(matches!(
        other.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
    let prepared = world.prepare_initial(&initial).unwrap();
    assert_eq!(
        other.apply_initial(prepared),
        Err(PhysicsError::ForeignInitialState)
    );
    let first = world.state(kin);
    assert!(world.remove(kin));
    let transient = world
        .spawn_link(sphere(), Default::default(), None)
        .unwrap();
    let prepared = world.prepare_initial(&initial).unwrap();
    world.apply_initial(prepared).unwrap();
    assert_eq!(world.state(transient), None);
    assert_eq!(world.state(kin), first);
    let next = world
        .spawn_link(
            BodySpec {
                position: [10.0; 3],
                ..sphere()
            },
            Default::default(),
            None,
        )
        .unwrap();
    assert!(next > transient);
    world.step(Duration::from_millis(250)).unwrap();
    assert_eq!(world.state(kin).unwrap().velocity, speed.velocity);
    assert!((world.state(kin).unwrap().position[1] - 0.5).abs() < 1e-12);
}

#[test]
fn rotating_boundary_converges_to_the_constant_twist_solution() {
    let error_at = |steps: u32| {
        let mut world = backend();
        let id = world
            .spawn_kinematic_link(
                sphere(),
                RotationSpec {
                    angular_velocity: [0.0, 0.0, 1.0],
                    ..Default::default()
                },
                None,
            )
            .unwrap();
        let dt = Duration::from_secs(1) / steps;
        for _ in 0..steps {
            world.step(dt).unwrap();
        }
        let q = world.state(id).unwrap().orientation;
        (1.0 - 2.0 * q[2].atan2(q[3])).abs()
    };
    let coarse = error_at(10);
    let fine = error_at(20);
    println!("constant-twist angular error (rad), dt=100ms {coarse:.12e}, dt=50ms {fine:.12e}");
    // Normalized Euler rotates by 2 atan(omega*h/2), giving O(h^2)
    // global error. Solver subdivisions can further reduce this upper bound.
    assert!(coarse < 0.1_f64.powi(2) / 12.0 + 1e-12, "{coarse}");
    assert!(fine <= 0.26 * coarse + 1e-12, "{coarse}, {fine}");
}

#[test]
fn overflow_during_integration_is_reported_before_bevy_commits() {
    use gz_sim_core::Simulation;
    let mut world = backend();
    // The pose, twist, mass and COM all start finite. Rotating a body whose
    // COM is near the f64 limit makes the next link pose unrepresentable.
    let body = world
        .spawn_link(
            sphere(),
            RotationSpec {
                angular_velocity: [0.0, 0.0, 1.0],
                ..Default::default()
            },
            Some(Inertia {
                center: [1.7e308, 1.7e308, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(700)).unwrap();
    gz_physics::install(&mut sim, world).unwrap();
    let result = sim.step_exact(1);
    assert!(
        result.is_err(),
        "result={result:?}, state={:?}",
        sim.world()
            .resource::<gz_physics::PhysicsBackend>()
            .state(body)
    );
    assert_eq!(
        sim.fault().unwrap().downcast_ref::<PhysicsError>(),
        Some(&PhysicsError::NonFiniteStep {
            engine: "multifidelity"
        })
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(sim.is_faulted());
    assert!(
        sim.world()
            .resource::<gz_physics::PhysicsBackend>()
            .state(body)
            .unwrap()
            .position
            .iter()
            .all(|v| v.is_finite())
    );
}

#[test]
fn ordinary_rapier_requires_explicit_backend_selection_for_kinematic_links() {
    let mut rapier = gz_physics_rapier::RapierBackend::new([0.0; 3]).unwrap();
    let spec = BodySpec {
        velocity: [4.0, 0.0, 0.0],
        ..sphere()
    };
    assert_eq!(
        rapier.spawn_kinematic_link(spec, Default::default(), None),
        Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "kinematic link"
        })
    );
    let id = rapier.spawn_link(spec, Default::default(), None).unwrap();
    assert_eq!(id.raw(), 0);
    assert_eq!(rapier.is_dynamic(id), Some(true));
    assert_eq!(rapier.is_kinematic(id), Some(false));
    assert!(rapier.remove(id));
    assert_eq!(rapier.is_kinematic(id), None);
}

#[test]
fn prescribed_speed_limits_refuse_before_changing_commands_or_advancing() {
    let mut world = backend();
    let zero = [0.0; 3];
    // Finite inputs may still overflow a norm or exceed the solver's cap.
    for velocity in [
        [1.0e308, 0.0, 0.0],
        [300.0, 300.0, 0.0],
        [f64::from_bits(1), 0.0, 0.0],
    ] {
        assert_eq!(
            world.spawn_kinematic_link(
                BodySpec {
                    velocity,
                    ..sphere()
                },
                Default::default(),
                None
            ),
            Err(PhysicsError::InvalidValue)
        );
    }
    let spin = [0.0, 0.0, 2.0];
    let kin = world
        .spawn_kinematic_link(
            sphere(),
            RotationSpec {
                angular_velocity: spin,
                ..Default::default()
            },
            None,
        )
        .unwrap();
    assert_eq!(kin.raw(), 0);
    let initial = world.state(kin).unwrap();
    let initial_world = world.initial_state();
    assert_eq!(
        world.validate_kinematic_velocity([400.0, 0.0, 0.0], spin, Duration::from_millis(1)),
        Ok(())
    );
    assert_eq!(
        world.set_world_velocity(kin, [401.0, 0.0, 0.0], zero),
        Err(PhysicsError::InvalidValue)
    );
    let invalid = gz_physics::BodyState {
        angular_velocity: [f64::MAX; 3],
        ..initial
    };
    assert_eq!(
        world.validate_body_state(kin, invalid),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        world.set_body_state(kin, invalid),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(world.state(kin), Some(initial));

    let free = world
        .spawn_link(
            BodySpec {
                position: [5.0, 0.0, 0.0],
                ..sphere()
            },
            Default::default(),
            None,
        )
        .unwrap();
    world.add_world_wrench(free, [2.0, 0.0, 0.0], zero).unwrap();
    assert_eq!(
        world.step(Duration::from_secs(1)),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(world.state(kin), Some(initial));
    assert_eq!(world.state(free).unwrap().velocity, zero);
    world.step(Duration::from_millis(100)).unwrap();
    assert!((world.state(free).unwrap().velocity[0] - 0.2).abs() < 1e-12);
    world.set_world_velocity(kin, zero, zero).unwrap();
    world.step(Duration::from_secs(1)).unwrap();

    world.reset().unwrap();
    assert_eq!(
        world.step(Duration::from_secs(1)),
        Err(PhysicsError::InvalidValue)
    );
    assert!(world.remove(kin));
    world.step(Duration::from_secs(1)).unwrap();
    let prepared = world.prepare_initial(&initial_world).unwrap();
    world.apply_initial(prepared).unwrap();
    assert_eq!(
        world.step(Duration::from_secs(1)),
        Err(PhysicsError::InvalidValue)
    );
}
