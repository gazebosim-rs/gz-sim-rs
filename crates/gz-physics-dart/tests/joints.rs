// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ports of gz-physics-rapier/tests/joints.rs.

use gz_physics::{
    BodyId, BodySpec, DynamicBackend, Inertia, JointId, PhysicsBackend, PhysicsError, RotationSpec,
    Shape, install,
};
use gz_physics_dart::DartBackend;
use gz_sim_core::Simulation;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::time::Duration;

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

fn pendulum() -> (Simulation, BodyId, JointId) {
    let theta = 0.02_f64;
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [-theta.sin(), 0.0, -theta.cos()],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [0.0, (theta / 2.0).sin(), 0.0, (theta / 2.0).cos()],
                angular_velocity: [0.0; 3],
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert!(backend.joint_state(joint).unwrap().position.abs() < 1e-12);
    assert_eq!(backend.is_jointed(body), Some(true));
    assert_eq!(backend.joint_child(joint), Some(body));
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, backend).unwrap();
    (sim, body, joint)
}

#[test]
fn world_revolute_keeps_anchor_and_small_angle_energy_and_resets_its_reference() {
    let (mut sim, body, joint) = pendulum();
    let initial = dart(&sim).state(body).unwrap();
    let initial_energy = 9.81 * (initial.position[2] + 1.0);
    sim.tick().unwrap();
    assert_eq!(dart(&sim).state(body).unwrap(), initial);
    for tick in 1..=4000 {
        sim.step_exact(1).unwrap();
        let s = dart(&sim).state(body).unwrap();
        let j = dart(&sim).joint_state(joint).unwrap();
        let q = s.orientation;
        let rotation = UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]));
        let anchor = Vector3::from(s.position) + rotation * Vector3::z();
        assert!(anchor.norm() < 1e-5, "tick{tick} anchor {anchor:?}");
        let speed2 = s.velocity.into_iter().map(|v| v * v).sum::<f64>();
        let omega2 = s.angular_velocity.into_iter().map(|v| v * v).sum::<f64>();
        let energy = 0.5 * speed2 + 0.002 * omega2 + 9.81 * (s.position[2] + 1.0);
        assert!(
            (energy - initial_energy).abs() < initial_energy * 0.02,
            "tick{tick} energy {energy}"
        );
        let expected = 0.02 * ((9.81_f64 / 1.004).sqrt() * tick as f64 * 0.001).cos() - 0.02;
        assert!(
            (j.position - expected).abs() < 1e-4,
            "tick{tick}:{} versus{expected}",
            j.position
        );
        assert!(s.angular_velocity[0].abs() < 1e-12 && s.angular_velocity[2].abs() < 1e-12);
    }
    sim.reset().unwrap();
    assert_eq!(dart(&sim).state(body).unwrap(), initial);
    let state = dart(&sim).joint_state(joint).unwrap();
    assert!(state.position.abs() < 1e-12);
    assert_eq!(state.velocity, 0.0);
    sim.step_exact(1).unwrap();
    assert!(dart(&sim).joint_state(joint).unwrap().velocity < 0.0);
}

#[test]
fn joint_retirement_and_prepared_reset_restore_constraints_without_id_reuse() {
    let (mut sim, body, joint) = pendulum();
    let initial = dart(&sim).initial_state();
    sim.step_exact(10).unwrap();
    let backend = dart_mut(&mut sim);
    assert_eq!(
        backend.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
    let moving = backend.state(body).unwrap();
    assert!(backend.remove_joint(joint));
    assert!(!backend.remove_joint(joint));
    assert!(backend.joint_state(joint).is_none());
    assert_eq!(backend.joint_child(joint), None);
    assert_eq!(backend.is_jointed(body), Some(false));
    // The released body keeps its pose and velocity, then falls freely.
    let released = backend.state(body).unwrap();
    for (actual, expected) in released
        .position
        .into_iter()
        .chain(released.velocity)
        .zip(moving.position.into_iter().chain(moving.velocity))
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.state(body).unwrap().velocity[2] < released.velocity[2]);
    backend.reset().unwrap();
    let added = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(added, joint);
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert!(backend.joint_state(joint).is_some());
    assert!(backend.joint_state(added).is_none());
    assert!(backend.remove(body));
    assert!(backend.joint_state(joint).is_none());
    assert_eq!(backend.is_jointed(body), None);
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert!(backend.joint_state(joint).is_some());
    assert_eq!(
        backend.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::Unsupported {
            engine: "dart",
            capability: "more than one world joint on a body",
        })
    );
    assert!(backend.remove_joint(joint));
    let another = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(another, added);
    assert_ne!(another, joint);
    let foreign = DartBackend::new([0.0; 3]).unwrap();
    assert!(!backend.owns_initial_state(&foreign.initial_state()));
    assert!(backend.owns_initial_state(&initial));
    assert!(matches!(
        backend.prepare_initial(&foreign.initial_state()),
        Err(PhysicsError::ForeignInitialState)
    ));
}

/// The gz-sim Server fixture recorded for the Rapier slice (docs/JOINTS.md) also
/// runs DART 6.13.2 through the dartsim plugin, so the same engine reproduces it
/// far inside Rapier's cross-engine bounds.
#[test]
fn native_gz_sim_pendulum_observations_and_reset_match() {
    let rows: Vec<serde_json::Value> = serde_json::from_str(include_str!(
        "../../gz-physics-rapier/tests/fixtures/world_pendulum.json"
    ))
    .unwrap();
    let (mut sim, body, joint) = pendulum();
    let mut epoch = 0;
    let mut previous = 0;
    let mut worst_joint = 0.0_f64;
    for row in rows {
        let next_epoch = row["epoch"].as_u64().unwrap();
        let tick = row["iteration"].as_u64().unwrap();
        if next_epoch != epoch {
            sim.reset().unwrap();
            epoch = next_epoch;
            previous = 0;
        }
        if tick == previous {
            sim.tick().unwrap();
        } else {
            sim.step_exact(tick - previous).unwrap();
        }
        previous = tick;
        assert_eq!(
            sim.elapsed().as_nanos(),
            u128::from(row["time_ns"].as_u64().unwrap())
        );
        let state = dart(&sim).state(body).unwrap();
        let hinge = dart(&sim).joint_state(joint).unwrap();
        worst_joint = worst_joint
            .max((hinge.position - row["q"].as_f64().unwrap()).abs())
            .max((hinge.velocity - row["dq"].as_f64().unwrap()).abs());
        // Native body poses come from gz-sim's 1e-6 changed-pose cache.
        for (actual, key) in [(state.position[0], "x"), (state.position[2], "z")] {
            assert!((actual - row[key].as_f64().unwrap()).abs() < 2e-6);
        }
    }
    // Joint feedback has no cache; differences are solver-input rounding.
    assert!(worst_joint < 1e-12, "{worst_joint:e}");
}

#[test]
fn world_revolute_tracks_complete_turns_in_both_directions_and_reset() {
    for speed in [-4.0_f64, 4.0] {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let body = backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.1 },
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let joint = backend
            .add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0])
            .unwrap();
        backend
            .set_world_velocity(body, [0.0; 3], [0.0, 0.0, speed])
            .unwrap();
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        install(&mut sim, backend).unwrap();
        for tick in 1..=5000 {
            sim.step_exact(1).unwrap();
            let state = dart(&sim).joint_state(joint).unwrap();
            // DART integrates the joint coordinate itself: q += qdot * dt.
            assert!(
                (state.position - speed * 0.001 * tick as f64).abs() < 1e-10,
                "tick{tick}: angle {}",
                state.position
            );
            assert!((state.velocity - speed).abs() < 1e-10);
        }
        sim.reset().unwrap();
        let state = dart(&sim).joint_state(joint).unwrap();
        assert_eq!(state.position, 0.0);
        assert_eq!(state.velocity, 0.0);
    }
}

#[test]
fn offset_hinge_takes_least_squares_rates_and_impulses_through_its_constraint() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    // Unit point mass 1 m from a Z hinge: rotational inertia 0.004 + 1.
    let body = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [1.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    // The COM twist for rate w is (w*z, w*y); this request is consistent with w = 2.
    backend
        .set_world_velocity(body, [0.0, 2.0, 0.0], [0.0, 0.0, 2.0])
        .unwrap();
    assert!((backend.joint_state(joint).unwrap().velocity - 2.0).abs() < 1e-12);
    // A tangential impulse p at r = 1 changes the rate by p / (I + m r^2).
    backend
        .apply_impulse(body, [0.0, 1.004, 0.0], [0.0; 3])
        .unwrap();
    assert!((backend.joint_state(joint).unwrap().velocity - 3.0).abs() < 1e-12);
    assert_eq!(
        backend.apply_impulse(body, [0.0, f64::MAX, 0.0], [0.0, 0.0, f64::MAX]),
        Err(PhysicsError::InvalidValue)
    );
    assert!((backend.joint_state(joint).unwrap().velocity - 3.0).abs() < 1e-12);
    // Linear velocity along the hinge axis is not representable and fits to zero.
    backend
        .set_world_velocity(body, [0.0, 0.0, 5.0], [0.0; 3])
        .unwrap();
    assert_eq!(backend.joint_state(joint).unwrap().velocity, 0.0);
    // Wrenches act through the constraint too: torque tau gives tau/I_axis * dt.
    backend
        .add_world_wrench(body, [0.0; 3], [0.0, 0.0, 1.004])
        .unwrap();
    backend.step(Duration::from_millis(10)).unwrap();
    assert!((backend.joint_state(joint).unwrap().velocity - 0.01).abs() < 1e-12);
}

/// Joint rate right after construction, after one 1 ms step, and after reset,
/// for a bob 1 m under a Y hinge spawned already swinging at 1 rad/s.
fn swinging_hinge_rates(backend: &mut dyn DynamicBackend) -> [f64; 3] {
    let body = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [0.0, 0.0, -1.0],
                velocity: [-1.0, 0.0, 0.0],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0, 1.0, 0.0],
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let constructed = backend.joint_state(joint).unwrap().velocity;
    backend.step(Duration::from_millis(1)).unwrap();
    let stepped = backend.joint_state(joint).unwrap().velocity;
    backend.reset().unwrap();
    [
        constructed,
        stepped,
        backend.joint_state(joint).unwrap().velocity,
    ]
}

#[test]
fn hinge_construction_keeps_the_body_motion_it_allows_like_rapier() {
    let mut dart = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let mut rapier = gz_physics_rapier::RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let dart = swinging_hinge_rates(&mut dart);
    let rapier = swinging_hinge_rates(&mut rapier);
    // The spawn twist is exactly the hinge motion at 1 rad/s.
    assert!((dart[0] - 1.0).abs() < 1e-12, "{dart:?}");
    assert!((dart[2] - 1.0).abs() < 1e-12, "{dart:?}");
    for i in [0, 2] {
        assert!(
            (dart[i] - rapier[i]).abs() < 1e-12,
            "{dart:?} versus {rapier:?}"
        );
    }
    // Over the step the swing raises the bob and gravity slows it by about
    // (g/L)*dt^2/2 = 4.9e-6 rad/s. Semi-implicit Euler (DART) applies that torque
    // a step later; Rapier's substeps apply part of it now (it reports 0.9999962).
    assert!(
        (dart[1] - rapier[1]).abs() < 1e-5,
        "{dart:?} versus {rapier:?}"
    );
}
