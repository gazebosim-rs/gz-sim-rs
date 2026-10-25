// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, Inertia, JointId, PhysicsError, RapierBackend, RotationSpec, Shape, install,
};
use gz_sim_core::Simulation;
use std::time::Duration;

fn pendulum() -> (Simulation, BodyId, JointId) {
    let theta = 0.02_f64;
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
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
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, backend).unwrap();
    (sim, body, joint)
}

#[test]
fn world_revolute_keeps_anchor_and_small_angle_energy_and_resets_its_reference() {
    let (mut sim, body, joint) = pendulum();
    let initial = *sim
        .world()
        .resource::<RapierBackend>()
        .state(body)
        .as_ref()
        .unwrap();
    let initial_energy = 9.81 * (initial.position[2] + 1.0);
    sim.tick().unwrap();
    assert_eq!(
        sim.world().resource::<RapierBackend>().state(body).unwrap(),
        initial
    );
    for tick in 1..=4000 {
        sim.step_exact(1).unwrap();
        let backend = sim.world().resource::<RapierBackend>();
        let s = backend.state(body).unwrap();
        let j = backend.joint_state(joint).unwrap();
        let q = rapier3d_f64::prelude::Rotation::from_xyzw(
            s.orientation[0],
            s.orientation[1],
            s.orientation[2],
            s.orientation[3],
        );
        let anchor = rapier3d_f64::prelude::Vector::from_array(s.position)
            + q * rapier3d_f64::prelude::Vector::Z;
        assert!(anchor.length() < 1e-5, "tick{tick} anchor {anchor:?}");
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
    assert_eq!(
        sim.world().resource::<RapierBackend>().state(body).unwrap(),
        initial
    );
    let state = sim
        .world()
        .resource::<RapierBackend>()
        .joint_state(joint)
        .unwrap();
    assert!(state.position.abs() < 1e-12);
    assert_eq!(state.velocity, 0.0);
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .resource::<RapierBackend>()
            .joint_state(joint)
            .unwrap()
            .velocity
            < 0.0
    );
}

#[test]
fn joint_retirement_and_prepared_reset_restore_constraints_without_id_reuse() {
    let (mut sim, body, joint) = pendulum();
    let initial = sim.world().resource::<RapierBackend>().initial_state();
    sim.step_exact(10).unwrap();
    let mut backend = sim.world_mut().resource_mut::<RapierBackend>();
    assert_eq!(
        backend.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
    assert!(backend.remove_joint(joint));
    assert!(!backend.remove_joint(joint));
    assert!(backend.joint_state(joint).is_none());
    assert_eq!(backend.is_jointed(body), Some(false));
    backend.reset();
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
    let another = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(another, added);
    assert_ne!(another, joint);
    let foreign = RapierBackend::new([0.0; 3]).unwrap();
    assert!(matches!(
        backend.prepare_initial(&foreign.initial_state()),
        Err(PhysicsError::ForeignInitialState)
    ));
}

#[test]
fn real_native_pendulum_joint_observations_and_reset_are_within_declared_bounds() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("fixtures/world_pendulum.json")).unwrap();
    let (mut sim, body, joint) = pendulum();
    let mut epoch = 0;
    let mut previous = 0;
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
        let backend = sim.world().resource::<RapierBackend>();
        let state = backend.state(body).unwrap();
        let hinge = backend.joint_state(joint).unwrap();
        // Different integrators; native body output also has a 1e-6 pose-update cache.
        assert!(
            (hinge.position - row["q"].as_f64().unwrap()).abs() < 1e-4,
            "epoch{epoch}/tick{tick}:{} versus{}",
            hinge.position,
            row["q"]
        );
        assert!(
            (hinge.velocity - row["dq"].as_f64().unwrap()).abs() < 3e-4,
            "epoch{epoch}/tick{tick}:{} versus{}",
            hinge.velocity,
            row["dq"]
        );
        for (actual, key) in [(state.position[0], "x"), (state.position[2], "z")] {
            assert!((actual - row[key].as_f64().unwrap()).abs() < 1e-4);
        }
    }
}

#[test]
fn world_revolute_tracks_complete_turns_in_both_directions_and_reset() {
    for speed in [-4.0_f64, 4.0] {
        // Rapier uses normalized quaternion Euler updates at each solver substep:
        // one update rotates by 2*atan(omega*substep/2), not exactly omega*substep.
        let n =
            rapier3d_f64::prelude::IntegrationParameters::default().num_solver_iterations as f64;
        let angle_per_step = 2.0 * n * (speed * 0.001 / (2.0 * n)).atan();
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let body = backend
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            })
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
            let state = sim
                .world()
                .resource::<RapierBackend>()
                .joint_state(joint)
                .unwrap();
            assert!(
                (state.position - angle_per_step * tick as f64).abs() < 1e-10,
                "tick{tick}: angle {} expected {}",
                state.position,
                angle_per_step * tick as f64
            );
            assert!((state.velocity - speed).abs() < 1e-10);
        }
        sim.reset().unwrap();
        let state = sim
            .world()
            .resource::<RapierBackend>()
            .joint_state(joint)
            .unwrap();
        assert_eq!(state.position, 0.0);
        assert_eq!(state.velocity, 0.0);
    }
}
