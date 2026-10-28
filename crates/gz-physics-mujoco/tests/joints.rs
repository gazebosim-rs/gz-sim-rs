// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! World-revolute behavior ported from crates/gz-physics-rapier/tests/joints.rs.
use gz_physics::{BodyId, DynamicBackend, JointId, PhysicsBackend, PhysicsError, install};
use gz_physics_mujoco::{BodySpec, Inertia, MujocoBackend, RotationSpec, Shape};
use gz_sim_core::Simulation;
use std::time::Duration;

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

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]
}

fn pendulum() -> (Simulation, BodyId, JointId) {
    let theta = 0.02_f64;
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = mujoco
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
    let joint = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert!(mujoco.joint_state(joint).unwrap().position.abs() < 1e-12);
    assert_eq!(mujoco.is_jointed(body), Some(true));
    assert_eq!(mujoco.joint_child(joint), Some(body));
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, mujoco).unwrap();
    (sim, body, joint)
}

#[test]
fn world_revolute_keeps_anchor_and_small_angle_energy_and_resets_its_reference() {
    let (mut sim, body, joint) = pendulum();
    let initial = backend(&sim).state(body).unwrap();
    let initial_energy = 9.81 * (initial.position[2] + 1.0);
    sim.tick().unwrap();
    assert_eq!(backend(&sim).state(body).unwrap(), initial);
    for tick in 1..=4000 {
        sim.step_exact(1).unwrap();
        let mujoco = backend(&sim);
        let s = mujoco.state(body).unwrap();
        let j = mujoco.joint_state(joint).unwrap();
        let tip = rotate(s.orientation, [0.0, 0.0, 1.0]);
        let anchor: f64 = (0..3)
            .map(|i| (s.position[i] + tip[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        // A reduced-coordinate hinge holds the anchor to rounding error.
        assert!(anchor < 1e-12, "tick{tick} anchor {anchor}");
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
        assert!((s.angular_velocity[1] - j.velocity).abs() < 1e-12);
    }
    sim.reset().unwrap();
    assert_eq!(backend(&sim).state(body).unwrap(), initial);
    let state = backend(&sim).joint_state(joint).unwrap();
    assert_eq!(state.position, 0.0);
    assert_eq!(state.velocity, 0.0);
    sim.step_exact(1).unwrap();
    assert!(backend(&sim).joint_state(joint).unwrap().velocity < 0.0);
}

#[test]
fn joint_retirement_and_prepared_reset_restore_constraints_without_id_reuse() {
    let (mut sim, body, joint) = pendulum();
    let initial = backend(&sim).initial_state();
    sim.step_exact(10).unwrap();
    let mujoco = backend_mut(&mut sim);
    assert_eq!(
        mujoco.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
    assert!(mujoco.remove_joint(joint));
    assert!(!mujoco.remove_joint(joint));
    assert!(mujoco.joint_state(joint).is_none());
    assert_eq!(mujoco.joint_child(joint), None);
    assert_eq!(mujoco.is_jointed(body), Some(false));
    mujoco.reset().unwrap();
    let added = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(added, joint);
    let prepared = mujoco.prepare_initial(&initial).unwrap();
    mujoco.apply_initial(prepared).unwrap();
    assert!(mujoco.joint_state(joint).is_some());
    assert!(mujoco.joint_state(added).is_none());
    assert!(mujoco.remove(body));
    assert!(mujoco.joint_state(joint).is_none());
    assert_eq!(mujoco.is_jointed(body), None);
    let prepared = mujoco.prepare_initial(&initial).unwrap();
    mujoco.apply_initial(prepared).unwrap();
    assert!(mujoco.joint_state(joint).is_some());
    // MuJoCo joints add degrees of freedom rather than constraints, so a second
    // world joint on one body is rejected without consuming an identifier.
    assert_eq!(
        mujoco.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::Unsupported {
            engine: "mujoco",
            capability: "more than one world joint on a body",
        })
    );
    assert!(mujoco.remove_joint(joint));
    let another = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(another.raw(), added.raw() + 1);
    assert_ne!(another, joint);
    let foreign = MujocoBackend::new([0.0; 3]).unwrap();
    assert!(matches!(
        mujoco.prepare_initial(&foreign.initial_state()),
        Err(PhysicsError::ForeignInitialState)
    ));
}

#[test]
fn world_revolute_tracks_complete_turns_in_both_directions_and_reset() {
    for speed in [-4.0_f64, 4.0] {
        let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
        let body = mujoco
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            })
            .unwrap();
        let joint = mujoco
            .add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0])
            .unwrap();
        mujoco
            .set_world_velocity(body, [0.0; 3], [0.0, 0.0, speed])
            .unwrap();
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        install(&mut sim, mujoco).unwrap();
        // MuJoCo integrates hinge angles linearly: q(n) = n * omega * dt.
        for tick in 1..=5000 {
            sim.step_exact(1).unwrap();
            let state = backend(&sim).joint_state(joint).unwrap();
            assert!(
                (state.position - speed * 0.001 * tick as f64).abs() < 1e-10,
                "tick{tick}: angle {}",
                state.position
            );
            assert!((state.velocity - speed).abs() < 1e-10);
        }
        sim.reset().unwrap();
        let state = backend(&sim).joint_state(joint).unwrap();
        assert_eq!(state.position, 0.0);
        assert_eq!(state.velocity, 0.0);
    }
}

#[test]
fn invalid_joint_construction_is_atomic_including_exhaustion_and_large_offsets() {
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 0.1 },
        position: [0.0, 0.0, -1.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let body = mujoco.spawn(spec).unwrap();
    let before = mujoco.state(body).unwrap();
    for (anchor, axis) in [
        ([f64::INFINITY, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0; 3], [0.0; 3]),
        ([0.0; 3], [f64::NAN, 0.0, 0.0]),
        ([0.0; 3], [1e308, 1e308, 0.0]),
        ([0.0; 3], [1e-300, 0.0, 0.0]),
    ] {
        assert_eq!(
            mujoco.add_world_revolute(body, anchor, axis),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(mujoco.is_jointed(body), Some(false));
        assert_eq!(mujoco.state(body).unwrap(), before);
    }
    let fixed = mujoco
        .spawn(BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..spec
        })
        .unwrap();
    assert_eq!(
        mujoco.add_world_revolute(fixed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    let removed = mujoco.spawn(spec).unwrap();
    assert!(mujoco.remove(removed));
    assert_eq!(
        mujoco.add_world_revolute(removed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::MissingBody)
    );
    // A finite world anchor whose body-frame expression overflows.
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotated = mujoco
        .spawn_link(
            spec,
            RotationSpec {
                orientation: [0.0, 0.0, s, s],
                angular_velocity: [0.0; 3],
            },
            None,
        )
        .unwrap();
    assert_eq!(
        mujoco.add_world_revolute(rotated, [f64::MAX, 0.0, 0.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(mujoco.is_jointed(rotated), Some(false));
    assert_eq!(mujoco.is_jointed(BodyId::from_raw(999)), None);
    assert_eq!(mujoco.joint_state(JointId::from_raw(999)), None);
    assert!(!mujoco.remove_joint(JointId::from_raw(999)));
}

#[test]
fn rotated_child_hinge_axis_and_offset_anchor_follow_world_coordinates() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = mujoco
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.1, 0.2, 0.3],
                },
                position: [1.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(2.0),
            },
            RotationSpec {
                orientation: [s, 0.0, 0.0, s],
                angular_velocity: [0.0; 3],
            },
            None,
        )
        .unwrap();
    let joint = mujoco
        .add_world_revolute(body, [0.0, 0.0, 0.0], [0.0, 2.0, 0.0])
        .unwrap();
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(200).unwrap();
    let mujoco = backend(&sim);
    let state = mujoco.state(body).unwrap();
    let hinge = mujoco.joint_state(joint).unwrap();
    // The body swings down about world +Y, which is a positive hinge rotation:
    // rotating +X about +Y moves it toward -Z.
    assert!(hinge.position > 0.1, "{}", hinge.position);
    let radius = (state.position[0].powi(2) + state.position[2].powi(2)).sqrt();
    assert!((radius - 1.0).abs() < 1e-12);
    assert!(state.position[1].abs() < 1e-12);
    assert!((state.position[2] + hinge.position.sin()).abs() < 1e-12);
    assert!((state.angular_velocity[1] - hinge.velocity).abs() < 1e-12);
}
