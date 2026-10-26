// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::BodyState;
use gz_physics_tpe::{ModelBounds, Motion, install};
use gz_sim_core::Simulation;
use std::time::Duration;

#[test]
fn static_bounds_hold_pose_and_clear_stored_velocity_through_pause_step_and_reset() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim).unwrap();
    let initial = BodyState {
        position: [-0.0, 2.0, 3.0],
        orientation: glam::DQuat::from_rotation_x(0.7).to_array(),
        velocity: [f64::MAX; 3],
        angular_velocity: [f64::MAX; 3],
    };
    let entity = sim
        .world_mut()
        .spawn((
            Motion::new(initial).unwrap(),
            ModelBounds::new([-1.0; 3], [1.0; 3], true, u16::MAX).unwrap(),
        ))
        .id();
    for reset in [false, true] {
        if reset {
            sim.reset().unwrap();
        } else {
            sim.tick().unwrap();
        }
        for steps in [0, 10] {
            sim.step_exact(steps).unwrap();
            let state = sim.world().get::<Motion>(entity).unwrap().state();
            assert_eq!(
                state.position.map(f64::to_bits),
                initial.position.map(f64::to_bits)
            );
            assert_eq!(
                state.orientation.map(f64::to_bits),
                initial.orientation.map(f64::to_bits)
            );
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
            assert!(sim.fault().is_none());
        }
        sim.world_mut()
            .get_mut::<Motion>(entity)
            .unwrap()
            .set_world_velocity([3.0; 3], [2.0; 3])
            .unwrap();
    }
}

#[test]
fn dynamic_and_unbounded_motion_still_advance_normally() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim).unwrap();
    let initial = BodyState {
        position: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        velocity: [1.0, -2.0, 0.5],
        angular_velocity: [0.0; 3],
    };
    let plain = sim.world_mut().spawn(Motion::new(initial).unwrap()).id();
    let dynamic = sim
        .world_mut()
        .spawn((
            Motion::new(initial).unwrap(),
            ModelBounds::new([-1.0; 3], [1.0; 3], false, u16::MAX).unwrap(),
        ))
        .id();
    sim.step_exact(100).unwrap();
    for entity in [plain, dynamic] {
        let state = sim.world().get::<Motion>(entity).unwrap().state();
        for (actual, expected) in state.position.into_iter().zip([0.1, -0.2, 0.05]) {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert_eq!(state.velocity, initial.velocity);
        assert_eq!(state.orientation, initial.orientation);
    }
}

#[test]
fn native_static_observations_are_retained_with_explicit_corrected_rust_policy() {
    let proof: serde_json::Value =
        serde_json::from_str(include_str!("../../../planning/reference/tpe-policy.json")).unwrap();
    for fixed in [false, true] {
        let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
        install(&mut sim).unwrap();
        let initial = BodyState {
            position: [0.0, 0.0, 5.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            velocity: [1.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 1.0],
        };
        let motion = if fixed {
            Motion::new_fixed(initial)
        } else {
            Motion::new(initial)
        }
        .unwrap();
        let entity = sim.world_mut().spawn(motion).id();
        let rows: Vec<_> = proof["static"]["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|r| r["static"] == fixed)
            .collect();
        assert_eq!(rows.len(), 3);
        for (index, native) in rows.into_iter().enumerate() {
            assert_eq!(native["iteration"], index + 1);
            sim.step_exact(1).unwrap();
            let state = sim.world().get::<Motion>(entity).unwrap().state();
            for (name, actual, corrected) in [
                (
                    "position",
                    state.position.as_slice(),
                    initial.position.as_slice(),
                ),
                (
                    "orientation",
                    state.orientation.as_slice(),
                    initial.orientation.as_slice(),
                ),
                ("velocity", state.velocity.as_slice(), &[0.0; 3]),
                (
                    "angular_velocity",
                    state.angular_velocity.as_slice(),
                    &[0.0; 3],
                ),
            ] {
                let values = native[name].as_array().unwrap();
                assert_eq!(values.len(), actual.len());
                for ((actual, native), corrected) in actual.iter().zip(values).zip(corrected) {
                    let expected = if fixed {
                        *corrected
                    } else {
                        native.as_f64().unwrap()
                    };
                    assert!(
                        (actual - expected).abs() < 1e-12,
                        "{fixed} {name}: {native}"
                    );
                }
            }
            if fixed {
                assert!(
                    (native["position"][0].as_f64().unwrap() - (index + 1) as f64 * 0.01).abs()
                        < 1e-12
                );
                assert_eq!(native["velocity"][0], 1);
                assert!(native["orientation"][2].as_f64().unwrap() > 0.0);
                assert_eq!(native["angular_velocity"][2], 1);
            }
        }
    }
}
