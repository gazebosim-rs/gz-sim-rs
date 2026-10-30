// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsError, PlaneSpec, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use gz_physics_rapier::RapierBackend;
use std::time::Duration;

fn plane() -> PlaneSpec {
    PlaneSpec {
        position: [0.0, 0.0, 1.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        size: [2.0; 2],
    }
}
fn sphere(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}
fn step(backend: &mut DartBackend, count: usize) {
    for _ in 0..count {
        backend.step(Duration::from_millis(1)).unwrap();
    }
}
fn body(backend: &mut DartBackend, position: [f64; 3]) -> gz_physics::BodyId {
    backend
        .spawn_link(
            sphere(position),
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap()
}

#[test]
fn static_plane_keeps_link_frame_contacts_and_both_reset_paths() {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = backend.spawn_static_plane(plane()).unwrap();
    let ball = body(&mut backend, [0.0, 0.0, 3.0]);
    let original = backend.state(ground).unwrap();
    assert_eq!(original.position, plane().position);
    assert_eq!(original.orientation, plane().orientation);
    assert_eq!(backend.is_dynamic(ground), Some(false));
    let initial = backend.initial_state();
    step(&mut backend, 2000);
    let settled = backend.state(ball).unwrap();
    assert!((settled.position[2] - 1.5).abs() < 0.01, "{settled:?}");
    assert!(settled.velocity[2].abs() < 0.01, "{settled:?}");
    assert_eq!(backend.state(ground), Some(original));
    // Existing fixed-body velocity admission is a no-op, including nonzero input.
    backend
        .set_world_velocity(ground, [1.0; 3], [1.0; 3])
        .unwrap();
    step(&mut backend, 1);
    assert_eq!(backend.state(ground), Some(original));
    backend.reset().unwrap();
    assert_eq!(backend.state(ball).unwrap().position, [0.0, 0.0, 3.0]);
    step(&mut backend, 2000);
    assert_eq!(backend.state(ball), Some(settled));
    assert!(backend.remove(ground));
    step(&mut backend, 50);
    assert!(backend.state(ball).unwrap().position[2] < settled.position[2] - 0.005);
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(ground), Some(original));
    step(&mut backend, 2000);
    assert_eq!(backend.state(ball), Some(settled));
}

#[test]
fn native_dart_plane_extent_ignores_sdf_size() {
    let mut outcomes = Vec::new();
    for size in [[1.0; 2], [100_000.0; 2]] {
        let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        backend
            .spawn_static_plane(PlaneSpec { size, ..plane() })
            .unwrap();
        let ball = body(&mut backend, [50.0, 0.0, 3.0]);
        step(&mut backend, 1500);
        let state = backend.state(ball).unwrap();
        assert!((state.position[2] - 1.5).abs() < 0.01);
        outcomes.push(state);
    }
    assert_eq!(outcomes[0], outcomes[1]);
    let mut outside = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    outside.spawn_static_plane(plane()).unwrap();
    let ball = body(&mut outside, [1100.0, 0.0, 3.0]);
    step(&mut outside, 1000);
    assert!(outside.state(ball).unwrap().position[2] < 0.0);
}

#[test]
fn invalid_plane_admission_preserves_world_and_identifiers() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    for size in [
        [0.0, 1.0],
        [-1.0, 1.0],
        [f64::NAN, 1.0],
        [f64::INFINITY, 1.0],
        [f64::MAX, 1.0],
        [f64::from_bits(1), 1.0],
    ] {
        assert!(matches!(
            backend.spawn_static_plane(PlaneSpec { size, ..plane() }),
            Err(PhysicsError::InvalidValue)
        ));
    }
    for value in [
        PlaneSpec {
            position: [0.0, 0.0, f64::NAN],
            ..plane()
        },
        PlaneSpec {
            position: [0.0, 0.0, -500_000_000.0 + 1500.0],
            ..plane()
        },
        PlaneSpec {
            orientation: [0.0; 4],
            ..plane()
        },
    ] {
        assert!(matches!(
            backend.spawn_static_plane(value),
            Err(PhysicsError::InvalidValue)
        ));
    }
    let id = backend.spawn_static_plane(plane()).unwrap();
    assert_eq!(id.raw(), 0);
}

#[test]
fn unimplemented_engine_returns_explicit_capability_without_allocating() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    assert!(matches!(
        backend.spawn_static_plane(plane()),
        Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "static plane collider"
        })
    ));
    let id = backend
        .spawn_link(sphere([0.0; 3]), RotationSpec::default(), None)
        .unwrap();
    assert_eq!(id.raw(), 0);
}

fn array<const N: usize>(row: &serde_json::Value) -> [f64; N] {
    row.as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn assert_state(actual: gz_physics::BodyState, expected: &serde_json::Value, label: &str) {
    for (name, actual, expected) in [
        (
            "position",
            actual.position.to_vec(),
            array::<3>(&expected["position"]).to_vec(),
        ),
        (
            "orientation",
            actual.orientation.to_vec(),
            array::<4>(&expected["orientation"]).to_vec(),
        ),
        (
            "velocity",
            actual.velocity.to_vec(),
            array::<3>(&expected["velocity"]).to_vec(),
        ),
        (
            "angular_velocity",
            actual.angular_velocity.to_vec(),
            array::<3>(&expected["angular_velocity"]).to_vec(),
        ),
    ] {
        for (index, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= 1e-14,
                "{label} {name}[{index}]: {actual} != {expected}"
            );
        }
    }
}

#[test]
fn fixed_plane_trajectories_and_link_frames_match_pinned_native_engine() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/planes.json")).unwrap();
    let rows = fixture["rows"].as_array().unwrap();
    let cases: std::collections::BTreeSet<_> = rows
        .iter()
        .map(|row| row["case"].as_str().unwrap())
        .collect();
    assert_eq!(cases.len(), 7);
    assert_eq!(rows.len(), 49);
    for name in cases {
        let selected: Vec<_> = rows.iter().filter(|row| row["case"] == name).collect();
        let input = &selected[0]["input"];
        let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        let ground = backend
            .spawn_static_plane(PlaneSpec {
                position: array(&input["position"]),
                orientation: array(&input["orientation"]),
                size: array(&input["size"]),
            })
            .unwrap();
        let ball = body(&mut backend, array(&input["ball"]));
        let mut previous = 0;
        for row in selected {
            let current = row["step"].as_u64().unwrap() as usize;
            step(&mut backend, current - previous);
            previous = current;
            assert_state(
                backend.state(ground).unwrap(),
                &row["ground"],
                &format!("{name}:{current}:ground"),
            );
            assert_state(
                backend.state(ball).unwrap(),
                &row["ball"],
                &format!("{name}:{current}:ball"),
            );
        }
    }
}

#[test]
fn conservative_grid_admission_covers_both_proxy_and_translated_bounds() {
    let angle = std::f64::consts::FRAC_PI_2;
    let turned = [0.0, (angle * 0.5).sin(), 0.0, (angle * 0.5).cos()];
    for value in [
        PlaneSpec {
            position: [0.0, 0.0, 500_000_000.0 - 2000.0],
            ..plane()
        },
        PlaneSpec {
            position: [0.0, 0.0, -500_000_000.0 + 2200.0],
            ..plane()
        },
        PlaneSpec {
            position: [-500_000_000.0 + 2200.0, 0.0, 0.0],
            orientation: turned,
            ..plane()
        },
    ] {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let id = backend.spawn_static_plane(value).unwrap();
        let initial = backend.state(id).unwrap();
        step(&mut backend, 2);
        assert_eq!(backend.state(id), Some(initial));
    }
    for value in [
        PlaneSpec {
            position: [0.0, 0.0, 500_000_000.0 - 500.0],
            ..plane()
        },
        PlaneSpec {
            position: [500_000_000.0 - 500.0, 0.0, 0.0],
            orientation: turned,
            ..plane()
        },
    ] {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        assert!(matches!(
            backend.spawn_static_plane(value),
            Err(PhysicsError::InvalidValue)
        ));
        assert_eq!(backend.spawn_static_plane(plane()).unwrap().raw(), 0);
    }
}
