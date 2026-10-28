// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ordinary scenes that diverge in DART itself. Two reviewer search scenes
//! (seeds 215 and 2) grow a body's spin exponentially until it is refused.
//! tests/fixtures/native_scene.cc ran the same bodies through the installed
//! gz-physics dartsim plugin: its trajectory matches the adapter's at every
//! recorded sample, and it then aborts the process in ODE (docs/DART.md).

use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::time::Duration;

/// Spawns `scene` (one body per line, as written by the native comparison).
fn build(scene: &str) -> (DartBackend, Vec<BodyId>) {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = BodySpec {
        shape: Shape::Box {
            half_extents: [50.0, 50.0, 0.5],
        },
        position: [0.0, 0.0, -0.5],
        velocity: [0.0; 3],
        mass: None,
    };
    backend
        .spawn_link(floor, RotationSpec::default(), None)
        .unwrap();
    let ids = scene
        .lines()
        .map(|line| {
            let mut fields = line.split_whitespace();
            let kind = fields.next().unwrap();
            let v: Vec<f64> = fields.map(|f| f.parse().unwrap()).collect();
            let shape = match kind {
                "sphere" => Shape::Sphere { radius: v[0] },
                "box" => Shape::Box {
                    half_extents: [v[0] / 2.0, v[1] / 2.0, v[2] / 2.0],
                },
                "cylinder" => Shape::Cylinder {
                    radius: v[0],
                    half_height: v[1] / 2.0,
                },
                _ => Shape::Capsule {
                    radius: v[0],
                    half_length: v[1] / 2.0,
                },
            };
            backend
                .spawn_link(
                    BodySpec {
                        shape,
                        position: [v[7], v[8], v[9]],
                        velocity: [v[14], v[15], v[16]],
                        mass: Some(v[3]),
                    },
                    RotationSpec {
                        orientation: [v[10], v[11], v[12], v[13]],
                        angular_velocity: [v[17], v[18], v[19]],
                    },
                    Some(Inertia {
                        center: [0.0; 3],
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        tensor: [v[4], v[5], v[6], 0.0, 0.0, 0.0],
                    }),
                )
                .unwrap()
        })
        .collect();
    (backend, ids)
}

/// Steps until refused, comparing each native sample (every 100 steps: largest
/// body spin, first body position). Returns the refused step.
fn diverge(scene: &str, native: &str) -> usize {
    let (mut backend, ids) = build(scene);
    let mut samples = native.lines().map(|line| {
        let f: Vec<&str> = line.split_whitespace().collect();
        let step: usize = f[1].parse().unwrap();
        let spin: f64 = f[3].parse().unwrap();
        let position: Vec<f64> = f[5..8].iter().map(|v| v.parse().unwrap()).collect();
        (step, spin, position)
    });
    let mut next = samples.next();
    for step in 1..=3000 {
        if let Err(error) = backend.step(Duration::from_millis(1)) {
            // Native DART is past every recorded sample by now.
            assert!(next.is_none(), "refused at {step} before {next:?}: {error}");
            return step;
        }
        if let Some((at, spin, position)) = &next
            && *at == step
        {
            let largest = ids
                .iter()
                .map(|&id| {
                    let w = backend.state(id).unwrap().angular_velocity;
                    w.iter().map(|v| v * v).sum::<f64>().sqrt()
                })
                .fold(0.0, f64::max);
            // The fixture prints 7 significant digits and 9 decimals.
            assert!(
                (largest - spin).abs() <= 1e-6 * spin,
                "step {step}: {largest} {spin}"
            );
            let actual = backend.state(ids[0]).unwrap().position;
            for (a, n) in actual.iter().zip(position) {
                assert!((a - n).abs() < 1e-8, "step {step}: {actual:?} {position:?}");
            }
            next = samples.next();
        }
    }
    panic!("never refused");
}

#[test]
fn capsule_and_sphere_scene_diverges_like_native_dart() {
    let refused = diverge(
        include_str!("fixtures/divergent_scene_215.txt"),
        include_str!("fixtures/divergent_scene_215_native.txt"),
    );
    // Native aborted after its last sample at step 2000.
    assert!((2001..2100).contains(&refused), "{refused}");
}

#[test]
fn mixed_scene_with_cylinders_diverges_like_native_dart() {
    let refused = diverge(
        include_str!("fixtures/divergent_scene_2.txt"),
        include_str!("fixtures/divergent_scene_2_native.txt"),
    );
    assert!((2701..2800).contains(&refused), "{refused}");
}
