// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodySpec, DynamicBackend, RapierBackend, RotationSpec, Shape};
use rapier3d_f64::prelude::{Rotation, Vector};
use std::time::Duration;
#[test]
fn rotating_world_axis_keeps_coordinate_and_velocity_consistent() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut make = |angular, rotation| {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec {
                    angular_velocity: angular,
                    orientation: Rotation::from_scaled_axis(rotation).to_array(),
                },
                None,
            )
            .unwrap()
    };
    let parent = make([0.6, 0.2, -0.3], Vector::new(0.4, -0.3, 0.2));
    let child = make([0.6, 1.2, -0.3], Vector::new(-0.2, 0.1, 0.5));
    let initial = Rotation::from_array(backend.state(parent).unwrap().orientation);
    let local_axis = initial.inverse() * Vector::Y;
    let joint = backend
        .add_reduced_revolute(Some(parent), child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let mut previous = backend.joint_state(joint).unwrap();
    let mut max_error = 0.0_f64;
    for _ in 0..1000 {
        backend.step(Duration::from_micros(100)).unwrap();
        let p = backend.state(parent).unwrap();
        let c = backend.state(child).unwrap();
        let sample = backend.joint_state(joint).unwrap();
        let axis = Rotation::from_array(p.orientation) * local_axis;
        let relative =
            Vector::from_array(c.angular_velocity) - Vector::from_array(p.angular_velocity);
        assert!((relative - axis * relative.dot(axis)).length() < 1e-8);
        max_error = max_error.max(
            (sample.position - previous.position - 0.00005 * (sample.velocity + previous.velocity))
                .abs(),
        );
        previous = sample;
    }
    let axis = Rotation::from_array(backend.state(parent).unwrap().orientation) * local_axis;
    assert!((axis - Vector::Y).length() > 0.03);
    println!(
        "maximum midpoint angle error={max_error:e}; axis change={:e}",
        (axis - Vector::Y).length()
    );
    assert!(max_error < 1e-7);
}
