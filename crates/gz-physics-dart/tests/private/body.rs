// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn spec(shape: Shape) -> BodySpec {
    BodySpec {
        shape,
        position: [0.0; 3],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

#[test]
fn dart_dimensions_use_full_sizes_and_cylindrical_lengths() {
    let cases = [
        (Shape::Sphere { radius: 0.5 }, 0, [0.5, 0.0, 0.0]),
        (
            Shape::Box {
                half_extents: [0.5, 1.0, 1.5],
            },
            1,
            [1.0, 2.0, 3.0],
        ),
        (
            Shape::Cylinder {
                radius: 0.6,
                half_height: 1.4,
            },
            2,
            [0.6, 2.8, 0.0],
        ),
        (
            Shape::Capsule {
                radius: 0.6,
                half_length: 1.4,
            },
            3,
            [0.6, 2.8, 0.0],
        ),
    ];
    for (shape, code, dimensions) in cases {
        let config = config(spec(shape), RotationSpec::default(), None).unwrap();
        assert_eq!((config.shape, config.dimensions), (code, dimensions));
        assert!(config.mobile);
    }
    let fixed = config(
        BodySpec {
            mass: None,
            ..spec(Shape::Sphere { radius: 0.5 })
        },
        RotationSpec::default(),
        None,
    )
    .unwrap();
    assert!(!fixed.mobile);
    assert_eq!(fixed.mass, 1.0);
}

#[test]
fn rotated_extents_bound_representable_positions() {
    let quarter_x = Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    let extents = |shape| geometry(shape, &quarter_x).unwrap().2;
    assert_eq!(
        extents(Shape::Box {
            half_extents: [0.5, 1.0, 1.5],
        }),
        [0.5, 1.5, 1.0]
    );
    assert_eq!(
        extents(Shape::Cylinder {
            radius: 0.6,
            half_height: 1.4,
        }),
        [0.6, 1.4, 0.6]
    );
    assert_eq!(
        extents(Shape::Capsule {
            radius: 0.6,
            half_length: 1.4,
        }),
        [0.6, 2.0, 0.6]
    );
    // The thinnest admitted disk cannot offset from z = 5, but is valid once its axis lies along
    // world Y at y = 0, where the tiny extent stays representable.
    let disk = BodySpec {
        position: [0.0, 0.0, 5.0],
        ..spec(Shape::Cylinder {
            radius: 0.6,
            half_height: 1e-60,
        })
    };
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotated = |orientation| RotationSpec {
        orientation,
        angular_velocity: [0.0; 3],
    };
    assert_eq!(
        config(disk, rotated([0.0, 0.0, 0.0, 1.0]), None).map(|c| c.shape),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        config(disk, rotated([s, 0.0, 0.0, s]), None).map(|c| c.shape),
        Ok(2)
    );
}

#[test]
fn explicit_inertia_is_rotated_into_body_axes_about_its_center() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let config = config(
        spec(Shape::Sphere { radius: 0.5 }),
        RotationSpec::default(),
        Some(Inertia {
            center: [0.1, 0.2, 0.3],
            orientation: [0.0, 0.0, s, s],
            tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
        }),
    )
    .unwrap();
    assert_eq!(config.center, [0.1, 0.2, 0.3]);
    let expected = [3.0, 2.0, 4.0, 0.0, 0.0, 0.0];
    for (actual, expected) in config.moment.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-12, "{:?}", config.moment);
    }
}

#[test]
fn overflowing_shape_moments_are_rejected() {
    // Valid mass and extents whose solid-sphere moment 0.4*m*r^2 overflows.
    let heavy = BodySpec {
        mass: Some(1e300),
        ..spec(Shape::Sphere { radius: 1e10 })
    };
    assert_eq!(
        config(heavy, RotationSpec::default(), None).map(|c| c.shape),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn rotations_need_unit_finite_quaternions() {
    assert!(rotation([0.0, 0.0, 0.0, 1.0]).is_ok());
    assert!(rotation([0.0, 0.0, 0.0, 1.0 + 1e-11]).is_err());
    assert!(rotation([f64::NAN, 0.0, 0.0, 1.0]).is_err());
}
