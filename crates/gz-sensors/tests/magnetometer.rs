// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Faithful ideal conventions from gz-sensors8ddf test/integration/magnetometer.cc.
use gz_sensors::magnetometer::{InvalidMagnetometerInput, MagnetometerInput, magnetometer};

#[test]
fn field_is_inverse_sensor_rotation_in_tesla_without_reference_or_translation() {
    let field = [2e-5, -3e-5, 4e-5];
    let identity = [0.0, 0.0, 0.0, 1.0];
    assert_eq!(
        magnetometer(MagnetometerInput {
            world_orientation: identity,
            field_world: field
        })
        .unwrap()
        .field_tesla,
        field
    );
    // Positive sensor yaw pi/2: world X becomes sensor negative Y.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let q = [0.0, 0.0, h, h];
    let reading = magnetometer(MagnetometerInput {
        world_orientation: q,
        field_world: field,
    })
    .unwrap();
    for (actual, expected) in reading.field_tesla.into_iter().zip([-3e-5, -2e-5, 4e-5]) {
        assert!((actual - expected).abs() < 1e-18);
    }
    let opposite = q.map(|v| -v);
    assert_eq!(
        reading,
        magnetometer(MagnetometerInput {
            world_orientation: opposite,
            field_world: field
        })
        .unwrap()
    );
    assert_eq!(
        magnetometer(MagnetometerInput {
            world_orientation: q,
            field_world: [0.0; 3]
        })
        .unwrap()
        .field_tesla,
        [0.0; 3]
    );
}

#[test]
fn asymmetric_all_axis_rotation_matches_independent_scalar_transpose() {
    let q = [0.2, -0.3, 0.4, 0.8426149773176358];
    let [x, y, z, w] = q;
    let [a, b, c] = [5.5645e-6, 22.8758e-6, -42.3884e-6];
    let expected = [
        (1.0 - 2.0 * (y * y + z * z)) * a + 2.0 * (x * y + z * w) * b + 2.0 * (x * z - y * w) * c,
        2.0 * (x * y - z * w) * a + (1.0 - 2.0 * (x * x + z * z)) * b + 2.0 * (y * z + x * w) * c,
        2.0 * (x * z + y * w) * a + 2.0 * (y * z - x * w) * b + (1.0 - 2.0 * (x * x + y * y)) * c,
    ];
    let reading = magnetometer(MagnetometerInput {
        world_orientation: q,
        field_world: [a, b, c],
    })
    .unwrap();
    for (actual, expected) in reading.field_tesla.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-18);
    }
}

#[test]
fn nonfinite_nonunit_and_finite_rotation_overflow_are_typed_errors() {
    let field = [2e-5, -3e-5, 4e-5];
    for q in [
        [0.0; 4],
        [0.0, 0.0, 0.0, 2.0],
        [f64::NAN, 0.0, 0.0, 1.0],
        [0.0, f64::INFINITY, 0.0, 1.0],
        [f64::MAX; 4],
    ] {
        assert_eq!(
            magnetometer(MagnetometerInput {
                world_orientation: q,
                field_world: field
            }),
            Err(InvalidMagnetometerInput)
        );
    }
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for axis in 0..3 {
            let mut field = field;
            field[axis] = bad;
            assert_eq!(
                magnetometer(MagnetometerInput {
                    world_orientation: [0.0, 0.0, 0.0, 1.0],
                    field_world: field
                }),
                Err(InvalidMagnetometerInput)
            );
        }
    }
    let angle = std::f64::consts::FRAC_PI_8;
    assert_eq!(
        magnetometer(MagnetometerInput {
            world_orientation: [0.0, 0.0, angle.sin(), angle.cos()],
            field_world: [f64::MAX, f64::MAX, 0.0]
        }),
        Err(InvalidMagnetometerInput)
    );
}
