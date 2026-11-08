// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::altimeter::{AltimeterInput, InvalidAltimeterInput, altimeter};

#[test]
fn native_ideal_reference_position_and_velocity_conventions_are_independent() {
    // Native gz-sensors/test/integration/altimeter.cc SensorReadings controls.
    for (position, velocity, reference, expected) in [
        (2.0, 3.0, 1.0, 1.0),
        (-6.0, 3.0, 1.0, -7.0),
        (3.05, 0.0, 3.05, 0.0),
        (0.05, -2.0, 3.05, -3.0),
        (-4.0, -0.0, -5.0, 1.0),
    ] {
        let reading = altimeter(AltimeterInput {
            world_z: position,
            world_velocity_z: velocity,
            vertical_reference: reference,
        })
        .expect("finite native model control");
        assert!((reading.vertical_position - expected).abs() < 1e-15);
        assert_eq!(reading.vertical_velocity.to_bits(), velocity.to_bits());
        assert_eq!(reading.vertical_reference.to_bits(), reference.to_bits());
    }
}

#[test]
fn every_nonfinite_input_and_finite_subtraction_overflow_is_refused() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for field in 0..3 {
            let mut values = [2.0, 3.0, 1.0];
            values[field] = invalid;
            assert_eq!(
                altimeter(AltimeterInput {
                    world_z: values[0],
                    world_velocity_z: values[1],
                    vertical_reference: values[2],
                }),
                Err(InvalidAltimeterInput)
            );
        }
    }
    for (world_z, vertical_reference) in [(f64::MAX, -f64::MAX), (-f64::MAX, f64::MAX)] {
        assert_eq!(
            altimeter(AltimeterInput {
                world_z,
                world_velocity_z: 0.0,
                vertical_reference,
            }),
            Err(InvalidAltimeterInput)
        );
    }
    assert!(
        altimeter(AltimeterInput {
            world_z: f64::MAX,
            world_velocity_z: f64::MAX,
            vertical_reference: f64::MAX,
        })
        .is_ok()
    );
}
