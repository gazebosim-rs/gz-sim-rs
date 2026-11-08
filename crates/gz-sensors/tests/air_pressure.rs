// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_pressure::{AirPressureInput, InvalidAirPressureInput, air_pressure};

#[test]
fn native_sensor_reading_is_absolute_pressure_with_unknown_variance() {
    // gz-sensors/test/integration/air_pressure.cc SensorReadings uses a 1m
    // reference plus 2m pose: equivalent absolute height 3m in this ref-zero API.
    let reading = air_pressure(AirPressureInput { world_z: 3.0 }).expect("native ideal control");
    assert!((reading.pressure - 101288.9657925308).abs() < 1e-9);
    assert_eq!(reading.variance, 0.0);
    for zero in [0.0, -0.0] {
        let reading = air_pressure(AirPressureInput { world_z: zero }).expect("sea level");
        assert_eq!(reading.pressure, 101325.0);
        assert_eq!(reading.variance, 0.0);
    }
    let mut previous = f64::INFINITY;
    for height in [-1000.0, -3.0, 0.0, 3.0, 100.0, 1000.0, 11000.0] {
        let reading = air_pressure(AirPressureInput { world_z: height }).expect("physical height");
        assert!(reading.pressure > 0.0 && reading.pressure < previous);
        assert_eq!(reading.variance, 0.0);
        previous = reading.pressure;
    }
}

#[test]
fn nonfinite_singular_subearth_nonpositive_temperature_and_overflow_are_refused() {
    for world_z in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        -f64::MAX,
        -6356766.0,
        -6356767.0,
        1e6,
    ] {
        assert_eq!(
            air_pressure(AirPressureInput { world_z }),
            Err(InvalidAirPressureInput)
        );
    }
    // Near the positive-temperature boundary an underflowed pressure is not a
    // fabricated zero reading; the exact singular temperature is also refused.
    let geopotential_limit = 288.15 / 0.0065;
    let limit = 6356766.0 * geopotential_limit / (6356766.0 - geopotential_limit);
    assert_eq!(
        air_pressure(AirPressureInput {
            world_z: limit + 1.0
        }),
        Err(InvalidAirPressureInput)
    );
    assert!(
        air_pressure(AirPressureInput {
            world_z: limit - 1.0
        })
        .is_ok()
    );
}
