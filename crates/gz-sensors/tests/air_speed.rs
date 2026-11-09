// Copyright (C) 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_speed::{AirSpeedInput, InvalidAirSpeedInput, air_speed};

fn at(z: f64, v: [f64; 3]) -> gz_sensors::air_speed::AirSpeedReading {
    air_speed(AirSpeedInput {
        world_z: z,
        velocity_sensor: v,
    })
    .unwrap()
}

#[test]
fn native_signed_pressure_scale_and_float_temperature_are_preserved() {
    let zero = at(0.0, [0.0; 3]);
    assert_eq!(zero.diff_pressure.to_bits(), 0.0_f64.to_bits());
    assert_eq!(zero.temperature, f64::from(288.15_f32));
    let forward = at(0.0, [10.0, 0.0, 0.0]);
    assert!((forward.diff_pressure - 61.25).abs() < 1e-5);
    assert!(
        forward.diff_pressure > 60.0,
        "wire number is Pa scale, not .6125 hPa"
    );
    assert_eq!(
        at(0.0, [-10.0, 0.0, 0.0]).diff_pressure,
        -forward.diff_pressure
    );
    assert_eq!(at(0.0, [0.0, 10.0, -10.0]), zero);
    assert_eq!(at(0.0, [-0.0, 0.0, 0.0]), zero);
}

#[test]
fn native_mixed_float_double_chain_absolute_altitude_and_small_inputs_match() {
    for height in [-100.0, 0.0, 3.5, 1000.0, 11000.0] {
        for velocity in [-31.23456789, -1e-100, 0.0, 3.125, 19.9999999] {
            // Independent spelling of the pinned C++ scalar expression.
            let z = height as f32;
            let t = 288.15_f32 - 0.0065_f32 * z;
            let rho = 1.225_f32 / (288.15_f32 / t).powf(4.256_f32);
            let sign = if velocity > 0.0 {
                1_f32
            } else if velocity < 0.0 {
                -1_f32
            } else {
                0_f32
            };
            let intermediate = ((sign * 0.005_f32 * rho) as f64 * velocity * velocity) as f32;
            let measured = at(height, [velocity, 100.0, -50.0]);
            assert_eq!(measured.temperature, f64::from(t));
            assert_eq!(measured.diff_pressure, f64::from(intermediate * 100_f32));
        }
    }
    assert!(at(1000.0, [10.0, 0.0, 0.0]).diff_pressure < at(0.0, [10.0, 0.0, 0.0]).diff_pressure);
    assert_ne!(
        at(3.5, [10.0, 0.0, 0.0]).temperature,
        at(0.0, [10.0, 0.0, 0.0]).temperature
    );
}

#[test]
fn every_nonfinite_channel_float_cast_and_atmosphere_or_pressure_overflow_is_refused() {
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            air_speed(AirSpeedInput {
                world_z: value,
                velocity_sensor: [0.0; 3]
            }),
            Err(InvalidAirSpeedInput)
        );
        for axis in 0..3 {
            let mut v = [0.0; 3];
            v[axis] = value;
            assert_eq!(
                air_speed(AirSpeedInput {
                    world_z: 0.0,
                    velocity_sensor: v
                }),
                Err(InvalidAirSpeedInput)
            );
        }
    }
    for z in [f64::MAX, -f64::MAX, 50000.0, -f64::from(f32::MAX)] {
        assert!(
            air_speed(AirSpeedInput {
                world_z: z,
                velocity_sensor: [0.0; 3]
            })
            .is_err()
        );
    }
    for x in [f64::MAX, -f64::MAX, 1e30, -1e30] {
        assert!(
            air_speed(AirSpeedInput {
                world_z: 0.0,
                velocity_sensor: [x, 0.0, 0.0]
            })
            .is_err()
        );
    }
}

#[test]
fn precursor_seam_preserves_ideal_bits_without_premature_final_scale_check() {
    use gz_sensors::air_speed::{air_speed_unscaled, scale_air_speed};
    for z in [-100.0, 0.0, 3.5, 1000.0] {
        for x in [-31.23456789, 0.0, 3.125, 19.9999999] {
            let input = AirSpeedInput {
                world_z: z,
                velocity_sensor: [x, 0.0, 0.0],
            };
            let (p, t) = air_speed_unscaled(input).unwrap();
            assert_eq!(scale_air_speed(p, t).unwrap(), air_speed(input).unwrap());
        }
    }
    let input = AirSpeedInput {
        world_z: 0.0,
        velocity_sensor: [1e20, 0.0, 0.0],
    };
    let (p, t) = air_speed_unscaled(input).unwrap();
    assert!(p.is_finite() && (p * 100.0).is_infinite());
    assert_eq!(air_speed(input), Err(InvalidAirSpeedInput));
    // Noise may cancel the finite precursor before the final native scale.
    assert_eq!(scale_air_speed(0.0, t).unwrap().diff_pressure, 0.0);
    for (p, t) in [
        (f32::INFINITY, 288.0),
        (0.0, f32::NAN),
        (0.0, 0.0),
        (0.0, -1.0),
    ] {
        assert_eq!(scale_air_speed(p, t), Err(InvalidAirSpeedInput));
    }
}
