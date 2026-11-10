// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact canonical encoder replay of independently audited M41 component data.
use super::packet;
use gz_sensors::air_speed::AirSpeedReading;
use gz_sim_proto::{
    msgs::{AirSpeed, Header},
    prost::Message,
};
use gz_sim_scene::{AirSpeedPublication, AirSpeedSample};
use std::{collections::BTreeSet, time::Duration};
mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m41_native_air_speed_wind.rs");
}

#[test]
fn actual_native_wind_components_keep_all_91_canonical_packets_and_headers_byte_exact() {
    use native::Publication;
    let expected_modes: BTreeSet<_> = [
        "calm-omitted",
        "calm-signed-zero",
        "headwind",
        "tailwind",
        "cancelling",
        "crosswind",
        "tilted-world-z",
        "tilted-mixed-zero",
        "reverse-flow",
        "changing-world-frame",
        "float-height",
        "float-pressure",
        "gaussian-zero",
        "gaussian-mean",
        "gaussian-cancel",
        "rate100",
        "active100",
        "reload",
    ]
    .into_iter()
    .collect();
    let mut modes = BTreeSet::new();
    let mut instances = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut calls = 0;
    let mut observed = 0;
    let mut suppressed = 0;
    assert_eq!(native::CASES.len(), 19);
    for (label, config, inputs) in native::CASES {
        modes.insert(config.mode);
        assert!(
            instances.insert((config.mode, config.instance)),
            "duplicate component instance"
        );
        assert!(
            topics.insert(config.topic),
            "each native instance has a distinct subscriber lease/topic"
        );
        let expected_label = if config.mode == "reload" {
            format!("reload/{}", config.instance)
        } else {
            config.mode.to_owned()
        };
        assert_eq!(*label, expected_label);
        assert_eq!(
            config.topic,
            format!(
                "/m41/native_air_speed_wind/{}/{}",
                config.mode, config.instance
            )
        );
        assert_eq!(config.frame_id, "rig/body/air_speed");
        assert!(config.instance < if config.mode == "reload" { 2 } else { 1 });
        assert_eq!(
            config.frequency,
            if matches!(config.mode, "rate100" | "active100") {
                100
            } else {
                0
            }
        );
        assert!(
            config
                .requested_wind
                .into_iter()
                .chain(config.effective_wind)
                .chain(config.noise)
                .map(f64::from_bits)
                .all(f64::is_finite)
        );
        let wholezero = config
            .requested_wind
            .iter()
            .all(|b| b & 0x7fffffffffffffff == 0);
        assert_eq!(
            config.effective_wind,
            if wholezero {
                [0; 3]
            } else {
                config.requested_wind
            }
        );
        assert_eq!(config.wind_present, config.mode != "calm-omitted");
        if !config.wind_present {
            assert_eq!(config.requested_wind, [0; 3]);
        }
        let noisy = matches!(
            config.mode,
            "gaussian-zero" | "gaussian-mean" | "gaussian-cancel"
        ) || (config.mode == "reload" && config.instance == 1);
        assert_eq!(config.noise_kind, if noisy { "gaussian" } else { "none" });
        assert_eq!(
            config.noise[0],
            if noisy && config.mode != "gaussian-zero" {
                0.1_f64.to_bits()
            } else {
                0.0_f64.to_bits()
            }
        );
        assert!(config.noise[1..].iter().all(|b| *b == 0));
        let metadata = AirSpeedPublication {
            entity_name: "rig::body::air_speed".into(),
            frame_id: config.frame_id.into(),
            topic: config.topic.into(),
        };
        let expected_times: &[u64] = match config.mode {
            "rate100" => &[1, 2, 9, 10, 11, 20],
            "active100" => &[1, 2, 9, 10, 10, 11, 20],
            _ => &[1, 2, 3, 4, 5],
        };
        assert_eq!(inputs.len(), expected_times.len());
        let mut instance_observed = 0;
        for (index, input) in inputs.iter().enumerate() {
            calls += 1;
            assert_eq!(input.index, index as u32);
            assert_eq!(input.time_ns, expected_times[index] * 1_000_000);
            assert!(!input.force && input.has_connections);
            assert_eq!(
                input.active,
                config.mode != "active100" || [false, true, true, false, true, true, true][index]
            );
            assert!(
                input
                    .world_pose
                    .into_iter()
                    .chain(input.velocity_sensor)
                    .chain(input.native_math_wind_sensor)
                    .chain(input.native_math_relative_air)
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            // These are DIRECT WORLD/sensor-frame component inputs and native
            // Math handoff observations, not a physical Scene or Sensor getter.
            // Wholezero Rust bypass retains the old velocity bits; this encoder
            // control does not impose native rotated-zero intermediate bits.
            match input.publication {
                Publication::Suppressed => {
                    suppressed += 1;
                    assert_eq!(config.frequency, 100);
                    assert!(!input.active || input.time_ns < input.next_before_ns);
                    assert_eq!(input.next_before_ns, input.next_after_ns);
                    // No reading or encoded sample is invented for this call.
                }
                Publication::Observed(output) => {
                    observed += 1;
                    assert!(input.active);
                    if config.frequency == 0 {
                        assert_eq!((input.next_before_ns, input.next_after_ns), (0, 0));
                    } else {
                        assert!(
                            input.time_ns >= input.next_before_ns
                                && input.next_after_ns > input.time_ns
                        );
                    }
                    let sequence: u64 = output.sequence.parse().unwrap();
                    assert_eq!(output.sequence, sequence.to_string());
                    assert_eq!(sequence, instance_observed);
                    instance_observed += 1;
                    let reading = AirSpeedReading {
                        diff_pressure: f64::from_bits(output.pressure),
                        temperature: f64::from_bits(output.temperature),
                    };
                    assert!(
                        reading.diff_pressure.is_finite()
                            && reading.temperature.is_finite()
                            && reading.temperature > 0.0
                    );
                    let sample = AirSpeedSample {
                        time: Duration::from_nanos(input.time_ns),
                        epoch: 42 + u64::from(config.instance),
                        reading: Ok(reading),
                    };
                    let message = packet(&metadata, &sample, sequence)
                        .expect("actual finite native reading is encodable");
                    assert_eq!(
                        message.diff_pressure.to_bits(),
                        output.pressure,
                        "{label} call{index}"
                    );
                    assert_eq!(
                        message.temperature.to_bits(),
                        output.temperature,
                        "{label} call{index}"
                    );
                    assert_eq!(
                        message.encode_to_vec().as_slice(),
                        output.wire,
                        "{label} call{index}"
                    );
                    let header = message.header.as_ref().unwrap();
                    assert_eq!(
                        header.encode_to_vec().as_slice(),
                        output.header_wire,
                        "{label} header{index}"
                    );
                    assert_eq!(Header::decode(output.header_wire).unwrap(), *header);
                    assert_eq!(AirSpeed::decode(output.wire).unwrap(), message);
                }
            }
        }
        assert_eq!(
            instance_observed,
            if config.frequency == 100 { 3 } else { 5 }
        );
    }
    assert_eq!(modes, expected_modes);
    assert_eq!(
        (instances.len(), calls, observed, suppressed),
        (19, 98, 91, 7)
    );
    assert!(instances.contains(&("reload", 0)) && instances.contains(&("reload", 1)));
    // Native component scheduling/configuration and deterministic stddev0 cases
    // do not establish Scene pause/demand semantics or native RNG stream parity.
}
