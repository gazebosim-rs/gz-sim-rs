// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured-value encoding only; native scheduling and Rust acquisition are separate proofs.
//! WORLD pose/point-velocity/wind and native handoff inputs are a ledger, never an output estimator.
//! Original family-wide demand/removal is distinct from Rust owner demand.
use super::packet;
use gz_sensors::air_speed::AirSpeedReading;
use gz_sim_proto::{
    msgs::{AirSpeed, Header},
    prost::Message,
};
use gz_sim_scene::{AirSpeedPublication, AirSpeedSample};
use std::{collections::BTreeSet, time::Duration};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m51_native_multiple_air_speeds.rs");
}

#[test]
fn actual_native_multiple_air_speeds_keep_all_55_packets_and_headers_byte_exact() {
    let expected = [
        ("pair", 2_usize, 60_usize, 11_usize, 31_u32),
        ("eight", 8, 240, 37, 31),
        ("remove", 2, 31, 7, 17),
    ];
    let mut modes = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut configurations = 0;
    let mut input_rows = 0;
    let mut packets = 0;
    let mut without_packet = 0;
    let mut removals = 0;
    let mut system_calls = 0;
    assert_eq!(native::CASES.len(), expected.len());
    for ((label, configs, inputs, removal, summary), expected) in native::CASES.iter().zip(expected)
    {
        let (mode, count, rows, observed, calls) = expected;
        assert_eq!(*label, mode);
        assert!(modes.insert(*label));
        assert_eq!(configs.len(), count);
        assert_eq!(inputs.len(), rows);
        assert_eq!(summary.configuration_count as usize, count);
        assert_eq!(summary.input_rows as usize, rows);
        assert_eq!(summary.canonical_packets as usize, observed);
        // Calls are native summary metadata, not individually captured paused calls.
        assert_eq!(summary.system_calls, calls);
        assert!(summary.native_family_demand_policy);
        assert!(!summary.native_physics_executed && !summary.old_native_modes_replayed);
        configurations += count;
        input_rows += rows;
        system_calls += calls;
        let mut metadata = Vec::new();
        for (index, config) in configs.iter().enumerate() {
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            assert_eq!(
                config.topic,
                format!("/m51/native/air_speed/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m51/native/{index}"));
            assert!(topics.insert(config.topic));
            let period_ms = [5_u64, 10, 20][index % 3];
            assert_eq!(config.frequency, (1000 / period_ms) as u32);
            assert_eq!(config.period_ns, period_ms * 1_000_000);
            for values in [
                config.initial_world_pose.as_slice(),
                config.sensor_local_pose.as_slice(),
            ] {
                assert!(
                    values
                        .iter()
                        .copied()
                        .map(f64::from_bits)
                        .all(f64::is_finite)
                );
            }
            assert_eq!(
                config.sensor_local_pose[..3],
                [0.25, -0.5, 0.5 + 0.25 * index as f64].map(f64::to_bits)
            );
            assert_eq!(
                config.initial_world_pose[..3],
                [2.25 + index as f64, -1.5, 3.5 + 1.25 * index as f64].map(f64::to_bits)
            );
            assert_eq!(config.world_wind, [1.0, -2.0, 0.5].map(f64::to_bits));
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert_eq!(
                xml.matches("<sensor name='air_speed' type='air_speed'>")
                    .count(),
                1
            );
            assert_eq!(xml.matches("<air_speed/>").count(), 1);
            assert!(!xml.contains("<noise"));
            assert!(xml.contains(&format!("<topic>{}</topic>", config.topic)));
            assert!(xml.contains(&format!("<frame_id>{}</frame_id>", config.frame_id)));
            assert!(xml.contains(&format!("<update_rate>{}</update_rate>", config.frequency)));
            metadata.push(AirSpeedPublication {
                entity_name: format!("rig{index}::body::air_speed"),
                frame_id: config.frame_id.into(),
                topic: config.topic.into(),
            });
        }
        let mut sequences = vec![0_u64; count];
        let mut last_time = vec![0_u64; count];
        let mut per_sensor_inputs = vec![0_usize; count];
        let mut case_packets = 0;
        for input in *inputs {
            let index = input.sensor as usize;
            assert!(index < count);
            let next_input = if mode == "remove" && index == 1 && last_time[index] == 15_000_000 {
                20_000_000
            } else {
                last_time[index] + 1_000_000
            };
            assert_eq!(input.time_ns, next_input);
            assert!(input.time_ns.is_multiple_of(1_000_000));
            assert!(input.time_ns <= 30_000_000);
            last_time[index] = input.time_ns;
            per_sensor_inputs[index] += 1;
            assert!(
                input
                    .world_pose
                    .into_iter()
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            for values in [
                input.world_point_velocity,
                input.world_wind,
                input.native_relative_velocity_handoff,
            ] {
                assert!(values.into_iter().map(f64::from_bits).all(f64::is_finite));
            }
            assert_eq!(input.world_wind, configs[index].world_wind);
            // These are actual supplied point velocities/native handoffs. Do
            // not derive a velocity from pose or estimate pressure from them.
            // None records only the absence of a matching captured packet. It
            // does not expose internal Update/draw state or Rust demand behavior.
            let Some(output) = input.observed else {
                without_packet += 1;
                continue;
            };
            assert!(
                input.time_ns == 1_000_000
                    || input.time_ns.is_multiple_of(configs[index].period_ns)
            );
            let sequence: u64 = output.sequence.parse().unwrap();
            assert_eq!(output.sequence, sequence.to_string());
            assert_eq!(sequence, sequences[index]);
            sequences[index] += 1;
            let sample = AirSpeedSample {
                time: Duration::from_nanos(input.time_ns),
                epoch: 0,
                reading: Ok(AirSpeedReading {
                    diff_pressure: f64::from_bits(output.diff_pressure),
                    temperature: f64::from_bits(output.temperature),
                }),
            };
            let message = packet(&metadata[index], &sample, sequence)
                .expect("actual finite native reading is encodable");
            let header = message.header.as_ref().expect("canonical header");
            assert_eq!(
                header.encode_to_vec(),
                output.header_wire,
                "{mode}/{index} header"
            );
            assert_eq!(
                message.encode_to_vec(),
                output.wire,
                "{mode}/{index} full wire"
            );
            assert_eq!(Header::decode(output.header_wire).unwrap(), *header);
            let decoded = AirSpeed::decode(output.wire).unwrap();
            assert_eq!(decoded, message);
            assert_eq!(decoded.diff_pressure.to_bits(), output.diff_pressure);
            assert_eq!(decoded.temperature.to_bits(), output.temperature);
            assert!(decoded.diff_pressure.is_finite());
            assert!(decoded.temperature.is_finite() && decoded.temperature > 0.0);
            case_packets += 1;
            packets += 1;
        }
        assert_eq!(case_packets, observed);
        assert_eq!(sequences.iter().sum::<u64>() as usize, observed);
        if let Some(removal) = removal {
            removals += 1;
            assert_eq!(mode, "remove");
            assert_eq!(removal.sensor, 0);
            assert_eq!(removal.after_time_ns, 15_000_000);
            assert_eq!(removal.next_system_time_ns, 20_000_000);
            assert!(removal.peer_connected_required);
            assert_eq!(last_time, [15_000_000, 20_000_000]);
            assert_eq!(per_sensor_inputs, [15, 16]);
            assert_eq!(summary.removal_after_time_ns, Some(removal.after_time_ns));
            assert_eq!(
                summary.retirement_observed_system_time_ns,
                Some(removal.next_system_time_ns)
            );
        } else {
            assert_ne!(mode, "remove");
            assert!(last_time.iter().all(|time| *time == 30_000_000));
            assert!(per_sensor_inputs.iter().all(|rows| *rows == 30));
            assert_eq!(summary.removal_after_time_ns, None);
            assert_eq!(summary.retirement_observed_system_time_ns, None);
        }
    }
    assert_eq!((modes.len(), topics.len(), configurations), (3, 12, 12));
    assert_eq!(
        (input_rows, packets, without_packet, removals),
        (331, 55, 276, 1)
    );
    assert_eq!(system_calls, 79);
}
