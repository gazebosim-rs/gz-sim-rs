// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured-value encoding only; native scheduling and Rust acquisition are separate proofs.
//! WORLD pose/constant-field inputs and warning provenance are a ledger, never an output estimator.
//! Original family-wide demand/removal is distinct from Rust owner demand.
use super::packet;
use gz_sensors::magnetometer::MagnetometerReading;
use gz_sim_proto::{
    msgs::{Header, Magnetometer},
    prost::Message,
};
use gz_sim_scene::{MagnetometerPublication, MagnetometerSample};
use std::{collections::BTreeSet, time::Duration};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m52_native_multiple_magnetometers.rs");
}

#[test]
fn actual_native_multiple_magnetometers_keep_all_42_packets_and_headers_byte_exact() {
    let expected = [
        ("pair", 2_usize, 90_usize, 8_usize, 46_u32),
        ("eight", 8, 360, 28, 46),
        ("remove", 2, 51, 6, 27),
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
        let mut native_entities = BTreeSet::new();
        for (index, config) in configs.iter().enumerate() {
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            assert!(config.native_entity > 0 && native_entities.insert(config.native_entity));
            assert_eq!(config.console_verbosity, 2);
            assert_eq!(
                config.topic,
                format!("/m52/native/magnetometer/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m52/native/{index}"));
            assert!(topics.insert(config.topic));
            let period_ms = [10_u64, 20, 40][index % 3];
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
            assert_eq!(config.world_field, [2e-5, -3e-5, 4e-5].map(f64::to_bits));
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert_eq!(
                xml.matches("<sensor name='magnetometer' type='magnetometer'>")
                    .count(),
                1
            );
            assert_eq!(xml.matches("<magnetometer/>").count(), 1);
            assert!(!xml.contains("<noise"));
            assert!(xml.contains(&format!("<topic>{}</topic>", config.topic)));
            assert!(xml.contains(&format!("<frame_id>{}</frame_id>", config.frame_id)));
            assert!(xml.contains(&format!("<update_rate>{}</update_rate>", config.frequency)));
            metadata.push(MagnetometerPublication {
                entity_name: format!("rig{index}::body::magnetometer"),
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
            let next_input = if mode == "remove" && index == 1 && last_time[index] == 25_000_000 {
                40_000_000
            } else {
                last_time[index] + 1_000_000
            };
            assert_eq!(input.time_ns, next_input);
            assert!(input.time_ns.is_multiple_of(1_000_000));
            assert!(input.time_ns <= 45_000_000);
            last_time[index] = input.time_ns;
            per_sensor_inputs[index] += 1;
            assert!(
                input
                    .world_pose
                    .into_iter()
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            assert!(
                input
                    .world_field
                    .into_iter()
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            assert_eq!(input.world_field, configs[index].world_field);
            // These are actual supplied WORLD poses and the constant WORLD
            // field. Do not rotate them or estimate an expected output here.
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
            let sample = MagnetometerSample {
                time: Duration::from_nanos(input.time_ns),
                epoch: 0,
                reading: Ok(MagnetometerReading {
                    field_tesla: output.field_tesla.map(f64::from_bits),
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
            let decoded = Magnetometer::decode(output.wire).unwrap();
            assert_eq!(decoded, message);
            let field = decoded.field_tesla.as_ref().expect("captured Tesla vector");
            assert_eq!(
                [field.x, field.y, field.z].map(f64::to_bits),
                output.field_tesla
            );
            assert!([field.x, field.y, field.z].into_iter().all(f64::is_finite));
            case_packets += 1;
            packets += 1;
        }
        assert_eq!(case_packets, observed);
        assert_eq!(sequences.iter().sum::<u64>() as usize, observed);
        if let Some(removal) = removal {
            removals += 1;
            assert_eq!(mode, "remove");
            assert_eq!(removal.sensor, 0);
            assert_eq!(removal.after_time_ns, 25_000_000);
            assert_eq!(removal.next_system_time_ns, 40_000_000);
            assert!(removal.peer_connected_required);
            assert_eq!(last_time, [25_000_000, 40_000_000]);
            assert_eq!(per_sensor_inputs, [25, 26]);
            assert_eq!(summary.removal_after_time_ns, Some(removal.after_time_ns));
            assert_eq!(
                summary.retirement_observed_system_time_ns,
                Some(removal.next_system_time_ns)
            );
        } else {
            assert_ne!(mode, "remove");
            assert!(last_time.iter().all(|time| *time == 45_000_000));
            assert!(per_sensor_inputs.iter().all(|rows| *rows == 45));
            assert_eq!(summary.removal_after_time_ns, None);
            assert_eq!(summary.retirement_observed_system_time_ns, None);
        }
    }
    assert_eq!((modes.len(), topics.len(), configurations), (3, 12, 12));
    assert_eq!(
        (input_rows, packets, without_packet, removals),
        (501, 42, 459, 1)
    );
    assert_eq!(system_calls, 119);
}
