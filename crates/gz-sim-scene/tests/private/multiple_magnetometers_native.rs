// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DQuat;
use gz_sensors::magnetometer::{MagnetometerInput, magnetometer};
use std::collections::{BTreeMap, BTreeSet};

// Exact independently audited borrowed literals; no generated arrays on the stack.
// Every field is checked here. Canonical byte encoding remains a separate Server proof.
mod native {
    include!("../fixtures/m52_native_multiple_magnetometers.rs");
}

#[test]
fn actual_native_magnetometer_multiplicity_matches_all_42_scalar_vectors_and_owner_inputs() {
    let mut modes = BTreeSet::new();
    let (mut configurations, mut inputs, mut observed, mut no_packet, mut removals) =
        (0, 0, 0, 0, 0);
    let mut system_calls = 0;
    for &(mode, configs, rows, removal, summary) in native::CASES {
        assert!(modes.insert(mode));
        assert!(matches!(mode, "pair" | "eight" | "remove"));
        let expected_count = if mode == "eight" { 8 } else { 2 };
        assert_eq!(configs.len(), expected_count);
        assert_eq!(summary.configuration_count as usize, configs.len());
        assert_eq!(summary.input_rows as usize, rows.len());
        assert_eq!(summary.system_calls, if mode == "remove" { 27 } else { 46 });
        assert!(summary.native_family_demand_policy);
        assert!(!summary.native_physics_executed && !summary.old_native_modes_replayed);
        // Calls include paused creation without a fabricated input card. This
        // count is original helper summary/Source evidence, not internal Update calls.
        system_calls += summary.system_calls;
        let mut owners = BTreeMap::new();
        let mut sequences = BTreeMap::new();
        let mut owner_packets = BTreeMap::new();
        let mut topics = BTreeSet::new();
        let mut native_ids = BTreeSet::new();
        for (index, config) in configs.iter().enumerate() {
            configurations += 1;
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            assert!(owners.insert(config.sensor, config).is_none());
            assert!(config.native_entity > 0 && native_ids.insert(config.native_entity));
            assert!(topics.insert(config.topic));
            assert_eq!(
                config.topic,
                format!("/m52/native/magnetometer/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m52/native/{index}"));
            assert_eq!(config.console_verbosity, 2);
            let period_ms = [10, 20, 40, 10, 20, 40, 10, 20][index];
            assert_eq!(config.period_ns, period_ms * 1_000_000);
            assert_eq!(u64::from(config.frequency), 1000 / period_ms);
            let initial = config.initial_world_pose.map(f64::from_bits);
            let local = config.sensor_local_pose.map(f64::from_bits);
            let field = config.world_field.map(f64::from_bits);
            assert!(
                initial
                    .into_iter()
                    .chain(local)
                    .chain(field)
                    .all(f64::is_finite)
            );
            assert_eq!(
                &initial[..3],
                &[2.25 + index as f64, -1.5, 3.5 + 1.25 * index as f64]
            );
            assert_eq!(&local[..3], &[0.25, -0.5, 0.5 + 0.25 * index as f64]);
            assert_eq!(
                config.world_field,
                [2e-5_f64, -3e-5_f64, 4e-5_f64].map(f64::to_bits)
            );
            for pose in [initial, local] {
                let q = DQuat::from_xyzw(pose[3], pose[4], pose[5], pose[6]);
                assert!((q.length_squared() - 1.0).abs() <= 1e-12);
            }
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains("<sensor name='magnetometer' type='magnetometer'>"));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(
                xml.contains(format!("<update_rate>{}</update_rate>", config.frequency).as_str())
            );
            assert!(xml.contains("<magnetometer/>") && !xml.contains("<noise"));
            sequences.insert(config.sensor, 0_u64);
            owner_packets.insert(config.sensor, 0_u64);
        }
        let mut expected_inputs = Vec::new();
        for tick in 1..=45 {
            if mode == "remove" && tick > 25 && tick != 40 {
                continue;
            }
            for index in 0..expected_count {
                if mode == "remove" && tick == 40 && index == 0 {
                    continue;
                }
                expected_inputs.push((index as u32, tick * 1_000_000));
            }
        }
        assert_eq!(rows.len(), expected_inputs.len());
        let mut input_keys = BTreeSet::new();
        let mut previous_time = BTreeMap::new();
        let mut mode_packets = 0;
        let mut surviving_peer_at_40 = false;
        for (row, expected_input) in rows.iter().zip(expected_inputs) {
            inputs += 1;
            assert_eq!((row.sensor, row.time_ns), expected_input);
            let config = owners
                .get(&row.sensor)
                .expect("input belongs to one configured owner");
            assert!(input_keys.insert((row.sensor, row.time_ns)));
            if let Some(previous) = previous_time.insert(row.sensor, row.time_ns) {
                assert!(previous < row.time_ns);
            }
            let world = row.world_pose.map(f64::from_bits);
            let field = row.world_field.map(f64::from_bits);
            assert!(world.into_iter().chain(field).all(f64::is_finite));
            assert_eq!(row.world_field, config.world_field);
            let q = [world[3], world[4], world[5], world[6]];
            assert!((DQuat::from_array(q).length_squared() - 1.0).abs() <= 1e-12);
            let packet_due =
                row.time_ns == 1_000_000 || row.time_ns.is_multiple_of(config.period_ns);
            assert_eq!(
                row.observed.is_some(),
                packet_due,
                "captured packet cadence only"
            );
            let Some(output) = row.observed else {
                // None is no matching packet, never an observed internal Update
                // return, skipped RNG draw, deadline or individual-demand proof.
                no_packet += 1;
                continue;
            };
            // Direct supplied sensor WORLD quaternion and constant WORLD field;
            // no link kinematics, native Physics, geographic field or output estimator.
            let reading = magnetometer(MagnetometerInput {
                world_orientation: q,
                field_world: field,
            })
            .unwrap();
            for (actual, captured) in reading.field_tesla.into_iter().zip(output.field_tesla) {
                let native_value = f64::from_bits(captured);
                assert!(actual.is_finite() && native_value.is_finite());
                // Exact retained M17 component bound, distinct from live1e-18
                // geometry and the separate exact canonical wire obligation.
                assert!(
                    (actual - native_value).abs() < 1e-15,
                    "{mode}/{}",
                    row.sensor
                );
            }
            let next = sequences.get_mut(&row.sensor).unwrap();
            assert_eq!(output.sequence, next.to_string());
            *next += 1;
            *owner_packets.get_mut(&row.sensor).unwrap() += 1;
            assert!(!output.header_wire.is_empty() && output.header_wire.len() <= 8192);
            assert!(!output.wire.is_empty() && output.wire.len() <= 8192);
            observed += 1;
            mode_packets += 1;
            surviving_peer_at_40 |=
                mode == "remove" && row.sensor == 1 && row.time_ns == 40_000_000;
        }
        assert_eq!(summary.canonical_packets, mode_packets);
        assert_eq!(
            mode_packets,
            if mode == "pair" {
                8
            } else if mode == "eight" {
                28
            } else {
                6
            }
        );
        assert!(owner_packets.values().all(|count| *count > 0));
        if let Some(event) = removal {
            removals += 1;
            assert_eq!(mode, "remove");
            assert_eq!(event.sensor, 0);
            assert_eq!(event.after_time_ns, 25_000_000);
            assert_eq!(event.next_system_time_ns, 40_000_000);
            assert!(event.peer_connected_required && surviving_peer_at_40);
            assert_eq!(previous_time[&0], event.after_time_ns);
            assert_eq!(previous_time[&1], event.next_system_time_ns);
            assert_eq!(owner_packets[&0], 3);
            assert_eq!(owner_packets[&1], 3);
            assert_eq!(summary.removal_after_time_ns, Some(event.after_time_ns));
            assert_eq!(
                summary.retirement_observed_system_time_ns,
                Some(event.next_system_time_ns)
            );
            // Original removal is witnessed after a connected due peer call;
            // it is not all-disconnected or Rust individual-demand equivalence.
        } else {
            assert_ne!(mode, "remove");
            assert!(summary.removal_after_time_ns.is_none());
            assert!(summary.retirement_observed_system_time_ns.is_none());
        }
    }
    assert_eq!(modes, BTreeSet::from(["pair", "eight", "remove"]));
    assert_eq!(
        (
            configurations,
            system_calls,
            inputs,
            observed,
            no_packet,
            removals
        ),
        (12, 119, 501, 42, 459, 1)
    );
}
