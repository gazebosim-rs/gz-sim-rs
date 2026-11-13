// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::altimeter::{AltimeterInput, altimeter};
use std::collections::{BTreeMap, BTreeSet};

// The shared audited cards also retain canonical bytes for the separate Server
// encoder control. Limit the unused-field disposition to this fixture module.
#[allow(dead_code)]
mod native {
    // include! keeps the exact independently audited literal bytes under fmt.
    include!("../fixtures/m49_native_multiple_altimeters.rs");
}

fn close(actual: f64, expected: u64, mode: &str, sensor: u32, channel: &str) {
    let expected = f64::from_bits(expected);
    assert!(actual.is_finite() && expected.is_finite());
    assert!(
        (actual - expected).abs() < 1e-9,
        "{mode}/{sensor}/{channel}: {actual} vs {expected}"
    );
}

#[test]
fn actual_native_multiplicity_matches_all_42_scalar_readings_and_owner_references() {
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
        assert_eq!(summary.system_calls, if mode == "remove" { 22 } else { 41 });
        assert!(summary.native_family_demand_policy);
        assert!(!summary.native_physics_executed && !summary.old_native_modes_replayed);
        // System-call totals include the actual paused creation call, which has
        // no fabricated NativeInput card. They remain aggregate metadata.
        system_calls += summary.system_calls;
        let mut owners = BTreeMap::new();
        let mut sequences = BTreeMap::new();
        let mut owner_packets = BTreeMap::new();
        let mut topics = BTreeSet::new();
        for (index, config) in configs.iter().enumerate() {
            configurations += 1;
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            assert!(owners.insert(config.sensor, config).is_none());
            assert!(topics.insert(config.topic));
            assert_eq!(
                config.topic,
                format!("/m49/native/altimeter/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m49/native/{index}"));
            let period_ms = [10, 20, 40][index % 3];
            assert_eq!(config.period_ns, period_ms * 1_000_000);
            assert_eq!(u64::from(config.frequency), 1000 / period_ms);
            let reference = f64::from_bits(config.initial_reference);
            assert!(reference.is_finite());
            assert_eq!(reference, 3.5 + 1.25 * index as f64);
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains("<sensor name='altimeter' type='altimeter'>"));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(
                xml.contains(format!("<update_rate>{}</update_rate>", config.frequency).as_str())
            );
            assert!(xml.contains("<altimeter/>") && !xml.contains("<noise"));
            sequences.insert(config.sensor, 0_u64);
            owner_packets.insert(config.sensor, 0_u64);
        }
        let mut input_keys = BTreeSet::new();
        let mut previous_time = BTreeMap::new();
        let mut mode_packets = 0;
        let mut surviving_peer_at_40 = false;
        for row in rows {
            inputs += 1;
            let config = owners
                .get(&row.sensor)
                .expect("input belongs to one configured owner");
            assert!(input_keys.insert((row.sensor, row.time_ns)));
            assert!(row.time_ns > 0 && row.time_ns <= 40_000_000);
            if let Some(previous) = previous_time.insert(row.sensor, row.time_ns) {
                assert!(previous < row.time_ns);
            }
            if mode == "remove" && row.sensor == 0 {
                assert!(row.time_ns <= 20_000_000);
            }
            let world = row.world_pose.map(f64::from_bits);
            let point_velocity = row.world_point_velocity.map(f64::from_bits);
            assert!(world.into_iter().chain(point_velocity).all(f64::is_finite));
            assert_eq!(&world[3..], &[0.0, 0.0, 0.0, 1.0]);
            // These are the actual sensor WORLD pose and WORLD point velocity
            // supplied to the original System. No link pose/omega, integration,
            // offset transform or native Physics trajectory is reconstructed.
            let Some(output) = row.observed else {
                // Missing captured output is not Sensor::Update(false), a
                // skipped draw, a deadline observation or Rust demand parity.
                no_packet += 1;
                continue;
            };
            let reading = altimeter(AltimeterInput {
                world_z: world[2],
                world_velocity_z: point_velocity[2],
                vertical_reference: f64::from_bits(config.initial_reference),
            })
            .unwrap();
            close(
                reading.vertical_position,
                output.vertical_position,
                mode,
                row.sensor,
                "position",
            );
            close(
                reading.vertical_velocity,
                output.vertical_velocity,
                mode,
                row.sensor,
                "velocity",
            );
            close(
                reading.vertical_reference,
                output.vertical_reference,
                mode,
                row.sensor,
                "reference",
            );
            assert_eq!(output.vertical_reference, config.initial_reference);
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
            assert_eq!(event.after_time_ns, 20_000_000);
            assert_eq!(event.next_system_time_ns, 40_000_000);
            assert!(event.peer_connected_required && surviving_peer_at_40);
            assert_eq!(previous_time[&0], event.after_time_ns);
            assert_eq!(previous_time[&1], event.next_system_time_ns);
            assert_eq!(owner_packets[&0], 3);
            assert_eq!(owner_packets[&1], 3);
            assert_eq!(summary.removal_after_time_ns, Some(event.after_time_ns));
            assert_eq!(summary.retirement_observed_system_time_ns, Some(40_000_000));
            // Original-System retirement is witnessed only after the due
            // connected-peer call. No all-disconnected immediacy is claimed.
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
        (12, 104, 441, 42, 399, 1)
    );
}
