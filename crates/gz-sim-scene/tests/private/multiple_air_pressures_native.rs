// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_pressure::{AirPressureInput, air_pressure};
use std::collections::{BTreeMap, BTreeSet};

// The shared audited cards also retain canonical bytes for the separate Server
// encoder control. Limit the unused-field disposition to this fixture module.
#[allow(dead_code)]
mod native {
    // include! keeps the exact independently audited literal bytes under fmt.
    include!("../fixtures/m50_native_multiple_air_pressures.rs");
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
fn actual_native_pressure_multiplicity_matches_all_42_scalar_readings_and_owner_inputs() {
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
                format!("/m50/native/air_pressure/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m50/native/{index}"));
            let period_ms = [10, 20, 40][index % 3];
            assert_eq!(config.period_ns, period_ms * 1_000_000);
            assert_eq!(u64::from(config.frequency), 1000 / period_ms);
            let initial_z = f64::from_bits(config.initial_world_z);
            assert!(initial_z.is_finite());
            assert_eq!(initial_z, 3.5 + 1.25 * index as f64);
            assert_eq!(config.reference_altitude, 0.0_f64.to_bits());
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains("<sensor name='air_pressure' type='air_pressure'>"));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(
                xml.contains(format!("<update_rate>{}</update_rate>", config.frequency).as_str())
            );
            assert!(xml.contains("<air_pressure/>") && !xml.contains("<noise"));
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
            assert!(world.into_iter().all(f64::is_finite));
            assert_eq!(&world[3..], &[0.0, 0.0, 0.0, 1.0]);
            assert_eq!(world[0], 2.0 + f64::from(row.sensor));
            assert_eq!(world[1], -1.0);
            assert!(world[2] > f64::from_bits(config.initial_world_z));
            // The acquisition input is the actual sensor WORLD pose supplied
            // to the original System. Initial WORLD Z is ownership metadata,
            // not an altitude reference to subtract. There is no point velocity,
            // link pose transform or reconstructed native Physics trajectory.
            let packet_due =
                row.time_ns == 1_000_000 || row.time_ns.is_multiple_of(config.period_ns);
            assert_eq!(
                row.observed.is_some(),
                packet_due,
                "captured packet cadence only"
            );
            let Some(output) = row.observed else {
                // Missing captured output is not Sensor::Update(false), a
                // skipped draw, a deadline observation or Rust demand parity.
                no_packet += 1;
                continue;
            };
            let reading = air_pressure(AirPressureInput { world_z: world[2] }).unwrap();
            // Original ideal pressure is compared at the unchanged M15/M22
            // numerical bound. Exact header/full-wire encoding is a distinct
            // Server obligation; no noise/RNG or bitwise libm claim is added.
            close(
                reading.pressure,
                output.pressure,
                mode,
                row.sensor,
                "pressure",
            );
            close(
                reading.variance,
                output.variance,
                mode,
                row.sensor,
                "variance",
            );
            assert!(reading.pressure > 0.0 && f64::from_bits(output.pressure) > 0.0);
            assert_eq!(output.variance, 0.0_f64.to_bits());
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
