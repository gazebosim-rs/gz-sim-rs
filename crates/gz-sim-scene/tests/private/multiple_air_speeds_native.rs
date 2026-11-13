// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::{BTreeMap, BTreeSet};

// The shared audited cards also retain canonical bytes for the separate Server
// encoder control. Limit the unused-field disposition to this fixture module.
#[allow(dead_code)]
mod native {
    // include! keeps the exact independently audited literal bytes under fmt.
    include!("../fixtures/m51_native_multiple_air_speeds.rs");
}

#[test]
fn actual_native_air_speed_multiplicity_matches_all_55_exact_scalar_readings_and_owner_inputs() {
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
        assert_eq!(summary.system_calls, if mode == "remove" { 17 } else { 31 });
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
                format!("/m51/native/air_speed/{mode}/{index}")
            );
            assert_eq!(config.frame_id, format!("m51/native/{index}"));
            let period_ms = [5, 10, 20][index % 3];
            assert_eq!(config.period_ns, period_ms * 1_000_000);
            assert_eq!(u64::from(config.frequency), 1000 / period_ms);
            let initial = config.initial_world_pose.map(f64::from_bits);
            let local = config.sensor_local_pose.map(f64::from_bits);
            let wind = config.world_wind.map(f64::from_bits);
            assert!(
                initial
                    .into_iter()
                    .chain(local)
                    .chain(wind)
                    .all(f64::is_finite)
            );
            assert_eq!(
                &initial[..3],
                &[2.25 + index as f64, -1.5, 3.5 + 1.25 * index as f64]
            );
            assert_eq!(&local[..3], &[0.25, -0.5, 0.5 + 0.25 * index as f64]);
            assert_eq!(wind, [1.0, -2.0, 0.5]);
            for pose in [initial, local] {
                let q = DQuat::from_xyzw(pose[3], pose[4], pose[5], pose[6]);
                assert!((q.length_squared() - 1.0).abs() <= 1e-12);
            }
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains("<sensor name='air_speed' type='air_speed'>"));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(
                xml.contains(format!("<update_rate>{}</update_rate>", config.frequency).as_str())
            );
            assert!(xml.contains("<air_speed/>") && !xml.contains("<noise"));
            sequences.insert(config.sensor, 0_u64);
            owner_packets.insert(config.sensor, 0_u64);
        }
        let mut input_keys = BTreeSet::new();
        let mut previous_time = BTreeMap::new();
        let mut mode_packets = 0;
        let mut surviving_peer_at_20 = false;
        for row in rows {
            inputs += 1;
            let config = owners
                .get(&row.sensor)
                .expect("input belongs to one configured owner");
            assert!(input_keys.insert((row.sensor, row.time_ns)));
            assert!(row.time_ns > 0 && row.time_ns <= 30_000_000);
            if let Some(previous) = previous_time.insert(row.sensor, row.time_ns) {
                assert!(previous < row.time_ns);
            }
            if mode == "remove" && row.sensor == 0 {
                assert!(row.time_ns <= 15_000_000);
            }
            let world = row.world_pose.map(f64::from_bits);
            assert!(world.into_iter().all(f64::is_finite));
            let q = [world[3], world[4], world[5], world[6]];
            let rotation = DQuat::from_array(q);
            assert!((rotation.length_squared() - 1.0).abs() <= 1e-12);
            let point = row.world_point_velocity.map(f64::from_bits);
            let wind = row.world_wind.map(f64::from_bits);
            let native_relative = row.native_relative_velocity_handoff.map(f64::from_bits);
            assert!(
                point
                    .into_iter()
                    .chain(wind)
                    .chain(native_relative)
                    .all(f64::is_finite)
            );
            assert_eq!(row.world_wind, config.world_wind);
            // This public native relativeVel handoff is already sensor-axis
            // POINT velocity. Feed it directly once, with actual WORLD q/wind;
            // never substitute link-center velocity or invert this handoff again.
            let relative = relative_air_velocity(native_relative, q, wind);
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
            let reading = gz_sensors::air_speed::air_speed(AirSpeedInput {
                world_z: world[2],
                velocity_sensor: relative,
            })
            .unwrap();
            // The same-API handoff/scalar obligation retains M41's EXACT bits,
            // including signed differential pressure. Temperature stays positive.
            assert_eq!(
                reading.diff_pressure.to_bits(),
                output.diff_pressure,
                "{mode}/{}",
                row.sensor
            );
            assert_eq!(
                reading.temperature.to_bits(),
                output.temperature,
                "{mode}/{}",
                row.sensor
            );
            assert!(reading.diff_pressure.is_finite() && reading.temperature > 0.0);
            // Independent hierarchy/POINT-WORLD geometry is a separate witness
            // at the existing live bound, not a relaxation of the exact proof.
            // This uses supplied sensor-point velocity, not reconstructed Physics.
            let geometry = relative_air_velocity(
                (rotation.inverse() * DVec3::from_array(point)).to_array(),
                q,
                wind,
            );
            let geometry_reading = gz_sensors::air_speed::air_speed(AirSpeedInput {
                world_z: world[2],
                velocity_sensor: geometry,
            })
            .unwrap();
            assert!(
                (geometry_reading.diff_pressure - f64::from_bits(output.diff_pressure)).abs()
                    < 1e-6
            );
            assert_eq!(geometry_reading.temperature.to_bits(), output.temperature);
            let next = sequences.get_mut(&row.sensor).unwrap();
            assert_eq!(output.sequence, next.to_string());
            *next += 1;
            *owner_packets.get_mut(&row.sensor).unwrap() += 1;
            assert!(!output.header_wire.is_empty() && output.header_wire.len() <= 8192);
            assert!(!output.wire.is_empty() && output.wire.len() <= 8192);
            observed += 1;
            mode_packets += 1;
            surviving_peer_at_20 |=
                mode == "remove" && row.sensor == 1 && row.time_ns == 20_000_000;
        }
        assert_eq!(summary.canonical_packets, mode_packets);
        assert_eq!(
            mode_packets,
            if mode == "pair" {
                11
            } else if mode == "eight" {
                37
            } else {
                7
            }
        );
        assert!(owner_packets.values().all(|count| *count > 0));
        if let Some(event) = removal {
            removals += 1;
            assert_eq!(mode, "remove");
            assert_eq!(event.sensor, 0);
            assert_eq!(event.after_time_ns, 15_000_000);
            assert_eq!(event.next_system_time_ns, 20_000_000);
            assert!(event.peer_connected_required && surviving_peer_at_20);
            assert_eq!(previous_time[&0], event.after_time_ns);
            assert_eq!(previous_time[&1], event.next_system_time_ns);
            assert_eq!(owner_packets[&0], 4);
            assert_eq!(owner_packets[&1], 3);
            assert_eq!(summary.removal_after_time_ns, Some(event.after_time_ns));
            assert_eq!(summary.retirement_observed_system_time_ns, Some(20_000_000));
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
        (12, 79, 331, 55, 276, 1)
    );
}
