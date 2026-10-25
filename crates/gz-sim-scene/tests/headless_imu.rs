// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{ImuSample, PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::Value;
use std::time::Duration;

#[test]
fn shared_unpaced_sdf_matches_actual_native_acquisition_on_tpe_and_freefall_dynamics() {
    let proof: Value = serde_json::from_str(include_str!(
        "../../../planning/reference/headless-imu.json"
    ))
    .unwrap();
    let controls = proof["observations"].as_array().unwrap();
    assert_eq!(controls.len(), 2);
    assert_eq!(controls[0]["engine"], "dartsim");
    assert_eq!(controls[1]["engine"], "tpe");
    for control in controls {
        let engine = if control["engine"] == "tpe" {
            PhysicsEngine::Tpe
        } else {
            assert_eq!(control["engine"], "dartsim");
            PhysicsEngine::Rapier
        };
        let mut sim = load_world_with_sdf_step(
            include_str!("fixtures/headless_imu.sdf"),
            "headless_imu.sdf",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        assert_eq!(sim.step_duration(), Duration::from_millis(1));
        sim.tick().unwrap();
        assert!(
            sim.world()
                .iter_entities()
                .all(|e| !e.contains::<ImuSample>())
        );
        let rows = control["native"].as_array().unwrap();
        assert_eq!(rows.len(), 100);
        for (index, row) in rows.iter().enumerate() {
            sim.step_exact(1).unwrap();
            let samples: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<ImuSample>())
                .collect();
            assert_eq!(samples.len(), 1);
            let sample = samples[0];
            assert_eq!(sample.time.as_nanos(), ((index + 1) * 1_000_000) as u128);
            assert_eq!(
                sample.time.as_nanos(),
                row["time_ns"].as_u64().unwrap() as u128
            );
            assert_eq!(sample.epoch, 0);
            let reading = sample.reading.as_ref().unwrap();
            for (actual, key) in [
                (reading.specific_force.as_slice(), "specific_force"),
                (reading.angular_velocity.as_slice(), "angular_velocity"),
                (reading.orientation.as_slice(), "orientation"),
            ] {
                let expected = row[key].as_array().unwrap();
                assert_eq!(actual.len(), expected.len());
                for (actual, expected) in actual.iter().zip(expected) {
                    // Declared physical bound, independent of backend integration details.
                    assert!(
                        (actual - expected.as_f64().unwrap()).abs() < 1e-9,
                        "{:?} sample{} {key}: {actual} versus {expected}",
                        engine,
                        index + 1
                    );
                }
            }
        }
        let elapsed = sim.elapsed();
        sim.tick().unwrap();
        assert_eq!(sim.elapsed(), elapsed);
        let sample = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<ImuSample>())
            .unwrap();
        assert_eq!(sample.time, elapsed);
    }
}
