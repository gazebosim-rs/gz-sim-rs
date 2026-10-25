// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ImuSample, InputError, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use serde_json::Value;

#[test]
fn velocity_replacement_is_not_an_imu_force_and_real_wrench_acceleration_remains() {
    let proof: Value =
        serde_json::from_str(include_str!("../../../planning/reference/input-imu.json")).unwrap();
    let controls = proof["observations"].as_array().unwrap();
    assert_eq!(controls.len(), 3);
    assert_eq!(
        controls
            .iter()
            .map(|row| (
                row["engine"].as_str().unwrap(),
                row["mode"].as_str().unwrap()
            ))
            .collect::<Vec<_>>(),
        [
            ("dartsim", "velocity"),
            ("dartsim", "velocity_force"),
            ("tpe", "velocity")
        ]
    );
    for control in controls {
        let engine = match control["engine"].as_str().unwrap() {
            "dartsim" => PhysicsEngine::Rapier,
            "tpe" => PhysicsEngine::Tpe,
            other => panic!("unknown reference engine {other}"),
        };
        let mut sim = load_world_with_sdf_step(
            include_str!("fixtures/headless_imu.sdf"),
            "input_imu.sdf",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let link = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::ball::body")
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        queue_link_world_velocity(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        let rows = control["native"].as_array().unwrap();
        assert_eq!(rows.len(), 100);
        for (index, row) in rows.iter().enumerate() {
            if index == 1 && control["mode"] == "velocity_force" {
                queue_link_world_wrench(&mut sim, link, [2.0, 0.0, 0.0], [0.0; 3]).unwrap();
            }
            sim.step_exact(1).unwrap();
            let samples: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<ImuSample>())
                .collect();
            assert_eq!(samples.len(), 1);
            let sample = samples[0];
            assert_eq!(
                sample.time.as_nanos(),
                row["time_ns"].as_u64().unwrap() as u128
            );
            assert_eq!(sample.time.as_nanos(), ((index + 1) * 1_000_000) as u128);
            let reading = sample.reading.as_ref().unwrap();
            for (values, key) in [
                (reading.specific_force.as_slice(), "specific_force"),
                (reading.angular_velocity.as_slice(), "angular_velocity"),
                (reading.orientation.as_slice(), "orientation"),
            ] {
                let expected = row[key].as_array().unwrap();
                assert_eq!(values.len(), expected.len());
                for (actual, expected) in values.iter().zip(expected) {
                    assert!(
                        (actual - expected.as_f64().unwrap()).abs() < 1e-9,
                        "{engine:?}/{} sample{} {key}: {actual} vs {expected}",
                        control["mode"],
                        index + 1
                    );
                }
            }
        }
    }
}

fn named_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn assert_two_readings(sim: &Simulation, time_ms: u64, x: f64, z: f64) {
    let readings: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<ImuSample>())
        .collect();
    assert_eq!(readings.len(), 2);
    for sample in readings {
        assert_eq!(sample.time, std::time::Duration::from_millis(time_ms));
        let reading = sample.reading.as_ref().unwrap();
        for (actual, expected) in reading.specific_force.into_iter().zip([x, 0.0, z]) {
            assert!((actual - expected).abs() < 1e-9, "{actual} vs {expected}");
        }
    }
}

#[test]
fn all_attached_imus_preserve_history_through_pause_replacement_reset_and_removal() {
    let xml = include_str!("fixtures/headless_imu.sdf").replace(
        "</sensor>",
        "</sensor><sensor name='second' type='imu'><always_on>true</always_on><update_rate>1000</update_rate></sensor>",
    );
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim =
            load_world_with_sdf_step(&xml, "input-imu-lifecycle", "default", &NoResources, engine)
                .unwrap();
        let link = named_id(&sim, "default::ball::body");
        let z = if engine == PhysicsEngine::Tpe {
            9.81
        } else {
            0.0
        };
        sim.step_exact(1).unwrap();
        assert_two_readings(&sim, 1, 0.0, z);
        queue_link_world_velocity(&mut sim, link, [9.0, 0.0, 0.0], [0.0; 3]).unwrap();
        queue_link_world_velocity(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [f64::NAN; 3], [0.0; 3]),
            Err(InputError::InvalidValue)
        ));
        sim.tick().unwrap();
        assert_two_readings(&sim, 1, 0.0, z);
        // A real force in the same step remains visible after the velocity history is rebased.
        if engine == PhysicsEngine::Rapier {
            queue_link_world_wrench(&mut sim, link, [2.0, 0.0, 0.0], [0.0; 3]).unwrap();
        }
        sim.step_exact(1).unwrap();
        assert_two_readings(
            &sim,
            2,
            if engine == PhysicsEngine::Rapier {
                1.0
            } else {
                0.0
            },
            z,
        );
        sim.step_exact(1).unwrap();
        assert_two_readings(&sim, 3, 0.0, z);
        for loaded in [false, true] {
            queue_link_world_velocity(&mut sim, link, [7.0, 0.0, 0.0], [0.0; 3]).unwrap();
            if loaded {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            } else {
                sim.reset().unwrap();
            }
            sim.step_exact(1).unwrap();
            assert_two_readings(&sim, 1, 0.0, z);
        }
        queue_link_world_velocity(&mut sim, link, [7.0, 0.0, 0.0], [0.0; 3]).unwrap();
        let model = named_id(&sim, "default::ball");
        remove_model(&mut sim, model).unwrap();
        sim.step_exact(1).unwrap();
        assert!(
            sim.world()
                .iter_entities()
                .all(|e| e.get::<ImuSample>().is_none())
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_two_readings(&sim, 1, 0.0, z);
    }
}
