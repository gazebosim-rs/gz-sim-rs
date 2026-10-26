// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::EntityId;
use gz_sim_scene::{
    ImuPublication, ImuSample, PhysicsEngine, load_world_with_engine, set_imu_interest,
    set_imu_rate,
};
use sdformat::NoResources;
use std::time::Duration;

#[test]
fn raw_future_deadline_requires_a_due_connected_imu_before_zero_or_nan_updates() {
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/runtime_imu_future_gate.json")).unwrap();
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        for row in reference["observations"].as_array().unwrap() {
            let mode = row["mode"].as_str().unwrap();
            let peer_connected = mode.ends_with("-peer");
            let mut sim = load_world_with_engine(
                include_str!("fixtures/runtime_imu_future_gate.sdf"),
                "future.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            let sensors: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|entity| {
                    let publication = entity.get::<ImuPublication>()?;
                    Some((
                        publication.topic.clone(),
                        entity.id(),
                        entity.get::<EntityId>()?.get(),
                    ))
                })
                .collect();
            let (_, target, target_id) = sensors
                .iter()
                .find(|(topic, _, _)| topic == "/p2/imu")
                .unwrap();
            let (_, peer, _) = sensors
                .iter()
                .find(|(topic, _, _)| topic == "/p2/other")
                .unwrap();
            set_imu_interest(
                &mut sim,
                [Some(*target), peer_connected.then_some(*peer)]
                    .into_iter()
                    .flatten(),
            );
            set_imu_rate(&mut sim, *target_id, 20.0).unwrap();
            let mut actual = Vec::new();
            let mut actual_peer = Vec::new();
            for step in 1..=53 {
                if step == 2 {
                    set_imu_rate(
                        &mut sim,
                        *target_id,
                        if mode.starts_with("nan") {
                            f64::NAN
                        } else {
                            0.0
                        },
                    )
                    .unwrap();
                }
                sim.step_exact(1).unwrap();
                for (entity, samples) in [(*target, &mut actual), (*peer, &mut actual_peer)] {
                    if let Some(sample) = sim.world().get::<ImuSample>(entity)
                        && sample.time == sim.elapsed()
                    {
                        assert!(sample.reading.is_ok());
                        samples.push(sample.time.as_nanos() as u64);
                    }
                }
            }
            let times = |key: &str| {
                row[key]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_u64().unwrap())
                    .collect::<Vec<_>>()
            };
            assert_eq!(actual, times("target_times_ns"), "{engine:?}/{mode}");
            if peer_connected {
                assert_eq!(
                    actual_peer,
                    times("peer_times_ns"),
                    "{engine:?}/{mode}/peer"
                );
            }
        }
    }
}

#[test]
fn always_acquire_without_interest_still_obeys_the_single_imu_raw_deadline() {
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/runtime_imu_future_gate.json")).unwrap();
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        for row in reference["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|row| !row["mode"].as_str().unwrap().ends_with("-peer"))
        {
            let xml = include_str!("fixtures/headless_imu.sdf").replace(
                "<update_rate>1000</update_rate>",
                "<update_rate>0</update_rate>",
            );
            let mut sim = load_world_with_engine(
                &xml,
                "single.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            gz_sim_scene::clear_imu_interest(&mut sim);
            let entity = sim
                .world()
                .iter_entities()
                .find(|entity| entity.contains::<ImuPublication>())
                .unwrap();
            let (sensor, id) = (entity.id(), entity.get::<EntityId>().unwrap().get());
            set_imu_rate(&mut sim, id, 20.0).unwrap();
            let mut actual = Vec::new();
            for step in 1..=53 {
                if step == 2 {
                    let rate = if row["mode"] == "nan" { f64::NAN } else { 0.0 };
                    set_imu_rate(&mut sim, id, rate).unwrap();
                }
                sim.step_exact(1).unwrap();
                if let Some(sample) = sim.world().get::<ImuSample>(sensor)
                    && sample.time == sim.elapsed()
                {
                    assert!(sample.reading.is_ok());
                    actual.push(sample.time.as_nanos() as u64);
                }
            }
            let expected: Vec<_> = row["target_times_ns"]
                .as_array()
                .unwrap()
                .iter()
                .map(|time| time.as_u64().unwrap())
                .collect();
            assert_eq!(
                actual, expected,
                "{engine:?}/{} without interest",
                row["mode"]
            );
            assert_eq!(actual.len(), 5);
        }
    }
}
