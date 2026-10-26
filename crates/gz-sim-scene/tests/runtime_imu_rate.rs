// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, StepFault};
use gz_sim_scene::{
    ImuPublication, ImuRateError, ImuSample, Name, PhysicsEngine, RateChange, imu_rate,
    load_world_with_engine, set_imu_rate,
};
use sdformat::NoResources;
use std::time::Duration;

fn loaded(maximum: f64) -> (gz_sim_core::Simulation, u64) {
    loaded_with_engine(maximum, PhysicsEngine::Tpe)
}

fn loaded_with_engine(maximum: f64, engine: PhysicsEngine) -> (gz_sim_core::Simulation, u64) {
    let fixture = include_str!("fixtures/fall.sdf");
    assert_eq!(
        fixture.matches("<update_rate>1000</update_rate>").count(),
        1
    );
    let xml = fixture.replace(
        "<update_rate>1000</update_rate>",
        &format!("<update_rate>{maximum}</update_rate>"),
    );
    let sim = load_world_with_engine(
        &xml,
        "rate.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<ImuPublication>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (sim, id)
}

fn value(data: &serde_json::Value) -> f64 {
    match data.as_str() {
        Some("nan") => f64::NAN,
        Some("inf") => f64::INFINITY,
        Some("-inf") => f64::NEG_INFINITY,
        None => data.as_f64().unwrap(),
        _ => panic!("unexpected native numeric tag"),
    }
}

#[test]
fn scene_rate_admission_matches_all_native_getter_and_sign_observations() {
    let data: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/runtime_imu_rate.json")).unwrap();
    let groups = data["observations"]["bounds"].as_array().unwrap();
    assert_eq!(groups.len(), 3);
    for group in groups {
        let maximum = group["sdf_max"].as_f64().unwrap();
        let (mut sim, id) = loaded(maximum);
        assert_eq!(imu_rate(&sim, id).unwrap().rate_hz, maximum);
        let rows = group["rows"].as_array().unwrap();
        assert_eq!(rows.len(), 15);
        for row in rows {
            let mut requested = value(&row["request"]);
            if row["request_negative_zero"].as_bool().unwrap() {
                requested = -0.0;
            }
            let change = set_imu_rate(&mut sim, id, requested).unwrap();
            let current = imu_rate(&sim, id).unwrap();
            let expected = value(&row["rate"]);
            assert_eq!(current.sdf_max_hz, maximum);
            assert!(
                current.rate_hz == expected || (current.rate_hz.is_nan() && expected.is_nan()),
                "{row}"
            );
            assert_eq!(
                current.rate_hz == 0.0 && current.rate_hz.is_sign_negative(),
                row["rate_negative_zero"].as_bool().unwrap()
            );
            if maximum == 10.0
                && (requested == 0.0
                    || requested == 20.0
                    || requested == -5.0
                    || requested == 1e-7
                    || requested >= 10.0 + 1e-6
                    || requested.is_nan()
                    || requested == f64::NEG_INFINITY)
            {
                assert_eq!(change, RateChange::Ignored);
            } else {
                assert_eq!(change, RateChange::Applied);
            }
        }
    }
}

#[test]
fn invalid_rate_and_entity_admission_preserve_existing_configuration() {
    let (mut sim, id) = loaded(0.0);
    set_imu_rate(&mut sim, id, 5.0).unwrap();
    for rate in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        assert!(matches!(
            set_imu_rate(&mut sim, id, rate),
            Err(ImuRateError::InvalidPeriod)
        ));
        assert_eq!(imu_rate(&sim, id).unwrap().rate_hz, 5.0);
    }
    let body = sim
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
    for missing in [0, u64::MAX, body] {
        assert!(matches!(
            imu_rate(&sim, missing),
            Err(ImuRateError::MissingImu(_))
        ));
        assert!(matches!(
            set_imu_rate(&mut sim, missing, 1.0),
            Err(ImuRateError::MissingImu(_))
        ));
    }
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("test fault"));
    assert!(matches!(
        set_imu_rate(&mut sim, id, 1.0),
        Err(ImuRateError::Clock(_))
    ));
    assert_eq!(imu_rate(&sim, id).unwrap().rate_hz, 5.0);
}

#[test]
fn runtime_rates_preserve_pending_phase_and_acquisition_on_both_engines() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut sim, id) = loaded_with_engine(10.0, engine);
        let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
        let sample_time =
            |sim: &gz_sim_core::Simulation| sim.world().get::<ImuSample>(entity).unwrap().time;
        sim.step_exact(50).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(1));
        set_imu_rate(&mut sim, id, 5.0).unwrap();
        sim.step_exact(50).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(100));
        sim.step_exact(199).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(100));
        sim.step_exact(1).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(300));
        set_imu_rate(&mut sim, id, 10.0).unwrap();
        sim.step_exact(199).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(300));
        sim.step_exact(1).unwrap();
        assert_eq!(sample_time(&sim), Duration::from_millis(500));
        let reading = sim
            .world()
            .get::<ImuSample>(entity)
            .unwrap()
            .reading
            .as_ref()
            .unwrap();
        assert!(reading.specific_force.into_iter().all(f64::is_finite));
        assert_eq!(imu_rate(&sim, id).unwrap().sdf_max_hz, 10.0);
    }
}
