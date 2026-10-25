// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, ImuReading, NoiseStream};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    EntityKind, ImuSample, Name, PhysicsEngine, load_world_seeded, reset_loaded_world,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, time::Duration};

fn xml(config: &str) -> String {
    include_str!("fixtures/fall.sdf").replace("</sensor>", &format!("<imu>{config}</imu></sensor>"))
}

fn load(xml: &str, engine: PhysicsEngine) -> Simulation {
    load_world_seeded(
        xml,
        "noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        engine,
        [42; 32],
    )
    .unwrap()
}

fn readings(sim: &Simulation) -> BTreeMap<String, (u64, Duration, ImuReading)> {
    sim.world()
        .iter_entities()
        .filter_map(|e| {
            let sample = e.get::<ImuSample>()?;
            Some((
                e.get::<Name>()?.0.clone(),
                (
                    e.get::<EntityId>()?.get(),
                    sample.time,
                    *sample.reading.as_ref().unwrap(),
                ),
            ))
        })
        .collect()
}

#[test]
fn loaded_mean_precision_covariance_and_unchanged_physical_conventions() {
    let config = "<angular_velocity><x><noise type='gaussian'><mean>2.25</mean><precision>0.5</precision></noise></x><y><noise type='gaussian_quantized'><mean>-2.25</mean><precision>0.5</precision></noise></y><z><noise type='gaussian'><stddev>0.1</stddev></noise></z></angular_velocity><linear_acceleration><x><noise type='gaussian_quantized'><mean>0.32</mean><precision>0.3</precision></noise></x><z><noise type='gaussian'><mean>0.1</mean></noise></z></linear_acceleration>";
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(&xml(config), engine);
        sim.step_exact(1).unwrap();
        let (_, time, reading) = readings(&sim).into_values().next().unwrap();
        assert_eq!(time, Duration::from_millis(1));
        assert_eq!(reading.orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(&reading.angular_velocity[..2], &[2.5, -2.5]);
        assert_eq!(reading.specific_force[0], 0.3);
        assert_eq!(reading.specific_force[1], 0.0);
        let gravity = if engine == PhysicsEngine::Tpe {
            9.81
        } else {
            0.0
        };
        assert!((reading.specific_force[2] - gravity - 0.1).abs() < 1e-12);
        assert_eq!(
            reading.angular_velocity_covariance,
            [0.0, 0.0, 0.01f32 as f64]
        );
        assert_eq!(reading.linear_acceleration_covariance, [0.0; 3]);
    }
}

#[test]
fn independent_sensor_streams_pause_and_loaded_reset_replay_initial_samples() {
    let fixture = xml(
        "<angular_velocity><x><noise type='gaussian'><stddev>0.2</stddev><bias_stddev>0.3</bias_stddev><dynamic_bias_stddev>0.4</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise></x></angular_velocity>",
    );
    let sensor = fixture
        .split("<sensor ")
        .nth(1)
        .unwrap()
        .split("</sensor>")
        .next()
        .unwrap();
    let doubled = fixture.replace(
        "</link>\n    </model>\n  </world>",
        &format!(
            "<sensor {}</sensor></link>\n    </model>\n  </world>",
            sensor.replacen("name=\"imu\"", "name=\"imu2\"", 1)
        ),
    );
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut a = load(&doubled, engine);
        let mut b = load(&doubled, engine);
        let mut first = None;
        for _ in 0..100 {
            a.step_exact(1).unwrap();
            b.step_exact(1).unwrap();
            let actual = readings(&a);
            assert_eq!(actual.len(), 2);
            assert_eq!(actual, readings(&b));
            let values: Vec<_> = actual
                .values()
                .map(|(_, _, r)| r.angular_velocity[0])
                .collect();
            assert_ne!(values[0], values[1]);
            if first.is_none() {
                first = Some(actual.clone());
            }
            a.set_paused(true);
            a.tick().unwrap();
            assert_eq!(readings(&a), actual);
            a.set_paused(false);
        }
        reset_loaded_world(&mut a, |_| ()).unwrap();
        assert!(readings(&a).is_empty());
        a.step_exact(1).unwrap();
        assert_eq!(readings(&a), first.unwrap());
    }
}

#[test]
fn dynamic_bias_uses_elapsed_acquisition_time_at_slow_output_rate() {
    let fixture = xml("<angular_velocity><x><noise type='gaussian'><dynamic_bias_stddev>0.7</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise></x></angular_velocity>").replace("<update_rate>1000</update_rate>", "<update_rate>10</update_rate>");
    let mut sim = load(&fixture, PhysicsEngine::Tpe);
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::Sensor))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let mut random = NoiseStream::new([42; 32], id);
    let mut oracle = GaussianNoise::new(
        GaussianParameters {
            dynamic_bias_stddev: 0.7,
            dynamic_bias_correlation_time: 2.0,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        readings(&sim)
            .into_values()
            .next()
            .unwrap()
            .2
            .angular_velocity[0],
        oracle.apply(0.0, 0.0, &mut random).unwrap()
    );
    sim.step_exact(98).unwrap();
    assert_eq!(
        readings(&sim).into_values().next().unwrap().1,
        Duration::from_millis(1)
    );
    sim.step_exact(1).unwrap();
    let (_, time, reading) = readings(&sim).into_values().next().unwrap();
    assert_eq!(time, Duration::from_millis(100));
    assert_eq!(
        reading.angular_velocity[0],
        oracle
            .apply(0.0, (99_000_000u64 as f32 / 1e9f32) as f64, &mut random)
            .unwrap()
    );
}

#[test]
fn overflowing_dynamic_bias_faults_without_publishing_a_nonfinite_measurement() {
    let fixture = xml(
        "<angular_velocity><x><noise type='gaussian'><dynamic_bias_stddev>1e308</dynamic_bias_stddev><dynamic_bias_correlation_time>1</dynamic_bias_correlation_time></noise></x></angular_velocity>",
    );
    let mut sim = load(&fixture, PhysicsEngine::Tpe);
    sim.step_exact(1).unwrap();
    let before = readings(&sim);
    assert!(sim.step_exact(1).is_err());
    assert_eq!(readings(&sim), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    assert!(sim.step_exact(1).is_err());
}

#[test]
fn formerly_rejected_default_gaussian_xml_preserves_noise_free_readings() {
    let old_payload =
        "<imu><angular_velocity><x><noise type='gaussian'/></x></angular_velocity></imu>";
    let baseline = include_str!("fixtures/fall.sdf");
    let configured = baseline.replace("</sensor>", &format!("{old_payload}</sensor>"));
    assert!(configured.contains(old_payload));
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut plain = load(baseline, engine);
        let mut gaussian = load(&configured, engine);
        for _ in 0..100 {
            plain.step_exact(1).unwrap();
            gaussian.step_exact(1).unwrap();
            assert_eq!(readings(&gaussian), readings(&plain));
        }
    }
}
