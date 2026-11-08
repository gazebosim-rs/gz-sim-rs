// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_pressure::{AirPressureInput, AirPressureReading, air_pressure};
use gz_sensors::{GaussianNoise, GaussianParameters, NoiseStream};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    AirPressureSample, AirPressureSampleError, Name, PhysicsEngine, load_world_seeded,
    reset_loaded_world, set_air_pressure_interest,
};
use sdformat::NoResources;
use std::time::Duration;

const XML: &str = include_str!("fixtures/m15_air_pressure.sdf");

fn xml(noise: &str) -> String {
    XML.replace(
        "<air_pressure/>",
        &format!("<air_pressure><pressure>{noise}</pressure></air_pressure>"),
    )
}

fn load(xml: &str, seed: [u8; 32]) -> Simulation {
    load_world_seeded(
        xml,
        "pressure-noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
    .expect("admitted pressure noise")
}

fn sensor(sim: &Simulation) -> (bevy_ecs::entity::Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::rig::body::air_pressure")
        })
        .expect("pressure sensor");
    (
        entity.id(),
        entity.get::<EntityId>().expect("public identity").get(),
    )
}

fn reading(sim: &Simulation) -> (Duration, AirPressureReading) {
    let sample = sim
        .world()
        .get::<AirPressureSample>(sensor(sim).0)
        .expect("acquired sample");
    assert_eq!(sample.epoch, sim.epoch());
    (
        sample.time,
        *sample.reading.as_ref().expect("valid noisy pressure"),
    )
}

#[test]
fn gaussian_defaults_and_both_quantized_kinds_preserve_ideal_or_exact_native_rounding() {
    let mut ideal = load(XML, [42; 32]);
    let mut default = load(&xml("<noise type='gaussian'/>"), [42; 32]);
    for _ in 0..30 {
        ideal.step_exact(1).unwrap();
        default.step_exact(1).unwrap();
        assert_eq!(reading(&default), reading(&ideal));
    }
    let pressure = air_pressure(AirPressureInput { world_z: 3.5 })
        .unwrap()
        .pressure;
    for kind in ["gaussian", "gaussian_quantized"] {
        let configured = xml(&format!(
            "<noise type='{kind}'><mean>1.23</mean><precision>0.25</precision></noise>"
        ));
        let mut sim = load(&configured, [42; 32]);
        sim.step_exact(1).unwrap();
        let (time, measured) = reading(&sim);
        assert_eq!(time, Duration::from_millis(1));
        assert_eq!(measured.pressure, ((pressure + 1.23) / 0.25).round() * 0.25);
        assert_eq!(measured.variance, 0.0);
    }
    for precision in ["0", "0.000001"] {
        let mut sim = load(
            &xml(&format!(
                "<noise type='gaussian_quantized'><mean>-1.23</mean><precision>{precision}</precision></noise>"
            )),
            [42; 32],
        );
        sim.step_exact(1).unwrap();
        assert_eq!(reading(&sim).1.pressure, pressure - 1.23);
    }
}

#[test]
fn seeded_white_pressure_repeats_and_other_sensor_streams_do_not_perturb_it() {
    let configured = xml(
        "<noise type='gaussian'><mean>1</mean><stddev>0.2</stddev><bias_stddev>0.3</bias_stddev></noise>",
    );
    // Added after pressure, so its stable public sensor identity is unchanged.
    let with_imu = configured.replace("</sensor>", "</sensor><sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate><imu><angular_velocity><x><noise type='gaussian'><stddev>2</stddev></noise></x></angular_velocity></imu></sensor>");
    let mut a = load(&configured, [42; 32]);
    let mut b = load(&configured, [42; 32]);
    let mut c = load(&with_imu, [42; 32]);
    let mut other_seed = load(&configured, [43; 32]);
    assert_eq!(sensor(&a).1, sensor(&c).1);
    let mut different = false;
    let mut samples = Vec::new();
    for tick in 1..=100 {
        for sim in [&mut a, &mut b, &mut c, &mut other_seed] {
            sim.step_exact(1).unwrap();
        }
        assert_eq!(reading(&a), reading(&b));
        assert_eq!(reading(&a), reading(&c));
        different |= reading(&a).1 != reading(&other_seed).1;
        if tick == 1 || tick % 10 == 0 {
            samples.push(reading(&a).1.pressure);
            assert_eq!(reading(&a).1.variance, 0.2_f64.sqrt());
        }
    }
    assert!(different);
    assert!(samples.windows(2).any(|pair| pair[0] != pair[1]));
}

#[test]
fn loaded_white_pressure_uses_upstream_hundred_sample_moment_bounds() {
    let configured = xml("<noise type='gaussian'><mean>1</mean><stddev>0.2</stddev></noise>")
        .replace(
            "<update_rate>100</update_rate>",
            "<update_rate>1000</update_rate>",
        );
    let mut sim = load(&configured, [42; 32]);
    let ideal = air_pressure(AirPressureInput { world_z: 3.5 })
        .unwrap()
        .pressure;
    let mut values = Vec::new();
    for tick in 1..=100 {
        sim.step_exact(1).unwrap();
        assert_eq!(reading(&sim).0, Duration::from_millis(tick));
        assert_eq!(reading(&sim).1.variance, 0.2_f64.sqrt());
        values.push(reading(&sim).1.pressure - ideal);
    }
    let mean = values.iter().sum::<f64>() / 100.0;
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / 99.0;
    assert!((mean - 1.0).abs() <= 5.0 * 0.2 / 100_f64.sqrt());
    assert!((variance - 0.2_f64.powi(2)).abs() <= 5.0 * (2.0 * 0.2_f64.powi(4) / 99.0).sqrt());
}

#[test]
fn pause_demand_and_deadlines_do_not_draw_and_native_dynamic_bias_always_uses_zero_dt() {
    let configured = xml(
        "<noise type='gaussian'><stddev>0.2</stddev><bias_mean>0.3</bias_mean><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise>",
    );
    let mut sim = load(&configured, [42; 32]);
    let (entity, identity) = sensor(&sim);
    let mut stream = NoiseStream::new([42; 32], identity);
    let mut oracle = GaussianNoise::new(
        GaussianParameters {
            stddev: 0.2,
            bias_mean: 0.3,
            dynamic_bias_stddev: 100.0,
            dynamic_bias_correlation_time: 2.0,
            ..Default::default()
        },
        &mut stream,
    )
    .unwrap();
    let ideal = air_pressure(AirPressureInput { world_z: 3.5 })
        .unwrap()
        .pressure;
    set_air_pressure_interest(&mut sim, []);
    sim.step_exact(20).unwrap();
    assert!(sim.world().get::<AirPressureSample>(entity).is_none());
    set_air_pressure_interest(&mut sim, [entity]);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim).0, Duration::from_millis(21));
    assert_eq!(
        reading(&sim).1.pressure,
        oracle.apply(ideal, 0.0, &mut stream).unwrap()
    );
    let held = reading(&sim);
    sim.step_exact(8).unwrap();
    assert_eq!(reading(&sim), held);
    sim.set_paused(true);
    for _ in 0..20 {
        sim.tick().unwrap();
    }
    assert_eq!(reading(&sim), held);
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim).0, Duration::from_millis(30));
    assert_eq!(
        reading(&sim).1.pressure,
        oracle.apply(ideal, 0.0, &mut stream).unwrap()
    );
    set_air_pressure_interest(&mut sim, []);
    sim.step_exact(50).unwrap();
    assert!(sim.world().get::<AirPressureSample>(entity).is_none());
    set_air_pressure_interest(&mut sim, [entity]);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim).0, Duration::from_millis(81));
    assert_eq!(
        reading(&sim).1.pressure,
        oracle.apply(ideal, 0.0, &mut stream).unwrap()
    );
    // A separate time-varying oracle would differ; native pressure does not
    // pass these 9ms/51ms gaps to its noise model.
    assert_eq!(oracle.bias().abs(), 0.3);
}

#[test]
fn direct_and_loaded_reset_replay_seeded_pressure_with_the_same_public_identity() {
    let configured = xml(
        "<noise type='gaussian_quantized'><stddev>0.2</stddev><bias_stddev>0.5</bias_stddev><precision>0.01</precision></noise>",
    );
    let mut sim = load(&configured, [42; 32]);
    let identity = sensor(&sim).1;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(1).unwrap();
    let first = reading(&sim);
    sim.step_exact(99).unwrap();
    sim.reset().unwrap();
    assert!(
        sim.world()
            .get::<AirPressureSample>(sensor(&sim).0)
            .is_none()
    );
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim), first);
    sim.step_exact(99).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sensor(&sim).1, identity);
    assert_eq!(entity_by_id(sim.world(), identity), Some(sensor(&sim).0));
    assert!(
        sim.world()
            .get::<AirPressureSample>(sensor(&sim).0)
            .is_none()
    );
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim), first);
    let model = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::rig")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sensor(&sim).1, identity);
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim), first);
}

#[test]
fn finite_correlation_times_cannot_evolve_native_pressure_bias_without_elapsed_input() {
    let ideal = air_pressure(AirPressureInput { world_z: 3.5 })
        .unwrap()
        .pressure;
    for correlation in [-2.0, 0.0, 2.0] {
        let configured = xml(&format!(
            "<noise type='gaussian'><bias_mean>-0.3</bias_mean><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>{correlation}</dynamic_bias_correlation_time></noise>"
        ));
        let mut sim = load(&configured, [42; 32]);
        sim.step_exact(1).unwrap();
        let first = reading(&sim).1;
        assert!(((first.pressure - ideal).abs() - 0.3).abs() < 1e-9);
        for _ in 0..100 {
            sim.step_exact(10).unwrap();
            assert_eq!(reading(&sim).1, first);
        }
    }
}

#[test]
fn unsupported_noise_is_refused_and_invalid_noisy_outputs_are_distinct_fresh_errors() {
    for body in [
        "<noise type='gaussian'><precision>-1</precision></noise>",
        "<noise type='gaussian_quantized'><precision>-1</precision></noise>",
        "<noise type='none'><mean>1</mean></noise>",
    ] {
        let configured = xml(body);
        sdformat::load_str(&configured, "parsed-pressure-noise", &NoResources).unwrap();
        assert!(matches!(
            load_world_seeded(
                &configured,
                "pressure-noise",
                "default",
                Some(Duration::from_millis(1)),
                &NoResources,
                PhysicsEngine::Rapier,
                [42; 32]
            ),
            Err(gz_sim_scene::LoadError::Unsupported {
                feature: "unrepresentable air_pressure noise",
                ..
            })
        ));
    }
    for body in [
        "<noise type='gaussian'><stddev>-1</stddev></noise>",
        "<noise type='gaussian'><bias_stddev>-1</bias_stddev></noise>",
        "<noise type='gaussian'><dynamic_bias_stddev>-1</dynamic_bias_stddev></noise>",
        "<noise type='gaussian'><mean>NaN</mean></noise>",
        "<noise type='gaussian'><stddev>inf</stddev></noise>",
    ] {
        assert!(sdformat::load_str(&xml(body), "invalid-pressure-xml", &NoResources).is_err());
    }
    for (body, nonpositive) in [
        ("<noise type='gaussian'><mean>-200000</mean></noise>", true),
        (
            "<noise type='gaussian'><mean>1e308</mean><precision>0.00001</precision></noise>",
            false,
        ),
    ] {
        let mut sim = load(&xml(body), [42; 32]);
        for expected_time in [1, 10] {
            let count = expected_time - sim.elapsed().as_millis() as u64;
            sim.step_exact(count).unwrap();
            let sample = sim
                .world()
                .get::<AirPressureSample>(sensor(&sim).0)
                .unwrap();
            assert_eq!(sample.time, Duration::from_millis(expected_time));
            if nonpositive {
                assert_eq!(
                    sample.reading,
                    Err(AirPressureSampleError::NonpositiveNoisyPressure)
                );
            } else {
                assert!(matches!(
                    sample.reading,
                    Err(AirPressureSampleError::Noise(_))
                ));
            }
        }
    }
}
