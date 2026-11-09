// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_speed::{AirSpeedInput, AirSpeedReading, air_speed_unscaled, scale_air_speed};
use gz_sensors::{GaussianNoise, GaussianParameters, NoiseStream};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    AirSpeedSample, Name, PhysicsEngine, load_world_seeded, reset_loaded_world,
    set_air_speed_interest,
};
use sdformat::NoResources;
use std::time::Duration;
const XML: &str = include_str!("fixtures/m26_air_speed.sdf");
fn xml(noise: &str) -> String {
    XML.replace(
        "<air_speed/>",
        &format!("<air_speed><pressure>{noise}</pressure></air_speed>"),
    )
}
fn load(xml: &str, seed: [u8; 32]) -> Simulation {
    load_world_seeded(
        xml,
        "m28-noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
    .unwrap()
}
fn sensor(sim: &Simulation) -> (bevy_ecs::entity::Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rig::body::air_speed")
        })
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn reading(sim: &Simulation) -> (Duration, AirSpeedReading) {
    let sample = sim.world().get::<AirSpeedSample>(sensor(sim).0).unwrap();
    assert_eq!(sample.epoch, sim.epoch());
    (sample.time, *sample.reading.as_ref().unwrap())
}
fn scaled(noisy: f64) -> AirSpeedReading {
    let (_, t) = air_speed_unscaled(AirSpeedInput {
        world_z: 3.5,
        velocity_sensor: [0.0; 3],
    })
    .unwrap();
    scale_air_speed(noisy as f32, t).unwrap()
}
#[test]
fn enabled_zero_is_ideal_and_noise_precedes_float_conversion_and_scale_for_both_kinds() {
    let mut ideal = load(XML, [42; 32]);
    let mut zero = load(&xml("<noise type='gaussian'/>"), [42; 32]);
    for _ in 0..30 {
        ideal.step_exact(1).unwrap();
        zero.step_exact(1).unwrap();
        assert_eq!(reading(&zero), reading(&ideal));
    }
    for kind in ["gaussian", "gaussian_quantized"] {
        for (mean, precision) in [
            (0.1, 0.0),
            (-1.0, 0.0),
            (-0.125, 0.25),
            (1.75e-6, 1e-6),
            (1.75e-6, f64::from_bits(1e-6_f64.to_bits() + 1)),
        ] {
            let mut sim = load(
                &xml(&format!(
                    "<noise type='{kind}'><mean>{mean}</mean><precision>{precision}</precision></noise>"
                )),
                [42; 32],
            );
            sim.step_exact(1).unwrap();
            let expected = if precision > 1e-6 {
                (mean / precision).round() * precision
            } else {
                mean
            };
            assert_eq!(reading(&sim).1, scaled(expected));
        }
    }
    let mut noisy = load(
        &xml("<noise type='gaussian'><mean>0.1</mean></noise>"),
        [42; 32],
    );
    noisy.step_exact(1).unwrap();
    assert_eq!(reading(&noisy).1.diff_pressure, 10.0);
    assert_eq!(reading(&noisy).1.temperature, reading(&ideal).1.temperature);
}
#[test]
fn seeded_pressure_stream_is_repeatable_and_independent_of_other_seeds_and_imu_draws() {
    let config =
        xml("<noise type='gaussian'><stddev>0.2</stddev><bias_stddev>0.3</bias_stddev></noise>");
    let with_imu=config.replace("</sensor>","</sensor><sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate><imu><angular_velocity><x><noise type='gaussian'><stddev>2</stddev></noise></x></angular_velocity></imu></sensor>");
    let mut a = load(&config, [42; 32]);
    let mut b = load(&config, [42; 32]);
    let mut c = load(&with_imu, [42; 32]);
    let mut d = load(&config, [43; 32]);
    assert_eq!(sensor(&a).1, sensor(&c).1);
    let mut different = false;
    let mut values = Vec::new();
    for tick in 1..=100 {
        for sim in [&mut a, &mut b, &mut c, &mut d] {
            sim.step_exact(1).unwrap();
        }
        assert_eq!(reading(&a), reading(&b));
        assert_eq!(reading(&a), reading(&c));
        different |= reading(&a).1 != reading(&d).1;
        if tick == 1 || tick % 10 == 0 {
            values.push(reading(&a).1.diff_pressure);
        }
    }
    assert!(different && values.windows(2).any(|v| v[0] != v[1]));
}
#[test]
fn white_hundred_sample_moments_use_published_hundredfold_noise_units() {
    let config = xml("<noise type='gaussian'><mean>0.1</mean><stddev>0.2</stddev></noise>")
        .replace(
            "<update_rate>100</update_rate>",
            "<update_rate>1000</update_rate>",
        );
    let mut sim = load(&config, [42; 32]);
    let mut values = Vec::new();
    for tick in 1..=100 {
        sim.step_exact(1).unwrap();
        assert_eq!(reading(&sim).0, Duration::from_millis(tick));
        values.push(reading(&sim).1.diff_pressure);
    }
    let mean = values.iter().sum::<f64>() / 100.0;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / 99.0;
    assert!((mean - 10.0).abs() <= 5.0 * 20.0 / 100_f64.sqrt());
    assert!((variance - 20_f64.powi(2)).abs() <= 5.0 * (2.0 * 20_f64.powi(4) / 99.0).sqrt());
}
#[test]
fn pause_non_due_and_missing_demand_do_not_draw_and_dynamic_bias_receives_zero_dt() {
    let config = xml(
        "<noise type='gaussian'><stddev>0.2</stddev><bias_mean>0.3</bias_mean><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise>",
    );
    let mut sim = load(&config, [42; 32]);
    let (e, id) = sensor(&sim);
    let mut stream = NoiseStream::new([42; 32], id);
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
    set_air_speed_interest(&mut sim, []);
    sim.step_exact(20).unwrap();
    assert!(sim.world().get::<AirSpeedSample>(e).is_none());
    set_air_speed_interest(&mut sim, [e]);
    sim.step_exact(1).unwrap();
    assert_eq!(
        reading(&sim),
        (
            Duration::from_millis(21),
            scaled(oracle.apply(0.0, 0.0, &mut stream).unwrap())
        )
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
    assert_eq!(
        reading(&sim),
        (
            Duration::from_millis(30),
            scaled(oracle.apply(0.0, 0.0, &mut stream).unwrap())
        )
    );
    set_air_speed_interest(&mut sim, []);
    sim.step_exact(50).unwrap();
    assert!(sim.world().get::<AirSpeedSample>(e).is_none());
    set_air_speed_interest(&mut sim, [e]);
    sim.step_exact(1).unwrap();
    assert_eq!(
        reading(&sim),
        (
            Duration::from_millis(81),
            scaled(oracle.apply(0.0, 0.0, &mut stream).unwrap())
        )
    );
    assert_eq!(oracle.bias().abs(), 0.3);
}
#[test]
fn core_and_loaded_resets_replay_the_seed_with_stable_public_identity_and_fresh_handle() {
    let config = xml(
        "<noise type='gaussian_quantized'><stddev>0.2</stddev><bias_stddev>0.5</bias_stddev><precision>0.01</precision></noise>",
    );
    let mut sim = load(&config, [42; 32]);
    let (old, id) = sensor(&sim);
    sim.step_exact(1).unwrap();
    let first = reading(&sim);
    sim.step_exact(99).unwrap();
    sim.reset().unwrap();
    assert!(sim.world().get::<AirSpeedSample>(old).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim), first);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let (fresh, newid) = sensor(&sim);
    assert_eq!(id, newid);
    assert_ne!(old, fresh);
    assert_eq!(entity_by_id(sim.world(), id), Some(fresh));
    assert!(sim.world().get::<AirSpeedSample>(fresh).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim), first);
}
#[test]
fn malformed_noise_is_refused_at_its_real_parser_or_scene_boundary() {
    for field in ["stddev", "bias_stddev", "dynamic_bias_stddev"] {
        assert!(
            sdformat::load_str(
                &xml(&format!(
                    "<noise type='gaussian'><{field}>-1</{field}></noise>"
                )),
                "negative",
                &NoResources
            )
            .is_err()
        );
    }
    for noise in [
        "<noise type='none'><mean>1</mean></noise>",
        "<noise type='gaussian'><precision>-1</precision></noise>",
    ] {
        let config = xml(noise);
        sdformat::load_str(&config, "parsed", &NoResources).unwrap();
        assert!(
            load_world_seeded(
                &config,
                "bad",
                "default",
                None,
                &NoResources,
                PhysicsEngine::Rapier,
                [42; 32]
            )
            .is_err()
        );
    }
}
