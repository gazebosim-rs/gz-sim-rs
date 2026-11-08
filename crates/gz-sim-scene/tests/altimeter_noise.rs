// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, NoiseStream};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    AltimeterSample, Name, PhysicsEngine, clear_altimeter_interest, load_world_seeded,
    reset_loaded_world, set_altimeter_interest,
};
use sdformat::NoResources;
use std::time::Duration;

const XML: &str = include_str!("fixtures/m13_altimeter.sdf");
fn xml(position: &str, velocity: &str) -> String {
    XML.replace("<altimeter/>", &format!("<altimeter><vertical_position>{position}</vertical_position><vertical_velocity>{velocity}</vertical_velocity></altimeter>"))
}
fn load(xml: &str, seed: u8) -> Simulation {
    load_world_seeded(
        xml,
        "m21-noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        PhysicsEngine::Rapier,
        [seed; 32],
    )
    .expect("admitted noisy altimeter")
}
fn identity(sim: &Simulation) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rig::body::altimeter")
        })
        .expect("sensor")
        .get::<EntityId>()
        .expect("stable ID")
        .get()
}
fn sample(sim: &Simulation) -> (Duration, gz_sensors::altimeter::AltimeterReading) {
    let sensor = entity_by_id(sim.world(), identity(sim)).expect("current identity");
    let sample = sim
        .world()
        .get::<AltimeterSample>(sensor)
        .expect("due acquisition");
    (
        sample.time,
        *sample.reading.as_ref().expect("finite reading"),
    )
}

#[test]
fn independent_channels_round_signed_means_and_leave_reference_and_default_readings_unchanged() {
    let configured = xml(
        "<noise type='gaussian'><mean>2.25</mean><precision>0.5</precision></noise>",
        "<noise type='gaussian_quantized'><mean>-2.25</mean><precision>0.5</precision></noise>",
    );
    let mut sim = load(&configured, 42);
    sim.step_exact(1).unwrap();
    let (time, value) = sample(&sim);
    assert_eq!(time, Duration::from_millis(1));
    assert_eq!(
        (
            value.vertical_position,
            value.vertical_velocity,
            value.vertical_reference
        ),
        (2.5, -2.5, 3.5)
    );
    for channel in [0, 1] {
        let mut one = load(
            &xml(
                if channel == 0 {
                    "<noise type='gaussian'><mean>3</mean></noise>"
                } else {
                    ""
                },
                if channel == 1 {
                    "<noise type='gaussian'><mean>3</mean></noise>"
                } else {
                    ""
                },
            ),
            42,
        );
        one.step_exact(1).unwrap();
        let reading = sample(&one).1;
        assert_eq!(
            [reading.vertical_position, reading.vertical_velocity],
            if channel == 0 { [3.0, 0.0] } else { [0.0, 3.0] }
        );
        assert_eq!(reading.vertical_reference, 3.5);
    }
    let mut ideal = load(XML, 42);
    let mut explicit = load(
        &xml(
            "<noise type='gaussian'/>",
            "<noise type='gaussian_quantized'/>",
        ),
        42,
    );
    ideal.step_exact(20).unwrap();
    explicit.step_exact(20).unwrap();
    assert_eq!(sample(&ideal), sample(&explicit));
}

#[test]
fn seeded_channels_match_ordered_stream_draws_and_altimeter_dynamic_bias_has_zero_elapsed_time() {
    let noise = "<noise type='gaussian'><mean>0.2</mean><stddev>0.3</stddev><bias_mean>0.4</bias_mean><bias_stddev>0.1</bias_stddev><dynamic_bias_stddev>8</dynamic_bias_stddev><dynamic_bias_correlation_time>0.2</dynamic_bias_correlation_time></noise>";
    let configured = xml(noise, noise).replace(
        "<update_rate>100</update_rate>",
        "<update_rate>10</update_rate>",
    );
    let mut sim = load(&configured, 42);
    let mut twin = load(&configured, 42);
    let mut other_seed = load(&configured, 43);
    let mut random = NoiseStream::new([42; 32], identity(&sim));
    let parameters = GaussianParameters {
        mean: 0.2,
        stddev: 0.3,
        bias_mean: 0.4,
        bias_stddev: 0.1,
        dynamic_bias_stddev: 8.0,
        dynamic_bias_correlation_time: 0.2,
        precision: 0.0,
    };
    let mut position = GaussianNoise::new(parameters, &mut random).unwrap();
    let mut velocity = GaussianNoise::new(parameters, &mut random).unwrap();
    let initial_biases = (position.bias(), velocity.bias());
    for steps in [1, 99, 100] {
        sim.step_exact(steps).unwrap();
        twin.step_exact(steps).unwrap();
        other_seed.step_exact(steps).unwrap();
        assert_eq!(sample(&sim), sample(&twin));
        let value = sample(&sim).1;
        assert_eq!(
            value.vertical_position,
            position.apply(0.0, 0.0, &mut random).unwrap()
        );
        assert_eq!(
            value.vertical_velocity,
            velocity.apply(0.0, 0.0, &mut random).unwrap()
        );
        assert_eq!(value.vertical_reference, 3.5);
        assert_eq!((position.bias(), velocity.bias()), initial_biases);
        assert_ne!(value, sample(&other_seed).1);
    }
    assert_eq!(sample(&sim).0, Duration::from_millis(200));
}

#[test]
fn no_demand_pause_and_non_due_steps_preserve_draws_and_both_reset_paths_replay() {
    let configured = xml(
        "<noise type='gaussian'><stddev>0.2</stddev><bias_stddev>0.3</bias_stddev></noise>",
        "<noise type='gaussian'><stddev>0.1</stddev></noise>",
    );
    let mut a = load(&configured, 7);
    let mut b = load(&configured, 7);
    let id = identity(&a);
    set_altimeter_interest(&mut a, []);
    a.step_exact(20).unwrap();
    let entity = entity_by_id(a.world(), id).unwrap();
    assert!(a.world().get::<AltimeterSample>(entity).is_none());
    set_altimeter_interest(&mut a, [entity]);
    a.step_exact(1).unwrap();
    b.step_exact(1).unwrap();
    assert_eq!(sample(&a).1, sample(&b).1);
    let first = sample(&a).1;
    a.set_paused(true);
    for _ in 0..5 {
        a.tick().unwrap();
    }
    assert_eq!(sample(&a), (Duration::from_millis(21), first));
    a.set_paused(false);
    a.step_exact(8).unwrap();
    assert_eq!(sample(&a), (Duration::from_millis(21), first));
    a.step_exact(1).unwrap();
    b.step_exact(9).unwrap();
    assert_eq!(sample(&a).1, sample(&b).1);
    clear_altimeter_interest(&mut a);
    a.reset().unwrap();
    assert!(a.world().get::<AltimeterSample>(entity).is_none());
    a.step_exact(1).unwrap();
    assert_eq!(sample(&a).1, first);
    reset_loaded_world(&mut a, |_| ()).unwrap();
    let restored = entity_by_id(a.world(), id).unwrap();
    assert_ne!(restored, entity);
    assert!(a.world().get::<AltimeterSample>(restored).is_none());
    a.step_exact(1).unwrap();
    assert_eq!(sample(&a).1, first);
}

#[test]
fn coexisting_noisy_imu_does_not_advance_the_altimeter_stream() {
    let configured = xml("<noise type='gaussian'><stddev>0.2</stddev></noise>", "");
    let coexist = configured.replace("</link></model>", "<sensor name='imu' type='imu'><update_rate>1000</update_rate><imu><angular_velocity><x><noise type='gaussian'><stddev>0.4</stddev></noise></x></angular_velocity></imu></sensor></link></model>");
    let mut alone = load(&configured, 42);
    let mut together = load(&coexist, 42);
    assert_eq!(identity(&alone), identity(&together));
    for _ in 0..30 {
        alone.step_exact(1).unwrap();
        together.step_exact(1).unwrap();
        assert_eq!(sample(&alone), sample(&together));
    }
}

#[test]
fn noisy_offset_point_velocity_and_load_reference_equal_ideal_physics_plus_channel_means() {
    let configured = xml(
        "<noise type='gaussian'><mean>2</mean></noise>",
        "<noise type='gaussian'><mean>-3</mean></noise>",
    );
    let mut ideal = load(XML, 0);
    let mut noisy = load(&configured, 0);
    for sim in [&mut ideal, &mut noisy] {
        let link = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::body"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        gz_sim_scene::queue_link_world_velocity(sim, link, [0.0, 0.0, 3.0], [0.0, 2.0, 0.0])
            .unwrap();
    }
    for _ in 0..20 {
        ideal.step_exact(1).unwrap();
        noisy.step_exact(1).unwrap();
        let base = sample(&ideal);
        let actual = sample(&noisy);
        assert_eq!(base.0, actual.0);
        assert!((actual.1.vertical_position - base.1.vertical_position - 2.0).abs() < 1e-12);
        assert!((actual.1.vertical_velocity - base.1.vertical_velocity + 3.0).abs() < 1e-12);
        assert_eq!(actual.1.vertical_reference, base.1.vertical_reference);
        assert_eq!(actual.1.vertical_reference, 3.5);
    }
}

#[test]
fn noisy_nonintegral_and_continuous_rates_keep_existing_phase_without_repeated_draws() {
    for (rate, expected) in [
        ("300", vec![1, 4, 7, 10]),
        ("0", (1..=10).collect()),
        ("-2", (1..=10).collect()),
    ] {
        let configured = xml("<noise type='gaussian'><stddev>0.1</stddev></noise>", "").replace(
            "<update_rate>100</update_rate>",
            &format!("<update_rate>{rate}</update_rate>"),
        );
        let mut sim = load(&configured, 4);
        let mut random = NoiseStream::new([4; 32], identity(&sim));
        let mut oracle = GaussianNoise::new(
            GaussianParameters {
                stddev: 0.1,
                ..Default::default()
            },
            &mut random,
        )
        .unwrap();
        let mut times = Vec::new();
        let mut last = Duration::ZERO;
        for _ in 0..10 {
            sim.step_exact(1).unwrap();
            let (time, reading) = sample(&sim);
            if time != last {
                times.push(time.as_millis() as u64);
                last = time;
                assert_eq!(
                    reading.vertical_position,
                    oracle.apply(0.0, 0.0, &mut random).unwrap()
                );
            }
        }
        assert_eq!(times, expected);
    }
}
