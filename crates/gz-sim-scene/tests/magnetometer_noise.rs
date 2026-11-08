// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, NoiseStream};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    MagnetometerSample, Name, PhysicsEngine, WorldPose, clear_magnetometer_interest,
    load_world_seeded, reset_loaded_world, set_magnetometer_interest,
};
use sdformat::NoResources;
use std::time::Duration;

const XML: &str = include_str!("fixtures/m17_magnetometer.sdf");
fn xml(channels: [&str; 3]) -> String {
    XML.replace(
        "<magnetometer/>",
        &format!(
            "<magnetometer><x>{}</x><y>{}</y><z>{}</z></magnetometer>",
            channels[0], channels[1], channels[2]
        ),
    )
}
fn load(xml: &str, seed: u8) -> Simulation {
    load_world_seeded(
        xml,
        "m23-noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        PhysicsEngine::Rapier,
        [seed; 32],
    )
    .unwrap()
}
fn identity(sim: &Simulation) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rig::body::magnetometer")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn sample(sim: &Simulation) -> (Duration, [f64; 3]) {
    let entity = entity_by_id(sim.world(), identity(sim)).unwrap();
    let sample = sim.world().get::<MagnetometerSample>(entity).unwrap();
    (sample.time, sample.reading.as_ref().unwrap().field_tesla)
}
// Independent transpose of the sensor-to-world rotation, including its resolved link pose.
fn rotated(sim: &Simulation, field: [f64; 3]) -> [f64; 3] {
    let entity = entity_by_id(sim.world(), identity(sim)).unwrap();
    let [x, y, z, w] = sim.world().get::<WorldPose>(entity).unwrap().rotation;
    let [a, b, c] = field;
    [
        (1.0 - 2.0 * (y * y + z * z)) * a + 2.0 * (x * y + z * w) * b + 2.0 * (x * z - y * w) * c,
        2.0 * (x * y - z * w) * a + (1.0 - 2.0 * (x * x + z * z)) * b + 2.0 * (y * z + x * w) * c,
        2.0 * (x * z + y * w) * a + 2.0 * (y * z - x * w) * b + (1.0 - 2.0 * (x * x + y * y)) * c,
    ]
}

#[test]
fn distinct_axis_means_are_added_after_rotation_and_single_channels_and_defaults_preserve_ideal_axes()
 {
    let channels = [
        "<noise type='gaussian'><mean>0.00001</mean></noise>",
        "<noise type='gaussian'><mean>-0.00002</mean></noise>",
        "<noise type='gaussian'><mean>0.00003</mean></noise>",
    ];
    let field = [2e-5, -3e-5, 4e-5];
    let means = [1e-5, -2e-5, 3e-5];
    let mut sim = load(&xml(channels), 42);
    sim.step_exact(1).unwrap();
    let base = rotated(&sim, field);
    let actual = sample(&sim).1;
    for axis in 0..3 {
        assert!((actual[axis] - base[axis] - means[axis]).abs() < 1e-18);
    }
    let wrong = rotated(&sim, std::array::from_fn(|i| field[i] + means[i]));
    assert!(
        actual
            .into_iter()
            .zip(wrong)
            .any(|(a, b)| (a - b).abs() > 1e-6)
    );
    for enabled in 0..3 {
        let mut selected = [""; 3];
        selected[enabled] = channels[enabled];
        let mut one = load(&xml(selected), 42);
        one.step_exact(1).unwrap();
        let base = rotated(&one, field);
        let actual = sample(&one).1;
        for axis in 0..3 {
            assert!(
                (actual[axis] - base[axis] - if axis == enabled { means[axis] } else { 0.0 }).abs()
                    < 1e-18
            );
        }
    }
    let mut ideal = load(XML, 42);
    let mut defaults = load(
        &xml([
            "<noise type='gaussian'/>",
            "<noise type='gaussian_quantized'/>",
            "",
        ]),
        42,
    );
    ideal.step_exact(20).unwrap();
    defaults.step_exact(20).unwrap();
    assert_eq!(sample(&ideal), sample(&defaults));
}

#[test]
fn both_gaussian_kinds_use_signed_rounding_after_the_current_rotated_field() {
    for kind in ["gaussian", "gaussian_quantized"] {
        let channels = [
            format!("<noise type='{kind}'><mean>2.25</mean><precision>0.5</precision></noise>"),
            format!("<noise type='{kind}'><mean>-2.25</mean><precision>0.5</precision></noise>"),
            format!("<noise type='{kind}'><mean>0.32</mean><precision>0.3</precision></noise>"),
        ];
        let mut sim = load(&xml([&channels[0], &channels[1], &channels[2]]), 42);
        sim.step_exact(1).unwrap();
        let base = rotated(&sim, [2e-5, -3e-5, 4e-5]);
        let actual = sample(&sim).1;
        for (axis, (mean, precision)) in [(2.25, 0.5), (-2.25, 0.5), (0.32, 0.3)]
            .into_iter()
            .enumerate()
        {
            assert_eq!(
                actual[axis],
                ((base[axis] + mean) / precision).round() * precision
            );
        }
    }
}

#[test]
fn xyz_draw_order_matches_seeded_stream_and_dynamic_bias_does_not_evolve_at_slow_rates() {
    let noise = "<noise type='gaussian'><mean>0.2</mean><stddev>0.3</stddev><bias_mean>0.4</bias_mean><bias_stddev>0.1</bias_stddev><dynamic_bias_stddev>8</dynamic_bias_stddev><dynamic_bias_correlation_time>0.2</dynamic_bias_correlation_time></noise>";
    let configured = xml([noise; 3]).replace(
        "<update_rate>100</update_rate>",
        "<update_rate>10</update_rate>",
    );
    let mut a = load(&configured, 42);
    let mut twin = load(&configured, 42);
    let mut different = load(&configured, 43);
    let mut random = NoiseStream::new([42; 32], identity(&a));
    let p = GaussianParameters {
        mean: 0.2,
        stddev: 0.3,
        bias_mean: 0.4,
        bias_stddev: 0.1,
        dynamic_bias_stddev: 8.0,
        dynamic_bias_correlation_time: 0.2,
        precision: 0.0,
    };
    let mut channels: [GaussianNoise; 3] =
        std::array::from_fn(|_| GaussianNoise::new(p, &mut random).unwrap());
    let initial_biases = channels.each_ref().map(GaussianNoise::bias);
    for steps in [1, 99, 100] {
        a.step_exact(steps).unwrap();
        twin.step_exact(steps).unwrap();
        different.step_exact(steps).unwrap();
        assert_eq!(sample(&a), sample(&twin));
        assert_ne!(sample(&a).1, sample(&different).1);
        let base = rotated(&a, [2e-5, -3e-5, 4e-5]);
        let actual = sample(&a).1;
        for axis in 0..3 {
            assert!(
                (actual[axis] - channels[axis].apply(base[axis], 0.0, &mut random).unwrap()).abs()
                    < 1e-15
            );
        }
        assert_eq!(channels.each_ref().map(GaussianNoise::bias), initial_biases);
    }
    assert_eq!(sample(&a).0, Duration::from_millis(200));
}

#[test]
fn demand_pause_non_due_steps_and_both_resets_preserve_the_owned_seeded_stream() {
    let noise = "<noise type='gaussian'><stddev>0.2</stddev><bias_stddev>0.1</bias_stddev></noise>";
    let configured = xml([noise; 3]);
    let mut a = load(&configured, 7);
    let mut b = load(&configured, 7);
    let id = identity(&a);
    let old = entity_by_id(a.world(), id).unwrap();
    set_magnetometer_interest(&mut a, []);
    a.step_exact(20).unwrap();
    assert!(a.world().get::<MagnetometerSample>(old).is_none());
    set_magnetometer_interest(&mut a, [old]);
    a.step_exact(1).unwrap();
    b.step_exact(1).unwrap();
    let first = sample(&a).1;
    assert_eq!(first, sample(&b).1);
    a.set_paused(true);
    for _ in 0..3 {
        a.tick().unwrap();
    }
    assert_eq!(sample(&a), (Duration::from_millis(21), first));
    a.set_paused(false);
    a.step_exact(8).unwrap();
    assert_eq!(sample(&a).1, first);
    a.step_exact(1).unwrap();
    b.step_exact(9).unwrap();
    assert_eq!(sample(&a).1, sample(&b).1);
    clear_magnetometer_interest(&mut a);
    a.reset().unwrap();
    assert!(a.world().get::<MagnetometerSample>(old).is_none());
    a.step_exact(1).unwrap();
    assert_eq!(sample(&a).1, first);
    reset_loaded_world(&mut a, |_| ()).unwrap();
    let restored = entity_by_id(a.world(), id).unwrap();
    assert_ne!(restored, old);
    assert!(a.world().get::<MagnetometerSample>(restored).is_none());
    a.step_exact(1).unwrap();
    assert_eq!(sample(&a).1, first);
}

#[test]
fn coexisting_imu_and_actual_rotating_link_do_not_reassign_magnetic_noise_to_world_axes() {
    let noise = "<noise type='gaussian'><mean>0.00001</mean></noise>";
    let configured = xml([noise; 3]);
    let coexist=configured.replace("</link></model>","<sensor name='imu' type='imu'><update_rate>1000</update_rate><imu><angular_velocity><x><noise type='gaussian'><stddev>0.4</stddev></noise></x></angular_velocity></imu></sensor></link></model>");
    let mut a = load(&configured, 42);
    let mut b = load(&coexist, 42);
    assert_eq!(identity(&a), identity(&b));
    for sim in [&mut a, &mut b] {
        let link = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::body"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        gz_sim_scene::queue_link_world_velocity(sim, link, [2.0, -3.0, 4.0], [0.0, 2.0, 0.0])
            .unwrap();
    }
    let mut first = None;
    let mut held = None;
    let mut due_times = Vec::new();
    let mut orientations = Vec::new();
    for _ in 0..20 {
        a.step_exact(1).unwrap();
        b.step_exact(1).unwrap();
        let acquired = sample(&a);
        assert_eq!(acquired, sample(&b));
        if held.is_some_and(|previous: (Duration, [f64; 3])| previous.0 == acquired.0) {
            assert_eq!(
                Some(acquired),
                held,
                "non-due acquisition retains the entire time/value pair"
            );
        } else {
            let actual = acquired.1;
            let base = rotated(&a, [2e-5, -3e-5, 4e-5]);
            for axis in 0..3 {
                assert!((actual[axis] - base[axis] - 1e-5).abs() < 1e-18);
            }
            first.get_or_insert(actual);
            due_times.push(acquired.0);
            let entity = entity_by_id(a.world(), identity(&a)).unwrap();
            orientations.push(a.world().get::<WorldPose>(entity).unwrap().rotation);
        }
        held = Some(acquired);
    }
    assert_eq!(
        due_times,
        [
            Duration::from_millis(1),
            Duration::from_millis(10),
            Duration::from_millis(20)
        ]
    );
    assert!(
        orientations[0]
            .into_iter()
            .zip(*orientations.last().unwrap())
            .any(|(a, b)| (a - b).abs() > 1e-8)
    );
    assert!(
        sample(&a)
            .1
            .into_iter()
            .zip(first.unwrap())
            .any(|(a, b)| (a - b).abs() > 1e-8)
    );
}

#[test]
fn other_noisy_sensor_acquisitions_do_not_consume_magnetometer_random_draws() {
    let noise = "<noise type='gaussian'><stddev>0.0000002</stddev><bias_stddev>0.0000001</bias_stddev></noise>";
    let configured = xml([noise; 3]);
    let coexist=configured.replace("</link></model>","<sensor name='imu' type='imu'><update_rate>1000</update_rate><imu><angular_velocity><x><noise type='gaussian'><stddev>0.4</stddev></noise></x></angular_velocity></imu></sensor></link></model>");
    let mut a = load(&configured, 42);
    let mut b = load(&coexist, 42);
    assert_eq!(identity(&a), identity(&b));
    for _ in 0..30 {
        a.step_exact(1).unwrap();
        b.step_exact(1).unwrap();
        assert_eq!(sample(&a), sample(&b));
    }
}

#[test]
fn nonintegral_and_continuous_rates_draw_once_per_actual_magnetic_acquisition() {
    let noise = "<noise type='gaussian'><stddev>0.1</stddev></noise>";
    for (rate, expected) in [
        ("300", vec![1, 4, 7, 10]),
        ("0", (1..=10).collect()),
        ("-2", (1..=10).collect()),
    ] {
        let configured = xml([noise, "", ""]).replace(
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
            let (time, actual) = sample(&sim);
            if time != last {
                last = time;
                times.push(time.as_millis() as u64);
                assert!(
                    (actual[0]
                        - oracle
                            .apply(rotated(&sim, [2e-5, -3e-5, 4e-5])[0], 0.0, &mut random)
                            .unwrap())
                    .abs()
                        < 1e-15
                );
            }
        }
        assert_eq!(times, expected);
    }
}
