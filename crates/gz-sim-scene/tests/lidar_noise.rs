// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, NoiseStream};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    LidarSample, Name, PhysicsEngine, load_world_seeded, reset_loaded_world, set_lidar_interest,
};
use sdformat::NoResources;
use std::time::Duration;

const XML: &str = include_str!("fixtures/planar_lidar.sdf");
fn xml(noise: &str) -> String {
    XML.replace("<min>0.1</min>", "<min>0</min>")
        .replace("</range></lidar>", &format!("</range>{noise}</lidar>"))
}
fn load(xml: &str, seed: [u8; 32]) -> Simulation {
    load_world_seeded(
        xml,
        "lidar-noise.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
    .unwrap()
}
fn sensor(sim: &Simulation) -> (bevy_ecs::entity::Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::scanner::body::laser")
        })
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn scan(sim: &Simulation) -> (Duration, Vec<f64>) {
    let sample = sim.world().get::<LidarSample>(sensor(sim).0).unwrap();
    assert_eq!(sample.epoch, sim.epoch());
    assert_eq!(sample.world_pose.translation, [0.0; 3]);
    (sample.time, sample.ranges.clone())
}

#[test]
fn signed_means_clip_only_after_noise_on_real_analytic_rapier_hits() {
    let ideal = [4.0 * 2.0_f64.sqrt(), 4.0, 4.0 * 2.0_f64.sqrt()];
    for mean in [-20.0, -1.25, 1.25, 20.0] {
        let mut sim = load(
            &xml(&format!(
                "<noise><type>gaussian</type><mean>{mean}</mean></noise>"
            )),
            [42; 32],
        );
        sim.step_exact(1).unwrap();
        let actual = scan(&sim);
        assert_eq!(actual.0, Duration::from_millis(1));
        for (actual, ideal) in actual.1.iter().zip(ideal) {
            assert!((actual - (ideal + mean).clamp(0.0, 10.0)).abs() < 1e-10);
        }
    }
}

#[test]
fn cpu_activation_is_inclusive_and_legacy_noiseless_nonzero_minimum_is_preserved() {
    let mut baseline = load(&xml(""), [42; 32]);
    baseline.step_exact(1).unwrap();
    for mean in [-1e-6, 0.0, 1e-6] {
        let mut sim = load(
            &xml(&format!(
                "<noise><type>gaussian</type><mean>{mean}</mean><stddev>0.000001</stddev></noise>"
            )),
            [42; 32],
        );
        sim.step_exact(1).unwrap();
        assert_eq!(scan(&sim), scan(&baseline));
    }
    let mean = f64::from_bits(1e-6_f64.to_bits() + 1);
    let mut active = load(
        &xml(&format!(
            "<noise><type>gaussian</type><mean>{mean:.17e}</mean></noise>"
        )),
        [42; 32],
    );
    active.step_exact(1).unwrap();
    assert_ne!(scan(&active).1, scan(&baseline).1);
    for (actual, ideal) in scan(&active).1.iter().zip(scan(&baseline).1) {
        assert_eq!(*actual, ideal + mean);
    }
    let mut old = load(&XML.replace("<min>0.1</min>", "<min>6</min>"), [42; 32]);
    old.step_exact(1).unwrap();
    assert_eq!(scan(&old).1, [6.0; 3]);
}

#[test]
fn finite_beam_order_uses_one_sensor_stream_and_misses_consume_no_draws() {
    let configured =
        xml("<noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise>")
            .replace("<size>2 20 2</size>", "<size>2 1 2</size>");
    // Narrow wall: oblique beams miss, centre beam has the exact 4m hit.
    let mut sim = load(&configured, [42; 32]);
    let (entity, identity) = sensor(&sim);
    let mut stream = NoiseStream::new([42; 32], identity);
    let mut oracle = GaussianNoise::new(
        GaussianParameters {
            mean: 0.1,
            stddev: 0.2,
            ..Default::default()
        },
        &mut stream,
    )
    .unwrap();
    set_lidar_interest(&mut sim, []);
    sim.step_exact(20).unwrap();
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    set_lidar_interest(&mut sim, [entity]);
    for (steps, time) in [(1, 21), (9, 30)] {
        sim.step_exact(steps).unwrap();
        let (actual_time, ranges) = scan(&sim);
        assert_eq!(actual_time, Duration::from_millis(time));
        assert_eq!(ranges[0], f64::INFINITY);
        assert_eq!(ranges[2], f64::INFINITY);
        assert_eq!(
            ranges[1],
            oracle
                .apply(4.0, 0.0, &mut stream)
                .unwrap()
                .clamp(0.0, 10.0)
        );
    }
    let held = scan(&sim);
    sim.set_paused(true);
    for _ in 0..10 {
        sim.tick().unwrap();
    }
    assert_eq!(scan(&sim), held);
    sim.set_paused(false);
    sim.step_exact(9).unwrap();
    assert_eq!(scan(&sim), held);
    set_lidar_interest(&mut sim, []);
    sim.step_exact(41).unwrap();
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    set_lidar_interest(&mut sim, [entity]);
    sim.step_exact(1).unwrap();
    assert_eq!(scan(&sim).0, Duration::from_millis(81));
    assert_eq!(
        scan(&sim).1[1],
        oracle
            .apply(4.0, 0.0, &mut stream)
            .unwrap()
            .clamp(0.0, 10.0)
    );
}

#[test]
fn seeded_scans_replay_with_reset_and_removal_and_do_not_share_other_sensor_rng() {
    let configured =
        xml("<noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise>");
    let with_imu = configured.replace("</sensor>", "</sensor><sensor name='imu' type='imu'><always_on>true</always_on><imu><angular_velocity><x><noise type='gaussian'><stddev>2</stddev></noise></x></angular_velocity></imu></sensor>");
    let mut a = load(&configured, [42; 32]);
    let mut b = load(&configured, [42; 32]);
    let mut c = load(&with_imu, [42; 32]);
    let mut different_seed = load(&configured, [43; 32]);
    let identity = sensor(&a).1;
    let counts = a.scheduled_system_counts();
    assert_eq!(identity, sensor(&c).1);
    let mut first = None;
    let mut different = false;
    for _ in 0..100 {
        for sim in [&mut a, &mut b, &mut c, &mut different_seed] {
            sim.step_exact(1).unwrap();
        }
        assert_eq!(scan(&a), scan(&b));
        assert_eq!(scan(&a), scan(&c));
        different |= scan(&a) != scan(&different_seed);
        first.get_or_insert_with(|| scan(&a));
    }
    assert!(different);
    a.reset().unwrap();
    assert!(a.world().get::<LidarSample>(sensor(&a).0).is_none());
    a.step_exact(1).unwrap();
    assert_eq!(scan(&a), *first.as_ref().unwrap());
    let model = a
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::scanner")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut a, model).unwrap();
    assert!(entity_by_id(a.world(), identity).is_none());
    assert_eq!(a.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    reset_loaded_world(&mut a, |_| ()).unwrap();
    assert_eq!(sensor(&a).1, identity);
    assert_eq!(a.scheduled_system_counts(), counts);
    a.step_exact(1).unwrap();
    assert_eq!(scan(&a), first.unwrap());
}

#[test]
fn gaussian_nonzero_minimum_hidden_fields_and_quantized_noise_are_explicitly_refused() {
    for noise in [
        "<noise><type>gaussian</type></noise>",
        "<noise><type>gaussian</type><mean>0.000001</mean></noise>",
        "<noise><type>gaussian</type><stddev>0.2</stddev></noise>",
    ] {
        let nonzero = XML.replace("</range></lidar>", &format!("</range>{noise}</lidar>"));
        let parsed =
            sdformat::load_str(&nonzero, "parsed-nonzero-noisy-min", &NoResources).unwrap();
        assert_eq!(
            parsed.worlds[0].models[0].links[0].sensors[0]
                .lidar()
                .unwrap()
                .noise
                .kind,
            sdformat::NoiseKind::Gaussian
        );
        assert!(matches!(
            load_world_seeded(
                &nonzero,
                "nonzero-noisy-min",
                "default",
                Some(Duration::from_millis(1)),
                &NoResources,
                PhysicsEngine::Rapier,
                [42; 32]
            ),
            Err(gz_sim_scene::LoadError::Unsupported {
                feature: "Gaussian CPU lidar requires zero minimum range",
                ..
            })
        ));
    }
    for (body, kind) in [
        (
            "<noise><type>gaussian_quantized</type></noise>",
            sdformat::NoiseKind::GaussianQuantized,
        ),
        (
            "<noise><type>none</type><mean>1</mean></noise>",
            sdformat::NoiseKind::None,
        ),
    ] {
        let configured = xml(body);
        let parsed = sdformat::load_str(&configured, "parsed-lidar-noise", &NoResources).unwrap();
        assert_eq!(
            parsed.worlds[0].models[0].links[0].sensors[0]
                .lidar()
                .unwrap()
                .noise
                .kind,
            kind
        );
        assert!(matches!(
            load_world_seeded(
                &configured,
                "unsupported-lidar-noise",
                "default",
                Some(Duration::from_millis(1)),
                &NoResources,
                PhysicsEngine::Rapier,
                [42; 32]
            ),
            Err(gz_sim_scene::LoadError::Unsupported {
                feature: "unsupported CPU lidar noise",
                ..
            })
        ));
    }
    // The maintained lidar parser admits only type/mean/stddev. Hidden fields
    // must fail there; typed private controls independently exercise Scene's guard.
    for body in [
        "<noise><type>gaussian</type><bias_mean>1</bias_mean></noise>",
        "<noise><type>gaussian</type><bias_stddev>1</bias_stddev></noise>",
        "<noise><type>gaussian</type><dynamic_bias_stddev>1</dynamic_bias_stddev></noise>",
        "<noise><type>gaussian</type><dynamic_bias_correlation_time>1</dynamic_bias_correlation_time></noise>",
        "<noise><type>gaussian</type><precision>0.1</precision></noise>",
    ] {
        let configured = xml(body);
        assert!(
            sdformat::load_str(&configured, "unsupported-lidar-noise-field", &NoResources).is_err()
        );
    }
}
