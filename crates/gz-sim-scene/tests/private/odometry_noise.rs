// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn metadata(
    dimensions: GroundTruthOdometryDimensions,
    sigma: f64,
) -> GroundTruthOdometryPublication {
    GroundTruthOdometryPublication {
        dimensions,
        offsets: GroundTruthOdometryOffsets::default(),
        gaussian_noise: GroundTruthOdometryNoise::from_stddev(sigma).unwrap(),
        model_name: "rig".into(),
        odom_frame: "rig/odom".into(),
        robot_base_frame: "rig/base".into(),
        odom_topic: "/odom".into(),
        covariance_topic: "/cov".into(),
        tf_topic: "/tf".into(),
    }
}
fn clock(ns: u64) -> UpdateInfo {
    UpdateInfo {
        start_time: Duration::ZERO,
        sim_time: Duration::from_nanos(ns),
        dt: Duration::ZERO,
        iteration: ns / 1_000_000,
        epoch: 7,
        paused: false,
    }
}
fn origin() -> WorldPose {
    WorldPose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    }
}
fn twist(s: GroundTruthOdometrySample) -> [u64; 6] {
    [
        s.linear[0],
        s.linear[1],
        s.linear[2],
        s.angular[0],
        s.angular[1],
        s.angular[2],
    ]
    .map(f64::to_bits)
}
fn next_draws(p: &Publisher) -> Option<[u64; 6]> {
    p.noise_stream.as_deref().cloned().map(|mut stream| {
        std::array::from_fn(|_| {
            stream
                .white_gaussian(p.publication.gaussian_noise.stddev())
                .unwrap()
                .to_bits()
        })
    })
}
fn same_state(a: &Publisher, b: &Publisher) {
    assert_eq!(a.previous_time, b.previous_time);
    assert_eq!(
        a.previous_pose.translation.map(f64::to_bits),
        b.previous_pose.translation.map(f64::to_bits)
    );
    assert_eq!(
        a.previous_pose.rotation.map(f64::to_bits),
        b.previous_pose.rotation.map(f64::to_bits)
    );
    assert_eq!(
        (a.last_publication, a.revision),
        (b.last_publication, b.revision)
    );
    for (a, b) in a.means.iter().zip(&b.means) {
        assert_eq!(a.count, b.count);
        assert_eq!(a.values.map(f64::to_bits), b.values.map(f64::to_bits));
    }
    match (&a.spatial_means, &b.spatial_means) {
        (Some(a), Some(b)) => {
            for (a, b) in a.iter().zip(b) {
                assert_eq!(a.count, b.count);
                assert_eq!(a.values.map(f64::to_bits), b.values.map(f64::to_bits));
            }
        }
        (None, None) => {}
        _ => panic!("history shape changed"),
    }
    assert_eq!(next_draws(a), next_draws(b));
}
#[test]
fn noise_configuration_zero_threshold_and_finite_float_variance_are_explicit() {
    let zero = GroundTruthOdometryNoise::default();
    assert_eq!(GroundTruthOdometryNoise::from_stddev(-0.0).unwrap(), zero);
    assert_eq!(zero.stddev().to_bits(), 0);
    for sigma in [
        f64::MIN_POSITIVE,
        f64::EPSILON,
        f64::from_bits(f64::EPSILON.to_bits() + 1),
        0.01,
        1.0,
        1e10,
    ] {
        let config = GroundTruthOdometryNoise::from_stddev(sigma).unwrap();
        assert_eq!(config.stddev().to_bits(), sigma.to_bits());
        assert_ne!(config, zero);
        assert_eq!(
            config.variance().to_bits(),
            ((sigma * sigma) as f32).to_bits()
        );
        let publisher = Publisher::seeded(
            7,
            metadata(GroundTruthOdometryDimensions::Two, sigma),
            Duration::ZERO,
            [47; 32],
        );
        assert_eq!(publisher.noise_stream.is_some(), sigma > f64::EPSILON);
    }
    for bad in [
        -0.01,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        f32::MAX as f64,
    ] {
        assert_eq!(
            GroundTruthOdometryNoise::from_stddev(bad),
            Err(GroundTruthOdometryError::Noise)
        );
    }
}
#[test]
fn white_noise_consumes_six_ordered_values_per_eligible_update_in_both_modes() {
    for dimension in [
        GroundTruthOdometryDimensions::Two,
        GroundTruthOdometryDimensions::Three,
    ] {
        let mut publisher =
            Publisher::seeded(7, metadata(dimension, 0.1), Duration::ZERO, [47; 32]);
        let before = next_draws(&publisher);
        publisher.advance(origin(), clock(0)).unwrap();
        assert_eq!(next_draws(&publisher), before);
        for ns in [0, 500, 1000] {
            assert!(publisher.advance(origin(), clock(ns)).unwrap().is_none());
            assert_eq!(next_draws(&publisher), before);
        }
        let mut expected = NoiseStream::new([47; 32], 7);
        for ns in [1_000_000, 2_000_000] {
            let draws: [f64; 6] = std::array::from_fn(|_| expected.white_gaussian(0.1).unwrap());
            let actual = publisher.advance(origin(), clock(ns)).unwrap().unwrap();
            let expected_twist = if dimension == GroundTruthOdometryDimensions::Two {
                [
                    0.0 + draws[0],
                    0.0 + draws[1],
                    draws[2],
                    draws[3],
                    draws[4],
                    0.0 + draws[5],
                ]
            } else {
                draws.map(|draw| 0.0 + draw)
            };
            assert_eq!(twist(actual), expected_twist.map(f64::to_bits));
            assert_eq!(actual.pose.translation, [0.0; 3]);
            assert_eq!(actual.pose.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        let follow: [u64; 6] =
            std::array::from_fn(|_| expected.white_gaussian(0.1).unwrap().to_bits());
        assert_eq!(next_draws(&publisher), Some(follow));
    }
}
#[test]
fn throttled_updates_commit_rng_and_means_while_equal_time_and_pause_do_not_draw() {
    for dimension in [
        GroundTruthOdometryDimensions::Two,
        GroundTruthOdometryDimensions::Three,
    ] {
        let config = metadata(dimension, 0.1);
        let mut limited = Publisher::seeded(7, config.clone(), Duration::from_millis(20), [47; 32]);
        let mut every = Publisher::seeded(7, config, Duration::ZERO, [47; 32]);
        limited.advance(origin(), clock(0)).unwrap();
        every.advance(origin(), clock(0)).unwrap();
        for tick in 1..=20 {
            let mut pose = origin();
            pose.translation[0] = tick as f64 * 0.001;
            let all = every
                .advance(pose, clock(tick * 1_000_000))
                .unwrap()
                .unwrap();
            let due = limited.advance(pose, clock(tick * 1_000_000)).unwrap();
            assert_eq!(next_draws(&limited), next_draws(&every));
            if tick < 20 {
                assert!(due.is_none());
            } else {
                assert_eq!(twist(due.unwrap()), twist(all));
            }
        }
        let held = limited.clone();
        assert!(
            limited
                .advance(origin(), clock(20_000_000))
                .unwrap()
                .is_none()
        );
        same_state(&limited, &held);
        let mut world = World::new();
        world.insert_resource(StepFault::default());
        let entity = world
            .spawn((EntityKind::Model, limited.publication.clone(), origin()))
            .id();
        let owner = gz_sim_core::assign_id(&mut world, entity).unwrap();
        limited.owner = owner;
        world.entity_mut(entity).insert(limited.clone());
        let mut paused = clock(21_000_000);
        paused.paused = true;
        world.insert_resource(paused);
        sample(&mut world);
        same_state(world.get::<Publisher>(entity).unwrap(), &limited);
        assert!(world.resource::<StepFault>().error().is_none());
    }
}
#[test]
fn failed_noisy_candidates_rollback_rng_filters_clocks_and_revision() {
    for dimension in [
        GroundTruthOdometryDimensions::Two,
        GroundTruthOdometryDimensions::Three,
    ] {
        let mut publisher =
            Publisher::seeded(7, metadata(dimension, 0.1), Duration::ZERO, [47; 32]);
        publisher.advance(origin(), clock(0)).unwrap();
        let initial = publisher.clone();
        let mut bad = origin();
        bad.translation[0] = f64::NAN;
        assert_eq!(
            publisher.advance(bad, clock(1_000_000)).unwrap_err(),
            GroundTruthOdometryError::Pose
        );
        same_state(&publisher, &initial);
        publisher.previous_time = Some(Duration::from_millis(2));
        let before = publisher.clone();
        assert_eq!(
            publisher.advance(origin(), clock(1_000_000)).unwrap_err(),
            GroundTruthOdometryError::Time
        );
        same_state(&publisher, &before);
        publisher.previous_time = Some(Duration::ZERO);
        publisher.means[0].count = 10;
        publisher.means[0].values = [f64::MAX; 10];
        let before = publisher.clone();
        assert_eq!(
            publisher.advance(origin(), clock(1_000_000)).unwrap_err(),
            GroundTruthOdometryError::Arithmetic
        );
        same_state(&publisher, &before);
        publisher.means = std::array::from_fn(|_| Mean::default());
        // This error occurs AFTER all six random samples, proving rollback of
        // consumed candidate randomness, not just a pre-draw validation gate.
        publisher.revision = u64::MAX;
        let before = publisher.clone();
        assert_eq!(
            publisher.advance(origin(), clock(1_000_000)).unwrap_err(),
            GroundTruthOdometryError::Revision
        );
        same_state(&publisher, &before);
    }
}
#[test]
fn seeded_owner_streams_replay_independently_and_preserve_upstream_moment_bounds() {
    for dimension in [
        GroundTruthOdometryDimensions::Two,
        GroundTruthOdometryDimensions::Three,
    ] {
        let config = metadata(dimension, 1.0);
        let mut a = Publisher::seeded(7, config.clone(), Duration::ZERO, [47; 32]);
        let mut b = a.clone();
        let mut other = Publisher::seeded(8, config, Duration::ZERO, [47; 32]);
        a.advance(origin(), clock(0)).unwrap();
        b.advance(origin(), clock(0)).unwrap();
        other.advance(origin(), clock(0)).unwrap();
        let mut sum = [0.0; 6];
        let mut square = [0.0; 6];
        let mut diverged = false;
        for tick in 1..=500 {
            let row = a
                .advance(origin(), clock(tick * 1_000_000))
                .unwrap()
                .unwrap();
            let interleaved = other
                .advance(origin(), clock(tick * 1_000_000))
                .unwrap()
                .unwrap();
            let replay = b
                .advance(origin(), clock(tick * 1_000_000))
                .unwrap()
                .unwrap();
            assert_eq!(twist(row), twist(replay));
            diverged |= twist(row) != twist(interleaved);
            let values = [
                row.linear[0],
                row.linear[1],
                row.linear[2],
                row.angular[0],
                row.angular[1],
                row.angular[2],
            ];
            for i in 0..6 {
                sum[i] += values[i];
                square[i] += values[i] * values[i];
            }
        }
        assert!(diverged);
        for i in 0..6 {
            let mean = sum[i] / 500.0;
            let variance = square[i] / 500.0 - mean * mean;
            // Unchanged six-channel upstream GaussianNoiseTest population bounds.
            assert!(mean.abs() < 0.5);
            assert!((variance - 1.0).abs() < 0.5);
        }
    }
}
#[test]
fn tiny_and_exact_zero_paths_need_no_stream_and_bare_reset_only_clears_sample() {
    for dimension in [
        GroundTruthOdometryDimensions::Two,
        GroundTruthOdometryDimensions::Three,
    ] {
        for sigma in [0.0, -0.0, f64::MIN_POSITIVE, f64::EPSILON] {
            let mut publisher =
                Publisher::seeded(7, metadata(dimension, sigma), Duration::ZERO, [47; 32]);
            let mut original =
                Publisher::seeded(7, metadata(dimension, 0.0), Duration::ZERO, [1; 32]);
            assert!(publisher.noise_stream.is_none());
            publisher.advance(origin(), clock(0)).unwrap();
            original.advance(origin(), clock(0)).unwrap();
            for tick in 1..=20 {
                let a = publisher
                    .advance(origin(), clock(tick * 1_000_000))
                    .unwrap()
                    .unwrap();
                let b = original
                    .advance_noiseless(origin(), clock(tick * 1_000_000))
                    .unwrap()
                    .unwrap();
                assert_eq!(twist(a), twist(b));
            }
        }
        let mut world = World::new();
        let config = metadata(dimension, 0.1);
        let entity = world
            .spawn((EntityKind::Model, config.clone(), origin()))
            .id();
        let owner = gz_sim_core::assign_id(&mut world, entity).unwrap();
        let mut publisher = Publisher::seeded(owner, config, Duration::ZERO, [47; 32]);
        publisher.advance(origin(), clock(0)).unwrap();
        let value = publisher
            .advance(origin(), clock(1_000_000))
            .unwrap()
            .unwrap();
        let held = publisher.clone();
        world.entity_mut(entity).insert((publisher, value));
        // Run the exact registered Reset system; do not pretend it reconstructs
        // the loaded blueprint or seed/history. Loaded reset is tested publicly.
        let mut schedule = bevy_ecs::schedule::Schedule::default();
        schedule.add_systems(reset);
        schedule.run(&mut world);
        assert!(world.get::<GroundTruthOdometrySample>(entity).is_none());
        same_state(world.get::<Publisher>(entity).unwrap(), &held);
    }
}
