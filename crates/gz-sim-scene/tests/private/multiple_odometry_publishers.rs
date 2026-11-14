// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn configuration(owner: usize) -> GroundTruthOdometryPublication {
    GroundTruthOdometryPublication {
        dimensions: if owner.is_multiple_of(2) {
            GroundTruthOdometryDimensions::Two
        } else {
            GroundTruthOdometryDimensions::Three
        },
        offsets: GroundTruthOdometryOffsets::default(),
        gaussian_noise: GroundTruthOdometryNoise::from_stddev(if owner < 2 { 0.0 } else { 0.05 })
            .unwrap(),
        model_name: format!("rig{owner}"),
        odom_frame: format!("rig{owner}/odom"),
        robot_base_frame: format!("rig{owner}/base"),
        odom_topic: format!("/m56/{owner}/odom"),
        covariance_topic: format!("/m56/{owner}/cov"),
        tf_topic: format!("/m56/{owner}/tf"),
    }
}
fn origin() -> WorldPose {
    WorldPose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    }
}
type ReadingBits = ([u64; 3], [u64; 4], [u64; 3], [u64; 3], Duration, u64, u64);
fn bits(value: GroundTruthOdometrySample) -> ReadingBits {
    (
        value.pose.translation.map(f64::to_bits),
        value.pose.rotation.map(f64::to_bits),
        value.linear.map(f64::to_bits),
        value.angular.map(f64::to_bits),
        value.time,
        value.epoch,
        value.revision,
    )
}
fn future_draws(publisher: &Publisher) -> Option<[u64; 6]> {
    publisher
        .noise_stream
        .as_deref()
        .cloned()
        .map(|mut stream| {
            std::array::from_fn(|_| {
                stream
                    .white_gaussian(publisher.publication.gaussian_noise.stddev())
                    .unwrap()
                    .to_bits()
            })
        })
}
fn same_history(actual: &Publisher, expected: &Publisher) {
    assert_eq!(actual.previous_time, expected.previous_time);
    assert_eq!(
        actual.previous_pose.translation.map(f64::to_bits),
        expected.previous_pose.translation.map(f64::to_bits)
    );
    assert_eq!(
        actual.previous_pose.rotation.map(f64::to_bits),
        expected.previous_pose.rotation.map(f64::to_bits)
    );
    assert_eq!(
        (actual.last_publication, actual.revision),
        (expected.last_publication, expected.revision)
    );
    for (a, b) in actual.means.iter().zip(&expected.means) {
        assert_eq!(a.count, b.count);
        assert_eq!(a.values.map(f64::to_bits), b.values.map(f64::to_bits));
    }
    match (&actual.spatial_means, &expected.spatial_means) {
        (Some(a), Some(b)) => {
            for (a, b) in a.iter().zip(b) {
                assert_eq!(a.count, b.count);
                assert_eq!(a.values.map(f64::to_bits), b.values.map(f64::to_bits));
            }
        }
        (None, None) => {}
        _ => panic!("owner history dimensions changed"),
    }
    assert_eq!(future_draws(actual), future_draws(expected));
}

#[test]
fn owner_local_dispatch_preserves_every_history_rng_and_deadline_against_single_owner_calls() {
    let mut world = World::new();
    world.insert_resource(StepFault::default());
    let mut owners = Vec::new();
    let mut references = Vec::new();
    let mut held = vec![None; 4];
    for index in 0..4 {
        let config = configuration(index);
        let entity = world
            .spawn((EntityKind::Model, config.clone(), origin()))
            .id();
        let id = gz_sim_core::assign_id(&mut world, entity).unwrap();
        let publisher = Publisher::seeded(
            id,
            config,
            if index.is_multiple_of(2) {
                Duration::ZERO
            } else {
                Duration::from_millis(20)
            },
            [56; 32],
        );
        references.push(publisher.clone());
        world.entity_mut(entity).insert(publisher);
        owners.push(entity);
    }
    for (iteration, (ns, paused)) in [
        (0, false),
        (1_000, false),
        (2_000, false),
        (1_000_000, false),
        (2_000_000, true),
        (2_000_000, false),
        (2_000_000, false),
        (5_000_000, false),
        (10_000_000, false),
        (20_000_000, false),
        (40_000_000, false),
    ]
    .into_iter()
    .enumerate()
    {
        let info = UpdateInfo {
            start_time: Duration::ZERO,
            sim_time: Duration::from_nanos(ns),
            dt: Duration::ZERO,
            iteration: iteration as u64,
            epoch: 3,
            paused,
        };
        // Controlled private-dispatch inputs, not a loaded Physics trajectory.
        for (index, entity) in owners.iter().enumerate() {
            let pose = WorldPose {
                translation: [ns as f64 * 1e-9, index as f64 * 0.1, 0.2],
                rotation: [0.0, 0.0, 0.0, 1.0],
            };
            world.entity_mut(*entity).insert(pose);
            if !paused && let Some(reading) = references[index].advance(pose, info).unwrap() {
                held[index] = Some(bits(reading));
            }
        }
        world.insert_resource(info);
        sample(&mut world);
        assert!(world.resource::<StepFault>().error().is_none());
        for (index, entity) in owners.iter().enumerate() {
            same_history(world.get::<Publisher>(*entity).unwrap(), &references[index]);
            assert_eq!(
                world
                    .get::<GroundTruthOdometrySample>(*entity)
                    .copied()
                    .map(bits),
                held[index]
            );
        }
    }
    assert!(
        references
            .iter()
            .all(|publisher| publisher.means[0].count > 1)
    );
    assert_eq!(references[1].last_publication, Duration::from_millis(40));
    assert_eq!(references[3].last_publication, Duration::from_millis(40));
}

#[test]
fn a_bad_owner_reports_global_terminal_identity_time_and_revision_without_peer_rollback_claims() {
    for case in 0..3 {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        let mut owners = Vec::new();
        for index in 0..2 {
            let config = configuration(index);
            let entity = sim
                .world_mut()
                .spawn((EntityKind::Model, config.clone(), origin()))
                .id();
            let id = gz_sim_core::assign_id(sim.world_mut(), entity).unwrap();
            sim.world_mut().entity_mut(entity).insert(Publisher::seeded(
                id,
                config,
                Duration::ZERO,
                [56; 32],
            ));
            owners.push((entity, id));
        }
        install(&mut sim);
        sim.step_exact(2).unwrap();
        let (victim, id) = owners[1];
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(victim)
                .is_some()
        );
        let expected = match case {
            0 => {
                sim.world_mut()
                    .get_mut::<GroundTruthOdometryPublication>(victim)
                    .unwrap()
                    .odom_frame
                    .push_str("changed");
                GroundTruthOdometryError::Identity
            }
            1 => {
                sim.world_mut()
                    .get_mut::<Publisher>(victim)
                    .unwrap()
                    .previous_time = Some(Duration::from_secs(1));
                GroundTruthOdometryError::Time
            }
            _ => {
                sim.world_mut()
                    .get_mut::<Publisher>(victim)
                    .unwrap()
                    .revision = u64::MAX;
                GroundTruthOdometryError::Revision
            }
        };
        assert_eq!(
            sim.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(victim)
                .is_none()
        );
        let fault = sim
            .world()
            .resource::<StepFault>()
            .error()
            .unwrap()
            .downcast_ref::<GroundTruthOdometryScheduleFault>()
            .unwrap();
        assert_eq!((fault.entity, fault.source), (id, expected));
        // Global failure may follow an already visited healthy peer; do not
        // invent owner-order, peer rollback or independent continuation semantics.
        let clock = (sim.elapsed(), sim.epoch());
        let histories = owners
            .iter()
            .map(|(entity, _)| sim.world().get::<Publisher>(*entity).unwrap().clone())
            .collect::<Vec<_>>();
        assert_eq!(
            sim.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
        for ((entity, _), before) in owners.iter().zip(&histories) {
            same_history(sim.world().get::<Publisher>(*entity).unwrap(), before);
        }
    }
}
