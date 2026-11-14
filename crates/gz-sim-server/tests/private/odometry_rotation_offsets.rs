// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bit-aware rotated configuration remains part of each immutable endpoint lease.
use super::*;
use gz_sim_scene::GroundTruthOdometryOffsets as Offsets;

fn server() -> Server {
    let simulation = gz_sim_scene::load_world_with_sdf_step(
        include_str!("../../../../examples/m46_odometry_rotation_offsets.sdf"),
        "rotation-identity.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Rapier,
    )
    .unwrap();
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
    Server::with_zenoh_config(
        simulation,
        "default",
        ServerOptions {
            partition: format!("m46-identity-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}

#[test]
fn each_rotation_vector_bits_bind_prepared_and_live_stream_identity() {
    let mut server = server();
    server.simulation.step_exact(20).unwrap();
    let entity = server.ground_truth_odometries[0].entity;
    let original = server
        .simulation
        .world()
        .get::<Metadata>(entity)
        .unwrap()
        .clone();
    let translations = Offsets::from_translations([0.4, -0.3, 0.2], [-0.7, 0.5, -0.6]).unwrap();
    assert_eq!(
        translations
            .with_rotations([-0.0, 0.0, -0.0], [0.0, -0.0, -0.0])
            .unwrap(),
        translations
    );
    let clock = (server.simulation.epoch(), server.simulation.elapsed());
    let before = server.counters();
    // A zero component's sign remains significant inside a nonzero vector;
    // independently canonicalizing the opposite whole-zero vector changes none.
    for (a, b) in [
        (
            translations
                .with_rotations([0.3, -0.0, 0.0], [-0.0, 0.0, -0.0])
                .unwrap(),
            translations
                .with_rotations([0.3, 0.0, 0.0], [0.0; 3])
                .unwrap(),
        ),
        (
            translations
                .with_rotations([-0.0, 0.0, -0.0], [-0.2, 0.0, -0.0])
                .unwrap(),
            translations
                .with_rotations([0.0; 3], [-0.2, 0.0, 0.0])
                .unwrap(),
        ),
    ] {
        assert_ne!(a, b);
        for offsets in [a, b] {
            assert_eq!(
                offsets.local_translation().map(f64::to_bits),
                original.offsets.local_translation().map(f64::to_bits)
            );
            assert_eq!(
                offsets.global_translation().map(f64::to_bits),
                original.offsets.global_translation().map(f64::to_bits)
            );
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .insert(Metadata {
                    offsets,
                    ..original.clone()
                });
            assert!(
                server
                    .ground_truth_odometries
                    .iter()
                    .all(|stream| !stream.live(&server.simulation))
            );
            server.publish_ground_truth_odometry();
            let after = server.counters();
            assert_eq!(
                after.ground_truth_odometry_queued,
                before.ground_truth_odometry_queued
            );
            assert_eq!(
                after.ground_truth_odometry_covariance_queued,
                before.ground_truth_odometry_covariance_queued
            );
            assert_eq!(
                after.ground_truth_odometry_tf_queued,
                before.ground_truth_odometry_tf_queued
            );
            assert_eq!(
                after.ground_truth_odometry_invalid_samples,
                before.ground_truth_odometry_invalid_samples
            );
        }
        // No tick occurs with tampered metadata. Prepare same-owner/channel
        // witnesses for a, then refuse their reuse against bit-distinct b.
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                offsets: a,
                ..original.clone()
            });
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
        for (_, config, _, _, _) in prepared.values_mut() {
            config.offsets = a;
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                offsets: b,
                ..original.clone()
            });
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &BTreeSet::new(),
                Some(&prepared)
            ),
            Err(Error::GroundTruthOdometryIdentity)
        ));
        assert_eq!(
            (server.simulation.epoch(), server.simulation.elapsed()),
            clock
        );
        drop(prepared);
    }
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original);
    assert!(
        server
            .ground_truth_odometries
            .iter()
            .all(|stream| stream.live(&server.simulation))
    );
    server.shutdown();
}
