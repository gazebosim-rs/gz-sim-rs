// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Scalar noise configuration and covariance remain bound to immutable leases.
use super::*;
use gz_sim_scene::GroundTruthOdometryNoise as Noise;

fn server() -> Server {
    let simulation = gz_sim_scene::load_world_with_sdf_step(
        include_str!("../../../../examples/m47_odometry_noise.sdf"),
        "noise-identity.sdf",
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
            partition: format!("m47-identity-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}

#[test]
fn sigma_bits_bind_prepared_identity_and_stale_streams_never_publish() {
    let mut server = server();
    server.simulation.step_exact(20).unwrap();
    let entity = server.ground_truth_odometries[0].entity;
    let original = server
        .simulation
        .world()
        .get::<Metadata>(entity)
        .unwrap()
        .clone();
    assert_eq!(Noise::from_stddev(-0.0).unwrap(), Noise::default());
    let clock = (server.simulation.epoch(), server.simulation.elapsed());
    let before = server.counters();
    for (a, b) in [
        (0.02, 0.03),
        (f64::EPSILON, f64::from_bits(f64::EPSILON.to_bits() + 1)),
    ] {
        let a = Noise::from_stddev(a).unwrap();
        let b = Noise::from_stddev(b).unwrap();
        assert_ne!(a, b);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                gaussian_noise: a,
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
        // Never step with tampered metadata; Prepared witnesses bind sigma too.
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
        for (_, config, _, _) in prepared.values_mut() {
            config.gaussian_noise = a;
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                gaussian_noise: b,
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

#[test]
fn packet_covariance_and_two_mode_boundary_follow_native_sigma_domain() {
    for invalid in [
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        1e30,
    ] {
        assert!(Noise::from_stddev(invalid).is_err());
    }
    let mut server = server();
    server.simulation.step_exact(20).unwrap();
    let stream = &server.ground_truth_odometries[0];
    let mut metadata = stream.metadata.clone();
    metadata.dimensions = Dimensions::Two;
    let mut sample = *server
        .simulation
        .world()
        .get::<Sample>(stream.entity)
        .unwrap();
    sample.pose.translation[2] = 0.0;
    sample.linear = [0.4, -0.2, 0.7];
    sample.angular = [-0.3, 0.8, 0.9];
    for sigma in [0.01, 1.0] {
        metadata.gaussian_noise = Noise::from_stddev(sigma).unwrap();
        let (odom, covariance, transform) = packets(&metadata, stream.identity, &sample).unwrap();
        assert_eq!(covariance.header, odom.header);
        let pose = covariance.pose_with_covariance.unwrap();
        let twist = covariance.twist_with_covariance.unwrap();
        assert_eq!(pose.pose, odom.pose);
        assert_eq!(twist.twist, odom.twist);
        for matrix in [pose.covariance.unwrap(), twist.covariance.unwrap()] {
            assert!(matrix.header.is_none());
            assert_eq!(matrix.data.len(), 36);
            for (i, value) in matrix.data.into_iter().enumerate() {
                assert_eq!(
                    value.to_bits(),
                    if i % 7 == 0 {
                        metadata.gaussian_noise.variance().to_bits()
                    } else {
                        0
                    }
                );
            }
        }
        let mut actual_tf = transform.pose[0].clone();
        assert_eq!(actual_tf.header.take(), odom.header);
        assert_eq!(Some(actual_tf), odom.pose);
        let mut invalid = sample;
        invalid.pose.translation[2] = 1.0;
        assert!(packets(&metadata, stream.identity, &invalid).is_none());
        invalid = sample;
        invalid.linear[0] = f64::NAN;
        assert!(packets(&metadata, stream.identity, &invalid).is_none());
    }
    for sigma in [0.0, -0.0, f64::EPSILON] {
        metadata.gaussian_noise = Noise::from_stddev(sigma).unwrap();
        assert!(packets(&metadata, stream.identity, &sample).is_none());
        let mut planar = sample;
        planar.linear[2] = 0.0;
        planar.angular[0] = 0.0;
        planar.angular[1] = 0.0;
        let (_, covariance, _) = packets(&metadata, stream.identity, &planar).unwrap();
        assert_eq!(
            covariance
                .twist_with_covariance
                .unwrap()
                .covariance
                .unwrap()
                .data[0]
                .to_bits(),
            metadata.gaussian_noise.variance().to_bits()
        );
    }
    metadata.gaussian_noise =
        Noise::from_stddev(f64::from_bits(f64::EPSILON.to_bits() + 1)).unwrap();
    assert!(packets(&metadata, stream.identity, &sample).is_some());
    server.shutdown();
}
