// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ground-truth odometry streams; wheel-encoder odometry remains independent.
use super::*;
use crate::odometry_publisher_io::{
    GroundTruthOdometryChannel as Channel, GroundTruthOdometryOutput as Output,
};
use gz_sim_proto::msgs::{
    FloatV, Header, Odometry, OdometryWithCovariance, Pose, PoseV, PoseWithCovariance, Twist,
    TwistWithCovariance, header::Map,
};
use gz_sim_scene::{
    EntityKind, GroundTruthOdometryDimensions as Dimensions,
    GroundTruthOdometryPublication as Metadata, GroundTruthOdometrySample as Sample, Name,
    PendingResetRemoval,
};
pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    pub(super) metadata: Metadata,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<Output>,
    channel: Channel,
    last_sample: Option<(u64, u64)>,
}
fn channels(metadata: &Metadata) -> [(Channel, &str); 3] {
    [
        (Channel::Odometry, &metadata.odom_topic),
        (Channel::Covariance, &metadata.covariance_topic),
        (Channel::Transform, &metadata.tf_topic),
    ]
}
fn valid(metadata: &Metadata, owner: u64) -> bool {
    owner != 0
        && !metadata.model_name.is_empty()
        && !metadata.model_name.contains("::")
        && [&metadata.odom_frame, &metadata.robot_base_frame]
            .into_iter()
            .all(|v| {
                !v.is_empty()
                    && v.len() <= 65535
                    && !v
                        .bytes()
                        .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
            })
        && channels(metadata).iter().all(|(_, t)| {
            t.starts_with('/')
                && t.len() > 1
                && t.len() <= 65535
                && options::as_valid_topic(t) == *t
        })
        && channels(metadata)
            .into_iter()
            .map(|(_, t)| t)
            .collect::<BTreeSet<_>>()
            .len()
            == 3
}
fn packets(
    metadata: &Metadata,
    owner: u64,
    sample: &Sample,
) -> Option<(Odometry, OdometryWithCovariance, PoseV)> {
    if !valid(metadata, owner)
        || sample.revision == 0
        || (metadata.dimensions == Dimensions::Two
            && (sample.pose.translation[2] != 0.0
                || sample.linear[2] != 0.0
                || sample.angular[0] != 0.0
                || sample.angular[1] != 0.0))
        || !sample
            .pose
            .translation
            .into_iter()
            .chain(sample.pose.rotation)
            .chain(sample.linear)
            .chain(sample.angular)
            .all(f64::is_finite)
        || (sample.pose.rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-12
    {
        return None;
    }
    let header = Header {
        stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
        data: vec![
            Map {
                key: "frame_id".into(),
                value: vec![metadata.odom_frame.clone()],
            },
            Map {
                key: "child_frame_id".into(),
                value: vec![metadata.robot_base_frame.clone()],
            },
        ],
    };
    let pose = Pose {
        position: Some(vector3(sample.pose.translation)),
        orientation: Some(quaternion(sample.pose.rotation)),
        ..Default::default()
    };
    let twist = Twist {
        linear: Some(vector3(sample.linear)),
        angular: Some(vector3(sample.angular)),
        ..Default::default()
    };
    let odometry = Odometry {
        header: Some(header.clone()),
        pose: Some(pose.clone()),
        twist: Some(twist.clone()),
    };
    let covariance = FloatV {
        data: vec![0.0; 36],
        ..Default::default()
    };
    let cov = OdometryWithCovariance {
        header: Some(header.clone()),
        pose_with_covariance: Some(PoseWithCovariance {
            pose: Some(pose.clone()),
            covariance: Some(covariance.clone()),
        }),
        twist_with_covariance: Some(TwistWithCovariance {
            twist: Some(twist),
            covariance: Some(covariance),
        }),
    };
    let tf = PoseV {
        header: None,
        pose: vec![Pose {
            header: Some(header),
            ..pose
        }],
    };
    Some((odometry, cov, tf))
}
pub(super) type Prepared = BTreeMap<String, (u64, Metadata, Channel, Arc<Output>)>;
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let metadata: Vec<_> =
        gz_sim_scene::initial_ground_truth_odometry_publications(&server.simulation)?
            .map(|(id, m)| (id, m.clone()))
            .collect();
    if metadata.len() > 1 || metadata.iter().any(|(id, m)| !valid(m, *id)) {
        return Err(Error::GroundTruthOdometryIdentity);
    }
    let mut selected = Vec::new();
    let mut keys = occupied.clone();
    for (id, metadata) in metadata {
        for (channel, topic) in channels(&metadata) {
            let key = server.network_config.topic_key(topic)?;
            if !keys.insert(key.clone()) {
                return Err(Error::GroundTruthOdometryIdentity);
            }
            selected.push((id, metadata.clone(), channel, key));
        }
    }
    let mut result = BTreeMap::new();
    for (id, metadata, channel, key) in selected {
        let publisher = match server
            .ground_truth_odometries
            .iter()
            .find(|s| s.endpoint == key && s.channel == channel)
        {
            Some(s) => Arc::clone(&s.publisher),
            None => server
                .world_io
                .ground_truth_odometry_output(key.clone(), channel)
                .map_err(|e| Error::WorldNetwork(e.to_string()))?,
        };
        result.insert(key, (id, metadata, channel, publisher));
    }
    Ok(result)
}
fn binding(sim: &Simulation, entity: Entity, id: u64, metadata: &Metadata) -> bool {
    let world = sim.world();
    valid(metadata, id)
        && world.get::<EntityKind>(entity) == Some(&EntityKind::Model)
        && world.get::<EntityId>(entity).map(EntityId::get) == Some(id)
        && entity_by_id(world, id) == Some(entity)
        && world.get::<Name>(entity).is_some_and(|n| {
            n.0.split_once("::")
                .is_some_and(|(_, local)| local == metadata.model_name)
        })
        && !world.entity(entity).contains::<PendingResetRemoval>()
}
pub(super) fn streams(
    sim: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
    prepared: Option<&Prepared>,
) -> Result<Vec<Stream>, Error> {
    let candidates: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<PendingResetRemoval>() && e.contains::<Metadata>())
        .map(|e| e.id())
        .collect();
    if candidates.len() > 1 {
        return Err(Error::GroundTruthOdometryIdentity);
    }
    let mut keys = occupied.clone();
    let mut selected = Vec::new();
    for entity in candidates {
        let metadata = sim
            .world()
            .get::<Metadata>(entity)
            .expect("selected metadata");
        let id = sim
            .world()
            .get::<EntityId>(entity)
            .ok_or(Error::GroundTruthOdometryIdentity)?
            .get();
        if !binding(sim, entity, id, metadata) {
            return Err(Error::GroundTruthOdometryIdentity);
        }
        for (channel, topic) in channels(metadata) {
            let key = options.topic_key(topic)?;
            if !keys.insert(key.clone()) {
                return Err(Error::GroundTruthOdometryIdentity);
            }
            if let Some(p) = prepared {
                let Some((original, config, kind, _)) = p.get(&key) else {
                    return Err(Error::GroundTruthOdometryIdentity);
                };
                if *original != id || config != metadata || *kind != channel {
                    return Err(Error::GroundTruthOdometryIdentity);
                }
            }
            selected.push((entity, id, metadata.clone(), channel, key));
        }
    }
    if prepared.is_some_and(|p| p.len() != selected.len()) {
        return Err(Error::GroundTruthOdometryIdentity);
    }
    let mut result = Vec::new();
    for (entity, identity, metadata, channel, endpoint) in selected {
        let publisher = match prepared {
            Some(p) => Arc::clone(&p.get(&endpoint).expect("all keys preflighted").3),
            None => io
                .ground_truth_odometry_output(endpoint.clone(), channel)
                .map_err(|e| Error::WorldNetwork(e.to_string()))?,
        };
        result.push(Stream {
            entity,
            identity,
            metadata,
            endpoint,
            publisher,
            channel,
            last_sample: None,
        });
    }
    Ok(result)
}
impl Stream {
    fn live(&self, sim: &Simulation) -> bool {
        sim.world().get::<Metadata>(self.entity) == Some(&self.metadata)
            && binding(sim, self.entity, self.identity, &self.metadata)
    }
}
impl Server {
    pub(super) fn publish_ground_truth_odometry(&mut self) {
        for stream in &mut self.ground_truth_odometries {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self.simulation.world().get::<Sample>(stream.entity) else {
                continue;
            };
            let key = (sample.epoch, sample.revision);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let frames = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| packets(&stream.metadata, stream.identity, sample))
            .flatten();
            let Some((odom, cov, tf)) = frames else {
                self.counters.ground_truth_odometry_invalid_samples += 1;
                continue;
            };
            let bytes = match stream.channel {
                Channel::Odometry => odom.encode_to_vec(),
                Channel::Covariance => cov.encode_to_vec(),
                Channel::Transform => tf.encode_to_vec(),
            };
            if self
                .world_io
                .publish_ground_truth_odometry(&stream.publisher, bytes)
                .is_ok()
            {
                match stream.channel {
                    Channel::Odometry => self.counters.ground_truth_odometry_queued += 1,
                    Channel::Covariance => {
                        self.counters.ground_truth_odometry_covariance_queued += 1
                    }
                    Channel::Transform => self.counters.ground_truth_odometry_tf_queued += 1,
                }
            } else {
                self.counters.ground_truth_odometry_failures += 1;
            }
        }
    }
    pub(super) fn retire_ground_truth_odometry(&mut self) {
        self.ground_truth_odometries
            .retain(|s| s.live(&self.simulation));
        if self.ground_truth_odometries.is_empty() {
            self.ground_truth_odometries = Vec::new();
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn all_three_packets_share_pose_stamp_and_headers_without_encoder_sequence() {
        let metadata = Metadata {
            dimensions: Dimensions::Two,
            offsets: Default::default(),
            model_name: "rig".into(),
            odom_frame: "odom".into(),
            robot_base_frame: "base".into(),
            odom_topic: "/odom".into(),
            covariance_topic: "/cov".into(),
            tf_topic: "/tf".into(),
        };
        let sample = Sample {
            time: Duration::from_millis(20),
            epoch: 0,
            revision: 1,
            pose: gz_sim_scene::WorldPose {
                translation: [2.0, -1.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            linear: [1.0, 2.0, 0.0],
            angular: [0.0, 0.0, 0.4],
        };
        let (odom, cov, tf) = packets(&metadata, 1, &sample).unwrap();
        assert_eq!(odom.header.as_ref().unwrap().data.len(), 2);
        assert_eq!(cov.header, odom.header);
        assert_eq!(tf.pose[0].header, odom.header);
        assert!(tf.header.is_none());
        assert_eq!(cov.pose_with_covariance.as_ref().unwrap().pose, odom.pose);
        assert_eq!(
            cov.twist_with_covariance.as_ref().unwrap().twist,
            odom.twist
        );
        assert_eq!(
            cov.pose_with_covariance.unwrap().covariance.unwrap().data,
            vec![0.0; 36]
        );
        assert_eq!(
            cov.twist_with_covariance.unwrap().covariance.unwrap().data,
            vec![0.0; 36]
        );
        assert_eq!(tf.pose[0].id, 0);
        assert!(tf.pose[0].name.is_empty());
        for bad in [
            Sample {
                revision: 0,
                ..sample
            },
            Sample {
                linear: [f64::NAN, 0.0, 0.0],
                ..sample
            },
            Sample {
                angular: [1.0, 0.0, 0.0],
                ..sample
            },
            Sample {
                pose: gz_sim_scene::WorldPose {
                    translation: [0.0; 3],
                    rotation: [0.0; 4],
                },
                ..sample
            },
        ] {
            assert!(packets(&metadata, 1, &bad).is_none());
        }
        assert!(packets(&metadata, 0, &sample).is_none());
        let mut duplicate = metadata.clone();
        duplicate.tf_topic = metadata.odom_topic.clone();
        assert!(!valid(&duplicate, 1));
        let mut bad = metadata;
        bad.odom_frame = "with space".into();
        assert!(!valid(&bad, 1));
    }

    #[test]
    fn actual_native_scalar_values_encode_all_663_exact_canonical_packets() {
        mod reference {
            include!("../../gz-sim-scene/tests/fixtures/m34_native_odometry.rs");
        }
        let metadata = Metadata {
            dimensions: Dimensions::Two,
            offsets: Default::default(),
            model_name: "rig".into(),
            odom_frame: "m34/odom".into(),
            robot_base_frame: "m34/base".into(),
            odom_topic: "/m34/odometry".into(),
            covariance_topic: "/m34/odometry_covariance".into(),
            tf_topic: "/m34/transform".into(),
        };
        let mut acquisitions = 0;
        let mut canonical_packets = 0;
        for &(mode, _, ticks) in reference::CASES {
            for &(time_ns, _, _, _, expected) in ticks {
                let reference::Publication::Observed((pose_bits, linear_bits, angular_bits, wire)) =
                    expected
                else {
                    continue;
                };
                let pose = pose_bits.map(f64::from_bits);
                let sample = Sample {
                    time: Duration::from_nanos(time_ns),
                    epoch: 7,
                    revision: 1,
                    pose: gz_sim_scene::WorldPose {
                        translation: [pose[0], pose[1], pose[2]],
                        rotation: [pose[3], pose[4], pose[5], pose[6]],
                    },
                    linear: linear_bits.map(f64::from_bits),
                    angular: angular_bits.map(f64::from_bits),
                };
                let (odom, covariance, transform) =
                    packets(&metadata, 1, &sample).unwrap_or_else(|| {
                        panic!("actual native values were rejected: {mode}/{time_ns}")
                    });
                let encoded = [
                    odom.encode_to_vec(),
                    covariance.encode_to_vec(),
                    transform.encode_to_vec(),
                ];
                for (channel, (actual, expected)) in encoded.iter().zip(wire).enumerate() {
                    assert_eq!(
                        actual.as_slice(),
                        expected,
                        "actual native canonical bytes: {mode}/{time_ns}/channel{channel}"
                    );
                    canonical_packets += 1;
                }
                acquisitions += 1;
            }
        }
        assert_eq!((acquisitions, canonical_packets), (221, 663));
    }

    fn negative_control_server() -> Server {
        let sim = gz_sim_scene::load_world_with_sdf_step(
            include_str!("../../../examples/m34_odometry_publisher.sdf"),
            "odometry-negative.sdf",
            "default",
            &sdformat::NoResources,
            gz_sim_scene::PhysicsEngine::Rapier,
        )
        .unwrap();
        let options = ServerOptions {
            partition: format!("m34-negative-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        };
        let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
        Server::with_zenoh_config(
            sim,
            "default",
            options,
            config,
            NonZeroUsize::new(8).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn packet_refusal_covers_each_planar_finite_metadata_and_stamp_boundary() {
        let mut server = negative_control_server();
        server.simulation.step_exact(20).unwrap();
        let stream = &server.ground_truth_odometries[0];
        let metadata = stream.metadata.clone();
        let sample = *server
            .simulation
            .world()
            .get::<Sample>(stream.entity)
            .unwrap();
        assert!(packets(&metadata, stream.identity, &sample).is_some());
        for field in 0..8 {
            let mut bad = metadata.clone();
            match field {
                0 => bad.model_name.clear(),
                1 => bad.model_name = "parent::rig".into(),
                2 => bad.odom_frame.clear(),
                3 => bad.robot_base_frame = "bad\nframe".into(),
                4 => bad.odom_topic = "relative".into(),
                5 => bad.covariance_topic = "/".into(),
                6 => bad.tf_topic = "/bad topic".into(),
                _ => bad.tf_topic = bad.odom_topic.clone(),
            }
            assert!(
                packets(&bad, stream.identity, &sample).is_none(),
                "metadata {field}"
            );
        }
        for field in 0..9 {
            let mut bad = sample;
            match field {
                0 => bad.pose.translation[2] = 1.0,
                1 => bad.linear[2] = 1.0,
                2 => bad.angular[0] = 1.0,
                3 => bad.angular[1] = 1.0,
                4 => bad.pose.translation[0] = f64::INFINITY,
                5 => bad.pose.rotation[3] = f64::NAN,
                6 => bad.linear[0] = f64::NEG_INFINITY,
                7 => bad.angular[2] = f64::NAN,
                _ => bad.pose.rotation = [0.0; 4],
            }
            assert!(
                packets(&metadata, stream.identity, &bad).is_none(),
                "sample {field}"
            );
        }
        let mut too_late = sample;
        too_late.time = Duration::from_secs(i64::MAX as u64);
        assert!(packets(&metadata, stream.identity, &too_late).is_some());
        too_late.time += Duration::from_secs(1);
        assert!(packets(&metadata, stream.identity, &too_late).is_none());
        // Negative zeros are valid planar values, not grounds to reject a frame.
        let mut signed = sample;
        signed.pose.translation[2] = -0.0;
        signed.linear[2] = -0.0;
        signed.angular[0] = -0.0;
        signed.angular[1] = -0.0;
        assert!(packets(&metadata, stream.identity, &signed).is_some());
    }

    #[test]
    fn stream_preflight_rejects_identity_and_prepared_drift_without_replacing_live_leases() {
        let mut server = negative_control_server();
        let entity = server.ground_truth_odometries[0].entity;
        let identity = server.ground_truth_odometries[0].identity;
        let metadata = server.ground_truth_odometries[0].metadata.clone();
        let occupied = BTreeSet::new();
        let prepared = prepare_reset(&server, &occupied).unwrap();
        assert_eq!(prepared.len(), 3);
        for stream in &server.ground_truth_odometries {
            assert!(Arc::ptr_eq(
                &stream.publisher,
                &prepared[&stream.endpoint].3
            ));
        }
        let original_leases: Vec<_> = server
            .ground_truth_odometries
            .iter()
            .map(|s| Arc::downgrade(&s.publisher))
            .collect();
        let refuses = |sim: &Simulation, p: &Prepared| {
            assert!(matches!(
                streams(
                    sim,
                    &server.world_io,
                    &server.network_config,
                    &occupied,
                    Some(p)
                ),
                Err(Error::GroundTruthOdometryIdentity)
            ));
        };
        for change in 0..5 {
            let mut invalid = prepared.clone();
            match change {
                0 => {
                    invalid.pop_first();
                }
                1 => invalid.values_mut().next().unwrap().0 += 1,
                2 => invalid
                    .values_mut()
                    .next()
                    .unwrap()
                    .1
                    .odom_frame
                    .push_str("-changed"),
                3 => invalid.values_mut().next().unwrap().2 = Channel::Transform,
                _ => {
                    let value = invalid.values().next().unwrap().clone();
                    invalid.insert("@/extra@/key".into(), value);
                }
            }
            refuses(&server.simulation, &invalid);
        }
        let key = server.ground_truth_odometries[0].endpoint.clone();
        assert!(matches!(
            prepare_reset(&server, &BTreeSet::from([key])),
            Err(Error::GroundTruthOdometryIdentity)
        ));
        assert!(binding(&server.simulation, entity, identity, &metadata));
        assert!(!binding(
            &server.simulation,
            entity,
            identity + 1,
            &metadata
        ));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(EntityKind::Link);
        assert!(!binding(&server.simulation, entity, identity, &metadata));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(EntityKind::Model);
        let name = server
            .simulation
            .world()
            .get::<Name>(entity)
            .unwrap()
            .0
            .clone();
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Name("default::wrong".into()));
        assert!(!binding(&server.simulation, entity, identity, &metadata));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Name(name));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(PendingResetRemoval);
        assert!(!binding(&server.simulation, entity, identity, &metadata));
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &occupied,
                Some(&prepared)
            ),
            Err(Error::GroundTruthOdometryIdentity)
        ));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .remove::<PendingResetRemoval>();
        let orphan = server.simulation.world_mut().spawn(metadata.clone()).id();
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &occupied,
                None
            ),
            Err(Error::GroundTruthOdometryIdentity)
        ));
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .remove::<Metadata>();
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &occupied,
                None
            ),
            Err(Error::GroundTruthOdometryIdentity)
        ));
        server.simulation.world_mut().despawn(orphan);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(metadata);
        assert!(
            original_leases
                .iter()
                .all(|lease| lease.upgrade().is_some())
        );
        assert_eq!(server.ground_truth_odometries.len(), 3);
        drop(prepared);
        server.ground_truth_odometries.clear();
        assert!(
            original_leases
                .iter()
                .all(|lease| lease.upgrade().is_none())
        );
        let recreated = prepare_reset(&server, &occupied).unwrap();
        assert_eq!(recreated.len(), 3);
        let restored = streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &occupied,
            Some(&recreated),
        )
        .unwrap();
        assert_eq!(restored.len(), 3);
        assert!(
            restored
                .iter()
                .all(|stream| Arc::ptr_eq(&stream.publisher, &recreated[&stream.endpoint].3))
        );
    }

    #[test]
    fn invalid_or_retired_samples_never_requeue_and_closed_io_counts_each_channel_failure() {
        let mut server = negative_control_server();
        server.simulation.step_exact(20).unwrap();
        let entity = server.ground_truth_odometries[0].entity;
        let base = *server.simulation.world().get::<Sample>(entity).unwrap();
        server.publish_ground_truth_odometry();
        assert_eq!(
            (
                server.counters.ground_truth_odometry_queued,
                server.counters.ground_truth_odometry_covariance_queued,
                server.counters.ground_truth_odometry_tf_queued
            ),
            (1, 1, 1)
        );
        server.publish_ground_truth_odometry();
        assert_eq!(server.counters.ground_truth_odometry_queued, 1);
        for case in 0..4 {
            let mut bad = base;
            bad.revision += 1 + case;
            match case {
                0 => bad.epoch += 1,
                1 => bad.time += Duration::from_millis(1),
                2 => bad.pose.translation[2] = 1.0,
                _ => bad.linear[0] = f64::NAN,
            }
            server.simulation.world_mut().entity_mut(entity).insert(bad);
            server.publish_ground_truth_odometry();
            assert_eq!(
                server.counters.ground_truth_odometry_invalid_samples,
                (case + 1) * 3
            );
            server.publish_ground_truth_odometry();
            assert_eq!(
                server.counters.ground_truth_odometry_invalid_samples,
                (case + 1) * 3
            );
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .remove::<Sample>();
        server.publish_ground_truth_odometry();
        assert_eq!(server.counters.ground_truth_odometry_invalid_samples, 12);
        server.world_io.session.close().wait().unwrap();
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Sample {
                revision: 10,
                ..base
            });
        server.publish_ground_truth_odometry();
        assert_eq!(server.counters.ground_truth_odometry_failures, 3);
        assert_eq!(server.counters.ground_truth_odometry_queued, 1);
        let weak: Vec<_> = server
            .ground_truth_odometries
            .iter()
            .map(|s| Arc::downgrade(&s.publisher))
            .collect();
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .remove::<Metadata>();
        server.publish_ground_truth_odometry();
        assert_eq!(server.counters.ground_truth_odometry_failures, 3);
        server.retire_ground_truth_odometry();
        assert!(server.ground_truth_odometries.is_empty());
        assert!(weak.iter().all(|lease| lease.upgrade().is_none()));
    }

    #[test]
    fn three_dimensional_packets_retain_xyz_and_all_two_dimensional_zero_refusals() {
        let metadata = Metadata {
            dimensions: Dimensions::Three,
            offsets: Default::default(),
            model_name: "rig".into(),
            odom_frame: "m38/odom".into(),
            robot_base_frame: "m38/base".into(),
            odom_topic: "/m38/odometry".into(),
            covariance_topic: "/m38/odometry_covariance".into(),
            tf_topic: "/m38/transform".into(),
        };
        let sample = Sample {
            time: Duration::from_millis(20),
            epoch: 7,
            revision: 1,
            pose: gz_sim_scene::WorldPose {
                translation: [2.0, -1.0, 3.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            linear: [0.25, -0.5, 0.75],
            angular: [-0.1, 0.2, -0.3],
        };
        let (odom, covariance, transform) = packets(&metadata, 1, &sample).unwrap();
        let pose = odom.pose.as_ref().unwrap();
        assert_eq!(
            pose.position.as_ref().map(|v| [v.x, v.y, v.z]),
            Some(sample.pose.translation)
        );
        let twist = odom.twist.as_ref().unwrap();
        assert_eq!(
            twist.linear.as_ref().map(|v| [v.x, v.y, v.z]),
            Some(sample.linear)
        );
        assert_eq!(
            twist.angular.as_ref().map(|v| [v.x, v.y, v.z]),
            Some(sample.angular)
        );
        assert_eq!(covariance.header, odom.header);
        assert_eq!(
            covariance.pose_with_covariance.as_ref().unwrap().pose,
            odom.pose
        );
        assert_eq!(
            covariance.twist_with_covariance.as_ref().unwrap().twist,
            odom.twist
        );
        for values in [
            covariance.pose_with_covariance.unwrap().covariance,
            covariance.twist_with_covariance.unwrap().covariance,
        ] {
            assert_eq!(values.unwrap().data, [0.0; 36]);
        }
        assert!(transform.header.is_none());
        assert_eq!(transform.pose.len(), 1);
        let mut tf = transform.pose[0].clone();
        assert_eq!(tf.header.take(), odom.header);
        assert_eq!(Some(tf), odom.pose);
        let planar = Metadata {
            dimensions: Dimensions::Two,
            ..metadata.clone()
        };
        let zero = Sample {
            pose: gz_sim_scene::WorldPose {
                translation: [2.0, -1.0, 0.0],
                rotation: sample.pose.rotation,
            },
            linear: [0.25, -0.5, 0.0],
            angular: [0.0, 0.0, -0.3],
            ..sample
        };
        assert!(packets(&planar, 1, &zero).is_some());
        let mut cases = [zero; 4];
        cases[0].pose.translation[2] = 3.0;
        cases[1].linear[2] = 0.75;
        cases[2].angular[0] = -0.1;
        cases[3].angular[1] = 0.2;
        for value in cases {
            assert!(packets(&planar, 1, &value).is_none());
            assert!(packets(&metadata, 1, &value).is_some());
        }
        let mut bad = [sample; 7];
        bad[0].pose.translation[2] = f64::NAN;
        bad[1].linear[2] = f64::INFINITY;
        bad[2].angular[0] = f64::NAN;
        bad[3].angular[1] = f64::NEG_INFINITY;
        bad[4].pose.rotation = [0.0; 4];
        bad[5].revision = 0;
        bad[6].time = Duration::from_secs(i64::MAX as u64 + 1);
        for value in bad {
            assert!(packets(&metadata, 1, &value).is_none());
        }
    }

    #[test]
    fn dimensionality_mutation_cannot_reuse_a_prepared_stream_or_publish_stale_data() {
        let mut server = negative_control_server();
        server.simulation.step_exact(20).unwrap();
        let entity = server.ground_truth_odometries[0].entity;
        let metadata = server
            .simulation
            .world()
            .get::<Metadata>(entity)
            .unwrap()
            .clone();
        assert_eq!(metadata.dimensions, Dimensions::Two);
        assert!(
            server
                .ground_truth_odometries
                .iter()
                .all(|s| s.live(&server.simulation))
        );
        let clock = (server.simulation.epoch(), server.simulation.elapsed());
        let before = server.counters();
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                dimensions: Dimensions::Three,
                ..metadata.clone()
            });
        assert!(
            server
                .ground_truth_odometries
                .iter()
                .all(|s| !s.live(&server.simulation))
        );
        server.publish_ground_truth_odometry();
        assert_eq!(
            server.counters().ground_truth_odometry_queued,
            before.ground_truth_odometry_queued
        );
        assert_eq!(
            server.counters().ground_truth_odometry_covariance_queued,
            before.ground_truth_odometry_covariance_queued
        );
        assert_eq!(
            server.counters().ground_truth_odometry_tf_queued,
            before.ground_truth_odometry_tf_queued
        );
        assert_eq!(
            server.counters().ground_truth_odometry_invalid_samples,
            before.ground_truth_odometry_invalid_samples
        );
        assert_eq!(
            (server.simulation.epoch(), server.simulation.elapsed()),
            clock
        );
        // This is a metadata identity control, not an advance with a tampered mode.
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(metadata);
        assert!(
            server
                .ground_truth_odometries
                .iter()
                .all(|s| s.live(&server.simulation))
        );
        server.shutdown();
    }
    #[test]
    fn actual_three_dimensional_native_values_encode_all_1287_canonical_packets() {
        mod reference {
            include!("../../gz-sim-scene/tests/fixtures/m38_native_odometry_3d.rs");
        }
        let metadata = Metadata {
            dimensions: Dimensions::Three,
            offsets: Default::default(),
            model_name: "rig".into(),
            odom_frame: "m38/odom".into(),
            robot_base_frame: "m38/base".into(),
            odom_topic: "/m38/odometry".into(),
            covariance_topic: "/m38/odometry_covariance".into(),
            tf_topic: "/m38/transform".into(),
        };
        let mut inputs = 0;
        let mut acquisitions = 0;
        let mut suppressed = 0;
        let mut canonical_packets = 0;
        assert_eq!(reference::CASES.len(), 16);
        for &(mode, _, ticks) in reference::CASES {
            // Reload generations are separate fixture cases. Rust revision and
            // epoch are local bookkeeping; neither is invented in native headers.
            let mut revision = 0;
            for &(time_ns, _, _, _, expected) in ticks {
                inputs += 1;
                let reference::Publication::Observed((pose_bits, linear_bits, angular_bits, wire)) =
                    expected
                else {
                    suppressed += 1;
                    continue;
                };
                revision += 1;
                let pose = pose_bits.map(f64::from_bits);
                let sample = Sample {
                    time: Duration::from_nanos(time_ns),
                    epoch: 7,
                    revision,
                    pose: gz_sim_scene::WorldPose {
                        translation: [pose[0], pose[1], pose[2]],
                        rotation: [pose[3], pose[4], pose[5], pose[6]],
                    },
                    linear: linear_bits.map(f64::from_bits),
                    angular: angular_bits.map(f64::from_bits),
                };
                let (odometry, covariance, transform) = packets(&metadata, 1, &sample)
                    .unwrap_or_else(|| {
                        panic!("actual native 3D values were rejected: {mode}/{time_ns}")
                    });
                // These are actual output bits already in the final odometry
                // frame. MODEL->WORLD conversion belongs to input replay only.
                let encoded = [
                    odometry.encode_to_vec(),
                    covariance.encode_to_vec(),
                    transform.encode_to_vec(),
                ];
                for (channel, (actual, expected)) in encoded.iter().zip(wire).enumerate() {
                    assert_eq!(
                        actual.as_slice(),
                        expected,
                        "actual native 3D canonical bytes: {mode}/{time_ns}/channel{channel}"
                    );
                    canonical_packets += 1;
                }
                acquisitions += 1;
            }
        }
        assert_eq!(
            (inputs, acquisitions, suppressed, canonical_packets),
            (510, 429, 81, 1287)
        );
    }
    #[test]
    fn translation_bits_are_prepared_identity_and_changed_streams_cannot_publish() {
        use gz_sim_scene::GroundTruthOdometryOffsets as Offsets;
        let mut server = negative_control_server();
        server.simulation.step_exact(20).unwrap();
        let entity = server.ground_truth_odometries[0].entity;
        let original = server
            .simulation
            .world()
            .get::<Metadata>(entity)
            .unwrap()
            .clone();
        let canonical_zero =
            Offsets::from_translations([-0.0, 0.0, -0.0], [0.0, -0.0, -0.0]).unwrap();
        assert_eq!(canonical_zero, original.offsets);
        let local = Offsets::from_translations([0.4, -0.0, 0.0], [0.0; 3]).unwrap();
        let local_bit_change = Offsets::from_translations([0.4, 0.0, 0.0], [0.0; 3]).unwrap();
        assert_ne!(local, local_bit_change);
        let global = Offsets::from_translations([0.0; 3], [-0.7, 0.0, -0.0]).unwrap();
        let clock = (server.simulation.epoch(), server.simulation.elapsed());
        let before = server.counters();
        for offsets in [local, local_bit_change, global] {
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
                    .all(|s| !s.live(&server.simulation))
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
            assert_eq!(
                (server.simulation.epoch(), server.simulation.elapsed()),
                clock
            );
        }
        // This private control does not step with modified metadata. Build a
        // same-owner/channel Prepared witness with local offset bits, then
        // require its rejection against a one-bit-different live configuration.
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                offsets: local,
                ..original.clone()
            });
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
        for (_, config, _, _) in prepared.values_mut() {
            config.offsets = local;
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(Metadata {
                offsets: local_bit_change,
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
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(original);
        assert!(
            server
                .ground_truth_odometries
                .iter()
                .all(|s| s.live(&server.simulation))
        );
        drop(prepared);
        server.shutdown();
    }

    #[test]
    fn actual_translated_native_values_encode_all_2958_canonical_packets() {
        mod reference {
            include!(
                "../../gz-sim-scene/tests/fixtures/m39_native_odometry_translation_offsets.rs"
            );
        }
        use gz_sim_scene::GroundTruthOdometryOffsets as Offsets;
        let mut inputs = 0;
        let mut acquisitions = 0;
        let mut suppressed = 0;
        let mut canonical_packets = 0;
        assert_eq!(reference::CASES.len(), 36);
        for &(mode, config, ticks) in reference::CASES {
            let dimensions = match config.dimensions {
                2 => Dimensions::Two,
                3 => Dimensions::Three,
                other => panic!("invalid actual native dimensions: {mode}/{other}"),
            };
            assert!(matches!(config.frequency, 0 | 50));
            assert_eq!(
                config.period_ns,
                if config.frequency == 50 {
                    20_000_000
                } else {
                    0
                }
            );
            if !config.requested_local_present {
                assert_eq!(config.requested_local, [0; 3]);
            }
            if !config.requested_global_present {
                assert_eq!(config.requested_global, [0; 3]);
            }
            let offsets = Offsets::from_translations(
                config.requested_local.map(f64::from_bits),
                config.requested_global.map(f64::from_bits),
            )
            .unwrap();
            assert_eq!(
                offsets.local_translation().map(f64::to_bits),
                config.effective_local
            );
            assert_eq!(
                offsets.global_translation().map(f64::to_bits),
                config.effective_global
            );
            let metadata = Metadata {
                dimensions,
                offsets,
                model_name: "rig".into(),
                odom_frame: "m39/odom".into(),
                robot_base_frame: "m39/base".into(),
                odom_topic: "/m39/odometry".into(),
                covariance_topic: "/m39/odometry_covariance".into(),
                tf_topic: "/m39/transform".into(),
            };
            // Native generations have their own time/history. Rust revision and
            // epoch are bookkeeping, not invented native message-header fields.
            let mut revision = 0;
            for &(time_ns, _, _, _, expected) in ticks {
                inputs += 1;
                let reference::Publication::Observed((pose_bits, linear_bits, angular_bits, wire)) =
                    expected
                else {
                    suppressed += 1;
                    continue;
                };
                revision += 1;
                let pose = pose_bits.map(f64::from_bits);
                let sample = Sample {
                    time: Duration::from_nanos(time_ns),
                    epoch: 7,
                    revision,
                    pose: gz_sim_scene::WorldPose {
                        translation: [pose[0], pose[1], pose[2]],
                        rotation: [pose[3], pose[4], pose[5], pose[6]],
                    },
                    linear: linear_bits.map(f64::from_bits),
                    angular: angular_bits.map(f64::from_bits),
                };
                let (odometry, covariance, transform) = packets(&metadata, 1, &sample)
                    .unwrap_or_else(|| {
                        panic!("actual translated native values rejected: {mode}/{time_ns}")
                    });
                // Output bits are already in the final odometry frame. Neither
                // MODEL->WORLD nor the offsets are applied again by the encoder.
                let encoded = [
                    odometry.encode_to_vec(),
                    covariance.encode_to_vec(),
                    transform.encode_to_vec(),
                ];
                for (channel, (actual, expected)) in encoded.iter().zip(wire).enumerate() {
                    assert_eq!(
                        actual.as_slice(),
                        expected,
                        "actual translated native canonical bytes: {mode}/{time_ns}/channel{channel}"
                    );
                    canonical_packets += 1;
                }
                acquisitions += 1;
            }
        }
        assert_eq!(
            (inputs, acquisitions, suppressed, canonical_packets),
            (1152, 986, 166, 2958)
        );
    }
}
