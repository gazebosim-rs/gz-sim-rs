// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::assign_id;
use gz_sim_scene::load_world;
use sdformat::NoResources;

#[test]
fn local_pose_packet_preserves_native_groups_presence_names_and_time() {
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf").replace("<sensor name=", "<visual name='view'><pose>0 0 1 0 0 0</pose><geometry><sphere/></geometry></visual><sensor name=");
    let mut sim = load_world(
        &xml,
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    sim.step_exact(3).unwrap();
    let message = packet(sim.world(), sim.elapsed()).unwrap();
    assert_eq!(
        message.header.as_ref().unwrap().stamp,
        Some(gz_msgs::Time {
            sec: 0,
            nsec: 3_000_000
        })
    );
    assert!(message.header.as_ref().unwrap().data.is_empty());
    assert_eq!(
        message
            .pose
            .iter()
            .map(|p| p.name.as_str())
            .collect::<Vec<_>>(),
        ["ground", "ball", "base", "body", "view"]
    );
    let mut seen = std::collections::BTreeSet::new();
    for pose in &message.pose {
        assert!(pose.id > 0 && seen.insert(pose.id));
        let entity = gz_sim_core::entity_by_id(sim.world(), pose.id.into()).unwrap();
        assert_eq!(sim.world().get::<LocalName>(entity).unwrap().0, pose.name);
        assert!(pose.header.is_none());
        assert!(pose.position.as_ref().unwrap().header.is_none());
        assert!(pose.orientation.as_ref().unwrap().header.is_none());
    }
    let height = message.pose[1].position.as_ref().unwrap().z;
    assert!(height < 5.0);
    assert_eq!(message.pose[3].position.as_ref().unwrap().z, 0.0);
    assert_eq!(message.pose[4].position.as_ref().unwrap().z, 1.0);
    assert_eq!(message.pose[2].position.as_ref().unwrap().z, -0.5);
    assert_eq!(message.pose[1].orientation.as_ref().unwrap().w, 1.0);
    assert!(matches!(
        packet(sim.world(), Duration::MAX),
        Err(Error::TimeRange)
    ));
}

#[test]
fn invalid_pose_data_and_legacy_id_overflow_reject_the_whole_packet() {
    let valid = WorldPose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    assert_eq!(
        record(u64::from(u32::MAX), "limit", valid).unwrap().id,
        u32::MAX
    );
    for id in [0, u64::from(u32::MAX) + 1, u64::MAX] {
        assert!(matches!(record(id, "bad", valid), Err(Error::PoseIdRange)));
    }
    for pose in [
        WorldPose {
            translation: [f64::NAN, 0.0, 0.0],
            ..valid
        },
        WorldPose {
            rotation: [f64::INFINITY; 4],
            ..valid
        },
        WorldPose {
            rotation: [0.0; 4],
            ..valid
        },
    ] {
        assert!(matches!(record(1, "bad", pose), Err(Error::PoseData)));
    }
    let mut world = World::new();
    let entity = world.spawn(EntityKind::Model).id();
    assert!(matches!(
        packet(&world, Duration::ZERO),
        Err(Error::PoseData)
    ));
    assign_id(&mut world, entity).unwrap();
    assert!(matches!(
        packet(&world, Duration::ZERO),
        Err(Error::PoseData)
    ));
    world.entity_mut(entity).insert(LocalName("model".into()));
    assert!(matches!(
        packet(&world, Duration::ZERO),
        Err(Error::PoseData)
    ));
    world.entity_mut(entity).insert(LocalPose(valid));
    assert_eq!(packet(&world, Duration::ZERO).unwrap().pose.len(), 1);
    let moved = world.entity_mut(entity).take::<EntityId>().unwrap();
    world.despawn(entity);
    let foreign = world
        .spawn((
            moved,
            EntityKind::Model,
            LocalName("foreign".into()),
            LocalPose(valid),
        ))
        .id();
    assert!(matches!(
        packet(&world, Duration::ZERO),
        Err(Error::PoseData)
    ));
    world.despawn(foreign);
    assert!(packet(&world, Duration::ZERO).unwrap().pose.is_empty());
}

#[test]
fn native_wall_rate_ceiling_first_frame_and_explicit_policy_boundaries() {
    let mut rate = PublicationRate::default();
    let now = Instant::now();
    let period = Duration::from_nanos(16_666_667);
    assert_eq!(rate.period(), period);
    assert!(rate.ready(now));
    rate.attempted(now);
    assert!(!rate.ready(now));
    assert!(!rate.ready(now + period - Duration::from_nanos(1)));
    assert!(rate.ready(now + period));
    assert!(!rate.ready(now - Duration::from_nanos(1)));
    rate.set_period(Duration::ZERO);
    assert!(rate.ready(now));
    rate.attempted(now);
    assert!(rate.ready(now));
    rate.set_period(Duration::MAX);
    assert!(rate.ready(now));
    rate.attempted(now);
    assert!(!rate.ready(now + Duration::from_secs(1)));
}
