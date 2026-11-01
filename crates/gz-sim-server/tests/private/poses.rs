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
        Some(gz_sim_proto::msgs::Time {
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

#[test]
fn complete_world_lights_publish_native_visuals_and_reject_partial_metadata() {
    let xml = "<sdf version='1.12'><world name='default'><light name='sun' type='directional'><light_on>false</light_on></light></world></sdf>";
    let mut sim = load_world(
        xml,
        "pose-light.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    let message = packet(sim.world(), Duration::ZERO).unwrap();
    assert_eq!(
        message.header.as_ref().unwrap().stamp,
        Some(gz_sim_proto::msgs::Time { sec: 0, nsec: 0 })
    );
    assert!(message.header.as_ref().unwrap().data.is_empty());
    assert_eq!(message.pose.len(), 2);
    assert_eq!(
        message
            .pose
            .iter()
            .map(|pose| pose.name.as_str())
            .collect::<Vec<_>>(),
        ["sunVisual", "sun"]
    );
    for pose in &message.pose {
        assert_ne!(pose.id, 0);
        let entity = entity_by_id(sim.world(), u64::from(pose.id)).unwrap();
        assert_eq!(sim.world().get::<LocalName>(entity).unwrap().0, pose.name);
        assert_eq!(pose.position, Some(vector3([0.0; 3])));
        assert_eq!(pose.orientation, Some(quaternion([0.0, 0.0, 0.0, 1.0])));
    }
    let light = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<WorldLight>())
        .unwrap()
        .id();
    let metadata = sim
        .world_mut()
        .entity_mut(light)
        .take::<WorldLight>()
        .unwrap();
    assert!(matches!(
        packet(sim.world(), Duration::ZERO),
        Err(Error::PoseLighting)
    ));
    sim.world_mut()
        .entity_mut(light)
        .remove::<EntityKind>()
        .insert(metadata);
    assert!(matches!(
        packet(sim.world(), Duration::ZERO),
        Err(Error::PoseLighting)
    ));
    sim.world_mut().entity_mut(light).despawn();
    assert!(packet(sim.world(), Duration::ZERO).unwrap().pose.is_empty());
}

#[test]
fn cold_light_validation_preserves_name_and_metadata_error_order() {
    let xml = "<sdf version='1.12'><world name='default'><light name='sun' type='directional'/></world></sdf>";
    let sim = load_world(
        xml,
        "cold-light.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    let invalid = crate::ServerOptions {
        capacity: 0,
        ..Default::default()
    };
    assert!(matches!(
        crate::Server::new(sim, "", invalid, std::num::NonZeroUsize::new(1).unwrap()),
        Err(Error::Setup(crate::SetupError::InvalidName))
    ));
    let mut malformed = load_world(
        xml,
        "cold-light.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    let light = malformed
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<WorldLight>())
        .unwrap()
        .id();
    malformed
        .world_mut()
        .entity_mut(light)
        .remove::<WorldLight>();
    assert!(matches!(
        crate::Server::new(
            malformed,
            "",
            crate::ServerOptions {
                capacity: 0,
                ..Default::default()
            },
            std::num::NonZeroUsize::new(1).unwrap()
        ),
        Err(Error::PoseLighting)
    ));
}

#[test]
fn incomplete_native_light_children_never_produce_partial_packets() {
    for fault in 0..16 {
        let mut sim = load_world(
            "<sdf version='1.12'><world name='default'><light name='sun' type='directional'/></world></sdf>",
            "invalid-light-child.sdf", "default", Duration::from_millis(1), &NoResources,
        ).unwrap();
        let light = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<WorldLight>())
            .unwrap()
            .id();
        let child = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::LightVisual))
            .unwrap()
            .id();
        let world = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        match fault {
            0 => {
                sim.world_mut().entity_mut(child).despawn();
            }
            1 => {
                sim.world_mut()
                    .spawn((ChildOf(light), EntityKind::LightVisual));
            }
            2 => {
                sim.world_mut().get_mut::<LocalName>(child).unwrap().0 = "wrong".into();
            }
            3 => {
                sim.world_mut()
                    .entity_mut(child)
                    .remove::<VisualProperties>();
            }
            4 => {
                sim.world_mut()
                    .get_mut::<VisualProperties>(child)
                    .unwrap()
                    .0
                    .cast_shadows = true;
            }
            5 => {
                sim.world_mut()
                    .get_mut::<VisualProperties>(child)
                    .unwrap()
                    .0
                    .transparency = f32::NAN;
            }
            6 => {
                sim.world_mut().entity_mut(child).remove::<ChildOf>();
            }
            7 => {
                sim.world_mut().entity_mut(child).insert(ChildOf(world));
            }
            8 => {
                sim.world_mut()
                    .get_mut::<LocalPose>(child)
                    .unwrap()
                    .0
                    .translation = [1.0, 0.0, 0.0];
            }
            9 => {
                sim.world_mut().entity_mut(child).insert(EntityKind::Visual);
            }
            10 => {
                sim.world_mut().entity_mut(child).remove::<LocalName>();
            }
            11 => {
                sim.world_mut().entity_mut(child).remove::<LocalPose>();
            }
            12 => {
                sim.world_mut().entity_mut(light).remove::<LocalName>();
            }
            13 => {
                sim.world_mut().entity_mut(world).insert(EntityKind::Model);
            }
            14 => {
                sim.world_mut().entity_mut(light).remove::<ChildOf>();
            }
            15 => {
                let quarter = std::f64::consts::FRAC_1_SQRT_2;
                sim.world_mut()
                    .get_mut::<LocalPose>(child)
                    .unwrap()
                    .0
                    .rotation = [0.0, 0.0, quarter, quarter];
            }
            _ => unreachable!(),
        }
        assert!(
            matches!(
                packet(sim.world(), Duration::ZERO),
                Err(Error::PoseLighting)
            ),
            "fault {fault}"
        );
    }
}

#[test]
fn native_light_child_identity_cannot_be_reinstalled_without_ownership() {
    let mut sim = load_world("<sdf version='1.12'><world name='default'><light name='sun' type='directional'/></world></sdf>", "light-id.sdf", "default", Duration::from_millis(1), &NoResources).unwrap();
    let child = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::LightVisual))
        .unwrap()
        .id();
    let identity = sim
        .world_mut()
        .entity_mut(child)
        .take::<EntityId>()
        .unwrap();
    sim.world_mut().entity_mut(child).insert(identity);
    assert!(matches!(
        packet(sim.world(), Duration::ZERO),
        Err(Error::PoseData)
    ));
}

#[test]
fn light_metadata_on_a_world_rejects_the_whole_packet() {
    let mut sim = load_world(
        "<sdf version='1.12'><world name='default'/></sdf>",
        "misplaced-light.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    assert!(packet(sim.world(), Duration::ZERO).unwrap().pose.is_empty());
    let world = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::World))
        .unwrap()
        .id();
    sim.world_mut()
        .entity_mut(world)
        .insert(WorldLight(Box::default()));
    assert!(matches!(
        packet(sim.world(), Duration::ZERO),
        Err(Error::PoseLighting)
    ));
    sim.world_mut().entity_mut(world).remove::<WorldLight>();
    assert!(packet(sim.world(), Duration::ZERO).unwrap().pose.is_empty());
}
