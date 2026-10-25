// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{ImuSample, LoadError, Name, Velocity, WorldPose, load_world};
use sdformat::NoResources;
use std::time::Duration;

const WORLD: &str = include_str!("fixtures/fall.sdf");
fn load(xml: &str) -> Result<gz_sim_core::Simulation, LoadError> {
    load_world(
        xml,
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
}

#[test]
fn sdf_bevy_physics_and_imu_share_end_step_time_and_reset_epoch() {
    let mut sim = load(WORLD).unwrap();
    sim.tick().unwrap();
    assert_eq!(
        sim.world_mut()
            .query::<&ImuSample>()
            .iter(sim.world())
            .count(),
        0
    );
    sim.step_exact(100).unwrap();
    let links: Vec<_> = sim
        .world_mut()
        .query::<(&Name, &WorldPose, &Velocity)>()
        .iter(sim.world())
        .map(|(n, p, v)| (n.0.clone(), *p, *v))
        .collect();
    assert_eq!(links.len(), 2);
    let (_, pose, velocity) = links
        .iter()
        .find(|(n, _, _)| n == "default::ball::body")
        .unwrap();
    assert!((pose.translation[2] - 4.95095).abs() < 0.001);
    assert!((velocity.linear[2] + 0.981).abs() < 1e-10);
    let sample = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap();
    assert_eq!(sample.time, Duration::from_millis(100));
    assert_eq!(sample.epoch, 0);
    assert!(
        sample
            .reading
            .as_ref()
            .unwrap()
            .specific_force
            .iter()
            .all(|v| v.abs() < 1e-10)
    );
    sim.tick().unwrap();
    assert_eq!(
        sim.world_mut()
            .query::<&ImuSample>()
            .single(sim.world())
            .unwrap()
            .time,
        Duration::from_millis(100)
    );
    sim.reset().unwrap();
    assert_eq!(
        sim.world_mut()
            .query::<&ImuSample>()
            .iter(sim.world())
            .count(),
        0
    );
    let reset_height = sim
        .world_mut()
        .query::<(&Name, &WorldPose)>()
        .iter(sim.world())
        .find(|(n, _)| n.0 == "default::ball::body")
        .unwrap()
        .1
        .translation[2];
    assert_eq!(reset_height, 5.0);
    sim.step_exact(1).unwrap();
    let sample = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap();
    assert_eq!(sample.epoch, 1);
    assert!(
        sample
            .reading
            .as_ref()
            .unwrap()
            .specific_force
            .iter()
            .all(|v| v.abs() < 1e-10)
    );
}

#[test]
fn static_parent_and_nested_frame_rotation_reach_ecs_and_sensor() {
    let nested = "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><model name='parent'><static>true</static><pose>1 2 3 0 0 0</pose><model name='nested'><link name='body'><collision name='c'><geometry><sphere/></geometry></collision><sensor name='s' type='imu'><pose>0 0 0 1.5707963267948966 0 0</pose></sensor></link></model></model></world></sdf>";
    let mut sim = load(nested).unwrap();
    sim.step_exact(1).unwrap();
    let pose = sim
        .world_mut()
        .query_filtered::<&WorldPose, bevy_ecs::query::With<gz_sim_scene::BodyHandle>>()
        .single(sim.world())
        .unwrap();
    assert_eq!(pose.translation, [1.0, 2.0, 3.0]);
    let reading = &sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap()
        .reading
        .as_ref()
        .unwrap();
    assert!((reading.specific_force[1] - 9.81).abs() < 1e-12);
}

#[test]
fn unsupported_features_are_errors_and_existing_simulations_are_untouched() {
    let mut existing = load(WORLD).unwrap();
    existing.step_exact(3).unwrap();
    let changes = [
        (
            "<gravity>0 0 -9.81</gravity>",
            "<plugin name='x' filename='x'/>",
            "world plugins",
        ),
        (
            "<model name=\"ball\">",
            "<model name=\"ball\"><plugin name='x' filename='x'/>",
            "model plugins",
        ),
        (
            "<link name=\"body\">",
            "<link name=\"body\"><plugin name='x' filename='x'/>",
            "link plugins",
        ),
        (
            "<link name=\"body\">",
            "<link name=\"body\"><visual name='v'><plugin name='x' filename='x'/></visual>",
            "visual plugins",
        ),
        (
            "<collision name=\"shape\"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>",
            "",
            "collision count",
        ),
        (
            "<sphere><radius>0.5</radius></sphere>",
            "<cylinder/>",
            "collision geometry",
        ),
        (
            "<collision name=\"shape\"><geometry><sphere>",
            "<collision name=\"shape\"><pose>1 0 0 0 0 0</pose><geometry><sphere>",
            "offset collision",
        ),
        (
            "<mass>2</mass>",
            "<mass>2</mass><pose>1 0 0 0 0 0</pose>",
            "offset center",
        ),
        (
            "<update_rate>1000</update_rate>",
            "<update_rate>1e-300</update_rate>",
            "sensor rate",
        ),
        (
            "<always_on>true</always_on>",
            "<pose>1 0 0 0 0 0</pose>",
            "offset IMU",
        ),
        (
            "<always_on>true</always_on>",
            "<imu><orientation_reference_frame><localization>NED</localization></orientation_reference_frame></imu>",
            "IMU orientation",
        ),
        (
            "<always_on>true</always_on>",
            "<imu><angular_velocity><x><noise type='gaussian'><stddev>1e30</stddev></noise></x></angular_velocity></imu>",
            "IMU noise",
        ),
        (
            "<always_on>true</always_on>",
            "<enable_metrics>true</enable_metrics>",
            "sensor metrics",
        ),
        (
            "<link name=\"body\">",
            "<joint name='j' type='fixed'><parent>world</parent><child>body</child></joint><link name=\"body\">",
            "joints",
        ),
    ];
    for (from, to, feature) in changes {
        let error = load(&WORLD.replace(from, to)).err().unwrap();
        assert!(matches!(error, LoadError::Unsupported { .. }), "{error}");
        assert!(error.to_string().contains(feature), "{error}");
    }
    assert_eq!(existing.elapsed(), Duration::from_millis(3));
    let malformed = load("<bad>").err().unwrap();
    assert!(
        matches!(&malformed, LoadError::Sdf(diagnostic) if diagnostic.code == sdformat::ErrorCode::Xml && diagnostic.source.uri == "fall.sdf")
    );
    assert!(malformed.to_string().contains("Xml"));
    let error = load_world(
        WORLD,
        "test",
        "absent",
        Duration::from_millis(1),
        &NoResources,
    )
    .err()
    .unwrap();
    assert!(error.to_string().contains("absent"));
    let error = load_world(WORLD, "test", "default", Duration::ZERO, &NoResources)
        .err()
        .unwrap();
    assert!(error.to_string().contains("positive"));
    let huge_shape = WORLD.replace("<radius>0.5</radius>", "<radius>1e200</radius>");
    let error = load(&huge_shape).err().unwrap();
    assert!(matches!(error, LoadError::Physics(_)));
    assert!(error.to_string().contains("numeric"));
}

#[test]
fn imu_publication_uses_structural_topics_and_worldless_names() {
    use gz_sim_scene::ImuPublication;
    let mut sim = load(WORLD).unwrap();
    let metadata = sim
        .world_mut()
        .query::<&ImuPublication>()
        .single(sim.world())
        .unwrap();
    assert_eq!(metadata.entity_name, "ball::body::imu");
    assert_eq!(metadata.frame_id, "ball::body::imu");
    assert_eq!(
        metadata.topic,
        "/world/default/model/ball/link/body/sensor/imu/imu"
    );
    for topic in ["robot/imu", "/robot/imu"] {
        let xml = WORLD.replace(
            "<always_on>true</always_on>",
            &format!("<topic>{topic}</topic>"),
        );
        let mut sim = load(&xml).unwrap();
        assert_eq!(
            sim.world_mut()
                .query::<&ImuPublication>()
                .single(sim.world())
                .unwrap()
                .topic,
            topic
        );
    }
    let xml = WORLD
        .replace(
            "<model name=\"ball\">",
            "<model name='parent'><model name=\"ball\">",
        )
        .replace("</world>", "</model></world>");
    let mut sim = load(&xml).unwrap();
    let metadata = sim
        .world_mut()
        .query::<&ImuPublication>()
        .single(sim.world())
        .unwrap();
    assert_eq!(metadata.entity_name, "parent::ball::body::imu");
    assert_eq!(
        metadata.topic,
        "/world/default/model/parent/model/ball/link/body/sensor/imu/imu"
    );
}

#[test]
fn imu_orientation_is_relative_to_initial_sensor_world_orientation() {
    use gz_physics_rapier::RapierBackend;
    use gz_sim_scene::BodyHandle;
    let xml = WORLD.replace(
        "<always_on>true</always_on>",
        "<pose>0 0 0 1.5707963267948966 0 0</pose>",
    );
    let mut sim = load(&xml).unwrap();
    sim.step_exact(1).unwrap();
    let initial = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap()
        .reading
        .as_ref()
        .unwrap()
        .orientation;
    assert!(glam::DQuat::from_array(initial).abs_diff_eq(glam::DQuat::IDENTITY, 1e-12));
    let id = sim
        .world_mut()
        .query::<(&Name, &BodyHandle)>()
        .iter(sim.world())
        .find(|(name, _)| name.0 == "default::ball::body")
        .unwrap()
        .1
        .id();
    sim.world_mut()
        .resource_mut::<RapierBackend>()
        .apply_impulse(id, [0.0; 3], [0.0, 0.0, 0.2])
        .unwrap();
    sim.step_exact(100).unwrap();
    let orientation = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap()
        .reading
        .as_ref()
        .unwrap()
        .orientation;
    // World Z rotation becomes positive sensor Y after its initial 90-degree X mounting.
    assert!(
        glam::DQuat::from_array(orientation).abs_diff_eq(glam::DQuat::from_rotation_y(0.1), 1e-5)
    );
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    let reset = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap()
        .reading
        .as_ref()
        .unwrap()
        .orientation;
    assert!(glam::DQuat::from_array(reset).abs_diff_eq(glam::DQuat::IDENTITY, 1e-12));
}

#[test]
fn arbitrary_world_rotation_and_quaternion_sign_do_not_create_local_offsets() {
    let xml = WORLD.replace("<pose>0 0 5 0 0 0</pose>", "<pose>1 2 3 0.1 0.2 0.3</pose>").replace("<collision name=\"shape\"><geometry><sphere>", "<collision name=\"shape\"><pose rotation_format='quat_xyzw'>0 0 0 0 0 0 -1</pose><geometry><sphere>");
    let mut sim = load(&xml).unwrap();
    sim.step_exact(1).unwrap();
    let reading = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap()
        .reading
        .as_ref()
        .unwrap();
    assert!(glam::DQuat::from_array(reading.orientation).abs_diff_eq(glam::DQuat::IDENTITY, 1e-12));
    assert!(reading.specific_force.iter().all(|v| v.abs() < 1e-10));
}

#[test]
fn deferred_sdf_pose_errors_are_rejected_before_scene_construction() {
    let mut existing = load(WORLD).unwrap();
    existing.step_exact(3).unwrap();
    let missing = "<pose relative_to='missing'>0 0 0 0 0 0</pose>";
    let cases = [
        (
            "<collision name=\"shape\">",
            format!("<collision name=\"shape\">{missing}"),
        ),
        ("<always_on>true</always_on>", missing.to_owned()),
        ("<mass>2</mass>", format!("<mass>2</mass>{missing}")),
        (
            "<link name=\"body\">",
            format!("<link name=\"body\"><visual name='v'>{missing}</visual>"),
        ),
    ];
    for (from, to) in cases {
        let xml = WORLD.replace(from, &to);
        // DOM loading accepts these; scene validation also checks inertial references,
        // which native ResolveInertial currently ignores.
        sdformat::load_str(&xml, "fall.sdf", &NoResources).unwrap();
        let error = load(&xml).err().unwrap();
        assert!(matches!(error, LoadError::Sdf(_)), "{error}");
        assert!(error.to_string().contains("missing"), "{error}");
    }
    assert_eq!(existing.elapsed(), Duration::from_millis(3));
    existing.step_exact(1).unwrap();
    assert_eq!(existing.elapsed(), Duration::from_millis(4));
}

#[test]
fn retained_legacy_sdf_worlds_reach_the_same_physics_and_sensor_state() {
    for version in ["1.5", "1.6", "1.7"] {
        let xml = WORLD.replace("version=\"1.12\"", &format!("version=\"{version}\""));
        assert_ne!(xml, WORLD);
        let mut sim = load(&xml).unwrap();
        sim.step_exact(100).unwrap();
        let pose = sim
            .world_mut()
            .query::<(&Name, &WorldPose)>()
            .iter(sim.world())
            .find(|(name, _)| name.0 == "default::ball::body")
            .unwrap()
            .1;
        assert!((pose.translation[2] - 4.95095).abs() < 0.001);
        let sample = sim
            .world_mut()
            .query::<&ImuSample>()
            .single(sim.world())
            .unwrap();
        assert_eq!(sample.time, Duration::from_millis(100));
        assert!(
            sample
                .reading
                .as_ref()
                .unwrap()
                .specific_force
                .iter()
                .all(|v| v.abs() < 1e-10)
        );
    }
}

#[test]
fn imu_frame_fields_follow_native_precedence_after_scoped_naming() {
    use gz_sim_scene::ImuPublication;
    for (fields, expected) in [
        ("", "ball::body::imu"),
        ("<frame_id> body frame </frame_id>", "body frame"),
        ("<gz_frame_id> legacy </gz_frame_id>", "legacy"),
        (
            "<frame_id>main</frame_id><gz_frame_id>ignored</gz_frame_id>",
            "main",
        ),
        ("<frame_id></frame_id><gz_frame_id></gz_frame_id>", ""),
        ("<frame_id>__default__</frame_id>", "__default__"),
    ] {
        let xml = include_str!("fixtures/fall.sdf").replace("<always_on>true</always_on>", fields);
        let sim = load_world(
            &xml,
            "frames.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
        )
        .unwrap();
        let metadata = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<ImuPublication>())
            .unwrap();
        assert_eq!(metadata.entity_name, "ball::body::imu");
        assert_eq!(metadata.frame_id, expected);
    }
}

#[test]
fn native_default_topic_and_xml_whitespace_reach_the_transport_boundary() {
    use gz_sim_scene::ImuPublication;
    for (topic, expected) in [
        (
            "__default__",
            "/world/default/model/ball/link/body/sensor/imu/imu",
        ),
        ("  robot\t\n imu  ", "robot imu"),
        ("\u{a0}imu\u{a0}", "\u{a0}imu\u{a0}"),
    ] {
        let xml = include_str!("fixtures/fall.sdf").replace(
            "<always_on>true</always_on>",
            &format!("<topic>{topic}</topic>"),
        );
        let sim = load_world(
            &xml,
            "topics.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
        )
        .unwrap();
        let metadata = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<ImuPublication>())
            .unwrap();
        assert_eq!(metadata.topic, expected);
    }
}

#[test]
fn explicit_surface_metadata_is_rejected_until_physics_mapping_exists() {
    for surface in [
        "<surface/>",
        "<surface><friction><ode><mu>0.2</mu></ode></friction></surface>",
        "<surface><contact><collide_bitmask>2</collide_bitmask></contact></surface>",
    ] {
        let xml = include_str!("fixtures/fall.sdf").replace(
            "</geometry></collision>",
            &format!("</geometry>{surface}</collision>"),
        );
        let result = load_world(
            &xml,
            "surface.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
        );
        assert!(matches!(
            result,
            Err(LoadError::Unsupported {
                feature: "collision surface parameters",
                ..
            })
        ));
    }
}

#[test]
fn explicit_fluid_added_mass_is_rejected_for_static_and_dynamic_links() {
    for matrix in [
        "<fluid_added_mass/>",
        "<fluid_added_mass><xx>0</xx></fluid_added_mass>",
        "<fluid_added_mass><xx>2</xx><xq>0.5</xq></fluid_added_mass>",
    ] {
        for static_link in [false, true] {
            let xml = if static_link {
                WORLD.replace(
                    "<link name=\"base\">",
                    &format!("<link name='base'><inertial>{matrix}</inertial>"),
                )
            } else {
                WORLD.replace("</inertia>", &format!("</inertia>{matrix}"))
            };
            // DOM metadata remains available even when this backend has no mapping.
            let sdf = sdformat::load_str(&xml, "fluid.sdf", &NoResources).unwrap();
            assert!(
                sdf.worlds
                    .iter()
                    .flat_map(|world| &world.models)
                    .flat_map(|model| &model.links)
                    .any(|link| link.inertial.fluid_added_mass.is_some())
            );
            assert!(matches!(
                load(&xml),
                Err(LoadError::Unsupported {
                    feature: "fluid added mass",
                    ..
                })
            ));
            assert!(load(WORLD).is_ok());
        }
    }
}
