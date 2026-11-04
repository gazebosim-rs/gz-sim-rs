// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;
const XML: &str = include_str!("../fixtures/planar_lidar.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_engine(
        xml,
        "lidar.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}
#[test]
fn metric_off_axis_returns_pose_cadence_pause_demand_removal_and_reset() {
    let mut sim = load(XML);
    let (sensor, sensor_id) = named(&sim, "default::scanner::body::laser");
    let (_, model_id) = named(&sim, "default::scanner");
    let counts = sim.scheduled_system_counts();
    install(&mut sim);
    assert_eq!(sim.scheduled_system_counts(), counts);
    let publication = sim.world().get::<LidarPublication>(sensor).unwrap();
    assert_eq!(publication.topic, "/scan");
    assert_eq!(publication.frame_id, "laser_frame");
    assert_eq!(publication.entity_name, "scanner::body::laser");
    assert_eq!(publication.config.samples, 3);
    assert!((publication.angle_step - std::f64::consts::FRAC_PI_4).abs() < 1e-15);
    set_lidar_interest(&mut sim, []);
    sim.step_exact(3).unwrap();
    assert!(sim.world().get::<LidarSample>(sensor).is_none());
    set_lidar_interest(&mut sim, [sensor, sensor]);
    assert_eq!(sim.world().resource::<Interest>().0, [sensor_id]);
    sim.step_exact(1).unwrap();
    let sample = sim.world().get::<LidarSample>(sensor).unwrap();
    assert_eq!(sample.time, Duration::from_millis(4));
    assert_eq!(sample.epoch, 0);
    assert_eq!(sample.world_pose.translation, [0.0; 3]);
    assert_eq!(sample.world_pose.rotation, [0.0, 0.0, 0.0, 1.0]);
    for (actual, expected) in
        sample
            .ranges
            .iter()
            .zip([4.0 * 2.0_f64.sqrt(), 4.0, 4.0 * 2.0_f64.sqrt()])
    {
        assert!((actual - expected).abs() < 1e-10);
    }
    sim.step_exact(5).unwrap();
    assert_eq!(
        sim.world().get::<LidarSample>(sensor).unwrap().time,
        Duration::from_millis(4)
    );
    sim.set_paused(true);
    for _ in 0..3 {
        sim.tick().unwrap();
    }
    assert_eq!(
        sim.world().get::<LidarSample>(sensor).unwrap().time,
        Duration::from_millis(4)
    );
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<LidarSample>(sensor).unwrap().time,
        Duration::from_millis(10)
    );
    set_lidar_interest(&mut sim, []);
    assert!(sim.world().get::<LidarSample>(sensor).is_none());
    clear_lidar_interest(&mut sim);
    sim.step_exact(10).unwrap();
    assert_eq!(
        sim.world().get::<LidarSample>(sensor).unwrap().time,
        Duration::from_millis(20)
    );
    remove_model(&mut sim, model_id).unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_lidar_interest(&mut sim, [sensor]);
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(initial_lidar_publications(&sim).unwrap().count(), 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = gz_sim_core::entity_by_id(sim.world(), sensor_id).unwrap();
    assert!(sim.world().get::<LidarSample>(restored).is_none());
    assert!(sim.world().contains_resource::<Installed>());
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().get::<LidarSample>(restored).unwrap().epoch, 1);
    assert_eq!(
        sim.world().get::<LidarSample>(restored).unwrap().time,
        Duration::from_millis(1)
    );
}
#[test]
fn absent_family_default_topics_alias_frame_rotated_offset_and_misses() {
    let mut absent = load(&without_sensor());
    let counts = absent.scheduled_system_counts();
    install(&mut absent);
    retire_if_unused(&mut absent).unwrap();
    set_lidar_interest(&mut absent, []);
    clear_lidar_interest(&mut absent);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert_eq!(initial_lidar_publications(&absent).unwrap().count(), 0);
    let empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_lidar_publications(&empty),
        Err(SceneResetError::MissingInitialState)
    ));
    let xml = XML
        .replace("type=\"lidar\"", "type=\"ray\"")
        .replace(
            "<topic>/scan</topic><frame_id>laser_frame</frame_id>",
            "<gz_frame_id>legacy</gz_frame_id>",
        )
        .replace(
            "<pose>0 0 0 0 0 0</pose>",
            "<pose>0 1 0 0 0 3.141592653589793</pose>",
        );
    let mut sim = load(&xml);
    let (sensor, _) = named(&sim, "default::scanner::body::laser");
    let publication = sim.world().get::<LidarPublication>(sensor).unwrap();
    assert_eq!(publication.frame_id, "legacy");
    assert_eq!(
        publication.topic,
        "/world/default/model/scanner/link/body/sensor/laser/scan"
    );
    sim.step_exact(1).unwrap();
    let scan = sim.world().get::<LidarSample>(sensor).unwrap();
    assert_eq!(scan.world_pose.translation, [0.0, 1.0, 0.0]);
    assert!(scan.ranges.iter().all(|value| *value == f64::INFINITY));
    let fallback = load(&XML.replace("<frame_id>laser_frame</frame_id>", ""));
    assert_eq!(
        fallback
            .world()
            .iter_entities()
            .find_map(|entity| entity.get::<LidarPublication>())
            .unwrap()
            .frame_id,
        "scanner::body::laser"
    );
}
#[test]
fn unsupported_sensor_domains_fail_admission_explicitly() {
    for (from, to) in [
        ("type=\"lidar\"", "type=\"gpu_lidar\""),
        ("<samples>3</samples>", "<samples>129</samples>"),
        ("<samples>3</samples>", "<samples>1</samples>"),
        ("<resolution>1</resolution>", "<resolution>2</resolution>"),
        (
            "</horizontal></scan>",
            "</horizontal><vertical><samples>2</samples></vertical></scan>",
        ),
        (
            "<resolution>0.01</resolution>",
            "<resolution>-1</resolution>",
        ),
        (
            "</range></lidar>",
            "</range><noise type=\"gaussian\"><stddev>0.1</stddev></noise></lidar>",
        ),
        (
            "</range></lidar>",
            "</range><visibility_mask>1</visibility_mask></lidar>",
        ),
        ("<min>0.1</min>", "<min>11</min>"),
        (
            "<max_angle>0.7853981633974483</max_angle>",
            "<max_angle>-1</max_angle>",
        ),
    ] {
        assert!(
            load_world_with_engine(
                &XML.replace(from, to),
                "bad.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{from} -> {to}"
        );
    }
    let start = XML.find("<sensor name").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + "</sensor>".len();
    let doubled = XML.replace(
        &XML[start..end],
        &format!(
            "{}{}",
            &XML[start..end],
            XML[start..end].replace("name=\"laser\"", "name=\"second\"")
        ),
    );
    assert!(
        load_world_with_engine(
            &doubled,
            "two.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    assert!(
        load_world_with_engine(
            XML,
            "tpe.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
}

#[test]
fn moving_offset_sensor_uses_committed_pose_and_continuous_range_clipping() {
    let xml = XML
        .replacen("<static>true</static>", "<static>false</static>", 1)
        .replace("<pose>0 0 0 0 0 0</pose>", "<pose>0.5 0 0 0 0 0</pose>")
        .replace(
            "<update_rate>100</update_rate>",
            "<update_rate>0</update_rate>",
        );
    let mut sim = load(&xml);
    let (sensor, _) = named(&sim, "default::scanner::body::laser");
    let (_, body) = named(&sim, "default::scanner::body");
    queue_link_world_velocity(&mut sim, body, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    for index in 1..=4 {
        sim.step_exact(1).unwrap();
        let sample = sim.world().get::<LidarSample>(sensor).unwrap();
        assert_eq!(sample.time, Duration::from_millis(index));
        let x = 0.5 + index as f64 * 0.001;
        assert!((sample.world_pose.translation[0] - x).abs() < 1e-12);
        assert!((sample.ranges[1] - (4.0 - x)).abs() < 1e-10);
    }
    let mut near = load(&XML.replace("<min>0.1</min>", "<min>6</min>"));
    near.step_exact(1).unwrap();
    assert!(
        near.world()
            .iter_entities()
            .find_map(|entity| entity.get::<LidarSample>())
            .unwrap()
            .ranges
            .iter()
            .all(|distance| *distance == 6.0)
    );
    let mut far = load(&XML.replace("<max>10</max>", "<max>3</max>"));
    far.step_exact(1).unwrap();
    assert!(
        far.world()
            .iter_entities()
            .find_map(|entity| entity.get::<LidarSample>())
            .unwrap()
            .ranges
            .iter()
            .all(|distance| *distance == f64::INFINITY)
    );
}
