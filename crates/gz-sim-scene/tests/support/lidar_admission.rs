// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stronger dispositions for only the three retained CPU-lidar inputs.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, LidarPublication, LidarSample, Name, WorldPose, clear_lidar_interest,
    initial_lidar_publications, remove_model, reset_loaded_world, set_lidar_interest,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, time::Duration};

fn identities(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|entity| {
            Some((
                entity.get::<Name>()?.0.clone(),
                entity.get::<EntityId>()?.get(),
            ))
        })
        .collect()
}

fn sample(sim: &Simulation, id: u64, time: Duration, epoch: u64, beams: usize) {
    let entity = entity_by_id(sim.world(), id).unwrap();
    let reading = sim.world().get::<LidarSample>(entity).unwrap();
    assert_eq!((reading.time, reading.epoch), (time, epoch));
    let pose = sim.world().get::<WorldPose>(entity).unwrap();
    assert_eq!(reading.world_pose.translation, pose.translation);
    assert_eq!(reading.world_pose.rotation, pose.rotation);
    assert_eq!(reading.world_pose.rotation, [0.0, 0.0, 0.0, 1.0]);
    if beams == 16 {
        assert_eq!([pose.translation[0], pose.translation[1]], [0.0; 2]);
        assert!(
            pose.translation[2] > 4.0,
            "the retained short fall stays above the ground"
        );
    } else {
        assert_eq!(pose.translation, [0.0; 3]);
    }
    assert_eq!(reading.ranges.len(), beams);
    // The sole collision in lidar-world.sdf belongs to the excluded parent.
    // In typed-sensors.sdf the only other collision is ground below the
    // horizontal acquisition plane; the short fall remains above that ground.
    assert!(reading.ranges.iter().all(|range| *range == f64::INFINITY));
    assert!(!sim.is_faulted());
}

pub fn assert_cpu_lidar(
    mut sim: Simulation,
    xml: &str,
    uri: &str,
    scoped: &str,
    beams: usize,
    initial_position: [f64; 3],
) {
    assert!(matches!(
        (uri, beams),
        ("typed-sensors.sdf", 16) | ("lidar-world.sdf", 2)
    ));
    let parsed = sdformat::load_str(xml, uri, &NoResources).unwrap();
    let sensors: Vec<_> = parsed.worlds[0]
        .models
        .iter()
        .flat_map(|model| model.links.iter().flat_map(|link| &link.sensors))
        .collect();
    assert_eq!(sensors.len(), 1);
    let sensor = sensors[0];
    assert_eq!(sensor.effective_type(), Some(sdformat::SensorType::Lidar));
    assert_eq!(
        sensor.update_rate, 0.0,
        "these exact old inputs request continuous sampling"
    );
    assert_eq!(sensor.source.uri, uri);
    let start = xml.find("<sensor").unwrap();
    assert_eq!(
        sensor.source.line,
        u32::try_from(xml[..start].bytes().filter(|c| *c == b'\n').count() + 1).unwrap()
    );
    let column = xml[..start]
        .rfind('\n')
        .map_or(start + 1, |newline| start - newline);
    assert_eq!(sensor.source.column, u32::try_from(column).unwrap());
    let config = sensor.lidar().unwrap();
    assert_eq!(config.horizontal.samples as usize, beams);
    assert_eq!(config.horizontal.resolution, 1.0);
    assert_eq!(
        (config.range_min, config.range_max, config.range_resolution),
        (0.1, 10.0, 0.01)
    );
    assert_eq!(sim.step_duration(), Duration::from_millis(1));
    let original_ids = identities(&sim);
    let id = original_ids[scoped];
    let entity = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(
        sim.world().get::<EntityKind>(entity),
        Some(&EntityKind::Sensor)
    );
    let publication = sim.world().get::<LidarPublication>(entity).unwrap().clone();
    let entity_name = scoped.strip_prefix("default::").unwrap();
    assert_eq!(publication.entity_name, entity_name);
    assert_eq!(publication.frame_id, entity_name);
    assert_eq!(publication.config.samples, beams);
    assert_eq!(
        (publication.config.angle_min, publication.config.angle_max),
        (config.horizontal.min_angle, config.horizontal.max_angle)
    );
    assert_eq!(
        publication.angle_step,
        (config.horizontal.max_angle - config.horizontal.min_angle) / (beams - 1) as f64
    );
    assert_eq!(
        publication.topic,
        if beams == 16 {
            "/world/default/model/ball/link/body/sensor/unsupported/scan"
        } else {
            "/world/default/model/robot/link/body/sensor/scan/scan"
        }
    );
    assert_eq!(
        sim.world().get::<WorldPose>(entity).unwrap().translation,
        initial_position
    );
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    let counts = sim.scheduled_system_counts();
    set_lidar_interest(&mut sim, []);
    sim.step_exact(2).unwrap();
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    set_lidar_interest(&mut sim, [entity]);
    sim.step_exact(1).unwrap();
    sample(&sim, id, Duration::from_millis(3), 0, beams);
    sim.step_exact(1).unwrap();
    sample(&sim, id, Duration::from_millis(4), 0, beams);
    sim.set_paused(true);
    sim.tick().unwrap();
    sample(&sim, id, Duration::from_millis(4), 0, beams);
    set_lidar_interest(&mut sim, []);
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    sim.step_exact(1).unwrap();
    assert!(sim.world().get::<LidarSample>(entity).is_none());
    clear_lidar_interest(&mut sim);
    sim.step_exact(1).unwrap();
    sample(&sim, id, Duration::from_millis(6), 0, beams);
    assert_eq!(identities(&sim), original_ids);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), original_ids);
    assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 1));
    let restored = entity_by_id(sim.world(), id).unwrap();
    assert!(sim.world().get::<LidarSample>(restored).is_none());
    assert_eq!(
        sim.world().get::<WorldPose>(restored).unwrap().translation,
        initial_position
    );
    assert_eq!(
        sim.world().get::<LidarPublication>(restored).unwrap(),
        &publication
    );
    sim.tick().unwrap();
    assert!(sim.world().get::<LidarSample>(restored).is_none());
    sim.step_exact(1).unwrap();
    sample(&sim, id, Duration::from_millis(1), 1, beams);
    let model_name = if beams == 16 {
        "default::ball"
    } else {
        "default::robot"
    };
    remove_model(&mut sim, original_ids[model_name]).unwrap();
    assert!(entity_by_id(sim.world(), id).is_none());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    assert_eq!(initial_lidar_publications(&sim).unwrap().count(), 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), original_ids);
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(1).unwrap();
    sample(&sim, id, Duration::from_millis(1), 2, beams);
}
