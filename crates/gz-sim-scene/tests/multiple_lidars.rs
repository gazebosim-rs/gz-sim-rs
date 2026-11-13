// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    LidarPublication, LidarSample, LoadError, Name, PhysicsEngine, WorldPose, clear_lidar_interest,
    initial_lidar_publications, load_world_seeded, queue_link_world_wrench, remove_model,
    reset_loaded_world, set_lidar_interest,
};
use sdformat::NoResources;
use std::time::Duration;

type Scan = (Duration, u64, [f64; 3], [f64; 4], Vec<f64>);

fn sensor(index: usize, beams: usize, rate: u32, noise: &str, offset: f64) -> String {
    format!(
        "<sensor name='laser{index}' type='lidar'><pose>{offset} 0 0 0 0 0</pose><update_rate>{rate}</update_rate><topic>/m54/scan{index}</topic><frame_id>rig{index}/body/laser{index}</frame_id><lidar><scan><horizontal><samples>{beams}</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0</min><max>20</max><resolution>0.01</resolution></range>{noise}</lidar></sensor>"
    )
}
fn rig(index: usize, x: f64, y: f64, sensors: &str) -> String {
    format!(
        "<model name='rig{index}'><pose>{x} {y} 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='body'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>{sensors}</link></model>"
    )
}
fn envelope(rigs: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{rigs}<model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 200 200</size></box></geometry></collision></link></model></world></sdf>"
    )
}
fn xml(count: usize, beams: usize, noise: &str) -> String {
    let rigs = (0..count)
        .map(|i| {
            rig(
                i,
                0.0,
                3.0 * i as f64,
                &sensor(i, beams, [100, 50, 25][i % 3], noise, 0.5),
            )
        })
        .collect::<String>();
    envelope(&rigs)
}
fn try_load(source: &str, seed: [u8; 32]) -> Result<Simulation, LoadError> {
    load_world_seeded(
        source,
        "multiple-lidars.sdf",
        "default",
        None,
        &NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
}
fn load(source: &str) -> Simulation {
    try_load(source, [54; 32]).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn owner(sim: &Simulation, index: usize) -> Entity {
    named(sim, &format!("default::rig{index}::body::laser{index}")).0
}
fn scan(sim: &Simulation, entity: Entity) -> Scan {
    let reading = sim.world().get::<LidarSample>(entity).unwrap();
    (
        reading.time,
        reading.epoch,
        reading.world_pose.translation,
        reading.world_pose.rotation,
        reading.ranges.clone(),
    )
}
fn wall_geometry(sim: &Simulation, entity: Entity) {
    let actual = scan(sim, entity);
    let metadata = sim.world().get::<LidarPublication>(entity).unwrap();
    assert_eq!(actual.4.len(), metadata.config.samples);
    let rotation = DQuat::from_array(actual.3);
    for (index, range) in actual.4.iter().enumerate() {
        let angle = if index + 1 == actual.4.len() {
            metadata.config.angle_max
        } else {
            metadata.config.angle_min + index as f64 * metadata.angle_step
        };
        let direction = rotation * DVec3::new(angle.cos(), angle.sin(), 0.0);
        assert!(direction.x > 0.0);
        let expected = (4.0 - actual.2[0]) / direction.x;
        assert!(expected > metadata.config.range_min && expected < metadata.config.range_max);
        assert!((actual.2[1] + expected * direction.y).abs() < 100.0);
        assert!((actual.2[2] + expected * direction.z).abs() < 100.0);
        assert!(
            (range - expected).abs() < 1e-10,
            "beam{index}: {range} vs {expected}"
        );
    }
    if actual.0 == sim.elapsed() {
        let pose = sim.world().get::<WorldPose>(entity).unwrap();
        assert_eq!(actual.2, pose.translation);
        assert_eq!(actual.3, pose.rotation);
    }
}

#[test]
fn dynamic_pair_real_forces_and_rotation_use_acquisition_geometry_and_loaded_reset() {
    let source = xml(2, 3, "");
    let mut sim = load(&source);
    let ids = [0, 1].map(|i| named(&sim, &format!("default::rig{i}::body::laser{i}")).1);
    let bodies = [0, 1].map(|i| named(&sim, &format!("default::rig{i}::body")).1);
    let initial = bodies.map(|id| {
        *sim.world()
            .get::<WorldPose>(entity_by_id(sim.world(), id).unwrap())
            .unwrap()
    });
    for (index, id) in bodies.into_iter().enumerate() {
        queue_link_world_wrench(
            &mut sim,
            id,
            [8.0, 1.0 + index as f64, 0.0],
            [0.1, 0.2, 1.0 + index as f64],
        )
        .unwrap();
    }
    let mut held: [Option<Scan>; 2] = [None, None];
    let mut stamps: [Vec<Duration>; 2] = [Vec::new(), Vec::new()];
    for _ in 0..21 {
        sim.step_exact(1).unwrap();
        for (index, id) in ids.into_iter().enumerate() {
            let entity = entity_by_id(sim.world(), id).unwrap();
            let reading = scan(&sim, entity);
            if reading.0 == sim.elapsed() {
                stamps[index].push(reading.0);
            } else {
                assert_eq!(Some(&reading), held[index].as_ref());
            }
            wall_geometry(&sim, entity);
            held[index] = Some(reading);
        }
    }
    assert_eq!(stamps[0], [1, 10, 20].map(Duration::from_millis));
    assert_eq!(stamps[1], [1, 20].map(Duration::from_millis));
    for (id, initial) in bodies.into_iter().zip(initial) {
        let current = sim
            .world()
            .get::<WorldPose>(entity_by_id(sim.world(), id).unwrap())
            .unwrap();
        assert_ne!(current.translation, initial.translation);
        assert_ne!(current.rotation, initial.rotation);
    }
    let handles = ids.map(|id| entity_by_id(sim.world(), id).unwrap());
    let metadata =
        handles.map(|entity| sim.world().get::<LidarPublication>(entity).unwrap().clone());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 1));
    for ((id, handle), metadata) in ids.into_iter().zip(handles).zip(metadata) {
        let restored = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(restored, handle);
        assert_eq!(
            sim.world().get::<LidarPublication>(restored),
            Some(&metadata)
        );
        assert!(sim.world().get::<LidarSample>(restored).is_none());
    }
    sim.step_exact(1).unwrap();
    for id in ids {
        wall_geometry(&sim, entity_by_id(sim.world(), id).unwrap());
    }
}

#[test]
fn absent_single_eight_and_maximum_1024_beams_preserve_one_callback_pair_and_refusals() {
    let source = xml(8, 128, "");
    let mut absent_source = source.clone();
    for i in 0..8 {
        absent_source = absent_source.replace(&sensor(i, 128, [100, 50, 25][i % 3], "", 0.5), "");
    }
    let mut absent = load(&absent_source);
    let counts = absent.scheduled_system_counts();
    set_lidar_interest(&mut absent, []);
    clear_lidar_interest(&mut absent);
    absent.step_exact(2).unwrap();
    absent.reset().unwrap();
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert_eq!(initial_lidar_publications(&absent).unwrap().count(), 0);
    assert!(
        !absent
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<LidarSample>())
    );
    let mut single = load(&xml(1, 128, ""));
    single.step_exact(1).unwrap();
    wall_geometry(&single, owner(&single, 0));
    let mut eight = load(&source);
    assert_eq!(
        eight.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert_eq!(initial_lidar_publications(&eight).unwrap().count(), 8);
    eight.step_exact(1).unwrap();
    let beams: usize = (0..8).map(|i| scan(&eight, owner(&eight, i)).4.len()).sum();
    assert_eq!(beams, 1024);
    for i in 0..8 {
        wall_geometry(&eight, owner(&eight, i));
    }
    assert_eq!(
        scan(&single, owner(&single, 0)),
        scan(&eight, owner(&eight, 0))
    );
    assert!(matches!(
        try_load(&xml(9, 2, ""), [54; 32]),
        Err(LoadError::Unsupported {
            feature: "multiple CPU lidars",
            ..
        })
    ));
    let invalid = source.replace(
        "<sensor name='laser7' type='lidar'>",
        "<sensor name='laser7' type='lidar'><enable_metrics>true</enable_metrics>",
    );
    assert!(matches!(
        try_load(&invalid, [54; 32]),
        Err(LoadError::Unsupported {
            feature: "sensor metrics",
            ..
        })
    ));
}

#[test]
fn per_owner_demand_keeps_deadlines_and_held_scans_through_pause_and_loaded_reset() {
    let mut sim = load(&xml(2, 3, ""));
    let [a, b] = [0, 1].map(|i| owner(&sim, i));
    let ids = [a, b].map(|entity| sim.world().get::<EntityId>(entity).unwrap().get());
    sim.step_exact(1).unwrap();
    set_lidar_interest(&mut sim, [a, a]);
    assert!(sim.world().get::<LidarSample>(b).is_none());
    sim.step_exact(9).unwrap();
    assert_eq!(scan(&sim, a).0, Duration::from_millis(10));
    set_lidar_interest(&mut sim, [a, b]);
    let held = scan(&sim, a);
    sim.step_exact(9).unwrap();
    assert_eq!(scan(&sim, a), held);
    assert!(sim.world().get::<LidarSample>(b).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(scan(&sim, a).0, Duration::from_millis(20));
    assert_eq!(scan(&sim, b).0, Duration::from_millis(20));
    set_lidar_interest(&mut sim, [a]);
    sim.step_exact(5).unwrap();
    clear_lidar_interest(&mut sim);
    sim.step_exact(14).unwrap();
    assert_eq!(scan(&sim, a).0, Duration::from_millis(30));
    assert!(sim.world().get::<LidarSample>(b).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(scan(&sim, b).0, Duration::from_millis(40));
    let held = [a, b].map(|entity| scan(&sim, entity));
    sim.set_paused(true);
    for _ in 0..3 {
        sim.tick().unwrap();
    }
    assert_eq!([a, b].map(|entity| scan(&sim, entity)), held);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 1));
    sim.tick().unwrap();
    for id in ids {
        assert!(
            sim.world()
                .get::<LidarSample>(entity_by_id(sim.world(), id).unwrap())
                .is_none()
        );
    }
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    for id in ids {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert_eq!(
            (scan(&sim, entity).0, scan(&sim, entity).1),
            (Duration::from_millis(1), 1)
        );
        wall_geometry(&sim, entity);
    }
}

#[test]
fn same_input_noise_streams_and_appended_owner_preserve_identity_draws_and_reset() {
    let noise = "<noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise>";
    let sensors = format!(
        "{}{}",
        sensor(0, 3, 100, noise, 0.5),
        sensor(1, 3, 100, noise, 0.5)
    );
    let identical = envelope(&rig(0, 0.0, 0.0, &sensors));
    let mut both = load(&identical);
    let mut selected = load(&identical);
    let a = named(&both, "default::rig0::body::laser0").0;
    let b = named(&both, "default::rig0::body::laser1").0;
    let selected_a = named(&selected, "default::rig0::body::laser0").0;
    let selected_b = named(&selected, "default::rig0::body::laser1").0;
    assert_eq!(
        both.world().get::<WorldPose>(a).unwrap().translation,
        both.world().get::<WorldPose>(b).unwrap().translation
    );
    assert_eq!(
        both.world().get::<WorldPose>(a).unwrap().rotation,
        both.world().get::<WorldPose>(b).unwrap().rotation
    );
    both.step_exact(1).unwrap();
    let first_b = scan(&both, b);
    assert_ne!(scan(&both, a).4, first_b.4);
    set_lidar_interest(&mut selected, [selected_a]);
    selected.step_exact(20).unwrap();
    assert!(selected.world().get::<LidarSample>(selected_b).is_none());
    set_lidar_interest(&mut selected, [selected_a, selected_b]);
    selected.step_exact(1).unwrap();
    assert_eq!(scan(&selected, selected_b).0, Duration::from_millis(21));
    assert_eq!(scan(&selected, selected_b).4, first_b.4);
    let identity = both.world().get::<EntityId>(b).unwrap().get();
    reset_loaded_world(&mut both, |_| ()).unwrap();
    both.step_exact(1).unwrap();
    let replay = scan(&both, entity_by_id(both.world(), identity).unwrap());
    assert_eq!(replay.1, 1);
    assert_eq!(
        (replay.0, replay.2, replay.3, replay.4),
        (first_b.0, first_b.2, first_b.3, first_b.4)
    );
    let mut single = load(&xml(1, 3, noise));
    let mut appended = load(&xml(2, 3, noise));
    let first_id = single
        .world()
        .get::<EntityId>(owner(&single, 0))
        .unwrap()
        .get();
    assert_eq!(
        first_id,
        appended
            .world()
            .get::<EntityId>(owner(&appended, 0))
            .unwrap()
            .get()
    );
    for _ in 0..31 {
        single.step_exact(1).unwrap();
        appended.step_exact(1).unwrap();
        assert_eq!(
            scan(&single, owner(&single, 0)),
            scan(&appended, owner(&appended, 0))
        );
    }
}

#[test]
fn obstacle_and_owner_removal_keep_peer_then_retire_last_and_loaded_restore() {
    let mut sim = load(&xml(2, 3, ""));
    let ids = [0, 1].map(|i| named(&sim, &format!("default::rig{i}::body::laser{i}")).1);
    let models = [0, 1].map(|i| named(&sim, &format!("default::rig{i}")).1);
    let counts = sim.scheduled_system_counts();
    sim.step_exact(1).unwrap();
    let handles = ids.map(|id| entity_by_id(sim.world(), id).unwrap());
    let wall = named(&sim, "default::wall").1;
    remove_model(&mut sim, wall).unwrap();
    sim.step_exact(19).unwrap();
    for id in ids {
        let scan = scan(&sim, entity_by_id(sim.world(), id).unwrap());
        assert_eq!(scan.0, Duration::from_millis(20));
        assert!(scan.4.iter().all(|range| *range == f64::INFINITY));
    }
    remove_model(&mut sim, models[0]).unwrap();
    assert!(entity_by_id(sim.world(), ids[0]).is_none());
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(20).unwrap();
    assert_eq!(
        scan(&sim, entity_by_id(sim.world(), ids[1]).unwrap()).0,
        Duration::from_millis(40)
    );
    sim.reset().unwrap();
    assert!(entity_by_id(sim.world(), ids[0]).is_none());
    assert!(entity_by_id(sim.world(), wall).is_none());
    sim.step_exact(1).unwrap();
    assert!(
        scan(&sim, entity_by_id(sim.world(), ids[1]).unwrap())
            .4
            .iter()
            .all(|range| *range == f64::INFINITY)
    );
    remove_model(&mut sim, models[1]).unwrap();
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_lidar_interest(&mut sim, handles);
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<LidarSample>())
    );
    assert_eq!(initial_lidar_publications(&sim).unwrap().count(), 2);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), counts);
    for (id, handle) in ids.into_iter().zip(handles) {
        let restored = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(restored, handle);
        assert!(sim.world().get::<LidarSample>(restored).is_none());
    }
    sim.step_exact(1).unwrap();
    for id in ids {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert_eq!(scan(&sim, entity).1, 2);
        wall_geometry(&sim, entity);
    }
}

#[test]
fn each_ray_excludes_only_its_own_body_and_peer_colliders_remain_visible() {
    let source = envelope(&format!(
        "{}{}",
        rig(0, 0.0, 0.0, &sensor(0, 3, 100, "", 0.0)),
        rig(1, 2.0, 0.0, &sensor(1, 3, 100, "", 0.0))
    ));
    let mut sim = load(&source);
    sim.step_exact(1).unwrap();
    let first = scan(&sim, owner(&sim, 0));
    let second = scan(&sim, owner(&sim, 1));
    assert_eq!(first.2, [0.0; 3]);
    assert_eq!(second.2, [2.0, 0.0, 0.0]);
    assert!((first.4[1] - 1.9).abs() < 1e-10);
    assert!((second.4[1] - 2.0).abs() < 1e-10);
    assert!(first.4[1] > 0.0 && second.4[1] > 0.0);
}
