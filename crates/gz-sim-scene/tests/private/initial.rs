// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Original ResetTo assertions adapted from EntityComponentManager_TEST.cc.
/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
use super::*;
use gz_sim_core::assign_id;

#[test]
fn initial_joint_state_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_joint_state_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    assert!(matches!(
        initial_joint_state_publications(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_joint_state_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_joint_state_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_joint_state_identities_borrow_full_originals_through_removal_and_reset() {
    let xml = include_str!("../../../../examples/m57_multiple_joint_state_publishers.sdf").replace(
        "<world name=\"default\">",
        "<world name=\"warehouse_floor\">",
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "joint-state-identities.sdf",
        "warehouse_floor",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let collect = |sim: &Simulation| {
        initial_joint_state_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    let snapshot = collect(&sim);
    assert_eq!(snapshot.len(), 2);
    assert_ne!(snapshot[0].0, snapshot[1].0);
    assert_eq!(
        snapshot
            .iter()
            .map(|row| row.2.as_str())
            .collect::<Vec<_>>(),
        ["warehouse_floor::arm0", "warehouse_floor::arm1"]
    );
    assert_eq!(
        snapshot
            .iter()
            .map(|row| row.3.joints.len())
            .collect::<Vec<_>>(),
        [2, 1]
    );
    let original_handles = snapshot
        .iter()
        .map(|row| entity_by_id(sim.world(), row.0).unwrap())
        .collect::<Vec<_>>();
    for ((identity, kind, name, metadata), (legacy_id, legacy)) in
        initial_joint_state_publications_with_identity(&sim)
            .unwrap()
            .zip(initial_joint_state_publications(&sim).unwrap())
    {
        assert_eq!(identity, legacy_id);
        assert_eq!(identity, id(&sim, name));
        assert_eq!(kind, EntityKind::Model);
        assert_eq!(
            name.strip_prefix("warehouse_floor::"),
            Some(metadata.model_name.as_str())
        );
        assert!(std::ptr::eq(metadata, legacy));
    }

    let first = original_handles[0];
    sim.world_mut()
        .entity_mut(first)
        .insert((Name("changed-live-name".to_owned()), EntityKind::Link));
    sim.world_mut()
        .get_mut::<JointStatePublication>(first)
        .unwrap()
        .topic = "/changed-live-topic".to_owned();
    assert_eq!(collect(&sim), snapshot);
    sim.world_mut().entity_mut(first).insert((
        Name(snapshot[0].2.clone()),
        snapshot[0].1,
        snapshot[0].3.clone(),
    ));

    sim.step_exact(3).unwrap();
    for row in &snapshot {
        remove_model(&mut sim, row.0).unwrap();
        assert!(entity_by_id(sim.world(), row.0).is_none());
        assert_eq!(collect(&sim), snapshot);
    }
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_joint_state_publications(&sim).unwrap().count(), 2);
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for ((identity, kind, name, metadata), original) in snapshot.iter().zip(original_handles) {
        let restored = entity_by_id(sim.world(), *identity).unwrap();
        assert_ne!(restored, original);
        assert_eq!(sim.world().get::<Name>(restored).unwrap().0, *name);
        assert_eq!(sim.world().get::<EntityKind>(restored), Some(kind));
        assert_eq!(
            sim.world().get::<JointStatePublication>(restored),
            Some(metadata)
        );
    }
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_odometry_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_ground_truth_odometry_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(
        initial_ground_truth_odometry_publications(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        initial_ground_truth_odometry_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_odometry_identities_borrow_full_originals_through_removal_and_reset() {
    let xml = include_str!("../../../../examples/m56_multiple_odometry_publishers.sdf").replace(
        "<world name=\"default\">",
        "<world name=\"warehouse_floor\">",
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "odometry-identities.sdf",
        "warehouse_floor",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let collect = |sim: &Simulation| {
        initial_ground_truth_odometry_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    let snapshot = collect(&sim);
    assert_eq!(snapshot.len(), 2);
    assert_ne!(snapshot[0].0, snapshot[1].0);
    assert_eq!(
        snapshot
            .iter()
            .map(|row| row.2.as_str())
            .collect::<Vec<_>>(),
        ["warehouse_floor::planar", "warehouse_floor::spatial"]
    );
    let original_handles = snapshot
        .iter()
        .map(|row| entity_by_id(sim.world(), row.0).unwrap())
        .collect::<Vec<_>>();
    for ((identity, kind, name, metadata), (legacy_id, legacy)) in
        initial_ground_truth_odometry_publications_with_identity(&sim)
            .unwrap()
            .zip(initial_ground_truth_odometry_publications(&sim).unwrap())
    {
        assert_eq!(identity, legacy_id);
        assert_eq!(identity, id(&sim, name));
        assert_eq!(kind, EntityKind::Model);
        assert_eq!(
            name.strip_prefix("warehouse_floor::"),
            Some(metadata.model_name.as_str())
        );
        assert!(std::ptr::eq(metadata, legacy));
    }
    sim.step_exact(3).unwrap();
    for row in &snapshot {
        remove_model(&mut sim, row.0).unwrap();
        assert!(entity_by_id(sim.world(), row.0).is_none());
        assert_eq!(collect(&sim), snapshot);
    }
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(
        initial_ground_truth_odometry_publications(&sim)
            .unwrap()
            .count(),
        2
    );
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for ((identity, kind, name, metadata), original) in snapshot.iter().zip(original_handles) {
        let restored = entity_by_id(sim.world(), *identity).unwrap();
        assert_ne!(restored, original);
        assert_eq!(sim.world().get::<Name>(restored).unwrap().0, *name);
        assert_eq!(sim.world().get::<EntityKind>(restored), Some(kind));
        assert_eq!(
            sim.world().get::<GroundTruthOdometryPublication>(restored),
            Some(metadata)
        );
    }
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn camera_reset_entries_are_sparse_and_share_original_sensor_metadata() {
    let source = include_str!("../fixtures/camera-binding.sdf");
    let start = source.find("<sensor name=\"camera\"").unwrap();
    let end = start + source[start..].find("</sensor>").unwrap() + "</sensor>".len();
    let empty_source = format!("{}{}", &source[..start], &source[end..]);
    let empty = load_world_with_sdf_step(
        &empty_source,
        "empty.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let empty_cameras = &empty.world().resource::<InitialScene>().0.cameras;
    assert_eq!(empty_cameras.len(), 0);
    assert_eq!(empty_cameras.capacity(), 0);

    let two = source.replace("</sensor>", "</sensor><sensor name='second' type='camera'><topic>/second</topic><camera><image><width>64</width><height>64</height></image><clip><near>0.1</near><far>10</far></clip></camera></sensor>");
    let mut sim = load_world_with_sdf_step(
        &two,
        "two.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let first = id(&sim, "default::robot::body::camera");
    let second = id(&sim, "default::robot::body::second");
    let initial = sim.world().resource::<InitialScene>().0.clone();
    assert_eq!(initial.cameras.len(), 2);
    for (id, sensor) in &initial.cameras {
        assert!([first, second].contains(id));
        let entity = entity_by_id(sim.world(), *id).unwrap();
        assert!(Arc::ptr_eq(
            &sensor.0,
            &sim.world().get::<CameraSensor>(entity).unwrap().0
        ));
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (id, sensor) in &initial.cameras {
        let entity = entity_by_id(sim.world(), *id).unwrap();
        assert!(Arc::ptr_eq(
            &sensor.0,
            &sim.world().get::<CameraSensor>(entity).unwrap().0
        ));
    }
}

fn load() -> Simulation {
    load_world(
        include_str!("../fixtures/fall.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}

#[test]
fn initial_lidar_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_lidar_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_lidar_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_lidar_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_lidar_identities_borrow_both_originals_through_removal_and_reset() {
    let source = include_str!("../fixtures/planar_lidar.sdf");
    let start = source.find("<sensor name=\"laser\"").unwrap();
    let end = start + source[start..].find("</sensor>").unwrap() + "</sensor>".len();
    let peer = source[start..end]
        .replace("name=\"laser\"", "name=\"peer\"")
        .replace("<topic>/scan</topic>", "<topic>/peer_scan</topic>")
        .replace("laser_frame", "peer_frame");
    let xml = format!("{}{}{}", &source[..end], peer, &source[end..])
        .replace("world name=\"default\"", "world name=\"warehouse_floor\"");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "lidar-identities.sdf",
        "warehouse_floor",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let collect = |sim: &Simulation| {
        initial_lidar_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    let snapshots = collect(&sim);
    assert_eq!(snapshots.len(), 2);
    assert_ne!(snapshots[0].0, snapshots[1].0);
    let old_entities = [
        named(&sim, "warehouse_floor::scanner::body::laser"),
        named(&sim, "warehouse_floor::scanner::body::peer"),
    ];
    for ((identity, kind, name, metadata), old_metadata) in
        initial_lidar_publications_with_identity(&sim)
            .unwrap()
            .zip(initial_lidar_publications(&sim).unwrap())
    {
        assert_eq!(identity, id(&sim, name));
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(
            name.strip_prefix("warehouse_floor::"),
            Some(metadata.entity_name.as_str())
        );
        assert!(std::ptr::eq(metadata, old_metadata));
    }
    sim.step_exact(3).unwrap();
    let scanner = id(&sim, "warehouse_floor::scanner");
    remove_model(&mut sim, scanner).unwrap();
    for snapshot in &snapshots {
        assert!(entity_by_id(sim.world(), snapshot.0).is_none());
    }
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), snapshots);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (snapshot, old_entity) in snapshots.iter().zip(old_entities) {
        let restored = entity_by_id(sim.world(), snapshot.0).unwrap();
        assert_ne!(restored, old_entity);
        assert_eq!(
            sim.world().get::<EntityKind>(restored),
            Some(&EntityKind::Sensor)
        );
        assert_eq!(sim.world().get::<Name>(restored).unwrap().0, snapshot.2);
        assert_eq!(
            sim.world().get::<LidarPublication>(restored),
            Some(&snapshot.3)
        );
    }
    assert_eq!(collect(&sim), snapshots);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_altimeter_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_altimeter_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_altimeter_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_altimeter_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_altimeter_identities_borrow_original_metadata_through_removal_and_reset() {
    let mut sim = load_world_with_sdf_step(
        include_str!("../fixtures/m13_altimeter.sdf"),
        "identity-altimeter.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let sensor_name = "default::rig::body::altimeter";
    let sensor_id = id(&sim, sensor_name);
    let old_entity = named(&sim, sensor_name);
    let snapshot = {
        let mut identities = initial_altimeter_publications_with_identity(&sim).unwrap();
        let (identity, kind, name, metadata) = identities.next().unwrap();
        assert!(identities.next().is_none());
        assert_eq!(identity, sensor_id);
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(name, sensor_name);
        assert_eq!(metadata.entity_name, "rig::body::altimeter");
        assert_eq!(
            name.strip_prefix("default::"),
            Some(metadata.entity_name.as_str())
        );
        let old_metadata = initial_altimeter_publications(&sim)
            .unwrap()
            .next()
            .unwrap();
        assert!(std::ptr::eq(metadata, old_metadata));
        (identity, kind, name.to_owned(), metadata.clone())
    };
    let collect = |sim: &Simulation| {
        initial_altimeter_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    sim.step_exact(3).unwrap();
    let model_id = id(&sim, "default::rig");
    remove_model(&mut sim, model_id).unwrap();
    assert!(entity_by_id(sim.world(), sensor_id).is_none());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), vec![snapshot.clone()]);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), sensor_id).unwrap();
    assert_ne!(restored, old_entity);
    assert_eq!(sim.world().get::<Name>(restored).unwrap().0, sensor_name);
    assert_eq!(
        sim.world().get::<EntityKind>(restored),
        Some(&EntityKind::Sensor)
    );
    assert_eq!(
        sim.world().get::<AltimeterPublication>(restored),
        Some(&snapshot.3)
    );
    assert_eq!(collect(&sim), vec![snapshot]);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_air_pressure_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_air_pressure_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_air_pressure_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_air_pressure_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_air_pressure_identities_borrow_original_metadata_through_removal_and_reset() {
    let mut sim = load_world_with_sdf_step(
        include_str!("../fixtures/m15_air_pressure.sdf"),
        "identity-air-pressure.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let sensor_name = "default::rig::body::air_pressure";
    let sensor_id = id(&sim, sensor_name);
    let old_entity = named(&sim, sensor_name);
    let snapshot = {
        let mut identities = initial_air_pressure_publications_with_identity(&sim).unwrap();
        let (identity, kind, name, metadata) = identities.next().unwrap();
        assert!(identities.next().is_none());
        assert_eq!(identity, sensor_id);
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(name, sensor_name);
        assert_eq!(metadata.entity_name, "rig::body::air_pressure");
        assert_eq!(
            name.strip_prefix("default::"),
            Some(metadata.entity_name.as_str())
        );
        let old_metadata = initial_air_pressure_publications(&sim)
            .unwrap()
            .next()
            .unwrap();
        assert!(std::ptr::eq(metadata, old_metadata));
        (identity, kind, name.to_owned(), metadata.clone())
    };
    let collect = |sim: &Simulation| {
        initial_air_pressure_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    sim.step_exact(3).unwrap();
    let model_id = id(&sim, "default::rig");
    remove_model(&mut sim, model_id).unwrap();
    assert!(entity_by_id(sim.world(), sensor_id).is_none());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), vec![snapshot.clone()]);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), sensor_id).unwrap();
    assert_ne!(restored, old_entity);
    assert_eq!(sim.world().get::<Name>(restored).unwrap().0, sensor_name);
    assert_eq!(
        sim.world().get::<EntityKind>(restored),
        Some(&EntityKind::Sensor)
    );
    assert_eq!(
        sim.world().get::<AirPressurePublication>(restored),
        Some(&snapshot.3)
    );
    assert_eq!(collect(&sim), vec![snapshot]);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_air_speed_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_air_speed_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_air_speed_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_air_speed_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_air_speed_identities_borrow_original_metadata_through_removal_and_reset() {
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../../examples/m41_air_speed_wind.sdf"),
        "identity-air-speed.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let sensor_name = "default::rig::body::air_speed";
    let sensor_id = id(&sim, sensor_name);
    let old_entity = named(&sim, sensor_name);
    let snapshot = {
        let mut identities = initial_air_speed_publications_with_identity(&sim).unwrap();
        let (identity, kind, name, metadata) = identities.next().unwrap();
        assert!(identities.next().is_none());
        assert_eq!(identity, sensor_id);
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(name, sensor_name);
        assert_eq!(metadata.entity_name, "rig::body::air_speed");
        assert_eq!(
            name.strip_prefix("default::"),
            Some(metadata.entity_name.as_str())
        );
        let old_metadata = initial_air_speed_publications(&sim)
            .unwrap()
            .next()
            .unwrap();
        assert!(std::ptr::eq(metadata, old_metadata));
        (identity, kind, name.to_owned(), metadata.clone())
    };
    let collect = |sim: &Simulation| {
        initial_air_speed_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    sim.step_exact(3).unwrap();
    let model_id = id(&sim, "default::rig");
    remove_model(&mut sim, model_id).unwrap();
    assert!(entity_by_id(sim.world(), sensor_id).is_none());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), vec![snapshot.clone()]);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), sensor_id).unwrap();
    assert_ne!(restored, old_entity);
    assert_eq!(sim.world().get::<Name>(restored).unwrap().0, sensor_name);
    assert_eq!(
        sim.world().get::<EntityKind>(restored),
        Some(&EntityKind::Sensor)
    );
    assert_eq!(
        sim.world().get::<AirSpeedPublication>(restored),
        Some(&snapshot.3)
    );
    assert_eq!(collect(&sim), vec![snapshot]);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_magnetometer_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_magnetometer_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_magnetometer_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_magnetometer_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_magnetometer_identities_borrow_original_metadata_through_removal_and_reset() {
    let mut sim = load_world_with_sdf_step(
        include_str!("../fixtures/m17_magnetometer.sdf"),
        "identity-air-pressure.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let sensor_name = "default::rig::body::magnetometer";
    let sensor_id = id(&sim, sensor_name);
    let old_entity = named(&sim, sensor_name);
    let snapshot = {
        let mut identities = initial_magnetometer_publications_with_identity(&sim).unwrap();
        let (identity, kind, name, metadata) = identities.next().unwrap();
        assert!(identities.next().is_none());
        assert_eq!(identity, sensor_id);
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(name, sensor_name);
        assert_eq!(metadata.entity_name, "rig::body::magnetometer");
        assert_eq!(
            name.strip_prefix("default::"),
            Some(metadata.entity_name.as_str())
        );
        let old_metadata = initial_magnetometer_publications(&sim)
            .unwrap()
            .next()
            .unwrap();
        assert!(std::ptr::eq(metadata, old_metadata));
        (identity, kind, name.to_owned(), metadata.clone())
    };
    let collect = |sim: &Simulation| {
        initial_magnetometer_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    sim.step_exact(3).unwrap();
    let model_id = id(&sim, "default::rig");
    remove_model(&mut sim, model_id).unwrap();
    assert!(entity_by_id(sim.world(), sensor_id).is_none());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), vec![snapshot.clone()]);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), sensor_id).unwrap();
    assert_ne!(restored, old_entity);
    assert_eq!(sim.world().get::<Name>(restored).unwrap().0, sensor_name);
    assert_eq!(
        sim.world().get::<EntityKind>(restored),
        Some(&EntityKind::Sensor)
    );
    assert_eq!(
        sim.world().get::<MagnetometerPublication>(restored),
        Some(&snapshot.3)
    );
    assert_eq!(collect(&sim), vec![snapshot]);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn removed_initial_models_restore_ids_physics_and_sensors_without_running_normal_systems() {
    #[derive(Resource, Default)]
    struct Counts {
        ordinary: u64,
        resets: u64,
    }
    let mut sim = load();
    sim.world_mut().insert_resource(Counts::default());
    sim.add_systems(Phase::PreStep, |mut counts: ResMut<Counts>| {
        counts.ordinary += 1
    });
    sim.add_systems(Phase::Reset, |mut counts: ResMut<Counts>| {
        counts.resets += 1
    });
    let ball = id(&sim, "default::ball");
    let old_body_entity = named(&sim, "default::ball::body");
    let body_id = id(&sim, "default::ball::body");
    let physics = sim.world().get::<BodyHandle>(old_body_entity).unwrap().id();
    sim.step_exact(3).unwrap();
    remove_model(&mut sim, ball).unwrap();
    assert!(entity_by_id(sim.world(), ball).is_none());
    let value = reset_loaded_world(&mut sim, |sim| {
        let report = sim.world().resource::<SceneResetInfo>();
        assert_eq!(report.previous_time, Duration::from_millis(3));
        assert_eq!(report.epoch, 1);
        assert!(report.restored_ids.contains(&ball));
        assert!(report.removed_ids.is_empty());
        assert_eq!(sim.world().resource::<Counts>().ordinary, 3);
        assert_eq!(sim.world().resource::<Counts>().resets, 1);
        assert_eq!(
            sim.world()
                .get::<WorldPose>(named(sim, "default::ball"))
                .unwrap()
                .translation[2],
            5.0
        );
        assert!(
            !sim.world()
                .iter_entities()
                .any(|e| e.contains::<ImuSample>())
        );
        42
    })
    .unwrap();
    assert_eq!(value, 42);
    let restored = entity_by_id(sim.world(), body_id).unwrap();
    assert_ne!(restored, old_body_entity);
    assert_eq!(
        sim.world().get::<BodyHandle>(restored).unwrap().id(),
        physics
    );
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(physics)
            .unwrap()
            .position[2],
        5.0
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(sim.is_paused());
    sim.step_exact(1).unwrap();
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
        .unwrap();
    assert_eq!(sample.epoch, 1);
    assert_eq!(sample.time, Duration::from_millis(1));
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(physics)
            .unwrap()
            .position[2]
            < 5.0
    );
    sim.set_paused(false);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(!sim.is_paused());
    assert_eq!(sim.world().resource::<SceneResetInfo>().epoch, 2);
    assert_eq!(sim.world().resource::<Counts>().resets, 2);
}

#[test]
fn runtime_additions_remain_observable_through_reset_then_retire_without_resource_loss() {
    #[derive(Resource)]
    struct Sentinel(u64);
    let mut sim = load();
    sim.world_mut().insert_resource(Sentinel(123));
    let parent = named(&sim, "default::ball");
    let parent_id = id(&sim, "default::ball");
    let first = sim
        .world_mut()
        .spawn((Name("extra".into()), ChildOf(parent)))
        .id();
    let first_id = assign_id(sim.world_mut(), first).unwrap();
    let second = sim
        .world_mut()
        .spawn((Name("child".into()), ChildOf(first)))
        .id();
    let second_id = assign_id(sim.world_mut(), second).unwrap();
    reset_loaded_world(&mut sim, |sim| {
        let report = sim.world().resource::<SceneResetInfo>();
        assert_eq!(report.removed_ids, [first_id, second_id]);
        assert!(sim.world().get::<PendingResetRemoval>(first).is_some());
        assert!(sim.world().get::<PendingResetRemoval>(second).is_some());
        assert_eq!(sim.world().get::<Name>(first).unwrap().0, "extra");
        assert_eq!(
            sim.world().get::<ChildOf>(first).unwrap().parent(),
            entity_by_id(sim.world(), parent_id).unwrap()
        );
        assert_eq!(sim.world().get::<ChildOf>(second).unwrap().parent(), first);
        assert_eq!(sim.world().resource::<Sentinel>().0, 123);
    })
    .unwrap();
    assert!(entity_by_id(sim.world(), first_id).is_none());
    assert!(entity_by_id(sim.world(), second_id).is_none());
    assert_eq!(sim.world().resource::<Sentinel>().0, 123);
    let fresh = sim.world_mut().spawn_empty().id();
    assert!(assign_id(sim.world_mut(), fresh).unwrap() > second_id);
}

#[test]
fn invalid_initial_backend_identity_and_resource_ownership_fail_before_mutation() {
    let mut bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        reset_loaded_world(&mut bare, |_| ()),
        Err(SceneResetError::MissingInitialState)
    ));
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let ball = named(&sim, "default::ball");
    let count = sim.world().entities().len();
    let original = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::MissingPhysics)
    ));
    sim.world_mut()
        .insert_resource(PhysicsBackend::from(RapierBackend::new([0.0; 3]).unwrap()));
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Physics(PhysicsError::ForeignInitialState))
    ));
    sim.world_mut().insert_resource(original);
    #[derive(Resource)]
    struct Linked;
    sim.world_mut().insert_resource(Linked);
    let component = sim.world().component_id::<Linked>().unwrap();
    let resource = sim.world().resource_entities().get(component).unwrap();
    sim.world_mut().entity_mut(resource).insert(ChildOf(ball));
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    sim.world_mut().entity_mut(resource).remove::<ChildOf>();
    sim.world_mut().remove_resource::<Linked>();
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(sim.world().get_entity(ball).is_ok());
    assert!(sim.world().entities().len() >= count);
    let mut other = load();
    let initial = sim.world_mut().remove_resource::<InitialScene>().unwrap();
    other.world_mut().insert_resource(initial);
    assert!(matches!(
        reset_loaded_world(&mut other, |_| ()),
        Err(SceneResetError::Identity(IdentityError::ForeignReservation))
    ));
}

#[test]
fn unsupported_runtime_bindings_and_moved_identities_leave_scene_and_clock_unchanged() {
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let ball = id(&sim, "default::ball");
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let before = sim.world().resource::<PhysicsBackend>().state(body);
    let extra = sim.world_mut().spawn(BodyHandle(body)).id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::RuntimePhysics)
    ));
    assert_eq!(sim.world().resource::<PhysicsBackend>().state(body), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(entity_by_id(sim.world(), ball).is_some());
    sim.world_mut().entity_mut(extra).despawn();
    let mut other = load();
    let foreign = named(&other, "default::ball");
    let identity = other
        .world_mut()
        .entity_mut(foreign)
        .take::<EntityId>()
        .unwrap();
    let extra = sim.world_mut().spawn(identity).id();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Identity(IdentityError::MovedIdentity))
    ));
    assert_eq!(sim.world().resource::<PhysicsBackend>().state(body), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    sim.world_mut().entity_mut(extra).despawn();
    let unnamed_id = sim.world_mut().spawn(Name("unidentified".into())).id();
    reset_loaded_world(&mut sim, |sim| {
        assert!(sim.world().get::<PendingResetRemoval>(unnamed_id).is_some());
        assert!(
            sim.world()
                .resource::<SceneResetInfo>()
                .removed_ids
                .is_empty()
        );
    })
    .unwrap();
    assert!(sim.world().get_entity(unnamed_id).is_err());
}

fn two_named_entities() -> (Simulation, u64, u64) {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    gz_physics_rapier::install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()).unwrap();
    sim.world_mut().insert_resource(Gravity([0.0; 3]));
    let pose = Transform {
        translation: [1.0, 2.0, 3.0],
        rotation: DQuat::from_euler(glam::EulerRot::ZYX, 0.3, 0.2, 0.1).to_array(),
    };
    let first = spawn_node(&mut sim, None, "entity1", EntityKind::Model, pose, pose).unwrap();
    let second = spawn_node(
        &mut sim,
        None,
        "entity2",
        EntityKind::Model,
        Transform::default(),
        Transform::default(),
    )
    .unwrap();
    let first = sim.world().get::<EntityId>(first).unwrap().get();
    let second = sim.world().get::<EntityId>(second).unwrap().get();
    capture(&mut sim);
    (sim, first, second)
}

#[test]
fn native_reset_to_deleted_entity_retains_the_two_new_name_observations() {
    for _ in 1..10 {
        let (mut sim, first, second) = two_named_entities();
        let mut names = sim.world_mut().query_filtered::<Entity, Added<Name>>();
        assert_eq!(names.iter(sim.world()).count(), 2);
        let removed = entity_by_id(sim.world(), first).unwrap();
        sim.world_mut().entity_mut(removed).despawn();
        sim.world_mut().clear_trackers();
        assert_eq!(names.iter(sim.world()).count(), 0);
        reset_loaded_world(&mut sim, |sim| {
            let new: Vec<_> = names.iter(sim.world()).collect();
            assert_eq!(new.len(), 2);
            assert!(new.contains(&entity_by_id(sim.world(), first).unwrap()));
            assert!(new.contains(&entity_by_id(sim.world(), second).unwrap()));
            assert_eq!(
                sim.world()
                    .get::<WorldPose>(entity_by_id(sim.world(), first).unwrap())
                    .unwrap()
                    .translation,
                [1.0, 2.0, 3.0]
            );
        })
        .unwrap();
    }
}

#[test]
fn native_reset_to_added_entity_retains_the_one_removed_name_observation() {
    for _ in 1..10 {
        let (mut sim, first, second) = two_named_entities();
        let extra = sim.world_mut().spawn(Name("entity3".into())).id();
        let third = assign_id(sim.world_mut(), extra).unwrap();
        let mut names = sim.world_mut().query_filtered::<Entity, Added<Name>>();
        sim.world_mut().clear_trackers();
        assert_eq!(names.iter(sim.world()).count(), 0);
        reset_loaded_world(&mut sim, |sim| {
            assert!(names.iter(sim.world()).count() >= 2);
            let removed: Vec<_> = sim
                .world()
                .iter_entities()
                .filter(|e| e.contains::<PendingResetRemoval>() && e.contains::<Name>())
                .collect();
            assert_eq!(removed.len(), 1);
            assert_eq!(removed[0].id(), extra);
            assert_eq!(removed[0].get::<Name>().unwrap().0, "entity3");
            assert_eq!(
                sim.world().resource::<SceneResetInfo>().removed_ids,
                [third]
            );
        })
        .unwrap();
        assert!(entity_by_id(sim.world(), first).is_some());
        assert!(entity_by_id(sim.world(), second).is_some());
        assert!(entity_by_id(sim.world(), third).is_none());
    }
}

#[test]
fn nested_canonical_and_attached_frames_rebuild_with_original_parent_identities() {
    let mut sim = load_world(
        include_str!("../fixtures/nested.sdf"),
        "nested.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let expected: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<EntityKind>())
        .map(|e| {
            (
                e.get::<EntityId>().unwrap().get(),
                e.get::<ChildOf>()
                    .map(|p| sim.world().get::<EntityId>(p.parent()).unwrap().get()),
                *e.get::<WorldPose>().unwrap(),
                *e.get::<LocalPose>().unwrap(),
            )
        })
        .collect();
    sim.step_exact(4).unwrap();
    let outer = id(&sim, "default::outer");
    remove_model(&mut sim, outer).unwrap();
    reset_loaded_world(&mut sim, |sim| {
        for (id, parent, world_pose, local_pose) in &expected {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            assert_eq!(
                sim.world().get::<ChildOf>(entity).map(|p| sim
                    .world()
                    .get::<EntityId>(p.parent())
                    .unwrap()
                    .get()),
                *parent
            );
            let world = sim.world().get::<WorldPose>(entity).unwrap();
            let local = sim.world().get::<LocalPose>(entity).unwrap();
            for (actual, expected) in world
                .translation
                .into_iter()
                .chain(world.rotation)
                .chain(local.0.translation)
                .chain(local.0.rotation)
                .zip(
                    world_pose
                        .translation
                        .into_iter()
                        .chain(world_pose.rotation)
                        .chain(local_pose.0.translation)
                        .chain(local_pose.0.rotation),
                )
            {
                assert!((actual - expected).abs() < 1e-12);
            }
        }
    })
    .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<EntityKind>())
            .count(),
        expected.len()
    );
}

#[test]
fn identity_moved_to_a_resource_is_rejected_before_any_reset_mutation() {
    #[derive(Resource)]
    struct Holder;
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let original = named(&sim, "default");
    let moved = sim
        .world_mut()
        .entity_mut(original)
        .take::<EntityId>()
        .unwrap();
    let public_id = moved.get();
    sim.world_mut().insert_resource(Holder);
    let component = sim.world().component_id::<Holder>().unwrap();
    let resource = sim.world().resource_entities().get(component).unwrap();
    sim.world_mut().entity_mut(resource).insert(moved);
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let state = sim.world().resource::<PhysicsBackend>().state(body);
    let count = sim.world().entities().len();
    let info = *sim.world().resource::<UpdateInfo>();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Identity(IdentityError::MovedIdentity))
    ));
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.world().entities().len(), count);
    assert_eq!(sim.world().resource::<PhysicsBackend>().state(body), state);
    assert!(sim.world().get::<EntityId>(original).is_none());
    assert_eq!(
        sim.world().get::<EntityId>(resource).unwrap().get(),
        public_id
    );
    assert!(entity_by_id(sim.world(), public_id).is_none());
    assert!(!sim.world().contains_resource::<SceneResetInfo>());
}

#[test]
fn registered_systems_and_global_observers_survive_scene_reconstruction() {
    #[derive(Event)]
    struct Ping;
    #[derive(Resource, Default)]
    struct Count(u64);
    let mut sim = load();
    let system = sim.world_mut().register_system(|| 42_u32);
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
    sim.world_mut().init_resource::<Count>();
    let observer = sim
        .world_mut()
        .add_observer(|_: On<Ping>, mut count: ResMut<Count>| count.0 += 1)
        .id();
    sim.world_mut().trigger(Ping);
    assert_eq!(sim.world().resource::<Count>().0, 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
    assert!(sim.world().get_entity(observer).is_ok());
    sim.world_mut().trigger(Ping);
    assert_eq!(sim.world().resource::<Count>().0, 2);
    let parent = named(&sim, "default::ball");
    sim.world_mut().entity_mut(observer).insert(ChildOf(parent));
    let info = *sim.world().resource::<UpdateInfo>();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
}

#[test]
fn mixed_initial_node_and_observer_ownership_fails_before_reconstruction() {
    #[derive(Event)]
    struct Ping;
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let world = named(&sim, "default");
    let world_id = id(&sim, "default");
    sim.world_mut()
        .entity_mut(world)
        .insert(Observer::new(|_: On<Ping>| {}));
    let count = sim.world().entities().len();
    let info = *sim.world().resource::<UpdateInfo>();
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let state = sim.world().resource::<PhysicsBackend>().state(body);
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    assert_eq!(sim.world().entities().len(), count);
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert_eq!(entity_by_id(sim.world(), world_id), Some(world));
    assert!(sim.world().get::<Observer>(world).is_some());
    assert_eq!(sim.world().resource::<PhysicsBackend>().state(body), state);
}

#[test]
fn reported_reset_fault_cleans_pending_nodes_without_observation_or_retry() {
    use gz_sim_core::StepFault;
    use std::cell::Cell;
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let extra = sim.world_mut().spawn(Name("runtime extra".into())).id();
    let extra_id = assign_id(sim.world_mut(), extra).unwrap();
    sim.add_systems(Phase::Reset, |fault: Res<StepFault>| {
        fault.report(std::io::Error::other("reset callback failed"));
    });
    let observed = Cell::new(false);
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed.set(true)),
        Err(SceneResetError::CommittedReset)
    ));
    assert!(!observed.get());
    assert!(sim.world().get_entity(extra).is_err());
    assert!(entity_by_id(sim.world(), extra_id).is_none());
    assert!(!sim.world().contains_resource::<RestoredPhysics>());
    assert_eq!(
        (sim.elapsed(), sim.world().resource::<UpdateInfo>().epoch),
        (Duration::ZERO, 1)
    );
    assert!(
        sim.fault()
            .unwrap()
            .to_string()
            .contains("reset callback failed")
    );
    let count = sim.world().entities().len();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed.set(true)),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert_eq!(sim.world().entities().len(), count);
    assert!(!observed.get());
    assert_eq!(sim.tick(), Err(StepError::Faulted));
}

#[test]
fn orphaned_runtime_com_metadata_refuses_reset_before_mutation() {
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let ball = id(&sim, "default::ball");
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let before = sim.world().resource::<PhysicsBackend>().state(body);
    let extra = sim.world_mut().spawn(LinkCenterOfMass([0.1, 0., 0.])).id();
    assign_id(sim.world_mut(), extra).unwrap();
    let observed = std::cell::Cell::new(false);
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed.set(true)),
        Err(SceneResetError::RuntimePhysics)
    ));
    assert!(!observed.get());
    assert_eq!(sim.world().resource::<PhysicsBackend>().state(body), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(entity_by_id(sim.world(), ball).is_some());
    assert!(sim.world().get::<LinkCenterOfMass>(extra).is_some());
    sim.world_mut().entity_mut(extra).despawn();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn initial_logical_camera_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_logical_camera_identities(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(
        initial_logical_camera_publications(&sim).unwrap().count(),
        0
    );
    assert_eq!(initial_logical_camera_identities(&sim).unwrap().count(), 0);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_logical_camera_identities_borrow_both_originals_through_removal_and_reset() {
    let world_name = "warehouse_floor";
    let source = include_str!("../fixtures/m30_logical_camera.sdf")
        .replace("<world name='default'>", "<world name='warehouse_floor'>");
    let start = source.find("<sensor name='logical_camera'").unwrap();
    let end = start + source[start..].find("</sensor>").unwrap() + "</sensor>".len();
    let second = source[start..end]
        .replace("name='logical_camera'", "name='second'")
        .replace("/m30/logical_camera", "/m53/identity/second")
        .replace("rig/body/logical_camera", "rig/body/second");
    let xml = format!("{}{}{}", &source[..end], second, &source[end..]);
    // A scoped runtime owner prefix does not grant XML WORLD-name admission.
    // Retain the former raw XML input as the exact typed parser refusal.
    let invalid = xml.replace(
        "world name='warehouse_floor'",
        "world name='warehouse::floor'",
    );
    match load_world_with_sdf_step(
        &invalid,
        "logical-camera-identities.sdf",
        "warehouse::floor",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    ) {
        Err(LoadError::Sdf(diagnostic)) => {
            assert_eq!(diagnostic.code, sdformat::ErrorCode::InvalidValue);
            assert_eq!(diagnostic.message, "reserved or invalid name");
            assert_eq!(diagnostic.source.uri, "logical-camera-identities.sdf");
        }
        Err(error) => panic!("expected scoped WORLD-name parser refusal, got {error}"),
        Ok(_) => panic!("scoped XML WORLD name must remain refused"),
    }
    let mut sim = load_world_with_sdf_step(
        &xml,
        "logical-camera-identities.sdf",
        world_name,
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let collect = |sim: &Simulation| {
        initial_logical_camera_identities(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    let snapshot = collect(&sim);
    assert_eq!(snapshot.len(), 2);
    assert_eq!(
        initial_logical_camera_publications(&sim).unwrap().count(),
        2
    );
    assert_ne!(snapshot[0].0, snapshot[1].0);
    assert_eq!(
        snapshot
            .iter()
            .map(|row| row.2.as_str())
            .collect::<Vec<_>>(),
        [
            "warehouse_floor::rig::body::logical_camera",
            "warehouse_floor::rig::body::second",
        ]
    );
    let original_handles = snapshot
        .iter()
        .map(|row| entity_by_id(sim.world(), row.0).unwrap())
        .collect::<Vec<_>>();
    let prefix = format!("{world_name}::");
    for ((identity, kind, name, metadata), legacy) in initial_logical_camera_identities(&sim)
        .unwrap()
        .zip(initial_logical_camera_publications(&sim).unwrap())
    {
        assert_eq!(identity, id(&sim, name));
        assert_eq!(kind, EntityKind::Sensor);
        assert_eq!(
            name.strip_prefix(prefix.as_str()),
            Some(metadata.entity_name.as_str())
        );
        assert!(std::ptr::eq(metadata, legacy));
    }
    sim.step_exact(3).unwrap();
    let model_id = id(&sim, "warehouse_floor::rig");
    remove_model(&mut sim, model_id).unwrap();
    for row in &snapshot {
        assert!(entity_by_id(sim.world(), row.0).is_none());
    }
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(
        initial_logical_camera_publications(&sim).unwrap().count(),
        2
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for ((identity, kind, name, metadata), old_entity) in snapshot.iter().zip(original_handles) {
        let restored = entity_by_id(sim.world(), *identity).unwrap();
        assert_ne!(restored, old_entity);
        assert_eq!(sim.world().get::<Name>(restored).unwrap().0, *name);
        assert_eq!(sim.world().get::<EntityKind>(restored), Some(kind));
        assert_eq!(
            sim.world().get::<LogicalCameraPublication>(restored),
            Some(metadata)
        );
    }
    assert_eq!(collect(&sim), snapshot);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}

#[test]
fn initial_pose_identity_query_distinguishes_missing_and_empty_blueprints() {
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_pose_publications_with_identity(&bare),
        Err(SceneResetError::MissingInitialState)
    ));
    let sim = load();
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(initial_pose_publications(&sim).unwrap().count(), 0);
    assert_eq!(
        initial_pose_publications_with_identity(&sim)
            .unwrap()
            .count(),
        0
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn initial_pose_identities_borrow_all_originals_through_owner_removal_and_loaded_reset() {
    let source = include_str!("../fixtures/m31_pose_static_publisher.sdf");
    let start = source.find("<model name=\"rig\"").unwrap();
    let end = start + source[start..].find("</model>").unwrap() + "</model>".len();
    let peer = source[start..end]
        .replace("name=\"rig\"", "name=\"peer\"")
        .replace("<pose>1 2 3 0 0 0.6</pose>", "<pose>1 8 3 0 0 0.6</pose>")
        .replace("/m31/pose", "/m55/peer_pose");
    let xml = format!("{}{}{}", &source[..end], peer, &source[end..])
        .replace("world name=\"default\"", "world name=\"warehouse_floor\"");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "pose-identities.sdf",
        "warehouse_floor",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let collect = |sim: &Simulation| {
        initial_pose_publications_with_identity(sim)
            .unwrap()
            .map(|(identity, kind, name, metadata)| {
                (identity, kind, name.to_owned(), metadata.clone())
            })
            .collect::<Vec<_>>()
    };
    let snapshots = collect(&sim);
    assert_eq!(snapshots.len(), 2);
    assert_ne!(snapshots[0].0, snapshots[1].0);
    let mut actual_names = snapshots
        .iter()
        .map(|row| row.2.as_str())
        .collect::<Vec<_>>();
    actual_names.sort();
    assert_eq!(
        actual_names,
        ["warehouse_floor::peer", "warehouse_floor::rig"]
    );
    let old_entities = snapshots
        .iter()
        .map(|row| named(&sim, &row.2))
        .collect::<Vec<_>>();
    for ((identity, kind, name, metadata), (legacy_id, legacy)) in
        initial_pose_publications_with_identity(&sim)
            .unwrap()
            .zip(initial_pose_publications(&sim).unwrap())
    {
        assert_eq!(identity, legacy_id);
        assert_eq!(identity, id(&sim, name));
        assert_eq!(kind, EntityKind::Model);
        assert_eq!(
            name.strip_prefix("warehouse_floor::"),
            Some(metadata.model_name.as_str())
        );
        assert!(std::ptr::eq(metadata, legacy));
        assert!(metadata.static_publication.is_some());
    }
    sim.step_exact(3).unwrap();
    let removed_id = id(&sim, "warehouse_floor::rig");
    remove_model(&mut sim, removed_id).unwrap();
    let removed = snapshots
        .iter()
        .find(|row| row.2 == "warehouse_floor::rig")
        .unwrap();
    assert!(entity_by_id(sim.world(), removed.0).is_none());
    let peer = snapshots
        .iter()
        .find(|row| row.2 == "warehouse_floor::peer")
        .unwrap();
    assert!(entity_by_id(sim.world(), peer.0).is_some());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(collect(&sim), snapshots);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (snapshot, old_entity) in snapshots.iter().zip(old_entities) {
        let restored = entity_by_id(sim.world(), snapshot.0).unwrap();
        assert_ne!(restored, old_entity);
        assert_eq!(
            sim.world().get::<EntityKind>(restored),
            Some(&EntityKind::Model)
        );
        assert_eq!(sim.world().get::<Name>(restored).unwrap().0, snapshot.2);
        assert_eq!(
            sim.world().get::<PosePublication>(restored),
            Some(&snapshot.3)
        );
    }
    assert_eq!(collect(&sim), snapshots);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.epoch(), 1);
}
