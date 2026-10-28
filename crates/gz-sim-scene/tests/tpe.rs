// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    EntityKind, ImuSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_engine,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_engine(
        xml,
        "tpe.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Tpe,
    )
}

fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

#[test]
fn selected_tpe_world_has_no_gravity_dynamics_and_restores_removed_entities() {
    let mut sim = load(include_str!("fixtures/fall.sdf")).unwrap();
    assert!(
        !sim.world()
            .contains_resource::<gz_physics_rapier::PhysicsBackend>()
    );
    let ball = named(&sim, "default::ball");
    let ball_id = sim.world().get::<EntityId>(ball).unwrap().get();
    let link = named(&sim, "default::ball::body");
    let link_id = sim.world().get::<EntityId>(link).unwrap().get();
    sim.step_exact(100).unwrap();
    assert_eq!(sim.elapsed(), Duration::from_millis(100));
    assert_eq!(
        sim.world().get::<WorldPose>(link).unwrap().translation,
        [0.0, 0.0, 5.0]
    );
    assert_eq!(sim.world().get::<Velocity>(link).unwrap().linear, [0.0; 3]);
    let sample = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap();
    assert_eq!(sample.time, Duration::from_millis(100));
    assert_eq!(
        sample.reading.as_ref().unwrap().specific_force,
        [0.0, 0.0, 9.81]
    );
    let removed = remove_model(&mut sim, ball_id).unwrap();
    assert!(removed.contains(&link_id));
    assert!(sim.world().get_entity(link).is_err());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = named(&sim, "default::ball::body");
    assert_ne!(restored, link);
    assert_eq!(
        sim.world().get::<EntityId>(restored).unwrap().get(),
        link_id
    );
    assert_eq!(
        sim.world().get::<EntityKind>(restored),
        Some(&EntityKind::Link)
    );
    assert!(
        sim.world_mut()
            .query::<&ImuSample>()
            .iter(sim.world())
            .next()
            .is_none()
    );
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<WorldPose>(restored).unwrap().translation,
        [0.0, 0.0, 5.0]
    );
    assert_eq!(
        sim.world_mut()
            .query::<&ImuSample>()
            .single(sim.world())
            .unwrap()
            .epoch,
        1
    );
}

#[test]
fn nested_models_are_explicitly_unsupported_by_tpe_loader() {
    let xml = "<sdf version='1.12'><world name='default'><model name='outer'><link name='body'/><model name='inner'><link name='body'/></model></model></world></sdf>";
    assert!(matches!(
        load(xml),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "TPE nested models",
            ..
        })
    ));
}

#[test]
fn model_motion_keeps_fixed_link_frames_and_samples_committed_state() {
    let xml = include_str!("fixtures/fall.sdf")
        .replace("<pose>0 0 5 0 0 0</pose>", "<pose>0 0 5 0 0 0.5</pose>")
        .replace(
            "<link name=\"body\">",
            "<link name=\"body\"><pose>0.5 0 0 0 0 0.7</pose>",
        );
    let mut sim = load(&xml).unwrap();
    let model = named(&sim, "default::ball");
    let initial = *sim.world().get::<WorldPose>(model).unwrap();
    sim.world_mut().entity_mut(model).insert(
        gz_physics_tpe::Motion::new(gz_physics_rapier::BodyState {
            position: initial.translation,
            orientation: initial.rotation,
            velocity: [1.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 2.0],
        })
        .unwrap(),
    );
    sim.step_exact(100).unwrap();
    let link = named(&sim, "default::ball::body");
    let pose = sim.world().get::<WorldPose>(link).unwrap();
    for (actual, expected) in
        pose.translation
            .iter()
            .zip([0.1 + 0.5 * 0.7_f64.cos(), 0.5 * 0.7_f64.sin(), 5.0])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    let expected = glam::DQuat::from_rotation_z(1.4).to_array();
    for (actual, expected) in pose.rotation.iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-12);
    }
    let velocity = sim.world().get::<Velocity>(link).unwrap();
    assert_eq!(velocity.linear, [1.0, 0.0, 0.0]);
    assert_eq!(velocity.angular, [0.0, 0.0, 2.0]);
    let sample = sim
        .world_mut()
        .query::<&ImuSample>()
        .single(sim.world())
        .unwrap();
    let reading = sample.reading.as_ref().unwrap();
    for (actual, expected) in reading.specific_force.iter().zip([0.0, 0.0, 9.81]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    for (actual, expected) in reading
        .orientation
        .iter()
        .zip(glam::DQuat::from_rotation_z(0.2).to_array())
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let model = named(&sim, "default::ball");
    assert_eq!(
        *sim.world()
            .get::<gz_physics_tpe::Motion>(model)
            .unwrap()
            .state(),
        gz_physics_rapier::BodyState {
            position: initial.translation,
            orientation: initial.rotation,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3]
        }
    );
}

#[test]
fn pinned_native_world_model_and_link_motion_match_loaded_tpe_scene() {
    let reference: serde_json::Value = serde_json::from_str(include_str!(
        "../../../planning/reference/tpe-scene-motion.json"
    ))
    .unwrap();
    let xml = include_str!("fixtures/fall.sdf")
        .replace("<pose>0 0 5 0 0 0</pose>", "<pose>0 0 5 0 0 0.5</pose>")
        .replace(
            "<link name=\"body\">",
            "<link name=\"body\"><pose>0.5 0 0 0 0 0.7</pose>",
        );
    for row in reference["rows"].as_array().unwrap() {
        let mut sim = load(&xml).unwrap();
        let model = named(&sim, "default::ball");
        let initial = *sim.world().get::<WorldPose>(model).unwrap();
        let moving = row["moving"].as_f64().unwrap();
        sim.world_mut().entity_mut(model).insert(
            gz_physics_tpe::Motion::new(gz_physics_rapier::BodyState {
                position: initial.translation,
                orientation: initial.rotation,
                velocity: [moving, 0.0, 0.0],
                angular_velocity: [0.0, 0.0, 2.0 * moving],
            })
            .unwrap(),
        );
        sim.step_exact(100).unwrap();
        let link = named(&sim, "default::ball::body");
        let pose = sim.world().get::<WorldPose>(link).unwrap();
        for (values, key) in [
            (&pose.translation[..], "position"),
            (&pose.rotation[..], "orientation"),
        ] {
            let expected = row[key].as_array().unwrap();
            assert_eq!(values.len(), expected.len());
            for (actual, expected) in values.iter().zip(expected) {
                assert!(
                    (actual - expected.as_f64().unwrap()).abs() < 1e-12,
                    "{key}: {row}"
                );
            }
        }
    }
}

#[test]
fn nonfinite_sensor_gravity_is_rejected_for_tpe_before_construction() {
    let xml = include_str!("fixtures/fall.sdf")
        .replace("<gravity>0 0 -9.81</gravity>", "<gravity>0 0 nan</gravity>");
    let Err(gz_sim_scene::LoadError::Sdf(error)) = load(&xml) else {
        panic!("nonfinite gravity must fail during SDF vector parsing");
    };
    assert_eq!(error.code, sdformat::ErrorCode::InvalidValue);
    assert!(error.message.contains("expected finite vector"));
}
