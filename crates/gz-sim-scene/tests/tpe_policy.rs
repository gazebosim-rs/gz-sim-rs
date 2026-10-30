// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_tpe::Motion;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ImuSample, InputError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_engine,
    load_world_with_sdf_step, queue_link_world_velocity, queue_link_world_wrench,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

fn named(sim: &Simulation, name: &str) -> (bevy_ecs::entity::Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}

fn load(fixed: bool, model_rotation: &str, link_rotation: &str) -> Simulation {
    let xml = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='robot'><static>{fixed}</static><pose>2 3 4 {model_rotation}</pose><link name='body'><pose>0 0 0 {link_rotation}</pose><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><sensor name='imu' type='imu'><update_rate>1000</update_rate></sensor></link></model></world></sdf>"
    );
    load_world_with_engine(
        &xml,
        "tpe-policy",
        "default",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap()
}

#[test]
fn static_scene_ignores_stored_motion_and_keeps_sensor_and_original_ids_on_reset() {
    let mut sim = load(true, "0 0 0.5", "0.7 0 0");
    let (_, model_id) = named(&sim, "default::robot");
    let (link, link_id) = named(&sim, "default::robot::body");
    let initial = *sim.world().get::<WorldPose>(link).unwrap();
    for epoch in 0..2 {
        let (model, actual_model_id) = named(&sim, "default::robot");
        let (link, actual_link_id) = named(&sim, "default::robot::body");
        assert_eq!(actual_model_id, model_id);
        assert_eq!(actual_link_id, link_id);
        sim.world_mut()
            .get_mut::<Motion>(model)
            .unwrap()
            .set_world_velocity([f64::MAX; 3], [f64::MAX; 3])
            .unwrap();
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link_id, [1.0; 3], [0.0; 3]),
            Err(InputError::Unsupported(_))
        ));
        sim.step_exact(20).unwrap();
        let pose = sim.world().get::<WorldPose>(link).unwrap();
        assert_eq!(pose.translation, initial.translation);
        for (actual, expected) in pose.rotation.into_iter().zip(initial.rotation) {
            assert!((actual - expected).abs() < 1e-12);
        }
        let velocity = sim.world().get::<Velocity>(link).unwrap();
        assert_eq!(velocity.linear, [0.0; 3]);
        assert_eq!(velocity.angular, [0.0; 3]);
        let sample = sim
            .world_mut()
            .query::<&ImuSample>()
            .single(sim.world())
            .unwrap();
        assert_eq!(sample.time, Duration::from_millis(20));
        assert_eq!(sample.epoch, epoch);
        let reading = sample.reading.as_ref().unwrap();
        assert_eq!(reading.angular_velocity, [0.0; 3]);
        for value in reading.specific_force {
            assert!(value.abs() < 1e-12);
        }
        assert!(sim.fault().is_none());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(named(&sim, "default::robot").1, model_id);
        assert_eq!(named(&sim, "default::robot::body").1, link_id);
    }
}

#[test]
fn rotated_model_and_link_commands_use_world_axes_through_pause_and_reset() {
    for (model_rotation, link_rotation) in [
        ("0 0 1.5707963267948966", "0 0 0"),
        ("0 0 0", "0 0 1.5707963267948966"),
        ("0.4 -0.2 0.5", "-0.3 0.2 1.1"),
    ] {
        let mut sim = load(false, model_rotation, link_rotation);
        let (_, link_id) = named(&sim, "default::robot::body");
        let initial = *sim
            .world()
            .get::<WorldPose>(entity_by_id(sim.world(), link_id).unwrap())
            .unwrap();
        let linear = [1.0, -2.0, 0.5];
        let angular = [0.3, -0.4, 0.5];
        for reset in 0..3 {
            queue_link_world_velocity(&mut sim, link_id, linear, angular).unwrap();
            sim.tick().unwrap();
            let link = entity_by_id(sim.world(), link_id).unwrap();
            assert_eq!(sim.world().get::<Velocity>(link).unwrap().linear, [0.0; 3]);
            sim.step_exact(20).unwrap();
            let pose = sim.world().get::<WorldPose>(link).unwrap();
            for ((actual, initial), velocity) in pose
                .translation
                .into_iter()
                .zip(initial.translation)
                .zip(linear)
            {
                assert!((actual - initial - velocity * 0.02).abs() < 1e-12);
            }
            let expected = DQuat::from_scaled_axis(DVec3::from_array(angular) * 0.02)
                * DQuat::from_array(initial.rotation);
            for (actual, expected) in pose.rotation.into_iter().zip(expected.to_array()) {
                assert!((actual - expected).abs() < 1e-12);
            }
            let velocity = sim.world().get::<Velocity>(link).unwrap();
            assert_eq!(velocity.linear, linear);
            assert_eq!(velocity.angular, angular);
            let reading = sim
                .world_mut()
                .query::<&ImuSample>()
                .single(sim.world())
                .unwrap()
                .reading
                .as_ref()
                .unwrap();
            let gyro = expected.inverse() * DVec3::from_array(angular);
            for (actual, expected) in reading.angular_velocity.into_iter().zip(gyro.to_array()) {
                assert!((actual - expected).abs() < 1e-12);
            }
            for value in reading.specific_force {
                assert!(value.abs() < 1e-12);
            }
            queue_link_world_velocity(&mut sim, link_id, [9.0; 3], [0.0; 3]).unwrap();
            if reset == 0 {
                sim.reset().unwrap();
            } else {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            }
            assert_eq!(named(&sim, "default::robot::body").1, link_id);
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
}

#[test]
fn original_rotated_native_rows_remain_evidence_with_explicit_world_axis_differences() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("fixtures/physics_inputs.json")).unwrap();
    for row in rows
        .iter()
        .filter(|r| r["engine"] == "tpe" && r["frame"] != "identity")
    {
        let plugin = "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'><engine><filename>gz-physics-tpe-plugin</filename></engine></plugin>";
        let xml = row["xml"].as_str().unwrap();
        assert_eq!(xml.matches(plugin).count(), 1);
        let mut sim = load_world_with_sdf_step(
            xml,
            "d23-native",
            "default",
            &NoResources,
            PhysicsEngine::Tpe,
        )
        .unwrap();
        let (link, id) = named(&sim, "default::robot::body");
        let moving = matches!(row["mode"].as_str().unwrap(), "velocity" | "both");
        let angular = if row["mode"] == "angular" {
            [0.0, 0.0, 1.0]
        } else {
            [0.0; 3]
        };
        queue_link_world_velocity(
            &mut sim,
            id,
            if moving { [1.0, 0.0, 0.0] } else { [0.0; 3] },
            angular,
        )
        .unwrap();
        if matches!(row["mode"].as_str().unwrap(), "force" | "torque" | "both") {
            assert!(matches!(
                queue_link_world_wrench(&mut sim, id, [2.0, 0.0, 0.0], [0.0; 3]),
                Err(InputError::Unsupported(_))
            ));
        }
        for native in row["native"].as_array().unwrap() {
            sim.step_exact(1).unwrap();
            let pose = sim.world().get::<WorldPose>(link).unwrap();
            let velocity = sim.world().get::<Velocity>(link).unwrap();
            let x = if moving {
                native["iteration"].as_f64().unwrap() * 0.01
            } else {
                0.0
            };
            assert!((pose.translation[0] - x).abs() < 1e-12);
            assert!(pose.translation[1].abs() < 1e-12);
            assert_eq!(
                velocity.linear,
                if moving { [1.0, 0.0, 0.0] } else { [0.0; 3] }
            );
            assert_eq!(velocity.angular, angular);
            if moving {
                if row["frame"] == "model90" {
                    assert!((native["x"].as_f64().unwrap() - x).abs() < 1e-12);
                    assert!((native["vx"].as_f64().unwrap() + 1.0).abs() < 1e-12);
                } else {
                    assert!(native["x"].as_f64().unwrap().abs() < 1e-12);
                    assert!((native["y"].as_f64().unwrap() + x).abs() < 1e-12);
                    assert!((native["vy"].as_f64().unwrap() + 1.0).abs() < 1e-12);
                }
            }
        }
    }
}

#[test]
fn empty_static_model_remains_fixed_without_contact_bounds_across_resets() {
    let xml = "<sdf version='1.12'><world name='default'><model name='empty'><static>true</static><pose>2 3 4 0.2 -0.3 0.4</pose></model></world></sdf>";
    let mut sim = load_world_with_engine(
        xml,
        "empty-static",
        "default",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let (model, id) = named(&sim, "default::empty");
    let initial = *sim.world().get::<WorldPose>(model).unwrap();
    for epoch in 0..3 {
        let (model, actual_id) = named(&sim, "default::empty");
        assert_eq!(actual_id, id);
        assert!(
            sim.world()
                .get::<gz_physics_tpe::ModelBounds>(model)
                .is_none()
        );
        sim.world_mut()
            .get_mut::<Motion>(model)
            .unwrap()
            .set_world_velocity([f64::MAX; 3], [f64::MAX; 3])
            .unwrap();
        sim.tick().unwrap();
        for steps in [0, 20] {
            sim.step_exact(steps).unwrap();
            let state = sim.world().get::<Motion>(model).unwrap().state();
            assert_eq!(
                state.position.map(f64::to_bits),
                initial.translation.map(f64::to_bits)
            );
            assert_eq!(
                state.orientation.map(f64::to_bits),
                initial.rotation.map(f64::to_bits)
            );
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
            let pose = sim.world().get::<WorldPose>(model).unwrap();
            assert_eq!(pose.translation, initial.translation);
            assert_eq!(pose.rotation, initial.rotation);
            assert!(
                sim.world()
                    .resource::<gz_physics_tpe::Contacts>()
                    .as_slice()
                    .is_empty()
            );
            assert!(sim.fault().is_none());
        }
        if epoch == 0 {
            sim.reset().unwrap();
        } else {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert_eq!(sim.elapsed(), Duration::ZERO);
        let (model, _) = named(&sim, "default::empty");
        assert_eq!(
            sim.world().get::<Motion>(model).unwrap().state().velocity,
            [0.0; 3]
        );
    }
}
