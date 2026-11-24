// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ImuSample, InputError, LoadError, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_engine, load_world_with_sdf_step, queue_link_world_velocity,
    queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const MOTION: &str = r#"<plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system">
    <linear_velocity>3 -4 0</linear_velocity><angular_velocity>0 0 2</angular_velocity>
</plugin>"#;

fn source(plugin: &str) -> String {
    include_str!("fixtures/fall.sdf").replace(
        "<model name=\"ball\">",
        &format!("<model name=\"ball\">{plugin}"),
    )
}

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(xml, "twist.sdf", "default", &NoResources, engine)
}

#[test]
fn first_imu_observation_uses_prescribed_twist_on_load_and_reset() {
    let mut sim = load(&source(MOTION), PhysicsEngine::MultiFidelity).unwrap();
    for epoch in 0..2 {
        sim.step_exact(1).unwrap();
        let sample = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<ImuSample>())
            .unwrap();
        assert_eq!(sample.epoch, epoch);
        let reading = sample.reading.as_ref().unwrap();
        for (value, expected) in reading.specific_force.into_iter().zip([0.0, 0.0, 9.81]) {
            assert!((value - expected).abs() < 1e-10, "{value} != {expected}");
        }
        assert!((reading.angular_velocity[2] - 2.0).abs() < 1e-10);
        sim.reset().unwrap();
    }
}

#[test]
fn prescribed_motion_requires_explicit_engine_and_closed_configuration() {
    let xml = source(MOTION);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(matches!(
            load(&xml, engine),
            Err(LoadError::Unsupported {
                feature: "PrescribedMotion requires the multifidelity or distributed engine",
                ..
            })
        ));
    }
    for invalid in [
        MOTION.replace("3 -4 0", "0 0"),
        MOTION.replace("3 -4 0", "0 0 0 0"),
        MOTION.replace("3 -4 0", "oops 0 0"),
        MOTION.replace("<linear_velocity>", "<linear_velocity frame='body'>"),
        MOTION.replace("<linear_velocity>", "<linear_velocity><x/>"),
        MOTION.replace("</plugin>", "<other>0</other></plugin>"),
        MOTION.replace(
            "</plugin>",
            "<linear_velocity>0 0 0</linear_velocity></plugin>",
        ),
        format!("{MOTION}{MOTION}"),
    ] {
        assert!(
            matches!(
                load(&source(&invalid), PhysicsEngine::MultiFidelity),
                Err(LoadError::Unsupported { .. })
            ),
            "{invalid}"
        );
    }
}

#[test]
fn prescribed_motion_refuses_missing_and_nonprimitive_colliders() {
    let sphere = "<collision name=\"shape\"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>";
    let xml = source(MOTION);
    for collider in [
        "",
        "<collision name='surface'><geometry><plane><normal>0 0 1</normal><size>2 2</size></plane></geometry></collision>",
    ] {
        let invalid = xml.replace(sphere, collider);
        assert_ne!(invalid, xml);
        assert!(matches!(
            load(&invalid, PhysicsEngine::MultiFidelity),
            Err(LoadError::Unsupported {
                feature: "PrescribedMotion requires one primitive collider",
                ..
            })
        ));
    }
}

#[test]
fn initial_link_velocity_accounts_for_admitted_center_offset() {
    // The loader admits offsets within frame-composition roundoff (1e-12 m).
    let xml = source(MOTION).replace("<inertial>", "<inertial><pose>8e-13 0 0 0 0 0</pose>");
    let sim = load(&xml, PhysicsEngine::MultiFidelity).unwrap();
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::ball::body")
        })
        .unwrap();
    let velocity = link.get::<Velocity>().unwrap();
    assert!((velocity.linear[1] - (-4.0 - 1.6e-12)).abs() < 1e-14);
    let overflow = xml
        .replace("3 -4 0", &format!("0 -{} 0", f64::MAX))
        .replace(">0 0 2<", ">0 0 1e308<");
    assert!(matches!(
        load(&overflow, PhysicsEngine::MultiFidelity),
        Err(LoadError::Unsupported {
            feature: "unrepresentable prescribed link velocity",
            ..
        })
    ));
}

#[test]
fn prescribed_scene_preflights_velocity_at_the_selected_world_step() {
    let fast = source(&MOTION.replace("0 0 2", "0 0 900"));
    assert!(matches!(
        load(&fast, PhysicsEngine::MultiFidelity),
        Err(LoadError::Physics(gz_physics::PhysicsError::InvalidValue))
    ));
    let mut sim = load_world_with_engine(
        &fast,
        "twist.sdf",
        "default",
        Duration::from_micros(100),
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    sim.step_exact(1).unwrap();
    let overspeed = source(&MOTION.replace("3 -4 0", "500 0 0"));
    assert!(matches!(
        load(&overspeed, PhysicsEngine::MultiFidelity),
        Err(LoadError::Physics(gz_physics::PhysicsError::InvalidValue))
    ));
}

#[test]
fn velocity_replacement_survives_wrench_refusal_and_resets_to_construction_twist() {
    let mut sim = load(&source(MOTION), PhysicsEngine::MultiFidelity).unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::ball::body")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let changed = [0.0, 0.0, 4.0];
    for reconstruct in [false, true] {
        queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]).unwrap();
        queue_link_world_velocity(&mut sim, id, changed, [0.0; 3]).unwrap();
        assert!(matches!(
            queue_link_world_wrench(&mut sim, id, [0.0; 3], [1.0; 3]),
            Err(InputError::Unsupported(
                "prescribed kinematic external wrench"
            ))
        ));
        sim.set_paused(true);
        sim.tick().unwrap();
        let body = entity_by_id(sim.world(), id).unwrap();
        assert_eq!(
            sim.world().get::<Velocity>(body).unwrap().linear,
            [3.0, -4.0, 0.0]
        );
        sim.step_exact(2).unwrap();
        assert_eq!(sim.world().get::<Velocity>(body).unwrap().linear, changed);
        let position = sim.world().get::<WorldPose>(body).unwrap().translation;
        assert!((position[2] - 5.008).abs() < 1e-12);
        queue_link_world_velocity(&mut sim, id, [100.0; 3], [0.0; 3]).unwrap();
        if reconstruct {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        } else {
            sim.reset().unwrap();
        }
        let body = entity_by_id(sim.world(), id).unwrap();
        assert_eq!(
            sim.world().get::<WorldPose>(body).unwrap().translation,
            [0.0, 0.0, 5.0]
        );
        let velocity = sim.world().get::<Velocity>(body).unwrap();
        assert_eq!(velocity.linear, [3.0, -4.0, 0.0]);
        assert_eq!(velocity.angular, [0.0, 0.0, 2.0]);
    }
    sim.step_exact(1).unwrap();
    let body = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(
        sim.world().get::<Velocity>(body).unwrap().linear,
        [3.0, -4.0, 0.0]
    );
}

#[test]
fn rejected_twists_preserve_the_prior_valid_command_including_at_the_com_limit() {
    let xml = source(MOTION).replace("<inertial>", "<inertial><pose>8e-13 0 0 0 0 0</pose>");
    let mut sim = load(&xml, PhysicsEngine::MultiFidelity).unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::ball::body")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let changed = [1.0, 0.0, 0.0];
    queue_link_world_velocity(&mut sim, id, changed, [0.0; 3]).unwrap();
    for (linear, angular) in [
        ([0.0; 3], [0.0, 0.0, 900.0]),
        ([500.0, 0.0, 0.0], [0.0; 3]),
        // The origin is at the speed cap; w cross r puts the COM above it.
        ([0.0, 400.0, 0.0], [0.0, 0.0, 500.0]),
    ] {
        assert!(matches!(
            queue_link_world_velocity(&mut sim, id, linear, angular),
            Err(InputError::InvalidValue)
        ));
    }
    sim.step_exact(1).unwrap();
    let entity = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(sim.world().get::<Velocity>(entity).unwrap().linear, changed);
    assert!(sim.fault().is_none());
}

#[test]
fn initial_velocity_seeds_a_dynamic_sphere_once_and_replays_on_reset() {
    let plugin = MOTION
        .replace("PrescribedMotion", "InitialVelocity")
        .replace("prescribed-motion", "initial-velocity");
    let xml = source(&plugin);
    let mut sim = load(&xml, PhysicsEngine::MultiFidelity).unwrap();
    let velocity = |sim: &Simulation| {
        sim.world()
            .iter_entities()
            .find_map(|entity| {
                (entity.get::<Name>()?.0 == "default::ball::body")
                    .then(|| entity.get::<Velocity>().unwrap().linear)
            })
            .unwrap()
    };
    let mut first = None;
    for _ in 0..2 {
        assert_eq!(velocity(&sim), [3., -4., 0.]);
        sim.step_exact(10).unwrap();
        let falling = velocity(&sim);
        assert!(falling[2] < -0.09 && falling[2] > -0.11);
        assert_eq!((falling[0], falling[1]), (3., -4.));
        if let Some(previous) = first {
            assert_eq!(falling, previous);
        }
        first = Some(falling);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
    for invalid in [
        format!("{plugin}{plugin}"),
        plugin.replace("3 -4 0", "inf 0 0"),
        plugin.replace("</plugin>", "<extra/></plugin>"),
    ] {
        assert!(load(&source(&invalid), PhysicsEngine::MultiFidelity).is_err());
    }
    assert!(load(&xml, PhysicsEngine::Rapier).is_err());
}
