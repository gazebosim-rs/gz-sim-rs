// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{
    LoadError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_engine, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const WORLD: &str = include_str!("fixtures/fall.sdf");
const PHYSICS: &str = "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>";

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_engine(
        xml,
        "world-systems.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
}
fn configured(configuration: &str) -> String {
    WORLD.replace(
        "<world name=\"default\">",
        &format!("<world name=\"default\">{configuration}"),
    )
}
fn states(sim: &mut Simulation) -> Vec<(String, [f64; 3], [f64; 3])> {
    let mut rows: Vec<_> = sim
        .world_mut()
        .query::<(&Name, &WorldPose, &Velocity)>()
        .iter(sim.world())
        .map(|(name, pose, velocity)| (name.0.clone(), pose.translation, velocity.linear))
        .collect();
    rows.sort_by(|a, b| a.0.cmp(&b.0));
    rows
}

#[test]
fn physics_metadata_reuses_selected_backend_and_reset_without_duplicate_systems() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Tpe,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut plain = load(WORLD, engine).unwrap();
        let mut configured = load(&configured(PHYSICS), engine).unwrap();
        assert_eq!(
            plain.scheduled_system_counts(),
            configured.scheduled_system_counts()
        );
        assert_eq!(states(&mut plain), states(&mut configured));
        for _ in 0..3 {
            plain.step_exact(4).unwrap();
            configured.step_exact(4).unwrap();
            assert_eq!(plain.elapsed(), configured.elapsed());
            assert_eq!(states(&mut plain), states(&mut configured));
            reset_loaded_world(&mut plain, |_| ()).unwrap();
            reset_loaded_world(&mut configured, |_| ()).unwrap();
            assert_eq!(
                plain.scheduled_system_counts(),
                configured.scheduled_system_counts()
            );
            assert_eq!(states(&mut plain), states(&mut configured));
        }
    }
}

#[test]
fn only_empty_canonical_configuration_is_accepted() {
    let with_comment = PHYSICS.replace("/>", "> \n<!-- legacy metadata -->\t</plugin>");
    load(&configured(&with_comment), PhysicsEngine::Rapier).unwrap();
    for record in [
        PHYSICS.replace(
            "/>",
            "><engine><filename>gz-physics-dartsim-plugin</filename></engine></plugin>",
        ),
        PHYSICS.replace("/>", ">text</plugin>"),
        PHYSICS.replace("/>", ">\u{a0}</plugin>"),
        PHYSICS.replace("<plugin ", "<plugin extra='x' "),
        PHYSICS.replace("<plugin ", "<plugin xmlns='urn:extra' "),
        PHYSICS.replace("<plugin ", "<plugin xmlns:extra='urn:extra' "),
    ] {
        assert!(
            matches!(
                load(&configured(&record), PhysicsEngine::Rapier),
                Err(LoadError::Unsupported {
                    feature: "Physics system configuration",
                    ..
                })
            ),
            "{record}"
        );
    }
    for record in [
        PHYSICS.replace("systems::Physics", "systems::Other"),
        PHYSICS.replace("gz-sim-physics-system", "/tmp/libgz-sim-physics-system.so"),
    ] {
        assert!(matches!(
            load(&configured(&record), PhysicsEngine::Rapier),
            Err(LoadError::Unsupported {
                feature: "world plugins",
                ..
            })
        ));
    }
    assert!(matches!(
        load(&configured(&PHYSICS.repeat(2)), PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "duplicate Physics configuration",
            ..
        })
    ));
}
