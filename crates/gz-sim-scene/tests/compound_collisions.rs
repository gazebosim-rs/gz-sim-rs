// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{LoadError, PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

const FIRST: &str = "<collision name='first'><pose>0.5 0 0 0 0 0</pose><density>7</density><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>";
const SECOND: &str = "<collision name='second'><pose>-0.5 0 0 0 0 1.5707963267948966</pose><density>11</density><geometry><box><size>0.2 0.4 0.6</size></box></geometry></collision>";

fn xml(collisions: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='body'><link name='link'><pose>0 0 10 0 0 0</pose><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>{collisions}</link></model></world></sdf>"
    )
}
fn load(source: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(source, "compound.sdf", "default", &NoResources, engine)
}

#[test]
fn other_engines_keep_single_centered_collision_domain() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for (case, source) in [xml(FIRST), xml(&format!("{FIRST}{SECOND}"))]
            .into_iter()
            .enumerate()
        {
            if matches!(engine, PhysicsEngine::Rapier) && case == 0 {
                // M82 admits only this single posed sphere. Its local .5X
                // offset moves geometry, while the reported link stays at Z10.
                let sim = load(&source, engine).unwrap();
                let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
                for (x, expected) in [(0.5, 0.9), (0.56, 0.92)] {
                    // Radius .1 and transverse .06 leave the .08 chord root.
                    let actual = backend
                        .cast_ray([x, 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap()
                        .unwrap();
                    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
                }
                for (name, expected) in [
                    ("default::body::link", [0., 0., 10.]),
                    ("default::body::link::first", [0.5, 0., 10.]),
                ] {
                    let entity = sim
                        .world()
                        .iter_entities()
                        .find(|entity| {
                            entity
                                .get::<gz_sim_scene::Name>()
                                .is_some_and(|n| n.0 == name)
                        })
                        .unwrap();
                    let pose = entity.get::<gz_sim_scene::WorldPose>().unwrap();
                    for (actual, expected) in pose.translation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                    assert_eq!(pose.rotation, [0., 0., 0., 1.]);
                }
            } else if matches!(engine, PhysicsEngine::Rapier) && case == 1 {
                // M84 keeps the same nonidentity link frame and both primitives.
                let sim = load(&source, engine).unwrap();
                let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
                for (x, expected) in [(0.5, 0.9), (0.56, 0.92), (-0.5, 0.7)] {
                    let actual = backend
                        .cast_ray([x, 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap()
                        .unwrap();
                    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
                }
                assert_eq!(
                    backend
                        .cast_ray([0., 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap(),
                    None
                );
                for (name, expected) in [
                    ("default::body", [0., 0., 0.]),
                    ("default::body::link", [0., 0., 10.]),
                    ("default::body::link::first", [0.5, 0., 10.]),
                    ("default::body::link::second", [-0.5, 0., 10.]),
                ] {
                    let entity = sim
                        .world()
                        .iter_entities()
                        .find(|entity| {
                            entity
                                .get::<gz_sim_scene::Name>()
                                .is_some_and(|n| n.0 == name)
                        })
                        .unwrap();
                    let pose = entity.get::<gz_sim_scene::WorldPose>().unwrap();
                    for (actual, expected) in pose.translation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                    let expected = if name.ends_with("::second") {
                        [
                            0.,
                            0.,
                            std::f64::consts::FRAC_1_SQRT_2,
                            std::f64::consts::FRAC_1_SQRT_2,
                        ]
                    } else {
                        [0., 0., 0., 1.]
                    };
                    for (actual, expected) in pose.rotation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                }
                assert_eq!(
                    sim.world()
                        .iter_entities()
                        .filter(|e| e.contains::<gz_sim_scene::BodyHandle>())
                        .count(),
                    1
                );
            } else {
                assert!(matches!(
                    load(&source, engine),
                    Err(LoadError::Unsupported { .. })
                ));
            }
        }
    }
}
