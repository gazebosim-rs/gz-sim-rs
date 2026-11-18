// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use gz_sim_scene::{PhysicsEngine, load_world_with_engine};
use sdformat::NoResources;
use std::time::Duration;

const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><model name='ball'><pose>0 0 5 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>0.2</ixx><iyy>0.2</iyy><izz>0.2</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>";

#[test]
fn rapier_and_tpe_still_load_and_step_without_native_engines() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut simulation = load_world_with_engine(
            XML,
            "no-native.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        simulation.step_exact(10).unwrap();
        assert_eq!(simulation.elapsed(), Duration::from_millis(10));
        assert_eq!(simulation.epoch(), 0);
        let pose = simulation
            .world()
            .iter_entities()
            .find_map(|entity| {
                let name = entity.get::<gz_sim_scene::Name>()?;
                (name.0 == "default::ball::body")
                    .then(|| *entity.get::<gz_sim_scene::WorldPose>().unwrap())
            })
            .unwrap();
        if engine == PhysicsEngine::Rapier {
            assert!(pose.translation[2] < 5.0);
        } else {
            // TPE retains its existing model-motion proxy, without dynamic gravity.
            assert_eq!(pose.translation, [0., 0., 5.]);
        }
    }
}
