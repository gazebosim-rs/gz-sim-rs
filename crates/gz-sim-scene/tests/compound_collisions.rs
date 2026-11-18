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
        for source in [xml(FIRST), xml(&format!("{FIRST}{SECOND}"))] {
            assert!(matches!(
                load(&source, engine),
                Err(LoadError::Unsupported { .. })
            ));
        }
    }
}
