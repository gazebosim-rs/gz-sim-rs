// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, load_world_with_engine};
use sdformat::NoResources;
use std::time::Duration;

fn xml(position: [f64; 3], orientation: [f64; 4], size: [f64; 2], ball: [f64; 3]) -> String {
    let [x, y, z] = position;
    let [qx, qy, qz, qw] = orientation;
    let [sx, sy] = size;
    let [bx, by, bz] = ball;
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity>
        <model name='ground'><static>true</static><link name='base'>
        <pose rotation_format='quat_xyzw'>{x} {y} {z} {qx} {qy} {qz} {qw}</pose>
        <collision name='shape'><geometry><plane><size>{sx} {sy}</size></plane></geometry></collision>
        </link></model><model name='ball'><link name='body'><pose>{bx} {by} {bz} 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
        <collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
        </link></model></world></sdf>"
    )
}

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_engine(
        xml,
        "plane.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
}

#[test]
fn unsupported_plane_inputs_fail_at_loading() {
    let xml = xml([0.0; 3], [0.0, 0.0, 0.0, 1.0], [2.0; 2], [0.0, 0.0, 3.0]);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(load(&xml, engine).is_err(), "{engine:?}");
    }
}
