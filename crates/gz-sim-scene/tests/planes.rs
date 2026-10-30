// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, WorldPose, load_world_with_engine, remove_model,
    reset_loaded_world,
};
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

fn state(sim: &Simulation, name: &str) -> (WorldPose, Velocity) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (
        *entity.get::<WorldPose>().unwrap(),
        *entity.get::<Velocity>().unwrap(),
    )
}

fn assert_state_eq(actual: (WorldPose, Velocity), expected: (WorldPose, Velocity)) {
    assert_eq!(actual.0.translation, expected.0.translation);
    assert_eq!(actual.0.rotation, expected.0.rotation);
    assert_eq!(actual.1.linear, expected.1.linear);
    assert_eq!(actual.1.angular, expected.1.angular);
}

fn values<const N: usize>(value: &serde_json::Value) -> [f64; N] {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}

#[test]
fn loaded_static_planes_match_all_native_frames_and_trajectories() {
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../gz-physics-dart/tests/fixtures/planes.json"
    ))
    .unwrap();
    let rows = fixture["rows"].as_array().unwrap();
    let mut checked = 0;
    for first in rows.iter().filter(|row| row["step"] == 0) {
        let input = &first["input"];
        let xml = xml(
            values(&input["position"]),
            values(&input["orientation"]),
            values(&input["size"]),
            values(&input["ball"]),
        );
        let mut sim = load(&xml, PhysicsEngine::Dart).unwrap();
        let mut previous = 0;
        for row in rows.iter().filter(|row| row["case"] == first["case"]) {
            let step = row["step"].as_u64().unwrap();
            sim.step_exact(step - previous).unwrap();
            previous = step;
            for (name, expected) in [
                ("default::ground::base", &row["ground"]),
                ("default::ball::body", &row["ball"]),
            ] {
                let (pose, velocity) = state(&sim, name);
                let actual = pose
                    .translation
                    .into_iter()
                    .chain(pose.rotation)
                    .chain(velocity.linear)
                    .chain(velocity.angular);
                let expected = values::<3>(&expected["position"])
                    .into_iter()
                    .chain(values::<4>(&expected["orientation"]))
                    .chain(values::<3>(&expected["velocity"]))
                    .chain(values::<3>(&expected["angular_velocity"]));
                for (actual, expected) in actual.zip(expected) {
                    assert!(
                        (actual - expected).abs() <= 1e-14,
                        "{} step {step} {name}: {actual} != {expected}",
                        row["case"]
                    );
                }
            }
            checked += 1;
        }
    }
    assert_eq!(checked, 49);
}

#[test]
fn plane_removal_and_loaded_reset_reuse_existing_body_lifecycle() {
    let xml = xml(
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
        [2.0; 2],
        [0.0, 0.0, 3.0],
    );
    let mut sim = load(&xml, PhysicsEngine::Dart).unwrap();
    let original = state(&sim, "default::ground::base");
    sim.step_exact(2000).unwrap();
    let settled = state(&sim, "default::ball::body");
    assert!((settled.0.translation[2] - 1.5).abs() < 0.01);
    assert_state_eq(state(&sim, "default::ground::base"), original);
    let ground = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ground"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut sim, ground).unwrap();
    sim.step_exact(50).unwrap();
    assert!(state(&sim, "default::ball::body").0.translation[2] < settled.0.translation[2] - 0.005);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_state_eq(state(&sim, "default::ground::base"), original);
    sim.step_exact(2000).unwrap();
    assert_state_eq(state(&sim, "default::ball::body"), settled);
}

#[test]
fn unsupported_plane_inputs_fail_at_loading() {
    let xml = xml([0.0; 3], [0.0, 0.0, 0.0, 1.0], [2.0; 2], [0.0, 0.0, 3.0]);
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Mujoco,
        PhysicsEngine::Tpe,
    ] {
        assert!(load(&xml, engine).is_err(), "{engine:?}");
    }
    for invalid in [
        xml.replace("<static>true</static>", "<static>false</static>"),
        xml.replace("<plane>", "<plane><normal>1 0 0</normal>"),
        xml.replace("<plane>", "<plane><normal>0 0 0</normal>"),
        xml.replace("<size>2 2</size>", "<size>1e100 2</size>"),
        xml.replace(
            "<link name='base'>",
            "<link name='base'><inertial><mass>2</mass></inertial>",
        ),
        xml.replace(
            "<collision name='shape'>",
            "<collision name='shape'><pose>1 0 0 0 0 0</pose>",
        ),
    ] {
        assert!(load(&invalid, PhysicsEngine::Dart).is_err(), "{invalid}");
    }
}

#[test]
fn unsupported_geometry_keeps_priority_over_offset_inertia() {
    let xml = xml([0.0; 3], [0.0, 0.0, 0.0, 1.0], [2.0; 2], [0.0, 0.0, 3.0])
        .replace(
            "<plane><size>2 2</size></plane>",
            "<mesh><uri>file://unused.dae</uri></mesh>",
        )
        .replace(
            "<link name='base'>",
            "<link name='base'><inertial><pose>1 0 0 0 0 0</pose></inertial>",
        );
    assert!(matches!(
        load(&xml, PhysicsEngine::Dart),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "collision geometry",
            ..
        })
    ));
}

#[test]
fn nested_plane_inherits_static_from_its_parent_model() {
    let xml = xml(
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
        [2.0; 2],
        [0.0, 0.0, 3.0],
    )
    .replace(
        "<model name='ground'><static>true</static>",
        "<model name='anchor'><static>true</static><model name='ground'><static>false</static>",
    )
    .replace(
        "</link></model><model name='ball'>",
        "</link></model></model><model name='ball'>",
    );
    let mut sim = load(&xml, PhysicsEngine::Dart).unwrap();
    let original = state(&sim, "default::anchor::ground::base");
    sim.step_exact(2000).unwrap();
    assert_state_eq(state(&sim, "default::anchor::ground::base"), original);
    assert!((state(&sim, "default::ball::body").0.translation[2] - 1.5).abs() < 0.01);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_state_eq(state(&sim, "default::anchor::ground::base"), original);
}
