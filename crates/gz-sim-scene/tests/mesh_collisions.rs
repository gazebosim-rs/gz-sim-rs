// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{
    Name, PhysicsEngine, VisualGeometry, WorldPose, load_world_with_engine, reset_loaded_world,
};
use sdformat::NoResources;
use std::{fs, time::Duration};
use tempfile::TempDir;

fn surface(temp: &TempDir) {
    // glTF Y-up surface becomes Gazebo Z-up through the loader's admitted basis.
    let positions: [[f32; 3]; 4] = [[-2., 0., -2.], [2., 0., -2.], [2., 0., 2.], [-2., 0., 2.]];
    let mut bytes = Vec::new();
    for p in positions {
        for v in p {
            bytes.extend(v.to_le_bytes());
        }
    }
    bytes.extend([0u8, 2, 1, 0, 3, 2]);
    fs::write(temp.path().join("floor.bin"), bytes).unwrap();
    fs::write(temp.path().join("floor.gltf"),serde_json::json!({
        "asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],
        "nodes":[{"mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],
        "buffers":[{"uri":"floor.bin","byteLength":54}],
        "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":48},{"buffer":0,"byteOffset":48,"byteLength":6}],
        "accessors":[{"bufferView":0,"componentType":5126,"count":4,"type":"VEC3","min":[-2,0,-2],"max":[2,0,2]},
            {"bufferView":1,"componentType":5121,"count":6,"type":"SCALAR"}]
    }).to_string()).unwrap();
}
fn xml(uri: &str) -> String {
    format!("<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity>
    <model name='ground'><static>true</static><pose>2 3 0.5 0 0 0</pose><link name='body'>
    <collision name='mesh'><pose>0 0 1 0 0 0.4</pose><geometry><mesh><uri>{uri}</uri><scale>2 2 2</scale></mesh></geometry></collision>
    <visual name='mesh'><pose>0 0 1 0 0 0.4</pose><geometry><mesh><uri>{uri}</uri><scale>2 2 2</scale></mesh></geometry></visual>
    </link></model><model name='ball'><pose>2 3 3 0 0 0</pose><link name='body'>
    <inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
    <collision name='sphere'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
    </link></model></world></sdf>")
}
fn load(
    temp: &TempDir,
    xml: &str,
    engine: PhysicsEngine,
) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_engine(
        xml,
        &format!("file://{}", temp.path().join("world.sdf").display()),
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
}
fn position(sim: &Simulation, name: &str) -> [f64; 3] {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<WorldPose>()
        .unwrap()
        .translation
}
#[test]
fn static_mesh_contact_uses_collision_pose_without_moving_link_frame_and_reset_owns_geometry() {
    let temp = TempDir::new().unwrap();
    surface(&temp);
    let mut sim = load(&temp, &xml("floor.gltf"), PhysicsEngine::Rapier).unwrap();
    assert_eq!(position(&sim, "default::ground::body"), [2., 3., 0.5]);
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.contains::<VisualGeometry>()
                && e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::ground::body::mesh")
        })
        .unwrap();
    let sdformat::Geometry::Mesh(mesh) = visual.get::<VisualGeometry>().unwrap().0.as_ref() else {
        panic!("expected mesh visual")
    };
    assert_eq!(mesh.uri, "floor.gltf");
    assert_eq!(mesh.scale, [2.; 3]);
    let visual_pose = visual.get::<WorldPose>().unwrap();
    assert_eq!(visual_pose.translation, [2., 3., 1.5]);
    let rotation = visual_pose.rotation;
    assert!((rotation[2] - (0.2_f64).sin()).abs() < 1e-14);
    assert!((rotation[3] - (0.2_f64).cos()).abs() < 1e-14);
    sim.step_exact(1500).unwrap();
    let settled = position(&sim, "default::ball::body");
    assert!((settled[2] - 2.).abs() < 0.01, "{settled:?}");
    assert_eq!(position(&sim, "default::ground::body"), [2., 3., 0.5]);
    fs::remove_file(temp.path().join("floor.gltf")).unwrap();
    fs::remove_file(temp.path().join("floor.bin")).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(position(&sim, "default::ball::body"), [2., 3., 3.]);
    sim.step_exact(1500).unwrap();
    assert_eq!(position(&sim, "default::ball::body"), settled);
}
#[test]
fn missing_mesh_has_entity_diagnostic_but_unsupported_backend_refuses_before_io() {
    let temp = TempDir::new().unwrap();
    let error = load(&temp, &xml("missing.gltf"), PhysicsEngine::Rapier)
        .err()
        .unwrap();
    assert!(
        matches!(&error,gz_sim_scene::LoadError::Mesh{entity,..} if entity=="default::ground::body")
    );
    assert!(error.to_string().contains("missing.gltf"));
    let error = load(&temp, &xml("missing.gltf"), PhysicsEngine::Tpe)
        .err()
        .unwrap();
    assert!(matches!(
        error,
        gz_sim_scene::LoadError::Unsupported {
            feature: "collision geometry",
            ..
        }
    ));
}
#[test]
fn admitted_dynamic_mesh_demo_falls_and_restores_its_loaded_pose() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .canonicalize()
        .unwrap();
    let xml = include_str!("../../../examples/mvp_mesh_collisions.sdf");
    let mut sim = load_world_with_engine(
        xml,
        &format!("file://{}", root.join("mvp_mesh_collisions.sdf").display()),
        "default",
        Duration::from_millis(5),
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let initial = position(&sim, "default::local_mesh::body");
    sim.step_exact(100).unwrap();
    assert!(position(&sim, "default::local_mesh::body")[2] < initial[2] - 0.1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(position(&sim, "default::local_mesh::body"), initial);
}

#[test]
fn multi_link_meshes_exclude_mutual_contacts_but_separate_models_still_collide() {
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .canonicalize()
        .unwrap();
    let link = |name: &str| {
        format!(
            "<link name='{name}'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='mesh'><geometry><mesh><uri>assets/m2/pyramid.gltf</uri></mesh></geometry></collision></link>"
        )
    };
    let grouped = format!(
        "<model name='assembly'>{}{}</model>",
        link("left"),
        link("right")
    );
    let separate = format!(
        "<model name='first'>{}</model><model name='second'>{}</model>",
        link("left"),
        link("right")
    );
    let world = |models: &str| {
        format!(
            "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>{models}</world></sdf>"
        )
    };
    let source = format!("file://{}", directory.join("mesh-groups.sdf").display());
    let load = |models: &str| {
        load_world_with_engine(
            &world(models),
            &source,
            "default",
            Duration::from_millis(1),
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap()
    };
    let mut grouped = load(&grouped);
    let left = position(&grouped, "default::assembly::left");
    let right = position(&grouped, "default::assembly::right");
    assert_eq!(left, right, "fixture solids must overlap");
    grouped.step_exact(100).unwrap();
    assert_eq!(position(&grouped, "default::assembly::left"), left);
    assert_eq!(position(&grouped, "default::assembly::right"), right);
    reset_loaded_world(&mut grouped, |_| ()).unwrap();
    grouped.step_exact(100).unwrap();
    assert_eq!(position(&grouped, "default::assembly::left"), left);
    assert_eq!(position(&grouped, "default::assembly::right"), right);
    let mut separate = load(&separate);
    assert_eq!(position(&separate, "default::first::left"), left);
    assert_eq!(position(&separate, "default::second::right"), right);
    separate.step_exact(100).unwrap();
    let a = position(&separate, "default::first::left");
    let b = position(&separate, "default::second::right");
    let separation = a
        .into_iter()
        .zip(b)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        separation > 0.01,
        "independent mesh bodies must retain contacts: {a:?}, {b:?}"
    );
}
