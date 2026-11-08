// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_engine, remove_model,
    reset_loaded_world,
};
use std::{fs, time::Duration};

fn load(root: &std::path::Path, xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    let source = format!("file://{}", root.join("world.sdf").display());
    load_world_with_engine(
        xml,
        &source,
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}
fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("named body")
        .get::<WorldPose>()
        .expect("body pose")
}
fn ray(sim: &Simulation, origin: [f64; 3]) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, [0., 0., -1.], 20., None)
        .expect("native f64 ray")
}

#[test]
fn open_obj_surface_uses_pose_scale_contact_and_retained_reset_geometry() {
    let dir = tempfile::tempdir().expect("fixture directory");
    fs::write(
        dir.path().join("floor.obj"),
        b"v -2 -2 0\nv 2 -2 0\nv 2 2 0\nv -2 2 0\nf 1 2 3\nf 1 3 4\n",
    )
    .expect("write surface");
    let xml = "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><model name='ground'><static>true</static><pose>2 3 0.5 0 0 0</pose><link name='body'><collision name='mesh'><pose>0 0 1 0 0 0.4</pose><geometry><mesh><uri>floor.obj</uri><scale>2 3 1</scale></mesh></geometry></collision></link></model><model name='ball'><pose>2 3 3 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='sphere'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>";
    let mut sim = load(dir.path(), xml).expect("open static OBJ");
    // Avoid the sphere: the known surface remains at world z=1.5 after XY scale/yaw.
    assert!((ray(&sim, [3., 3., 5.]).expect("surface hit") - 3.5).abs() < 1e-12);
    assert_eq!(
        pose(&sim, "default::ground::body").translation,
        [2., 3., 0.5]
    );
    sim.step_exact(1500).expect("settle sphere");
    let settled = pose(&sim, "default::ball::body");
    assert!((settled.translation[2] - 2.).abs() < 0.01);
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ground"))
        .expect("ground model")
        .get::<EntityId>()
        .expect("model ID")
        .get();
    remove_model(&mut sim, id).expect("remove static OBJ");
    assert_eq!(ray(&sim, [3., 3., 5.]), None);
    fs::remove_file(dir.path().join("floor.obj")).expect("delete original file");
    reset_loaded_world(&mut sim, |_| ()).expect("reset captured loaded geometry");
    assert!((ray(&sim, [3., 3., 5.]).expect("restored surface") - 3.5).abs() < 1e-12);
    assert_eq!(pose(&sim, "default::ball::body").translation, [2., 3., 3.]);
    sim.step_exact(1500).expect("settle after reset");
    assert_eq!(
        pose(&sim, "default::ball::body").translation,
        settled.translation
    );
}

#[test]
fn asymmetric_indexed_obj_keeps_native_ray_geometry_and_dynamic_convex_admission() {
    let dir = tempfile::tempdir().expect("fixture directory");
    fs::write(
        dir.path().join("shape.obj"),
        include_bytes!("../../../examples/meshes/m16-asymmetric.obj"),
    )
    .expect("write indexed OBJ");
    let xml = "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><model name='shape'><pose>2 3 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='mesh'><pose>0 0 0.5 0 0 1.5707963267948966</pose><geometry><mesh><uri>shape.obj</uri><scale>2 3 0.5</scale></mesh></geometry></collision></link></model><model name='ground'><static>true</static><pose>0 0 -0.5 0 0 0</pose><link name='body'><collision name='floor'><geometry><box><size>100 100 1</size></box></geometry></collision></link></model></world></sdf>";
    let mut sim = load(dir.path(), xml).expect("outward closed convex tetrahedron");
    assert!((ray(&sim, [0.5, 3.5, 10.]).expect("sloped tetrahedron hit") - 4.75).abs() < 1e-12);
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::shape::body")
        })
        .expect("body");
    let body = entity.get::<BodyHandle>().expect("native body").id();
    let initial = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(body)
        .expect("native initial state");
    sim.step_exact(300).expect("free fall");
    assert!(pose(&sim, "default::shape::body").translation[2] < 3.8);
    reset_loaded_world(&mut sim, |_| ()).expect("reset dynamic mesh");
    let actual = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(body)
        .expect("restored body");
    assert_eq!(actual.position, initial.position);
    assert_eq!(actual.velocity, initial.velocity);
    assert_eq!(actual.orientation, initial.orientation);
    assert!((ray(&sim, [0.5, 3.5, 10.]).expect("restored native shape") - 4.75).abs() < 1e-12);
    // A single open triangle is valid static input, but may not silently become
    // a dynamic hull or repaired solid through the newly admitted file format.
    fs::write(
        dir.path().join("shape.obj"),
        b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .expect("open surface");
    assert!(load(dir.path(), xml).is_err());
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(body)
            .expect("original unaffected")
            .position,
        initial.position
    );
}
