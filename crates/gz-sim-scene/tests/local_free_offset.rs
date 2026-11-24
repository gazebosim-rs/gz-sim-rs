// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Free local primitive link origin, offset COM and public rigid inputs.
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, ModelState, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    model_state, prepare_model_template, queue_link_world_velocity, queue_link_world_wrench,
    reset_loaded_world, set_model_state, spawn_model_template,
};
use sdformat::NoResources;
const XML: &str = r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity><physics name="s" type="ode"><max_step_size>0.0005</max_step_size></physics>
<model name="foot"><pose>2 3 4 0 0 0.5</pose>
<link name="sole"><inertial><pose>0.02 -0.03 0.07 0 0 0</pose><mass>2</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial>
<collision name="c"><geometry><box><size>0.36 0.2 0.12</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.36 0.2 0.12</size></box></geometry><material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
</link></model></world></sdf>"#;
const OFFSET: DVec3 = DVec3::new(0.02, -0.03, 0.07);
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "com.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(s: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn read(s: &Simulation, link: u64) -> BodyState {
    let e = entity_by_id(s.world(), link).unwrap();
    s.world()
        .resource::<PhysicsBackend>()
        .state(s.world().get::<BodyHandle>(e).unwrap().id())
        .unwrap()
}
fn com(b: BodyState) -> DVec3 {
    DVec3::from(b.position) + DQuat::from_array(b.orientation) * OFFSET
}
fn near(a: DVec3, b: DVec3, tolerance: f64) {
    assert!((a - b).length() < tolerance, "{a:?} vs {b:?}");
}
#[test]
fn launching_an_offset_box_publishes_origin_velocity_and_integrates_com() {
    let xml=XML.replace("<link name=\"sole\">",r#"<plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>0.2 0.1 -0.3</linear_velocity><angular_velocity>0 2 0</angular_velocity></plugin><link name="sole">"#);
    let mut s = load(&xml);
    let link = id(&s, "default::foot::sole");
    let e = entity_by_id(s.world(), link).unwrap();
    let initial = read(&s, link);
    let v = DVec3::new(0.2, 0.1, -0.3);
    near(initial.velocity.into(), v, 1e-14);
    let arm = DQuat::from_array(initial.orientation) * OFFSET;
    near(
        s.world().get::<Velocity>(e).unwrap().linear.into(),
        v - DVec3::Y.cross(arm) * 2.,
        1e-13,
    );
    s.step_exact(80).unwrap();
    near(com(read(&s, link)), com(initial) + v * 0.04, 1e-10);
    let old = read(&s, link);
    let linear = DVec3::new(-0.7, 0.2, 0.3);
    let omega = -DVec3::Y;
    queue_link_world_velocity(&mut s, link, linear.into(), omega.into()).unwrap();
    queue_link_world_wrench(&mut s, link, [4., 0., 0.], [0.; 3]).unwrap();
    s.step_exact(1).unwrap();
    near(
        read(&s, link).velocity.into(),
        linear + omega.cross(DQuat::from_array(old.orientation) * OFFSET) + DVec3::X * 0.001,
        1e-10,
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert_eq!(read(&s, link), initial);
}
#[test]
fn clone_and_edit_roundtrip_the_link_origin_of_an_offset_primitive() {
    let mut s = load(XML);
    let parsed = sdformat::load_str(XML, "com.sdf", &NoResources).unwrap();
    let t = prepare_model_template(&s, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let clone = spawn_model_template(
        &mut s,
        &t,
        WorldPose {
            translation: [9., 1., 2.],
            rotation: DQuat::from_rotation_x(0.3).to_array(),
        },
    )
    .unwrap();
    let state = ModelState {
        translation: [5., 2., 1.],
        rotation: DQuat::from_rotation_z(-0.4).to_array(),
        linear_velocity: [0.1, 0.2, 0.3],
        angular_velocity: [-1., 2., -0.5],
    };
    set_model_state(&mut s, clone, state).unwrap();
    let public = model_state(&s, clone).unwrap();
    near(
        public.linear_velocity.into(),
        state.linear_velocity.into(),
        1e-12,
    );
    let link = id(&s, &format!("default::runtime_{clone}::body"));
    let b = read(&s, link);
    near(
        b.velocity.into(),
        DVec3::from(state.linear_velocity)
            + DVec3::from(state.angular_velocity).cross(DQuat::from_array(state.rotation) * OFFSET),
        1e-12,
    );
    let origin_com = com(b);
    s.step_exact(40).unwrap();
    near(
        com(read(&s, link)),
        origin_com + DVec3::from(b.velocity) * 0.02,
        1e-11,
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(entity_by_id(s.world(), clone).is_none());
}
