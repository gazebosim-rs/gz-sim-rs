// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::PhysicsBackend;
use gz_physics_rapier::{BodySpec, RapierBackend, Shape};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, InputError, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_link_world_velocity,
};
use sdformat::NoResources;
fn scene() -> (gz_sim_core::Simulation, u64) {
    let sim=load_world_with_sdf_step("<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='robot'><link name='body'><collision name='shape'><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model></world></sdf>","owner","default",&NoResources,PhysicsEngine::Rapier).unwrap();
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::robot::body")
        })
        .unwrap();
    let id = link.get::<EntityId>().unwrap().get();
    (sim, id)
}
fn replace(sim: &mut gz_sim_core::Simulation) {
    let old = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
        .unwrap();
    let mut foreign = RapierBackend::new([0.; 3]).unwrap();
    let new = foreign
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 1. },
            position: [100., 0., 0.],
            velocity: [0.; 3],
            mass: Some(1.),
        })
        .unwrap();
    assert_eq!(old, new, "backend-local IDs intentionally collide");
    sim.world_mut()
        .insert_resource(PhysicsBackend::from(foreign));
}
#[test]
fn foreign_world_must_not_accept_a_stale_link_body_id_at_admission() {
    let (mut sim, id) = scene();
    replace(&mut sim);
    assert!(matches!(
        gz_sim_scene::queue_link_world_wrench(&mut sim, id, [1.0; 3], [0.0; 3]),
        Err(InputError::InvalidPhysics)
    ));
    let result = queue_link_world_velocity(&mut sim, id, [1., 0., 0.], [0.; 3]);
    assert!(
        matches!(result, Err(InputError::InvalidPhysics)),
        "foreign backend admitted: {result:?}"
    );
}
#[test]
fn foreign_world_after_admission_must_fault_before_executing_stale_binding() {
    let (mut sim, id) = scene();
    queue_link_world_velocity(&mut sim, id, [1., 0., 0.], [0.; 3]).unwrap();
    replace(&mut sim);
    let result = sim.step_exact(1);
    assert!(
        result.is_err(),
        "foreign world was stepped successfully: {result:?}"
    );
    assert_eq!(result, Err(gz_sim_core::StepError::Faulted));
    assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
    let body = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
        .unwrap();
    let state = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(body)
        .unwrap();
    assert_eq!(state.position, [100.0, 0.0, 0.0]);
    assert_eq!(state.velocity, [0.0; 3]);
}
