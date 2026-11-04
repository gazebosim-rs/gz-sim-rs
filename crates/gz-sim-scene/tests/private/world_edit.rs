// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;

fn scene() -> (Simulation, [Entity; 2], [u64; 2]) {
    let model = |name| {
        format!(
            "<model name='{name}'><link name='body'><inertial><pose>0.0000000000001 0 0 0 0 0</pose><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='v'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>"
        )
    };
    let sim = load_world_with_sdf_step(
        &format!(
            "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{}{}</world></sdf>",
            model("a"),
            model("b")
        ),
        "private-edit.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let find = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .id()
    };
    let links = [find("w::a::body"), find("w::b::body")];
    let ids = [find("w::a"), find("w::b")]
        .map(|entity| sim.world().get::<EntityId>(entity).unwrap().get());
    (sim, links, ids)
}

#[test]
fn swapped_native_handles_and_missing_public_link_ids_are_refused_before_commit() {
    let (mut sim, links, ids) = scene();
    let handles = links.map(|entity| sim.world().get::<BodyHandle>(entity).unwrap().id());
    let before = handles.map(|body| {
        sim.world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()
    });
    let requested = model_state(&sim, ids[0]).unwrap();
    let first = sim
        .world_mut()
        .entity_mut(links[0])
        .take::<BodyHandle>()
        .unwrap();
    let second = sim
        .world_mut()
        .entity_mut(links[1])
        .take::<BodyHandle>()
        .unwrap();
    sim.world_mut().entity_mut(links[0]).insert(second);
    sim.world_mut().entity_mut(links[1]).insert(first);
    for id in ids {
        assert!(model_state(&sim, id).is_err());
        assert!(set_model_state(&mut sim, id, requested).is_err());
    }
    assert_eq!(
        handles.map(|body| sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()),
        before
    );
    sim.world_mut()
        .entity_mut(links[0])
        .insert(BodyHandle(handles[0]));
    sim.world_mut()
        .entity_mut(links[1])
        .insert(BodyHandle(handles[1]));
    sim.world_mut().entity_mut(links[0]).remove::<EntityId>();
    assert!(model_state(&sim, ids[0]).is_err());
    assert!(set_model_state(&mut sim, ids[0], requested).is_err());
    assert_eq!(
        handles.map(|body| sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()),
        before
    );
}

#[test]
fn admitted_offset_com_uses_the_new_orientation_and_overflowing_read_is_refused() {
    let (mut sim, links, ids) = scene();
    let requested = ModelState {
        translation: [4.0, 5.0, 6.0],
        rotation: [0.0, 0.0, 1.0, 0.0],
        linear_velocity: [1.0, 2.0, 3.0],
        angular_velocity: [0.0, 0.0, 2.0],
    };
    let actual = set_model_state(&mut sim, ids[0], requested).unwrap();
    assert_eq!(actual, requested);
    let body = sim.world().get::<BodyHandle>(links[0]).unwrap().id();
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(body)
        .unwrap();
    assert_eq!(native.velocity, [1.0, 2.0 - 2e-13, 3.0]);
    assert_eq!(model_state(&sim, ids[0]).unwrap(), requested);
    sim.world_mut()
        .entity_mut(links[0])
        .insert(LinkCenterOfMass([f64::MAX, 0.0, 0.0]));
    assert!(matches!(
        model_state(&sim, ids[0]),
        Err(WorldEditError::InvalidState)
    ));
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap(),
        native
    );
    sim.world_mut()
        .entity_mut(links[0])
        .insert(LinkCenterOfMass([f64::NAN, 0.0, 0.0]));
    assert!(matches!(
        model_state(&sim, ids[0]),
        Err(WorldEditError::InvalidState)
    ));
}
