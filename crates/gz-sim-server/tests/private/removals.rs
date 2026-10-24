// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::assign_id;

fn model(sim: &mut Simulation, name: &str, parent: Option<u64>) -> u64 {
    let parent = parent.map(|id| entity_by_id(sim.world(), id).unwrap());
    let mut entity = sim
        .world_mut()
        .spawn((EntityKind::Model, LocalName(name.into())));
    if let Some(parent) = parent {
        entity.insert(ChildOf(parent));
    }
    let entity = entity.id();
    assign_id(sim.world_mut(), entity).unwrap()
}

#[test]
fn name_resolution_checks_hierarchy_ambiguity_type_and_identity_ownership() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let world = sim
        .world_mut()
        .spawn((EntityKind::World, LocalName("default".into())))
        .id();
    let world = assign_id(sim.world_mut(), world).unwrap();
    let ball = model(&mut sim, "ball", Some(world));
    let mut request = Entity {
        name: "ball".into(),
        r#type: Type::Model as i32,
        ..Default::default()
    };
    assert_eq!(target(&sim, &request), Some(ball));
    request.name = "default::ball".into();
    assert_eq!(target(&sim, &request), Some(ball));
    let outer = model(&mut sim, "outer", Some(world));
    let nested = model(&mut sim, "ball", Some(outer));
    assert_eq!(target(&sim, &request), Some(ball));
    request.name = "ball".into();
    assert_eq!(target(&sim, &request), None);
    request.name = "outer::ball".into();
    assert_eq!(target(&sim, &request), Some(nested));
    for name in ["all", "::ball", "outer::", "", "missing::default::ball"] {
        request.name = name.into();
        assert_eq!(target(&sim, &request), None);
    }
    request.name = "default::ball".into();
    request.r#type = 999;
    assert_eq!(target(&sim, &request), None);
    request.id = ball;
    assert_eq!(target(&sim, &request), Some(ball));
    let mut other = Simulation::new(Duration::from_millis(1)).unwrap();
    let foreign = model(&mut other, "ball", None);
    let foreign_entity = entity_by_id(other.world(), foreign).unwrap();
    let identity = other
        .world_mut()
        .entity_mut(foreign_entity)
        .take::<EntityId>()
        .unwrap();
    sim.world_mut()
        .spawn((EntityKind::Model, LocalName("rogue".into()), identity));
    request.id = 0;
    request.r#type = Type::Model as i32;
    request.name = "rogue".into();
    assert_eq!(target(&sim, &request), None);
    sim.world_mut()
        .spawn((EntityKind::Model, LocalName("unassigned".into())));
    request.name = "unassigned".into();
    assert_eq!(target(&sim, &request), None);
    sim.world_mut().spawn(EntityKind::Model);
    request.name = "missing".into();
    assert_eq!(target(&sim, &request), None);
    assert!(entity_by_id(sim.world(), ball).is_some());
}
