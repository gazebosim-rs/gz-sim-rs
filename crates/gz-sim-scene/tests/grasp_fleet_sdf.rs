// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, distributed_hinge::distributed_grasp_fleet_specs,
    load_world_with_sdf_step,
};
use std::collections::BTreeSet;
const FLEET: &str = include_str!("../../../examples/distributed_grasp_fleet.sdf");

#[test]
fn authored_pairing_uses_global_names_and_agrees_with_ordinary_contacts() {
    let specs = distributed_grasp_fleet_specs(FLEET, "fleet.sdf", "default").unwrap();
    assert_eq!(specs.len(), 2);
    let mut sim = load_world_with_sdf_step(
        FLEET,
        "fleet.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut ids = BTreeSet::new();
    for (spec, pair) in specs
        .iter()
        .zip([("south", "south_part"), ("north", "north_part")])
    {
        let p = &spec.payload;
        let name = |id| {
            sim.world()
                .get::<Name>(entity_by_id(sim.world(), id).unwrap())
                .unwrap()
                .0
                .as_str()
        };
        assert_eq!(name(p.tree.model_id), format!("default::{}", pair.0));
        assert_eq!(name(p.payload.id), format!("default::{}::part", pair.1));
        assert!(ids.insert(p.tree.model_id));
        assert!(ids.insert(p.payload.id));
        assert!(p.tree.links.iter().all(|l| ids.insert(l.id)));
        assert!(p.tree.joints.iter().all(|j| ids.insert(j.joint_id)));
        assert!(!p.payload_can_sleep);
    }
    let native = |id| {
        sim.world()
            .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
            .unwrap()
            .id()
    };
    let bodies = specs
        .iter()
        .map(|s| {
            (
                native(s.payload.tree.links[0].id),
                native(s.payload.payload.id),
            )
        })
        .collect::<Vec<_>>();
    sim.step_exact(1).unwrap();
    let be = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    for &(link, _) in &bodies {
        for &(_, part) in &bodies {
            assert!(!be.payload_tree_reaction(link, part).unwrap().active);
            assert!(be.payload_tree_reaction(part, link).is_err());
        }
    }
}
#[test]
fn ambiguous_or_unrepresented_fleet_models_are_rejected() {
    let row = r#"<assembly gripper="north" payload="north_part"/>"#;
    for edited in [
        FLEET.replace(row, ""),
        FLEET.replace(row, r#"<assembly gripper="south" payload="north_part"/>"#),
        FLEET.replace(row, r#"<assembly gripper="north" payload="not_here"/>"#),
        FLEET.replace(
            row,
            r#"<assembly gripper="north" payload="north_part"><mask>none</mask></assembly>"#,
        ),
        FLEET.replace(row, r#"<assembly gripper="north_part" payload="north"/>"#),
        FLEET.replace(
            "</world>",
            r#"<model name="spare"><link name="extra"/></model></world>"#,
        ),
    ] {
        assert!(distributed_grasp_fleet_specs(&edited, "ambiguous.sdf", "default").is_err());
    }
    let swapped = FLEET
        .replace("payload=\"north_part\"", "payload=\"PLACEHOLDER\"")
        .replace("payload=\"south_part\"", "payload=\"north_part\"")
        .replace("PLACEHOLDER", "south_part");
    let a = distributed_grasp_fleet_specs(FLEET, "fleet.sdf", "default").unwrap();
    let b = distributed_grasp_fleet_specs(&swapped, "swapped.sdf", "default").unwrap();
    assert_eq!(a[0].payload.tree, b[0].payload.tree);
    assert_eq!(a[0].payload.payload, b[1].payload.payload);
}
