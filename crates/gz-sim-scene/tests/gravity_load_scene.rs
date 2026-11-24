// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use gz_sim_core::EntityId;
use gz_sim_scene::{
    InputError, Name, PhysicsEngine, joint_gravity_load, load_world_with_sdf_step,
    locked_subtree_inertia,
};
const XML: &str = include_str!("../../../examples/worlds/gravity_wrist.sdf");

#[test]
fn resolves_owned_public_ids_and_preserves_the_authored_axes_of_an_ordinary_hinge() {
    let sim = load_world_with_sdf_step(
        XML,
        "wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let get = |n: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|v| v.0 == n))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let link = get("default::held::tool");
    let joint = get("default::held::bearing");
    assert!(matches!(joint_gravity_load(&sim,link),Err(InputError::NotJoint(id)) if id==link));
    assert!(matches!(
        joint_gravity_load(&sim, u64::MAX),
        Err(InputError::MissingEntity(u64::MAX))
    ));
    assert!(matches!(
        locked_subtree_inertia(&sim, link),
        Err(InputError::NotJoint(_))
    ));
    assert!(matches!(
        locked_subtree_inertia(&sim, u64::MAX),
        Err(InputError::MissingEntity(_))
    ));
    assert_eq!(locked_subtree_inertia(&sim, joint).unwrap().mass_kg, 3.);
    assert!(gz_sim_scene::subtree_momentum_about(&sim, link, [0.; 3]).is_err());
    assert_eq!(
        gz_sim_scene::subtree_momentum_about(&sim, joint, [0.; 3])
            .unwrap()
            .kinetic_energy_j,
        0.
    );
    let held = joint_gravity_load(&sim, joint).unwrap();
    assert!(held.torque_nm.iter().all(|v| v.abs() > 0.05));
    let xml = r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="local" type="ignored"><max_step_size>0.001</max_step_size></physics><model name="hinge"><link name="load"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link><joint name="pivot" type="revolute"><pose>0.03 0.09 0.02 0.11 -0.23 0.58</pose><parent>world</parent><child>load</child><axis><xyz>1 0 0</xyz></axis></joint></model></world></sdf>"#;
    let ordinary = load_world_with_sdf_step(
        xml,
        "hinge.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let id = ordinary
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|v| v.0 == "default::hinge::pivot")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert_eq!(
        gz_sim_scene::subtree_momentum_about(&ordinary, id, [1., 2., 3.])
            .unwrap()
            .linear_kg_m_s,
        [0.; 3]
    );
    assert!(gz_sim_scene::subtree_momentum_about(&ordinary, id, [f64::NAN; 3]).is_err());
    let load = joint_gravity_load(&ordinary, id).unwrap();
    let axes = glam::DQuat::from_rotation_z(0.58)
        * glam::DQuat::from_rotation_y(-0.23)
        * glam::DQuat::from_rotation_x(0.11);
    let weight = glam::DVec3::new(0., 0., -9.81);
    let lever = glam::DVec3::new(-0.03, -0.09, -0.02);
    let inertia = locked_subtree_inertia(&ordinary, id).unwrap();
    let com = axes.inverse() * lever;
    assert!((glam::DVec3::from(inertia.center_of_mass_m) - com).length() < 1e-12);
    let shift = glam::DMat3::from_cols(com.x * com, com.y * com, com.z * com);
    let expected = glam::DMat3::IDENTITY * (1. + com.length_squared()) - shift;
    assert!(expected.abs_diff_eq(
        glam::DMat3::from_cols_array_2d(&inertia.inertia_kg_m2),
        1e-12
    ));
    assert!((axes * glam::DVec3::from(load.force_n)).distance(weight) < 1e-12);
    assert!((axes * glam::DVec3::from(load.torque_nm)).distance(lever.cross(weight)) < 1e-12);
}

#[test]
fn external_constraints_cannot_silently_turn_a_tree_load_into_a_closed_loop_reading() {
    use gz_physics::PhysicsBackend;
    use gz_sim_scene::BodyHandle;
    for engine in [PhysicsEngine::MultiFidelity, PhysicsEngine::Rapier] {
        let xml = r#"<sdf version="1.12"><world name="default"><model name="arm"><link name="tool"><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link><joint name="joint" type="revolute"><parent>world</parent><child>tool</child><axis><xyz>1 0 0</xyz></axis></joint></model></world></sdf>"#;
        let mut sim =
            load_world_with_sdf_step(xml, "arm.sdf", "default", &sdformat::NoResources, engine)
                .unwrap();
        let part = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|ent| ent.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
        };
        let id = part("default::arm::joint").get::<EntityId>().unwrap().get();
        let body = part("default::arm::tool").get::<BodyHandle>().unwrap().id();
        assert!(joint_gravity_load(&sim, id).is_ok());
        let extra = sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_revolute(body, [0.; 3], [0., 1., 0.])
            .unwrap();
        assert!(gz_sim_scene::subtree_momentum_about(&sim, id, [0.; 3]).is_err());
        assert!(matches!(
            locked_subtree_inertia(&sim, id),
            Err(InputError::Physics(_))
        ));
        assert!(matches!(
            joint_gravity_load(&sim, id),
            Err(InputError::Physics(_))
        ));
        assert!(
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .remove_joint(extra)
        );
        assert!(joint_gravity_load(&sim, id).is_ok());
    }
}
