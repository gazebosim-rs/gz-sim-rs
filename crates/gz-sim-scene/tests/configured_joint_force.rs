// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    JointForceCommand, JointForceConfiguration, JointSample, Name, PhysicsEngine,
    load_world_with_sdf_step, remove_model, reset_loaded_world, set_joint_force,
};
use sdformat::NoResources;
const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rotor'><link name='body'><inertial><mass>1</mass><inertia><ixx>0.016</ixx><iyy>0.016</iyy><izz>0.016</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 5 0</xyz></axis></joint></model></world></sdf>";
const PLUGIN: &str = "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin>";
fn xml(plugin: &str) -> String {
    XML.replace("</model>", &format!("{plugin}</model>"))
}
fn load(plugin: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(
        &xml(plugin),
        "configured-force.sdf",
        "default",
        &NoResources,
        engine,
    )
    .unwrap()
}
fn joint(sim: &Simulation) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rotor::hinge")
        })
        .unwrap()
        .id()
}
#[test]
fn configured_controller_starts_with_zero_and_applies_programmatic_latest_force() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(PLUGIN, engine);
        let entity = joint(&sim);
        let config = sim.world().get::<JointForceConfiguration>(entity).unwrap();
        assert_eq!(config.model_name(), "rotor");
        assert_eq!(config.joint_name(), "hinge");
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(entity)
                .unwrap()
                .torque(),
            0.0
        );
        sim.step_exact(10).unwrap();
        assert_eq!(
            sim.world()
                .get::<JointSample>(entity)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        set_joint_force(&mut sim, id, 1.5).unwrap();
        sim.step_exact(2).unwrap();
        assert!(
            (sim.world()
                .get::<JointSample>(entity)
                .unwrap()
                .state
                .velocity
                - 0.1875)
                .abs()
                < 1e-12
        );
    }
}
#[test]
fn loaded_reset_recreates_configuration_and_zero_command_after_retirement() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(PLUGIN, engine);
        let entity = joint(&sim);
        let config = sim
            .world()
            .get::<JointForceConfiguration>(entity)
            .unwrap()
            .clone();
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        set_joint_force(&mut sim, id, 1.5).unwrap();
        sim.step_exact(1).unwrap();
        sim.reset().unwrap();
        assert_eq!(
            sim.world().get::<JointForceConfiguration>(entity),
            Some(&config)
        );
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(entity)
                .unwrap()
                .torque(),
            0.0
        );
        let model = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rotor"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        remove_model(&mut sim, model).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let entity = joint(&sim);
        assert_eq!(
            sim.world().get::<JointForceConfiguration>(entity),
            Some(&config)
        );
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(entity)
                .unwrap()
                .torque(),
            0.0
        );
        sim.step_exact(2).unwrap();
        assert_eq!(
            sim.world()
                .get::<JointSample>(entity)
                .unwrap()
                .state
                .velocity,
            0.0
        );
    }
}
#[test]
fn configuration_rejects_ambiguous_or_unimplemented_payloads() {
    for plugin in [
        PLUGIN.replace("ApplyJointForce'", "Other'"),
        PLUGIN.replace("gz-sim-apply-joint-force-system", "other"),
        PLUGIN.replace("<joint_name>hinge</joint_name>", ""),
        PLUGIN.replace("hinge</joint_name>", "</joint_name>"),
        PLUGIN.replace("hinge</joint_name>", "missing</joint_name>"),
        PLUGIN.replace("hinge</joint_name>", "rotor::hinge</joint_name>"),
        PLUGIN.replace("<joint_name>", "<joint_name extra='x'>"),
        PLUGIN.replace("</joint_name>", "</joint_name><unknown/>"),
        PLUGIN.replace(
            "</joint_name>",
            "</joint_name><joint_name>hinge</joint_name>",
        ),
        PLUGIN.replace("<joint_name>hinge</joint_name>", "text"),
        PLUGIN.replace("hinge</joint_name>", "<nested/></joint_name>"),
        PLUGIN.replace("<plugin ", "<plugin extra='x' "),
        PLUGIN.replace("<plugin ", "<plugin xmlns:extra='urn:extra' "),
        format!("{PLUGIN}{PLUGIN}"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml(&plugin),
                "rejected.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{plugin}"
        );
    }
}
#[test]
fn native_single_text_and_cdata_forms_preserve_the_direct_joint_name() {
    for value in [
        "hinge",
        " hinge ",
        "<![CDATA[hinge]]>",
        "<!--leading-->hinge",
        "hinge<!--trailing-->",
        "hi&#110;ge",
    ] {
        let plugin = PLUGIN.replace(
            "<joint_name>hinge</joint_name>",
            &format!("<joint_name>{value}</joint_name>"),
        );
        let sim = load(&plugin, PhysicsEngine::Rapier);
        assert_eq!(
            sim.world()
                .get::<JointForceConfiguration>(joint(&sim))
                .unwrap()
                .joint_name(),
            "hinge"
        );
    }
}
#[test]
fn split_text_and_unicode_whitespace_cannot_silently_select_another_joint() {
    for value in [
        "hi<![CDATA[ng]]>e",
        "hi<!--split-->nge",
        "<![CDATA[hi]]><![CDATA[nge]]>",
        "\u{a0}hinge\u{a0}",
        "<?instruction value?>hinge",
    ] {
        let plugin = PLUGIN.replace(
            "<joint_name>hinge</joint_name>",
            &format!("<joint_name>{value}</joint_name>"),
        );
        assert!(
            load_world_with_sdf_step(
                &xml(&plugin),
                "text-boundary.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{value}"
        );
    }
}

#[test]
fn ordinary_and_cdata_whitespace_select_their_exact_distinct_joint_names() {
    for (declared, value) in [
        ("hi nge", " hi   nge "),
        ("hi   nge", "<![CDATA[ hi   nge ]]>"),
    ] {
        let source = XML.replace("name='hinge'", &format!("name='{declared}'"));
        let plugin = PLUGIN.replace(
            "<joint_name>hinge</joint_name>",
            &format!("<joint_name>{value}</joint_name>"),
        );
        let source = source.replace("</model>", &format!("{plugin}</model>"));
        let sim = load_world_with_sdf_step(
            &source,
            "whitespace.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<JointForceConfiguration>())
            .unwrap();
        assert_eq!(
            entity
                .get::<JointForceConfiguration>()
                .unwrap()
                .joint_name(),
            declared
        );
        assert_eq!(entity.get::<JointForceCommand>().unwrap().torque(), 0.0);
    }
}

#[test]
fn distinct_direct_joints_keep_independent_configured_commands() {
    let link = XML
        .split_once("<link name='body'>")
        .unwrap()
        .1
        .split_once("</link>")
        .unwrap()
        .0;
    let extra = format!(
        "<link name='second_body'><pose>2 0 0 0 0 0</pose>{link}</link><joint name='second_hinge' type='revolute'><parent>world</parent><child>second_body</child><axis><xyz>0 5 0</xyz></axis></joint>"
    );
    let second = PLUGIN.replace("hinge</joint_name>", "second_hinge</joint_name>");
    let source = XML.replace("</model>", &format!("{extra}{PLUGIN}{second}</model>"));
    let mut sim = load_world_with_sdf_step(
        &source,
        "two-controllers.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let first = joint(&sim);
    let first_id = sim.world().get::<EntityId>(first).unwrap().get();
    let second = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rotor::second_hinge")
        })
        .unwrap()
        .id();
    let second_id = sim.world().get::<EntityId>(second).unwrap().get();
    set_joint_force(&mut sim, first_id, 1.5).unwrap();
    set_joint_force(&mut sim, second_id, -0.5).unwrap();
    sim.step_exact(2).unwrap();
    assert!(
        (sim.world()
            .get::<JointSample>(first)
            .unwrap()
            .state
            .velocity
            - 0.1875)
            .abs()
            < 1e-12
    );
    assert!(
        (sim.world()
            .get::<JointSample>(second)
            .unwrap()
            .state
            .velocity
            + 0.0625)
            .abs()
            < 1e-12
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<JointForceConfiguration>())
            .count(),
        2
    );
    sim.step_exact(1).unwrap();
    for e in sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<JointForceConfiguration>())
    {
        assert_eq!(e.get::<JointForceCommand>().unwrap().torque(), 0.0);
        assert_eq!(e.get::<JointSample>().unwrap().state.velocity, 0.0);
    }
}
