// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ArticulationTier, FidelityError, JointHandle, LoadError, Name, PhysicsEngine,
    articulation_root_pose, load_world_with_sdf_step, model_fidelity, remove_model,
    reset_loaded_world, set_joint_force, set_model_fidelity,
};
use sdformat::NoResources;

const MARKER: &str = r#"<plugin name="gz::sim::systems::SwitchableArticulation" filename="gz-sim-switchable-articulation-system"/>"#;

fn fixture(marker: &str) -> String {
    let link = |name: &str, x: f64| {
        format!(
            "<link name='{name}'><pose>{x} 0 0 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>"
        )
    };
    let hinge = |name: &str, parent: &str, child: &str| {
        format!(
            "<joint name='{name}' type='revolute'><parent>{parent}</parent><child>{child}</child><axis><xyz>0 0 1</xyz></axis></joint>"
        )
    };
    // The public canonical link is the first declared link; the physical root is last.
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='arm'>{}{}{}{}{}{marker}</model></world></sdf>",
        link("tip", 3.0),
        link("middle", 2.0),
        link("base", 1.0),
        hinge("end_hinge", "middle", "tip"),
        hinge("base_hinge", "base", "middle")
    )
}

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(xml, "tree.sdf", "default", &NoResources, engine)
}

fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn marker_admits_only_complete_floating_trees_on_the_selected_engine() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(matches!(
            load(&fixture(MARKER), engine),
            Err(LoadError::Unsupported { .. })
        ));
    }
    for marker in [
        format!("{MARKER}{MARKER}"),
        MARKER.replace("/>", " extra='true'/>"),
        MARKER.replace("/>", "><unknown/></plugin>"),
    ] {
        assert!(matches!(
            load(&fixture(&marker), PhysicsEngine::MultiFidelity),
            Err(LoadError::Unsupported {
                feature: "SwitchableArticulation configuration",
                ..
            })
        ));
    }
    let xml = fixture(MARKER);
    for invalid in [
        xml.replace("<parent>base</parent>", "<parent>world</parent>"),
        xml.replace(MARKER, &format!("<link name='detached'/>{MARKER}")),
    ] {
        assert!(matches!(
            load(&invalid, PhysicsEngine::MultiFidelity),
            Err(LoadError::Unsupported {
                feature: "SwitchableArticulation requires one floating connected revolute tree",
                ..
            })
        ));
    }
}

#[test]
fn reverse_joint_order_preserves_public_handles_through_switch_removal_and_both_resets() {
    let mut sim = load(&fixture(MARKER), PhysicsEngine::MultiFidelity).unwrap();
    let model = id(&sim, "default::arm");
    let hinge = id(&sim, "default::arm::base_hinge");
    let tip_hinge = id(&sim, "default::arm::end_hinge");
    assert!(tip_hinge < hinge);
    assert_eq!(
        articulation_root_pose(&sim, model).unwrap().translation,
        [1.0, 0.0, 0.0]
    );
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Dynamic
    );
    let handles = |sim: &Simulation| {
        [hinge, tip_hinge].map(|id| {
            let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
            sim.world().get::<JointHandle>(entity).unwrap().id()
        })
    };
    let original = handles(&sim);
    set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed).unwrap();
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Prescribed
    );
    assert_eq!(handles(&sim), original);
    assert!(matches!(
        model_fidelity(&sim, hinge),
        Err(FidelityError::NotSwitchable(_))
    ));
    sim.reset().unwrap();
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Dynamic
    );
    set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed).unwrap();
    remove_model(&mut sim, model).unwrap();
    assert!(matches!(
        model_fidelity(&sim, model),
        Err(FidelityError::MissingModel(_))
    ));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Dynamic
    );
    assert_eq!(handles(&sim), original);
}

#[test]
fn persistent_effort_blocks_demotion_until_the_command_is_released() {
    let mut sim = load(&fixture(MARKER), PhysicsEngine::MultiFidelity).unwrap();
    let model = id(&sim, "default::arm");
    let hinge = id(&sim, "default::arm::base_hinge");
    set_joint_force(&mut sim, hinge, 0.25).unwrap();
    assert!(matches!(
        set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed),
        Err(FidelityError::PendingCommand)
    ));
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Dynamic
    );
    set_joint_force(&mut sim, hinge, 0.0).unwrap();
    set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed).unwrap();
    assert_eq!(
        model_fidelity(&sim, model).unwrap(),
        ArticulationTier::Prescribed
    );
}

#[test]
fn replacing_the_solver_cannot_redirect_a_model_transition_into_a_foreign_tree() {
    let xml = fixture(MARKER);
    let mut sim = load(&xml, PhysicsEngine::MultiFidelity).unwrap();
    let mut other = load(&xml, PhysicsEngine::MultiFidelity).unwrap();
    let model = id(&sim, "default::arm");
    let foreign = other
        .world_mut()
        .remove_resource::<gz_physics::PhysicsBackend>()
        .unwrap();
    sim.world_mut().insert_resource(foreign);
    assert!(matches!(
        set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed),
        Err(FidelityError::InvalidBinding)
    ));
    assert!(matches!(
        articulation_root_pose(&sim, model),
        Err(FidelityError::InvalidBinding)
    ));
}

#[test]
fn a_raw_joint_joining_two_models_does_not_expand_the_public_switch_target() {
    let xml = fixture(MARKER);
    let first = &xml[xml.find("<model").unwrap()..xml.find("</world>").unwrap()];
    let second = first.replace("name='arm'", "name='peer'");
    let mut sim = load(
        &xml.replace("</world>", &format!("{second}</world>")),
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let model = id(&sim, "default::arm");
    let tip = id(&sim, "default::arm::tip");
    let endpoints = ["default::arm::tip", "default::peer::base"].map(|name| {
        let entity = gz_sim_core::entity_by_id(sim.world(), id(&sim, name)).unwrap();
        sim.world()
            .get::<gz_sim_scene::BodyHandle>(entity)
            .unwrap()
            .id()
    });
    sim.world_mut()
        .resource_mut::<gz_physics::PhysicsBackend>()
        .add_reduced_revolute(
            Some(endpoints[0]),
            endpoints[1],
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )
        .unwrap();
    assert!(matches!(
        set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed),
        Err(FidelityError::InvalidBinding)
    ));
    assert!(matches!(
        gz_sim_scene::queue_link_world_wrench(&mut sim, tip, [1.0; 3], [0.0; 3]),
        Err(gz_sim_scene::InputError::InvalidPhysics)
    ));
}
