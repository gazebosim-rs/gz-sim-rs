// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier::{Dynamic, Prescribed},
    FidelityError, JointHandle, LoadError, Name, PhysicsEngine, load_world_with_sdf_step,
    model_fidelity, reset_loaded_world, set_joint_force, set_model_fidelity,
};
use sdformat::NoResources;

const ARM: &str = include_str!("../../../examples/local_manipulation_arm.sdf");

fn load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "arm.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
#[allow(deprecated)]
fn id(s: &Simulation, name: &str) -> u64 {
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn mixed_robot_marker_requires_one_connected_world_mount() {
    let lo = ARM.find("<plugin").unwrap();
    let hi = ARM[lo..].find("</plugin>").unwrap() + lo + "</plugin>".len();
    let xml = ARM.replace(&ARM[lo..hi], "");
    let unmarked = load(&xml).unwrap();
    assert!(matches!(
        model_fidelity(&unmarked, id(&unmarked, "default::arm")),
        Err(FidelityError::NotSwitchable(_))
    ));
    // A second world parent creates two independently attached subtrees. A
    // disconnected floating arm likewise has no stationary mount to park.
    for invalid in [
        ARM.replace("<parent>rotor</parent>", "<parent>world</parent>"),
        ARM.replace(
            &ARM[ARM.find("<joint name=\"shoulder\"").unwrap()
                ..ARM.find("</joint>").unwrap() + "</joint>".len()],
            "",
        ),
        ARM.replacen(
            "</plugin>",
            "<exact_revolute_locks>true</exact_revolute_locks></plugin>",
            1,
        ),
    ] {
        assert!(matches!(load(&invalid), Err(LoadError::Unsupported { .. })));
    }
}

#[test]
fn public_mounted_command_resumes_on_first_advancing_tick_and_reset_restores_native_ids() {
    let mut sim = load(ARM).unwrap();
    let arm = id(&sim, "default::arm");
    let lift = id(&sim, "default::arm::lift");
    let handle = |s: &Simulation| {
        let entity = entity_by_id(s.world(), lift).unwrap();
        s.world().get::<JointHandle>(entity).unwrap().id()
    };
    let joint = handle(&sim);
    let state = |s: &Simulation| {
        s.world()
            .resource::<PhysicsBackend>()
            .joint_state(joint)
            .unwrap()
    };
    let start = state(&sim);
    sim.step_exact(650).unwrap();
    let instant = sim.committed_update_info().unwrap().sim_time;
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    let held = state(&sim);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    set_joint_force(&mut sim, lift, 22.).unwrap();
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(sim.committed_update_info().unwrap().sim_time, instant);
    assert_eq!(state(&sim), held);
    assert_eq!(handle(&sim), joint);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    assert!(state(&sim).velocity > held.velocity + 0.001);
    assert!(matches!(
        set_model_fidelity(&mut sim, arm, Prescribed),
        Err(FidelityError::PendingCommand)
    ));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    assert_eq!(handle(&sim), joint);
    assert_eq!(state(&sim), start);
}
