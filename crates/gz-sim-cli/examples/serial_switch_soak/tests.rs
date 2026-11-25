// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{JointSample, set_joint_controller_command, validate_model_demotion};

#[test]
fn each_true_handoff_preserves_the_local_clock_and_loaded_receiver() -> Result<()> {
    let (digest, data) = episode(&mut Rig::new()?, &mut Rig::new()?, 4)?;
    assert_eq!(digest.len(), 64);
    assert_eq!(data["native_tier_changes"], 8);
    Ok(())
}

#[test]
fn inverted_and_off_vertical_pendulums_cannot_park_but_a_nearly_hanging_wrist_can() -> Result<()> {
    // A standalone universal provides a real native joint and free hanging COM, without
    // an unrelated motor or limit explaining the rejection.
    let damper = "<dynamics><damping>0.2</damping></dynamics>";
    let weak =
        "<dynamics><spring_stiffness>0.02</spring_stiffness><damping>0.2</damping></dynamics>";
    let stiff = "<dynamics><spring_stiffness>2</spring_stiffness><damping>0.2</damping></dynamics>";
    let loaded_stop = "<limit><lower>-0.4</lower><upper>0</upper></limit><dynamics><spring_reference>0.04</spring_reference><spring_stiffness>0.02</spring_stiffness><damping>0.2</damping></dynamics>";
    let hard_lock = "<limit><lower>0</lower><upper>0</upper></limit>";
    let one_sided = "<limit><lower>-0.4</lower><upper>0</upper></limit>";
    for (x, z, spring, second, expected) in [
        (1e-7, -0.1, "", "", true),
        (0., 0.1, "", "", false),
        (1e-7, 0.1, "", "", false),
        (0.001, -0.1, "", "", false),
        (
            0.,
            -0.1,
            "<dynamics><spring_reference>0.005</spring_reference><spring_stiffness>2</spring_stiffness></dynamics>",
            "",
            false,
        ),
        (0., 0.1, damper, damper, false),
        (1e-7, 0.1, damper, damper, false),
        (0., 0.1, weak, weak, false),
        (1e-7, 0.1, weak, weak, false),
        (0., 0.1, stiff, weak, false),
        (0., 0.1, stiff, stiff, true),
        (0., -0.1, weak, weak, true),
        (0., -0.1, damper, damper, true),
        (0., 0.1, loaded_stop, loaded_stop, true),
        (0., 0.1, hard_lock, hard_lock, true),
        (0., 0.1, one_sided, one_sided, false),
    ] {
        let xml=format!("<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity>
            <model name='arm'><link name='mass'><inertial><pose>{x} 0 {z} 0 0 0</pose><mass>1</mass>
            <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
            <collision name='body'><geometry><sphere><radius>0.02</radius></sphere></geometry></collision></link>
            <joint name='bearing' type='universal'><parent>world</parent><child>mass</child>
            <axis><xyz>1 0 0</xyz>{spring}</axis><axis2><xyz>0 1 0</xyz>{second}</axis2></joint>
            <plugin filename='gz-sim-switchable-articulation-system' name='gz::sim::systems::SwitchableArticulation'/>
            </model></world></sdf>");
        let mut sim = load_local_replay_world(&xml, "pendulum.sdf", "default", [0; 32])?;
        sim.step_exact(1)?;
        let model = id(&sim, "arm");
        assert_eq!(
            validate_model_demotion(&sim, model).is_ok(),
            expected,
            "{x} {z} {spring} {second}"
        );
        assert_eq!(
            set_model_fidelity(&mut sim, model, Tier::Prescribed).is_ok(),
            expected,
            "{x} {z} {spring} {second}"
        );
        assert_eq!(
            model_fidelity(&sim, model)?,
            if expected {
                Tier::Prescribed
            } else {
                Tier::Dynamic
            }
        );
    }
    Ok(())
}

#[test]
fn motor_disable_is_atomic_and_restart_retains_targets() -> Result<()> {
    let mut rig = Rig::new()?;
    let mut reference = Rig::new()?;
    let ids = rig.drives;
    assert!(set_joint_position_drives_enabled(&mut rig.sim, &[ids[0], u64::MAX], false).is_err());
    let floor = id(&rig.sim, "receiver");
    assert!(set_joint_position_drives_enabled(&mut rig.sim, &[ids[0], floor], false).is_err());
    rig.sim.step_exact(10)?;
    reference.sim.step_exact(10)?;
    assert_eq!(rig.state()?.bodies, reference.state()?.bodies);
    set_joint_position_drives_enabled(&mut rig.sim, &ids, false)?;
    let targets = rig.state()?.targets;
    rig.sim.step_exact(10)?;
    reference.sim.step_exact(10)?;
    assert_ne!(rig.state()?.bodies, reference.state()?.bodies);
    set_joint_position_drives_enabled(&mut rig.sim, &ids, true)?;
    assert_eq!(rig.state()?.targets, targets);
    set_joint_controller_command(&mut rig.sim, ids[0], 1.0)?;
    rig.sim.step_exact(2)?;
    let spindle = gz_sim_core::entity_by_id(rig.sim.world(), ids[0]).unwrap();
    assert!(
        rig.sim
            .world()
            .get::<JointSample>(spindle)
            .unwrap()
            .state
            .velocity
            > 0.
    );
    Ok(())
}

#[test]
fn actual_two_pad_contact_refuses_parking() -> Result<()> {
    let source = XML.replace("<model name=\"gripper\">", MARKER);
    let mut sim = load_local_replay_world(&source, "loaded.sdf", "default", [0; 32])?;
    let model = id(&sim, "gripper");
    let body = |sim: &Simulation, name| {
        sim.world()
            .get::<BodyHandle>(gz_sim_core::entity_by_id(sim.world(), id(sim, name)).unwrap())
            .unwrap()
            .id()
    };
    let parcel = body(&sim, "parcel::box");
    let pads = ["gripper::left", "gripper::right"].map(|n| body(&sim, n));
    let mut contacts = false;
    for _ in 0..1000 {
        sim.step_exact(1)?;
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        if pads.iter().all(|pad| {
            native
                .normal_reaction_on(parcel, *pad)
                .unwrap()
                .is_some_and(|r| r.magnitude > 1e-6)
        }) {
            assert!(validate_model_demotion(&sim, model).is_err());
            assert!(set_model_fidelity(&mut sim, model, Tier::Prescribed).is_err());
            contacts = true;
            break;
        }
    }
    assert!(contacts, "must first establish a physical two-sided grasp");
    assert_eq!(model_fidelity(&sim, model)?, Tier::Dynamic);
    Ok(())
}

#[test]
fn wrist_effort_wakes_an_unloaded_parked_arm_and_motion_refuses_redemotion() -> Result<()> {
    let mut rig = Rig::new()?;
    rig.settle()?;
    set_model_fidelity(&mut rig.sim, rig.model, Tier::Prescribed)?;
    gz_sim_scene::queue_universal_joint_effort(&mut rig.sim, rig.drives[1], [0.3, 0.3])?;
    rig.sim.step_exact(1)?;
    assert_eq!(model_fidelity(&rig.sim, rig.model)?, Tier::Dynamic);
    assert!(
        rig.state()?.bodies[2]
            .angular_velocity
            .into_iter()
            .any(|w| w.abs() > 0.001)
    );
    assert!(validate_model_demotion(&rig.sim, rig.model).is_err());
    Ok(())
}

#[test]
fn scalar_force_pid_is_not_a_disengageable_wrist_drive() -> Result<()> {
    let mut sim = load_local_replay_world(
        include_str!("../../../../examples/worlds/upstream_joint_position.sdf"),
        "pid.sdf",
        "default",
        [0; 32],
    )?;
    let joint = id(&sim, "joint_position_controller_test::j1");
    assert!(set_joint_position_drives_enabled(&mut sim, &[joint], false).is_err());
    Ok(())
}
