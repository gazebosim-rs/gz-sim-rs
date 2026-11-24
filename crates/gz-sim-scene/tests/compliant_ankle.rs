// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{PhysicsEngine, damped_sole::ankle_step, load_world_with_sdf_step};
const SDF: &str = include_str!("../../../examples/worlds/local_compliant_ankle.sdf");
#[test]
fn plantar_load_passes_through_the_free_and_locked_native_ankle() {
    for xml in [
        SDF.to_owned(),
        SDF.replace("type=\"revolute\"", "type=\"fixed\"")
            .replace("<axis><xyz>0 1 0</xyz></axis>", ""),
    ] {
        let mut sim = load_world_with_sdf_step(
            &xml,
            "ankle.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        for i in 0..30 {
            if let Err(e) = sim.step_exact(1) {
                panic!(
                    "tick {i} {e}: {:?}",
                    sim.world().resource::<gz_sim_core::StepFault>().error()
                )
            }
            let row = ankle_step(&sim).unwrap();
            assert!(row.panel.step.normal_impulse > 0.);
            assert_eq!(
                gz_sim_scene::damped_sole::panel_steps(&sim)[0].step.impulse,
                row.panel.step.impulse
            );
            assert!(row.panel.step.impulse.x < 0.);
        }
    }
}

#[allow(deprecated)]
fn entity<'a>(sim: &'a gz_sim_core::Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<gz_sim_scene::Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}
fn load(xml: &str) -> Result<gz_sim_core::Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "ankle.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn keys(sim: &gz_sim_core::Simulation) -> [u64; 2] {
    ["default::boot::upper", "default::boot::ankle"]
        .map(|n| entity(sim, n).get::<gz_sim_core::EntityId>().unwrap().get())
}
fn states(sim: &gz_sim_core::Simulation) -> [gz_physics::BodyState; 2] {
    let b = sim.world().resource::<gz_physics::PhysicsBackend>();
    ["default::boot::sole", "default::boot::upper"].map(|n| {
        b.state(
            entity(sim, n)
                .get::<gz_sim_scene::BodyHandle>()
                .unwrap()
                .id(),
        )
        .unwrap()
    })
}
fn drive(sim: &mut gz_sim_core::Simulation) {
    let [shank, joint] = keys(sim);
    gz_sim_scene::queue_joint_effort(sim, joint, 0.24).unwrap();
    gz_sim_scene::queue_link_world_wrench(sim, shank, [1., 0., 0.], [0., 0.37, 0.]).unwrap();
}
#[test]
fn a_public_joint_effort_and_external_shank_wrench_span_every_internal_interval() {
    let mut whole = load(SDF).unwrap();
    let mut split = load(
        &SDF.replace("0.0005</max_step_size>", "0.0000625</max_step_size>")
            .replace("<substeps>8", "<substeps>1"),
    )
    .unwrap();
    for _ in 0..30 {
        drive(&mut whole);
        whole.step_exact(1).unwrap();
        for _ in 0..8 {
            drive(&mut split);
            split.step_exact(1).unwrap();
        }
    }
    for (a, b) in states(&whole).into_iter().zip(states(&split)) {
        assert!(glam::DVec3::from(a.velocity).distance(b.velocity.into()) < 1e-11);
        assert!(glam::DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()) < 1e-11);
    }
    whole.set_paused(true);
    drive(&mut whole);
    let prior = states(&whole);
    whole.tick().unwrap();
    assert_eq!(prior, states(&whole));
    gz_sim_scene::reset_loaded_world(&mut whole, |_| ()).unwrap();
    assert!(ankle_step(&whole).is_none());
    let mut new = load(SDF).unwrap();
    for _ in 0..80 {
        whole.step_exact(1).unwrap();
        new.step_exact(1).unwrap();
    }
    assert_eq!(states(&whole), states(&new));
    assert_eq!(
        ankle_step(&whole).unwrap().panel.step.impulse,
        ankle_step(&new).unwrap().panel.step.impulse
    );
    let floor = entity(&whole, "default::deck")
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut whole, floor).unwrap();
    assert!(ankle_step(&whole).is_none());
    whole.step_exact(1).unwrap();
    assert!(ankle_step(&whole).is_none());
}
#[test]
fn finite_rim_refusal_keeps_clock_joint_input_link_states_and_last_reaction() {
    // Only 0.3 mm of clearance on the right, moving 0.18 m/s toward the rim.
    let mut sim = load(&SDF.replace("0 0 -0.03", "-0.4197 0 -0.03")).unwrap();
    sim.step_exact(1).unwrap();
    for _ in 0..10 {
        let before = states(&sim);
        let t = sim.elapsed();
        let previous = ankle_step(&sim).unwrap().panel.step.impulse;
        drive(&mut sim);
        if sim.step_exact(1).is_err() {
            assert!(
                sim.world()
                    .resource::<gz_sim_core::StepFault>()
                    .error()
                    .unwrap()
                    .to_string()
                    .contains("deck rim")
            );
            assert_eq!(t, sim.elapsed());
            assert_eq!(before, states(&sim));
            assert_eq!(previous, ankle_step(&sim).unwrap().panel.step.impulse);
            let handle = entity(&sim, "default::boot::ankle")
                .get::<gz_sim_scene::JointHandle>()
                .unwrap()
                .id();
            assert_eq!(
                sim.world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                    .unwrap()
                    .pending_joint_effort(handle),
                Some(0.24)
            );
            return;
        }
    }
    panic!("expected a rim crossing");
}
#[test]
fn construction_rejects_unowned_coating_and_shank_contacts() {
    for xml in [
        SDF.replace(
            "<parent>sole</parent><child>upper</child>",
            "<parent>upper</parent><child>sole</child>",
        ),
        SDF.replace("0 0 0.245 0 0 0", "0 0 0.10 0 0 0"),
        SDF.replace("0 0 -0.03", "-0.425 0 -0.03"),
        SDF.replace("<static>true</static>", "<static>false</static>"),
        SDF.replace(
            "<substeps>8",
            "<flat_elastic_fast_path>true</flat_elastic_fast_path><substeps>8",
        ),
        SDF.replace("<substeps>8", "<substeps>1000000"),
        SDF.replace("0 0 0.02477291666666667", "0 0 -0.01"),
    ] {
        assert!(load(&xml).is_err());
    }
}
