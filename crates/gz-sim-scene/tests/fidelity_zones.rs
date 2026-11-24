// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::fidelity_zone::{FidelityZone, ZoneError, clear, configure};
use gz_sim_scene::{
    ArticulationTier::{Dynamic, Prescribed},
    Name, PhysicsEngine, load_world_with_sdf_step, model_fidelity, remove_model,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

fn zone(x: f64) -> FidelityZone {
    FidelityZone {
        center: [x, 0., 0.],
        promote_radius: 1.,
        demote_radius: 2.,
    }
}

fn load() -> (Simulation, u64) {
    let xml = r#"<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><model name='arm'>
    <plugin name='gz::sim::systems::SwitchableArticulation' filename='gz-sim-switchable-articulation-system'/>
    <link name='base'><collision name='c'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
    <link name='tip'><pose>1 0 0 0 0 0</pose><collision name='c'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
    <joint name='j' type='revolute'><parent>base</parent><child>tip</child><axis><xyz>0 0 1</xyz></axis></joint>
    </model></world></sdf>"#;
    let sim = load_world_with_sdf_step(
        xml,
        "zone.sdf",
        "w",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::arm"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (sim, id)
}

#[test]
fn zones_use_simulation_dwell_reset_epoch_and_retire_after_removal() {
    let (mut sim, id) = load();
    let count = sim.scheduled_system_counts();
    let dwell = sim.step_duration() * 2;
    configure(&mut sim, id, vec![zone(3.0)], dwell).unwrap();
    assert_eq!(sim.scheduled_system_counts().0, count.0 + 1);
    assert!(matches!(
        configure(&mut sim, id, vec![zone(f64::NAN)], Duration::ZERO),
        Err(ZoneError::InvalidZone)
    ));
    sim.set_paused(true);
    for _ in 0..4 {
        sim.tick().unwrap();
    }
    sim.step_exact(2).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Prescribed);
    configure(&mut sim, id, vec![zone(3.0), zone(1.0)], dwell).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
    configure(&mut sim, id, vec![zone(3.0), zone(2.0)], Duration::ZERO).unwrap();
    sim.step_exact(3).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
    configure(&mut sim, id, vec![zone(3.0)], dwell).unwrap();
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Prescribed);
    clear(&mut sim, id).unwrap();
    assert_eq!(sim.scheduled_system_counts(), count);
    configure(&mut sim, id, vec![zone(3.0)], dwell).unwrap();
    remove_model(&mut sim, id).unwrap();
    assert!(sim.scheduled_system_counts().0 < count.0 + 1);
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(4).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
}

#[test]
fn deferred_demotion_retries_and_moving_root_triggers_zone_entry_and_exit() {
    let (mut sim, id) = load();
    let root = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::arm::base"))
        .unwrap()
        .get::<gz_sim_scene::BodyHandle>()
        .unwrap()
        .id();
    let zone = FidelityZone {
        center: [1., 0., 0.],
        promote_radius: 0.1,
        demote_radius: 0.2,
    };
    configure(&mut sim, id, vec![zone], Duration::ZERO).unwrap();
    // An axial force through both centers gives the aggregate 10 m/s.
    let impulse = 20.0;
    let force = impulse / sim.step_duration().as_secs_f64();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .add_world_wrench(root, [force, 0., 0.], [0.; 3])
        .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Dynamic);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, id).unwrap(), Prescribed);
    let mut saw_entry = false;
    let mut saw_exit = false;
    for _ in 0..200 {
        let x = gz_sim_scene::articulation_root_pose(&sim, id)
            .unwrap()
            .translation[0];
        sim.step_exact(1).unwrap();
        match model_fidelity(&sim, id).unwrap() {
            Dynamic if !saw_entry => {
                assert!((x - 1.).abs() <= 0.1);
                saw_entry = true;
            }
            Prescribed if saw_entry => {
                assert!(x > 1.2);
                saw_exit = true;
                break;
            }
            _ => {}
        }
    }
    assert!(saw_entry && saw_exit);
}

#[test]
fn zone_faults_on_a_stale_binding_before_committing_time() {
    let (mut sim, id) = load();
    configure(&mut sim, id, vec![zone(3.)], Duration::ZERO).unwrap();
    let base = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::arm::base"))
        .unwrap()
        .id();
    sim.world_mut()
        .entity_mut(base)
        .remove::<gz_sim_scene::BodyHandle>();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(matches!(
        sim.fault()
            .and_then(|e| e.downcast_ref::<gz_sim_scene::FidelityError>()),
        Some(gz_sim_scene::FidelityError::InvalidBinding)
    ));
}

#[test]
fn continuous_eligibility_dwell_restarts_after_a_deferred_boundary_reset_and_replacement() {
    use gz_sim_scene::fidelity_zone::configure_with_eligibility_dwell;
    let (mut sim, model) = load();
    let step = sim.step_duration();
    let configure_dwell = |sim: &mut Simulation| {
        configure_with_eligibility_dwell(sim, model, vec![zone(10.)], Duration::ZERO, step * 3)
            .unwrap();
    };
    configure_dwell(&mut sim);
    sim.step_exact(2).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
    let root = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::arm::base"))
        .unwrap()
        .get::<gz_sim_scene::BodyHandle>()
        .unwrap()
        .id();
    // A force through the hinge axis leaves the tree able to coast next tick.
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .add_world_wrench(root, [2., 0., 0.], [0.; 3])
        .unwrap();
    sim.step_exact(1).unwrap();
    sim.set_paused(true);
    let boundary = sim.elapsed();
    for _ in 0..5 {
        sim.tick().unwrap();
    }
    assert_eq!(sim.elapsed(), boundary);
    sim.step_exact(3).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2).unwrap();
    configure_dwell(&mut sim);
    sim.step_exact(3).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
}

#[test]
fn parked_eligibility_checks_preserve_the_reference_contact_state_until_actual_demotion() {
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::fidelity_zone::configure_with_eligibility_dwell;
    let load_lift = || {
        load_world_with_sdf_step(
            include_str!("../../../examples/multifidelity_support_reacquisition.sdf"),
            "lift.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap()
    };
    let mut candidate = load_lift();
    let mut reference = load_lift();
    for sim in [&mut candidate, &mut reference] {
        sim.step_exact(2400).unwrap();
    }
    let id = |name: &str| {
        candidate
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
    };
    let model = id("default::rocker").get::<EntityId>().unwrap().get();
    let [foot, upper, pad] = [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::lift::pad",
    ]
    .map(|n| id(n).get::<gz_sim_scene::BodyHandle>().unwrap().id());
    let step = candidate.step_duration();
    configure_with_eligibility_dwell(
        &mut candidate,
        model,
        vec![zone(10.)],
        Duration::ZERO,
        step * 10,
    )
    .unwrap();
    for _ in 0..10 {
        candidate.step_exact(1).unwrap();
        reference.step_exact(1).unwrap();
        let observations = |sim: &Simulation| {
            let backend = sim.world().resource::<PhysicsBackend>();
            let solver = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            let patches = solver
                .rigid_contact_patches()
                .into_iter()
                .map(|p| (p.a, p.b, p.normal_on_a, p.points))
                .collect::<Vec<_>>();
            (
                [foot, upper, pad].map(|id| backend.state(id).unwrap()),
                solver.native_contact_warmstart(foot, pad).unwrap(),
                patches,
            )
        };
        assert_eq!(observations(&candidate), observations(&reference));
        assert_eq!(model_fidelity(&candidate, model).unwrap(), Dynamic);
    }
    candidate.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&candidate, model).unwrap(), Prescribed);
}

#[test]
fn a_fault_applying_lift_input_leaves_a_mature_dwell_dynamic() {
    use gz_sim_scene::{
        BodyHandle, fidelity_zone::configure_with_eligibility_dwell, queue_link_world_velocity,
    };
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../examples/multifidelity_support_reacquisition.sdf"),
        "lift.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    sim.step_exact(2400).unwrap();
    let ids = |name: &str| {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap();
        (e.id(), e.get::<EntityId>().unwrap().get())
    };
    let (_, model) = ids("default::rocker");
    let (pad_entity, pad) = ids("default::lift::pad");
    let dt = sim.step_duration();
    let before = sim.scheduled_system_counts();
    configure_with_eligibility_dwell(&mut sim, model, vec![zone(10.)], Duration::ZERO, dt).unwrap();
    configure(&mut sim, model, vec![zone(10.)], Duration::ZERO).unwrap();
    clear(&mut sim, model).unwrap();
    assert_eq!(sim.scheduled_system_counts(), before);
    configure_with_eligibility_dwell(&mut sim, model, vec![zone(10.)], Duration::ZERO, dt).unwrap();
    sim.step_exact(1).unwrap();
    let t = sim.elapsed();
    queue_link_world_velocity(&mut sim, pad, [0., 0., -1.25], [0.; 3]).unwrap();
    sim.world_mut()
        .entity_mut(pad_entity)
        .remove::<BodyHandle>();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.elapsed(), t);
    assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
}
