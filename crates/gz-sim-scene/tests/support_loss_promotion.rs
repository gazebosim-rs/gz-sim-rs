// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier::{Dynamic, Prescribed},
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, model_fidelity, remove_model,
    reset_loaded_world, set_model_fidelity,
    support_loss_promotion::{SupportLossReason, last_promotion, set_enabled},
};
use sdformat::NoResources;

const SCENE: &str = include_str!("../../../examples/multifidelity_two_support_foot.sdf");

fn settled(sdf: &str) -> Simulation {
    let mut sim = load_world_with_sdf_step(
        sdf,
        "pads.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    sim.step_exact(2400).unwrap();
    sim
}

fn id(sim: &Simulation, name: &str) -> u64 {
    let scoped = format!("default::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
        .id()
}

fn solver(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}

#[test]
fn paused_removals_latch_the_first_loaded_pad_and_only_promote_its_assembly() {
    let start = SCENE.find("    <model").unwrap();
    let end = SCENE.find("  </world>").unwrap();
    let neighbor = SCENE[start..end]
        .replace("name=\"rocker\"", "name=\"other_rocker\"")
        .replace("name=\"left\"", "name=\"other_left\"")
        .replace("name=\"right\"", "name=\"other_right\"")
        .replace("<pose>0 0 0.1 ", "<pose>0 3 0.1 ")
        .replace("125 0 -0.06 ", "125 3 -0.06 ");
    let mut sim = settled(&format!("{}{}{}", &SCENE[..end], neighbor, &SCENE[end..]));
    let model = id(&sim, "rocker");
    let other = id(&sim, "other_rocker");
    let foot = body(&sim, "rocker::foot");
    let first_lost = body(&sim, "right::pad");
    for model in [model, other] {
        set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    }
    sim.set_paused(true);
    let boundary = sim.elapsed();
    for pad in ["right", "left"] {
        let pad = id(&sim, pad);
        remove_model(&mut sim, pad).unwrap();
        let paused = sim.tick().unwrap();
        assert!(paused.dt.is_zero());
        assert_eq!(sim.elapsed(), boundary);
        assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
        assert_eq!(last_promotion(&sim, model), None);
    }
    assert_eq!(
        solver(&sim).pending_support_loss(foot).unwrap().support,
        first_lost
    );
    sim.step_exact(1).unwrap();
    let event = last_promotion(&sim, model).unwrap();
    assert_eq!(
        (event.link, event.support, event.reason),
        (foot, first_lost, SupportLossReason::Retired)
    );
    assert_eq!(
        (event.start_time, event.sim_time),
        (boundary, boundary + sim.step_duration())
    );
    assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
    assert_eq!(model_fidelity(&sim, other).unwrap(), Prescribed);
    assert_eq!(last_promotion(&sim, other), None);
    assert!(!solver(&sim).has_pending_support_loss());
    sim.step_exact(2).unwrap();
    assert_eq!(last_promotion(&sim, model), Some(event));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(last_promotion(&sim, model), None);
    assert!(!solver(&sim).has_pending_support_loss());
}

#[test]
fn restored_model_has_default_protection_after_its_previous_registration_was_retired() {
    let mut sim = settled(SCENE);
    let model = id(&sim, "rocker");
    let right = id(&sim, "right");
    set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    remove_model(&mut sim, right).unwrap();
    remove_model(&mut sim, model).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(last_promotion(&sim, model), None);
    assert!(!solver(&sim).has_pending_support_loss());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2400).unwrap();
    set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    remove_model(&mut sim, right).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(last_promotion(&sim, model).unwrap().model, model);
}

#[test]
fn reenabled_policy_handles_a_pending_loss_on_the_next_advancing_step() {
    let mut sim = settled(SCENE);
    let model = id(&sim, "rocker");
    let right = id(&sim, "right");
    assert!(set_enabled(&mut sim, right, false).is_err());
    set_enabled(&mut sim, model, false).unwrap();
    set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    remove_model(&mut sim, right).unwrap();
    sim.step_exact(2).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    assert_eq!(last_promotion(&sim, model), None);
    let boundary = sim.elapsed();
    set_enabled(&mut sim, model, true).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(last_promotion(&sim, model).unwrap().start_time, boundary);
}

#[test]
fn moving_a_live_pad_triggers_only_after_the_shapes_separate() {
    let mut sim = settled(SCENE);
    let model = id(&sim, "rocker");
    let right = id(&sim, "right");
    let pad = body(&sim, "right::pad");
    set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    let mut state = gz_sim_scene::model_state(&sim, right).unwrap();
    state.translation[0] += 0.03;
    gz_sim_scene::set_model_state(&mut sim, right, state).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    assert_eq!(last_promotion(&sim, model), None);
    sim.set_paused(true);
    state.translation[0] += 1.;
    gz_sim_scene::set_model_state(&mut sim, right, state).unwrap();
    sim.tick().unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    let boundary = sim.elapsed();
    sim.set_paused(false);
    sim.tick().unwrap();
    let event = last_promotion(&sim, model).unwrap();
    assert_eq!(
        (event.support, event.reason),
        (pad, SupportLossReason::Separated)
    );
    assert_eq!(event.start_time, boundary);
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(pad)
            .is_some()
    );
}

/// The zone requests the cheaper tier on every advancing tick. The lift must
/// actually stop and the foot must settle before the native guard admits it again.
#[test]
fn zone_readmits_a_settled_foot_and_rearms_the_same_moving_support() {
    use gz_sim_scene::{
        fidelity_zone::{self, FidelityZone},
        queue_link_world_velocity,
    };
    use std::time::Duration;
    let mut sim = settled(include_str!(
        "../../../examples/multifidelity_support_reacquisition.sdf"
    ));
    let model = id(&sim, "rocker");
    let pad_link = id(&sim, "lift::pad");
    let foot = body(&sim, "rocker::foot");
    let pad = body(&sim, "lift::pad");
    let upper = body(&sim, "rocker::upper");
    // Each link is 1 kg with its origin at the center of mass in this fixture.
    let momentum_z = |sim: &Simulation| {
        let physics = sim.world().resource::<PhysicsBackend>();
        physics.state(foot).unwrap().velocity[2] + physics.state(upper).unwrap().velocity[2]
    };
    fidelity_zone::configure(
        &mut sim,
        model,
        vec![FidelityZone {
            center: [10., 0., 0.],
            promote_radius: 0.5,
            demote_radius: 1.,
        }],
        Duration::ZERO,
    )
    .unwrap();
    let velocity = |sim: &mut Simulation, z| {
        queue_link_world_velocity(sim, pad_link, [0., 0., z], [0.; 3]).unwrap();
    };
    let reaction = |sim: &Simulation| -> f64 {
        solver(sim)
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| (p.a, p.b) == (foot, pad) || (p.a, p.b) == (pad, foot))
            .map(|p| p.normal_impulse)
            .sum()
    };
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    // Replacing a queued command during pause must leave the watch armed.
    sim.set_paused(true);
    velocity(&mut sim, -0.8);
    velocity(&mut sim, 0.);
    sim.tick().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
    assert_eq!(last_promotion(&sim, model), None);

    let mut previous = None;
    for stroke_steps in [50, 70] {
        let boundary = sim.elapsed();
        velocity(&mut sim, -0.8);
        sim.tick().unwrap();
        assert_eq!(sim.elapsed(), boundary);
        assert_eq!(last_promotion(&sim, model), previous);
        assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
        sim.step_exact(1).unwrap();
        let event = last_promotion(&sim, model).unwrap();
        assert_eq!(event.start_time, boundary);
        assert_eq!(
            (event.link, event.support, event.reason),
            (foot, pad, SupportLossReason::MotionStarted)
        );
        assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
        previous = Some(event);
        sim.step_exact(stroke_steps - 1).unwrap();
        assert_eq!(reaction(&sim), 0.);
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .state(foot)
                .unwrap()
                .velocity[2]
                < 0.
        );
        velocity(&mut sim, 0.8);
        let mut landed = false;
        for _ in 0..stroke_steps {
            let before = momentum_z(&sim);
            sim.step_exact(1).unwrap();
            let normal_impulse = reaction(&sim);
            if !landed && normal_impulse > 0. {
                let gravity_impulse = -2. * 9.81 * sim.step_duration().as_secs_f64();
                let balance = momentum_z(&sim) - before - gravity_impulse;
                assert!(
                    (balance - normal_impulse).abs() < 0.001 * normal_impulse,
                    "landing impulse {normal_impulse} N·s, vertical momentum balance {balance} N·s"
                );
                landed = true;
            }
            assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
        }
        assert!(landed, "the free-falling foot must hit the ascending lift");
        velocity(&mut sim, 0.);
        sim.step_exact(1).unwrap();
        assert_eq!(model_fidelity(&sim, model).unwrap(), Dynamic);
        let mut supported = false;
        for _ in 0..3000 {
            supported = reaction(&sim) > 0.;
            sim.step_exact(1).unwrap();
            if model_fidelity(&sim, model).unwrap() == Prescribed {
                break;
            }
        }
        assert!(supported, "the last dynamic step must carry the foot");
        assert_eq!(model_fidelity(&sim, model).unwrap(), Prescribed);
        assert_eq!(last_promotion(&sim, model), previous);
    }
}
