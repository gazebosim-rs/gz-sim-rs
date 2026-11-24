// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointHandle, LoadError, Name, PhysicsEngine, damped_sole::ankle_step,
    load_world_with_sdf_step, queue_joint_effort, queue_link_world_wrench, reset_loaded_world,
};
const WORLD: &str = include_str!("../../../examples/worlds/local_compliant_ankle_edge.sdf");

fn load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "edge.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
#[allow(deprecated)]
fn entity<'a>(sim: &'a Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    entity(sim, name).get::<EntityId>().unwrap().get()
}
fn states(sim: &Simulation) -> [BodyState; 2] {
    let backend = sim.world().resource::<PhysicsBackend>();
    ["default::boot::sole", "default::boot::upper"].map(|name| {
        backend
            .state(entity(sim, name).get::<BodyHandle>().unwrap().id())
            .unwrap()
    })
}
fn drive(sim: &mut Simulation, tick: usize) {
    if tick >= 33 {
        queue_link_world_wrench(
            sim,
            id(sim, "default::boot::upper"),
            [0.; 3],
            [0., -0.45, 0.],
        )
        .unwrap();
    }
}
#[derive(PartialEq, Debug)]
struct Observations {
    impulse: f64,
    edge: bool,
    onset: Option<usize>,
    points: usize,
    boundary_work: f64,
    bodies: [BodyState; 2],
}
fn run(sim: &mut Simulation) -> Observations {
    let (mut impulse, mut edge, mut onset, mut points, mut boundary_work) =
        (0., false, None, 0, 0.);
    for tick in 1..=100 {
        drive(sim, tick);
        sim.step_exact(1).unwrap();
        let r = ankle_step(sim).unwrap();
        assert!(r.panel.step.normal_impulse > 0.);
        assert!(r.panel.step.impulse.x < 0.);
        let hard = r.shank_contact;
        impulse += hard.normal_ns;
        edge |= r.edge_margin_m < -0.005;
        boundary_work += r.panel.step.boundary_work;
        if hard.normal_ns > 0. {
            onset.get_or_insert(tick);
            points = points.max(hard.peak_points);
            assert_eq!(hard.impulse, [0., 0., hard.normal_ns]);
            assert!(hard.angular_impulse[1] > 0.);
            assert!(hard.max_loaded_gap_m.unwrap() < 5e-5);
        } else {
            assert!(hard.max_loaded_gap_m.is_none());
        }
    }
    Observations {
        impulse,
        edge,
        onset,
        points,
        boundary_work,
        bodies: states(sim),
    }
}
#[test]
fn sole_remains_loaded_as_real_upper_contact_changes_and_reset_repeats_the_strike() {
    let mut sim = load(WORLD).unwrap();
    let hit = run(&mut sim);
    let miss = run(&mut load(&WORLD.replace("0.08 0.10 0.34", "0.08 0.10 0.12")).unwrap());
    let full = run(&mut load(&WORLD.replace("-0.415 0 -0.03", "0 0 -0.03")).unwrap());
    assert!(hit.edge && miss.edge && !full.edge);
    assert!(hit.boundary_work < 0. && miss.boundary_work < 0.);
    assert_eq!(full.boundary_work, 0.);
    for r in [&hit, &full] {
        assert_eq!(r.onset, Some(65));
        assert_eq!(r.points, 2);
        assert!((0.35..0.37).contains(&r.impulse));
    }
    assert_eq!((miss.onset, miss.impulse, miss.points), (None, 0., 0));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(ankle_step(&sim).is_none());
    assert_eq!(run(&mut sim), hit);
}
#[test]
fn unsupported_rigid_contact_rolls_back_the_complete_loaded_tick() {
    let mut sim =
        load(&WORLD.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.6</mu><mu2>0.6</mu2>")).unwrap();
    let joint = id(&sim, "default::boot::ankle");
    let handle = entity(&sim, "default::boot::ankle")
        .get::<JointHandle>()
        .unwrap()
        .id();
    for tick in 1..=100 {
        let before = states(&sim);
        let t = sim.elapsed();
        let previous = ankle_step(&sim).map(|v| v.panel.step.impulse);
        drive(&mut sim, tick);
        queue_joint_effort(&mut sim, joint, 0.001).unwrap();
        if sim.step_exact(1).is_err() {
            assert!(tick > 50);
            assert!(
                sim.world()
                    .resource::<gz_sim_core::StepFault>()
                    .error()
                    .unwrap()
                    .to_string()
                    .contains("frictionless")
            );
            assert_eq!(t, sim.elapsed());
            assert_eq!(before, states(&sim));
            assert_eq!(previous, ankle_step(&sim).map(|v| v.panel.step.impulse));
            let solver = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                .unwrap();
            assert_eq!(solver.pending_joint_effort(handle), Some(0.001));
            return;
        }
    }
    panic!("expected a frictional upper strike to be refused");
}
#[test]
fn inward_rim_motion_invalid_initial_clearances_and_ambiguous_ownership_are_refused() {
    let mut incoming = load(
        &WORLD
            .replace("-0.415 0 -0.03", "-0.44 0 -0.03")
            .replace("0.75 0 0", "-0.75 0 0"),
    )
    .unwrap();
    assert!(incoming.step_exact(1).is_err());
    for bad in [
        WORLD.replace("-0.415 0 -0.03", "-0.415 0.34 -0.03"),
        WORLD.replace("0 0 0.02477291666666667", "0 0 -0.01"),
        WORLD.replace("-0.415 0 -0.03", "-0.415 0 -0.022"),
        WORLD.replace("<native_ankle>true", "<native_ankle>false"),
    ] {
        assert!(load(&bad).is_err());
    }
}

#[test]
fn a_distal_edge_over_the_back_of_a_short_deck_cannot_claim_its_top_face() {
    let xml = WORLD
        .replace("1.20 0.70 0.06", "0.45 0.70 0.06")
        .replace("-0.415 0 -0.03", "-0.04 0 -0.03");
    let mut sim = load(&xml).unwrap();
    for tick in 1..=100 {
        drive(&mut sim, tick);
        if sim.step_exact(1).is_err() {
            let error = sim
                .world()
                .resource::<gz_sim_core::StepFault>()
                .error()
                .unwrap()
                .to_string();
            assert!(error.contains("interior of the finite top face"), "{error}");
            assert!(ankle_step(&sim).unwrap().panel.step.normal_impulse > 0.);
            assert_eq!(ankle_step(&sim).unwrap().shank_contact.normal_ns, 0.);
            return;
        }
    }
    panic!("expected the back rim to refuse an unsupported shank collision");
}
