// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::{ContactWarmstart, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier::{Dynamic, Prescribed},
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, remove_model, reset_loaded_world,
    set_model_fidelity,
};
use sdformat::NoResources;

const SCENE: &str = include_str!("../../../examples/multifidelity_sliding_payload.sdf");

fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
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

fn settled() -> Simulation {
    let mut sim = load_world_with_sdf_step(
        &SCENE.replace(">0.001<", ">0.00025<"),
        "floor.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    sim.step_exact(8000).unwrap();
    sim
}

#[test]
fn both_foot_caches_survive_pause_hold_and_promotion_without_delivering_load_during_hold() {
    let mut sim = settled();
    let arm = id(&sim, "default::arm");
    let feet = ["default::arm::base", "default::arm::tip"].map(|n| body(&sim, n));
    let floor = body(&sim, "default::floor::ground");
    let ram = body(&sim, "default::ram::head");
    let seeds = |s: &Simulation| feet.map(|foot| solver(s).contact_warmstart(foot, floor).unwrap());
    let initial = seeds(&sim);
    for c in initial {
        assert!(c.points >= 3 && c.normal_ns > 0.);
    }
    let ram_before = solver(&sim).contact_warmstart(ram, floor).unwrap();
    sim.set_paused(true);
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    sim.tick().unwrap();
    assert_eq!(seeds(&sim), initial);
    assert_eq!(
        solver(&sim).contact_warmstart(ram, floor).unwrap(),
        ram_before
    );
    sim.step_exact(200).unwrap();
    let held = seeds(&sim);
    for (seed, original) in held.into_iter().zip(initial) {
        assert_eq!(seed.points, original.points);
        assert!((seed.normal_ns - original.normal_ns).abs() < 1e-12);
    }
    let delivered = solver(&sim).rigid_contact_observations();
    for foot in feet {
        assert!(
            delivered
                .iter()
                .filter(|p| (p.a, p.b) == (floor, foot) || (p.a, p.b) == (foot, floor))
                .all(|p| p.normal_impulse == 0.)
        );
    }
    let ram_load = delivered.iter().any(|p| {
        ((p.a, p.b) == (ram, floor) || (p.b, p.a) == (ram, floor)) && p.normal_impulse > 0.
    });
    assert!(
        ram_load,
        "unrelated dynamic payload must still have a floor reaction"
    );
    set_model_fidelity(&mut sim, arm, Dynamic).unwrap();
    assert_eq!(seeds(&sim), held);
    sim.step_exact(1).unwrap();
    for c in seeds(&sim) {
        assert!(c.normal_ns > 0.);
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(seeds(&sim), [ContactWarmstart::default(); 2]);
}

#[test]
fn retiring_a_loaded_floor_drops_its_seeds_and_the_arm_falls_on_promotion() {
    let mut sim = settled();
    let arm = id(&sim, "default::arm");
    let floor_model = id(&sim, "default::floor");
    let base = body(&sim, "default::arm::base");
    let floor = body(&sim, "default::floor::ground");
    assert!(
        solver(&sim)
            .contact_warmstart(base, floor)
            .unwrap()
            .normal_ns
            > 0.
    );
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    sim.step_exact(50).unwrap();
    remove_model(&mut sim, floor_model).unwrap();
    assert!(solver(&sim).contact_warmstart(base, floor).is_err());
    set_model_fidelity(&mut sim, arm, Dynamic).unwrap();
    let vz = |s: &Simulation| {
        s.world()
            .resource::<PhysicsBackend>()
            .state(base)
            .unwrap()
            .velocity[2]
    };
    let initial = vz(&sim);
    sim.step_exact(1).unwrap();
    assert!((vz(&sim) - initial + 9.81 * 0.00025).abs() < 1e-8);
}

/// The heel moves past the right lip of one pad while the toe remains on the
/// other. Both pads still exist and carry load after the dynamic solve.
#[test]
fn promotion_rebuilds_only_the_expired_contact_under_a_slipping_foot() {
    let rocker = include_str!("../../../examples/multifidelity_rocking_foot.sdf");
    let (scene, _) = rocker.split_once("    <model name=\"ledge\">").unwrap();
    let mut sdf = scene.to_owned();
    for (name, center) in [("heel_pad", -0.190), ("toe_pad", 0.030)] {
        sdf.push_str(&format!(
            r#"<model name="{name}"><static>true</static><pose>{center} 0 -0.08 0 0 0</pose>
            <link name="ground"><collision name="top"><geometry><box><size>0.20 0.50 0.16</size>
            </box></geometry></collision></link></model>"#
        ));
    }
    sdf.push_str("</world></sdf>");
    let load = || {
        load_world_with_sdf_step(
            &sdf,
            "two_edges.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap()
    };
    let mut sim = load();
    let mut reference = load();
    let foot = body(&sim, "default::rocker::foot");
    let model = id(&sim, "default::rocker");
    let pads = ["default::heel_pad::ground", "default::toe_pad::ground"].map(|n| body(&sim, n));
    let caches = |s: &Simulation| pads.map(|pad| solver(s).contact_warmstart(foot, pad).unwrap());
    let loads = |s: &Simulation| {
        let pairs = solver(s).rigid_contact_observations();
        pads.map(|pad| {
            pairs
                .iter()
                .filter(|p| (p.a, p.b) == (foot, pad) || (p.a, p.b) == (pad, foot))
                .map(|p| p.normal_impulse)
                .sum::<f64>()
        })
    };
    sim.step_exact(19).unwrap();
    reference.step_exact(19).unwrap();
    let initial = caches(&sim);
    assert!(initial.iter().all(|c| c.normal_ns > 0.));
    assert_eq!(caches(&reference), initial);
    let state = |s: &Simulation| s.world().resource::<PhysicsBackend>().state(foot).unwrap();
    let before = state(&sim);
    set_model_fidelity(&mut sim, model, Prescribed).unwrap();
    let demoted = caches(&sim);
    eprintln!(
        "demotion at tick 19, cache {initial:?}, foot velocity {:?}",
        before.velocity
    );
    sim.step_exact(800).unwrap();
    reference.step_exact(800).unwrap();
    let statuses = pads.map(|p| solver(&sim).held_support_pair_status(foot, p).unwrap());
    eprintln!("pair expiry during prescribed motion {statuses:?}");
    assert_eq!(
        statuses[0].expiry,
        Some(gz_physics_multifidelity::SupportExpiry::AnchorOutsideFace)
    );
    assert_eq!(statuses[1].expiry, None);
    assert!(statuses[0].max_face_overhang_m.unwrap() > 0.);
    assert!(statuses[1].max_face_overhang_m.unwrap() < 0.);
    let held = caches(&sim);
    eprintln!("held seeds {held:?}, last held reaction {:?}", loads(&sim));
    assert_eq!(loads(&sim), [0.; 2]);
    set_model_fidelity(&mut sim, model, Dynamic).unwrap();
    let resumed = caches(&sim);
    let raw =
        |s: &Simulation| pads.map(|pad| solver(s).native_contact_warmstart(foot, pad).unwrap());
    let pending = |s: &Simulation| {
        pads.map(|pad| {
            solver(s)
                .contact_warmstart_discard_pending(foot, pad)
                .unwrap()
        })
    };
    assert_eq!(raw(&sim), held);
    assert_eq!(pending(&sim), [true, false]);
    eprintln!("before first resumed solve {resumed:?}");
    sim.step_exact(1).unwrap();
    reference.step_exact(1).unwrap();
    assert_eq!(pending(&sim), [false; 2]);
    assert!(
        raw(&sim)
            .into_iter()
            .all(|v| v.points > 0 && v.normal_ns > 0.)
    );
    let actual = loads(&sim);
    let expected = loads(&reference);
    eprintln!("first resumed normal impulses (N s) {actual:?}, always-dynamic {expected:?}");
    assert!(actual.into_iter().chain(expected).all(|j| j > 0.));
    assert_eq!(demoted, initial);
    assert_eq!(held, initial);
    assert_eq!(resumed[0], ContactWarmstart::default());
    assert_eq!(resumed[1], held[1]);
    assert!(resumed[1].normal_ns > 0.);
    let mut late = [[0.; 2]; 2];
    for tick in 1..=200 {
        sim.step_exact(1).unwrap();
        reference.step_exact(1).unwrap();
        if tick > 160 {
            for (sum, impulse) in late.iter_mut().zip([loads(&sim), loads(&reference)]) {
                for (pad_sum, j) in sum.iter_mut().zip(impulse) {
                    *pad_sum += j;
                }
            }
        }
    }
    let sample_time = 40. * sim.step_duration().as_secs_f64();
    let forces = late.map(|pair| pair.map(|j| j / sample_time));
    eprintln!(
        "mean support forces (N) over 4–5 ms after the resumed solve: switched {:?}, reference {:?}",
        forces[0], forces[1]
    );
}
