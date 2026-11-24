// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local rigid contact against a dynamic peer, restitution and limiting friction.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
use std::time::Instant;

const DT: f64 = 0.0005;
const WELD: &str = include_str!("../../../examples/local_robot_contact.sdf");
const SINGLE: &str = include_str!("../../../examples/local_robot_contact_single.sdf");
const BOUNCE: &str = include_str!("../../../examples/local_robot_bounce.sdf");
const STICK: &str = include_str!("../../../examples/local_robot_ramp_stick.sdf");
const SLIDE: &str = include_str!("../../../examples/local_robot_ramp_slide.sdf");

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "local.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn body(sim: &mut Simulation, name: &str) -> BodyId {
    let world = sim.world_mut();
    world
        .query::<(&Name, &BodyHandle)>()
        .iter(world)
        .find(|(n, _)| n.0 == name)
        .unwrap()
        .1
        .id()
}

fn state(sim: &Simulation, id: BodyId) -> BodyState {
    sim.world().resource::<PhysicsBackend>().state(id).unwrap()
}

fn normal_impulse(sim: &Simulation, foot: BodyId, deck: BodyId) -> f64 {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_observations()
        .into_iter()
        .filter(|row| (row.a == foot && row.b == deck) || (row.b == foot && row.a == deck))
        .map(|r| r.normal_impulse)
        .sum()
}

#[derive(Clone, Copy)]
struct Landing {
    center: DVec3,
    deck: DVec3,
    spin: DVec3,
    deck_spin: DVec3,
    momentum: DVec3,
    deck_momentum: DVec3,
    impulse: f64,
}

fn trajectory(xml: &str, welded: bool) -> Vec<Landing> {
    let mut sim = load(xml);
    let foot = body(&mut sim, "default::rig::foot");
    let top = welded.then(|| body(&mut sim, "default::rig::upper"));
    let deck = body(&mut sim, "default::platform::deck");
    let mut samples = Vec::new();
    let mut times = Vec::new();
    for tick in 0..=400 {
        if tick > 0 {
            let start = Instant::now();
            sim.step_exact(1).unwrap();
            if tick > 5 {
                times.push(start.elapsed().as_nanos());
            }
        }
        let s = state(&sim, foot);
        let d = state(&sim, deck);
        let (center, momentum) = if let Some(upper) = top {
            let u = state(&sim, upper);
            (
                (DVec3::from(s.position) + DVec3::from(u.position)) / 2.,
                DVec3::from(s.velocity) + DVec3::from(u.velocity),
            )
        } else {
            (
                DVec3::from(s.position) + DQuat::from_array(s.orientation) * (0.07 * DVec3::Z),
                2. * DVec3::from(s.velocity),
            )
        };
        samples.push(Landing {
            center,
            momentum,
            spin: s.angular_velocity.into(),
            deck: d.position.into(),
            deck_spin: d.angular_velocity.into(),
            deck_momentum: 4. * DVec3::from(d.velocity),
            impulse: normal_impulse(&sim, foot, deck),
        });
    }
    times.sort_unstable();
    println!(
        "welded={welded} trajectory_step_p95_ns={}",
        times[(times.len() * 95).div_ceil(100) - 1]
    );
    samples
}

#[test]
fn both_sides_of_a_moving_impact_match_the_equivalent_body() {
    let weld = trajectory(WELD, true);
    let one = trajectory(SINGLE, false);
    let i = weld.iter().position(|s| s.impulse > 0.).unwrap();
    assert_eq!(i, one.iter().position(|s| s.impulse > 0.).unwrap());
    let w = weld[i];
    let before = weld[i - 1];
    let reaction = w.momentum - before.momentum;
    let deck_reaction = w.deck_momentum - before.deck_momentum;
    let mut max_pos: f64 = 0.;
    let mut max_spin: f64 = 0.;
    let mut max_normal: f64 = 0.;
    let mut max_residual: f64 = 0.;
    for (w, s) in weld.iter().zip(&one) {
        max_pos = max_pos
            .max(w.center.distance(s.center))
            .max(w.deck.distance(s.deck));
        max_spin = max_spin
            .max(w.spin.distance(s.spin))
            .max(w.deck_spin.distance(s.deck_spin));
        max_normal = max_normal.max((w.impulse - s.impulse).abs());
        max_residual = max_residual
            .max((w.momentum + w.deck_momentum).distance(weld[0].momentum + weld[0].deck_momentum));
    }
    println!(
        "impact_ms={} normal_ns={} single_ns={} linear_foot_ns={reaction:?} linear_deck_ns={deck_reaction:?} peak_position_m={max_pos:e} peak_spin_rad_s={max_spin:e} peak_impulse_difference_ns={max_normal:e} max_momentum_residual_ns={max_residual:e}",
        i as f64 * DT * 1000.,
        w.impulse,
        one[i].impulse
    );
    assert!(w.impulse > 0.2);
    assert!((reaction.z - w.impulse).abs() < 1e-10);
    assert!((reaction + deck_reaction).length() < 1e-10);
    assert!(max_residual < 1e-9);
    assert!(max_pos < 1e-8 && max_spin < 1e-6 && max_normal < 1e-6);
}

fn rebound(sim: &mut Simulation) -> f64 {
    let ball = body(sim, "default::ball::body");
    let floor = body(sim, "default::floor::body");
    for _ in 0..300 {
        let before = state(sim, ball);
        sim.step_exact(1).unwrap();
        let impulse = normal_impulse(sim, ball, floor);
        if impulse > 0. {
            let after = state(sim, ball);
            assert!((impulse - (after.velocity[2] - before.velocity[2])).abs() < 1e-9);
            return -after.velocity[2] / before.velocity[2];
        }
    }
    panic!("missing first contact");
}

#[test]
fn authored_restitution_controls_normal_rebound_and_survives_reset() {
    for coefficient in [0., 0.3, 0.65, 1.] {
        let xml = BOUNCE
            .replace(
                ">0.65</restitution>",
                &format!(">{coefficient}</restitution>"),
            )
            .replace(">0.2</restitution>", ">0</restitution>");
        let mut sim = load(&xml);
        let observed = rebound(&mut sim);
        println!("coefficient={coefficient} measured={observed}");
        assert!((observed - coefficient).abs() < 1e-8);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(rebound(&mut sim), observed);
    }
}

#[test]
fn invalid_and_duplicate_restitution_declarations_are_rejected() {
    let tag = "<restitution link=\"body\">0.65</restitution>";
    assert!(BOUNCE.contains(tag));
    for replacement in [
        "<restitution link=\"ghost\">0.5</restitution>",
        "<restitution link=\"body\">1.3</restitution>",
        "<restitution link=\"body\">NaN</restitution>",
        "<restitution link=\"body\">0.65</restitution><extra/>",
        "<restitution link=\"body\">0.65</restitution><restitution link=\"body\">0.2</restitution>",
    ] {
        let xml = BOUNCE.replace(tag, replacement);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "x.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn slope_one_degree_to_either_side_of_coulomb_limit_sticks_or_slides() {
    for (xml, delta) in [(STICK, -1.), (SLIDE, 1.)] {
        let theta = 0.4f64.atan() + f64::to_radians(delta);
        let downhill = DVec3::new(-theta.cos(), 0., -theta.sin());
        let mut sim = load(xml);
        let id = body(&mut sim, "default::block::body");
        let start = DVec3::from(state(&sim, id).position);
        sim.step_exact(2000).unwrap();
        let end = state(&sim, id);
        let displacement = (DVec3::from(end.position) - start).dot(downhill);
        let speed = DVec3::from(end.velocity).dot(downhill);
        let expected = 9.81 * (theta.sin() - 0.4 * theta.cos()).max(0.);
        println!(
            "ramp_deg={} downhill_travel_m={displacement} speed_m_s={speed} predicted_speed_m_s={expected}",
            theta.to_degrees()
        );
        assert!((speed - expected).abs() < 0.012);
        assert!((displacement - expected / 2.).abs() < 0.008);
    }
}
