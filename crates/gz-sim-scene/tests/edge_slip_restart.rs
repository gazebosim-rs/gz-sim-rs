// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::SeparatingFeature;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact};
use std::time::Duration;

const WORLD: &str = include_str!("../../../examples/multifidelity_resumed_edge_slip.sdf");

fn load(xml: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        xml,
        "resumed-edge-slip.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn first(xml: &str) -> primary_contact::Refinement {
    let mut sim = load(xml);
    sim.step_exact(1).unwrap();
    let r = primary_contact::last_refinement(&sim).unwrap().clone();
    assert_eq!(r.sim_time - r.start_time, Duration::from_micros(500));
    r
}

#[test]
fn a_sticking_edge_impact_launches_slip_within_the_resumed_public_tick() {
    // Two independently integrated meshes from the original pre-primary donor
    // give these same event time, signed impulse and deck-relative position.
    let r = first(WORLD);
    let events: Vec<_> = r
        .interval
        .point_transfers
        .iter()
        .flat_map(|t| {
            t.intervals
                .iter()
                .filter_map(move |s| s.impact.map(|h| (t.start.as_secs_f64(), h)))
        })
        .collect();
    assert_eq!(events.len(), 3, "{events:?}");
    let (start, edge) = events[2];
    let time = start + edge.local_bracket[0];
    assert!((time - 92.0702635585503e-6).abs() < 1e-10);
    assert!(edge.local_bracket[1] - edge.local_bracket[0] < 1e-12);
    let expected_point = DVec3::new(-0.14, 0.10009849431788469, -0.03419099999999534)
        + DVec3::Z * (0.522 * (0.0005 + time));
    assert!(expected_point.distance(edge.point.into()) < 1e-9);
    assert!(edge.sticking && edge.lost_energy > 0. && edge.opposite_opening_speed > 0.);
    let impulse = DVec3::from(edge.impulse_on_foot);
    assert!(
        impulse.distance(DVec3::new(
            -0.00991499524410628,
            -0.010068370032446523,
            0.18015849297037806
        )) < 1e-9
    );
    let launch = r
        .interval
        .point_transfers
        .iter()
        .flat_map(|t| {
            t.intervals
                .iter()
                .map(move |s| (t.start.as_secs_f64() + s.offset, s))
        })
        .find(|(at, s)| {
            (*at - time).abs() < 1e-12 && s.feature == SeparatingFeature::EdgePair(0, 1)
        })
        .expect("a measured outgoing skew edge immediately follows the sticking impulse")
        .1;
    assert!(!launch.sticking && launch.normal_impulse > 0. && launch.friction_work <= 0.);
    let witness = launch
        .slip_growth
        .expect("bounded slip acceleration from rest");
    let port = DVec3::from(launch.end_material_velocity);
    assert!(witness.acceleration > 0.);
    assert!(
        witness.direction.dot(port)
            >= witness.initial_component + witness.acceleration * launch.duration - 1e-12
    );
}

#[test]
fn disabling_skew_slip_launch_leaves_the_edge_transition_to_native_contacts() {
    let r = first(&WORLD.replace(
        "<follow_skew_slip_launch>true",
        "<follow_skew_slip_launch>false",
    ));
    assert!(
        r.interval
            .point_transfers
            .iter()
            .all(|t| t.intervals.iter().all(|s| s.slip_growth.is_none()))
    );
    assert!(
        r.interval
            .point_transfers
            .iter()
            .flat_map(|t| t.intervals.iter().filter_map(move |s| s
                .impact
                .map(|hit| t.start.as_secs_f64() + hit.local_bracket[0])))
            .all(|s| (s - 92.0702635585503e-6).abs() > 1e-10)
    );
    assert!(r.requested_pairs[0].normal_impulse > 0.);
}

#[test]
fn the_sliding_edge_remains_loaded_on_the_original_public_clock() {
    let mut sim = load(WORLD);
    let mut impulse = DVec3::ZERO;
    for step in 1..=18 {
        sim.step_exact(1).unwrap();
        let r = primary_contact::last_refinement(&sim).unwrap();
        assert_eq!(r.sim_time, Duration::from_micros(step * 500));
        assert_eq!(r.interval.native_steps, 1);
        let pair = &r.requested_pairs[0];
        assert!(pair.active_at_end && pair.normal_impulse > 0.);
        impulse += DVec3::from(pair.normal_impulse_on_foot)
            + DVec3::from(pair.tangent_impulse_on_foot.unwrap());
    }
    // The mechanical control is independently integrated from the pre-primary
    // donor on two meshes at these same 18 actuator boundaries.
    assert!(
        impulse.distance(DVec3::new(
            -0.5578776374015847,
            0.06790080283197969,
            1.6183670763048636
        )) < 1e-8
    );
    for (name, position) in [
        (
            "default::rocker::foot",
            [
                0.0004865183685305971,
                0.0012550117682743006,
                0.032328053354257834,
            ],
        ),
        (
            "default::rocker::upper",
            [
                0.00048045938381610545,
                0.0034453266726607755,
                0.17231091831964993,
            ],
        ),
    ] {
        let actual = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                if entity.get::<gz_sim_scene::Name>()?.0 != name {
                    return None;
                }
                Some(entity.get::<gz_sim_scene::WorldPose>()?.translation)
            })
            .unwrap();
        assert!(DVec3::from(actual).distance(position.into()) < 1e-9);
    }
}

#[test]
fn the_foot_rolls_from_the_skew_edge_onto_one_sliding_corner() {
    let mut sim = load(WORLD);
    sim.step_exact(19).unwrap();
    let r = primary_contact::last_refinement(&sim).unwrap();
    assert_eq!(r.start_time, Duration::from_millis(9));
    assert_eq!(r.sim_time, Duration::from_micros(9500));
    let mut hits = r.interval.point_transfers.iter().flat_map(|span| {
        span.intervals.windows(2).filter_map(move |arcs| {
            let h = arcs[0].impact?;
            Some((span.start, &arcs[0], h, &arcs[1]))
        })
    });
    let (start, incoming, hit, next) = hits.next().unwrap();
    assert!(hits.next().is_none());
    assert_eq!(incoming.feature, SeparatingFeature::EdgePair(0, 1));
    assert_eq!(next.feature, SeparatingFeature::SupportFace(2));
    assert!(!next.sticking && !hit.sticking);
    assert!(hit.lost_energy > 0. && hit.opposite_opening_speed > 0.);
    assert!(next.normal_impulse > 0. && next.friction_work < 0.);
    // Both finer reference grids close the event independently. Its clock is
    // relative to the cold restart, while the receipt uses this native span.
    let stamp = (r.start_time + start).as_secs_f64() + hit.local_bracket[0];
    assert!((stamp - 0.0092424361271666).abs() < 1e-10);
    assert!(
        DVec3::from(hit.impulse_on_foot).distance(DVec3::new(
            -0.002122121296044825,
            -0.011715946974594875,
            0.023813173858740103
        )) < 5e-10
    );
    let p = &r.requested_pairs[0];
    let j = DVec3::from(p.normal_impulse_on_foot) + DVec3::from(p.tangent_impulse_on_foot.unwrap());
    assert!(
        j.distance(DVec3::new(
            -0.0012168899278228729,
            -0.013473239015173166,
            0.028550301405128042
        )) < 1e-9
    );
}
