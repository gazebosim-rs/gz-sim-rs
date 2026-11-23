// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

fn deck(x: f64, vx: f64) -> BodySpec {
    BodySpec {
        position: [x, 0.0, 0.0],
        velocity: [vx, 0.0, 0.0],
        shape: Shape::Box {
            half_extents: [0.5; 3],
        },
        mass: Some(1.0),
    }
}

#[test]
fn translation_queries_edits_and_late_spawns_share_one_clock() {
    let mut physics = MultiFidelityBackend::new([0.0; 3]).unwrap();
    let a = physics
        .spawn_kinematic_link(deck(1.0, 2.0), Default::default(), None)
        .unwrap();
    physics.step(Duration::from_secs(2)).unwrap();
    assert_eq!(
        physics.cast_ray([0.0; 3], [1., 0., 0.], 12., None).unwrap(),
        Some(4.5)
    );
    assert!(
        physics
            .set_world_velocity(a, [f64::NAN; 3], [0.; 3])
            .is_err()
    );
    let b = physics
        .spawn_kinematic_link(deck(10.0, -2.0), Default::default(), None)
        .unwrap();
    assert_eq!(
        physics
            .cast_ray([0.0; 3], [1., 0., 0.], 12., Some(a))
            .unwrap(),
        Some(9.5)
    );
    physics
        .set_world_velocity(a, [1., 0., 0.], [0.; 3])
        .unwrap();
    physics.step(Duration::from_secs(1)).unwrap();
    assert_eq!(physics.state(a).unwrap().position, [6., 0., 0.]);
    assert_eq!(physics.state(b).unwrap().position, [8., 0., 0.]);
    let mut edited = physics.state(b).unwrap();
    edited.position[0] = -8.;
    physics.set_body_state(b, edited).unwrap();
    physics.step(Duration::from_millis(500)).unwrap();
    assert_eq!(physics.state(b).unwrap().position[0], -9.);
    physics
        .set_world_velocity(a, [1., 0., 0.], [0., 0., 1.])
        .unwrap();
    physics.step(Duration::from_millis(100)).unwrap();
    assert!((physics.state(a).unwrap().position[0] - 6.6).abs() < 1e-12);
    assert!(physics.state(a).unwrap().orientation[2] > 0.04);
    physics.reset().unwrap();
    physics.step(Duration::from_secs(1)).unwrap();
    assert_eq!(physics.state(a).unwrap().position, [3., 0., 0.]);
    assert_eq!(physics.state(b).unwrap().position, [8., 0., 0.]);
}

#[test]
fn contact_solver_receives_translated_boundary_before_first_dynamic_payload() {
    let mut physics = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
    let deck_id = physics
        .spawn_kinematic_link(deck(0.0, 1.0), Default::default(), None)
        .unwrap();
    physics.step(Duration::from_secs(10)).unwrap();
    assert_eq!(physics.active_contact_pairs(), 0);
    let mut load = deck(10.0, 1.0);
    load.position[2] = 0.61;
    load.shape = Shape::Box {
        half_extents: [0.1; 3],
    };
    let load = physics.spawn_link(load, Default::default(), None).unwrap();
    for _ in 0..200 {
        physics.step(Duration::from_millis(5)).unwrap();
    }
    assert_eq!(physics.active_contact_pairs(), 1);
    let state = physics.state(load).unwrap();
    assert!((state.position[2] - 0.6).abs() < 0.01);
    assert!((state.position[0] - 11.).abs() < 0.01);
    assert!((physics.state(deck_id).unwrap().position[0] - 11.).abs() < 1e-10);
}

#[test]
fn scoped_read_matches_scalar_bits_across_gaps_edits_insertions_reset_and_native_fallback() {
    use gz_physics::{BodyId, BodyState, RotationSpec};
    fn bits(s: BodyState) -> Vec<u64> {
        s.position
            .into_iter()
            .chain(s.orientation)
            .chain(s.velocity)
            .chain(s.angular_velocity)
            .map(f64::to_bits)
            .collect()
    }
    fn check(p: &MultiFidelityBackend, ids: &[BodyId]) {
        let mut calls = 0;
        p.read_states(&mut |r| {
            calls += 1;
            for &id in ids.iter().rev().chain(ids.iter()).chain(ids.iter().take(2)) {
                assert_eq!(r.state(id).map(bits), p.state(id).map(bits));
            }
        });
        assert_eq!(calls, 1);
    }
    let mut p = MultiFidelityBackend::new([0.; 3]).unwrap();
    let missing = BodyId::from_raw(u64::MAX);
    check(&p, &[missing]);
    let rotation = RotationSpec {
        orientation: [0., 0., 0.6, 0.8],
        angular_velocity: [-0., 0., -0.],
    };
    let a = p
        .spawn_kinematic_link(deck(-0., -0.), rotation, None)
        .unwrap();
    let b = p
        .spawn_kinematic_link(deck(3., 0.125), Default::default(), None)
        .unwrap();
    let c = p
        .spawn_kinematic_link(deck(6., -0.375), rotation, None)
        .unwrap();
    check(&p, &[a, b, c, missing]);
    p.step(Duration::from_nanos(10_000_003)).unwrap();
    p.set_world_velocity(b, [-0.25, 0.5, 0.], [0.; 3]).unwrap();
    assert!(p.remove(a));
    let late = p
        .spawn_kinematic_link(deck(12., -0.125), rotation, None)
        .unwrap();
    let mut state = p.state(c).unwrap();
    state.position = [1., 8., 2.];
    p.set_body_state(c, state).unwrap();
    p.step(Duration::from_nanos(99_000_004)).unwrap();
    check(&p, &[a, b, c, late, missing]);
    assert!(
        (p.state(b).unwrap().position[0] - (3. + 0.125 * 0.010000003 - 0.25 * 0.099000004)).abs()
            < 1e-15
    );
    assert_eq!(
        p.state(late).unwrap().position[0],
        12. - 0.125 * 0.099000004
    );
    let before = p.state(b).unwrap();
    p.set_world_velocity(c, [0.; 3], [0., 0., 1.]).unwrap();
    assert_eq!(bits(before), bits(p.state(b).unwrap()));
    p.step(Duration::from_millis(1)).unwrap();
    check(&p, &[c, missing, b, a, late]);
    p.reset().unwrap();
    check(&p, &[a, b, c, late, missing]);
    assert_eq!(p.state(late).unwrap().position, [12., 0., 0.]);
    p.step(Duration::from_secs(1)).unwrap();
    check(&p, &[a, b, c, late, missing]);
}
