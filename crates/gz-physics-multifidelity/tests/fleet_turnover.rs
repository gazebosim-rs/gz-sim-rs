// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, RotationSpec, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

const STEP: Duration = Duration::from_millis(10);

struct Lane {
    solver: MultiFidelityBackend,
    decks: Vec<BodyId>,
    cargo: BodyId,
}
impl Lane {
    fn new(automatic: bool) -> Self {
        Self::with_velocity(automatic, [0.25, 0., 0.])
    }
    fn with_velocity(automatic: bool, velocity: [f64; 3]) -> Self {
        let mut solver = MultiFidelityBackend::new([0.; 3]).unwrap();
        if !automatic {
            solver.disable_fleet_isolation();
        }
        let shape = |position, half_extents, velocity| BodySpec {
            shape: Shape::Box { half_extents },
            mass: Some(1.),
            position,
            velocity,
        };
        let decks = (0..256)
            .map(|i| {
                solver
                    .spawn_kinematic_link(
                        shape([f64::from(i) * 1.25, 0., 0.], [0.48, 0.23, 0.1], velocity),
                        RotationSpec {
                            orientation: [0., 0., 0.6, 0.8],
                            ..Default::default()
                        },
                        None,
                    )
                    .unwrap()
            })
            .collect();
        let cargo = solver
            .spawn_link(
                shape([0., 0., 0.201], [0.14, 0.05, 0.1], velocity),
                Default::default(),
                None,
            )
            .unwrap();
        Self {
            solver,
            decks,
            cargo,
        }
    }
}
fn compare(a: BodyState, b: BodyState) {
    let values = |s: BodyState| {
        s.position
            .into_iter()
            .chain(s.orientation)
            .chain(s.velocity)
            .chain(s.angular_velocity)
    };
    for (x, y) in values(a).zip(values(b)) {
        assert!((x - y).abs() < 1e-8, "state {a:?} vs {b:?}");
    }
}
#[test]
fn contacted_decks_retire_and_reactivate_at_current_pose_after_abrupt_reversal() {
    let mut automatic = Lane::new(true);
    let mut native = Lane::new(false);
    let mut returned_contact = false;
    let mut first_contact = false;
    let mut first_retired = false;
    let mut retirement_at_turn = 0;
    for tick in 1..=252 {
        let command = match tick {
            10 => Some([8.25, 0., 0.]),
            126 => {
                // The glancing edge deflects the first pass off-axis. Aim over
                // deck zero at barrier 209, then descend onto its top face.
                let [x, y, _] = native.solver.state(native.cargo).unwrap().position;
                Some([(0.25 * 2.09 - x) / 0.84, -y / 0.84, 0.])
            }
            210 => Some([0.25, 0., -4.]),
            _ => None,
        };
        for lane in [&mut automatic, &mut native] {
            if let Some(velocity) = command {
                lane.solver
                    .set_world_velocity(lane.cargo, velocity, [0.; 3])
                    .unwrap();
            }
            lane.solver.step(STEP).unwrap();
        }
        let stats = automatic.solver.fleet_isolation_stats().unwrap();
        if tick == 125 {
            retirement_at_turn = stats.retired_decks;
        }
        let auto = automatic.solver.rigid_contact_observations();
        let control = native.solver.rigid_contact_observations();
        assert_eq!(
            auto.iter()
                .map(|p| (p.a, p.b, p.active))
                .collect::<Vec<_>>(),
            control
                .iter()
                .map(|p| (p.a, p.b, p.active))
                .collect::<Vec<_>>(),
            "tick {tick}"
        );
        let touches_first = auto
            .iter()
            .any(|p| p.a == automatic.decks[0] && p.b == automatic.cargo && p.active);
        first_contact |= tick < 10 && touches_first;
        let first_native = automatic
            .solver
            .fleet_deck_is_native(automatic.decks[0])
            .unwrap();
        assert_eq!(automatic.solver.fleet_deck_is_native(automatic.cargo), None);
        if touches_first {
            assert!(first_native);
        }
        first_retired |= tick > 10 && tick < 200 && first_contact && !first_native;
        returned_contact |= tick > 200 && first_retired && touches_first;
        for (a, b) in auto.iter().zip(control) {
            assert!((a.normal_impulse - b.normal_impulse).abs() < 1e-8);
        }
        compare(
            automatic.solver.state(automatic.cargo).unwrap(),
            native.solver.state(native.cargo).unwrap(),
        );
        let first = automatic.decks[0];
        let p = automatic.solver.state(first).unwrap();
        compare(p, native.solver.state(native.decks[0]).unwrap());
        automatic
            .solver
            .read_states(&mut |q| assert_eq!(q.state(first), Some(p)));
        let ray = |s: &MultiFidelityBackend, cargo| {
            s.cast_ray([p.position[0], 0., 2.], [0., 0., -1.], 3., Some(cargo))
                .unwrap()
                .unwrap()
        };
        assert!(
            (ray(&automatic.solver, automatic.cargo) - ray(&native.solver, native.cargo)).abs()
                < 1e-10
        );
    }
    assert!(
        first_contact && returned_contact,
        "first={first_contact}, returned={returned_contact}"
    );
    assert!(retirement_at_turn >= 3, "retired={retirement_at_turn}");
    assert!(first_retired);
}

#[test]
fn stationary_receiver_struck_by_fast_dynamic_box_keeps_native_contact_response() {
    let mut automatic = Lane::new(true);
    let mut native = Lane::new(false);
    for lane in [&mut automatic, &mut native] {
        lane.solver
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.12, 0.05, 0.1],
                    },
                    position: [-0.259, 0., 0.201],
                    mass: Some(7.),
                    velocity: [64., 0., 0.],
                },
                Default::default(),
                None,
            )
            .unwrap();
    }
    let mut later_deck = false;
    for _ in 0..12 {
        automatic.solver.step(STEP).unwrap();
        native.solver.step(STEP).unwrap();
        compare(
            automatic.solver.state(automatic.cargo).unwrap(),
            native.solver.state(native.cargo).unwrap(),
        );
        let a = automatic.solver.rigid_contact_observations();
        let b = native.solver.rigid_contact_observations();
        assert_eq!(
            a.iter().map(|r| (r.a, r.b, r.active)).collect::<Vec<_>>(),
            b.iter().map(|r| (r.a, r.b, r.active)).collect::<Vec<_>>()
        );
        for (a, b) in a.iter().zip(b) {
            assert!((a.normal_impulse - b.normal_impulse).abs() < 1e-8);
        }
        later_deck |= a
            .iter()
            .any(|r| r.a == automatic.decks[1] && r.b == automatic.cargo && r.active);
    }
    assert!(later_deck);
}

#[test]
fn three_axis_motion_and_varying_steps_preserve_leaf_phase_pause_and_reset() {
    let v = [0.25, -0.13, 0.035];
    let mut auto = Lane::with_velocity(true, v);
    let mut native = Lane::with_velocity(false, v);
    let reset = auto.solver.initial_state();
    let mut saw_native = false;
    for tick in 0..150 {
        let dt = Duration::from_micros([5_001, 11_003, 7_997, 10_000][tick % 4]);
        for lane in [&mut auto, &mut native] {
            if tick == 7 {
                lane.solver
                    .add_world_wrench(lane.cargo, [1100., 0., 0.], [0.; 3])
                    .unwrap();
            }
            lane.solver.step(dt).unwrap();
        }
        let a = auto.solver.rigid_contact_observations();
        let n = native.solver.rigid_contact_observations();
        assert_eq!(a.len(), n.len(), "tick {tick}");
        for (a, n) in a.into_iter().zip(n) {
            assert_eq!(
                (a.a, a.b, a.active, a.normal_impulse.to_bits()),
                (n.a, n.b, n.active, n.normal_impulse.to_bits()),
                "tick {tick}"
            );
        }
        for (&id, &control) in auto.decks[..10].iter().zip(&native.decks) {
            if auto.solver.fleet_deck_is_native(id) == Some(true) {
                saw_native = true;
                assert_eq!(
                    auto.solver.state(id).unwrap().position.map(f64::to_bits),
                    native
                        .solver
                        .state(control)
                        .unwrap()
                        .position
                        .map(f64::to_bits),
                    "tick {tick}, {id:?}"
                );
            }
        }
    }
    assert!(saw_native);
    let held = auto.solver.state(auto.cargo).unwrap();
    let stats = auto.solver.fleet_isolation_stats().unwrap();
    auto.solver.step(Duration::ZERO).unwrap();
    assert_eq!(auto.solver.fleet_isolation_stats(), Some(stats));
    assert_eq!(auto.solver.state(auto.cargo), Some(held));
    auto.solver
        .apply_initial(auto.solver.prepare_initial(&reset).unwrap())
        .unwrap();
    assert!(auto.solver.fleet_isolation_stats().is_none());
    assert_eq!(
        auto.solver.state(auto.decks[8]).unwrap().position,
        [10., 0., 0.]
    );
    auto.solver.step(STEP).unwrap();
    assert_eq!(
        auto.solver.fleet_isolation_stats().unwrap().retired_decks,
        0
    );
}
