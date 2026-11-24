// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_rapier::{BoxContactActions, RapierBackend};
use rapier3d_f64::math::{Matrix as DMat3, Rotation as DQuat, Vector as DVec3};
use std::time::Duration;
const DT: Duration = Duration::from_micros(500);
const COM: DVec3 = DVec3::new(0.034, -0.025, 0.07);
fn fixture(axial: f64) -> (RapierBackend, BodyId, BodyId, DQuat, DQuat, DVec3) {
    let mut e = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let q = DQuat::from_scaled_axis(DVec3::new(0.07, -0.11, 0.25));
    let frame = DQuat::from_scaled_axis(DVec3::new(-0.3, 0.08, 0.21));
    let spin = DVec3::new(14., -32., 18.);
    let f = e
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.12, 0.04, 0.05],
                },
                position: [1., 2., 20.],
                velocity: [0.14, -0.21, 0.63],
                mass: Some(2.),
            },
            RotationSpec {
                orientation: q.to_array(),
                angular_velocity: (q * frame * spin).to_array(),
            },
            Some(Inertia {
                center: COM.to_array(),
                orientation: frame.to_array(),
                tensor: [0.041, 0.041, axial, 0., 0., 0.],
            }),
        )
        .unwrap();
    let p = e
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [50., 50., 0.05],
            },
            position: [0., 0., -0.05],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    (e, f, p, q, frame, spin)
}
#[test]
fn eligible_native_steps_conserve_a_rotated_symmetric_tops_momentum_energy_and_phase() {
    for axial in [0.012, 0.072] {
        let (mut e, f, p, q0, principal, spin) = fixture(axial);
        let (mut control, cf, _, _, _, _) = fixture(axial);
        let pair = [(f, p)];
        let actions = BoxContactActions {
            guard_separated_flight: &pair,
            integrate_free_rotation: &pair,
            ..Default::default()
        };
        let l = q0 * principal * (DVec3::new(0.041, 0.041, axial) * spin);
        let tensor = DMat3::from_quat(principal)
            * DMat3::from_diagonal(DVec3::new(0.041, 0.041, axial))
            * DMat3::from_quat(principal).transpose();
        let e0 = 0.5 * spin.dot(DVec3::new(0.041, 0.041, axial) * spin);
        let mut advanced = 0;
        {
            let (mut probe, id, _, _, _, _) = fixture(axial);
            let iw = DMat3::from_quat(q0) * tensor * DMat3::from_quat(q0).transpose();
            let saved = probe.state(id).unwrap();
            for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
                let w = DVec3::from(probe.state(id).unwrap().angular_velocity);
                probe.apply_impulse(id, [0.; 3], axis.to_array()).unwrap();
                let delta = DVec3::from(probe.state(id).unwrap().angular_velocity) - w;
                assert!((iw.inverse() * axis - delta).length() < 1e-10);
            }
            assert_eq!(saved.orientation, q0.to_array());
        }
        // The foot rotates out of a top-face orientation several times during one
        // second. Re-orient the stationary distant pad to its current local Z
        // between intervals, keeping the certified top-plane domain throughout.
        for i in 1..=2000 {
            let state = e.state(f).unwrap();
            let current = DQuat::from_array(state.orientation);
            let mut deck = e.state(p).unwrap();
            deck.orientation = current.to_array();
            let center = DVec3::from(state.position);
            deck.position = (center - current * DVec3::Z * 3.).to_array();
            e.set_body_state(p, deck).unwrap();
            let receipt = e
                .advance_box_contacts(DT, 1, 4.into(), actions, None)
                .unwrap();
            let slices = receipt.flight.unwrap().rotation_slices.unwrap();
            assert!(slices[0] >= 2);
            advanced += 1;
            control.step_contact_interval(DT, 1, 4).unwrap();
            let state = e.state(f).unwrap();
            let ordinary = control.state(cf).unwrap();
            let q = DQuat::from_array(state.orientation);
            let w = DVec3::from(state.angular_velocity);
            let t = i as f64 * DT.as_secs_f64();
            let exact = DQuat::from_scaled_axis(l * (t / 0.041))
                * q0
                * principal
                * DQuat::from_rotation_z((1. - axial / 0.041) * spin.z * t)
                * principal.conjugate();
            let dq = q * exact.conjugate();
            let angle = 2. * dq.xyz().length().atan2(dq.w.abs());
            assert!(angle < 2e-7, "phase {angle} at {t}s");
            let lm = q * (tensor * (q.conjugate() * w));
            assert!(lm.distance(l) < 1e-10);
            assert!((0.5 * w.dot(lm) - e0).abs() < 2e-9);
            let com = DVec3::from(state.position) + q * COM;
            let other =
                DVec3::from(ordinary.position) + DQuat::from_array(ordinary.orientation) * COM;
            assert!(com.distance(other) < 1e-10);
            assert_eq!(state.velocity, ordinary.velocity);
        }
        assert_eq!(advanced, 2000);
    }
}
#[test]
fn unguarded_or_forced_body_keeps_native_state_and_consumes_pending_input_once() {
    let (mut e, f, p, _, _, _) = fixture(0.018);
    let (mut control, cf, cp, _, _, _) = fixture(0.018);
    let pair = [(f, p)];
    e.add_world_wrench(f, [1., 0., 0.], [0.04, 0., 0.]).unwrap();
    control
        .add_world_wrench(cf, [1., 0., 0.], [0.04, 0., 0.])
        .unwrap();
    let before = e.state(f);
    assert!(matches!(
        e.advance_box_contacts(
            DT,
            4,
            4.into(),
            BoxContactActions {
                integrate_free_rotation: &pair,
                ..Default::default()
            },
            None
        ),
        Err(PhysicsError::InvalidValue)
    ));
    assert_eq!(before, e.state(f));
    let r = e
        .advance_box_contacts(
            DT,
            4,
            4.into(),
            BoxContactActions {
                integrate_free_rotation: &pair,
                guard_separated_flight: &pair,
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let c = control
        .advance_box_contacts(
            DT,
            4,
            4.into(),
            BoxContactActions {
                guard_separated_flight: &[(cf, cp)],
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let g = r.flight.unwrap();
    assert_eq!(g.guarded, vec![false; 4]);
    assert_eq!(g.rotation_slices, Some(vec![0; 4]));
    assert!(c.flight.unwrap().rotation_slices.is_none());
    assert_eq!(e.state(f), control.state(cf));
    e.step(DT).unwrap();
    control.step(DT).unwrap();
    assert_eq!(e.state(f), control.state(cf));
}

#[test]
fn apparently_separated_endpoints_with_an_interior_rotating_strike_decline_release() {
    // A y-axis principal spin makes the support height exactly sinusoidal.
    // Both endpoints remain above the deck, but the lowest edge crosses it in between.
    let r = 0.1_f64.hypot(0.05);
    let mid = 0.1_f64.atan2(0.05);
    for (offset, accept) in [(-5e-6, false), (8e-6, true)] {
        let mut e = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let f = e
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.1, 0.06, 0.05],
                    },
                    position: [0., 0., r + offset],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                RotationSpec {
                    orientation: DQuat::from_rotation_y(mid - 0.02).to_array(),
                    angular_velocity: [0., 80., 0.],
                },
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.11, 0.13, 0.11, 0., 0., 0.],
                }),
            )
            .unwrap();
        let p = e
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [1., 1., 0.05],
                    },
                    position: [0., 0., -0.05],
                    velocity: [0.; 3],
                    mass: None,
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        assert!(r * (1. - 0.02_f64.cos()) + offset > 0.);
        let pair = [(f, p)];
        let a = BoxContactActions {
            guard_separated_flight: &pair,
            integrate_free_rotation: &pair,
            integrate_released_rotation: &pair,
            ..Default::default()
        };
        let g = e
            .advance_box_contacts(DT, 1, 4.into(), a, None)
            .unwrap()
            .flight
            .unwrap();
        assert_eq!(g.guarded, [accept]);
        let s = g.released.unwrap()[0];
        assert_eq!(s.integrated, accept);
        if accept {
            assert!(s.minimum_gap_pm.unwrap() as f64 <= offset * 1e12 + 1.);
            assert!(s.corner_spans >= 4);
        } else {
            assert!(s.minimum_gap_pm.is_none());
        }
    }
}

#[test]
fn released_rotation_preflights_qualified_pairs_and_preserves_torque_and_third_body_contact() {
    let (mut e, f, p, _, _, _) = fixture(0.07);
    let pair = [(f, p)];
    let initial = e.state(f);
    let invalid = BoxContactActions {
        integrate_released_rotation: &pair,
        ..Default::default()
    };
    assert!(matches!(
        e.advance_box_contacts(DT, 1, 4.into(), invalid, None),
        Err(PhysicsError::InvalidValue)
    ));
    assert_eq!(e.state(f), initial);
    let a = BoxContactActions {
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        integrate_released_rotation: &pair,
        ..Default::default()
    };
    e.add_world_wrench(f, [0.; 3], [0.3, 0.2, -0.8]).unwrap();
    let r = e.advance_box_contacts(DT, 1, 4.into(), a, None).unwrap();
    assert_eq!(r.flight.unwrap().guarded, [false]);
    let state = e.state(f).unwrap();
    let third = e
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.12, 0.04, 0.05],
                },
                position: state.position,
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let r = e.advance_box_contacts(DT, 1, 4.into(), a, None).unwrap();
    assert_eq!(r.flight.unwrap().guarded, [false]);
    assert!(
        r.interval
            .pairs
            .iter()
            .any(|p| (p.a == third || p.b == third) && p.normal_impulse > 0.)
    );
}

#[test]
fn public_sweep_nominates_the_rotating_interior_strike_and_excludes_a_near_miss() {
    use gz_physics_rapier::RigidBoxSweep;
    let r = 0.1_f64.hypot(0.05);
    let peak = 0.1_f64.atan2(0.05);
    for (delta, possible) in [(-5e-6, true), (8e-6, false)] {
        let mut e = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let f = e
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.1, 0.06, 0.05],
                    },
                    position: [0., 0., r + delta],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                RotationSpec {
                    orientation: DQuat::from_rotation_y(peak - 0.02).to_array(),
                    angular_velocity: [0., 80., 0.],
                },
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.11, 0.13, 0.11, 0., 0., 0.],
                }),
            )
            .unwrap();
        let p = e
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [1., 1., 0.05],
                    },
                    position: [0., 0., -0.05],
                    velocity: [0.; 3],
                    mass: None,
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let state = e.state(f);
        let decision = e.rigid_box_sweep(f, p, DT).unwrap();
        assert_eq!(
            decision,
            if possible {
                RigidBoxSweep::Possible { vertices: 12 }
            } else {
                RigidBoxSweep::Clear
            }
        );
        assert_eq!(e.state(f), state);
        assert!(matches!(
            e.rigid_box_sweep(f, p, Duration::ZERO),
            Err(PhysicsError::InvalidValue)
        ));
        e.set_self_collision_group(f, Some(17)).unwrap();
        e.set_self_collision_group(p, Some(17)).unwrap();
        assert!(matches!(
            e.rigid_box_sweep(f, p, DT),
            Ok(RigidBoxSweep::Unavailable(_))
        ));
    }
}

#[test]
fn certified_gravity_moves_the_inertial_center_exactly_across_different_native_grids() {
    let gravity = DVec3::new(0., 0., -9.81);
    let dt = Duration::from_nanos(500003);
    for count in [1, 3, 8, 16] {
        let (mut e, f, p, _, _, _) = fixture(0.018);
        let start = e.state(f).unwrap();
        let origin = DVec3::from(start.position) + DQuat::from_array(start.orientation) * COM;
        let initial_v = DVec3::from(start.velocity);
        let pair = [(f, p)];
        for tick in 1..=30 {
            let r = e
                .advance_box_contacts(
                    dt,
                    count,
                    4.into(),
                    BoxContactActions {
                        guard_separated_flight: &pair,
                        integrate_free_rotation: &pair,
                        integrate_ballistic_translation: &pair,
                        ..Default::default()
                    },
                    None,
                )
                .unwrap();
            let flight = r.flight.unwrap();
            assert_eq!(flight.ballistic_steps, Some(vec![true; count as usize]));
            let t = dt.as_secs_f64() * f64::from(tick);
            let s = e.state(f).unwrap();
            let center = DVec3::from(s.position) + DQuat::from_array(s.orientation) * COM;
            assert!(center.distance(origin + (initial_v + 0.5 * t * gravity) * t) < 2e-12);
            assert!(DVec3::from(s.velocity).distance(initial_v + t * gravity) < 3e-14);
            assert!(r.interval.pairs.iter().all(|p| p.normal_impulse == 0.));
        }
    }
}

#[test]
fn ballistic_preflight_and_queued_external_wrench_preserve_native_motion() {
    let (mut e, f, p, _, _, _) = fixture(0.018);
    let (mut control, cf, cp, _, _, _) = fixture(0.018);
    for (b, id) in [(&mut e, f), (&mut control, cf)] {
        b.add_world_wrench(id, [2., 0., 3.], [0.1, -0.3, 0.])
            .unwrap();
    }
    let s = e.state(f);
    let pair = [(f, p)];
    let bad = e.advance_box_contacts(
        DT,
        4,
        4.into(),
        BoxContactActions {
            integrate_ballistic_translation: &pair,
            ..Default::default()
        },
        None,
    );
    assert!(matches!(bad, Err(PhysicsError::InvalidValue)));
    assert_eq!(e.state(f), s);
    for (native, body, pad, on) in [(&mut e, f, p, true), (&mut control, cf, cp, false)] {
        let named = [(body, pad)];
        let r = native
            .advance_box_contacts(
                DT,
                4,
                4.into(),
                BoxContactActions {
                    guard_separated_flight: &named,
                    integrate_free_rotation: &named,
                    integrate_ballistic_translation: if on { &named } else { &[] },
                    ..Default::default()
                },
                None,
            )
            .unwrap();
        assert_eq!(
            r.flight.unwrap().ballistic_steps,
            on.then_some(vec![false; 4])
        );
    }
    assert_eq!(e.state(f), control.state(cf));
}

#[test]
fn adjacent_third_body_receives_its_full_native_contact_during_ballistic_enrollment() {
    let mut matches = vec![];
    for on in [false, true] {
        let (mut e, f, p, _, _, _) = fixture(0.018);
        let blocker = e
            .spawn(BodySpec {
                shape: Shape::Box {
                    half_extents: [0.2, 0.2, 0.05],
                },
                position: [1., 2., 20.05],
                velocity: [0.; 3],
                mass: None,
            })
            .unwrap();
        let named = [(f, p)];
        let r = e
            .advance_box_contacts(
                DT,
                8,
                4.into(),
                BoxContactActions {
                    guard_separated_flight: &named,
                    integrate_free_rotation: &named,
                    integrate_ballistic_translation: if on { &named } else { &[] },
                    ..Default::default()
                },
                None,
            )
            .unwrap();
        let flight = r.flight.unwrap();
        assert_eq!(flight.guarded, vec![false; 8]);
        assert_eq!(flight.ballistic_steps, on.then_some(vec![false; 8]));
        assert!(
            r.interval
                .pairs
                .iter()
                .any(|pair| (pair.a == blocker || pair.b == blocker) && pair.normal_impulse > 0.)
        );
        matches.push((r.interval, e.state(f).unwrap()));
    }
    assert_eq!(matches[0], matches[1]);
}
