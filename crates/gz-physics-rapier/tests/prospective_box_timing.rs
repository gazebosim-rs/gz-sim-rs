// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, Inertia, NativeStepGrid, PhysicsError, RotationSpec, Shape,
};
use gz_physics_rapier::{BoxTimingDecline, RapierBackend};
use std::time::Duration;

const DT: Duration = Duration::from_nanos(500_003);
fn body(
    e: &mut RapierBackend,
    position: [f64; 3],
    velocity: [f64; 3],
    half: [f64; 3],
    mass: Option<f64>,
    moments: [f64; 3],
) -> BodyId {
    e.spawn_link(
        BodySpec {
            shape: Shape::Box { half_extents: half },
            position,
            velocity,
            mass,
        },
        RotationSpec::default(),
        mass.map(|_| Inertia {
            center: [0.; 3],
            orientation: [0., 0., 0., 1.],
            tensor: [moments[0], moments[1], moments[2], 0., 0., 0.],
        }),
    )
    .unwrap()
}
fn fixture(z: f64, x: f64, moments: [f64; 3]) -> (RapierBackend, BodyId, BodyId) {
    let mut e = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let f = body(
        &mut e,
        [x, 0., z],
        [0., 0., -1.],
        [0.1, 0.06, 0.05],
        Some(1.),
        moments,
    );
    let p = body(
        &mut e,
        [0., 0., -0.05],
        [0.; 3],
        [1., 1., 0.05],
        None,
        [0.; 3],
    );
    (e, f, p)
}
#[test]
fn exact_nanoseconds_and_full_tick_inputs_with_a_nonuniform_landing() {
    let (mut e, f, p) = fixture(0.05019, 0., [0.1; 3]);
    let witness = body(&mut e, [4., 0., 2.], [0.; 3], [0.1; 3], Some(2.), [0.12; 3]);
    let wheel = body(&mut e, [7., 0., 2.], [0.; 3], [0.1; 3], Some(1.), [0.2; 3]);
    let hinge = e
        .add_world_revolute(wheel, [7., 0., 2.], [0., 1., 0.])
        .unwrap();
    e.apply_impulse(witness, [0.2, 0., 0.], [0.; 3]).unwrap();
    e.add_world_wrench(witness, [3., 0., 0.], [0.; 3]).unwrap();
    e.add_joint_effort(hinge, 0.36).unwrap();
    let pairs = [(f, p)];
    let initial = e.state(witness).unwrap();
    let bad = BodyId::from_raw(u64::MAX);
    assert!(matches!(
        e.step_prospective_box_interval(DT, 8, 4, &pairs, &[(f, bad)]),
        Err(PhysicsError::MissingBody)
    ));
    assert_eq!(e.state(witness), Some(initial));
    assert!(
        e.step_prospective_box_interval(Duration::from_nanos(5), 8, 4, &pairs, &pairs)
            .is_err()
    );
    let (pause, _, clock) = e
        .observe_prospective_box_interval(Duration::ZERO, 8, 4, &pairs, &pairs, |_| {
            panic!("paused callback")
        })
        .unwrap();
    assert_eq!(pause.native_steps, 0);
    assert!(clock.durations.is_empty());
    let mut seen = vec![];
    let (r, _, timing) = e
        .observe_prospective_box_interval(DT, 8, 4, &pairs, &pairs, |s| {
            seen.push((s.elapsed, s.duration))
        })
        .unwrap();
    assert_eq!(timing.decline, None);
    assert!(timing.retimed > 0);
    assert_eq!(r.native_steps, 8);
    assert_eq!(seen.len(), 8);
    assert_eq!(
        timing.durations,
        seen.iter().map(|s| s.1).collect::<Vec<_>>()
    );
    let mut end = Duration::ZERO;
    for (t, h) in seen {
        assert!(!h.is_zero());
        end += h;
        assert_eq!(end, t);
    }
    assert_eq!(end, DT);
    assert!(
        (e.state(witness).unwrap().velocity[0] - (0.2 + 3. * DT.as_secs_f64()) / 2.).abs() < 1e-13
    );
    assert!((e.joint_state(hinge).unwrap().velocity - 0.36 * DT.as_secs_f64() / 0.2).abs() < 1e-11);
    let before = e.state(witness).unwrap().velocity;
    let angular = e.joint_state(hinge).unwrap().velocity;
    e.step(DT).unwrap();
    assert_eq!(before, e.state(witness).unwrap().velocity);
    assert!((e.joint_state(hinge).unwrap().velocity - angular).abs() < 1e-12);
}
#[test]
fn explicit_declines_keep_every_native_call_and_match_the_uniform_endpoint() {
    for (x, moments, forced, multiple, reason) in [
        (
            0.,
            [0.1, 0.2, 0.3],
            false,
            false,
            BoxTimingDecline::AngularTrajectoryUncertain,
        ),
        (
            1.,
            [0.1; 3],
            false,
            false,
            BoxTimingDecline::OutsideSupportFace,
        ),
        (0., [0.1; 3], true, false, BoxTimingDecline::DrivenBody),
        (
            0.,
            [0.1; 3],
            false,
            true,
            BoxTimingDecline::MultipleCandidates,
        ),
    ] {
        let (mut e, f, p) = fixture(0.05019, x, moments);
        let (mut control, fc, pc) = fixture(0.05019, x, moments);
        if reason == BoxTimingDecline::AngularTrajectoryUncertain {
            for (world, id) in [(&mut e, f), (&mut control, fc)] {
                world
                    .set_world_velocity(id, [0., 0., -1.], [75., -80., 45.])
                    .unwrap();
            }
        }
        if forced {
            for (world, id) in [(&mut e, f), (&mut control, fc)] {
                world
                    .add_world_wrench(id, [3., 0., 0.], [0., 0.2, 0.])
                    .unwrap();
            }
        }
        let mut pairs = vec![(f, p)];
        if multiple {
            pairs.push((f, p));
        }
        let (r, _, t) = e
            .step_prospective_box_interval(DT, 8, 4, &[(f, p)], &pairs)
            .unwrap();
        let (c, _) = control
            .step_contact_interval_with_box_features(DT, 8, 4, &[(fc, pc)])
            .unwrap();
        assert_eq!(t.decline, Some(reason));
        assert_eq!(t.decline_after, Some(0));
        assert_eq!(r, c);
        assert_eq!(e.state(f), control.state(fc));
        let grid = NativeStepGrid::new(DT, 8).unwrap();
        assert_eq!(
            t.durations,
            (0..8)
                .map(|i| grid.interval(i).unwrap())
                .collect::<Vec<_>>()
        );
    }
}
#[test]
fn landing_on_a_second_body_falls_back_at_the_first_observed_contact() {
    let (mut e, f, p) = fixture(0.05019, 0., [0.1; 3]);
    let extra = body(
        &mut e,
        [0., 0., -0.05003],
        [0.; 3],
        [0.13, 0.1, 0.05],
        None,
        [0.; 3],
    );
    let (receipt, _, clock) = e
        .step_prospective_box_interval(DT, 8, 4, &[(f, p)], &[(f, p)])
        .unwrap();
    assert!(receipt.pairs.iter().any(|r| r.a == extra || r.b == extra));
    assert_eq!(clock.decline, Some(BoxTimingDecline::OtherContact));
    let at = clock.decline_after.unwrap();
    assert!(at > 0 && at < 8);
    let used: Duration = clock.durations[..at as usize].iter().sum();
    let grid = NativeStepGrid::new(DT - used, 8 - at).unwrap();
    assert_eq!(
        clock.durations[at as usize..],
        (0..8 - at)
            .map(|i| grid.interval(i).unwrap())
            .collect::<Vec<_>>()
    );
}

#[test]
fn gravity_and_late_arrivals_leave_a_short_solve_after_the_predicted_crossing() {
    for t in [0.000100_f64, 0.000460] {
        let mut engine = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let foot = body(
            &mut engine,
            [0., 0., 0.05 + t + 0.5 * 9.81 * t * t],
            [0., 0., -1.],
            [0.1, 0.06, 0.05],
            Some(1.),
            [0.1; 3],
        );
        let pad = body(
            &mut engine,
            [0., 0., -0.05],
            [0.; 3],
            [1., 1., 0.05],
            None,
            [0.; 3],
        );
        let pairs = [(foot, pad)];
        let (r, _, clock) = engine
            .step_prospective_box_interval(DT, 8, 4, &pairs, &pairs)
            .unwrap();
        let positive = r.pairs[0].first_positive_end.unwrap().as_nanos() as i64;
        // Free flight is integrated by the native solver, so gravitational
        // truncation may slightly shift the measured strike after earlier calls.
        let exact = (t * 1e9).round() as i64;
        assert!((positive - exact).abs() < 4500, "{t} {positive} {clock:?}");
        assert!(clock.retimed > 0);
        assert_eq!(clock.decline, None);
    }
}
#[test]
fn free_ankle_and_rotating_or_reversed_geometry_keep_uniform_time() {
    let mut s = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let f = body(
        &mut s,
        [0., 0., 0.05019],
        [0.; 3],
        [0.1; 3],
        Some(1.),
        [0.1; 3],
    );
    let d = body(
        &mut s,
        [0., 0., -0.05],
        [0.; 3],
        [1., 1., 0.05],
        None,
        [0.; 3],
    );
    s.add_reduced_revolute(None, f, [0., 0., 0.05019], [0., 1., 0.])
        .unwrap();
    let (_, _, t) = s
        .step_prospective_box_interval(DT, 8, 4, &[], &[(f, d)])
        .unwrap();
    assert_eq!(t.decline, Some(BoxTimingDecline::NonrigidTree));
    let (mut s, f, d) = fixture(0.05019, 0., [0.1; 3]);
    let (_, _, t) = s
        .step_prospective_box_interval(DT, 8, 4, &[], &[(d, f)])
        .unwrap();
    assert_eq!(t.decline, Some(BoxTimingDecline::UnsupportedGeometry));

    let (mut s, f, _) = fixture(0.05019, 0., [0.1; 3]);
    let p = s
        .spawn_velocity_boundary(
            BodySpec {
                position: [0., 0., -0.05],
                velocity: [0.; 3],
                shape: Shape::Box {
                    half_extents: [1., 1., 0.05],
                },
                mass: Some(1.),
            },
            RotationSpec {
                orientation: [0., 0., 0., 1.],
                angular_velocity: [0., 0.03, 0.],
            },
            None,
        )
        .unwrap();
    let (_, _, t) = s
        .step_prospective_box_interval(DT, 8, 4, &[], &[(f, p)])
        .unwrap();
    assert_eq!(t.decline, Some(BoxTimingDecline::MovingSupportRotation));
}

#[test]
fn empty_explicit_timing_keeps_every_fallback_duration_but_empty_combined_actions_skip_it() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, _, _) = fixture(0.09, 0., [0.12; 3]);
    let (r, _, t) = e.step_prospective_box_interval(DT, 8, 4, &[], &[]).unwrap();
    assert_eq!(r.native_steps, 8);
    assert_eq!(t.decline, Some(BoxTimingDecline::MultipleCandidates));
    assert_eq!(t.decline_after, Some(0));
    assert_eq!(
        t.durations,
        NativeStepGrid::new(DT, 8)
            .unwrap()
            .intervals()
            .collect::<Vec<_>>()
    );
    let r = e
        .advance_box_contacts(DT, 8, 4.into(), BoxContactActions::default(), None)
        .unwrap();
    assert!(r.timing.is_none() && r.flight.is_none());
}
#[test]
fn separated_whole_tick_then_late_contact_restores_the_native_pair() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, f, p) = fixture(0.050527, 0., [0.1; 3]);
    let pairs = [(f, p)];
    let ordinary = e.ordinary_contact_iterations();
    let actions = BoxContactActions {
        guard_separated_flight: &pairs,
        ..Default::default()
    };
    let r = e
        .advance_box_contacts(Duration::from_micros(500), 1, ordinary, actions, None)
        .unwrap();
    assert_eq!(r.flight.unwrap().guarded, [true]);
    assert!(r.interval.pairs.iter().all(|p| p.normal_impulse == 0.));
    assert!((e.state(f).unwrap().velocity[2] + 1.).abs() < 1e-14);
    let actions = BoxContactActions {
        guard_separated_flight: &pairs,
        prospective_timing: &pairs,
        fresh_features: &pairs,
        ..Default::default()
    };
    let r = e
        .advance_box_contacts(DT, 8, 4.into(), actions, None)
        .unwrap();
    let g = r.flight.unwrap();
    assert_eq!(g.durations, r.timing.unwrap().durations);
    assert!(g.guarded.iter().any(|&v| !v));
    assert!(r.interval.pairs.iter().any(|p| p.normal_impulse > 0.7));
    let before = e.state(f);
    let r = e
        .advance_box_contacts(
            Duration::ZERO,
            8,
            4.into(),
            actions,
            Some(&mut |_| panic!("paused callback")),
        )
        .unwrap();
    assert_eq!(before, e.state(f));
    assert!(r.flight.is_none());
}
#[test]
fn collision_filtered_resting_pair_keeps_its_integrated_load_and_native_state() {
    use gz_physics_rapier::{BoxContactActions, ContactSegment};
    fn setup() -> (RapierBackend, BodyId, BodyId, BodyId, BodyId) {
        let mut e = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let sole = body(
            &mut e,
            [0., 0., 0.05031],
            [0., 0., -1.],
            [0.1; 3].map(|v| v * 0.5),
            Some(1.),
            [0.1; 3],
        );
        let pad = body(
            &mut e,
            [0., 0., -0.05],
            [0.; 3],
            [0.3, 0.3, 0.05],
            None,
            [0.; 3],
        );
        let witness = body(
            &mut e,
            [4., 0., 0.05],
            [0.; 3],
            [0.05; 3],
            Some(2.),
            [0.02; 3],
        );
        let base = body(
            &mut e,
            [4., 0., -0.05],
            [0.; 3],
            [0.3, 0.3, 0.05],
            None,
            [0.; 3],
        );
        for b in [sole, witness] {
            e.set_self_collision_group(b, Some(44)).unwrap();
        }
        (e, sole, pad, witness, base)
    }
    let (mut e, f, p, w, b) = setup();
    let (mut control, _, _, wc, bc) = setup();
    let pair = [(f, p)];
    let r = e
        .advance_box_contacts(
            DT,
            8,
            4.into(),
            BoxContactActions {
                prospective_timing: &pair,
                guard_separated_flight: &pair,
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let flight = r.flight.unwrap();
    assert!(flight.guarded.contains(&true));
    let schedule: Vec<_> = flight
        .durations
        .iter()
        .map(|&duration| ContactSegment {
            duration,
            native_steps: 1,
            iterations: 4.into(),
        })
        .collect();
    let (c, _) = control.step_contact_schedule(DT, &schedule, &[]).unwrap();
    let witness = r
        .interval
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (w, b))
        .unwrap();
    let other = c.pairs.iter().find(|p| (p.a, p.b) == (wc, bc)).unwrap();
    assert!(witness.normal_impulse > 0.001);
    assert_eq!(witness, other);
    assert_eq!(e.state(w), control.state(wc));
}
#[test]
fn advancing_competing_shape_cancels_guard_before_it_can_touch() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, f, p) = fixture(0.05086, 0., [0.1; 3]);
    let other = e
        .spawn_velocity_boundary(
            BodySpec {
                position: [0.16, 0., 0.055],
                velocity: [-150., 0., 0.],
                shape: Shape::Box {
                    half_extents: [0.01; 3],
                },
                mass: Some(1.),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let pairs = [(f, p)];
    let r = e
        .advance_box_contacts(
            DT,
            8,
            4.into(),
            BoxContactActions {
                guard_separated_flight: &pairs,
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let g = r.flight.unwrap();
    assert!(g.guarded[0]);
    assert_eq!(g.guarded.last(), Some(&false));
    assert!(
        r.interval
            .pairs
            .iter()
            .any(|x| (x.a == other || x.b == other) && x.normal_impulse > 0.)
    );
}

#[test]
fn cubic_requires_integrated_guard_and_safe_third_body_clearance_before_a_witness() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, f, p) = fixture(0.05019, 0., [0.1, 0.13, 0.16]);
    let pair = [(f, p)];
    let start = e.state(f).unwrap();
    for (timing, guard, rot) in [
        (false, true, true),
        (true, true, false),
        (true, false, true),
    ] {
        let a = BoxContactActions {
            prospective_timing: if timing { &pair } else { &[] },
            guard_separated_flight: if guard { &pair } else { &[] },
            integrate_free_rotation: if rot { &pair } else { &[] },
            refine_rotation_timing: &pair,
            ..Default::default()
        };
        assert!(matches!(
            e.advance_box_contacts(DT, 8, 4.into(), a, None),
            Err(PhysicsError::InvalidValue)
        ));
        assert_eq!(e.state(f), Some(start));
    }
    let other = body(
        &mut e,
        [0., 0., -0.049915],
        [0.; 3],
        [0.3, 0.3, 0.05],
        None,
        [0.; 3],
    );
    let a = BoxContactActions {
        prospective_timing: &pair,
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        refine_rotation_timing: &pair,
        ..Default::default()
    };
    let r = e.advance_box_contacts(DT, 8, 4.into(), a, None).unwrap();
    let t = r.timing.unwrap();
    assert_eq!(t.decline, Some(BoxTimingDecline::OtherContact));
    assert_eq!(t.decline_after, Some(0));
    assert!(t.crossings.is_empty());
    assert_eq!(t.durations.iter().sum::<Duration>(), DT);
    assert_eq!(t.durations.len(), 8);
    assert!(
        r.interval
            .pairs
            .iter()
            .any(|p| (p.a == other || p.b == other) && p.normal_impulse > 0.)
    );
}

#[test]
fn pulse_preflight_declines_invalid_duration_and_duplicate_pair_without_consuming_input() {
    use gz_physics_rapier::{BoxContactActions, RotationContactPulse};
    let (mut e, f, p) = fixture(0.05019, 0., [0.1, 0.13, 0.16]);
    let pair = [(f, p)];
    let witness = body(&mut e, [5., 0., 1.], [0.; 3], [0.1; 3], Some(2.), [0.1; 3]);
    e.add_world_wrench(witness, [8., 0., 0.], [0.; 3]).unwrap();
    let before = e.state(witness);
    for (refined, req) in [
        (false, vec![((f, p), RotationContactPulse::Measured)]),
        (true, vec![((f, p), RotationContactPulse::Fixed(0))]),
        (true, vec![((f, p), RotationContactPulse::Fixed(4_001))]),
        (
            true,
            vec![
                ((f, p), RotationContactPulse::Fixed(64)),
                ((f, p), RotationContactPulse::Measured),
            ],
        ),
    ] {
        let a = BoxContactActions {
            prospective_timing: &pair,
            guard_separated_flight: &pair,
            integrate_free_rotation: &pair,
            refine_rotation_timing: if refined { &pair } else { &[] },
            rotation_contact_pulses: &req,
            ..Default::default()
        };
        assert!(matches!(
            e.advance_box_contacts(DT, 8, 4.into(), a, None),
            Err(PhysicsError::InvalidValue)
        ));
        assert_eq!(e.state(witness), before);
    }
    e.step(DT).unwrap();
    assert!((e.state(witness).unwrap().velocity[0] - 4. * DT.as_secs_f64()).abs() < 1e-14);
}

#[test]
fn compact_budget_preserves_the_complete_schedule_and_inputs_of_another_awake_island() {
    use gz_physics_rapier::{BoxContactActions, CompactDecline, RotationContactPulse};
    fn setup() -> (
        RapierBackend,
        BodyId,
        BodyId,
        BodyId,
        BodyId,
        gz_physics::JointId,
    ) {
        let (mut engine, foot, pad) = fixture(0.05019, 0., [0.1, 0.11, 0.12]);
        let driven = body(
            &mut engine,
            [4., 0., 2.],
            [0.; 3],
            [0.1; 3],
            Some(2.),
            [0.12; 3],
        );
        let wheel = body(
            &mut engine,
            [7., 0., 2.],
            [0.; 3],
            [0.1; 3],
            Some(1.),
            [0.2; 3],
        );
        for id in [foot, driven, wheel] {
            engine.set_self_collision_group(id, Some(9)).unwrap();
        }
        let joint = engine
            .add_world_revolute(wheel, [7., 0., 2.], [0., 1., 0.])
            .unwrap();
        engine
            .add_world_wrench(driven, [3., 0., 0.], [0.; 3])
            .unwrap();
        engine.add_joint_effort(joint, 0.36).unwrap();
        (engine, foot, pad, driven, wheel, joint)
    }
    let mut results = vec![];
    for use_compact in [false, true] {
        let (mut e, f, p, d, w, j) = setup();
        let pairs = [(f, p)];
        let pulses = [((f, p), RotationContactPulse::Measured)];
        let mut out = vec![];
        for k in 0..3 {
            let r = e
                .advance_box_contacts(
                    DT,
                    8,
                    4.into(),
                    BoxContactActions {
                        prospective_timing: &pairs,
                        guard_separated_flight: &pairs,
                        integrate_free_rotation: &pairs,
                        refine_rotation_timing: &pairs,
                        rotation_contact_pulses: &pulses,
                        swept_box_bounds: &pairs,
                        compact_impact_budget: if use_compact { &pairs } else { &[] },
                        ..Default::default()
                    },
                    None,
                )
                .unwrap();
            let timing = r.timing.unwrap();
            assert_eq!(r.interval.native_steps, 8);
            assert_eq!(timing.durations.iter().sum::<Duration>(), DT);
            assert_eq!(
                timing.compact.map(|v| v.decline),
                use_compact.then_some(Some(CompactDecline::AdditionalDynamicBody))
            );
            let v = e.state(d).unwrap().velocity[0];
            assert!(
                (v - 3. * DT.as_secs_f64() / 2.).abs() < 1e-14,
                "step {k} {v}"
            );
            assert!(
                (e.joint_state(j).unwrap().velocity - 0.36 * DT.as_secs_f64() / 0.2).abs() < 1e-11
            );
            out.push((
                timing.durations,
                r.interval,
                e.state(f).unwrap(),
                e.state(w).unwrap(),
                e.state(d).unwrap(),
                e.joint_state(j).unwrap(),
            ));
        }
        results.push(out);
    }
    assert_eq!(results[0], results[1]);
}

#[test]
fn rigid_projection_preflight_rejects_partial_policy_before_pending_force_is_consumed() {
    use gz_physics_rapier::{BoxContactActions, RotationContactPulse};
    let (mut e, f, p) = fixture(0.05019, 0., [0.1, 0.11, 0.12]);
    let driven = body(&mut e, [4., 0., 0.], [0.; 3], [0.1; 3], Some(2.), [0.2; 3]);
    e.add_world_wrench(driven, [5., 0., 0.], [0.; 3]).unwrap();
    let pair = [(f, p)];
    let dup = [(f, p), (f, p)];
    let wrong = [(f, driven)];
    let pulses = [((f, p), RotationContactPulse::Measured)];
    let fixed = [((f, p), RotationContactPulse::Fixed(64))];
    let valid = BoxContactActions {
        prospective_timing: &pair,
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        refine_rotation_timing: &pair,
        rotation_contact_pulses: &pulses,
        swept_box_bounds: &pair,
        project_rigid_corner: &pair,
        ..Default::default()
    };
    let state = e.state(driven);
    for a in [
        BoxContactActions {
            swept_box_bounds: &[],
            ..valid
        },
        BoxContactActions {
            rotation_contact_pulses: &fixed,
            ..valid
        },
        BoxContactActions {
            project_rigid_corner: &dup,
            ..valid
        },
        BoxContactActions {
            project_rigid_corner: &wrong,
            ..valid
        },
    ] {
        assert!(matches!(
            e.advance_box_contacts(DT, 8, 4.into(), a, None),
            Err(PhysicsError::InvalidValue)
        ));
        assert_eq!(e.state(driven), state);
    }
    let outcome = e
        .advance_box_contacts(DT, 8, 4.into(), valid, None)
        .unwrap();
    assert_eq!(outcome.interval.native_steps, 8);
    assert!(outcome.interval.rigid_point_impacts.is_empty());
    assert!((e.state(driven).unwrap().velocity[0] - 2.5 * DT.as_secs_f64()).abs() < 1e-13);
}

#[test]
fn registered_opponent_filters_only_a_certified_face_and_rechecks_moved_boxes() {
    use gz_physics_rapier::BoxContactActions;
    for axis in 0..3 {
        for sign in [-1., 1.] {
            let (mut e, f, p) = fixture(0.6, 0., [0.1; 3]);
            let position = e.state(f).unwrap().position;
            let mut center = position;
            let foot_half = [0.1, 0.06, 0.05];
            center[axis] += sign * (foot_half[axis] + 0.02 + 0.002);
            let gate = body(&mut e, center, [0.; 3], [0.02; 3], None, [0.; 3]);
            let guards = [(f, p)];
            let opposing = [(f, p, gate)];
            let actions = BoxContactActions {
                guard_separated_flight: &guards,
                integrate_free_rotation: &guards,
                opposing_box_guards: &opposing,
                ..Default::default()
            };
            let result = e
                .advance_box_contacts(DT, 1, 4.into(), actions, None)
                .unwrap();
            let flight = result.flight.unwrap();
            assert_eq!(flight.guarded, [true]);
            let cert = flight.opposing.unwrap()[0].unwrap();
            assert_eq!(cert.opponent, gate);
            assert_eq!(cert.face, (2 * axis + usize::from(sign > 0.)) as u8);
            assert!(cert.minimum_gap_pm > 1_000_000_000);
            // A user edit moves the actual collider into the moving foot's path.
            // Both pairs go back to Rapier on the very next native call.
            let mut s = e.state(gate).unwrap();
            s.position = e.state(f).unwrap().position;
            s.position[2] -= 0.0699;
            e.set_body_state(gate, s).unwrap();
            let result = e
                .advance_box_contacts(DT, 1, 4.into(), actions, None)
                .unwrap();
            let flight = result.flight.unwrap();
            assert_eq!(flight.guarded, [false]);
            assert_eq!(flight.opposing.unwrap(), [None]);
            assert!(
                result
                    .interval
                    .pairs
                    .iter()
                    .any(|r| r.a.min(r.b) == f.min(gate)
                        && r.a.max(r.b) == f.max(gate)
                        && r.normal_impulse > 0.)
            );
        }
    }
}
#[test]
fn invalid_opponent_preflight_preserves_pending_public_wrenches() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, f, p) = fixture(1., 0., [0.1; 3]);
    let gate = body(&mut e, [1., 0., 0.], [0.; 3], [0.02; 3], None, [0.; 3]);
    let pairs = [(f, p)];
    let options = BoxContactActions {
        guard_separated_flight: &pairs,
        integrate_free_rotation: &pairs,
        ..Default::default()
    };
    let before = e.state(f).unwrap();
    e.add_world_wrench(f, [3., 0., 0.], [0.; 3]).unwrap();
    let missing = BodyId::from_raw(u64::MAX);
    for bad in [
        vec![(f, p, f)],
        vec![(f, p, p)],
        vec![(p, f, gate)],
        vec![(f, p, gate), (f, p, gate)],
        vec![(f, p, missing)],
    ] {
        assert!(
            e.advance_box_contacts(
                DT,
                1,
                4.into(),
                BoxContactActions {
                    opposing_box_guards: &bad,
                    ..options
                },
                None
            )
            .is_err()
        );
        assert_eq!(e.state(f), Some(before));
    }
    let result = e
        .advance_box_contacts(
            DT,
            1,
            4.into(),
            BoxContactActions {
                opposing_box_guards: &[(f, p, gate)],
                ..options
            },
            None,
        )
        .unwrap();
    assert_eq!(result.flight.unwrap().opposing.unwrap(), [None]);
    assert!((e.state(f).unwrap().velocity[0] - 3. * DT.as_secs_f64()).abs() < 1e-13);
}

#[test]
fn pair_specific_upper_projection_validates_before_consuming_inputs() {
    use gz_physics_rapier::BoxContactActions;
    let (mut e, foot, pad) = fixture(0.4, 0., [0.1; 3]);
    let gate = body(
        &mut e,
        [0., 0., 0.8],
        [0.; 3],
        [1., 1., 0.05],
        None,
        [0.; 3],
    );
    let pair = [(foot, pad)];
    let triple = [(foot, pad, gate)];
    let valid = BoxContactActions {
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        opposing_box_guards: &triple,
        project_opposing_corners: &triple,
        ..Default::default()
    };
    let duplicate = [triple[0], triple[0]];
    let missing = [(foot, pad, BodyId::from_raw(u64::MAX))];
    let invalid = [(foot, pad, pad)];
    let before = e.state(foot).unwrap();
    e.add_world_wrench(foot, [8., 0., 0.], [0.; 3]).unwrap();
    for a in [
        BoxContactActions {
            opposing_box_guards: &[],
            ..valid
        },
        BoxContactActions {
            project_opposing_corners: &duplicate,
            ..valid
        },
        BoxContactActions {
            opposing_box_guards: &invalid,
            project_opposing_corners: &invalid,
            ..valid
        },
        BoxContactActions {
            opposing_box_guards: &missing,
            project_opposing_corners: &missing,
            ..valid
        },
    ] {
        assert!(e.advance_box_contacts(DT, 8, 4.into(), a, None).is_err());
        assert_eq!(e.state(foot), Some(before));
    }
    let out = e
        .advance_box_contacts(DT, 8, 4.into(), valid, None)
        .unwrap();
    assert!(out.interval.opposing_corner_impacts.is_empty());
    assert_eq!(out.interval.native_steps, 8);
    assert!(
        out.flight
            .unwrap()
            .opposing_corner_calls
            .unwrap()
            .iter()
            .all(|guard| !guard)
    );
    assert!((e.state(foot).unwrap().velocity[0] - 8. * DT.as_secs_f64()).abs() < 1e-12);
}

#[path = "private/box_policy.rs"]
mod box_policy;
