// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, NativeStepGrid, PhysicsError, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

fn body(world: &mut MultiFidelityBackend, x: f64) -> BodyId {
    world
        .spawn_link(
            BodySpec {
                mass: Some(2.),
                shape: Shape::Sphere { radius: 0.2 },
                position: [x, 0., 0.],
                velocity: [0.; 3],
            },
            Default::default(),
            None,
        )
        .unwrap()
}
fn world() -> MultiFidelityBackend {
    MultiFidelityBackend::new([0.; 3]).unwrap()
}
fn push(world: &mut MultiFidelityBackend, id: BodyId) {
    world
        .add_world_wrench(id, [8., -3., 1.], [0., 0., 0.012])
        .unwrap();
}

#[test]
fn public_wrench_covers_each_native_interval_and_impulse_occurs_once() {
    let dt = Duration::from_nanos(5_000_001);
    for n in [1, 5, 64] {
        let mut a = world();
        let mut b = world();
        let x = body(&mut a, 0.);
        let y = body(&mut b, 0.);
        for (w, id) in [(&mut a, x), (&mut b, y)] {
            w.apply_impulse(id, [0.8, 0., 0.], [0.; 3]).unwrap();
        }
        push(&mut a, x);
        let before = a.state(x).unwrap();
        assert_eq!(
            a.step_contact_interval(Duration::ZERO, n, 4)
                .unwrap()
                .native_steps,
            0
        );
        assert!(
            a.step_contact_interval(Duration::from_nanos(1), 2, 4)
                .is_err()
        );
        assert!(a.step_contact_interval(dt, 0, 4).is_err());
        assert_eq!(a.state(x), Some(before));
        let receipt = a.step_contact_interval(dt, n, 4).unwrap();
        assert_eq!(receipt.native_steps, n);
        assert!(receipt.pairs.is_empty());
        for h in NativeStepGrid::new(dt, n).unwrap().intervals() {
            push(&mut b, y);
            b.step(h).unwrap();
        }
        assert_eq!(a.state(x), b.state(y));
        assert!((a.state(x).unwrap().velocity[0] - (0.4 + 4. * dt.as_secs_f64())).abs() < 1e-13);
        a.step(dt).unwrap();
        b.step(dt).unwrap();
        assert_eq!(a.state(x), b.state(y));
    }
}

#[test]
fn joint_effort_survives_preflight_and_is_consumed_once() {
    let mut a = world();
    let mut b = world();
    let x = body(&mut a, 0.);
    let y = body(&mut b, 0.);
    let axis = [0., 1., 0.];
    let ja = a.add_reduced_revolute(None, x, [0.; 3], axis).unwrap();
    let jb = b.add_reduced_revolute(None, y, [0.; 3], axis).unwrap();
    let dt = Duration::from_millis(2);
    let torque = 0.08;
    a.add_joint_effort(ja, torque).unwrap();
    assert_eq!(
        a.validate_contact_interval(dt, 8, 0),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(a.pending_joint_effort(ja), Some(torque));
    a.observe_contact_interval(dt, 8, 4, |sample| {
        b.add_joint_effort(jb, torque).unwrap();
        b.step(sample.duration).unwrap();
        let observed = sample.state(x).unwrap();
        let committed = b.state(y).unwrap();
        assert_eq!(observed, committed);
        let expected = torque * sample.elapsed.as_secs_f64() / 0.032;
        assert!((observed.angular_velocity[1] - expected).abs() < 1e-12);
    })
    .unwrap();
    assert_eq!(a.pending_joint_effort(ja), None);
    let actual = a.joint_state(ja).unwrap().velocity;
    assert!((actual - b.joint_state(jb).unwrap().velocity).abs() < 1e-12);
    assert!((actual - torque * dt.as_secs_f64() / 0.032).abs() < 1e-12);
    a.step(dt).unwrap();
    assert!((a.joint_state(ja).unwrap().velocity - actual).abs() < 1e-12);
}

#[test]
fn pgs_work_is_independent_of_timestep_and_retained_by_direct_reset() {
    let mut a = world();
    assert_eq!(a.contact_pgs_passes(), 1);
    a.set_contact_pgs_passes(7).unwrap();
    for invalid in [0, 33] {
        assert!(a.set_contact_pgs_passes(invalid).is_err());
        assert_eq!(a.contact_pgs_passes(), 7);
    }
    let _ = body(&mut a, 0.);
    a.step_contact_interval(Duration::from_millis(1), 8, 8)
        .unwrap();
    assert_eq!(a.contact_pgs_passes(), 7);
    a.reset().unwrap();
    assert_eq!(a.contact_pgs_passes(), 7);
}

#[test]
fn grazing_pair_receipt_survives_separation_and_retains_first_native_clock() {
    let make = || {
        let mut w = world();
        let ball = w
            .spawn_link(
                BodySpec {
                    mass: Some(2.0),
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [-0.5, 0.36, 0.],
                    velocity: [1.0, 0., 0.],
                },
                Default::default(),
                None,
            )
            .unwrap();
        let obstacle = w
            .spawn_link(
                BodySpec {
                    mass: None,
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.; 3],
                    velocity: [0.; 3],
                },
                Default::default(),
                None,
            )
            .unwrap();
        (w, ball, obstacle)
    };
    let (mut w, ball, obstacle) = make();
    let (mut reference, other, _) = make();
    let dt = Duration::from_nanos(1_000_000_037);
    let calls = 600;
    let mut elapsed = Duration::ZERO;
    let mut first = None;
    let mut total = 0.;
    let mut substeps = 0;
    let grid = NativeStepGrid::new(dt, calls).unwrap();
    let receipt = w
        .observe_contact_interval(dt, calls, 4, |s| {
            assert_eq!(grid.interval(substeps), Some(s.duration));
            substeps += 1;
            elapsed += s.duration;
            assert_eq!(s.elapsed, elapsed);
            reference.step(s.duration).unwrap();
            assert_eq!(s.state(ball), reference.state(other));
            for p in reference.rigid_contact_observations() {
                if p.normal_impulse > 0. {
                    first.get_or_insert(elapsed);
                }
                total += p.normal_impulse;
            }
        })
        .unwrap();
    assert_eq!(receipt.native_steps, calls);
    assert_eq!(elapsed, dt);
    assert_eq!(receipt.pairs.len(), 1);
    let r = receipt.pairs[0];
    assert_eq!((r.a, r.b), (ball, obstacle));
    assert_eq!(r.first_positive_end, first);
    assert!(
        first.is_some_and(|t| t > Duration::from_millis(250) && t < Duration::from_millis(400))
    );
    assert!(total > 0.1);
    assert_eq!(r.normal_impulse, total);
    assert!(!r.active_at_end);
    assert!(
        reference
            .rigid_contact_observations()
            .iter()
            .all(|p| !p.active)
    );
}

#[test]
fn scoped_solver_work_is_validated_before_consuming_commands() {
    use gz_physics_multifidelity::ContactSolverIterations;
    let mut w = world();
    let id = body(&mut w, 0.);
    push(&mut w, id);
    let at = w.state(id);
    for (temporal, pgs, relaxation) in [
        (0, None, None),
        (4, Some(0), Some(8)),
        (4, Some(3), Some(33)),
    ] {
        let budget = ContactSolverIterations {
            temporal,
            pgs,
            relaxation,
        };
        for dt in [Duration::ZERO, Duration::from_millis(1)] {
            assert!(w.step_contact_interval(dt, 4, budget).is_err());
            assert_eq!(w.state(id), at);
        }
    }
    let dt = Duration::from_millis(1);
    w.set_contact_pgs_passes(2).unwrap();
    assert!(
        w.observe_contact_interval(Duration::ZERO, 4, 4, |_| panic!("paused solve"))
            .unwrap()
            .pairs
            .is_empty()
    );
    w.step_contact_interval(
        dt,
        8,
        ContactSolverIterations {
            temporal: 6,
            pgs: Some(3),
            relaxation: Some(16),
        },
    )
    .unwrap();
    assert_eq!(w.contact_pgs_passes(), 2);
    assert!((w.state(id).unwrap().velocity[0] - 0.004).abs() < 1e-14);
}

#[test]
fn scheduled_substeps_hold_inputs_and_inherit_each_unset_budget_from_world() {
    use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations};
    let mut a = world();
    let mut b = world();
    let x = body(&mut a, 0.);
    let y = body(&mut b, 0.);
    push(&mut a, x);
    a.set_contact_pgs_passes(3).unwrap();
    b.set_contact_pgs_passes(3).unwrap();
    let spans = [
        ContactSegment {
            duration: Duration::from_nanos(300_003),
            native_steps: 2,
            iterations: 2.into(),
        },
        ContactSegment {
            duration: Duration::from_nanos(123_007),
            native_steps: 5,
            iterations: ContactSolverIterations {
                temporal: 8,
                pgs: Some(9),
                relaxation: Some(16),
            },
        },
        ContactSegment {
            duration: Duration::from_nanos(76_991),
            native_steps: 1,
            iterations: 4.into(),
        },
    ];
    let dt = spans.iter().map(|s| s.duration).sum();
    let mut expected = vec![];
    let mut elapsed = Duration::ZERO;
    for s in &spans {
        for h in NativeStepGrid::new(s.duration, s.native_steps)
            .unwrap()
            .intervals()
        {
            push(&mut b, y);
            b.step_contact_interval(h, 1, s.iterations).unwrap();
            elapsed += h;
            expected.push((elapsed, h, b.state(y).unwrap()));
        }
    }
    let before = a.state(x);
    for invalid in [
        vec![],
        spans[..2].to_vec(),
        vec![
            spans[0],
            ContactSegment {
                native_steps: u32::MAX,
                ..spans[1]
            },
            spans[2],
        ],
        vec![
            spans[0],
            spans[1],
            ContactSegment {
                iterations: 0.into(),
                ..spans[2]
            },
        ],
    ] {
        assert!(
            a.observe_contact_schedule(dt, &invalid, &[], |_| panic!("invalid schedule advanced"))
                .is_err()
        );
    }
    assert!(a.step_contact_schedule(dt, &spans, &[(x, x)]).is_err());
    assert_eq!(a.state(x), before);
    assert_eq!(
        a.step_contact_schedule(Duration::ZERO, &[], &[])
            .unwrap()
            .0
            .native_steps,
        0
    );
    let mut actual = vec![];
    let (receipt, rebuilt) = a
        .observe_contact_schedule(dt, &spans, &[], |s| {
            actual.push((s.elapsed, s.duration, s.state(x).unwrap()))
        })
        .unwrap();
    assert_eq!(actual, expected);
    assert_eq!(actual.last().unwrap().0, dt);
    assert_eq!((receipt.native_steps, rebuilt), (8, 0));
    assert_eq!(a.contact_pgs_passes(), 3);
    a.step(dt).unwrap();
    b.step(dt).unwrap();
    assert_eq!(a.state(x), b.state(y));
}

#[test]
fn unequal_contact_segments_match_explicit_scoped_solves_at_every_boundary() {
    use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations};
    let make = || {
        let mut n = world();
        let a = n
            .spawn_link(
                BodySpec {
                    mass: Some(1.),
                    shape: Shape::Box {
                        half_extents: [0.15, 0.12, 0.1],
                    },
                    position: [0., 0., 0.10014],
                    velocity: [0.4, 0., -1.],
                },
                Default::default(),
                None,
            )
            .unwrap();
        let b = n
            .spawn_link(
                BodySpec {
                    mass: None,
                    shape: Shape::Box {
                        half_extents: [0.2, 0.2, 0.1],
                    },
                    position: [0.11, 0., -0.1],
                    velocity: [0.; 3],
                },
                Default::default(),
                None,
            )
            .unwrap();
        (n, a, b)
    };
    let (mut w, a, b) = make();
    let (mut other, c, d) = make();
    let make_span = |ns, calls, relax| ContactSegment {
        duration: Duration::from_nanos(ns),
        native_steps: calls,
        iterations: ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: relax,
        },
    };
    let segments = [
        make_span(120_003, 1, None),
        make_span(100_002, 4, Some(24)),
        make_span(279_995, 2, None),
    ];
    let mut first = None;
    let mut normal = 0.;
    let mut elapsed = Duration::ZERO;
    let mut expected = vec![];
    let mut queries = 0;
    for seg in segments {
        for h in NativeStepGrid::new(seg.duration, seg.native_steps)
            .unwrap()
            .intervals()
        {
            let (r, g) = other
                .step_contact_interval_with_box_features(h, 1, seg.iterations, &[(c, d)])
                .unwrap();
            queries += g;
            elapsed += h;
            if let Some(pair) = r.pairs.first() {
                if pair.normal_impulse > 0. {
                    first.get_or_insert(elapsed);
                }
                normal += pair.normal_impulse;
            }
            expected.push(other.state(c).unwrap());
        }
    }
    let mut actual = vec![];
    let (r, n) = w
        .observe_contact_schedule(elapsed, &segments, &[(a, b)], |s| {
            actual.push(s.state(a).unwrap())
        })
        .unwrap();
    assert_eq!(expected, actual);
    assert_eq!(n, queries);
    assert!(n > 0 && normal > 0.);
    assert_eq!(r.pairs[0].first_positive_end, first);
    assert_eq!(r.pairs[0].normal_impulse, normal);
}

#[test]
fn prescribed_spinning_link_observations_use_the_endpoint_tangent() {
    use gz_physics::RotationSpec;
    use gz_physics_multifidelity::ArticulationTier;
    let dt = Duration::from_millis(8);
    let make = || {
        let mut w = world();
        let ids = [-0.25, 0.25].map(|x| {
            w.spawn_articulation_link(
                BodySpec {
                    mass: Some(1.),
                    shape: Shape::Sphere { radius: 0.1 },
                    position: [x, 0., 0.],
                    velocity: [0., x * 3., 0.],
                },
                RotationSpec {
                    angular_velocity: [0., 0., 3.],
                    ..Default::default()
                },
                None,
            )
            .unwrap()
        });
        w.add_reduced_revolute(Some(ids[0]), ids[1], [0.; 3], [0., 0., 1.])
            .unwrap();
        w.switch_articulation(ids[0], ArticulationTier::Prescribed, dt)
            .unwrap();
        (w, ids)
    };
    let (mut w, ids) = make();
    let (mut separate, peers) = make();
    w.observe_contact_interval(dt, 8, 4, |sample| {
        separate.step(sample.duration).unwrap();
        for (id, peer) in ids.into_iter().zip(peers) {
            let s = sample.state(id).unwrap();
            let independent = separate.state(peer).unwrap();
            for (a, b) in s.velocity.into_iter().zip(independent.velocity) {
                assert!((a - b).abs() < 1e-12, "{s:?} {independent:?}");
            }
            assert!((s.velocity[0] + 3. * s.position[1]).abs() < 1e-11);
            assert!((s.velocity[1] - 3. * s.position[0]).abs() < 1e-11);
        }
    })
    .unwrap();
}
