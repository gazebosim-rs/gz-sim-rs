// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn pair() -> SpherePairLane {
    let EnvironmentState::SpherePair(mut p) = crate::mixed::tests::initial()[1].state else {
        unreachable!()
    };
    p.b.position[1] = 0.03;
    p.a.angular_velocity = [0.2, 0.8, 1.1];
    p
}
fn env(p: SpherePairLane) -> EnvironmentState {
    EnvironmentState::SpherePair(p)
}
#[test]
fn all_persistent_native_state_and_contact_cache_match_continuous_run() {
    let mut whole = NativePair::new(1, env(pair())).unwrap();
    let mut staged = whole.fork();
    let mut a = env(pair());
    let mut b = a;
    let mut impact = false;

    for tick in 0..48 {
        if tick == 7 {
            let EnvironmentState::SpherePair(mut v) = a else {
                unreachable!()
            };
            v.a.velocity[0] += 0.1;
            a = env(v);
            b = a;
        }
        a = whole.advance_in_place(1, a).unwrap();
        let step = staged.advance(1, b).unwrap();
        staged = step.0;
        b = step.1;
        assert_eq!(a, b);
        assert_eq!(whole.contact, staged.contact);
        impact |= whole.contact.first_impulse_interval.is_some();
    }
    assert!(impact);
}
#[test]
fn transfer_checks_pending_swept_clearance_both_spins_and_observed_history() {
    let ok = env(pair());
    guard(1, ok, ok, None).unwrap();
    let mut reach = pair();
    reach.a.velocity[0] = 100.;
    assert!(matches!(
        guard(1, ok, env(reach), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::SweptClearance,
            ..
        })
    ));
    let mut fast = pair();
    fast.b.angular_velocity[2] = 1e6;
    assert!(motion_envelope(2, env(fast)).is_err());
    let mut recorded = pair();
    recorded.normal_impulse = 0.01;
    assert!(matches!(
        guard(1, env(recorded), env(recorded), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::ContactHistory,
            ..
        })
    ));
    let mut w = NativePair::new(1, env(pair())).unwrap();
    w.contact.retained_points = 1;
    assert!(guard(1, ok, ok, Some(&w)).is_err());
}
#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn ready_native_mutations_and_pending_input_rollback_after_a_real_gpu_failure() {
    let device = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let create = || {
        let mut seeds = crate::mixed::tests::initial();
        seeds[1].state = env(closing_pair());
        MixedBatch::from_fixed_scene(&device, seeds)
            .unwrap()
            .with_guarded_native_pairs(&[(0, PairSolver::NativeRapier)])
            .unwrap()
    };
    let mut b = create();
    let mut control = create();
    for _ in 0..10 {
        b.step().unwrap();
        control.step().unwrap();
        let witness = b.native_contact(0).unwrap().unwrap();
        if witness.retained_points > 0 && witness.active_solver_contact {
            break;
        }
    }
    let at = b.tick();
    let witness = b.native_contact(0).unwrap().unwrap();
    assert!(witness.retained_points > 0 && witness.active_solver_contact);
    assert!(witness.impulse_on_b[0] > 0.1);
    let action = CentralImpulse {
        id: 0,
        body: BodySlot::A,
        impulse: [0.25, 0.1, 0.],
    };
    for a in [&mut b, &mut control] {
        a.queue_impulses(at, &[action]).unwrap();
    }
    let original = b.environments().to_vec();
    let cache = b.native_contact(0).unwrap();
    let offset = b.active_offsets()[b.index(9).unwrap()] as u64;
    b.gpu
        .queue
        .write_buffer(&b.gpu.state, offset, &f64::NAN.to_le_bytes());
    assert!(matches!(
        b.step(),
        Err(GpuError::NonfiniteEnvironment { id: 9, .. })
    ));
    assert_eq!(b.tick(), at);
    assert_eq!(b.environments(), original);
    assert_eq!(b.native_contact(0).unwrap(), cache);
    assert_eq!(b.pending_impulses().next(), Some(&action));
    for _ in 0..40 {
        b.step().unwrap();
        control.step().unwrap();
        assert_eq!(b.environments(), control.environments());
        assert_eq!(
            b.native_contact(0).unwrap(),
            control.native_contact(0).unwrap()
        );
    }
    b.reset();
    assert_eq!(b.pair_solver(0).unwrap(), PairSolver::NativeRapier);
    assert_eq!(b.native_contact(0).unwrap(), Some(NativeContact::default()));
    assert_eq!(b.pending_impulses().len(), 0);
}

fn closing_pair() -> SpherePairLane {
    let mut p = pair();
    p.a.velocity = [0.5, 0., 0.];
    p.b.velocity = [-0.5, 0., 0.];
    p.a.angular_velocity = [0.; 3];
    p.b.angular_velocity = [0.; 3];
    p.restitution = 0.;
    p.a.position = [0.; 3];
    p.b.position = [p.a.radius + p.b.radius + 0.003937, 0., 0.];
    p
}

#[test]
fn actual_native_manifold_remains_with_the_solver_despite_an_otherwise_clear_public_record() {
    let p = closing_pair();
    let mut w = NativePair::new(1, env(p)).unwrap();
    let mut state = env(p);
    for _ in 0..10 {
        state = w.advance_in_place(1, state).unwrap();
        if w.contact.retained_points > 0 && w.contact.active_solver_contact {
            assert!(w.contact.first_impulse_interval.is_some());
            assert!(w.contact.impulse_on_b[0] > 0.1);
            let raw = w
                .world
                .narrow_phase
                .contact_pair(w.colliders[0], w.colliders[1])
                .unwrap();
            let mut points = raw.manifolds().iter().flat_map(|manifold| &manifold.points);
            let point = points.next().unwrap();
            assert_eq!(
                w.contact.warmstart_normal_impulse,
                point.data.warmstart_impulse
            );
            assert!(points.next().is_none());
            let clear = env(pair());
            assert!(matches!(
                guard(1, clear, clear, Some(&w)),
                Err(GpuError::NativeHandoff {
                    reason: NativeHandoffRefusal::ContactHistory,
                    ..
                })
            ));
            return;
        }
    }
    panic!("closing pair never registered an active native manifold at a committed boundary");
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn supported_native_friction_cache_and_a_pending_liftoff_survive_real_sibling_failure() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let create = || {
        MixedBatch::from_fixed_scene(&adapter, crate::mixed::tests::initial())
            .unwrap()
            .with_guarded_native_pairs(&[(9, PairSolver::NativeRapier)])
            .unwrap()
    };
    let mut b = create();
    let mut control = create();
    for _ in 0..52 {
        b.step().unwrap();
        control.step().unwrap();
    }
    let at = b.tick();
    let observed = b.native_contact(9).unwrap().unwrap();
    assert!(observed.active_solver_contact && observed.retained_points > 0);
    assert!(
        observed.warmstart_normal_impulse > 1e-6 && observed.warmstart_tangent_impulse > 1e-7,
        "expected measured persistent normal and tangent load {observed:?}"
    );
    let action = CentralImpulse {
        id: 9,
        body: BodySlot::A,
        impulse: [0.09, 0.03, 0.7],
    };
    for a in [&mut b, &mut control] {
        a.queue_impulses(at, &[action]).unwrap();
    }
    let original = b.environments().to_vec();
    assert!(matches!(
        b.set_pair_solver(at, &[9], PairSolver::SpecializedGpu),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::ContactHistory,
            ..
        })
    ));
    let offset = b.active_offsets()[b.index(0).unwrap()] as u64;
    b.gpu
        .queue
        .write_buffer(&b.gpu.state, offset, &f64::NAN.to_le_bytes());
    assert!(matches!(
        b.step(),
        Err(GpuError::NonfiniteEnvironment { id: 0, .. })
    ));
    assert_eq!(b.native_contact(9).unwrap(), Some(observed));
    assert_eq!(b.pending_impulses().next(), Some(&action));
    assert_eq!(b.tick(), at);
    assert_eq!(b.environments(), original);
    for k in 0..150 {
        b.step().unwrap();
        control.step().unwrap();
        assert_eq!(
            b.native_contact(9).unwrap(),
            control.native_contact(9).unwrap()
        );
        for (v, w) in b.environments().iter().zip(control.environments()) {
            assert!(
                v.state.values().iter().map(|n| n.to_bits()).eq(w
                    .state
                    .values()
                    .iter()
                    .map(|n| n.to_bits()))
            );
        }
        if k == 4 {
            for a in [&mut b, &mut control] {
                assert_eq!(
                    a.set_pair_solver(a.tick(), &[9], PairSolver::SpecializedGpu)
                        .unwrap()
                        .moved,
                    1
                );
            }
        }
    }
    let EnvironmentState::SpherePlane(s) = b.environments()[b.index(9).unwrap()].state else {
        unreachable!()
    };
    assert!(s.normal_impulse > 0. && s.velocity[2].abs() < 1e-8);
}

#[test]
fn plane_guard_contains_both_integrators_with_oblique_gravity_and_next_impulse() {
    let state = crate::mixed::tests::initial()[2].state;
    let EnvironmentState::SpherePlane(mut s) = state else {
        unreachable!()
    };
    s.plane_normal = [0.6, 0., 0.8];
    s.position = [(s.radius + 0.0001) * 0.6, 0., (s.radius + 0.0001) * 0.8];
    s.velocity = [0.; 3];
    s.gravity = [0.; 3];
    let env = |s| EnvironmentState::SpherePlane(s);
    guard(9, env(s), env(s), None).unwrap();
    let mut inward = s;
    inward.velocity = [-0.1, 0., -0.2];
    inward.gravity = [0., 0., 0.01];
    assert!(matches!(
        guard(9, env(s), env(inward), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::SweptClearance,
            ..
        })
    ));
    s.velocity = [0.01, 0., 0.01];
    s.gravity = [-80., 0., -100.];
    assert!(matches!(
        guard(9, env(s), env(s), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::SweptClearance,
            ..
        })
    ));
}

#[test]
fn cold_supported_authored_owner_can_start_but_requires_clearance_to_change_owner() {
    let mut seed = crate::mixed::tests::initial()[2];
    let EnvironmentState::SpherePlane(mut s) = seed.state else {
        unreachable!()
    };
    s.velocity = [0.; 3];
    s.position[2] = s.radius;
    seed.state = EnvironmentState::SpherePlane(s);
    seed.validate_native_pair().unwrap();
    assert!(matches!(
        guard(seed.id, seed.state, seed.state, None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::SweptClearance,
            ..
        })
    ));
    s.plane_normal = [0., 0., 1. + 1e-13];
    seed.state = EnvironmentState::SpherePlane(s);
    assert!(matches!(
        seed.validate_native_pair(),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::NativeRepresentation,
            ..
        })
    ));
    s.plane_normal = [0.6, 0., 0.8];
    for offset in [1e-9, -0.123456789, 17., 103.5903] {
        s.plane_offset = offset;
        let a = plane_anchor(9, s).unwrap();
        assert_eq!(Vector::from(s.plane_normal).dot(a), offset);
    }
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn registry_preserves_fixed_scene_topology_and_rejects_unresolved_native_commands_atomically() {
    let device = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let fixed = || MixedBatch::from_fixed_scene(&device, crate::mixed::tests::initial()).unwrap();
    let mark = (0, PairSolver::NativeRapier);
    assert!(matches!(
        fixed().with_native_staging(2),
        Err(GpuError::NativeStaging)
    ));
    for workers in [0, MAX_NATIVE_WORKERS + 1] {
        assert!(matches!(
            fixed()
                .with_guarded_native_pairs(&[mark])
                .unwrap()
                .with_native_staging(workers),
            Err(GpuError::NativeStaging)
        ));
    }
    let mut advanced = fixed().with_guarded_native_pairs(&[mark]).unwrap();
    advanced.step().unwrap();
    assert!(matches!(
        advanced.with_native_staging(2),
        Err(GpuError::NativeStaging)
    ));
    let mut pending = fixed().with_guarded_native_pairs(&[mark]).unwrap();
    pending
        .queue_impulses(
            0,
            &[CentralImpulse {
                id: 0,
                body: BodySlot::A,
                impulse: [0.01, 0., 0.],
            }],
        )
        .unwrap();
    assert!(matches!(
        pending.with_native_staging(2),
        Err(GpuError::NativeStaging)
    ));
    let registration_error = |batch: MixedBatch, marks: &[(u64, PairSolver)]| {
        matches!(
            batch.with_guarded_native_pairs(marks),
            Err(GpuError::NativeHandoff {
                reason: NativeHandoffRefusal::Registry,
                ..
            })
        )
    };
    assert!(registration_error(
        MixedBatch::new(&device, crate::mixed::tests::initial()).unwrap(),
        &[mark]
    ));
    assert!(registration_error(fixed(), &[mark; MAX_NATIVE_PAIRS + 1]));
    assert!(matches!(
        fixed().with_guarded_native_pairs(&[mark, mark]),
        Err(GpuError::DuplicateEnvironment(0))
    ));
    assert!(matches!(
        fixed().with_guarded_native_pairs(&[(70, PairSolver::NativeRapier)]),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::Unregistered,
            ..
        })
    ));
    assert!(registration_error(
        fixed().with_initial_cpu(&[0]).unwrap(),
        &[mark]
    ));
    let mut later = fixed();
    later.step().unwrap();
    assert!(registration_error(later, &[mark]));
    let zero = CentralImpulse {
        id: 9,
        body: BodySlot::A,
        impulse: [0.; 3],
    };
    let mut queued = fixed();
    queued.queue_impulses(0, &[zero]).unwrap();
    assert!(registration_error(queued, &[mark]));
    let mut batch = fixed().with_guarded_native_pairs(&[mark]).unwrap();
    assert!(batch.has_guarded_native_pairs());
    assert!(matches!(
        batch.set_pair_solver(1, &[0], PairSolver::SpecializedGpu),
        Err(GpuError::StaleBoundary)
    ));
    assert!(matches!(
        fixed().with_guarded_native_pairs(&[(77, PairSolver::SpecializedGpu)]),
        Err(GpuError::UnknownEnvironment(77))
    ));
    assert!(batch.native_contact(9).is_err());
    assert!(matches!(
        batch.queue_impulses(
            0,
            &[
                zero,
                CentralImpulse {
                    id: 0,
                    body: BodySlot::B,
                    impulse: [10000., 0., 0.]
                }
            ]
        ),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::MotionEnvelope,
            ..
        })
    ));
    assert_eq!(batch.pending_impulses().len(), 0);
    assert!(matches!(
        batch.with_initial_cpu(&[0]),
        Err(GpuError::FixedScene)
    ));
    assert!(registration_error(
        fixed().with_guarded_native_pairs(&[mark]).unwrap(),
        &[mark]
    ));
}

#[test]
fn invalid_inertia_and_unresolved_pose_refuse_the_native_record() {
    let mut s = pair();
    s.a.inverse_mass = 0.;
    assert!(matches!(
        guard(1, env(s), env(s), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::MotionEnvelope,
            ..
        })
    ));
    s = pair();
    s.a.orientation = [0.; 4];
    assert!(matches!(
        guard(1, env(s), env(s), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::NativeRepresentation,
            ..
        })
    ));
}

#[test]
fn equal_velocity_coast_and_near_cap_acceleration_have_a_transactional_bound() {
    let mut p = pair();
    p.a.radius = 1.;
    p.b.radius = 1.;
    p.a.position = [0.; 3];
    p.b.position = [0., 5., 0.];
    p.a.velocity = [399.4, 0., 0.];
    p.b.velocity = p.a.velocity;
    p.gravity = [500., 0., 0.];
    guard(7, env(p), env(p), None).unwrap();
    let w = NativePair::new(7, env(p)).unwrap();
    let before = w.world.bodies[w.handles[0]].translation();
    assert!(matches!(
        w.advance(7, env(p)),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::MotionEnvelope,
            ..
        })
    ));
    assert_eq!(w.world.bodies[w.handles[0]].translation(), before);
    let mut unsafe_input = p;
    unsafe_input.a.velocity = [401., 0., 0.];
    assert!(matches!(
        guard(7, env(p), env(unsafe_input), None),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::MotionEnvelope,
            ..
        })
    ));
}
#[test]
fn authored_plane_rejects_nonphysical_depth_and_unresolved_absolute_speed() {
    let mut e = crate::mixed::tests::initial()[2];
    let EnvironmentState::SpherePlane(mut p) = e.state else {
        unreachable!()
    };
    p.position[2] = p.radius * 0.5;
    e.state = EnvironmentState::SpherePlane(p);
    assert!(matches!(
        e.validate_native_pair(),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::NativeRepresentation,
            ..
        })
    ));
    p.position[2] = 2. * p.radius;
    p.velocity = [800., 0., 0.];
    e.state = EnvironmentState::SpherePlane(p);
    assert!(matches!(
        e.validate_native_pair(),
        Err(GpuError::NativeHandoff {
            reason: NativeHandoffRefusal::MotionEnvelope,
            ..
        })
    ));
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn parallel_native_errors_use_stable_id_order_and_retry_from_the_complete_barrier() {
    let device = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut p = pair();
    p.a.radius = 1.;
    p.b.radius = 1.;
    p.a.position = [0.; 3];
    p.b.position = [0., 5., 0.];
    p.a.velocity = [399.4, 0., 0.];
    p.b.velocity = p.a.velocity;
    p.gravity = [500., 0., 0.];
    let seeds = [33, 1, 17]
        .map(|id| Environment { id, state: env(p) })
        .to_vec();
    let marked = [33, 1, 17].map(|id| (id, PairSolver::NativeRapier));
    for cap in [1, 2, 4] {
        let mut b = MixedBatch::from_fixed_scene(&device, seeds.clone())
            .unwrap()
            .with_guarded_native_pairs(&marked)
            .unwrap()
            .with_native_staging(cap)
            .unwrap();
        let before = b.environments().to_vec();
        // Repeated faults select the lowest stable ID even if another chunk starts first.
        for _ in 0..2 {
            assert!(matches!(
                b.step(),
                Err(GpuError::NativeHandoff {
                    id: 1,
                    reason: NativeHandoffRefusal::MotionEnvelope
                })
            ));
            assert_eq!(b.tick(), 0);
            assert_eq!(b.environments(), before);
        }
        assert_eq!(b.native_staging_workers(), cap);
        b.reset();
        assert_eq!(b.native_staging_workers(), cap);
    }
}
#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn parallel_loaded_siblings_rollback_all_warm_caches_and_pending_inputs_after_device_failure() {
    let device = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let initial = crate::mixed::tests::initial();
    let make = |cap| {
        let mut seed = initial.clone();
        let mut second = initial[2];
        second.id = 12;
        seed.push(second);
        let b = MixedBatch::from_fixed_scene(&device, seed)
            .unwrap()
            .with_guarded_native_pairs(&[
                (9, PairSolver::NativeRapier),
                (12, PairSolver::NativeRapier),
            ])
            .unwrap();
        if cap == 1 {
            b.with_native_staging(cap)
        } else {
            b.with_overlapped_native_staging(cap)
        }
        .unwrap()
    };
    let mut b = make(4);
    let mut serial = make(1);
    for _ in 0..40 {
        b.step().unwrap();
        serial.step().unwrap();
    }
    let cache = [9, 12].map(|id| b.native_contact(id).unwrap().unwrap());
    assert!(
        cache
            .iter()
            .all(|c| c.warmstart_normal_impulse > 0. && c.warmstart_tangent_impulse > 0.)
    );
    let input = CentralImpulse {
        id: 9,
        body: BodySlot::A,
        impulse: [0.1, 0.05, 0.6],
    };
    for s in [&mut b, &mut serial] {
        s.queue_impulses(40, &[input]).unwrap();
    }
    let original = b.environments().to_vec();
    let offset = b.active_offsets()[b.index(0).unwrap()] as u64;
    b.gpu
        .queue
        .write_buffer(&b.gpu.state, offset, &f64::NAN.to_le_bytes());
    assert!(matches!(
        b.step(),
        Err(GpuError::NonfiniteEnvironment { id: 0, .. })
    ));
    assert_eq!(b.tick(), 40);
    assert_eq!(b.environments(), original);
    assert_eq!(b.pending_impulses().next(), Some(&input));
    assert_eq!(
        [9, 12].map(|id| b.native_contact(id).unwrap().unwrap()),
        cache
    );
    for _ in 0..32 {
        b.step().unwrap();
        serial.step().unwrap();
        for (a, c) in b.environments().iter().zip(serial.environments()) {
            assert!(
                a.state.values().iter().map(|x| x.to_bits()).eq(c
                    .state
                    .values()
                    .iter()
                    .map(|x| x.to_bits()))
            );
        }
        for id in [9, 12] {
            assert_eq!(
                b.native_contact(id).unwrap(),
                serial.native_contact(id).unwrap()
            );
        }
    }
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn early_device_submission_is_drained_and_reseeded_after_a_native_error() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut seed = crate::mixed::tests::initial();
    let EnvironmentState::SpherePlane(mut plane) = seed[2].state else {
        unreachable!()
    };
    plane.radius = 1.;
    plane.position = [0., 0., 10.];
    plane.velocity = [399.4, 0., 0.];
    plane.gravity = [500., 0., 0.];
    seed[2].state = EnvironmentState::SpherePlane(plane);
    let make = |overlap| {
        let b = MixedBatch::from_fixed_scene(&adapter, seed.clone())
            .unwrap()
            .with_guarded_native_pairs(&[(9, PairSolver::NativeRapier)])
            .unwrap();
        if overlap {
            b.with_overlapped_native_staging(2)
        } else {
            b.with_native_staging(2)
        }
        .unwrap()
    };
    let mut b = make(true);
    let mut control = make(false);
    let gpu = CentralImpulse {
        id: 0,
        body: BodySlot::A,
        impulse: [0., 0.1, 0.],
    };
    let correction = CentralImpulse {
        id: 9,
        body: BodySlot::A,
        impulse: [-1., 0., 0.],
    };
    b.queue_impulses(0, &[gpu]).unwrap();
    let committed = b.environments().to_vec();
    let start = b.begin_tick().unwrap();
    assert!(matches!(start.submitted, Some(Ok(_))));
    assert!(matches!(
        b.finish_tick(start),
        Err(GpuError::NativeHandoff {
            id: 9,
            reason: NativeHandoffRefusal::MotionEnvelope
        })
    ));
    assert_eq!(b.tick(), 0);
    assert_eq!(b.environments(), committed);
    assert_eq!(b.pending_impulses().collect::<Vec<_>>(), [&gpu]);
    b.queue_impulses(0, &[correction]).unwrap();
    control.queue_impulses(0, &[gpu, correction]).unwrap();
    let retried = b.step().unwrap();
    assert_eq!(retried.uploaded_bytes, b.active_gpu_bytes());
    assert!(retried.cpu_between_submit_and_fence > Duration::ZERO);
    let baseline = control.step().unwrap();
    assert_eq!(baseline.cpu_between_submit_and_fence, Duration::ZERO);
    assert_eq!(b.environments(), control.environments());
    assert_eq!(
        b.native_contact(9).unwrap(),
        control.native_contact(9).unwrap()
    );
    b.reset();
    assert!(b.overlaps_native_gpu());
    b.set_pair_solver(0, &[9], PairSolver::SpecializedGpu)
        .unwrap();
    assert_eq!(
        b.step().unwrap().cpu_between_submit_and_fence,
        Duration::ZERO
    );
}

#[test]
#[ignore = "destroys this test's private physical Vulkan device"]
fn destroying_live_device_after_early_submission_retains_the_last_complete_warm_frame() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let make = || {
        MixedBatch::from_fixed_scene(&adapter, crate::mixed::tests::initial())
            .unwrap()
            .with_guarded_native_pairs(&[(9, PairSolver::NativeRapier)])
            .unwrap()
            .with_overlapped_native_staging(2)
            .unwrap()
    };
    let mut b = make();
    for _ in 0..40 {
        b.step().unwrap();
    }
    let actions = [
        CentralImpulse {
            id: 9,
            body: BodySlot::A,
            impulse: [0.02, 0., 0.1],
        },
        CentralImpulse {
            id: 0,
            body: BodySlot::B,
            impulse: [0.1, 0., 0.],
        },
    ];
    b.queue_impulses(40, &actions).unwrap();
    let previous = b.environments().to_vec();
    let cache = b.native_contact(9).unwrap().unwrap();
    assert!(cache.warmstart_normal_impulse > 0. && cache.warmstart_tangent_impulse > 0.);
    let started = b.begin_tick().unwrap();
    assert!(matches!(started.submitted, Some(Ok(_))));
    b.gpu.device.destroy();
    let error = b.finish_tick(started).unwrap_err();
    assert!(
        matches!(error, GpuError::Device(_) | GpuError::Readback(_)),
        "{error}"
    );
    for _ in 0..2 {
        assert_eq!(b.tick(), 40);
        assert_eq!(b.environments(), previous);
        assert_eq!(b.native_contact(9).unwrap(), Some(cache));
        assert_eq!(
            b.pending_impulses().copied().collect::<Vec<_>>(),
            [actions[1], actions[0]]
        );
        assert!(b.step().is_err());
    }
    let mut replay = make();
    for _ in 0..40 {
        replay.step().unwrap();
    }
    assert_eq!(replay.environments(), previous);
    assert_eq!(replay.native_contact(9).unwrap(), Some(cache));
    replay.queue_impulses(40, &actions).unwrap();
    replay.step().unwrap();
    assert_eq!(replay.tick(), 41);
    assert_eq!(replay.pending_impulses().len(), 0);
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn simultaneous_native_and_gpu_faults_keep_the_bevy_clock_and_return_native_error_first() {
    use crate::simulation::{BatchTiming, install_mixed};
    use gz_sim_core::{Simulation, UpdateInfo};
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut seed = crate::mixed::tests::initial();
    let EnvironmentState::SpherePlane(plane) = &mut seed[2].state else {
        unreachable!()
    };
    plane.radius = 1.;
    plane.position = [0., 0., 10.];
    plane.velocity = [398.9, 0., 0.];
    plane.gravity = [500., 0., 0.];
    let batch = MixedBatch::from_fixed_scene(&adapter, seed)
        .unwrap()
        .with_guarded_native_pairs(&[(9, PairSolver::NativeRapier)])
        .unwrap()
        .with_overlapped_native_staging(2)
        .unwrap();
    let mut sim = Simulation::new(STEP).unwrap();
    install_mixed(&mut sim, batch).unwrap();
    sim.step_exact(1).unwrap();
    let before = *sim.world().resource::<UpdateInfo>();
    let timing = sim.world().resource::<BatchTiming>().last.unwrap();
    assert!(timing.cpu_between_submit_and_fence > Duration::ZERO);
    let batch = sim.world().resource::<MixedBatch>();
    let previous = batch.environments().to_vec();
    let history = batch.native_contact(9).unwrap();
    let offset = batch.active_offsets()[batch.index(0).unwrap()] as u64;
    // The resident GPU row will be nonfinite while the host's next gravity
    // increment exits the resolved native speed envelope on the same advance.
    batch
        .gpu
        .queue
        .write_buffer(&batch.gpu.state, offset, &f64::NAN.to_le_bytes());
    assert_eq!(sim.step_exact(1), Err(gz_sim_core::StepError::Faulted));
    assert!(matches!(
        sim.fault().unwrap().downcast_ref::<GpuError>(),
        Some(GpuError::NativeHandoff {
            id: 9,
            reason: NativeHandoffRefusal::MotionEnvelope
        })
    ));
    assert!(sim.is_faulted());
    assert_eq!(*sim.world().resource::<UpdateInfo>(), before);
    assert_eq!(
        sim.world().resource::<BatchTiming>().last.unwrap().latency,
        timing.latency
    );
    let batch = sim.world().resource::<MixedBatch>();
    assert_eq!(batch.native_contact(9).unwrap(), history);
    assert_eq!(batch.environments(), previous);
    assert_eq!(batch.tick(), 1);
}
