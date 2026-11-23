// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn world() -> RapierBackend {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for x in [-1., 1.] {
        backend
            .spawn_awake_frictionless_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.25; 3],
                    },
                    mass: Some(2.),
                    position: [x, 0., 0.],
                    velocity: [0.; 3],
                },
                RotationSpec::default(),
                false,
            )
            .unwrap();
    }
    backend
}

#[test]
fn native_frame_requires_current_codec_size_complete_topology_and_portable_construction() {
    let base = world().checkpoint_boxes().unwrap();
    let mut wrong = base.clone();
    wrong[0] ^= 1;
    assert!(RapierBackend::restore_trusted_boxes(&wrong).is_err());
    assert!(
        RapierBackend::restore_trusted_boxes(&vec![0; PARTITION_CHECKPOINT_LIMIT + 1]).is_err()
    );
    type Image = (u32, u64, bool, Vec<BoxRecord>, PhysicsWorld);
    fn reject(base: &[u8], edit: impl FnOnce(&mut Image)) {
        let mut image: Image = codec().deserialize(base).unwrap();
        edit(&mut image);
        let bytes = codec().serialize(&image).unwrap();
        assert!(RapierBackend::restore_trusted_boxes(&bytes).is_err());
    }
    reject(&base, |i| {
        i.3.clear();
    });
    reject(&base, |i| {
        i.3.pop();
    });
    // Prefix the first legitimate record with a corrupt count, without an oversized allocation.
    let mut oversized = base.clone();
    let count_start = 4 + 8 + 1;
    oversized[count_start..count_start + 8].copy_from_slice(&u64::MAX.to_le_bytes());
    assert!(RapierBackend::restore_trusted_boxes(&oversized).is_err());
    reject(&base, |i| i.1 = i.3[1].id);
    reject(&base, |i| i.3.swap(0, 1));
    reject(&base, |i| i.3[1].handle = i.3[0].handle);
    reject(&base, |i| i.3[1].handle = RigidBodyHandle::invalid());
    reject(&base, |i| i.3[0].mass = 10.);
    reject(&base, |i| i.3[0].orientation = [0.; 4]);
    reject(&base, |i| i.3[0].half_extents = [0.4; 3]);
    reject(&base, |i| i.4.gravity = Vector::Z);
    reject(&base, |i| i.4.integration_parameters.dt = 0.011);
    reject(&base, |i| i.4.integration_parameters.length_unit = 2.);
    reject(&base, |i| {
        i.4.bodies.insert(RigidBodyBuilder::fixed());
    });
    reject(&base, |i| {
        i.4.colliders.insert(ColliderBuilder::ball(1.));
    });
    reject(&base, |i| {
        i.4.impulse_joints
            .insert(i.3[0].handle, i.3[1].handle, FixedJointBuilder::new(), true);
    });
}

#[test]
fn changed_material_locked_or_sleeping_body_and_unowned_wrench_refuse_capture() {
    fn reject(edit: impl FnOnce(&mut PhysicsWorld, RigidBodyHandle, ColliderHandle)) {
        let mut backend = world();
        let handle = backend.bodies[&BodyId::from_raw(0)].0;
        let shape = backend.world.bodies[handle].colliders()[0];
        edit(&mut backend.world, handle, shape);
        assert!(backend.checkpoint_boxes().is_err());
    }
    reject(|w, h, _| w.bodies[h].set_enabled(false));
    reject(|w, h, _| w.bodies[h].sleep());
    reject(|w, h, _| w.bodies[h].set_locked_axes(LockedAxes::ROTATION_LOCKED, true));
    reject(|w, h, _| w.bodies[h].set_linear_damping(0.1));
    reject(|w, h, _| w.bodies[h].add_force(Vector::X, true));
    reject(|w, h, _| w.bodies[h].add_torque(Vector::Z, true));
    reject(|w, _, c| w.colliders[c].set_enabled(false));
    reject(|w, _, c| w.colliders[c].set_sensor(true));
    reject(|w, _, c| w.colliders[c].set_friction(1.));
    reject(|w, _, c| w.colliders[c].set_restitution(1.));
    reject(|w, _, c| w.colliders[c].set_collision_groups(InteractionGroups::none()));
    reject(|w, _, c| w.colliders[c].set_position_wrt_parent(Pose::translation(0.1, 0., 0.)));
    reject(|w, _, c| w.colliders[c].set_shape(SharedShape::ball(0.2)));
    reject(|w, _, c| w.colliders.set_parent(c, None, &mut w.bodies));
    let mut backend = world();
    backend.remove(BodyId::from_raw(0));
    assert!(backend.checkpoint_boxes().is_err());
}

fn impact_world(count: usize) -> RapierBackend {
    let mut out = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for i in 0..count {
        let (pos, vel, width, mass) = match i {
            0 => ([-1.5, 0., 0.], 1.5, 0.5, 1.),
            1 => ([0., 0., 0.], 0.5, 0.5, 2.),
            2 => ([1.5, 0., 0.], -0.5, 0.5, 1.),
            3 => ([-1.65, 0., 12.], 0.65, 0.6, 1.5),
            _ => ([1.95, 0., -12.], -0.85, 0.6, 2.5),
        };
        out.spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [width / 2.; 3],
                },
                position: pos,
                velocity: [vel, 0., 0.],
                mass: Some(mass),
            },
            RotationSpec::default(),
            false,
        )
        .unwrap();
    }
    out
}

fn cold_reseed(native: &RapierBackend) -> RapierBackend {
    let mut out = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for i in 0..3 {
        let id = BodyId::from_raw(i);
        let state = native.state(id).unwrap();
        let (size, mass) = native.partition_box(id).unwrap();
        out.spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: size.map(|x| x / 2.),
                },
                position: state.position,
                velocity: state.velocity,
                mass: Some(mass),
            },
            RotationSpec {
                orientation: state.orientation,
                angular_velocity: state.angular_velocity,
            },
            false,
        )
        .unwrap();
    }
    out
}

#[test]
fn cold_rebuild_matches_uninterrupted_simultaneous_impact_with_both_manifolds_active() {
    let ordinary = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    assert!(ordinary.world.integration_parameters.contact_recycling);
    let mut baseline = impact_world(5);
    let mut isolated = impact_world(3);
    let mut cold = impact_world(3);
    assert!(!cold.world.integration_parameters.contact_recycling);
    assert_eq!(cold.world.integration_parameters.warmstart_coefficient, 1.);
    for t in 1..=150 {
        if t == 63 {
            cold = cold_reseed(&baseline);
        }
        for solver in [&mut baseline, &mut isolated, &mut cold] {
            solver.step(Duration::from_millis(10)).unwrap();
        }
        for raw in 0..3 {
            let id = BodyId::from_raw(raw);
            assert_eq!(cold.state(id), baseline.state(id));
            assert_eq!(isolated.state(id), baseline.state(id));
        }
        if t == 100 {
            let p = cold.rigid_contact_observations();
            assert_eq!(p.len(), 2);
            assert!(p.iter().all(|p| p.active && p.normal_impulse > 0.5));
            assert_eq!(cold.partition_warmstart(), baseline.partition_warmstart());
            assert!(
                (cold.state(BodyId::from_raw(1)).unwrap().velocity[0] - 0.415022222224).abs()
                    < 1e-11
            );
        }
    }
}

/// Bounded wall-clock comparison of the native approximation on a compressed
/// zero-gravity chain. Run the ignored benchmark with --release --nocapture.
#[test]
#[ignore = "release timing benchmark"]
fn compare_contact_refresh_cost_in_compressed_rows() {
    let dt = Duration::from_millis(10);
    fn sample(bytes: &[u8], recycle: bool, dt: Duration) -> (f64, usize) {
        let mut solver = RapierBackend::restore_trusted_boxes(bytes).unwrap();
        // Warm scratch buffers and the prior-pose cache identically for both arms.
        solver.world.integration_parameters.contact_recycling = true;
        solver.step(dt).unwrap();
        solver.world.integration_parameters.contact_recycling = recycle;
        let start = std::time::Instant::now();
        solver.step(dt).unwrap();
        let us = start.elapsed().as_secs_f64() * 1e6;
        (us, solver.active_contact_pairs())
    }
    for n in [3, 12, 32] {
        let mut seed = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        for i in 0..n {
            seed.spawn_awake_frictionless_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.25; 3],
                    },
                    position: [i as f64 * 0.625, 0., 0.],
                    velocity: [(n - 1) as f64 * 0.5 - i as f64, 0., 0.],
                    mass: Some(1.),
                },
                RotationSpec::default(),
                false,
            )
            .unwrap();
        }
        let mut template = None;
        for _ in 0..150 {
            seed.step(dt).unwrap();
            if seed.active_contact_pairs() != n - 1 {
                continue;
            }
            let bytes = seed.checkpoint_boxes().unwrap();
            if [true, false]
                .into_iter()
                .all(|mode| sample(&bytes, mode, dt).1 == n - 1)
            {
                template = Some(bytes);
                break;
            }
        }
        let bytes = template.expect("row with matching active constraints");
        let mut times = [Vec::new(), Vec::new()];
        for round in 0..9 {
            let mut total = [0.; 2];
            for i in 0..48 {
                for mode in [(round + i) % 2, 1 - (round + i) % 2] {
                    let (us, pairs) = sample(&bytes, mode == 0, dt);
                    assert_eq!(pairs, n - 1);
                    total[mode] += us;
                }
            }
            for (series, time) in times.iter_mut().zip(total) {
                series.push(time / 48.);
            }
        }
        for series in &mut times {
            series.sort_by(f64::total_cmp);
        }
        eprintln!(
            "{n} boxes / {} active pairs, identical inputs and warmed scratch, median of 9x48 alternate samples: native us/tick recycling={:.3}, refresh={:.3}, ratio={:.3}",
            n - 1,
            times[0][4],
            times[1][4],
            times[1][4] / times[0][4]
        );
    }
}

#[test]
fn stationary_prescribed_body_wakes_to_the_same_twist_after_native_restore() {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let id = backend
        .spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.3; 3],
                },
                position: [0., 0., 12.],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            true,
        )
        .unwrap();
    for tick in 0..75 {
        if tick == 50 {
            assert!(backend.world.bodies[backend.bodies[&id].0].is_sleeping());
        }
        let mut restored =
            RapierBackend::restore_trusted_solver(&backend.checkpoint_solver().unwrap()).unwrap();
        if tick == 74 {
            let (v, w) = ([1., 0.1, 0.], [0., 0.3, 0.]);
            backend.set_world_velocity(id, v, w).unwrap();
            restored.set_world_velocity(id, v, w).unwrap();
            restored.step(Duration::from_millis(10)).unwrap();
        }
        backend.step(Duration::from_millis(10)).unwrap();
        if tick == 74 {
            assert_eq!(backend.state(id), restored.state(id));
        }
    }
}

#[test]
fn terrain_images_preserve_native_contacts_and_validate_one_horizontal_frictionless_halfspace() {
    let mut bare = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    assert!(bare.set_partition_ground(f64::NAN).is_err());
    bare.set_partition_ground(0.5).unwrap();
    assert!(bare.set_partition_ground(0.5).is_err());
    for x in [-2., 2.] {
        bare.spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.25; 3],
                },
                mass: Some(1.),
                position: [x, 0., 1.],
                velocity: [0.; 3],
            },
            RotationSpec::default(),
            false,
        )
        .unwrap();
    }
    let raw = bare.checkpoint_boxes().unwrap();
    assert_eq!(
        RapierBackend::restore_trusted_boxes(&raw)
            .unwrap()
            .partition_ground_height()
            .unwrap(),
        Some(0.5)
    );
    assert!(bare.set_partition_ground(0.).is_err());
    let g = partition_ground::ground_of(&bare.world).unwrap().unwrap().0;
    bare.world.colliders[g].set_friction(0.5);
    assert!(bare.checkpoint_solver().is_err());
    bare.world.colliders[g].set_friction(0.);
    bare.world.colliders[g].set_shape(SharedShape::halfspace(Vector::X));
    let mut wrong_profile = bare.checkpoint_solver().unwrap();
    assert_eq!(
        wrong_profile[..4],
        (GROUND_FORMAT | ORIENTED_GROUND).to_le_bytes()
    );
    wrong_profile[..4].copy_from_slice(&GROUND_FORMAT.to_le_bytes());
    assert!(RapierBackend::restore_trusted_solver(&wrong_profile).is_err());
    let mut invalid_oriented = raw;
    invalid_oriented[..4].copy_from_slice(&(GROUND_FORMAT | ORIENTED_GROUND).to_le_bytes());
    assert!(RapierBackend::restore_trusted_solver(&invalid_oriented).is_err());
    bare.world.colliders[g].set_shape(SharedShape::ball(1.));
    assert!(bare.checkpoint_solver().is_err());
    bare.world.colliders[g].set_shape(SharedShape::halfspace(Vector::Z));
    bare.world
        .colliders
        .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Z)));
    assert!(bare.checkpoint_solver().is_err());
}

#[test]
fn material_images_bind_each_box_and_ground_coefficient_and_combination_rule() {
    type Image = (u32, u64, bool, Vec<(BoxKind, f64, BoxRecord)>, PhysicsWorld);
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    b.set_partition_ground_material(-0.25, 0.37).unwrap();
    for (i, mu) in [0., 0.75].into_iter().enumerate() {
        b.spawn_partition_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.25; 3],
                },
                position: [2. * i as f64, 0., 0.75],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            false,
            mu,
        )
        .unwrap();
    }
    let encoded = b.checkpoint_boxes().unwrap();
    assert_eq!(&encoded[..4], &MATERIAL_FORMAT.to_le_bytes());
    let restored = RapierBackend::restore_trusted_boxes(&encoded).unwrap();
    assert_eq!(
        restored.partition_ground_material().unwrap(),
        Some((-0.25, 0.37))
    );
    assert_eq!(restored.partition_friction(BodyId::from_raw(1)), Some(0.75));
    let reject = |edit: fn(&mut Image)| {
        let mut x: Image = codec().deserialize(&encoded).unwrap();
        edit(&mut x);
        assert!(RapierBackend::restore_trusted_solver(&codec().serialize(&x).unwrap()).is_err());
    };
    reject(|x| x.3[1].1 = 0.3);
    reject(|x| x.3[1].1 = f64::NAN);
    reject(|x| x.3[1].1 = -1.);
    reject(|x| {
        let c = x.4.bodies[x.3[1].2.handle].colliders()[0];
        x.4.colliders[c].set_friction_combine_rule(CoefficientCombineRule::Max);
    });
    reject(|x| {
        let (id, _) = partition_ground::ground_of(&x.4).unwrap().unwrap();
        x.4.colliders[id].set_friction_combine_rule(CoefficientCombineRule::Average);
    });
    let mut reset = restored;
    let captured = reset.capture_initial();
    reset.reset().unwrap();
    assert_eq!(
        reset.partition_ground_material().unwrap(),
        Some((-0.25, 0.37))
    );
    let prepared = reset.prepare_initial(&captured).unwrap();
    reset.commit_initial(prepared).unwrap();
    assert_eq!(reset.partition_friction(BodyId::from_raw(1)), Some(0.75));
    assert_eq!(
        reset.partition_ground_material().unwrap(),
        Some((-0.25, 0.37))
    );
}

#[test]
fn only_the_ground_needs_positive_friction_to_use_the_material_format() {
    let mut w = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    w.set_partition_ground_material(0., 0.7).unwrap();
    for x in [-2., 2.] {
        w.spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.2; 3],
                },
                position: [x, 0., 0.21],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            false,
        )
        .unwrap();
    }
    let bytes = w.checkpoint_boxes().unwrap();
    assert_eq!(&bytes[..4], &MATERIAL_FORMAT.to_le_bytes());
    assert_eq!(
        RapierBackend::restore_trusted_boxes(&bytes)
            .unwrap()
            .partition_ground_material()
            .unwrap(),
        Some((0., 0.7))
    );
}

#[test]
fn mixed_primitive_images_bind_shape_material_and_native_motion_to_each_record() {
    let mut solver = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for (index, shape) in [
        Shape::Sphere { radius: 0.25 },
        Shape::Box {
            half_extents: [0.25; 3],
        },
        Shape::Sphere { radius: 0.375 },
    ]
    .into_iter()
    .enumerate()
    {
        solver
            .spawn_partition_primitive(
                BodySpec {
                    shape,
                    mass: (index != 1).then_some(2.),
                    position: [index as f64 * 2., 0., 0.],
                    velocity: if index == 2 { [0.1, 0., 0.] } else { [0.; 3] },
                },
                RotationSpec::default(),
                index == 2,
                0.45,
            )
            .unwrap();
    }
    let image = solver.checkpoint_solver().unwrap();
    assert!(image.starts_with(&PRIMITIVE_FORMAT.to_le_bytes()));
    assert!(solver.checkpoint_boxes().is_err());
    assert!(RapierBackend::restore_trusted_boxes(&image).is_err());
    let mut restored = RapierBackend::restore_trusted_solver(&image).unwrap();
    assert!(restored.partition_box(BodyId::from_raw(0)).is_none());
    assert_eq!(
        restored.partition_box(BodyId::from_raw(1)),
        Some(([0.5; 3], 0.))
    );
    for _ in 0..60 {
        for w in [&mut solver, &mut restored] {
            w.step(Duration::from_millis(10)).unwrap();
        }
        for i in 0..3 {
            let id = BodyId::from_raw(i);
            assert_eq!(solver.state(id), restored.state(id));
        }
    }
    type Image = (
        u32,
        u64,
        bool,
        Vec<(BoxKind, f64, bool, BoxRecord)>,
        PhysicsWorld,
    );
    let reject = |edit: fn(&mut Image)| {
        let mut decoded: Image = codec().deserialize(&image).unwrap();
        edit(&mut decoded);
        assert!(
            RapierBackend::restore_trusted_solver(&codec().serialize(&decoded).unwrap()).is_err()
        );
    };
    reject(|i| i.3[0].2 = false);
    reject(|i| i.3[1].2 = true);
    reject(|i| i.3[0].3.half_extents[1] = 0.375);
    reject(|i| i.3[0].3.half_extents = [0.375; 3]);
    reject(|i| i.3[0].1 = 0.37);
    reject(|i| i.3[2].0 = BoxKind::Dynamic);
    reject(|i| {
        for r in &mut i.3 {
            r.2 = false;
        }
    });
    reject(|i| {
        let c = i.4.bodies[i.3[0].3.handle].colliders()[0];
        i.4.colliders[c].set_shape(SharedShape::cuboid(0.25, 0.25, 0.25));
    });
}

#[test]
fn capsule_checkpoint_binds_centerline_length_axis_radius_and_motion() {
    let mut solver = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for (i, shape) in [
        Shape::Capsule {
            radius: 0.2,
            half_length: 0.45,
        },
        Shape::Sphere { radius: 0.2 },
        Shape::Box {
            half_extents: [0.2; 3],
        },
    ]
    .into_iter()
    .enumerate()
    {
        solver
            .spawn_partition_primitive(
                BodySpec {
                    shape,
                    mass: Some(1.),
                    position: [2. * (i as f64), 0., 0.],
                    velocity: [0.; 3],
                },
                RotationSpec::default(),
                i == 1,
                0.51,
            )
            .unwrap();
    }
    let raw = solver.checkpoint_solver().unwrap();
    assert!(raw.starts_with(&CAPSULE_FORMAT.to_le_bytes()));
    assert!(RapierBackend::restore_trusted_boxes(&raw).is_err());
    let fresh = RapierBackend::restore_trusted_solver(&raw).unwrap();
    assert!(matches!(
        fresh.partition_shape(BodyId::from_raw(0)),
        Some((
            Shape::Capsule {
                radius: 0.2,
                half_length: 0.45
            },
            1.
        ))
    ));
    type Image = (
        u32,
        u64,
        bool,
        Vec<(BoxKind, f64, PrimitiveKind, BoxRecord)>,
        PhysicsWorld,
    );
    let reject = |edit: fn(&mut Image)| {
        let mut image: Image = codec().deserialize(&raw).unwrap();
        edit(&mut image);
        assert!(
            RapierBackend::restore_trusted_solver(&codec().serialize(&image).unwrap()).is_err()
        );
    };
    reject(|i| i.3[0].2 = PrimitiveKind::Sphere);
    reject(|i| i.3[2].2 = PrimitiveKind::Capsule);
    reject(|i| i.3[0].3.half_extents[1] = 0.17);
    reject(|i| i.3[0].3.half_extents[2] = 0.40);
    reject(|i| {
        let handle = i.4.bodies[i.3[0].3.handle].colliders()[0];
        i.4.colliders[handle].set_shape(SharedShape::capsule_y(0.45, 0.2));
    });
    reject(|i| {
        let handle = i.4.bodies[i.3[0].3.handle].colliders()[0];
        i.4.colliders[handle].set_shape(SharedShape::capsule(
            Vector::new(0., 0., -0.35),
            Vector::new(0., 0., 0.55),
            0.2,
        ));
    });
}

#[test]
fn cylinder_checkpoint_pins_flat_cap_dimensions_and_local_axis() {
    let mut solver = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for (n, shape) in [
        Shape::Cylinder {
            radius: 0.45,
            half_height: 0.0625,
        },
        Shape::Capsule {
            radius: 0.2,
            half_length: 0.35,
        },
        Shape::Sphere { radius: 0.2 },
    ]
    .into_iter()
    .enumerate()
    {
        solver
            .spawn_partition_primitive(
                BodySpec {
                    shape,
                    mass: Some(1.),
                    position: [0., 3. * n as f64, 0.],
                    velocity: [0.; 3],
                },
                RotationSpec::default(),
                n == 2,
                0.51,
            )
            .unwrap();
    }
    let raw = solver.checkpoint_solver().unwrap();
    assert!(raw.starts_with(&CYLINDER_FORMAT.to_le_bytes()));
    let restored = RapierBackend::restore_trusted_solver(&raw).unwrap();
    assert!(matches!(
        restored.partition_shape(BodyId::from_raw(0)),
        Some((
            Shape::Cylinder {
                radius: 0.45,
                half_height: 0.0625
            },
            1.
        ))
    ));
    type Image = (
        u32,
        u64,
        bool,
        Vec<(BoxKind, f64, PrimitiveKind, BoxRecord)>,
        PhysicsWorld,
    );
    let reject = |edit: fn(&mut Image)| {
        let mut i: Image = codec().deserialize(&raw).unwrap();
        edit(&mut i);
        assert!(RapierBackend::restore_trusted_solver(&codec().serialize(&i).unwrap()).is_err());
    };
    reject(|i| i.0 = CAPSULE_FORMAT);
    reject(|i| i.3[0].2 = PrimitiveKind::Capsule);
    reject(|i| i.3[0].3.half_extents[2] = 0.086);
    reject(|i| {
        let h = i.4.bodies[i.3[0].3.handle].colliders()[0];
        i.4.colliders[h].set_position_wrt_parent(Pose::IDENTITY);
    });
}
