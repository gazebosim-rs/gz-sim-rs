// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn isolated_box_friction_and_sleep_settings_survive_native_reset() {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let id = backend
        .spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.25; 3],
                },
                position: [0.; 3],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            false,
        )
        .unwrap();
    for _ in 0..2 {
        for _ in 0..100 {
            backend.step(Duration::from_millis(10)).unwrap();
        }
        let handle = backend.bodies[&id].0;
        let body = &backend.world.bodies[handle];
        assert!(!body.is_sleeping());
        assert!(body.activation().normalized_linear_threshold < 0.);
        assert_eq!(backend.world.colliders[body.colliders()[0]].friction(), 0.);
        backend.reset().unwrap();
    }
}

#[test]
fn insertion_checks_materialized_translation_before_allocating_a_native_body() {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let mut spec = BodySpec {
        shape: Shape::Box {
            half_extents: [0.5; 3],
        },
        position: [0.; 3],
        velocity: [2., 0., 0.],
        mass: Some(1.),
    };
    let body = backend
        .spawn_awake_frictionless_box(spec, RotationSpec::default(), true)
        .unwrap();
    backend.step(Duration::from_millis(500)).unwrap();
    spec.position = [1.75, 0., 0.];
    spec.velocity = [0.; 3];
    // At construction coordinates these boxes would appear disjoint.
    assert!(matches!(
        backend.spawn_awake_frictionless_box(spec, RotationSpec::default(), false),
        Err(PhysicsError::TransitionDeferred { .. })
    ));
    assert_eq!(backend.state(body).unwrap().position, [1., 0., 0.]);
    spec.position[0] = 2.5;
    let id = backend
        .spawn_awake_frictionless_box(spec, RotationSpec::default(), false)
        .unwrap();
    assert_eq!(id.raw(), body.raw() + 1);
    assert!(matches!(
        backend.spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                ..spec
            },
            RotationSpec::default(),
            false
        ),
        Err(PhysicsError::InvalidValue)
    ));
}

#[cfg(feature = "partition-checkpoint")]
#[test]
fn warm_box_world_retains_one_shot_native_forces_and_torques() {
    let mut source = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    for x in [-1., 1.] {
        source
            .spawn_awake_frictionless_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.25; 3],
                    },
                    position: [x, 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                RotationSpec::default(),
                false,
            )
            .unwrap();
    }
    let id = BodyId::from_raw(1);
    source
        .add_world_wrench(id, [4., 0., 0.], [0., 0., 1.])
        .unwrap();
    let native = source.checkpoint_boxes().unwrap();
    assert!(RapierBackend::restore_trusted_boxes(&native[..native.len() - 1]).is_err());
    let mut extra = native.clone();
    extra.push(0);
    assert!(RapierBackend::restore_trusted_boxes(&extra).is_err());
    let mut restored = RapierBackend::restore_trusted_boxes(&native).unwrap();
    for _ in 0..3 {
        for world in [&mut source, &mut restored] {
            world.step(Duration::from_millis(10)).unwrap();
        }
        assert_eq!(source.state(id), restored.state(id));
        let b = restored.state(id).unwrap();
        assert!((b.velocity[0] - 0.02).abs() < 1e-12);
        assert!((b.angular_velocity[2] - 0.12).abs() < 1e-12);
    }
}

#[test]
fn oriented_admission_never_discards_a_near_pair_in_independent_native_distance_queries() {
    use rapier3d_f64::parry::{query, shape::Cuboid};
    let mut seed = 0x62ae52be3u64;
    let mut next = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / (1u64 << 53) as f64
    };
    let mut refused = 0;
    let mut refined = 0;
    for i in 0..12_000 {
        let mut triplet = || [next(), next(), next()];
        let [a, b, c] = triplet();
        let size1 = triplet().map(|n| 0.05 + 1.85 * n);
        let size2 = triplet().map(|n| 0.05 + 1.85 * n);
        let origin = Vector::from(triplet().map(|n| if i % 4 == 0 { 2e6 * n } else { 7. * n }));
        let t1 = Pose::from_parts(
            origin,
            Rotation::from_scaled_axis(Vector::new(a - 0.5, b - 0.5, c - 0.5) * 6.),
        );
        let [a, b, c] = triplet();
        let r = if i % 5 == 0 {
            t1.rotation * Rotation::from_rotation_y(a * 1e-8)
        } else {
            Rotation::from_scaled_axis(Vector::new(a - 0.5, b - 0.5, c - 0.5) * 6.)
        };
        let t2 = Pose::from_parts(origin + Vector::from(triplet().map(|n| 3.2 * (n - 0.5))), r);
        let a = Cuboid::new(Vector::from(size1) * 0.5);
        let b = Cuboid::new(Vector::from(size2) * 0.5);
        let ba = PartitionBoxBounds::new(size1, t1.translation.to_array(), t1.rotation.to_array())
            .unwrap();
        let bb = PartitionBoxBounds::new(size2, t2.translation.to_array(), t2.rotation.to_array())
            .unwrap();
        let actual = ba.predicts_overlap(&bb);
        assert_eq!(actual, bb.predicts_overlap(&ba));
        let distance = query::distance(&t1, &a, &t2, &b).unwrap().distance;
        if distance <= partition_prediction_distance() - 1e-8 {
            refused += 1;
            assert!(
                actual,
                "false separation for configuration {i}, distance {distance}"
            );
        }
        if !actual
            && a.aabb(&t1)
                .loosened(partition_prediction_distance())
                .intersects(&b.aabb(&t2))
        {
            refined += 1;
            assert!(distance >= partition_prediction_distance() - 1e-8);
        }
        assert_eq!(ba.minimum_z().to_bits(), a.aabb(&t1).mins.z.to_bits());
    }
    assert!(refused > 1500 && refined > 500);
    eprintln!(
        "12000 independently queried oriented box pairs; {refused} within the contact margin; {refined} world AABB false positives separated by native box projections"
    );
}

#[test]
fn sphere_prediction_matches_independent_rotated_box_and_sphere_distances() {
    use rapier3d_f64::parry::{
        query,
        shape::{Ball, Cuboid},
    };
    let mut seed = 0x89dd9d112u64;
    let mut sample = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / (1u64 << 53) as f64
    };
    let mut near = [0; 2];
    let mut separated = [0; 2];
    for i in 0..16000 {
        let a = Vector::from(std::array::from_fn(|_| {
            (sample() - 0.5) * if i % 9 == 0 { 1e8 } else { 20. }
        }));
        let p1 = Pose::from_parts(a, Rotation::from_rotation_z(sample() * 6.));
        let radius = 0.04 + sample() * 0.91;
        let size = std::array::from_fn(|_| 0.10 + sample() * 1.5);
        let rotation = Rotation::from_scaled_axis(Vector::from(std::array::from_fn(|_| {
            (sample() - 0.5) * 6.
        })));
        let p2 = Pose::from_parts(
            a + Vector::from(std::array::from_fn(|_| (sample() - 0.5) * 3.4)),
            rotation,
        );
        let lhs = PartitionBounds::new(
            Shape::Sphere { radius },
            p1.translation.to_array(),
            p1.rotation.to_array(),
        )
        .unwrap();
        let ball = Ball::new(radius);
        let k = i % 2;
        let (geometry, distance) = if k == 0 {
            let shape = Shape::Sphere { radius: size[0] };
            let b = Ball::new(size[0]);
            (
                shape,
                query::distance(&p1, &ball, &p2, &b).unwrap().distance,
            )
        } else {
            let shape = Shape::Box { half_extents: size };
            let b = Cuboid::new(Vector::from(size));
            (
                shape,
                query::distance(&p1, &ball, &p2, &b).unwrap().distance,
            )
        };
        let rhs = PartitionBounds::new(geometry, p2.translation.to_array(), p2.rotation.to_array())
            .unwrap();
        let answer = lhs.predicts_overlap(&rhs);
        assert_eq!(answer, rhs.predicts_overlap(&lhs));
        assert_eq!(lhs.minimum_z(), a.z - radius);
        // A world-coordinate rounding band is intentionally conservative near the margin.
        let band = 1e-5;
        if distance <= partition_prediction_distance() - band {
            assert!(answer, "pair {i}, d={distance}");
            near[k] += 1;
        }
        if distance > partition_prediction_distance() + band {
            assert!(!answer, "pair {i}, d={distance}");
            separated[k] += 1;
        }
    }
    for k in 0..2 {
        assert!(near[k] > 500 && separated[k] > 500);
    }
    eprintln!(
        "16000 sphere-sphere and rotated sphere-box native Parry comparisons, near={near:?}, separated={separated:?}"
    );
    for (radius, center) in [
        (0., [0.; 3]),
        (-1., [0.; 3]),
        (f64::NAN, [0.; 3]),
        (f64::MAX, [f64::MAX, 0., 0.]),
        (1., [0., f64::INFINITY, 0.]),
    ] {
        assert!(PartitionBounds::new(Shape::Sphere { radius }, center, [0., 0., 0., 1.]).is_err());
    }
}

#[test]
fn capsule_prediction_preserves_near_features_across_all_rotated_pairings() {
    use rapier3d_f64::parry::{
        query,
        shape::{Ball, Capsule, Cuboid, Shape as ParryShape},
    };
    let mut seed = 0xb6accedu64;
    let mut next = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut near = [0; 3];
    let mut strict_separations = [0; 3];
    let mut conservative = [0; 3];
    for i in 0..18000 {
        let rotation =
            Rotation::from_scaled_axis(Vector::from(std::array::from_fn(|_| (next() - 0.5) * 6.)));
        let pos = Vector::from(std::array::from_fn(|_| {
            (next() - 0.5) * if i % 7 == 0 { 1e7 } else { 4. }
        }));
        let a = Pose::from_parts(pos, rotation);
        let b = Pose::from_parts(
            pos + Vector::from(std::array::from_fn(|_| (next() - 0.5) * 2.6)),
            if i % 8 == 0 {
                rotation
            } else {
                Rotation::from_scaled_axis(Vector::from(std::array::from_fn(|_| {
                    (next() - 0.5) * 6.
                })))
            },
        );
        let r = next() * 0.42 + 0.05;
        let h = next() * 1.1 + 0.025;
        let c = Capsule::new_z(h, r);
        let lhs = PartitionBounds::new(
            Shape::Capsule {
                radius: r,
                half_length: h,
            },
            pos.to_array(),
            rotation.to_array(),
        )
        .unwrap();
        let r2 = 0.13 + 0.53 * next();
        let h2 = 0.1 + next() * 0.6;
        let k = i % 3;
        let (shape, oracle): (Shape, Box<dyn ParryShape>) = match k {
            0 => (
                Shape::Capsule {
                    radius: r2,
                    half_length: h2,
                },
                Box::new(Capsule::new_z(h2, r2)),
            ),
            1 => (Shape::Sphere { radius: r2 }, Box::new(Ball::new(r2))),
            _ => {
                let half = [r2, r2 * 0.8, h2];
                (
                    Shape::Box { half_extents: half },
                    Box::new(Cuboid::new(Vector::from(half))),
                )
            }
        };
        let rhs =
            PartitionBounds::new(shape, b.translation.to_array(), b.rotation.to_array()).unwrap();
        let overlap = lhs.predicts_overlap(&rhs);
        let native = query::distance(&a, &c, &b, &*oracle).unwrap().distance;
        let margin = partition_prediction_distance();
        if native < margin - 1e-6 {
            near[k] += 1;
            assert!(
                overlap && rhs.predicts_overlap(&lhs),
                "unsafe removal of near feature {i}, distance {native}"
            );
        } else if native > margin + 1e-6 {
            if overlap {
                conservative[k] += 1;
            } else {
                strict_separations[k] += 1;
            }
        }
        let bound = c.compute_aabb(&a);
        assert_eq!(lhs.minimum_z(), bound.mins.z);
    }
    assert!(near.into_iter().all(|n| n > 500));
    assert!(strict_separations.into_iter().all(|n| n > 1000));
    eprintln!(
        "18000 seeded capsule/capsule, capsule/sphere, capsule/box; native near {near:?}, proven clear {strict_separations:?}, conservatively retained {conservative:?}"
    );
}

#[test]
fn tilted_cylinders_compare_to_four_independent_native_shape_distance_queries() {
    use rapier3d_f64::parry::{
        query,
        shape::{Ball, Capsule, Cuboid, Cylinder, Shape as ParryShape},
    };
    let mut seed = 0x95ee42_u64;
    let mut next = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut near = [0; 4];
    let mut clear = [0; 4];
    let mut retained = [0; 4];
    for i in 0..24000 {
        let rotation =
            Rotation::from_scaled_axis(Vector::from(std::array::from_fn(|_| (next() - 0.5) * 5.)));
        let origin = Vector::from(std::array::from_fn(|_| {
            (next() - 0.5) * if i % 13 == 0 { 1e7 } else { 4. }
        }));
        let a = Pose::from_parts(origin, rotation);
        let b = Pose::from_parts(
            origin + Vector::from(std::array::from_fn(|_| (next() - 0.5) * 2.3)),
            if i % 5 == 0 {
                rotation
            } else {
                Rotation::from_scaled_axis(Vector::from(std::array::from_fn(|_| {
                    (next() - 0.5) * 6.
                })))
            },
        );
        let r = 0.075 + next() * 0.6;
        let h = 0.007 + next() * 1.8;
        let r2 = 0.05 + next() * 0.6;
        let h2 = 0.07 + next();
        let lhs = PartitionBounds::new(
            Shape::Cylinder {
                radius: r,
                half_height: h,
            },
            origin.to_array(),
            rotation.to_array(),
        )
        .unwrap();
        let axis_conversion = Pose::from_parts(
            Vector::ZERO,
            Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        let c = Cylinder::new(h, r);
        let k = i % 4;
        let (shape, oracle, pose): (Shape, Box<dyn ParryShape>, Pose) = match k {
            0 => (
                Shape::Cylinder {
                    radius: r2,
                    half_height: h2,
                },
                Box::new(Cylinder::new(h2, r2)),
                b * axis_conversion,
            ),
            1 => (
                Shape::Capsule {
                    radius: r2,
                    half_length: h2,
                },
                Box::new(Capsule::new_z(h2, r2)),
                b,
            ),
            2 => (Shape::Sphere { radius: r2 }, Box::new(Ball::new(r2)), b),
            _ => {
                let half = [r2, 0.65 * r2, h2];
                (
                    Shape::Box { half_extents: half },
                    Box::new(Cuboid::new(Vector::from(half))),
                    b,
                )
            }
        };
        let rhs =
            PartitionBounds::new(shape, b.translation.to_array(), b.rotation.to_array()).unwrap();
        let dist = query::distance(&(a * axis_conversion), &c, &pose, &*oracle)
            .unwrap()
            .distance;
        let margin = partition_prediction_distance();
        let both = [lhs.predicts_overlap(&rhs), rhs.predicts_overlap(&lhs)];
        if dist < margin - 1e-6 {
            assert_eq!(both, [true, true], "missed pair {i} at {dist} m");
            near[k] += 1;
        } else if dist > margin + 1e-6 {
            if both == [false, false] {
                clear[k] += 1;
            } else {
                retained[k] += 1;
            }
        }
        assert!((lhs.minimum_z() - c.compute_aabb(&(a * axis_conversion)).mins.z).abs() < 1e-8);
    }
    assert!(near.into_iter().all(|n| n > 300));
    assert!(clear.into_iter().all(|n| n > 1500));
    eprintln!(
        "24000 rotated cylinder/cylinder, capsule, sphere, box queries: native near {near:?}, separated {clear:?}, conservatively retained {retained:?}"
    );
}

#[test]
fn contact_identity_lookup_handles_slot_reuse_and_internal_roots() {
    let mut s = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let add = |s: &mut RapierBackend, i: u64| {
        s.spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.1; 3],
                },
                position: [i as f64, 0., 0.],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap()
    };
    let initial = (0..24).map(|i| add(&mut s, i)).collect::<Vec<_>>();
    for id in initial.iter().step_by(2) {
        assert!(s.remove(*id));
    }
    let live = (0..12).map(|i| add(&mut s, 55 + i)).collect::<Vec<_>>();
    let (_, hidden) = s
        .world
        .insert(RigidBodyBuilder::fixed(), ColliderBuilder::ball(0.08));
    let index = ContactBodyIndex::default();
    assert_eq!(index.get(&s, hidden), None);
    for id in live.iter().chain(initial.iter().skip(1).step_by(2)) {
        let body = s.bodies[id].0;
        let collider = s.world.bodies[body].colliders()[0];
        assert_eq!(index.get(&s, collider), Some(*id));
    }
    assert!(index.full.get().is_some());
    assert_eq!(index.get(&s, hidden), None);
}
