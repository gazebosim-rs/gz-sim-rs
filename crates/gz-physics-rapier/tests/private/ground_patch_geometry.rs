// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn shared_plane_patches_select_finite_ids_and_signed_world_witnesses_in_both_collider_orders() {
    let n = Vector::new(0., -0.6, 0.8);
    let origin = Vector::new(3., -2., 1.);
    let q = Rotation::from_rotation_x(0.6_f64.atan2(0.8));
    let center = origin + q * Vector::new(1.2, -0.8, 0.15);
    let mut seen = std::collections::BTreeSet::new();
    for ground_first in [true, false] {
        let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let plane = NativeGroundPlane::new(origin.to_array(), n.to_array(), 0.6).unwrap();
        if ground_first {
            backend.world.colliders.insert(collider_at(plane));
        }
        let body = backend
            .spawn_partition_primitive(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.3, 0.2, 0.1],
                    },
                    position: center.to_array(),
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                RotationSpec {
                    orientation: [q.x, q.y, q.z, q.w],
                    angular_velocity: [0.; 3],
                },
                false,
                0.35,
            )
            .unwrap();
        assert!(backend.partition_ground_patches().is_empty());
        if !ground_first {
            backend.world.colliders.insert(collider_at(plane));
        }
        backend
            .set_partition_gravity((-9.81 * n).to_array())
            .unwrap();
        for _ in 0..400 {
            backend.step(Duration::from_millis(1)).unwrap();
        }
        let ground = backend.ground_collider().unwrap();
        let other = backend.world.bodies[backend.bodies[&body].0].colliders()[0];
        let pair = backend
            .world
            .narrow_phase
            .contact_pair(ground, other)
            .unwrap();
        seen.insert(pair.collider1 == ground);
        let rows = backend.partition_ground_patches();
        assert_eq!(rows.len(), 1);
        let patch = &rows[0];
        assert_eq!(patch.body, body);
        assert!((Vector::from(patch.normal_on_body) - n).length() < 1e-12);
        assert_eq!(patch.points.len(), 4);
        let j = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
        assert_eq!(
            backend.partition_ground_contacts()[0]
                .normal_impulse
                .to_bits(),
            j.to_bits()
        );
        assert!((j - 2. * 9.81 * 0.001).abs() < 2e-5);
        let body_position = Vector::from(backend.state(body).unwrap().position);
        let mut corners = std::collections::BTreeSet::new();
        for p in &patch.points {
            assert!(p.normal_impulse > 0.);
            let witness = Vector::from(p.plane_witness);
            let solver = Vector::from(p.solver_point);
            let lever = Vector::from(p.body_lever);
            assert!((witness - origin).dot(n).abs() < 1e-12);
            assert!((solver - witness).cross(n).length() < 1e-10);
            assert!((solver - lever - body_position).length() < 2e-5);
            let local = q.inverse() * (witness - center);
            corners.insert((local.x.is_sign_positive(), local.y.is_sign_positive()));
            assert!((local.x.abs() - 0.3).abs() < 1e-4 && (local.y.abs() - 0.2).abs() < 1e-4);
        }
        assert_eq!(corners.len(), 4);
        backend.set_partition_gravity([0.; 3]).unwrap();
        let restored =
            RapierBackend::restore_trusted_solver(&backend.checkpoint_solver().unwrap()).unwrap();
        assert_eq!(restored.partition_ground_patches(), rows);
    }
    assert_eq!(
        seen.len(),
        2,
        "both actual native pair orderings are required"
    );
}

#[test]
fn rocking_cuboid_selects_changing_ground_rows_and_closes_vertical_impulse() {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let n = Vector::Z;
    let origin = [-1.7, 0.9, -0.65];
    let g = NativeGroundPlane::new(origin, n.to_array(), 0.65).unwrap();
    backend.world.colliders.insert(collider_at(g));
    // The first touchdown is a single inclined corner. The settling box tips
    // onto changing edges and then a full face under its own weight.
    let q = Rotation::from_rotation_y(0.21) * Rotation::from_rotation_x(-0.16);
    let mass = 2.;
    let id = backend
        .spawn_partition_primitive(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.4, 0.2, 0.1],
                },
                position: [1.4, -0.7, 0.],
                velocity: [0.; 3],
                mass: Some(mass),
            },
            RotationSpec {
                orientation: [q.x, q.y, q.z, q.w],
                angular_velocity: [0.; 3],
            },
            false,
            0.35,
        )
        .unwrap();
    backend.set_partition_gravity([0., 0., -9.81]).unwrap();
    let handle = backend.ground_collider().unwrap();
    let finite = backend.world.bodies[backend.bodies[&id].0].colliders()[0];
    let mut counts = std::collections::BTreeSet::new();
    let mut native_total = 0.;
    let mut velocity = 0.;
    let mut max_delta = 0_f64;
    for _ in 0..1200 {
        backend.step(Duration::from_millis(1)).unwrap();
        let rows = backend.partition_ground_patches();
        let mut normal = 0.;
        let mut count = 0;
        for row in rows {
            assert_eq!(row.body, id);
            assert_eq!(row.normal_on_body, n.to_array());
            for p in row.points {
                assert!((p.plane_witness[2] - origin[2]).abs() < 1e-12);
                assert!(p.normal_impulse >= 0.);
                normal += p.normal_impulse;
                count += 1;
            }
        }
        counts.insert(count);
        native_total += normal;
        let v = backend.state(id).unwrap().velocity[2];
        let expected = mass * (v - velocity + 9.81 * 0.001);
        velocity = v;
        max_delta = max_delta.max((normal - expected).abs());
        let summed = backend
            .partition_ground_contacts()
            .into_iter()
            .filter(|r| r.active)
            .fold(0.0, |sum, row| sum + row.normal_impulse);
        assert_eq!(normal.to_bits(), summed.to_bits());
    }
    eprintln!(
        "solved-point counts {counts:?}; maximum single-step vertical impulse residual {max_delta} N s; integrated residual {} N s",
        (native_total - mass * (velocity + 9.81 * 1.2)).abs()
    );
    assert!(
        counts == [0, 1, 2, 3, 4].into_iter().collect(),
        "observed counts {counts:?}"
    );
    // The trajectory exercised every face point count, but the resting
    // native solver selected all four of its retained geometric points.
    // Remove a nonterminal constraint from a copy to model a retained warm
    // impulse with no corresponding solver row and verify the shared reader
    // respects native contact IDs even when the row offsets shift.
    let pair = backend
        .world
        .narrow_phase
        .contact_pair(handle, finite)
        .unwrap();
    let mut partial = pair.solver_manifolds()[0].clone();
    let excluded = (partial.data.solver_contacts.remove(1).contact_id[0]
        & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize;
    partial.points[excluded].data.impulse = 1e6;
    let indices = crate::partition::solved_point_indices(&partial).collect::<Vec<_>>();
    assert_eq!(indices.len(), 3);
    assert!(!indices.contains(&excluded));
    assert!(
        indices
            .iter()
            .all(|&i| partial.points[i].data.impulse < 0.1)
    );
    assert!(
        max_delta < 1e-9,
        "single-step linear momentum residual {max_delta}"
    );
    assert!((native_total - mass * (velocity + 9.81 * 1.2)).abs() < 1e-8);
}

#[test]
fn smooth_inclined_plane_resolves_weight_normal_and_free_tangent_acceleration() {
    // Gravity here remains upright in the lab. A smooth sphere's tangent
    // velocity must integrate the unopposed component of gravity even on impact.
    let n = Vector::new(0., 0.6, 0.8);
    let g = Vector::new(0., 0., -9.81);
    let tangent_gravity = g - n * g.dot(n);
    let origin = Vector::new(-1.5, 2.25, 1.3);
    let mass = 3.;
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    backend
        .set_partition_plane(origin.to_array(), n.to_array(), 0.)
        .unwrap();
    let id = backend
        .spawn_partition_primitive(
            BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: (origin + 0.35 * n).to_array(),
                velocity: [0.; 3],
                mass: Some(mass),
            },
            RotationSpec::default(),
            false,
            0.,
        )
        .unwrap();
    backend.set_partition_gravity(g.to_array()).unwrap();
    let mut steady_ticks = 0;
    for tick in 1..=800 {
        backend.step(Duration::from_millis(1)).unwrap();
        let v = Vector::from(backend.state(id).unwrap().velocity);
        let tangent_v = v - n * v.dot(n);
        assert!((tangent_v - tangent_gravity * (f64::from(tick) * 0.001)).length() < 1e-10);
        if tick < 400 {
            continue;
        }
        let rows = backend.partition_ground_patches();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].body, id);
        assert!((Vector::from(rows[0].normal_on_body) - n).length() < 1e-12);
        assert_eq!(rows[0].points.len(), 1);
        let p = &rows[0].points[0];
        assert!((Vector::from(p.plane_witness) - origin).dot(n).abs() < 1e-12);
        assert!((p.normal_impulse / (-mass * g.dot(n) * 0.001) - 1.).abs() < 1e-3);
        steady_ticks += 1;
    }
    assert_eq!(steady_ticks, 401);
}
