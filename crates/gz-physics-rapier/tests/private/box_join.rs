// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodySpec, BodyState, DynamicBackend, RotationSpec, Shape};
const DT: Duration = Duration::from_millis(10);
fn add_pair(b: &mut RapierBackend, x: f64, rotated: bool, backwards: bool) -> [BodyId; 2] {
    add_material_pair(b, x, rotated, backwards, 0.)
}
fn add_material_pair(
    b: &mut RapierBackend,
    x: f64,
    rotated: bool,
    backwards: bool,
    friction: f64,
) -> [BodyId; 2] {
    let mut result = [BodyId::from_raw(0); 2];
    for index in if backwards { [1, 0] } else { [0, 1] } {
        let base = index == 0;
        let angle = if !base && rotated { 0.43_f64 } else { 0. };
        let state = if base {
            [1.2, 1.6, 0.1]
        } else {
            [0.38, 0.23, 0.35]
        };
        result[index] = b
            .spawn_partition_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: state,
                    },
                    position: [x, 0., if base { 0. } else { 0.52 }],
                    velocity: [0.; 3],
                    mass: (!base).then_some(1.7),
                },
                RotationSpec {
                    orientation: [0., 0., angle.sin(), angle.cos()],
                    angular_velocity: [0.; 3],
                },
                false,
                friction,
            )
            .unwrap();
    }
    result
}
fn step(b: &mut RapierBackend) {
    b.set_partition_gravity([0., 0., -9.81]).unwrap();
    b.step(DT).unwrap();
    b.set_partition_gravity([0.; 3]).unwrap();
}
fn same(a: BodyState, b: BodyState) {
    assert_eq!(a.position.map(f64::to_bits), b.position.map(f64::to_bits));
    assert_eq!(a.velocity.map(f64::to_bits), b.velocity.map(f64::to_bits));
    assert_eq!(
        a.orientation.map(f64::to_bits),
        b.orientation.map(f64::to_bits)
    );
    assert_eq!(
        a.angular_velocity.map(f64::to_bits),
        b.angular_velocity.map(f64::to_bits)
    );
}
#[test]
fn rejoined_independent_supported_boxes_preserve_full_pose_twist_and_native_impulse_bits() {
    for backwards in [false, true] {
        let mut a = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let mut control = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let a_ids = add_pair(&mut a, -8., false, false);
        let b_ids = add_pair(&mut b, 8., true, backwards);
        let control_a = add_pair(&mut control, -8., false, false);
        let control_b = add_pair(&mut control, 8., true, backwards);
        for _ in 0..42 {
            step(&mut a);
            step(&mut b);
            step(&mut control);
        }
        assert!(a.rigid_contact_observations()[0].normal_impulse > 0.1);
        assert!(
            b.partition_warmstart()
                .1
                .iter()
                .any(|&i| f64::from_bits(i) > 0.005)
        );
        let (mut joined, maps) = a.join_box_solvers(&[&b]).unwrap();
        assert!(joined.checkpoint_solver().is_err());
        for tick in 0..120 {
            if tick == 13 {
                for (world, handle) in [
                    (&mut joined, maps[0][&b_ids[1]]),
                    (&mut control, control_b[1]),
                ] {
                    world
                        .apply_impulse(handle, [0.04, -0.15, 0.03], [0.2, -0.12, 0.23])
                        .unwrap();
                }
            }
            step(&mut joined);
            step(&mut control);
            for (j, c) in a_ids
                .into_iter()
                .zip(control_a)
                .chain(b_ids.map(|id| maps[0][&id]).into_iter().zip(control_b))
            {
                same(joined.state(j).unwrap(), control.state(c).unwrap());
            }
            let sj = joined.rigid_contact_observations();
            let sc = control.rigid_contact_observations();
            assert_eq!(sj.len(), sc.len());
            for (j, c) in sj.iter().zip(sc) {
                assert_eq!(j.active, c.active);
                assert_eq!(
                    j.normal_impulse.to_bits(),
                    c.normal_impulse.to_bits(),
                    "tick {tick}"
                );
            }
            if tick == 3 {
                joined =
                    RapierBackend::restore_trusted_solver(&joined.checkpoint_solver().unwrap())
                        .unwrap();
            }
        }
    }
}

#[test]
fn two_independent_donors_keep_shear_warmstarts_and_rotation_after_join_and_recovery() {
    let field = |w: &mut RapierBackend| {
        w.set_partition_gravity([1.7, -0.9, -9.81]).unwrap();
        w.step(DT).unwrap();
        w.set_partition_gravity([0.; 3]).unwrap();
    };
    let mut worlds: [RapierBackend; 3] =
        std::array::from_fn(|_| RapierBackend::new_multifidelity([0.; 3]).unwrap());
    let mut native = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let positions = [-7., 0., 7.];
    let mu = [0.8, 0.57, 0.61];
    let sources: [_; 3] = std::array::from_fn(|i| {
        add_material_pair(&mut worlds[i], positions[i], i > 0, i == 2, mu[i])
    });
    let direct: [_; 3] =
        std::array::from_fn(|i| add_material_pair(&mut native, positions[i], i > 0, i == 2, mu[i]));
    for _ in 0..55 {
        for world in &mut worlds {
            field(world);
        }
        field(&mut native);
    }
    for world in &worlds {
        assert!(
            world
                .world
                .narrow_phase
                .contact_pairs()
                .flat_map(|p| p.manifolds())
                .flat_map(|m| &m.points)
                .any(|p| p.data.warmstart_tangent_world.length() > 1e-4),
            "donor has measurable shear history"
        );
    }
    let (mut joined, maps) = worlds[0]
        .join_box_solvers(&[&worlds[1], &worlds[2]])
        .unwrap();
    let handles = [
        sources[0],
        sources[1].map(|id| maps[0][&id]),
        sources[2].map(|id| maps[1][&id]),
    ];
    for tick in 1..=140 {
        if tick == 25 || tick == 85 {
            for i in 0..3 {
                for (world, id) in [(&mut joined, handles[i][1]), (&mut native, direct[i][1])] {
                    world
                        .apply_impulse(id, [0.09, -0.13, 0.02], [0.04, -0.015, 0.09])
                        .unwrap();
                }
            }
        }
        field(&mut joined);
        field(&mut native);
        for i in 0..3 {
            for j in 0..2 {
                same(
                    joined.state(handles[i][j]).unwrap(),
                    native.state(direct[i][j]).unwrap(),
                );
            }
        }
        let get_shear = |world: &RapierBackend| {
            let mut data = world
                .world
                .narrow_phase
                .contact_pairs()
                .flat_map(|p| p.manifolds().iter().flat_map(|m| &m.points))
                .map(|p| {
                    (
                        p.data.impulse.to_bits(),
                        p.data.warmstart_tangent_world.to_array().map(f64::to_bits),
                        p.data.warmstart_impulse.to_bits(),
                        p.data.warmstart_twist_impulse.to_bits(),
                    )
                })
                .collect::<Vec<_>>();
            data.sort();
            data
        };
        assert_eq!(
            get_shear(&joined),
            get_shear(&native),
            "contact shear history on tick {tick}"
        );
        if tick == 2 || tick == 86 {
            joined = RapierBackend::restore_trusted_solver(&joined.checkpoint_solver().unwrap())
                .unwrap();
        }
    }
}

#[test]
fn invalid_cross_source_overlap_or_step_is_refused_without_changing_the_inputs() {
    let mut a = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    add_pair(&mut a, 0., false, false);
    add_pair(&mut b, 0.4, true, false);
    for _ in 0..23 {
        step(&mut a);
        step(&mut b);
    }
    let before = [
        a.checkpoint_solver().unwrap(),
        b.checkpoint_solver().unwrap(),
    ];
    assert!(a.join_box_solvers(&[&b]).is_err());
    assert!(a.join_box_solvers(&[]).is_err());
    assert_eq!(
        before,
        [
            a.checkpoint_solver().unwrap(),
            b.checkpoint_solver().unwrap()
        ]
    );
    let mut c = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    c.set_partition_step(Duration::from_millis(2)).unwrap();
    add_pair(&mut c, 8., false, false);
    assert!(a.join_box_solvers(&[&c]).is_err());
}

#[test]
fn warm_dispatcher_forwards_public_queries_during_first_step_of_a_join() {
    use rapier3d_f64::parry::{
        query::{self as q, PersistentQueryDispatcher, QueryDispatcher},
        shape::{Capsule, Cuboid},
    };
    let saved = crate::contact_dispatcher::NativeContacts(RestorePairs(std::sync::Mutex::new(
        Cache::new(),
    )));
    let direct = q::DefaultQueryDispatcher;
    let (a, b) = (
        Capsule::new_z(0.35, 0.2),
        Cuboid::new(Vector::new(0.4, 0.3, 0.7)),
    );
    let v = Vector::new(-1., 0.01, 0.);
    for x in [0.3, 0.69, 1.35, 2.8] {
        let pose = Pose::from_parts(Vector::new(x, -0.08, 0.02), Rotation::from_rotation_y(0.17));
        assert_eq!(
            saved.intersection_test(&pose, &a, &b).unwrap(),
            direct.intersection_test(&pose, &a, &b).unwrap()
        );
        assert_eq!(
            saved.distance(&pose, &a, &b).unwrap(),
            direct.distance(&pose, &a, &b).unwrap()
        );
        let contact =
            |c: Option<q::Contact>| c.map(|c| (c.point1, c.point2, c.normal1, c.normal2, c.dist));
        assert_eq!(
            contact(saved.contact(&pose, &a, &b, 0.4).unwrap()),
            contact(direct.contact(&pose, &a, &b, 0.4).unwrap())
        );
        assert_eq!(
            saved.closest_points(&pose, &a, &b, 0.4).unwrap(),
            direct.closest_points(&pose, &a, &b, 0.4).unwrap()
        );
        let hit = |h: Option<q::ShapeCastHit>| {
            h.map(|h| {
                (
                    h.time_of_impact,
                    h.witness1,
                    h.witness2,
                    h.normal1,
                    h.normal2,
                    h.status,
                )
            })
        };
        let opts = q::ShapeCastOptions::with_max_time_of_impact(1.);
        assert_eq!(
            hit(saved.cast_shapes(&pose, v, &a, &b, opts).unwrap()),
            hit(direct.cast_shapes(&pose, v, &a, &b, opts).unwrap())
        );
        let stationary = q::NonlinearRigidMotion::constant_position(Pose::IDENTITY);
        let movement =
            q::NonlinearRigidMotion::new(pose, Vector::ZERO, v, Vector::new(0., 0.12, -0.03));
        assert_eq!(
            hit(saved
                .cast_shapes_nonlinear(&stationary, &a, &movement, &b, 0., 1., true)
                .unwrap()),
            hit(direct
                .cast_shapes_nonlinear(&stationary, &a, &movement, &b, 0., 1., true)
                .unwrap())
        );
        let mut m = ContactManifold::new();
        let mut n = ContactManifold::new();
        saved
            .contact_manifold_convex_convex(&pose, &a, &b, None, None, 0.4, &mut m)
            .unwrap();
        direct
            .contact_manifold_convex_convex(&pose, &a, &b, None, None, 0.4, &mut n)
            .unwrap();
        assert_eq!(
            (m.local_n1, m.local_n2, m.points.len()),
            (n.local_n1, n.local_n2, n.points.len())
        );
        for (p, q) in m.points.iter().zip(n.points.iter()) {
            assert_eq!(
                (p.local_p1, p.local_p2, p.fid1, p.fid2),
                (q.local_p1, q.local_p2, q.fid1, q.fid2)
            );
        }
    }
}
