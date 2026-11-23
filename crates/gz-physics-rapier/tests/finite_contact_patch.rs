// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_rapier::RapierBackend;
use std::{error::Error, time::Duration};

#[test]
fn patch_ids_normals_and_center_levers_follow_both_creation_orders() -> Result<(), Box<dyn Error>> {
    for block_first in [false, true] {
        let mut solver = RapierBackend::new_multifidelity([0., 0., -9.81])?;
        let spec = |dynamic| BodySpec {
            shape: Shape::Box {
                half_extents: if dynamic {
                    [0.4, 0.2, 0.1]
                } else {
                    [3., 1.5, 0.2]
                },
            },
            position: [0., 0., if dynamic { 0.15 } else { -0.2 }],
            velocity: [0.; 3],
            mass: if dynamic { Some(2.) } else { None },
        };
        let a =
            solver.spawn_partition_box(spec(block_first), RotationSpec::default(), false, 0.0)?;
        let b =
            solver.spawn_partition_box(spec(!block_first), RotationSpec::default(), false, 0.0)?;
        assert!(solver.rigid_contact_patches().is_empty());
        for _ in 0..299 {
            solver.step(Duration::from_millis(1))?;
        }
        let dynamic = if block_first { a } else { b };
        use rapier3d_f64::prelude::Vector;
        let velocity = Vector::from(solver.state(dynamic).unwrap().velocity);
        solver.step(Duration::from_millis(1))?;
        let mut patches = solver.rigid_contact_patches();
        assert_eq!(patches.len(), 1);
        let patch = patches.pop().unwrap();
        assert_eq!((patch.a, patch.b), (a, b));
        assert_eq!(patch.points.len(), 4);
        // With zero friction and one finite contact pair its vector impulse
        // must account for the measured change in the dynamic body's COM momentum.
        let normal = Vector::from(patch.normal_on_a) * if block_first { 1. } else { -1. };
        assert!((normal.length_squared() - 1.).abs() < 1e-12);
        assert!(normal.z > 0.99);
        let rigid = solver.rigid_contact_observations();
        assert_eq!(rigid.len(), 1);
        let sum: f64 = patch.points.iter().map(|p| p.normal_impulse).sum();
        assert_eq!(sum.to_bits(), rigid[0].normal_impulse.to_bits());
        assert!((sum - 0.01962).abs() < 5e-6);
        let measured = (Vector::from(solver.state(dynamic).unwrap().velocity)
            - velocity
            - Vector::new(0., 0., -0.00981))
            * 2.;
        assert!(
            (normal * sum - measured).length() < 1e-9,
            "normal={normal:?} J={sum} measured={measured:?}"
        );
        for p in &patch.points {
            let r = if block_first { p.lever_a } else { p.lever_b };
            assert!((r[0].abs() - 0.4).abs() < 1e-4);
            assert!((r[1].abs() - 0.2).abs() < 1e-4);
            assert!((r[2] + 0.1).abs() < 1e-4);
        }
        // A read is nondestructive, including previously solved native point impulses.
        assert_eq!(patch.points, solver.rigid_contact_patches()[0].points);
    }
    Ok(())
}

#[test]
fn pointwise_tangents_close_the_linear_impulse_for_horizontal_and_vertical_contacts()
-> Result<(), Box<dyn Error>> {
    use rapier3d_f64::prelude::Vector;
    let dt = Duration::from_millis(1);
    for n in [Vector::Z, Vector::X] {
        let slide = if n == Vector::Z { Vector::X } else { Vector::Z };
        let gravity = n * -9.81;
        for block_first in [false, true] {
            for point in [false, true] {
                let mut solver = if point {
                    RapierBackend::new_per_point_coulomb(gravity.to_array())?
                } else {
                    RapierBackend::new(gravity.to_array())?
                };
                let mut dynamic = None;
                for block in [block_first, !block_first] {
                    let mut half = if block {
                        [0.4, 0.2, 0.1]
                    } else {
                        [4., 1.5, 0.2]
                    };
                    if n == Vector::X {
                        half.swap(0, 2);
                    }
                    let id = solver.spawn_partition_box(
                        BodySpec {
                            shape: Shape::Box { half_extents: half },
                            mass: block.then_some(2.),
                            position: (n * if block { 0.15 } else { -0.2 }).to_array(),
                            velocity: [0.; 3],
                        },
                        RotationSpec::default(),
                        false,
                        if block { 0.35 } else { 0.65 },
                    )?;
                    if block {
                        dynamic = Some(id);
                    }
                }
                let dynamic = dynamic.unwrap();
                for _ in 0..250 {
                    solver.step(dt)?;
                }
                solver.apply_impulse(dynamic, (slide * 4.8).to_array(), (n * 0.96).to_array())?;
                let v0 = Vector::from(solver.state(dynamic).unwrap().velocity);
                solver.step(dt)?;
                let v1 = Vector::from(solver.state(dynamic).unwrap().velocity);
                let patches = solver.rigid_contact_patches();
                assert_eq!(patches.len(), 1);
                let patch = &patches[0];
                assert_eq!(patch.points.len(), 4);
                let support = if patch.a == dynamic { patch.b } else { patch.a };
                let on_block = solver.contact_warmstart(dynamic, support)?;
                let on_support = solver.contact_warmstart(support, dynamic)?;
                assert_eq!(on_block.points, 4);
                assert_eq!(on_block.normal_ns, on_support.normal_ns);
                assert!(on_block.normal_ns > 0.);
                let friction = Vector::from(on_block.tangent_on_a_ns);
                assert!((friction + Vector::from(on_support.tangent_on_a_ns)).length() < 1e-14);
                assert!(
                    friction.dot(slide) < 0.,
                    "warm friction must oppose sliding"
                );
                assert_eq!(on_block, solver.contact_warmstart(dynamic, support)?);
                let sign = if dynamic == patch.a { 1. } else { -1. };
                let mut total = Vector::ZERO;
                for p in &patch.points {
                    assert_eq!(p.tangent_impulse_on_a.is_some(), point);
                    #[cfg(feature = "partition-checkpoint")]
                    assert_eq!(
                        serde_json::to_value(p)?
                            .get("tangent_impulse_on_a")
                            .is_some(),
                        point
                    );
                    if let Some(t) = p.tangent_impulse_on_a {
                        let t = Vector::from(t);
                        let unit = Vector::from(patch.normal_on_a);
                        assert!(unit.dot(t).abs() < 1e-12);
                        assert!(t.length() <= 0.35 * p.normal_impulse + 1e-11);
                        total += sign * (unit * p.normal_impulse + t);
                    }
                }
                if point {
                    let measured = (v1 - v0 - gravity * dt.as_secs_f64()) * 2.;
                    assert!(
                        (measured - total).length() < 1e-9,
                        "n={n:?} block_first={block_first} measured={measured:?}, points={total:?}"
                    );
                    assert!(total.dot(slide) < -0.001);
                }
            }
        }
    }
    Ok(())
}

#[test]
fn generalized_contact_reports_friction_in_both_public_body_orders() -> Result<(), Box<dyn Error>> {
    use rapier3d_f64::prelude::Vector;

    let dt = Duration::from_micros(500);
    for pad_first in [false, true] {
        let mut solver = RapierBackend::new_multifidelity([0., 0., -9.81])?;
        let cube = |z, half_extents| BodySpec {
            position: [0., 0., z],
            mass: Some(1.),
            velocity: [0.; 3],
            shape: Shape::Box { half_extents },
        };
        let insert_pad = |solver: &mut RapierBackend| {
            solver.spawn_velocity_boundary(
                cube(-0.06, [0.33, 0.18, 0.06]),
                RotationSpec::default(),
                None,
            )
        };
        let pad = pad_first.then(|| insert_pad(&mut solver)).transpose()?;
        let root = solver.spawn_articulation_link(
            cube(0.06, [0.18, 0.10, 0.06]),
            RotationSpec::default(),
            None,
        )?;
        let upper = solver.spawn_articulation_link(
            cube(0.20, [0.05, 0.06, 0.06]),
            RotationSpec::default(),
            None,
        )?;
        solver.add_limited_revolute(Some(root), upper, [0., 0., 0.13], [1., 0., 0.], [0., 0.])?;
        solver.enable_exact_revolute_locks(root)?;
        let pad = match pad {
            Some(id) => id,
            None => insert_pad(&mut solver)?,
        };
        for _ in 0..100 {
            solver.step(dt)?;
        }
        solver.set_world_velocity(pad, [0.4, 0., 0.], [0.; 3])?;
        let link_momentum = |s: &RapierBackend| {
            Vector::from(s.state(root).unwrap().velocity)
                + Vector::from(s.state(upper).unwrap().velocity)
        };
        let before = link_momentum(&solver);
        solver.step(dt)?;
        let expected = link_momentum(&solver) - before + Vector::Z * (2. * 9.81 * dt.as_secs_f64());
        let patches = solver.rigid_contact_patches();
        assert_eq!(patches.len(), 1);
        let patch = &patches[0];
        assert_eq!((patch.a, patch.b), (root.min(pad), root.max(pad)));
        let sign = if pad_first { -1. } else { 1. };
        let normal = Vector::from(patch.normal_on_a) * sign;
        let mut impulse = Vector::ZERO;
        for p in &patch.points {
            let tangent = Vector::from(
                p.tangent_impulse_on_a
                    .expect("two generalized friction rows"),
            ) * sign;
            assert!(tangent.dot(normal).abs() < 1e-12);
            impulse += normal * p.normal_impulse + tangent;
        }
        assert!(
            impulse.x > 0.001,
            "moving pad accelerates the foot: {impulse:?}"
        );
        assert!(
            (expected - impulse).length() < 1e-6,
            "{expected:?} vs {impulse:?}"
        );
    }
    Ok(())
}
