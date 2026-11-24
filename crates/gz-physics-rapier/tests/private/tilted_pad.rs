// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn unloaded_box_corners_keep_their_share_of_the_pad_material() {
    // A wide flat jaw contacts the near edge of a smaller, tilted jaw. Only
    // their X translations are free. The far two corners carry no load.
    let stiffness = 30_000.;
    let dt = Duration::from_micros(100);
    let (hx, hz) = (0.02, 0.08);
    const FORCE: f64 = 3.;
    for (angle, use_hook, selected, expected_depth) in [
        // The aligned crossed strips clip at four edge intersections.
        (0.0_f64, false, 4, FORCE / stiffness),
        (0.09, false, 4, 2. * FORCE / stiffness),
        (0.125, false, 4, 2. * FORCE / stiffness),
        (0.128, false, 2, 2. * FORCE / stiffness),
        (0.19, false, 2, 2. * FORCE / stiffness),
        (0.19, true, 2, FORCE / stiffness),
    ] {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let width = hx * angle.cos() + hz * angle.sin();
        let left = b
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [hx, 0.04, if angle == 0. { 0.01 } else { 0.16 }],
                    },
                    position: [-hx, 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(0.08),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let right = b
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [hx, 0.02, hz],
                    },
                    position: [width, 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(0.2),
                },
                RotationSpec {
                    orientation: [0., (angle * 0.5).sin(), 0., (angle * 0.5).cos()],
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap();
        let mut slides = Vec::new();
        for (id, x, axis) in [(left, -hx, 1.), (right, width, -1.)] {
            slides.push(
                b.add_reduced_scalar_robot_joint(
                    None,
                    id,
                    [x, 0., 0.],
                    RobotJoint::Prismatic {
                        axis: [axis, 0., 0.],
                        limits: Some([-0.01, 0.01]),
                    },
                )
                .unwrap(),
            );
        }
        let native = b.bodies[&left].0;
        let pad_collider = b.world.bodies[native].colliders()[0];
        b.world.colliders[pad_collider].set_friction(0.);
        let right_collider = b.world.bodies[b.bodies[&right].0].colliders()[0];
        b.world.colliders[right_collider].set_friction(0.);
        // The additional collision is outside the rocking pair. Its active
        // hooks must not change which law the two contacting boxes use.
        b.world.colliders.insert_with_parent(
            ColliderBuilder::ball(0.001)
                .density(0.)
                .translation(Vector::Y)
                .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
                .build(),
            native,
            &mut b.world.bodies,
        );
        if use_hook {
            b.world.colliders[pad_collider].set_active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);
        }
        b.set_initial_pad_stiffness(left, stiffness).unwrap();
        for _ in 0..5000 {
            for &joint in &slides {
                b.add_joint_effort(joint, FORCE).unwrap();
            }
            b.step(dt).unwrap();
        }
        let patches = b.rigid_contact_patches();
        let points = &patches[0].points;
        let depth =
            hx + width - (b.state(right).unwrap().position[0] - b.state(left).unwrap().position[0]);
        let active = points.iter().filter(|p| p.normal_impulse > 0.).count();
        eprintln!(
            "angle={angle:.3} selected={} loaded={active} depth_um={:.6}",
            points.len(),
            depth * 1e6
        );
        assert_eq!(points.len(), selected);
        assert_eq!(active, if angle == 0. { 4 } else { 2 });
        let patch = &patches[0];
        assert_eq!((patch.a, patch.b), (left, right));
        let (mut reaction, mut moment_y) = (0., 0.);
        for point in points {
            let fx = -patch.normal_on_a[0] * point.normal_impulse / dt.as_secs_f64();
            reaction += fx;
            moment_y += point.lever_b[2] * fx;
        }
        assert!((reaction - FORCE).abs() < 1e-6);
        let expected_moment = if angle == 0. {
            0.
        } else {
            FORCE * (hx * angle.sin() - hz * angle.cos())
        };
        assert!((moment_y - expected_moment).abs() < 1e-6);
        assert!((depth - expected_depth).abs() < 5e-7, "{depth}");
    }
}

#[test]
fn moving_the_frictionless_plane_tangentially_keeps_the_corner_lever() {
    const FORCE: f64 = 3.;
    let angle = 0.09_f64;
    let (hx, hz) = (0.02, 0.08);
    let width = hx * angle.cos() + hz * angle.sin();
    let z = 0.75;
    let dt = Duration::from_micros(100);
    for wall_first in [true, false] {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let mut spawn = |is_wall| {
            let (x, half_extents, rotation, mass) = if is_wall {
                (-hx, [hx, 0.04, 0.16], RotationSpec::default(), None)
            } else {
                (
                    width,
                    [hx, 0.02, hz],
                    RotationSpec {
                        orientation: [0., (angle * 0.5).sin(), 0., (angle * 0.5).cos()],
                        ..RotationSpec::default()
                    },
                    Some(0.2),
                )
            };
            let id = b
                .spawn_link(
                    BodySpec {
                        shape: Shape::Box { half_extents },
                        position: [x, 0., z],
                        velocity: [0.; 3],
                        mass,
                    },
                    rotation,
                    None,
                )
                .unwrap();
            let collider = b.world.bodies[b.bodies[&id].0].colliders()[0];
            b.world.colliders[collider].set_friction(0.);
            id
        };
        let first = spawn(wall_first);
        let second = spawn(!wall_first);
        let (wall, pad) = if wall_first {
            (first, second)
        } else {
            (second, first)
        };
        let joint = b
            .add_reduced_scalar_robot_joint(
                None,
                pad,
                [width, 0., z],
                RobotJoint::Prismatic {
                    axis: [-1., 0., 0.],
                    limits: Some([-0.01, 0.01]),
                },
            )
            .unwrap();
        b.set_initial_pad_stiffness(pad, 30_000.).unwrap();
        for _ in 0..5000 {
            b.add_joint_effort(joint, FORCE).unwrap();
            b.step(dt).unwrap();
        }
        // Both offsets are well inside the plane's footprint. Sliding the
        // plane along itself changes its cached witness but never moves the
        // tilted pad's bearing vertices on the plane.
        for offset in [0.0, 0.0002, -0.0003] {
            let rigid = &mut b.world.bodies[b.bodies[&wall].0];
            let mut pose = *rigid.position();
            pose.translation.z = z + offset;
            rigid.set_position(pose, true);
            b.add_joint_effort(joint, FORCE).unwrap();
            b.step(dt).unwrap();
            let patches = b.rigid_contact_patches();
            let patch = &patches[0];
            let expected_z = hx * angle.sin() - hz * angle.cos();
            let mut force = 0.;
            for p in patch.points.iter().filter(|p| p.normal_impulse > 0.) {
                let lever_z = if patch.a == pad {
                    p.lever_a[2]
                } else {
                    p.lever_b[2]
                };
                assert!(
                    (lever_z - expected_z).abs() < 1e-8,
                    "offset={offset} measured={lever_z} expected={expected_z}"
                );
                force += p.normal_impulse / dt.as_secs_f64();
            }
            assert!((force - FORCE).abs() < 1e-3);
        }
    }
}
