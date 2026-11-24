// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn body(b: &mut RapierBackend, p: [f64; 3], mass: f64) -> BodyId {
    b.spawn(BodySpec {
        shape: Shape::Sphere { radius: 0.01 },
        position: p,
        mass: Some(mass),
        velocity: [0.; 3],
    })
    .unwrap()
}

fn weld(b: &mut RapierBackend, parent: Option<BodyId>, child: BodyId) -> JointId {
    b.add_robot_joint(parent, child, [0.; 3], RobotJoint::Fixed)
        .unwrap()
}

#[test]
fn a_floating_branch_tracks_only_its_descendants_actual_masses_and_gravity_scales() {
    let mut b = RapierBackend::new([0., 0., -10.]).unwrap();
    let root = body(&mut b, [0.; 3], 1.);
    let wrist = body(&mut b, [0.1, 0.4, 0.2], 2.);
    let bearing = weld(&mut b, Some(root), wrist);
    let load = |b: &RapierBackend| b.joint_gravity_load(bearing).unwrap();
    assert_eq!(load(&b).force_n, [0., 0., -20.]);
    let sibling = body(&mut b, [3., 1., 0.], 7.);
    weld(&mut b, Some(root), sibling);
    let payload = body(&mut b, [0.5, -0.2, 0.], 4.);
    let edge = weld(&mut b, Some(wrist), payload);
    b.world.bodies[b.bodies[&payload].0].set_gravity_scale(0.25, true);
    assert_eq!(load(&b).force_n, [0., 0., -30.]);
    assert!(Vector::from_array(load(&b).torque_nm).distance(Vector::new(-6., 7., 0.)) < 1e-12);
    let composite = b.locked_subtree_inertia(bearing).unwrap();
    assert_eq!(composite.mass_kg, 6.);
    let center = (Vector::new(0.1, 0.4, 0.2) * 2. + Vector::new(0.5, -0.2, 0.) * 4.) / 6.;
    assert!(center.distance(composite.center_of_mass_m.into()) < 1e-12);
    assert!(b.remove_joint(edge));
    assert_eq!(b.locked_subtree_inertia(bearing).unwrap().mass_kg, 2.);
    assert_eq!(load(&b).force_n, [0., 0., -20.]);
    weld(&mut b, Some(wrist), payload);
    assert!(b.remove(payload));
    assert_eq!(load(&b).force_n, [0., 0., -20.]);
    let handle = b.bodies[&wrist].0;
    let before = b.locked_subtree_inertia(bearing).unwrap();
    b.world.gravity = Vector::splat(f64::MAX);
    assert_eq!(b.locked_subtree_inertia(bearing).unwrap(), before);
    assert!(matches!(
        b.locked_subtree_inertia(edge),
        Err(PhysicsError::MissingJoint)
    ));
    b.world.bodies[handle].set_translation(Vector::splat(1e200), true);
    assert!(matches!(
        b.locked_subtree_inertia(bearing),
        Err(PhysicsError::InvalidValue)
    ));
    assert!(matches!(
        b.joint_gravity_load(bearing),
        Err(PhysicsError::InvalidValue)
    ));
    assert!(matches!(
        b.joint_gravity_load(edge),
        Err(PhysicsError::MissingJoint)
    ));
}

#[test]
fn ambiguous_or_external_edges_reject_the_connected_robot_but_leave_its_neighbor_readable() {
    for mode in 0..4 {
        let mut b = RapierBackend::new([0., 0., -10.]).unwrap();
        let root = body(&mut b, [0.; 3], 1.);
        let wrist = body(&mut b, [0.1, 0., 0.], 2.);
        let distal = body(&mut b, [0.2, 0., 0.], 3.);
        let independent = body(&mut b, [1., 0., 0.], 4.);
        let other = weld(&mut b, None, independent);
        let j = weld(&mut b, Some(root), wrist);
        weld(&mut b, Some(wrist), distal);
        assert_eq!(b.joint_gravity_load(j).unwrap().force_n[2], -50.);
        let bad = match mode {
            0 => weld(&mut b, Some(root), distal),  // duplicate parent
            1 => weld(&mut b, Some(distal), root),  // rootless cycle
            2 => weld(&mut b, Some(distal), wrist), // reachable cycle
            _ => {
                let external = body(&mut b, [0.3, 0., 0.], 2.);
                b.add_revolute(distal, external, [0.; 3], [0., 1., 0.])
                    .unwrap()
            }
        };
        assert!(matches!(
            b.joint_gravity_load(j),
            Err(PhysicsError::InvalidJoint)
        ));
        assert_eq!(b.joint_gravity_load(other).unwrap().force_n[2], -40.);
        assert!(b.remove_joint(bad));
        assert_eq!(b.joint_gravity_load(j).unwrap().force_n[2], -50.);
    }
}

#[test]
fn moving_descendants_contribute_independent_spin_translation_and_work() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let carrier = body(&mut b, [0.; 3], 9.);
    let arm = body(&mut b, [1., 0., 0.], 2.);
    let sphere = body(&mut b, [1., 1., 0.], 3.);
    let sibling = body(&mut b, [0., 0., 2.], 7.);
    let root = b
        .add_robot_joint(
            Some(carrier),
            arm,
            [0.; 3],
            RobotJoint::Ball {
                orientation: [0., 0., 0., 1.],
            },
        )
        .unwrap();
    b.add_robot_joint(
        Some(arm),
        sphere,
        [1., 0., 0.],
        RobotJoint::Ball {
            orientation: [0., 0., 0., 1.],
        },
    )
    .unwrap();
    weld(&mut b, Some(carrier), sibling);
    for (id, linear, angular) in [
        (arm, [0., 1., 0.], [0., 0., 1.]),
        (sphere, [-2., 1., 0.], [0., 0., 2.]),
        (sibling, [7., -9., 8.], [1., 2., 3.]),
    ] {
        let rb = &mut b.world.bodies[b.bodies[&id].0];
        rb.set_linvel(linear.into(), true);
        rb.set_angvel(angular.into(), true);
    }
    let at = b.subtree_momentum_about(root, [0.; 3]).unwrap();
    assert_eq!(at.linear_kg_m_s, [-6., 5., 0.]);
    // Solid spheres, both of radius 1 cm. Their child COM velocities are
    // consistent with a 1 rad/s root and 2 rad/s absolute distal spin.
    let (i1, i2) = (0.4 * 2. * 0.01_f64.powi(2), 0.4 * 3. * 0.01_f64.powi(2));
    assert!((at.angular_kg_m2_s[2] - (11. + i1 + 2. * i2)).abs() < 1e-12);
    assert!((at.kinetic_energy_j - (8.5 + 0.5 * i1 + 2. * i2)).abs() < 1e-12);
    let point = Vector::new(3., -4., 5.);
    let other = b.subtree_momentum_about(root, point.to_array()).unwrap();
    let expected = Vector::from(at.angular_kg_m2_s) - point.cross(at.linear_kg_m_s.into());
    assert!(expected.distance(other.angular_kg_m2_s.into()) < 1e-12);
    assert_eq!(at.kinetic_energy_j, other.kinetic_energy_j);
    assert!(matches!(
        b.subtree_momentum_about(root, [f64::NAN; 3]),
        Err(PhysicsError::InvalidValue)
    ));
    b.world.bodies[b.bodies[&sphere].0].set_angvel(Vector::splat(1e250), true);
    assert!(matches!(
        b.subtree_momentum_about(root, [0.; 3]),
        Err(PhysicsError::InvalidValue)
    ));
    assert!(b.remove_joint(root));
    assert!(matches!(
        b.subtree_momentum_about(root, [0.; 3]),
        Err(PhysicsError::MissingJoint)
    ));
}
