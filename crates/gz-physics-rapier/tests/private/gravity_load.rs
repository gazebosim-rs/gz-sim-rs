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
    assert!(b.remove_joint(edge));
    assert_eq!(load(&b).force_n, [0., 0., -20.]);
    weld(&mut b, Some(wrist), payload);
    assert!(b.remove(payload));
    assert_eq!(load(&b).force_n, [0., 0., -20.]);
    b.world.gravity = Vector::splat(f64::MAX);
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
