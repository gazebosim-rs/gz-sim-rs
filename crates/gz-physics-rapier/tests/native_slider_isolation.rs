// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use rapier3d_f64::prelude::*;

#[test]
fn an_unmarked_native_coupled_motor_still_reaches_its_ordinary_stop() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = 0.001;
    let base = world.bodies.insert(RigidBodyBuilder::fixed());
    let body = |world: &mut PhysicsWorld| {
        world
            .insert(
                RigidBodyBuilder::dynamic(),
                ColliderBuilder::ball(0.02).mass(1.).sensor(true),
            )
            .0
    };
    let driver = body(&mut world);
    let follower = body(&mut world);
    let drive = world
        .insert_multibody_joint(
            base,
            driver,
            PrismaticJointBuilder::new(Vector::X)
                .limits([-0.015, 0.025])
                .motor_velocity(0.1, 100.)
                .motor_max_force(5.),
        )
        .unwrap();
    let gear = world
        .insert_multibody_joint(driver, follower, PrismaticJointBuilder::new(Vector::X))
        .unwrap();
    let (_, id2) = world.multibody_joints.get(gear).unwrap();
    let (tree, id1) = world.multibody_joints.get_mut(drive).unwrap();
    tree.frictions_mut().fill(0.01);
    tree.add_dof_coupling(MultibodyDofCoupling {
        link1: id1,
        dof1: 0,
        axis1: 0,
        link2: id2,
        dof2: 0,
        axis2: 0,
        coeff: 0.5,
        offset: 0.,
    });
    for tick in 1..=700 {
        world.step();
        if tick == 100 || tick == 700 {
            let (tree, _) = world.multibody_joints.get(drive).unwrap();
            let q = tree.link(id1).unwrap().joint().coords()[0];
            let coupled = tree.link(id2).unwrap().joint().coords()[0];
            assert!((coupled - 0.5 * q).abs() < 1e-6, "{q} {coupled}");
            if tick == 100 {
                assert!((0.001..0.015).contains(&q), "{q}");
            } else {
                assert!((q - 0.025).abs() < 1e-5, "{q}");
            }
        }
    }
}
