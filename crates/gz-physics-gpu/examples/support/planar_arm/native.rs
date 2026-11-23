// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{angles, servo};
use gz_physics_gpu::PlanarArmLane as Lane;
use rapier3d_f64::prelude::*;

pub struct Native {
    pub(super) world: PhysicsWorld,
    pub(super) links: [RigidBodyHandle; 2],
}
impl Native {
    pub fn new(l: &Lane) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -l.gravity);
        world.integration_parameters.dt = 0.001 / 8.;
        let fixed = world.insert_body(RigidBodyBuilder::fixed());
        let q = angles(l);
        let points = l.points();
        let total = l.mass[1] + l.tip_mass;
        let c = (l.mass[1] * l.center[1] + l.tip_mass * l.length[1]) / total;
        let inertia = l.inertia[1]
            + l.mass[1] * (l.center[1] - c).powi(2)
            + l.tip_mass * (l.length[1] - c).powi(2);
        let body = |w: &mut PhysicsWorld, x: f64, z: f64, angle: f64, m: f64, com: f64, i: f64| {
            w.insert_body(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x, 0., z))
                    .rotation(Vector::new(0., -angle, 0.))
                    .additional_mass_properties(MassProperties::new(
                        Vector::new(com, 0., 0.),
                        m,
                        Vector::splat(i),
                    ))
                    .can_sleep(false),
            )
        };
        let a = body(
            &mut world,
            0.,
            0.,
            q[0],
            l.mass[0],
            l.center[0],
            l.inertia[0],
        );
        let b = body(
            &mut world,
            points[0][0],
            points[0][1],
            q[0] + q[1],
            total,
            c,
            inertia,
        );
        // Fixed joint frames define the physical starting pose. Multibody coordinates start at zero.
        for (parent, child, x, angle) in [(fixed, a, 0., q[0]), (a, b, l.length[0], q[1])] {
            let mut hinge = RevoluteJointBuilder::new(-Vector::Y)
                .local_anchor1(Vector::X * x)
                .build();
            hinge.data.local_frame1.rotation =
                Rotation::from_rotation_y(-angle) * hinge.data.local_frame1.rotation;
            let id = world.insert_multibody_joint(parent, child, hinge).unwrap();
            // Rapier seeds each free angular DOF with 0.1 viscous damping. The
            // fixture supplies all damping through the same torque-limited servo.
            world
                .multibody_joints
                .get_mut(id)
                .unwrap()
                .0
                .damping_mut()
                .fill(0.0);
        }
        Self {
            world,
            links: [a, b],
        }
    }
    pub fn step(&mut self, l: &Lane) {
        for _ in 0..8 {
            self.step_once(l);
        }
    }
    pub fn step_once(&mut self, l: &Lane) {
        let abs = self.links.map(|id| {
            let x = *self.world.bodies[id].rotation() * Vector::X;
            x.z.atan2(x.x)
        });
        let rate = self.links.map(|id| -self.world.bodies[id].angvel().y);
        let [a, b] = servo(l, [abs[0], abs[1] - abs[0]], [rate[0], rate[1] - rate[0]]);
        for (id, t) in self.links.into_iter().zip([a - b, b]) {
            let rb = &mut self.world.bodies[id];
            rb.reset_torques(true);
            rb.add_torque(-Vector::Y * t, true);
        }
        self.world.step();
    }
    pub fn tip(&self, l: &Lane) -> [f64; 2] {
        let link = &self.world.bodies[self.links[1]];
        let p = link.translation() + *link.rotation() * Vector::X * l.length[1];
        [p.x, p.z]
    }
}
