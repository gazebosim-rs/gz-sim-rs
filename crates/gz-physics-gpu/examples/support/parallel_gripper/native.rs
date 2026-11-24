// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::ParallelGripperLane as Lane;
use rapier3d_f64::prelude::*;

pub struct Native {
    world: PhysicsWorld,
    sliders: [RigidBodyHandle; 3],
    ball: RigidBodyHandle,
    pads: [ColliderHandle; 2],
    table: ColliderHandle,
    shape: ColliderHandle,
    substeps: usize,
}
impl Native {
    pub fn new(l: &Lane, substeps: usize) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = -Vector::Z * l.gravity;
        world.integration_parameters.dt = 0.001 / substeps as f64;
        world
            .integration_parameters
            .normalized_contact_recycle_distance = 0.;
        world.integration_parameters.normalized_prediction_distance = 0.;
        world.integration_parameters.normalized_allowed_linear_error = 0.;
        world.integration_parameters.num_internal_pgs_iterations = 4;
        let fixed = world.insert_body(RigidBodyBuilder::fixed());
        let sliders = std::array::from_fn(|i| {
            let x = if i == 0 { 0. } else { l.position[i] };
            // Carriage center is above the pads; pure prismatic links keep orientations fixed.
            let h = l.position[0] + if i == 0 { 0.08 } else { 0. };
            world.insert_body(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x, 0., h))
                    .additional_mass_properties(MassProperties::new(
                        Vector::ZERO,
                        l.mass[i],
                        Vector::splat(0.0001),
                    ))
                    .can_sleep(false),
            )
        });
        for (i, &body) in sliders.iter().enumerate() {
            let parent = if i == 0 { fixed } else { sliders[0] };
            let axis = if i == 0 { Vector::Z } else { Vector::X };
            let offset = world.bodies[body].translation() - world.bodies[parent].translation();
            let joint = PrismaticJointBuilder::new(axis)
                .local_anchor1(offset)
                .limits([l.lower[i] - l.position[i], l.upper[i] - l.position[i]]);
            let id = world.insert_multibody_joint(parent, body, joint).unwrap();
            world
                .multibody_joints
                .get_mut(id)
                .unwrap()
                .0
                .damping_mut()
                .fill(0.);
        }
        let material = |s: ColliderBuilder, mu: f64| {
            s.friction(mu)
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.)
        };
        let pads = std::array::from_fn(|i| {
            world.insert_collider(
                material(
                    ColliderBuilder::cuboid(l.half_size[i][0], 0.032, l.half_size[i][1])
                        .density(0.),
                    l.friction[i],
                ),
                Some(sliders[i + 1]),
            )
        });
        let table = world.insert_collider(
            material(
                ColliderBuilder::cuboid(1., 1., 0.02).translation(Vector::Z * (l.ground - 0.02)),
                l.friction[2],
            ),
            None,
        );
        let ball = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(l.ball[0], 0., l.ball[1]))
                .can_sleep(false),
        );
        let shape = world.insert_collider(
            material(ColliderBuilder::ball(l.radius).mass(l.ball_mass), 1.),
            Some(ball),
        );
        Self {
            world,
            sliders,
            ball,
            pads,
            table,
            shape,
            substeps,
        }
    }
    pub fn state(&self) -> ([f64; 3], [f64; 2]) {
        let links = self.sliders.map(|h| self.world.bodies[h].translation());
        let b = self.world.bodies[self.ball].translation();
        ([links[0].z - 0.08, links[1].x, links[2].x], [b.x, b.z])
    }
    pub fn step(&mut self, l: &Lane) -> ([f64; 3], f64) {
        let vz = self.world.bodies[self.ball].linvel().z;
        let mut n = [0.; 3];
        for _ in 0..self.substeps {
            let q = self.state().0;
            let vel = self.sliders.map(|h| self.world.bodies[h].linvel());
            let vel = [vel[0].z, vel[1].x, vel[2].x];
            let force: [f64; 3] = std::array::from_fn(|i| {
                (l.stiffness[i] * (l.target[i] - q[i]) - l.damping[i] * vel[i]
                    + if i == 0 {
                        l.mass.iter().sum::<f64>() * l.gravity
                    } else {
                        0.
                    })
                .clamp(-l.force_limit[i], l.force_limit[i])
            });
            for (body, f) in self.sliders.into_iter().zip([
                Vector::new(-force[1] - force[2], 0., force[0]),
                Vector::X * force[1],
                Vector::X * force[2],
            ]) {
                let rb = &mut self.world.bodies[body];
                rb.reset_forces(true);
                rb.add_force(f, true);
            }
            self.world.step();
            for (i, handle) in [self.pads[0], self.pads[1], self.table]
                .into_iter()
                .enumerate()
            {
                if let Some(p) = self.world.narrow_phase.contact_pair(handle, self.shape) {
                    n[i] += p.total_impulse_magnitude();
                }
            }
        }
        // The sphere has only gravity, the plane and the two finger contacts.
        let jaw_impulse =
            l.ball_mass * (self.world.bodies[self.ball].linvel().z - vz + l.gravity * 0.001) - n[2];
        (n, jaw_impulse)
    }
}
