// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::CarryGripperLane;
use rapier3d_f64::prelude::*;
pub struct Native {
    world: PhysicsWorld,
    bodies: [RigidBodyHandle; 4],
    ball: RigidBodyHandle,
    shape: ColliderHandle,
    contact: [ColliderHandle; 4],
    n: usize,
    pub pad_up: [f64; 2],
}
impl Native {
    pub fn new(l: &CarryGripperLane, n: usize) -> Self {
        let h = &l.hand;
        let mut world = PhysicsWorld::new();
        world.gravity = -Vector::Z * h.gravity;
        world.integration_parameters.dt = 0.001 / n as f64;
        world.integration_parameters.normalized_allowed_linear_error = 0.;
        world.integration_parameters.normalized_prediction_distance = 0.;
        world
            .integration_parameters
            .normalized_contact_recycle_distance = 0.;
        world.integration_parameters.num_internal_pgs_iterations = 4;
        let fixed = world.insert_body(RigidBodyBuilder::fixed());
        let mass = [l.track_mass, h.mass[0], h.mass[1], h.mass[2]];
        let positions = [
            Vector::new(l.track, 0., 0.25),
            Vector::new(l.track, 0., h.position[0] + 0.08),
            Vector::new(h.position[1], 0., h.position[0]),
            Vector::new(h.position[2], 0., h.position[0]),
        ];
        let bodies = std::array::from_fn(|i| {
            world.insert_body(
                RigidBodyBuilder::dynamic()
                    .translation(positions[i])
                    .additional_mass_properties(MassProperties::new(
                        Vector::ZERO,
                        mass[i],
                        Vector::splat(0.0001),
                    ))
                    .can_sleep(false),
            )
        });
        let limits = [
            [l.track_lower - l.track, l.track_upper - l.track],
            [h.lower[0] - h.position[0], h.upper[0] - h.position[0]],
            [h.lower[1] - h.position[1], h.upper[1] - h.position[1]],
            [h.lower[2] - h.position[2], h.upper[2] - h.position[2]],
        ];
        for i in 0..4 {
            let parent = [fixed, bodies[0], bodies[1], bodies[1]][i];
            let dir = if i == 1 { Vector::Z } else { Vector::X };
            let joint = PrismaticJointBuilder::new(dir)
                .local_anchor1(positions[i] - world.bodies[parent].translation())
                .limits(limits[i]);
            let key = world
                .insert_multibody_joint(parent, bodies[i], joint)
                .unwrap();
            world
                .multibody_joints
                .get_mut(key)
                .unwrap()
                .0
                .damping_mut()
                .fill(0.);
        }
        let mat = |c: ColliderBuilder, mu: f64| {
            c.friction(mu)
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.)
        };
        let pads: [ColliderHandle; 2] = std::array::from_fn(|i| {
            world.insert_collider(
                mat(
                    ColliderBuilder::cuboid(h.half_size[i][0], 0.032, h.half_size[i][1])
                        .density(0.),
                    h.friction[i],
                ),
                Some(bodies[i + 2]),
            )
        });
        let plane = world.insert_collider(
            mat(
                ColliderBuilder::cuboid(1., 1., 0.02).translation(Vector::Z * (h.ground - 0.02)),
                h.friction[2],
            ),
            None,
        );
        let shelf = world.insert_collider(
            mat(
                ColliderBuilder::cuboid(l.shelf_half[0], 0.06, l.shelf_half[1])
                    .translation(Vector::new(l.shelf_center[0], 0., l.shelf_center[1])),
                l.shelf_friction,
            ),
            None,
        );
        let ball = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(h.ball[0], 0., h.ball[1]))
                .can_sleep(false),
        );
        let shape = world.insert_collider(
            mat(
                ColliderBuilder::ball(h.radius).mass_properties(MassProperties::new(
                    Vector::new(l.com_offset[0], 0., l.com_offset[1]),
                    h.ball_mass,
                    Vector::splat(l.angular_inertia),
                )),
                1.,
            ),
            Some(ball),
        );
        Self {
            world,
            bodies,
            ball,
            shape,
            contact: [pads[0], pads[1], plane, shelf],
            pad_up: [0.; 2],
            n,
        }
    }
    pub fn state(&self) -> ([f64; 4], [f64; 2], [f64; 2]) {
        let p = self.bodies.map(|id| self.world.bodies[id].translation());
        let b = &self.world.bodies[self.ball];
        let pos = b.translation();
        let vel = b.velocity_at_point(pos);
        (
            [p[0].x, p[1].z - 0.08, p[2].x, p[3].x],
            [pos.x, pos.z],
            [vel.x, vel.z],
        )
    }
    pub fn pad_velocity(&self) -> [[f64; 2]; 2] {
        [self.bodies[2], self.bodies[3]].map(|id| {
            let v = self.world.bodies[id].linvel();
            [v.x, v.z]
        })
    }
    pub fn payload_rotation(&self) -> [f64; 2] {
        let b = &self.world.bodies[self.ball];
        let marker = b.rotation() * Vector::X;
        [marker.z.atan2(marker.x), -b.angvel().y]
    }
    pub fn step(&mut self, l: &CarryGripperLane, target: [f64; 4]) -> ([f64; 4], f64) {
        self.pad_up = [0.; 2];
        let mut n = [0.; 4];
        let mut shelf_up = 0.;
        let h = l.hand;
        for _ in 0..self.n {
            let q = self.state().0;
            let v = self.bodies.map(|id| self.world.bodies[id].linvel());
            let capped = |j: usize, position: f64, speed: f64, preload: f64| {
                (h.stiffness[j] * (target[j + 1] - position) - h.damping[j] * speed + preload)
                    .clamp(-h.force_limit[j], h.force_limit[j])
            };
            let rail = (l.track_stiffness * (target[0] - q[0]) - l.track_damping * v[0].x)
                .clamp(-l.track_effort, l.track_effort);
            let up = capped(0, q[1], v[1].z, h.mass.iter().sum::<f64>() * h.gravity);
            let left = capped(1, q[2] - q[0] + l.track_origin, v[2].x - v[0].x, 0.);
            let right = capped(2, q[3] - q[0] + l.track_origin, v[3].x - v[0].x, 0.);
            for (id, force) in self.bodies.into_iter().zip([
                Vector::new(rail, 0., -up),
                Vector::new(-left - right, 0., up),
                Vector::X * left,
                Vector::X * right,
            ]) {
                let b = &mut self.world.bodies[id];
                b.reset_forces(true);
                b.add_force(force, true);
            }
            self.world.step();
            for (i, c) in self.contact.iter().enumerate() {
                if let Some(pair) = self.world.narrow_phase.contact_pair(*c, self.shape) {
                    n[i] += pair.total_impulse_magnitude();
                    if i < 2 {
                        use rapier3d_f64::utils::OrthonormalBasis;
                        for manifold in pair.solver_manifolds() {
                            let dir = -manifold.data.normal;
                            let t1 = dir.orthonormal_vector();
                            let t2 = dir.cross(t1);
                            let sign = if pair.collider1 == self.shape {
                                1.
                            } else {
                                -1.
                            };
                            for point in &manifold.points {
                                let d = point.data;
                                self.pad_up[i] += sign
                                    * (dir.z * d.impulse
                                        + t1.z * d.tangent_impulse.x
                                        + t2.z * d.tangent_impulse.y);
                            }
                        }
                    }
                    if i == 3 {
                        let impulse = pair.total_impulse();
                        shelf_up += if pair.collider2 == self.shape {
                            impulse.z
                        } else {
                            -impulse.z
                        };
                    }
                }
            }
        }
        (n, shelf_up)
    }
}
