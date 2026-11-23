// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{STEP, SpherePlaneLane as Lane};
use rapier3d_f64::prelude::*;

pub type V = Vector;
pub type Q = Rotation;

pub fn fixture(count: usize) -> Vec<Lane> {
    (0..count)
        .map(|i| {
            let pick = |stride: usize, offset: usize| ((i * stride + offset) % 2048) as f64 / 2047.;
            let theta = std::f64::consts::TAU * pick(811, 19);
            let n = V::new(0.30 * theta.cos(), 0.27 * theta.sin(), 1.).normalize();
            let a = (V::X - n * n.x).normalize();
            let b = n.cross(a);
            let radius = 0.075 + 0.065 * pick(103, 217);
            let offset = 0.08 * (pick(333, 18) - 0.5);
            let mass = 0.45 + 1.75 * pick(541, 79);
            let gap = 0.045 + 0.15 * pick(683, 307);
            let gravity = V::new(
                -1.4 + 2.8 * pick(797, 401),
                -0.8 + 1.6 * pick(431, 123),
                -7.6 - 2.8 * pick(313, 7),
            );
            let vx = 0.65 + 0.65 * pick(971, 95);
            let vy = -0.55 + 1.1 * pick(419, 503);
            let velocity = a * vx + b * vy - n * (0.2 + 0.65 * pick(859, 45));
            let omega = a * (-2.5 + 5. * pick(371, 613))
                + b * (-4. + 8. * pick(937, 1031))
                + n * (0.6 + 2.5 * pick(599, 763));
            let q = Q::from_axis_angle((a + b + n).normalize(), pick(277, 19) * 1.2);
            let p = n * (offset + gap + radius) - a * 0.36 - b * 0.1;
            let friction = match i % 4 {
                0 => 0.,
                1 => 0.035 + 0.065 * pick(167, 207),
                2 => 0.25 + 0.2 * pick(379, 537),
                _ => 0.75 + 0.2 * pick(179, 401),
            };
            Lane {
                position: p.to_array(),
                velocity: velocity.to_array(),
                angular_velocity: omega.to_array(),
                orientation: q.to_array(),
                gravity: gravity.to_array(),
                plane_normal: n.to_array(),
                plane_offset: offset,
                radius,
                inverse_mass: 1. / mass,
                friction,
                normal_impulse: 0.,
                tangential_impulse: [0.; 3],
            }
        })
        .collect()
}

fn advance_orientation(q: Q, w: V, dt: f64) -> Q {
    let d = w * (dt * 0.5);
    (Q::from_xyzw(d.x, d.y, d.z, 1.).normalize() * q).normalize()
}

/// Independent scalar contact calculation. Compute directional point mobility
/// from the moment arm and solid-sphere inertia, independently of the shader's
/// simplified 7/2 constant. Integrate the two pieces of the swept trajectory.
pub fn analytical(s: &mut Lane) {
    let h = STEP.as_secs_f64();
    let n = V::from(s.plane_normal);
    let arm = -n * s.radius;
    let mut v = V::from(s.velocity) + h * V::from(s.gravity);
    let mut w = V::from(s.angular_velocity);
    let q = Q::from_array(s.orientation);
    let p = V::from(s.position);
    let speed = v.dot(n);
    let initial_gap = n.dot(p) - s.plane_offset - s.radius;
    let (mut jn, mut jt) = (0., V::ZERO);
    let (mut position, mut orientation) = (p + h * v, advance_orientation(q, w, h));
    if speed < 0. && initial_gap + h * speed <= 0. {
        let until = (initial_gap / -speed).clamp(0., h);
        let mass = 1. / s.inverse_mass;
        let inertia = (2. / 5.) * mass * s.radius.powi(2);
        let slip = v + w.cross(arm);
        let tangent = slip - n * slip.dot(n);
        jn = -mass * speed;
        if tangent.length_squared() > 0. {
            let direction = tangent.normalize();
            let mobility = 1. / mass + arm.cross(direction).length_squared() / inertia;
            jt = -direction * (tangent.length() / mobility).min(s.friction * jn);
        }
        let at = p + v * until;
        let q_at = advance_orientation(q, w, until);
        v += (n * jn + jt) / mass;
        w += arm.cross(jt) / inertia;
        position = at + v * (h - until);
        position += n * (s.radius + s.plane_offset - position.dot(n)).max(0.);
        orientation = advance_orientation(q_at, w, h - until);
    }
    s.position = position.to_array();
    s.orientation = orientation.to_array();
    s.velocity = v.to_array();
    s.angular_velocity = w.to_array();
    s.normal_impulse = jn;
    s.tangential_impulse = jt.to_array();
}

pub struct Native {
    world: PhysicsWorld,
    sphere: RigidBodyHandle,
    substeps: usize,
    pub state: Lane,
    pub gpu_first: Option<usize>,
    pub native_first: Option<usize>,
}
impl Native {
    pub fn new(initial: Lane) -> Self {
        Self::with_substeps(initial, 8)
    }
    pub fn with_substeps(initial: Lane, substeps: usize) -> Self {
        assert!(substeps > 0);
        let mut world = PhysicsWorld::new();
        // Refine the reference clock 8x. Reduce Rapier's default gameplay
        // tolerances (5 mm slop, 20 mm speculative distance) for a hard plane.
        world.integration_parameters.dt = STEP.as_secs_f64() / substeps as f64;
        world.integration_parameters.normalized_allowed_linear_error = 1e-7;
        world.integration_parameters.normalized_prediction_distance = 1e-6;
        world.integration_parameters.contact_recycling = false;
        world.gravity = V::from(initial.gravity);
        let n = V::from(initial.plane_normal);
        world.insert(
            RigidBodyBuilder::fixed().translation(n * initial.plane_offset),
            ColliderBuilder::new(SharedShape::halfspace(n))
                .friction(initial.friction)
                .restitution(0.),
        );
        let (sphere, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .pose(Pose::from_parts(
                    V::from(initial.position),
                    Q::from_array(initial.orientation),
                ))
                .linvel(V::from(initial.velocity))
                .angvel(V::from(initial.angular_velocity)),
            ColliderBuilder::ball(initial.radius)
                .mass(initial.inverse_mass.recip())
                .friction(initial.friction)
                .restitution(0.),
        );
        Self {
            world,
            sphere,
            substeps,
            state: initial,
            gpu_first: None,
            native_first: None,
        }
    }
    pub fn impulse_a(&mut self, j: [f64; 3]) {
        self.world.bodies[self.sphere].apply_impulse(V::from(j), true);
    }
    pub fn step(&mut self) {
        // Capture velocity after the caller's center impulse. A separately supplied
        // actuator impulse must never be counted as a plane contact reaction.
        let old = self.world.bodies[self.sphere].linvel();
        for _ in 0..self.substeps {
            self.world.step();
        }
        let b = &self.world.bodies[self.sphere];
        self.state.position = b.translation().to_array();
        self.state.orientation = b.rotation().to_array();
        self.state.angular_velocity = b.angvel().to_array();
        self.state.velocity = b.linvel().to_array();
        let impulse =
            (b.linvel() - old - self.world.gravity * STEP.as_secs_f64()) / self.state.inverse_mass;
        let n = V::from(self.state.plane_normal);
        self.state.normal_impulse = impulse.dot(n);
        self.state.tangential_impulse = (impulse - n * self.state.normal_impulse).to_array();
    }
}
