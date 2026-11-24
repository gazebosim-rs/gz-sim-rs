// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{native::Native, tool_reference::Observation};
use gz_physics_gpu::PlanarToolLane as Lane;
use rapier3d_f64::prelude::*;
use rapier3d_f64::utils::OrthonormalBasis;
pub struct NativeTool {
    pub(super) arm: Native,
    tool: ColliderHandle,
    ground: ColliderHandle,
    measure_limits: bool,
    steps: usize,
    pub limit_residual: [f64; 2],
}
impl NativeTool {
    pub fn new(l: &Lane) -> Self {
        Self::new_with_limits(l, [None; 2])
    }
    pub fn new_with_limits(l: &Lane, limits: [Option<[f64; 2]>; 2]) -> Self {
        let mut arm = Native::new_with_limits(&l.arm, limits);
        arm.world
            .integration_parameters
            .normalized_prediction_distance = 0.;
        arm.world
            .integration_parameters
            .normalized_allowed_linear_error = 0.;

        let tool = arm.world.insert_collider(
            ColliderBuilder::ball(l.radius)
                .translation(Vector::X * l.arm.length[1])
                .density(0.)
                .friction(l.friction)
                .restitution(0.),
            Some(arm.links[1]),
        );
        let ground = arm.world.insert_collider(
            ColliderBuilder::cuboid(3., 3., 0.02)
                .translation(Vector::Z * (l.ground_z - 0.02))
                .friction(l.friction)
                .restitution(0.),
            None,
        );
        let mut result = Self {
            arm,
            tool,
            ground,
            measure_limits: limits.iter().any(Option::is_some),
            steps: 16,
            limit_residual: [0.; 2],
        };
        result.set_steps_per_ms(std::num::NonZeroUsize::new(16).unwrap());
        result
    }
    /// Keep commands on their original clock while refining Rapier's internal solve.
    pub fn set_steps_per_ms(&mut self, steps: std::num::NonZeroUsize) {
        self.steps = steps.get();
        self.arm.world.integration_parameters.dt = 0.001 / self.steps as f64;
    }
    // Independent generalized momentum and smooth ∂L/∂q from the rigid links.
    // Rapier does not expose impulse writeback on internal multibody limits.
    fn balance_terms(&self, l: &Lane) -> ([f64; 2], [f64; 2]) {
        let [a, b] = self.arm.angles();
        let [u, v] = self.arm.rates();
        let arm = &l.arm;
        let distal = arm.mass[1] * arm.center[1] + arm.tip_mass * arm.length[1];
        let prox = arm.mass[0] * arm.center[0] + (arm.mass[1] + arm.tip_mass) * arm.length[0];
        let i2 = arm.inertia[1]
            + arm.mass[1] * arm.center[1].powi(2)
            + arm.tip_mass * arm.length[1].powi(2);
        let i1 = arm.inertia[0]
            + arm.mass[0] * arm.center[0].powi(2)
            + (arm.mass[1] + arm.tip_mass) * arm.length[0].powi(2);
        let cross = arm.length[0] * distal;
        let p = [
            (i1 + i2 + 2. * cross * b.cos()) * u + (i2 + cross * b.cos()) * v,
            (i2 + cross * b.cos()) * u + i2 * v,
        ];
        let g2 = -arm.gravity * distal * (a + b).cos();
        (
            p,
            [
                g2 - arm.gravity * prox * a.cos(),
                g2 - cross * b.sin() * u * (u + v),
            ],
        )
    }
    pub fn step(&mut self, l: &Lane) -> Observation {
        let mut obs = Observation::default();
        self.limit_residual = [0.; 2];
        let dt = self.arm.world.integration_parameters.dt;
        for k in 0..self.steps {
            let before = self.measure_limits.then(|| self.balance_terms(l));
            let motor = self
                .arm
                .step_with_motor(&l.arm, l.velocity_mode, l.velocity_target);
            let mut floor = [0.; 2];
            if let Some(pair) = self
                .arm
                .world
                .narrow_phase
                .contact_pair(self.tool, self.ground)
            {
                for m in pair.solver_manifolds() {
                    let n = -m.data.normal;
                    let t = n.orthonormal_vector();
                    let b = n.cross(t);
                    let sign = if pair.collider1 == self.tool { 1. } else { -1. };
                    for point in &m.data.solver_contacts {
                        let p = &m.points[(point.contact_id[0]
                            & !rapier3d_f64::geometry::NEW_CONTACT_BIT)
                            as usize]
                            .data;
                        let tangent = sign * (t * p.tangent_impulse.x + b * p.tangent_impulse.y);
                        if self.measure_limits {
                            let elbow = self.arm.world.bodies[self.arm.links[1]].translation();
                            let body = &self.arm.world.bodies[self.arm.links[1]];
                            let local = if pair.collider1 == self.tool {
                                point.anchor1
                            } else {
                                point.anchor2
                            };
                            let contact = body.center_of_mass() + *body.rotation() * local;
                            let force = sign * n * p.impulse + tangent;
                            floor[0] += contact.x * force.z - contact.z * force.x;
                            floor[1] +=
                                (contact.x - elbow.x) * force.z - (contact.z - elbow.z) * force.x;
                        }
                        obs.add(p.impulse, tangent.x, (k + 1) as f64 * dt);
                    }
                }
            }
            if let Some((p0, g0)) = before {
                let (p1, g1) = self.balance_terms(l);
                for i in 0..2 {
                    self.limit_residual[i] +=
                        p1[i] - p0[i] - (motor[i] + (g0[i] + g1[i]) * 0.5) * dt - floor[i];
                }
            }
        }
        obs.tip = self.arm.tip(&l.arm);
        obs
    }
}
