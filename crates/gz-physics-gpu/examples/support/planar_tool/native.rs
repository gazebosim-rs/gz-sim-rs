// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{native::Native, tool_reference::Observation};
use gz_physics_gpu::PlanarToolLane as Lane;
use rapier3d_f64::prelude::*;
use rapier3d_f64::utils::OrthonormalBasis;
pub struct NativeTool {
    arm: Native,
    tool: ColliderHandle,
    ground: ColliderHandle,
}
impl NativeTool {
    pub fn new(l: &Lane) -> Self {
        let mut arm = Native::new(&l.arm);
        arm.world
            .integration_parameters
            .normalized_prediction_distance = 0.;
        arm.world
            .integration_parameters
            .normalized_allowed_linear_error = 0.;
        arm.world.integration_parameters.dt = 0.001 / 16.;
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
        Self { arm, tool, ground }
    }
    pub fn step(&mut self, l: &Lane) -> Observation {
        let mut obs = Observation::default();
        for k in 0..16 {
            self.arm.step_once(&l.arm);
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
                        obs.add(p.impulse, tangent.x, (k + 1) as f64 * 0.001 / 16.);
                    }
                }
            }
        }
        obs.tip = self.arm.tip(&l.arm);
        obs
    }
}
