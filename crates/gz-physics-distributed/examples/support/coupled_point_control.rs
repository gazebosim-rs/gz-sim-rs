// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Body, BodyState};
use rapier3d_f64::{prelude::*, utils::OrthonormalBasis};
pub(super) struct ContactSample {
    pub(super) normal: Vector,
    pub(super) lever: Vector,
    pub(super) impulse: f64,
    pub(super) tangent: Option<Vector>,
}
/// An independent direct Rapier world. Only the integration friction selection varies between controls.
pub(super) struct Direct {
    world: PhysicsWorld,
    handles: [RigidBodyHandle; 2],
    block: ColliderHandle,
}
impl Direct {
    pub fn new(seeds: &[Body; 2], pointwise: bool) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::from(super::G);
        world.integration_parameters.contact_recycling = false;
        if pointwise {
            world.integration_parameters.friction_model = FrictionModel::Coulomb;
        }
        let mut ids = vec![];
        for (i, b) in seeds.iter().enumerate() {
            let body = if i == 0 {
                RigidBodyBuilder::kinematic_velocity_based()
            } else {
                RigidBodyBuilder::dynamic()
            };
            let [x, y, z] = b.size.map(|v| v * 0.5);
            let (h, c) = world.insert(
                body.can_sleep(false)
                    .pose(Pose::from_parts(
                        Vector::from(b.position),
                        Rotation::from_array(b.orientation),
                    ))
                    .linvel(Vector::from(b.linear))
                    .angvel(Vector::from(b.angular)),
                ColliderBuilder::cuboid(x, y, z)
                    .mass(b.mass)
                    .friction(b.friction)
                    .friction_combine_rule(CoefficientCombineRule::Min),
            );
            world.bodies[h].recompute_mass_properties_from_colliders(&world.colliders);
            ids.push((h, c));
        }
        Self {
            world,
            handles: [ids[0].0, ids[1].0],
            block: ids[1].1,
        }
    }
    pub fn state(&self, i: usize) -> BodyState {
        let b = &self.world.bodies[self.handles[i]];
        BodyState {
            position: b.translation().to_array(),
            orientation: b.rotation().to_array(),
            velocity: b.linvel().to_array(),
            angular_velocity: b.angvel().to_array(),
        }
    }
    pub fn kick(&mut self, linear: [f64; 3], angular: [f64; 3]) {
        let b = &mut self.world.bodies[self.handles[1]];
        b.apply_impulse(Vector::from(linear), true);
        b.apply_torque_impulse(Vector::from(angular), true);
    }
    pub fn step(&mut self, dt: f64) {
        self.world.integration_parameters.dt = dt;
        self.world.step();
    }
    /// Sum selected geometric points in their canonical native manifold order.
    pub fn normal_receipt(&self) -> Option<f64> {
        let pair = self
            .world
            .narrow_phase
            .contact_pairs()
            .find(|p| p.has_any_active_contact())?;
        Some(
            pair.solver_manifolds()
                .iter()
                .map(|m| {
                    let mut indices = m
                        .data
                        .solver_contacts
                        .iter()
                        .map(|p| {
                            (p.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize
                        })
                        .collect::<Vec<_>>();
                    indices.sort_unstable();
                    indices.dedup();
                    indices
                        .into_iter()
                        .map(|i| m.points[i].data.impulse)
                        .sum::<f64>()
                })
                .sum(),
        )
    }
    pub fn points(&self) -> Vec<ContactSample> {
        let mut out = vec![];
        for p in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.has_any_active_contact())
        {
            let first = p.collider1 == self.block;
            let sign = if first { 1. } else { -1. };
            for m in p.solver_manifolds() {
                let normal = -m.data.normal;
                let t1 = normal.orthonormal_vector();
                let t2 = normal.cross(t1);
                for solver in &m.data.solver_contacts {
                    let id =
                        (solver.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize;
                    let d = &m.points[id].data;
                    out.push(ContactSample {
                        normal: normal * sign,
                        lever: if first { d.solver_dp1 } else { d.solver_dp2 },
                        impulse: d.impulse,
                        tangent: Some((t1 * d.tangent_impulse.x + t2 * d.tangent_impulse.y) * sign),
                    });
                }
            }
        }
        out
    }
}
