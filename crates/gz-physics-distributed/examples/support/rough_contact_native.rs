// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Body, G, RADIUS};
use rapier3d_f64::{prelude::*, utils::OrthonormalBasis};
use serde::Serialize;

#[derive(Serialize, Default)]
pub(super) struct Reaction {
    pub active: bool,
    pub normal_n_s: f64,
    pub normal_on_sphere: [f64; 3],
    pub tangent_on_sphere: [f64; 3],
    pub points: Vec<ContactPoint>,
}
#[derive(Serialize)]
pub(super) struct ContactPoint {
    /// Geometric witness on the sphere transported through its committed pose.
    pub sphere_witness_world_m: [f64; 3],
    /// Geometric signed separation at contact generation; negative means penetration.
    pub geometric_distance_m: f64,
    /// Final native substep warm-start normal, distinct from the public interval sum.
    pub final_warmstart_normal_n_s: f64,
    /// Midpoint between the two geometric witnesses at the start of integration, relative to the sphere COM.
    pub pre_step_lever_arm_m: [f64; 3],
    /// World-space solver lever arm on the sphere, as written by native narrow phase.
    pub native_solver_lever_arm_m: [f64; 3],
    pub normal_impulse_on_sphere_n_s: [f64; 3],
    pub tangent_impulse_on_sphere_n_s: [f64; 3],
}
pub(super) struct Native {
    pub(super) world: PhysicsWorld,
    slab: RigidBodyHandle,
    sphere: RigidBodyHandle,
    sphere_shape: ColliderHandle,
}
impl Native {
    pub fn new(seeds: &[Body; 2], dt: f64, point_friction: bool) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -G);
        world.integration_parameters.dt = dt;
        if point_friction {
            world.integration_parameters.friction_model = FrictionModel::Coulomb;
        }
        world.integration_parameters.contact_recycling = false;
        let mut ids = vec![];
        for (i, b) in seeds.iter().enumerate() {
            let (rigid, shape) = if i == 0 {
                let [x, y, z] = b.size.map(|n| n * 0.5);
                (
                    RigidBodyBuilder::kinematic_velocity_based(),
                    ColliderBuilder::cuboid(x, y, z),
                )
            } else {
                (RigidBodyBuilder::dynamic(), ColliderBuilder::ball(RADIUS))
            };
            let (h, c) = world.insert(
                rigid
                    .can_sleep(false)
                    .pose(Pose::from_parts(
                        Vector::from(b.position),
                        Rotation::from_xyzw(
                            b.orientation[0],
                            b.orientation[1],
                            b.orientation[2],
                            b.orientation[3],
                        ),
                    ))
                    .linvel(Vector::from(b.linear))
                    .angvel(Vector::from(b.angular)),
                shape
                    .mass(b.mass)
                    .friction(b.friction)
                    .friction_combine_rule(CoefficientCombineRule::Min),
            );
            world.bodies[h].recompute_mass_properties_from_colliders(&world.colliders);
            ids.push((h, c));
        }
        Self {
            world,
            slab: ids[0].0,
            sphere: ids[1].0,
            sphere_shape: ids[1].1,
        }
    }
    pub fn kick(&mut self, impulse: [f64; 3]) {
        self.world.bodies[self.sphere].apply_impulse(Vector::from(impulse), true);
    }
    pub fn step(&mut self, bodies: &mut [Body; 2]) -> Reaction {
        let sphere_before = *self.world.bodies[self.sphere].position();
        let slab_before = *self.world.bodies[self.slab].position();
        self.world.step();
        for (b, h) in bodies.iter_mut().zip([self.slab, self.sphere]) {
            let s = &self.world.bodies[h];
            b.position = s.translation().to_array();
            b.orientation = s.rotation().to_array();
            b.linear = s.linvel().to_array();
            b.angular = s.angvel().to_array();
        }
        let mut r = Reaction::default();
        for pair in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.has_any_active_contact())
        {
            r.active = true;
            let first_sphere = pair.collider1 == self.sphere_shape;
            assert!(first_sphere || pair.collider2 == self.sphere_shape);
            let sign = if first_sphere { 1. } else { -1. };
            for manifold in pair.solver_manifolds() {
                // Pinned Rapier builds a deterministic tangent basis from the
                // normal pointing toward collider 1; its opposite acts on 2.
                let n1 = -manifold.data.normal;
                let t1 = n1.orthonormal_vector();
                let t2 = n1.cross(t1);
                for row in &manifold.data.solver_contacts {
                    let index =
                        (row.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize;
                    let p = &manifold.points[index];
                    let normal = n1 * (sign * p.data.impulse);
                    let tangent =
                        (t1 * p.data.tangent_impulse.x + t2 * p.data.tangent_impulse.y) * sign;
                    let local_point = if first_sphere { p.local_p1 } else { p.local_p2 };
                    let point = self.world.colliders[self.sphere_shape]
                        .position()
                        .transform_point(local_point);
                    r.normal_n_s += p.data.impulse;
                    for axis in 0..3 {
                        r.normal_on_sphere[axis] += normal[axis];
                        r.tangent_on_sphere[axis] += tangent[axis];
                    }
                    let local_slab = if first_sphere { p.local_p2 } else { p.local_p1 };
                    let midpoint = (sphere_before.transform_point(local_point)
                        + slab_before.transform_point(local_slab))
                        * 0.5;
                    r.points.push(ContactPoint {
                        native_solver_lever_arm_m: if first_sphere {
                            p.data.solver_dp1.to_array()
                        } else {
                            p.data.solver_dp2.to_array()
                        },
                        pre_step_lever_arm_m: (midpoint - sphere_before.translation).to_array(),
                        sphere_witness_world_m: point.to_array(),
                        geometric_distance_m: p.dist,
                        final_warmstart_normal_n_s: p.data.warmstart_impulse,
                        normal_impulse_on_sphere_n_s: normal.to_array(),
                        tangent_impulse_on_sphere_n_s: tangent.to_array(),
                    });
                }
            }
        }
        r
    }
}
