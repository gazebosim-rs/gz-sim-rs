// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct, independently constructed solver for the authored stationary-table SDF.
use gz_physics::BodyState;
use rapier3d_f64::prelude::*;

pub struct Control {
    world: PhysicsWorld,
    bodies: [RigidBodyHandle; 2],
}
impl Control {
    pub fn new(point: bool) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -9.81);
        world.integration_parameters.dt = 0.001;
        if point {
            world.integration_parameters.friction_model = FrictionModel::Coulomb;
        }
        let table = world
            .insert(
                RigidBodyBuilder::fixed().translation(Vector::new(0., 0., -0.2)),
                ColliderBuilder::cuboid(4., 1.5, 0.2).density(0.),
            )
            .0;
        let inertia = Matrix::from_diagonal(Vector::new(
            0.03333333333333334,
            0.11333333333333334,
            0.13333333333333336,
        ));
        let block = world
            .insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(0., 0., 0.15))
                    .additional_mass_properties(MassProperties::with_inertia_matrix(
                        Vector::ZERO,
                        2.,
                        inertia,
                    )),
                ColliderBuilder::cuboid(0.4, 0.2, 0.1).density(0.),
            )
            .0;
        for id in [table, block] {
            world.bodies[id].recompute_mass_properties_from_colliders(&world.colliders);
        }
        Self {
            world,
            bodies: [table, block],
        }
    }
    pub fn kick(&mut self) {
        let b = &mut self.world.bodies[self.bodies[1]];
        b.apply_impulse(Vector::new(4.8, 0., 0.), true);
        b.apply_torque_impulse(Vector::new(0., 0., 0.96), true);
    }
    pub fn step(&mut self) {
        self.world.step();
    }
    pub fn check(&self, index: usize, observed: BodyState) {
        let native = &self.world.bodies[self.bodies[index]];
        assert_eq!(
            observed.position.map(f64::to_bits),
            native.translation().to_array().map(f64::to_bits)
        );
        assert_eq!(
            observed.orientation.map(f64::to_bits),
            native.rotation().to_array().map(f64::to_bits)
        );
        assert_eq!(
            observed.velocity.map(f64::to_bits),
            native.linvel().to_array().map(f64::to_bits)
        );
        assert_eq!(
            observed.angular_velocity.map(f64::to_bits),
            native.angvel().to_array().map(f64::to_bits)
        );
    }
}
