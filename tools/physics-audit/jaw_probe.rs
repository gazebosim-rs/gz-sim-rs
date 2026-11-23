// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use rapier3d_f64::math::{Matrix, Rotation, Vector};
use rapier3d_f64::prelude::*;
fn main() {
    let target = Matrix::from_diagonal(Vector::new(1.15e-6, 1.15e-6, 0.30e-6));
    let m = MassProperties::with_inertia_matrix(Vector::ZERO, 0.010, target);
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let q = Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::X * std::f64::consts::FRAC_PI_2)
            .additional_mass_properties(m),
    );
    colliders.insert_with_parent(
        ColliderBuilder::capsule_z(0.012, 0.008).density(0.),
        body,
        &mut bodies,
    );
    bodies[body].recompute_mass_properties_from_colliders(&colliders);
    let m = &bodies[body].mass_properties().local_mprops;
    let frame = Matrix::from_quat(m.principal_inertia_local_frame);
    let t = frame * Matrix::from_diagonal(m.inv_principal_inertia.recip()) * frame.transpose();
    let world = Matrix::from_quat(q) * t * Matrix::from_quat(q).transpose();
    println!(
        "legacy principal {:?}, axes {:?}",
        m.inv_principal_inertia.recip().to_array(),
        m.principal_inertia_local_frame.to_array()
    );
    println!("link columns {:?}", t.to_cols_array());
    println!("world columns {:?}", world.to_cols_array());
    for axis in [Vector::X, Vector::Y, Vector::Z] {
        bodies[body].set_angvel(Vector::ZERO, false);
        bodies[body].apply_torque_impulse(axis, true);
        println!(
            "unit world torque {:?} => {:?}",
            axis.to_array(),
            bodies[body].angvel().to_array()
        );
    }
}
