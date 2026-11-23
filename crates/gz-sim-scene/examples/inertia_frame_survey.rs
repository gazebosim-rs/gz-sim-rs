// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Probe ordinary native inverse angular inertia along each world axis.
use glam::{DMat3, DQuat, DVec3};
use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_rapier::RapierBackend;
use rapier3d_f64::prelude::MassProperties;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
const LEVELS: [(&str, [f64; 3]); 8] = [
    ("spherical", [0.041, 0.041, 0.041]),
    ("triaxial", [0.031, 0.042, 0.067]),
    ("oblate", [0.041, 0.041, 0.072]),
    ("prolate", [0.041, 0.041, 0.012]),
    ("near_oblate_1e-12", [0.041, 0.041000000000041, 0.072]),
    ("near_prolate_1e-12", [0.041, 0.041000000000041, 0.012]),
    ("near_oblate_1e-7", [0.041, 0.0410000041, 0.072]),
    ("near_prolate_1e-7", [0.041, 0.0410000041, 0.012]),
];
fn pose(seed: f64) -> DQuat {
    DQuat::from_euler(
        glam::EulerRot::ZYX,
        1.38 * (seed * 2.81).sin(),
        1.55 * (seed * 7.97).cos(),
        2.05 * (seed * 1.9).sin(),
    )
}
fn matrix(q: DQuat, d: DVec3) -> DMat3 {
    let r = DMat3::from_quat(q);
    r * DMat3::from_diagonal(d) * r.transpose()
}
fn max_norm(m: DMat3) -> f64 {
    m.to_cols_array()
        .into_iter()
        .map(f64::abs)
        .fold(0., f64::max)
}
fn field(
    id: usize,
    tree: bool,
    body: DQuat,
    local: DQuat,
    tensor: DMat3,
    scale: f64,
) -> Result<DMat3, Box<dyn Error>> {
    let mut native = RapierBackend::new_multifidelity([0.; 3])?;
    let spec = BodySpec {
        position: [0.12, 0.3, 1.7],
        velocity: [0.; 3],
        mass: Some(2.),
        shape: Shape::Box {
            half_extents: [0.18, 0.1, 0.06],
        },
    };
    let center = if id.is_multiple_of(2) {
        [0.; 3]
    } else {
        [0.005, -0.012, 0.07]
    };
    let inertia = Inertia {
        orientation: local.to_array(),
        center,
        tensor: [
            tensor.x_axis.x,
            tensor.y_axis.y,
            tensor.z_axis.z,
            tensor.y_axis.x,
            tensor.z_axis.x,
            tensor.z_axis.y,
        ],
    };
    let rot = RotationSpec {
        orientation: body.to_array(),
        angular_velocity: [0.; 3],
    };
    let id = if tree {
        native.spawn_articulation_link(spec, rot, Some(inertia))?
    } else {
        native.spawn_link(spec, rot, Some(inertia))?
    };
    let mut cols = [DVec3::ZERO; 3];
    for (i, axis) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().enumerate() {
        let before = DVec3::from(native.state(id).unwrap().angular_velocity);
        native.apply_impulse(id, [0.; 3], (axis * scale).to_array())?;
        let now = native.state(id).unwrap();
        assert_eq!(now.velocity, [0.; 3]);
        cols[i] = (DVec3::from(now.angular_velocity) - before) / scale;
    }
    Ok(DMat3::from_cols(cols[0], cols[1], cols[2]))
}
/// Reproduce the pinned pre-correction body and massless collider construction,
/// including collider-frame recomposition on an offset articulation link.
fn original_field(id: usize, member: bool, orientation: DQuat, tensor: DMat3, scale: f64) -> DMat3 {
    use rapier3d_f64::prelude::{ColliderBuilder, PhysicsWorld, Pose, RigidBodyBuilder};
    let mut world = PhysicsWorld::new();
    let offset = if id.is_multiple_of(2) {
        DVec3::ZERO
    } else {
        DVec3::new(0.005, -0.012, 0.07)
    };
    let mut declared = MassProperties::with_inertia_matrix(offset, 2., tensor);
    let mut shape = ColliderBuilder::cuboid(0.18, 0.1, 0.06).density(0.);
    let mut origin = DVec3::new(0.12, 0.3, 1.7);
    if member {
        declared.local_com = DVec3::ZERO;
        shape = shape.translation(-offset);
        origin += orientation * offset;
    }
    let body = RigidBodyBuilder::dynamic()
        .pose(Pose::from_parts(origin, orientation))
        .additional_mass_properties(declared);
    let handle = world.insert(body, shape).0;
    let b = &mut world.bodies[handle];
    b.recompute_mass_properties_from_colliders(&world.colliders);
    let mut cols = [DVec3::ZERO; 3];
    for (i, axis) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().enumerate() {
        let before = b.angvel();
        b.apply_torque_impulse(scale * axis, true);
        cols[i] = (b.angvel() - before) / scale;
    }
    DMat3::from_cols(cols[0], cols[1], cols[2])
}
fn main() -> Result<(), Box<dyn Error>> {
    let dest = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("pass an output directory")?,
    );
    fs::create_dir_all(&dest)?;
    let mut rows = vec![];
    let mut featured = None;
    let mut largest = 0.;
    for (class, values) in LEVELS {
        for ordering in 0..3 {
            let base = DVec3::new(
                values[ordering],
                values[(ordering + 1) % 3],
                values[(ordering + 2) % 3],
            );
            for scale in [1e-6, 1., 1e6] {
                for orientation in 0..12 {
                    let body = if orientation == 0 {
                        DQuat::IDENTITY
                    } else {
                        pose(0.72 + orientation as f64)
                    };
                    let author = if orientation == 0 {
                        DQuat::IDENTITY
                    } else {
                        pose(1.12 + orientation as f64)
                    };
                    for full in [false, true] {
                        let id = rows.len();
                        let internal = if full {
                            pose(2.87 + orientation as f64)
                        } else {
                            DQuat::IDENTITY
                        };
                        let d = base * scale;
                        let authored = matrix(internal, d);
                        let r = DMat3::from_quat(author);
                        let link = r * authored * r.transpose();
                        let exact = matrix(body * author * internal, d.recip());
                        let legacy = MassProperties::with_inertia_matrix(DVec3::ZERO, 2., link);
                        let legacy_orientation = body * legacy.principal_inertia_local_frame;
                        let original = matrix(legacy_orientation, legacy.inv_principal_inertia);
                        let (rigid, member) = (
                            field(id, false, body, author, authored, scale)?,
                            field(id, true, body, author, authored, scale)?,
                        );
                        let denom = max_norm(exact);
                        let error = |a: DMat3| max_norm(a - exact) / denom;
                        if error(rigid) > 2e-11 || error(member) > 2e-11 {
                            return Err(
                                format!("native inertia residual for row {id} {class} old={} rigid={} member={} exact={exact:?} actual={rigid:?}",error(original),error(rigid),error(member)).into()
                            );
                        }
                        let legacy_rigid = original_field(id, false, body, link, scale);
                        let legacy_member = original_field(id, true, body, link, scale);
                        let old = error(legacy_rigid);
                        if old > largest && old < 0.25 && scale == 1. && class == "oblate" {
                            largest = old;
                            featured = Some(
                                json!({"id":id,"class":class,"expected_tensor":link.to_cols_array(),"effective_old_tensor":(DMat3::from_quat(body).transpose()*legacy_rigid.inverse()*DMat3::from_quat(body)).to_cols_array(),"authored_tensor":authored.to_cols_array(),"authored_frame":author.to_array()}),
                            );
                        }
                        rows.push(json!({"id":id,"class":class,"ordering":ordering,"scale":scale,"orientation":orientation,"full":full,"principal":d.to_array(),"body_orientation":body.to_array(),"principal_to_link":(author*internal).to_array(),"original_inverse":original.to_cols_array(),"original_rigid_inverse":legacy_rigid.to_cols_array(),"original_member_inverse":legacy_member.to_cols_array(),"native_rigid_inverse":rigid.to_cols_array(),"native_member_inverse":member.to_cols_array(),"expected_inverse":exact.to_cols_array()}));
                    }
                }
            }
        }
    }
    fs::write(
        dest.join("frames.json"),
        serde_json::to_vec(&json!({"cases":rows,"featured":featured}))?,
    )?;
    Ok(())
}
