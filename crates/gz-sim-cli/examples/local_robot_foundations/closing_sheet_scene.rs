// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Author real refined cloth, material laws and visual bindings for the live arm.
use super::*;
use crate::arm_sheet::{BEND, COUNT, NODE_MASS, RADIUS, SHEAR, STRETCH};
use crate::fold_mesh::Anchor;
use gz_physics_xpbd::Particle;
use std::fmt::Write;

pub(super) fn scene(columns: usize) -> Result<String> {
    with_clock(columns, 8, 24)
}

pub(super) fn with_clock(columns: usize, substeps: u32, iterations: u32) -> Result<String> {
    if !matches!(columns, 3 | 5 | 7) {
        return Err("expected an odd material grid with 3, 5 or 7 columns".into());
    }
    let sdf = sdformat::load_str(arm::SCENE, "arm.sdf", &sdformat::NoResources)?;
    let half = [
        arm_cable::half_size(&sdf, "arm", "left")?,
        arm_cable::half_size(&sdf, "arm", "right")?,
    ];
    let loaded = (0..COUNT)
        .map(|i| Particle {
            position: arm_sheet::seed(i, 0.).to_array(),
            velocity: [0.; 3],
            inverse_mass: NODE_MASS.recip(),
        })
        .collect();
    let body = fold_mesh::mesh(&Anchor { loaded, half }, columns)?;
    let mut material = format!(
        r#"<cloth name="sheet" pad_loads="true" self_contact_loads="true" columns="{columns}" substeps="{substeps}" iterations="{iterations}" radius="{RADIUS}" self_contact="true" self_friction="0.3" friction="0.7" stretch="{STRETCH}" shear="{SHEAR}" bend="{BEND}">"#
    );
    let mut bindings = String::new();
    for (i, node) in body.particles().iter().enumerate() {
        let [x, y, z] = node.position;
        writeln!(
            material,
            r#"<particle position="{x} {y} {z}" mass="{}"/>"#,
            node.inverse_mass.recip()
        )
        .expect("writing to a String is infallible");
        writeln!(
            bindings,
            r#"<particle_visual material="sheet" particle="{i}" visual="{}"/>"#,
            arm_sheet::name(i)
        )
        .expect("writing to a String is infallible");
    }
    for row in body.constraints() {
        writeln!(
            material,
            r#"<distance_compliance particles="{} {}" value="{}"/>"#,
            row.a, row.b, row.compliance
        )
        .expect("writing to a String is infallible");
    }
    for row in body.bending_constraints() {
        let [a, b, _, _] = row.vertices;
        writeln!(
            material,
            r#"<hinge_compliance particles="{a} {b}" value="{}"/>"#,
            row.compliance
        )
        .expect("writing to a String is infallible");
    }
    for s in fold_patch::material_samples(half[0].y, columns) {
        let [a, b, c] = s.particles;
        let [u, v, w] = s.weights;
        writeln!(
            material,
            r#"<pad_sample particles="{a} {b} {c}" weights="{u} {v} {w}" stiffness_weight="{}"/>"#,
            s.stiffness_weight
        )
        .expect("writing to a String is infallible");
    }
    material.push_str("</cloth>");
    arm_sheet::scene_with_material(arm::SCENE, columns, 0., &material, &bindings)
}

#[cfg(test)]
#[path = "closing_sheet_scene_tests.rs"]
mod tests;
