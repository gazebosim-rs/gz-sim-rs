// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{MeshError, TriangleMesh, invalid, stl::encode_visual_glb};
use glam::DVec3;
use std::f64::consts::{FRAC_PI_2, TAU};

/// Centered Z-axis SDF capsule, in metres. `length` measures the straight
/// barrel between hemisphere centers. Its complete axial span is length + 2r.
pub fn capsule_visual_glb(
    radius: f64,
    length: f64,
    diffuse: [f32; 4],
) -> Result<Vec<u8>, MeshError> {
    if diffuse[3] != 1. || !diffuse.iter().all(|v| (0.0..=1.0).contains(v)) {
        return Err(invalid("capsule requires an opaque finite diffuse color"));
    }
    const SECTORS: u32 = 64;
    const BANDS: u32 = 16;
    let half = length * 0.5;
    let tip = half + radius;
    // All vertices are written as f32. Keep even the smallest cap height
    // increment and meridian arc representable after translation and rounding.
    let polar_angle = FRAC_PI_2 / f64::from(BANDS);
    let rounded_tip = tip as f32;
    let axial_resolution = f64::from(rounded_tip) - f64::from(rounded_tip.next_down());
    let shortest_arc = radius * polar_angle.sin() * (TAU / f64::from(SECTORS)).sin();
    if !((half as f32) > 0.
        && (shortest_arc as f32) > 0.
        && rounded_tip.is_finite()
        && radius * (1. - polar_angle.cos()) >= axial_resolution)
    {
        return Err(invalid(
            "capsule dimensions must resolve its barrel and hemispheres in glTF",
        ));
    }
    let mut mesh = TriangleMesh {
        vertices: vec![[0., 0., -tip]],
        triangles: Vec::new(),
    };
    let mut normals = vec![DVec3::NEG_Z];
    let meridians: [(f64, f64); SECTORS as usize] =
        std::array::from_fn(|i| (TAU * i as f64 / f64::from(SECTORS)).sin_cos());
    for side in [-1., 1.] {
        for ring in 0..BANDS {
            // Traverse from bottom to top, including both barrel rims.
            let latitude = if side < 0. { BANDS - 1 - ring } else { ring };
            let (nz, radial) = (side * f64::from(latitude) * polar_angle).sin_cos();
            for (y, x) in meridians {
                let n = DVec3::new(radial * x, radial * y, nz);
                mesh.vertices
                    .push((radius * n + DVec3::Z * (side * half)).to_array());
                normals.push(n);
            }
        }
    }
    let top = 1 + 2 * BANDS * SECTORS;
    mesh.vertices.push([0., 0., tip]);
    normals.push(DVec3::Z);
    for i in 0..SECTORS {
        let j = (i + 1) % SECTORS;
        mesh.triangles.push([0, 1 + j, 1 + i]);
        for ring in 0..2 * BANDS - 1 {
            let lower = 1 + ring * SECTORS;
            let upper = lower + SECTORS;
            mesh.triangles.extend([
                [lower + i, lower + j, upper + j],
                [lower + i, upper + j, upper + i],
            ]);
        }
        mesh.triangles
            .push([top - SECTORS + i, top - SECTORS + j, top]);
    }
    encode_visual_glb(&mesh, normals, "capsule", Some(diffuse))
}
