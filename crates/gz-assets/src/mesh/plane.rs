// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{MeshError, TriangleMesh, invalid, stl::colored_visual_glb};
use glam::{DQuat, DVec3};

/// Finite SDF visual rotated from local +Z to the authored normal, as in Gazebo.
/// The physical collision half-space remains independent of these dimensions.
pub fn plane_visual_glb(
    normal: [f64; 3],
    size: [f64; 2],
    diffuse: [f32; 4],
) -> Result<Vec<u8>, MeshError> {
    let n = DVec3::from(normal);
    let unit = n
        .try_normalize()
        .ok_or_else(|| invalid("invalid plane normal"))?;
    if !size
        .iter()
        .all(|&v| (v as f32).is_finite() && (v * 0.5) as f32 > 0.0)
    {
        return Err(invalid("invalid plane size"));
    }
    if diffuse[3] != 1.0 || !diffuse.iter().all(|v| (0.0..=1.0).contains(v)) {
        return Err(invalid(
            "generated plane requires finite opaque diffuse color",
        ));
    }
    let rotation = DQuat::from_rotation_arc(DVec3::Z, unit);
    let [x, y] = size.map(|v| v * 0.5);
    let corners = [(-x, -y), (x, -y), (x, y), (-x, y)];
    let mesh = TriangleMesh {
        vertices: corners
            .into_iter()
            .map(|(x, y)| (rotation * DVec3::new(x, y, 0.0)).to_array())
            .collect(),
        triangles: vec![[0, 1, 2], [0, 2, 3]],
    };
    colored_visual_glb(&mesh, "plane", Some(diffuse))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_rectangle_preserves_authored_size_orientation_and_color() {
        let diffuse = [0.8, 0.6, 0.2, 1.0];
        for normal in [[0., 0., 1.], [0., 0., -1.], [-2., 3., 4.]] {
            let bytes = plane_visual_glb(normal, [3., 9.], diffuse).unwrap();
            let doc = gltf::Gltf::from_slice(&bytes).unwrap();
            let prim = doc.meshes().next().unwrap().primitives().next().unwrap();
            assert_eq!(
                prim.material().pbr_metallic_roughness().base_color_factor(),
                diffuse
            );
            let reader = prim.reader(|_| doc.blob.as_deref());
            let to_sim = |[x, y, z]: [f32; 3]| DVec3::new(x.into(), (-z).into(), y.into());
            let p: Vec<_> = reader.read_positions().unwrap().map(to_sim).collect();
            let n = DVec3::from(normal).normalize();
            assert!(
                reader
                    .read_normals()
                    .unwrap()
                    .map(to_sim)
                    .all(|v| v.distance(n) < 1e-6)
            );
            assert_eq!(
                reader
                    .read_indices()
                    .unwrap()
                    .into_u32()
                    .collect::<Vec<_>>(),
                [0, 1, 2, 0, 2, 3]
            );
            assert!(((p[1] - p[0]).length() - 3.).abs() < 1e-6);
            assert!(((p[2] - p[1]).length() - 9.).abs() < 1e-6);
            assert!((p[1] - p[0]).cross(p[2] - p[0]).normalize().distance(n) < 1e-6);
        }
    }

    #[test]
    fn rejects_invalid_geometry_and_colors() {
        for normal in [[0.; 3], [f64::NAN, 0., 1.]] {
            assert!(plane_visual_glb(normal, [1.; 2], [1.; 4]).is_err());
        }
        for size in [0., -1., f64::MIN_POSITIVE, f64::INFINITY] {
            assert!(plane_visual_glb([0., 0., 1.], [1., size], [1.; 4]).is_err());
        }
        for color in [[0.; 4], [2., 0., 0., 1.]] {
            assert!(plane_visual_glb([0., 0., 1.], [1.; 2], color).is_err());
        }
    }
}
