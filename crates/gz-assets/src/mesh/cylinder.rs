// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{MeshError, TriangleMesh, invalid, stl::colored_visual_glb};

/// A centered unit-radius, unit-length Z cylinder for SDF visuals. The viewer
/// applies each instance's [radius, radius, length] scale in simulation axes.
pub fn cylinder_visual_glb(diffuse: [f32; 4]) -> Result<Vec<u8>, MeshError> {
    if diffuse[3] != 1.0 || !diffuse.iter().all(|v| (0.0..=1.0).contains(v)) {
        return Err(invalid(
            "generated cylinder requires finite opaque diffuse color",
        ));
    }
    const N: u32 = 128;
    let mut mesh = TriangleMesh {
        vertices: vec![[0.0, 0.0, -0.5], [0.0, 0.0, 0.5]],
        triangles: Vec::with_capacity(4 * N as usize),
    };
    // Separate barrel and cap vertices preserve the sharp normal at each rim.
    for z in [-0.5, 0.5, -0.5, 0.5] {
        for i in 0..N {
            let (y, x) = (f64::from(i) * std::f64::consts::TAU / f64::from(N)).sin_cos();
            mesh.vertices.push([x, y, z]);
        }
    }
    for i in 0..N {
        let j = (i + 1) % N;
        let [bottom, top, lower, upper] = [2, 2 + N, 2 + 2 * N, 2 + 3 * N];
        mesh.triangles.extend([
            [0, bottom + j, bottom + i],
            [1, top + i, top + j],
            [lower + i, lower + j, upper + j],
            [lower + i, upper + j, upper + i],
        ]);
    }
    colored_visual_glb(&mesh, "cylinder", Some(diffuse))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_flat_caps_outward_barrel_and_opaque_material() {
        let color = [0.2, 0.35, 0.87, 1.0];
        let bytes = cylinder_visual_glb(color).unwrap();
        assert_eq!(cylinder_visual_glb(color).unwrap(), bytes);
        let document = gltf::Gltf::from_slice(&bytes).unwrap();
        let primitive = document
            .meshes()
            .next()
            .unwrap()
            .primitives()
            .next()
            .unwrap();
        assert_eq!(
            primitive
                .material()
                .pbr_metallic_roughness()
                .base_color_factor(),
            color
        );
        let reader = primitive.reader(|_| document.blob.as_deref());
        let points: Vec<_> = reader.read_positions().unwrap().collect();
        let normals: Vec<_> = reader.read_normals().unwrap().collect();
        let indices: Vec<_> = reader.read_indices().unwrap().into_u32().collect();
        assert_eq!(indices.len(), 3 * 512);
        let mut caps = [0; 2];
        for tri in indices.chunks_exact(3) {
            let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| glam::Vec3::from(points[i as usize]));
            let outward = (b - a).cross(c - a).normalize();
            if a.y == b.y && b.y == c.y {
                let sign = a.y.signum();
                assert_eq!(a.y, 0.5 * sign);
                assert!((outward.y - sign).abs() <= f32::EPSILON);
                for &i in tri {
                    assert_eq!(normals[i as usize], [0.0, sign, 0.0]);
                }
                caps[usize::from(sign > 0.0)] += 1;
            } else {
                for &i in tri {
                    let [x, _, z] = points[i as usize];
                    assert!((x.hypot(z) - 1.0).abs() < 1e-6);
                    assert!(outward.x * x + outward.z * z > 0.999);
                    assert_eq!(normals[i as usize][1], 0.0);
                }
            }
        }
        assert_eq!(caps, [128, 128]);
        for color in [[0.0; 4], [2.0; 4], [f32::NAN, 0.0, 0.0, 1.0]] {
            assert!(cylinder_visual_glb(color).is_err());
        }
    }
}
