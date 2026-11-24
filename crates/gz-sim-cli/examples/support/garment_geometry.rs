// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A self-contained, double-sided indexed glTF surface in Gazebo coordinates.
use serde_json::json;
use std::io::{self, ErrorKind};

fn invalid(message: &str) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, message)
}

pub fn surface_glb(
    generator: &str,
    mesh_name: &str,
    particles: &[[f64; 3]],
    triangles: &[[usize; 3]],
    color: [f32; 3],
) -> io::Result<Vec<u8>> {
    if particles.len() < 3
        || particles.len() > u16::MAX as usize
        || triangles.is_empty()
        || !color.iter().all(|v| (0.0..=1.0).contains(v))
    {
        return Err(invalid("invalid cloth geometry or color"));
    }
    let mut bin = Vec::new();
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    let mut positions = Vec::with_capacity(particles.len());
    for &[x, y, z] in particles {
        // gzweb rotates glTF Y-up to the Gazebo world on import.
        let position = [x as f32, z as f32, -y as f32];
        for (i, &v) in position.iter().enumerate() {
            if !v.is_finite() {
                return Err(invalid("non-finite cloth vertex"));
            }
            lo[i] = lo[i].min(v);
            hi[i] = hi[i].max(v);
            bin.extend_from_slice(&v.to_le_bytes());
        }
        positions.push(position.map(f64::from));
    }
    let position_bytes = bin.len();
    let mut normals = vec![[0.; 3]; particles.len()];
    for &[a, b, c] in triangles {
        if [a, b, c].iter().any(|&i| i >= particles.len()) {
            return Err(invalid("cloth index out of range"));
        }
        let u: [f64; 3] = std::array::from_fn(|i| positions[b][i] - positions[a][i]);
        let v: [f64; 3] = std::array::from_fn(|i| positions[c][i] - positions[a][i]);
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        for index in [a, b, c] {
            for (sum, d) in normals[index].iter_mut().zip(n) {
                *sum += d;
            }
        }
    }
    for n in normals {
        let length = n[0].hypot(n[1]).hypot(n[2]);
        if length == 0. {
            return Err(invalid("cloth vertex has no surface normal"));
        }
        for v in n {
            bin.extend_from_slice(&((v / length) as f32).to_le_bytes());
        }
    }
    let indices_start = bin.len();
    for &index in triangles.iter().flatten() {
        bin.extend_from_slice(&(index as u16).to_le_bytes());
    }
    let index_bytes = bin.len() - indices_start;
    let doc = json!({
        "asset":{"version":"2.0","generator":generator},
        "scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],
        "meshes":[{"name":mesh_name,"primitives":[{"attributes":{"POSITION":0,"NORMAL":1},"indices":2,"material":0}]}],
        "materials":[{"doubleSided":true,"pbrMetallicRoughness":{"baseColorFactor":[color[0],color[1],color[2],1.0],"metallicFactor":0,"roughnessFactor":1}}],
        "buffers":[{"byteLength":bin.len()}],"bufferViews":[
            {"buffer":0,"byteOffset":0,"byteLength":position_bytes,"target":34962},
            {"buffer":0,"byteOffset":position_bytes,"byteLength":position_bytes,"target":34962},
            {"buffer":0,"byteOffset":indices_start,"byteLength":index_bytes,"target":34963}],
        "accessors":[{"bufferView":0,"componentType":5126,"count":particles.len(),"type":"VEC3","min":lo,"max":hi},
            {"bufferView":1,"componentType":5126,"count":particles.len(),"type":"VEC3"},
            {"bufferView":2,"componentType":5123,"count":triangles.len()*3,"type":"SCALAR"}]
    });
    let mut header = serde_json::to_vec(&doc)?;
    header.resize(header.len().next_multiple_of(4), b' ');
    bin.resize(bin.len().next_multiple_of(4), 0);
    let length = u32::try_from(28 + header.len() + bin.len())
        .map_err(|_| invalid("GLB exceeds format limit"))?;
    let mut glb = Vec::with_capacity(length as usize);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(&length.to_le_bytes());
    for (kind, data) in [(b"JSON", header), (b"BIN\0", bin)] {
        glb.extend_from_slice(&(data.len() as u32).to_le_bytes());
        glb.extend_from_slice(kind);
        glb.extend_from_slice(&data);
    }
    Ok(glb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexed_surface_preserves_world_vertices_winding_and_units() {
        let particles = [[0.01, 0.02, 0.03], [0.05, 0.02, 0.03], [0.01, 0.08, 0.03]];
        let color = [0.125, 0.25, 0.5];
        let bytes = surface_glb(
            "gz-sim-rs garment replay",
            "XPBD swatch",
            &particles,
            &[[0, 1, 2]],
            color,
        )
        .unwrap();
        let glb = gltf::Gltf::from_slice(&bytes).unwrap();
        let bin = glb.blob.as_ref().unwrap();
        let primitive = glb.meshes().next().unwrap().primitives().next().unwrap();
        assert_eq!(primitive.mode(), gltf::mesh::Mode::Triangles);
        assert!(primitive.material().double_sided());
        assert_eq!(
            primitive
                .material()
                .pbr_metallic_roughness()
                .base_color_factor(),
            [0.125, 0.25, 0.5, 1.0]
        );
        let read_vec3 = |semantic| {
            let accessor = primitive.get(&semantic).unwrap();
            assert_eq!(accessor.count(), 3);
            let view = accessor.view().unwrap();
            let start = view.offset() + accessor.offset();
            bin[start..start + view.length()]
                .chunks_exact(12)
                .map(|xyz| {
                    std::array::from_fn::<_, 3, _>(|i| {
                        f32::from_le_bytes(xyz[i * 4..(i + 1) * 4].try_into().unwrap())
                    })
                })
                .collect::<Vec<_>>()
        };
        let positions = read_vec3(gltf::Semantic::Positions);
        for (world, [x, up, back]) in particles.iter().zip(positions) {
            assert_eq!(world.map(|v| v as f32), [x, -back, up]);
        }
        assert_eq!(read_vec3(gltf::Semantic::Normals), [[0., 1., 0.]; 3]);
        let indices = primitive.indices().unwrap();
        let view = indices.view().unwrap();
        assert_eq!(
            &bin[view.offset()..view.offset() + view.length()],
            &[0, 0, 1, 0, 2, 0]
        );
    }
}
