// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resource envelopes around the maintained streaming STL reader.
use super::{FILE_BYTES, MAX_TRIANGLES, MAX_VERTICES, MeshError, TriangleMesh, invalid};
use glam::DVec3;
use std::{collections::BTreeMap, io::Cursor};

/// Decode bounded local STL bytes into unscaled Z-up geometry.
/// Exactly equal f32 coordinates share an index; signed zero is normalized.
/// Declared normals are checked for finiteness but geometry retains its winding.
pub fn decode(bytes: &[u8]) -> Result<TriangleMesh, MeshError> {
    if bytes.is_empty() || bytes.len() > FILE_BYTES {
        return Err(invalid("STL input is empty or exceeds 16 MiB"));
    }
    let binary_count = bytes.get(80..84).and_then(|count| {
        let count = u32::from_le_bytes(count.try_into().ok()?) as usize;
        count
            .checked_mul(50)
            .and_then(|n| n.checked_add(84))
            .filter(|length| *length == bytes.len())
            .map(|_| count)
    });
    // stl_io probes ASCII first. A binary header is arbitrary and may start
    // with `solid `; masking its ignored first byte selects its binary reader.
    // This bounded working copy never replaces the original source bytes.
    let mut binary;
    let input = if let Some(count) = binary_count {
        if count == 0 || count > MAX_TRIANGLES {
            return Err(invalid("STL triangle count is empty or exceeds 262144"));
        }
        if bytes[84..]
            .chunks_exact(50)
            .any(|face| face[48..50] != [0, 0])
        {
            return Err(invalid("STL color/attribute payloads are unsupported"));
        }
        binary = bytes.to_vec();
        binary[0] = 0;
        binary.as_slice()
    } else {
        let text = std::str::from_utf8(bytes)
            .map_err(|_| invalid("STL binary length or ASCII encoding is invalid"))?;
        if text.lines().any(|line| line.len() > 2048) {
            return Err(invalid("STL ASCII line exceeds 2048 bytes"));
        }
        let mut end = false;
        for line in text.lines().filter(|line| !line.trim().is_empty()) {
            if end {
                return Err(invalid("STL data follows endsolid"));
            }
            end = line.split_whitespace().next() == Some("endsolid");
        }
        if !end {
            return Err(invalid("STL ASCII needs one terminal endsolid"));
        }
        bytes
    };
    let mut cursor = Cursor::new(input);
    let reader = stl_io::create_stl_reader(&mut cursor)
        .map_err(|error| invalid(format!("invalid STL: {error}")))?;
    let mut indexed = BTreeMap::new();
    let mut mesh = TriangleMesh {
        vertices: Vec::new(),
        triangles: Vec::new(),
    };
    for face in reader {
        if mesh.triangles.len() == MAX_TRIANGLES {
            return Err(invalid("STL exceeds 262144 triangles"));
        }
        let face = face.map_err(|error| invalid(format!("invalid STL: {error}")))?;
        if face.normal.0.iter().any(|v| !v.is_finite()) {
            return Err(invalid("STL normal must be finite"));
        }
        let mut indices = [0; 3];
        for (index, point) in indices.iter_mut().zip(face.vertices) {
            if point.0.iter().any(|v| !v.is_finite()) {
                return Err(invalid("STL vertex must be finite"));
            }
            let point = point.0.map(|v| if v == 0.0 { 0.0 } else { v });
            let key = point.map(f32::to_bits);
            *index = if let Some(&id) = indexed.get(&key) {
                id
            } else {
                if mesh.vertices.len() == MAX_VERTICES {
                    return Err(invalid("STL exceeds 131072 unique vertices"));
                }
                let id = u32::try_from(mesh.vertices.len())
                    .map_err(|_| invalid("STL vertex index overflow"))?;
                indexed.insert(key, id);
                mesh.vertices.push(point.map(f64::from));
                id
            };
        }
        normal(&mesh.vertices, indices, "STL")?;
        mesh.triangles.push(indices);
    }
    if mesh.triangles.is_empty() {
        return Err(invalid("STL has no triangles"));
    }
    Ok(mesh)
}

pub(super) fn normal(
    vertices: &[[f64; 3]],
    triangle: [u32; 3],
    kind: &str,
) -> Result<DVec3, MeshError> {
    let [a, b, c] = triangle.map(|i| DVec3::from(vertices[i as usize]));
    let cross = (b - a).cross(c - a);
    if !cross.is_finite() || !cross.length_squared().is_finite() || cross.length_squared() == 0.0 {
        return Err(invalid(format!(
            "{kind} triangle must be finite and nondegenerate"
        )));
    }
    Ok(cross)
}

pub(super) fn scaled(bytes: &[u8], scale: [f64; 3]) -> Result<TriangleMesh, MeshError> {
    scale_mesh(decode(bytes)?, scale, "STL")
}

pub(super) fn scale_mesh(
    mut mesh: TriangleMesh,
    scale: [f64; 3],
    kind: &str,
) -> Result<TriangleMesh, MeshError> {
    for point in &mut mesh.vertices {
        for (value, scale) in point.iter_mut().zip(scale) {
            *value *= scale;
            if !value.is_finite() {
                return Err(invalid(format!("{kind} scaled vertex must remain finite")));
            }
        }
    }
    for triangle in &mesh.triangles {
        normal(&mesh.vertices, *triangle, kind)?;
    }
    Ok(mesh)
}

/// Convert admitted STL to an immutable self-contained glTF 2.0 binary visual.
/// Positions remain unscaled: the existing Scene/viewer applies SDF scale once.
/// STL Z-up is rotated to glTF Y-up; the accepted viewer reverses this basis.
/// Vertex normals are area-weighted; exact cancellation uses the first incident
/// face normal. No input triangle is dropped, welded approximately or repaired.
pub fn visual_glb(bytes: &[u8]) -> Result<Vec<u8>, MeshError> {
    mesh_visual_glb(&decode(bytes)?, "STL")
}

pub(super) fn mesh_visual_glb(mesh: &TriangleMesh, kind: &str) -> Result<Vec<u8>, MeshError> {
    let mut sums = vec![DVec3::ZERO; mesh.vertices.len()];
    let mut first = vec![DVec3::ZERO; mesh.vertices.len()];
    for &triangle in &mesh.triangles {
        let cross = normal(&mesh.vertices, triangle, kind)?;
        for i in triangle {
            let i = i as usize;
            if first[i] == DVec3::ZERO {
                first[i] = cross;
            }
            sums[i] += cross;
        }
    }
    let mut bin = Vec::new();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for point in &mesh.vertices {
        let point = [point[0] as f32, point[2] as f32, -point[1] as f32];
        for i in 0..3 {
            min[i] = min[i].min(point[i]);
            max[i] = max[i].max(point[i]);
        }
        for value in point {
            bin.extend(value.to_le_bytes());
        }
    }
    let normals_offset = bin.len();
    for (sum, fallback) in sums.into_iter().zip(first) {
        let normal = if sum == DVec3::ZERO { fallback } else { sum }.normalize();
        for value in [normal.x as f32, normal.z as f32, -normal.y as f32] {
            bin.extend(value.to_le_bytes());
        }
    }
    let indices_offset = bin.len();
    for triangle in &mesh.triangles {
        for index in triangle {
            bin.extend(index.to_le_bytes());
        }
    }
    let document = serde_json::json!({
        "asset":{"version":"2.0"}, "scene":0, "scenes":[{"nodes":[0]}],
        "nodes":[{"mesh":0}], "meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1},"indices":2}]}],
        "buffers":[{"byteLength":bin.len()}],
        "bufferViews":[
            {"buffer":0,"byteOffset":0,"byteLength":normals_offset,"target":34962},
            {"buffer":0,"byteOffset":normals_offset,"byteLength":indices_offset-normals_offset,"target":34962},
            {"buffer":0,"byteOffset":indices_offset,"byteLength":bin.len()-indices_offset,"target":34963}],
        "accessors":[
            {"bufferView":0,"componentType":5126,"count":mesh.vertices.len(),"type":"VEC3","min":min,"max":max},
            {"bufferView":1,"componentType":5126,"count":mesh.vertices.len(),"type":"VEC3"},
            {"bufferView":2,"componentType":5125,"count":mesh.triangles.len()*3,"type":"SCALAR"}]
    });
    let mut json = serde_json::to_vec(&document).map_err(|error| invalid(error.to_string()))?;
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let total = 12 + 8 + json.len() + 8 + bin.len();
    if total > FILE_BYTES {
        return Err(invalid(format!("generated {kind} visual exceeds 16 MiB")));
    }
    let word =
        |value| u32::try_from(value).map_err(|_| invalid(format!("{kind} GLB size overflow")));
    let mut glb = Vec::with_capacity(total);
    for value in [0x4654_6c67, 2, word(total)?, word(json.len())?, 0x4e4f_534a] {
        glb.extend(value.to_le_bytes());
    }
    glb.extend(json);
    glb.extend(word(bin.len())?.to_le_bytes());
    glb.extend(0x004e_4942_u32.to_le_bytes());
    glb.extend(bin);
    Ok(glb)
}
