// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission envelope around maintained tobj; no material or filesystem access.
use super::{FILE_BYTES, MAX_TRIANGLES, MAX_VERTICES, MeshError, TriangleMesh, invalid, stl};
use std::{collections::BTreeMap, io::Cursor};

fn reference(value: &str, count: usize) -> Result<(), MeshError> {
    // Bound before upstream's optional-channel index arithmetic. Zero may not
    // become its missing-index sentinel; negative references use current counts.
    if value.is_empty() || value.len() > 12 {
        return Err(invalid("OBJ face index is missing or excessive"));
    }
    let index: i64 = value
        .parse()
        .map_err(|_| invalid("invalid OBJ face index"))?;
    let count = i64::try_from(count).map_err(|_| invalid("OBJ declaration overflow"))?;
    if index == 0 || index > count || index < -count {
        return Err(invalid("OBJ face index is zero, forward or out of bounds"));
    }
    Ok(())
}

fn corner(value: &str, counts: [usize; 3]) -> Result<(), MeshError> {
    let fields: Vec<_> = value.split('/').collect();
    match fields.as_slice() {
        [v] => reference(v, counts[0]),
        [v, vt] if !vt.is_empty() => {
            reference(v, counts[0])?;
            reference(vt, counts[1])
        }
        [v, vt, vn] if !vn.is_empty() => {
            reference(v, counts[0])?;
            if !vt.is_empty() {
                reference(vt, counts[1])?;
            }
            reference(vn, counts[2])
        }
        _ => Err(invalid("OBJ face corner has unsupported index arity")),
    }
}

fn envelope(bytes: &[u8]) -> Result<Vec<u8>, MeshError> {
    if bytes.is_empty() || bytes.len() > FILE_BYTES || bytes.contains(&0) {
        return Err(invalid(
            "OBJ input is empty, contains NUL or exceeds 16 MiB",
        ));
    }
    let text = std::str::from_utf8(bytes).map_err(|_| invalid("OBJ must be UTF-8 text"))?;
    let mut counts = [0_usize; 3];
    let (mut faces, mut groups) = (0_usize, 0_usize);
    let mut admitted = Vec::new();
    for line in text.lines() {
        if line.len() > 2048 || line.contains('\\') {
            return Err(invalid("OBJ line is excessive or uses continuation syntax"));
        }
        let line = line
            .split('#')
            .next()
            .expect("split retains first part")
            .trim();
        let words: Vec<_> = line.split_whitespace().collect();
        let Some(&command) = words.first() else {
            continue;
        };
        match command {
            "v" | "vt" | "vn" => {
                let (channel, arity) = match command {
                    "v" => (0, 3),
                    "vt" => (1, 2),
                    _ => (2, 3),
                };
                if words.len() != arity + 1 {
                    return Err(invalid("OBJ declaration arity is unsupported"));
                }
                for word in &words[1..] {
                    let value: f32 = word
                        .parse()
                        .map_err(|_| invalid("OBJ coordinate is invalid"))?;
                    if !value.is_finite() {
                        return Err(invalid("OBJ coordinate must remain finite as f32"));
                    }
                }
                counts[channel] += 1;
                if counts[channel] > MAX_VERTICES {
                    return Err(invalid("OBJ exceeds 131072 channel declarations"));
                }
            }
            "f" => {
                if words.len() != 4 {
                    return Err(invalid("OBJ requires triangle faces"));
                }
                faces += 1;
                if faces > MAX_TRIANGLES {
                    return Err(invalid("OBJ exceeds 262144 triangles"));
                }
                for value in &words[1..] {
                    corner(value, counts)?;
                }
            }
            "o" | "g" => {
                groups += 1;
                if groups > 128
                    || words.len() != 2
                    || words[1].len() > 255
                    || !words[1]
                        .bytes()
                        .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
                {
                    return Err(invalid("OBJ object/group label is outside bounded subset"));
                }
            }
            "s" => {
                if words.len() != 2 || !(words[1] == "off" || words[1].parse::<u32>().is_ok()) {
                    return Err(invalid("OBJ smoothing metadata is invalid"));
                }
            }
            _ => {
                return Err(invalid(
                    "OBJ directive/material/external resource is unsupported",
                ));
            }
        }
        admitted.extend(line.as_bytes());
        admitted.push(b'\n');
    }
    if faces == 0 {
        return Err(invalid("OBJ has no triangle faces"));
    }
    Ok(admitted)
}

/// Decode local triangle-only OBJ to unscaled Z-up geometry. f32 parsing matches
/// existing STL/glTF inputs; no decimal f64 precision preservation is promised.
pub fn decode(bytes: &[u8]) -> Result<TriangleMesh, MeshError> {
    let admitted = envelope(bytes)?;
    let options = tobj::LoadOptions {
        single_index: false,
        triangulate: false,
        ..Default::default()
    };
    let (models, materials) = tobj::load_obj_buf(&mut Cursor::new(admitted), &options, |_| {
        Err(tobj::LoadError::MaterialParseError)
    })
    .map_err(|error| invalid(format!("invalid OBJ: {error}")))?;
    if !materials
        .map_err(|_| invalid("OBJ material loading is forbidden"))?
        .is_empty()
        || models.len() > 128
    {
        return Err(invalid("OBJ model/material count is unsupported"));
    }
    let mut mesh = TriangleMesh {
        vertices: Vec::new(),
        triangles: Vec::new(),
    };
    let mut indexed = BTreeMap::new();
    for model in models {
        let parsed = model.mesh;
        if parsed.material_id.is_some()
            || !parsed.vertex_color.is_empty()
            || !parsed.positions.len().is_multiple_of(3)
            || !parsed.indices.len().is_multiple_of(3)
            || parsed.face_arities.iter().any(|&n| n != 3)
        {
            return Err(invalid("OBJ contains unsupported mesh data"));
        }
        for triangle in parsed.indices.chunks_exact(3) {
            if mesh.triangles.len() == MAX_TRIANGLES {
                return Err(invalid("OBJ aggregate triangle limit"));
            }
            let mut face = [0; 3];
            for (output, &index) in face.iter_mut().zip(triangle) {
                let start = usize::try_from(index)
                    .map_err(|_| invalid("OBJ index overflow"))?
                    .checked_mul(3)
                    .ok_or_else(|| invalid("OBJ index overflow"))?;
                let point = parsed
                    .positions
                    .get(start..start + 3)
                    .ok_or_else(|| invalid("OBJ position index out of bounds"))?;
                let point: [f32; 3] =
                    std::array::from_fn(|i| if point[i] == 0.0 { 0.0 } else { point[i] });
                if point.iter().any(|v| !v.is_finite()) {
                    return Err(invalid("OBJ position must be finite"));
                }
                let key = point.map(f32::to_bits);
                *output = if let Some(&id) = indexed.get(&key) {
                    id
                } else {
                    if mesh.vertices.len() == MAX_VERTICES {
                        return Err(invalid("OBJ exceeds 131072 unique vertices"));
                    }
                    let id = u32::try_from(mesh.vertices.len())
                        .map_err(|_| invalid("OBJ vertex index overflow"))?;
                    indexed.insert(key, id);
                    mesh.vertices.push(point.map(f64::from));
                    id
                };
            }
            stl::normal(&mesh.vertices, face, "OBJ")?;
            mesh.triangles.push(face);
        }
    }
    if mesh.triangles.is_empty() {
        return Err(invalid("OBJ has no decoded triangles"));
    }
    Ok(mesh)
}

pub(super) fn scaled(bytes: &[u8], scale: [f64; 3]) -> Result<TriangleMesh, MeshError> {
    stl::scale_mesh(decode(bytes)?, scale, "OBJ")
}

/// Immutable GLB with the existing Z-up/Y-up conversion and generated geometric
/// normals. Validated source UVs, normals, groups and smoothing do not add shading.
pub fn visual_glb(bytes: &[u8]) -> Result<Vec<u8>, MeshError> {
    stl::mesh_visual_glb(&decode(bytes)?, "OBJ")
}
