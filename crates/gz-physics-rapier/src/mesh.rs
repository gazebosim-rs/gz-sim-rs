// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission for the maintained Parry mesh constructors; no hull approximation.
use gz_physics::PhysicsError;
use rapier3d_f64::{parry::utils::ccw_face_normal, prelude::*};
use std::collections::{BTreeMap, BTreeSet};

const MAX_VERTICES: usize = 131_072;
const MAX_TRIANGLES: usize = 262_144;
// Supporting-plane admission is O(vertices * triangles), before construction.
const MAX_DYNAMIC_VERTICES: usize = 512;
const MAX_DYNAMIC_TRIANGLES: usize = 1_024;

fn invalid(message: &str) -> PhysicsError {
    PhysicsError::Native {
        engine: "rapier",
        message: message.into(),
    }
}

/// Build a static triangle surface or a closed, already-convex dynamic mesh.
///
/// Input positions are collision-local, Z-up and already scaled. Static surfaces
/// may be open or planar. Dynamic seams may repeat exactly equal coordinates;
/// signed zero is normalized when identifying them, with no epsilon welding.
/// Every dynamic face must be an outward supporting plane, and each vertex must
/// participate in one closed connected manifold. Nonrepresentable arithmetic or
/// a positive computed plane distance is refused, without a tolerance or repair.
pub(super) fn collider(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    dynamic: bool,
) -> Result<ColliderBuilder, PhysicsError> {
    if vertices.len() < 3
        || vertices.len() > MAX_VERTICES
        || triangles.is_empty()
        || triangles.len() > MAX_TRIANGLES
    {
        return Err(invalid(
            "mesh vertex/triangle count is empty or exceeds its budget",
        ));
    }
    if vertices.iter().flatten().any(|v| !v.is_finite()) {
        return Err(invalid("mesh vertices must be finite"));
    }
    for triangle in triangles {
        if triangle.iter().any(|&i| i as usize >= vertices.len()) {
            return Err(invalid("mesh triangle index is outside its vertex buffer"));
        }
        let points = triangle.map(|i| Vector::from(vertices[i as usize]));
        if ccw_face_normal(points).is_none() {
            return Err(invalid("mesh triangle needs a representable nonzero area"));
        }
    }
    if !dynamic {
        // Empty flags preserve the supplied triangles: no deletion, merge or repair.
        return ColliderBuilder::trimesh(
            vertices.iter().copied().map(Vector::from).collect(),
            triangles.to_vec(),
        )
        .map_err(|error| PhysicsError::Native {
            engine: "rapier",
            message: format!("triangle surface construction failed: {error}"),
        });
    }
    if triangles.len() > MAX_DYNAMIC_TRIANGLES {
        return Err(invalid("dynamic mesh exceeds 1024 triangles"));
    }

    let mut canonical = BTreeMap::new();
    let mut points = Vec::new();
    let mut remap = Vec::with_capacity(vertices.len());
    for vertex in vertices {
        let vertex = vertex.map(|v| if v == 0.0 { 0.0 } else { v });
        let key = vertex.map(f64::to_bits);
        let id = if let Some(&id) = canonical.get(&key) {
            id
        } else {
            if points.len() == MAX_DYNAMIC_VERTICES {
                return Err(invalid("dynamic mesh exceeds 512 distinct vertices"));
            }
            let id = points.len() as u32;
            points.push(Vector::from(vertex));
            canonical.insert(key, id);
            id
        };
        remap.push(id);
    }
    let indices: Vec<_> = triangles
        .iter()
        .map(|face| face.map(|i| remap[i as usize]))
        .collect();
    convex_boundary(&points, &indices)?;
    // Rapier 0.36.1 / Parry 0.31.1 convex_mesh assumes manifold convexity.
    // Its maintained face/edge construction is used only after proving that input.
    ColliderBuilder::convex_mesh(points, &indices)
        .ok_or_else(|| invalid("maintained convex mesh construction refused the admitted boundary"))
}

fn convex_boundary(points: &[Vector], faces: &[[u32; 3]]) -> Result<(), PhysicsError> {
    let mut unique_faces = BTreeSet::new();
    let mut edges: BTreeMap<(u32, u32), Vec<(usize, usize)>> = BTreeMap::new();
    let mut incident = vec![Vec::new(); points.len()];
    for (face_id, face) in faces.iter().enumerate() {
        let mut key = *face;
        key.sort_unstable();
        if key[0] == key[1] || key[1] == key[2] || !unique_faces.insert(key) {
            return Err(invalid("dynamic mesh has a collapsed or duplicate face"));
        }
        for edge in 0..3 {
            let (a, b) = (face[edge], face[(edge + 1) % 3]);
            let entries = edges.entry((a.min(b), a.max(b))).or_default();
            entries.push((face_id, edge));
            if entries.len() > 2 {
                return Err(invalid("dynamic mesh has a nonmanifold edge"));
            }
            incident[a as usize].push(face_id);
        }
    }
    let mut adjacent = vec![[0; 3]; faces.len()];
    for entries in edges.values() {
        let [(first, a), (second, b)] = entries.as_slice() else {
            return Err(invalid("dynamic mesh must be closed"));
        };
        if faces[*first][*a] != faces[*second][(*b + 1) % 3]
            || faces[*first][(*a + 1) % 3] != faces[*second][*b]
        {
            return Err(invalid(
                "dynamic mesh edges must have opposite face winding",
            ));
        }
        adjacent[*first][*a] = *second;
        adjacent[*second][*b] = *first;
    }
    let mut visited = BTreeSet::new();
    let mut pending = vec![0];
    while let Some(face) = pending.pop() {
        if visited.insert(face) {
            pending.extend(adjacent[face]);
        }
    }
    if visited.len() != faces.len() {
        return Err(invalid("dynamic mesh must have one connected boundary"));
    }
    for (vertex, incident_faces) in incident.iter().enumerate() {
        let Some(&first) = incident_faces.first() else {
            return Err(invalid("dynamic mesh contains an unused vertex"));
        };
        let mut fan = BTreeSet::new();
        let mut pending = vec![first];
        while let Some(face) = pending.pop() {
            if fan.insert(face) {
                pending.extend(
                    adjacent[face]
                        .into_iter()
                        .filter(|&other| faces[other].contains(&(vertex as u32))),
                );
            }
        }
        if fan.len() != incident_faces.len() {
            return Err(invalid("dynamic mesh has a nonmanifold vertex"));
        }
    }
    for face in faces {
        let [a, b, c] = face.map(|i| points[i as usize]);
        let normal = ccw_face_normal([a, b, c])
            .ok_or_else(|| invalid("dynamic face normal is not representable"))?;
        let mut interior = false;
        for (i, point) in points.iter().enumerate() {
            if face.contains(&(i as u32)) {
                continue;
            }
            let distance = normal.dot(*point - a);
            if !distance.is_finite() || distance > 0.0 {
                return Err(invalid(
                    "dynamic mesh must already be convex and outward-oriented",
                ));
            }
            interior |= distance < 0.0;
        }
        if !interior {
            return Err(invalid(
                "dynamic mesh must enclose representable nonzero volume",
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/mesh.rs"]
mod tests;
