// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn tetrahedron() -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    (
        vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
    )
}

fn cube() -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    (
        vec![
            [-1., -1., -1.],
            [1., -1., -1.],
            [1., 1., -1.],
            [-1., 1., -1.],
            [-1., -1., 1.],
            [1., -1., 1.],
            [1., 1., 1.],
            [-1., 1., 1.],
        ],
        vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ],
    )
}

fn rejected(vertices: &[[f64; 3]], triangles: &[[u32; 3]], dynamic: bool, diagnostic: &str) {
    let Err(error) = collider(vertices, triangles, dynamic) else {
        panic!("malformed or unsupported mesh was admitted: {diagnostic}");
    };
    let PhysicsError::Native { engine, message } = error else {
        panic!("mesh errors must retain the selected engine and diagnostic");
    };
    assert_eq!(engine, "rapier");
    assert!(message.contains(diagnostic), "{message}");
}

#[test]
fn static_surfaces_keep_open_planar_triangles_and_dynamic_solids_remain_convex() {
    let vertices = [[0., 0., 0.], [2., 0., 0.], [0., 3., 0.]];
    let triangle = [[0, 1, 2]];
    let surface = collider(&vertices, &triangle, false).unwrap().build();
    let mesh = surface.shape().as_trimesh().unwrap();
    assert_eq!(mesh.indices(), &triangle);
    assert_eq!(mesh.vertices(), &vertices.map(Vector::from));
    assert!(mesh.flags().is_empty());
    rejected(&vertices, &triangle, true, "closed");
    for (vertices, faces) in [tetrahedron(), cube()] {
        let solid = collider(&vertices, &faces, true).unwrap().build();
        let shape = solid.shape().as_convex_polyhedron().unwrap();
        assert_eq!(shape.points().len(), vertices.len());
        for point in vertices {
            assert!(shape.points().contains(&Vector::from(point)));
        }
        assert!(solid.mass_properties().inv_mass.is_finite());
        assert!(solid.mass_properties().inv_mass > 0.0);
    }
}

#[test]
fn exact_position_seams_and_signed_zero_do_not_change_the_dynamic_boundary() {
    let (vertices, faces) = tetrahedron();
    let mut split = Vec::new();
    let mut indices = Vec::new();
    for face in faces {
        let start = split.len() as u32;
        for index in face {
            let mut point = vertices[index as usize];
            if point[0] == 0.0 {
                point[0] = -0.0;
            }
            split.push(point);
        }
        indices.push([start, start + 1, start + 2]);
    }
    let solid = collider(&split, &indices, true).unwrap().build();
    let shape = solid.shape().as_convex_polyhedron().unwrap();
    assert_eq!(shape.points().len(), 4);
    for point in vertices {
        assert!(shape.points().contains(&Vector::from(point)));
    }
    // Nearby points are distinct: no epsilon seam repair.
    split[3][0] = f64::EPSILON;
    rejected(&split, &indices, true, "closed");
}

#[test]
fn concavity_reflection_and_open_boundaries_are_never_silently_hulled_or_repaired() {
    let (mut vertices, faces) = cube();
    vertices[6] = [0., 0., 0.];
    assert!(collider(&vertices, &faces, false).is_ok());
    rejected(&vertices, &faces, true, "convex and outward-oriented");
    let (mut vertices, mut faces) = tetrahedron();
    for vertex in &mut vertices {
        vertex[0] = -vertex[0];
    }
    rejected(&vertices, &faces, true, "convex and outward-oriented");
    for face in &mut faces {
        face.swap(1, 2);
    }
    assert!(collider(&vertices, &faces, true).is_ok());
    faces.pop();
    rejected(&vertices, &faces, true, "closed");
    let (vertices, mut faces) = tetrahedron();
    faces[0].swap(1, 2);
    rejected(&vertices, &faces, true, "opposite face winding");
}

#[test]
fn malformed_indices_nonfinite_or_unrepresentable_faces_fail_without_constructor_panics() {
    let (vertices, faces) = tetrahedron();
    for dynamic in [false, true] {
        rejected(&[], &faces, dynamic, "count");
        rejected(&vertices, &[], dynamic, "count");
        rejected(&vertices, &[[0, 1, u32::MAX]], dynamic, "index");
        rejected(&vertices, &[[0, 0, 1]], dynamic, "nonzero area");
        let mut collinear = vertices.clone();
        collinear[2] = [2., 0., 0.];
        rejected(&collinear, &faces, dynamic, "nonzero area");
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut invalid = vertices.clone();
            invalid[0][0] = value;
            rejected(&invalid, &faces, dynamic, "finite");
        }
        for scale in [1e-200, 1e200] {
            let scaled: Vec<_> = vertices.iter().map(|p| p.map(|v| v * scale)).collect();
            rejected(&scaled, &faces, dynamic, "nonzero area");
        }
    }
}

#[test]
fn dynamic_topology_refuses_duplicate_unused_nonmanifold_and_disconnected_input() {
    let (mut vertices, mut faces) = tetrahedron();
    faces.push(faces[0]);
    rejected(&vertices, &faces, true, "duplicate face");
    faces.pop();
    vertices.push([0.25, 0.25, 0.25]);
    rejected(&vertices, &faces, true, "unused vertex");
    faces.push([0, 1, 4]);
    rejected(&vertices, &faces, true, "nonmanifold edge");
    let (mut vertices, mut faces) = tetrahedron();
    let (second, other_faces) = tetrahedron();
    vertices.extend(second.iter().map(|p| [p[0] + 3., p[1], p[2]]));
    faces.extend(other_faces.iter().map(|f| f.map(|i| i + 4)));
    rejected(&vertices, &faces, true, "connected boundary");
    // Two coplanar, oppositely wound triangles are closed by edge counts, not a solid.
    let planar = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]];
    rejected(&planar, &[[0, 1, 2], [0, 2, 1]], true, "duplicate face");
}

#[test]
fn dynamic_plane_proof_refuses_closed_flat_volume_and_large_static_input_is_bounded() {
    let (mut flat, faces) = tetrahedron();
    flat[3] = [0.25, 0.25, 0.];
    rejected(&flat, &faces, true, "nonzero volume");
    let (vertices, faces) = tetrahedron();
    rejected(&vec![[0.; 3]; MAX_VERTICES + 1], &faces, false, "budget");
    rejected(
        &vertices,
        &vec![[0, 1, 2]; MAX_TRIANGLES + 1],
        false,
        "budget",
    );
    rejected(
        &vertices,
        &vec![faces[0]; MAX_DYNAMIC_TRIANGLES + 1],
        true,
        "1024 triangles",
    );
    let points: Vec<_> = (0..=MAX_DYNAMIC_VERTICES)
        .map(|i| [i as f64, 0., 0.])
        .chain([[0., 1., 0.], [0., 0., 1.]])
        .collect();
    rejected(
        &points,
        &[[0, 1, (points.len() - 1) as u32]],
        true,
        "512 distinct vertices",
    );
}

#[test]
fn touching_vertex_fans_are_not_a_single_manifold_boundary() {
    let (mut vertices, cube_faces) = cube();
    // The two capped ends meet at a point inside the side wall. Each edge is
    // paired and the faces are connected, but the shared vertex has two fans.
    vertices.extend([[0., 0., 0.], [0., 0., 0.]]);
    let mut faces = cube_faces[4..].to_vec();
    faces.extend([
        [8, 4, 5],
        [8, 5, 6],
        [8, 6, 7],
        [8, 7, 4],
        [9, 1, 0],
        [9, 2, 1],
        [9, 3, 2],
        [9, 0, 3],
    ]);
    rejected(&vertices, &faces, true, "nonmanifold vertex");
}
