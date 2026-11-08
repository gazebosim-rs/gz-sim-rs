// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::mesh::{decode_stl, load, stl_visual_glb};
use sdformat::{Mesh, MeshOptimization, Source};
use std::fs;

const ASCII: &[u8] = include_bytes!("../../../examples/meshes/m14-asymmetric-ascii.stl");
const BINARY: &[u8] = include_bytes!("../../../examples/meshes/m14-asymmetric-binary.stl");

fn input(root: &std::path::Path, uri: &str, scale: [f64; 3]) -> Mesh {
    Mesh {
        uri: uri.into(),
        scale,
        submesh: String::new(),
        center_submesh: false,
        optimization: MeshOptimization::None,
        convex_decomposition: None,
        source: Source {
            uri: url::Url::from_file_path(root.join("world.sdf"))
                .expect("absolute fixture path")
                .into(),
            line: 1,
            column: 1,
        },
    }
}

fn binary(faces: impl IntoIterator<Item = [[f32; 3]; 3]>) -> Vec<u8> {
    let mut bytes = vec![0; 84];
    let mut count = 0_u32;
    for face in faces {
        count += 1;
        for value in [0.0_f32; 3].into_iter().chain(face.into_iter().flatten()) {
            bytes.extend(value.to_le_bytes());
        }
        bytes.extend([0, 0]);
    }
    bytes[80..84].copy_from_slice(&count.to_le_bytes());
    bytes
}

#[test]
fn ascii_and_solid_header_binary_preserve_winding_and_apply_nonuniform_scale_once() {
    let root = tempfile::tempdir().expect("fixture directory");
    let a = decode_stl(ASCII).expect("ASCII tetrahedron");
    let b = decode_stl(BINARY).expect("solid-prefixed binary tetrahedron");
    assert_eq!(a.vertices, b.vertices);
    assert_eq!(a.triangles, b.triangles);
    assert_eq!(a.vertices.len(), 4);
    assert_eq!(a.triangles.len(), 4);
    for (name, bytes) in [("a.stl", ASCII), ("b.stl", BINARY)] {
        fs::write(root.path().join(name), bytes).expect("write mesh");
        let mesh = load(&input(root.path(), name, [2.0, 3.0, 5.0])).expect("load scaled STL");
        assert_eq!(mesh.triangles, a.triangles);
        for (actual, point) in mesh.vertices.iter().zip(&a.vertices) {
            assert_eq!(*actual, [2.0 * point[0], 3.0 * point[1], 5.0 * point[2]]);
        }
    }
    let mut reflected = input(root.path(), "a.stl", [-1.0, 1.0, 1.0]);
    assert!(load(&reflected).is_err());
    reflected.scale = [1.0, f64::NAN, 1.0];
    assert!(load(&reflected).is_err());
    reflected.scale = [f64::MAX; 3];
    assert!(load(&reflected).is_err());
    reflected.scale = [f64::MIN_POSITIVE; 3];
    assert!(load(&reflected).is_err());
}

#[test]
fn generated_glb_uses_existing_gltf_loader_basis_and_valid_recomputed_normals() {
    let root = tempfile::tempdir().expect("fixture directory");
    let glb = stl_visual_glb(BINARY).expect("generate GLB");
    assert_eq!(glb, stl_visual_glb(ASCII).expect("equivalent ASCII visual"));
    let document = gltf::Gltf::from_slice(&glb).expect("maintained glTF parser");
    let blob = document.blob.as_deref().expect("embedded binary");
    let primitive = document
        .meshes()
        .next()
        .expect("mesh")
        .primitives()
        .next()
        .expect("primitive");
    let reader = primitive.reader(|_| Some(blob));
    let normals: Vec<_> = reader.read_normals().expect("normals").collect();
    assert_eq!(normals.len(), 4);
    for n in normals {
        let length: f64 = n.into_iter().map(|v| f64::from(v).powi(2)).sum();
        assert!((length - 1.0).abs() < 1e-6);
    }
    assert_eq!(
        reader
            .read_positions()
            .expect("positions")
            .collect::<Vec<_>>(),
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, -2.0],
            [1.0, 0.0, 0.0],
            [0.0, 3.0, 0.0]
        ]
    );
    fs::write(root.path().join("generated.glb"), glb).expect("write visual");
    fs::write(root.path().join("source.stl"), BINARY).expect("write STL");
    let stl = load(&input(root.path(), "source.stl", [2.0, 3.0, 5.0])).expect("STL");
    let visual = load(&input(root.path(), "generated.glb", [2.0, 3.0, 5.0]))
        .expect("GLB through existing path");
    assert_eq!(visual.triangles, stl.triangles);
    for (a, b) in visual
        .vertices
        .iter()
        .flatten()
        .zip(stl.vertices.iter().flatten())
    {
        assert!((a - b).abs() < 1e-12);
    }
}

#[test]
fn malformed_envelopes_attributes_numeric_and_topology_inputs_refuse() {
    let mut attribute = BINARY.to_vec();
    attribute[132] = 1;
    let mut nan = BINARY.to_vec();
    nan[84..88].copy_from_slice(&f32::NAN.to_le_bytes());
    let mut infinite = BINARY.to_vec();
    infinite[96..100].copy_from_slice(&f32::INFINITY.to_le_bytes());
    let mut count = BINARY.to_vec();
    count[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
    let mut trailing = BINARY.to_vec();
    trailing.push(0);
    let empty_binary = [0; 84];
    let collinear = binary([[[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]]);
    for bytes in [
        &attribute[..],
        &nan,
        &infinite,
        &count,
        &trailing,
        &BINARY[..BINARY.len() - 1],
        &collinear,
        &empty_binary,
        b"",
        b"solid empty\nendsolid empty\n",
        b"solid x\nfacet normal 0 0 0\nendsolid x\n",
    ] {
        assert!(decode_stl(bytes).is_err());
        assert!(stl_visual_glb(bytes).is_err());
    }
    let text = std::str::from_utf8(ASCII).expect("ASCII fixture");
    for text in [
        format!("{text}garbage\n"),
        format!("{text}{text}"),
        text.replace("vertex 1.0", "vertex NaN"),
        text.replace("endfacet", "endfacet extra"),
        text.replace("endsolid m14_asymmetric", ""),
    ] {
        assert!(decode_stl(text.as_bytes()).is_err());
    }
    assert!(decode_stl(&vec![0; 16 * 1024 * 1024 + 1]).is_err());
    assert!(decode_stl(format!("solid {}\nendsolid\n", "x".repeat(2049)).as_bytes()).is_err());
}

#[test]
fn face_budget_is_independent_of_unique_vertices_and_signed_zero_is_canonical() {
    let triangle = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let admitted = binary(std::iter::repeat_n(triangle, 262144));
    let mesh = decode_stl(&admitted).expect("full face budget with only three unique vertices");
    assert_eq!(mesh.triangles.len(), 262144);
    assert_eq!(mesh.vertices.len(), 3);
    let excess = binary(std::iter::repeat_n(triangle, 262145));
    assert!(decode_stl(&excess).is_err());
    let unique = binary((0..43691).map(|i| {
        let x = i as f32;
        [[x, 0., 0.], [x, 1., 0.], [x, 0., 1.]]
    }));
    assert!(decode_stl(&unique).is_err());
    let negative = binary([
        triangle,
        [[-0.0, -0.0, -0.0], [1.0, -0.0, -0.0], [-0.0, 1.0, -0.0]],
    ]);
    let mesh = decode_stl(&negative).expect("signed zero is the same point");
    assert_eq!(mesh.vertices.len(), 3);
    assert_eq!(mesh.triangles[0], mesh.triangles[1]);
}

#[test]
fn opposite_static_faces_keep_both_triangles_and_deterministic_normal_fallback() {
    let bytes = binary([
        [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
        [[0., 0., 0.], [0., 1., 0.], [1., 0., 0.]],
    ]);
    let mesh = decode_stl(&bytes).expect("open two-sided surface");
    assert_eq!(mesh.triangles.len(), 2);
    let glb = stl_visual_glb(&bytes).expect("normal cancellation fallback");
    let parsed = gltf::Gltf::from_slice(&glb).expect("GLB");
    let blob = parsed.blob.as_deref().expect("BIN");
    let primitive = parsed
        .meshes()
        .next()
        .expect("mesh")
        .primitives()
        .next()
        .expect("primitive");
    let reader = primitive.reader(|_| Some(blob));
    assert_eq!(
        reader.read_normals().expect("normals").collect::<Vec<_>>(),
        vec![[0., 1., 0.]; 3]
    );
    assert_eq!(
        reader.read_indices().expect("indices").into_u32().count(),
        6
    );
}

#[test]
fn local_resource_and_mesh_options_keep_existing_refusals() {
    let root = tempfile::tempdir().expect("fixture directory");
    fs::write(root.path().join("mesh.stl"), ASCII).expect("write mesh");
    let mut mesh = input(root.path(), "mesh.stl", [1.0; 3]);
    mesh.submesh = "part".into();
    assert!(load(&mesh).is_err());
    mesh.submesh.clear();
    mesh.center_submesh = true;
    assert!(load(&mesh).is_err());
    mesh.center_submesh = false;
    mesh.optimization = MeshOptimization::ConvexHull;
    assert!(load(&mesh).is_err());
    for uri in [
        "https://example.invalid/a.stl",
        "mesh.stl?q=1",
        "mesh.stl#x",
        "bad name.stl",
    ] {
        assert!(load(&input(root.path(), uri, [1.0; 3])).is_err());
    }
    #[cfg(unix)]
    {
        let outside = tempfile::tempdir().expect("outside directory");
        fs::write(outside.path().join("mesh.stl"), ASCII).expect("outside file");
        std::os::unix::fs::symlink(
            outside.path().join("mesh.stl"),
            root.path().join("escape.stl"),
        )
        .expect("symlink");
        assert!(load(&input(root.path(), "escape.stl", [1.0; 3])).is_err());
    }
}
