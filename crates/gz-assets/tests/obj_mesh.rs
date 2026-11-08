// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::mesh::{decode_obj, decode_stl, load, obj_visual_glb, stl_visual_glb};
use sdformat::{Mesh, MeshOptimization, Source};
use std::fs;

const OBJ: &[u8] = include_bytes!("../../../examples/meshes/m16-asymmetric.obj");
const STL: &[u8] = include_bytes!("../../../examples/meshes/m14-asymmetric-ascii.stl");
const TRIANGLE: &str = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
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
                .expect("absolute fixture")
                .into(),
            line: 1,
            column: 1,
        },
    }
}

#[test]
fn obj_indices_groups_and_attributes_preserve_geometry_without_importing_shading() {
    let expected = decode_stl(STL).expect("accepted tetrahedron");
    let actual = decode_obj(OBJ).expect("OBJ tetrahedron");
    assert_eq!(actual.vertices, expected.vertices);
    assert_eq!(actual.triangles, expected.triangles);
    assert_eq!(
        obj_visual_glb(OBJ).expect("OBJ visual"),
        stl_visual_glb(STL).expect("STL visual unchanged")
    );
    let alternate = b"  # inline comments and CRLF are admitted\r\nv -0 0 0\nv 0 2 0\nv 1 0 0\nv 0 0 3\nvt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\nvn 1 0 0\ng first\ns 1\nf -4//-1 -3//-1 -2//-1 # base\nf 1/1/1 3/3/1 4/4/1\ng second\nf 1/1 4/4 2/2\nf 3 2 4\n";
    let actual = decode_obj(alternate).expect("relative and all approved corner forms");
    assert_eq!(actual.vertices, expected.vertices);
    assert_eq!(actual.triangles, expected.triangles);
    assert_eq!(
        obj_visual_glb(alternate).expect("recomputed visual"),
        obj_visual_glb(OBJ).expect("same geometry")
    );
    assert_eq!(actual.vertices[0][0].to_bits(), 0.0_f64.to_bits());
}

#[test]
fn obj_visual_round_trip_and_nonuniform_scale_use_one_f32_backed_geometry() {
    let root = tempfile::tempdir().expect("fixture");
    fs::write(root.path().join("shape.obj"), OBJ).expect("OBJ");
    let glb = obj_visual_glb(OBJ).expect("generated GLB");
    let document = gltf::Gltf::from_slice(&glb).expect("maintained glTF reader");
    let primitive = document
        .meshes()
        .next()
        .expect("mesh")
        .primitives()
        .next()
        .expect("primitive");
    let reader = primitive.reader(|_| document.blob.as_deref());
    assert_eq!(
        reader
            .read_positions()
            .expect("positions")
            .collect::<Vec<_>>(),
        vec![[0., 0., 0.], [0., 0., -2.], [1., 0., 0.], [0., 3., 0.]]
    );
    for n in reader.read_normals().expect("geometric normals") {
        assert!((n.into_iter().map(|v| f64::from(v).powi(2)).sum::<f64>() - 1.).abs() < 1e-6);
    }
    fs::write(root.path().join("visual.glb"), glb).expect("GLB");
    let obj = load(&input(root.path(), "shape.obj", [2., 3., 5.])).expect("scaled OBJ");
    let visual =
        load(&input(root.path(), "visual.glb", [2., 3., 5.])).expect("existing visual basis");
    assert_eq!(obj.triangles, visual.triangles);
    for (a, b) in obj
        .vertices
        .iter()
        .flatten()
        .zip(visual.vertices.iter().flatten())
    {
        assert!((a - b).abs() < 1e-12);
    }
    let original = decode_obj(OBJ).expect("unscaled");
    for (point, base) in obj.vertices.iter().zip(original.vertices) {
        assert_eq!(*point, [base[0] * 2., base[1] * 3., base[2] * 5.]);
    }
    for scale in [
        [-1., 1., 1.],
        [0., 1., 1.],
        [f64::NAN; 3],
        [f64::MAX; 3],
        [f64::MIN_POSITIVE; 3],
    ] {
        assert!(load(&input(root.path(), "shape.obj", scale)).is_err());
    }
    assert!(
        decode_obj(b"v 16777216 0 0\nv 16777217 0 0\nv 16777216 1 0\nf 1 2 3\n").is_err(),
        "f32 rounding must not collapse a face silently"
    );
    assert!(decode_obj(b"v 0 0 0\nv 1e-50 0 0\nv 0 1e-50 0\nf 1 2 3\n").is_err());
}

#[test]
fn obj_malformed_and_external_or_nontriangle_syntax_fails_closed() {
    for bytes in [
        b"".as_slice(),
        b"\xff",
        b"# no geometry\n",
        b"v 0\nf 1 1 1\n",
        b"v 0 0 0\0\n",
        b"v 0 0 0\\\n",
    ] {
        assert!(decode_obj(bytes).is_err());
    }
    for directive in [
        "mtllib ../../outside.mtl",
        "mtllib https://example.invalid/model.mtl",
        "usemtl material",
        "l 1 2",
        "p 1",
        "curv 1 2 3",
        "vp 0 0 0",
        "unknown data",
        "v 0 0 0 1",
        "v 0 0 0 1 0 0",
        "vt 0 0 1",
        "vn 0 0 1 extra",
        "s nonsense",
        "s",
        "g",
        "o a b",
        "g ../outside",
    ] {
        let value = format!("{TRIANGLE}{directive}\n");
        assert!(decode_obj(value.as_bytes()).is_err(), "refuse {directive}");
    }
    for face in [
        "f",
        "f 1",
        "f 1 2",
        "f 1 2 3 1",
        "f 0 2 3",
        "f -4 2 3",
        "f 4 2 3",
        "f 9999999999999 2 3",
        "f bad 2 3",
        "f 1/ 2 3",
        "f 1// 2 3",
        "f 1///1 2 3",
        "f /1/1 2 3",
        "f 1/0/1 2/1/1 3/1/1",
        "f 1/2/1 2/1/1 3/1/1",
        "f 1/1/0 2/1/1 3/1/1",
        "f 1/1/-2 2/1/1 3/1/1",
    ] {
        let value = format!("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\n{face}\n");
        assert!(decode_obj(value.as_bytes()).is_err(), "refuse {face}");
    }
    assert!(
        decode_obj(b"f 1 2 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\n").is_err(),
        "forward references"
    );
    assert!(decode_obj(b"v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n").is_err());
    for declaration in ["v NaN 0 0", "vt inf 0", "vn 0 1e40 0"] {
        assert!(
            decode_obj(format!("{TRIANGLE}{declaration}\n").as_bytes()).is_err(),
            "even unused {declaration}"
        );
    }
}

#[test]
fn obj_envelopes_bound_eager_parser_input_without_reducing_face_capacity() {
    assert!(decode_obj(&vec![b' '; 16 * 1024 * 1024 + 1]).is_err());
    assert!(decode_obj(format!("#{}\n{TRIANGLE}", "x".repeat(2048)).as_bytes()).is_err());
    assert!(decode_obj(format!("o {}\n{TRIANGLE}", "x".repeat(256)).as_bytes()).is_err());
    for directive in ["v 0 0 0\n", "vt 0 0\n", "vn 0 0 1\n"] {
        let excessive = format!("{}{TRIANGLE}", directive.repeat(131073));
        assert!(decode_obj(excessive.as_bytes()).is_err());
    }
    let prefix = "v 0 0 0\nv 1 0 0\nv 0 1 0\n";
    let at_limit = format!("{prefix}{}", "f 1 2 3\n".repeat(262144));
    let mesh =
        decode_obj(at_limit.as_bytes()).expect("full face budget and only three unique vertices");
    assert_eq!((mesh.vertices.len(), mesh.triangles.len()), (3, 262144));
    assert!(decode_obj(format!("{at_limit}f 1 2 3\n").as_bytes()).is_err());
    let groups = format!("{TRIANGLE}{}", "g x\n".repeat(129));
    assert!(decode_obj(groups.as_bytes()).is_err());
    let grouped = format!(
        "{prefix}{}",
        (0..128)
            .map(|i| format!("g part{i}\nf 1 2 3\n"))
            .collect::<String>()
    );
    assert_eq!(
        decode_obj(grouped.as_bytes())
            .expect("128 bounded models")
            .triangles
            .len(),
        128
    );
    let model_excess = format!(
        "{prefix}f 1 2 3\n{}",
        (0..128)
            .map(|i| format!("g part{i}\nf 1 2 3\n"))
            .collect::<String>()
    );
    assert!(
        decode_obj(model_excess.as_bytes()).is_err(),
        "initial unnamed model counts too"
    );
    let mut vertices = String::new();
    for i in 0..131072 {
        vertices.push_str(&format!("v {i} {} 0\n", i % 2));
    }
    for i in 1..131071 {
        vertices.push_str(&format!("f 1 {} {}\n", i + 1, i + 2));
    }
    let mesh = decode_obj(vertices.as_bytes()).expect("full unique position budget");
    assert_eq!(mesh.vertices.len(), 131072);
}

#[test]
fn obj_local_resource_admission_preserves_options_and_symlink_boundary() {
    let root = tempfile::tempdir().expect("fixture");
    fs::write(root.path().join("shape.obj"), OBJ).expect("OBJ");
    let mut mesh = input(root.path(), "shape.obj", [1.; 3]);
    mesh.submesh = "part".into();
    assert!(load(&mesh).is_err());
    mesh.submesh.clear();
    mesh.center_submesh = true;
    assert!(load(&mesh).is_err());
    mesh.center_submesh = false;
    mesh.optimization = MeshOptimization::ConvexHull;
    assert!(load(&mesh).is_err());
    let mut mesh = input(root.path(), "https://example.invalid/a.obj", [1.; 3]);
    assert!(load(&mesh).is_err());
    mesh.uri = "shape.obj?x=1".into();
    assert!(load(&mesh).is_err());
    fs::create_dir(root.path().join("folder.obj")).expect("directory");
    assert!(load(&input(root.path(), "folder.obj", [1.; 3])).is_err());
    #[cfg(unix)]
    {
        let other = tempfile::tempdir().expect("outside");
        fs::write(other.path().join("shape.obj"), OBJ).expect("outside OBJ");
        std::os::unix::fs::symlink(
            other.path().join("shape.obj"),
            root.path().join("escape.obj"),
        )
        .expect("symlink");
        assert!(load(&input(root.path(), "escape.obj", [1.; 3])).is_err());
    }
}
