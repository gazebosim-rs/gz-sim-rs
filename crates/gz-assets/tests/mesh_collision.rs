// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use base64::Engine;
use gz_assets::mesh::{MeshError, TriangleMesh, load};
use sdformat::{Mesh, MeshOptimization, Source};
use std::fs;
use tempfile::TempDir;

struct Fixture {
    root: TempDir,
    bytes: Vec<u8>,
    document: String,
}

impl Fixture {
    fn new(index_type: Option<u32>) -> Self {
        let root = TempDir::new().unwrap();
        let mut bytes: Vec<u8> = [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
            .into_iter()
            .flatten()
            .flat_map(f32::to_le_bytes)
            .collect();
        let (index_view, index_accessor, indices) = if let Some(component) = index_type {
            for index in 0..3_u32 {
                match component {
                    5121 => bytes.push(u8::try_from(index).unwrap()),
                    5123 => bytes.extend(u16::try_from(index).unwrap().to_le_bytes()),
                    5125 => bytes.extend(index.to_le_bytes()),
                    _ => panic!("fixture index type must be unsigned"),
                }
            }
            (
                format!(
                    r#",{{"buffer":0,"byteOffset":36,"byteLength":{}}}"#,
                    bytes.len() - 36
                ),
                format!(
                    r#",{{"bufferView":1,"componentType":{component},"count":3,"type":"SCALAR"}}"#
                ),
                r#", "indices":1"#,
            )
        } else {
            (String::new(), String::new(), "")
        };
        let document = format!(
            r#"{{"asset":{{"version":"2.0"}},
            "buffers":[{{"uri":"points.bin","byteLength":{}}}],
            "bufferViews":[{{"buffer":0,"byteLength":36}}{index_view}],
            "accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]}}{index_accessor}],
            "meshes":[{{"primitives":[{{"attributes":{{"POSITION":0}}{indices}}}]}}],
            "nodes":[{{"mesh":0}}],"scenes":[{{"nodes":[0]}}],"scene":0}}"#,
            bytes.len()
        );
        Self {
            root,
            bytes,
            document,
        }
    }

    fn replace(&mut self, from: &str, to: &str) {
        assert!(
            self.document.contains(from),
            "fixture substitution must match: {from}"
        );
        self.document = self.document.replacen(from, to, 1);
    }

    fn mesh(&self, name: &str) -> Mesh {
        Mesh {
            uri: name.into(),
            scale: [1.0; 3],
            submesh: String::new(),
            center_submesh: false,
            optimization: MeshOptimization::None,
            convex_decomposition: None,
            source: Source {
                uri: url::Url::from_file_path(self.root.path().join("world.sdf"))
                    .unwrap()
                    .into(),
                line: 3,
                column: 7,
            },
        }
    }

    fn write(&self) -> Mesh {
        fs::write(self.root.path().join("points.bin"), &self.bytes).unwrap();
        fs::write(self.root.path().join("shape.gltf"), &self.document).unwrap();
        self.mesh("shape.gltf")
    }

    fn glb(&self) -> Mesh {
        let mut json = self
            .document
            .replace(r#""uri":"points.bin","#, "")
            .into_bytes();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let mut bin = self.bytes.clone();
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let mut bytes = Vec::new();
        for word in [
            0x4654_6c67,
            2,
            u32::try_from(12 + 8 + json.len() + 8 + bin.len()).unwrap(),
            u32::try_from(json.len()).unwrap(),
            0x4e4f_534a,
        ] {
            bytes.extend(word.to_le_bytes());
        }
        bytes.extend(json);
        bytes.extend(u32::try_from(bin.len()).unwrap().to_le_bytes());
        bytes.extend(0x004e_4942_u32.to_le_bytes());
        bytes.extend(bin);
        fs::write(self.root.path().join("shape.glb"), bytes).unwrap();
        self.mesh("shape.glb")
    }
}

fn near(actual: [f64; 3], expected: [f64; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
    }
}

fn triangle(mesh: &TriangleMesh) {
    assert_eq!(mesh.triangles, [[0, 1, 2]]);
    assert_eq!(mesh.vertices.len(), 3);
    near(mesh.vertices[0], [0.0, 0.0, 0.0]);
    near(mesh.vertices[1], [1.0, 0.0, 0.0]);
    near(mesh.vertices[2], [0.0, 0.0, 1.0]);
}

fn refuses(mesh: &Mesh, message: &str) {
    let error: MeshError = load(mesh).unwrap_err();
    assert!(
        error.0.contains(message),
        "expected {message:?}, got {error}"
    );
}

#[test]
fn full_node_matrices_precede_basis_and_sdf_scale_and_reflections_reverse_winding() {
    let mut fixture = Fixture::new(Some(5123));
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""nodes":[{"matrix":[0,2,0,0,-3,0,0,0,0,0,4,0,1,2,3,1],"children":[1,2]}, {"mesh":0,"translation":[1,0,0]}, {"mesh":0,"translation":[0,1,0],"scale":[-1,1,1]}]"#,
    );
    let mut input = fixture.write();
    input.scale = [2.0, 3.0, 4.0];
    let mesh = load(&input).unwrap();
    assert_eq!(mesh.triangles, [[0, 1, 2], [3, 5, 4]]);
    assert_eq!(mesh.vertices.len(), 6);
    for (actual, expected) in mesh.vertices.into_iter().zip([
        [2.0, -9.0, 16.0],
        [2.0, -9.0, 24.0],
        [-4.0, -9.0, 16.0],
        [-4.0, -9.0, 8.0],
        [-4.0, -9.0, 0.0],
        [-10.0, -9.0, 8.0],
    ]) {
        near(actual, expected);
    }
}

#[test]
fn rotated_child_below_nonuniform_parent_preserves_accumulated_shear() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""nodes":[{"scale":[2,1,1],"children":[1]},{"mesh":0,"matrix":[1,1,0,0,-1,1,0,0,0,0,1,0,0,0,0,1]}]"#,
    );
    let mesh = load(&fixture.write()).unwrap();
    assert_eq!(mesh.triangles, [[0, 1, 2]]);
    assert_eq!(mesh.vertices.len(), 3);
    near(mesh.vertices[0], [0.0, 0.0, 0.0]);
    near(mesh.vertices[1], [2.0, 0.0, 1.0]);
    near(mesh.vertices[2], [-2.0, 0.0, 1.0]);
}

#[test]
fn unsigned_index_widths_nonindexed_glb_and_embedded_buffers_decode_the_same_triangle() {
    for component in [5121, 5123, 5125] {
        let fixture = Fixture::new(Some(component));
        triangle(&load(&fixture.write()).unwrap());
    }
    let fixture = Fixture::new(None);
    triangle(&load(&fixture.glb()).unwrap());
    assert!(
        !fixture.root.path().join("points.bin").exists(),
        "GLB must use its actual BIN chunk"
    );
    for media in ["application/octet-stream", "application/gltf-buffer"] {
        let mut fixture = Fixture::new(None);
        let uri = format!(
            "data:{media};base64,{}",
            base64::engine::general_purpose::STANDARD.encode(&fixture.bytes)
        );
        fixture.replace("points.bin", &uri);
        let input = fixture.write();
        fs::remove_file(fixture.root.path().join("points.bin")).unwrap();
        triangle(&load(&input).unwrap());
    }
}

#[test]
fn default_scene_otherwise_first_scene_selects_only_its_reachable_nodes() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""nodes":[{"mesh":0},{"mesh":0,"translation":[2,3,4]}]"#,
    );
    fixture.replace(
        r#""scenes":[{"nodes":[0]}],"scene":0"#,
        r#""scenes":[{"nodes":[0]},{"nodes":[1]}],"scene":1"#,
    );
    let selected = load(&fixture.write()).unwrap();
    assert_eq!(selected.vertices.len(), 3);
    near(selected.vertices[0], [2.0, -4.0, 3.0]);
    fixture.replace(",\"scene\":1", "");
    triangle(&load(&fixture.write()).unwrap());
}

#[test]
fn bad_local_resources_and_input_metadata_fail_with_typed_errors() {
    let fixture = Fixture::new(None);
    let input = fixture.write();
    for uri in [
        "https://example.invalid/shape.gltf",
        "model://shape/shape.gltf",
        "shape.obj",
        "shape.gltf?x=1",
        "shape.gltf#node",
    ] {
        let mut bad = input.clone();
        bad.uri = uri.into();
        let error: MeshError = load(&bad).unwrap_err();
        assert!(!error.0.is_empty());
    }
    for scale in [
        [0.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [f64::NAN, 1.0, 1.0],
        [1.0, f64::INFINITY, 1.0],
    ] {
        let mut bad = input.clone();
        bad.scale = scale;
        refuses(&bad, "positive and finite");
    }
    let mut bad = input.clone();
    bad.submesh = "part".into();
    refuses(&bad, "submesh");
    let mut bad = input.clone();
    bad.center_submesh = true;
    refuses(&bad, "submesh");
    let mut bad = input.clone();
    bad.source.uri = "https://example.invalid/world.sdf".into();
    refuses(&bad, "local SDF");
    fs::remove_file(fixture.root.path().join("points.bin")).unwrap();
    refuses(&input, "points.bin");
    fs::create_dir(fixture.root.path().join("points.bin")).unwrap();
    refuses(&input, "regular file");
}

#[test]
fn resource_paths_cannot_escape_or_invoke_remote_loading() {
    for uri in [
        "../outside.bin",
        "https://example.invalid/a.bin",
        "/absolute.bin",
        "a%2ebin",
        "a.bin?query",
        "a\\\\b.bin",
        "./points.bin",
        "a//b.bin",
    ] {
        let mut fixture = Fixture::new(None);
        fixture.replace("points.bin", uri);
        refuses(&fixture.write(), "local path subset");
    }
    let mut fixture = Fixture::new(None);
    fixture.replace("points.bin", "data:application/octet-stream;base64,!!");
    refuses(&fixture.write(), "invalid embedded");
}

#[cfg(unix)]
#[test]
fn symlinked_buffers_and_documents_cannot_leave_the_resource_directory() {
    use std::os::unix::fs::symlink;
    let fixture = Fixture::new(None);
    let input = fixture.write();
    let outside = TempDir::new().unwrap();
    fs::write(outside.path().join("outside.bin"), &fixture.bytes).unwrap();
    fs::remove_file(fixture.root.path().join("points.bin")).unwrap();
    symlink(
        outside.path().join("outside.bin"),
        fixture.root.path().join("points.bin"),
    )
    .unwrap();
    refuses(&input, "buffer symlink escapes");
    fs::write(outside.path().join("outside.gltf"), &fixture.document).unwrap();
    fs::remove_file(fixture.root.path().join("shape.gltf")).unwrap();
    symlink(
        outside.path().join("outside.gltf"),
        fixture.root.path().join("shape.gltf"),
    )
    .unwrap();
    refuses(&input, "document symlink escapes");
}

#[test]
fn accessor_bounds_indices_and_degenerate_triangles_are_rejected_before_backend_use() {
    for (from, to) in [
        (r#""count":3,"type":"VEC3""#, r#""count":4,"type":"VEC3""#),
        (
            r#""buffer":0,"byteLength":36"#,
            r#""buffer":0,"byteOffset":4,"byteLength":36"#,
        ),
    ] {
        let mut fixture = Fixture::new(None);
        fixture.replace(from, to);
        refuses(&fixture.write(), "declared buffer view");
    }
    let mut fixture = Fixture::new(Some(5121));
    fixture.bytes[38] = 3;
    refuses(&fixture.write(), "index is out of range");
    let mut fixture = Fixture::new(None);
    let repeated = fixture.bytes[12..24].to_vec();
    fixture.bytes[24..36].copy_from_slice(&repeated);
    refuses(&fixture.write(), "nondegenerate");
    let mut fixture = Fixture::new(None);
    fixture.bytes[..4].copy_from_slice(&f32::INFINITY.to_le_bytes());
    refuses(&fixture.write(), "vertex must remain finite");
    let mut fixture = Fixture::new(None);
    fixture.bytes.truncate(35);
    refuses(&fixture.write(), "invalid declared or actual byte length");
}

#[test]
fn binary_and_accessor_claims_are_bounded_without_allocating_the_claimed_geometry() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""count":3,"type":"VEC3""#,
        r#""count":131073,"type":"VEC3""#,
    );
    refuses(&fixture.write(), "accessor count");
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""uri":"points.bin","byteLength":36"#,
        r#""uri":"points.bin","byteLength":16777217"#,
    );
    refuses(&fixture.write(), "claimed mesh buffer");
    let fixture = Fixture::new(None);
    let input = fixture.write();
    fs::OpenOptions::new()
        .write(true)
        .open(fixture.root.path().join("points.bin"))
        .unwrap()
        .set_len(16 * 1024 * 1024 + 1)
        .unwrap();
    refuses(&input, "exceeds 16 MiB");
}

#[test]
fn sparse_accessors_are_refused_and_interleaved_positions_use_the_declared_stride() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""bufferView":0,"componentType":5126"#,
        r#""bufferView":0,"sparse":{"count":1,"indices":{"bufferView":0,"componentType":5121},"values":{"bufferView":0}},"componentType":5126"#,
    );
    refuses(&fixture.write(), "sparse collision accessor");
    let mut fixture = Fixture::new(None);
    let original = fixture.bytes.clone();
    fixture.bytes.clear();
    for point in original.chunks_exact(12) {
        fixture.bytes.extend(point);
        fixture.bytes.extend(123.0_f32.to_le_bytes());
    }
    fixture.replace(
        r#""uri":"points.bin","byteLength":36"#,
        r#""uri":"points.bin","byteLength":48"#,
    );
    fixture.replace(
        r#""buffer":0,"byteLength":36"#,
        r#""buffer":0,"byteLength":48,"byteStride":16"#,
    );
    triangle(&load(&fixture.write()).unwrap());
}

#[test]
fn collision_decoding_does_not_load_unneeded_images() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""images":[{"uri":"missing.png"}],"nodes":[{"mesh":0}]"#,
    );
    triangle(&load(&fixture.write()).unwrap());
    assert!(!fixture.root.path().join("missing.png").exists());
}

#[test]
fn skin_morph_animation_and_nontriangle_modes_are_explicitly_unsupported() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""skins":[{"joints":[1]}],"nodes":[{"mesh":0,"skin":0},{}]"#,
    );
    refuses(&fixture.write(), "skinned or morph");
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""attributes":{"POSITION":0}"#,
        r#""targets":[{"POSITION":0}],"attributes":{"POSITION":0}"#,
    );
    refuses(&fixture.write(), "without morph targets");
    let mut fixture = Fixture::new(None);
    fixture.replace(r#""nodes":[{"mesh":0}]"#, r#""animations":[{"channels":[{"sampler":0,"target":{"node":0,"path":"translation"}}],"samplers":[{"input":1,"output":0}]}],"nodes":[{"mesh":0}]"#);
    fixture.replace(r#""max":[1,1,0]}]"#, r#""max":[1,1,0]},{"bufferView":1,"componentType":5126,"count":3,"type":"SCALAR","min":[0],"max":[2]}]"#);
    fixture.replace(
        r#""uri":"points.bin","byteLength":36"#,
        r#""uri":"points.bin","byteLength":48"#,
    );
    fixture.replace(r#""bufferViews":[{"buffer":0,"byteLength":36}]"#, r#""bufferViews":[{"buffer":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12}]"#);
    fixture
        .bytes
        .extend([0.0_f32, 1.0, 2.0].into_iter().flat_map(f32::to_le_bytes));
    refuses(&fixture.write(), "animated collision input");
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""attributes":{"POSITION":0}"#,
        r#""mode":1,"attributes":{"POSITION":0}"#,
    );
    refuses(&fixture.write(), "require TRIANGLES");
}

#[test]
fn versions_empty_scenes_singular_and_projective_nodes_do_not_succeed() {
    for (from, to, message) in [
        (r#""version":"2.0""#, r#""version":"1.0""#, "version2.0"),
        (
            r#""version":"2.0""#,
            r#""version":"2.0","minVersion":"2.1""#,
            "version2.0",
        ),
        (r#""nodes":[{"mesh":0}]"#, r#""nodes":[{}]"#, "no triangles"),
        (
            r#""nodes":[{"mesh":0}]"#,
            r#""nodes":[{"mesh":0,"scale":[0,1,1]}]"#,
            "nonsingular",
        ),
        (
            r#""nodes":[{"mesh":0}]"#,
            r#""nodes":[{"mesh":0,"matrix":[1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1]}]"#,
            "affine",
        ),
    ] {
        let mut fixture = Fixture::new(None);
        fixture.replace(from, to);
        refuses(&fixture.write(), message);
    }
}

#[test]
fn node_depth_is_bounded_even_without_geometry_on_the_parent_chain() {
    let mut fixture = Fixture::new(None);
    let mut nodes = Vec::new();
    for index in 0..34 {
        nodes.push(format!(r#"{{"children":[{}]}}"#, index + 1));
    }
    nodes.push(r#"{"mesh":0}"#.into());
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        &format!(r#""nodes":[{}]"#, nodes.join(",")),
    );
    refuses(&fixture.write(), "depth/instance budget");
}

#[test]
fn requested_geometry_rewriting_is_not_silently_ignored() {
    let fixture = Fixture::new(None);
    let input = fixture.write();
    for optimization in [
        MeshOptimization::ConvexHull,
        MeshOptimization::ConvexDecomposition,
    ] {
        let mut requested = input.clone();
        requested.optimization = optimization;
        refuses(
            &requested,
            "optimization/decomposition requests are unsupported",
        );
    }
    let mut requested = input;
    requested.convex_decomposition = Some(sdformat::ConvexDecomposition {
        max_convex_hulls: 4,
        voxel_resolution: 100_000,
    });
    refuses(
        &requested,
        "optimization/decomposition requests are unsupported",
    );
}

#[test]
fn malformed_source_urls_filenames_and_documents_keep_resource_diagnostics() {
    let fixture = Fixture::new(None);
    let input = fixture.write();
    let mut invalid = input.clone();
    invalid.source.uri = "not an absolute URL".into();
    refuses(&invalid, "relative URL");
    for (uri, message) in [
        ("http://[", "IPv6"),
        (
            "file://remote.invalid/shape.gltf",
            "invalid local mesh file URI",
        ),
        ("a name.gltf", "ASCII path subset"),
    ] {
        let mut invalid = input.clone();
        invalid.uri = uri.into();
        refuses(&invalid, message);
    }
    let mut missing_parent = input.clone();
    missing_parent.uri = "missing/shape.gltf".into();
    refuses(&missing_parent, "missing/shape.gltf");
    fs::write(fixture.root.path().join("shape.gltf"), b"not JSON or GLB").unwrap();
    refuses(&input, "shape.gltf");
}

#[test]
fn missing_bin_scene_and_excess_declared_buffer_count_are_rejected() {
    let mut fixture = Fixture::new(None);
    fixture.replace(r#""uri":"points.bin","#, "");
    refuses(&fixture.write(), "GLB has no declared BIN chunk");
    let mut fixture = Fixture::new(None);
    fixture.replace(",\"scenes\":[{\"nodes\":[0]}],\"scene\":0", "");
    refuses(&fixture.write(), "no scene");
    let mut fixture = Fixture::new(None);
    let buffers = [r#"{"uri":"points.bin","byteLength":36}"#; 129].join(",");
    fixture.replace(
        r#""buffers":[{"uri":"points.bin","byteLength":36}]"#,
        &format!(r#""buffers":[{buffers}]"#),
    );
    refuses(&fixture.write(), "exceeds128 binary buffers");
}

#[test]
fn total_buffer_bytes_are_bounded_even_when_each_declared_file_is_within_its_limit() {
    let mut fixture = Fixture::new(None);
    let buffers = [r#"{"uri":"points.bin","byteLength":16777216}"#; 4].join(",");
    fixture.replace(
        r#""buffers":[{"uri":"points.bin","byteLength":36}]"#,
        &format!(r#""buffers":[{buffers}]"#),
    );
    let input = fixture.write();
    // A sparse temporary file bounds fixture setup memory; actual decoder reads
    // remain real and the fourth buffer plus the document exceeds 64 MiB.
    fs::OpenOptions::new()
        .write(true)
        .open(fixture.root.path().join("points.bin"))
        .unwrap()
        .set_len(16 * 1024 * 1024)
        .unwrap();
    refuses(&input, "exceeds64 MiB");
}

#[test]
fn position_and_index_accessors_require_the_exact_admitted_representations() {
    for (from, to) in [
        (r#""componentType":5126"#, r#""componentType":5123"#),
        (r#""type":"VEC3""#, r#""type":"VEC2""#),
        (
            r#""componentType":5126"#,
            r#""componentType":5126,"normalized":true"#,
        ),
    ] {
        let mut fixture = Fixture::new(None);
        fixture.replace(from, to);
        refuses(&fixture.write(), "POSITION must be unnormalized F32 VEC3");
    }
    for (from, to) in [
        (r#""type":"SCALAR""#, r#""type":"VEC2""#),
        (
            r#""componentType":5121"#,
            r#""componentType":5121,"normalized":true"#,
        ),
        (r#""componentType":5121"#, r#""componentType":5126"#),
    ] {
        let mut fixture = Fixture::new(Some(5121));
        fixture.replace(from, to);
        refuses(
            &fixture.write(),
            "indices require unsigned scalar accessors",
        );
    }
}

#[test]
fn accessors_without_storage_with_invalid_stride_or_overflowing_offset_fail_before_reading() {
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""bufferView":0,"componentType":5126"#,
        r#""componentType":5126"#,
    );
    // The maintained schema rejects missing storage before the decoder's own
    // defensive view check; the public error must retain that field identity.
    refuses(&fixture.write(), "bufferView");
    for stride in [4, 14] {
        let mut fixture = Fixture::new(None);
        fixture.replace(
            r#""buffer":0,"byteLength":36"#,
            &format!(r#""buffer":0,"byteLength":36,"byteStride":{stride}"#),
        );
        refuses(&fixture.write(), "invalid byte stride");
    }
    let mut fixture = Fixture::new(None);
    fixture.replace(
        r#""bufferView":0,"componentType":5126"#,
        &format!(
            r#""bufferView":0,"byteOffset":{},"componentType":5126"#,
            usize::MAX
        ),
    );
    refuses(&fixture.write(), "accessor byte range overflows");
    let mut fixture = Fixture::new(None);
    fixture.replace(r#""count":3,"type":"VEC3""#, r#""count":0,"type":"VEC3""#);
    refuses(&fixture.write(), "accessor count");
}

#[test]
fn repeated_geometry_cannot_exceed_the_aggregate_decoded_vertex_budget() {
    let mut fixture = Fixture::new(None);
    fixture.bytes = fixture.bytes.repeat(21_846);
    let size = fixture.bytes.len();
    fixture.replace(
        r#""uri":"points.bin","byteLength":36"#,
        &format!(r#""uri":"points.bin","byteLength":{size}"#),
    );
    fixture.replace(
        r#""buffer":0,"byteLength":36"#,
        &format!(r#""buffer":0,"byteLength":{size}"#),
    );
    fixture.replace(
        r#""count":3,"type":"VEC3""#,
        r#""count":65538,"type":"VEC3""#,
    );
    fixture.replace(
        r#""nodes":[{"mesh":0}]"#,
        r#""nodes":[{"mesh":0},{"mesh":0}]"#,
    );
    fixture.replace(
        r#""scenes":[{"nodes":[0]}]"#,
        r#""scenes":[{"nodes":[0,1]}]"#,
    );
    refuses(&fixture.write(), "decoded vertex budget");
}

#[test]
fn incomplete_triangle_sequences_and_aggregate_triangle_budget_are_explicit_failures() {
    let mut fixture = Fixture::new(Some(5121));
    fixture.replace(
        r#""componentType":5121,"count":3"#,
        r#""componentType":5121,"count":2"#,
    );
    refuses(&fixture.write(), "triangle/index budget or topology");
    let mut fixture = Fixture::new(Some(5121));
    fixture.bytes.truncate(36);
    fixture.bytes.extend([0_u8, 1, 2].repeat(1_024));
    fixture.replace(
        r#""uri":"points.bin","byteLength":39"#,
        r#""uri":"points.bin","byteLength":3108"#,
    );
    fixture.replace(
        r#""byteOffset":36,"byteLength":3"#,
        r#""byteOffset":36,"byteLength":3072"#,
    );
    fixture.replace(
        r#""componentType":5121,"count":3"#,
        r#""componentType":5121,"count":3072"#,
    );
    let nodes = [r#"{"mesh":0}"#; 257].join(",");
    fixture.replace(r#""nodes":[{"mesh":0}]"#, &format!(r#""nodes":[{nodes}]"#));
    let roots = (0..257)
        .map(|i| i.to_string())
        .collect::<Vec<_>>()
        .join(",");
    fixture.replace(
        r#""scenes":[{"nodes":[0]}]"#,
        &format!(r#""scenes":[{{"nodes":[{roots}]}}]"#),
    );
    refuses(&fixture.write(), "triangle/index budget or topology");
}

#[test]
fn shallow_node_width_is_bounded_independently_of_depth() {
    let mut fixture = Fixture::new(None);
    let nodes = std::iter::once(r#"{"mesh":0}"#)
        .chain(std::iter::repeat_n("{}", 1_024))
        .collect::<Vec<_>>()
        .join(",");
    fixture.replace(r#""nodes":[{"mesh":0}]"#, &format!(r#""nodes":[{nodes}]"#));
    let roots = (0..1_025)
        .map(|i| i.to_string())
        .collect::<Vec<_>>()
        .join(",");
    fixture.replace(
        r#""scenes":[{"nodes":[0]}]"#,
        &format!(r#""scenes":[{{"nodes":[{roots}]}}]"#),
    );
    refuses(&fixture.write(), "depth/instance budget");
}
