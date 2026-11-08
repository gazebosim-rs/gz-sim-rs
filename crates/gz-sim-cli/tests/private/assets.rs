// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::io::{Read, Write};

fn mesh(path: &Path) -> sdformat::Mesh {
    sdformat::Mesh {
        uri: path.to_str().unwrap().into(),
        scale: [1.5, 0.75, 2.0],
        submesh: String::new(),
        center_submesh: false,
        optimization: sdformat::MeshOptimization::None,
        convex_decomposition: None,
        source: sdformat::Source {
            uri: url::Url::from_file_path(path.with_file_name("world.sdf"))
                .unwrap()
                .into(),
            line: 1,
            column: 1,
        },
    }
}
fn document() -> &'static [u8] {
    br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data.bin","byteLength":4}],"images":[{"uri":"tex/pixel.png"}]}"#
}
fn files() -> (tempfile::TempDir, PathBuf) {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("tex")).unwrap();
    std::fs::write(dir.path().join("data.bin"), [0, 1, 2, 3]).unwrap();
    std::fs::write(dir.path().join("tex/pixel.png"), b"texture").unwrap();
    let path = dir.path().join("shape.gltf");
    std::fs::write(&path, document()).unwrap();
    (dir, path)
}

#[test]
fn declared_resources_are_immutable_scoped_and_relative() {
    let (dir, path) = files();
    let mut data = Files::new();
    let mut total = 0;
    assert_eq!(
        admit(&mesh(&path), 7, &mut data, &mut total).unwrap(),
        "/assets/7/shape.gltf"
    );
    assert_eq!(data.len(), 3);
    assert_eq!(data["/assets/7/data.bin"].as_ref(), &[0, 1, 2, 3]);
    assert_eq!(data["/assets/7/tex/pixel.png"].as_ref(), b"texture");
    std::fs::write(dir.path().join("data.bin"), b"changed after admission").unwrap();
    assert_eq!(data["/assets/7/data.bin"].as_ref(), &[0, 1, 2, 3]);
    assert_eq!(total, document().len() + 4 + 7);
    // Repeated routes do not inflate accounting.
    insert_file(
        &mut data,
        "/assets/7/data.bin".into(),
        Bytes::from_static(b"ignored"),
        &mut total,
    )
    .unwrap();
    assert_eq!(total, document().len() + 11);
}

#[test]
fn admitted_names_cannot_escape_or_change_http_meaning() {
    for allowed in ["cube.glb", "tex/a-1_2.png", ".hidden.gltf", "dir/a..b.bin"] {
        assert!(relative_path(allowed));
    }
    for rejected in [
        "",
        ".",
        "..",
        "/file",
        "a//b",
        "a/../b",
        "a/./b",
        "https://host/a",
        "data:x",
        "a?b",
        "a#b",
        "a%2fb",
        "a\\b",
        "a b",
        "é.bin",
    ] {
        assert!(!relative_path(rejected), "{rejected}");
    }
}

#[test]
fn manifest_validation_rejects_missing_invalid_remote_and_escaping_resources() {
    let (_dir, path) = files();
    let original = mesh(&path);
    for resource in [
        "missing.bin",
        "../secret",
        "http://remote/a.bin",
        "/tmp/data.bin",
        "x%2fy",
        "bad name",
    ] {
        std::fs::write(
            &path,
            String::from_utf8(document().to_vec())
                .unwrap()
                .replace("data.bin", resource),
        )
        .unwrap();
        assert!(
            admit(&original, 0, &mut Files::new(), &mut 0).is_err(),
            "{resource}"
        );
    }
    for bytes in [
        b"invalid JSON".as_slice(),
        br#"{"asset":{"version":"1.0"}}"#,
        br#"{"asset":{"version":"2.0"}}"#,
    ] {
        std::fs::write(&path, bytes).unwrap();
        assert!(admit(&original, 0, &mut Files::new(), &mut 0).is_err());
    }
    std::fs::remove_file(&path).unwrap();
    assert!(admit(&original, 0, &mut Files::new(), &mut 0).is_err());
}

#[test]
fn local_mesh_source_and_declared_dependency_symlinks_are_checked() {
    let (dir, path) = files();
    let mut value = mesh(&path);
    value.uri = "shape.gltf".into();
    assert_eq!(mesh_path(&value).unwrap(), path);
    value.uri = "a.obj".into();
    assert_eq!(mesh_path(&value).unwrap(), dir.path().join("a.obj"));
    for uri in [
        "http://host/a.gltf",
        "model://robot/a.gltf",
        "a.dae",
        "a.gltf?x",
        "a.gltf#x",
    ] {
        value.uri = uri.into();
        assert!(mesh_path(&value).is_err());
    }
    value = mesh(&path);
    value.source.uri = "https://remote/world.sdf".into();
    assert!(mesh_path(&value).is_err());
    value.source.uri = "not a URL".into();
    assert!(mesh_path(&value).is_err());
    value = mesh(&path);
    value.submesh = "piece".into();
    assert!(mesh_path(&value).is_err());
    value.submesh.clear();
    value.center_submesh = true;
    assert!(mesh_path(&value).is_err());
    let external = tempfile::tempdir().unwrap();
    std::fs::write(external.path().join("private.bin"), b"private").unwrap();
    std::fs::remove_file(dir.path().join("data.bin")).unwrap();
    std::os::unix::fs::symlink(
        external.path().join("private.bin"),
        dir.path().join("data.bin"),
    )
    .unwrap();
    assert!(admit(&mesh(&path), 0, &mut Files::new(), &mut 0).is_err());
    std::fs::write(external.path().join("shape.gltf"), document()).unwrap();
    let alias = dir.path().join("alias.gltf");
    std::os::unix::fs::symlink(external.path().join("shape.gltf"), &alias).unwrap();
    assert!(admit(&mesh(&alias), 0, &mut Files::new(), &mut 0).is_err());
}

#[test]
fn resource_byte_and_count_budgets_are_hard_limits() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("large.bin");
    let file = File::create(&path).unwrap();
    file.set_len((FILE_LIMIT + 1) as u64).unwrap();
    assert!(read_file(&path).is_err());
    file.set_len(FILE_LIMIT as u64).unwrap();
    assert_eq!(read_file(&path).unwrap().len(), FILE_LIMIT);
    let mut files = Files::new();
    let mut total = TOTAL_LIMIT;
    assert!(
        insert_file(
            &mut files,
            "new".into(),
            Bytes::from_static(b"x"),
            &mut total
        )
        .is_err()
    );
    for i in 0..FILE_COUNT {
        files.insert(i.to_string(), Bytes::new());
    }
    assert!(insert_file(&mut files, "extra".into(), Bytes::new(), &mut 0).is_err());
}

#[test]
fn http_only_serves_declared_gets_to_admitted_origins() {
    let data = BTreeMap::from([("/assets/0/a.gltf".into(), Bytes::from_static(b"document"))]);
    for origin in [
        None,
        Some("http://localhost:8080"),
        Some("http://127.0.0.1:8081"),
    ] {
        let mut builder = Request::builder().uri("/assets/0/a.gltf");
        if let Some(origin) = origin {
            builder = builder.header("Origin", origin);
        }
        let response = reply(builder.body(()).unwrap(), &data);
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()["content-type"], "model/gltf+json");
        assert_eq!(response.headers()["connection"], "close");
        assert_eq!(
            response
                .headers()
                .get("access-control-allow-origin")
                .map(|v| v.to_str().unwrap()),
            origin
        );
    }
    for uri in [
        "/assets/0/../a.gltf",
        "/assets/0/a.gltf?x",
        "http://host/assets/0/a.gltf",
        "/assets/0/%61.gltf",
        "/etc/passwd",
    ] {
        assert_ne!(
            reply(Request::builder().uri(uri).body(()).unwrap(), &data).status(),
            StatusCode::OK
        );
    }
    for (name, value) in [
        ("Origin", "https://remote"),
        ("Origin", "http://localhost:9999"),
        ("content-length", "1"),
        ("transfer-encoding", "chunked"),
    ] {
        assert_eq!(
            reply(
                Request::builder()
                    .uri("/assets/0/a.gltf")
                    .header(name, value)
                    .body(())
                    .unwrap(),
                &data
            )
            .status(),
            StatusCode::BAD_REQUEST
        );
    }
    assert_eq!(
        reply(
            Request::builder()
                .method("POST")
                .uri("/assets/0/a.gltf")
                .body(())
                .unwrap(),
            &data
        )
        .status(),
        StatusCode::BAD_REQUEST
    );
}

#[test]
fn real_http_resource_and_partial_peer_have_bounded_shutdown() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    listener.set_nonblocking(true).unwrap();
    let address = listener.local_addr().unwrap();
    let files = Arc::new(BTreeMap::from([(
        "/assets/0/data.bin".into(),
        Bytes::from_static(b"actual binary bytes"),
    )]));
    let stop = Arc::new(AtomicBool::new(false));
    let worker_stop = stop.clone();
    let worker = thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(serve(listener, files, worker_stop));
    });
    let mut stream = std::net::TcpStream::connect(address).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    stream
        .write_all(b"GET /assets/0/data.bin HTTP/1.1\r\nHost: localhost\r\n\r\n")
        .unwrap();
    let mut response = Vec::new();
    stream.read_to_end(&mut response).unwrap();
    assert!(response.starts_with(b"HTTP/1.1 200"));
    assert!(response.ends_with(b"actual binary bytes"));
    let mut partial = std::net::TcpStream::connect(address).unwrap();
    partial
        .write_all(b"GET /assets/0/data.bin HTTP/1.1\r\n")
        .unwrap();
    partial
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let mut partial_response = Vec::new();
    partial.read_to_end(&mut partial_response).unwrap();
    assert!(
        partial_response.is_empty(),
        "deadline closes incomplete headers without a successful response"
    );
    stop.store(true, Ordering::Release);
    let (done, completed) = std::sync::mpsc::channel();
    let joiner = thread::spawn(move || {
        worker.join().unwrap();
        done.send(()).unwrap();
    });
    completed
        .recv_timeout(Duration::from_secs(2))
        .expect("idle resource worker must stop promptly");
    joiner.join().unwrap();
}

fn mesh_simulation() -> Simulation {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/mvp_local_mesh.sdf")
        .canonicalize()
        .unwrap();
    gz_sim_scene::load_world(
        &std::fs::read_to_string(&path).unwrap(),
        url::Url::from_file_path(path).unwrap().as_str(),
        "default",
        Duration::from_millis(5),
        &sdformat::NoResources,
    )
    .unwrap()
}

#[test]
fn live_mesh_snapshot_tracks_physics_and_reset_without_changing_collision_shapes() {
    let mut sim = mesh_simulation();
    assert!(super::super::snapshot::extract(&sim, "default", 0).is_err());
    let assets = Assets::new(&sim).unwrap().unwrap();
    assert!(assets.healthy());
    let extract = |sim: &Simulation, sequence| {
        serde_json::from_str::<serde_json::Value>(
            &super::super::snapshot::encode(
                &super::super::snapshot::extract_with_assets(
                    sim,
                    "default",
                    sequence,
                    Some(&assets),
                )
                .unwrap(),
            )
            .unwrap(),
        )
        .unwrap()
    };
    let initial = extract(&sim, 0);
    assert_eq!(initial["version"], 2);
    let visual = initial["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|v| v["shape"]["kind"] == "mesh")
        .unwrap();
    assert_eq!(
        visual["shape"]["scale"],
        serde_json::json!([1.5, 0.75, 2.0])
    );
    let uri = url::Url::parse(visual["shape"]["uri"].as_str().unwrap()).unwrap();
    assert_eq!(uri.host_str(), Some("127.0.0.1"));
    assert_eq!(uri.port(), Some(assets.port));
    assert_eq!(uri.path(), "/assets/0/pyramid.gltf");
    assert_eq!(initial["visuals"].as_array().unwrap().len(), 3);
    sim.step_exact(20).unwrap();
    let moving = extract(&sim, 1);
    let moved = moving["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|v| v["id"] == visual["id"])
        .unwrap();
    assert!(moved["translation"][2].as_f64().unwrap() < visual["translation"][2].as_f64().unwrap());
    assert_eq!(moved["shape"], visual["shape"]);
    for (actual, expected) in moved["rotation"]
        .as_array()
        .unwrap()
        .iter()
        .zip(visual["rotation"].as_array().unwrap())
    {
        assert!((actual.as_f64().unwrap() - expected.as_f64().unwrap()).abs() < 1e-12);
    }
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
    let reset = extract(&sim, 2);
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["visuals"], initial["visuals"]);
    // The local resource bytes remain available after reset, using the same route.
    let mut stream = std::net::TcpStream::connect((Ipv4Addr::LOCALHOST, assets.port)).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    stream
        .write_all(b"GET /assets/0/pyramid.bin HTTP/1.1\r\nHost: localhost\r\n\r\n")
        .unwrap();
    let mut bytes = Vec::new();
    stream.read_to_end(&mut bytes).unwrap();
    assert!(bytes.starts_with(b"HTTP/1.1 200"));
    let old = assets.entries.values().next().unwrap().0.clone();
    assert!(assets.uri(u64::MAX, &old).is_err());
    let mut changed = old.clone();
    changed.scale[0] = 4.0;
    assert!(
        assets
            .uri(*assets.entries.keys().next().unwrap(), &changed)
            .is_err()
    );
}

#[test]
fn primitive_world_allocates_no_asset_worker_or_file_snapshot() {
    let sim = gz_sim_scene::load_world(
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/mvp_primitives.sdf"
        )),
        "mvp.sdf",
        "default",
        Duration::from_millis(5),
        &sdformat::NoResources,
    )
    .unwrap();
    assert!(Assets::new(&sim).unwrap().is_none());
}

#[test]
fn maintained_parser_accepts_glb_and_embedded_resources_without_extra_routes() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("embedded.glb");
    let mut json = br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"byteLength":4}]}"#.to_vec();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let mut glb = Vec::new();
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2_u32.to_le_bytes());
    glb.extend_from_slice(&((32 + json.len()) as u32).to_le_bytes());
    glb.extend_from_slice(&(json.len() as u32).to_le_bytes());
    glb.extend_from_slice(b"JSON");
    glb.extend(json);
    glb.extend_from_slice(&4_u32.to_le_bytes());
    glb.extend_from_slice(b"BIN\0");
    glb.extend_from_slice(&[1, 2, 3, 4]);
    assert_eq!(
        gltf::Gltf::from_slice(&glb).unwrap().blob.unwrap(),
        [1, 2, 3, 4]
    );
    std::fs::write(&path, &glb).unwrap();
    let mut data = Files::new();
    let mut total = 0;
    assert_eq!(
        admit(&mesh(&path), 0, &mut data, &mut total).unwrap(),
        "/assets/0/embedded.glb"
    );
    assert_eq!(data.len(), 1);
    assert_eq!(data.values().next().unwrap().as_ref(), glb);
    let path = dir.path().join("inline.gltf");
    std::fs::write(&path,br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data:application/octet-stream;base64,AAAAAA==","byteLength":4}]}"#).unwrap();
    assert!(admit(&mesh(&path), 1, &mut data, &mut total).is_ok());
    assert_eq!(data.len(), 2);
}

#[test]
fn binary_texture_types_and_malformed_origin_preserve_the_http_boundary() {
    let mut data = Files::new();
    for path in [
        "shape.glb",
        "texture.png",
        "texture.jpg",
        "texture.jpeg",
        "data.bin",
    ] {
        data.insert(format!("/assets/0/{path}"), Bytes::from_static(b"declared"));
    }
    for (path, mime) in [
        ("shape.glb", "model/gltf-binary"),
        ("texture.png", "image/png"),
        ("texture.jpg", "image/jpeg"),
        ("texture.jpeg", "image/jpeg"),
        ("data.bin", "application/octet-stream"),
    ] {
        let response = reply(
            Request::builder()
                .uri(format!("/assets/0/{path}"))
                .header("content-length", "0")
                .body(())
                .unwrap(),
            &data,
        );
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()["content-type"], mime);
    }
    let bad = hyper::header::HeaderValue::from_bytes(&[0xff]).unwrap();
    assert_eq!(
        reply(
            Request::builder()
                .uri("/assets/0/data.bin")
                .header("Origin", bad)
                .body(())
                .unwrap(),
            &data
        )
        .status(),
        StatusCode::BAD_REQUEST
    );
}

#[test]
fn otherwise_valid_scenes_require_explicit_supported_asset_versions() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("version.gltf");
    for (version, min_version, accepted) in [
        ("2.0", None, true),
        ("2.0", Some("2.0"), true),
        ("1.0", None, false),
        ("3.0", None, false),
        ("2.0", Some("2.1"), false),
        ("2.0", Some("garbage"), false),
    ] {
        let mut asset = serde_json::json!({"version":version});
        if let Some(version) = min_version {
            asset["minVersion"] = version.into();
        }
        let document = serde_json::json!({"asset":asset,"scene":0,"scenes":[{"nodes":[]}]});
        std::fs::write(&path, document.to_string()).unwrap();
        assert_eq!(
            admit(&mesh(&path), 0, &mut Files::new(), &mut 0).is_ok(),
            accepted,
            "{version} / {min_version:?}"
        );
    }
}

#[test]
fn repeated_xml_visuals_share_documents_despite_source_positions_and_scale() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/mvp_local_mesh.sdf")
        .canonicalize()
        .unwrap();
    let xml = std::fs::read_to_string(&path).unwrap().replace("</link></model>",r#"
      <visual name="shared"><geometry><mesh><uri>assets/m2/pyramid.gltf</uri><scale>2 1 1</scale></mesh></geometry></visual>
    </link></model>"#);
    let sim = gz_sim_scene::load_world(
        &xml,
        url::Url::from_file_path(path).unwrap().as_str(),
        "default",
        Duration::from_millis(5),
        &sdformat::NoResources,
    )
    .unwrap();
    let assets = Assets::new(&sim).unwrap().unwrap();
    assert_eq!(assets.entries.len(), 4);
    let routes: std::collections::BTreeSet<_> =
        assets.entries.values().map(|(_, route)| route).collect();
    assert_eq!(routes.len(), 1);
    assert!(
        assets
            .entries
            .values()
            .map(|(mesh, _)| mesh.source.line)
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            > 1
    );
    assert!(
        assets
            .entries
            .values()
            .any(|(mesh, _)| mesh.scale == [2., 1., 1.])
    );
    assert!(
        assets
            .entries
            .values()
            .any(|(mesh, _)| mesh.scale == [1.5, 0.75, 2.])
    );
}

#[test]
fn included_models_resolve_original_file_bases_and_do_not_share_distinct_documents() {
    let dir = tempfile::tempdir().unwrap();
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/assets/m2");
    for name in ["a", "b"] {
        let base = dir.path().join(name);
        std::fs::create_dir(&base).unwrap();
        for file in ["pyramid.gltf", "pyramid.bin", "pixel.png"] {
            std::fs::copy(fixture.join(file), base.join(file)).unwrap();
        }
        std::fs::write(base.join("body.sdf"),format!(r#"<sdf version="1.12"><model name="{name}"><link name="body"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision><visual name="mesh"><geometry><mesh><uri>pyramid.gltf</uri></mesh></geometry></visual></link></model></sdf>"#)).unwrap();
    }
    let xml = r#"<sdf version="1.12"><world name="default"><include><uri>a/body.sdf</uri></include><include><uri>b/body.sdf</uri></include></world></sdf>"#;
    let resolver =
        gz_assets::LocalResources::new(dir.path().to_path_buf(), Vec::new(), 65536).unwrap();
    let sim = gz_sim_scene::load_world(
        xml,
        url::Url::from_file_path(dir.path().join("world.sdf"))
            .unwrap()
            .as_str(),
        "default",
        Duration::from_millis(5),
        &resolver,
    )
    .unwrap();
    let assets = Assets::new(&sim).unwrap().unwrap();
    assert_eq!(assets.entries.len(), 2);
    let routes: std::collections::BTreeSet<_> =
        assets.entries.values().map(|(_, route)| route).collect();
    assert_eq!(routes.len(), 2);
    for (mesh, _) in assets.entries.values() {
        assert!(
            mesh.source.uri.ends_with("/a/body.sdf") || mesh.source.uri.ends_with("/b/body.sdf")
        );
    }
    // No mesh exists beside world.sdf; a resolution against the main world would fail.
    assert!(!dir.path().join("pyramid.gltf").exists());
}

#[test]
fn named_pipes_are_rejected_before_opening_and_reading() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("blocked.bin");
    assert!(
        std::process::Command::new("mkfifo")
            .arg(&path)
            .status()
            .unwrap()
            .success()
    );
    let (sender, receiver) = std::sync::mpsc::channel();
    let read_path = path.clone();
    let worker = thread::spawn(move || {
        sender.send(read_file(&read_path).is_err()).unwrap();
    });
    let result = receiver.recv_timeout(Duration::from_secs(1));
    if result.is_err() {
        // Release a regressed blocking open before failing, rather than leaking its thread.
        let mut writer = File::options().write(true).open(&path).unwrap();
        writer.write_all(b"not a regular file").unwrap();
        drop(writer);
    }
    worker.join().unwrap();
    assert!(result.unwrap());
    assert!(read_file(dir.path()).is_err());
}

#[test]
fn mesh_visual_bound_is_independent_of_shared_resource_file_count() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/mvp_local_mesh.sdf")
        .canonicalize()
        .unwrap();
    let load = |count: usize| {
        let visuals = (0..count).map(|i| format!(r#"<visual name="mesh{i}"><geometry><mesh><uri>assets/m2/pyramid.gltf</uri></mesh></geometry></visual>"#)).collect::<String>();
        let xml = format!(
            r#"<sdf version="1.12"><world name="default"><model name="body"><link name="body"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision>{visuals}</link></model></world></sdf>"#
        );
        gz_sim_scene::load_world(
            &xml,
            url::Url::from_file_path(&path).unwrap().as_str(),
            "default",
            Duration::from_millis(5),
            &sdformat::NoResources,
        )
        .unwrap()
    };
    let accepted = load(128);
    let assets = Assets::new(&accepted).unwrap().unwrap();
    assert_eq!(assets.entries.len(), 128);
    assert_eq!(
        assets
            .entries
            .values()
            .map(|(_, route)| route)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        1
    );
    let scene = super::super::snapshot::extract_with_assets(&accepted, "default", 0, Some(&assets))
        .unwrap();
    let json: serde_json::Value =
        serde_json::from_str(&super::super::snapshot::encode(&scene).unwrap()).unwrap();
    assert_eq!(json["visuals"].as_array().unwrap().len(), 128);
    let rejected = load(129);
    let error = match Assets::new(&rejected) {
        Err(error) => error,
        Ok(_) => panic!("129 meshes must fail independently of their shared resource count"),
    };
    assert!(error.to_string().contains("128 mesh visuals"));
}

#[test]
fn invalid_mesh_scale_has_its_own_diagnostic_before_asset_lookup() {
    let mut sim = mesh_simulation();
    let assets = Assets::new(&sim).unwrap().unwrap();
    let (entity, original) = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            let geometry = entity.get::<VisualGeometry>()?;
            if let sdformat::Geometry::Mesh(mesh) = geometry.0.as_ref() {
                Some((entity.id(), mesh.clone()))
            } else {
                None
            }
        })
        .unwrap();
    for invalid_scale in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::MAX, 1e-300] {
        let mut mesh = original.clone();
        mesh.scale[1] = invalid_scale;
        sim.world_mut()
            .entity_mut(entity)
            .insert(VisualGeometry(Arc::new(sdformat::Geometry::Mesh(mesh))));
        let error =
            match super::super::snapshot::extract_with_assets(&sim, "default", 1, Some(&assets)) {
                Err(error) => error,
                Ok(_) => panic!("invalid mesh scale accepted"),
            };
        assert!(
            error
                .to_string()
                .contains("mesh scale must be positive, finite and representable")
        );
    }
}

#[test]
fn displayed_pyramid_has_outward_faces_matching_normals_and_positive_closed_volume() {
    fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
        a.iter().zip(b).map(|(a, b)| a * b).sum()
    }
    fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }
    fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| a[i] - b[i])
    }
    fn vectors(accessor: gltf::Accessor<'_>, bytes: &[u8]) -> Vec<[f64; 3]> {
        assert_eq!(accessor.data_type(), gltf::accessor::DataType::F32);
        assert_eq!(accessor.dimensions(), gltf::accessor::Dimensions::Vec3);
        let view = accessor.view().unwrap();
        let offset = view.offset() + accessor.offset();
        let stride = view.stride().unwrap_or(12);
        (0..accessor.count())
            .map(|i| {
                std::array::from_fn(|axis| {
                    let at = offset + i * stride + axis * 4;
                    f64::from(f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()))
                })
            })
            .collect()
    }
    let document = gltf::Gltf::from_slice(include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/assets/m2/pyramid.gltf"
    )))
    .unwrap();
    let bytes = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/assets/m2/pyramid.bin"
    ));
    let primitive = document
        .meshes()
        .next()
        .unwrap()
        .primitives()
        .next()
        .unwrap();
    assert_eq!(primitive.mode(), gltf::mesh::Mode::Triangles);
    assert!(!primitive.material().double_sided());
    assert!(primitive.indices().is_none());
    let positions = vectors(primitive.get(&gltf::Semantic::Positions).unwrap(), bytes);
    let normals = vectors(primitive.get(&gltf::Semantic::Normals).unwrap(), bytes);
    assert_eq!(positions.len(), 12);
    assert_eq!(normals.len(), 12);
    let interior = [0.0, 0.25, -0.025];
    let mut volume = 0.0;
    for (triangle, normal) in positions.chunks_exact(3).zip(normals.chunks_exact(3)) {
        let face = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]));
        assert!(
            dot(face, face) > 0.0,
            "every triangle must have nonzero area"
        );
        assert!(
            dot(face, sub(interior, triangle[0])) < 0.0,
            "front faces must face away from the interior for exterior visibility"
        );
        for normal in normal {
            assert!(dot(face, *normal) > 0.0);
            assert!((dot(*normal, *normal) - 1.0).abs() < 1e-6);
        }
        volume += dot(triangle[0], cross(triangle[1], triangle[2])) / 6.0;
    }
    assert!(
        (volume - 2.0 / 15.0).abs() < 1e-7,
        "closed outward pyramid must have its analytic positive volume"
    );
}

#[test]
fn mesh_materials_are_embedded_and_sdf_overrides_are_explicitly_rejected() {
    let mut sim = mesh_simulation();
    let assets = Assets::new(&sim).unwrap().unwrap();
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<VisualGeometry>()
                .is_some_and(|geometry| matches!(geometry.0.as_ref(), sdformat::Geometry::Mesh(_)))
        })
        .unwrap()
        .id();
    assert!(
        !sim.world()
            .entity(entity)
            .contains::<gz_sim_scene::VisualMaterial>()
    );
    let initial =
        super::super::snapshot::extract_with_assets(&sim, "default", 0, Some(&assets)).unwrap();
    let initial = super::super::snapshot::encode(&initial).unwrap();
    let mut material = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<gz_sim_scene::VisualMaterial>())
        .unwrap()
        .0
        .clone();
    material.diffuse = [1., 0., 0., 1.];
    sim.world_mut()
        .entity_mut(entity)
        .insert(gz_sim_scene::VisualMaterial(material));
    let error = match super::super::snapshot::extract_with_assets(&sim, "default", 0, Some(&assets))
    {
        Err(error) => error,
        Ok(_) => {
            panic!("SDF overrides must not silently replace or disappear beside embedded materials")
        }
    };
    assert!(
        error
            .to_string()
            .contains("use embedded glTF materials; omit the SDF material override")
    );
    sim.world_mut()
        .entity_mut(entity)
        .remove::<gz_sim_scene::VisualMaterial>();
    let restored =
        super::super::snapshot::extract_with_assets(&sim, "default", 0, Some(&assets)).unwrap();
    assert_eq!(super::super::snapshot::encode(&restored).unwrap(), initial);
}

#[test]
fn real_mesh_bridge_delivers_resources_and_committed_controls_and_refuses_dead_asset_worker() {
    use super::super::Bridge;
    use serde_json::{Value, json};
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let mut simulation = mesh_simulation();
    simulation.set_paused(true);
    let mut server = gz_sim_server::Server::with_zenoh_config(
        simulation,
        "default",
        gz_sim_server::ServerOptions {
            partition: format!("m2-bridge-{}-{}", std::process::id(), address.port()),
            ..Default::default()
        },
        zenoh::Config::default(),
        std::num::NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let mut bridge = Bridge::with_listener(server.simulation(), "default", listener).unwrap();
    let (finished, completed) = std::sync::mpsc::channel();
    let client = thread::spawn(move || {
        let (mut ws, _) = tungstenite::connect(format!("ws://{address}/")).unwrap();
        if let tungstenite::stream::MaybeTlsStream::Plain(tcp) = ws.get_mut() {
            tcp.set_read_timeout(Some(Duration::from_secs(4))).unwrap();
        }
        let read = |ws: &mut tungstenite::WebSocket<_>, kind: &str| -> Value {
            loop {
                if let tungstenite::Message::Text(text) = ws.read().unwrap() {
                    let frame: Value = serde_json::from_str(&text).unwrap();
                    if frame["kind"] == kind {
                        return frame;
                    }
                }
            }
        };
        let initial = read(&mut ws, "scene");
        assert_eq!(initial["version"], 2);
        assert_eq!(initial["paused"], true);
        let mesh = initial["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .find(|v| v["shape"]["kind"] == "mesh")
            .unwrap();
        let uri = url::Url::parse(mesh["shape"]["uri"].as_str().unwrap()).unwrap();
        let buffer = uri.join("pyramid.bin").unwrap();
        let mut http =
            std::net::TcpStream::connect((Ipv4Addr::LOCALHOST, buffer.port().unwrap())).unwrap();
        http.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        write!(
            http,
            "GET {} HTTP/1.1\r\nHost: localhost\r\nOrigin: http://127.0.0.1:8081\r\n\r\n",
            buffer.path()
        )
        .unwrap();
        let mut response = Vec::new();
        http.read_to_end(&mut response).unwrap();
        assert!(response.starts_with(b"HTTP/1.1 200"));
        let body = response.windows(4).position(|v| v == b"\r\n\r\n").unwrap() + 4;
        assert_eq!(
            &response[body..],
            include_bytes!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../examples/assets/m2/pyramid.bin"
            ))
        );
        for (id, action) in [(1, "resume"), (2, "pause"), (3, "reset")] {
            ws.send(tungstenite::Message::Text(
                json!({"version":1,"kind":"control","request_id":id,"action":action})
                    .to_string()
                    .into(),
            ))
            .unwrap();
            let ack = read(&mut ws, "ack");
            assert_eq!(ack["version"], 1);
            assert_eq!(ack["request_id"], id);
            assert_eq!(ack["ok"], true);
            assert_eq!(ack["paused"], action != "resume");
            if action == "reset" {
                assert_eq!(ack["epoch"], 1);
                assert_eq!(ack["seconds"], 0);
                assert_eq!(ack["nanoseconds"], 0);
            }
        }
        let reset = read(&mut ws, "scene");
        assert_eq!(reset["version"], 2);
        assert_eq!(reset["epoch"], 1);
        assert_eq!(reset["visuals"], initial["visuals"]);
        finished.send(()).unwrap();
    });
    let deadline = std::time::Instant::now() + Duration::from_secs(8);
    while completed.try_recv().is_err() {
        assert!(
            std::time::Instant::now() < deadline,
            "real mesh bridge did not finish within its deadline"
        );
        let controls = bridge.controls(&mut server);
        server.prepare_io().unwrap();
        server.tick().unwrap();
        bridge
            .publish(server.simulation(), "default", controls)
            .unwrap();
        thread::sleep(Duration::from_millis(2));
    }
    client.join().unwrap();
    let assets = bridge.assets.as_mut().unwrap();
    assets.stop.store(true, Ordering::Release);
    assets.worker.take().unwrap().join().unwrap();
    assert!(!assets.healthy());
    let error = bridge
        .publish(server.simulation(), "default", Vec::new())
        .unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::BrokenPipe);
    drop(bridge);
    server.shutdown();
}

#[test]
fn unreadable_names_missing_resources_and_remote_file_hosts_fail_before_publication() {
    let (dir, path) = files();
    let missing = dir.path().join("missing.bin");
    let error = read_file(&missing).unwrap_err();
    assert!(error.to_string().contains("missing.bin"));
    let bad_name = dir.path().join("bad name.gltf");
    std::fs::write(&bad_name, document()).unwrap();
    assert!(
        admit(&mesh(&bad_name), 0, &mut Files::new(), &mut 0)
            .unwrap_err()
            .to_string()
            .contains("filenames require ASCII")
    );
    let mut value = mesh(&path);
    value.uri = "file://remote-host/shape.gltf".into();
    assert!(
        mesh_path(&value)
            .unwrap_err()
            .to_string()
            .contains("invalid local mesh file URI")
    );
    let mut sim = mesh_simulation();
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<VisualGeometry>()
                .is_some_and(|geometry| matches!(geometry.0.as_ref(), sdformat::Geometry::Mesh(_)))
        })
        .unwrap()
        .id();
    sim.world_mut().entity_mut(entity).remove::<EntityId>();
    let error = match Assets::new(&sim) {
        Err(error) => error,
        Ok(_) => panic!("unowned mesh must not be published"),
    };
    assert!(error.to_string().contains("mesh visual has no identity"));
}

#[test]
fn mesh_bridge_startup_rejects_missing_documents_and_explicit_material_overrides() {
    let mut sim = mesh_simulation();
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<VisualGeometry>()
                .is_some_and(|geometry| matches!(geometry.0.as_ref(), sdformat::Geometry::Mesh(_)))
        })
        .unwrap()
        .id();
    let original = sim.world().get::<VisualGeometry>(entity).unwrap().0.clone();
    let sdformat::Geometry::Mesh(original_mesh) = original.as_ref() else {
        panic!("fixture needs mesh")
    };
    let mut invalid = original_mesh.clone();
    invalid.uri = "missing.gltf".into();
    sim.world_mut()
        .entity_mut(entity)
        .insert(VisualGeometry(Arc::new(sdformat::Geometry::Mesh(invalid))));
    let error = match super::super::Bridge::with_listener(
        &sim,
        "default",
        TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap(),
    ) {
        Err(error) => error,
        Ok(_) => panic!("missing mesh document must prevent bridge startup"),
    };
    assert!(error.to_string().contains("missing.gltf"));
    sim.world_mut()
        .entity_mut(entity)
        .insert(VisualGeometry(original));
    let material = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<gz_sim_scene::VisualMaterial>())
        .unwrap()
        .clone();
    sim.world_mut().entity_mut(entity).insert(material);
    let error = match super::super::Bridge::with_listener(
        &sim,
        "default",
        TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap(),
    ) {
        Err(error) => error,
        Ok(_) => panic!("mesh material override must prevent bridge startup"),
    };
    assert!(error.to_string().contains("omit the SDF material override"));
}

#[test]
fn glb_embedded_image_buffer_view_needs_no_external_http_route() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("image.glb");
    let image = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/assets/m2/pixel.png"
    ));
    let mut blob = image.to_vec();
    while !blob.len().is_multiple_of(4) {
        blob.push(0);
    }
    let mut json=serde_json::json!({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"byteLength":blob.len()}],"bufferViews":[{"buffer":0,"byteLength":image.len()}],"images":[{"bufferView":0,"mimeType":"image/png"}]}).to_string().into_bytes();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let mut glb = Vec::new();
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2_u32.to_le_bytes());
    glb.extend_from_slice(&((28 + json.len() + blob.len()) as u32).to_le_bytes());
    glb.extend_from_slice(&(json.len() as u32).to_le_bytes());
    glb.extend_from_slice(b"JSON");
    glb.extend(json);
    glb.extend_from_slice(&(blob.len() as u32).to_le_bytes());
    glb.extend_from_slice(b"BIN\0");
    glb.extend(blob);
    std::fs::write(&path, &glb).unwrap();
    let mut files = Files::new();
    admit(&mesh(&path), 0, &mut files, &mut 0).unwrap();
    assert_eq!(files.len(), 1);
    assert_eq!(files["/assets/0/image.glb"].as_ref(), glb);
}
