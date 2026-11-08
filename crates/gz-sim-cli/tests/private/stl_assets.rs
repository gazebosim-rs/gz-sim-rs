// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{io::Write, net::TcpStream};

const ASCII: &[u8] = include_bytes!("../../../../examples/meshes/m14-asymmetric-ascii.stl");
const BINARY: &[u8] = include_bytes!("../../../../examples/meshes/m14-asymmetric-binary.stl");

fn input(path: &Path) -> sdformat::Mesh {
    sdformat::Mesh {
        uri: path.to_str().expect("UTF8 fixture path").into(),
        scale: [2.0, 3.0, 5.0],
        submesh: String::new(),
        center_submesh: false,
        optimization: sdformat::MeshOptimization::None,
        convex_decomposition: None,
        source: sdformat::Source {
            uri: url::Url::from_file_path(path.with_file_name("world.sdf"))
                .expect("absolute fixture path")
                .into(),
            line: 1,
            column: 1,
        },
    }
}

fn get(prefix: &str, route: &str) -> Vec<u8> {
    let url = url::Url::parse(prefix).expect("asset origin");
    let mut socket =
        TcpStream::connect((Ipv4Addr::LOCALHOST, url.port().expect("nondefault port")))
            .expect("asset listener");
    socket
        .set_read_timeout(Some(Duration::from_secs(3)))
        .expect("read deadline");
    socket
        .set_write_timeout(Some(Duration::from_secs(3)))
        .expect("write deadline");
    write!(socket,"GET {route} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\nOrigin: http://127.0.0.1:8081\r\n\r\n").expect("request");
    let mut response = Vec::new();
    socket.read_to_end(&mut response).expect("response");
    response
}

#[test]
fn stl_visual_snapshot_serves_generated_glb_after_source_and_alias_retirement() {
    let dir = tempfile::tempdir().expect("fixture directory");
    std::fs::write(dir.path().join("shape.stl"), BINARY).expect("write mesh");
    let source = url::Url::from_file_path(dir.path().join("world.sdf")).expect("source URI");
    let sim=gz_sim_scene::load_world(
        "<sdf version='1.12'><world name='default'><model name='mesh'><static>true</static><link name='body'><collision name='box'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='a'><geometry><mesh><uri>shape.stl</uri><scale>2 3 5</scale></mesh></geometry></visual><visual name='b'><geometry><mesh><uri>shape.stl</uri><scale>1 1 1</scale></mesh></geometry></visual></link></model></world></sdf>",
        source.as_str(),"default",Duration::from_millis(5),&sdformat::NoResources).expect("load visual world");
    let mut assets = Assets::new(&sim)
        .expect("capture STL")
        .expect("mesh asset server");
    assert_eq!(
        std::fs::read(dir.path().join("shape.stl")).expect("original source"),
        BINARY
    );
    let snapshot = assets.snapshot();
    assert_eq!(snapshot.files().len(), 1);
    let (id, (mesh, route)) = assets.entries.first_key_value().expect("visual identity");
    let id = *id;
    let mesh = mesh.clone();
    let route = route.clone();
    assert!(route.ends_with("shape.stl.glb"));
    assert_eq!(mesh.uri, "shape.stl");
    let bytes = snapshot.files()[&route].clone();
    assert_eq!(
        bytes.as_ref(),
        gz_assets::mesh::stl_visual_glb(BINARY).expect("same decoder visual")
    );
    validate_recorded_document(
        &assets.uri(id, &mesh).expect("admitted URI"),
        snapshot.prefix(),
        snapshot.files(),
    )
    .expect("existing recording admission");
    let alias = assets
        .prepare_alias(id, u64::from(u32::MAX), &mesh)
        .expect("alias preflight");
    assets.commit_alias(alias);
    assert_eq!(
        assets.uri(id, &mesh).expect("original"),
        assets.uri(u64::from(u32::MAX), &mesh).expect("alias")
    );
    assets.retain_aliases(&Default::default());
    assert!(assets.uri(u64::from(u32::MAX), &mesh).is_err());
    std::fs::write(dir.path().join("shape.stl"), b"changed invalid source").expect("change source");
    std::fs::remove_file(dir.path().join("shape.stl")).expect("remove source");
    assert_eq!(snapshot.files()[&route], bytes);
    assert!(Arc::ptr_eq(&snapshot.files, &assets.snapshot().files));
    drop(assets);
    drop(sim);
    drop(dir);
    let replay = PlaybackAssets::new(snapshot.files().clone())
        .expect("captured GLB playback")
        .expect("resource listener");
    let response = get(&replay.prefix(), &route);
    assert!(response.starts_with(b"HTTP/1.1 200"));
    let separator = response
        .windows(4)
        .position(|v| v == b"\r\n\r\n")
        .expect("HTTP separator");
    assert_eq!(&response[separator + 4..], bytes.as_ref());
    assert!(get(&replay.prefix(), "/assets/0/shape.stl").starts_with(b"HTTP/1.1 404"));
}

#[test]
fn stl_asset_refusals_leave_captured_files_and_accounting_unchanged() {
    let dir = tempfile::tempdir().expect("fixture directory");
    let path = dir.path().join("shape.stl");
    let mut files = Files::new();
    files.insert("/assets/99/old.bin".into(), Bytes::from_static(b"old"));
    let mut total = 3;
    let original = files.clone();
    for bytes in [b"bad STL".as_slice(), &BINARY[..BINARY.len() - 1]] {
        std::fs::write(&path, bytes).expect("write invalid file");
        assert!(admit(&input(&path), 0, &mut files, &mut total).is_err());
        assert_eq!(files, original);
        assert_eq!(total, 3);
    }
    std::fs::write(&path, ASCII).expect("write valid file");
    for scale in [[0.0, 1.0, 1.0], [1.0, -1.0, 1.0], [1.0, f64::INFINITY, 1.0]] {
        let mut mesh = input(&path);
        mesh.scale = scale;
        assert!(document_paths(&mesh).is_err());
    }
    let mut mesh = input(&path);
    mesh.optimization = sdformat::MeshOptimization::ConvexHull;
    assert!(mesh_path(&mesh).is_err());
    mesh.optimization = sdformat::MeshOptimization::None;
    mesh.submesh = "part".into();
    assert!(mesh_path(&mesh).is_err());
    let entry = admit(&input(&path), 0, &mut files, &mut total).expect("generated GLB");
    let expected = gz_assets::mesh::stl_visual_glb(ASCII).expect("same geometry");
    assert_eq!(files[&entry].as_ref(), expected);
    assert_eq!(total, 3 + expected.len());
    let mut full = Files::new();
    for i in 0..FILE_COUNT {
        full.insert(format!("/assets/9/{i}.bin"), Bytes::new());
    }
    let before = full.clone();
    let mut count = 0;
    assert!(admit(&input(&path), 0, &mut full, &mut count).is_err());
    assert_eq!(full, before);
    assert_eq!(count, 0);
    let mut bytes = Files::new();
    let mut count = TOTAL_LIMIT;
    assert!(admit(&input(&path), 0, &mut bytes, &mut count).is_err());
    assert!(bytes.is_empty());
    assert_eq!(count, TOTAL_LIMIT);
}
