// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::io::Write;
use std::net::TcpStream;

fn request(prefix: &str, path: &str) -> Vec<u8> {
    let url = url::Url::parse(prefix).unwrap();
    let mut stream = TcpStream::connect((Ipv4Addr::LOCALHOST, url.port().unwrap())).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    stream
        .set_write_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: localhost\r\nOrigin: http://127.0.0.1:8081\r\n\r\n"
    )
    .unwrap();
    let mut response = Vec::new();
    stream.read_to_end(&mut response).unwrap();
    response
}
fn body(response: &[u8]) -> &[u8] {
    let at = response
        .windows(4)
        .position(|part| part == b"\r\n\r\n")
        .unwrap();
    &response[at + 4..]
}

#[test]
fn captured_bytes_survive_source_changes_alias_retirement_and_live_server_drop() {
    let dir = tempfile::tempdir().unwrap();
    let document = br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data.bin","byteLength":4}]}"#;
    std::fs::write(dir.path().join("shape.gltf"), document).unwrap();
    std::fs::write(dir.path().join("data.bin"), [1, 2, 3, 4]).unwrap();
    let source = url::Url::from_file_path(dir.path().join("world.sdf")).unwrap();
    let sim = gz_sim_scene::load_world(
        r#"<sdf version="1.12"><world name="default"><model name="mesh"><static>true</static><link name="body"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision><visual name="visual"><geometry><mesh><uri>shape.gltf</uri></mesh></geometry></visual></link></model></world></sdf>"#,
        source.as_str(), "default", Duration::from_millis(5), &sdformat::NoResources,
    ).unwrap();
    let mut assets = Assets::new(&sim).unwrap().unwrap();
    let snapshot = assets.snapshot();
    assert!(assets.healthy());
    assert_eq!(
        snapshot.prefix(),
        format!("http://127.0.0.1:{}", assets.port)
    );
    assert_eq!(snapshot.files().len(), 2);
    assert!(Arc::ptr_eq(&snapshot.files, &assets.files));
    let (id, (mesh, route)) = assets.entries.first_key_value().unwrap();
    let id = *id;
    let mesh = mesh.clone();
    let route = route.clone();
    let uri = assets.uri(id, &mesh).unwrap();
    assert_eq!(uri, format!("{}{route}", snapshot.prefix()));
    let alias = assets
        .prepare_alias(id, u64::from(u32::MAX), &mesh)
        .unwrap();
    assets.commit_alias(alias);
    assert_eq!(assets.uri(u64::from(u32::MAX), &mesh).unwrap(), uri);
    assert!(Arc::ptr_eq(&snapshot.files, &assets.snapshot().files));
    assert_eq!(snapshot.files().len(), 2); // Alias adds no captured files or byte copies.
    assets.retain_aliases(&Default::default());
    assert!(assets.uri(u64::from(u32::MAX), &mesh).is_err());
    assert_eq!(assets.uri(id, &mesh).unwrap(), uri);
    std::fs::write(dir.path().join("shape.gltf"), b"not glTF anymore").unwrap();
    std::fs::remove_file(dir.path().join("data.bin")).unwrap();
    assert_eq!(snapshot.files()[&route].as_ref(), document);
    assert_eq!(
        snapshot.files()["/assets/0/data.bin"].as_ref(),
        &[1, 2, 3, 4]
    );
    let original_stop = assets.stop.clone();
    drop(assets);
    assert!(original_stop.load(Ordering::Acquire));
    drop(sim);
    drop(dir);
    let playback = PlaybackAssets::new(snapshot.files().clone())
        .unwrap()
        .unwrap();
    assert!(playback.healthy());
    let origin = url::Url::parse(&playback.prefix()).unwrap();
    assert_eq!(origin.host_str(), Some("127.0.0.1"));
    assert!(origin.port().unwrap() > 0);
    for (path, bytes) in snapshot.files() {
        let response = request(&playback.prefix(), path);
        assert!(response.starts_with(b"HTTP/1.1 200"));
        assert_eq!(body(&response), bytes.as_ref());
    }
    assert!(request(&playback.prefix(), "/assets/0/undeclared.bin").starts_with(b"HTTP/1.1 404"));
    assert!(request(&playback.prefix(), "/assets/0/data.bin?x").starts_with(b"HTTP/1.1 400"));
}

#[test]
fn playback_validates_routes_and_all_three_resource_bounds_before_startup() {
    assert!(PlaybackAssets::new(Files::new()).unwrap().is_none());
    for path in [
        "",
        "assets/0/data.bin",
        "/assets//data.bin",
        "/assets/a/data.bin",
        "/assets/0/",
        "/assets/0/../data.bin",
        "/assets/0/./data.bin",
        "/assets/0/a//data.bin",
        "/assets/0/%64ata.bin",
        "/assets/0/data.bin?x",
        "/assets/0/data.bin#x",
        "/assets/0/a\\b",
        "/assets/0/é.bin",
        "http://127.0.0.1:1/assets/0/a.bin",
    ] {
        assert!(
            PlaybackAssets::new(Files::from([(path.into(), Bytes::new())])).is_err(),
            "{path}"
        );
    }
    let boundary: Files = (0..FILE_COUNT)
        .map(|i| (format!("/assets/0/{i}.bin"), Bytes::new()))
        .collect();
    let mut too_many = boundary.clone();
    too_many.insert("/assets/0/extra.bin".into(), Bytes::new());
    assert!(PlaybackAssets::new(too_many).is_err());
    let server = PlaybackAssets::new(boundary).unwrap().unwrap();
    assert!(server.healthy());
    drop(server);
    let large = Bytes::from(vec![7; FILE_LIMIT]);
    let exact: Files = (0..4)
        .map(|i| (format!("/assets/1/{i}.bin"), large.clone()))
        .collect();
    let mut too_large = exact.clone();
    too_large.insert("/assets/1/extra.bin".into(), Bytes::from_static(b"x"));
    assert!(PlaybackAssets::new(too_large).is_err());
    assert!(
        PlaybackAssets::new(Files::from([(
            "/assets/0/large.bin".into(),
            Bytes::from(vec![0; FILE_LIMIT + 1])
        )]))
        .is_err()
    );
    let server = PlaybackAssets::new(exact).unwrap().unwrap();
    assert!(server.healthy());
    drop(server);
}

#[test]
fn playback_reuses_http_policy_and_drop_joins_even_an_incomplete_request() {
    let server = PlaybackAssets::new(Files::from([(
        "/assets/8/sub/data.bin".into(),
        Bytes::from_static(b"recorded"),
    )]))
    .unwrap()
    .unwrap();
    let prefix = server.prefix();
    let url = url::Url::parse(&prefix).unwrap();
    let address = (Ipv4Addr::LOCALHOST, url.port().unwrap());
    let mut wrong = TcpStream::connect(address).unwrap();
    wrong
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    wrong.write_all(b"GET /assets/8/sub/data.bin HTTP/1.1\r\nHost: localhost\r\nOrigin: https://remote\r\n\r\n").unwrap();
    let mut response = Vec::new();
    wrong.read_to_end(&mut response).unwrap();
    assert!(response.starts_with(b"HTTP/1.1 400"));
    let mut partial = TcpStream::connect(address).unwrap();
    partial
        .set_write_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    partial
        .write_all(b"GET /assets/8/sub/data.bin HTTP/1.1\r\nHost:")
        .unwrap();
    let (done, finished) = std::sync::mpsc::sync_channel(1);
    let stop = server.stop.clone();
    let joiner = thread::spawn(move || {
        drop(server);
        done.send(()).unwrap();
    });
    finished
        .recv_timeout(Duration::from_secs(3))
        .expect("bounded worker stop/join");
    joiner.join().unwrap();
    assert!(stop.load(Ordering::Acquire));
    drop(partial);
}

#[test]
fn recorded_document_requires_exact_origin_and_same_scope_resource_closure() {
    let prefix = "http://127.0.0.1:9003";
    let entry = "/assets/7/nested/shape.gltf";
    let document = br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data.bin","byteLength":4}],"images":[{"uri":"tex/pixel.png"}]}"#;
    let files = Files::from([
        (entry.into(), Bytes::from_static(document)),
        (
            "/assets/7/nested/data.bin".into(),
            Bytes::from_static(&[1, 2, 3, 4]),
        ),
        (
            "/assets/7/nested/tex/pixel.png".into(),
            Bytes::from_static(b"opaque captured image bytes"),
        ),
    ]);
    validate_recorded_document(&format!("{prefix}{entry}"), prefix, &files).unwrap();
    for absent in [
        entry,
        "/assets/7/nested/data.bin",
        "/assets/7/nested/tex/pixel.png",
    ] {
        let mut missing = files.clone();
        missing.remove(absent);
        assert!(
            validate_recorded_document(&format!("{prefix}{entry}"), prefix, &missing).is_err(),
            "{absent}"
        );
    }
    let mut wrong_scope = files.clone();
    let bytes = wrong_scope.remove("/assets/7/nested/data.bin").unwrap();
    wrong_scope.insert("/assets/8/nested/data.bin".into(), bytes);
    assert!(validate_recorded_document(&format!("{prefix}{entry}"), prefix, &wrong_scope).is_err());
    for bad_prefix in [
        "http://localhost:9003",
        "https://127.0.0.1:9003",
        "http://127.0.0.1:0",
        "http://127.0.0.1:09003",
        "http://127.0.0.1:9003/",
        "http://remote:9003",
    ] {
        assert!(
            validate_recorded_document(&format!("{bad_prefix}{entry}"), bad_prefix, &files)
                .is_err(),
            "{bad_prefix}"
        );
    }
    for uri in [
        "http://127.0.0.1:90030/assets/7/nested/shape.gltf",
        "http://127.0.0.1:9004/assets/7/nested/shape.gltf",
        "http://127.0.0.1:9003/assets/7/nested/shape.gltf?x",
        "http://127.0.0.1:9003/assets/7/nested/shape.gltf#x",
        "http://127.0.0.1:9003/assets/7/../shape.gltf",
        "http://127.0.0.1:9003/assets/7/nested/shape.bin",
    ] {
        assert!(
            validate_recorded_document(uri, prefix, &files).is_err(),
            "{uri}"
        );
    }
    let long = format!("{prefix}/assets/7/{}.gltf", "a".repeat(2048));
    assert!(validate_recorded_document(&long, prefix, &files).is_err());
    for resource in [
        "http://remote/data.bin",
        "http://127.0.0.1:9003/assets/8/data.bin",
        "../data.bin",
        "a/../../data.bin",
        "/data.bin",
        "a%2fdata.bin",
        "a?x",
        "a#x",
        "a\\data.bin",
    ] {
        for kind in ["buffers", "images"] {
            let mut doc: serde_json::Value = serde_json::from_slice(document).unwrap();
            doc[kind][0]["uri"] = serde_json::json!(resource);
            let mut bad = files.clone();
            bad.insert(entry.into(), Bytes::from(serde_json::to_vec(&doc).unwrap()));
            assert!(
                validate_recorded_document(&format!("{prefix}{entry}"), prefix, &bad).is_err(),
                "{kind}: {resource}"
            );
        }
    }
}

#[test]
fn recorded_document_reuses_maintained_gltf_version_scene_and_embedded_glb_admission() {
    let prefix = "http://127.0.0.1:9003";
    let route = "/assets/0/shape.gltf";
    let valid = serde_json::json!({"asset":{"version":"2.0","minVersion":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data:application/octet-stream;base64,AQIDBA==","byteLength":4}]});
    let accept = |bytes: Bytes| {
        validate_recorded_document(
            &format!("{prefix}{route}"),
            prefix,
            &Files::from([(route.into(), bytes)]),
        )
    };
    accept(Bytes::from(serde_json::to_vec(&valid).unwrap())).unwrap();
    assert!(accept(Bytes::from_static(b"{ malformed JSON")).is_err());
    for (field, value) in [
        ("version", "1.0"),
        ("version", "2.1"),
        ("minVersion", "2.1"),
    ] {
        let mut bad = valid.clone();
        bad["asset"][field] = serde_json::json!(value);
        assert!(
            accept(Bytes::from(serde_json::to_vec(&bad).unwrap())).is_err(),
            "{field}: {value}"
        );
    }
    let mut no_scene = valid.clone();
    no_scene.as_object_mut().unwrap().remove("scene");
    no_scene["scenes"] = serde_json::json!([]);
    assert!(accept(Bytes::from(serde_json::to_vec(&no_scene).unwrap())).is_err());
    let mut json = br#"{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"byteLength":4}]}"#.to_vec();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let mut glb = Vec::new();
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(
        &u32::try_from(12 + 8 + json.len() + 8 + 4)
            .unwrap()
            .to_le_bytes(),
    );
    glb.extend_from_slice(&u32::try_from(json.len()).unwrap().to_le_bytes());
    glb.extend_from_slice(b"JSON");
    glb.extend_from_slice(&json);
    glb.extend_from_slice(&4u32.to_le_bytes());
    glb.extend_from_slice(b"BIN\0");
    glb.extend_from_slice(&[1, 2, 3, 4]);
    let files = Files::from([("/assets/0/embedded.glb".into(), Bytes::from(glb))]);
    validate_recorded_document(&format!("{prefix}/assets/0/embedded.glb"), prefix, &files).unwrap();
}
