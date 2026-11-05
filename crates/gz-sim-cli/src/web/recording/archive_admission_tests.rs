// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::{Value, json};

fn fixture(sequence: u64) -> (Value, Value) {
    (
        json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,"nanoseconds":0,"sequence":sequence,"paused":true,"visuals":[{"id":5,"name":"default::box::visual","translation":[0,0,2],"rotation":[0,0,0,1],"shape":{"kind":"box","size":[1,1,1]},"color":[0,0,1,1]}]}),
        json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":[{"model_id":2,"name":"default::box","visual_ids":[5],"can_remove":true,"can_set_state":true}]}),
    )
}
fn record(ordinal: u32, scene: Value, catalog: Value, outcomes: Vec<Value>) -> Value {
    json!({"kind":"frame","ordinal":ordinal,"scene":scene.to_string(),"catalog":catalog.to_string(),"outcomes":outcomes})
}
fn first() -> Value {
    let (scene, catalog) = fixture(1);
    record(1, scene, catalog, vec![])
}
fn write_archive(records: &[Value], files: &BTreeMap<String, Bytes>) -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    let mut stream = serde_json::to_vec(&json!({"kind":"header","version":1})).unwrap();
    stream.push(b'\n');
    for record in records {
        stream.extend(serde_json::to_vec(record).unwrap());
        stream.push(b'\n');
    }
    stream.extend(serde_json::to_vec(&json!({"kind":"footer","frames":records.len()})).unwrap());
    stream.push(b'\n');
    std::fs::write(dir.path().join("frames.jsonl"), &stream).unwrap();
    let mut assets = Vec::new();
    for (route, bytes) in files {
        let path = dir.path().join(route.strip_prefix('/').unwrap());
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, bytes).unwrap();
        assets.push(Asset {
            route: route.clone(),
            bytes: bytes.len(),
            sha256: digest(bytes),
        });
    }
    let manifest = Manifest {
        version: 1,
        complete: true,
        frames: u32::try_from(records.len()).unwrap(),
        stream_bytes: u64::try_from(stream.len()).unwrap(),
        stream_sha256: digest(&stream),
        asset_prefix: "http://127.0.0.1:9003".into(),
        assets,
    };
    std::fs::write(
        dir.path().join("manifest.json"),
        serde_json::to_vec(&manifest).unwrap(),
    )
    .unwrap();
    dir
}
fn manifest(dir: &Path) -> Value {
    serde_json::from_slice(&std::fs::read(dir.join("manifest.json")).unwrap()).unwrap()
}
fn put_manifest(dir: &Path, value: &Value) {
    std::fs::write(
        dir.join("manifest.json"),
        serde_json::to_vec(value).unwrap(),
    )
    .unwrap();
}
fn opens(records: &[Value]) -> bool {
    Archive::open(write_archive(records, &BTreeMap::new()).path()).is_ok()
}

#[test]
fn disk_manifest_must_be_complete_strict_bounded_and_local() {
    let dir = write_archive(&[first()], &BTreeMap::new());
    assert_eq!(
        Archive::open(dir.path())
            .unwrap()
            .frame(0)
            .unwrap()
            .scene
            .sequence(),
        1
    );
    let good = manifest(dir.path());
    for (field, value) in [
        ("version", json!(2)),
        ("complete", json!(false)),
        ("frames", json!(0)),
        ("frames", json!(4097)),
        ("stream_bytes", json!(lines::FILE_BYTES + 1)),
        ("unknown", json!(0)),
        ("asset_prefix", json!("https://127.0.0.1:9003")),
        ("asset_prefix", json!("http://127.0.0.1:9003/?x")),
        ("asset_prefix", json!("http://user@127.0.0.1:9003")),
    ] {
        let mut bad = good.clone();
        bad[field] = value;
        put_manifest(dir.path(), &bad);
        assert!(Archive::open(dir.path()).is_err(), "{field}");
    }
    let mut missing = good.clone();
    missing.as_object_mut().unwrap().remove("complete");
    put_manifest(dir.path(), &missing);
    assert!(Archive::open(dir.path()).is_err());
    let text = good
        .to_string()
        .replacen("\"version\":1", "\"version\":1,\"version\":1", 1);
    for bytes in [
        text.into_bytes(),
        b"{ invalid".to_vec(),
        vec![b' '; lines::LINE_BYTES + 1],
    ] {
        std::fs::write(dir.path().join("manifest.json"), bytes).unwrap();
        assert!(Archive::open(dir.path()).is_err());
    }
    std::fs::remove_file(dir.path().join("manifest.json")).unwrap();
    assert!(Archive::open(dir.path()).is_err());
}

#[test]
fn stream_hash_size_finalization_and_ordinal_fail_before_any_frame_is_exposed() {
    let dir = write_archive(&[first()], &BTreeMap::new());
    let good = manifest(dir.path());
    for field in ["stream_bytes", "stream_sha256", "frames"] {
        let mut bad = good.clone();
        match field {
            "stream_bytes" => bad[field] = json!(good[field].as_u64().unwrap() + 1),
            "frames" => bad[field] = json!(2),
            _ => bad[field][0] = json!((good[field][0].as_u64().unwrap() + 1) % 256),
        }
        put_manifest(dir.path(), &bad);
        assert!(Archive::open(dir.path()).is_err(), "{field}");
    }
    let original = std::fs::read(dir.path().join("frames.jsonl")).unwrap();
    for bytes in [
        original[..original.len() - 1].to_vec(),
        b"{\"kind\":\"header\",\"version\":1}\n".to_vec(),
    ] {
        std::fs::write(dir.path().join("frames.jsonl"), &bytes).unwrap();
        let mut admitted_hash = good.clone();
        admitted_hash["stream_bytes"] = json!(bytes.len());
        admitted_hash["stream_sha256"] = json!(digest(&bytes));
        put_manifest(dir.path(), &admitted_hash);
        assert!(Archive::open(dir.path()).is_err());
    }
    let mut wrong = first();
    wrong["ordinal"] = json!(2);
    assert!(!opens(&[wrong]));
    std::fs::remove_file(dir.path().join("frames.jsonl")).unwrap();
    put_manifest(dir.path(), &good);
    assert!(Archive::open(dir.path()).is_err());
}

#[test]
fn captured_file_hashes_missing_paths_and_symlinks_are_not_admitted_as_resources() {
    let files = BTreeMap::from([("/assets/0/data.bin".into(), Bytes::from_static(b"original"))]);
    let dir = write_archive(&[first()], &files);
    let good = manifest(dir.path());
    for (field, value) in [
        ("bytes", json!(ASSET_FILE_BYTES + 1)),
        ("route", json!("/assets/0/../escape")),
        ("route", json!("/assets/128/a.bin")),
    ] {
        let mut bad = good.clone();
        bad["assets"][0][field] = value;
        put_manifest(dir.path(), &bad);
        assert!(Archive::open(dir.path()).is_err(), "{field}");
    }
    let mut duplicate = good.clone();
    duplicate["assets"]
        .as_array_mut()
        .unwrap()
        .push(good["assets"][0].clone());
    put_manifest(dir.path(), &duplicate);
    assert!(Archive::open(dir.path()).is_err());
    let mut count = good.clone();
    count["assets"] = json!(vec![good["assets"][0].clone(); 129]);
    put_manifest(dir.path(), &count);
    assert!(Archive::open(dir.path()).is_err());
    put_manifest(dir.path(), &good);
    let asset = dir.path().join("assets/0/data.bin");
    std::fs::write(&asset, b"modified").unwrap();
    assert!(Archive::open(dir.path()).is_err());
    std::fs::remove_file(&asset).unwrap();
    assert!(Archive::open(dir.path()).is_err());
    #[cfg(unix)]
    {
        let outside = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(outside.path(), b"original").unwrap();
        std::os::unix::fs::symlink(outside.path(), &asset).unwrap();
        assert!(Archive::open(dir.path()).is_err());
    }
}

#[test]
fn archive_mesh_admission_validates_document_closure_then_retains_owned_bytes() {
    let (mut scene, catalog) = fixture(1);
    scene["version"] = json!(2);
    scene["visuals"][0]["shape"] =
        json!({"kind":"mesh","uri":"http://127.0.0.1:9003/assets/0/shape.gltf","scale":[1,2,3]});
    let document = json!({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[]}],"buffers":[{"uri":"data.bin","byteLength":4}],"images":[{"uri":"pixel.png"}]});
    let route = "/assets/0/shape.gltf";
    let files = BTreeMap::from([
        (route.into(), Bytes::from(document.to_string())),
        (
            "/assets/0/data.bin".into(),
            Bytes::from_static(&[1, 2, 3, 4]),
        ),
        (
            "/assets/0/pixel.png".into(),
            Bytes::from_static(b"captured image bytes; no pixel decoding in archive admission"),
        ),
    ]);
    let frame = record(1, scene.clone(), catalog.clone(), vec![]);
    let dir = write_archive(std::slice::from_ref(&frame), &files);
    let archive = Archive::open(dir.path()).unwrap();
    std::fs::remove_dir_all(dir.path().join("assets")).unwrap();
    assert_eq!(
        archive.assets[route].as_ref(),
        document.to_string().as_bytes()
    );
    assert_eq!(
        archive.frame(0).unwrap().scene.mesh_uris().next(),
        Some("http://127.0.0.1:9003/assets/0/shape.gltf")
    );
    assert!(Archive::open(dir.path()).is_err());
    for missing in ["/assets/0/data.bin", "/assets/0/pixel.png", route] {
        let mut incomplete = files.clone();
        incomplete.remove(missing);
        assert!(
            Archive::open(write_archive(std::slice::from_ref(&frame), &incomplete).path()).is_err(),
            "{missing}"
        );
    }
    for invalid_document in [
        "{ invalid".to_string(),
        document.to_string().replace("2.0", "1.0"),
        document.to_string().replace("data.bin", "../data.bin"),
        document
            .to_string()
            .replace("pixel.png", "http://remote/pixel.png"),
    ] {
        let mut invalid = files.clone();
        invalid.insert(route.into(), Bytes::from(invalid_document));
        // Hashes are correct for these malicious bytes: manifest integrity alone is insufficient.
        assert!(
            Archive::open(write_archive(std::slice::from_ref(&frame), &invalid).path()).is_err()
        );
    }
}

#[test]
fn scene_catalog_role_ownership_and_stable_definitions_are_archive_invariants() {
    let (first_scene, first_catalog) = fixture(1);
    let (mut moving, second_catalog) = fixture(2);
    moving["visuals"][0]["translation"] = json!([3, 4, 5]);
    let initial = record(1, first_scene.clone(), first_catalog.clone(), vec![]);
    assert!(opens(&[
        initial.clone(),
        record(2, moving.clone(), second_catalog.clone(), vec![])
    ]));
    for (field, value) in [
        ("world_id", json!(9)),
        ("world", json!("foreign")),
        ("version", json!(2)),
    ] {
        let mut bad = moving.clone();
        bad[field] = value;
        let mut catalog = second_catalog.clone();
        if field == "world_id" {
            catalog[field] = json!(9);
        }
        assert!(
            !opens(&[initial.clone(), record(2, bad, catalog, vec![])]),
            "{field}"
        );
    }
    for field in ["shape", "name", "color"] {
        let mut bad = moving.clone();
        bad["visuals"][0][field] = match field {
            "shape" => json!({"kind":"sphere","radius":1}),
            "name" => json!("renamed"),
            _ => json!([1, 0, 0, 1]),
        };
        assert!(
            !opens(&[
                initial.clone(),
                record(2, bad, second_catalog.clone(), vec![])
            ]),
            "{field}"
        );
    }
    let mut role = moving.clone();
    role["visuals"][0]["id"] = json!(2);
    let mut role_catalog = second_catalog.clone();
    role_catalog["models"][0]["model_id"] = json!(5);
    role_catalog["models"][0]["visual_ids"] = json!([2]);
    assert!(!opens(&[
        initial.clone(),
        record(2, role, role_catalog, vec![])
    ]));
    for bad_catalog in [
        {
            let mut c = first_catalog.clone();
            c["models"][0]["visual_ids"] = json!([6]);
            c
        },
        {
            let mut c = first_catalog.clone();
            let duplicate = c["models"][0].clone();
            c["models"].as_array_mut().unwrap().push(duplicate);
            c
        },
        {
            let mut c = first_catalog.clone();
            c["models"][0]["model_id"] = json!(1);
            c
        },
    ] {
        assert!(!opens(&[record(
            1,
            first_scene.clone(),
            bad_catalog,
            vec![]
        )]));
    }
    let mut renamed = second_catalog;
    renamed["models"][0]["name"] = json!("other owner");
    assert!(!opens(&[initial, record(2, moving, renamed, vec![])]));
}

fn life_outcome(ok: bool, model_id: u64) -> Value {
    let state = json!({"translation":[1,2,3],"rotation":[0,0,0,1],"linear_velocity":[4,5,6],"angular_velocity":[0,0,2]});
    json!({"action":"get_state","model_id":model_id,"ok":ok,
        "request":json!({"version":1,"kind":"lifecycle","request_id":3,"world_id":1,"epoch":0,"action":"get_state","model_id":model_id}).to_string(),
        "result":json!({"version":1,"kind":"lifecycle_ack","request_id":3,"action":"get_state","world_id":1,"epoch":0,"scene_sequence":1,"seconds":0,"nanoseconds":0,"paused":true,"ok":ok,"error":if ok {Value::Null}else{json!("unknown model")},"model_id":if ok {json!(model_id)}else{Value::Null},"state":if ok {state}else{Value::Null}}).to_string()})
}
#[test]
fn typed_outcomes_require_nullable_provenance_and_preserve_strict_state_norm() {
    let (scene, catalog) = fixture(1);
    let with = |outcome| record(1, scene.clone(), catalog.clone(), vec![outcome]);
    let valid = life_outcome(true, 2);
    assert!(opens(&[with(valid.clone())]));
    assert!(opens(&[with(life_outcome(false, u64::MAX))]));
    for field in ["model_id", "error", "state"] {
        let mut result: Value = serde_json::from_str(valid["result"].as_str().unwrap()).unwrap();
        result.as_object_mut().unwrap().remove(field);
        let mut bad = valid.clone();
        bad["result"] = json!(result.to_string());
        assert!(!opens(&[with(bad)]), "{field}");
    }
    let mut missing = valid.clone();
    missing.as_object_mut().unwrap().remove("model_id");
    assert!(!opens(&[with(missing)]));
    for (norm_error, admitted) in [(2.0e-13, true), (2.0e-12, false)] {
        let mut result: Value = serde_json::from_str(valid["result"].as_str().unwrap()).unwrap();
        result["state"]["rotation"] = json!([0.0, 0.0, 0.0, (1.0_f64 + norm_error).sqrt()]);
        let mut outcome = valid.clone();
        outcome["result"] = json!(result.to_string());
        assert_eq!(opens(&[with(outcome)]), admitted, "norm error {norm_error}");
    }
    let mut refused = life_outcome(false, u64::MAX);
    refused["model_id"] = json!(2);
    assert!(!opens(&[with(refused)]));
    let mut wrong_result: Value = serde_json::from_str(valid["result"].as_str().unwrap()).unwrap();
    wrong_result["request_id"] = json!(4);
    let mut wrong = valid.clone();
    wrong["result"] = json!(wrong_result.to_string());
    assert!(!opens(&[with(wrong)]));
}
