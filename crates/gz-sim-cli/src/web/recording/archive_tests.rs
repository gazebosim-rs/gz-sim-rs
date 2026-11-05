// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn scene(epoch: u64, sequence: u64, seconds: u64, visible: bool) -> String {
    let visuals = if visible {
        vec![
            json!({"id":5,"name":"default::box::visual","translation":[0.0,0.0,2.0],"rotation":[0.0,0.0,0.0,1.0],"shape":{"kind":"box","size":[1.0,1.0,1.0]},"color":[0.0,0.0,1.0,1.0]}),
        ]
    } else {
        vec![]
    };
    json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":epoch,"seconds":seconds,"nanoseconds":0,"sequence":sequence,"paused":true,"visuals":visuals}).to_string()
}
fn catalog(epoch: u64, sequence: u64, visible: bool) -> String {
    let models = if visible {
        vec![
            json!({"model_id":2,"name":"default::box","visual_ids":[5],"can_remove":true,"can_set_state":true}),
        ]
    } else {
        vec![]
    };
    json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":epoch,"scene_sequence":sequence,"templates":[],"models":models}).to_string()
}
fn frame(ordinal: u32, epoch: u64, seconds: u64, visible: bool) -> Record {
    Record::Frame {
        ordinal,
        scene: scene(epoch, ordinal as u64, seconds, visible),
        catalog: catalog(epoch, ordinal as u64, visible),
        outcomes: vec![],
    }
}
fn stream(records: &[Record], frames: u32) -> Arc<[u8]> {
    let mut bytes = lines::encode(&Record::Header { version: 1 }, lines::LINE_BYTES).unwrap();
    for record in records {
        bytes.extend(lines::encode(record, lines::LINE_BYTES).unwrap());
    }
    bytes.extend(lines::encode(&Record::Footer { frames }, lines::LINE_BYTES).unwrap());
    bytes.into()
}
fn admit(records: &[Record]) -> io::Result<Archive> {
    Archive::admit(
        stream(records, records.len() as u32),
        records.len() as u32,
        "http://127.0.0.1:1".into(),
        BTreeMap::new(),
    )
}
fn disk(root: &Path, bytes: &[u8], assets: Vec<Asset>) {
    std::fs::write(root.join("frames.jsonl"), bytes).unwrap();
    let manifest = Manifest {
        version: 1,
        complete: true,
        frames: 1,
        stream_bytes: bytes.len() as u64,
        stream_sha256: digest(bytes),
        asset_prefix: "http://127.0.0.1:1".into(),
        assets,
    };
    std::fs::write(
        root.join("manifest.json"),
        serde_json::to_vec(&manifest).unwrap(),
    )
    .unwrap();
}

#[test]
fn immutable_stream_ownership_order_and_original_clocks() {
    let archive = admit(&[
        frame(1, 0, 0, true),
        frame(2, 0, 2, false),
        frame(3, 1, 0, true),
    ])
    .unwrap();
    assert_eq!(
        archive.stamps,
        vec![
            Stamp {
                epoch: 0,
                sequence: 1,
                seconds: 0,
                nanoseconds: 0
            },
            Stamp {
                epoch: 0,
                sequence: 2,
                seconds: 2,
                nanoseconds: 0
            },
            Stamp {
                epoch: 1,
                sequence: 3,
                seconds: 0,
                nanoseconds: 0
            }
        ]
    );
    assert_eq!(
        archive
            .frame(0)
            .unwrap()
            .scene
            .visual_ids()
            .collect::<Vec<_>>(),
        vec![5]
    );
    assert_eq!(archive.frame(1).unwrap().catalog.model_ids().count(), 0);
    assert_eq!(
        archive
            .frame(2)
            .unwrap()
            .catalog
            .model_ids()
            .collect::<Vec<_>>(),
        vec![2]
    );
    assert!(archive.frame(3).is_err());
    let dir = tempfile::tempdir().unwrap();
    let bytes = stream(&[frame(1, 0, 0, true)], 1);
    disk(dir.path(), &bytes, vec![]);
    let opened = Archive::open(dir.path()).unwrap();
    std::fs::write(
        dir.path().join("frames.jsonl"),
        b"changed after complete admission",
    )
    .unwrap();
    assert_eq!(opened.frame(0).unwrap().scene.sequence(), 1);
    assert!(Archive::open(dir.path()).is_err());
}

#[test]
fn complete_archive_refuses_truncation_unknown_fields_and_recycled_ids() {
    assert!(admit(&[frame(2, 0, 0, true)]).is_err());
    assert!(
        admit(&[
            frame(1, 0, 0, true),
            frame(2, 0, 1, false),
            frame(3, 0, 2, true)
        ])
        .is_err()
    );
    assert!(admit(&[frame(1, 0, 0, true), frame(2, 1, 0, false)]).is_err());
    assert!(admit(&[frame(1, 0, 1, true)]).is_err());
    let good = stream(&[frame(1, 0, 0, true)], 1);
    for bytes in [
        good[..good.len() - 1].to_vec(),
        b"{\"kind\":\"header\",\"version\":1,\"extra\":0}\n".to_vec(),
        b"{\"kind\":\"header\",\"version\":1,\"version\":1}\n".to_vec(),
    ] {
        assert!(
            Archive::admit(
                bytes.into(),
                1,
                "http://127.0.0.1:1".into(),
                BTreeMap::new()
            )
            .is_err()
        );
    }
    let mut trailing = good.to_vec();
    trailing.extend(b"{}\n");
    assert!(
        Archive::admit(
            trailing.into(),
            1,
            "http://127.0.0.1:1".into(),
            BTreeMap::new()
        )
        .is_err()
    );
    let mut bad = frame(1, 0, 0, true);
    if let Record::Frame { ref mut scene, .. } = bad {
        *scene = scene.replace("\"world_id\":1", "\"world_id\":1,\"world_id\":1");
    }
    assert!(admit(&[bad]).is_err());
    let mut bad = frame(1, 0, 0, true);
    if let Record::Frame {
        ref mut catalog, ..
    } = bad
    {
        *catalog = catalog.replace("[5]", "[6]");
    }
    assert!(admit(&[bad]).is_err());
}

#[test]
fn captured_assets_are_digest_bound_and_never_reopened_after_admission() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("assets/0")).unwrap();
    let captured = b"immutable captured resource";
    std::fs::write(dir.path().join("assets/0/box.bin"), captured).unwrap();
    let asset = || Asset {
        route: "/assets/0/box.bin".into(),
        bytes: captured.len(),
        sha256: digest(captured),
    };
    disk(
        dir.path(),
        &stream(&[frame(1, 0, 0, true)], 1),
        vec![asset()],
    );
    let archive = Archive::open(dir.path()).unwrap();
    std::fs::write(dir.path().join("assets/0/box.bin"), vec![0; captured.len()]).unwrap();
    assert_eq!(archive.assets["/assets/0/box.bin"].as_ref(), captured);
    assert!(Archive::open(dir.path()).is_err());
    for route in [
        "/assets/0/../escape",
        "/assets/128/box.bin",
        "/assets/0//box.bin",
        "/assets/0/%2e%2e",
        "https://example.com/a",
        "/assets/0/box.bin?x",
    ] {
        assert!(!asset_route(route), "{route}");
    }
    for route in ["/assets/0/box.bin", "/assets/127/nested/box-2.bin"] {
        assert!(asset_route(route));
    }
    disk(
        dir.path(),
        &stream(&[frame(1, 0, 0, true)], 1),
        vec![asset(), asset()],
    );
    assert!(Archive::open(dir.path()).is_err());
}

#[test]
fn recorded_geometry_and_catalog_preserve_admitted_limits() {
    let good = scene(0, 1, 0, true);
    for text in [
        good.replace("\"kind\":\"scene\"", "\"kind\":\"other\""),
        good.replace("[1.0,1.0,1.0]", "[0.0,1.0,1.0]"),
        good.replace("[0.0,0.0,0.0,1.0]", "[0.0,0.0,0.0,2.0]"),
        good.replace("\"nanoseconds\":0", "\"nanoseconds\":1000000000"),
        good.replace("\"id\":5", "\"id\":4294967296"),
    ] {
        assert!(Scene::decode(&text, |_| false).is_err(), "{text}");
    }
    let mut parsed = Scene::decode(&good, |_| false).unwrap();
    let valid_catalog = catalog(0, 1, true);
    for text in [
        valid_catalog.replace("\"kind\":\"lifecycle_catalog\"", "\"kind\":\"other\""),
        valid_catalog.replace("\"kind\":\"lifecycle_catalog\",", ""),
        valid_catalog.replace(
            "\"kind\":\"lifecycle_catalog\"",
            "\"kind\":\"lifecycle_catalog\",\"kind\":\"lifecycle_catalog\"",
        ),
    ] {
        assert!(Catalog::decode(&text, &parsed).is_err());
    }

    let mut read_only = Catalog::decode(&catalog(0, 1, true), &parsed).unwrap();
    parsed.playback(5, 9, false, |_| unreachable!()).unwrap();
    read_only.read_only(&parsed);
    let encoded = read_only.replay_encode().unwrap();
    assert!(encoded.contains("\"can_remove\":false"));
    assert!(encoded.contains("\"can_set_state\":false"));
    assert!(encoded.contains("\"scene_sequence\":9"));
    assert!(encoded.contains("\"epoch\":5"));
}

#[test]
fn outcomes_retain_typed_original_provenance_and_refuse_fabricated_summary() {
    let outcome = Outcome { action:Action::Pause, model_id:None, ok:true,
        request:r#"{"version":1,"kind":"control","request_id":1,"action":"pause"}"#.into(),
        result:r#"{"version":1,"kind":"ack","request_id":1,"ok":true,"epoch":0,"seconds":0,"nanoseconds":0,"paused":true,"error":null}"#.into() };
    let with = |outcomes| Record::Frame {
        ordinal: 1,
        scene: scene(0, 1, 0, true),
        catalog: catalog(0, 1, true),
        outcomes,
    };
    assert!(admit(&[with(vec![outcome.clone()])]).is_ok());
    let mut bad = outcome.clone();
    bad.ok = false;
    assert!(admit(&[with(vec![bad])]).is_err());
    let mut bad = outcome.clone();
    bad.action = Action::Resume;
    assert!(admit(&[with(vec![bad])]).is_err());
    let mut bad = outcome.clone();
    bad.result = bad.result.replace("\"request_id\":1", "\"request_id\":2");
    assert!(admit(&[with(vec![bad])]).is_err());
    let mut bad = outcome.clone();
    bad.request = bad
        .request
        .replace("\"request_id\":1", "\"request_id\":1,\"request_id\":1");
    assert!(admit(&[with(vec![bad])]).is_err());
    assert!(admit(&[with(vec![outcome; 9])]).is_err());
}

#[test]
fn contradictory_actions_missing_nullable_fields_and_foreign_success_are_refused() {
    let captured = Scene::decode(&scene(0, 1, 0, true), |_| false).unwrap();
    let request = r#"{"version":1,"kind":"control","request_id":1,"action":"resume"}"#;
    let reply = r#"{"version":1,"kind":"ack","request_id":1,"ok":true,"epoch":0,"seconds":0,"nanoseconds":0,"paused":true,"error":null}"#;
    let outcome = Outcome {
        action: Action::Resume,
        model_id: None,
        ok: true,
        request: request.into(),
        result: reply.into(),
    };
    assert!(outcome.validate(&captured).is_err());
    for text in [
        reply.replace(",\"error\":null", ""),
        reply.replace("\"error\":null", "\"error\":null,\"error\":null"),
    ] {
        assert!(serde_json::from_str::<WorldReply>(&text).is_err());
    }
    let serialized = serde_json::to_string(&outcome).unwrap();
    assert!(
        serde_json::from_str::<Outcome>(&serialized.replace("\"model_id\":null,", "")).is_err()
    );
    let request = r#"{"version":1,"kind":"lifecycle","request_id":2,"world_id":1,"epoch":0,"action":"remove","model_id":2}"#;
    let reply = r#"{"version":1,"kind":"lifecycle_ack","request_id":2,"action":"remove","world_id":1,"epoch":0,"scene_sequence":1,"seconds":0,"nanoseconds":0,"paused":true,"ok":true,"error":null,"model_id":2,"state":null}"#;
    let valid = Outcome {
        action: Action::Remove,
        model_id: Some(2),
        ok: true,
        request: request.into(),
        result: reply.into(),
    };
    assert!(valid.validate(&captured).is_ok());
    for request in [
        request.replace("\"world_id\":1", "\"world_id\":9"),
        request.replace("\"epoch\":0", "\"epoch\":1"),
    ] {
        let mut bad = valid.clone();
        bad.request = request;
        assert!(bad.validate(&captured).is_err());
    }
    for field in [",\"error\":null", ",\"state\":null", ",\"model_id\":2"] {
        assert!(serde_json::from_str::<LifeReply>(&reply.replace(field, "")).is_err());
    }
    let foreign = scene(0, 1, 0, true).replace("\"world\":\"default\"", "\"world\":\"foreign\"");
    assert!(Scene::decode(&foreign, |_| false).is_err());
}
