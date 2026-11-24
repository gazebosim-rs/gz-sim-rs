// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn empty() -> (String, String) {
    (r#"{"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,"nanoseconds":0,"sequence":0,"paused":true,"visuals":[]}"#.into(),
     r#"{"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":0,"templates":[],"models":[]}"#.into())
}

#[test]
fn real_writer_finalizes_only_complete_admitted_capture_and_refuses_existing_output() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    let mut recorder = Recorder::start(&path, None).unwrap();
    assert!(!path.join("manifest.json").exists());
    let (scene, catalog) = empty();
    recorder.record(scene, catalog, vec![]).unwrap();
    recorder.finish().unwrap();
    let archive = Archive::open(&path).unwrap();
    assert_eq!(archive.frame(0).unwrap().scene.world_name(), "default");
    assert!(archive.frame(1).is_err());
    assert!(Recorder::start(&path, None).is_err());
    assert!(!path.join("manifest.pending").exists());
    assert_eq!(archive.stamps.len(), 1);
}

#[test]
fn interrupted_or_inconsistent_capture_never_publishes_complete_manifest() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("interrupted");
    let mut recorder = Recorder::start(&path, None).unwrap();
    let (scene, catalog) = empty();
    recorder.record(scene, catalog, vec![]).unwrap();
    drop(recorder);
    assert!(!path.join("manifest.json").exists());
    assert!(Archive::open(&path).is_err());
    let path = temp.path().join("invalid");
    let mut recorder = Recorder::start(&path, None).unwrap();
    recorder.record("{}".into(), "{}".into(), vec![]).unwrap();
    assert!(recorder.finish().is_err());
    assert!(!path.join("manifest.json").exists());
    let path = temp.path().join("zero-frames");
    assert!(Recorder::start(&path, None).unwrap().finish().is_err());
    assert!(!path.join("manifest.json").exists());
}

#[test]
fn bounded_queue_busy_is_terminal_and_does_not_retry_or_increment_refused_capture() {
    // Hold the actual bounded receiver without draining, making overload
    // deterministic independently of filesystem speed and thread scheduling.
    let (sender, receiver) = mpsc::sync_channel(8);
    let mut recorder = Recorder {
        sender: Some(sender),
        worker: None,
        frames: 0,
        bytes: 0,
        failed: false,
    };
    for _ in 0..8 {
        let (scene, catalog) = empty();
        recorder.record(scene, catalog, vec![]).unwrap();
    }
    let (scene, catalog) = empty();
    assert_eq!(
        recorder
            .record(scene.clone(), catalog.clone(), vec![])
            .unwrap_err()
            .kind(),
        io::ErrorKind::WouldBlock
    );
    assert_eq!(recorder.frames, 8);
    assert!(recorder.failed);
    assert!(recorder.sender.is_none());
    assert!(recorder.record(scene, catalog, vec![]).is_err());
    assert_eq!(receiver.try_iter().count(), 8);
    let (sender, _receiver) = mpsc::sync_channel(8);
    let mut recorder = Recorder {
        sender: Some(sender),
        worker: None,
        frames: 4096,
        bytes: 0,
        failed: false,
    };
    let (scene, catalog) = empty();
    assert!(recorder.record(scene, catalog, vec![]).is_err());
    assert_eq!(recorder.frames, 4096);
    let (sender, _receiver) = mpsc::sync_channel(8);
    let mut recorder = Recorder {
        sender: Some(sender),
        worker: None,
        frames: 0,
        bytes: lines::FILE_BYTES,
        failed: false,
    };
    let (scene, catalog) = empty();
    assert!(recorder.record(scene, catalog, vec![]).is_err());
    assert_eq!(recorder.frames, 0);
}

#[test]
fn offline_export_admits_more_frames_than_the_live_queue_capacity() {
    let scratch = tempfile::tempdir().unwrap();
    let output = scratch.path().join("export");
    let frames = (1..=32).map(|sequence| {
        let (scene, catalog) = empty();
        let mut scene: serde_json::Value = serde_json::from_str(&scene).unwrap();
        let mut catalog: serde_json::Value = serde_json::from_str(&catalog).unwrap();
        scene["sequence"] = sequence.into();
        catalog["scene_sequence"] = sequence.into();
        (scene.to_string(), catalog.to_string())
    });
    super::super::export_archive(&output, BTreeMap::new(), frames).unwrap();
    let archive = Archive::open(&output).unwrap();
    assert_eq!(archive.stamps.len(), 32);
    assert_eq!(archive.stamps[31].sequence, 32);
    assert_eq!(archive.asset_prefix, super::super::EXPORT_ASSET_PREFIX);
}

#[test]
fn playback_session_preserves_the_worker_io_error_after_its_receiver_has_closed() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().to_owned();
    let file = path.join("frames.jsonl");
    File::create(&file).unwrap();
    let mut stream = File::open(file).unwrap();
    let expected = stream
        .write_all(b"frame")
        .unwrap_err()
        .raw_os_error()
        .unwrap();
    let (sender, receiver) = mpsc::sync_channel(8);
    let worker = thread::spawn(move || {
        run(
            &path,
            &mut stream,
            receiver,
            String::new(),
            BTreeMap::new(),
            Vec::new(),
        )
    });
    let mut recorder = Recorder {
        sender: Some(sender),
        worker: Some(worker),
        frames: 0,
        bytes: 0,
        failed: false,
    };
    let (scene, catalog) = empty();
    recorder.record(scene, catalog, Vec::new()).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while recorder.healthy() {
        assert!(std::time::Instant::now() < deadline);
        thread::yield_now();
    }
    assert_eq!(
        recorder.finish().unwrap_err().raw_os_error(),
        Some(expected)
    );
    assert!(!temp.path().join("manifest.json").exists());
}

#[test]
fn playback_session_rejects_out_of_scope_assets_before_creating_the_archive() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    let too_many = (0..129)
        .map(|i| (format!("/assets/0/{i}.obj"), Bytes::from_static(b"mesh")))
        .collect();
    for (assets, expected) in [
        (too_many, "recording asset count exceeds bound"),
        (
            BTreeMap::from([("/assets/no-scope".to_owned(), Bytes::from_static(b"mesh"))]),
            "recording asset paths or bytes exceed bounds",
        ),
    ] {
        let error = super::super::export_archive(&path, assets, [empty()]).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::InvalidData);
        assert_eq!(error.to_string(), expected);
        assert!(!path.exists());
    }
}

#[test]
fn playback_session_export_refuses_mismatched_owners_templates_and_oversized_catalog() {
    use serde_json::{Value, json};
    let temp = tempfile::tempdir().unwrap();
    let (scene, catalog) = empty();
    let original: Value = serde_json::from_str(&catalog).unwrap();
    let template = json!({"template_id":1,"label":"deck"});
    for (index,(key,value)) in [
        ("scene_sequence",json!(1)),
        ("world_id",json!(2)),
        ("models",json!([{"model_id": u64::from(u32::MAX)+1,"name":"deck","visual_ids":[9],"can_remove":false,"can_set_state":false}])),
        ("templates",json!([template.clone(),template])),
        ("templates",json!([{"template_id":0,"label":"deck"}])),
        ("templates",json!([{"template_id":1,"label":"x".repeat(crate::web::snapshot::MAX_BYTES)}])),
    ].into_iter().enumerate(){
        let mut bad=original.clone();
        bad[key]=value;
        let output=temp.path().join(index.to_string());
        assert!(super::super::export_archive(&output,BTreeMap::new(),[(scene.clone(),bad.to_string())]).is_err(),"{key}");
        assert!(!output.join("manifest.json").exists());
        assert!(Archive::open(&output).is_err());
    }
    let valid = temp.path().join("valid");
    super::super::export_archive(&valid, BTreeMap::new(), [(scene, catalog)]).unwrap();
    assert_eq!(Archive::open(&valid).unwrap().stamps.len(), 1);
}
