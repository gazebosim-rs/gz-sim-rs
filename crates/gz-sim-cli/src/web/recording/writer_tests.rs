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
