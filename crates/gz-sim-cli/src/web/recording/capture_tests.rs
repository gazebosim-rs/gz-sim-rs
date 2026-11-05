// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::archive::Archive;
use super::*;
use serde_json::json;

fn values(sequence: u64) -> (Scene, String) {
    let scene =
        json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,
        "nanoseconds":0,"sequence":sequence,"paused":true,"visuals":[]})
        .to_string();
    let catalog=json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":[]}).to_string();
    (Scene::decode(&scene, |_| false).unwrap(), catalog)
}
#[test]
fn paused_redundancy_omits_only_unchanged_no_outcome_frames() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    let (initial, catalog) = values(0);
    let mut capture = Capture::start(&path, None, &initial, &catalog).unwrap();
    let (same, catalog) = values(1);
    capture.record(&same, &catalog, vec![]).unwrap();
    let (same, catalog) = values(2);
    let outcome=world_outcome(&crate::web::Action::Pause,
        r#"{"version":1,"kind":"control","request_id":1,"action":"pause"}"#.into(),
        r#"{"version":1,"kind":"ack","request_id":1,"ok":true,"epoch":0,"seconds":0,"nanoseconds":0,"paused":true,"error":null}"#.into(),true);
    capture.record(&same, &catalog, vec![outcome]).unwrap();
    capture.finish().unwrap();
    let archive = Archive::open(&path).unwrap();
    assert_eq!(archive.stamps.len(), 2);
    assert_eq!(archive.stamps[0].sequence, 0);
    assert_eq!(archive.stamps[1].sequence, 2);
    assert_eq!(archive.frame(1).unwrap().outcomes.len(), 1);
}
