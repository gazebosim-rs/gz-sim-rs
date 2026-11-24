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
    capture
        .finish(&same, &Catalog::decode(&catalog, &same).unwrap())
        .unwrap();
    let archive = Archive::open(&path).unwrap();
    assert_eq!(archive.stamps.len(), 2);
    assert_eq!(archive.stamps[0].sequence, 0);
    assert_eq!(archive.stamps[1].sequence, 2);
    assert_eq!(archive.frame(1).unwrap().outcomes.len(), 1);
}

#[test]
fn sealing_a_running_capture_preserves_an_unseen_tick_without_duplicating_the_previous_cut() {
    for already_published in [true, false] {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("running");
        let (mut initial, first_catalog) = values(0);
        initial.paused = false;
        let mut capture = Capture::start(&path, None, &initial, &first_catalog).unwrap();
        let (mut changed, catalog) = values(1);
        changed.paused = false;
        changed.nanoseconds = 500_000;
        if already_published {
            capture.record(&changed, &catalog, vec![]).unwrap();
        }
        let (mut final_scene, final_catalog) = values(2);
        final_scene.paused = false;
        final_scene.nanoseconds = changed.nanoseconds;
        let final_catalog = Catalog::decode(&final_catalog, &final_scene).unwrap();
        capture.finish(&final_scene, &final_catalog).unwrap();
        let archive = Archive::open(&path).unwrap();
        assert_eq!(archive.stamps.len(), 2);
        assert_eq!(archive.stamps[1].nanoseconds, 500_000);
        assert_eq!(
            archive.stamps[1].sequence,
            if already_published { 1 } else { 2 }
        );
    }
}

#[test]
fn a_full_running_archive_cannot_be_sealed_before_its_final_cut() {
    let tmp = tempfile::tempdir().unwrap();
    let path = tmp.path().join("full");
    let (initial, catalog) = values(0);
    let mut capture = Capture::start(&path, None, &initial, &catalog).unwrap();
    for ordinal in 1..4096 {
        let (scene, catalog) = values(ordinal);
        capture
            .recorder
            .submit_offline(snapshot::encode(&scene).unwrap(), catalog)
            .unwrap();
    }
    let (mut final_scene, catalog) = values(4096);
    final_scene.nanoseconds = 500_000;
    let catalog = Catalog::decode(&catalog, &final_scene).unwrap();
    let error = capture.finish(&final_scene, &catalog).unwrap_err();
    assert!(error.to_string().contains("exceeds 4096 frames"));
    assert!(Archive::open(&path).is_err());
}

#[test]
fn final_scene_and_lifecycle_catalog_must_each_fit_the_wire_limit() {
    for excessive_scene in [false, true] {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("too-large");
        let (initial, wire) = values(0);
        let capture = Capture::start(&path, None, &initial, &wire).unwrap();
        let (scene, wire) = values(1);
        let mut scene = serde_json::to_value(scene).unwrap();
        let mut catalog: serde_json::Value = serde_json::from_str(&wire).unwrap();
        let long_name = "part".repeat(snapshot::MAX_BYTES / 4);
        if excessive_scene {
            scene["visuals"] = json!([{"id":3,"name":long_name,"translation":[0,0,0],
                "rotation":[0,0,0,1],"color":[1,1,1,1],"shape":{"kind":"box","size":[1,1,1]}}]);
        } else {
            catalog["models"] = json!([{"model_id":2,"name":long_name,"visual_ids":[3],
                "can_remove":false,"can_set_state":false}]);
        }
        let scene: Scene = serde_json::from_value(scene).unwrap();
        let catalog: Catalog = serde_json::from_value(catalog).unwrap();
        assert!(capture.finish(&scene, &catalog).is_err());
        assert!(Archive::open(&path).is_err());
    }
}
