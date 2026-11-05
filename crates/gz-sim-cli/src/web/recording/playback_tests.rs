// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::web::recording::writer::Recorder;
use serde_json::{Value, json};
use std::time::Duration;

fn archive(path: &Path) {
    let mut recorder = Recorder::start(path, None).unwrap();
    for (sequence, epoch, seconds, paused) in [(0, 0, 0, false), (1, 0, 1, false), (2, 1, 0, true)]
    {
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":epoch,"seconds":seconds,
            "nanoseconds":0,"sequence":sequence,"paused":paused,"visuals":[]}).to_string();
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":epoch,"scene_sequence":sequence,"templates":[],"models":[]}).to_string();
        recorder.record(scene, catalog, vec![]).unwrap();
    }
    recorder.finish().unwrap();
}
fn control(action: &str, id: u64) -> web::Control {
    web::Control::decode(
        &json!({"version":1,"kind":"control","request_id":id,"action":action}).to_string(),
        0,
    )
    .unwrap()
}
fn value(text: &str) -> Value {
    serde_json::from_str(text).unwrap()
}

#[test]
fn real_cursor_controls_preserve_source_time_and_credit_fences() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    archive(&path);
    let now = Instant::now();
    let mut player = Player::open(&path, now).unwrap();
    assert!(player.assets.is_none());
    let initial = player.present().unwrap();
    assert_eq!(value(&initial.scene)["paused"], true);
    assert_eq!(value(&initial.status)["recorded_paused"], false);
    assert_eq!(value(&initial.status)["recorded_sequence"], 0);
    assert_eq!(value(&initial.status)["frame"], 1);
    let (_, ack) = player.control(&control("resume", 1), now).unwrap();
    assert_eq!(value(&ack)["ok"], true);
    assert!(!player.advance(now + Duration::from_secs(2)).unwrap());
    player.accept(initial.credit);
    assert!(player.advance(now + Duration::from_secs(2)).unwrap());
    let second = player.present().unwrap();
    assert_eq!(value(&second.scene)["seconds"], 1);
    assert_eq!(value(&second.status)["frame"], 2);
    player.accept(second.credit);
    assert!(player.advance(now + Duration::from_secs(2)).unwrap());
    let last = player.present().unwrap();
    assert_eq!(value(&last.scene)["epoch"], 1);
    assert_eq!(value(&last.scene)["seconds"], 0);
    assert_eq!(value(&last.status)["recorded_epoch"], 1);
    assert_eq!(value(&last.status)["finished"], true);
    let (_, ack) = player.control(&control("resume", 2), now).unwrap();
    assert_eq!(value(&ack)["ok"], false);
    assert_eq!(value(&ack)["paused"], true);
    let (reset, ack) = player.control(&control("reset", 3), now).unwrap();
    assert_eq!(value(&ack)["epoch"], 2);
    assert_eq!(value(&ack)["seconds"], 0);
    assert_eq!(value(&ack)["paused"], true);
    assert_eq!(value(&reset.status)["frame"], 1);
    player.control(&control("resume", 4), now).unwrap();
    player.accept(last.credit);
    player.accept(initial.credit);
    assert!(!player.advance(now + Duration::from_secs(10)).unwrap());
    player.accept(reset.credit);
    assert!(player.advance(now + Duration::from_secs(10)).unwrap());
}

#[test]
fn readonly_refusals_and_idle_tuples_remain_coherent_without_advancing() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    archive(&path);
    let mut player = Player::open(&path, Instant::now()).unwrap();
    let initial = player.present().unwrap();
    let later = player.present().unwrap();
    assert_eq!(initial.credit, later.credit);
    assert!(
        value(&later.scene)["sequence"].as_u64().unwrap()
            > value(&initial.scene)["sequence"].as_u64().unwrap()
    );
    assert_eq!(
        value(&later.status)["recorded_sequence"],
        value(&initial.status)["recorded_sequence"]
    );
    assert_eq!(
        value(&later.catalog)["scene_sequence"],
        value(&later.status)["scene_sequence"]
    );
    let request = Request::decode(r#"{"version":1,"kind":"lifecycle","request_id":9,"world_id":1,"epoch":0,"action":"get_state","model_id":2}"#,0).unwrap();
    let (presentation, ack) = player.refuse_lifecycle(&request).unwrap();
    assert_eq!(value(&ack)["ok"], false);
    assert_eq!(value(&ack)["error"], "recorded playback is read-only");
    assert_eq!(
        value(&ack)["scene_sequence"],
        value(&presentation.scene)["sequence"]
    );
    assert_eq!(value(&presentation.status)["frame"], 1);
    assert_eq!(value(&presentation.catalog)["templates"], json!([]));
    player.sequence = u64::MAX;
    assert!(player.present().is_err());
    assert_eq!(player.sequence, u64::MAX);
}

#[test]
fn startup_preflights_worst_case_transport_budget_and_control_failure_is_atomic() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("near-wire-bound");
    let visuals:Vec<_> = (0..128).map(|i| json!({"id":1000+i,"name":"","translation":[0.0,0.0,0.0],
        "rotation":[0.0,0.0,0.0,1.0],"shape":{"kind":"box","size":[1.0,1.0,1.0]},"color":[0.0,0.0,1.0,1.0]})).collect();
    let mut scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,
        "nanoseconds":0,"sequence":0,"paused":true,"visuals":visuals});
    let target = web::snapshot::MAX_BYTES - 8;
    let mut remaining = target - scene.to_string().len();
    for visual in scene["visuals"].as_array_mut().unwrap() {
        let controls = (remaining / 6).min(250);
        let mut name = "\u{1}".repeat(controls);
        remaining -= controls * 6;
        if remaining < 6 {
            name.push_str(&"x".repeat(remaining));
            remaining = 0;
        }
        visual["name"] = json!(name);
        if remaining == 0 {
            break;
        }
    }
    assert_eq!(remaining, 0);
    let scene = scene.to_string();
    assert_eq!(scene.len(), target);
    let models:Vec<_>=(0..128).map(|i|json!({"model_id":2+i,"name":"model","visual_ids":[1000+i],"can_remove":true,"can_set_state":true})).collect();
    let catalog=json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":0,"templates":[],"models":models}).to_string();
    let mut recorder = Recorder::start(&path, None).unwrap();
    recorder.record(scene, catalog, vec![]).unwrap();
    recorder.finish().unwrap();
    assert!(Archive::open(&path).is_ok());
    assert!(Player::open(&path, Instant::now()).is_err());
    let path = temp.path().join("control-overflow");
    archive(&path);
    let now = Instant::now();
    let mut player = Player::open(&path, now).unwrap();
    player.sequence = u64::MAX;
    for action in ["resume", "pause", "reset"] {
        assert!(player.control(&control(action, 1), now).is_err());
        assert_eq!(player.clock.frame(), 0);
        assert_eq!(player.clock.epoch(), 0);
        assert!(!player.clock.playing());
    }
}

#[test]
fn reconnect_cannot_reuse_an_old_consumers_accepted_frame() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("capture");
    archive(&path);
    let now = Instant::now();
    let mut player = Player::open(&path, now).unwrap();
    let initial = player.present().unwrap();
    player.accept(initial.credit);
    player.disconnect(now);
    player.control(&control("resume", 1), now).unwrap();
    assert!(!player.advance(now + Duration::from_secs(10)).unwrap());
    let fresh = player.present().unwrap();
    player.accept(fresh.credit);
    assert!(player.advance(now + Duration::from_secs(10)).unwrap());
}

#[test]
fn public_replay_entry_checks_archives_and_ports_and_honors_requested_shutdown() {
    use std::sync::atomic::AtomicBool;
    let temporary = tempfile::tempdir().unwrap();
    let path = temporary.path().join("capture");
    archive(&path);
    let manifest = std::fs::read(path.join("manifest.json")).unwrap();
    let stream = std::fs::read(path.join("frames.jsonl")).unwrap();
    let stop = AtomicBool::new(true);
    // A termination request arriving during startup must still release the
    // admitted snapshot and ephemeral local listener without constructing a world.
    crate::web::replay(&path, 0, &stop).unwrap();
    assert!(crate::web::replay(&temporary.path().join("missing"), 0, &stop).is_err());
    let occupied = std::net::TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, 0)).unwrap();
    let port = occupied.local_addr().unwrap().port();
    let error = crate::web::replay(&path, port, &stop).unwrap_err();
    assert_eq!(error.kind(), std::io::ErrorKind::AddrInUse);
    assert_eq!(std::fs::read(path.join("manifest.json")).unwrap(), manifest);
    assert_eq!(std::fs::read(path.join("frames.jsonl")).unwrap(), stream);
}
