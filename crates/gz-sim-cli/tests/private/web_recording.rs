// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Current-delivery capture through real native Rapier and the maintained socket.
use super::*;

#[test]
fn real_native_browser_mutations_motion_reset_and_raw_outcomes_are_recorded_before_coalescing() {
    let mut fixture = Fixture::new(true);
    assert!(fixture.bridge.recording.is_none());
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("capture");
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    fixture.address = listener.local_addr().unwrap();
    let bridge = Bridge::with_recording_options(
        fixture.server.simulation(),
        "default",
        listener,
        Some(&fixture.sdf),
        Some(&path),
    )
    .unwrap();
    drop(std::mem::replace(&mut fixture.bridge, bridge));
    let mut socket = fixture.connect("/?lifecycle=1");
    let (initial, catalog) = fixture.bootstrap(&mut socket);
    let world = initial["world_id"].as_u64().unwrap();
    let dynamic_visual = catalog["models"]
        .as_array()
        .unwrap()
        .iter()
        .find(|model| model["name"] == "default::box")
        .unwrap()["visual_ids"][0]
        .as_u64()
        .unwrap();
    let initial_position = initial["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|visual| visual["id"] == dynamic_visual)
        .unwrap()["translation"]
        .clone();
    let (_, _, added) = fixture.command(
        &mut socket,
        spawn(1, world, 0, template(&catalog, "box"), [4.0, 1.0, 3.0]),
    );
    assert_eq!(added["ok"], true);
    let id = added["model_id"].as_u64().unwrap();
    let (_, _, removed) = fixture.command(&mut socket, target(2, world, 0, "remove", id));
    assert_eq!(removed["ok"], true);
    let (moving, _, resumed) = fixture.command(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":3,"action":"resume"}),
    );
    assert_eq!(resumed["ok"], true);
    assert_eq!(moving["nanoseconds"], 5_000_000);
    let moving_visual = moving["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|visual| visual["id"] == dynamic_visual)
        .unwrap();
    assert_eq!(moving_visual["id"], dynamic_visual);
    assert!(
        moving_visual["translation"][2].as_f64().unwrap() < initial_position[2].as_f64().unwrap(),
        "the same native dynamic body must fall during the recorded advancing tick"
    );
    let (_, _, paused) = fixture.command(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":4,"action":"pause"}),
    );
    assert_eq!(paused["ok"], true);
    let (_, _, reset) = fixture.command(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":5,"action":"reset"}),
    );
    assert_eq!(reset["ok"], true);
    fixture.bridge.finish_recording().unwrap();
    let frames = super::super::recording::admitted_frames(&path).unwrap();
    assert_eq!(frames.len(), 6);
    assert_eq!(frames[0]["scene"]["epoch"], 0);
    assert_eq!(frames[0]["scene"]["nanoseconds"], 0);
    assert_eq!(frames[1]["outcomes"][0]["action"], "spawn");
    assert_eq!(frames[1]["outcomes"][0]["model_id"], id);
    assert_eq!(frames[2]["outcomes"][0]["action"], "remove");
    assert_eq!(
        frames[1]["scene"]["nanoseconds"],
        frames[2]["scene"]["nanoseconds"]
    );
    assert_eq!(frames[3]["scene"], moving);
    assert_eq!(frames[5]["scene"]["epoch"], 1);
    assert_eq!(frames[5]["scene"]["nanoseconds"], 0);
    let initial_ids: Vec<_> = initial["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["id"].clone())
        .collect();
    let final_ids: Vec<_> = frames[5]["scene"]["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["id"].clone())
        .collect();
    assert_eq!(initial_ids, final_ids);
    for (frame, action) in frames[1..]
        .iter()
        .zip(["spawn", "remove", "resume", "pause", "reset"])
    {
        let outcome = &frame["outcomes"][0];
        assert_eq!(outcome["ok"], true);
        let request: Value = serde_json::from_str(outcome["request"].as_str().unwrap()).unwrap();
        assert_eq!(request["action"], action);
        let result: Value = serde_json::from_str(outcome["result"].as_str().unwrap()).unwrap();
        assert_eq!(result["ok"], true);
    }
    assert!(fixture.bridge.recording.is_none());
    socket.close(None).unwrap();
    drop(socket);
}

#[test]
fn public_recording_constructor_preserves_native_state_and_refuses_existing_output() {
    let fixture = Fixture::new(true);
    let before = native(fixture.server.simulation());
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("capture");
    let mut bridge = Bridge::new_recording(
        fixture.server.simulation(),
        "default",
        0,
        &fixture.sdf,
        &path,
    )
    .unwrap();
    bridge.finish_recording().unwrap();
    drop(bridge);
    let frames = super::super::recording::admitted_frames(&path).unwrap();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0]["scene"]["epoch"], 0);
    assert_eq!(frames[0]["scene"]["seconds"], 0);
    assert_eq!(frames[0]["scene"]["nanoseconds"], 0);
    assert_eq!(native(fixture.server.simulation()), before);
    assert!(
        Bridge::new_recording(
            fixture.server.simulation(),
            "default",
            0,
            &fixture.sdf,
            &path,
        )
        .is_err()
    );
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        super::super::recording::admitted_frames(&path).unwrap(),
        frames
    );
}
