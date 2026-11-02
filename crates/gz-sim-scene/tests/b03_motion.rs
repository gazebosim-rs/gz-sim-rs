// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Strict whole-world comparison against the reviewed actual native capture.
#[path = "support/b03_workload.rs"]
mod workload;
use serde_json::Value;
use std::{
    collections::{BTreeMap, BTreeSet},
    io::Write,
    process::{Command, Stdio},
};
use workload::{BODY_FIELDS, BODY_NAMES, JOINT_NAMES, TICKS, URI, Workload, XML, XML_SHA256};

// Bind the actual capture rather than constructing expected trajectories.
const NATIVE: &str = include_str!("fixtures/b03_world_native.json");
const PROVENANCE: &[u8] = include_bytes!("../../../reference/b03_world.json");
const GENERATOR: &[u8] = include_bytes!("../../../reference/b03_world.cc");
// Keep cached Server observations above as immutable historical evidence;
// current body comparisons need states resolved freshly from the engine.
const FRESH: &str = include_str!("fixtures/b03_world_fresh_native.json");
const FRESH_PROVENANCE: &[u8] = include_bytes!("../../../reference/b03_world_fresh.json");
const FRESH_GENERATOR: &[u8] = include_bytes!("../../../reference/b03_world_fresh.cc");
const FRESH_VALIDATOR: &[u8] = include_bytes!("../../../scripts/inspect-b03-fresh-world.py");
fn checksum(bytes: &[u8]) -> String {
    let mut child = Command::new("sha256sum")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(bytes).unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success() && output.stderr.is_empty());
    String::from_utf8(output.stdout)
        .unwrap()
        .split_whitespace()
        .next()
        .unwrap()
        .to_owned()
}
fn close(a: &[f64], b: &Value, tick: u64, name: &str) {
    let b: Vec<_> = b
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    assert_eq!(a.len(), b.len());
    for (index, (a, b)) in a.iter().zip(&b).enumerate() {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-12,
            "tick{tick} {name}[{index}]: {a:.17e} != {b:.17e}"
        );
    }
}
#[test]
fn unchanged_b03_actual_server_motion_matches_all_bodies_and_joints() {
    let native: Value = serde_json::from_str(NATIVE).unwrap();
    let provenance: Value = serde_json::from_slice(PROVENANCE).unwrap();
    let fresh: Value = serde_json::from_str(FRESH).unwrap();
    let fresh_provenance: Value = serde_json::from_slice(FRESH_PROVENANCE).unwrap();
    assert_eq!(fresh["schema_version"], 1);
    assert_eq!(fresh["xml_sha256"], XML_SHA256);
    assert_eq!(fresh["body_fields"], serde_json::json!(BODY_FIELDS));
    assert_eq!(
        fresh["historical_cached_fixture_sha256"],
        checksum(NATIVE.as_bytes())
    );
    assert_eq!(fresh["provenance_sha256"], checksum(FRESH_PROVENANCE));
    assert_eq!(fresh["generator_sha256"], checksum(FRESH_GENERATOR));
    assert_eq!(fresh["validator_sha256"], checksum(FRESH_VALIDATOR));
    assert_eq!(
        fresh_provenance["generator_sha256"],
        checksum(FRESH_GENERATOR)
    );
    assert_eq!(
        fresh_provenance["validator_sha256"],
        checksum(FRESH_VALIDATOR)
    );
    assert_eq!(
        fresh_provenance["native_baseline_sha256"],
        checksum(NATIVE.as_bytes())
    );
    assert_eq!(
        fresh["observations_sha256"],
        fresh_provenance["observations_sha256"]
    );
    assert_eq!(fresh["all101_cached_maximum_delta"], 0.0);
    assert_eq!(fresh["native_selected_maximum_delta"], 0.0);
    assert_eq!(
        fresh_provenance["state"],
        "Authoritative native query diagnostic collected; all101 cached observer fidelity passed against completed capture; savedRust101 comparison only; no acceptance claimed"
    );
    assert!(
        fresh_provenance.get("failure").is_none() && fresh_provenance.get("traceback").is_none()
    );
    assert_eq!(native["schema_version"], 1);
    assert_eq!(native["xml"], XML);
    assert_eq!(checksum(XML.as_bytes()), XML_SHA256);
    assert_eq!(native["xml_sha256"], XML_SHA256);
    assert_eq!(native["declared_uri"], URI);
    assert_eq!(native["generator_sha256"], checksum(GENERATOR));
    assert_eq!(provenance["generator_sha256"], checksum(GENERATOR));
    assert_eq!(native["provenance_sha256"], checksum(PROVENANCE));
    assert_eq!(
        provenance["state"],
        "Actual B03 Server observations collected; no Rust/native comparison claimed"
    );
    assert!(provenance.get("failure").is_none() && provenance.get("traceback").is_none());
    assert_eq!(
        native["capture_sha256"][".state"],
        checksum(native["raw_state"].as_str().unwrap().as_bytes())
    );
    assert_eq!(native["capture_sha256"][".input.sdf"], XML_SHA256);
    assert_eq!(native["body_fields"], serde_json::json!(BODY_FIELDS));
    for (key, value) in [
        ("steps", 100),
        ("dt_ns", 1_000_000),
        ("body_queries", 700),
        ("joint_queries", 400),
        ("worlds", 1),
        ("bodies", 7),
        ("joints", 4),
        ("colliders", 15),
        ("sensors", 0),
    ] {
        assert_eq!(native["metrics"][key], value);
    }
    assert_eq!(native["metrics"]["rendering"], false);
    // Actual native FilePath/line and SetSdfFile argument are retained, not
    // equated to the Rust caller URI: the SDK may canonicalize its file input.
    assert!(
        !native["native_source"]["set_sdf_file"]
            .as_str()
            .unwrap()
            .is_empty()
    );
    assert!(native["native_source"]["world_file_path"].is_string());
    assert!(native["native_source"]["world_line"].is_i64());
    let mut actual = Workload::new();
    close(&actual.gravity, &native["gravity"], 0, "gravity");
    let identity_map = |rows: &[Value]| {
        let mut by_name = BTreeMap::new();
        let mut ids = BTreeSet::new();
        for row in rows {
            assert!(ids.insert(row["id"].as_u64().unwrap()));
            assert!(
                by_name
                    .insert(row["name"].as_str().unwrap().to_owned(), row.clone())
                    .is_none()
            );
        }
        by_name
    };
    let native_ids = identity_map(native["entities"].as_array().unwrap());
    let actual_ids = identity_map(&actual.entities);
    assert_eq!(native_ids.len(), 31);
    assert_eq!(
        native_ids.keys().collect::<Vec<_>>(),
        actual_ids.keys().collect::<Vec<_>>()
    );
    for (name, row) in &native_ids {
        assert_eq!(row["family"], actual_ids[name]["family"]);
        let parent_name = |map: &BTreeMap<String, Value>, id: &Value| {
            map.iter()
                .find(|(_, row)| &row["id"] == id)
                .map(|(name, _)| name.clone())
        };
        assert_eq!(
            parent_name(&native_ids, &row["parent_id"]),
            parent_name(&actual_ids, &actual_ids[name]["parent_id"])
        );
    }
    let (frames, _) = actual.run();
    let actual_frames_json: Vec<_> = frames.iter().map(|frame| frame.json()).collect();
    eprintln!(
        "B03_ACTUAL_FRAMES {}",
        serde_json::to_string(&actual_frames_json).unwrap()
    );
    assert_eq!(actual_frames_json.len(), 3);
    let historical = native["frames"].as_array().unwrap();
    assert_eq!(historical.len(), 3);
    let expected = fresh["frames"].as_array().unwrap();
    assert_eq!(expected.len(), 3);
    for ((frame, expected), tick) in frames.iter().zip(expected).zip(TICKS) {
        assert_eq!(frame.tick, tick);
        assert_eq!(expected["tick"], tick);
        assert_eq!(expected["time_ns"], frame.time_ns);
        for (rows, names, family) in [
            (&expected["bodies"], BODY_NAMES.as_slice(), "BODY"),
            (&expected["joints"], JOINT_NAMES.as_slice(), "JOINT"),
        ] {
            let set: BTreeSet<_> = rows
                .as_array()
                .unwrap()
                .iter()
                .map(|r| r["name"].as_str().unwrap())
                .collect();
            assert_eq!(set, names.iter().copied().collect());
            assert_eq!(rows.as_array().unwrap().len(), names.len());
            for row in rows.as_array().unwrap() {
                assert_eq!(row["id"], native_ids[row["name"].as_str().unwrap()]["id"]);
                assert_eq!(native_ids[row["name"].as_str().unwrap()]["family"], family);
            }
        }
        for body in &frame.bodies {
            let row = expected["bodies"]
                .as_array()
                .unwrap()
                .iter()
                .find(|r| r["name"] == body.name)
                .unwrap();
            assert_eq!(actual_ids[&body.name]["id"], body.id);
            close(&body.values, &row["values"], tick, &body.name);
        }
        for joint in &frame.joints {
            let row = expected["joints"]
                .as_array()
                .unwrap()
                .iter()
                .find(|r| r["name"] == joint.name)
                .unwrap();
            assert_eq!(actual_ids[&joint.name]["id"], joint.id);
            close(&joint.values, &row["values"], tick, &joint.name);
        }
    }
}
