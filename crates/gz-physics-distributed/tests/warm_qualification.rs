// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/warm_qualification.rs"]
mod qualification;

use serde_json::Value;
use std::{fs, path::Path};

#[test]
fn lost_activation_ack_matches_reliable_handoff_and_preserves_both_replay_poses() {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir().unwrap();
    let reliable = dir.path().join("reliable");
    let retry = dir.path().join("retry");
    let tick = qualification::run(worker, Some(&reliable), false).unwrap();
    assert_eq!(
        qualification::run(worker, Some(&retry), true).unwrap(),
        tick
    );
    for file in ["manifest.json", "frames.jsonl"] {
        assert_eq!(
            fs::read(reliable.join(file)).unwrap(),
            fs::read(retry.join(file)).unwrap()
        );
    }
    let stream = fs::read_to_string(retry.join("frames.jsonl")).unwrap();
    let transfer_ns = u128::from(tick) * gz_physics_distributed::physics::STEP.as_nanos();
    let mut at_barrier = Vec::new();
    for line in stream.lines() {
        let row: Value = serde_json::from_str(line).unwrap();
        if row["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(row["scene"].as_str().unwrap()).unwrap();
        let ns = u128::from(scene["seconds"].as_u64().unwrap()) * 1_000_000_000
            + u128::from(scene["nanoseconds"].as_u64().unwrap());
        if ns != transfer_ns {
            continue;
        }
        let visuals = scene["visuals"].as_array().unwrap();
        let point = |name: &str| {
            let v = visuals.iter().find(|v| v["name"] == name).unwrap();
            serde_json::from_value::<[f64; 3]>(v["translation"].clone()).unwrap()
        };
        let sampled = ["incoming", "resident"].map(|body| {
            let field = |part: &str| point(&format!("default::{body}::{part}"));
            let center = field("core");
            let visible = [
                field("owner_0_0_plus") != center,
                field("owner_1_0_plus") != center,
            ];
            (center, visible)
        });
        at_barrier.push(sampled);
    }
    assert_eq!(at_barrier.len(), 2);
    for (before, after) in at_barrier[0].iter().zip(&at_barrier[1]) {
        assert_eq!(before.0, after.0);
        assert_eq!(before.1, [true, false]);
        assert_eq!(after.1, [false, true]);
    }
}
