// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/qualification.rs"]
mod qualification;

use std::{fs, path::Path};

#[test]
fn separate_worker_runs_with_faults_produce_identical_native_archives() {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir().unwrap();
    let archives = ["first", "second"].map(|name| dir.path().join(name));
    for archive in &archives {
        qualification::run(worker, Some(archive)).unwrap();
    }
    for file in ["frames.jsonl", "manifest.json"] {
        let data = fs::read(archives[0].join(file)).unwrap();
        assert!(!data.is_empty());
        assert_eq!(data, fs::read(archives[1].join(file)).unwrap());
    }
}

#[test]
fn unchanged_pose_has_exactly_one_visible_owner_at_the_transfer_tick() {
    use gz_physics_distributed::physics::{Body, STEP, fixture};
    use serde_json::Value;
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir().unwrap();
    let archive = dir.path().join("capture");
    qualification::run(worker, Some(&archive)).unwrap();
    let stream = fs::read_to_string(archive.join("frames.jsonl")).unwrap();
    let seed = fixture();
    let incoming: &Body = &seed[0];
    let mut before_after = vec![];
    for line in stream.lines().skip(1) {
        let row: Value = serde_json::from_str(line).unwrap();
        if row["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(row["scene"].as_str().unwrap()).unwrap();
        if scene["seconds"] != 0 || scene["nanoseconds"] != 850_000_000 {
            continue;
        }
        let visuals = scene["visuals"].as_array().unwrap();
        let point = |name: &str| {
            let v = visuals.iter().find(|v| v["name"] == name).unwrap();
            serde_json::from_value::<[f64; 3]>(v["translation"].clone()).unwrap()
        };
        let center = point("default::incoming::core");
        let cyan = point("default::incoming::owner_0_0_plus");
        let orange = point("default::incoming::owner_1_0_plus");
        assert!(
            (center[1] + incoming.position[0] + 85. * STEP.as_secs_f64() * incoming.linear[0])
                .abs()
                < 1e-12
        );
        assert_ne!(cyan == center, orange == center);
        before_after.push((center, cyan == center));
    }
    assert_eq!(before_after.len(), 2);
    assert_eq!(before_after[0].0, before_after[1].0);
    assert!(!before_after[0].1 && before_after[1].1);
}
