// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/selective_qualification.rs"]
mod qualification;

use gz_physics_distributed::physics::STEP;
use serde_json::Value;
use std::{fs, path::Path};

const PAIR: [&str; 4] = [
    "incoming",
    "resident",
    "source_bystander_at_plus12m",
    "destination_bystander_at_minus12m",
];

const CHAIN: [&str; 5] = [
    "body_1",
    "body_2",
    "body_3",
    "source_bystander_at_plus12m",
    "destination_bystander_at_minus12m",
];

fn sample<const N: usize>(scene: &Value, models: [&str; N]) -> [([f64; 3], u8); N] {
    let visuals = scene["visuals"].as_array().unwrap();
    models.map(|name| {
        let position = |part: &str| {
            let full = format!("default::{name}::{part}");
            let visual = visuals.iter().find(|v| v["name"] == full).unwrap();
            serde_json::from_value::<[f64; 3]>(visual["translation"].clone()).unwrap()
        };
        let center = position("core");
        let cyan = position("owner_0_0_plus") != center;
        let orange = position("owner_1_0_plus") != center;
        assert_ne!(cyan, orange);
        (center, u8::from(orange))
    })
}

fn check<const N: usize>(models: [&str; N]) {
    let size = N - 2;
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir().unwrap();
    let reliable = dir.path().join("reliable");
    let retry = dir.path().join("retry");
    let tick = qualification::run(worker, Some(&reliable), false, size).unwrap();
    assert_eq!(
        tick,
        qualification::run(worker, Some(&retry), true, size).unwrap()
    );
    for file in ["manifest.json", "frames.jsonl"] {
        assert_eq!(
            fs::read(reliable.join(file)).unwrap(),
            fs::read(retry.join(file)).unwrap()
        );
    }
    let stream = fs::read_to_string(retry.join("frames.jsonl")).unwrap();
    let mut samples = Vec::new();
    for line in stream.lines() {
        let record: Value = serde_json::from_str(line).unwrap();
        if record["kind"] == "frame" {
            let scene: Value = serde_json::from_str(record["scene"].as_str().unwrap()).unwrap();
            let ns = u128::from(scene["seconds"].as_u64().unwrap()) * 1_000_000_000
                + u128::from(scene["nanoseconds"].as_u64().unwrap());
            samples.push((ns, sample(&scene, models)));
        }
    }
    let at_transfer = samples
        .iter()
        .filter(|(ns, _)| *ns == u128::from(tick) * STEP.as_nanos())
        .collect::<Vec<_>>();
    assert_eq!(at_transfer.len(), 2);
    let (_, before) = at_transfer[0];
    let (_, after) = at_transfer[1];
    assert_eq!(
        before.map(|(center, _)| center),
        after.map(|(center, _)| center)
    );
    for (i, ((_, old), (_, new))) in before.iter().zip(after).enumerate() {
        assert_eq!(*old, u8::from(i == N - 1));
        assert_eq!(*new, u8::from(i != N - 2));
    }
    for i in 0..N {
        let bounds = samples
            .iter()
            .map(|(_, frame)| frame[i].0[1])
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
                (lo.min(x), hi.max(x))
            });
        assert!(
            bounds.1 - bounds.0 > 0.2,
            "body {i} must visibly move along physical X"
        );
    }
    let (_, first) = samples.first().unwrap();
    assert!((first[N - 2].0[2] - 1.15).abs() < 1e-12 && (first[N - 1].0[2] + 1.15).abs() < 1e-12);
}
#[test]
fn selective_retry_and_reliable_replays_agree_with_monolithic_solve() {
    check(PAIR);
}

#[test]
fn three_boxes_with_two_live_contacts_move_together_after_lost_activation() {
    check(CHAIN);
}
