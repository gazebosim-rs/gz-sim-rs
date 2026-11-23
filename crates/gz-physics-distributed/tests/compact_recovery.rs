// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/compact_run.rs"]
mod run;
use std::path::Path;

#[test]
fn bounded_native_baselines_recover_both_real_owners_and_retain_warm_pending_input() {
    let dir = tempfile::tempdir().unwrap();
    let report = run::run(Path::new(env!("CARGO_BIN_EXE_physics_worker")), dir.path()).unwrap();
    assert_eq!(report.ticks, 200);
    assert!(report.baselines.len() > 15 && report.capacity_refusals > 10);
    assert_eq!(report.replacements.len(), 4);
    assert_eq!(
        report
            .replacements
            .iter()
            .map(|r| r.suffix_entries)
            .collect::<Vec<_>>(),
        [0, 0, 5, 5]
    );
    assert!(report.replacements.iter().all(|r| r.full_entries > 100));
    assert_eq!(report.post_handoff_generations, [3, 2]);
    assert!(report.queued_impulse_residual_ns < 1e-12);
    assert!(report.replay_samples >= 50);
    // Every read-only visual uses a committed measurement, including both
    // sides of a same-tick compaction and a replay of an empty suffix.
    let raw = std::fs::read_to_string(dir.path().join("replay/frames.jsonl")).unwrap();
    let accepted: serde_json::Value =
        serde_json::from_reader(std::fs::File::open(dir.path().join("verification.json")).unwrap())
            .unwrap();
    let trace = accepted["display_trace"].as_array().unwrap();
    let mut count = 0;
    for line in raw.lines() {
        let record: serde_json::Value = serde_json::from_str(line).unwrap();
        if record["kind"] != "frame" {
            continue;
        }
        let scene: serde_json::Value =
            serde_json::from_str(record["scene"].as_str().unwrap()).unwrap();
        let sample = &trace[count];
        count += 1;
        let tick = sample["tick"].as_u64().unwrap();
        assert_eq!(
            scene["seconds"].as_u64().unwrap() * 1_000_000_000
                + scene["nanoseconds"].as_u64().unwrap(),
            tick * 10_000_000
        );
        for w in 0..2 {
            let name = format!("default::worker_{w}_retained_entries_0_to_12::retained");
            let cursor = scene["visuals"]
                .as_array()
                .unwrap()
                .iter()
                .find(|v| v["name"] == name)
                .unwrap();
            let expected = 2.5 - 5.0 * sample["retained"][w].as_u64().unwrap() as f64 / 12.0;
            assert_eq!(
                cursor["translation"][1].as_f64().unwrap().to_bits(),
                expected.to_bits()
            );
        }
    }
    assert_eq!(count, report.replay_samples);
}

#[test]
fn empty_suffix_preserves_baseline_and_cannot_admit_an_over_budget_handoff() {
    use gz_physics_distributed::{
        coordinator::{Coordinator, Error},
        physics::fixture,
        transport::WorkerProcess,
    };
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let run = u64::from(std::process::id());
    let bodies = fixture();
    let peers = [
        WorkerProcess::spawn(worker, run, 0, &bodies[..1]).unwrap(),
        WorkerProcess::spawn(worker, run, 1, &bodies[1..]).unwrap(),
    ];
    let mut world = Coordinator::new(run, peers, 1).unwrap();
    let start = world.frame().clone();
    assert!(matches!(world.advance(), Err(Error::Capacity)));
    let original = world.compact().unwrap();
    assert_eq!(original.released_entries, [1, 1]);
    let duplicate = world.compact().unwrap();
    assert_eq!(duplicate.sequences, original.sequences);
    assert_eq!(duplicate.bootstrap_bytes, 0);
    assert!(matches!(world.transfer(1, true), Err(Error::Capacity)));
    assert_eq!(world.frame(), &start);
    assert_eq!(world.retained_entries(), [0, 0]);
    world.recover(0).unwrap();
    world.recover(1).unwrap();
    let next = world.advance_overlapped().unwrap();
    assert_eq!(next.tick, 1);
    assert_eq!(next.workers[0].generations[&1], 0);
    assert!(next.body(1).unwrap().1.position[0] > bodies[0].position[0]);
    for mut worker in world.into_peers() {
        worker.shutdown().unwrap();
    }
}
