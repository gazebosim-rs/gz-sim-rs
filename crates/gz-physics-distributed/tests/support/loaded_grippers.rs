// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Inputs, Result, production_example, sha256};
use serde_json::{Value, json};
use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
    process::Command,
};

const LOADED_CUT: usize = 750;
const LAST_TICK: usize = 1020;

pub(super) fn run(input: Inputs) -> Result<()> {
    let (artifact, program) = production_example::build(
        &input,
        "gz-sim-cli",
        "grasp_fleet_profiles",
        "worker-grippers",
    )?;
    let sandbox = tempfile::tempdir_in(&input.directory)?;
    let report = sandbox.path().join("report");
    let status = Command::new(&program)
        .arg(input.root.join("examples/distributed_grasp_fleet.sdf"))
        .arg("--worker")
        .arg(&input.worker)
        .arg("--storage")
        .arg(sandbox.path())
        .args([
            "--fleet",
            "round-robin",
            "--ticks",
            &LAST_TICK.to_string(),
            "--record-at",
            &LOADED_CUT.to_string(),
        ])
        .arg(&report)
        .env(
            "LLVM_PROFILE_FILE",
            input.directory.join("raw/worker-grippers-%p-%m.profraw"),
        )
        .stdout(File::create_new(
            input.directory.join("worker-grippers.stdout"),
        )?)
        .stderr(File::create_new(
            input.directory.join("worker-grippers.stderr"),
        )?)
        .status()?;
    if !status.success() {
        return Err("the two-gripper comparison against ordinary physics failed".into());
    }
    // The example compares every pose, velocity, joint, motor and pad impulse
    // against its synchronous ordinary SDF world, including immediately on reopen.
    let mut witnesses = vec![];
    let mut ticks = 0;
    let frames = BufReader::new(File::open(report.join("states.jsonl"))?);
    for line in frames.lines() {
        let frame: Value = serde_json::from_str(&line?)?;
        if frame["tick"] != ticks {
            return Err("missing consecutive robot state".into());
        }
        if [0, LOADED_CUT - 1, LOADED_CUT, LOADED_CUT + 1, LAST_TICK].contains(&ticks) {
            let hands = frame["assemblies"].as_array().ok_or("missing grippers")?;
            if hands.len() != 2 || frame["owners"] != json!([0, 1]) {
                return Err("expected two independently owned grippers".into());
            }
            let mut impulses = Vec::new();
            for (i, hand) in hands.iter().enumerate() {
                let reactions = hand["payload_reactions"]
                    .as_array()
                    .ok_or("missing reactions")?;
                let [_, left, right] = reactions.as_slice() else {
                    return Err("missing finger or palm".into());
                };
                let pads = [left, right].map(|r| r["reaction"]["normal_impulse"].as_f64());
                let [Some(left), Some(right)] = pads else {
                    return Err("missing pad impulse".into());
                };
                let loaded = ticks != 0 && (ticks != LAST_TICK || i == 1);
                if (loaded && (left < 0.001 || right < 0.001))
                    || (!loaded && [left, right] != [0.0, 0.0])
                {
                    return Err(format!("unexpected pad load on hand {i} at {ticks} ms").into());
                }
                impulses.push([left, right]);
            }
            witnesses.push(json!({"tick": ticks, "pad_impulses_ns": impulses}));
        }
        ticks += 1;
    }
    let summary: Value = serde_json::from_reader(File::open(report.join("summary.json"))?)?;
    let cut = report.join("cut/barrier.gzgraspfleet");
    if ticks != LAST_TICK + 1 || summary["cut_tick"] != LOADED_CUT || fs::metadata(&cut)?.len() == 0
    {
        return Err("missing loaded recovery checkpoint or final physical state".into());
    }
    let receipt = json!({"example_sha256":sha256(&program)?, "artifact":artifact, "worker_sha256":sha256(&input.worker)?,
                         "matched_frames":ticks, "checkpoint_sha256":sha256(&cut)?, "witnesses":witnesses, "summary":summary});
    fs::write(
        input.directory.join("worker-grippers.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    Ok(())
}
