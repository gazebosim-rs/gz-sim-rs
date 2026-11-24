// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Inputs, Result, production_example, sha256};
use serde_json::{Value, json};
use std::{
    fs,
    io::Write,
    path::Path,
    process::{Command, Stdio},
};

fn actor(
    program: &Path,
    worker: &Path,
    session: &Path,
    cut: Option<&str>,
    input: &[&str],
    pattern: &Path,
) -> Result<Vec<Value>> {
    let mut command = Command::new(program);
    command
        .arg("actor")
        .arg(worker)
        .arg(session)
        .arg("5")
        .args(cut)
        .env("LLVM_PROFILE_FILE", pattern)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command.spawn()?;
    // Closing stdin allows the production actor to exit after its last command.
    writeln!(
        child.stdin.take().ok_or("missing actor stdin")?,
        "{}",
        input.join("\n")
    )?;
    let output = child.wait_with_output()?;
    if !output.status.success() {
        return Err(format!(
            "checkpoint actor failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    let states = std::str::from_utf8(&output.stdout)?
        .lines()
        .map(serde_json::from_str)
        .collect::<std::result::Result<Vec<_>, _>>()?;
    if states.len() != input.len() + 1 {
        return Err("checkpoint actor omitted a command response".into());
    }
    Ok(states)
}

pub(super) fn run(inputs: Inputs) -> Result<()> {
    let directory = &inputs.directory;
    let worker = &inputs.worker;
    let (artifact, program) = production_example::build(
        &inputs,
        "gz-sim-cli",
        "curved_scene_crash",
        "worker-trajectory",
    )?;
    let session = tempfile::tempdir_in(directory)?;
    let pattern = directory.join("raw/worker-trajectory-%p-%m.profraw");
    let output = Command::new(&program)
        .arg("run")
        .arg(worker)
        .arg(session.path())
        .args(["5", "120"])
        .env("LLVM_PROFILE_FILE", &pattern)
        .output()?;
    fs::write(
        directory.join("worker-trajectory-crash.stdout"),
        &output.stdout,
    )?;
    fs::write(
        directory.join("worker-trajectory-crash.stderr"),
        &output.stderr,
    )?;
    if !output.status.success() {
        return Err("curved-scene crash run failed".into());
    }
    let crash: Value = serde_json::from_slice(&output.stdout)?;
    if crash["controller_sigkills"] != 16
        || crash["reopened_immutable_cuts"] != 16
        || crash["repeated_discarded_successors"] != 8
        || crash["verified_complete_native_and_public_states"] != 257
    {
        return Err("curved-scene crash run omitted a physical recovery comparison".into());
    }

    // Every crash actor receives SIGKILL by design. These two additional ordinary
    // exits retain coverage from loading, saving and advancing the same real cut.
    let saved = actor(
        &program,
        worker,
        session.path(),
        None,
        &[
            "step",
            "step",
            "step",
            "step",
            "prepare",
            "move",
            "save warm.gzcurve",
            "step",
        ],
        &pattern,
    )?;
    let resumed = actor(
        &program,
        worker,
        session.path(),
        Some("warm.gzcurve"),
        &["step", "reset"],
        &pattern,
    )?;
    if resumed[0] != saved[4]
        || resumed[1] != saved[8]
        || resumed[2]["epoch"] != 1
        || resumed[2]["tick"] != 0
    {
        return Err("graceful reopen changed the saved state, pending input, or reset".into());
    }
    let receipt = json!({"example_sha256": sha256(&program)?, "artifact":artifact, "crash":crash,
                         "restored":resumed[0], "advanced":resumed[1], "reset":resumed[2]});
    fs::write(
        directory.join("worker-trajectory.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    Ok(())
}
