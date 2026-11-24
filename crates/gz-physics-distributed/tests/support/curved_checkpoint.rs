// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, executable, sha256};
use serde_json::{Value, json};
use std::{
    fs::{self, File},
    io::{BufRead, BufReader, Write},
    path::Path,
    process::{Command, Stdio},
};

const EXAMPLE: &str = "crates/gz-sim-cli/examples/curved_scene_crash.rs";

/// Cargo's default test pass builds a libtest harness for an example. Its
/// recursive actor protocol needs the ordinary Cargo example executable.
fn build(
    root: &Path,
    target: &Path,
    directory: &Path,
    input: &[Value],
) -> Result<(Value, std::path::PathBuf)> {
    let source = root.join(EXAMPLE);
    let harness = executable(input, &source, target, true)?;
    let test = input
        .iter()
        .find(|a| a["executable"].as_str().map(Path::new) == Some(&harness))
        .ok_or("missing Cargo test artifact")?;
    let features = test["features"]
        .as_array()
        .ok_or("missing Cargo features")?
        .iter()
        .map(|v| v.as_str().ok_or("invalid Cargo feature"))
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let output = directory.join("worker-trajectory-build.stdout");
    let status = Command::new("cargo")
        .args([
            "build",
            "--locked",
            "--profile",
            "test",
            "-p",
            "gz-sim-cli",
            "--example",
            "curved_scene_crash",
            "--no-default-features",
            "--features",
            &features.join(","),
            "--message-format=json",
        ])
        .stdout(File::create_new(&output)?)
        .stderr(File::create_new(
            directory.join("worker-trajectory-build.stderr"),
        )?)
        .status()?;
    if !status.success() {
        return Err("building the production curved-scene example failed".into());
    }
    let mut binaries = Vec::new();
    for line in BufReader::new(File::open(output)?).lines() {
        let artifact: Value = serde_json::from_str(&line?)?;
        if artifact["reason"] == "compiler-artifact"
            && artifact["features"] == test["features"]
            && artifact["target"]["src_path"] == test["target"]["src_path"]
            && artifact["profile"]["test"] == false
        {
            binaries.push(artifact);
        }
    }
    let program = executable(&binaries, &source, target, false)?;
    Ok((binaries.pop().ok_or("missing production example")?, program))
}

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

pub(super) fn run(
    root: &Path,
    target: &Path,
    directory: &Path,
    artifacts: &[Value],
    worker: &Path,
) -> Result<()> {
    let (artifact, program) = build(root, target, directory, artifacts)?;
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
