// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, executable};
use serde_json::Value;
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
    process::Command,
};

/// Cargo's default test pass builds a libtest harness for an example. Its
/// physical scenarios need the ordinary executable with the same feature graph.
pub(super) fn build(
    root: &Path,
    target: &Path,
    directory: &Path,
    input: &[Value],
    example: &str,
    label: &str,
) -> Result<(Value, std::path::PathBuf)> {
    let source = root
        .join("crates/gz-sim-cli/examples")
        .join(format!("{example}.rs"));
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
    let output = directory.join(format!("{label}-build.stdout"));
    let mut cargo = Command::new("cargo");
    cargo.args(["build", "--profile", "test"]);
    if let Some(selection) = std::env::var_os("GZ_PHYSICS_CARGO_SELECTION") {
        let selected: Vec<String> =
            serde_json::from_str(selection.to_str().ok_or("invalid Cargo selection")?)?;
        // Preserve the profile's workspace package selection and unified dependency features.
        // Only the test-run switches change when building an ordinary example.
        cargo.args(
            selected
                .into_iter()
                .filter(|v| v != "--no-fail-fast" && v != "--all-targets"),
        );
    } else {
        cargo.args([
            "--locked",
            "-p",
            "gz-sim-cli",
            "--no-default-features",
            "--features",
            &features.join(","),
        ]);
    }
    let status = cargo
        .args(["--example", example, "--message-format=json"])
        .stdout(File::create_new(&output)?)
        .stderr(File::create_new(
            directory.join(format!("{label}-build.stderr")),
        )?)
        .status()?;
    if !status.success() {
        return Err(format!("building the production {example} example failed").into());
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
