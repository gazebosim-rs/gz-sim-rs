// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Inputs, Result, executable, production_example, sha256};
use serde_json::{Value, json};
use std::{fs, net::TcpListener, path::Path, process::Command};

fn checked(command: &mut Command, directory: &Path, label: &str) -> Result<()> {
    let output = command
        .env(
            "LLVM_PROFILE_FILE",
            directory.join("raw/worker-supervisor-%p-%m.profraw"),
        )
        .output()?;
    fs::write(
        directory.join(format!("worker-supervisor-{label}.stdout")),
        &output.stdout,
    )?;
    fs::write(
        directory.join(format!("worker-supervisor-{label}.stderr")),
        &output.stderr,
    )?;
    if !output.status.success() {
        return Err(format!(
            "supervisor {label}: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    Ok(())
}

pub(super) fn run(input: Inputs) -> Result<()> {
    let bins = input
        .worker
        .parent()
        .ok_or("missing Cargo binary directory")?;
    let mut examples = vec![];
    for (package, name) in [
        ("gz-sim-cli", "convoy_operator_fixture"),
        ("gz-sim-cli", "convoy_supervisor_trial"),
        ("gz-sim-cli", "driven_hinge"),
        ("gz-physics-distributed", "convoy_drive_oracle"),
    ] {
        let (artifact, program) =
            production_example::build(&input, package, name, &format!("worker-supervisor-{name}"))?;
        if program != bins.join("examples").join(name) {
            return Err(format!("unexpected Cargo example path: {program:?}").into());
        }
        examples.push(json!({"artifact":artifact,"example_sha256":sha256(&program)?}));
    }
    for (name, path) in [
        (
            "convoy_inspect",
            "gz-physics-distributed/src/bin/convoy_inspect.rs",
        ),
        (
            "gz-sim-convoy-checkpoint",
            "gz-sim-cli/src/bin/gz-sim-convoy-checkpoint.rs",
        ),
        (
            "gz-sim-convoy-control",
            "gz-sim-cli/src/bin/gz-sim-convoy-control.rs",
        ),
    ] {
        if executable(
            &input.artifacts,
            &input.root.join("crates").join(path),
            &input.target,
            false,
        )? != bins.join(name)
        {
            return Err(format!("unexpected Cargo tool path: {name}").into());
        }
    }
    let scratch = tempfile::tempdir_in(&input.directory)?;
    let cuts = scratch.path().join("cuts");
    checked(
        Command::new(bins.join("examples/convoy_operator_fixture"))
            .arg("--worker")
            .arg(&input.worker)
            .args(["generate", "--tick", "150", "--pending"])
            .arg(&cuts),
        &input.directory,
        "generate",
    )?;
    let references = json!({"samples":[{"tick":150,"sha256":sha256(&cuts.join("initial.gzmultisphere"))?},
                                     {"tick":151,"sha256":sha256(&cuts.join("next.gzmultisphere"))?}]});
    let reference = scratch.path().join("reference.json");
    fs::write(&reference, serde_json::to_vec(&references)?)?;
    let program = scratch.path().join("program.json");
    fs::write(
        &program,
        serde_json::to_vec(&json!({"version":1,"interpolation":"hold","end":"hold",
        "knots":[{"tick":0,"torque_nm":0.01},{"tick":152,"torque_nm":0.01}]}))?,
    )?;
    let trial = scratch.path().join("trial");
    let port = TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, 0))?
        .local_addr()?
        .port();
    checked(
        Command::new(bins.join("examples/convoy_supervisor_trial"))
            .args(["--case", "paused", "--release"])
            .arg(bins)
            .arg("--worker")
            .arg(&input.worker)
            .args(["--port", &port.to_string(), "--run"])
            .arg(&trial)
            .arg("--source")
            .arg(cuts.join("initial.gzmultisphere"))
            .arg("--sdf")
            .arg(
                input
                    .root
                    .join("examples/distributed_sphere_pair_island.sdf"),
            )
            .arg("--program")
            .arg(&program)
            .arg("--reference")
            .arg(&reference),
        &input.directory,
        "trial",
    )?;
    let read =
        |name| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(trial.join(name))?)?) };
    let crash = read("result.json")?;
    let orderly = read("orderly.json")?;
    if crash["ordinary_observations"] != 2
        || crash["exact_native_archives"] != 2
        || crash["durable_pending_nm"] != 0.01
        || !crash["source_impulse_ns"].as_f64().is_some_and(|i| i > 0.)
        || orderly["normal_exits"] != 2
        || orderly["refused_commands"] != 2
        || orderly["tick"] != 151
        || orderly["archive_sha256"] != references["samples"][1]["sha256"]
    {
        return Err("incomplete supervisor contact or operator comparison".into());
    }
    let receipt = json!({"examples":examples,"crash":crash,"orderly":orderly,"oracle":read("oracle.json")?,"events":read("events.json")?});
    fs::write(
        input.directory.join("worker-supervisor.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    Ok(())
}
