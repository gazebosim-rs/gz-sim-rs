// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Follow the crash cases with an ordinary SIGTERM, preserving the selected successor.
pub(super) fn reopen(
    a: &Args,
    session: &Path,
    next: &Value,
    durable: &Path,
    events: &mut Events,
) -> Result<Value> {
    let at = tick(next)?;
    let previous = fs::read_dir(a.run.join("snapshots"))?.collect::<std::io::Result<Vec<_>>>()?;
    check(
        previous.len() == 1,
        "expected one immutable paused snapshot",
    )?;
    let capture = a.run.join("orderly.gzconvoy");
    let mut live = Supervisor::start(a, session, &capture, None, "orderly-live")?;
    let status = live.connection(a.port)?;
    check_poses(&status.scene, next)?;
    drop(status);
    let published = live.snapshot(a, Some(&previous[0].path()))?;
    check(
        fs::read(&published)? == fs::read(&capture)?,
        "signal and startup captures disagree",
    )?;
    let workers = live.workers()?;
    check(
        workers.len() == 3,
        "unexpected warm supervisor worker count",
    )?;
    live.stop()?;
    wait_workers_gone(&workers)?;

    let (mut cold, preview) = parked(a, session, &published, "orderly-cold", at, true, events)?;
    let token = preview.offer(at, true)?.to_owned();
    check_poses(&preview.scene, next)?;
    drop(preview);
    let at_text = at.to_string();
    let operator = |args: &[&str]| {
        let mut command = Command::new(a.release.join("gz-sim-convoy-control"));
        command.args(["--port", &a.port.to_string()]).args(args);
        command
    };
    let wire = output(&mut operator(&["status"]))?;
    check(
        wire["operator_fence"]["token"] == token,
        "CLI observed a different fence",
    )?;
    for (args, message) in [
        (vec!["step", "0", &at_text], "parked workers cannot advance"),
        (
            vec!["recover", "0", &at_text, "stale"],
            "does not authorize this request",
        ),
    ] {
        let denial = operator(&args).output()?;
        check(
            !denial.status.success() && String::from_utf8_lossy(&denial.stderr).contains(message),
            format!("unexpected parked command response {denial:?}"),
        )?;
    }
    let ack = output(&mut operator(&["recover", "0", &at_text, &token]))?;
    check(
        ack["accepted"] == true,
        "CLI could not recover the selected cut",
    )?;
    let restored = cold.connection(a.port)?;
    check_poses(&restored.scene, next)?;
    check(
        restored
            .restored
            .as_ref()
            .is_some_and(|proof| proof["token"] == token),
        "lost operator identity",
    )?;
    drop(restored);
    let workers = cold.workers()?;
    check(workers.len() == 3, "three restored local workers required")?;
    cold.stop()?;
    wait_workers_gone(&workers)?;
    check(
        fs::read(session.join("barrier.gzmultisphere"))? == fs::read(durable)?,
        "normal shutdown advanced the paused archive",
    )?;
    Ok(
        json!({"tick":at,"package_sha256":hash(&published)?,"archive_sha256":hash(durable)?,
              "normal_exits":2,"restored_poses":4,"refused_commands":2}),
    )
}
