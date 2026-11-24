// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use run::{actual, boot, compare, need, reference, replay, transfer};
use serde_json::json;
use std::{thread, time::Duration};
#[derive(Clone, Copy, Serialize, Deserialize)]
struct Marker {
    stage: Stage,
    coordinator: u32,
    source: u32,
    target: u32,
}
struct WriteStop {
    path: PathBuf,
    linked: bool,
    dir: PathBuf,
    marker: Marker,
}
static WRITE_STOP: std::sync::OnceLock<WriteStop> = std::sync::OnceLock::new();
fn park(dir: &Path, marker: Marker) -> Result<()> {
    // This test-only process rendezvous is separate from the record whose
    // interrupted hard-link publication is being observed.
    let staged = dir.join("cut-ready.writing");
    let mut f = fs::File::create_new(&staged)?;
    serde_json::to_writer(&mut f, &marker)?;
    f.sync_all()?;
    fs::rename(staged, dir.join("cut-ready.json"))?;
    std::io::stdin().lock().read_to_end(&mut Vec::new())?;
    Err("crash harness closed before killing coordinator".into())
}
/// Active only in a separately launched qualification role.
pub(super) fn publication_edge(path: &Path, linked: bool) -> Result<()> {
    if let Some(stop) = WRITE_STOP.get()
        && stop.path == path
        && stop.linked == linked
    {
        return park(&stop.dir, stop.marker);
    }
    Ok(())
}
pub(super) fn role(dir: &Path, at: Stage) -> Result<()> {
    fs::create_dir(dir)?;
    let path = dir.join("decision.bin");
    durable::start_session(&path)?;
    let _controller = durable::controller(&path)?;
    let mut a = Client::start(0, &path)?;
    let mut b = Client::start(1, &path)?;
    boot(&mut a, &mut b)?;
    let (source, target) = (a.child.id(), b.child.id());
    let marker = Marker {
        stage: at,
        coordinator: std::process::id(),
        source,
        target,
    };
    let write_target = match at {
        Stage::DecisionStaged => Some((path.clone(), false)),
        Stage::DecisionLinked => Some((path.clone(), true)),
        Stage::IntentStaged => Some((path.with_extension(format!("{}.intent", CUT + 17)), false)),
        Stage::ReceiptStaged => Some((path.with_extension(format!("{}.receipt", CUT + 17)), false)),
        Stage::ReceiptLinked => Some((path.with_extension(format!("{}.receipt", CUT + 17)), true)),
        _ => None,
    };
    if let Some((output, linked)) = write_target {
        WRITE_STOP
            .set(WriteStop {
                path: output,
                linked,
                dir: dir.to_owned(),
                marker,
            })
            .map_err(|_| "write cut already installed")?;
    }
    let halt = |s: Stage| if s == at { park(dir, marker) } else { Ok(()) };
    let (ticket, _, _) = transfer(&mut a, &mut b, &path, false, halt)?;
    for offset in 1..=17 {
        let s = Step::new(
            CUT + offset,
            match offset {
                4 => Some(1.25),
                17 => Some(-0.625),
                _ => None,
            },
        );
        durable::intent(&path, ticket, s)?;
        if offset == 17 {
            halt(Stage::Intent)?;
            if at == Stage::Applied {
                b.omit(Op::Advance(s), &[])?;
                need(
                    matches!(b.call(Op::Inspect,&[])?.0,Reply::Status{tick,..} if tick==s.tick),
                    "unknown physics interval never applied",
                )?;
                halt(Stage::Applied)?;
            }
        }
        let Reply::Observed { digest, .. } = b.call(Op::Advance(s), &[])?.0 else {
            return Err("missing interval receipt".into());
        };
        durable::receipt(&path, ticket, s, digest)?;
        if offset == 17 {
            halt(Stage::Receipted)?;
        }
    }
    Err("unknown crash stage".into())
}
// A stopped participant can retain its OS lease after the controller's pipes close.
fn await_owner_release(path: &Path, limit: Duration) -> Result<u64> {
    let start = Instant::now();
    let mut waited = false;
    loop {
        if durable::try_lease(path, "owner")?.is_some() {
            return Ok(if waited {
                start.elapsed().as_nanos() as u64
            } else {
                0
            });
        }
        waited = true;
        if start.elapsed() >= limit {
            return Err("prior local owner still holds the hinge lease; recovery is fenced".into());
        }
        thread::sleep(Duration::from_millis(2));
    }
}
pub(super) fn resume(dir: &Path) -> Result<()> {
    let path = dir.join("decision.bin");
    let _controller = durable::controller(&path)?;
    let _session = durable::session(&path)?;
    let previous_owner_wait_ns = await_owner_release(&path, Duration::from_secs(5))?;
    let retired = path.with_extension("retired");
    let disposition = if path.try_exists()? {
        durable::committed(&path)?;
        "existing durable transfer"
    } else if retired.try_exists()? {
        let (d, img) = load_decision(&retired)?;
        // Preparation is local and repeatable. Re-prepare before publishing a
        // missing decision that follows a permanently fenced source retirement.
        let mut dst = Client::start(1, &path)?;
        need(
            matches!(dst.call(Op::Prepare(d.ticket),&img)?.0,Reply::Prepared(t) if t==d.ticket),
            "cannot rebuild retired target",
        )?;
        publish(&path, d, &img)?;
        "finished durable source retirement"
    } else {
        // No input can reach the target until a durable retirement and commit.
        // This pilot's deterministic authored source starts at exactly tick 120.
        let mut src = Client::start(0, &path)?;
        let mut dst = Client::start(1, &path)?;
        boot(&mut src, &mut dst)?;
        transfer(&mut src, &mut dst, &path, false, |_| Ok(()))?;
        "rebuilt deterministic precommit source"
    };
    let (d, img) = durable::committed(&path)?;
    let journal = durable::journal(&path, d.ticket)?;
    let mut target = Client::start(1, &path)?;
    replay(&mut target, &path, &journal.confirmed)?;
    let mut baseline = Tree::warm()?;
    need(
        baseline.image()? == img,
        "wrong baseline construction or native cut",
    )?;
    let mut last = CUT;
    for (s, h) in &journal.confirmed {
        baseline.step(s.effort.map(f64::from_bits))?;
        need(
            Hash::from(Sha256::digest(baseline.image()?)) == *h,
            "independent native replay differs from recorded receipt",
        )?;
        last = s.tick;
    }
    if let Some(s) = journal.outstanding {
        baseline.step(s.effort.map(f64::from_bits))?;
        let Reply::Observed { digest, .. } = target.call(Op::Advance(s), &[])?.0 else {
            return Err("outstanding interval was refused".into());
        };
        need(
            digest == Hash::from(Sha256::digest(baseline.image()?)),
            "outstanding interval digest",
        )?;
        durable::receipt(&path, d.ticket, s, digest)?;
        last = s.tick;
    }
    let observed = actual(target.call(Op::Observe, &[])?.0, target.child.id(), 1)?;
    compare(&reference(&baseline, last)?, &observed)?;
    let report = json!({
        "pid": std::process::id(),
        "target_pid": target.child.id(),
        "previous_owner_wait_ns": previous_owner_wait_ns,
        "disposition": disposition,
        "resolved_input": journal.outstanding.map(|s|s.tick),
        "prior_committed_intervals": journal.confirmed.len(),
        "logical_tick": last,
        "native_image_sha256": format!("{:x}",Sha256::digest(baseline.image()?)),
        "state": observed,
    });
    println!("{}", serde_json::to_string(&report)?);
    Ok(())
}
#[cfg(target_os = "linux")]
pub(super) fn proof(dir: &Path) -> Result<()> {
    use clap::ValueEnum;
    use rustix::process::{
        Pid, PidfdFlags, Signal, WaitOptions, pidfd_open, pidfd_send_signal, set_child_subreaper,
        waitpid,
    };
    use std::os::unix::process::ExitStatusExt;
    set_child_subreaper(Pid::from_raw(1))?;
    fs::create_dir(dir)?;
    let mut reports = Vec::new();
    for &stage in Stage::value_variants() {
        let name = stage.to_possible_value().unwrap().get_name().to_owned();
        let case = dir.join(&name);
        let mut child = Command::new(std::env::current_exe()?)
            .arg("crash-role")
            .arg(&case)
            .arg(&name)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn()?;
        let ready = case.join("cut-ready.json");
        let deadline = Instant::now() + Duration::from_secs(20);
        while !ready.try_exists()? {
            if let Some(s) = child.try_wait()? {
                return Err(format!("coordinator crashed prematurely at {stage:?}: {s}").into());
            }
            if Instant::now() > deadline {
                child.kill()?;
                child.wait()?;
                return Err(format!("timeout before {stage:?}").into());
            }
            thread::sleep(Duration::from_millis(2));
        }
        let marker: Marker = serde_json::from_slice(&fs::read(&ready)?)?;
        need(
            marker.stage == stage && marker.coordinator == child.id(),
            "unexpected members at cut",
        )?;
        let competing = Command::new(std::env::current_exe()?)
            .arg("resume")
            .arg(&case)
            .output()?;
        need(
            !competing.status.success()
                && String::from_utf8_lossy(&competing.stderr)
                    .contains("another local hinge controller"),
            "duplicate controller started",
        )?;
        let children = [marker.source, marker.target].map(|id| Pid::from_raw(id as i32).unwrap());
        let fds = children
            .map(|id| pidfd_open(id, PidfdFlags::empty()))
            .into_iter()
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let stopped = match stage {
            Stage::Frozen => Some(0),
            Stage::Applied => Some(1),
            _ => None,
        };
        if let Some(i) = stopped {
            pidfd_send_signal(&fds[i], Signal::STOP)?;
            let proc = PathBuf::from(format!("/proc/{}/status", children[i].as_raw_nonzero()));
            while !fs::read_to_string(&proc)?
                .lines()
                .any(|s| s.starts_with("State:") && s.contains("T (stopped)"))
            {
                if Instant::now() > deadline {
                    pidfd_send_signal(&fds[i], Signal::CONT)?;
                    return Err("owner did not stop at requested cut".into());
                }
                thread::sleep(Duration::from_millis(1));
            }
        }
        child.kill()?;
        need(
            child.wait()?.signal() == Some(9),
            "coordinator was not killed",
        )?;
        let mut fresh = Command::new(std::env::current_exe()?)
            .arg("resume")
            .arg(&case)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()?;
        if let Some(i) = stopped {
            let path = case.join("decision.bin");
            while durable::try_lease(&path, "controller")?.is_some() {
                if Instant::now() > deadline {
                    pidfd_send_signal(&fds[i], Signal::CONT)?;
                    return Err("fresh controller failed to obtain its own lease".into());
                }
                thread::sleep(Duration::from_millis(1));
            }
            thread::sleep(Duration::from_millis(25));
            let still_running = fresh.try_wait()?.is_none();
            pidfd_send_signal(&fds[i], Signal::CONT)?;
            need(
                still_running,
                "fresh controller advanced while the old owner was stopped",
            )?;
        }
        for (id, fd) in children.into_iter().zip(fds) {
            loop {
                if let Some((_, s)) = waitpid(Some(id), WaitOptions::NOHANG)? {
                    need(s.exit_status() == Some(0), "orphaned participant faulted")?;
                    break;
                }
                if Instant::now() > deadline {
                    let _ = pidfd_send_signal(fd, Signal::KILL);
                    let _ = waitpid(Some(id), WaitOptions::empty());
                    return Err("participant outlived killed coordinator".into());
                }
                thread::sleep(Duration::from_millis(2));
            }
        }
        let fresh = fresh.wait_with_output()?;
        need(
            fresh.status.success(),
            &format!(
                "failed fresh resume: {}",
                String::from_utf8_lossy(&fresh.stderr)
            ),
        )?;
        let first: serde_json::Value = serde_json::from_slice(&fresh.stdout)?;
        if stopped.is_some() {
            need(
                first["previous_owner_wait_ns"].as_u64().unwrap_or(0) >= 15_000_000,
                "positive old-owner fence was skipped",
            )?;
        }
        let repeat = Command::new(std::env::current_exe()?)
            .arg("resume")
            .arg(&case)
            .output()?;
        need(
            repeat.status.success(),
            &String::from_utf8_lossy(&repeat.stderr),
        )?;
        let repeated: serde_json::Value = serde_json::from_slice(&repeat.stdout)?;
        need(
            first["logical_tick"] == repeated["logical_tick"]
                && first["native_image_sha256"] == repeated["native_image_sha256"]
                && repeated["resolved_input"].is_null(),
            "repeated resume double-applied an input",
        )?;
        let expect = match stage {
            Stage::IntentStaged => u64::from(CUT + 16),
            Stage::Intent
            | Stage::Applied
            | Stage::ReceiptStaged
            | Stage::ReceiptLinked
            | Stage::Receipted => u64::from(CUT + 17),
            _ => u64::from(CUT),
        };
        let expect_pending = matches!(stage, Stage::Intent | Stage::Applied | Stage::ReceiptStaged);
        need(
            first["resolved_input"].as_u64() == expect_pending.then_some(u64::from(CUT + 17)),
            "wrong durable-input visibility at crash cut",
        )?;
        let staging_inodes = fs::read_dir(&case)?
            .filter_map(|f| f.ok())
            .filter(|f| f.file_name().to_string_lossy().ends_with(".pending"))
            .count();
        if matches!(
            stage,
            Stage::DecisionStaged
                | Stage::DecisionLinked
                | Stage::IntentStaged
                | Stage::ReceiptStaged
                | Stage::ReceiptLinked
        ) {
            need(
                staging_inodes == 1,
                "interrupted staging inode was unexpectedly published or missing",
            )?;
        }
        need(
            first["logical_tick"] == expect,
            "resolution crossed wrong committed cut",
        )?;
        reports.push(json!({"cut":marker,"reaped_workers":2,"interrupted_staging_inodes":staging_inodes,"stopped_owner_slot":stopped,"duplicate_controller":"refused","resume":first,"second_resume":repeated}));
    }
    run::output(&dir.join("results.json"), &reports)?;
    println!(
        "{} coordinator SIGKILL boundaries, {} actual participant processes reaped, every resume and repeat agrees with independent native evolution.",
        reports.len(),
        reports.len() * 2
    );
    Ok(())
}
#[cfg(not(target_os = "linux"))]
pub(super) fn proof(_: &Path) -> Result<()> {
    Err("coordinator process-death proof requires Linux pidfds and a child subreaper".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_surviving_owner_fences_timeout_and_allows_a_later_retry() -> Result<()> {
        let root = tempfile::tempdir()?;
        let path = root.path().join("decision.bin");
        let old =
            durable::try_lease(&path, "owner")?.ok_or("failed to create initial test lease")?;
        let result = await_owner_release(&path, Duration::from_millis(5));
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("recovery is fenced")
        );
        drop(old);
        assert_eq!(await_owner_release(&path, Duration::from_millis(5))?, 0);
        Ok(())
    }
}
