// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Linux qualification for coordinator death during paired cold/restore bootstrap.
//! The shim only stops its own direct parent, records its slot, then execs the real worker.
#[cfg(target_os = "linux")]
mod linux {
    use clap::Parser;
    use rustix::process::{
        Pid, PidfdFlags, Signal, WaitOptions, getppid, kill_process, pidfd_open, pidfd_send_signal,
        set_child_subreaper, waitpid,
    };
    use serde::Serialize;
    use sha2::{Digest, Sha256};
    use std::{
        collections::BTreeSet,
        env,
        error::Error,
        fs,
        os::unix::process::{CommandExt, ExitStatusExt},
        path::{Path, PathBuf},
        process::{Child, Command, Stdio},
        thread,
        time::{Duration, Instant},
    };
    type Result<T> = std::result::Result<T, Box<dyn Error>>;
    #[derive(Parser)]
    struct Args {
        #[arg(long)]
        cli: PathBuf,
        #[arg(long)]
        worker: PathBuf,
        #[arg(long)]
        sdf: PathBuf,
        output: PathBuf,
    }
    struct Guard(Child);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    fn shim() -> Result<()> {
        let arguments = env::args_os().skip(1).collect::<Vec<_>>();
        let slot = arguments
            .windows(2)
            .find(|pair| pair[0] == "--worker")
            .and_then(|pair| pair[1].to_str())
            .ok_or("bad worker index")?
            .parse::<u8>()?;
        let dir = PathBuf::from(env::var_os("GZ_AUDIT_BOOT_SLOTS").ok_or("missing slots")?);
        let target = env::var("GZ_AUDIT_BOOT_STOP_SLOT")?.parse::<u8>()?;
        if slot == target {
            kill_process(getppid().ok_or("coordinator exited")?, Signal::STOP)?;
        }
        let pending = dir.join(format!(".slot-{slot}"));
        fs::write(&pending, std::process::id().to_string())?;
        fs::rename(pending, dir.join(slot.to_string()))?;
        Err(
            Command::new(env::var_os("GZ_AUDIT_BOOT_WORKER").ok_or("missing worker")?)
                .args(arguments)
                .exec()
                .into(),
        )
    }
    fn command(a: &Args, worker: &Path, width: u8) -> Command {
        let mut c = Command::new(&a.cli);
        c.arg(&a.sdf)
            .args([
                "--physics-engine",
                "distributed",
                "--physics-workers",
                &width.to_string(),
                "--adaptive-contacts",
                "--physics-worker",
            ])
            .arg(worker);
        c
    }
    fn children(pid: u32) -> Result<BTreeSet<i32>> {
        let mut ids = BTreeSet::new();
        for task in fs::read_dir(format!("/proc/{pid}/task"))? {
            let path = task?.path().join("children");
            if let Ok(ids_text) = fs::read_to_string(path) {
                for child in ids_text.split_ascii_whitespace() {
                    ids.insert(child.parse()?);
                }
            }
        }
        Ok(ids)
    }
    fn wait_for(end: Instant, mut condition: impl FnMut() -> bool) -> Result<()> {
        loop {
            if condition() {
                return Ok(());
            }
            if Instant::now() > end {
                return Err("actual child startup or exit deadline".into());
            }
            thread::sleep(Duration::from_millis(1));
        }
    }
    #[derive(Serialize)]
    struct Cut {
        width: u8,
        mode: &'static str,
        slot: u8,
        coordinator: u32,
        children: Vec<i32>,
        exited_normally: usize,
        exited_with_error: usize,
    }
    fn interrupt(
        a: &Args,
        real: &Path,
        width: u8,
        slot: u8,
        archive: Option<&Path>,
    ) -> Result<Cut> {
        let dir = tempfile::tempdir()?;
        let marker = dir.path().join(slot.to_string());
        let mut c = command(a, &env::current_exe()?, width);
        c.env("GZ_AUDIT_BOOT_SLOTS", dir.path())
            .env("GZ_AUDIT_BOOT_WORKER", real)
            .env("GZ_AUDIT_BOOT_STOP_SLOT", slot.to_string())
            .arg("--iterations")
            .arg("32")
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        if let Some(archive) = archive {
            c.arg("--resume-pool-checkpoint").arg(archive);
        }
        let mut owner = Guard(c.spawn()?);
        let pid = owner.0.id();
        let pfd = pidfd_open(Pid::from_child(&owner.0), PidfdFlags::empty())?;
        let end = Instant::now() + Duration::from_secs(12);
        wait_for(end, || {
            let stopped = fs::read_to_string(format!("/proc/{pid}/status")).is_ok_and(|s| {
                s.lines()
                    .any(|l| l.starts_with("State:") && l.contains("T (stopped)"))
            });
            let real_child = fs::read_to_string(&marker)
                .is_ok_and(|s| fs::read_link(format!("/proc/{s}/exe")).is_ok_and(|e| e == real));
            stopped && real_child
        })?;
        let live = children(pid)?;
        let lower = usize::from(slot / 2) * 2 + 1;
        let upper = (usize::from(slot / 2 + 1) * 2).min(usize::from(width));
        if !(lower..=upper).contains(&live.len()) {
            return Err(format!("out-of-pair launch at slot {slot}: {live:?}").into());
        }
        let children = live.into_iter().collect::<Vec<_>>();
        let handles = children
            .iter()
            .map(|&id| {
                Ok((
                    Pid::from_raw(id).ok_or("invalid pid")?,
                    pidfd_open(Pid::from_raw(id).unwrap(), PidfdFlags::empty())?,
                ))
            })
            .collect::<Result<Vec<_>>>()?;
        pidfd_send_signal(&pfd, Signal::KILL)?;
        assert_eq!(owner.0.wait()?.signal(), Some(9));
        let mut exited_normally = 0;
        let mut exited_with_error = 0;
        for (child, fd) in handles {
            loop {
                if let Some((_, status)) = waitpid(Some(child), WaitOptions::NOHANG)? {
                    if status.exit_status() == Some(0) {
                        exited_normally += 1;
                    } else {
                        exited_with_error += 1;
                    }
                    break;
                }
                if Instant::now() > end {
                    let _ = pidfd_send_signal(fd, Signal::KILL);
                    let _ = waitpid(Some(child), WaitOptions::empty());
                    return Err(format!(
                        "real managed child {} outlived dead coordinator",
                        child.as_raw_nonzero()
                    )
                    .into());
                }
                thread::sleep(Duration::from_millis(1));
            }
            assert!(!Path::new(&format!("/proc/{}", child.as_raw_nonzero())).exists());
        }
        Ok(Cut {
            width,
            mode: if archive.is_some() { "restore" } else { "cold" },
            slot,
            coordinator: pid,
            children,
            exited_normally,
            exited_with_error,
        })
    }
    pub fn run() -> Result<()> {
        if env::args_os().nth(1).is_some_and(|a| a == "--run") {
            return shim();
        }
        let a = Args::parse();
        let real = a.worker.canonicalize()?;
        set_child_subreaper(Pid::from_raw(1))?;
        let scratch = tempfile::tempdir()?;
        let mut results = Vec::new();
        for width in [3, 4, 5, 6, 7, 8] {
            let archive = scratch.path().join(format!("warm-{width}"));
            let seed = command(&a, &real, width)
                .args(["--checkpoint-at", "20", "--save-pool-checkpoint"])
                .arg(&archive)
                .args(["--iterations", "0"])
                .output()?;
            if !seed.status.success() {
                return Err(String::from_utf8_lossy(&seed.stderr).into_owned().into());
            }
            let original = Sha256::digest(fs::read(&archive)?);
            let suffix = || -> Result<Vec<u8>> {
                let output = command(&a, &real, width)
                    .arg("--resume-pool-checkpoint")
                    .arg(&archive)
                    .args(["--iterations", "17"])
                    .output()?;
                if !output.status.success() {
                    return Err(String::from_utf8_lossy(&output.stderr).into_owned().into());
                }
                Ok(output.stdout)
            };
            let reference = suffix()?;
            for source in [None, Some(archive.as_path())] {
                for slot in BTreeSet::from([0, width / 2, width - 1]) {
                    results.push(interrupt(&a, &real, width, slot, source)?);
                }
            }
            assert_eq!(Sha256::digest(fs::read(&archive)?), original);
            assert_eq!(suffix()?, reference);
        }
        let reaped = results.iter().map(|c| c.children.len()).sum::<usize>();
        fs::write(a.output, serde_json::to_vec_pretty(&results)?)?;
        println!(
            "{} hard coordinator deaths during native cold/image bootstrap, {reaped} observed worker children exited and were reaped; six immutable archives resumed to identical 17-tick CLI output.",
            results.len()
        );
        Ok(())
    }
}
#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    linux::run()
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("This hard-death qualification requires Linux procfs and pidfd.");
}
