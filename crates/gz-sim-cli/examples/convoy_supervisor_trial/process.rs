// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Args, Result, check, wire::Connection};
use std::{
    collections::BTreeSet,
    fs, io,
    os::unix::process::ExitStatusExt,
    path::Path,
    process::{Child, Command, Stdio},
    thread,
    time::{Duration, Instant},
};

pub struct Supervisor(Child);
impl Supervisor {
    pub fn start(
        a: &Args,
        session: &Path,
        package: &Path,
        sha: Option<&str>,
        label: &str,
    ) -> Result<Self> {
        let mut c = Command::new(a.release.join("examples/driven_hinge"));
        c.args(["--worker"])
            .arg(&a.worker)
            .arg("--session")
            .arg(session)
            .arg("--web-viewer")
            .arg(a.port.to_string());
        if let Some(digest) = sha {
            c.arg("--recover-package")
                .arg(package)
                .arg("--package-sha256")
                .arg(digest);
        } else {
            c.args([
                "--workers",
                "3",
                "--operator-recovery",
                "--reopen-attempts",
                "1",
                "--sdf",
            ])
            .arg(&a.sdf)
            .arg("--drive-program")
            .arg(&a.program)
            .arg("--checkpoint-directory")
            .arg(a.run.join("snapshots"))
            .arg("--checkpoint-package")
            .arg(package);
        }
        if a.zenoh {
            c.arg("--zenoh");
        }
        let log = fs::File::create_new(a.run.join(format!("{label}.log")))?;
        Ok(Self(
            c.stdin(Stdio::null())
                .stdout(log.try_clone()?)
                .stderr(log)
                .spawn()?,
        ))
    }

    pub fn connection(&mut self, port: u16) -> Result<Connection> {
        let start = Instant::now();
        loop {
            if let Some(exit) = self.0.try_wait()? {
                return Err(format!("supervisor {} exited {exit}", self.0.id()).into());
            }
            match Connection::open(port) {
                Ok(c) => return Ok(c),
                Err(e) if start.elapsed() >= Duration::from_secs(25) => return Err(e),
                Err(_) => thread::sleep(Duration::from_millis(5)),
            }
        }
    }
    pub fn snapshot(&self, a: &Args, previous: Option<&Path>) -> Result<std::path::PathBuf> {
        use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
        let fd = pidfd_open(
            Pid::from_raw(self.pid().try_into()?).ok_or("invalid supervisor pid")?,
            PidfdFlags::empty(),
        )?;
        pidfd_send_signal(fd, Signal::USR1)?;
        let until = Instant::now() + Duration::from_secs(5);
        loop {
            for file in fs::read_dir(a.run.join("snapshots"))? {
                let path = file?.path();
                if path.extension().is_some_and(|s| s == "gzcap") && previous != Some(&path) {
                    return Ok(path);
                }
            }
            check(Instant::now() < until, "missing immutable live snapshot")?;
            thread::sleep(Duration::from_millis(4));
        }
    }
    pub fn pid(&self) -> u32 {
        self.0.id()
    }
    pub fn workers(&self) -> Result<BTreeSet<u32>> {
        let mut children = BTreeSet::new();
        for task in fs::read_dir(format!("/proc/{}/task", self.pid()))? {
            match fs::read_to_string(task?.path().join("children")) {
                Ok(text) => {
                    for pid in text.split_whitespace() {
                        children.insert(pid.parse()?);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::NotFound => (),
                Err(e) => return Err(e.into()),
            }
        }
        Ok(children)
    }
    pub fn stop(&mut self) -> Result<()> {
        use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
        let fd = pidfd_open(Pid::from_child(&self.0), PidfdFlags::empty())?;
        pidfd_send_signal(fd, Signal::TERM)?;
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(status) = self.0.try_wait()? {
                return check(
                    status.success(),
                    format!("supervisor shutdown returned {status}"),
                );
            }
            check(
                Instant::now() < deadline,
                "supervisor did not acknowledge shutdown",
            )?;
            thread::sleep(Duration::from_millis(5));
        }
    }

    pub fn kill(&mut self) -> Result<()> {
        self.0.kill()?;
        check(
            self.0.wait()?.signal() == Some(9),
            "supervisor did not terminate on SIGKILL",
        )
    }
}
impl Drop for Supervisor {
    fn drop(&mut self) {
        if self.0.try_wait().is_ok_and(|s| s.is_none()) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

pub fn wait_workers_gone(pids: &BTreeSet<u32>) -> Result<()> {
    let until = Instant::now() + Duration::from_secs(15);
    loop {
        let mut live = Vec::new();
        for &pid in pids {
            match fs::read_to_string(format!("/proc/{pid}/stat")) {
                Ok(s)
                    if !s
                        .rsplit_once(") ")
                        .is_some_and(|(_, tail)| tail.starts_with("Z ")) =>
                {
                    live.push(pid)
                }
                Err(e) if e.kind() != io::ErrorKind::NotFound => return Err(e.into()),
                _ => (),
            }
        }
        if live.is_empty() {
            return Ok(());
        }
        if Instant::now() >= until {
            return Err(format!("surviving original worker transports {live:?}").into());
        }
        thread::sleep(Duration::from_millis(10));
    }
}

/// Retain a pidfd so cleanup always resumes the same process after the parent dies.
pub struct Suspended(std::os::fd::OwnedFd);
impl Drop for Suspended {
    fn drop(&mut self) {
        let _ = rustix::process::pidfd_send_signal(&self.0, rustix::process::Signal::CONT);
    }
}

/// Suspend the known active worker, then observe a real request queued on its
/// stdin without consuming a byte. The coordinator can no longer finish that RPC.
pub fn interrupt_pipe(
    workers: &BTreeSet<u32>,
    session: &Path,
    slot: u64,
) -> Result<(Suspended, u32, u64)> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    use std::os::unix::{ffi::OsStrExt, fs::FileTypeExt};
    let lease = fs::canonicalize(session)?.join(format!("worker-{slot}.lock"));
    for &pid in workers {
        let fd = pidfd_open(
            Pid::from_raw(pid.try_into()?).ok_or("invalid worker PID")?,
            PidfdFlags::empty(),
        )?;
        let cmd = fs::read(format!("/proc/{pid}/cmdline"))?;
        if !cmd
            .split(|&b| b == 0)
            .any(|arg| arg == lease.as_os_str().as_bytes())
        {
            continue;
        }
        let stdin = fs::File::open(format!("/proc/{pid}/fd/0"))?;
        check(
            stdin.metadata()?.file_type().is_fifo(),
            "worker input must be a pipe",
        )?;
        pidfd_send_signal(&fd, Signal::STOP)?;
        let stopped = Suspended(fd);
        let until = Instant::now() + Duration::from_secs(3);
        loop {
            let pending = rustix::io::ioctl_fionread(&stdin)?;
            if pending > 0 {
                return Ok((stopped, pid, pending));
            }
            check(
                Instant::now() < until,
                "no request reached the suspended active worker",
            )?;
            thread::sleep(Duration::from_millis(1));
        }
    }
    Err("could not identify trial's active worker lease".into())
}
