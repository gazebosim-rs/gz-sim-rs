// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real forwarded worker lifetimes on Linux pipes and stream sockets.
#![cfg(target_os = "linux")]

use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use rustix::process::{
    Pid, PidfdFlags, Signal, WaitOptions, pidfd_open, pidfd_send_signal, set_child_subreaper,
    waitpid,
};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{self, Write},
    net::{Shutdown, TcpListener},
    os::{fd::OwnedFd, unix::fs::MetadataExt, unix::net::UnixStream},
    path::{Path, PathBuf},
    process::{Child, Command, ExitStatus, Stdio},
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");

fn until(label: &str, mut ready: impl FnMut() -> Result<bool>) -> Result<()> {
    let deadline = Instant::now() + Duration::from_secs(6);
    while !ready()? {
        if Instant::now() >= deadline {
            return Err(format!("timed out waiting for {label}").into());
        }
        thread::sleep(Duration::from_millis(5));
    }
    Ok(())
}

struct Workers {
    supervisor: Child,
    solver: Option<(Pid, OwnedFd)>,
    lease: PathBuf,
}

impl Drop for Workers {
    fn drop(&mut self) {
        // Cleanup on all failure paths, including one with a stopped child.
        if let Some((_, fd)) = &self.solver {
            let _ = pidfd_send_signal(fd, Signal::KILL);
        }
        let _ = self.supervisor.kill();
        let _ = self.supervisor.wait();
        if let Some((pid, _)) = &self.solver {
            let _ = waitpid(Some(*pid), WaitOptions::empty());
        }
    }
}

impl Workers {
    fn launch(dir: &Path, input: Stdio) -> Result<(Self, u16)> {
        // Adopt only orphaned descendants of this dedicated test binary. Each
        // fixture waits for its own PID when the supervisor dies before reaping.
        set_child_subreaper(Pid::from_raw(1))?;
        let lease = dir.join("slot.lock");
        let ready = dir.join("ready");
        let port = TcpListener::bind(("127.0.0.1", 0))?.local_addr()?.port();
        let mut hash = Sha256::new();
        io::copy(&mut File::open(WORKER)?, &mut hash)?;
        let supervisor = Command::new(WORKER)
            .args(["--managed-multisphere-ssh-v1", "0"])
            .arg(&lease)
            .args([
                port.to_string(),
                port.to_string(),
                format!("{:x}", hash.finalize()),
            ])
            .stdin(input)
            .stdout(File::create_new(&ready)?)
            .spawn()?;
        Ok((
            Self {
                supervisor,
                solver: None,
                lease,
            },
            port,
        ))
    }

    fn start(dir: &Path, input: Stdio, owner: &mut impl Write) -> Result<Self> {
        let (mut workers, port) = Self::launch(dir, input)?;
        let ready = dir.join("ready");

        let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
        let boot =
            bincode::serialize(&(*b"GZ2SWK02", 0_u8, [7_u8; 16], serde_json::to_vec(&spec)?))?;
        owner.write_all(&u32::try_from(8 + boot.len())?.to_le_bytes())?;
        owner.write_all(b"GZPZNH01")?;
        owner.write_all(&boot)?;
        owner.flush()?;
        until("the forwarded worker offer", || {
            if let Some(status) = workers.supervisor.try_wait()? {
                return Err(format!("supervisor exited before ready: {status}").into());
            }
            let frame = fs::read(&ready)?;
            let Some(header) = frame.first_chunk::<4>() else {
                return Ok(false);
            };
            if frame.len() - 4 != u32::from_le_bytes(*header) as usize {
                return Ok(false);
            }
            let (digest, endpoint): ([u8; 32], String) = bincode::deserialize(&frame[4..])?;
            assert_eq!(digest[..], Sha256::digest(&boot)[..]);
            assert_eq!(endpoint, format!("tcp/127.0.0.1:{port}"));
            Ok(true)
        })?;

        // The PID is the kernel's recorded owner of this exact inode,
        // independent of executable name or any private worker command line.
        let stat = fs::metadata(&workers.lease)?;
        let key = format!(
            "{:02x}:{:02x}:{}",
            rustix::fs::major(stat.dev()),
            rustix::fs::minor(stat.dev()),
            stat.ino()
        );
        let owner = fs::read_to_string("/proc/locks")?
            .lines()
            .find_map(|line| {
                let fields: Vec<_> = line.split_ascii_whitespace().collect();
                if let [_, "FLOCK", "ADVISORY", "WRITE", pid, inode, ..] = fields.as_slice()
                    && *inode == key
                {
                    return pid.parse::<i32>().ok().and_then(Pid::from_raw);
                }
                None
            })
            .ok_or("missing native solver lock owner")?;
        let pid = owner.as_raw_nonzero().get();
        let status = fs::read_to_string(format!("/proc/{pid}/status"))?;
        assert!(
            status
                .lines()
                .any(|s| s == format!("PPid:\t{}", workers.supervisor.id()))
        );
        workers.solver = Some((owner, pidfd_open(owner, PidfdFlags::empty())?));
        Ok(workers)
    }

    fn stop_solver(&self) -> Result<()> {
        let (pid, fd) = self.solver.as_ref().unwrap();
        pidfd_send_signal(fd, Signal::STOP)?;
        let status = format!("/proc/{}/status", pid.as_raw_nonzero());
        until("the remote solver to stop", || {
            Ok(fs::read_to_string(&status)?
                .lines()
                .any(|s| s.starts_with("State:\tT")))
        })?;
        Ok(())
    }

    fn stop_lease_holder(&self) -> Result<()> {
        self.stop_solver()?;
        assert!(matches!(
            File::open(&self.lease)?.try_lock(),
            Err(fs::TryLockError::WouldBlock)
        ));
        Ok(())
    }

    fn exit(&mut self) -> Result<ExitStatus> {
        until("supervisor exit", || {
            Ok(self.supervisor.try_wait()?.is_some())
        })?;
        Ok(self.supervisor.wait()?)
    }

    fn released(&self) -> Result<()> {
        let lease = File::open(&self.lease)?;
        until("remote lease release", || match lease.try_lock() {
            Ok(()) => Ok(true),
            Err(fs::TryLockError::WouldBlock) => Ok(false),
            Err(e) => Err(e.into()),
        })
    }

    fn assert_solver_reaped(&self) {
        // As a subreaper this test would own the solver (even as a zombie)
        // if the supervisor exited without collecting it.
        let (pid, _) = self.solver.as_ref().unwrap();
        assert!(matches!(
            waitpid(Some(*pid), WaitOptions::NOHANG),
            Err(rustix::io::Errno::CHILD)
        ));
    }
}

#[test]
fn stopped_solver_without_bootstrap_cannot_report_success_on_eof() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (input, owner) = io::pipe()?;
    let (mut workers, _) = Workers::launch(dir.path(), Stdio::from(input))?;
    let supervisor = workers.supervisor.id();
    let children = format!("/proc/{supervisor}/task/{supervisor}/children");
    until("the solver to exec before bootstrap", || {
        if let Some(status) = workers.supervisor.try_wait()? {
            return Err(format!("supervisor exited before spawning solver: {status}").into());
        }
        for child in fs::read_to_string(&children)?.split_ascii_whitespace() {
            let cmd = fs::read(format!("/proc/{child}/cmdline"))?;
            if cmd.split(|b| *b == 0).nth(1) == Some(b"--managed-multisphere-ssh-solver-v1") {
                let pid = Pid::from_raw(child.parse()?).ok_or("invalid solver PID")?;
                workers.solver = Some((pid, pidfd_open(pid, PidfdFlags::empty())?));
                return Ok(true);
            }
        }
        Ok(false)
    })?;
    workers.stop_solver()?;
    drop(owner);
    assert!(!workers.exit()?.success());
    workers.assert_solver_reaped();
    assert!(fs::read(dir.path().join("ready"))?.is_empty());
    assert!(!workers.lease.try_exists()?);
    Ok(())
}

#[test]
fn stopped_solver_releases_lease_on_pipe_eof() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (input, mut owner) = io::pipe()?;
    let mut workers = Workers::start(dir.path(), Stdio::from(input), &mut owner)?;
    workers.stop_lease_holder()?;
    drop(owner);
    assert!(workers.exit()?.success());
    workers.assert_solver_reaped();
    workers.released()
}

#[test]
fn stopped_solver_releases_lease_on_socket_half_close() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (input, mut owner) = UnixStream::pair()?;
    let mut workers = Workers::start(dir.path(), Stdio::from(OwnedFd::from(input)), &mut owner)?;
    workers.stop_lease_holder()?;
    owner.shutdown(Shutdown::Write)?;
    assert!(workers.exit()?.success());
    workers.assert_solver_reaped();
    workers.released()
}

#[test]
fn stopped_solver_dies_with_supervisor_while_owner_stays_connected() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (input, mut owner) = io::pipe()?;
    let mut workers = Workers::start(dir.path(), Stdio::from(input), &mut owner)?;
    workers.stop_lease_holder()?;
    workers.supervisor.kill()?;
    assert!(!workers.exit()?.success());
    workers.released()?;
    let (pid, _) = workers.solver.as_ref().unwrap();
    let (_, status) = waitpid(Some(*pid), WaitOptions::empty())?.ok_or("missing adopted solver")?;
    assert_eq!(status.terminating_signal(), Some(Signal::KILL.as_raw()));
    workers.solver.take();
    Ok(())
}

#[test]
fn solver_failure_ends_the_supervisor_while_owner_stays_connected() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (input, mut owner) = io::pipe()?;
    let mut workers = Workers::start(dir.path(), Stdio::from(input), &mut owner)?;
    pidfd_send_signal(&workers.solver.as_ref().unwrap().1, Signal::KILL)?;
    assert_eq!(workers.exit()?.code(), Some(128 + Signal::KILL.as_raw()));
    workers.assert_solver_reaped();
    workers.released()
}
