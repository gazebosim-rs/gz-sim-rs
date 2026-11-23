// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Watch the SSH control channel independently of the process holding the slot lease.
use rustix::{
    event::{PollFd, PollFlags, poll},
    io::Errno,
    process::{Pid, PidfdFlags, Signal, getppid, pidfd_open, set_parent_process_death_signal},
};
use std::{
    io::{self, Read, Write},
    os::unix::process::ExitStatusExt,
    process::{Child, ChildStdout, Command, Stdio},
};

pub(super) const SOLVER: &str = "--managed-multisphere-ssh-solver-v1";

pub(super) fn arm(expected_parent: u32) -> io::Result<()> {
    set_parent_process_death_signal(Some(Signal::KILL))?;
    if getppid().map(|p| p.as_raw_nonzero().get() as u32) != Some(expected_parent) {
        return Err(io::Error::new(
            io::ErrorKind::BrokenPipe,
            "SSH supervisor exited before solver startup",
        ));
    }
    Ok(())
}

enum End {
    Disconnected,
    BeforeReady,
    SolverExited { admitted: bool },
}

fn before_ready() -> io::Error {
    io::Error::new(
        io::ErrorKind::UnexpectedEof,
        "forwarded slot closed before bootstrap readiness",
    )
}

fn watch(child: &Child, mut output: ChildStdout) -> io::Result<End> {
    let pid =
        Pid::from_raw(child.id() as i32).ok_or_else(|| io::Error::other("invalid solver PID"))?;
    let process = pidfd_open(pid, PidfdFlags::empty())?;
    let owner = io::stdin();
    // The unchanged ready payload contains a SHA-256 digest and a numeric
    // loopback locator. Only the solver consumes bootstrap bytes from stdin.
    let mut frame = [0u8; 4 + 256];
    let mut filled = 0;
    let mut target = 4;
    let mut admitted = false;
    loop {
        // Stop watching stdout once its sole frame has reached the caller.
        let mut fds = [
            PollFd::new(&owner, PollFlags::RDHUP),
            PollFd::new(&process, PollFlags::IN),
            PollFd::new(&output, PollFlags::IN),
        ];
        let count = if admitted { 2 } else { 3 };
        match poll(&mut fds[..count], None) {
            Err(Errno::INTR) => continue,
            result => result?,
        };
        if fds[..count]
            .iter()
            .any(|fd| fd.revents().contains(PollFlags::NVAL))
        {
            return Err(io::Error::other("invalid forwarded worker descriptor"));
        }
        if !fds[1].revents().is_empty() {
            return Ok(End::SolverExited { admitted });
        }
        // HUP covers a closed pipe; RDHUP also covers an SSH socket half-close.
        if !fds[0].revents().is_empty() {
            return Ok(if admitted {
                End::Disconnected
            } else {
                End::BeforeReady
            });
        }
        if !fds[2].revents().is_empty() {
            // Read only bytes presently available. A stopped child may have
            // written a partial header or payload; owner HUP must still win.
            let n = match output.read(&mut frame[filled..target]) {
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                result => result?,
            };
            if n == 0 {
                return Ok(End::BeforeReady);
            }
            filled += n;
            if filled == 4 && target == 4 {
                let length = u32::from_le_bytes(frame[..4].try_into().unwrap()) as usize;
                if length == 0 || length > frame.len() - 4 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "invalid forwarded ready frame length",
                    ));
                }
                target += length;
            } else if filled == target {
                let mut parent = io::stdout().lock();
                parent.write_all(&frame[..target])?;
                parent.flush()?;
                admitted = true;
            }
        }
    }
}

pub(super) fn supervise(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    // This single-threaded supervisor is also the stable parent of the solver.
    // Re-exec the same inode even if its deployment path has been replaced.
    let mut child = Command::new("/proc/self/exe")
        .arg(SOLVER)
        .args(args)
        .arg(std::process::id().to_string())
        .stdout(Stdio::piped())
        .spawn()?;
    let output = child.stdout.take().expect("piped solver stdout");
    let end = watch(&child, output);
    let killed = match &end {
        Ok(End::SolverExited { .. }) => Ok(()),
        _ => child.kill(),
    };
    let status = child.wait()?;
    killed?;
    match end? {
        End::BeforeReady => {
            // HUP may have beaten the solver's own argument validation. Keep
            // digest/slot refusals deterministic without hashing twice on admission.
            super::forwarded_slot(args)?;
            Err(before_ready().into())
        }
        End::SolverExited { .. } if !status.success() => {
            std::process::exit(
                status
                    .code()
                    .unwrap_or_else(|| 128 + status.signal().unwrap_or(1)),
            );
        }
        End::SolverExited { admitted: false } => Err(before_ready().into()),
        _ => Ok(()),
    }
}
