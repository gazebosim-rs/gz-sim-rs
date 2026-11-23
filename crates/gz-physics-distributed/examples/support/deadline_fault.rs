// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Suspend a particular local worker incarnation for a genuine transport deadline.
use super::{PoolError, Result, WorkerPool, pids};
use gz_physics_distributed::{physics::BodyWrench, transport::TransportError};
use serde::Serialize;
use std::time::{Duration, Instant};

#[cfg(target_os = "linux")]
pub struct Stopped(rustix::fd::OwnedFd);
#[cfg(target_os = "linux")]
impl Stopped {
    pub fn worker(pool: &WorkerPool, owner: u8) -> Result<Self> {
        use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
        let raw = pool.worker_pid(owner).ok_or("missing worker")?;
        let pid = Pid::from_raw(i32::try_from(raw)?).ok_or("invalid process ID")?;
        let fd = pidfd_open(pid, PidfdFlags::empty())?;
        pidfd_send_signal(&fd, Signal::STOP)?;
        let guard = Self(fd);
        let until = Instant::now() + Duration::from_secs(2);
        while !std::fs::read_to_string(format!("/proc/{raw}/status"))?
            .lines()
            .any(|l| l.starts_with("State:") && l.split_whitespace().nth(1) == Some("T"))
        {
            if Instant::now() >= until {
                return Err("worker did not stop".into());
            }
            std::thread::yield_now();
        }
        Ok(guard)
    }
}
#[cfg(target_os = "linux")]
impl Drop for Stopped {
    fn drop(&mut self) {
        // PIDFD cannot accidentally resume a different worker after a fenced restart.
        let _ = rustix::process::pidfd_send_signal(&self.0, rustix::process::Signal::CONT);
    }
}
#[cfg(not(target_os = "linux"))]
pub struct Stopped;
#[cfg(not(target_os = "linux"))]
impl Stopped {
    pub fn worker(_: &WorkerPool, _: u8) -> Result<Self> {
        Err("real deadline injection requires a Linux pidfd".into())
    }
}
#[derive(Serialize)]
pub struct Observation {
    pub before_pids: Vec<u32>,
    pub after_pids: Vec<u32>,
    pub phase: &'static str,
    pub owner: u8,
    pub tick: u64,
    pub timeout_us: f64,
    pub retry_us: f64,
    pub replacements: usize,
}
pub fn check_timeout<T>(
    result: std::result::Result<T, PoolError>,
    started: Instant,
) -> Result<f64> {
    match result {
        Err(PoolError::Transport(TransportError::Unavailable(_)))
            if started.elapsed() >= Duration::from_millis(800) =>
        {
            Ok(started.elapsed().as_secs_f64() * 1e6)
        }
        Err(e) => Err(format!(
            "expected a real one-second unavailable reply, got {e} after {:?}",
            started.elapsed()
        )
        .into()),
        Ok(_) => Err("suspended worker unexpectedly acknowledged the operation".into()),
    }
}
pub fn step(
    pool: &mut WorkerPool,
    width: u8,
    owner: u8,
    loads: &[BodyWrench],
    keep_stopped: bool,
) -> Result<Observation> {
    // Keep any automatic baseline capture outside the suspended interval.
    pool.compact()?;
    let before = pids(pool, width);
    let frame = pool.frame().clone();
    let mut stopped = Some(Stopped::worker(pool, owner)?);
    let at = Instant::now();
    let timeout_us = check_timeout(
        pool.advance_islands_in_field(super::fixture::GRAVITY, loads),
        at,
    )?;
    if pool.frame() != &frame {
        return Err("timed-out solve changed the public barrier".into());
    }
    if !keep_stopped {
        drop(stopped.take());
    }
    let retry = Instant::now();
    pool.advance_islands_in_field(super::fixture::GRAVITY, loads)?;
    let retry_us = retry.elapsed().as_secs_f64() * 1e6;
    drop(stopped);
    let after = pids(pool, width);
    let replacements = after.iter().zip(&before).filter(|(a, b)| a != b).count();
    if before[usize::from(owner)].eq(&after[usize::from(owner)]) == keep_stopped
        || (!keep_stopped && replacements != 0)
    {
        return Err(format!("unexpected fenced workers after deadline: {replacements}").into());
    }
    Ok(Observation {
        before_pids: before,
        after_pids: after,
        phase: if keep_stopped {
            "step_fence"
        } else {
            "step_thaw"
        },
        owner,
        tick: frame.tick + 1,
        timeout_us,
        retry_us,
        replacements,
    })
}
