// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a caller's ordinary rollback/retry with confirmation in the same transaction.
use super::{PoolError, Result, WorkerPool, deadline, pids};
use clap::ValueEnum;
use serde::Serialize;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Policy {
    Recover,
    Confirm,
}

#[derive(Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Fault {
    Lost,
    Killed,
    Thaw,
    Stopped,
}
impl Fault {
    pub fn for_move(seed: u64, index: usize, suspended: bool) -> Self {
        let second = (seed / 48 + seed % 6 + index as u64) % 2 == 1;
        match (suspended, second) {
            (false, false) => Self::Lost,
            (false, true) => Self::Killed,
            (true, false) => Self::Thaw,
            (true, true) => Self::Stopped,
        }
    }
}

pub struct Transfer {
    pub member: u64,
    pub destination: u8,
    pub endpoint: u8,
    pub skip: usize,
    pub width: u8,
    pub grace_ms: u16,
    pub thaw_ms: u16,
}
#[derive(Serialize)]
pub struct Observation {
    pub policy: Policy,
    pub fault: Fault,
    pub endpoint: u8,
    pub before_pids: Vec<u32>,
    pub after_pids: Vec<u32>,
    pub elapsed_us: f64,
    pub grace_ms: u16,
    pub thaw_ms: u16,
    pub requests: u64,
    pub request_bytes: u64,
    pub reply_bytes: u64,
}
fn traffic(p: &WorkerPool) -> [u64; 3] {
    p.traffic().iter().fold([0; 3], |[n, a, b], c| {
        [n + c.requests, a + c.request_bytes, b + c.reply_bytes]
    })
}
pub fn run(p: &mut WorkerPool, x: Transfer, policy: Policy, fault: Fault) -> Result<Observation> {
    p.set_handoff_confirmation_window(
        (policy == Policy::Confirm).then_some(Duration::from_millis(u64::from(x.grace_ms))),
    )?;
    let before_pids = pids(p, x.width);
    let committed = p.frame().clone();
    let stopped = match fault {
        Fault::Lost => {
            p.drop_acknowledgement_after(x.endpoint, x.skip)?;
            None
        }
        Fault::Killed => {
            p.terminate_after_command(x.endpoint, x.skip)?;
            None
        }
        Fault::Thaw | Fault::Stopped => {
            // Baseline writes reach every peer. Complete them before suspension.
            p.compact()?;
            Some(deadline::Stopped::worker(p, x.endpoint)?)
        }
    };
    let before = traffic(p);
    let began = Instant::now();
    let mut stopped = stopped;
    let thaw = if fault == Fault::Thaw {
        let guard = stopped.take();
        Some(std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(1000 + u64::from(x.thaw_ms)));
            drop(guard);
        }))
    } else {
        None
    };
    let result = p.transfer_component(x.member, x.destination);
    match policy {
        Policy::Confirm => {
            result?;
        }
        Policy::Recover => {
            if !matches!(result, Err(PoolError::Transport(_))) || p.frame() != &committed {
                return Err("expected an unpublished migration after the failed receipt".into());
            }
            p.transfer_component(x.member, x.destination)?;
        }
    }
    let elapsed_us = began.elapsed().as_secs_f64() * 1e6;
    let after = traffic(p);
    drop(stopped);
    if let Some(thaw) = thaw {
        thaw.join().map_err(|_| "thaw thread failed")?;
    }
    let after_pids = pids(p, x.width);
    if policy == Policy::Confirm {
        for (i, (old, new)) in before_pids.iter().zip(&after_pids).enumerate() {
            let changed = old != new;
            let selected = i == usize::from(x.endpoint);
            let valid = match fault {
                Fault::Killed | Fault::Stopped => changed == selected,
                Fault::Lost => !changed,
                // A resumption at the retry boundary can race the transport deadline.
                Fault::Thaw => !changed || selected,
            };
            if !valid {
                return Err("unexpected migration recovery scope".into());
            }
        }
    }
    if matches!(fault, Fault::Thaw | Fault::Stopped) && elapsed_us < 800_000. {
        return Err("suspended migration finished before the first real deadline".into());
    }
    Ok(Observation {
        policy,
        fault,
        endpoint: x.endpoint,
        before_pids,
        after_pids,
        elapsed_us,
        grace_ms: x.grace_ms,
        thaw_ms: x.thaw_ms,
        requests: after[0] - before[0],
        request_bytes: after[1] - before[1],
        reply_bytes: after[2] - before[2],
    })
}
