// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A worker-owned final backstop for an unusable reply channel.
use rustix::process::{Pid, Signal, kill_process};
use std::{
    io,
    sync::mpsc::{self, Receiver, SyncSender},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const FAILURE_GRACE: Duration = Duration::from_secs(8);

pub(super) struct ExitDeadline {
    target: Pid,
    trigger: Option<SyncSender<Instant>>,
    thread: Option<JoinHandle<()>>,
}

impl ExitDeadline {
    // Production passes its own PID. Tests retain ownership of a live child
    // through signaling, watcher join and reaping; they never target strangers.
    pub(super) fn new(target: Pid) -> io::Result<Self> {
        let (trigger, incoming) = mpsc::sync_channel(1);
        let thread = thread::Builder::new()
            .name("render-exit-deadline".into())
            .spawn(move || watch(incoming, target))?;
        Ok(Self {
            target,
            trigger: Some(trigger),
            thread: Some(thread),
        })
    }

    pub(super) fn arm(&self) {
        // A live guard retains this sender until its exclusive Drop begins.
        signal_deadline(
            self.trigger.as_ref().expect("live exit guard owns sender"),
            self.target,
            Instant::now() + FAILURE_GRACE,
        );
    }
}

fn signal_deadline(trigger: &SyncSender<Instant>, target: Pid, deadline: Instant) {
    if trigger.try_send(deadline).is_err() {
        // A closed or saturated handoff cannot silently leave exit unguarded.
        // Under the supported process permissions, self-SIGKILL never returns.
        let _ = kill_process(target, Signal::KILL);
    }
}

fn watch(incoming: Receiver<Instant>, target: Pid) {
    if let Ok(deadline) = incoming.recv() {
        thread::sleep(deadline.saturating_duration_since(Instant::now()));
        let _ = kill_process(target, Signal::KILL);
    }
}

impl Drop for ExitDeadline {
    fn drop(&mut self) {
        // Healthy exits cancel and join the parked watcher. Fatal reply exits
        // call process::exit instead, so neither this join nor GPU Drop runs.
        self.trigger.take();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/exit_deadline.rs"]
mod tests;
