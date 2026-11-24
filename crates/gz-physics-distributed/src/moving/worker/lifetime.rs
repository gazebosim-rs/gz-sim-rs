// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep Linux parent-death signals attached to one thread for the controller's full lifetime.
//! Solver requests and blocking I/O remain on their normal group/exchange threads.
use std::{
    io,
    process::{Child, Command},
};
#[cfg(target_os = "linux")]
const OWNER: &str = "GZ_TRAJECTORY_LAUNCH_OWNER";
#[cfg(target_os = "linux")]
pub(crate) fn spawn(mut cmd: Command) -> io::Result<Child> {
    use std::sync::{
        Mutex,
        mpsc::{SyncSender, sync_channel},
    };
    type Launch = (Command, SyncSender<io::Result<Child>>);
    static LAUNCH: Mutex<Option<SyncSender<Launch>>> = Mutex::new(None);
    let sender = {
        let mut owner = LAUNCH
            .lock()
            .map_err(|_| io::Error::other("trajectory launcher lock is poisoned"))?;
        if owner.is_none() {
            let (tx, rx) = sync_channel::<Launch>(8);
            std::thread::Builder::new()
                .name("trajectory-launcher".into())
                .spawn(move || {
                    for (mut command, reply) in rx {
                        if let Err(e) = reply.send(command.spawn())
                            && let Ok(mut abandoned) = e.0
                        {
                            let _ = abandoned.kill();
                            let _ = abandoned.wait();
                        }
                    }
                })?;
            *owner = Some(tx);
        }
        owner.as_ref().unwrap().clone()
    };
    cmd.env(OWNER, std::process::id().to_string());
    let (reply, child) = sync_channel(1);
    sender
        .send((cmd, reply))
        .map_err(|_| io::Error::other("trajectory launcher retired"))?;
    child
        .recv()
        .map_err(|_| io::Error::other("trajectory launcher did not return its child"))?
}
#[cfg(not(target_os = "linux"))]
pub(crate) fn spawn(mut cmd: Command) -> io::Result<Child> {
    cmd.spawn()
}
/// The PID check after installing the signal also covers controller death before process boot.
#[cfg(target_os = "linux")]
pub(crate) fn arm() -> io::Result<()> {
    use rustix::process::{Signal, getppid, set_parent_process_death_signal};
    let Some(parent) = std::env::var_os(OWNER) else {
        return Ok(());
    };
    let id = parent
        .to_str()
        .and_then(|v| v.parse::<i32>().ok())
        .filter(|v| *v > 0)
        .ok_or_else(|| io::Error::other("invalid trajectory launch owner"))?;
    set_parent_process_death_signal(Some(Signal::KILL))?;
    if getppid().map(|p| p.as_raw_nonzero().get()) != Some(id) {
        return Err(io::Error::other(
            "trajectory launch owner retired before boot",
        ));
    }
    Ok(())
}
#[cfg(not(target_os = "linux"))]
pub(crate) fn arm() -> io::Result<()> {
    Ok(())
}
