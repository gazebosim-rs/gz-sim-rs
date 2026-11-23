// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::wire;
use std::{
    io::{self, BufReader},
    path::Path,
    process::{Child as Process, Command, Stdio},
    sync::{
        Mutex,
        mpsc::{self, Receiver, SyncSender, TrySendError},
    },
    thread::{self, JoinHandle},
    time::Instant,
};
/// One blocking pipe reader/writer per child lets the Bevy owner dispatch a whole barrier
/// in parallel and enforce a wall deadline even when a worker stops reading a large request.
pub(super) struct Child {
    process: Process,
    commands: Option<SyncSender<Vec<u8>>>,
    replies: Mutex<Receiver<io::Result<Vec<u8>>>>,
    thread: Option<JoinHandle<()>>,
}
impl Child {
    pub(super) fn start(
        path: &Path,
        adapter: &str,
        rows: bool,
        until: Instant,
    ) -> io::Result<Self> {
        let mut process = Command::new(path)
            .arg(adapter)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let mut input = process.stdin.take().expect("piped child");
        let mut output = BufReader::new(process.stdout.take().expect("piped child"));
        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(1);
        let (reply, replies) = mpsc::channel();
        let thread = match thread::Builder::new()
            .name("gz-pair-worker-io".into())
            .spawn(move || {
                let ready = wire::handshake(&mut output).map(|_| Vec::new());
                let ok = ready.is_ok();
                if reply.send(ready).is_err() || !ok {
                    return;
                }
                for cmd in rx {
                    let out = wire::send(&mut input, &cmd).and_then(|_| {
                        wire::receive(&mut output)?.ok_or_else(|| {
                            io::Error::new(
                                io::ErrorKind::UnexpectedEof,
                                "GPU worker closed a pending reply",
                            )
                        })
                    });
                    let ok = out.is_ok();
                    if reply.send(out).is_err() || !ok {
                        break;
                    }
                }
            }) {
            Ok(t) => t,
            Err(e) => {
                let _ = process.kill();
                let _ = process.wait();
                return Err(e);
            }
        };
        let mut child = Self {
            process,
            commands: Some(tx),
            replies: Mutex::new(replies),
            thread: Some(thread),
        };
        child.receive(until)?;
        if rows {
            child.send(wire::ASK_ROWS.to_vec())?;
            wire::check_format_reply(&child.receive(until)?)?;
        }
        Ok(child)
    }
    pub(super) fn pid(&self) -> u32 {
        self.process.id()
    }
    pub(super) fn send(&self, b: Vec<u8>) -> io::Result<()> {
        self.commands
            .as_ref()
            .expect("live child")
            .try_send(b)
            .map_err(|e| match e {
                TrySendError::Full(_) => io::Error::new(
                    io::ErrorKind::WouldBlock,
                    "GPU worker already has a request",
                ),
                TrySendError::Disconnected(_) => {
                    io::Error::new(io::ErrorKind::BrokenPipe, "GPU worker is gone")
                }
            })
    }
    pub(super) fn receive(&mut self, until: Instant) -> io::Result<Vec<u8>> {
        self.replies
            .get_mut()
            .expect("exclusive worker")
            .recv_timeout(until.saturating_duration_since(Instant::now()))
            .map_err(|e| match e {
                mpsc::RecvTimeoutError::Timeout => {
                    io::Error::new(io::ErrorKind::TimedOut, "GPU worker deadline elapsed")
                }
                mpsc::RecvTimeoutError::Disconnected => {
                    io::Error::new(io::ErrorKind::BrokenPipe, "GPU worker is gone")
                }
            })?
    }
}
impl Drop for Child {
    fn drop(&mut self) {
        self.commands.take();
        let _ = self.process.kill();
        let _ = self.process.wait();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
