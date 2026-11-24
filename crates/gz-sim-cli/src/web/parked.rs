// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only presentation of a fenced physics cut and single-use operator admission.
use super::{Ack, CheckedStep, Control, snapshot::Scene, socket::parked_handshake};
use serde::{Deserialize, Serialize};
use std::{
    hash::{BuildHasher, RandomState},
    io,
    net::{Ipv4Addr, TcpListener, TcpStream},
    sync::atomic::{AtomicU64, Ordering},
    thread,
    time::{Duration, Instant, SystemTime},
};
use tungstenite::{Message, WebSocket};

const HEARTBEAT: Duration = Duration::from_secs(2);
const MAX_PEERS: usize = 8;

fn instance_token() -> io::Result<String> {
    static NEXT: AtomicU64 = AtomicU64::new(1);
    let serial = NEXT
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_add(1))
        .map_err(|_| io::Error::other("operator fence serial exhausted"))?;
    // The serial distinguishes re-parks at the same cut in this process. The
    // random process salt and clock distinguish replacement coordinators.
    let nonce = RandomState::new().hash_one((std::process::id(), SystemTime::now()));
    Ok(format!("{nonce:016x}{serial:016x}"))
}

#[derive(Serialize)]
struct Status {
    version: u8,
    kind: &'static str,
    epoch: u64,
    tick: u64,
    token: String,
    eligible: bool,
    reason: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Recover {
    version: u8,
    kind: String,
    epoch: u64,
    tick: u64,
    token: String,
    request_id: u64,
}

#[derive(Serialize)]
struct Reply {
    version: u8,
    kind: &'static str,
    request_id: u64,
    accepted: bool,
    error: Option<&'static str>,
}

struct Peer {
    socket: WebSocket<TcpStream>,
    offered: bool,
    request_id: u64,
    sequence: u64,
    sent_at: Instant,
}

enum Poll {
    Stay,
    Remove,
    Admitted,
}

fn send(socket: &mut WebSocket<TcpStream>, payload: &str) -> bool {
    socket.send(Message::Text(payload.into())).is_ok()
}

impl Peer {
    fn open(stream: TcpStream, fence: &ParkedFence) -> Option<Self> {
        let (mut socket, offered) = parked_handshake(stream)?;
        if offered && !send(&mut socket, &fence.status_frame)
            || !send(&mut socket, &fence.initial_scene)
        {
            return None;
        }
        Some(Self {
            socket,
            offered,
            request_id: 0,
            sent_at: Instant::now(),
            sequence: fence.scene.sequence,
        })
    }

    fn heartbeat(&mut self, fence: &ParkedFence) -> bool {
        if self.sent_at.elapsed() < HEARTBEAT {
            return true;
        }
        self.sent_at = Instant::now();
        if self.offered {
            return send(&mut self.socket, &fence.status_frame);
        }
        // Legacy viewers keep the ordinary scene heartbeat and an advancing
        // sequence. The physics clock and poses remain at the cut.
        let Some(sequence) = self.sequence.checked_add(1) else {
            return false;
        };
        self.sequence = sequence;
        let mut scene = fence.scene.clone();
        scene.sequence = sequence;
        serde_json::to_string(&scene).is_ok_and(|msg| send(&mut self.socket, &msg))
    }

    fn poll(&mut self, fence: &ParkedFence) -> Poll {
        if !self.heartbeat(fence) {
            return Poll::Remove;
        }
        let text = match self.socket.read() {
            Ok(Message::Text(text)) => text,
            Ok(Message::Ping(_) | Message::Pong(_)) => {
                return if self.socket.flush().is_ok() {
                    Poll::Stay
                } else {
                    Poll::Remove
                };
            }
            Err(tungstenite::Error::Io(e))
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                return Poll::Stay;
            }
            _ => return Poll::Remove,
        };
        let world_request = Control::decode(&text, self.request_id)
            .ok()
            .map(|control| control.request_id)
            .or_else(|| {
                if !self.offered {
                    return None;
                }
                let step: CheckedStep = serde_json::from_str(&text).ok()?;
                (step.version == 1
                    && step.kind == "operator_step"
                    && step.request_id > self.request_id)
                    .then_some(step.request_id)
            });
        if let Some(request_id) = world_request {
            self.request_id = request_id;
            let ack = Ack {
                version: 1,
                kind: "ack",
                request_id,
                ok: false,
                epoch: fence.scene.epoch,
                seconds: fence.scene.seconds,
                nanoseconds: fence.scene.nanoseconds,
                paused: fence.scene.paused,
                error: Some("Worker session fenced. Inspect the committed checkpoint."),
            };
            return if serde_json::to_string(&ack).is_ok_and(|msg| send(&mut self.socket, &msg)) {
                Poll::Stay
            } else {
                Poll::Remove
            };
        }
        if !self.offered {
            return Poll::Remove;
        }
        let Ok(request) = serde_json::from_str::<Recover>(&text) else {
            return Poll::Remove;
        };
        if request.version != 1 || request.kind != "recover_workers" || request.request_id == 0 {
            return Poll::Remove;
        }
        let status = &fence.status;
        let error = if request.request_id <= self.request_id {
            Some("Stale operator request ID")
        } else if request.token != status.token
            || request.epoch != status.epoch
            || request.tick != status.tick
        {
            Some("Operator request belongs to a different fence")
        } else if !status.eligible {
            Some("This checkpoint requires inspection")
        } else {
            None
        };
        self.request_id = self.request_id.max(request.request_id);
        let reply = Reply {
            version: 1,
            kind: "operator_reply",
            request_id: request.request_id,
            accepted: error.is_none(),
            error,
        };
        let sent = serde_json::to_string(&reply).is_ok_and(|msg| send(&mut self.socket, &msg));
        // Losing the reply cannot withdraw a valid request already read.
        if error.is_none() {
            Poll::Admitted
        } else if sent {
            Poll::Stay
        } else {
            Poll::Remove
        }
    }
}

/// Local endpoint held between a terminal fence and an explicit guarded reload.
/// This server presents an exact cached primitive scene and has no worker handles.
pub struct ParkedFence {
    listener: TcpListener,
    status: Status,
    status_frame: String,
    scene: Scene,
    initial_scene: String,
    admitted: bool,
}

impl ParkedFence {
    /// Bind the port after dropping the failed live server. Each new call
    /// creates a fresh token even when the physics epoch and tick repeat.
    pub fn bind(
        port: u16,
        scene: &str,
        epoch: u64,
        tick: u64,
        eligible: bool,
        reason: &str,
    ) -> io::Result<Self> {
        let scene = Scene::decode(scene, |_| false)?;
        let elapsed = Duration::from_millis(tick);
        if scene.epoch != epoch
            || scene.seconds != elapsed.as_secs()
            || scene.nanoseconds != elapsed.subsec_nanos()
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "parked scene differs from committed physics clock",
            ));
        }
        if reason.is_empty() || reason.len() > 1024 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid operator fence reason",
            ));
        }
        let status = Status {
            version: 1,
            kind: "operator_fence",
            epoch,
            tick,
            token: instance_token()?,
            eligible,
            reason: reason.into(),
        };
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port))?;
        listener.set_nonblocking(true)?;
        Ok(Self {
            listener,
            status_frame: serde_json::to_string(&status)?,
            status,
            initial_scene: serde_json::to_string(&scene)?,
            scene,
            admitted: false,
        })
    }

    /// Identify the accepted operator request to clients that reconnect to
    /// the restored live owner. Emit this notice only after guarded install.
    #[cfg(feature = "distributed-physics")]
    pub(crate) fn admitted_handoff(&self) -> io::Result<String> {
        if !self.admitted {
            return Err(io::Error::other("no admitted operator request"));
        }
        Ok(serde_json::json!({
            "version":1, "kind":"operator_restored", "epoch":self.status.epoch,
            "tick":self.status.tick, "token":self.status.token,
        })
        .to_string())
    }

    pub fn local_port(&self) -> io::Result<u16> {
        Ok(self.listener.local_addr()?.port())
    }

    /// Block until local termination (`false`) or one eligible request (`true`).
    /// The caller drops this listener and validates the checkpoint again under
    /// the writer lease before constructing a new paused simulation.
    pub fn wait(&mut self, stopped: impl Fn() -> bool) -> io::Result<bool> {
        if self.admitted {
            return Err(io::Error::other(
                "operator recovery request already admitted",
            ));
        }
        let mut peers = Vec::new();
        while !stopped() {
            match self.listener.accept() {
                Ok((stream, _)) if peers.len() < MAX_PEERS => {
                    if let Some(peer) = Peer::open(stream, self) {
                        peers.push(peer);
                    }
                }
                Ok(_) => (),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => (),
                Err(e) => return Err(e),
            }
            if stopped() {
                return Ok(false);
            }
            let mut i = 0;
            while i < peers.len() {
                match peers[i].poll(self) {
                    Poll::Stay => i += 1,
                    Poll::Remove => {
                        peers.swap_remove(i);
                    }
                    Poll::Admitted => {
                        self.admitted = true;
                        for peer in &mut peers {
                            let _ = peer.socket.close(None);
                        }
                        return Ok(true);
                    }
                }
            }
            thread::sleep(Duration::from_millis(2));
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests;
