// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit local viewer ownership; no resources exist without --web-viewer.
mod snapshot;
mod socket;

type Latest = Mutex<(u64, Arc<String>)>;

use gz_sim_core::Simulation;
use gz_sim_proto::msgs::{WorldControl, WorldReset};
use gz_sim_server::Server;
use serde::{Deserialize, Serialize};
use std::{
    io,
    net::{Ipv4Addr, TcpListener},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Action {
    Pause,
    Resume,
    Reset,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Control {
    version: u8,
    kind: String,
    request_id: u64,
    action: Action,
}

impl Control {
    fn decode(text: &str, previous: u64) -> io::Result<Self> {
        let value: Self = serde_json::from_str(text).map_err(io::Error::other)?;
        if value.version != 1 || value.kind != "control" || value.request_id <= previous {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid control version, kind or request order",
            ));
        }
        Ok(value)
    }
    fn world_control(&self) -> WorldControl {
        match self.action {
            Action::Pause => WorldControl {
                pause: true,
                ..Default::default()
            },
            Action::Resume => WorldControl::default(),
            Action::Reset => WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            },
        }
    }
}

pub(super) struct Pending {
    control: Control,
    alive: Arc<AtomicBool>,
    reply: mpsc::SyncSender<String>,
}

#[derive(Serialize)]
struct Ack {
    version: u8,
    kind: &'static str,
    request_id: u64,
    ok: bool,
    epoch: u64,
    seconds: u64,
    nanoseconds: u32,
    paused: bool,
    error: Option<&'static str>,
}

pub(super) struct Bridge {
    latest: Arc<Latest>,
    connected: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    commands: mpsc::Receiver<Pending>,
    worker: Option<JoinHandle<()>>,
    sequence: u64,
    published: Instant,
    published_generation: u64,
}

impl Bridge {
    pub(super) fn new(sim: &Simulation, world: &str, port: u16) -> io::Result<Self> {
        Self::with_listener(sim, world, TcpListener::bind((Ipv4Addr::LOCALHOST, port))?)
    }

    fn with_listener(sim: &Simulation, world: &str, listener: TcpListener) -> io::Result<Self> {
        let initial = snapshot::encode(&snapshot::extract(sim, world, 0)?)?;
        let port = listener.local_addr()?.port();
        listener.set_nonblocking(true)?;
        let latest = Arc::new(Mutex::new((0, Arc::new(initial))));
        let stop = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicU64::new(0));
        let (sender, commands) = mpsc::sync_channel(8);
        let worker = {
            let (latest, stop, connected) = (latest.clone(), stop.clone(), connected.clone());
            thread::Builder::new()
                .name("gzweb-local".into())
                .spawn(move || socket::serve(listener, &latest, &stop, &connected, &sender))?
        };
        eprintln!(
            "gzweb-rs WebSocket: ws://127.0.0.1:{port}/ (viewer HTTP origin port 8080 or 8081)"
        );
        Ok(Self {
            latest,
            connected,
            stop,
            commands,
            worker: Some(worker),
            sequence: 0,
            published: Instant::now(),
            published_generation: 0,
        })
    }

    pub(super) fn controls(&mut self, server: &mut Server) -> Vec<(Pending, bool, u64)> {
        self.commands
            .try_iter()
            .take(8)
            .filter_map(|pending| {
                if !pending.alive.load(Ordering::Acquire) {
                    return None;
                }
                let before = server.simulation().epoch();
                let accepted = server.apply_local_control(pending.control.world_control());
                Some((pending, accepted, before))
            })
            .collect()
    }

    /// Called after prepare_io/tick, so an acknowledgement describes committed state.
    pub(super) fn publish(
        &mut self,
        sim: &Simulation,
        world: &str,
        controls: Vec<(Pending, bool, u64)>,
    ) -> io::Result<()> {
        if self.worker.as_ref().is_some_and(JoinHandle::is_finished) {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "gzweb network worker stopped",
            ));
        }
        let generation = self.connected.load(Ordering::Acquire);
        if controls.is_empty()
            && (generation == 0
                || (generation == self.published_generation
                    && self.published.elapsed() < Duration::from_millis(34)))
        {
            return Ok(());
        }
        self.sequence = self
            .sequence
            .checked_add(1)
            .ok_or_else(|| io::Error::other("gzweb sequence exhausted"))?;
        let scene = snapshot::extract(sim, world, self.sequence)?;
        let encoded = Arc::new(snapshot::encode(&scene)?);
        *self
            .latest
            .lock()
            .map_err(|_| io::Error::other("gzweb snapshot lock poisoned"))? = (generation, encoded);
        self.published = Instant::now();
        self.published_generation = generation;
        for (pending, accepted, before) in controls {
            let completed = match pending.control.action {
                Action::Pause => scene.paused,
                Action::Resume => !scene.paused,
                Action::Reset => {
                    before
                        .checked_add(1)
                        .is_some_and(|epoch| scene.epoch == epoch)
                        && scene.paused
                        && scene.seconds == 0
                        && scene.nanoseconds == 0
                }
            };
            let accepted = accepted && completed;
            let ack = Ack {
                version: 1,
                kind: "ack",
                request_id: pending.control.request_id,
                ok: accepted,
                epoch: scene.epoch,
                seconds: scene.seconds,
                nanoseconds: scene.nanoseconds,
                paused: scene.paused,
                error: (!accepted).then_some("control rejected by the current server state"),
            };
            let encoded = serde_json::to_string(&ack).map_err(io::Error::other)?;
            // A disconnected client cannot hold the simulation or block shutdown.
            let _ = pending.reply.try_send(encoded);
        }
        Ok(())
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/web.rs"]
mod tests;
