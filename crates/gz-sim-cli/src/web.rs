// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit local viewer ownership; no resources exist without --web-viewer.
mod assets;
mod lifecycle;
mod scan;
mod snapshot;
mod socket;

type Latest = Mutex<(u64, Arc<String>, Option<Arc<String>>, Option<Arc<String>>)>;

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

enum Queued {
    Control(Pending),
    Lifecycle(lifecycle::Pending),
}
impl Queued {
    fn alive(&self) -> &AtomicBool {
        match self {
            Self::Control(pending) => &pending.alive,
            Self::Lifecycle(pending) => &pending.alive,
        }
    }
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
    assets: Option<assets::Assets>,
    latest: Arc<Latest>,
    scan_connected: Arc<AtomicBool>,
    connected: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    commands: mpsc::Receiver<Queued>,
    lifecycle: Option<lifecycle::Lifecycle>,
    worker: Option<JoinHandle<()>>,
    sequence: u64,
    published: Instant,
    published_generation: u64,
}

impl Bridge {
    pub(super) fn new(sim: &Simulation, world: &str, port: u16) -> io::Result<Self> {
        Self::with_listener(sim, world, TcpListener::bind((Ipv4Addr::LOCALHOST, port))?)
    }

    pub(super) fn new_with_lifecycle(
        sim: &Simulation,
        world: &str,
        port: u16,
        sdf: &sdformat::Sdf,
    ) -> io::Result<Self> {
        Self::with_options(
            sim,
            world,
            TcpListener::bind((Ipv4Addr::LOCALHOST, port))?,
            Some(sdf),
        )
    }

    fn with_listener(sim: &Simulation, world: &str, listener: TcpListener) -> io::Result<Self> {
        Self::with_options(sim, world, listener, None)
    }

    fn with_options(
        sim: &Simulation,
        world: &str,
        listener: TcpListener,
        sdf: Option<&sdformat::Sdf>,
    ) -> io::Result<Self> {
        let assets = assets::Assets::new(sim)?;
        let scene = match &assets {
            Some(assets) => snapshot::extract_with_assets(sim, world, 0, Some(assets))?,
            None => snapshot::extract(sim, world, 0)?,
        };
        let lifecycle = sdf
            .map(|sdf| lifecycle::Lifecycle::new(sim, sdf, world, &scene))
            .transpose()?;
        let catalog = lifecycle
            .as_ref()
            .map(|lifecycle| lifecycle.encode_catalog(sim, &scene).map(Arc::new))
            .transpose()?;
        let initial = snapshot::encode(&scene)?;
        let lifecycle_enabled = lifecycle.is_some();
        let port = listener.local_addr()?.port();
        listener.set_nonblocking(true)?;
        let latest = Arc::new(Mutex::new((0, Arc::new(initial), None, catalog)));
        let scan_connected = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicU64::new(0));
        let (sender, commands) = mpsc::sync_channel(8);
        let worker = {
            let (latest, stop, connected) = (latest.clone(), stop.clone(), connected.clone());
            let scan_connected = scan_connected.clone();
            thread::Builder::new()
                .name("gzweb-local".into())
                .spawn(move || {
                    socket::serve(
                        listener,
                        &latest,
                        &stop,
                        &connected,
                        &scan_connected,
                        &sender,
                        lifecycle_enabled,
                    )
                })?
        };
        eprintln!(
            "gzweb-rs WebSocket: ws://127.0.0.1:{port}/ (viewer HTTP origin port 8080 or 8081)"
        );
        Ok(Self {
            assets,
            latest,
            scan_connected,
            connected,
            stop,
            commands,
            lifecycle,
            worker: Some(worker),
            sequence: 0,
            published: Instant::now(),
            published_generation: 0,
        })
    }

    pub(super) fn scan_interested(&self) -> bool {
        self.scan_connected.load(Ordering::Acquire)
    }

    pub(super) fn controls(&mut self, server: &mut Server) -> Vec<(Pending, bool, u64)> {
        let mut controls = Vec::new();
        for command in self.commands.try_iter().take(8) {
            if !command.alive().load(Ordering::Acquire) {
                continue;
            }
            match command {
                Queued::Control(pending) => {
                    let before = server.simulation().epoch();
                    let accepted = server.apply_local_control(pending.control.world_control());
                    controls.push((pending, accepted, before));
                }
                Queued::Lifecycle(pending) => {
                    if let Some(lifecycle) = &mut self.lifecycle {
                        // The socket has one pending slot. Keep internal callers
                        // bounded as well; overflow drops the reply and closes it.
                        if lifecycle.pending.len() < 8 {
                            lifecycle.pending.push(pending);
                        }
                    }
                }
            }
        }
        controls
    }

    pub(super) fn apply_lifecycle(&mut self, server: &mut Server, world: &str) -> io::Result<()> {
        if let Some(lifecycle) = &mut self.lifecycle
            && !lifecycle.pending.is_empty()
        {
            let sequence = self
                .sequence
                .checked_add(1)
                .ok_or_else(|| io::Error::other("gzweb sequence exhausted"))?;
            lifecycle.apply(server, world, sequence, &mut self.assets)?;
        }
        Ok(())
    }

    /// Called after prepare_io/tick, so an acknowledgement describes committed state.
    pub(super) fn publish(
        &mut self,
        sim: &Simulation,
        world: &str,
        controls: Vec<(Pending, bool, u64)>,
    ) -> io::Result<()> {
        if self.worker.as_ref().is_some_and(JoinHandle::is_finished)
            || self.assets.as_ref().is_some_and(|assets| !assets.healthy())
        {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "gzweb network worker stopped",
            ));
        }
        let generation = self.connected.load(Ordering::Acquire);
        if controls.is_empty()
            && !self
                .lifecycle
                .as_ref()
                .is_some_and(lifecycle::Lifecycle::has_replies)
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
        let scene = match &self.assets {
            Some(assets) => snapshot::extract_with_assets(sim, world, self.sequence, Some(assets))?,
            None => snapshot::extract(sim, world, self.sequence)?,
        };
        if self.lifecycle.is_some()
            && let Some(assets) = &mut self.assets
        {
            assets.retain_aliases(&scene.visual_ids().collect());
        }
        let encoded = Arc::new(snapshot::encode(&scene)?);
        let measured_scan = if self.scan_interested() {
            Some(Arc::new(scan::encode(sim)?))
        } else {
            None
        };
        let catalog = self
            .lifecycle
            .as_ref()
            .map(|lifecycle| lifecycle.encode_catalog(sim, &scene).map(Arc::new))
            .transpose()?;
        // The network owner clones this pair under one lock: every scan follows
        // the exact committed Scene publication captured with it, including reset.
        *self
            .latest
            .lock()
            .map_err(|_| io::Error::other("gzweb snapshot lock poisoned"))? = (
            generation,
            encoded.clone(),
            measured_scan.clone(),
            catalog.clone(),
        );
        self.published = Instant::now();
        self.published_generation = generation;
        if let Some(lifecycle) = &mut self.lifecycle {
            lifecycle.send_replies(
                encoded,
                measured_scan,
                catalog.expect("configured lifecycle catalog"),
            );
        }
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

#[cfg(test)]
#[path = "../tests/private/web_scan.rs"]
mod scan_tests;

#[cfg(test)]
#[path = "../tests/private/web_lifecycle.rs"]
mod lifecycle_tests;
