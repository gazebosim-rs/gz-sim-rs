// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit local viewer ownership; no resources exist without --web-viewer.
mod assets;
mod lifecycle;
pub(crate) mod parked;
pub(crate) mod recording;
pub(crate) mod recovery;
use recovery::Recovery;
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum Admission {
    CheckedStep { epoch: u64, tick: u64 },
    Boundary,
    AppliedResume,
    RecoveryPause,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CheckedStep {
    version: u8,
    kind: String,
    request_id: u64,
    epoch: u64,
    tick: u64,
}

impl CheckedStep {
    fn into_command(self) -> (Control, Admission) {
        // A checked step requests a paused final mode. Its admission carries
        // the extra interval and must match the authoritative simulation clock.
        (
            Control {
                version: self.version,
                kind: self.kind,
                request_id: self.request_id,
                action: Action::Pause,
            },
            Admission::CheckedStep {
                epoch: self.epoch,
                tick: self.tick,
            },
        )
    }
}

pub(super) struct Pending {
    admission: Admission,
    trace: Option<String>,
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

pub(crate) use snapshot::ConvoyView;

// Compare every wire-visible field except the per-publication sequence.
#[derive(PartialEq)]
struct PausedView {
    scene: snapshot::Scene,
    scan: Option<Arc<String>>,
    catalog: Option<lifecycle::Catalog>,
}

pub(super) struct Bridge {
    recovery: Option<Arc<Recovery>>,
    paused_view: Option<PausedView>,
    last_network_publication: Instant,
    credit_connected: Arc<AtomicBool>,
    assets: Option<assets::Assets>,
    recording: Option<recording::Capture>,
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

fn next_scene_sequence(current: u64) -> io::Result<u64> {
    current
        .checked_add(1)
        .ok_or_else(|| io::Error::other("gzweb sequence exhausted"))
}

impl Bridge {
    pub(super) fn new(
        sim: &Simulation,
        world: &str,
        port: u16,
        recovery: Option<Arc<Recovery>>,
    ) -> io::Result<Self> {
        Self::with_recording_options(
            sim,
            world,
            TcpListener::bind((Ipv4Addr::LOCALHOST, port))?,
            None,
            None,
            recovery,
        )
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

    pub(super) fn new_recording(
        sim: &Simulation,
        world: &str,
        port: u16,
        sdf: &sdformat::Sdf,
        path: &std::path::Path,
    ) -> io::Result<Self> {
        Self::with_recording_options(
            sim,
            world,
            TcpListener::bind((Ipv4Addr::LOCALHOST, port))?,
            Some(sdf),
            Some(path),
            None,
        )
    }

    #[cfg(test)]
    fn with_listener(sim: &Simulation, world: &str, listener: TcpListener) -> io::Result<Self> {
        Self::with_options(sim, world, listener, None)
    }

    fn with_options(
        sim: &Simulation,
        world: &str,
        listener: TcpListener,
        sdf: Option<&sdformat::Sdf>,
    ) -> io::Result<Self> {
        Self::with_recording_options(sim, world, listener, sdf, None, None)
    }

    fn with_recording_options(
        sim: &Simulation,
        world: &str,
        listener: TcpListener,
        sdf: Option<&sdformat::Sdf>,
        recording_path: Option<&std::path::Path>,
        recovery: Option<Arc<Recovery>>,
    ) -> io::Result<Self> {
        if recording_path.is_some() && (sdf.is_none() || world != "default") {
            return Err(io::Error::other(
                "recording requires the default lifecycle world",
            ));
        }
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
        let captured = recording_path
            .and(assets.as_ref())
            .map(assets::Assets::snapshot);
        let recording = recording_path
            .map(|path| {
                recording::Capture::start(
                    path,
                    captured.as_ref(),
                    &scene,
                    catalog
                        .as_ref()
                        .expect("recording requires lifecycle")
                        .as_str(),
                )
            })
            .transpose()?;
        let recording_enabled = recording.is_some();
        let initial = snapshot::encode(&scene)?;
        let lifecycle_enabled = lifecycle.is_some();
        let port = listener.local_addr()?.port();
        listener.set_nonblocking(true)?;
        let latest = Arc::new(Mutex::new((0, Arc::new(initial), None, catalog)));
        let scan_connected = Arc::new(AtomicBool::new(false));
        let credit_connected = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicU64::new(0));
        let (sender, commands) = mpsc::sync_channel(8);
        let worker = {
            let (latest, stop, connected) = (latest.clone(), stop.clone(), connected.clone());
            let scan_connected = scan_connected.clone();
            let credit_connected = credit_connected.clone();
            let recovery = recovery.clone();
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
                        socket::Modes {
                            recovery,
                            lifecycle: lifecycle_enabled,
                            recording: recording_enabled,
                            credit_connected,
                        },
                    )
                })?
        };
        eprintln!(
            "gzweb-rs WebSocket: ws://127.0.0.1:{port}/ (viewer HTTP origin port 8080, 8081 or 8082)"
        );
        Ok(Self {
            recovery,
            paused_view: None,
            last_network_publication: Instant::now(),
            credit_connected,
            assets,
            recording,
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

    pub(super) fn finish_recording(&mut self, sim: &Simulation, world: &str) -> io::Result<()> {
        if let Some(recording) = self.recording.take() {
            // Display updates are throttled; the final committed state may be newer.
            let sequence = next_scene_sequence(self.sequence)?;
            let scene = snapshot::extract_with_assets(sim, world, sequence, self.assets.as_ref())?;
            let catalog = self
                .lifecycle
                .as_ref()
                .expect("recording requires lifecycle")
                .catalog(sim, &scene)?;
            return recording.finish(&scene, &catalog);
        }
        Ok(())
    }

    pub(super) fn scan_interested(&self) -> bool {
        self.scan_connected.load(Ordering::Acquire)
    }

    pub(super) fn controls(&mut self, server: &mut Server) -> Vec<(Pending, bool, u64)> {
        let mut controls = Vec::new();
        for command in self.commands.try_iter().take(8) {
            let sticky =
                matches!(&command, Queued::Control(p) if p.admission == Admission::RecoveryPause);
            if !sticky && !command.alive().load(Ordering::Acquire) {
                continue;
            }
            match command {
                Queued::Control(pending) => {
                    let before = server.simulation().epoch();
                    let checked_step = matches!(pending.admission, Admission::CheckedStep { .. });
                    let mut control = pending.control.world_control();
                    let admitted = match pending.admission {
                        Admission::CheckedStep { epoch, tick } => {
                            control.step = true;
                            let sim = server.simulation();
                            controls.is_empty()
                                && tick.checked_add(1).is_some()
                                && server.pending_steps() == 0
                                && sim.is_paused()
                                && sim.epoch() == epoch
                                && sim.step_duration() == Duration::from_millis(1)
                                && sim.elapsed() == Duration::from_millis(tick)
                        }
                        _ => true,
                    };
                    let accepted = admitted && server.apply_local_control(control);
                    if sticky && let Some(recovery) = &self.recovery {
                        recovery.pause_pending.store(false, Ordering::Release);
                    }
                    controls.push((pending, accepted, before));
                    // Apply an admitted Pause at its own boundary even if a
                    // reconnect enqueued another control before this drain.
                    if sticky || checked_step {
                        break;
                    }
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
        // Resets may enter worker recovery from prepare_io while paused.
        // Rejected commands carry this actual mode, including at a terminal fence.
        if let Some(recovery) = &self.recovery {
            recovery
                .step_paused
                .store(server.simulation().is_paused(), Ordering::Release);
        }
        controls
    }

    /// Resume has already changed the server pause flag. Publish its actual
    /// clock now so this connection can queue Pause during the first long step.
    pub(super) fn publish_applied_resume(
        &mut self,
        sim: &Simulation,
        world: &str,
        controls: &mut Vec<(Pending, bool, u64)>,
    ) -> io::Result<()> {
        if matches!(controls.as_slice(), [(p, true, _)] if p.admission == Admission::AppliedResume)
        {
            self.publish(sim, world, std::mem::take(controls))?;
        }
        Ok(())
    }

    pub(super) fn flush_terminal_recovery(&self) {
        let Some(recovery) = &self.recovery else {
            return;
        };
        let generation = self.connected.load(Ordering::Acquire);
        if generation == 0 || !recovery.status().is_some_and(|s| s.phase == "fenced") {
            return;
        }
        // Preserve the runtime error. Allow the independent socket worker to
        // deliver the terminal state and any pending refusal before teardown.
        let start = Instant::now();
        while recovery.terminal_sent.load(Ordering::Acquire) != generation
            && self.connected.load(Ordering::Acquire) == generation
            && start.elapsed() < Duration::from_millis(300)
        {
            thread::sleep(Duration::from_millis(2));
        }
    }

    pub(super) fn apply_lifecycle(&mut self, server: &mut Server, world: &str) -> io::Result<()> {
        if let Some(lifecycle) = &mut self.lifecycle
            && !lifecycle.pending.is_empty()
        {
            let sequence = next_scene_sequence(self.sequence)?;
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
            && ((generation == 0 && self.recording.is_none())
                || (generation == self.published_generation
                    && self.published.elapsed() < Duration::from_millis(34)))
        {
            return Ok(());
        }
        self.sequence = next_scene_sequence(self.sequence)?;
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
        let catalog_view = self
            .lifecycle
            .as_ref()
            .map(|lifecycle| lifecycle.catalog(sim, &scene))
            .transpose()?;
        let catalog = catalog_view
            .as_ref()
            .map(|view| view.encode().map(Arc::new))
            .transpose()?;
        let refresh = !controls.is_empty()
            || self
                .lifecycle
                .as_ref()
                .is_some_and(lifecycle::Lifecycle::has_replies)
            || generation != self.published_generation
            || self.last_network_publication.elapsed() >= Duration::from_secs(2);
        let mut outcomes = if self.recording.is_some() {
            Some(
                self.lifecycle
                    .as_ref()
                    .expect("recording lifecycle owner")
                    .recorded_outcomes()?,
            )
        } else {
            None
        };
        let mut control_replies = Vec::with_capacity(controls.len());
        for (mut pending, accepted, before) in controls {
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
            let advanced = match pending.admission {
                Admission::CheckedStep { epoch, tick } => {
                    scene.epoch == epoch
                        && tick.checked_add(1).is_some_and(|end| {
                            Duration::from_millis(end)
                                == Duration::new(scene.seconds, scene.nanoseconds)
                        })
                }
                _ => true,
            };
            let other_run_mode = accepted && advanced && !scene.paused
                && matches!(pending.admission, Admission::CheckedStep { .. });
            let accepted = accepted && completed && advanced;
            let ack = Ack {
                version: 1,
                kind: "ack",
                request_id: pending.control.request_id,
                ok: accepted,
                epoch: scene.epoch,
                seconds: scene.seconds,
                nanoseconds: scene.nanoseconds,
                paused: scene.paused,
                error: if other_run_mode {
                    Some("Requested interval committed, but another control changed the run mode.")
                } else {
                    (!accepted).then_some("control rejected by the current server state")
                },
            };
            let encoded = serde_json::to_string(&ack).map_err(io::Error::other)?;
            if let Some(outcomes) = &mut outcomes {
                let trace = pending.trace.take().ok_or_else(|| {
                    io::Error::other("enabled recording lost world control provenance")
                })?;
                outcomes.push(recording::world_outcome(
                    &pending.control.action,
                    trace,
                    encoded.clone(),
                    accepted,
                ));
            }
            control_replies.push((pending, encoded));
        }
        // Capture the exact committed outcome before Latest coalescing and
        // before any successful control ACK is exposed to the socket worker.
        if let Some(recording) = &mut self.recording {
            recording.record(
                &scene,
                catalog.as_ref().expect("recording catalog").as_str(),
                outcomes.expect("enabled capture outcomes"),
            )?;
        }
        // An uncredited viewer can be busy initializing its renderer for seconds.
        // Keep recordings and acknowledgements on their original cadence. On this
        // socket, send visible changes and a two-second heartbeat so the viewer
        // stays inside its ten-second idle limit. New peers get the current tuple.
        let paused_view =
            (scene.paused && generation != 0 && !self.credit_connected.load(Ordering::Acquire))
                .then(|| {
                    let mut scene = scene;
                    scene.sequence = 0;
                    let catalog = catalog_view.map(|mut view| {
                        view.scene_sequence = 0;
                        view
                    });
                    PausedView {
                        scene,
                        catalog,
                        scan: measured_scan.clone(),
                    }
                });
        if refresh || paused_view.is_none() || self.paused_view != paused_view {
            self.last_network_publication = Instant::now();
            *self
                .latest
                .lock()
                .map_err(|_| io::Error::other("gzweb snapshot lock poisoned"))? = (
                generation,
                encoded.clone(),
                measured_scan.clone(),
                catalog.clone(),
            );
        }
        self.paused_view = paused_view;
        self.published = Instant::now();
        self.published_generation = generation;
        if let Some(lifecycle) = &mut self.lifecycle {
            lifecycle.send_replies(
                encoded,
                measured_scan,
                catalog.expect("configured lifecycle catalog"),
            );
        }
        for (pending, encoded) in control_replies {
            // A disconnected client cannot hold simulation or block shutdown.
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

pub(super) fn replay(path: &std::path::Path, port: u16, stop: &AtomicBool) -> io::Result<()> {
    recording::replay(path, port, stop)
}

/// Immutable visual resources for an offline recording. Building this table opens no listener.
#[derive(Default)]
pub struct ArchiveVisuals(Option<assets::Assets>);

impl ArchiveVisuals {
    pub fn new(simulation: &Simulation) -> io::Result<Self> {
        assets::Assets::offline(simulation).map(Self)
    }

    /// Capture the actual scene and lifecycle catalog against the archive resource table.
    pub fn frame(
        &self,
        simulation: &Simulation,
        sdf: &sdformat::Sdf,
        world: &str,
        sequence: u64,
    ) -> io::Result<(String, String)> {
        let scene = snapshot::extract_with_assets(simulation, world, sequence, self.0.as_ref())?;
        let lifecycle = lifecycle::Lifecycle::new(simulation, sdf, world, &scene)?;
        Ok((
            snapshot::encode(&scene)?,
            lifecycle.encode_catalog(simulation, &scene)?,
        ))
    }

    /// Each epoch's manifest owns its asset declarations. Byte buffers share immutable storage.
    pub fn files(&self) -> std::collections::BTreeMap<String, bytes::Bytes> {
        self.0
            .as_ref()
            .map(|a| a.snapshot().files().clone())
            .unwrap_or_default()
    }
}

/// Serialize the committed primitive scene and native model catalog for a local recording.
pub fn primitive_frame(
    simulation: &Simulation,
    sdf: &sdformat::Sdf,
    world: &str,
    sequence: u64,
) -> io::Result<(String, String)> {
    ArchiveVisuals::default().frame(simulation, sdf, world, sequence)
}

#[cfg(test)]
#[path = "../tests/private/web_operator_step.rs"]
mod operator_step_tests;
