// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::net::{Shutdown, TcpStream};
use tungstenite::{
    HandshakeError, Message, WebSocket,
    handshake::server::{Callback, ErrorResponse, Request, Response},
    protocol::WebSocketConfig,
};

struct LocalViewer {
    scan: Arc<AtomicBool>,
    lifecycle: Arc<AtomicBool>,
    lifecycle_enabled: bool,
    replay_enabled: bool,
    credit: Arc<AtomicBool>,
}

impl Callback for LocalViewer {
    fn on_request(self, request: &Request, response: Response) -> Result<Response, ErrorResponse> {
        let valid_origin = request.headers().get("origin").is_none_or(|origin| {
            matches!(
                origin.to_str(),
                Ok("http://127.0.0.1:8080"
                    | "http://localhost:8080"
                    | "http://127.0.0.1:8081"
                    | "http://localhost:8081")
            )
        });
        if request.uri().path() != "/" || !valid_origin {
            let mut rejected =
                ErrorResponse::new(Some("local viewer origin or path rejected".into()));
            *rejected.status_mut() = tungstenite::http::StatusCode::FORBIDDEN;
            return Err(rejected);
        }
        let mut selected = false;
        let mut lifecycle = false;
        let mut credit = false;
        let mut replay = false;
        for field in request.uri().query().unwrap_or_default().split('&') {
            let (key, value) = field.split_once('=').unwrap_or((field, ""));
            if key == "replay" {
                if replay || value != "1" || !self.replay_enabled {
                    let mut rejected =
                        ErrorResponse::new(Some("invalid or unavailable replay opt-in".into()));
                    *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
                    return Err(rejected);
                }
                replay = true;
            }
            if key == "scan" {
                if self.replay_enabled || selected || value != "1" {
                    let mut rejected = ErrorResponse::new(Some("invalid scan opt-in".into()));
                    *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
                    return Err(rejected);
                }
                selected = true;
            }
            if key == "lifecycle" {
                if lifecycle || value != "1" || !self.lifecycle_enabled {
                    let mut rejected =
                        ErrorResponse::new(Some("invalid or unavailable lifecycle opt-in".into()));
                    *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
                    return Err(rejected);
                }
                lifecycle = true;
            }
            if key == "lifecycle_credit" {
                if credit || value != "1" {
                    let mut rejected =
                        ErrorResponse::new(Some("invalid publication credit opt-in".into()));
                    *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
                    return Err(rejected);
                }
                credit = true;
            }
        }
        if self.replay_enabled && !(replay && lifecycle && credit) {
            let mut rejected = ErrorResponse::new(Some(
                "playback requires replay, lifecycle and publication credit opt-ins".into(),
            ));
            *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
            return Err(rejected);
        }
        if credit && !lifecycle {
            let mut rejected =
                ErrorResponse::new(Some("publication credit requires lifecycle opt-in".into()));
            *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
            return Err(rejected);
        }
        self.credit.store(credit, Ordering::Release);
        self.scan.store(selected, Ordering::Release);
        self.lifecycle.store(lifecycle, Ordering::Release);
        Ok(response)
    }
}

fn handshake(
    stream: TcpStream,
    stop: &AtomicBool,
    lifecycle_enabled: bool,
) -> Option<(WebSocket<TcpStream>, bool, bool, bool)> {
    handshake_mode(stream, stop, lifecycle_enabled, false)
}

pub(super) fn replay_handshake(
    stream: TcpStream,
    stop: &AtomicBool,
) -> Option<WebSocket<TcpStream>> {
    handshake_mode(stream, stop, true, true).map(|(socket, _, _, _)| socket)
}

fn handshake_mode(
    stream: TcpStream,
    stop: &AtomicBool,
    lifecycle_enabled: bool,
    replay_enabled: bool,
) -> Option<(WebSocket<TcpStream>, bool, bool, bool)> {
    stream.set_nonblocking(true).ok()?;
    let config = WebSocketConfig::default()
        .read_buffer_size(4096)
        .write_buffer_size(0)
        .max_write_buffer_size(131072)
        .max_message_size(Some(4096))
        .max_frame_size(Some(4096));
    let opted_in = Arc::new(AtomicBool::new(false));
    let lifecycle = Arc::new(AtomicBool::new(false));
    let credit = Arc::new(AtomicBool::new(false));
    let mut result = tungstenite::accept_hdr_with_config(
        stream,
        LocalViewer {
            scan: opted_in.clone(),
            lifecycle: lifecycle.clone(),
            lifecycle_enabled,
            replay_enabled,
            credit: credit.clone(),
        },
        Some(config),
    );
    let deadline = Instant::now() + Duration::from_millis(250);
    loop {
        match result {
            Ok(socket) => {
                socket.get_ref().set_nonblocking(false).ok()?;
                socket
                    .get_ref()
                    .set_read_timeout(Some(Duration::from_millis(2)))
                    .ok()?;
                socket
                    .get_ref()
                    .set_write_timeout(Some(Duration::from_millis(100)))
                    .ok()?;
                return Some((
                    socket,
                    opted_in.load(Ordering::Acquire),
                    lifecycle.load(Ordering::Acquire),
                    credit.load(Ordering::Acquire),
                ));
            }
            Err(HandshakeError::Interrupted(mid))
                if Instant::now() < deadline && !stop.load(Ordering::Acquire) =>
            {
                thread::sleep(Duration::from_millis(2));
                result = mid.handshake();
            }
            Err(_) => return None,
        }
    }
}

/// One normal publication may be in flight on an explicitly credited socket.
/// Priority replies do not touch this connection-local state.
pub(super) struct PublicationCredit {
    next: u64,
    awaiting: Option<u64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PublicationReady {
    version: u8,
    kind: String,
    token: u64,
}

impl PublicationCredit {
    pub(super) fn pending(&self) -> bool {
        self.awaiting.is_some()
    }

    pub(super) fn new() -> Self {
        Self {
            next: 1,
            awaiting: None,
        }
    }

    pub(super) fn reserve(&mut self) -> Option<u64> {
        if self.awaiting.is_some() {
            return None;
        }
        let token = self.next;
        self.next = self.next.checked_add(1)?;
        self.awaiting = Some(token);
        Some(token)
    }

    pub(super) fn ready(&mut self, text: &str) -> bool {
        let Ok(ready) = serde_json::from_str::<PublicationReady>(text) else {
            return false;
        };
        if ready.version != 1
            || ready.kind != "publication_ready"
            || ready.token == 0
            || self.awaiting != Some(ready.token)
        {
            return false;
        }
        self.awaiting = None;
        true
    }
}

struct Peer {
    socket: WebSocket<TcpStream>,
    alive: Arc<AtomicBool>,
    generation: u64,
    last_id: u64,
    sent: Option<Arc<String>>,
    scan: bool,
    lifecycle: bool,
    recording: bool,
    pending: Option<Awaiting>,
    credit: Option<PublicationCredit>,
}

enum Awaiting {
    Control(mpsc::Receiver<String>),
    Lifecycle(mpsc::Receiver<lifecycle::Committed>),
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.alive.store(false, Ordering::Release);
        let _ = self.socket.get_ref().shutdown(Shutdown::Both);
    }
}

impl Peer {
    fn poll(&mut self, latest: &Latest, commands: &mpsc::SyncSender<Queued>) -> bool {
        if let Some(Awaiting::Control(receiver)) = &self.pending {
            match receiver.try_recv() {
                Ok(ack) => {
                    if self.socket.send(Message::Text(ack.into())).is_err() {
                        return false;
                    }
                    self.pending = None;
                }
                Err(mpsc::TryRecvError::Disconnected) => return false,
                Err(mpsc::TryRecvError::Empty) => {}
            }
        }
        if let Some(Awaiting::Lifecycle(receiver)) = &self.pending {
            match receiver.try_recv() {
                Ok(committed) => {
                    // Deliver the exact committed frame, even when Latest has
                    // advanced while this bounded reply waited for the socket.
                    if self
                        .socket
                        .send(Message::Text(committed.scene.as_str().into()))
                        .is_err()
                        || (self.scan
                            && committed.scan.as_ref().is_some_and(|scan| {
                                self.socket
                                    .send(Message::Text(scan.as_str().into()))
                                    .is_err()
                            }))
                        || self
                            .socket
                            .send(Message::Text(committed.catalog.as_str().into()))
                            .is_err()
                        || self
                            .socket
                            .send(Message::Text(committed.ack.into()))
                            .is_err()
                    {
                        return false;
                    }
                    self.sent = Some(committed.scene);
                    self.pending = None;
                    return true;
                }
                Err(mpsc::TryRecvError::Disconnected) => return false,
                Err(mpsc::TryRecvError::Empty) => {}
            }
        }
        let Ok((generation, scene, scan, catalog)) = latest.lock().map(|value| {
            (
                value.0,
                Arc::clone(&value.1),
                value.2.clone(),
                value.3.clone(),
            )
        }) else {
            return false;
        };
        if generation == self.generation
            && !matches!(self.pending, Some(Awaiting::Lifecycle(_)))
            && self
                .credit
                .as_ref()
                .is_none_or(|credit| credit.awaiting.is_none())
            && self
                .sent
                .as_ref()
                .is_none_or(|old| !Arc::ptr_eq(old, &scene))
        {
            // Reserve before any writes: exhaustion must not expose a tuple
            // whose credit token could be reused or absent.
            let token = match self.credit.as_mut().map(PublicationCredit::reserve) {
                Some(Some(token)) => Some(token),
                Some(None) => return false,
                None => None,
            };
            if self
                .socket
                .send(Message::Text(scene.as_str().into()))
                .is_err()
            {
                return false;
            }
            if self.lifecycle
                && let Some(catalog) = catalog
                && self
                    .socket
                    .send(Message::Text(catalog.as_str().into()))
                    .is_err()
            {
                return false;
            }
            self.sent = Some(scene);
            if self.scan
                && let Some(scan) = scan
                && self
                    .socket
                    .send(Message::Text(scan.as_str().into()))
                    .is_err()
            {
                return false;
            }
            if let Some(token) = token {
                let marker =
                    serde_json::json!({"version":1,"kind":"publication_credit","token":token});
                if self
                    .socket
                    .send(Message::Text(marker.to_string().into()))
                    .is_err()
                {
                    return false;
                }
            }
        }
        match self.socket.read() {
            Ok(Message::Text(text)) => {
                let kind = serde_json::from_str::<serde_json::Value>(&text)
                    .ok()
                    .and_then(|value| {
                        value
                            .get("kind")
                            .and_then(|kind| kind.as_str())
                            .map(str::to_owned)
                    });
                // Ready is transport admission, not a world command. The
                // command pump can send it after a command in the same frame.
                if kind.as_deref() == Some("publication_ready") {
                    return self
                        .credit
                        .as_mut()
                        .is_some_and(|credit| credit.ready(&text));
                }
                if self.pending.is_some() {
                    return false;
                }
                if kind.as_deref() == Some("lifecycle") {
                    if !self.lifecycle {
                        return false;
                    }
                    let Ok(request) = lifecycle::Request::decode(&text, self.last_id) else {
                        return false;
                    };
                    self.last_id = request.request_id;
                    let (reply, receiver) = mpsc::sync_channel(1);
                    if commands
                        .try_send(Queued::Lifecycle(lifecycle::Pending {
                            trace: self.recording.then(|| text.to_string()),
                            request,
                            alive: self.alive.clone(),
                            reply,
                        }))
                        .is_err()
                    {
                        return false;
                    }
                    self.pending = Some(Awaiting::Lifecycle(receiver));
                } else {
                    let Ok(control) = Control::decode(&text, self.last_id) else {
                        return false;
                    };
                    self.last_id = control.request_id;
                    let (reply, receiver) = mpsc::sync_channel(1);
                    if commands
                        .try_send(Queued::Control(Pending {
                            trace: self.recording.then(|| text.to_string()),
                            control,
                            alive: self.alive.clone(),
                            reply,
                        }))
                        .is_err()
                    {
                        return false;
                    }
                    self.pending = Some(Awaiting::Control(receiver));
                }
                true
            }
            Ok(Message::Ping(_) | Message::Pong(_)) => self.socket.flush().is_ok(),
            Err(tungstenite::Error::Io(error))
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                true
            }
            _ => false,
        }
    }
}

pub(super) struct Modes {
    pub lifecycle: bool,
    pub recording: bool,
}

pub(super) fn serve(
    listener: TcpListener,
    latest: &Latest,
    stop: &AtomicBool,
    connected: &AtomicU64,
    scan_connected: &AtomicBool,
    commands: &mpsc::SyncSender<Queued>,
    modes: Modes,
) {
    let Modes {
        lifecycle: lifecycle_enabled,
        recording,
    } = modes;
    let mut peer: Option<Peer> = None;
    let mut generation = 0_u64;
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _)) => {
                if peer.is_some() {
                    let _ = stream.shutdown(Shutdown::Both);
                } else if let Some((socket, scan, lifecycle, credit)) =
                    handshake(stream, stop, lifecycle_enabled)
                {
                    let Some(next) = generation.checked_add(1) else {
                        break;
                    };
                    generation = next;
                    peer = Some(Peer {
                        socket,
                        generation,
                        alive: Arc::new(AtomicBool::new(true)),
                        last_id: 0,
                        sent: None,
                        scan,
                        lifecycle,
                        recording,
                        pending: None,
                        credit: credit.then(PublicationCredit::new),
                    });
                    scan_connected.store(scan, Ordering::Release);
                    connected.store(generation, Ordering::Release);
                }
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(_) => break,
        }
        if let Some(active) = &mut peer
            && !active.poll(latest, commands)
        {
            peer = None;
            connected.store(0, Ordering::Release);
            scan_connected.store(false, Ordering::Release);
        }
        thread::sleep(Duration::from_millis(2));
    }
    connected.store(0, Ordering::Release);
    scan_connected.store(false, Ordering::Release);
}

#[cfg(test)]
#[path = "../../tests/private/web_credit_contract.rs"]
mod credit_contract_tests;

#[cfg(test)]
#[path = "../../tests/private/web_replay_admission.rs"]
mod replay_admission_tests;
