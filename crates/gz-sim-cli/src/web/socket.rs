// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::net::{Shutdown, TcpStream};
use tungstenite::{
    HandshakeError, Message, WebSocket,
    handshake::server::{Callback, ErrorResponse, Request, Response},
    protocol::WebSocketConfig,
};

struct LocalViewer(Arc<AtomicBool>);

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
        for field in request.uri().query().unwrap_or_default().split('&') {
            let (key, value) = field.split_once('=').unwrap_or((field, ""));
            if key == "scan" {
                if selected || value != "1" {
                    let mut rejected = ErrorResponse::new(Some("invalid scan opt-in".into()));
                    *rejected.status_mut() = tungstenite::http::StatusCode::BAD_REQUEST;
                    return Err(rejected);
                }
                selected = true;
            }
        }
        self.0.store(selected, Ordering::Release);
        Ok(response)
    }
}

fn handshake(stream: TcpStream, stop: &AtomicBool) -> Option<(WebSocket<TcpStream>, bool)> {
    stream.set_nonblocking(true).ok()?;
    let config = WebSocketConfig::default()
        .read_buffer_size(4096)
        .write_buffer_size(0)
        .max_write_buffer_size(131072)
        .max_message_size(Some(4096))
        .max_frame_size(Some(4096));
    let opted_in = Arc::new(AtomicBool::new(false));
    let mut result =
        tungstenite::accept_hdr_with_config(stream, LocalViewer(opted_in.clone()), Some(config));
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
                return Some((socket, opted_in.load(Ordering::Acquire)));
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

struct Peer {
    socket: WebSocket<TcpStream>,
    alive: Arc<AtomicBool>,
    generation: u64,
    last_id: u64,
    sent: Option<Arc<String>>,
    scan: bool,
    pending: Option<mpsc::Receiver<String>>,
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.alive.store(false, Ordering::Release);
        let _ = self.socket.get_ref().shutdown(Shutdown::Both);
    }
}

impl Peer {
    fn poll(&mut self, latest: &Latest, commands: &mpsc::SyncSender<Pending>) -> bool {
        if let Some(receiver) = &self.pending {
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
        let Ok((generation, scene, scan)) = latest
            .lock()
            .map(|value| (value.0, Arc::clone(&value.1), value.2.clone()))
        else {
            return false;
        };
        if generation == self.generation
            && self
                .sent
                .as_ref()
                .is_none_or(|old| !Arc::ptr_eq(old, &scene))
        {
            if self
                .socket
                .send(Message::Text(scene.as_str().into()))
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
        }
        match self.socket.read() {
            Ok(Message::Text(text)) if self.pending.is_none() => {
                let Ok(control) = Control::decode(&text, self.last_id) else {
                    return false;
                };
                self.last_id = control.request_id;
                let (reply, receiver) = mpsc::sync_channel(1);
                if commands
                    .try_send(Pending {
                        control,
                        alive: self.alive.clone(),
                        reply,
                    })
                    .is_err()
                {
                    return false;
                }
                self.pending = Some(receiver);
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

pub(super) fn serve(
    listener: TcpListener,
    latest: &Latest,
    stop: &AtomicBool,
    connected: &AtomicU64,
    scan_connected: &AtomicBool,
    commands: &mpsc::SyncSender<Pending>,
) {
    let mut peer: Option<Peer> = None;
    let mut generation = 0_u64;
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _)) => {
                if peer.is_some() {
                    let _ = stream.shutdown(Shutdown::Both);
                } else if let Some((socket, scan)) = handshake(stream, stop) {
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
                        pending: None,
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
