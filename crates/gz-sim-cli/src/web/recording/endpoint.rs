// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One local read-only replay peer, sharing the accepted handshake/credit IO.
use super::{
    clock::Credit,
    playback::{Player, Presentation},
};
use crate::web::{
    Control,
    lifecycle::Request,
    socket::{PublicationCredit, replay_handshake},
};
use std::{
    io,
    net::{Shutdown, TcpListener, TcpStream},
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant},
};
use tungstenite::{Message, WebSocket};

struct Peer {
    socket: WebSocket<TcpStream>,
    credit: PublicationCredit,
    flight: Option<Credit>,
    last_id: u64,
    published: Instant,
    dirty: bool,
}
impl Drop for Peer {
    fn drop(&mut self) {
        let _ = self.socket.get_ref().shutdown(Shutdown::Both);
    }
}
impl Peer {
    fn tuple(&mut self, presentation: &Presentation) -> io::Result<()> {
        for text in [
            &presentation.scene,
            &presentation.catalog,
            &presentation.status,
        ] {
            self.socket
                .send(Message::Text(text.as_str().into()))
                .map_err(io::Error::other)?;
        }
        Ok(())
    }
    fn poll(&mut self, player: &mut Player, now: Instant) -> io::Result<bool> {
        if player.advance(now)? {
            self.dirty = true;
        }
        if !self.credit.pending()
            && (self.dirty
                || now.saturating_duration_since(self.published) >= Duration::from_secs(1))
        {
            let token = self
                .credit
                .reserve()
                .ok_or_else(|| io::Error::other("playback publication token exhausted"))?;
            let presentation = player.present()?;
            if self.tuple(&presentation).is_err() {
                return Ok(false);
            }
            let marker = serde_json::json!({"version":1,"kind":"publication_credit","token":token});
            if self
                .socket
                .send(Message::Text(marker.to_string().into()))
                .is_err()
            {
                return Ok(false);
            }
            self.flight = Some(presentation.credit);
            self.published = now;
            self.dirty = false;
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
                if kind.as_deref() == Some("publication_ready") {
                    if !self.credit.ready(&text) {
                        return Ok(false);
                    }
                    if let Some(credit) = self.flight.take() {
                        player.accept(credit);
                    }
                    return Ok(true);
                }
                let (presentation, ack) = if kind.as_deref() == Some("lifecycle") {
                    let Ok(request) = Request::decode(&text, self.last_id) else {
                        return Ok(false);
                    };
                    self.last_id = request.request_id;
                    player.refuse_lifecycle(&request)?
                } else {
                    let Ok(control) = Control::decode(&text, self.last_id) else {
                        return Ok(false);
                    };
                    self.last_id = control.request_id;
                    player.control(&control, now)?
                };
                // Priority commits never consume or replace the old normal
                // flight. A late Ready retires exactly its captured generation.
                if self.tuple(&presentation).is_err()
                    || self.socket.send(Message::Text(ack.into())).is_err()
                {
                    return Ok(false);
                }
                self.dirty = true;
                Ok(true)
            }
            Ok(Message::Ping(_) | Message::Pong(_)) => Ok(self.socket.flush().is_ok()),
            Err(tungstenite::Error::Io(error))
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                Ok(true)
            }
            _ => Ok(false),
        }
    }
}

pub(super) fn serve(
    listener: TcpListener,
    stop: &AtomicBool,
    player: &mut Player,
) -> io::Result<()> {
    listener.set_nonblocking(true)?;
    let mut peer: Option<Peer> = None;
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _)) if peer.is_some() => {
                let _ = stream.shutdown(Shutdown::Both);
            }
            Ok((stream, _)) => {
                if let Some(socket) = replay_handshake(stream, stop) {
                    player.disconnect(Instant::now());
                    peer = Some(Peer {
                        socket,
                        credit: PublicationCredit::new(),
                        flight: None,
                        last_id: 0,
                        published: Instant::now(),
                        dirty: true,
                    });
                }
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(error) => return Err(error),
        }
        if let Some(active) = &mut peer {
            // Transport failure closes this bounded peer, leaving the admitted
            // recording available for a fresh current snapshot on reconnect.
            if !active.poll(player, Instant::now())? {
                peer = None;
                player.disconnect(Instant::now());
            }
        }
        thread::sleep(Duration::from_millis(2));
    }
    Ok(())
}

#[cfg(test)]
#[path = "endpoint_tests.rs"]
mod tests;
