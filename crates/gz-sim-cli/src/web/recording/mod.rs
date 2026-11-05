// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in bounded local scene recording and read-only presentation.
mod archive;
mod capture;
mod clock;
mod endpoint;
mod lines;
mod playback;
mod writer;

pub(in crate::web) fn replay(
    path: &std::path::Path,
    port: u16,
    stop: &std::sync::atomic::AtomicBool,
) -> std::io::Result<()> {
    let mut player = playback::Player::open(path, std::time::Instant::now())?;
    let listener = std::net::TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, port))?;
    eprintln!(
        "gzweb-rs read-only playback: ws://127.0.0.1:{}/ (explicit replay/lifecycle/publication-credit opt-ins required)",
        listener.local_addr()?.port()
    );
    endpoint::serve(listener, stop, &mut player)
}

pub(super) use archive::Outcome;
pub(super) use capture::Capture;

pub(super) use capture::world_outcome;

#[cfg(test)]
pub(super) fn admitted_frames(path: &std::path::Path) -> std::io::Result<Vec<serde_json::Value>> {
    let archive = archive::Archive::open(path)?;
    (0..archive.stamps.len()).map(|index| {
        let frame = archive.frame(index)?;
        Ok(serde_json::json!({"scene":frame.scene,"catalog":frame.catalog,"outcomes":frame.outcomes}))
    }).collect()
}
