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

/// Captured mesh URLs use this placeholder origin. Playback remaps them to the
/// loopback asset service for the session. Export performs no network requests.
pub const EXPORT_ASSET_PREFIX: &str = "http://127.0.0.1:1";

/// Finalize an offline capture using the native recorder's file format, asset
/// hashing and full cross-frame admission. Each pair contains one encoded scene
/// and its matching lifecycle catalog. The output directory must be new.
pub fn export_archive(
    path: &std::path::Path,
    assets: std::collections::BTreeMap<String, bytes::Bytes>,
    frames: impl IntoIterator<Item = (String, String)>,
) -> std::io::Result<()> {
    export_archive_at(path, assets, 0, std::time::Duration::ZERO, frames)
}

/// Export an offline capture whose first physics frame is a measured checkpoint.
/// The recorder verifies the declared origin against the first scene and
/// validates the same sequence, ownership, asset and frame limits as live capture.
pub fn export_archive_at(
    path: &std::path::Path,
    assets: std::collections::BTreeMap<String, bytes::Bytes>,
    epoch: u64,
    time: std::time::Duration,
    frames: impl IntoIterator<Item = (String, String)>,
) -> std::io::Result<()> {
    let origin = clock::Origin {
        epoch,
        seconds: time.as_secs(),
        nanoseconds: time.subsec_nanos(),
    };
    let mut writer =
        writer::Recorder::from_files(path, EXPORT_ASSET_PREFIX.to_owned(), assets, origin)?;
    for (scene, catalog) in frames {
        writer.submit_offline(scene, catalog)?;
    }
    writer.finish()
}

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

#[cfg(test)]
mod checkpoint_export_tests {
    use super::*;
    use serde_json::json;
    use std::time::Duration;

    fn frame(epoch: u64, millis: u64) -> (String, String) {
        let time = Duration::from_millis(millis);
        let scene = json!({
            "version": 1, "kind": "scene", "world": "default", "world_id": 1,
            "epoch": epoch, "seconds": time.as_secs(), "nanoseconds": time.subsec_nanos(),
            "sequence": 1, "paused": true, "visuals": [],
        });
        let catalog = json!({
            "version": 1, "kind": "lifecycle_catalog", "world_id": 1, "epoch": epoch,
            "scene_sequence": 1, "templates": [], "models": [],
        });
        (scene.to_string(), catalog.to_string())
    }

    #[test]
    fn offline_checkpoint_export_admits_only_the_declared_physical_origin() {
        let tmp = tempfile::tempdir().unwrap();
        let at = Duration::from_millis(1150);
        let path = tmp.path().join("valid");
        export_archive_at(&path, Default::default(), 7, at, [frame(7, 1150)]).unwrap();
        let archive = archive::Archive::open(&path).unwrap();
        assert_eq!(
            (
                archive.origin.epoch,
                archive.origin.seconds,
                archive.origin.nanoseconds
            ),
            (7, 1, 150_000_000)
        );
        for (epoch, tick) in [(6, 1150), (7, 150)] {
            let path = tmp.path().join(format!("invalid-{epoch}-{tick}"));
            assert!(
                export_archive_at(&path, Default::default(), 7, at, [frame(epoch, tick)]).is_err()
            );
            assert!(!path.join("manifest.json").exists());
        }
    }
}
