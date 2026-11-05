// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only presentation of admitted source frames. No native world is owned.
use super::{
    archive::{Action, Archive},
    clock::{Clock, Credit},
};
use crate::web::{self, assets::PlaybackAssets, lifecycle::Request};
use serde::Serialize;
use std::{io, path::Path, sync::Arc, time::Instant};

pub(super) struct Presentation {
    pub scene: Arc<String>,
    pub catalog: Arc<String>,
    pub status: Arc<String>,
    pub credit: Credit,
}

pub(super) struct Player {
    archive: Archive,
    clock: Clock,
    assets: Option<PlaybackAssets>,
    prefix: Option<String>,
    sequence: u64,
}

#[derive(Serialize)]
struct Summary<'a> {
    action: &'a Action,
    model_id: Option<u64>,
    ok: bool,
}
#[derive(Serialize)]
struct Status<'a> {
    version: u8,
    kind: &'static str,
    world_id: u64,
    epoch: u64,
    scene_sequence: u64,
    frame: usize,
    frames: usize,
    recorded_epoch: u64,
    recorded_sequence: u64,
    recorded_seconds: u64,
    recorded_nanoseconds: u32,
    recorded_paused: bool,
    playing: bool,
    finished: bool,
    outcomes: Vec<Summary<'a>>,
}

impl Player {
    pub fn open(path: &Path, now: Instant) -> io::Result<Self> {
        let archive = Archive::open(path)?;
        let clock = Clock::new(archive.stamps.clone(), now)?;
        let assets = PlaybackAssets::new(archive.assets.clone())?;
        let prefix = assets.as_ref().map(PlaybackAssets::prefix);
        // Validate the largest possible transport counters and rewritten asset
        // URLs for every frame before exposing even the initial presentation.
        for ordinal in 0..archive.stamps.len() {
            let mut frame = archive.frame(ordinal)?;
            frame.scene.playback(u64::MAX, u64::MAX, false, |uri| {
                remap(uri, &archive, prefix.as_deref())
            })?;
            web::snapshot::encode(&frame.scene)?;
            frame.catalog.read_only(&frame.scene);
            frame.catalog.replay_encode()?;
        }
        Ok(Self {
            archive,
            clock,
            assets,
            prefix,
            sequence: 0,
        })
    }

    pub fn disconnect(&mut self, now: Instant) {
        self.clock.pause(now);
        self.clock.forget_credit();
    }

    pub fn accept(&mut self, credit: Credit) {
        self.clock.accept(credit);
    }
    pub fn advance(&mut self, now: Instant) -> io::Result<bool> {
        self.clock.advance(now)
    }

    /// Called for each actual publication, including idle/EOF heartbeats. Its
    /// fresh transport sequence never changes the original recording clock.
    pub fn present(&mut self) -> io::Result<Presentation> {
        if self.assets.as_ref().is_some_and(|assets| !assets.healthy()) {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "playback asset worker stopped",
            ));
        }
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or_else(|| io::Error::other("playback transport sequence exhausted"))?;
        let mut frame = self.archive.frame(self.clock.frame())?;
        let recorded_epoch = frame.scene.epoch;
        let recorded_sequence = frame.scene.sequence();
        let recorded_paused = frame.scene.paused;
        let recorded_seconds = frame.scene.seconds;
        let recorded_nanoseconds = frame.scene.nanoseconds;
        frame
            .scene
            .playback(self.clock.epoch(), sequence, !self.clock.playing(), |uri| {
                remap(uri, &self.archive, self.prefix.as_deref())
            })?;
        frame.catalog.read_only(&frame.scene);
        let status = Status {
            version: 1,
            kind: "replay_status",
            world_id: frame.scene.world_id(),
            epoch: self.clock.epoch(),
            scene_sequence: sequence,
            frame: self.clock.frame() + 1,
            frames: self.clock.frames(),
            recorded_epoch,
            recorded_sequence,
            recorded_seconds,
            recorded_nanoseconds,
            recorded_paused,
            playing: self.clock.playing(),
            finished: self.clock.finished(),
            outcomes: frame
                .outcomes
                .iter()
                .map(|outcome| Summary {
                    action: &outcome.action,
                    model_id: outcome.model_id,
                    ok: outcome.ok,
                })
                .collect(),
        };
        let scene = web::snapshot::encode(&frame.scene)?;
        let catalog = frame.catalog.replay_encode()?;
        let status = serde_json::to_string(&status).map_err(io::Error::other)?;
        self.sequence = sequence;
        Ok(Presentation {
            scene: Arc::new(scene),
            catalog: Arc::new(catalog),
            status: Arc::new(status),
            credit: self.clock.credit(),
        })
    }

    /// Only playback cursor controls are interpreted. Recorded old controls are
    /// metadata and are never injected into this command path.
    pub fn control(
        &mut self,
        control: &web::Control,
        now: Instant,
    ) -> io::Result<(Presentation, String)> {
        self.sequence
            .checked_add(1)
            .ok_or_else(|| io::Error::other("playback transport sequence exhausted"))?;
        let before = self.clock.clone();
        let ok = match control.action {
            web::Action::Pause => {
                self.clock.pause(now);
                true
            }
            web::Action::Resume => self.clock.play(now).is_ok(),
            web::Action::Reset => self.clock.restart(now).is_ok(),
        };
        let presentation = match self.present() {
            Ok(presentation) => presentation,
            Err(error) => {
                self.clock = before;
                return Err(error);
            }
        };
        let frame = self.archive.frame(self.clock.frame())?;
        let ack = web::Ack {
            version: 1,
            kind: "ack",
            request_id: control.request_id,
            ok,
            epoch: self.clock.epoch(),
            seconds: frame.scene.seconds,
            nanoseconds: frame.scene.nanoseconds,
            paused: !self.clock.playing(),
            error: (!ok).then_some("playback control refused by the current cursor state"),
        };
        Ok((
            presentation,
            serde_json::to_string(&ack).map_err(io::Error::other)?,
        ))
    }

    pub fn refuse_lifecycle(&mut self, request: &Request) -> io::Result<(Presentation, String)> {
        let presentation = self.present()?;
        let frame = self.archive.frame(self.clock.frame())?;
        let (action, _) = request.recorded_request();
        let ack = serde_json::json!({"version":1,"kind":"lifecycle_ack","request_id":request.request_id,
            "action":action,"world_id":frame.scene.world_id(),"epoch":self.clock.epoch(),"scene_sequence":self.sequence,
            "seconds":frame.scene.seconds,"nanoseconds":frame.scene.nanoseconds,"paused":!self.clock.playing(),
            "ok":false,"error":"recorded playback is read-only","model_id":null,"state":null});
        Ok((presentation, ack.to_string()))
    }
}

fn remap(uri: &str, archive: &Archive, prefix: Option<&str>) -> io::Result<String> {
    let route = uri
        .strip_prefix(&archive.asset_prefix)
        .filter(|route| archive.assets.contains_key(*route))
        .ok_or_else(|| io::Error::other("playback asset left admitted snapshot"))?;
    let prefix = prefix.ok_or_else(|| io::Error::other("playback asset server absent"))?;
    let mapped = format!("{prefix}{route}");
    if mapped.len() > 2048 {
        return Err(io::Error::other(
            "rewritten playback asset URI exceeds admitted bound",
        ));
    }
    Ok(mapped)
}

#[cfg(test)]
#[path = "playback_tests.rs"]
mod tests;
