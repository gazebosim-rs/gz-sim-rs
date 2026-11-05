// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recording state is owned only when explicitly enabled on the live bridge.
use super::{
    archive::{Action, Outcome},
    writer::Recorder,
};
use crate::web::{
    assets::CapturedAssets,
    lifecycle::Catalog,
    snapshot::{self, Scene},
};
use std::{io, path::Path};

pub(in crate::web) struct Capture {
    recorder: Recorder,
    previous: (String, String),
}
impl Capture {
    pub fn start(
        path: &Path,
        assets: Option<&CapturedAssets>,
        scene: &Scene,
        catalog: &str,
    ) -> io::Result<Self> {
        let catalog = Catalog::decode(catalog, scene)?;
        let previous = (scene.recording_key()?, catalog.recording_key()?);
        let mut recorder = Recorder::start(path, assets)?;
        recorder.record(snapshot::encode(scene)?, catalog.replay_encode()?, vec![])?;
        Ok(Self { recorder, previous })
    }

    pub fn record(
        &mut self,
        scene: &Scene,
        catalog: &str,
        outcomes: Vec<Outcome>,
    ) -> io::Result<()> {
        if !self.recorder.healthy() {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "recording writer stopped",
            ));
        }
        if outcomes.len() > 8 {
            return Err(io::Error::other(
                "recording exceeds eight outcomes per frame",
            ));
        }
        let catalog = Catalog::decode(catalog, scene)?;
        let key = (scene.recording_key()?, catalog.recording_key()?);
        if scene.paused && outcomes.is_empty() && key == self.previous {
            return Ok(());
        }
        self.recorder
            .record(snapshot::encode(scene)?, catalog.replay_encode()?, outcomes)?;
        self.previous = key;
        Ok(())
    }
    pub fn finish(self) -> io::Result<()> {
        self.recorder.finish()
    }
}

pub(in crate::web) fn world_outcome(
    action: &crate::web::Action,
    trace: String,
    result: String,
    ok: bool,
) -> Outcome {
    let action = match action {
        crate::web::Action::Pause => Action::Pause,
        crate::web::Action::Resume => Action::Resume,
        crate::web::Action::Reset => Action::Reset,
    };
    Outcome {
        action,
        model_id: None,
        ok,
        request: trace,
        result,
    }
}

#[cfg(test)]
#[path = "capture_tests.rs"]
mod tests;
