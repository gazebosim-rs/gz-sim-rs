// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in writer. Only nonblocking bounded queue submission enters capture.
use super::{
    archive::{self, Archive, Asset, Manifest, Outcome, Record},
    lines,
};
use crate::web::assets::CapturedAssets;
use bytes::Bytes;
use std::{
    collections::BTreeMap,
    fs::{self, File, OpenOptions},
    io::{self, Read, Write},
    path::Path,
    sync::mpsc::{self, SyncSender},
    thread::{self, JoinHandle},
};

enum Message {
    Frame(Vec<u8>),
    Finish,
}

pub(super) struct Recorder {
    sender: Option<SyncSender<Message>>,
    worker: Option<JoinHandle<io::Result<()>>>,
    frames: u32,
    bytes: u64,
    failed: bool,
}
fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

impl Recorder {
    /// Called before the first physics tick. File creation/resource copying and
    /// readiness wait are confined to startup, never to a simulation step.
    pub fn start(path: &Path, captured: Option<&CapturedAssets>) -> io::Result<Self> {
        let prefix = captured
            .map_or("http://127.0.0.1:1", CapturedAssets::prefix)
            .to_owned();
        let files = captured.map_or_else(BTreeMap::new, |assets| assets.files().clone());
        let (sender, receiver) = mpsc::sync_channel(8);
        let (ready, readiness) = mpsc::sync_channel(1);
        let path = path.to_owned();
        let header = lines::encode(&Record::Header { version: 1 }, lines::LINE_BYTES)?;
        let bytes = header.len() as u64;
        let worker = thread::Builder::new()
            .name("gz-scene-record".into())
            .spawn(move || {
                let prepared = prepare(&path, &files);
                match prepared {
                    Ok((mut stream, assets)) => {
                        stream.write_all(&header)?;
                        let _ = ready.send(Ok(()));
                        run(&path, &mut stream, receiver, prefix, files, assets)
                    }
                    Err(error) => {
                        let _ = ready.send(Err(io::Error::new(error.kind(), error.to_string())));
                        Err(error)
                    }
                }
            })?;
        match readiness.recv() {
            Ok(Ok(())) => Ok(Self {
                sender: Some(sender),
                worker: Some(worker),
                frames: 0,
                bytes,
                failed: false,
            }),
            result => {
                drop(sender);
                let _ = worker.join();
                match result {
                    Ok(Err(error)) => Err(error),
                    _ => Err(io::Error::new(
                        io::ErrorKind::BrokenPipe,
                        "recording worker failed during startup",
                    )),
                }
            }
        }
    }

    pub fn healthy(&self) -> bool {
        !self.failed
            && self
                .worker
                .as_ref()
                .is_some_and(|worker| !worker.is_finished())
    }

    /// Call exactly once per committed captured scene/outcome. A refused queue
    /// is terminal; the caller reports failure and never retries that commit.
    pub fn record(
        &mut self,
        scene: String,
        catalog: String,
        outcomes: Vec<Outcome>,
    ) -> io::Result<()> {
        if self.failed {
            return Err(invalid("recording already failed"));
        }
        let result = self.submit(scene, catalog, outcomes);
        if result.is_err() {
            self.failed = true;
            self.sender.take();
        }
        result
    }
    fn submit(&mut self, scene: String, catalog: String, outcomes: Vec<Outcome>) -> io::Result<()> {
        if self.frames >= 4096 {
            return Err(invalid("recording exceeds 4096 frames"));
        }
        let frame = Record::Frame {
            ordinal: self.frames + 1,
            scene,
            catalog,
            outcomes,
        };
        let bytes = lines::encode(&frame, lines::LINE_BYTES)?;
        // Reserve the largest possible footer before admitting this frame.
        let footer =
            lines::encode(&Record::Footer { frames: 4096 }, lines::LINE_BYTES)?.len() as u64;
        if bytes.len() as u64 + footer > lines::FILE_BYTES.saturating_sub(self.bytes) {
            return Err(invalid("recording exceeds 64 MiB stream bound"));
        }
        let length = bytes.len() as u64;
        self.sender
            .as_ref()
            .ok_or_else(|| invalid("recording sender absent"))?
            .try_send(Message::Frame(bytes))
            .map_err(|error| match error {
                mpsc::TrySendError::Full(_) => io::Error::new(
                    io::ErrorKind::WouldBlock,
                    "recording writer Busy; committed capture was not retried",
                ),
                mpsc::TrySendError::Disconnected(_) => {
                    io::Error::new(io::ErrorKind::BrokenPipe, "recording writer stopped")
                }
            })?;
        self.frames += 1;
        self.bytes += length;
        Ok(())
    }

    /// Explicit shutdown finalization outside stepping. Closing/dropping without
    /// this message never manufactures a complete manifest for a partial prefix.
    pub fn finish(mut self) -> io::Result<()> {
        let submitted = self
            .sender
            .take()
            .ok_or_else(|| invalid("failed recording cannot finalize"))?
            .send(Message::Finish)
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "recording writer stopped"));
        let result = self
            .worker
            .take()
            .expect("owned recording worker")
            .join()
            .map_err(|_| io::Error::other("recording worker panicked"))?;
        submitted?;
        result
    }
}
impl Drop for Recorder {
    fn drop(&mut self) {
        self.sender.take();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn create(path: &Path) -> io::Result<File> {
    OpenOptions::new().write(true).create_new(true).open(path)
}

fn prepare(path: &Path, files: &BTreeMap<String, Bytes>) -> io::Result<(File, Vec<Asset>)> {
    if files.len() > 128 {
        return Err(invalid("recording asset count exceeds bound"));
    }
    let mut total = 0;
    for (route, bytes) in files {
        if !archive::asset_route(route)
            || bytes.len() > archive::ASSET_FILE_BYTES
            || bytes.len() > archive::ASSET_TOTAL_BYTES.saturating_sub(total)
        {
            return Err(invalid("recording asset paths or bytes exceed bounds"));
        }
        total += bytes.len();
    }
    fs::create_dir(path)?; // Refuse an existing directory, including unfinished captures.
    let stream = create(&path.join("frames.jsonl"))?;
    let mut assets = Vec::with_capacity(files.len());
    for (route, bytes) in files {
        let asset_path = path.join(&route[1..]);
        fs::create_dir_all(
            asset_path
                .parent()
                .expect("validated resource has a parent"),
        )?;
        let mut file = create(&asset_path)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        assets.push(Asset {
            route: route.clone(),
            bytes: bytes.len(),
            sha256: archive::digest(bytes),
        });
    }
    Ok((stream, assets))
}

fn run(
    path: &Path,
    stream: &mut File,
    receiver: mpsc::Receiver<Message>,
    prefix: String,
    files: BTreeMap<String, Bytes>,
    assets: Vec<Asset>,
) -> io::Result<()> {
    let mut frames = 0;
    loop {
        match receiver.recv() {
            Ok(Message::Frame(bytes)) => {
                stream.write_all(&bytes)?;
                frames += 1;
            }
            Ok(Message::Finish) => break,
            Err(_) => return Err(invalid("recording interrupted without finalization")),
        }
    }
    stream.write_all(&lines::encode(
        &Record::Footer { frames },
        lines::LINE_BYTES,
    )?)?;
    stream.sync_all()?;
    let mut bytes = Vec::new();
    File::open(path.join("frames.jsonl"))?
        .take(lines::FILE_BYTES + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() as u64 > lines::FILE_BYTES {
        return Err(invalid("recording stream exceeds bound at finalization"));
    }
    let stream_bytes = bytes.len() as u64;
    let stream_sha256 = archive::digest(&bytes);
    // Refuse inconsistent native capture before marking any archive complete.
    Archive::admit(bytes.into(), frames, prefix.clone(), files)?;
    let manifest = Manifest {
        version: 1,
        complete: true,
        frames,
        stream_bytes,
        stream_sha256,
        asset_prefix: prefix,
        assets,
    };
    let bytes = lines::encode(&manifest, lines::LINE_BYTES)?;
    let temporary = path.join("manifest.pending");
    let mut manifest_file = create(&temporary)?;
    manifest_file.write_all(&bytes)?;
    manifest_file.sync_all()?;
    fs::rename(temporary, path.join("manifest.json"))?;
    File::open(path)?.sync_all()?;
    Ok(())
}

#[cfg(test)]
#[path = "writer_tests.rs"]
mod tests;
