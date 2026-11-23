// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::ledger::Ledger;
use bincode::Options;
use std::{
    fs::File,
    io::{Read, Write},
};

#[derive(Clone)]
pub(super) struct Cut {
    pub spec: MovingSpec,
    pub width: u8,
    pub tick: u64,
    pub epoch: u64,
    pub generation: u64,
    pub owners: [u8; 2],
    pub motion: Motion,
    // Derived from the accepted image on reopen. Successful advancing steps consume all effort.
    pub pending_effort: Option<f64>,
    pub hinge: HingeSample,
    pub sphere: ArticulationLinkState,
    pub reactions: [MovingReaction; 2],
}

// The SDF recipe has optional fields encoded by name, so only that part uses
// JSON. Body, joint, contact and solver values use an exact binary encoding.
#[derive(Serialize, Deserialize)]
struct Record {
    recipe: Vec<u8>,
    width: u8,
    tick: u64,
    epoch: u64,
    generation: u64,
    owners: [u8; 2],
    motion: Motion,
    hinge: HingeSample,
    sphere: ArticulationLinkState,
    reactions: [MovingReaction; 2],
}
pub(super) struct Disk(Ledger);
const MAGIC: &[u8; 8] = b"GZMOVP01";
impl Disk {
    pub fn lock(dir: &Path, create: bool) -> Result<Self> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(crate::articulation::ledger::MOVING_FILE)? {
            return Err(invalid("directory belongs to a different hinge profile"));
        }
        Ok(Self(l))
    }
    pub fn slot(&self, i: u8) -> PathBuf {
        self.0.slot(i)
    }
    pub fn exists(&self) -> io::Result<bool> {
        self.0.moving_path().try_exists()
    }
    pub fn publish(&self, c: &Cut) -> io::Result<()> {
        let buf = worker::encode(&Record {
            recipe: serde_json::to_vec(&c.spec)?,
            width: c.width,
            tick: c.tick,
            epoch: c.epoch,
            generation: c.generation,
            owners: c.owners,
            motion: c.motion.clone(),
            hinge: c.hinge.clone(),
            sphere: c.sphere,
            reactions: c.reactions,
        })?;
        let path = self.0.moving_path();
        let dir = path.parent().expect("session directory");
        let mut temp = tempfile::Builder::new()
            .prefix("barrier.pending.gzhinge")
            .rand_bytes(0)
            .tempfile_in(dir)?;
        temp.write_all(MAGIC)?;
        temp.write_all(&Sha256::digest(&buf))?;
        temp.write_all(&buf)?;
        temp.as_file().sync_all()?;
        #[cfg(test)]
        super::crash_tests::gate("before-rename");
        temp.persist(&path)?;
        #[cfg(test)]
        super::crash_tests::gate("after-rename");
        File::open(dir)?.sync_all()
    }
    pub fn read(&self) -> Result<Cut> {
        let mut buf = Vec::new();
        File::open(self.0.moving_path())?
            .take((worker::LIMIT + 41) as u64)
            .read_to_end(&mut buf)?;
        if buf.len() < 40
            || buf.len() > worker::LIMIT + 40
            || &buf[..8] != MAGIC
            || buf[8..40] != Sha256::digest(&buf[40..])[..]
        {
            return Err(invalid("invalid moving barrier checksum, length or format"));
        }
        let r: Record = worker::codec()
            .deserialize(&buf[40..])
            .map_err(io::Error::other)?;
        let spec: MovingSpec = serde_json::from_slice(&r.recipe).map_err(io::Error::from)?;
        spec.validate()?;
        if !(2..=8).contains(&r.width)
            || r.owners.iter().any(|&i| i >= r.width)
            || r.epoch > r.generation
            || matches!(
                (&r.motion, r.tick),
                (Motion::Warm(_), 0) | (Motion::Cold(_), 1..)
            )
        {
            return Err(invalid("invalid moving barrier ownership or image"));
        }
        Ok(Cut {
            spec,
            width: r.width,
            tick: r.tick,
            epoch: r.epoch,
            generation: r.generation,
            owners: r.owners,
            pending_effort: r.motion.pending_effort()?,
            motion: r.motion,
            hinge: r.hinge,
            sphere: r.sphere,
            reactions: r.reactions,
        })
    }
}
