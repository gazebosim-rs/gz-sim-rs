// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::ledger::Ledger;
use bincode::Options;
use sha2::{Digest, Sha256};
use std::{
    fs::File,
    io::{Read, Write},
};
pub(super) struct Disk(Ledger);
const V2_MAGIC: &[u8; 8] = b"GZ2SCU02";
const FILE_MAGIC: &[u8; 8] = b"GZ2SCU01";
impl Disk {
    pub fn lock(dir: &Path, create: bool) -> Result<Self> {
        Self::from_ledger(Ledger::lock(dir, create)?)
    }
    pub fn lock_existing(dir: &Path) -> Result<Self> {
        Self::from_ledger(Ledger::lock_existing(dir)?)
    }
    fn from_ledger(l: Ledger) -> Result<Self> {
        if l.other_archive_exists(crate::articulation::ledger::MULTI_SPHERE_FILE)? {
            return Err(invalid("directory belongs to a different hinge profile"));
        }
        Ok(Self(l))
    }
    pub fn writer_identity(&self) -> io::Result<crate::articulation::ledger::WriterIdentity> {
        self.0.writer_identity()
    }
    pub fn lock_matching(
        dir: &Path,
        original: &crate::articulation::ledger::WriterIdentity,
    ) -> Result<Self> {
        let disk = Self::lock_existing(dir)?;
        if disk.writer_identity()? != *original {
            return Err(invalid(
                "checkpoint belongs to a different session or controller marker",
            ));
        }
        Ok(disk)
    }
    pub fn reclaim_staging(&self) -> io::Result<()> {
        self.0.reclaim_staging()
    }
    pub fn exists(&self) -> io::Result<bool> {
        self.0.multi_path().try_exists()
    }
    pub fn slot(&self, i: u8) -> PathBuf {
        self.0.slot(i)
    }
    pub fn publish(&self, spec: &MultiSphereSpec, c: &Cut) -> io::Result<()> {
        #[cfg(feature = "sphere-pair-trace")]
        let _timing = timing::Scope::begin(
            "disk",
            if c.pending.is_some() {
                "pending"
            } else {
                "committed"
            },
            c.tick,
            None,
        );
        let (magic, bytes) = encode_payload(spec, c)?;
        let path = self.0.multi_path();
        let dir = path.parent().unwrap();
        let mut t = tempfile::Builder::new()
            .prefix("barrier.pending.gzhinge")
            .rand_bytes(0)
            .tempfile_in(dir)?;
        t.write_all(magic)?;
        t.write_all(&timed!(archive_encode_ns, Sha256::digest(&bytes)))?;
        t.write_all(&bytes)?;
        timed!(file_sync_ns, t.as_file().sync_all())?;
        #[cfg(test)]
        crate::moving::crash_tests::gate("before-rename");
        t.persist(&path)?;
        #[cfg(test)]
        crate::moving::crash_tests::gate("after-rename");
        let directory = File::open(dir)?;
        timed!(dir_sync_ns, directory.sync_all())
    }
    pub fn read(&self) -> Result<(MultiSphereSpec, Cut)> {
        let bytes = read_envelope(open_archive(&self.0.multi_path(), false)?)?;
        decode_cut(&bytes)
    }

    pub fn read_identified(&self) -> Result<(MultiSphereSpec, Cut, String)> {
        identified_cut(open_archive(&self.0.multi_path(), true)?)
    }
}

fn encode_payload(spec: &MultiSphereSpec, c: &Cut) -> io::Result<(&'static [u8; 8], Vec<u8>)> {
    let recipe = timed!(archive_encode_ns, serde_json::to_vec(spec))?;
    Ok(match &c.pair {
        Some(pair) => (
            V2_MAGIC,
            timed!(archive_encode_ns, worker::encode(&(recipe, c, pair)))?,
        ),
        None => (
            FILE_MAGIC,
            timed!(archive_encode_ns, worker::encode(&(recipe, c)))?,
        ),
    })
}

/// Compare every supplied byte to the envelope the retained cut would publish.
/// The comparison never opens the on-disk archive or contacts a worker.
pub(super) fn matches_publication(
    spec: &MultiSphereSpec,
    cut: &Cut,
    archive: &[u8],
) -> io::Result<bool> {
    let (magic, payload) = encode_payload(spec, cut)?;
    Ok(archive.len() == payload.len() + 40
        && archive[..8] == magic[..]
        && archive[40..] == payload
        && archive[8..40] == Sha256::digest(&payload)[..])
}

fn open_archive(path: &Path, require_published_entry: bool) -> io::Result<File> {
    let mut options = File::options();
    options.read(true);
    #[cfg(unix)]
    {
        use rustix::fs::OFlags;
        use std::os::unix::fs::OpenOptionsExt;
        let mut flags = OFlags::NONBLOCK;
        if require_published_entry {
            flags |= OFlags::NOFOLLOW;
        }
        options.custom_flags(flags.bits() as i32);
    }
    #[cfg(not(unix))]
    if require_published_entry && path.symlink_metadata()?.is_symlink() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected a published archive entry",
        ));
    }
    let file = options.open(path)?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected a regular multi-sphere archive",
        ));
    }
    Ok(file)
}

/// Inspect a published checkpoint directly without acquiring a lease, touching
/// the session directory, or invoking a physics worker.
pub fn inspect_published_path(path: &Path) -> Result<InspectedCut> {
    inspect_published_cut(open_archive(path, false)?)
}

/// Read through one open handle. Atomic replacement of the pathname while
/// this runs cannot mix a new receipt with an older native payload.
fn read_envelope(input: impl Read) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    input
        .take((worker::LIMIT + 41) as u64)
        .read_to_end(&mut bytes)?;
    if bytes.len() < 40
        || bytes.len() > worker::LIMIT + 40
        || (&bytes[..8] != FILE_MAGIC && &bytes[..8] != V2_MAGIC)
        || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
    {
        return Err(invalid(
            "invalid multi-sphere archive checksum, version or length",
        ));
    }
    Ok(bytes)
}

fn decode_cut(bytes: &[u8]) -> Result<(MultiSphereSpec, Cut)> {
    let (s, c): (Vec<u8>, Cut) = if &bytes[..8] == FILE_MAGIC {
        worker::codec()
            .deserialize(&bytes[40..])
            .map_err(io::Error::other)?
    } else {
        let (s, mut c, p): (Vec<u8>, Cut, PairPart) = worker::codec()
            .deserialize(&bytes[40..])
            .map_err(io::Error::other)?;
        c.pair = Some(p);
        (s, c)
    };
    let spec: MultiSphereSpec = serde_json::from_slice(&s).map_err(io::Error::from)?;
    spec.validate()?;
    c.validate(&spec)?;
    Ok((spec, c))
}

fn identified_cut(input: impl Read) -> Result<(MultiSphereSpec, Cut, String)> {
    let bytes = read_envelope(input)?;
    let (spec, cut) = decode_cut(&bytes)?;
    let sha256 = format!("{:x}", Sha256::digest(&bytes));
    Ok((spec, cut, sha256))
}

/// A diagnostic snapshot of a published archive. This identifies the bytes read
/// and does not certify how a coordinator failed or whether it may retry.
#[derive(Debug, Serialize)]
pub struct InspectedCut {
    pub sha256: String,
    pub hinge_native_sha256: String,
    pub pair_native_sha256: Option<String>,
    pub cut: PublishedCut,
}

/// Validate and identify the same bounded archive read for inspection.
pub fn inspect_published_cut(input: impl Read) -> Result<InspectedCut> {
    let (spec, cut, sha256) = identified_cut(input)?;
    Ok(inspected(spec, cut, sha256))
}

/// Verify all resident images and their samples using the same native loader
/// and receipt checks as a worker reinstall, without spawning any workers.
pub fn inspect_restart_cut(input: impl Read) -> Result<InspectedCut> {
    let (spec, cut, sha256) = identified_cut(input)?;
    native::verify_restart(&spec, &cut)?;
    Ok(inspected(spec, cut, sha256))
}

pub(super) fn inspected(spec: MultiSphereSpec, cut: Cut, sha256: String) -> InspectedCut {
    InspectedCut {
        sha256,
        hinge_native_sha256: format!("{:x}", Sha256::digest(&cut.hinge.image)),
        pair_native_sha256: cut
            .pair
            .as_ref()
            .map(|p| format!("{:x}", Sha256::digest(&p.image))),
        cut: published(spec, cut),
    }
}

/// Inspect the atomic receipt without acquiring the writer lease or starting workers.
/// Reading the committed metadata verifies the same envelope and receipt as pool
/// recovery. Installing opaque native solver state remains part of recovery.
pub fn read_published_cut(input: impl Read) -> Result<PublishedCut> {
    let (spec, cut) = decode_cut(&read_envelope(input)?)?;
    Ok(published(spec, cut))
}

fn published(spec: MultiSphereSpec, cut: Cut) -> PublishedCut {
    PublishedCut {
        spec,
        width: cut.width,
        epoch: cut.epoch,
        generation: cut.generation,
        owners: cut.owners,
        pending_effort: cut.pending,
        observation: cut.state(spec.hinge.joint_id),
    }
}
