// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable restart input for a supervisor operating a two-sphere convoy.
//! The package identifies a cut and drive. Attaching to a live session requires
//! a separate operator decision and the physics coordinator's writer lease.
use super::JointTorqueDrive;
use gz_sim_scene::distributed_hinge::{
    ColdOrigin, InspectedCut, distributed_multi_sphere_spec, inspect_restart_cut,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    ffi::OsStr,
    fs::File,
    io::{self, Read, Write},
    path::Path,
};

const MAGIC: &[u8; 8] = b"GZSCAP01";
const HEADER: usize = 16;
const CHECKSUM: usize = 32;
const MAX_SDF: usize = 1024 * 1024;
const MAX_MANIFEST: usize = 8 * 1024 * 1024;
const MAX_ARCHIVE: usize = 1024 * 1024;
const MAX_PACKAGE: usize = HEADER + MAX_MANIFEST + MAX_ARCHIVE + CHECKSUM;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    sdf: String,
    world: String,
    drive: JointTorqueDrive,
    origin: ColdOrigin,
}

/// A verified package with the exact source SDF, drive definition and committed
/// worker checkpoint. The digest covers every byte, including the envelope.
pub struct ConvoyCapsule {
    bytes: Vec<u8>,
    archive_start: usize,
    manifest: Manifest,
    cut: InspectedCut,
    sha256: String,
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn directory(path: &Path) -> &Path {
    path.parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."))
}

fn read_regular(path: &Path, limit: usize) -> io::Result<Vec<u8>> {
    let mut options = File::options();
    options.read(true);
    #[cfg(unix)]
    {
        use rustix::fs::OFlags;
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags((OFlags::NONBLOCK | OFlags::NOFOLLOW).bits() as i32);
    }
    let file = options.open(path)?;
    let meta = file.metadata()?;
    if !meta.is_file() || meta.len() > limit as u64 {
        return Err(invalid("expected a bounded regular file"));
    }
    let mut bytes = Vec::with_capacity(meta.len() as usize);
    file.take((limit + 1) as u64).read_to_end(&mut bytes)?;
    if bytes.len() > limit {
        return Err(invalid("file exceeds size limit"));
    }
    Ok(bytes)
}

fn check_manifest(sdf: &str, world: &str) -> io::Result<()> {
    if sdf.is_empty() || sdf.len() > MAX_SDF || world.is_empty() || world.len() > 256 {
        return Err(invalid("invalid restart SDF or world name size"));
    }
    Ok(())
}

fn verify_cut(manifest: &Manifest, archive: &[u8]) -> io::Result<InspectedCut> {
    check_manifest(&manifest.sdf, &manifest.world)?;
    let cut = inspect_restart_cut(archive).map_err(io::Error::other)?;
    let spec = distributed_multi_sphere_spec(&manifest.sdf, "restart.sdf", &manifest.world)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
    if !spec.same_recipe(&cut.cut.spec).map_err(io::Error::other)? {
        return Err(invalid(
            "archived convoy recipe differs from the authored SDF",
        ));
    }
    let next = manifest
        .drive
        .interval_effort(cut.cut.observation.hinge.tick);
    if !next.is_finite()
        || cut
            .cut
            .pending_effort
            .is_some_and(|effort| effort.to_bits() != next.to_bits())
    {
        return Err(invalid(
            "archived pending effort differs from the selected drive interval",
        ));
    }
    Ok(cut)
}

impl ConvoyCapsule {
    /// Capture one committed atomic publication. Works while a supervisor owns
    /// the writer lease. Subsequent advances produce other archive identities.
    pub fn capture(
        archive: &Path,
        sdf: &str,
        world: &str,
        drive: &JointTorqueDrive,
    ) -> io::Result<Self> {
        if archive.file_name() != Some(OsStr::new("barrier.gzmultisphere")) {
            return Err(invalid(
                "expected the session's published barrier.gzmultisphere",
            ));
        }
        check_manifest(sdf, world)?;
        // Resolve the user's directory once. A symlink alias changing between
        // origin inspection and the archive read must not change the selection.
        let session = directory(archive).canonicalize()?;
        let origin = ColdOrigin::capture(&session)?;
        let archived = read_regular(&session.join("barrier.gzmultisphere"), MAX_ARCHIVE)?;
        if origin != ColdOrigin::capture(&session)? {
            return Err(invalid(
                "session identity changed during checkpoint capture",
            ));
        }
        let manifest = Manifest {
            sdf: sdf.to_owned(),
            world: world.to_owned(),
            drive: drive.clone(),
            origin,
        };
        let cut = verify_cut(&manifest, &archived)?;
        let metadata = serde_json::to_vec(&manifest)?;
        if metadata.len() > MAX_MANIFEST {
            return Err(invalid("restart manifest exceeds size limit"));
        }
        let archive_start = HEADER + metadata.len();
        let mut bytes = Vec::with_capacity(archive_start + archived.len() + CHECKSUM);
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&(metadata.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&(archived.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&metadata);
        bytes.extend_from_slice(&archived);
        bytes.extend_from_slice(&Sha256::digest(&bytes));
        let sha256 = format!("{:x}", Sha256::digest(&bytes));
        Ok(Self {
            bytes,
            archive_start,
            manifest,
            cut,
            sha256,
        })
    }

    /// Publish atomically in the destination directory without replacing a
    /// previous package. Successful return includes file and directory sync.
    pub fn publish(&self, path: &Path) -> io::Result<String> {
        let parent = directory(path);
        let dir = File::open(parent)?;
        let mut tmp = tempfile::NamedTempFile::new_in(parent)?;
        tmp.write_all(&self.bytes)?;
        tmp.as_file().sync_all()?;
        tmp.persist_noclobber(path).map_err(|e| e.error)?;
        dir.sync_all().map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("restart package published but directory sync failed: {e}"),
            )
        })?;
        Ok(self.sha256.clone())
    }

    /// Inspect the contents of a bounded local package without starting workers.
    /// Callers authorizing a restart should use `open` with an external digest.
    pub fn inspect(path: &Path) -> io::Result<Self> {
        Self::decode(read_regular(path, MAX_PACKAGE)?, None)
    }

    /// Verify the exact externally selected package before interpreting any of
    /// its authored scene or native data.
    pub fn open(path: &Path, expected_sha256: &str) -> io::Result<Self> {
        if expected_sha256.len() != 64 || !expected_sha256.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(invalid("expected a 64-digit package SHA-256"));
        }
        Self::decode(read_regular(path, MAX_PACKAGE)?, Some(expected_sha256))
    }

    fn decode(bytes: Vec<u8>, expected: Option<&str>) -> io::Result<Self> {
        let sha256 = format!("{:x}", Sha256::digest(&bytes));
        if expected.is_some_and(|s| !sha256.eq_ignore_ascii_case(s)) {
            return Err(invalid("restart package differs from the selected SHA-256"));
        }
        if bytes.len() < HEADER + CHECKSUM || &bytes[..8] != MAGIC {
            return Err(invalid("invalid restart package version or header"));
        }
        let manifest_len = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
        let archive_len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
        if manifest_len > MAX_MANIFEST
            || archive_len > MAX_ARCHIVE
            || bytes.len() != HEADER + manifest_len + archive_len + CHECKSUM
        {
            return Err(invalid("invalid restart package lengths"));
        }
        let end = bytes.len() - CHECKSUM;
        if bytes[end..] != Sha256::digest(&bytes[..end])[..] {
            return Err(invalid("restart package checksum mismatch"));
        }
        let archive_start = HEADER + manifest_len;
        let manifest = serde_json::from_slice(&bytes[HEADER..archive_start])?;
        let cut = verify_cut(&manifest, &bytes[archive_start..end])?;
        Ok(Self {
            bytes,
            archive_start,
            manifest,
            cut,
            sha256,
        })
    }

    pub fn sdf(&self) -> &str {
        &self.manifest.sdf
    }
    pub fn world(&self) -> &str {
        &self.manifest.world
    }
    pub fn drive(&self) -> &JointTorqueDrive {
        &self.manifest.drive
    }
    pub fn origin(&self) -> &ColdOrigin {
        &self.manifest.origin
    }
    pub fn cut(&self) -> &InspectedCut {
        &self.cut
    }
    pub fn archive(&self) -> &[u8] {
        &self.bytes[self.archive_start..self.bytes.len() - CHECKSUM]
    }
    pub fn sha256(&self) -> &str {
        &self.sha256
    }
}

#[cfg(test)]
mod tests;
