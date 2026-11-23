// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{disk, *};
use crate::articulation::ledger::{Ledger, WriterIdentity};

/// Origin of an existing Unix session in its trusted local filesystem namespace.
/// The marker's change time conservatively invalidates a captured package if the
/// marker is edited, replaced or relinked. Physics publication never writes it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ColdOrigin {
    directory: PathBuf,
    device: u64,
    inode: u64,
    change_seconds: i64,
    change_nanoseconds: i64,
}

impl ColdOrigin {
    /// Inspect the committed marker through one descriptor, including while a
    /// live coordinator holds the writer lease. This does not create a claim.
    pub fn capture(directory: &Path) -> io::Result<Self> {
        Self::from_writer(&Ledger::inspect_marker(directory)?)
    }

    pub(super) fn check_disk(&self, disk: &disk::Disk) -> Result<()> {
        if Self::from_writer(&disk.writer_identity()?)? != *self {
            return Err(invalid(
                "cold checkpoint origin differs from the active writer",
            ));
        }
        Ok(())
    }

    #[cfg(unix)]
    fn from_writer(writer: &WriterIdentity) -> io::Result<Self> {
        use std::os::unix::fs::MetadataExt;
        let metadata = writer.metadata()?;
        Ok(Self {
            directory: writer.directory().into(),
            device: metadata.dev(),
            inode: metadata.ino(),
            change_seconds: metadata.ctime(),
            change_nanoseconds: metadata.ctime_nsec(),
        })
    }

    #[cfg(not(unix))]
    fn from_writer(_: &WriterIdentity) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "cold origin capture requires Unix file identity",
        ))
    }
}

/// A fresh replacement supervisor's selection. The temporary inspection lease
/// is released on return. The pinned regular-file handle prevents marker inode
/// reuse between selection and operator admission.
#[derive(Debug)]
pub struct ColdSelection {
    cut: InspectedCut,
    origin: ColdOrigin,
    writer: WriterIdentity,
}

impl ColdSelection {
    pub fn inspect(directory: &Path, expected_sha256: &str, origin: &ColdOrigin) -> Result<Self> {
        let disk = disk::Disk::lock_existing(directory)?;
        let writer = disk.writer_identity()?;
        if ColdOrigin::from_writer(&writer)? != *origin {
            return Err(invalid("cold checkpoint origin differs from this session"));
        }
        let (spec, cut, sha256) = disk.read_identified()?;
        if sha256 != expected_sha256 {
            return Err(invalid(
                "cold checkpoint digest differs from selected archive",
            ));
        }
        native::verify_restart(&spec, &cut)?;
        Ok(Self {
            cut: disk::inspected(spec, cut, sha256),
            writer,
            origin: origin.clone(),
        })
    }

    pub fn cut(&self) -> &InspectedCut {
        &self.cut
    }

    pub(super) fn writer(&self) -> &WriterIdentity {
        &self.writer
    }

    pub(super) fn check_origin(&self, disk: &disk::Disk) -> Result<()> {
        self.origin.check_disk(disk)
    }
}
