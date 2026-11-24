// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local session lock and a single atomically replaced, checksummed barrier.
use super::*;
use bincode::Options;
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    sync::Arc,
};
pub(crate) const HINGE_FILE: &str = "barrier.gzhinge";
pub(crate) const MOVING_FILE: &str = "barrier.gzmoving";
pub(crate) const MULTI_SPHERE_FILE: &str = "barrier.gzmultisphere";
pub(crate) const FLOATING_TREE_FILE: &str = "barrier.gzfloatingtree";
pub(crate) const STOPPED_TREE_FILE: &str = "barrier.gzstoppedtree";
pub(crate) const GROUNDED_ROBOT_FILE: &str = "barrier.gzgroundedrobot";
pub(crate) const ROTOR_MATERIAL_FILE: &str = "barrier.gzrotmaterial";
pub(crate) const AUTOMATIC_MATERIAL_FILE: &str = "barrier.gzautmaterial";
pub(crate) const MATERIAL_TREE_FILE: &str = "barrier.gzmaterialtree";
pub(crate) const CONTACT_TREE_FILE: &str = "barrier.gzcontacttree";
pub(crate) const MOTOR_TREE_FILE: &str = "barrier.gzmotorizedtree";
pub(crate) const GROUNDED_TREE_FILE: &str = "barrier.gzgroundedtree";
pub(crate) const ADAPTIVE_GRASP_FLEET_FILE: &str = "barrier.gzadaptivegraspfleet";
pub(crate) const COUPLED_GRASP_FILE: &str = "barrier.gzgraspcluster";
pub(crate) const GRASP_FLEET_FILE: &str = "barrier.gzgraspfleet";
pub(crate) const SPLIT_GRASP_FILE: &str = "barrier.gzsplitgrasp";
pub(crate) const GRASP_TREE_FILE: &str = "barrier.gzgrasptree";
pub(crate) const PAYLOAD_TREE_FILE: &str = "barrier.gzpayloadtree";

const SESSION: &[u8] = b"GZHNG-SESSION-1\n";
const MAGIC: &[u8; 8] = b"GZHNGP01";
const MAX_FILE: usize = 16 * 1024;
const GROUND_MAGIC: &[u8; 8] = b"GZHNGP02";
const GROUND_FILE_LIMIT: usize = MAX_ARTICULATION_IMAGE + MAX_FILE;

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Cut {
    pub width: u8,
    pub owner: u8,
    pub generation: u64,
    #[serde(default)]
    pub epoch: u64,
    pub spec: ArticulationSpec,
    pub observation: ArticulationObservation,
    pub image: Vec<u8>,
    // Derived from the authoritative native graph on reopen. The existing
    // warm/cold archive formats already carry this value inside the image.
    #[serde(skip)]
    pub pending_effort: Option<f64>,
}

/// A live handle pins the original marker even while the old coordinator is
/// gone, so remove/recreate cannot recycle its identity into a valid receipt.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct WriterIdentity {
    directory: PathBuf,
    marker: Arc<same_file::Handle>,
}

#[cfg(unix)]
impl WriterIdentity {
    pub(crate) fn directory(&self) -> &Path {
        &self.directory
    }
    pub(crate) fn metadata(&self) -> io::Result<fs::Metadata> {
        self.marker.as_file().metadata()
    }
}

pub(crate) struct Ledger {
    directory: PathBuf,
    controller: File,
}

impl Drop for Ledger {
    fn drop(&mut self) {
        // A concurrent fork temporarily shares this open-file description until
        // exec closes its CLOEXEC descriptor. Release the actual lease now.
        let _ = self.controller.unlock();
    }
}

impl Ledger {
    pub fn lock(directory: &Path, create: bool) -> io::Result<Self> {
        Self::acquire(directory, create, false)
    }

    /// A guarded operator reopen may only attach to a previously claimed session.
    pub(crate) fn lock_existing(directory: &Path) -> io::Result<Self> {
        Self::acquire(directory, false, true)
    }

    /// Observe an existing claim without taking ownership or changing staging.
    pub(crate) fn inspect_marker(directory: &Path) -> io::Result<WriterIdentity> {
        let directory = directory.canonicalize()?;
        let mut options = OpenOptions::new();
        options.read(true);
        #[cfg(unix)]
        {
            use rustix::fs::OFlags;
            use std::os::unix::fs::OpenOptionsExt;
            options.custom_flags((OFlags::NONBLOCK | OFlags::NOFOLLOW).bits() as i32);
        }
        let file = options.open(directory.join("controller.lock"))?;
        if !file.metadata()?.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "expected a regular controller lock",
            ));
        }
        let mut marker = Vec::new();
        (&file).take(64).read_to_end(&mut marker)?;
        if marker != SESSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "unknown hinge session lock",
            ));
        }
        Ok(WriterIdentity {
            directory,
            marker: Arc::new(same_file::Handle::from_file(file)?),
        })
    }

    fn acquire(directory: &Path, create: bool, require_claimed: bool) -> io::Result<Self> {
        if create {
            fs::create_dir_all(directory)?;
        }
        let directory = directory.canonicalize()?;
        let mut options = OpenOptions::new();
        options.read(true).write(true).create(!require_claimed);
        #[cfg(unix)]
        if require_claimed {
            use rustix::fs::OFlags;
            use std::os::unix::fs::OpenOptionsExt;
            options.custom_flags((OFlags::NONBLOCK | OFlags::NOFOLLOW).bits() as i32);
        }
        let mut file = options.open(directory.join("controller.lock"))?;
        if require_claimed && !file.metadata()?.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "expected an existing regular controller lock",
            ));
        }
        file.try_lock().map_err(|e| match e {
            fs::TryLockError::WouldBlock => io::Error::new(
                io::ErrorKind::WouldBlock,
                "another controller owns this hinge session",
            ),
            fs::TryLockError::Error(e) => e,
        })?;
        // Reserve the staging name before the first publish. An interrupted
        // publisher can then reclaim its own file even before the first cut.
        let mut marker = Vec::new();
        (&file).take(64).read_to_end(&mut marker)?;
        let pending = directory.join("barrier.pending.gzhinge");
        if marker.is_empty() && !require_claimed {
            if pending.try_exists()? {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    "unclaimed hinge staging name already exists",
                ));
            }
            file.write_all(SESSION)?;
            file.sync_all()?;
        } else if marker != SESSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "unknown hinge session lock",
            ));
        }
        let ledger = Self {
            directory,
            controller: file,
        };
        if !require_claimed {
            ledger.reclaim_staging()?;
        }
        Ok(ledger)
    }
    pub(crate) fn writer_identity(&self) -> io::Result<WriterIdentity> {
        Ok(WriterIdentity {
            directory: self.directory.clone(),
            marker: Arc::new(same_file::Handle::from_file(self.controller.try_clone()?)?),
        })
    }

    pub(crate) fn reclaim_staging(&self) -> io::Result<()> {
        // The exclusive controller owns this reserved name. A guarded reopener
        // leaves it available for inspection until its archive checks succeed.
        match fs::remove_file(self.directory.join("barrier.pending.gzhinge")) {
            Ok(()) => Ok(()),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(e) => Err(e),
        }
    }
    pub(crate) fn archive_path(&self, file: &str) -> PathBuf {
        self.directory.join(file)
    }
    /// A trusted session directory admits a single durable profile.
    pub(crate) fn other_archive_exists(&self, own: &str) -> io::Result<bool> {
        for file in [
            HINGE_FILE,
            MOVING_FILE,
            MULTI_SPHERE_FILE,
            FLOATING_TREE_FILE,
            STOPPED_TREE_FILE,
            MOTOR_TREE_FILE,
            CONTACT_TREE_FILE,
            MATERIAL_TREE_FILE,
            AUTOMATIC_MATERIAL_FILE,
            ROTOR_MATERIAL_FILE,
            GROUNDED_ROBOT_FILE,
            GROUNDED_TREE_FILE,
            PAYLOAD_TREE_FILE,
            GRASP_TREE_FILE,
            SPLIT_GRASP_FILE,
            GRASP_FLEET_FILE,
            COUPLED_GRASP_FILE,
            ADAPTIVE_GRASP_FLEET_FILE,
        ] {
            if file != own && self.archive_path(file).try_exists()? {
                return Ok(true);
            }
        }
        Ok(false)
    }
    pub(crate) fn multi_path(&self) -> PathBuf {
        self.archive_path(MULTI_SPHERE_FILE)
    }
    pub(crate) fn moving_path(&self) -> PathBuf {
        self.archive_path(MOVING_FILE)
    }
    fn path(&self) -> PathBuf {
        self.archive_path(HINGE_FILE)
    }
    pub fn exists(&self) -> io::Result<bool> {
        self.path().try_exists()
    }
    pub fn slot(&self, slot: u8) -> PathBuf {
        self.directory.join(format!("worker-{slot}.lock"))
    }

    pub(super) fn publish(&self, cut: &Cut) -> io::Result<()> {
        // Retain the exact existing JSON file for isolated sessions. The packed
        // ground profile has a larger solver image and a separate discriminator.
        let (magic, limit, data) = if cut.spec.ground.is_some() {
            let bytes = bincode::options()
                .with_fixint_encoding()
                .with_limit((GROUND_FILE_LIMIT - 40) as u64)
                .serialize(cut)
                .map_err(io::Error::other)?;
            (GROUND_MAGIC, GROUND_FILE_LIMIT, bytes)
        } else {
            (MAGIC, MAX_FILE, serde_json::to_vec(cut)?)
        };
        if data.len() + 40 > limit {
            return Err(io::Error::other("articulation cut exceeds size limit"));
        }
        self.publish_bytes(&self.path(), magic, &data, |_edge| {
            #[cfg(test)]
            super::crash_tests::gate(_edge);
        })
    }

    pub(crate) fn publish_bytes(
        &self,
        destination: &Path,
        magic: &[u8; 8],
        data: &[u8],
        boundary: impl Fn(&str),
    ) -> io::Result<()> {
        let mut temp = tempfile::Builder::new()
            .prefix("barrier.pending.gzhinge")
            .rand_bytes(0)
            .tempfile_in(&self.directory)?;
        temp.write_all(magic)?;
        temp.write_all(&Sha256::digest(data))?;
        temp.write_all(data)?;
        temp.as_file().sync_all()?;
        boundary("before-rename");
        temp.persist(destination)?;
        boundary("after-rename");
        File::open(&self.directory)?.sync_all()
    }

    pub(super) fn read(&self) -> Result<Cut> {
        let mut data = Vec::new();
        File::open(self.path())?
            .take((GROUND_FILE_LIMIT + 1) as u64)
            .read_to_end(&mut data)?;
        if data.len() > GROUND_FILE_LIMIT
            || data.len() < 40
            || data[8..40] != Sha256::digest(&data[40..])[..]
        {
            return Err(ArticulationError::Invalid(
                "invalid durable hinge checksum or format",
            ));
        }
        let mut cut: Cut = match &data[..8] {
            magic if magic == MAGIC && data.len() <= MAX_FILE => {
                serde_json::from_slice(&data[40..]).map_err(io::Error::from)?
            }
            magic if magic == GROUND_MAGIC => bincode::options()
                .with_fixint_encoding()
                .with_limit((GROUND_FILE_LIMIT - 40) as u64)
                .reject_trailing_bytes()
                .deserialize(&data[40..])
                .map_err(io::Error::other)?,
            _ => {
                return Err(ArticulationError::Invalid(
                    "wrong hinge file profile or size",
                ));
            }
        };
        if cut.spec.ground.is_some() != (&data[..8] == GROUND_MAGIC) {
            return Err(ArticulationError::Invalid(
                "hinge recipe differs from durable file profile",
            ));
        }
        if !(2..=8).contains(&cut.width)
            || cut.owner >= cut.width
            || cut.epoch > cut.generation
            || cut.image.len() > cut.spec.image_limit()
        {
            return Err(ArticulationError::Invalid(
                "invalid durable hinge ownership or size",
            ));
        }
        let native = NativeArticulation::restore(&cut.spec, cut.observation.tick, &cut.image)?;
        if native.observe() != cut.observation {
            return Err(ArticulationError::Invalid(
                "native hinge differs from published observation",
            ));
        }
        cut.pending_effort = native.pending_joint_effort();
        Ok(cut)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::articulation::native::tests::spec;

    #[test]
    fn grounded_cut_preserves_reactions_and_rejects_cross_profile_or_forged_observations()
    -> Result<()> {
        let dir = tempfile::tempdir()?;
        let ledger = Ledger::lock(dir.path(), true)?;
        let mut spec = spec();
        spec.ground = Some(ArticulationGroundSpec {
            link_id: 1,
            height: 0.85,
            size: [8., 5.],
            gravity_z: -9.81,
        });
        let mut native = NativeArticulation::new(&spec)?;
        let mut cut = Cut {
            width: 2,
            owner: 0,
            generation: 0,
            epoch: 0,
            observation: native.observe(),
            pending_effort: native.pending_joint_effort(),
            image: native.capture()?,
            spec,
        };
        ledger.publish(&cut)?;
        assert_eq!(ledger.read()?.image, cut.image);
        for _ in 0..100 {
            native.advance()?;
            if native
                .observe()
                .ground_reactions
                .unwrap()
                .iter()
                .any(|g| g.normal_impulse > 0.)
            {
                break;
            }
        }
        native.effort(spec.joint_id, 0.625)?;
        cut.observation = native.observe();
        cut.image = native.capture()?;
        assert!(
            cut.observation
                .ground_reactions
                .unwrap()
                .iter()
                .any(|r| r.active && r.normal_impulse > 0.)
        );
        ledger.publish(&cut)?;
        let reopened = ledger.read()?;
        assert_eq!(cut.observation, reopened.observation);
        assert_eq!(cut.image, reopened.image);
        let mut bytes = fs::read(ledger.path())?;
        assert_eq!(&bytes[..8], GROUND_MAGIC);
        assert!(bytes.len() < cut.image.len() + 2048);
        bytes[..8].copy_from_slice(MAGIC);
        fs::write(ledger.path(), bytes)?;
        assert!(ledger.read().is_err());
        cut.observation.ground_reactions.as_mut().unwrap()[0].normal_impulse += 0.125;
        ledger.publish(&cut)?;
        assert!(ledger.read().is_err());
        Ok(())
    }

    #[test]
    fn fresh_session_preserves_unclaimed_pending_file() -> io::Result<()> {
        let dir = tempfile::tempdir()?;
        let path = dir.path().join("barrier.pending.gzhinge");
        fs::write(&path, b"preexisting")?;
        assert_eq!(
            Ledger::lock(dir.path(), true).err().unwrap().kind(),
            io::ErrorKind::AlreadyExists
        );
        assert_eq!(fs::read(path)?, b"preexisting");
        Ok(())
    }

    #[test]
    fn durable_cut_rejects_corruption_and_inconsistent_checked_metadata() -> Result<()> {
        let root = tempfile::tempdir()?;
        let ledger = Ledger::lock(root.path(), true)?;
        let spec = spec();
        let mut native = NativeArticulation::new(&spec)?;
        native.effort(spec.joint_id, 2.)?;
        native.advance()?;
        let cut = Cut {
            width: 3,
            owner: 2,
            generation: 4,
            epoch: 1,
            spec,
            observation: native.observe(),
            pending_effort: native.pending_joint_effort(),
            image: native.capture()?,
        };
        ledger.publish(&cut)?;
        assert_eq!(ledger.read()?.image, cut.image);
        for invalid in [
            Cut {
                width: 1,
                ..cut.clone()
            },
            Cut {
                owner: 3,
                ..cut.clone()
            },
            Cut {
                epoch: 5,
                ..cut.clone()
            },
            Cut {
                observation: ArticulationObservation {
                    qdot: cut.observation.qdot + 0.25,
                    ..cut.observation
                },
                ..cut.clone()
            },
            Cut {
                observation: ArticulationObservation {
                    tick: 2,
                    ..cut.observation
                },
                ..cut.clone()
            },
        ] {
            ledger.publish(&invalid)?;
            assert!(ledger.read().is_err());
        }
        let mut valid = fs::read(ledger.path())?;
        valid[18] ^= 1;
        fs::write(ledger.path(), valid)?;
        assert!(ledger.read().is_err());
        fs::write(ledger.path(), vec![0; MAX_FILE + 1])?;
        assert!(ledger.read().is_err());
        Ok(())
    }
}

#[cfg(test)]
mod controller_lease_tests {
    use super::*;
    #[test]
    fn releasing_controller_explicitly_unlocks_while_forked_description_survives() {
        let d = tempfile::tempdir().unwrap();
        let first = Ledger::lock(d.path(), true).unwrap();
        let inherited = first.controller.try_clone().unwrap();
        assert!(
            matches!(Ledger::lock(d.path(),false),Err(error) if error.kind()==io::ErrorKind::WouldBlock)
        );
        drop(first);
        let next = Ledger::lock(d.path(), false).unwrap();
        drop(inherited);
        assert!(
            matches!(Ledger::lock(d.path(),false),Err(error) if error.kind()==io::ErrorKind::WouldBlock)
        );
        drop(next);
        assert!(Ledger::lock(d.path(), false).is_ok());
    }
}
