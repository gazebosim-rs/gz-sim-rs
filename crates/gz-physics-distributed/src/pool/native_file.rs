// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable, same-build native barriers for a fixed local worker pool.
use super::*;
use crate::{
    coordinator::{RecoveryError, read_verified, write_immutable},
    protocol::{Worker, WorkerImage},
};
use serde::{Deserialize, Serialize};

const SCENE_MAGIC: &[u8; 8] = b"GZSCN001";
const POPULATION_SCENE_MAGIC: &[u8; 8] = b"GZSCN002";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScenePopulation {
    Loaded,
    Edited,
}
impl ScenePopulation {
    fn magic(self) -> &'static [u8; 8] {
        match self {
            Self::Loaded => SCENE_MAGIC,
            Self::Edited => POPULATION_SCENE_MAGIC,
        }
    }
}
/// Bounds the application-specific manifest independently of native image storage.
pub const MAX_SCENE_METADATA_BYTES: usize = 512 * 1024;
const MAGIC: &[u8; 8] = b"GZPOOL01";
const EXTENDED_MAGIC: &[u8; 8] = b"GZPOOL03";
const POINT_MAGIC: &[u8; 8] = b"GZPOOL04";
const EDITED_MAGIC: &[u8; 8] = b"GZPOOL02";

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Barrier {
    #[serde(default, skip_serializing_if = "NativeSolverProfile::is_standard")]
    profile: NativeSolverProfile,
    journal_limit: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    population_high_water: Option<u64>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    last_refined: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    last_island_ids: Option<Vec<u64>>,
    queued: Vec<PendingImpulse>,
    images: Vec<WorkerImage>,
}

/// A trusted same-build checkpoint of one complete three- to eight-worker run.
/// Opening verifies every native partition before it creates a worker process.
pub struct PoolNativeRecovery {
    barrier: Barrier,
    frame: PoolFrame,
}

struct ScenePayload<'a> {
    metadata: &'a [u8],
    magic: [u8; 8],
    native: &'a [u8],
}

impl<'a> ScenePayload<'a> {
    fn split(payload: &'a [u8]) -> Result<Self, RecoveryError> {
        let length = payload
            .first_chunk::<4>()
            .map(|b| u32::from_le_bytes(*b) as usize)
            .filter(|n| (1..=MAX_SCENE_METADATA_BYTES).contains(n))
            .ok_or(RecoveryError::Format("scene metadata length"))?;
        let bytes = &payload[4..];
        let (metadata, native) = bytes
            .split_at_checked(length)
            .ok_or(RecoveryError::Format("truncated scene metadata"))?;
        let (magic, native) = native
            .split_first_chunk::<8>()
            .ok_or(RecoveryError::Format("missing nested native pool"))?;
        Ok(Self {
            metadata,
            magic: *magic,
            native,
        })
    }
}

impl Barrier {
    fn magic(&self) -> &'static [u8; 8] {
        if self.profile.friction() == crate::physics::NativeFrictionModel::PerPoint {
            POINT_MAGIC
        } else if self.profile == NativeSolverProfile::Extended64 {
            EXTENDED_MAGIC
        } else if self.population_high_water.is_some() {
            EDITED_MAGIC
        } else {
            MAGIC
        }
    }
}

impl WorkerPool {
    /// Atomically publish one committed barrier to a new file in a trusted
    /// directory. Native caches, queued impulses and ownership fences survive.
    /// Per-step force and torque arguments belong to the caller until advance.
    pub fn save_native_recovery(&mut self, path: impl AsRef<Path>) -> Result<(), RecoveryError> {
        let barrier = self.native_barrier()?;
        write_immutable(path.as_ref(), barrier.magic(), |out| {
            serde_json::to_writer(out, &barrier)?;
            Ok(())
        })
    }

    pub(crate) fn save_scene_recovery(
        &mut self,
        path: &Path,
        metadata: &[u8],
        population: ScenePopulation,
    ) -> Result<(), RecoveryError> {
        if metadata.is_empty() || metadata.len() > MAX_SCENE_METADATA_BYTES {
            return Err(RecoveryError::Format("scene metadata size"));
        }
        let barrier = self.native_barrier()?;
        write_immutable(path, population.magic(), |out| {
            out.write_all(&(metadata.len() as u32).to_le_bytes())?;
            out.write_all(metadata)?;
            out.write_all(barrier.magic())?;
            serde_json::to_writer(out, &barrier)?;
            Ok(())
        })
    }

    fn native_barrier(&mut self) -> Result<Barrier, RecoveryError> {
        if self.placement_replay.attempt.is_some() {
            return Err(Error::Pending.into());
        }
        Ok(Barrier {
            profile: self.profile,
            images: self.checked_images()?,
            journal_limit: self.limit,
            population_high_water: self.population_high_water,
            last_refined: self.last_refined,
            last_island_ids: self.last_island_ids.clone(),
            queued: self.queued.clone(),
        })
    }
}

/// Geometry selected by one worker's final native solve at a public barrier.
#[derive(Debug, PartialEq, Serialize)]
pub struct WorkerContactPatches {
    pub finite: Vec<crate::physics::FiniteContactPatch>,
    pub ground: Vec<crate::physics::GroundContactPatch>,
}

/// Read-only contact diagnostics captured during the ordinary archive integrity checks.
pub struct NativeContactSnapshot {
    pub frame: PoolFrame,
    /// One entry per worker in the same order as `frame.workers`.
    pub patches: Vec<WorkerContactPatches>,
    /// Present only when the input was a public scene envelope.
    pub population: Option<ScenePopulation>,
}

impl NativeContactSnapshot {
    /// Read a native pool or public scene file. Each saved worker is hydrated once.
    /// The frame contains complete public intervals. Patches reflect each solver's
    /// last native substep, which may be shorter in a refined interval.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RecoveryError> {
        let (magic, bytes) = read_verified(
            path.as_ref(),
            &[
                MAGIC,
                EDITED_MAGIC,
                EXTENDED_MAGIC,
                POINT_MAGIC,
                SCENE_MAGIC,
                POPULATION_SCENE_MAGIC,
            ],
        )?;
        let (magic, native, population) =
            if magic == *SCENE_MAGIC || magic == *POPULATION_SCENE_MAGIC {
                let nested = ScenePayload::split(&bytes)?;
                let population = if magic == *SCENE_MAGIC {
                    ScenePopulation::Loaded
                } else {
                    ScenePopulation::Edited
                };
                (nested.magic, nested.native, Some(population))
            } else {
                (magic, bytes.as_slice(), None)
            };
        let mut patches = Vec::new();
        let plan = PoolNativeRecovery::from_payload(magic, native, |worker| {
            patches.push(WorkerContactPatches {
                finite: worker.finite_contact_patches(),
                ground: worker.ground_contact_patches(),
            });
        })?;
        Ok(Self {
            frame: plan.frame,
            patches,
            population,
        })
    }
}

impl PoolNativeRecovery {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RecoveryError> {
        let (magic, payload) = read_verified(
            path.as_ref(),
            &[MAGIC, EDITED_MAGIC, EXTENDED_MAGIC, POINT_MAGIC],
        )?;
        Self::from_payload(magic, &payload, |_| {})
    }

    /// Open one bounded, immutable application manifest and its native pool.
    /// Both parts share the same atomic publication and integrity check.
    pub fn open_scene(path: impl AsRef<Path>) -> Result<(Vec<u8>, Self), RecoveryError> {
        let (population, metadata, plan) = Self::open_scene_manifest(path)?;
        if population != ScenePopulation::Loaded {
            return Err(RecoveryError::Format(
                "edited population requires a versioned scene reader",
            ));
        }
        Ok((metadata, plan))
    }

    pub fn open_scene_manifest(
        path: impl AsRef<Path>,
    ) -> Result<(ScenePopulation, Vec<u8>, Self), RecoveryError> {
        let (magic, payload) =
            read_verified(path.as_ref(), &[SCENE_MAGIC, POPULATION_SCENE_MAGIC])?;
        let population = if magic == *SCENE_MAGIC {
            ScenePopulation::Loaded
        } else {
            ScenePopulation::Edited
        };
        let nested = ScenePayload::split(&payload)?;
        let plan = Self::from_payload(nested.magic, nested.native, |_| {})?;
        Ok((population, nested.metadata.to_vec(), plan))
    }

    fn from_payload(
        magic: [u8; 8],
        payload: &[u8],
        mut inspect: impl FnMut(&Worker),
    ) -> Result<Self, RecoveryError> {
        let barrier: Barrier = serde_json::from_slice(payload)?;
        if magic != *barrier.magic() {
            return Err(RecoveryError::Format(
                "pool profile or population allocator version",
            ));
        }
        let width = barrier.images.len();
        if !(3..=8).contains(&width) || barrier.journal_limit == 0 {
            return Err(RecoveryError::Format("pool width or journal capacity"));
        }
        let first = &barrier.images[0];
        let mut workers = Vec::with_capacity(width);
        let mut latest_release = BTreeMap::<u64, u64>::new();
        let mut releases = BTreeSet::new();
        let mut serial_counts = BTreeMap::<u64, u8>::new();
        for (w, image) in barrier.images.iter().enumerate() {
            if image.native_solver_profile() != barrier.profile {
                return Err(RecoveryError::Format("inconsistent worker native profile"));
            }
            if image.sequence() == 0 {
                return Err(RecoveryError::Format("empty pool command sequence"));
            }
            let worker = Worker::from_scoped_image(first.run, w as u8, image, width as u8)
                .map_err(|_| RecoveryError::Format("pool native identity or image"))?;
            if image.pending().iter().ne(barrier
                .queued
                .iter()
                .filter(|p| image.generations.contains_key(&p.id)))
            {
                return Err(RecoveryError::Format("pool input order or ownership"));
            }
            if image.serial > 0 {
                let count = serial_counts.entry(image.serial).or_default();
                *count += 1;
                if *count > 2 {
                    return Err(RecoveryError::Format(
                        "pool transfer has more than two participants",
                    ));
                }
            }
            if image
                .generations
                .values()
                .chain(image.retired.values())
                .any(|&g| g > image.serial)
            {
                return Err(RecoveryError::Format(
                    "pool ownership generation exceeds its transfer clock",
                ));
            }
            for (&id, &generation) in &image.retired {
                if !releases.insert((id, generation)) {
                    return Err(RecoveryError::Format("duplicate pool release generation"));
                }
                let latest = latest_release.entry(id).or_default();
                *latest = (*latest).max(generation);
            }
            inspect(&worker);
            workers.push(worker.observation());
        }
        // A later transfer can supersede either participant's earlier serial.
        // The final transfer must still be visible on both of its endpoints.
        if serial_counts
            .last_key_value()
            .is_some_and(|(_, count)| *count != 2)
        {
            return Err(RecoveryError::Format(
                "missing participant in final pool transfer",
            ));
        }
        let frame = PoolFrame {
            tick: first.tick(),
            workers,
        };
        frame.check(barrier.profile)?;
        if !crate::adaptive::valid_last_interval(
            frame.tick,
            frame.step(),
            barrier.last_refined,
            barrier.last_island_ids.as_deref(),
            frame.workers.len(),
        ) {
            return Err(RecoveryError::Format("invalid adaptive interval"));
        }
        if let Some(high) = barrier.population_high_water {
            // Last-step refinement may name a body retired later at the same tick.
            if frame.highest_body_id().is_some_and(|id| id > high)
                || barrier
                    .last_island_ids
                    .as_ref()
                    .and_then(|ids| ids.last())
                    .is_some_and(|&id| id > high)
            {
                return Err(RecoveryError::Format(
                    "pool population allocator behind worker state",
                ));
            }
        } else if barrier
            .last_island_ids
            .as_ref()
            .is_some_and(|ids| ids.iter().any(|&id| frame.body(id).is_none()))
        {
            return Err(RecoveryError::Format("unknown body in island interval"));
        }
        for obs in &frame.workers {
            for (&id, &generation) in &obs.generations {
                if latest_release.remove(&id).unwrap_or(0) != generation {
                    return Err(RecoveryError::Format("inconsistent pool ownership fence"));
                }
            }
        }
        if !latest_release.is_empty() || barrier.queued.iter().any(|p| frame.body(p.id).is_none()) {
            return Err(RecoveryError::Format("orphan pool fence or input"));
        }
        Ok(Self { barrier, frame })
    }

    /// Validate global next-interval identity, force accumulation and forecast
    /// against the exact saved impulse order before starting any worker.
    pub(crate) fn validate_pending_scene_field(
        &self,
        gravity: [f64; 3],
        held: &[BodyWrench],
    ) -> Result<(), RecoveryError> {
        super::routing::ProspectiveGroups::new(
            &self.frame,
            gravity,
            &self.barrier.queued,
            held,
            self.barrier.profile,
        )?;
        Ok(())
    }

    /// Highest native ID issued in the current native population history.
    pub fn population_high_water(&self) -> Option<u64> {
        self.barrier.population_high_water
    }

    pub fn native_solver_profile(&self) -> NativeSolverProfile {
        self.barrier.profile
    }
    /// Rehydrate saved workers on demand and return their authoritative shared-plane
    /// points. With refinement these impulses cover only the final native substep.
    pub fn ground_contact_patches(
        &self,
    ) -> Result<Vec<Vec<crate::physics::GroundContactPatch>>, RecoveryError> {
        let images = &self.barrier.images;
        images
            .iter()
            .enumerate()
            .map(|(i, image)| {
                Worker::from_scoped_image(image.run, i as u8, image, images.len() as u8)
                    .map(|worker| worker.ground_contact_patches())
                    .map_err(|_| RecoveryError::Format("invalid saved native ground patch"))
            })
            .collect()
    }
    /// Inspect the authoritative finite contact patches saved for each owner.
    /// Rehydrates native state locally on demand. The stored normal impulses
    /// describe the final native substep when the public interval was refined.
    pub fn finite_contact_patches(
        &self,
    ) -> Result<Vec<Vec<crate::physics::FiniteContactPatch>>, RecoveryError> {
        let images = &self.barrier.images;
        images
            .iter()
            .enumerate()
            .map(|(i, image)| {
                Worker::from_scoped_image(image.run, i as u8, image, images.len() as u8)
                    .map(|worker| worker.finite_contact_patches())
                    .map_err(|_| RecoveryError::Format("invalid saved native contact patch"))
            })
            .collect()
    }

    pub fn frame(&self) -> &PoolFrame {
        &self.frame
    }

    /// Resume after fencing the former coordinator. Each child opens a new local
    /// endpoint. Read back its pinned native image before exposing the pool.
    pub fn resume(self, executable: impl AsRef<Path>) -> Result<WorkerPool, RecoveryError> {
        let Barrier {
            profile,
            journal_limit,
            population_high_water,
            last_refined,
            last_island_ids,
            queued,
            images,
        } = self.barrier;
        let width = images.len();
        let executable = executable.as_ref();
        let peers = super::startup::start_in_pairs(&images, |_, image| {
            WorkerProcess::spawn_from_pool_image(executable, width as u8, image)
        })?;
        let mut pool = WorkerPool {
            profile,
            local_helper_limit: None,
            last_helper_mask: None,
            pending_step: None,
            placement_replay: Default::default(),
            handoff_confirmation: None,
            step_confirmation: crate::transport::REQUEST_TIMEOUT,
            periodic: None,
            active_probe: None,
            local_lanes: std::thread::available_parallelism().map_or(1, usize::from),
            last_refined,
            last_island_ids,
            run: images[0].run,
            population_high_water,
            frame: self.frame,
            peers,
            queued,
            limit: journal_limit,
            history: (0..width).map(|_| Journal::default()).collect(),
            baseline: images.iter().map(WorkerImage::sequence).collect(),
            serials: images.iter().map(|i| i.serial).collect(),
            serial: images.iter().map(|i| i.serial).max().unwrap(),
            retired: images.iter().map(|i| i.retired.clone()).collect(),
            dirty: vec![false; width],
        };
        if pool
            .checked_images()?
            .iter()
            .zip(&images)
            .any(|(a, b)| a.digest() != b.digest())
        {
            return Err(RecoveryError::Divergence);
        }
        Ok(pool)
    }
}
