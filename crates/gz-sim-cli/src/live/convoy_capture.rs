// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local signal capture of the authored drive actually installed in the Bevy world.
use super::{ConvoyCapsule, JointTorqueDrive};
use bevy_ecs::prelude::Resource;
use gz_sim_core::Simulation;
use std::{
    io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

struct Signal {
    pending: Arc<AtomicBool>,
    #[cfg(unix)]
    id: signal_hook::SigId,
}
#[cfg(unix)]
impl Drop for Signal {
    fn drop(&mut self) {
        signal_hook::low_level::unregister(self.id);
    }
}

/// An opt-in SIGUSR1 request shared across worker reinstalls in this supervisor.
/// File I/O and physics inspection occur on the simulation thread at a committed boundary.
#[derive(Clone)]
pub struct ConvoyCaptureRequest {
    directory: PathBuf,
    signal: Arc<Signal>,
}

impl ConvoyCaptureRequest {
    pub fn new(directory: &Path) -> io::Result<Self> {
        #[cfg(unix)]
        {
            let directory = directory.canonicalize()?;
            if !directory.is_dir() {
                return Err(io::Error::other("expected a checkpoint output directory"));
            }
            let pending = Arc::new(AtomicBool::new(false));
            let id = signal_hook::flag::register(signal_hook::consts::SIGUSR1, pending.clone())?;
            Ok(Self {
                directory,
                signal: Arc::new(Signal { pending, id }),
            })
        }
        #[cfg(not(unix))]
        {
            let _ = directory;
            Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "SIGUSR1 capture requires Unix",
            ))
        }
    }

    /// Attach the signal to this installation. Requests received while workers
    /// were unavailable do not carry forward to a later installation.
    pub fn attach(&self, sim: &mut Simulation, session: &Path, sdf: &str, world: &str) {
        self.signal.pending.store(false, Ordering::Release);
        sim.world_mut().insert_resource(LiveCapture {
            request: self.clone(),
            archive: session.join("barrier.gzmultisphere"),
            sdf: Arc::from(sdf),
            world: Arc::from(world),
        });
    }
}

#[derive(Clone, Resource)]
pub(super) struct LiveCapture {
    request: ConvoyCaptureRequest,
    archive: PathBuf,
    sdf: Arc<str>,
    world: Arc<str>,
}

impl LiveCapture {
    pub(super) fn poll(&self, sim: &Simulation) {
        if !self.request.signal.pending.swap(false, Ordering::AcqRel) {
            return;
        }
        match self.save(sim) {
            Ok(path) => eprintln!("Published supervisor checkpoint {}", path.display()),
            Err(error) => eprintln!("Supervisor checkpoint capture refused: {error}"),
        }
    }

    fn save(&self, sim: &Simulation) -> io::Result<PathBuf> {
        let cap = ConvoyCapsule::capture_installed(sim, &self.archive, &self.sdf, &self.world)?;
        let cut = &cap.cut().cut;
        let path = self.request.directory.join(format!(
            "{}-{}-{}.gzcap",
            cut.epoch,
            cut.observation.hinge.tick,
            cap.sha256()
        ));
        match cap.publish(&path) {
            Ok(_) => Ok(path),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                ConvoyCapsule::open(&path, cap.sha256())?;
                // A preceding publish may have reported a directory-sync error.
                std::fs::File::open(&self.request.directory)?.sync_all()?;
                Ok(path)
            }
            Err(e) => Err(e),
        }
    }
}

impl ConvoyCapsule {
    /// Capture the actual drive and full retained native cut from an installed
    /// Bevy convoy. Even a valid replacement file at the same clock must match
    /// the current worker pool before it can be published as this live cut.
    pub fn capture_installed(
        sim: &Simulation,
        archive: &Path,
        sdf: &str,
        world: &str,
    ) -> io::Result<Self> {
        use gz_physics::PhysicsBackend;
        use gz_sim_scene::distributed_hinge::DistributedMultiSphereBackend;
        let drive = sim
            .world()
            .get_resource::<JointTorqueDrive>()
            .ok_or_else(|| io::Error::other("no installed convoy drive"))?;
        let backend = sim
            .world()
            .get_resource::<PhysicsBackend>()
            .and_then(|b| b.downcast_ref::<DistributedMultiSphereBackend>())
            .ok_or_else(|| io::Error::other("expected installed two-sphere worker pool"))?;
        let cap = Self::capture(archive, sdf, world, drive)?;
        let cut = &cap.cut().cut;
        if sim.epoch() != cut.epoch
            || sim.elapsed() != std::time::Duration::from_millis(cut.observation.hinge.tick)
        {
            return Err(io::Error::other(
                "Bevy clock differs from committed checkpoint",
            ));
        }
        backend
            .pool()
            .verify_live_cut(cap.origin(), cap.archive())
            .map_err(io::Error::other)?;
        Ok(cap)
    }
}
