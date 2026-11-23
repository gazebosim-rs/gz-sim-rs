// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One operator-selected local destination. No network path or new wire message.
use gz_sim_server::Server;
#[cfg(all(unix, feature = "distributed-physics"))]
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::{io, path::PathBuf};

pub struct OperatorCheckpoint {
    #[cfg(all(unix, feature = "distributed-physics"))]
    path: PathBuf,
    #[cfg(all(unix, feature = "distributed-physics"))]
    requested: Arc<AtomicBool>,
    #[cfg(all(unix, feature = "distributed-physics"))]
    signal: signal_hook::SigId,
}
impl OperatorCheckpoint {
    /// Register only when the operator explicitly supplies a destination.
    pub fn install(path: PathBuf) -> io::Result<Self> {
        #[cfg(all(unix, feature = "distributed-physics"))]
        {
            let requested = Arc::new(AtomicBool::new(false));
            let signal =
                signal_hook::flag::register(signal_hook::consts::SIGUSR1, Arc::clone(&requested))?;
            Ok(Self {
                path,
                requested,
                signal,
            })
        }
        #[cfg(not(all(unix, feature = "distributed-physics")))]
        {
            let _ = path;
            Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "live pool checkpoint signals require Unix and the distributed-physics Cargo feature",
            ))
        }
    }
    pub(super) fn announce(&self) {
        #[cfg(all(unix, feature = "distributed-physics"))]
        eprintln!(
            "live pool checkpoint armed pid={} signal=SIGUSR1 path={:?}",
            std::process::id(),
            self.path
        );
    }
    pub(super) fn poll(&self, server: &mut Server) {
        #[cfg(all(unix, feature = "distributed-physics"))]
        if self.requested.swap(false, Ordering::AcqRel) {
            let info = *server
                .simulation()
                .world()
                .resource::<gz_sim_core::UpdateInfo>();
            match server.save_paused_pool_checkpoint(&self.path) {
                Ok(()) => eprintln!(
                    "live pool checkpoint saved epoch={} tick={} path={:?}",
                    info.epoch, info.iteration, self.path
                ),
                Err(error) => eprintln!(
                    "live pool checkpoint refused epoch={} tick={} path={:?} reason={error}",
                    info.epoch, info.iteration, self.path
                ),
            }
        }
        #[cfg(not(all(unix, feature = "distributed-physics")))]
        let _ = server;
    }
}
#[cfg(all(unix, feature = "distributed-physics"))]
impl Drop for OperatorCheckpoint {
    fn drop(&mut self) {
        signal_hook::low_level::unregister(self.signal);
    }
}
