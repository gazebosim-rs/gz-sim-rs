// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{native::ClusterWorld, *};
pub(super) use crate::paged_native::{Answer, Op, Req};
use crate::paged_native::{PagedWorld, State};
use std::{io, path::Path};
impl PagedWorld for ClusterWorld {
    type Spec = GraspFleetSpec;
    type Version = Version;
    const LIMIT: usize = SNAP_LIMIT;
    fn restore_bytes(spec: &Self::Spec, version: Version, tick: u64, bytes: &[u8]) -> Result<Self> {
        let snap: ClusterSnapshot = codec(SNAP_LIMIT)
            .deserialize(bytes)
            .map_err(io::Error::other)?;
        if snap.tick != tick {
            return Err(invalid("coupled fleet stage tick differs"));
        }
        Self::restore(spec, &snap, version)
    }
    fn capture_bytes(&self, tick: u64) -> Result<Vec<u8>> {
        let snap = self.capture()?;
        if snap.tick != tick {
            return Err(invalid("coupled native clock differs from worker"));
        }
        Ok(codec(SNAP_LIMIT)
            .serialize(&snap)
            .map_err(io::Error::other)?)
    }
    fn queue(&mut self, id: u64, e: f64) -> Result<()> {
        self.queue(id, e)
    }
    fn motor(&mut self, id: u64, m: Option<MotorSetting>) -> Result<()> {
        self.motor(id, m)
    }
    fn advance(&mut self, efforts: &[(u64, f64)], forces: &[(u64, [f64; 3])]) -> Result<()> {
        self.advance(efforts, forces)
    }
}
/// Serve the full two-gripper native graph over bounded replayable receipt pages.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_version(slot, lease, Version::Legacy)
}
/// Serve the complete free–free shear observation for newly created coupled worlds.
pub fn serve_v2(slot: u8, lease: &Path) -> io::Result<()> {
    serve_version(slot, lease, Version::ObservedShear)
}
fn serve_version(slot: u8, lease: &Path, version: Version) -> io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(io::Error::other("invalid coupled grasp worker slot"));
    }
    crate::moving::worker::serve_profile(
        slot,
        lease,
        version.wire(),
        move |spec: GraspFleetSpec| {
            admissible(&spec)?;
            let mut state = State::<ClusterWorld>::default();
            Ok(move |tick, req, steps: &mut u64| state.handle(&spec, version, tick, req, steps))
        },
    )
}
