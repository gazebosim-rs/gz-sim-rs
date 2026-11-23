// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Idempotent manual execution placement at a stamped Gazebo barrier.
use gz_sim_core::Simulation;
use gz_sim_proto::prost::{Enumeration, Message};
use gz_sim_scene::gpu_scene::{
    BatchBoundary, ExecutionPlacement, PairSolver, batch_boundary, execution_counts,
    set_environment_placement, set_pair_solver,
};
pub const REQUEST_TYPE: &str = "gz.sim.rs.EnvironmentPlacement";
pub const REPLY_TYPE: &str = "gz.sim.rs.EnvironmentPlacementAck";
pub const MAX_IDS: usize = 256;
pub(crate) const MAX_REQUEST_BYTES: usize = MAX_IDS * 10 + 64;
#[derive(Debug, Clone, Copy, PartialEq, Eq, Enumeration)]
#[prost(prost_path = "gz_sim_proto::prost")]
#[repr(i32)]
pub enum Target {
    Unspecified = 0,
    Cpu = 1,
    Gpu = 2,
    NativeRapier = 3,
    GuardedGpu = 4,
}
#[derive(Clone, PartialEq, Message)]
#[prost(prost_path = "gz_sim_proto::prost")]
pub struct EnvironmentPlacement {
    #[prost(uint64, optional, tag = "1")]
    pub epoch: Option<u64>,
    #[prost(uint64, optional, tag = "2")]
    pub tick: Option<u64>,
    #[prost(uint64, repeated, tag = "3")]
    pub environments: Vec<u64>,
    #[prost(enumeration = "Target", tag = "4")]
    pub target: i32,
}
#[derive(Clone, PartialEq, Message)]
#[prost(prost_path = "gz_sim_proto::prost")]
pub struct EnvironmentPlacementAck {
    #[prost(bool, tag = "1")]
    pub accepted: bool,
    #[prost(uint64, tag = "2")]
    pub epoch: u64,
    #[prost(uint64, tag = "3")]
    pub tick: u64,
    #[prost(uint64, tag = "4")]
    pub moved: u64,
    #[prost(uint64, tag = "5")]
    pub reseeded_gpu_rows: u64,
    #[prost(uint64, tag = "6")]
    pub cpu: u64,
    #[prost(uint64, tag = "7")]
    pub gpu: u64,
    #[prost(string, tag = "8")]
    pub reason: String,
}
pub(super) fn command(
    sim: &mut Simulation,
    payload: &[u8],
    resetting: bool,
) -> EnvironmentPlacementAck {
    let at = batch_boundary(sim);
    let mut ack = EnvironmentPlacementAck {
        epoch: at.epoch,
        tick: at.tick,
        ..Default::default()
    };
    let result = (|| -> Result<(), String> {
        if resetting || sim.is_faulted() {
            return Err("world reset or fault is pending".into());
        }
        let request =
            EnvironmentPlacement::decode(payload).map_err(|_| "malformed placement protobuf")?;
        let stamp = BatchBoundary {
            epoch: request.epoch.ok_or("missing epoch")?,
            tick: request.tick.ok_or("missing tick")?,
        };
        if request.environments.is_empty() || request.environments.len() > MAX_IDS {
            return Err("expected between 1 and 256 stable environment IDs".into());
        }
        let ids = &request.environments;
        let report = match Target::try_from(request.target) {
            Ok(Target::Cpu) => set_environment_placement(sim, stamp, ids, ExecutionPlacement::Cpu),
            Ok(Target::Gpu) => set_environment_placement(sim, stamp, ids, ExecutionPlacement::Gpu),
            Ok(Target::NativeRapier) => set_pair_solver(sim, stamp, ids, PairSolver::NativeRapier),
            Ok(Target::GuardedGpu) => set_pair_solver(sim, stamp, ids, PairSolver::SpecializedGpu),
            _ => {
                return Err(
                    "expected CPU or GPU execution target, or a guarded pair solver".into(),
                );
            }
        }
        .map_err(|e| e.to_string())?;
        ack.moved = report.moved as u64;
        ack.reseeded_gpu_rows = report.reseeded_gpu_rows as u64;
        Ok(())
    })();
    match result {
        Ok(()) => ack.accepted = true,
        Err(reason) => ack.reason = reason,
    }
    if let Some((cpu, gpu)) = execution_counts(sim) {
        ack.cpu = cpu as u64;
        ack.gpu = gpu as u64;
    }
    ack
}
