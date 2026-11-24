// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Normal Bevy scheduling and barrier-guarded changes of worker count.
use super::{MAX_PAIR_WORKERS, PairProcessPool};
use crate::{
    GpuError, STEP, SpherePairBlockBatch, StepMetrics,
    simulation::{BatchBoundary, BatchTiming, GpuReset, batch_boundary},
};
use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};

/// Install local process ownership on a fresh simulation at its authored observation cadence.
pub fn install(
    sim: &mut Simulation,
    batch: SpherePairBlockBatch,
    ticks: u32,
    pool: PairProcessPool,
) -> Result<(), GpuError> {
    crate::simulation::check_pair_attachment(sim, &batch, ticks)?;
    if pool.width > batch.lanes().len() || sim.world().contains_resource::<PairProcessPool>() {
        return Err(GpuError::Checkpoint(
            "resident process width exceeds its population or a pool already exists".into(),
        ));
    }
    sim.world_mut().insert_resource(batch);
    sim.world_mut().insert_resource(pool);
    sim.world_mut().init_resource::<BatchTiming>();
    let dt = STEP * ticks;
    sim.add_systems(
        Phase::PhysicsStep,
        move |info: Res<UpdateInfo>,
              mut batch: ResMut<SpherePairBlockBatch>,
              mut pool: ResMut<PairProcessPool>,
              mut timing: ResMut<BatchTiming>,
              fault: Res<StepFault>| {
            if info.paused {
                return;
            }
            if info.start_time != batch.sim_time() || info.dt != dt {
                fault.report(GpuError::StaleBoundary);
                return;
            }
            match pool.advance(&mut batch, ticks) {
                Ok(m) => {
                    timing.last = Some(StepMetrics {
                        latency: m.latency,
                        uploaded_bytes: m.upload_bytes,
                        readback_bytes: m.readback_bytes,
                        ..Default::default()
                    })
                }
                Err(e) => fault.report(e),
            }
        },
    );
    sim.add_systems(
        Phase::Reset,
        (|mut batch: ResMut<SpherePairBlockBatch>,
          mut pool: ResMut<PairProcessPool>,
          mut timing: ResMut<BatchTiming>| {
            batch.reset();
            pool.clear_metrics();
            timing.last = None;
        })
        .in_set(GpuReset),
    );
    Ok(())
}

/// Change physical worker ownership at a committed Bevy boundary without advancing scene time.
/// Resizing is transactional and the next advancing tick seeds newly assigned ranges.
pub fn set_width(sim: &mut Simulation, at: BatchBoundary, width: usize) -> Result<(), GpuError> {
    if sim.is_faulted() || batch_boundary(sim) != at {
        return Err(GpuError::StaleBoundary);
    }
    let batch = sim
        .world()
        .get_resource::<SpherePairBlockBatch>()
        .ok_or(GpuError::StaleBoundary)?;
    if !(1..=MAX_PAIR_WORKERS.min(batch.lanes().len())).contains(&width) {
        return Err(GpuError::Checkpoint(
            "each resident worker needs a nonempty range".into(),
        ));
    }
    sim.world_mut()
        .get_resource_mut::<PairProcessPool>()
        .ok_or(GpuError::StaleBoundary)?
        .resize(width)
}
