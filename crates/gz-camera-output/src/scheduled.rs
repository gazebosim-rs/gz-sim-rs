// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{CameraOutput, CameraOutputConfig, CameraOutputError, CameraWriterScope, Progress};
use gz_render_scene::SnapshotError;
use gz_sim_core::{EntityId, Simulation, UpdateInfo};
use gz_sim_scene::{CameraSensor, SensorSchedule};
use std::time::Duration;
use zenoh::Session;

/// Application-owned cadence for the existing single-camera capture path.
///
/// All methods, including preparation, requests, pumping and retirement, belong
/// outside the simulation's critical step. They can resolve local Zenoh state,
/// spawn a supervisor or retire a process; this is not a real-time interface.
/// No ECS callback is installed. The caller prepares after load, then drops and
/// reconstructs this owner after reset, replacement or removal. A camera-free
/// world returns `None`, allocating no camera owner or declarations. The supplied
/// writer scope and session remain caller-owned shared infrastructure.
pub struct ScheduledCameraOutput {
    output: CameraOutput,
    schedule: SensorSchedule,
    epoch: u64,
    last_attempt: Option<Duration>,
    active: bool,
}

impl ScheduledCameraOutput {
    /// Find zero or one loaded camera before any worker or publication setup.
    /// Multiple cameras are explicitly outside this bounded application path.
    pub fn prepare(
        sim: &Simulation,
        session: &Session,
        scope: &CameraWriterScope,
        config: CameraOutputConfig,
    ) -> Result<Option<Self>, CameraOutputError> {
        let mut cameras = sim
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<CameraSensor>());
        let Some(camera) = cameras.next() else {
            return Ok(None);
        };
        if cameras.next().is_some() {
            return Err(CameraOutputError::MultipleCameras);
        }
        let id = camera
            .get::<EntityId>()
            .ok_or(SnapshotError::MissingCamera(0))?
            .get();
        let schedule = SensorSchedule::new(
            camera
                .get::<CameraSensor>()
                .expect("camera query includes this component")
                .0
                .update_rate,
        )?;
        Ok(Some(Self {
            output: CameraOutput::new(sim, session, scope, id, config)?,
            schedule,
            epoch: sim.epoch(),
            last_attempt: None,
            active: true,
        }))
    }

    /// Local application admission gate, initially true. This does not implement
    /// SDF always_on, triggering or the native Manager interface.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Disable new samples and cancel any buffered acquisition. A live worker
    /// then requires explicit `retire`, even while inactive; it is never hidden
    /// behind an idle result. Re-enabling preserves the sampling phase.
    pub fn set_active(&mut self, active: bool) {
        self.active = active;
        if !active {
            self.output.acquisition = None;
            self.output.retiring |= self.output.client.is_some();
        }
    }

    pub(crate) fn demand(&self) -> Result<(bool, bool), CameraOutputError> {
        self.output.demand()
    }

    pub(crate) fn binding_valid(&self, sim: &Simulation) -> bool {
        self.epoch == sim.epoch() && self.output.binding_valid(sim)
    }

    fn validate_binding(&mut self, sim: &Simulation) -> Result<(), CameraOutputError> {
        if !self.binding_valid(sim) {
            self.output.invalidated = true;
            self.output.acquisition = None;
            self.output.retiring |= self.output.client.is_some();
            return Err(CameraOutputError::OwnerInvalid);
        }
        Ok(())
    }

    /// Attempt at most one sample from a committed advancing step. `None` means
    /// inactive, paused, already attempted at this timestamp or not yet due.
    /// Explicit steps while configured paused remain eligible: their committed
    /// UpdateInfo has nonzero dt and paused=false.
    ///
    /// No-demand, Busy and retirement checks precede admission. Once demand is
    /// observed, an eligible attempt consumes one period even on error, as the
    /// native Sensor clock advances after its virtual Update returns false.
    /// This also prevents repeated camera-info output after partial failure.
    /// A missed interval produces one current sample, never a catch-up backlog.
    pub fn request_due(&mut self, sim: &Simulation) -> Result<Option<Progress>, CameraOutputError> {
        self.validate_binding(sim)?;
        if self.output.retiring {
            return Err(CameraOutputError::RetireRequired);
        }
        let time = sim.world().resource::<UpdateInfo>();
        if !self.active
            || time.paused
            || time.dt.is_zero()
            || self.last_attempt == Some(sim.elapsed())
            || !self.schedule.is_due(sim.elapsed())
        {
            return Ok(None);
        }
        if self.output.acquisition.is_some() {
            return Err(CameraOutputError::Busy);
        }
        let observed = self.demand()?;
        self.request_observed(sim, observed)
    }

    fn request_observed(
        &mut self,
        sim: &Simulation,
        (image, info): (bool, bool),
    ) -> Result<Option<Progress>, CameraOutputError> {
        if !image {
            if self.output.client.is_some() {
                self.output.retiring = true;
                return Ok(Some(Progress::RetireRequired));
            }
            if !info {
                return Ok(Some(Progress::NoDemand));
            }
        }
        self.schedule.take_sample(sim.elapsed(), false)?;
        self.last_attempt = Some(sim.elapsed());
        self.output.request(sim).map(Some)
    }

    /// Advance the existing bounded IPC state machine. A paused simulation can
    /// complete its preceding acquisition; reset/removal/replacement cannot
    /// publish that buffered frame. This performs no new sampling admission.
    pub fn pump(&mut self, sim: &Simulation) -> Result<Progress, CameraOutputError> {
        self.validate_binding(sim)?;
        self.output.pump(sim)
    }

    /// Join the existing client outside stepping. Invalidated owners still must
    /// be dropped and prepared again; retirement does not rebind scene identity.
    pub fn retire(&mut self) {
        self.output.retire();
    }
}

#[cfg(test)]
#[path = "../tests/private/scheduled.rs"]
mod tests;
