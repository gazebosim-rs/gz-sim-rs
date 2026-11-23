// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Deformables, Simulation, Surface};

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum MaterialResolutionError {
    #[error("the simulation is faulted")]
    Faulted,
    #[error("unknown deformable material")]
    UnknownMaterial,
    #[error(
        "resolution requires positive substeps and iterations and at least one nanosecond per substep"
    )]
    InvalidResolution,
    #[error("two-way materials must retain a common substep clock")]
    SharedClock,
    #[error("resolution conflicts with the scheduled slip-transition interval")]
    SlipInterval,
    #[cfg(feature = "distributed-physics")]
    #[error("the material is owned by a worker")]
    RemoteOwner,
}

/// Select the local material resolution for the next advancing tick. The rigid
/// clock, queued reactions and published measurements are preserved. Reset
/// restores the authored resolution along with the initial material state.
pub fn set_material_resolution(
    sim: &mut Simulation,
    name: &str,
    substeps: u32,
    iterations: u32,
) -> Result<(), MaterialResolutionError> {
    use MaterialResolutionError::*;
    if sim.is_faulted() {
        return Err(Faulted);
    }
    if substeps == 0 || iterations == 0 || u128::from(substeps) > sim.step_duration().as_nanos() {
        return Err(InvalidResolution);
    }
    #[cfg(feature = "distributed-physics")]
    if sim
        .world()
        .contains_resource::<super::managed_impact::ManagedAutomaticMaterial>()
    {
        return Err(RemoteOwner);
    }
    let mut soft = sim
        .world_mut()
        .get_resource_mut::<Deformables>()
        .ok_or(UnknownMaterial)?;
    #[cfg(feature = "distributed-physics")]
    if soft.remote_material {
        return Err(RemoteOwner);
    }
    let shared_clock = soft.surfaces.iter().any(|surface| {
        matches!(
            surface,
            Surface::Rigid {
                mobility: Some(_),
                ..
            }
        )
    }) && soft
        .current
        .iter()
        .any(|(n, body)| n != name && body.config().substeps != substeps);
    let slip_conflict = soft
        .automatic_impact
        .iter()
        .chain(soft.impact_barriers.values())
        .any(|event| {
            event.material == name
                && event
                    .slip_transition_step
                    .is_some_and(|cap| cap.as_nanos() < 2 * u128::from(substeps))
        });
    let body = soft.current.get_mut(name).ok_or(UnknownMaterial)?;
    if shared_clock {
        return Err(SharedClock);
    }
    if slip_conflict {
        return Err(SlipInterval);
    }
    body.set_resolution(substeps, iterations)
        .map_err(|_| InvalidResolution)
}
