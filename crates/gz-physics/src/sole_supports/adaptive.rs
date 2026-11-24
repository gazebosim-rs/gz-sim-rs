// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Per-support accuracy checks for one shared rigid-body tick.
use super::*;
use crate::sole_contact::{SoleAccuracy, SoleDifference, adaptive::select_grid};

#[derive(Debug, Clone, Copy, Default)]
pub struct PanelDifference {
    /// Pose of the single foot; all impulse and normal-heat errors are maxima
    /// over individual support IDs. Opposing panel errors cannot cancel.
    pub contact: SoleDifference,
    pub sliding_work_j: f64,
    pub rim_work_j: f64,
    pub support_work_j: f64,
}
impl PanelDifference {
    fn compare(a: &(SoleState, Vec<PanelReaction>), b: &(SoleState, Vec<PanelReaction>)) -> Self {
        let mut e = Self {
            contact: SoleDifference::between_states(&a.0, &b.0),
            ..Self::default()
        };
        for (x, y) in a.1.iter().zip(&b.1) {
            debug_assert_eq!(x.id, y.id);
            let (p, q) = (&x.on_sole, &y.on_sole);
            e.contact.impulse_ns = e.contact.impulse_ns.max(p.impulse.distance(q.impulse));
            e.contact.angular_impulse_nms = e
                .contact
                .angular_impulse_nms
                .max(p.angular_impulse.distance(q.angular_impulse));
            e.contact.heat_j = e
                .contact
                .heat_j
                .max((p.dissipated_energy - q.dissipated_energy).abs());
            e.sliding_work_j = e
                .sliding_work_j
                .max((p.sliding_work - q.sliding_work).abs());
            e.rim_work_j = e.rim_work_j.max((p.boundary_work - q.boundary_work).abs());
            e.support_work_j = e
                .support_work_j
                .max((x.support_work_j - y.support_work_j).abs());
        }
        e
    }
}
#[derive(Debug, Clone)]
pub struct AdaptivePanelStep {
    pub panels: Vec<PanelReaction>,
    pub substeps: u32,
    pub depth: u8,
    pub trials: u32,
    pub temporal: PanelDifference,
    pub spatial: PanelDifference,
}

/// Advance every panel on the same accepted grid. Each support must satisfy
/// its own absolute impulse, heat and work tolerances on both neighboring
/// grids. Unloaded sweeps with possible missed contact force a finer time grid.
#[allow(clippy::too_many_arguments)]
pub fn advance_panels_adaptive(
    body: &SoleBody,
    state: &mut SoleState,
    panels: &[SolePanel],
    gravity: DVec3,
    external_force: DVec3,
    external_torque: DVec3,
    dt: f64,
    accuracy: &SoleAccuracy,
    work_tolerance_j: f64,
) -> Result<AdaptivePanelStep, PanelError> {
    if !work_tolerance_j.is_finite() || work_tolerance_j < 0. {
        return Err(SoleError::InvalidInput.into());
    }
    let sorted = prepare_panels(panels, dt)?;
    let initial = *state;
    let grid = select_grid(
        accuracy,
        |n, d| -> Result<_, PanelError> {
            let mut next = initial;
            let result = advance_prepared(
                body,
                &mut next,
                &sorted,
                gravity,
                external_force,
                external_torque,
                dt,
                n,
                d,
            )?;
            Ok((next, result))
        },
        PanelDifference::compare,
        |e| {
            e.contact.within(accuracy)
                && e.sliding_work_j <= work_tolerance_j
                && e.rim_work_j <= work_tolerance_j
                && e.support_work_j <= work_tolerance_j
        },
        |(_, rows)| rows.iter().any(|r| r.on_sole.unresolved()),
    )?;
    *state = grid.value.0;
    Ok(AdaptivePanelStep {
        panels: grid.value.1,
        substeps: grid.substeps,
        depth: grid.depth,
        trials: grid.trials,
        temporal: grid.temporal,
        spatial: grid.spatial,
    })
}

#[cfg(test)]
mod tests;
