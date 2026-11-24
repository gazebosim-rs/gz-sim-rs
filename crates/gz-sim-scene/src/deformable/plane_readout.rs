// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::DeformableSample;
use glam::DVec3;
use gz_physics_xpbd::{ParticleBoundaryImpulse, PlaneImpulseProbe, XpbdError};
use std::collections::BTreeMap;

/// The native or exchange-integrated diagnostic sum exceeded its numeric range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("per-particle plane impulse is outside the representable range")]
pub struct PlaneImpulseOverflow;

#[derive(Default)]
pub(super) struct PlaneReading {
    rows: Vec<ParticleBoundaryImpulse>,
    overflow: bool,
}

impl DeformableSample {
    /// Normal and tangent world impulses (N·s) **on the plane**, integrated over
    /// the sample's public tick. Indices are the original SDF boundary slot and
    /// construction particle, even after an earlier boundary is removed.
    /// `None` means the material did not request `plane_impulses="true"`.
    /// An enabled empty result means zero impulse, including at construction or reset.
    /// Overflow invalidates only this measurement; a failed physics step publishes no new sample.
    pub fn last_plane_impulses(
        &self,
    ) -> Option<Result<&[ParticleBoundaryImpulse], PlaneImpulseOverflow>> {
        self.plane.as_ref().map(|v| {
            if v.overflow {
                Err(PlaneImpulseOverflow)
            } else {
                Ok(v.rows.as_slice())
            }
        })
    }
}

#[derive(Default)]
pub(super) struct PlaneMeter {
    pub native: PlaneImpulseProbe,
    pub total: PlaneIntegral,
}

#[derive(Default)]
pub(super) struct PlaneIntegral {
    rows: BTreeMap<(usize, usize), ParticleBoundaryImpulse>,
    overflow: bool,
}

impl PlaneIntegral {
    pub fn clear(&mut self) {
        self.rows.clear();
        self.overflow = false;
    }

    pub fn add(
        &mut self,
        readings: Result<&[ParticleBoundaryImpulse], XpbdError>,
        slots: &[usize],
    ) {
        let Ok(readings) = readings else {
            self.overflow = true;
            return;
        };
        for row in readings {
            let boundary = slots[row.boundary];
            let total =
                self.rows
                    .entry((boundary, row.particle))
                    .or_insert(ParticleBoundaryImpulse {
                        boundary,
                        particle: row.particle,
                        normal: [0.; 3],
                        tangent: [0.; 3],
                    });
            let normal = DVec3::from(total.normal) + DVec3::from(row.normal);
            let tangent = DVec3::from(total.tangent) + DVec3::from(row.tangent);
            self.overflow |= !normal.is_finite() || !tangent.is_finite();
            total.normal = normal.into();
            total.tangent = tangent.into();
        }
    }

    pub fn publish(&self, reading: &mut PlaneReading, reset: bool) {
        reading.rows.clear();
        reading.overflow = !reset && self.overflow;
        if !reset && !reading.overflow {
            reading.rows.extend(
                self.rows
                    .values()
                    .copied()
                    .filter(|v| v.normal != [0.; 3] || v.tangent != [0.; 3]),
            );
        }
    }
}

#[cfg(test)]
#[path = "../../tests/private/plane_impulse_readout.rs"]
mod tests;
