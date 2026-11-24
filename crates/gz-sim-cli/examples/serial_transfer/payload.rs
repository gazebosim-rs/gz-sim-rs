// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Physical load measurements for a user-supplied serial workcell.
use super::*;

/// The receiving face in this workcell is horizontal. Its load is the vertical
/// component of parcel weight, measured in the same world axes as the sensor.
pub(super) fn weight(sdf: &sdformat::Sdf) -> Result<f64> {
    let world = sdf
        .worlds
        .iter()
        .find(|world| world.name == "default")
        .ok_or("missing serial workcell")?;
    let link = world
        .models
        .iter()
        .find(|model| model.name == "parcel")
        .and_then(|model| model.links.iter().find(|link| link.name == "box"))
        .ok_or("missing parcel inertial")?;
    Ok(-world.gravity[2] * link.inertial.mass)
}

#[derive(Debug, PartialEq, Serialize)]
pub(super) struct Loads {
    pub(super) weight_n: f64,
    min_pad_normal_n: [f64; 2],
    mean_pad_normal_n: [f64; 2],
    first_missing_bilateral_grip_ms: Option<usize>,
    tray_loaded_traverse_ticks: usize,
    shelf_loaded_traverse_ticks: usize,
}
impl Loads {
    pub(super) fn new(weight_n: f64) -> Self {
        Self {
            weight_n,
            min_pad_normal_n: [f64::INFINITY; 2],
            mean_pad_normal_n: [0.; 2],
            first_missing_bilateral_grip_ms: None,
            tray_loaded_traverse_ticks: 0,
            shelf_loaded_traverse_ticks: 0,
        }
    }
    pub(super) fn observe(&mut self, tick: usize, pad: [f64; 2], tray: bool, shelf: bool) {
        for (i, force) in pad.into_iter().enumerate() {
            self.min_pad_normal_n[i] = self.min_pad_normal_n[i].min(force);
            self.mean_pad_normal_n[i] += force / (MOVE_END - MOVE_START + 1) as f64;
        }
        if !pad.iter().all(|f| *f > 0.1) {
            self.first_missing_bilateral_grip_ms.get_or_insert(tick);
        }
        self.tray_loaded_traverse_ticks += usize::from(tray);
        self.shelf_loaded_traverse_ticks += usize::from(shelf);
    }
}

#[cfg(test)]
#[path = "payload/tests.rs"]
mod tests;
