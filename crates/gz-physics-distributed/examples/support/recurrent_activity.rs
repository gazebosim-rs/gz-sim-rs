// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde::Serialize;
use std::{collections::VecDeque, error::Error};

pub const PERIOD: u64 = 100;
pub const WARMUP: u64 = 100;
pub const WINDOW: usize = 10;
pub const GRAVITY: f64 = -9.81;
pub const LIFT: f64 = 25.;
pub const DT: f64 = 0.01;
pub const IMPULSE_TOLERANCE: f64 = 1e-6;

#[derive(Clone, Debug, Default, Serialize)]
pub struct Cycle {
    pub cycle: u64,
    pub stable_positive: usize,
    pub driven_positive: usize,
    pub driven_absent: usize,
    pub first_absent_phase: Option<u64>,
    pub landing_phase: Option<u64>,
    pub landing_impulse_n_s: f64,
    pub stable_contact_n_s: f64,
    pub driven_contact_n_s: f64,
    pub external_drive_n_s: f64,
    pub max_clearance_m: f64,
    pub momentum_residual_n_s: [f64; 2],
}

impl Cycle {
    pub fn observe(
        &mut self,
        phase: u64,
        stable: Option<f64>,
        driven: Option<f64>,
        force: f64,
        clearance: f64,
    ) {
        self.stable_positive += usize::from(stable.is_some_and(|v| v > 0.));
        self.driven_positive += usize::from(driven.is_some_and(|v| v > 0.));
        if driven.is_none() {
            self.driven_absent += 1;
            self.first_absent_phase.get_or_insert(phase);
        }
        if let Some(value) = driven.filter(|v| *v > 0.)
            && self.first_absent_phase.is_some()
            && self.landing_phase.is_none()
        {
            self.landing_phase = Some(phase);
            self.landing_impulse_n_s = value;
        }
        self.stable_contact_n_s += stable.unwrap_or_default();
        self.driven_contact_n_s += driven.unwrap_or_default();
        self.external_drive_n_s += force * DT;
        self.max_clearance_m = self.max_clearance_m.max(clearance);
    }
    pub fn complete(&mut self, before: [f64; 2], after: [f64; 2]) -> Result<(), Box<dyn Error>> {
        for i in 0..2 {
            let applied = if i == 0 { 0. } else { self.external_drive_n_s };
            let contact = if i == 0 {
                self.stable_contact_n_s
            } else {
                self.driven_contact_n_s
            };
            // Both moving spheres have mass one kg. A centered horizontal finite
            // table exerts its normal impulse vertically. The user force and
            // gravity are integrated over the full one-second interval.
            self.momentum_residual_n_s[i] =
                (after[i] - before[i] - (GRAVITY * DT * PERIOD as f64 + applied + contact)).abs();
        }
        if self.stable_positive < 99
            || self.driven_positive < 25
            || self.driven_absent < 20
            || self.landing_phase.is_none()
            || self.landing_impulse_n_s < 0.5
            || self.max_clearance_m < 0.05
            || self
                .momentum_residual_n_s
                .iter()
                .any(|v| !v.is_finite() || *v > IMPULSE_TOLERANCE)
        {
            return Err(format!("physical cycle gate failed: {self:?}").into());
        }
        Ok(())
    }
}

#[derive(Default, Debug, Serialize)]
pub struct Windows {
    pub checked: u64,
    pub smallest_stable_positive: Option<usize>,
    pub smallest_driven_positive: Option<usize>,
    pub smallest_driven_absent: Option<usize>,
    pub smallest_landings: Option<usize>,
    pub highest_momentum_residual_n_s: f64,
    #[serde(skip)]
    cycles: VecDeque<Cycle>,
}
impl Windows {
    pub fn observe(&mut self, cycle: &Cycle) -> Result<(), Box<dyn Error>> {
        self.highest_momentum_residual_n_s = cycle
            .momentum_residual_n_s
            .into_iter()
            .fold(self.highest_momentum_residual_n_s, f64::max);
        self.cycles.push_back(cycle.clone());
        if self.cycles.len() > WINDOW {
            self.cycles.pop_front();
        }
        if self.cycles.len() != WINDOW {
            return Ok(());
        }
        let stable: usize = self.cycles.iter().map(|c| c.stable_positive).sum();
        let driven: usize = self.cycles.iter().map(|c| c.driven_positive).sum();
        let absent: usize = self.cycles.iter().map(|c| c.driven_absent).sum();
        let landings = self
            .cycles
            .iter()
            .filter(|c| c.landing_phase.is_some())
            .count();
        if stable < 990 || driven < 250 || absent < 200 || landings < 10 {
            return Err(format!("rolling 1,000-tick activity failed: stable={stable}, driven={driven}, absent={absent}, landings={landings}").into());
        }
        self.smallest_stable_positive =
            Some(stable.min(self.smallest_stable_positive.unwrap_or(stable)));
        self.smallest_driven_positive =
            Some(driven.min(self.smallest_driven_positive.unwrap_or(driven)));
        self.smallest_driven_absent =
            Some(absent.min(self.smallest_driven_absent.unwrap_or(absent)));
        self.smallest_landings = Some(landings.min(self.smallest_landings.unwrap_or(landings)));
        self.checked += 1;
        Ok(())
    }
}
