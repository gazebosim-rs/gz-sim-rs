// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Place measured sampling opportunities on the same timeline as positive pad-contact changes.
use super::{Result, Row, View, lines};
use serde::Serialize;
use serde_json::Value;
use std::{collections::BTreeSet, path::Path};
#[derive(Serialize)]
pub struct Cadence {
    pub opportunities: Vec<u64>,
    pub starts: BTreeSet<u64>,
    pub contacts: BTreeSet<u64>,
    pub deferrals: u64,
    pub early_wakes: u64,
    pub cap_periods: u64,
    pub off_cadence: bool,
}
impl Cadence {
    pub fn read(src: &Path, proof: &Value, rows: &[Row]) -> Result<Self> {
        let data = lines::<Value>(&src.join("trials.jsonl"))?;
        let [trial] = data.as_slice() else {
            return Err("need a single measured cadence".into());
        };
        let s = &trial["data"];
        let starts: BTreeSet<u64> = serde_json::from_value(s["sample_starts"].clone())?;
        let contacts: BTreeSet<u64> = serde_json::from_value(s["contact_transitions"].clone())?;
        let period = proof["policy"]["period_ticks"]
            .as_u64()
            .ok_or("missing cadence")?;
        if period != 10 {
            return Err("this fixture charts ten-barrier opportunities".into());
        }
        let opportunities: Vec<_> = (0..350).step_by(period as usize).collect();
        let deferrals = s["windows_deferred"]
            .as_u64()
            .ok_or("missing deferral total")?;
        let early_wakes = s["windows_woken"]
            .as_u64()
            .ok_or("missing early wake total")?;
        let cap_periods = proof["policy"]["max_idle_periods"].as_u64().unwrap_or(1);
        if starts.len() as u64 != s["windows_started"].as_u64().unwrap()
            || starts.len() as u64 + deferrals != opportunities.len() as u64
            || starts.iter().any(|t| !opportunities.contains(t))
        {
            return Err("inconsistent cadence evidence".into());
        }
        for f in rows.windows(2) {
            if f[1].events.iter().any(|e| e == "fresh_pool") {
                continue;
            }
            let marked = starts.contains(&f[0].physical.tick);
            if marked != (f[1].window_counters[0] > f[0].window_counters[0]) {
                return Err("actual diagnostic counter disagrees with recorded start".into());
            }
        }
        for &t in &contacts {
            let prev = &rows[t as usize - 1].physical;
            let now = &rows[t as usize].physical;
            if prev.loaded_pairs == now.loaded_pairs {
                return Err("reported landing or release retained both counts".into());
            }
        }
        let off_cadence = contacts.iter().any(|t| !opportunities.contains(t));
        Ok(Self {
            opportunities,
            off_cadence,
            starts,
            contacts,
            deferrals,
            early_wakes,
            cap_periods,
        })
    }
    pub fn draw(&self, v: &mut View, t: u64) {
        for (i, &at) in self.opportunities.iter().enumerate() {
            let x = -10.8 + i as f64 * 0.63;
            let (state, color) = if t < at {
                ("future", [0.36, 0.40, 0.45, 1.])
            } else if self.starts.contains(&at) {
                ("sample_started", [0.15, 0.74, 0.49, 1.])
            } else {
                ("deferred", [0.96, 0.58, 0.19, 1.])
            };
            let stage = u64::from(t >= at);
            v.cuboid(
                4_000_000 + 4 * i as u64 + 2 * stage,
                &format!("native_barrier_{at}_{state}"),
                [x, -6.3, 0.26],
                [0.49, 0.42, 0.32],
                color,
            );
            if !self.off_cadence && at <= t && self.contacts.contains(&at) {
                v.cuboid(
                    5_000_000 + 2 * i as u64,
                    &format!("real_contact_load_changed_at_{at}_fresh_sample"),
                    [x, -5.80, 0.24],
                    [0.19, 0.24, 0.28],
                    [0.89, 0.83, 0.43, 1.],
                );
            }
        }
        if self.off_cadence {
            for (i, &at) in self.contacts.range(..=t).enumerate() {
                let x = -10.8 + at as f64 * 0.063;
                let received = self.starts.range(at..=t).next().copied();
                let (label, color) = match received {
                    Some(start) => (
                        format!("sample_at_{start}_wait_{}_ticks", start - at),
                        [0.33, 0.80, 0.69, 1.],
                    ),
                    None => (
                        "awaiting_next_observed_sample".into(),
                        [0.97, 0.59, 0.24, 1.],
                    ),
                };
                v.cuboid(
                    6_000_000 + 4 * i as u64 + 2 * u64::from(received.is_some()),
                    &format!("physical_load_transition_at_{at}_{label}"),
                    [x, -5.78, 0.26],
                    [0.16, 0.23, 0.30],
                    color,
                );
                if let Some(start) = received.filter(|&start| start > at) {
                    let width = (start - at) as f64 * 0.063;
                    v.cuboid(
                        7_000_000 + 2 * i as u64,
                        &format!("observed_wait_from_{at}_to_{start}"),
                        [x + width / 2., -5.62, 0.22],
                        [width, 0.10, 0.16],
                        [0.88, 0.80, 0.46, 1.],
                    );
                }
            }
        }
    }
}
