// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport};
use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Serialize)]
pub struct Episode {
    ids: Vec<u64>,
    first_tick: u64,
    first_impulse_n_s: f64,
    first_positive: Option<(u64, f64)>,
    first_nonpositive_after_load: Option<u64>,
    first_absent: Option<u64>,
    peak_impulse_n_s: f64,
}
impl Episode {
    fn new(ids: Vec<u64>, tick: u64, impulse: f64) -> Self {
        Self {
            ids,
            first_tick: tick,
            first_impulse_n_s: impulse,
            first_positive: None,
            first_nonpositive_after_load: None,
            first_absent: None,
            peak_impulse_n_s: impulse,
        }
    }
    fn update(&mut self, tick: u64, impulse: Option<f64>) {
        if self.first_absent.is_some() {
            return;
        }
        if let Some(positive) = impulse.filter(|i| *i > 0.) {
            self.first_positive.get_or_insert((tick, positive));
            self.peak_impulse_n_s = self.peak_impulse_n_s.max(positive);
        } else if self.first_positive.is_some() {
            self.first_nonpositive_after_load.get_or_insert(tick);
        }
        if impulse.is_none() {
            self.first_absent = Some(tick);
        }
    }
}

#[derive(Serialize)]
pub struct Encounters {
    #[serde(skip)]
    original_groups: BTreeMap<u64, usize>,
    pub ground: Option<Episode>,
    pub cross_assembly: Option<Episode>,
}
impl Encounters {
    pub fn new(groups: &[Vec<Body>]) -> Self {
        Self {
            original_groups: groups
                .iter()
                .enumerate()
                .flat_map(|(i, g)| g.iter().map(move |b| (b.id, i)))
                .collect(),
            ground: None,
            cross_assembly: None,
        }
    }
    pub fn observe(
        &mut self,
        tick: u64,
        base_floors: &[u64],
        pairs: &[&ContactImpulse],
        floors: &[&GroundSupport],
    ) {
        if self.ground.is_none()
            && !base_floors.is_empty()
            && let Some(f) = floors
                .iter()
                .find(|f| f.normal_impulse > 0. && base_floors.binary_search(&f.body).is_err())
        {
            self.ground = Some(Episode::new(vec![f.body], tick, f.normal_impulse));
        }
        if self.cross_assembly.is_none()
            && let Some(p) = pairs
                .iter()
                .find(|p| self.original_groups[&p.a] != self.original_groups[&p.b])
        {
            self.cross_assembly = Some(Episode::new(vec![p.a, p.b], tick, p.normal_impulse));
        }
        if let Some(e) = &mut self.ground {
            let load = floors
                .iter()
                .find(|f| e.ids[0] == f.body)
                .map(|f| f.normal_impulse);
            e.update(tick, load);
        }
        if let Some(e) = &mut self.cross_assembly {
            let load = pairs
                .iter()
                .find(|p| e.ids == [p.a, p.b])
                .map(|p| p.normal_impulse);
            e.update(tick, load);
        }
    }
}

/// Five-tick views of each physical encounter, augmented with the exact
/// samples immediately before, during and after every observed change of load.
pub fn clip_tick(tick: u64) -> bool {
    const EVENTS: [u64; 7] = [8_451, 8_520, 8_522, 10_122, 10_124, 10_127, 10_138];
    let regular = tick.is_multiple_of(5)
        && ((8_375..=8_525).contains(&tick) || (10_070..=10_195).contains(&tick));
    regular || EVENTS.into_iter().any(|event| tick.abs_diff(event) <= 1)
}
