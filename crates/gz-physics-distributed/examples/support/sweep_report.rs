// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Frame,
    physics::{ContactImpulse, Snapshot},
};
use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Clone, Default, Serialize)]
pub struct Differences {
    pub first_body_tick: Option<u64>,
    pub first_contact_set_tick: Option<u64>,
    pub first_impulse_tick: Option<u64>,
    // World-frame Euclidean norms; orientation is sign-invariant quaternion chord distance.
    pub position_m: f64,
    pub orientation_chord: f64,
    pub linear_m_s: f64,
    pub angular_rad_s: f64,
    pub pair_impulse_n_s: f64,
}
fn norm<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter().zip(b).fold(0_f64, |v, (x, y)| v.hypot(x - y))
}
impl Differences {
    pub fn compare(
        &mut self,
        frame: &Frame,
        references: &[Snapshot],
    ) -> Result<(), Box<dyn std::error::Error>> {
        for expected in references.iter().flat_map(|s| &s.bodies) {
            let actual = frame
                .body(expected.id)
                .ok_or("missing committed body ID")?
                .1;
            for body in [expected, actual] {
                if body
                    .position
                    .iter()
                    .chain(&body.orientation)
                    .chain(&body.linear)
                    .chain(&body.angular)
                    .any(|x| !x.is_finite())
                {
                    return Err("nonfinite observed or reference body state".into());
                }
            }
            if actual != expected {
                self.first_body_tick.get_or_insert(frame.tick);
                self.position_m = self
                    .position_m
                    .max(norm(actual.position, expected.position));
                self.orientation_chord = self.orientation_chord.max(
                    norm(actual.orientation, expected.orientation)
                        .min(norm(actual.orientation, expected.orientation.map(|x| -x))),
                );
                self.linear_m_s = self.linear_m_s.max(norm(actual.linear, expected.linear));
                self.angular_rad_s = self
                    .angular_rad_s
                    .max(norm(actual.angular, expected.angular));
            }
        }
        let pairs = |p: &ContactImpulse| ((p.a, p.b), p.normal_impulse);
        let expected: BTreeMap<_, _> = references
            .iter()
            .flat_map(|s| &s.impulses)
            .map(pairs)
            .collect();
        let actual: BTreeMap<_, _> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(pairs)
            .collect();
        if actual
            .values()
            .chain(expected.values())
            .any(|j| !j.is_finite() || *j < 0.)
        {
            return Err("invalid observed or reference normal impulse".into());
        }
        if !actual.keys().eq(expected.keys()) {
            self.first_contact_set_tick.get_or_insert(frame.tick);
        }
        for (key, a) in &actual {
            if let Some(b) = expected.get(key)
                && a.to_bits() != b.to_bits()
            {
                self.first_impulse_tick.get_or_insert(frame.tick);
                self.pair_impulse_n_s = self.pair_impulse_n_s.max((a - b).abs());
            }
        }
        Ok(())
    }

    fn merge_peaks(&mut self, other: &Self) {
        self.position_m = self.position_m.max(other.position_m);
        self.orientation_chord = self.orientation_chord.max(other.orientation_chord);
        self.linear_m_s = self.linear_m_s.max(other.linear_m_s);
        self.angular_rad_s = self.angular_rad_s.max(other.angular_rad_s);
        self.pair_impulse_n_s = self.pair_impulse_n_s.max(other.pair_impulse_n_s);
        for (into, from) in [
            (&mut self.first_body_tick, other.first_body_tick),
            (
                &mut self.first_contact_set_tick,
                other.first_contact_set_tick,
            ),
            (&mut self.first_impulse_tick, other.first_impulse_tick),
        ] {
            if let Some(t) = from {
                *into = Some(into.map_or(t, |old| old.min(t)));
            }
        }
    }
}
#[derive(Clone, Default, Serialize)]
pub struct Case {
    pub seed: u64,
    pub bodies: usize,
    pub ticks: u64,
    pub target_ticks: u64,
    pub stage: &'static str,
    pub error: Option<String>,
    pub migrations: Vec<u64>,
    pub replaced_at: Option<u64>,
    pub original_pid: Option<u32>,
    pub replacement_pid: Option<u32>,
    pub queued_inputs: u32,
    pub peak_native_bytes: usize,
    pub min_cached_positive_points: Option<usize>,
    pub min_exported_pending: Option<usize>,
    pub min_export_impulse_n_s: Option<f64>,
    pub max_export_impulse_n_s: f64,
    pub monolithic: Differences,
    pub same_partition: Differences,
}
impl Case {
    fn categories(&self) -> Vec<String> {
        let mut result = Vec::new();
        if self.error.is_some() {
            result.push(format!("{}_error", self.stage));
        }
        if self.migrations.len() < 2 {
            result.push("fewer_than_two_loaded_migrations".into());
        }
        if self.replacement_pid.is_none() {
            result.push("no_verified_replacement".into());
        }
        if self.min_exported_pending.is_some_and(|n| n < 2) {
            result.push("lost_pending_input".into());
        }

        for (name, diff) in [
            ("monolithic", &self.monolithic),
            ("same_partition", &self.same_partition),
        ] {
            for (what, first) in [
                ("body", diff.first_body_tick),
                ("contact_set", diff.first_contact_set_tick),
                ("contact_impulse", diff.first_impulse_tick),
            ] {
                if first.is_some() {
                    result.push(format!("{name}_{what}"));
                }
            }
        }
        result
    }
}

#[derive(Default, Serialize)]
pub struct SizeCount {
    pub cases: u64,
    pub exact: u64,
    pub migrated_twice: u64,
    pub peak_monolithic_position_m: f64,
}
#[derive(Default, Serialize)]
pub struct Summary {
    pub start_seed: u64,
    pub requested_cases: u64,
    pub stop_unix: Option<u64>,
    pub completed: u64,
    pub exact: u64,
    pub steps: u64,
    pub migrations: u64,
    pub worker_replacements: u64,
    pub inputs: u64,
    pub max_native_bytes: usize,
    pub cases_with_zero_cached_points_at_any_export: u64,
    pub cases_with_positive_cached_points_at_every_export: u64,
    pub min_loaded_export_impulse_n_s: Option<f64>,
    pub first_failure: Option<Case>,
    pub first_by_category: BTreeMap<String, Case>,
    pub category_counts: BTreeMap<String, u64>,
    pub by_box_count: BTreeMap<usize, SizeCount>,
    pub peak_monolithic: Differences,
    pub peak_same_partition: Differences,
    pub last_seed: Option<u64>,
    pub elapsed_s: f64,
    pub finished: bool,
    pub interrupted_seed: Option<u64>,
}
impl Summary {
    pub fn add(&mut self, case: &Case) {
        self.completed += 1;
        self.last_seed = Some(case.seed);
        self.steps += case.ticks;
        self.migrations += case.migrations.len() as u64;
        self.worker_replacements += u64::from(case.replacement_pid.is_some());
        self.inputs += u64::from(case.queued_inputs);
        self.max_native_bytes = self.max_native_bytes.max(case.peak_native_bytes);
        self.peak_monolithic.merge_peaks(&case.monolithic);
        self.peak_same_partition.merge_peaks(&case.same_partition);
        if let Some(n) = case.min_cached_positive_points {
            self.cases_with_zero_cached_points_at_any_export += u64::from(n == 0);
            self.cases_with_positive_cached_points_at_every_export += u64::from(n > 0);
        }
        if let Some(load) = case.min_export_impulse_n_s {
            self.min_loaded_export_impulse_n_s = Some(
                self.min_loaded_export_impulse_n_s
                    .map_or(load, |old| old.min(load)),
            );
        }
        let group = self.by_box_count.entry(case.bodies).or_default();
        group.cases += 1;
        group.migrated_twice += u64::from(case.migrations.len() >= 2);
        group.peak_monolithic_position_m = group
            .peak_monolithic_position_m
            .max(case.monolithic.position_m);
        let categories = case.categories();
        if categories.is_empty() {
            self.exact += 1;
            group.exact += 1;
        } else {
            self.first_failure.get_or_insert_with(|| case.clone());
        }
        for reason in categories {
            *self.category_counts.entry(reason.clone()).or_default() += 1;
            self.first_by_category
                .entry(reason)
                .or_insert_with(|| case.clone());
        }
    }
}
