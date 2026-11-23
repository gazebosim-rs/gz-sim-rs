// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconcile independently seeded numerical controls and durable process recovery.
use super::{Path, Result, Value, json, records};
use serde::Deserialize;

#[derive(Default)]
struct Sequence {
    first: Option<u64>,
    last: Option<u64>,
    total: u64,
}
impl Sequence {
    fn add(&mut self, seed: u64) -> Result<()> {
        if self
            .last
            .is_some_and(|prev| prev.checked_add(1) != Some(seed))
        {
            return Err(format!("duplicate or missing GPU episode before {seed}").into());
        }
        self.first.get_or_insert(seed);
        self.last = Some(seed);
        self.total += 1;
        Ok(())
    }
    fn value(&self) -> Value {
        json!({"first_seed":self.first,"last_seed":self.last,"episodes":self.total})
    }
}
#[derive(Deserialize)]
struct Errors {
    hits: u64,
    misses: u64,
    position_m: f64,
    velocity_mps: f64,
    spin_radps: f64,
    orientation_rad: f64,
    orientation_error_over_bound: f64,
    toi_s: f64,
    impulse_ns: f64,
}
#[derive(Deserialize)]
struct Geometry {
    seed: u64,
    lanes: u64,
    ticks: u64,
    reset_verified: bool,
    errors: Errors,
}
fn geometry(p: &Path) -> Result<Value> {
    let mut seq = Sequence::default();
    let mut totals = [0u64; 3];
    let mut peaks = [0f64; 7];
    records(p, |r: Geometry| {
        seq.add(r.seed)?;
        let e = r.errors;
        if !r.reset_verified || e.hits + e.misses != r.lanes || !(1..=4096).contains(&r.ticks) {
            return Err(format!("invalid geometry receipt for seed {}", r.seed).into());
        }
        for (max, v) in peaks.iter_mut().zip([
            e.position_m,
            e.velocity_mps,
            e.spin_radps,
            e.orientation_rad,
            e.orientation_error_over_bound,
            e.toi_s,
            e.impulse_ns,
        ]) {
            *max = max.max(v);
        }
        totals[0] += e.hits;
        totals[1] += e.misses;
        totals[2] += r.lanes * r.ticks;
        Ok(())
    })?;
    Ok(
        json!({"sequence":seq.value(),"hit_lanes":totals[0],"miss_lanes":totals[1],"single_pass_lane_ticks":totals[2],
    "worst":{"position_m":peaks[0],"velocity_mps":peaks[1],"spin_radps":peaks[2],"orientation_rad":peaks[3],"orientation_over_bound":peaks[4],"toi_s":peaks[5],"impulse_ns":peaks[6]}}),
    )
}
#[derive(Deserialize)]
struct Attitude {
    seed: u64,
    lanes: u64,
    ticks: u64,
    bodies_compared: u64,
    max_fourth_rad: f64,
    max_midpoint_rad: f64,
}
fn attitude(p: &Path) -> Result<Value> {
    let mut seq = Sequence::default();
    let mut bodies = 0;
    let mut ticks = 0;
    let (mut mid, mut fourth) = (0f64, 0f64);
    records(p, |r: Attitude| {
        seq.add(r.seed)?;
        if r.bodies_compared != 2 * r.lanes || !(1..=4096).contains(&r.ticks) {
            return Err(format!("invalid attitude receipt for seed {}", r.seed).into());
        }
        bodies += r.bodies_compared;
        ticks += r.lanes * r.ticks;
        mid = mid.max(r.max_midpoint_rad);
        fourth = fourth.max(r.max_fourth_rad);
        Ok(())
    })?;
    Ok(
        json!({"sequence":seq.value(),"bodies":bodies,"lane_ticks_per_profile":ticks,"max_midpoint_rad":mid,"max_fourth_rad":fourth}),
    )
}
#[derive(Deserialize)]
struct Recovery {
    seed: u64,
    blocks: u64,
    workers_killed: u64,
    lanes: u64,
    physical_ticks: u64,
    loaded_lane_ticks: u64,
    verified_bytes: u64,
    sha256: String,
}
fn recovery(p: &Path) -> Result<Value> {
    let mut seq = Sequence::default();
    let mut kills = 0;
    let mut ticks = 0;
    let mut hits = 0;
    let mut bytes = 0;
    records(p, |r: Recovery| {
        seq.add(r.seed)?;
        if r.blocks == 0
            || r.workers_killed != r.blocks
            || r.lanes == 0
            || r.sha256.len() != 64
            || !r.sha256.bytes().all(|c| c.is_ascii_hexdigit())
        {
            return Err(format!("invalid worker recovery receipt for seed {}", r.seed).into());
        }
        kills += r.workers_killed;
        ticks += r.physical_ticks * r.lanes;
        hits += r.loaded_lane_ticks;
        bytes += r.verified_bytes;
        Ok(())
    })?;
    Ok(
        json!({"sequence":seq.value(),"killed_workers":kills,"lane_ticks":ticks,"loaded_lane_ticks":hits,"verified_complete_checkpoint_bytes":bytes}),
    )
}
pub(super) fn audit(dir: &Path) -> Result<Value> {
    Ok(
        json!({"analytic_geometry":geometry(&dir.join("multifidelity-resident-pair/seeded.jsonl"))?,
        "angular_profiles":attitude(&dir.join("multifidelity-gpu-rotation/campaign.jsonl"))?,
        "restored_processes":recovery(&dir.join("multifidelity-resident-pair-recovery/episodes.jsonl"))?}),
    )
}
