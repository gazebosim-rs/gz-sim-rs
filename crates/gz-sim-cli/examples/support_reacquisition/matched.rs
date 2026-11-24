// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Controlled third-landing experiments on one fixed lift command history.
use super::*;
use std::{path::Path, time::Duration};

const STEP_US: u64 = 500;
const CUTS: [usize; 2] = [1363, 1423];
const END: usize = 1433;
// Post-solve opportunities from the three-hold recording. Each request that
// isn't immediately feasible is retried until the next commanded departure.
const REQUEST: [usize; 3] = [0, 693, 1161];

pub(super) struct Case {
    mask: u8,
    pub(super) lane: Lane,
    admitted: [Option<usize>; 3],
    promoted: [Option<usize>; 3],
    pub(super) samples: Vec<Sample>,
}

impl Case {
    pub(super) fn new(mask: u8, capacity: usize) -> Result<Self> {
        Ok(Self::from_lane(mask, capacity, Lane::new_at(STEP_US)?))
    }

    pub(super) fn from_lane(mask: u8, capacity: usize, lane: Lane) -> Self {
        Self {
            mask,
            lane,
            admitted: [None; 3],
            promoted: [None; 3],
            samples: Vec::with_capacity(capacity),
        }
    }

    pub(super) fn advance(&mut self, tick: usize, command: Option<f64>) -> Result<()> {
        if let Some(speed) = command {
            self.lane.drive(speed)?;
        }
        if tick > 0 {
            self.lane.sim.step_exact(1)?;
        }
        self.observe(tick)
    }

    pub(super) fn observe(&mut self, tick: usize) -> Result<()> {
        if let Some(receipt) =
            support_loss_promotion::last_promotion(&self.lane.sim, self.lane.model)
            && receipt.iteration == SETTLE_STEPS + tick as u64
        {
            let cycle = sweep::DEPART_US
                .iter()
                .position(|&us| tick == (us / STEP_US) as usize + 1)
                .ok_or("automatic promotion outside a lift departure")?;
            if receipt.support != self.lane.pad
                || receipt.link != self.lane.foot
                || receipt.reason != support_loss_promotion::SupportLossReason::MotionStarted
                || self.lane.tier()? != Dynamic
            {
                return Err("wrong support-loss promotion receipt on a switched branch".into());
            }
            self.promoted[cycle] = Some(tick);
        }
        self.samples.push(self.lane.sample()?);
        for (cycle, earliest) in REQUEST.into_iter().enumerate() {
            let last = sweep::DEPART_US[cycle] as usize / STEP_US as usize;
            if self.mask & (1 << cycle) != 0
                && self.admitted[cycle].is_none()
                && (earliest..=last).contains(&tick)
            {
                match self.lane.demote() {
                    Ok(()) => self.admitted[cycle] = Some(tick),
                    Err(FidelityError::Physics(PhysicsError::TransitionDeferred { .. })) => {}
                    Err(e) => return Err(e.into()),
                }
            }
        }
        Ok(())
    }

    pub(super) fn backend(&self) -> &MultiFidelityBackend {
        self.lane
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("fixture has a local multifidelity backend")
    }
}

struct Cross {
    cut: usize,
    geometry: usize,
    motion: Option<usize>,
    trial: gz_physics_multifidelity::LandingTrial,
}

fn landing(trace: &[f64], first_tick: usize) -> Result<(usize, f64, f64, f64)> {
    let i = trace
        .iter()
        .position(|&impulse| impulse > 0.)
        .ok_or("no measured impact")?;
    let window = trace
        .get(i..i + WINDOW_STEPS)
        .ok_or("incomplete five-millisecond reaction window")?;
    Ok((first_tick + i, window[0], window[1], window.iter().sum()))
}

/// Bottom corners in foot coordinates, iterated by X then Y. The native pad is horizontal.
fn corner_gaps(s: Sample) -> [f64; 4] {
    let rotation = DQuat::from_array(s.foot.orientation);
    let [x, y, z] = FOOT_HALF;
    [(-x, -y), (-x, y), (x, -y), (x, y)].map(|(x, y)| {
        s.foot.position[2] + (rotation * DVec3::new(x, y, -z)).z - s.pad.position[2] - PAD_HALF[2]
    })
}

fn study() -> Result<Value> {
    let mut cases = (0..8)
        .map(|mask| Case::new(mask, END + 1))
        .collect::<Result<Vec<_>>>()?;
    let mut crosses = Vec::new();
    let mut offsets = Vec::new();
    let mut last_speed = 0.;
    for tick in 0..=END {
        let (_, speed) = sweep::command(tick.saturating_sub(1) as u64 * STEP_US, 0);
        let command = (speed != last_speed).then_some(speed);
        for c in &mut cases {
            c.advance(tick, command)?;
        }
        last_speed = speed;
        if CUTS.contains(&tick) {
            for geometry in [0, 7] {
                let target = &cases[geometry];
                if tick == CUTS[1] {
                    let gap = target.samples[tick].vertical_gap_m;
                    let other_gap = cases[7 - geometry].samples[tick].vertical_gap_m;
                    for factor in [0., 0.5, 1., 2.] {
                        let shift = factor * (gap - other_gap);
                        let tr = target.backend().landing_trial(
                            None,
                            target.lane.foot,
                            target.lane.pad,
                            END - tick,
                            Duration::from_micros(STEP_US),
                            shift,
                        )?;
                        let (landing_tick, first, second, window) =
                            landing(&tr.impulses, tick + 1)?;
                        offsets.push(json!({"source_mask":geometry,"cut_tick":tick,"factor":factor,
                            "original_gap_m":gap,"target_gap_m":other_gap,"pad_height_offset_m":shift,
                            "adjusted_gap_m":gap-shift,"landing_tick":landing_tick,"first_ns":first,
                            "second_ns":second,"window_ns":window}));
                    }
                }
                for motion in [None, Some(0), Some(7)] {
                    let source = motion.map(|id: usize| cases[id].backend());
                    let trial = target
                        .backend()
                        .landing_trial(
                            source,
                            target.lane.foot,
                            target.lane.pad,
                            END - tick,
                            Duration::from_micros(STEP_US),
                            0.,
                        )
                        .map_err(|e| {
                            format!("cut {tick} geometry {geometry} motion {motion:?}: {e}")
                        })?;
                    let donor = &cases[motion.unwrap_or(geometry)].samples[tick];
                    if trial.initial.map(Body::from) != [donor.foot, donor.upper, donor.pad] {
                        return Err(
                            "crossover failed to reproduce the three incoming link states".into(),
                        );
                    }
                    crosses.push(Cross {
                        cut: tick,
                        geometry,
                        motion,
                        trial,
                    });
                }
            }
        }
    }
    let mut masks = Vec::new();
    for c in &cases {
        let from = CUTS[0] + 1;
        if c.samples[from - 1].vertical_gap_m < 0.015 || c.samples[from - 1].normal_impulse_ns != 0.
        {
            return Err("third flight was not measured".into());
        }
        let impulses = c.samples[from..]
            .iter()
            .map(|s| s.normal_impulse_ns)
            .collect::<Vec<_>>();
        let (tick, first, second, window) = landing(&impulses, from)?;
        let realized = c
            .admitted
            .iter()
            .enumerate()
            .fold(0, |mask, (i, a)| mask | (u8::from(a.is_some()) << i));
        masks.push(
            json!({"requested_mask": c.mask, "realized_mask":realized,"admission_ticks":c.admitted, "promotion_ticks":c.promoted,
            "landing_tick":tick, "first_ns":first,"second_ns":second,"window_ns":window,
            "incoming":c.samples[tick-1],
            "before_hold_requests":REQUEST.map(|i| c.samples[i]),
            "before_departures":sweep::DEPART_US.map(|us| c.samples[(us / STEP_US) as usize]),
            "sole_corner_gaps_m":corner_gaps(c.samples[tick-1])}),
        );
    }
    let mut crossovers = Vec::new();
    let label = |mask| if mask == 0 { "reference" } else { "switched" };
    for c in &crosses {
        let tr = &c.trial;
        let (tick, first, second, window) = landing(&tr.impulses, c.cut + 1)?;
        let live = &cases[c.geometry].samples;
        let max_source_delta = tr
            .impulses
            .iter()
            .zip(&live[c.cut + 1..])
            .map(|(j, s)| (j - s.normal_impulse_ns).abs())
            .fold(0.0_f64, f64::max);
        let live_end = live.last().unwrap();
        if c.motion.is_none()
            && (max_source_delta != 0.
                || tr.final_states.map(Body::from) != [live_end.foot, live_end.upper, live_end.pad])
        {
            return Err(format!(
                "untreated native continuation changed live history by {max_source_delta} N s"
            )
            .into());
        }
        let points = tr
            .incoming_points
            .iter()
            .map(|p| {
                json!({
            "local_p1_m":p.local_p1,"local_p2_m":p.local_p2,"stored_gap_m":p.gap_m,
            "warm_normal_ns":p.warm_normal_ns,"warm_tangent_world_ns":p.warm_tangent_world_ns,
            "warm_twist_nm_s":p.warm_twist_nm_s})
            })
            .collect::<Vec<_>>();
        crossovers.push(json!({"cut_tick":c.cut,"geometry":label(c.geometry),"motion":label(c.motion.unwrap_or(c.geometry)),
            "treatment":if c.motion.is_some() { "copied" } else { "native" },
            "landing_tick":tick,"first_ns":first,"second_ns":second,"window_ns":window,
            "max_impulse_delta_vs_receiver_live_ns":max_source_delta,
            "initial":tr.initial.map(Body::from),"final_states":tr.final_states.map(Body::from),
            "generalized_velocity":tr.generalized_velocity,"joint_coordinates":tr.joint_coordinates,
            "incoming_manifolds":tr.incoming_manifolds,"incoming_points":points.len(),"point_cache":points,
            "incoming_warmstart":{"normal_ns":tr.incoming_warmstart.normal_ns,"tangent_on_foot_ns":tr.incoming_warmstart.tangent_on_a_ns,
            "points":tr.incoming_warmstart.points},"pad_first_in_pair":tr.pad_first_in_pair,
            "impulses_ns":tr.impulses}));
    }
    Ok(
        json!({"step_us":STEP_US,"window_us":5000,"cut_ticks":CUTS,"request_ticks":REQUEST,"masks":masks,"crossovers":crossovers,"height_sensitivity":offsets}),
    )
}

pub(super) fn write(output: &Path) -> Result<()> {
    let summary = study()?;
    fs::create_dir_all(output)?;
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossed_histories_share_native_motion_and_bracket_the_original_error() -> Result<()> {
        let report = study()?;
        let masks = report["masks"].as_array().unwrap();
        assert_eq!(masks.len(), 8);
        for mask in masks {
            assert_eq!(mask["requested_mask"], mask["realized_mask"]);
            assert_eq!(mask["landing_tick"], 1424);
            for (i, us) in sweep::DEPART_US.into_iter().enumerate() {
                let held = mask["realized_mask"].as_u64().unwrap() & (1 << i) != 0;
                assert_eq!(
                    mask["promotion_ticks"][i],
                    json!(held.then_some(us / STEP_US + 1))
                );
            }
            let corners = mask["sole_corner_gaps_m"].as_array().unwrap();
            let minimum = corners
                .iter()
                .map(|x| x.as_f64().unwrap())
                .fold(f64::INFINITY, f64::min);
            assert!((minimum - mask["incoming"]["vertical_gap_m"].as_f64().unwrap()).abs() < 1e-14);
        }
        let first = |mask: usize| masks[mask]["first_ns"].as_f64().unwrap();
        assert!(first(4) < first(7) && first(7) < first(0) && first(0) < first(3));
        let rows = report["crossovers"].as_array().unwrap();
        assert_eq!(rows.len(), 12);
        for row in rows {
            assert_eq!(row["landing_tick"], 1424);
            let points = row["point_cache"].as_array().unwrap();
            assert_eq!(points.len(), 4);
            for p in points {
                assert_eq!(p["warm_normal_ns"], 0.);
                assert_eq!(p["warm_twist_nm_s"], 0.);
                assert_eq!(p["warm_tangent_world_ns"], json!([0., 0., 0.]));
            }
            let find = |geometry: &Value| {
                rows.iter()
                    .find(|r| {
                        r["cut_tick"] == row["cut_tick"]
                            && r["geometry"] == *geometry
                            && r["motion"] == row["motion"]
                            && r["treatment"] == "copied"
                    })
                    .unwrap()
            };
            let sham = find(&row["motion"]);
            assert_eq!(row["initial"], sham["initial"]);
            assert_eq!(row["generalized_velocity"], sham["generalized_velocity"]);
            assert_eq!(row["joint_coordinates"], sham["joint_coordinates"]);
            let native_or_sham = row["geometry"] == row["motion"];
            let delta = row["max_impulse_delta_vs_receiver_live_ns"]
                .as_f64()
                .unwrap();
            if native_or_sham && (row["cut_tick"] == CUTS[1] || row["treatment"] == "native") {
                assert_eq!(delta, 0.);
            } else if native_or_sham {
                assert!(delta < 5e-5);
            } else {
                let gap =
                    (row["first_ns"].as_f64().unwrap() - sham["first_ns"].as_f64().unwrap()).abs();
                assert!(gap < 0.0002 && delta > 0.05);
            }
        }
        for row in report["height_sensitivity"].as_array().unwrap() {
            if row["factor"] == 0. {
                let id = row["source_mask"].as_u64().unwrap() as usize;
                assert_eq!(row["first_ns"], masks[id]["first_ns"]);
                assert_eq!(row["window_ns"], masks[id]["window_ns"]);
            }
            if row["factor"] == 1. {
                assert_eq!(row["adjusted_gap_m"], row["target_gap_m"]);
            }
        }
        Ok(())
    }

    #[test]
    fn landing_trial_rejects_parked_support_before_copying() -> Result<()> {
        let lane = Lane::new_at(STEP_US)?;
        let before = lane.sample()?;
        let backend = lane
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        for (steps, dt, offset) in [
            (10, STEP_US, 0.),
            (257, STEP_US, 0.),
            (1, 0, 0.),
            (1, STEP_US, f64::NAN),
        ] {
            assert!(
                backend
                    .landing_trial(
                        None,
                        lane.foot,
                        lane.pad,
                        steps,
                        Duration::from_micros(dt),
                        offset
                    )
                    .is_err()
            );
        }
        assert!(lane.sample()? == before);
        Ok(())
    }
}
