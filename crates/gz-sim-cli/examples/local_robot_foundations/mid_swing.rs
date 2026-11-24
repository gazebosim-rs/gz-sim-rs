// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite downward tugs during the airborne portion of the rotary transfer.
use super::*;
use crate::envelope::Tactile;
use glam::DQuat;
use gz_sim_scene::queue_link_world_wrench;
use std::{collections::BTreeMap, time::Duration};

const MASS_KG: f64 = 0.2;
const FORCE_N: f64 = 6.;
const PULL_TICKS: usize = 65; // Fixture clock is 1 ms.

#[derive(Debug, PartialEq, Serialize)]
struct MotionState {
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
}

#[derive(Debug, PartialEq, Serialize)]
struct Incoming {
    /// Rotor, palm, left pad, right pad and parcel. Each row contains position,
    /// quaternion, linear velocity and angular velocity in world coordinates.
    bodies: Vec<MotionState>,
    pad_normal_n: [f64; 2],
    pad_slip_m_s: [f64; 2],
    station_ns: [f64; 3],
    /// Separating-axis clearance from the source (Y) and target (X).
    stand_gaps_m: [f64; 2],
}

#[derive(Debug, Serialize)]
pub(super) struct Swing {
    start_tick: usize,
    force_n: f64,
    applied_impulse_ns: f64,
    incoming: Option<Incoming>,
    initial_relative_z_m: f64,
    first_slip_tick: Option<usize>,
    peak_drop_m: f64,
    pub(super) parcel_obstacle_impulse_ns: f64,
    set_down_yaw_rad: Option<f64>,
    min_support_n_before_release: Option<f64>,
    #[serde(skip)]
    recent_support_n: [f64; 20],
    pub(super) captures: BTreeMap<&'static str, usize>,
}
impl Swing {
    fn new(start_tick: usize, force_n: f64) -> Self {
        Self {
            start_tick,
            force_n,
            applied_impulse_ns: 0.,
            incoming: None,
            initial_relative_z_m: 0.,
            first_slip_tick: None,
            peak_drop_m: 0.,
            parcel_obstacle_impulse_ns: 0.,
            set_down_yaw_rad: None,
            min_support_n_before_release: None,
            recent_support_n: [0.; 20],
            captures: BTreeMap::new(),
        }
    }
    pub(super) fn pull(&mut self, sim: &mut Simulation, parcel: u64, tick: usize) -> Result<()> {
        if self.force_n > 0. && (self.start_tick..self.start_tick + PULL_TICKS).contains(&tick) {
            queue_link_world_wrench(sim, parcel, [0., 0., -self.force_n], [0.; 3])?;
            self.applied_impulse_ns += self.force_n * sim.step_duration().as_secs_f64();
        }
        Ok(())
    }
    pub(super) fn observe(
        &mut self,
        sim: &Simulation,
        links: &[u64],
        tick: usize,
        parcel: BodyState,
        palm: BodyState,
        sample: &Tactile,
    ) -> Result<()> {
        let z = parcel.position[2] - palm.position[2];
        if tick + 1 == self.start_tick {
            self.initial_relative_z_m = z;
            let rotor = entity_id(sim, "default::arm::rotor")?;
            let mut bodies = vec![];
            for id in [rotor, links[7], links[0], links[1], links[6]] {
                let b = state(sim, id)?;
                bodies.push(MotionState {
                    position: b.position,
                    orientation: b.orientation,
                    velocity: b.velocity,
                    angular_velocity: b.angular_velocity,
                });
            }
            let rotation = DQuat::from_array(parcel.orientation);
            let extent = (rotation * DVec3::X).abs() * 0.043
                + (rotation * DVec3::Y).abs() * 0.0375
                + (rotation * DVec3::Z).abs() * 0.05;
            self.incoming = Some(Incoming {
                bodies,
                pad_normal_n: sample.pads.map(|s| s.normal_force_n),
                pad_slip_m_s: sample.pads.map(|s| s.slip_speed_m_s),
                station_ns: sample.station_ns,
                stand_gaps_m: [
                    parcel.position[1] - extent.y - 0.09,
                    parcel.position[0] - extent.x - 0.10,
                ],
            });
        }
        // Opening the jaws on a stand later is deliberate. Measure the tug and
        // its following 100 ms against the last pre-tug parcel/palm separation.
        if (self.start_tick..self.start_tick + PULL_TICKS + 100).contains(&tick) {
            let drop = self.initial_relative_z_m - z;
            self.peak_drop_m = self.peak_drop_m.max(drop);
            if drop > 0.00025 {
                self.first_slip_tick.get_or_insert(tick);
            }
        }
        Ok(())
    }
    pub(super) fn record_recovery(
        &mut self,
        tick: usize,
        park: Option<Park>,
        release: Option<usize>,
        sample: &Tactile,
        dt: Duration,
    ) {
        if let Some(park) = park {
            self.set_down_yaw_rad.get_or_insert(park.yaw);
            self.recent_support_n[tick % 20] =
                sample.station_up_ns[usize::from(park.goal > 0.)] / dt.as_secs_f64();
            if release == Some(tick) {
                self.min_support_n_before_release = Some(
                    self.recent_support_n
                        .iter()
                        .copied()
                        .fold(f64::INFINITY, f64::min),
                );
            }
        }
    }
    pub(super) fn capture_tick(&self, tick: usize, trial: &Trial) -> bool {
        tick + 1 == self.start_tick
            || ((self.start_tick..=self.start_tick + 150).contains(&tick) && tick.is_multiple_of(5))
            || [trial.alarm_tick, trial.set_down_tick, trial.release_tick].contains(&Some(tick))
    }
    pub(super) fn label_frame(&mut self, tick: usize, trial: &Trial, frame: usize) {
        for (label, when) in [
            ("before_tug", Some(self.start_tick - 1)),
            ("tug", Some(self.start_tick)),
            ("alarm", trial.alarm_tick),
            ("set_down", trial.set_down_tick),
            ("release", trial.release_tick),
            ("settled", Some(6100)),
        ] {
            if when == Some(tick) {
                self.captures.insert(label, frame);
            }
        }
    }
}

#[derive(Debug, Serialize)]
struct Run {
    trial: Trial,
    swing: Swing,
}
fn run_case(start: usize, force: f64, feedback: bool, path: Option<&Path>) -> Result<Run> {
    let mut swing = Swing::new(start, force);
    let trial = trial_with_tug(MASS_KG, 0.35, feedback, path, Some(&mut swing))?;
    Ok(Run { trial, swing })
}

#[derive(Debug, Serialize)]
struct Comparison {
    stand: &'static str,
    quiet: Run,
    fixed: Run,
    guarded: Run,
}
impl Comparison {
    fn check(&self) -> Result<()> {
        let Self {
            stand,
            quiet,
            fixed,
            guarded,
        } = self;
        let incoming = guarded
            .swing
            .incoming
            .as_ref()
            .ok_or("missing pre-tug observation")?;
        if fixed.swing.incoming.as_ref() != Some(incoming)
            || quiet.swing.incoming.as_ref() != Some(incoming)
            || !incoming.stand_gaps_m.iter().all(|&gap| gap > 0.)
            || !incoming.pad_normal_n.iter().all(|&n| n > 0.05)
            || incoming.station_ns != [0.; 3]
        {
            return Err(
                "the trials must begin the tug in the same unsupported grasp, clear of both stands"
                    .into(),
            );
        }
        let alarm = guarded.trial.alarm_tick.ok_or("missing tactile alarm")?;
        let park = guarded
            .trial
            .set_down_tick
            .ok_or("missing guarded set-down")?;
        let yaw = guarded
            .swing
            .set_down_yaw_rad
            .ok_or("missing set-down yaw")?;
        let release = guarded
            .trial
            .release_tick
            .ok_or("missing supported release")?;
        let start = guarded.swing.start_tick;
        if quiet.trial.outcome != "placed"
            || quiet.trial.alarm_tick.is_some()
            || quiet.swing.first_slip_tick.is_some()
            || (guarded.swing.applied_impulse_ns - FORCE_N * PULL_TICKS as f64 * 0.001).abs()
                > 1e-12
            || fixed.swing.applied_impulse_ns != guarded.swing.applied_impulse_ns
            || fixed.trial.alarm_tick != Some(alarm)
            || !(start..start + PULL_TICKS).contains(&alarm)
            || !(alarm + 1..start + PULL_TICKS).contains(&park)
            || guarded.swing.first_slip_tick.is_none()
            || fixed.swing.peak_drop_m <= guarded.swing.peak_drop_m
            || fixed.swing.parcel_obstacle_impulse_ns <= 0.
        {
            return Err(
                "mid-swing tug, tactile response or fixed-pressure collision failed to reproduce"
                    .into(),
            );
        }
        let source = *stand == "source";
        let expected_support = if source {
            guarded.trial.settled_source_ticks
        } else {
            guarded.trial.settled_target_ticks
        };
        if (yaw <= TURN * 0.5) != source
            || release < park + 1520
            || expected_support != 200
            || guarded.trial.outcome != "set_down"
            || guarded.trial.max_jaw_command_n > 9.
            || guarded
                .swing
                .min_support_n_before_release
                .is_none_or(|force| force < MASS_KG * 9.81 * 0.8)
        {
            return Err(format!(
                "guarded {stand} return failed placement or support qualification"
            )
            .into());
        }
        Ok(())
    }
}
pub(crate) fn run(output: &Path) -> Result<()> {
    let mut results = vec![];
    for (stand, start) in [("source", 1860), ("target", 2020)] {
        let fixed = (stand == "source").then(|| output.join("fixed"));
        let comparison = Comparison {
            stand,
            quiet: run_case(start, 0., true, None)?,
            fixed: run_case(start, FORCE_N, false, fixed.as_deref())?,
            guarded: run_case(start, FORCE_N, true, Some(&output.join(stand)))?,
        };
        comparison.check()?;
        results.push(comparison);
    }
    write_json(
        &output.join("comparison.json"),
        &serde_json::to_value(results)?,
    )
}

#[cfg(test)]
#[path = "mid_swing_tests.rs"]
mod tests;
