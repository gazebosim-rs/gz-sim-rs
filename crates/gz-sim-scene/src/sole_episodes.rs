// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional contact-episode recorder for the committed local sole scene.
use crate::damped_sole::{PanelStep, panel_steps};
use gz_sim_core::{Simulation, UpdateInfo};
use std::{collections::BTreeMap, time::Duration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactChange {
    Loaded,
    Unloaded,
    Transient,
    Removed,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactTotals {
    pub normal_impulse_ns: f64,
    pub normal_heat_j: f64,
    pub friction_work_j: f64,
    pub support_work_j: f64,
    pub peak_force_n: f64,
}
impl ContactTotals {
    fn add(&mut self, row: &PanelStep) {
        self.normal_impulse_ns += row.step.normal_impulse;
        self.normal_heat_j += row.step.dissipated_energy;
        self.friction_work_j += row.step.sliding_work;
        self.support_work_j += row.support_work_j;
        self.peak_force_n = self.peak_force_n.max(row.step.peak_normal_force);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ContactEvent {
    pub support: u64,
    pub epoch: u64,
    /// Endpoints bracket the transition on the public clock.
    pub earliest: Duration,
    pub latest: Duration,
    pub change: ContactChange,
    /// The completed episode on unload/removal, or the current tick on entry.
    pub totals: ContactTotals,
}
#[derive(Clone, Copy, Debug, Default)]
struct Plate {
    active: bool,
    episode: ContactTotals,
}

/// Call once per advancing public tick. Repeated reads at the same tick are
/// idempotent and can report a support removed while paused.
pub struct SoleEpisodes {
    enter: f64,
    leave: f64,
    previous: Option<(u64, u64)>,
    plates: BTreeMap<u64, Plate>,
}
#[derive(Debug, thiserror::Error)]
pub enum EpisodeError {
    #[error("contact observation requires a healthy committed clock")]
    Clock,
    #[error("the recorder missed a public tick; start a new recording at this boundary")]
    MissedTick,
}
impl SoleEpisodes {
    /// Thresholds in N, with a lower exit threshold to suppress small ripples.
    pub fn new(enter: f64, leave: f64) -> Option<Self> {
        (enter.is_finite() && leave.is_finite() && leave >= 0. && enter > leave).then_some(Self {
            enter,
            leave,
            previous: None,
            plates: BTreeMap::new(),
        })
    }
    pub fn loaded(&self, id: u64) -> bool {
        self.plates.get(&id).is_some_and(|s| s.active)
    }
    pub fn observe(&mut self, sim: &Simulation) -> Result<Vec<ContactEvent>, EpisodeError> {
        if sim.is_faulted() {
            return Err(EpisodeError::Clock);
        }
        let clock = sim
            .committed_update_info()
            .map_err(|_| EpisodeError::Clock)?;
        self.update(clock, panel_steps(sim))
    }
    fn update(
        &mut self,
        clock: UpdateInfo,
        rows: &[PanelStep],
    ) -> Result<Vec<ContactEvent>, EpisodeError> {
        let current = (clock.epoch, clock.iteration);
        if let Some((epoch, tick)) = self.previous {
            if epoch != clock.epoch {
                self.plates.clear()
            } else if clock.iteration < tick || clock.iteration - tick > 1 {
                return Err(EpisodeError::MissedTick);
            }
        }
        let repeated = self.previous == Some(current);
        let event = |id, change, totals| ContactEvent {
            support: id,
            epoch: clock.epoch,
            earliest: if repeated {
                clock.sim_time
            } else {
                clock.start_time
            },
            latest: clock.sim_time,
            change,
            totals,
        };
        let mut result = vec![];
        self.plates.retain(|id, s| {
            let keep = rows.binary_search_by_key(id, |r| r.support).is_ok();
            if s.active && !keep {
                result.push(event(*id, ContactChange::Removed, s.episode))
            }
            keep
        });
        if !repeated {
            for r in rows {
                let p = self.plates.entry(r.support).or_default();
                if p.active {
                    p.episode.add(r);
                    if r.step.contact.normal_force <= self.leave {
                        result.push(event(r.support, ContactChange::Unloaded, p.episode));
                        *p = Plate::default();
                    }
                } else if r.step.contact.normal_force >= self.enter {
                    p.active = true;
                    p.episode.add(r);
                    result.push(event(r.support, ContactChange::Loaded, p.episode));
                } else if r.step.peak_normal_force >= self.enter && r.step.normal_impulse > 0. {
                    let mut episode = ContactTotals::default();
                    episode.add(r);
                    result.push(event(r.support, ContactChange::Transient, episode));
                }
            }
        }
        self.previous = Some(current);
        Ok(result)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};
    use gz_physics::sole_contact::{SoleDeck, SoleStep};
    fn row(id: u64, f: f64, peak: f64, j: f64) -> PanelStep {
        let mut step = SoleStep::default();
        step.contact.normal_force = f;
        step.peak_normal_force = peak;
        step.normal_impulse = j;
        PanelStep {
            support: id,
            deck: SoleDeck {
                center: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                half_extents: DVec3::ONE,
                velocity: DVec3::ZERO,
            },
            step,
            support_work_j: 0.,
            spin_work_j: 0.,
            angular_velocity: DVec3::ZERO,
        }
    }
    fn tick(i: u64, epoch: u64) -> UpdateInfo {
        UpdateInfo {
            iteration: i,
            epoch,
            start_time: Duration::from_millis(i.saturating_sub(1)),
            dt: Duration::from_millis(1),
            sim_time: Duration::from_millis(i),
            paused: false,
        }
    }
    #[test]
    fn hysteresis_idempotence_and_within_tick_contact() {
        let mut c = SoleEpisodes::new(2., 0.5).unwrap();
        let mut rows = [row(1, 3., 3., 0.003), row(2, 0., 4., 0.001)];
        let first = c.update(tick(1, 0), &rows).unwrap();
        assert_eq!(first[0].change, ContactChange::Loaded);
        assert_eq!(first[1].change, ContactChange::Transient);
        assert!(c.update(tick(1, 0), &rows).unwrap().is_empty());
        rows[0] = row(1, 1., 2., 0.002);
        assert!(c.update(tick(2, 0), &rows[..1]).unwrap().is_empty());
        rows[0] = row(1, 0.4, 1., 0.0008);
        let done = c.update(tick(3, 0), &rows[..1]).unwrap();
        assert_eq!(done[0].change, ContactChange::Unloaded);
        assert!((done[0].totals.normal_impulse_ns - 0.0058).abs() < 1e-15);
        assert!(!c.loaded(1));
    }
    #[test]
    fn paused_removal_epoch_reset_and_skipped_tick() {
        let mut c = SoleEpisodes::new(2., 0.5).unwrap();
        let rows = [row(1, 3., 3., 0.002), row(2, 4., 4., 0.003)];
        c.update(tick(1, 0), &rows).unwrap();
        let removed = c.update(tick(1, 0), &rows[1..]).unwrap();
        assert_eq!(removed[0].change, ContactChange::Removed);
        assert_eq!(removed[0].earliest, removed[0].latest);
        assert!(!c.loaded(1) && c.loaded(2));
        assert!(matches!(
            c.update(tick(3, 0), &[]),
            Err(EpisodeError::MissedTick)
        ));
        assert!(c.loaded(2));
        c.update(tick(0, 1), &[]).unwrap();
        assert!(!c.loaded(2));
    }
}
