// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the changing-contact pool with eight serial native controls across ownership changes.
#[path = "changing_handoff.rs"]
mod handoff;
#[path = "native_contact_control.rs"]
mod native;
use crate::fixture;
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, ContactImpulse, GroundPlane, GroundSupport, SimulationStep},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
pub use handoff::SplitHandoff;
use handoff::WireUsage;
use serde::Serialize;
use std::{collections::BTreeMap, error::Error as StdError, path::Path};

pub type Result<T> = std::result::Result<T, Box<dyn StdError>>;
/// Faults are injected at deterministic public barriers. Worker replacement is
/// performed after its named committed tick and before the next advance.
#[derive(Clone, Copy)]
pub struct Schedule {
    pub migrate: u64,
    pub retry: u64,
    pub replace: u64,
    pub restore: u64,
    pub pad: usize,
}

pub const CHANGING: Schedule = Schedule {
    migrate: 105,
    retry: 120,
    replace: 140,
    restore: 185,
    pad: 1,
};
pub const LANDING: Schedule = Schedule {
    migrate: 145,
    retry: 160,
    replace: 219,
    restore: 225,
    pad: 0,
};

impl Schedule {
    pub fn events(self) -> [(u64, &'static str); 4] {
        [
            (self.migrate, "handoff"),
            (self.retry, "retry"),
            (self.replace, "replacement"),
            (self.restore, "restore"),
        ]
    }
}
#[derive(Serialize)]
pub struct Frame<'a> {
    pub tick: u64,
    pub bodies: Vec<&'a Body>,
    pub owners: Vec<u8>,
    pub pairs: Vec<&'a ContactImpulse>,
    pub floors: Vec<&'a GroundSupport>,
    pub pad_receipts: [usize; 2],
    pub loaded_pairs: [usize; 2],
}
impl<'a> Frame<'a> {
    fn read(frame: &'a PoolFrame, pads: [u64; 2]) -> Self {
        let mut bodies: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.bodies)
            .collect();
        let mut pairs: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .collect();
        let mut floors: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .collect();
        bodies.sort_unstable_by_key(|b| b.id);
        pairs.sort_unstable_by_key(|p| (p.a, p.b));
        floors.sort_unstable_by_key(|f| f.body);
        let owners = bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect();
        let pad_receipts = pads.map(|id| pairs.iter().filter(|p| p.a == id || p.b == id).count());
        let loaded_pairs = pads.map(|id| {
            pairs
                .iter()
                .filter(|p| (p.a == id || p.b == id) && p.normal_impulse > 0.)
                .count()
        });
        Self {
            tick: frame.tick,
            bodies,
            owners,
            pairs,
            floors,
            pad_receipts,
            loaded_pairs,
        }
    }
}
#[derive(Default, Serialize)]
pub struct Report {
    pub ticks: u64,
    pub exact_body_states: usize,
    pub exact_pair_receipts: usize,
    pub exact_floor_receipts: usize,
    pub events: [u64; 4],
    pub migrant_ids: Vec<u64>,
    pub replacement_pids: [u32; 2],
    pub resumed_ticks: u64,
    pub changes_after_restore: usize,
    pub loaded_pairs_at_events: Vec<[usize; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub helpers: Option<HelperUsage>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wire: Option<WireUsage>,
}
pub struct Scenario {
    pub schedule: Schedule,
    pub placement: fixture::Placement,
    pub helper_limit: Option<usize>,
    /// Exercise the complete single-to-split four-group move with this native encoding.
    pub split_wire: Option<SplitHandoff>,
}

#[derive(Default, Serialize)]
pub struct HelperUsage {
    pub limit: usize,
    /// Actual worker-index bit masks granted on successful steps in each ownership interval.
    pub through_handoff: BTreeMap<u8, usize>,
    pub after_handoff_through_restore: BTreeMap<u8, usize>,
    pub after_restore: BTreeMap<u8, usize>,
}
pub fn qualify(
    worker: &Path,
    checkpoint: &Path,
    groups: &[Vec<Body>],
    schedule: Schedule,
    record: impl FnMut(&Frame<'_>) -> Result<()>,
) -> Result<Report> {
    qualify_scenario(
        worker,
        checkpoint,
        groups,
        Scenario {
            schedule,
            placement: fixture::Placement::Single,
            helper_limit: None,
            split_wire: None,
        },
        record,
    )
}

pub fn qualify_scenario(
    worker: &Path,
    checkpoint: &Path,
    groups: &[Vec<Body>],
    scenario: Scenario,
    mut record: impl FnMut(&Frame<'_>) -> Result<()>,
) -> Result<Report> {
    let Scenario {
        schedule,
        placement,
        helper_limit,
        split_wire,
    } = scenario;
    let ground = Some(GroundPlane::new(0.)?);
    let mut controls: Vec<_> = groups
        .iter()
        .map(|g| native::Control::new(g, ground, fixture::GRAVITY))
        .collect::<Result<_>>()?;
    let ids: Vec<_> = groups.iter().flatten().map(|b| b.id).collect();
    let cargo: Vec<_> = groups.iter().filter(|g| g.len() == 25).collect();
    let [first, second] = cargo.as_slice() else {
        return Err("expected two loaded pads".into());
    };
    let pads = [first[0].id, second[0].id];
    let migrant = cargo[schedule.pad];
    let migrating_groups: Vec<&Vec<Body>> = if split_wire.is_some() {
        if placement != fixture::Placement::Single || schedule.pad != 1 {
            return Err("full split requires single-owner start and second slab".into());
        }
        groups
            .iter()
            .zip(fixture::Placement::Split.owners(groups))
            .filter_map(|(g, w)| (w == 1).then_some(g))
            .collect()
    } else {
        vec![migrant]
    };
    let mut report = Report {
        ticks: fixture::END,
        events: schedule.events().map(|(tick, _)| tick),
        migrant_ids: migrating_groups
            .iter()
            .flat_map(|g| g.iter().map(|b| b.id))
            .collect(),
        wire: split_wire.map(SplitHandoff::usage),
        resumed_ticks: fixture::END - schedule.restore,
        helpers: helper_limit.map(|limit| HelperUsage {
            limit,
            ..Default::default()
        }),
        ..Default::default()
    };
    let worker_seeds = placement.worker_seeds(groups);
    let mut expected_owners: BTreeMap<_, _> = worker_seeds
        .iter()
        .enumerate()
        .flat_map(|(w, bodies)| bodies.iter().map(move |b| (b.id, w as u8)))
        .collect();
    let target = 1 - expected_owners[&migrant[0].id];
    let mut pool = WorkerPool::spawn(
        worker,
        0x6368616e6765,
        worker_seeds,
        SimulationStep::Ms10,
        ground,
        1024,
    )?;
    #[cfg(test)]
    pool.set_native_work_timeout(std::time::Duration::from_secs(5))?;
    pool.set_local_helper_limit(helper_limit);
    if let Some(wire) = &mut report.wire {
        wire.negotiate(&mut pool)?;
    }
    let mut previous_pairs = [0; 2];
    record(&Frame::read(pool.frame(), pads))?;
    for tick in 1..=fixture::END {
        let loads = fixture::wrenches(tick, groups);
        if tick == schedule.retry {
            let barrier = pool.frame().clone();
            let refined = pool.last_island_refined_bodies().map(<[_]>::to_vec);
            let grants = pool.last_local_helper_mask();
            pool.drop_next_acknowledgement(target)?;
            if loads.is_empty()
                || !matches!(
                    pool.advance_islands_in_field(fixture::GRAVITY, &loads),
                    Err(Error::Transport(_))
                )
                || pool.frame() != &barrier
                || pool.last_island_refined_bodies() != refined.as_deref()
                || pool.last_local_helper_mask() != grants
            {
                return Err(
                    "lost acknowledgement on the launch changed the published barrier".into(),
                );
            }
        }
        pool.advance_islands_in_field(fixture::GRAVITY, &loads)?;
        if let Some(usage) = &mut report.helpers {
            let mask = pool
                .last_local_helper_mask()
                .ok_or("missing committed worker grants")?;
            if mask.count_ones() as usize > usage.limit {
                return Err(format!("helper budget exceeded at tick {tick}").into());
            }
            let counts = if tick <= schedule.migrate {
                &mut usage.through_handoff
            } else if tick <= schedule.restore {
                &mut usage.after_handoff_through_restore
            } else {
                &mut usage.after_restore
            };
            *counts.entry(mask).or_default() += 1;
        }
        if tick == schedule.migrate {
            if let (Some(wire), Some(mode)) = (&mut report.wire, split_wire) {
                wire.execute(
                    &mut pool,
                    &migrating_groups,
                    target,
                    pads[schedule.pad],
                    mode,
                )?;
            } else {
                pool.transfer_component(migrant[0].id, target)?;
            }
            for id in &report.migrant_ids {
                expected_owners.insert(*id, target);
            }
        }
        if tick == schedule.replace || tick == schedule.restore {
            let before = pool.frame().clone();
            if tick == schedule.replace {
                let old = pool.worker_pid(target).ok_or("missing owner")?;
                pool.terminate_worker(target)?;
                pool.recover(target)?;
                let new = pool.worker_pid(target).ok_or("missing replacement")?;
                if new == old {
                    return Err("expected fresh worker".into());
                }
                report.replacement_pids = [old, new];
            } else {
                let old: Vec<_> = (0..4).map(|w| pool.worker_pid(w).unwrap()).collect();
                pool.save_native_recovery(checkpoint)?;
                drop(pool);
                pool = PoolNativeRecovery::open(checkpoint)?.resume(worker)?;
                if pool.last_local_helper_mask().is_some() {
                    return Err("restored coordinator exposed stale last-step grants".into());
                }
                #[cfg(test)]
                pool.set_native_work_timeout(std::time::Duration::from_secs(5))?;
                pool.set_local_helper_limit(helper_limit);
                if (0..4).any(|w| old.contains(&pool.worker_pid(w).unwrap())) {
                    return Err("restore reused a prior worker".into());
                }
            }
            if let Some(wire) = &mut report.wire {
                wire.negotiate(&mut pool)?;
            }
            if &before != pool.frame() {
                return Err(format!("recovery changed barrier {tick}").into());
            }
        }
        if pool.last_island_refined_bodies() != Some(ids.as_slice()) {
            return Err(format!("unexpected refinement at {tick}").into());
        }
        let f = Frame::read(pool.frame(), pads);
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for (g, c) in groups.iter().zip(&mut controls) {
            let local: Vec<_> = loads
                .iter()
                .filter(|w| g.binary_search_by_key(&w.id, |b| b.id).is_ok())
                .cloned()
                .collect();
            let (p, s) = c.advance(10, &local, &[])?;
            pairs.extend(p);
            floors.extend(s);
        }
        let native_bodies: Vec<_> = controls.iter().flat_map(|c| &c.bodies).collect();
        if f.tick != tick
            || bincode::serialize(&(&f.bodies, &f.pairs, &f.floors))?
                != bincode::serialize(&(native_bodies, &pairs, &floors))?
        {
            return Err(format!("native state or contacts diverged at {tick}").into());
        }
        for (b, &owner) in f.bodies.iter().zip(&f.owners) {
            if owner != expected_owners[&b.id] {
                return Err(format!("wrong owner for {} at {tick}", b.id).into());
            }
        }
        if tick >= fixture::SETTLED
            && f.floors.iter().filter(|s| s.normal_impulse > 0.).count() != groups.len()
        {
            return Err(format!("lost ground support at {tick}").into());
        }
        if tick == schedule.migrate && f.loaded_pairs[schedule.pad] != migrant.len() - 1 {
            return Err("migrating cargo was not loaded".into());
        }
        if report.events.contains(&tick) {
            report.loaded_pairs_at_events.push(f.loaded_pairs);
        }
        if tick > schedule.restore && f.loaded_pairs != previous_pairs {
            report.changes_after_restore += 1;
        }
        previous_pairs = f.loaded_pairs;
        report.exact_body_states += f.bodies.len();
        report.exact_pair_receipts += f.pairs.len();
        report.exact_floor_receipts += f.floors.len();
        record(&f)?;
    }
    if report.changes_after_restore < 3 {
        return Err("insufficient post-restore contact changes".into());
    }
    Ok(report)
}
