// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare loaded native worlds in a migrating, fault-injected pool with independent solvers.
#[path = "native_contact_control.rs"]
mod native;

use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, ContactImpulse, GroundPlane, GroundSupport, SimulationStep},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use std::{error::Error as StdError, path::Path};

pub type Result<T> = std::result::Result<T, Box<dyn StdError>>;
const FIELD: [f64; 3] = [0., 0., -9.81];
pub const SETTLED: u64 = 110;
pub const HANDOFF: u64 = 125;
pub const RETRY: u64 = 135;
pub const REPLACEMENT: u64 = 145;
pub const RESTORE: u64 = 155;
pub const END: u64 = 210;

#[derive(Serialize)]
pub struct Frame<'a> {
    pub tick: u64,
    pub bodies: Vec<&'a Body>,
    pub owners: Vec<u8>,
    pub pairs: Vec<&'a ContactImpulse>,
    pub floors: Vec<&'a GroundSupport>,
}

impl<'a> Frame<'a> {
    fn read(frame: &'a PoolFrame) -> Self {
        let mut bodies = Vec::new();
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for w in &frame.workers {
            bodies.extend(&w.snapshot.bodies);
            pairs.extend(&w.snapshot.impulses);
            floors.extend(&w.snapshot.supports);
        }
        bodies.sort_unstable_by_key(|b: &&Body| b.id);
        pairs.sort_unstable_by_key(|p: &&ContactImpulse| (p.a, p.b));
        floors.sort_unstable_by_key(|p: &&GroundSupport| p.body);
        let owners = bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect();
        Self {
            tick: frame.tick,
            owners,
            bodies,
            pairs,
            floors,
        }
    }
}

#[derive(Serialize)]
pub struct Summary {
    pub compared_ticks: u64,
    pub exact_body_states: u64,
    pub fully_loaded_ticks: u64,
    pub handoff_barrier: u64,
    pub retried_commit: u64,
    pub replacement_barrier: u64,
    pub replaced_worker_pids: [u32; 2],
    pub archive_barrier: u64,
    pub archive_resumed_ticks: u64,
}

pub fn qualify(
    worker: &Path,
    checkpoint: &Path,
    groups: Vec<Vec<Body>>,
    migrant_group: usize,
    mut record: impl FnMut(&Frame<'_>) -> Result<()>,
) -> Result<Summary> {
    let plane = Some(GroundPlane::new(0.)?);
    let mut controls: Vec<_> = groups
        .iter()
        .map(|b| native::Control::new(b, plane, FIELD))
        .collect::<Result<_>>()?;
    let seeds = vec![groups.concat(), vec![], vec![], vec![]];
    let mut pool = WorkerPool::spawn(
        worker,
        0x6c6f63616c,
        seeds,
        SimulationStep::Ms10,
        plane,
        1024,
    )?;
    let ids: Vec<_> = groups.iter().flatten().map(|b| b.id).collect();
    let components: Vec<Vec<_>> = groups
        .iter()
        .map(|g| g.iter().map(|b| b.id).collect())
        .collect();
    let migrant = components
        .get(migrant_group)
        .ok_or("unknown migrating group")?;
    let edges: Vec<_> = components
        .iter()
        .flat_map(|group| group[1..].iter().map(|&id| (group[0], id)))
        .collect();
    let base_ids: Vec<_> = components.iter().map(|group| group[0]).collect();
    record(&Frame::read(pool.frame()))?;
    let mut pids = [0; 2];
    for tick in 1..=END {
        if tick == RETRY {
            let barrier = pool.frame().clone();
            let schedule = pool.last_island_refined_bodies().unwrap().to_vec();
            pool.drop_next_acknowledgement(1)?;
            if !matches!(
                pool.advance_islands_in_field(FIELD, &[]),
                Err(Error::Transport(_))
            ) || pool.frame() != &barrier
                || pool.last_island_refined_bodies() != Some(schedule.as_slice())
            {
                return Err(
                    "missing acknowledgement did not preserve the committed barrier".into(),
                );
            }
        }
        pool.advance_islands_in_field(FIELD, &[])?;
        if pool.last_island_refined_bodies() != Some(&ids) {
            return Err(format!("unexpected refinement schedule at {tick}").into());
        }
        if tick == HANDOFF {
            pool.transfer_component(migrant[0], 1)?;
        }
        if tick == REPLACEMENT || tick == RESTORE {
            let barrier = pool.frame().clone();
            if tick == REPLACEMENT {
                pids[0] = pool.worker_pid(1).unwrap();
                pool.terminate_worker(1)?;
                pool.recover(1)?;
                pids[1] = pool.worker_pid(1).unwrap();
                if pids[0] == pids[1] {
                    return Err("lost owner process was not replaced".into());
                }
            } else {
                pool.save_native_recovery(checkpoint)?;
                let old: Vec<_> = (0..4).map(|i| pool.worker_pid(i).unwrap()).collect();
                drop(pool);
                pool = PoolNativeRecovery::open(checkpoint)?.resume(worker)?;
                if (0..4).any(|i| old.contains(&pool.worker_pid(i).unwrap())) {
                    return Err("archive restore reused a process".into());
                }
            }
            if pool.frame() != &barrier {
                return Err(format!("recovery changed tick {tick} state").into());
            }
        }
        if pool.last_island_refined_bodies() != Some(&ids) {
            return Err(format!("lost refinement schedule at tick {tick}").into());
        }
        let sample = Frame::read(pool.frame());
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for control in &mut controls {
            let (p, f) = control.advance(10, &[], &[])?;
            pairs.extend(p);
            floors.extend(f);
        }
        let native_bodies: Vec<_> = controls.iter().flat_map(|world| &world.bodies).collect();
        if sample.tick != tick
            || bincode::serialize(&(&sample.bodies, &sample.pairs, &sample.floors))?
                != bincode::serialize(&(native_bodies, &pairs, &floors))?
        {
            return Err(format!("native state or load differs at barrier {tick}").into());
        }
        for (b, owner) in sample.bodies.iter().zip(&sample.owners) {
            if *owner != u8::from(tick >= HANDOFF && migrant.contains(&b.id)) {
                return Err(format!("wrong owner of body {} at tick {tick}", b.id).into());
            }
        }
        if tick >= SETTLED {
            let mut actual: Vec<_> = pool
                .frame()
                .workers
                .iter()
                .flat_map(|w| w.components.iter().map(|c| c.ids.clone()))
                .collect();
            actual.sort_unstable();
            if actual != components
                || pairs.iter().map(|p| (p.a, p.b)).ne(edges.iter().copied())
                || floors.iter().map(|p| p.body).ne(base_ids.iter().copied())
                || pairs
                    .iter()
                    .any(|p| !(p.normal_impulse > 0. && p.normal_impulse.is_finite()))
                || floors
                    .iter()
                    .any(|p| !(p.normal_impulse > 0. && p.normal_impulse.is_finite()))
            {
                return Err(format!("missing independent loaded stack at tick {tick}").into());
            }
        }
        record(&sample)?;
    }
    Ok(Summary {
        compared_ticks: END,
        exact_body_states: END * ids.len() as u64,
        fully_loaded_ticks: END - SETTLED + 1,
        handoff_barrier: HANDOFF,
        retried_commit: RETRY,
        replacement_barrier: REPLACEMENT,
        replaced_worker_pids: pids,
        archive_barrier: RESTORE,
        archive_resumed_ticks: END - RESTORE,
    })
}
