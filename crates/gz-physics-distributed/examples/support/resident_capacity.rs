// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::native;

use gz_physics_distributed::{
    physics::{
        Body, GroundPlane, NativeSolverProfile, PendingImpulse, Primitive, SimulationStep, fixture,
    },
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::{error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];

/// Four 4×4×4 sphere lattices. Consecutive cells approach each other. The
/// 15 m center spacing isolates their dynamics throughout the sampled interval.
pub fn lattice(site: usize) -> Vec<Body> {
    (0..64)
        .map(|cell| {
            let offset = [cell % 4, cell / 4 % 4, cell / 16].map(|v| (v as f64 - 1.5) * 0.44);
            Body {
                id: (site * 64 + cell + 1) as u64,
                primitive: Primitive::Sphere,
                size: [0.4; 3],
                position: [site as f64 * 15. + offset[0], offset[1], 0.91 + offset[2]],
                linear: [-4. * offset[0], -4. * offset[1], -4. * offset[2] - 5.],
                ..fixture()[1].clone()
            }
        })
        .collect()
}

#[derive(Default, Debug, serde::Serialize)]
pub struct Report {
    pub peak_pairs: usize,
    pub peak_floor: usize,
    pub peak_worker_barrier_reply_bytes: u64,
    pub peak_advance_us: u128,
    pub full_archive_bytes: usize,
    pub native_bytes: Vec<usize>,
    pub events: Vec<serde_json::Value>,
}

pub fn exercise(
    worker: &Path,
    packed: bool,
    scratch: &Path,
    last_tick: u64,
    mut sample: impl FnMut(&WorkerPool) -> Result<()>,
) -> Result<Report> {
    use serde_json::json;
    let sites: Vec<_> = (0..4).map(lattice).collect();
    let ground = Some(GroundPlane::new(0.)?);
    let mut stats = Report::default();
    let mut whole = native::Control::new(&sites.concat(), ground, G)?;
    let mut control: Vec<_> = sites
        .iter()
        .map(|site| native::Control::new(site, ground, G))
        .collect::<Result<_>>()?;
    let mut pool = WorkerPool::spawn_with_profile(
        worker,
        7081,
        vec![sites.concat(), vec![], vec![]],
        SimulationStep::Ms10,
        ground,
        128,
        NativeSolverProfile::Extended64,
    )?;
    // Four 64-body warm graphs serialize and solve together on the busiest owner.
    // Include the full physical solve and reply encoding in its request budget.
    let work_budget = std::time::Duration::from_secs(5);
    pool.set_native_work_timeout(work_budget)?;
    pool.set_packed_observations(packed);
    pool.set_packed_handoffs(packed);
    for body in &whole.bodies {
        assert_eq!(
            pool.frame().body(body.id).ok_or("missing initial body")?.1,
            body
        );
    }
    sample(&pool)?;
    for tick in 1..=last_tick {
        let pending = (tick == 6).then_some(PendingImpulse {
            id: 17,
            impulse: [0.001, 0., 0.],
        });
        if let Some(p) = &pending {
            pool.queue_impulse(p.id, p.impulse)?;
            let old = pool.frame().clone();
            pool.transfer_component(1, 1)?;
            stats.events.push(json!({
                "operation": "whole warm transfer with pending impulse",
                "committed_tick": old.tick, "first_recorded_tick": tick,
                "source": 0, "destination": 1,
                "member_ids": pool.frame().workers[1].components[0].ids,
                "impulse": p,
            }));
            for body in &old.workers[0].snapshot.bodies {
                let (owner, current) = pool.frame().body(body.id).ok_or("lost body")?;
                assert_eq!(owner, u8::from(body.id <= 64));
                assert_eq!(body, current);
            }
        }
        if tick == 8 {
            let frame = pool.frame().clone();
            let old_pid = pool.worker_pid(0);
            pool.terminate_worker(0)?;
            pool.recover(0)?;
            assert_eq!(pool.frame(), &frame);
            assert_ne!(pool.worker_pid(0), old_pid);
            stats.events.push(json!({
                "operation": "worker replacement", "owner": 0,
                "committed_tick": frame.tick, "first_recorded_tick": tick,
                "prior_pid": old_pid, "replacement_pid": pool.worker_pid(0),
            }));
        }
        if tick == 10 {
            let frame = pool.frame().clone();
            let path = scratch.join("split.gzpool");
            pool.save_native_recovery(&path)?;
            drop(pool);
            pool = PoolNativeRecovery::open(path)?.resume(worker)?;
            assert_eq!(pool.frame(), &frame);
            pool.set_native_work_timeout(work_budget)?;
            pool.set_packed_observations(packed);
            pool.set_packed_handoffs(packed);
            stats.events.push(json!({
                "operation": "whole GZPOOL03 reload", "workers": frame.workers.len(),
                "committed_tick": frame.tick, "first_recorded_tick": tick,
            }));
        }
        let before = pool.traffic();
        let started = std::time::Instant::now();
        pool.advance_adaptive_in_field(G, &[])
            .map_err(|e| format!("resident capacity packed={packed} tick={tick} advance: {e}"))?;
        stats.peak_advance_us = stats.peak_advance_us.max(started.elapsed().as_micros());
        if tick <= 5 {
            stats.peak_worker_barrier_reply_bytes = stats
                .peak_worker_barrier_reply_bytes
                .max(pool.traffic()[0].reply_bytes - before[0].reply_bytes);
            let source = &pool.frame().workers[0];
            assert_eq!(source.snapshot.bodies.len(), 256);
            if tick >= 3 {
                assert_eq!(source.components.len(), 4);
                assert!(source.components.iter().all(|c| c.ids.len() == 64));
            }
            stats.peak_pairs = stats.peak_pairs.max(source.snapshot.impulses.len());
            stats.peak_floor = stats.peak_floor.max(source.snapshot.supports.len());
        }
        let mut expected_pairs = vec![];
        let mut expected_floors = vec![];
        for (i, native) in control.iter_mut().enumerate() {
            let impulses = if i == 0 { pending.as_slice() } else { &[] };
            let (pairs, floors) = native.advance(pool.last_substeps().unwrap(), &[], impulses)?;
            expected_pairs.extend(pairs);
            expected_floors.extend(floors);
            for body in &native.bodies {
                assert!((body.position[0] - i as f64 * 15.).abs() < 1.);
                assert_eq!(
                    pool.frame().body(body.id).ok_or("missing observed body")?.1,
                    body
                );
            }
        }
        let mut pairs: Vec<_> = pool
            .frame()
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter().cloned())
            .collect();
        pairs.sort_by_key(|p| (p.a, p.b));
        let mut floors: Vec<_> = pool
            .frame()
            .workers
            .iter()
            .flat_map(|w| w.snapshot.supports.iter().cloned())
            .collect();
        floors.sort_by_key(|p| p.body);
        assert_eq!(pairs, expected_pairs);
        assert_eq!(floors, expected_floors);
        let (whole_pairs, whole_floors) =
            whole.advance(pool.last_substeps().unwrap(), &[], pending.as_slice())?;
        for body in &whole.bodies {
            assert_eq!(
                pool.frame()
                    .body(body.id)
                    .ok_or("missing whole-world body")?
                    .1,
                body
            );
        }
        assert_eq!(pairs, whole_pairs);
        assert_eq!(floors, whole_floors);
        if tick == 5 {
            let path = scratch.join("full.gzpool");
            pool.save_native_recovery(&path)?;
            let bytes = fs::read(path)?;
            let archive: serde_json::Value = serde_json::from_slice(&bytes[40..])?;
            let images = archive["images"][0]["partition"]["solvers"]
                .as_array()
                .unwrap();
            let native_sizes: Vec<_> = images
                .iter()
                .map(|i| i["native"].as_array().unwrap().len())
                .collect();
            assert_eq!(native_sizes.len(), 4);
            assert!(
                native_sizes
                    .iter()
                    .all(|&len| len < gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT)
            );
            stats.full_archive_bytes = bytes.len();
            stats.native_bytes = native_sizes;
        }
        sample(&pool)?;
    }
    Ok(stats)
}
