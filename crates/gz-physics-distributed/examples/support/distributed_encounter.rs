// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Four rotating dynamic boxes on four real workers, compared with an independent native world.
use crate::train_fixture::train_fixture;
#[path = "native_contact_control.rs"]
mod native_contact_control;
use gz_physics_distributed::{
    adaptive_substeps,
    physics::{Body, ContactImpulse, PendingImpulse, SimulationStep},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use native_contact_control::Control;
use rapier3d_f64::{
    parry::{query, shape::Cuboid},
    prelude::{Pose, Rotation, Vector},
};
use serde::Serialize;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
const STEP: SimulationStep = SimulationStep::Ms10;
const ZERO: [f64; 3] = [0.; 3];
const ONE_SHOT: [f64; 3] = [0., 0., 0.125];

fn momentum(bodies: &[Body]) -> Vector {
    bodies.iter().map(|b| Vector::from(b.linear) * b.mass).sum()
}
fn bodies(frame: &PoolFrame) -> Vec<Body> {
    (1..=4)
        .map(|id| frame.body(id).unwrap().1.clone())
        .collect()
}
fn owners(frame: &PoolFrame) -> [u8; 4] {
    std::array::from_fn(|i| frame.body(i as u64 + 1).unwrap().0)
}
fn contacts(frame: &PoolFrame) -> Vec<ContactImpulse> {
    let mut all: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|obs| obs.snapshot.impulses.iter().cloned())
        .collect();
    all.sort_by_key(|p| (p.a, p.b));
    all
}
fn gap(bodies: &[Body]) -> Result<f64> {
    let mut min = f64::INFINITY;
    for a in &bodies[..2] {
        for b in &bodies[2..] {
            let [pa, pb] = [a, b].map(|b| {
                Pose::from_parts(
                    Vector::from(b.position),
                    Rotation::from_array(b.orientation),
                )
            });
            let [ca, cb] = [a, b].map(|b| Cuboid::new(Vector::from(b.size) * 0.5));
            let mut d = query::distance(&pa, &ca, &pb, &cb)?.distance;
            if d == 0. {
                d = query::contact(&pa, &ca, &pb, &cb, 0.)?.map_or(0., |p| p.dist);
            }
            min = min.min(d);
        }
    }
    Ok(min)
}
fn cross_load(pairs: &[ContactImpulse]) -> f64 {
    pairs
        .iter()
        .filter(|p| p.a <= 2 && p.b >= 3)
        .fold(0., |sum, p| sum + p.normal_impulse)
}

#[derive(Serialize, Default)]
pub struct Run {
    pub seed: u64,
    pub displaced: bool,
    pub exact_public_frames: u64,
    pub refined_intervals: u64,
    pub first_cross_tick: Option<u64>,
    pub max_cross_impulse_ns: f64,
    pub minimum_public_gap_m: f64,
    pub coarse_minimum_public_gap_m: f64,
    pub refined_frames_with_load_before_final_microstep: u64,
    pub max_cross_load_before_final_microstep_ns: f64,
    pub coarse_max_cross_impulse_ns: f64,
    pub peak_left_momentum_exchange_ns: f64,
    pub max_total_momentum_error_ns: f64,
    pub auto_body_moves: u64,
    pub migrated_at: Option<u64>,
    pub migrated_workers: Option<[u8; 2]>,
    pub lost_refined_reply_at: Option<u64>,
    pub replaced_worker_at: Option<u64>,
    pub replaced_worker_pids: Option<[u32; 2]>,
    pub restarted_from_file_at: Option<u64>,
    pub one_shot_at: Option<u64>,
    pub exact_frames_after_restart: u64,
    pub checkpoint_bytes: u64,
}

#[derive(Serialize)]
struct Trace<'a> {
    tick: u64,
    time_s: f64,
    substeps: u8,
    owners: Option<[u8; 4]>,
    bodies: &'a [Body],
    gap_m: f64,
    impulses: &'a [ContactImpulse],
}

pub fn run(
    worker: &Path,
    seed: u64,
    displaced: bool,
    inject_faults: bool,
    directory: Option<&Path>,
) -> Result<Run> {
    let (mut initial, loads) = train_fixture(seed);
    if displaced {
        for b in &mut initial[2..] {
            b.position[1] += 12.;
        }
    }
    let mut control = Control::new(&initial, None, ZERO)?;
    let mut coarse = Control::new(&initial, None, ZERO)?;
    let shards = initial.iter().map(|b| vec![b.clone()]).collect();
    let mut pool = WorkerPool::spawn(worker, seed, shards, STEP, None, 1024)?;
    let tmp = tempfile::tempdir()?;
    let checkpoint = directory
        .unwrap_or(tmp.path())
        .join("native-barrier.gzpool");
    let mut traces = directory
        .map(|dir| -> Result<_> {
            Ok((
                BufWriter::new(File::create_new(dir.join("distributed.jsonl"))?),
                BufWriter::new(File::create_new(dir.join("coarse.jsonl"))?),
            ))
        })
        .transpose()?;
    let mut result = Run {
        seed,
        displaced,
        minimum_public_gap_m: f64::INFINITY,
        coarse_minimum_public_gap_m: f64::INFINITY,
        ..Default::default()
    };
    let mut expected_p = momentum(&initial);
    let mut old_left = momentum(&initial[..2]);
    let mut pending = Vec::new();
    for tick in 1..=335 {
        let before = owners(pool.frame());
        let selected = adaptive_substeps(&control.bodies, STEP, &pending, ZERO, &loads)?;
        let (expected, _) = control.advance(selected, &loads, &pending)?;
        let (coarse_pairs, _) = coarse.advance(1, &loads, &pending)?;
        let supplied = pending
            .iter()
            .map(|p| Vector::from(p.impulse))
            .sum::<Vector>();
        expected_p += supplied
            + loads
                .iter()
                .map(|w| Vector::from(w.force) * 0.01)
                .sum::<Vector>();
        pending.clear();

        if inject_faults
            && result.migrated_at.is_some()
            && result.lost_refined_reply_at.is_none()
            && selected == 10
        {
            let prior = pool.frame().clone();
            let worker = prior.body(1).unwrap().0;
            pool.drop_next_acknowledgement(worker)?;
            if !matches!(
                pool.advance_adaptive_in_field(ZERO, &loads),
                Err(gz_physics_distributed::coordinator::Error::Transport(_))
            ) || pool.frame() != &prior
            {
                return Err(
                    "missing refined acknowledgement exposed a partial public barrier".into(),
                );
            }
            result.lost_refined_reply_at = Some(tick);
        }
        pool.advance_adaptive_in_field(ZERO, &loads)?;
        if pool.last_substeps() != Some(selected) {
            return Err(format!("seed {seed}, tick {tick}: subdivision mismatch").into());
        }
        result.refined_intervals += u64::from(selected == 10);
        let f = pool.frame();
        let observed = bodies(f);
        let pairs = contacts(f);
        if observed != control.bodies {
            let id = observed
                .iter()
                .zip(&control.bodies)
                .find(|(a, b)| a != b)
                .unwrap()
                .0
                .id;
            return Err(format!("seed {seed}, tick {tick}: native state mismatch on body {id}, subdivision {selected}").into());
        }
        if pairs != expected {
            return Err(format!(
                "seed {seed}, tick {tick}: aggregate impulses differ {pairs:?} vs {expected:?}"
            )
            .into());
        }
        let now = owners(f);
        result.auto_body_moves += before
            .iter()
            .zip(now)
            .filter(|(old, new)| **old != *new)
            .count() as u64;
        result.exact_public_frames += 1;
        if result.restarted_from_file_at.is_some() {
            result.exact_frames_after_restart += 1;
        }
        let left = momentum(&observed[..2]);
        // The queued one-shot belongs to body 1. Only the outer left box has external force.
        let exchange = (left - old_left - Vector::from(loads[0].force) * 0.01 - supplied).length();
        old_left = left;
        result.peak_left_momentum_exchange_ns = result.peak_left_momentum_exchange_ns.max(exchange);
        result.max_total_momentum_error_ns = result
            .max_total_momentum_error_ns
            .max((momentum(&observed) - expected_p).length());
        let distributed_gap = gap(&observed)?;
        let coarse_gap = gap(&coarse.bodies)?;
        result.minimum_public_gap_m = result.minimum_public_gap_m.min(distributed_gap);
        result.coarse_minimum_public_gap_m = result.coarse_minimum_public_gap_m.min(coarse_gap);
        let cross = cross_load(&pairs);
        if selected == 10 {
            let last: f64 = control
                .solver
                .rigid_contact_observations()
                .iter()
                .filter(|p| p.active && p.a.raw() < 2 && p.b.raw() >= 2)
                .fold(0., |sum, p| sum + p.normal_impulse);
            let earlier = cross - last;
            if earlier > 1e-8 {
                result.refined_frames_with_load_before_final_microstep += 1;
            }
            result.max_cross_load_before_final_microstep_ns =
                result.max_cross_load_before_final_microstep_ns.max(earlier);
        }
        result.max_cross_impulse_ns = result.max_cross_impulse_ns.max(cross);
        result.coarse_max_cross_impulse_ns = result
            .coarse_max_cross_impulse_ns
            .max(cross_load(&coarse_pairs));
        if cross > 1e-6 {
            result.first_cross_tick.get_or_insert(tick);
        }
        if let Some((dist, base)) = &mut traces {
            for (out, bs, own, n, js, signed_gap) in [
                (
                    dist,
                    &observed,
                    Some(now),
                    selected,
                    &pairs,
                    distributed_gap,
                ),
                (base, &coarse.bodies, None, 1, &coarse_pairs, coarse_gap),
            ] {
                serde_json::to_writer(
                    &mut *out,
                    &Trace {
                        tick,
                        time_s: tick as f64 * 0.01,
                        substeps: n,
                        owners: own,
                        bodies: bs,
                        gap_m: signed_gap,
                        impulses: js,
                    },
                )?;
                writeln!(out)?;
            }
        }

        if inject_faults && let Some(hit) = result.first_cross_tick {
            if tick == hit {
                let original = f.clone();
                let source = now[0];
                let dest = (source + 1) % 4;
                pool.transfer_component(1, dest)?;
                if owners(pool.frame()) == now
                    || bodies(pool.frame()) != observed
                    || contacts(pool.frame()) != contacts(&original)
                {
                    return Err("warm transfer changed the public physical barrier".into());
                }
                result.migrated_at = Some(tick);
                result.migrated_workers = Some([source, dest]);
            } else if tick == hit + 1 {
                let owner = now[0];
                let pid = pool.worker_pid(owner).unwrap();
                pool.terminate_worker(owner)?;
                pool.recover(owner)?;
                let next = pool.worker_pid(owner).unwrap();
                if pid == next {
                    return Err("worker did not restart".into());
                }
                result.replaced_worker_at = Some(tick);
                result.replaced_worker_pids = Some([pid, next]);
            } else if tick == hit + 2 {
                pool.queue_impulse_in_field(1, ONE_SHOT, ZERO, &loads)?;
                pending.push(PendingImpulse {
                    id: 1,
                    impulse: ONE_SHOT,
                });
                let saved = pool.frame().clone();
                let substeps = pool.last_substeps();
                let old_endpoints: Vec<_> = (0..4)
                    .map(|w| pool.worker_endpoint(w).unwrap().to_owned())
                    .collect();
                pool.save_native_recovery(&checkpoint)?;
                result.checkpoint_bytes = std::fs::metadata(&checkpoint)?.len();
                drop(pool);
                let archive = PoolNativeRecovery::open(&checkpoint)?;
                if archive.frame() != &saved {
                    return Err("checkpoint observation differs from committed state".into());
                }
                pool = archive.resume(worker)?;
                if pool.frame() != &saved
                    || pool.last_substeps() != substeps
                    || (0..4).any(|w| {
                        pool.worker_endpoint(w) == Some(old_endpoints[w as usize].as_str())
                    })
                {
                    return Err("coordinator restore lost public state, interval choice or endpoint fencing".into());
                }
                result.restarted_from_file_at = Some(tick);
                result.one_shot_at = Some(tick + 1);
            }
        }
    }
    if let Some((mut d, mut c)) = traces {
        d.flush()?;
        c.flush()?;
    }
    if result.max_total_momentum_error_ns > 1e-10
        || (displaced
            && (result.max_cross_impulse_ns != 0. || result.peak_left_momentum_exchange_ns > 1e-10))
        || result.auto_body_moves == 0
        || result.refined_intervals == 0
        || (inject_faults
            && (result.max_cross_impulse_ns <= 0.01
                || result.peak_left_momentum_exchange_ns <= 0.01
                || result.exact_frames_after_restart == 0
                || result.lost_refined_reply_at.is_none()))
    {
        return Err(format!(
            "failed physics or recovery witness: {}",
            serde_json::to_string(&result)?
        )
        .into());
    }
    Ok(result)
}
