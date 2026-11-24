// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A dense, non-colliding lattice departs from and returns to a continuously loaded pair.
#[path = "native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, BodyWrench, ContactImpulse, GroundSupport, Primitive, SimulationStep, Tier},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use std::{
    error::Error as StdError,
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
};

pub type Result<T> = std::result::Result<T, Box<dyn StdError>>;
pub const END: u64 = 230;
pub const GRAVITY: [f64; 3] = [-12., -12., 0.];
const MIGRATE: u64 = 116;

pub struct Scene {
    pub bodies: Vec<Body>,
    pub loads: [BodyWrench; 2],
}

impl Scene {
    pub fn new() -> Self {
        let a = Body {
            id: 1,
            primitive: Primitive::Sphere,
            size: [0.2; 3],
            mass: 1.,
            tier: Tier::Dynamic,
            friction: 0.,
            position: [-0.125, 0., 3.],
            orientation: [0., 0., 0., 1.],
            linear: [4., 0., 0.],
            angular: [0.; 3],
        };
        let b = Body {
            id: 2,
            position: [0.125, 0., 3.],
            linear: [-4., 0., 0.],
            ..a.clone()
        };
        let mut bodies = vec![a.clone(), b];
        for i in 0..24 {
            bodies.push(Body {
                id: i + 3,
                position: [
                    1. + 0.32 * (i % 4) as f64,
                    1. + 0.32 * (i / 4 % 3) as f64,
                    3. + 0.32 * (i / 12) as f64,
                ],
                linear: [14., 14., 0.],
                ..a.clone()
            });
        }
        // Counter the common field for the pair, then press the two spheres together.
        let loads = [(1, 5.), (2, -5.)].map(|(id, press)| BodyWrench {
            id,
            force: [12. + press, 12., 0.],
            torque: [0.; 3],
        });
        Self { bodies, loads }
    }
    pub fn spawn(&self, worker: &Path) -> Result<WorkerPool> {
        let mut seeds = vec![vec![]; 4];
        for (i, b) in self.bodies.iter().enumerate() {
            seeds[i % 4].push(b.clone());
        }
        Ok(WorkerPool::spawn(
            worker,
            2026111955,
            seeds,
            SimulationStep::Ms10,
            None,
            1024,
        )?)
    }
}

#[derive(Serialize)]
struct Sample<'a> {
    tick: u64,
    time_s: f64,
    fine_ids: &'a [u64],
    owners: Vec<u8>,
    components: Vec<Vec<u64>>,
    bodies: Vec<&'a Body>,
    impulses: Vec<&'a ContactImpulse>,
    supports: Vec<&'a GroundSupport>,
}
impl<'a> Sample<'a> {
    fn read(frame: &'a PoolFrame, fine_ids: &'a [u64]) -> Self {
        let mut bodies = Vec::new();
        let mut components = Vec::new();
        let mut impulses = Vec::new();
        let mut supports = Vec::new();
        for obs in &frame.workers {
            bodies.extend(&obs.snapshot.bodies);
            components.extend(obs.components.iter().map(|c| c.ids.clone()));
            impulses.extend(&obs.snapshot.impulses);
            supports.extend(&obs.snapshot.supports);
        }
        bodies.sort_by_key(|b: &&Body| b.id);
        components.sort_unstable_by_key(|ids| ids[0]);
        impulses.sort_by_key(|p: &&ContactImpulse| (p.a, p.b));
        supports.sort_by_key(|p: &&GroundSupport| p.body);
        Self {
            tick: frame.tick,
            time_s: frame.tick as f64 * 0.01,
            fine_ids,
            owners: bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect(),
            components,
            bodies,
            impulses,
            supports,
        }
    }
    fn record(&self, out: &mut Option<BufWriter<File>>) -> Result<()> {
        if let Some(out) = out {
            serde_json::to_writer(&mut *out, self)?;
            writeln!(out)?;
        }
        Ok(())
    }
    fn same_physics(&self, other: &Self) -> Result<bool> {
        let bytes = |s: &Sample<'_>| bincode::serialize(&(&s.bodies, &s.impulses, &s.supports));
        Ok(bytes(self)? == bytes(other)?)
    }
}

/// Free spheres are geometrically disjoint for the whole flight. Solve each one
/// independently at its published rate. The contacting pair always gets a
/// separate uninterrupted 1 ms reference, independent of all worker decisions.
struct Reference {
    pair: native::Control,
    fleet: Vec<native::Control>,
}
impl Reference {
    fn new(s: &Scene) -> Result<Self> {
        Ok(Self {
            pair: native::Control::new(&s.bodies[..2], None, GRAVITY)?,
            fleet: s.bodies[2..]
                .iter()
                .map(|b| native::Control::new(std::slice::from_ref(b), None, GRAVITY))
                .collect::<Result<_>>()?,
        })
    }
    fn verify(&mut self, s: &Scene, sample: &Sample<'_>) -> Result<f64> {
        let (impulses, supports) = self.pair.advance(10, &s.loads, &[])?;
        if impulses.len() != 1
            || impulses[0].normal_impulse <= 0.
            || (impulses[0].a, impulses[0].b) != (1, 2)
            || bincode::serialize(&impulses)? != bincode::serialize(&sample.impulses)?
            || bincode::serialize(&supports)? != bincode::serialize(&sample.supports)?
        {
            return Err(format!(
                "contact receipt mismatch at tick {}: native {impulses:?}, worker {:?}",
                sample.tick, sample.impulses
            )
            .into());
        }
        for c in &mut self.fleet {
            let n = if sample.fine_ids.contains(&c.bodies[0].id) {
                10
            } else {
                1
            };
            c.advance(n, &[], &[])?;
        }
        let expected = self
            .pair
            .bodies
            .iter()
            .chain(self.fleet.iter().map(|c| &c.bodies[0]));
        if sample.bodies.len() != s.bodies.len() {
            return Err("incomplete barrier".into());
        }
        for (a, b) in sample.bodies.iter().zip(expected) {
            if bincode::serialize(a)? != bincode::serialize(b)? {
                return Err(format!(
                    "body mismatch at tick {}: worker {a:?}, native {b:?}",
                    sample.tick
                )
                .into());
            }
        }
        Ok(impulses[0].normal_impulse)
    }
}

#[derive(Default, Serialize)]
pub struct Qualification {
    pub exact_public_frames: u64,
    pub native_body_comparisons: u64,
    pub first_pair_only_component_tick: Option<u64>,
    pub first_fleet_all_coarse_tick: Option<u64>,
    pub first_rejoined_component_tick: Option<u64>,
    pub peak_component_count: usize,
    pub planned_body_solves: u64,
    pub integrated_pair_impulse_ns: f64,
    pub migration_tick: u64,
    pub migration_owners: [u8; 2],
    pub lost_ack_retried_tick: u64,
    pub worker_replacement_barrier: u64,
    pub worker_replacement_pids: [u32; 2],
    pub archive_restore_barrier: u64,
    pub resumed_ticks: u64,
}

pub fn qualify(worker: &Path, dir: &Path, save_trace: bool) -> Result<Qualification> {
    let scene = Scene::new();
    let mut reference = Reference::new(&scene)?;
    let mut pool = scene.spawn(worker)?;
    let mut out = save_trace
        .then(|| File::create_new(dir.join("distributed.jsonl")).map(BufWriter::new))
        .transpose()?;
    Sample::read(pool.frame(), &[]).record(&mut out)?;
    let mut r = Qualification::default();
    let archive = dir.join("native-barrier.gzpool");
    for tick in 1..=END {
        let was = pool.frame().tick;
        if tick == MIGRATE + 1 {
            let frame = pool.frame().clone();
            let fine = pool.last_island_refined_bodies().map(<[_]>::to_vec);
            let dest = r.migration_owners[1];
            pool.drop_next_acknowledgement(dest)?;
            if !matches!(
                pool.advance_islands_in_field(GRAVITY, &scene.loads),
                Err(Error::Transport(_))
            ) || pool.frame() != &frame
                || pool.last_island_refined_bodies() != fine.as_deref()
            {
                return Err("lost acknowledgement changed the committed barrier".into());
            }
            r.lost_ack_retried_tick = tick;
        }
        if tick == MIGRATE + 2 {
            let frame = pool.frame().clone();
            let fine = pool.last_island_refined_bodies().map(<[_]>::to_vec);
            let dest = r.migration_owners[1];
            let old_pid = pool.worker_pid(dest).unwrap();
            pool.terminate_worker(dest)?;
            pool.recover(dest)?;
            let new_pid = pool.worker_pid(dest).unwrap();
            if old_pid == new_pid
                || pool.frame() != &frame
                || pool.last_island_refined_bodies() != fine.as_deref()
            {
                return Err("worker replacement failed to restore the barrier".into());
            }
            r.worker_replacement_pids = [old_pid, new_pid];
            r.worker_replacement_barrier = was;
        }
        if tick == MIGRATE + 3 {
            let frame = pool.frame().clone();
            let fine = pool.last_island_refined_bodies().map(<[_]>::to_vec);
            pool.save_native_recovery(&archive)?;
            let pids: Vec<_> = (0..4)
                .map(|owner| pool.worker_pid(owner).unwrap())
                .collect();
            drop(pool);
            let disk = PoolNativeRecovery::open(&archive)?;
            if disk.frame() != &frame {
                return Err("native archive lost committed state".into());
            }
            pool = disk.resume(worker)?;
            if pool.frame() != &frame
                || pool.last_island_refined_bodies() != fine.as_deref()
                || (0..4).any(|owner| pids.contains(&pool.worker_pid(owner).unwrap()))
            {
                return Err("fresh coordinator/worker recovery failed".into());
            }
            r.archive_restore_barrier = was;
        }
        pool.advance_islands_in_field(GRAVITY, &scene.loads)?;
        let fine = pool
            .last_island_refined_bodies()
            .ok_or("missing subdivision receipt")?
            .to_vec();
        let frame = pool.frame().clone();
        let sample = Sample::read(&frame, &fine);
        if sample.tick != tick || !fine.starts_with(&[1, 2]) {
            return Err(format!("unrefined or missing load at tick {tick}").into());
        }
        r.integrated_pair_impulse_ns += reference.verify(&scene, &sample)?;
        r.exact_public_frames += 1;
        r.native_body_comparisons += sample.bodies.len() as u64;
        r.planned_body_solves += (sample.bodies.len() + 9 * fine.len()) as u64;
        r.peak_component_count = r.peak_component_count.max(sample.components.len());
        let pair_group = &sample.components[0];
        if tick == 1 && pair_group.len() != scene.bodies.len() {
            return Err(format!("dense fixture did not start joined: {pair_group:?}").into());
        }
        if pair_group.len() == 2 {
            r.first_pair_only_component_tick.get_or_insert(tick);
        } else if r.first_pair_only_component_tick.is_some() {
            r.first_rejoined_component_tick.get_or_insert(tick);
        }
        if fine.len() == 2 {
            r.first_fleet_all_coarse_tick.get_or_insert(tick);
        }
        if tick == MIGRATE {
            if pair_group != &[1, 2] {
                return Err(
                    format!("pair not isolated at planned transfer: {pair_group:?}").into(),
                );
            }
            let from = sample.owners[0];
            let to = (from + 1) % 4;
            pool.transfer_component(1, to)?;
            let moved = Sample::read(pool.frame(), &fine);
            if moved.owners[..2] != [to, to]
                || moved.owners[2..] != sample.owners[2..]
                || moved.components != sample.components
                || !moved.same_physics(&sample)?
                || pool.last_island_refined_bodies() != Some(&fine)
            {
                return Err("migration changed contact, state or membership".into());
            }
            r.migration_tick = tick;
            r.migration_owners = [from, to];
            moved.record(&mut out)?;
        } else {
            sample.record(&mut out)?;
        }
        if r.archive_restore_barrier != 0 {
            r.resumed_ticks += 1;
        }
    }
    if r.first_fleet_all_coarse_tick.is_none() || r.first_rejoined_component_tick.is_none() {
        return Err(format!("missing separation/return: {}", serde_json::to_string(&r)?).into());
    }
    if let Some(out) = &mut out {
        out.flush()?;
    }
    Ok(r)
}

/// Restore another run's native checkpoint and compare every remaining published
/// barrier, including component membership, owner, resolution and contact load.
pub fn verify_previous_checkpoint(worker: &Path, directory: &Path) -> Result<(u64, u64)> {
    let archive = PoolNativeRecovery::open(directory.join("native-barrier.gzpool"))?;
    let first = archive.frame().tick;
    if first >= END {
        return Err("checkpoint must precede the end of the encounter".into());
    }
    let scene = Scene::new();
    let mut pool = archive.resume(worker)?;
    let mut trace = BufReader::new(File::open(directory.join("distributed.jsonl"))?).lines();
    for tick in 0..=END {
        let expected = trace.next().ok_or("incomplete reference trace")??;
        if tick < first {
            continue;
        }
        if tick > first {
            pool.advance_islands_in_field(GRAVITY, &scene.loads)?;
        }
        let fine = pool
            .last_island_refined_bodies()
            .ok_or("missing restored schedule")?;
        let sample = Sample::read(pool.frame(), fine);
        if sample.tick != tick || serde_json::to_string(&sample)? != expected {
            return Err(format!("restored run diverged at tick {tick}").into());
        }
    }
    if trace.next().is_some() {
        return Err("unexpected frames after the reference end".into());
    }
    Ok((first, END - first))
}
