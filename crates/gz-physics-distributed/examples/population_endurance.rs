// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Repeatedly land, migrate and retire newly issued cargo against an unmigrated native worker.
use clap::Parser;
use gz_physics_distributed::{
    physics::{Body, GroundPlane, Primitive, SimulationStep, Snapshot, StepField, Tier},
    pool::{PoolNativeRecovery, WorkerPool},
    protocol::{Command, Observation, Outcome, Request, Worker},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const G: [f64; 3] = [0., 0., -9.81];
const RUN: u64 = 0x6269727468;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    #[arg(long,default_value_t=384,value_parser=clap::value_parser!(u16).range(1..))]
    cycles: u16,
    output: PathBuf,
}
#[derive(Serialize)]
struct Summary {
    width: u8,
    cycles: u16,
    ticks: u64,
    highest_issued: u64,
    permanent_retirements: u16,
    loaded_handoffs: u16,
    lost_retirement_acks: u16,
    lost_step_acks: u16,
    worker_kills: u16,
    coordinator_restores: u16,
    loaded_pair_impulse_range_ns: [f64; 2],
    floor_support_range_ns: [f64; 2],
    verified_body_frames: u64,
    verified_pair_frames: u64,
    verified_support_frames: u64,
    physics_sha256: String,
}

fn sphere(id: u64, x: f64, z: f64) -> Body {
    Body {
        id,
        tier: Tier::Dynamic,
        primitive: Primitive::Sphere,
        size: [0.6; 3],
        mass: 1.,
        friction: 0.,
        position: [x, 0., z],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    }
}

struct Run {
    pool: WorkerPool,
    reference: Worker,
    sequence: u64,
    hash: Sha256,
    verified: [u64; 3],
}
impl Run {
    fn request(&mut self, command: Command) -> Result<Observation> {
        self.sequence += 1;
        let Outcome::State(o) = self
            .reference
            .handle(Request {
                run_id: RUN,
                worker_id: 0,
                sequence: self.sequence,
                command,
            })?
            .outcome
        else {
            return Err("native worker did not publish a state".into());
        };
        Ok(o)
    }
    fn compare(&mut self, reference: &Observation, advancing: bool) -> Result<()> {
        let frame = self.pool.frame();
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
        let mut floor: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .collect();
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floor.sort_by_key(|s| s.body);
        let n = [bodies.len(), pairs.len(), floor.len()];
        let bytes = bincode::serialize(&(frame.tick, bodies, pairs, floor))?;
        let s = &reference.snapshot;
        if bytes != bincode::serialize(&(s.tick, &s.bodies, &s.impulses, &s.supports))? {
            return Err(format!(
                "diverged from unmigrated native worker at tick {}",
                frame.tick
            )
            .into());
        }
        if advancing {
            self.hash.update(bytes);
            for (total, count) in self.verified.iter_mut().zip(n) {
                *total += count as u64;
            }
        }
        Ok(())
    }
    fn advance(&mut self, lost: Option<u8>) -> Result<()> {
        let tick = self.pool.frame().tick;
        if let Some(w) = lost {
            let before = self.pool.frame().clone();
            self.pool.drop_next_acknowledgement(w)?;
            if self.pool.advance_islands_in_field(G, &[]).is_ok() || self.pool.frame() != &before {
                return Err("unacknowledged step was published".into());
            }
        }
        self.pool.advance_islands_in_field(G, &[])?;
        let refined = self
            .pool
            .last_island_refined_bodies()
            .ok_or("missing refined membership")?
            .to_vec();
        let native = self.request(Command::StepIslands {
            tick,
            field: StepField {
                gravity: G,
                wrenches: vec![],
            },
            refined,
        })?;
        self.compare(&native, true)
    }
    fn input(&mut self, id: u64, impulse: [f64; 3]) -> Result<()> {
        self.pool.queue_impulse(id, impulse)?;
        let reference = self.request(Command::QueueImpulse {
            tick: self.pool.frame().tick,
            id,
            impulse,
        })?;
        self.compare(&reference, false)
    }
    fn resume(mut self, worker: &Path, directory: &Path, cycle: u16, packed: bool) -> Result<Self> {
        let file = directory.join(format!("cycle-{cycle}.native"));
        self.pool.save_native_recovery(&file)?;
        let plan = PoolNativeRecovery::open(&file)?;
        if plan.frame() != self.pool.frame() {
            return Err("saved frame changed".into());
        }
        drop(self.pool);
        let mut pool = plan.resume(worker)?;
        pool.set_packed_handoffs(packed);
        fs::remove_file(file)?;
        Ok(Self { pool, ..self })
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let ground = Some(GroundPlane::new(0.)?);
    let a = sphere(1, 0., 0.305);
    let bystander = sphere(2, 20., 0.305);
    let mut seeds = vec![vec![]; usize::from(args.width)];
    seeds[0].push(a.clone());
    seeds[1].push(bystander.clone());
    let mut pool = WorkerPool::spawn(&args.worker, RUN, seeds, SimulationStep::Ms10, ground, 48)?;
    if pool.set_packed_handoffs(true) != usize::from(args.width) {
        return Err("current worker required".into());
    }
    let reference = Worker::new(
        RUN,
        0,
        Snapshot {
            ground,
            step: SimulationStep::Ms10,
            bodies: vec![a, bystander],
            ..Default::default()
        },
    )?;
    let mut run = Run {
        pool,
        reference,
        sequence: 0,
        hash: Sha256::new(),
        verified: [0; 3],
    };
    let directory = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let scratch = tempfile::tempdir_in(directory)?;
    let mut out = Summary {
        width: args.width,
        cycles: args.cycles,
        ticks: 0,
        highest_issued: 2,
        permanent_retirements: 0,
        loaded_handoffs: 0,
        lost_retirement_acks: 0,
        lost_step_acks: 0,
        worker_kills: 0,
        coordinator_restores: 0,
        loaded_pair_impulse_range_ns: [f64::INFINITY, 0.],
        floor_support_range_ns: [f64::INFINITY, 0.],
        verified_body_frames: 0,
        verified_pair_frames: 0,
        verified_support_frames: 0,
        physics_sha256: String::new(),
    };
    for _ in 0..12 {
        run.advance(None)?;
    }
    for cycle in 1..=args.cycles {
        let id = 2 + u64::from(cycle);
        let base = run.pool.frame().body(1).unwrap().1;
        let new = sphere(id, base.position[0], base.position[2] + 0.665);
        run.pool.insert_body_in_field(new.clone(), G, &[])?;
        let observation = run.request(Command::InsertBody {
            tick: run.pool.frame().tick,
            body: new,
        })?;
        run.compare(&observation, false)?;
        for _ in 0..28 {
            run.advance(None)?;
        }
        let (owner, _) = run.pool.frame().body(id).unwrap();
        let native = &run.pool.frame().workers[usize::from(owner)].snapshot;
        let load = native
            .impulses
            .iter()
            .find(|p| p.a == 1 && p.b == id && p.normal_impulse > 0.)
            .ok_or("new cargo did not load the base")?
            .normal_impulse;
        let support = native
            .supports
            .iter()
            .find(|p| p.body == 1 && p.normal_impulse > 0.)
            .ok_or("loaded base lost floor support")?
            .normal_impulse;
        for (range, value) in [
            (&mut out.loaded_pair_impulse_range_ns, load),
            (&mut out.floor_support_range_ns, support),
        ] {
            range[0] = range[0].min(value);
            range[1] = range[1].max(value);
        }
        run.pool.transfer_component(id, (owner + 1) % args.width)?;
        out.loaded_handoffs += 1;
        let native = run.request(Command::Observe)?;
        run.compare(&native, false)?;
        run.input(1, [0., -0., 0.001])?;
        run.input(id, [0., 0., -0.01])?;
        let dx = if cycle.is_multiple_of(2) {
            0.003
        } else {
            -0.003
        };
        run.input(2, [dx, 0., -0.])?;
        if cycle.is_multiple_of(23) {
            let w = (cycle % u16::from(args.width)) as u8;
            let old = run.pool.worker_pid(w).unwrap();
            run.pool.terminate_worker(w)?;
            run.pool.recover(w)?;
            if run.pool.worker_pid(w) == Some(old) {
                return Err("worker was not replaced".into());
            }
            out.worker_kills += 1;
        }
        if cycle.is_multiple_of(17) {
            let before = run.pool.frame().clone();
            run.pool
                .drop_next_acknowledgement((cycle % u16::from(args.width)) as u8)?;
            if run.pool.retire(id).is_ok() || run.pool.frame() != &before {
                return Err("partial deletion became authoritative".into());
            }
            out.lost_retirement_acks += 1;
        }
        run.pool.retire(id)?;
        let obs = run.request(Command::RetireBody {
            tick: run.pool.frame().tick,
            id,
            owner: 0,
            generation: 0,
        })?;
        run.compare(&obs, false)?;
        out.permanent_retirements += 1;
        out.highest_issued = id;
        if cycle.is_multiple_of(19) {
            run = run.resume(
                &args.worker,
                scratch.path(),
                cycle,
                out.coordinator_restores.is_multiple_of(2),
            )?;
            out.coordinator_restores += 1;
            if run.pool.insert_body(sphere(id, 40., 2.)).is_ok() {
                return Err("restored allocator reused a retired ID".into());
            }
        }
        let lost = if cycle.is_multiple_of(13) {
            out.lost_step_acks += 1;
            Some((cycle % u16::from(args.width)) as u8)
        } else {
            None
        };
        run.advance(lost)?;
    }
    out.ticks = run.pool.frame().tick;
    [
        out.verified_body_frames,
        out.verified_pair_frames,
        out.verified_support_frames,
    ] = run.verified;
    out.physics_sha256 = format!("{:x}", run.hash.finalize());
    fs::write(
        &args.output,
        format!("{}\n", serde_json::to_string_pretty(&out)?),
    )?;
    Ok(())
}
