// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fault qualification with a loaded sphere–capsule pair and a cylinder on the floor.
#[path = "native_contact_control.rs"]
mod native_contact_control;
use gz_physics_distributed::{
    adaptive_substeps_with_ground,
    coordinator::Error,
    physics::{
        Body, BodyWrench, ContactImpulse, GroundPlane, GroundSupport, PendingImpulse, Primitive,
        SimulationStep, fixture,
    },
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use native_contact_control::Control;
use serde::Serialize;
use std::{
    error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
pub type Result<T> = std::result::Result<T, Box<dyn error::Error>>;
const STEP: SimulationStep = SimulationStep::Ms10;
const GRAVITY: [f64; 3] = [0., 0., -9.81];
const ONE_SHOT: [f64; 3] = [0.125, 0., 0.];

fn scene() -> (Vec<Body>, [BodyWrench; 2]) {
    let b = fixture()[0].promoted();
    let sphere = Body {
        id: 1,
        primitive: Primitive::Sphere,
        size: [0.7; 3],
        position: [-1.9, 0., 1.65],
        linear: [3., 0., 0.],
        ..b.clone()
    };
    let capsule = Body {
        id: 2,
        primitive: Primitive::Capsule,
        size: [0.7, 0.7, 0.85],
        position: [1.2, 0., 1.65],
        linear: [-1., 0., 0.],
        ..b.clone()
    };
    let cylinder = Body {
        id: 3,
        primitive: Primitive::Cylinder,
        size: [0.8, 0.8, 0.65],
        position: [2.6, 0., 2.5],
        linear: [0., 0., -1.],
        ..b
    };
    // The pair is suspended by equal upward forces. Opposed horizontal loads
    // maintain a measurable contact through the separate cylinder's touchdown.
    (
        vec![sphere, capsule, cylinder],
        [(1, 4.), (2, -4.)].map(|(id, f)| BodyWrench {
            id,
            force: [f, 0., 9.81],
            torque: [0.; 3],
        }),
    )
}

#[derive(PartialEq, Serialize)]
struct Physical {
    bodies: Vec<Body>,
    impulses: Vec<ContactImpulse>,
    supports: Vec<GroundSupport>,
}

impl Physical {
    fn read(f: &PoolFrame) -> Self {
        let mut impulses = Vec::new();
        let mut supports = Vec::new();
        for w in &f.workers {
            impulses.extend_from_slice(&w.snapshot.impulses);
            supports.extend_from_slice(&w.snapshot.supports);
        }
        impulses.sort_unstable_by_key(|p| (p.a, p.b));
        supports.sort_unstable_by_key(|p| p.body);
        Self {
            bodies: (1..=3).map(|id| f.body(id).unwrap().1.clone()).collect(),
            impulses,
            supports,
        }
    }

    fn native(
        c: &mut Control,
        selected: u8,
        loads: &[BodyWrench],
        pending: &[PendingImpulse],
    ) -> Result<Self> {
        let (impulses, supports) = c.advance(selected, loads, pending)?;
        Ok(Self {
            bodies: c.bodies.clone(),
            impulses,
            supports,
        })
    }

    fn pair_load(&self) -> f64 {
        self.impulses
            .iter()
            .filter(|p| (p.a, p.b) == (1, 2))
            .map(|p| p.normal_impulse)
            .sum()
    }

    fn cylinder_support(&self) -> f64 {
        self.supports
            .iter()
            .filter(|p| p.body == 3)
            .map(|p| p.normal_impulse)
            .sum()
    }

    fn cylinder_penetration(&self) -> f64 {
        let b = &self.bodies[2];
        let [x, y, z, w] = b.orientation;
        let axial = 1. - 2. * (x * x + y * y);
        let radial = (2. * (x * z + y * w)).hypot(2. * (y * z - x * w));
        (0.5 * (b.size[2] * axial.abs() + b.size[0] * radial) - b.position[2]).max(0.)
    }
}

fn owners(f: &PoolFrame) -> [u8; 3] {
    std::array::from_fn(|i| f.body(i as u64 + 1).unwrap().0)
}

fn drop_refined_reply(pool: &mut WorkerPool, owner: u8, loads: &[BodyWrench]) -> Result<()> {
    let before = pool.frame().clone();
    pool.drop_next_acknowledgement(owner)?;
    if pool.adaptive_substeps(GRAVITY, loads)? != 10
        || !matches!(
            pool.advance_adaptive_in_field(GRAVITY, loads),
            Err(Error::Transport(_))
        )
        || pool.frame() != &before
    {
        return Err("lost refined acknowledgement changed the public frame".into());
    }
    Ok(())
}

#[derive(Default, Serialize)]
pub struct Run {
    pub exact_public_frames: u64,
    pub refined_intervals: u64,
    pub first_pair_load_at: Option<u64>,
    pub first_cylinder_support_at: Option<u64>,
    pub peak_pair_load_ns: f64,
    pub peak_cylinder_support_ns: f64,
    pub peak_cylinder_penetration_m: f64,
    pub coarse_peak_cylinder_penetration_m: f64,
    pub migrated_at: Option<u64>,
    pub owners_before_migration: Option<[u8; 3]>,
    pub owners_after_migration: Option<[u8; 3]>,
    pub migrated_pair_load_ns: f64,
    pub migrated_cylinder_support_ns: f64,
    pub lost_refined_reply_at: Option<u64>,
    pub replaced_worker_at: Option<u64>,
    pub replaced_worker_pids: Option<[u32; 2]>,
    pub restarted_from_file_at: Option<u64>,
    pub unacknowledged_step_before_disk_restore: Option<u64>,
    pub one_shot_applied_at: Option<u64>,
    pub one_shot_delta_vx_m_s: Option<f64>,
    pub checkpoint_bytes: u64,
    pub exact_frames_after_restart: u64,
    pub dual_loaded_frames_after_restart: u64,
}

#[derive(Serialize)]
struct Trace<'a> {
    tick: u64,
    time_s: f64,
    substeps: u8,
    owners: Option<[u8; 3]>,
    #[serde(flatten)]
    state: &'a Physical,
}

fn write_frame(
    out: &mut BufWriter<File>,
    tick: u64,
    substeps: u8,
    owners: Option<[u8; 3]>,
    state: &Physical,
) -> Result<()> {
    serde_json::to_writer(
        &mut *out,
        &Trace {
            tick,
            time_s: tick as f64 * 0.01,
            substeps,
            owners,
            state,
        },
    )?;
    writeln!(out)?;
    Ok(())
}

pub fn run(worker: &Path, directory: &Path, trace: bool) -> Result<Run> {
    let (initial, loads) = scene();
    let ground = Some(GroundPlane::new(0.)?);
    let mut reference = Control::new(&initial, ground, GRAVITY)?;
    let mut coarse = Control::new(&initial, ground, GRAVITY)?;
    let mut seeds: Vec<_> = initial.iter().map(|b| vec![b.clone()]).collect();
    seeds.push(Vec::new());
    let mut pool = WorkerPool::spawn(worker, 2026111901, seeds, STEP, ground, 1024)?;
    let archive_path = directory.join("native-barrier.gzpool");
    let mut output = trace
        .then(|| -> Result<_> {
            Ok((
                BufWriter::new(File::create_new(directory.join("distributed.jsonl"))?),
                BufWriter::new(File::create_new(directory.join("coarse.jsonl"))?),
            ))
        })
        .transpose()?;
    if let Some((dist, base)) = &mut output {
        let frame = pool.frame();
        let first = Physical::read(frame);
        if frame.tick != 0
            || first.bodies != coarse.bodies
            || !first.impulses.is_empty()
            || !first.supports.is_empty()
        {
            return Err("worker and control initial frames differ".into());
        }
        write_frame(dist, frame.tick, 0, Some(owners(frame)), &first)?;
        write_frame(base, frame.tick, 0, None, &first)?;
    }
    let mut r = Run::default();
    let mut pending = Vec::new();
    for tick in 1..=150 {
        let cylinder_vx = pool.frame().body(3).unwrap().1.linear[0];
        let selected = adaptive_substeps_with_ground(
            &reference.bodies,
            STEP,
            ground,
            &pending,
            GRAVITY,
            &loads,
        )?;
        let expected = Physical::native(&mut reference, selected, &loads, &pending)?;
        let base = Physical::native(&mut coarse, 1, &loads, &pending)?;
        pending.clear();
        if let Some(hit) = r.migrated_at {
            if tick == hit + 1 {
                drop_refined_reply(&mut pool, 3, &loads)?;
                r.lost_refined_reply_at = Some(tick);
            } else if tick == hit + 2 {
                let before = pool.frame().clone();
                let old_pids: Vec<_> = (0..4).map(|i| pool.worker_pid(i).unwrap()).collect();
                let n = pool.last_substeps();
                drop_refined_reply(&mut pool, 3, &loads)?;
                drop(pool);
                let disk = PoolNativeRecovery::open(&archive_path)?;
                if disk.frame() != &before {
                    return Err("checkpoint readback differs from saved barrier".into());
                }
                pool = disk.resume(worker)?;
                if pool.frame() != &before
                    || pool.last_substeps() != n
                    || (0..4).any(|i| old_pids.contains(&pool.worker_pid(i).unwrap()))
                {
                    return Err("fresh workers did not restore the exact checkpoint".into());
                }
                r.restarted_from_file_at = Some(before.tick);
                r.unacknowledged_step_before_disk_restore = Some(tick);
                r.one_shot_applied_at = Some(tick);
            }
        }
        pool.advance_adaptive_in_field(GRAVITY, &loads)?;
        let observed = Physical::read(pool.frame());
        if pool.frame().tick != tick
            || pool.last_substeps() != Some(selected)
            || observed != expected
        {
            return Err(format!("native mismatch at tick {tick} (worker substeps {:?}, expected {selected}): observed {}, expected {}",pool.last_substeps(),serde_json::to_string(&observed)?,serde_json::to_string(&expected)?).into());
        }
        if r.one_shot_applied_at == Some(tick) {
            let delta = observed.bodies[2].linear[0] - cylinder_vx;
            if (delta - ONE_SHOT[0] / observed.bodies[2].mass).abs() > 1e-12 {
                return Err(format!(
                    "frictionless cylinder received incorrect one-shot velocity change {delta}"
                )
                .into());
            }
            r.one_shot_delta_vx_m_s = Some(delta);
        }
        r.exact_public_frames += 1;
        r.refined_intervals += u64::from(selected == 10);
        let (pair, support) = (observed.pair_load(), observed.cylinder_support());
        r.peak_pair_load_ns = r.peak_pair_load_ns.max(pair);
        r.peak_cylinder_support_ns = r.peak_cylinder_support_ns.max(support);
        r.peak_cylinder_penetration_m = r
            .peak_cylinder_penetration_m
            .max(observed.cylinder_penetration());
        r.coarse_peak_cylinder_penetration_m = r
            .coarse_peak_cylinder_penetration_m
            .max(base.cylinder_penetration());
        if pair > 0. {
            r.first_pair_load_at.get_or_insert(tick);
        }
        if support > 0. {
            r.first_cylinder_support_at.get_or_insert(tick);
        }
        if r.restarted_from_file_at.is_some() {
            r.exact_frames_after_restart += 1;
            r.dual_loaded_frames_after_restart += u64::from(pair > 0. && support > 0.);
        }
        if r.migrated_at.is_none() && pair > 0. && support > 0. {
            let before = owners(pool.frame());
            pool.transfer_component(1, 3)?;
            // A conservative route may have already joined the cylinder to the pair.
            if pool.frame().body(3).unwrap().0 != 3 {
                pool.transfer_component(3, 3)?;
            }
            let after = owners(pool.frame());
            if before.contains(&3) || after != [3; 3] || Physical::read(pool.frame()) != observed {
                return Err("loaded migration lost a body, contact or support".into());
            }
            r.migrated_at = Some(tick);
            r.owners_before_migration = Some(before);
            r.owners_after_migration = Some(after);
            r.migrated_pair_load_ns = pair;
            r.migrated_cylinder_support_ns = support;
        } else if r.lost_refined_reply_at == Some(tick) {
            let old = pool.worker_pid(3).unwrap();
            pool.terminate_worker(3)?;
            pool.recover(3)?;
            let new = pool.worker_pid(3).unwrap();
            if new == old || Physical::read(pool.frame()) != observed {
                return Err("worker replacement lost the committed barrier".into());
            }
            r.replaced_worker_at = Some(tick);
            r.replaced_worker_pids = Some([old, new]);
            pool.queue_impulse_in_field(3, ONE_SHOT, GRAVITY, &loads)?;
            pending.push(PendingImpulse {
                id: 3,
                impulse: ONE_SHOT,
            });
            pool.save_native_recovery(&archive_path)?;
            r.checkpoint_bytes = archive_path.metadata()?.len();
        }
        if let Some((dist, coarse)) = &mut output {
            for (out, state, n, hosts) in [
                (dist, &observed, selected, Some(owners(pool.frame()))),
                (coarse, &base, 1, None),
            ] {
                write_frame(out, tick, n, hosts, state)?;
            }
        }
    }
    if let Some((mut dist, mut coarse)) = output {
        dist.flush()?;
        coarse.flush()?;
    }
    if !(1..150).contains(&r.refined_intervals)
        || r.dual_loaded_frames_after_restart < 40
        || r.replaced_worker_pids.is_none()
        || r.checkpoint_bytes == 0
    {
        return Err(format!(
            "incomplete curved/ground qualification: {}",
            serde_json::to_string(&r)?
        )
        .into());
    }
    Ok(r)
}
