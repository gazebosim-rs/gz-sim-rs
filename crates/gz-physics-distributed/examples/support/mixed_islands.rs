// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent native controls for a loaded pair, a landing cylinder, and free bodies.
#[path = "native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    adaptive_substeps_with_ground,
    coordinator::Error,
    physics::{
        Body, BodyWrench, ContactImpulse, GroundPlane, GroundSupport, PendingImpulse, Primitive,
        SimulationStep, fixture,
    },
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use std::{
    error::Error as StdError,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

pub type Result<T> = std::result::Result<T, Box<dyn StdError>>;
const STEP: SimulationStep = SimulationStep::Ms10;
pub const GRAVITY: [f64; 3] = [0., 0., -2.];
pub const END: u64 = 135;

pub struct Scene {
    pub bodies: Vec<Body>,
    pub loads: [BodyWrench; 2],
    ground: GroundPlane,
}

impl Scene {
    pub fn new(free: usize) -> Result<Self> {
        let base = fixture()[0].promoted();
        let sphere = Body {
            id: 1,
            primitive: Primitive::Sphere,
            size: [0.7; 3],
            position: [-1.25, 0., 1.65],
            linear: [1.2, 0., 0.],
            ..base.clone()
        };
        let capsule = Body {
            id: 2,
            primitive: Primitive::Capsule,
            size: [0.7, 0.7, 0.85],
            position: [1.15, 0., 1.65],
            linear: [-1.2, 0., 0.],
            ..base.clone()
        };
        let cylinder = Body {
            id: 3,
            primitive: Primitive::Cylinder,
            size: [0.8, 0.8, 0.65],
            position: [0., 10., 1.6],
            linear: [0., 0., -1.2],
            ..base.clone()
        };
        let mut bodies = vec![sphere, capsule, cylinder];
        for i in 0..free {
            let (primitive, size) = match i % 4 {
                0 => (Primitive::Box, [0.48, 0.48, 0.48]),
                1 => (Primitive::Sphere, [0.50; 3]),
                2 => (Primitive::Cylinder, [0.48, 0.48, 0.6]),
                _ => (Primitive::Capsule, [0.4, 0.4, 0.46]),
            };
            bodies.push(Body {
                id: i as u64 + 4,
                primitive,
                size,
                position: [1.6 * (i % 5) as f64 - 3.2, -6.5 - 2. * (i / 5) as f64, 2.6],
                linear: [0.75, 0., 0.45],
                ..base.clone()
            });
        }
        Ok(Self {
            bodies,
            ground: GroundPlane::new(0.)?,
            loads: [(1, 1.5), (2, -1.5)].map(|(id, x)| BodyWrench {
                id,
                force: [x, 0., 2.],
                torque: [0.; 3],
            }),
        })
    }
    pub fn spawn(&self, worker: &Path) -> Result<WorkerPool> {
        let mut seeds = vec![vec![]; 4];
        for (i, b) in self.bodies.iter().enumerate() {
            seeds[i % 4].push(b.clone());
        }
        Ok(WorkerPool::spawn(
            worker,
            2026111954,
            seeds,
            STEP,
            Some(self.ground),
            1024,
        )?)
    }
}

/// Off-worker controls use analytically independent sets, without consulting the coordinator's
/// plan. The free fleet always advances at its ordinary public step.
struct Controls {
    pair: native::Control,
    cylinder: native::Control,
    free: native::Control,
    ground: GroundPlane,
}

impl Controls {
    fn new(s: &Scene) -> Result<Self> {
        let mut c = [&s.bodies[..2], &s.bodies[2..3], &s.bodies[3..]]
            .into_iter()
            .map(|b| native::Control::new(b, Some(s.ground), GRAVITY));
        Ok(Self {
            pair: c.next().unwrap()?,
            cylinder: c.next().unwrap()?,
            free: c.next().unwrap()?,
            ground: s.ground,
        })
    }
    fn advance(
        &mut self,
        loads: &[BodyWrench],
        pending: &[PendingImpulse],
    ) -> Result<(Vec<u64>, State)> {
        let mut fine = Vec::new();
        let mut expected = State::default();
        for (i, c) in [&mut self.pair, &mut self.cylinder, &mut self.free]
            .into_iter()
            .enumerate()
        {
            let commands: Vec<_> = pending
                .iter()
                .filter(|p| c.bodies.iter().any(|b| b.id == p.id))
                .cloned()
                .collect();
            let w = if i == 0 { loads } else { &[] };
            let n = if i == 2 {
                1
            } else {
                adaptive_substeps_with_ground(
                    &c.bodies,
                    STEP,
                    Some(self.ground),
                    &commands,
                    GRAVITY,
                    w,
                )?
            };
            if n == 10 {
                fine.extend(c.bodies.iter().map(|b| b.id));
            }
            let (pairs, supports) = c.advance(n, w, &commands)?;
            expected.bodies.extend_from_slice(&c.bodies);
            expected.impulses.extend(pairs);
            expected.supports.extend(supports);
        }
        Ok((fine, expected))
    }
}

#[derive(Default, PartialEq, Serialize, Debug)]
struct State {
    bodies: Vec<Body>,
    impulses: Vec<ContactImpulse>,
    supports: Vec<GroundSupport>,
}
impl State {
    fn read(f: &PoolFrame) -> Self {
        let mut s = Self::default();
        for w in &f.workers {
            s.bodies.extend_from_slice(&w.snapshot.bodies);
            s.impulses.extend_from_slice(&w.snapshot.impulses);
            s.supports.extend_from_slice(&w.snapshot.supports);
        }
        s.bodies.sort_by_key(|b| b.id);
        s.impulses.sort_by_key(|p| (p.a, p.b));
        s.supports.sort_by_key(|p| p.body);
        s
    }
}

#[derive(Serialize)]
struct Sample<'a> {
    tick: u64,
    time_s: f64,
    fine_ids: &'a [u64],
    owners: Vec<u8>,
    #[serde(flatten)]
    state: &'a State,
}

fn record(out: &mut BufWriter<File>, frame: &PoolFrame, state: &State, fine: &[u64]) -> Result<()> {
    serde_json::to_writer(
        &mut *out,
        &Sample {
            tick: frame.tick,
            time_s: frame.tick as f64 * 0.01,
            fine_ids: fine,
            owners: state
                .bodies
                .iter()
                .map(|b| frame.body(b.id).unwrap().0)
                .collect(),
            state,
        },
    )?;
    writeln!(out)?;
    Ok(())
}

#[derive(Default, Serialize)]
pub struct Qualification {
    pub exact_public_frames: u64,
    pub exact_after_restart: u64,
    pub first_pair_impulse_tick: Option<u64>,
    pub first_ground_impulse_tick: Option<u64>,
    pub peak_pair_impulse_ns: f64,
    pub peak_ground_impulse_ns: f64,
    pub island_body_solves: u64,
    pub comparable_scene_body_solves: u64,
    pub mixed_ticks: u64,
    pub fine_and_coarse_same_worker_ticks: u64,
    pub migrated_at: Option<u64>,
    pub moved_pair_from: Option<u8>,
    pub moved_pair_to: Option<u8>,
    pub lost_ack_tick: Option<u64>,
    pub replaced_worker_pids: Option<[u32; 2]>,
    pub restarted_from_file_at: Option<u64>,
    pub native_checkpoint_bytes: u64,
    pub queued_impulses_applied_at: Option<u64>,
    pub queued_ground_delta_vx_m_s: f64,
    pub queued_free_delta_vx_m_s: f64,
}

fn lost_ack(pool: &mut WorkerPool, loads: &[BodyWrench]) -> Result<()> {
    let before = pool.frame().clone();
    let ids = pool.last_island_refined_bodies().map(<[_]>::to_vec);
    pool.drop_next_acknowledgement(3)?;
    if !matches!(
        pool.advance_islands_in_field(GRAVITY, loads),
        Err(Error::Transport(_))
    ) || pool.frame() != &before
        || pool.last_island_refined_bodies() != ids.as_deref()
    {
        return Err(
            "unacknowledged mixed step changed the public barrier or subdivision receipt".into(),
        );
    }
    Ok(())
}

pub fn qualify(worker: &Path, path: &Path, save_trace: bool) -> Result<Qualification> {
    let s = Scene::new(5)?;
    let mut controls = Controls::new(&s)?;
    let mut pool = s.spawn(worker)?;
    let mut trace = save_trace
        .then(|| File::create_new(path.join("distributed.jsonl")).map(BufWriter::new))
        .transpose()?;
    if let Some(out) = &mut trace {
        record(out, pool.frame(), &State::read(pool.frame()), &[])?;
    }
    let mut q = Qualification::default();
    let mut pending = Vec::new();
    let checkpoint = path.join("native-barrier.gzpool");
    for tick in 1..=END {
        let (expected_ids, expected) = controls.advance(&s.loads, &pending)?;
        let plan = pool.refined_island_bodies(GRAVITY, &s.loads)?;
        if plan != expected_ids {
            return Err(format!("island policy mismatch at {tick}: distributed {plan:?}, independent {expected_ids:?}").into());
        }
        if let Some(hit) = q.migrated_at {
            if tick == hit + 1 {
                lost_ack(&mut pool, &s.loads)?;
                q.lost_ack_tick = Some(tick);
            }
            if tick == hit + 2 {
                let before = pool.frame().clone();
                let last = pool.last_island_refined_bodies().map(<[_]>::to_vec);
                let pids: Vec<_> = (0..4).map(|w| pool.worker_pid(w).unwrap()).collect();
                lost_ack(&mut pool, &s.loads)?;
                drop(pool);
                let disk = PoolNativeRecovery::open(&checkpoint)?;
                if disk.frame() != &before {
                    return Err("durable checkpoint differs from prior public barrier".into());
                }
                pool = disk.resume(worker)?;
                if pool.frame() != &before
                    || pool.last_island_refined_bodies() != last.as_deref()
                    || (0..4).any(|w| pids.contains(&pool.worker_pid(w).unwrap()))
                {
                    return Err(
                        "fresh workers failed to restore mixed subdivision receipt/state".into(),
                    );
                }
                q.restarted_from_file_at = Some(before.tick);
            }
        }
        let vx = [3, 4].map(|id| pool.frame().body(id).unwrap().1.linear[0]);
        pool.advance_islands_in_field(GRAVITY, &s.loads)?;
        let measured = State::read(pool.frame());
        if pool.frame().tick != tick
            || measured != expected
            || pool.last_island_refined_bodies() != Some(plan.as_slice())
        {
            return Err(format!(
                "mixed native mismatch at {tick}: distributed {measured:?}, control {expected:?}"
            )
            .into());
        }
        q.exact_public_frames += 1;
        q.island_body_solves += s.bodies.len() as u64 + 9 * plan.len() as u64;
        q.comparable_scene_body_solves +=
            s.bodies.len() as u64 * if plan.is_empty() { 1 } else { 10 };
        if !plan.is_empty() && plan.len() < s.bodies.len() {
            q.mixed_ticks += 1;
            let shared = plan.iter().any(|&a| {
                measured.bodies.iter().any(|b| {
                    plan.binary_search(&b.id).is_err()
                        && pool.frame().body(b.id).unwrap().0 == pool.frame().body(a).unwrap().0
                })
            });
            q.fine_and_coarse_same_worker_ticks += u64::from(shared);
        }
        if q.restarted_from_file_at.is_some() {
            q.exact_after_restart += 1;
        }
        if !pending.is_empty() {
            let delta = [
                measured.bodies[2].linear[0] - vx[0],
                measured.bodies[3].linear[0] - vx[1],
            ];
            if delta != [0.125, -0.25] {
                return Err(format!("restored impulse velocity changes {delta:?}").into());
            }
            q.queued_impulses_applied_at = Some(tick);
            [q.queued_ground_delta_vx_m_s, q.queued_free_delta_vx_m_s] = delta;
            pending.clear();
        }
        let pair = measured
            .impulses
            .iter()
            .filter(|p| (p.a, p.b) == (1, 2))
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        let ground = measured
            .supports
            .iter()
            .filter(|p| p.body == 3)
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        if pair > 0. {
            q.first_pair_impulse_tick.get_or_insert(tick);
        }
        if ground > 0. {
            q.first_ground_impulse_tick.get_or_insert(tick);
        }
        q.peak_pair_impulse_ns = q.peak_pair_impulse_ns.max(pair);
        q.peak_ground_impulse_ns = q.peak_ground_impulse_ns.max(ground);
        if q.migrated_at.is_none() && pair > 0. && ground > 0. {
            q.moved_pair_from = Some(pool.frame().body(1).unwrap().0);
            assert_ne!(q.moved_pair_from, Some(3));
            pool.transfer_component(1, 3)?;
            assert_eq!(State::read(pool.frame()), measured);
            assert_eq!(pool.frame().body(2).unwrap().0, 3);
            assert_eq!(pool.last_island_refined_bodies(), Some(plan.as_slice()));
            q.moved_pair_to = Some(3);
            q.migrated_at = Some(tick);
        } else if q.lost_ack_tick == Some(tick) {
            let old = pool.worker_pid(3).unwrap();
            pool.terminate_worker(3)?;
            pool.recover(3)?;
            let new = pool.worker_pid(3).unwrap();
            assert_ne!(old, new);
            q.replaced_worker_pids = Some([old, new]);
            assert_eq!(State::read(pool.frame()), measured);
            for (id, j) in [(3, 0.125), (4, -0.25)] {
                let impulse = [j, 0., 0.];
                pool.queue_impulse_in_field(id, impulse, GRAVITY, &s.loads)?;
                pending.push(PendingImpulse { id, impulse });
            }
            pool.save_native_recovery(&checkpoint)?;
            q.native_checkpoint_bytes = checkpoint.metadata()?.len();
        }
        if let Some(out) = &mut trace {
            record(out, pool.frame(), &measured, &plan)?;
        }
    }
    if let Some(out) = &mut trace {
        out.flush()?;
    }
    if q.exact_after_restart < 25
        || q.fine_and_coarse_same_worker_ticks < 25
        || q.replaced_worker_pids.is_none()
        || q.queued_impulses_applied_at.is_none()
    {
        return Err(format!("incomplete mixed run: {}", serde_json::to_string(&q)?).into());
    }
    Ok(q)
}
