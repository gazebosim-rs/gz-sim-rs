// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Account for external linear impulses on dynamic bodies and momentum crossing worker ownership.
use gz_physics_distributed::{
    physics::{BodyWrench, PendingImpulse, Tier},
    pool::PoolFrame,
};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const LIMIT_NS: f64 = 1e-9;
type Vector = [f64; 3];
fn add(sum: &mut Vector, v: Vector) {
    for (a, b) in sum.iter_mut().zip(v) {
        *a += b;
    }
}
fn scale(v: Vector, n: f64) -> Vector {
    v.map(|a| n * a)
}
fn peak(v: Vector) -> f64 {
    v.into_iter().map(f64::abs).fold(0., f64::max)
}

struct Member {
    owner: usize,
    mass: f64,
    momentum: Vector,
}
struct State {
    tick: u64,
    members: BTreeMap<u64, Member>,
    net: Vec<Vector>,
}
impl State {
    fn read(frame: &PoolFrame) -> Result<Self> {
        let mut members = BTreeMap::new();
        let mut net = vec![[0.; 3]; frame.workers.len()];
        for (w, obs) in frame.workers.iter().enumerate() {
            if obs.snapshot.ground.is_some_and(|g| g.friction() != 0.) {
                return Err("unmeasured tangential ground impulse".into());
            }
            for b in &obs.snapshot.bodies {
                if b.tier != Tier::Dynamic {
                    return Err("an external fixed or prescribed reaction is unmeasured".into());
                }
                let momentum = scale(b.linear, b.mass);
                if !momentum.into_iter().all(f64::is_finite) {
                    return Err("invalid body momentum".into());
                }
                add(&mut net[w], momentum);
                if members
                    .insert(
                        b.id,
                        Member {
                            owner: w,
                            mass: b.mass,
                            momentum,
                        },
                    )
                    .is_some()
                {
                    return Err("duplicate owner".into());
                }
            }
        }
        Ok(Self {
            tick: frame.tick,
            members,
            net,
        })
    }
}

#[derive(Default, Serialize)]
struct Terms {
    owner: usize,
    delta_ns: Vector,
    migration_ns: Vector,
    gravity_ns: Vector,
    applied_ns: Vector,
    queued_ns: Vector,
    floor_ns: Vector,
    residual_ns: Vector,
}
#[derive(Default, Serialize)]
pub struct Summary {
    pub solves: u64,
    pub same_tick_checks: u64,
    pub moved_bodies: u64,
    pub peak_worker_residual_ns: f64,
    pub peak_global_residual_ns: f64,
    pub peak_worker_flux_ns: f64,
    pub peak_floor_ns: f64,
}
pub struct Audit {
    prior: State,
    stats: Summary,
    trace: Option<BufWriter<File>>,
}
impl Audit {
    pub fn new(frame: &PoolFrame, trace: Option<&Path>) -> Result<Self> {
        Ok(Self {
            prior: State::read(frame)?,
            stats: Summary::default(),
            trace: trace.map(File::create_new).transpose()?.map(BufWriter::new),
        })
    }
    pub fn advance(
        &mut self,
        frame: &PoolFrame,
        gravity: Vector,
        loads: &[BodyWrench],
        queued: &[PendingImpulse],
    ) -> Result<()> {
        self.record(frame, gravity, loads, queued, true)
    }
    pub fn same_tick(&mut self, frame: &PoolFrame) -> Result<()> {
        self.record(frame, [0.; 3], &[], &[], false)
    }
    fn record(
        &mut self,
        frame: &PoolFrame,
        gravity: Vector,
        inputs: &[BodyWrench],
        queued: &[PendingImpulse],
        advancing: bool,
    ) -> Result<()> {
        let next = State::read(frame)?;
        if next.tick != self.prior.tick + u64::from(advancing)
            || next.net.len() != self.prior.net.len()
            || next.members.keys().ne(self.prior.members.keys())
        {
            return Err("momentum audit requires consecutive barriers and fixed membership".into());
        }
        let mut terms: Vec<_> = next
            .net
            .iter()
            .zip(&self.prior.net)
            .enumerate()
            .map(|(owner, (p, prev))| Terms {
                owner,
                delta_ns: std::array::from_fn(|i| p[i] - prev[i]),
                ..Default::default()
            })
            .collect();
        for (id, body) in &next.members {
            let prior = &self.prior.members[id];
            if prior.mass.to_bits() != body.mass.to_bits() {
                return Err("body mass changed".into());
            }
            if !advancing && body.momentum.map(f64::to_bits) != prior.momentum.map(f64::to_bits) {
                return Err("same-tick migration or recovery changed body momentum".into());
            }
            if prior.owner != body.owner {
                add(
                    &mut terms[prior.owner].migration_ns,
                    scale(prior.momentum, -1.),
                );
                add(&mut terms[body.owner].migration_ns, prior.momentum);
                self.stats.moved_bodies += 1;
            }
            if advancing {
                let dt = frame.workers[body.owner]
                    .snapshot
                    .step
                    .duration()
                    .as_secs_f64();
                add(
                    &mut terms[body.owner].gravity_ns,
                    scale(gravity, body.mass * dt),
                );
            }
        }
        if advancing {
            for input in inputs {
                let w = next
                    .members
                    .get(&input.id)
                    .ok_or("unknown COM force")?
                    .owner;
                let dt = frame.workers[w].snapshot.step.duration().as_secs_f64();
                add(&mut terms[w].applied_ns, scale(input.force, dt));
            }
            for input in queued {
                let w = next
                    .members
                    .get(&input.id)
                    .ok_or("unknown queued impulse")?
                    .owner;
                add(&mut terms[w].queued_ns, input.impulse);
            }
            for (w, obs) in frame.workers.iter().enumerate() {
                for support in &obs.snapshot.supports {
                    let normal = obs
                        .snapshot
                        .ground
                        .ok_or("ground impulse without plane")?
                        .normal();
                    add(
                        &mut terms[w].floor_ns,
                        scale(normal, support.normal_impulse),
                    );
                }
            }
            self.stats.solves += 1;
        } else {
            self.stats.same_tick_checks += 1;
        }
        let mut global = [0.; 3];
        for row in &mut terms {
            row.residual_ns = std::array::from_fn(|i| {
                row.delta_ns[i]
                    - row.migration_ns[i]
                    - row.gravity_ns[i]
                    - row.applied_ns[i]
                    - row.queued_ns[i]
                    - row.floor_ns[i]
            });
            if !row.residual_ns.into_iter().all(f64::is_finite) || peak(row.residual_ns) > LIMIT_NS
            {
                return Err(format!(
                    "momentum balance at tick {} owner {}: {:?}",
                    frame.tick, row.owner, row.residual_ns
                )
                .into());
            }
            self.stats.peak_worker_residual_ns = self
                .stats
                .peak_worker_residual_ns
                .max(peak(row.residual_ns));
            self.stats.peak_worker_flux_ns =
                self.stats.peak_worker_flux_ns.max(peak(row.migration_ns));
            self.stats.peak_floor_ns = self.stats.peak_floor_ns.max(peak(row.floor_ns));
            add(&mut global, row.residual_ns);
        }
        if peak(global) > LIMIT_NS {
            return Err(
                format!("system momentum balance at tick {}: {global:?}", next.tick).into(),
            );
        }
        self.stats.peak_global_residual_ns = self.stats.peak_global_residual_ns.max(peak(global));
        if let Some(trace) = &mut self.trace {
            writeln!(
                trace,
                "{}",
                serde_json::json!({"tick":frame.tick,"advancing":advancing,"workers":terms,"global_residual_ns":global})
            )?;
        }
        self.prior = next;
        Ok(())
    }
    pub fn finish(mut self) -> Result<Summary> {
        if let Some(f) = &mut self.trace {
            f.flush()?;
        }
        Ok(self.stats)
    }
}
