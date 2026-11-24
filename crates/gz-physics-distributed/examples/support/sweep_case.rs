// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{replay, report::Case};
use gz_physics_distributed::{
    coordinator::{Coordinator, Frame, Peer},
    physics::{Body, Island, Partition, Snapshot, Tier, active_contact_group},
    protocol::{Outcome, Reply, Request},
    transport::{TransportError, WorkerProcess},
};
use std::{
    error::Error,
    io,
    path::Path,
    time::{SystemTime, UNIX_EPOCH},
};

pub fn unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock")
        .as_secs()
}

// SplitMix64, seeded per case. Reproducing a case never depends on earlier draws.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut n = self.0;
        n = (n ^ (n >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        n = (n ^ (n >> 27)).wrapping_mul(0x94D049BB133111EB);
        n ^ (n >> 31)
    }
    fn between(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * ((self.next() >> 11) as f64 / ((1_u64 << 53) as f64))
    }
}

struct Fixture {
    initial: Vec<Body>,
    size: usize,
    steps: u64,
    squeeze: f64,
}
impl Fixture {
    fn new(seed: u64) -> Self {
        let mut rng = Rng(seed);
        let size = 2 + (rng.next() % 7) as usize;
        // The ends and middle start with relative velocities chosen to bring all
        // adjacent faces together after the same nominal approach interval.
        let close_speed = rng.between(0.18, 1.45);
        let approach = rng.between(0.10, 0.80);
        let drift = rng.between(-0.65, 0.65);
        let squeeze = rng.between(0.04, 0.25);
        let mut initial = Vec::with_capacity(size + 2);
        let mut x = 0.;
        let mut previous = 0.;
        for i in 0..size {
            let dimensions = [
                rng.between(0.35, 0.75),
                rng.between(0.4, 0.75),
                rng.between(0.4, 0.75),
            ];
            if i > 0 {
                x += 0.5 * (previous + dimensions[0]) + approach * close_speed;
            }
            let body = Body {
                primitive: Default::default(),
                friction: 0.,
                id: i as u64 + 1,
                tier: Tier::Dynamic,
                size: dimensions,
                mass: rng.between(0.55, 3.0),
                position: [x, 0., 0.],
                orientation: [0., 0., 0., 1.],
                linear: [
                    drift + ((size - 1) as f64 / 2. - i as f64) * close_speed,
                    0.,
                    0.,
                ],
                angular: [0.; 3],
            };
            previous = dimensions[0];
            initial.push(body);
        }
        for body in &mut initial {
            body.position[0] -= x / 2.;
        }
        for (offset, z, v) in [
            (1, replay::PHYSICAL_GAP, 0.65),
            (2, -replay::PHYSICAL_GAP, -0.8),
        ] {
            initial.push(Body {
                id: size as u64 + offset,
                position: [-v, 0., z],
                linear: [v, 0., 0.],
                size: [0.6; 3],
                mass: 1.,
                ..initial[0].clone()
            });
        }
        // Periodically keep running well past the final handoff. The maximum
        // of 1024 recorded barriers bounds both recovery histories.
        let steps = if seed.is_multiple_of(16) {
            1024
        } else {
            160 + rng.next() % 161
        };
        Self {
            initial,
            size,
            steps,
            squeeze,
        }
    }
}

#[derive(Default)]
struct Witness {
    bytes: usize,
    positive: Option<usize>,
    pending: Option<usize>,
    impulse: f64,
    min_impulse: Option<f64>,
}
struct Live {
    process: WorkerProcess,
    witness: Witness,
}
impl Peer for Live {
    fn exchange(&mut self, request: &Request) -> Result<Reply, TransportError> {
        let reply = self.process.request(request)?;
        if let Outcome::WarmExported(export) = &reply.outcome {
            let c = &export.checkpoint;
            self.witness.bytes = self.witness.bytes.max(c.native_bytes());
            let positive = c.warmstart_normal_impulses().filter(|j| *j > 0.).count();
            self.witness.positive = Some(
                self.witness
                    .positive
                    .map_or(positive, |old| old.min(positive)),
            );
            let pending = c.pending.len();
            self.witness.pending =
                Some(self.witness.pending.map_or(pending, |old| old.min(pending)));
            let load = c
                .snapshot
                .impulses
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
            self.witness.impulse = self.witness.impulse.max(load);
            self.witness.min_impulse =
                Some(self.witness.min_impulse.map_or(load, |old| old.min(load)));
        }
        Ok(reply)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.process.restart()
    }
}

fn push_frame(frames: &mut Vec<Frame>, frame: &Frame) {
    if frames.last() != Some(frame) {
        frames.push(frame.clone());
    }
}

pub fn run(
    case: &mut Case,
    worker: &Path,
    capture: Option<&Path>,
    deadline: Option<u64>,
) -> Result<(), Box<dyn Error>> {
    let Fixture {
        initial,
        size,
        steps,
        squeeze,
    } = Fixture::new(case.seed);
    case.bodies = size;
    case.target_ticks = steps;
    let ids: Vec<_> = (1..=size as u64).collect();
    let run = case.seed;
    let mut controls = Vec::new();
    let mut peers = Vec::new();
    for (i, bodies) in [&initial[..size + 1], &initial[size + 1..]]
        .into_iter()
        .enumerate()
    {
        controls.push(Partition::restore(&Snapshot {
            bodies: bodies.to_vec(),
            ..Default::default()
        })?);
        peers.push(Live {
            process: WorkerProcess::spawn(worker, run, i as u8, bodies)?,
            witness: Witness::default(),
        });
    }
    case.original_pid = Some(peers[1].process.pid());
    let [a, b] = peers.try_into().map_err(|_| "expected two peers")?;
    let mut group = Coordinator::new(run, [a, b], steps as usize + 380)?;
    let mut mono = Island::restore(&Snapshot {
        bodies: initial,
        ..Default::default()
    })?;
    let mut frames = Vec::new();
    if capture.is_some() {
        push_frame(&mut frames, group.frame());
    }
    for tick in 1..=steps {
        if deadline.is_some_and(|end| unix() >= end) {
            case.stage = "deadline";
            return Ok(());
        }
        case.stage = "advance";
        let state = group.advance()?;
        case.ticks = tick;
        mono.step()?;
        for c in &mut controls {
            c.step()?;
        }
        case.stage = "comparison";
        case.monolithic.compare(state, &[mono.snapshot()])?;
        let baseline = [controls[0].snapshot(), controls[1].snapshot()];
        case.same_partition.compare(state, &baseline)?;
        let owner = (case.migrations.len() % 2) as u8;
        for id in 1..=size as u64 + 2 {
            let expected = if id <= size as u64 {
                owner
            } else {
                u8::from(id == size as u64 + 2)
            };
            let generation = if id <= size as u64 {
                case.migrations.len() as u64
            } else {
                0
            };
            if state.body(id).map(|v| v.0) != Some(expected)
                || state.workers[usize::from(expected)].generations.get(&id) != Some(&generation)
            {
                return Err("unexpected published ownership or generation".into());
            }
        }
        if capture.is_some() && (tick % steps.div_ceil(45) == 0 || tick == steps) {
            push_frame(&mut frames, state);
        }
        if case.migrations.len() == 2 {
            continue;
        }
        let working = &state.workers[usize::from(owner)].snapshot;
        let carries_load = active_contact_group(working, &ids)
            && working.impulses.iter().any(|p| p.normal_impulse > 1e-6);
        let first = case.migrations.first().copied();
        let switch = carries_load && first.is_none_or(|t| tick >= t + 4);
        // Continue compressing while bringing the group to its second transfer.
        // A disconnected chain is never passed off as a successful transfer.
        if !switch && !first.is_some_and(|t| tick <= t + 40) {
            continue;
        }
        let before = capture.is_some().then(|| state.clone());
        case.stage = "input";
        for (id, value) in [
            (1, squeeze),
            (size as u64, -squeeze),
            (size as u64 + 1, 0.019),
            (size as u64 + 2, -0.023),
        ] {
            let impulse = [value, 0., 0.];
            group.queue_impulse(id, impulse)?;
            controls[usize::from(id == size as u64 + 2)].queue_impulse(id, impulse)?;
            mono.queue_impulse(id, impulse)?;
            case.queued_inputs += 1;
        }
        if first.is_some_and(|t| tick > t) && case.replaced_at.is_none() {
            case.stage = "replacement";
            group.recover(1)?;
            case.replaced_at = Some(tick);
        }
        if switch {
            case.stage = "migration";
            let changed = group.transfer_group(ids[size / 2])?;
            if let Some(before) = before {
                push_frame(&mut frames, &before);
                push_frame(&mut frames, changed);
            }
            case.monolithic.compare(changed, &[mono.snapshot()])?;
            case.same_partition.compare(changed, &baseline)?;
            case.migrations.push(tick);
        }
    }
    case.stage = "shutdown";
    for (i, mut peer) in group.into_peers().into_iter().enumerate() {
        let witness = peer.witness;
        case.peak_native_bytes = case.peak_native_bytes.max(witness.bytes);
        if let Some(j) = witness.min_impulse {
            case.min_export_impulse_n_s =
                Some(case.min_export_impulse_n_s.map_or(j, |old| old.min(j)));
        }
        case.max_export_impulse_n_s = case.max_export_impulse_n_s.max(witness.impulse);
        if let Some(n) = witness.positive {
            case.min_cached_positive_points =
                Some(case.min_cached_positive_points.map_or(n, |old| old.min(n)));
        }
        if let Some(n) = witness.pending {
            case.min_exported_pending = Some(case.min_exported_pending.map_or(n, |old| old.min(n)));
        }
        if i == 1 && case.replaced_at.is_some() {
            let pid = peer.process.pid();
            if Some(pid) == case.original_pid {
                return Err("replacement retained original PID".into());
            }
            case.replacement_pid = Some(pid);
        }
        peer.process.shutdown()?;
    }
    if let Some(dir) = capture {
        case.stage = "capture";
        replay::write(dir, &frames)?;
    }
    case.stage = "complete";
    Ok(())
}
