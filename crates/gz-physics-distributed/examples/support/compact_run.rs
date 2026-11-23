// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent real-process worlds with a bounded suffix and a full unpruned prefix.
#[path = "compact_display.rs"]
mod display;
#[path = "replay.rs"]
mod replay;
use display::{JournalState, Sample};
use gz_physics_distributed::{
    coordinator::{Coordinator, Error as BarrierError, Frame},
    physics::fixture,
    transport::WorkerProcess,
};
use serde::Serialize;
use std::{error::Error, fs, path::Path, time::Instant};

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
const LIMIT: usize = 12;
const END_TICK: u64 = 200;
const IMPULSE: [f64; 3] = [1.2, 0., 0.];
type World = Coordinator<WorkerProcess>;

#[derive(Serialize)]
pub struct Baseline {
    tick: u64,
    sequences: [u64; 2],
    released: [usize; 2],
    pub image_bytes: usize,
}
#[derive(Serialize)]
pub struct Replacement {
    pub tick: u64,
    worker: u8,
    pub suffix_entries: usize,
    pub full_entries: usize,
    suffix_restart_and_replay_us: u128,
    full_restart_and_replay_us: u128,
}
#[derive(Serialize)]
pub struct Report {
    pub ticks: u64,
    journal_limit_per_worker: usize,
    pub capacity_refusals: usize,
    pub contact_tick: u64,
    pub uninterrupted_pids: [u32; 2],
    pub original_pids: [[u32; 2]; 2],
    pub final_pids: [[u32; 2]; 2],
    pub post_handoff_generations: [u64; 2],
    pub initial_contact_impulse_ns: f64,
    pub queued_impulse_residual_ns: f64,
    pub baselines: Vec<Baseline>,
    pub replacements: Vec<Replacement>,
    pub replay_samples: usize,
    display_trace: Vec<JournalState>,
}

#[derive(Clone, Copy)]
enum Action {
    Step(bool),
    ColdPromote,
    Queue,
    WarmTransfer,
}
impl Action {
    fn apply(self, c: &mut World) -> std::result::Result<(), BarrierError> {
        match self {
            Self::Step(false) => {
                c.advance()?;
            }
            Self::Step(true) => {
                c.advance_overlapped()?;
            }
            Self::ColdPromote => {
                c.transfer(1, true)?;
            }
            Self::Queue => {
                c.queue_impulse(1, IMPULSE)?;
            }
            Self::WarmTransfer => {
                c.transfer_group(1)?;
            }
        }
        Ok(())
    }
}

fn spawn(worker: &Path, run: u64, limit: usize) -> Result<(World, [u32; 2])> {
    let bodies = fixture();
    let peers = [
        WorkerProcess::spawn(worker, run, 0, &bodies[..1])?,
        WorkerProcess::spawn(worker, run, 1, &bodies[1..])?,
    ];
    let pids = peers.each_ref().map(|w| w.pid());
    Ok((Coordinator::new(run, peers, limit)?, pids))
}

struct Trial {
    bounded: World,
    full: World,
    uninterrupted: World,
    capacity: usize,
    baselines: Vec<Baseline>,
    replacements: Vec<Replacement>,
    samples: Vec<Sample>,
}
impl Trial {
    fn compact(&mut self) -> Result<()> {
        let before = self.bounded.frame().clone();
        let retained = self.bounded.retained_entries();
        self.sample();
        let stats = self.bounded.compact()?;
        assert_eq!(self.bounded.frame(), &before);
        assert_eq!(stats.released_entries, retained);
        assert_eq!(self.bounded.retained_entries(), [0; 2]);
        assert!(stats.bootstrap_bytes > 0);
        self.baselines.push(Baseline {
            tick: stats.tick,
            sequences: stats.sequences,
            released: stats.released_entries,
            image_bytes: stats.bootstrap_bytes,
        });
        self.sample();
        Ok(())
    }
    fn apply(&mut self, action: Action) -> Result<()> {
        match action.apply(&mut self.bounded) {
            Err(BarrierError::Capacity) => {
                // A full journal refuses before changing either native worker.
                assert_eq!(self.bounded.frame(), self.full.frame());
                self.capacity += 1;
                self.compact()?;
                action.apply(&mut self.bounded)?;
            }
            result => result?,
        }
        // Compare both real dispatch modes in each direction over consecutive ticks.
        let control = match action {
            Action::Step(overlap) => Action::Step(!overlap),
            other => other,
        };
        control.apply(&mut self.full)?;
        let serial = match action {
            Action::Step(_) => Action::Step(false),
            other => other,
        };
        serial.apply(&mut self.uninterrupted)?;
        assert_eq!(self.bounded.frame(), self.uninterrupted.frame());
        assert_eq!(self.bounded.frame(), self.full.frame());
        assert!(
            self.bounded
                .retained_entries()
                .iter()
                .all(|&len| len <= LIMIT)
        );
        Ok(())
    }
    fn recover_both(&mut self) -> Result<()> {
        let published = self.bounded.frame().clone();
        let suffix = self.bounded.retained_entries();
        let full = self.full.retained_entries();
        for worker in [0u8, 1] {
            let start = Instant::now();
            self.bounded.recover(worker)?;
            let suffix_us = start.elapsed().as_micros();
            let start = Instant::now();
            self.full.recover(worker)?;
            let full_us = start.elapsed().as_micros();
            assert_eq!(self.bounded.frame(), &published);
            assert_eq!(self.full.frame(), &published);
            let w = usize::from(worker);
            self.replacements.push(Replacement {
                worker,
                tick: published.tick,
                suffix_entries: suffix[w],
                full_entries: full[w],
                suffix_restart_and_replay_us: suffix_us,
                full_restart_and_replay_us: full_us,
            });
        }
        Ok(())
    }
    fn sample(&mut self) {
        let sample = Sample {
            frame: self.bounded.frame().clone(),
            journals: JournalState {
                tick: self.bounded.frame().tick,
                retained: self.bounded.retained_entries(),
                full: self.full.retained_entries(),
                baselines: self.baselines.len(),
                recoveries: self.replacements.len(),
            },
        };
        if self.samples.last() != Some(&sample) {
            self.samples.push(sample);
        }
    }
}

fn momentum(frame: &Frame) -> f64 {
    frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .map(|b| b.mass * b.linear[0])
        .sum()
}

pub fn run(worker: &Path, dir: &Path) -> Result<Report> {
    fs::create_dir_all(dir)?;
    if dir.join("verification.json").exists() || dir.join("replay").exists() {
        return Err("output already exists; choose a fresh qualification directory".into());
    }
    let run = u64::from(std::process::id());
    let (bounded, pa) = spawn(worker, run, LIMIT)?;
    let (full, pb) = spawn(worker, run, 1024)?;
    let (uninterrupted, uninterrupted_pids) = spawn(worker, run, 1024)?;
    let initial_p = momentum(bounded.frame());
    let mut trial = Trial {
        bounded,
        full,
        uninterrupted,
        capacity: 0,
        baselines: vec![],
        replacements: vec![],
        samples: vec![],
    };
    let mut contact = None;
    let mut impulse_residual = 0.;
    trial.sample();
    for tick in 1..=END_TICK {
        // Move the still separated prescribed body and promote it on worker 1.
        if tick == 41 {
            trial.apply(Action::ColdPromote)?;
            trial.sample();
        }
        trial.apply(Action::Step(tick % 2 == 0))?;
        if tick % 4 == 0 {
            trial.sample();
        }
        let total_contact = trial.bounded.frame().workers[1]
            .snapshot
            .impulses
            .iter()
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        if contact.is_none() && total_contact > 1e-3 {
            contact = Some((tick, total_contact));
            trial.sample();
            trial.apply(Action::Queue)?;
            trial.apply(Action::WarmTransfer)?;
            assert_eq!(trial.bounded.frame().workers[0].generations[&1], 2);
            assert_eq!(trial.bounded.frame().workers[0].generations[&2], 1);
            trial.sample();
            // The image now contains a loaded contact manifold, the pending
            // impulse, and the retired generations on the empty previous owner.
            trial.compact()?;
            trial.recover_both()?;
            trial.sample();
            // The previous owner accepts both IDs with their next generations.
            // This also proves the pending input could be exported by the replacement.
            trial.apply(Action::WarmTransfer)?;
            assert_eq!(trial.bounded.frame().workers[1].generations[&1], 3);
            assert_eq!(trial.bounded.frame().workers[1].generations[&2], 2);
            trial.sample();
            trial.compact()?;
        } else if let Some((collision, _)) = contact {
            if tick == collision + 1 {
                impulse_residual = (momentum(trial.bounded.frame()) - initial_p - IMPULSE[0]).abs();
                assert!(impulse_residual < 1e-12);
            }
            if tick == collision + 5 {
                assert_eq!(trial.bounded.retained_entries(), [5, 5]);
                trial.recover_both()?;
                trial.sample();
            }
            assert!((momentum(trial.bounded.frame()) - initial_p - IMPULSE[0]).abs() < 1e-10);
        }
    }
    let (contact_tick, initial_contact_impulse_ns) =
        contact.ok_or("fixture never developed an active contact")?;
    assert!(trial.capacity > 10);
    let generations = &trial.bounded.frame().workers[1].generations;
    let post_handoff_generations = [generations[&1], generations[&2]];
    let uninterrupted_peers = trial.uninterrupted.into_peers();
    assert_eq!(
        uninterrupted_pids,
        uninterrupted_peers.each_ref().map(|p| p.pid())
    );
    for mut peer in uninterrupted_peers {
        peer.shutdown()?;
    }
    let mut final_pids = [[0; 2]; 2];
    for (set, peers) in [trial.bounded.into_peers(), trial.full.into_peers()]
        .into_iter()
        .enumerate()
    {
        for (i, mut peer) in peers.into_iter().enumerate() {
            final_pids[set][i] = peer.pid();
            assert_ne!(peer.pid(), [pa, pb][set][i]);
            peer.shutdown()?;
        }
    }
    replay::write(&dir.join("replay"), &trial.samples)?;
    let report = Report {
        ticks: END_TICK,
        journal_limit_per_worker: LIMIT,
        capacity_refusals: trial.capacity,
        contact_tick,
        original_pids: [pa, pb],
        uninterrupted_pids,
        final_pids,
        post_handoff_generations,
        initial_contact_impulse_ns,
        queued_impulse_residual_ns: impulse_residual,
        baselines: trial.baselines,
        replacements: trial.replacements,
        replay_samples: trial.samples.len(),
        display_trace: trial.samples.into_iter().map(|s| s.journals).collect(),
    };
    serde_json::to_writer_pretty(
        fs::File::create_new(dir.join("verification.json"))?,
        &report,
    )?;
    println!(
        "{} published ticks matched exactly; {} capacity refusals with {} verified native baselines (combined image {}–{} B); contact at tick {} generated {:.6} N s; next-step impulse residual {:.3e} N s.",
        report.ticks,
        report.capacity_refusals,
        report.baselines.len(),
        report
            .baselines
            .iter()
            .map(|b| b.image_bytes)
            .min()
            .unwrap(),
        report
            .baselines
            .iter()
            .map(|b| b.image_bytes)
            .max()
            .unwrap(),
        contact_tick,
        initial_contact_impulse_ns,
        impulse_residual
    );
    for r in &report.replacements {
        println!(
            "tick {} worker {}: suffix {} entries in {} µs; full {} in {} µs (fence, bootstrap and native replay)",
            r.tick,
            r.worker,
            r.suffix_entries,
            r.suffix_restart_and_replay_us,
            r.full_entries,
            r.full_restart_and_replay_us
        );
    }
    Ok(report)
}
