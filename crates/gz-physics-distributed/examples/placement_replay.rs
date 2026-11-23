// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record committed automatic placements, replay the persisted tape with real faults,
//! and compare every public native observation and ownership fence.
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
#[path = "support/historical_placement.rs"]
mod historical;
use clap::Parser;
use control::Oracle;
use gz_physics_distributed::{
    coordinator::Error as PoolError,
    physics::{GroundPlane, SimulationStep},
    pool::{PlacementRecord, PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
type Owners = BTreeMap<u64, (u8, u64)>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    /// Read an existing tape and complete public receipts without starting a live sampler.
    #[arg(long)]
    from_recording: Option<PathBuf>,
    /// Keep every recorded worker address and add idle pool peers.
    #[arg(long, requires = "from_recording", value_parser = clap::value_parser!(u8).range(4..=8))]
    historical_workers: Option<u8>,
    out: PathBuf,
}
#[derive(Clone, Debug, PartialEq, Eq)]
struct Receipt {
    state: [u8; 32],
    owners: [u8; 32],
    substeps: Option<u8>,
    refined: Option<Vec<u64>>,
}
fn owners(f: &PoolFrame) -> Owners {
    f.workers
        .iter()
        .enumerate()
        .flat_map(|(i, w)| {
            w.generations
                .iter()
                .map(move |(&id, &g)| (id, (i as u8, g)))
        })
        .collect()
}
fn receipt(pool: &WorkerPool) -> Result<Receipt> {
    let f = pool.frame();
    Ok(Receipt {
        state: Sha256::digest(bincode::serialize(&(f.tick, &f.workers))?).into(),
        owners: Sha256::digest(bincode::serialize(&(f.tick, owners(f)))?).into(),
        substeps: pool.last_substeps(),
        refined: pool.last_island_refined_bodies().map(<[_]>::to_vec),
    })
}
fn pids(p: &WorkerPool) -> [Option<u32>; 4] {
    (0..4)
        .map(|w| p.worker_pid(w))
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn sample(pool: &WorkerPool, choice: Option<&PlacementRecord>, events: &[Value]) -> Result<Value> {
    let f = pool.frame();
    let ownership = owners(f);
    let bodies: Vec<_> = ownership.keys().map(|&id| f.body(id).unwrap().1).collect();
    let mut pairs: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    pairs.sort_by_key(|p| (p.a, p.b));
    let mut floors: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    floors.sort_by_key(|p| p.body);
    let state = receipt(pool)?;
    let sampler = pool.placement_status().unwrap();
    Ok(
        json!({"tick":f.tick,"bodies":bodies,"owners":ownership.values().map(|v|v.0).collect::<Vec<_>>(),
        "generations":ownership.values().map(|v|v.1).collect::<Vec<_>>(),"pairs":pairs,"floors":floors,
        "components":f.workers.iter().map(|w|&w.components).collect::<Vec<_>>(),
        "committed_placement":choice,"events":events,"pids":pids(pool),"window_counters":[sampler.windows_started,sampler.windows_completed,sampler.windows_interrupted,sampler.unavailable_probes],
        "substeps":state.substeps,"refined":state.refined,"public_sha256":format!("{:x}",Sha256::digest(bincode::serialize(&(f.tick,&f.workers))?)),
        "physical_sha256":format!("{:x}",Sha256::digest(bincode::serialize(&(f.tick,bodies,pairs,floors))?))}),
    )
}
fn write(w: &mut impl Write, row: &impl serde::Serialize) -> Result<()> {
    serde_json::to_writer(&mut *w, row)?;
    writeln!(w)?;
    Ok(())
}
fn writer(path: &Path) -> Result<BufWriter<File>> {
    Ok(BufWriter::new(File::create_new(path)?))
}
fn configure_replay(pool: &mut WorkerPool) -> Result<()> {
    pool.set_periodic_placement(Some("2,2,64,0".parse()?))?;
    pool.set_local_helper_limit(Some(1));
    pool.set_handoff_confirmation(true);
    pool.set_packed_handoffs(true);
    pool.set_packed_observations(true);
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir_all(&args.out)?;
    let oracle = Oracle::new(args.layout)?;
    let mut initial = fixture::Placement::Single.worker_seeds(&oracle.groups);
    initial.resize_with(args.historical_workers.unwrap_or(4) as usize, Vec::new);
    let mut recorded = WorkerPool::spawn(
        &args.worker,
        81044,
        initial,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        96,
    )?;
    if let Some(from) = &args.from_recording {
        return historical::verify(&args, from, &oracle, recorded);
    }
    recorded.set_periodic_placement(Some("8,40,24,0".parse()?))?;
    recorded.set_local_helper_limit(Some(0));
    recorded.set_handoff_confirmation(true);
    let scratch = tempfile::tempdir_in(&args.out)?;
    let start = scratch.path().join("initial.gzpool");
    recorded.save_native_recovery(&start)?;
    let tape_path = args.out.join("tape.jsonl");
    let mut tape_writer = writer(&tape_path)?;
    let mut recorded_frames = writer(&args.out.join("recorded.jsonl"))?;
    write(&mut recorded_frames, &sample(&recorded, None, &[])?)?;
    let initial_receipt = receipt(&recorded)?;
    let mut expected = vec![initial_receipt.clone()];
    let mut expected_owners = owners(recorded.frame());
    let mut decisions = Vec::new();
    for t in 1..=fixture::END {
        recorded
            .advance_islands_in_field(fixture::GRAVITY, &fixture::wrenches(t, &oracle.groups))?;
        let rec = recorded
            .last_island_placement()
            .ok_or("missing published decision")?;
        if rec.target_tick != t {
            return Err("incorrect published clock".into());
        }
        if let Some(m) = &rec.optional_move {
            for member in &m.members {
                let owner = expected_owners
                    .get_mut(&member.id)
                    .ok_or("unknown taped body")?;
                if *owner != (m.source, member.generation) {
                    return Err("incorrect recorded generation".into());
                }
                *owner = (m.destination, member.generation + 1);
            }
            decisions.push(rec.clone());
        }
        oracle.verify(recorded.frame(), &expected_owners)?;
        write(&mut tape_writer, rec)?;
        write(&mut recorded_frames, &sample(&recorded, Some(rec), &[])?)?;
        expected.push(receipt(&recorded)?);
    }
    if decisions.is_empty() {
        return Err(
            "the live scheduler made no transfers; run with at least two available CPUs".into(),
        );
    }
    tape_writer.flush()?;
    recorded_frames.flush()?;
    let automatic = recorded.placement_status().unwrap().clone();
    drop(recorded);
    let mut replay = PoolNativeRecovery::open(&start)?.resume(&args.worker)?;
    configure_replay(&mut replay)?;
    if receipt(&replay)? != initial_receipt {
        return Err("cold barrier differs after restore".into());
    }
    let mut replay_frames = writer(&args.out.join("replay.jsonl"))?;
    write(&mut replay_frames, &sample(&replay, None, &[])?)?;
    let mut summary_events = vec![];
    let mut state_hash = Sha256::new();
    let mut owner_hash = Sha256::new();
    let mut faulted = false;
    let mut count = 0;
    for line in BufReader::new(File::open(&tape_path)?).lines() {
        let record: PlacementRecord = serde_json::from_str(&line?)?;
        count += 1;
        let t = record.target_tick;
        if t != count || t > fixture::END {
            return Err("noncontiguous tape".into());
        }
        let loads = fixture::wrenches(t, &oracle.groups);
        let mut events = vec![];
        if let Some(m) = record.optional_move.as_ref().filter(|_| !faulted) {
            let lost = (0..4u8)
                .find(|w| *w != m.source && *w != m.destination)
                .unwrap();
            replay.drop_next_acknowledgement(lost)?;
            let previous = receipt(&replay)?;
            let choice = replay.last_island_placement().cloned();
            if !matches!(
                replay.advance_islands_with_placement(fixture::GRAVITY, &loads, &record),
                Err(PoolError::Transport(_))
            ) || receipt(&replay)? != previous
                || replay.last_island_placement() != choice.as_ref()
            {
                return Err("failed step changed public state or placement".into());
            }
            let wrong = PlacementRecord {
                target_tick: t,
                optional_move: None,
            };
            if !matches!(
                replay.advance_islands_with_placement(fixture::GRAVITY, &loads, &wrong),
                Err(PoolError::Pending)
            ) {
                return Err("different replay record accepted after an ambiguous outcome".into());
            }
            let before = pids(&replay);
            replay.terminate_worker(m.destination)?;
            replay.advance_islands_with_placement(fixture::GRAVITY, &loads, &record)?;
            let after = pids(&replay);
            if before[m.destination as usize] == after[m.destination as usize]
                || before[lost as usize] != after[lost as usize]
            {
                return Err("replacement or same-session confirmation mismatch".into());
            }
            events.push(json!({"kind":"lost_final_ack_and_recipient_death","tick":t,"lost_ack_worker":lost,"killed_recipient":m.destination,"before_pids":before,"after_pids":after,"visible_tick_on_failure":t-1,"changed_directive_rejected":true}));
            faulted = true;
        } else {
            replay.advance_islands_with_placement(fixture::GRAVITY, &loads, &record)?;
        }
        let actual = receipt(&replay)?;
        if expected[t as usize] != actual || replay.last_island_placement() != Some(&record) {
            return Err(format!("replay public receipt mismatch at {t}").into());
        }
        state_hash.update(actual.state);
        owner_hash.update(actual.owners);
        let status = replay.placement_status().unwrap();
        if status.windows_started != 0
            || status.windows_completed != 0
            || status.committed_moves != 0
        {
            return Err("replay consumed a timing measurement".into());
        }
        if [70, 160, 280].contains(&t) {
            let path = scratch.path().join(format!("tick-{t}.gzpool"));
            let old = pids(&replay);
            replay.save_native_recovery(&path)?;
            drop(replay);
            replay = PoolNativeRecovery::open(path)?.resume(&args.worker)?;
            let new = pids(&replay);
            if actual != receipt(&replay)?
                || replay.last_island_placement().is_some()
                || new.iter().zip(old).any(|(a, b)| *a == b)
            {
                return Err(
                    "fresh recovery changed the public state or carried an uncommitted directive"
                        .into(),
                );
            }
            configure_replay(&mut replay)?;
            events.push(json!({"kind":"fresh_coordinator_restore","tick":t,"before_pids":old,"after_pids":new}));
        }
        summary_events.extend_from_slice(&events);
        write(
            &mut replay_frames,
            &sample(&replay, Some(&record), &events)?,
        )?;
    }
    replay_frames.flush()?;
    if count != fixture::END || !faulted {
        return Err("partial replay coverage".into());
    }
    let summary = json!({"layout":args.layout,"ticks":count,"workers":4,"bodies":expected_owners.len(),"launch_ticks":fixture::LAUNCHES,
        "native_sha256":oracle.digest,"native_pairs":oracle.pairs,"native_floors":oracle.floors,
        "recorded_windows_started":automatic.windows_started,"recorded_windows_completed":automatic.windows_completed,"recorded_moves":decisions,
        "recorded_helpers":0,"replay_helpers":1,"replay_windows_started":0,"replay_windows_completed":0,
        "recorded_handoff_encoding":"json","replay_handoff_encoding":"packed","observation_encoding":"packed","events":summary_events,
        "tape_sha256":format!("{:x}",Sha256::digest(fs::read(tape_path)?)),
        "public_barrier_sha256":format!("{:x}",state_hash.finalize()),"ownership_barrier_sha256":format!("{:x}",owner_hash.finalize())});
    serde_json::to_writer_pretty(File::create_new(args.out.join("summary.json"))?, &summary)?;
    Ok(())
}
