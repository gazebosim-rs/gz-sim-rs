// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Re-run a selected historical tape without ever collecting a new timing session.
use super::*;
use serde::Deserialize;

#[derive(Deserialize)]
struct Historical {
    layout: fixture::Layout,
    ticks: u64,
    tape_sha256: String,
    native_sha256: String,
    public_barrier_sha256: String,
    ownership_barrier_sha256: String,
    recorded_moves: Vec<PlacementRecord>,
}
#[derive(Deserialize)]
struct Published {
    tick: u64,
    committed_placement: Option<PlacementRecord>,
    public_sha256: String,
    substeps: Option<u8>,
    refined: Option<Vec<u64>>,
    owners: Vec<u8>,
    generations: Vec<u64>,
}
pub(super) fn verify(
    args: &Args,
    from: &Path,
    oracle: &Oracle,
    mut pool: WorkerPool,
) -> Result<()> {
    let source: Historical = serde_json::from_reader(File::open(from.join("summary.json"))?)?;
    let bytes = fs::read(from.join("tape.jsonl"))?;
    if source.layout != args.layout
        || source.ticks != fixture::END
        || source.native_sha256 != oracle.digest
        || format!("{:x}", Sha256::digest(&bytes)) != source.tape_sha256
    {
        return Err("historical summary, native reference, layout or tape digest mismatch".into());
    }
    let tape = bytes
        .lines()
        .map(|s| Ok(serde_json::from_str::<PlacementRecord>(&s?)?))
        .collect::<Result<Vec<_>>>()?;
    let moves: Vec<_> = tape
        .iter()
        .filter(|r| r.optional_move.is_some())
        .cloned()
        .collect();
    if tape.len() != fixture::END as usize || moves != source.recorded_moves || moves.is_empty() {
        return Err("incomplete or inconsistent historical tape".into());
    }
    let mut published = BufReader::new(File::open(from.join("recorded.jsonl"))?).lines();
    let mut ledger = Sha256::new();
    let mut full_ledger = Sha256::new();
    let width = pool.frame().workers.len();
    let mut empty = pool.frame().workers[3].clone();
    let mut authority = Sha256::new();
    let scratch = tempfile::tempdir_in(&args.out)?;
    let mut restarts = vec![];
    configure_replay(&mut pool)?;
    let mut steps = 0;
    for (tick, record) in std::iter::once(None)
        .chain(tape.iter().map(Some))
        .enumerate()
    {
        let tick = tick as u64;
        let row: Published = serde_json::from_str(
            &published
                .next()
                .ok_or("historical public stream is truncated")??,
        )?;
        if row.tick != tick
            || row.committed_placement.as_ref() != record
            || row.owners.len() != 62
            || row.generations.len() != 62
        {
            return Err(format!("bad historical barrier at {tick}").into());
        }
        if let Some(r) = record {
            let inputs = fixture::wrenches(tick, &oracle.groups);
            if r.optional_move.is_some() {
                let image = scratch.path().join(format!("before-{tick}.gzpool"));
                let previous = receipt(&pool)?;
                pool.save_native_recovery(&image)?;
                drop(pool);
                pool = PoolNativeRecovery::open(image)?.resume(&args.worker)?;
                if previous != receipt(&pool)? || pool.last_island_placement().is_some() {
                    return Err("historical pre-move restore changed barrier".into());
                }
                configure_replay(&mut pool)?;
                restarts.push(tick - 1);
            }
            pool.advance_islands_with_placement(fixture::GRAVITY, &inputs, r)?;
            let owners = (1..=62)
                .zip(row.owners.into_iter().zip(row.generations))
                .collect();
            oracle.verify(pool.frame(), &owners)?;
            steps += 1;
        }
        let actual = receipt(&pool)?;
        // A wider pool retains the original four stable addresses. Compare each
        // extra endpoint to the observed empty bootstrap state at the real clock.
        empty.snapshot.tick = tick;
        if pool.frame().workers[4..]
            .iter()
            .any(|worker| worker != &empty)
        {
            return Err(format!("an extra worker became authoritative at tick {tick}").into());
        }
        let prefix = Sha256::digest(bincode::serialize(&(tick, &pool.frame().workers[..4]))?);
        let public = format!("{prefix:x}");
        if public != row.public_sha256
            || actual.substeps != row.substeps
            || actual.refined != row.refined
            || pool.last_island_placement() != record
            || pool.placement_status().unwrap().windows_started != 0
        {
            return Err(
                format!("historical replay public state or ownership mismatch at {tick}").into(),
            );
        }
        if record.is_some() {
            ledger.update(prefix);
            full_ledger.update(actual.state);
            authority.update(actual.owners);
        }
    }
    if published.next().is_some()
        || format!("{:x}", ledger.finalize()) != source.public_barrier_sha256
        || format!("{:x}", authority.finalize()) != source.ownership_barrier_sha256
    {
        return Err("recorded aggregate does not match complete reproduced trajectory".into());
    }
    let out = json!({"layout":args.layout,"ticks":steps,"historical_moves":moves.len(),"replay_workers":width,"full_width_public_sha256":format!("{:x}",full_ledger.finalize()),"before_move_restore_ticks":restarts,
        "native_sha256":oracle.digest,"native_pairs":oracle.pairs,"native_floors":oracle.floors,"tape_sha256":source.tape_sha256,
        "public_barrier_sha256":source.public_barrier_sha256,"ownership_barrier_sha256":source.ownership_barrier_sha256,
        "available_parallelism":std::thread::available_parallelism()?.get(),"replay_windows_started":0});
    serde_json::to_writer_pretty(
        File::create_new(args.out.join("historical-replay.json"))?,
        &out,
    )?;
    Ok(())
}
