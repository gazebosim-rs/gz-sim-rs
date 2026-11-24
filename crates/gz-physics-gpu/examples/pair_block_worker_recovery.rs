// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify real process death and driver re-creation around one durable GPU block.
#[path = "support/pair_recovery/partition.rs"]
mod partition;
#[allow(dead_code)]
#[path = "../../gz-physics-distributed/examples/support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[allow(dead_code)]
#[path = "support/sphere_pair/reference.rs"]
mod reference;
#[path = "support/pair_recovery/witness.rs"]
mod witness;
use gz_physics_gpu::{SpherePairBlockBatch, SpherePairBlockCheckpoint as Cut, SphereRotation};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{File, OpenOptions},
    io::{BufRead, BufReader, Read, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const NAME: &str = "pair-block.gzpair";
fn read(p: &Path) -> Result<(Vec<u8>, Cut)> {
    let f = File::open(p.join(NAME))?;
    if f.metadata()?.len() > Cut::MAX_BYTES as u64 {
        return Err("oversized checkpoint".into());
    }
    let mut b = Vec::new();
    f.take(Cut::MAX_BYTES as u64 + 1).read_to_end(&mut b)?;
    let c = Cut::from_bytes(&b)?;
    Ok((b, c))
}
fn pause(where_: &str, cut: &str) -> Result<()> {
    if where_ == cut {
        println!("at {where_}");
        std::io::stdout().flush()?;
        // The parent sends a real OS kill at this boundary. A vanished parent
        // instead closes stdin, so its worker can exit without holding the lease.
        let mut byte = [0];
        std::io::stdin().read_exact(&mut byte)?;
    }
    Ok(())
}
fn save(dir: &Path, cut: &Cut, at: &str) -> Result<()> {
    let mut f = tempfile::NamedTempFile::new_in(dir)?;
    f.write_all(&cut.to_bytes()?)?;
    f.as_file().sync_all()?;
    pause("prepared", at)?;
    f.persist(dir.join(NAME))?;
    File::open(dir)?.sync_all()?;
    pause("committed", at)?;
    Ok(())
}
fn worker(dir: &Path, adapter: &str, expected: u64, ticks: u32, at: &str) -> Result<()> {
    let lock = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(dir.join("owner.lock"))?;
    lock.try_lock()?;
    let (_, cut) = read(dir)?;
    if cut.tick() != expected {
        return Err(format!("stale boundary {} vs {expected}", cut.tick()).into());
    }
    let mut b = cut.restore(adapter)?;
    pause("restored", at)?;
    b.step_block(ticks)?;
    pause("solved", at)?;
    save(dir, &Cut::capture(&b)?, at)?;
    println!("done {}", b.tick());
    Ok(())
}
fn launch(dir: &Path, adapter: &str, t: u64, k: u32, where_: &str) -> Result<()> {
    let mut child = Command::new(std::env::current_exe()?)
        .args(["worker", adapter])
        .arg(dir)
        .args([t.to_string(), k.to_string(), where_.to_owned()])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()?;
    let mut line = String::new();
    BufReader::new(child.stdout.take().unwrap()).read_line(&mut line)?;
    if where_ == "none" {
        let status = child.wait()?;
        if !status.success() || line.trim() != format!("done {}", t + u64::from(k)) {
            return Err(format!("worker failed: {status} {line}").into());
        }
    } else {
        if line.trim() != format!("at {where_}") {
            let _ = child.kill();
            let _ = child.wait();
            return Err(format!("worker missed {where_}: {line}").into());
        }
        child.kill()?;
        let status = child.wait()?;
        if status.success() {
            return Err("worker unexpectedly exited normally".into());
        }
    }
    Ok(())
}
fn exercise(
    adapter: &str,
    dir: &Path,
    seed: u64,
    n: usize,
    blocks: u32,
    mode: SphereRotation,
) -> Result<()> {
    // Unique geometry per episode, including approaching, grazing and
    // separating controls. Initial translations do not change relative speed.
    let mut seeds = reference::fixture(n);
    let mut rng = seed;
    for lane in &mut seeds {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u = (rng >> 11) as f64 / 9007199254740992.;
        let scale = 0.65 + u;
        for b in [&mut lane.a, &mut lane.b] {
            b.position[2] += u;
            b.velocity = b.velocity.map(|v| v * scale);
        }
        if rng & 8 != 0 {
            std::mem::swap(&mut lane.a, &mut lane.b);
        }
    }
    let mut oracle = SpherePairBlockBatch::with_rotation(adapter, seeds, mode)?;
    let cold = Cut::capture(&oracle)?;
    save(dir, &cold, "none")?;
    let start = Instant::now();
    let mut hits = 0u64;
    let mut kills = 0u32;
    let mut restored_bytes = 0;
    let mut digest = Sha256::new();
    for block in 0..blocks {
        let before = Cut::capture(&oracle)?.to_bytes()?;
        let t = oracle.tick();
        let size = [23u32, 51, 16, 80, 1, 128, 7, 255][(block as usize + seed as usize % 8) % 8];
        oracle.step_block(size)?;
        let next = Cut::capture(&oracle)?.to_bytes()?;
        let at = ["restored", "solved", "prepared", "committed"]
            [(block as usize + seed as usize % 4) % 4];
        launch(dir, adapter, t, size, at)?;
        kills += 1;
        let (actual, _) = read(dir)?;
        if actual.as_slice()
            != if at == "committed" {
                next.as_slice()
            } else {
                before.as_slice()
            }
        {
            return Err(format!("partial durable frame after {at}, block {block}").into());
        }
        if at != "committed" {
            launch(dir, adapter, t, size, "none")?;
        }
        let (actual, cut) = read(dir)?;
        if actual != next {
            return Err(format!("reopened block differs at {}", cut.tick()).into());
        }
        restored_bytes += actual.len();
        digest.update(Sha256::digest(&actual));
        hits += cut
            .observations()
            .iter()
            .map(|r| u64::from(r.contact_ticks()))
            .sum::<u64>();
    }
    let (_, last) = read(dir)?;
    let mut reopened = last.restore(adapter)?;
    reopened.reset();
    if Cut::capture(&reopened)?.to_bytes()? != cold.to_bytes()? {
        return Err("reopened worker lost original reset seeds".into());
    }
    println!(
        "{}",
        json!({"seed":seed,"rotation":format!("{mode:?}"),"blocks":blocks,"workers_killed":kills,"physical_ticks":oracle.tick(),"lanes":n,"loaded_lane_ticks":hits,"verified_bytes":restored_bytes,"elapsed_s":start.elapsed().as_secs_f64(),"sha256":format!("{:x}",digest.finalize())})
    );
    Ok(())
}
fn main() -> Result<()> {
    let mut a = std::env::args().skip(1);
    let mode = a.next().ok_or("need run or worker")?;
    let adapter = a.next().ok_or("need adapter")?;
    let dir = PathBuf::from(a.next().ok_or("need checkpoint root")?);
    match mode.as_str() {
        "compare-witness" => witness::compare(
            Path::new(&adapter),
            &dir,
            &PathBuf::from(a.next().ok_or("need comparison view directory")?),
        ),
        "shard-worker" => partition::worker(&adapter),
        "shard-owner" => partition::coordinator(&adapter, &dir),
        "partition" | "partition-compact" | "partition-timing" => {
            let seed = a.next().ok_or("seed")?.parse()?;
            let n = a.next().ok_or("lanes")?.parse()?;
            let k = a.next().ok_or("intervals")?.parse()?;
            let trial = if mode == "partition-timing" {
                let width = a.next().ok_or("worker count")?.parse()?;
                let compact = match a.next().as_deref() {
                    Some("compact") => true,
                    Some("full") => false,
                    _ => return Err("timing requires compact or full".into()),
                };
                partition::Trial::Timing { width, compact }
            } else {
                partition::Trial::Faults {
                    compact: mode == "partition-compact",
                }
            };
            let view = a.next().map(PathBuf::from);
            let episode = tempfile::tempdir_in(dir)?;
            let result =
                partition::exercise(&adapter, episode.path(), seed, n, k, view.as_deref(), trial);
            if result.is_err() {
                eprintln!("failed input retained at {}", episode.keep().display());
            }
            result
        }
        "witness" => witness::record(&adapter, &dir),
        "worker" => worker(
            &dir,
            &adapter,
            a.next().ok_or("clock")?.parse()?,
            a.next().ok_or("block")?.parse()?,
            &a.next().ok_or("fault cut")?,
        ),
        "run" => {
            let seed: u64 = a.next().ok_or("seed")?.parse()?;
            let n = a.next().ok_or("size")?.parse()?;
            let k = a.next().ok_or("blocks")?.parse()?;
            let d = tempfile::tempdir_in(dir)?;
            let res = exercise(
                &adapter,
                d.path(),
                seed,
                n,
                k,
                if seed.is_multiple_of(2) {
                    SphereRotation::Midpoint
                } else {
                    SphereRotation::FourthOrder
                },
            );
            if res.is_err() {
                eprintln!("failed input retained at {}", d.keep().display());
            }
            res
        }
        _ => Err("expected run|worker|witness".into()),
    }
}
