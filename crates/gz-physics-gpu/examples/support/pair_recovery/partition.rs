// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise warm local device workers and crash the coordinator around one durable global publication.
#[path = "process.rs"]
mod process;
use super::{Cut, Result, primitive_replay_view::View, read, reference, save, witness};
use gz_physics_gpu::{
    PairBlockBarrier, PairBlockReceipt, SphereRotation,
    simulation::sphere_pair_block_simulation_with_rotation,
};
pub(crate) use process::worker;
use process::{Process, get_frame, get_number};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, OpenOptions},
    io::{self, Read, Write},
    path::Path,
    time::Instant,
};
const STEP: u32 = 8;

pub(crate) fn coordinator(adapter: &str, dir: &Path) -> Result<()> {
    let lease = OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(dir.join("owner.lock"))?;
    lease.try_lock()?;
    let (_, mut cut) = read(dir)?;
    let mut workers = Vec::<Process>::new();
    let mut generation = 0;
    loop {
        let mut cmd = String::new();
        if io::stdin().read_line(&mut cmd)? == 0 {
            return Ok(());
        }
        let mut words = cmd.split_whitespace();
        let expected: u64 = words.next().ok_or("missing source tick")?.parse()?;
        let width: usize = words.next().ok_or("missing width")?.parse()?;
        let victim: usize = words.next().ok_or("missing selected fault")?.parse()?;
        let fault = words.next().ok_or("global cut")?;
        let compact = words.next() == Some("compact");
        let rotate = words.next() == Some("rotate");
        if words.next().is_some()
            || !matches!(fault, "none" | "prepared" | "committed")
            || cut.tick() != expected
            || !(1..=16).contains(&width)
        {
            return Err("stale coordinator request or invalid local worker count".into());
        }
        let timer = Instant::now();
        let barrier = PairBlockBarrier::new(&cut, STEP, width)?;
        workers.truncate(width);
        while workers.len() < width {
            workers.push(Process::start("shard-worker", adapter, dir)?);
        }
        let mut placements: Vec<_> = (0..width).collect();
        // Assignments rotate across OS workers occasionally while keeping their
        // contiguous public lane identities and the canonical output ordering.
        if rotate && generation % 10 >= 6 {
            placements.rotate_left(1 % width);
        }
        let mut sent = 0u64;
        let mut received = 0u64;
        let mut misses = 0;
        for (i, &slot) in placements.iter().enumerate() {
            let job = if compact {
                barrier.continuation(i)?.to_bytes().to_vec()
            } else {
                barrier.assignment(i)?.to_bytes()?
            };
            workers[slot].send(&job, slot == victim)?;
            sent += job.len() as u64;
        }
        let mut receipts = Vec::with_capacity(width + 1);
        let (mut killed, mut warm, mut upload, mut download, mut dispatch) = (0, 0, 0, 0, 0);
        // Reverse receipt order relative to the owned lane ranges.
        for (i, &slot) in placements.iter().enumerate().rev() {
            let mut status = [0];
            workers[slot].output.read_exact(&mut status)?;
            if status == *b"M" && compact {
                misses += 1;
                let job = barrier.assignment(i)?.to_bytes()?;
                workers[slot].send(&job, slot == victim)?;
                sent += job.len() as u64;
                workers[slot].output.read_exact(&mut status)?;
            }
            if status == *b"H" && slot == victim {
                workers[slot].kill()?;
                killed += 1;
                workers[slot] = Process::start("shard-worker", adapter, dir)?;
                let job = barrier.assignment(i)?.to_bytes()?;
                workers[slot].send(&job, false)?;
                sent += job.len() as u64;
                workers[slot].output.read_exact(&mut status)?;
            }
            if status != *b"R" {
                return Err("unexpected local worker response".into());
            }
            let out = &mut workers[slot].output;
            warm += get_number(out)?;
            upload += get_number(out)?;
            download += get_number(out)?;
            dispatch += get_number(out)?;
            let data = get_frame(out)?;
            received += data.len() as u64;
            if i == 0 {
                receipts.push(PairBlockReceipt::from_bytes(&data)?);
            }
            receipts.push(PairBlockReceipt::from_bytes(&data)?);
        }
        let next = barrier.commit(receipts)?;
        save(dir, &next, fault)?;
        cut = next;
        generation += 1;
        println!(
            "{}",
            json!({"tick":cut.tick(),"workers":width,"placements":placements,"worker_kills":killed,"resident_reuses":warm,"upload_bytes":upload,"readback_bytes":download,"wire_bytes":sent+received,"request_bytes":sent,"reply_bytes":received,"continuation_misses":misses,"dispatch_sum_ns":dispatch,"wall_us":timer.elapsed().as_micros()})
        );
        io::stdout().flush()?;
    }
}
#[derive(Clone, Copy)]
pub(crate) enum Trial {
    Faults { compact: bool },
    Timing { compact: bool, width: usize },
}
impl Trial {
    fn compact(self) -> bool {
        match self {
            Self::Faults { compact } | Self::Timing { compact, .. } => compact,
        }
    }
    fn faults(self) -> bool {
        matches!(self, Self::Faults { .. })
    }
}
fn call(
    owner: &mut Process,
    at: u64,
    width: usize,
    victim: usize,
    cut: &str,
    trial: Trial,
) -> Result<String> {
    let encoding = if trial.compact() { "compact" } else { "full" };
    let rotate = if trial.faults() { "rotate" } else { "fixed" };
    writeln!(
        owner.input,
        "{at} {width} {victim} {cut} {encoding} {rotate}"
    )?;
    owner.input.flush()?;
    owner.line()
}
pub(crate) fn exercise(
    adapter: &str,
    dir: &Path,
    seed: u64,
    n: usize,
    intervals: usize,
    view: Option<&Path>,
    trial: Trial,
) -> Result<()> {
    if n < 4 || intervals == 0 {
        return Err("need at least four lanes and a positive interval count".into());
    }
    let mut lanes = reference::fixture(n);
    let mut rng = seed;
    for (i, l) in lanes.iter_mut().enumerate() {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        if i != 0 {
            let boost = (rng >> 11) as f64 / 9007199254740992. - 0.5;
            for b in [&mut l.a, &mut l.b] {
                b.velocity[2] += boost;
                b.position[1] += 0.07 * boost;
            }
        }
    }
    let mut oracle = sphere_pair_block_simulation_with_rotation(
        adapter,
        lanes,
        STEP,
        if seed.is_multiple_of(2) {
            SphereRotation::FourthOrder
        } else {
            SphereRotation::Midpoint
        },
    )?;
    oracle.reset()?;
    let first = Cut::capture_simulation(&oracle)?;
    save(dir, &first, "none")?;
    let mut owner = Process::start("shard-owner", adapter, dir)?;
    let mut frames = Vec::new();
    let mut rows = Vec::new();
    let mut total = json!({"worker_kills":0,"controller_kills":0,"resident_reuses":0,"wire_bytes":0,"loaded_lane_ticks":0});
    let mut measurements = Vec::new();
    for i in 0..=intervals {
        let width = match trial {
            Trial::Faults { .. } => [1, 3, 2, 4][(i.saturating_sub(1) / 7) % 4],
            Trial::Timing { width, .. } => width,
        };
        let at = if trial.faults() && i > 0 && (i + seed as usize % 17) % 17 == 3 {
            "prepared"
        } else if trial.faults() && i > 0 && (i + seed as usize % 19) % 19 == 7 {
            "committed"
        } else {
            "none"
        };
        let victim = if trial.faults() && i > 0 && (i + seed as usize % 5) % 5 == 2 {
            (i / 5) % width
        } else {
            width
        };
        let mut record = Value::Null;
        if i != 0 {
            let (previous, cut) = read(dir)?;
            let t = cut.tick();
            oracle.step_exact(1)?;
            let next = Cut::capture_simulation(&oracle)?.to_bytes()?;
            let response = call(&mut owner, t, width, victim, at, trial)?;
            if at != "none" {
                if response != format!("at {at}") {
                    return Err(format!("owner failed to reach {at}: {response}").into());
                }
                owner.kill()?;
                let (raw, found) = read(dir)?;
                if raw.as_slice()
                    != if at == "prepared" {
                        &previous[..]
                    } else {
                        &next[..]
                    }
                {
                    return Err(format!("torn global cut at {}", found.tick()).into());
                }
                total["controller_kills"] = json!(total["controller_kills"].as_u64().unwrap() + 1);
                owner = Process::start("shard-owner", adapter, dir)?;
                if at == "prepared" {
                    record =
                        serde_json::from_str(&call(&mut owner, t, width, width, "none", trial)?)?;
                }
            } else {
                record = serde_json::from_str(&response)?;
            }
            let (raw, c) = read(dir)?;
            if raw != next {
                return Err(format!(
                    "partition changed complete Bevy/device bytes at {}",
                    c.tick()
                )
                .into());
            }
            // The selected victim died before either global crash cut.
            total["worker_kills"] =
                json!(total["worker_kills"].as_u64().unwrap() + u64::from(victim < width));
            for key in ["resident_reuses", "wire_bytes"] {
                if let Some(v) = record[key].as_u64() {
                    total[key] = json!(total[key].as_u64().unwrap() + v);
                }
            }
            let loaded = c
                .observations()
                .iter()
                .map(|v| u64::from(v.contact_ticks()))
                .sum::<u64>();
            total["loaded_lane_ticks"] =
                json!(total["loaded_lane_ticks"].as_u64().unwrap() + loaded);
        }
        let (bytes, committed) = read(dir)?;
        if view.is_some() {
            let origin = Cut::capture_simulation(&oracle)?;
            let mut v = View::default();
            let a = witness::row(
                &mut v,
                "uninterrupted",
                100,
                0.26,
                &origin.observations()[0],
            );
            let b = witness::row(
                &mut v,
                "distributed",
                200,
                -0.26,
                &committed.observations()[0],
            );
            let y = 0.27 - (width - 1) as f64 * 0.18;
            v.cuboid(
                300,
                "active_workers_scale",
                [0., 0., -0.51],
                [0.025, 0.62, 0.02],
                [0.4, 0.45, 0.52, 1.],
            );
            v.add(
                304,
                "active_workers",
                [0., y, -0.488],
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.035}),
                [0.17, 0.86, 0.49, 1.],
            );
            frames.push(v.finish_at(i as u64 + 1, committed.tick() * 1_000_000));
            rows.push(json!({"tick":committed.tick(),"workers":width,"global_fault":at,"victim":(victim<width).then_some(victim),"cut_sha256":format!("{:x}",Sha256::digest(&bytes)),"rows":[a,b]}));
        }
        if record.is_object() {
            measurements.push(record);
        }
    }
    let (_, end) = read(dir)?;
    let mut restored = end.restore_simulation(adapter)?;
    restored.reset()?;
    oracle.reset()?;
    if Cut::capture_simulation(&restored)?.to_bytes()?
        != Cut::capture_simulation(&oracle)?.to_bytes()?
    {
        return Err("distributed reset differs from unbroken Bevy world".into());
    }
    if let Some(out) = view {
        fs::create_dir(out)?;
        gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
        fs::write(
            out.join("index.json"),
            serde_json::to_vec(
                &json!({"seed":seed,"lanes":n,"totals":total,"measurements":measurements,"frames":rows}),
            )?,
        )?;
    }
    if let Trial::Timing { compact, width } = trial {
        let steady = &measurements[1..]; // Exclude first device creation from steady-state latency.
        if steady.is_empty() {
            return Err("timing needs two or more intervals".into());
        }
        let mut wall: Vec<_> = steady
            .iter()
            .map(|m| m["wall_us"].as_u64().unwrap())
            .collect();
        wall.sort_unstable();
        let total = |k: &str| steady.iter().map(|m| m[k].as_u64().unwrap()).sum::<u64>();
        println!(
            "{}",
            json!({"kind":"timing","seed":seed,"lanes":n,"workers":width,"compact":compact,"intervals":wall.len(),
        "p50_us":wall[wall.len().div_ceil(2)-1],"p95_us":wall[(wall.len()*95).div_ceil(100)-1],"p99_us":wall[(wall.len()*99).div_ceil(100)-1],
        "request_bytes":total("request_bytes"),"reply_bytes":total("reply_bytes"),"upload_bytes":total("upload_bytes"),"resident_reuses":total("resident_reuses"),"first_us":measurements[0]["wall_us"]})
        );
        return Ok(());
    }
    println!(
        "{}",
        json!({"seed":seed,"lanes":n,"intervals":intervals,"ticks":end.tick(),"totals":total,"completed_timed_calls":measurements.len(),"compact":trial.compact(),"reset":true})
    );
    Ok(())
}
