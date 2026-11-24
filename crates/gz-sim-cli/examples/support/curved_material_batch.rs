// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure independent materials driven by a shared native guide. All physical material rows
//! and public sphere transforms are checked against an independent serial Bevy world.
use super::*;
use std::{fmt::Write as _, time::Duration};

pub(super) fn fixture(sizes: &[usize], seed: u64, threads: u8, processes: u8) -> Result<String> {
    if sizes.is_empty() || sizes.len() > 16 || sizes.iter().any(|n| !(16..=4096).contains(n)) {
        return Err("choose 1–16 cables with 16–4096 particles each".into());
    }
    let split = if processes == 1 {
        String::new()
    } else {
        format!(" worker_processes='{processes}'")
    };
    let mut sdf = xml(seed).replace(
        "filename=\"gz-sim-deformables-system\"",
        &format!("filename=\"gz-sim-deformables-system\" worker_threads=\"{threads}\"{split}"),
    );
    let a = sdf
        .find("        <visual name=\"node_")
        .ok_or("missing guide nodes")?;
    let b = a + sdf[a..].find("      </link>").ok_or("missing guide link")?;
    let mut visuals = String::new();
    let mut bodies = String::new();
    let mut bindings = String::new();
    for (lane, &n) in sizes.iter().enumerate() {
        let name = format!("line_{lane:02}");
        let phase = 0.7 * lane as f64;
        let offset = 0.065 * (lane as f64 / (sizes.len().max(2) - 1) as f64 - 0.5);
        let mut start = format!(
            "<cable name='{name}' substeps='4' iterations='6' stretch='0.00001' bend='0.01' radius='0.005' self_contact='false' friction='0'>\n"
        );
        for i in 0..n {
            let u = i as f64 / (n - 1) as f64;
            let x = offset + 0.003 * (phase + 6. * u).sin();
            let y = -0.276 + 0.552 * u;
            let z = 0.592 + 0.004 * (phase + 7. * u).cos() + 0.0004 * lane as f64;
            let mass = (0.13 + 0.01 * lane as f64) / n as f64;
            writeln!(
                start,
                "<particle position='{x:.17} {y:.17} {z:.17}' mass='{mass:.17}'/>"
            )?;
        }
        bodies.push_str(&start);
        bodies.push_str("</cable>\n");
        for sample in 0..16 {
            let vis = format!("line_{lane:02}_node_{sample:02}");
            let rgb = match lane % 4 {
                0 => "0.99 0.31 0.08",
                1 => "0.12 0.72 0.35",
                2 => "0.94 0.76 0.14",
                _ => "0.74 0.44 0.91",
            };
            writeln!(
                visuals,
                "        <visual name='{vis}'><geometry><sphere><radius>0.005</radius></sphere></geometry><material><diffuse>{rgb} 1</diffuse></material></visual>"
            )?;
            writeln!(
                bindings,
                "<particle_visual material='{name}' particle='{}' visual='default::guide::roller::{vis}'/>",
                sample * (n - 1) / 15
            )?;
        }
    }
    sdf.replace_range(a..b, &visuals);
    let a = sdf.find("<cable name=").ok_or("missing guide cable")?;
    let b = a + sdf[a..]
        .find("</plugin>")
        .ok_or("missing material plugin end")?;
    sdf.replace_range(a..b, &(bodies + &bindings));
    Ok(sdf)
}

pub(super) fn digest(sim: &Simulation) -> Result<(String, Vec<f64>)> {
    let mut hash = Sha256::new();
    let mut loads = Vec::new();
    for (name, body) in deformable::bodies(sim) {
        hash.update((name.len() as u64).to_le_bytes());
        hash.update(name);
        hash.update((body.particles().len() as u64).to_le_bytes());
        hash.update(body.last_path_substeps().to_le_bytes());
        for p in body.particles() {
            for v in p
                .position
                .into_iter()
                .chain(p.velocity)
                .chain([p.inverse_mass])
            {
                hash.update(v.to_le_bytes());
            }
        }
        for r in body.last_boundary_impulses() {
            hash.update((r.boundary as u64).to_le_bytes());
            for x in r.linear.into_iter().chain(r.angular) {
                hash.update(x.to_le_bytes());
            }
        }
        // This witness reports the complete upward support on each material. Different
        // materials can load different guides in the same physics cut.
        loads.push(
            body.last_boundary_impulses()
                .iter()
                .map(|r| (-r.linear[2]).max(0.))
                .sum(),
        );
    }
    if loads.is_empty() {
        return Err("no published materials".into());
    }
    // Covers the native graph, sensor, and every published local and world pose as well.
    hash.update(serde_json::to_vec(&observation(sim)?)?);
    Ok((format!("{:x}", hash.finalize()), loads))
}

pub(super) struct Run {
    pub ticks: u32,
    pub faults: bool,
    pub seed: u64,
    pub threads: u8,
    pub processes: u8,
    pub transport: String,
}
pub(super) fn run(worker: &Path, scratch: &Path, sizes: &[usize], options: Run) -> Result<()> {
    let Run {
        ticks,
        faults,
        seed,
        threads,
        processes,
        transport,
    } = options;
    if ticks < 8 {
        return Err("need at least eight public ticks".into());
    }
    let dir = tempfile::Builder::new()
        .prefix("gz-independent-cables-")
        .tempdir_in(scratch)?;
    let sdf = rpc::authored(fixture(sizes, seed, threads, processes)?, &transport)?;
    let mut local = setup(&sdf, None)?;
    let mut managed = setup(&sdf, Some(worker))?;
    let groups = usize::from(processes);
    if managed
        .world()
        .resource::<TrajectoryProcessPool>()
        .material_owners()
        .len()
        != groups
    {
        return Err("incorrect number of independent process owners".into());
    }
    let mut generations = vec![std::collections::BTreeSet::new(); groups];
    let mut deaths = vec![0u32; groups];
    let mut total_sent = 0u64;
    let mut total_received = 0u64;
    let mut stages = Vec::new();
    let mut aggregate = Sha256::new();
    let mut loaded = vec![0u32; sizes.len()];
    let mut max_load = vec![0f64; sizes.len()];
    let mut comparisons = 0;
    let mut recoveries = 0;
    let mut moves = 0;
    let mut reopens = 0;
    let mut max_snapshot = 0usize;
    let mut last = None;
    let mut actual_threads = std::collections::BTreeSet::new();
    managed
        .world_mut()
        .resource_mut::<TrajectoryProcessPool>()
        .set_profiling(true);
    for t in 0..ticks {
        let info = managed.committed_update_info()?;
        if faults && (t == 25 || t == 51 || t % 71 == 36) {
            let g = moves as usize % groups;
            let owners = managed
                .world()
                .resource::<TrajectoryProcessPool>()
                .material_owners();
            trajectory_workers::select_material_group(
                &mut managed,
                info.epoch,
                info.iteration,
                g,
                1 - owners[g],
            )?;
            moves += 1;
        }
        if faults && (t == 38 || t == 77 || t % 109 == 41) {
            let file = dir.path().join(format!("cut-{t}"));
            checkpoint::save(&managed, &file)?;
            let expected = digest(&managed)?;
            let owners = managed
                .world()
                .resource::<TrajectoryProcessPool>()
                .material_owners();
            drop(managed);
            managed = checkpoint::open(&file, worker)?;
            managed
                .world_mut()
                .resource_mut::<TrajectoryProcessPool>()
                .set_profiling(true);
            if managed
                .world()
                .resource::<TrajectoryProcessPool>()
                .material_owners()
                != owners
                || digest(&managed)? != expected
            {
                return Err("multi-material durable publication changed".into());
            }
            reopens += 1;
        }
        let kill = faults && (t == 24 || t == 40 || t % 62 == 32);
        let mut victims = Vec::new();
        if kill {
            let p = managed.world().resource::<TrajectoryProcessPool>();
            let owners = p.material_owners();
            if t % 62 == 32 {
                victims.extend(0..groups);
            } else {
                victims.push((recoveries as usize + t as usize) % groups);
            }
            for &g in &victims {
                terminate_worker(
                    p.group_pid(g, owners[g])
                        .ok_or("no resident owner to interrupt")?,
                )?;
                deaths[g] += 1;
            }
        }
        managed.step_exact(1)?;
        local.step_exact(1)?;
        let state = digest(&managed)?;
        let wanted = digest(&local)?;
        if state != wanted {
            return Err(format!("independent material bundle diverged at {t}").into());
        }
        aggregate.update(&state.0);
        for (i, j) in state.1.iter().enumerate() {
            loaded[i] += u32::from(*j > 0.);
            max_load[i] = max_load[i].max(j / 0.002);
        }
        last = Some(state.0);
        let r = managed
            .world()
            .resource::<TrajectoryProcessPool>()
            .last()
            .unwrap();
        let sample = managed
            .world()
            .resource::<TrajectoryProcessPool>()
            .last_material_steps();
        let unique: std::collections::BTreeSet<_> = sample.iter().map(|v| v.pid).collect();
        if sample.len() != groups || unique.len() != groups {
            return Err("worker leases do not have distinct live processes".into());
        }
        for (g, row) in sample.iter().enumerate() {
            if row.restarted != victims.contains(&g) {
                return Err(format!("unexpected recovery at {t} of group {g}").into());
            }
            generations[g].insert(row.pid);
            total_sent += row.source_bytes as u64;
            total_received += row.successor_bytes as u64;
        }
        comparisons += 1;
        if r.restarted != kill {
            return Err(format!("worker recovery mismatch at {t}").into());
        }
        recoveries += victims.len() as u32;
        let p = r.profile.ok_or("worker did not provide profile")?;
        actual_threads.insert(p.material_threads);
        let ns = |t: Duration| t.as_nanos() as u64;
        stages.push([
            ns(r.wall),
            ns(p.child_solve),
            ns(p.child_snapshot),
            ns(p.capture),
            ns(p.install),
        ]);
        max_snapshot = max_snapshot.max(r.successor_bytes);
    }
    let mut percentiles = Vec::new();
    for col in 0..5 {
        let mut v = stages.iter().map(|row| row[col]).collect::<Vec<_>>();
        v.sort_unstable();
        percentiles.push([
            v[(v.len() * 50).div_ceil(100) - 1],
            v[(v.len() * 95).div_ceil(100) - 1],
            v[(v.len() * 99).div_ceil(100) - 1],
        ]);
    }
    println!(
        "{}",
        json!({"sizes":sizes,"requested_threads":threads,"transport":transport,"distinct_processes_per_tick":groups,"observed_process_generations":generations,"deaths_by_group":deaths,"total_transferred_source_bytes":total_sent,"total_successor_bytes":total_received,"active_threads":actual_threads,"public_ticks":ticks,"seed":seed,"exact_cuts":comparisons,"migrations":moves,"worker_sigkills":recoveries,
        "durable_reopens":reopens,"loaded_ticks":loaded,"peak_interval_loads_n":max_load,"largest_successor":max_snapshot,
        "stage_order":["wall","child_solve","child_snapshot","capture","install"],"stage_p50_p95_p99_ns":percentiles,
        "all_ticks_sha256":format!("{:x}",aggregate.finalize()),"last_sha256":last})
    );
    Ok(())
}

pub(super) fn sizes(arg: &str) -> Result<Vec<usize>> {
    Ok(arg
        .split(',')
        .map(str::parse)
        .collect::<std::result::Result<_, _>>()?)
}

/// Validate every public material and native row before and after an immutable scene save.
/// Optional input can be an older-format archive at the same tick, for compatibility checks.
pub(super) fn archives(
    worker: &Path,
    sdf: &Path,
    dir: &Path,
    tick: u32,
    repeats: usize,
    prior: Option<&Path>,
) -> Result<()> {
    use std::time::Instant;
    if repeats == 0 {
        return Err("need one or more immutable saves".into());
    }
    let text = fs::read_to_string(sdf)?;
    let mut serial = setup(&text, None)?;
    let mut remote = setup(&text, Some(worker))?;
    serial.step_exact(tick as u64)?;
    remote.step_exact(tick as u64)?;
    let expected = digest(&serial)?;
    if expected != digest(&remote)? {
        return Err("material bundle differs before saving".into());
    }
    let scratch = tempfile::Builder::new()
        .prefix("gz-material-archive-")
        .tempdir_in(dir)?;
    let mut sizes = Vec::new();
    let mut times = Vec::new();
    let mut resumes = Vec::new();
    let mut checks = 0usize;
    let mut controllers = Vec::new();
    for i in 0..repeats {
        let path = scratch.path().join(format!("cut-{i}"));
        let start = Instant::now();
        checkpoint::save(&remote, &path)?;
        times.push(start.elapsed().as_nanos());
        sizes.push(fs::metadata(&path)?.len());
        let start = Instant::now();
        let restored = checkpoint::open(&path, worker)?;
        resumes.push(start.elapsed().as_nanos());
        if digest(&restored)? != expected {
            return Err("packed material publication differs after open".into());
        }
        checks += 1;
        controllers.push(restored);
    }
    if let Some(legacy) = prior {
        let restored = checkpoint::open(legacy, worker)?;
        if digest(&restored)? != expected {
            return Err("previous-format state differs at reopen".into());
        }
        controllers.push(restored);
        checks += 1;
    }
    let mut trajectory = Sha256::new();
    for _ in 0..32 {
        serial.step_exact(1)?;
        let expected = digest(&serial)?;
        for restored in &mut controllers {
            restored.step_exact(1)?;
            if digest(restored)? != expected {
                return Err("material continuation diverged".into());
            }
            checks += 1;
        }
        trajectory.update(&expected.0);
    }
    println!(
        "{}",
        json!({"authored_bytes":text.len(),"particles":deformable::bodies(&serial).map(|(_,m)|m.particles().len()).sum::<usize>(),"materials":expected.1.len(),
        "load_tick":tick,"saves":repeats,"legacy_checked":prior.is_some(),"exact_comparisons":checks,"archive_bytes":sizes,
        "save_ns":times,"resume_ns":resumes,"next_32_sha256":format!("{:x}",trajectory.finalize())})
    );
    Ok(())
}
