// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Restart the complete Bevy controller from immutable cuts with independently placed materials.
use super::*;
use std::process::Command;

pub(super) fn state(sim: &Simulation) -> Result<Value> {
    let (digest, loads) = batch::digest(sim)?;
    Ok(json!({"hash":digest,"loads":loads}))
}
fn owners(sim: &Simulation) -> Vec<usize> {
    sim.world()
        .resource::<TrajectoryProcessPool>()
        .material_owners()
}
fn receipt(sim: &Simulation) -> Result<Value> {
    Ok(json!({"state":state(sim)?,"owners":owners(sim)}))
}
pub(super) fn child(sdf: &Path, worker: &Path, dir: &Path, cut: Option<&Path>) -> Result<Actor> {
    let mut c = Command::new(std::env::current_exe()?);
    c.env("TMPDIR", dir)
        .arg("partition-child")
        .arg(sdf)
        .arg(worker);
    if let Some(p) = cut {
        c.arg(p);
    }
    Actor::from_command(c)
}
pub(super) fn serve(sdf: &Path, worker: &Path, cut: Option<&Path>) -> Result<()> {
    let mut sim = if let Some(p) = cut {
        checkpoint::open(p, worker)?
    } else {
        setup(&fs::read_to_string(sdf)?, Some(worker))?
    };
    let mut out = std::io::stdout().lock();
    writeln!(out, "{}", receipt(&sim)?)?;
    out.flush()?;
    for line in std::io::stdin().lock().lines() {
        let line = line?;
        let (verb, arg) = line.split_once(' ').unwrap_or((&line, ""));
        let result = match verb {
            "advance" => {
                sim.step_exact(1)?;
                let rows = sim
                    .world()
                    .resource::<TrajectoryProcessPool>()
                    .last_material_steps();
                json!({"state":state(&sim)?,"restarted":rows.iter().map(|r|r.restarted).collect::<Vec<_>>(),"pids":rows.iter().map(|r|r.pid).collect::<Vec<_>>(),"warm_deadline_us":rows.iter().map(|r|r.warm_deadline.map(|d|d.as_micros())).collect::<Vec<_>>(),"warm_timeouts":rows.iter().map(|r|r.warm_timeout).collect::<Vec<_>>()})
            }
            "routes" => json!(sim.world().resource::<TrajectoryProcessPool>().transports()),
            "transport" => {
                let (g, route) = arg.split_once(' ').ok_or("need group and RPC transport")?;
                let i = sim.committed_update_info()?;
                let route = serde_json::from_value(json!(route))?;
                trajectory_workers::select_transport(
                    &mut sim,
                    i.epoch,
                    i.iteration,
                    g.parse()?,
                    route,
                )?;
                json!(sim.world().resource::<TrajectoryProcessPool>().transports())
            }
            "stale-route" => {
                let i = sim.committed_update_info()?;
                json!(
                    trajectory_workers::select_transport(
                        &mut sim,
                        i.epoch,
                        i.iteration + 1,
                        0,
                        trajectory_workers::WorkerTransport::Pipe
                    )
                    .is_err()
                )
            }
            "tcp" => rpc::sockets(&sim)?,
            "delay" => {
                let (g, ms) = arg
                    .split_once(' ')
                    .ok_or("need group and delay in milliseconds")?;
                let g: usize = g.parse()?;
                let p = sim.world().resource::<TrajectoryProcessPool>();
                let slot = p.material_owners()[g];
                let pid = p.group_pid(g, slot).ok_or("missing delayed child")?;
                rpc::brief_delay(pid, ms.parse()?)?;
                json!(pid)
            }
            "stall" => {
                let group: usize = arg.parse()?;
                let pool = sim.world().resource::<TrajectoryProcessPool>();
                let slot = pool.material_owners()[group];
                let pid = pool
                    .group_pid(group, slot)
                    .ok_or("missing child to stall")?;
                rpc::suspend(pid)?;
                json!(pid)
            }
            "park-save" => {
                sim.set_paused(true);
                checkpoint::save(&sim, Path::new(arg))?;
                receipt(&sim)?
            }
            "save" => {
                checkpoint::save(&sim, Path::new(arg))?;
                receipt(&sim)?
            }
            "move" => {
                let g: usize = arg.parse()?;
                let i = sim.committed_update_info()?;
                let slot = *owners(&sim).get(g).ok_or("unknown material group")?;
                trajectory_workers::select_material_group(
                    &mut sim,
                    i.epoch,
                    i.iteration,
                    g,
                    1 - slot,
                )?;
                json!(owners(&sim))
            }
            "kill" => {
                let g: usize = arg.parse()?;
                let p = sim.world().resource::<TrajectoryProcessPool>();
                let s = *p.material_owners().get(g).ok_or("unknown target")?;
                let pid = p
                    .group_pid(g, s)
                    .ok_or("selected group has no active process")?;
                terminate_worker(pid)?;
                json!(pid)
            }
            "prepare" => {
                prepare(&mut sim)?;
                Value::Null
            }
            "reset" => {
                reset_loaded_world(&mut sim, |_| ())?;
                receipt(&sim)?
            }
            _ => return Err("unknown partition controller command".into()),
        };
        writeln!(out, "{result}")?;
        out.flush()?;
    }
    Ok(())
}
pub(super) fn run(sdf: &Path, worker: &Path, scratch: &Path, ticks: u32) -> Result<()> {
    if ticks < 36 {
        return Err("need at least 36 ticks to exercise loaded owner changes".into());
    }
    let dir = tempfile::Builder::new()
        .prefix("gz-material-cuts-")
        .tempdir_in(scratch)?;
    let mut local = setup(&fs::read_to_string(sdf)?, None)?;
    let mut proc = child(sdf, worker, dir.path(), None)?;
    let initial = proc.read()?;
    let groups = initial["owners"].as_array().ok_or("missing owners")?.len();
    if groups < 2 {
        return Err("require independent processes".into());
    }
    let mut owners = vec![0; groups];
    let mut comparisons = 1;
    equal(&initial, &json!({"owners":owners,"state":state(&local)?}))?;
    let (mut resets, mut moves, mut crashes, mut discarded, mut deaths, mut loaded) =
        (0, 0, 0, 0, vec![0; groups], 0);
    let mut generations = std::collections::BTreeSet::new();
    let mut history = Vec::new();
    for epoch in 0..2 {
        let mut digest = Sha256::new();
        if epoch > 0 {
            reset_loaded_world(&mut local, |_| ())?;
            owners.fill(0);
            equal(
                &proc.call("reset")?,
                &json!({"owners":owners,"state":state(&local)?}),
            )?;
            resets += 1;
            comparisons += 1;
        }
        for t in 0..ticks {
            proc.call("prepare")?;
            prepare(&mut local)?;
            let should_save = [0, 19, 32, 49, 87, 199, 410, 421].contains(&t) || t % 137 == 51;
            let mut expected_restarts = vec![false; groups];
            if [9, 16, 24, 33, 46, 60, 195, 408].contains(&t) || t % 79 == 41 {
                let g = moves % groups;
                owners[g] = 1 - owners[g];
                equal(&proc.call(&format!("move {g}"))?, &json!(owners))?;
                moves += 1;
            } else if !should_save
                && ([12, 21, 28, 40, 61, 71, 198, 411].contains(&t) || t % 81 == 26)
            {
                let victims = if t % 81 == 26 {
                    (0..groups).collect()
                } else {
                    vec![(t as usize / 3) % groups]
                };
                for g in victims {
                    proc.call(&format!("kill {g}"))?;
                    deaths[g] += 1;
                    expected_restarts[g] = true;
                }
            }
            let mut replay = None;
            if should_save {
                let f = dir.path().join(format!("cut-{epoch}-{t}"));
                let checkpoint = proc.call(&format!("save {}", f.display()))?;
                equal(
                    &checkpoint,
                    &json!({"owners":owners,"state":state(&local)?}),
                )?;
                if crashes % 2 == 1 {
                    replay = Some(proc.call("advance")?["state"].clone());
                }
                proc.abort()?;
                proc = child(sdf, worker, dir.path(), Some(&f))?;
                equal(&proc.read()?, &checkpoint)?;
                comparisons += 1;
                crashes += 1;
            }
            let result = proc.call("advance")?;
            equal(&result["restarted"], &json!(expected_restarts))?;
            let pids = result["pids"].as_array().ok_or("missing worker receipt")?;
            let ids = pids
                .iter()
                .map(|x| x.as_u64().unwrap())
                .collect::<std::collections::BTreeSet<_>>();
            if ids.len() != groups {
                return Err("workers shared a process".into());
            }
            generations.extend(ids);
            local.step_exact(1)?;
            let s = state(&local)?;
            equal(&result["state"], &s)?;
            comparisons += 1;
            if let Some(old) = replay {
                equal(&old, &s)?;
                discarded += 1;
                comparisons += 1;
            }
            if s["loads"]
                .as_array()
                .unwrap()
                .iter()
                .all(|x| x.as_f64().unwrap() > 0.)
            {
                loaded += 1;
            }
            digest.update(s["hash"].as_str().unwrap());
        }
        history.push(format!("{:x}", digest.finalize()));
    }
    // Public digest includes epoch and sensor clock. Physical state comparisons use the correct
    // current epoch and include every tick against the independently stepped local simulation.
    println!(
        "{}",
        json!({"groups":groups,"epochs":2,"ticks_per_epoch":ticks,"all_groups_loaded_ticks":loaded,"complete_comparisons":comparisons,
       "group_migrations":moves,"group_deaths":deaths,"controller_sigkills":crashes,"discarded_successors_reproduced":discarded,"actual_worker_generations":generations.len(),
       "loaded_resets":resets,"all_cuts_sha256":history})
    );
    Ok(())
}

/// Run one real child at a different interval. The returned receipt authenticates the altered
/// request. The committed public cut must survive while the other groups finish normally.
pub(super) fn disagreement(sdf: &Path, worker: &Path, scratch: &Path) -> Result<()> {
    let dir = tempfile::Builder::new()
        .prefix("gz-material-disagreement-")
        .tempdir_in(scratch)?;
    let exe = std::env::current_exe()?;
    let status = Command::new(&exe)
        .arg("partition-private-cut")
        .arg(sdf)
        .arg(&exe)
        .arg(dir.path())
        .env("GZ_EXAMPLE_REAL_WORKER", worker.canonicalize()?)
        .env("GZ_EXAMPLE_BREAKPOINTS", dir.path())
        .env("TMPDIR", dir.path())
        .status()?;
    if !status.success() {
        return Err("partition disagreement witness failed".into());
    }
    Ok(())
}
pub(super) fn refused(sdf: &Path, relay: &Path, scratch: &Path) -> Result<()> {
    let text = fs::read_to_string(sdf)?;
    let mut committed = setup(&text, Some(relay))?;
    let mut control = setup(&text, None)?;
    committed.step_exact(49)?;
    control.step_exact(49)?;
    let cut = scratch.join("positive-load");
    checkpoint::save(&committed, &cut)?;
    let reference = state(&control)?;
    equal(&state(&committed)?, &reference)?;
    if !reference["loads"]
        .as_array()
        .unwrap()
        .iter()
        .all(|x| x.as_f64().unwrap() > 0.)
    {
        return Err("expect all groups loaded".into());
    }
    let groups = owners(&committed).len();
    let mut seen = Vec::new();
    let mut packed_events = Vec::new();
    let mut checks = 0;
    for g in 0..groups {
        for mode in ["interval", "digest", "length", "loss"] {
            let mut a = checkpoint::open(&cut, relay)?;
            a.step_exact(1)?;
            let before = state(&a)?;
            let i = a.committed_update_info()?.iteration;
            let rows = a
                .world()
                .resource::<TrajectoryProcessPool>()
                .last_material_steps();
            let packed = rows[g].successor_payload_bytes < rows[g].successor_bytes;
            if mode != "interval" && !packed {
                continue;
            }
            let (file, marker, expected) = match mode {
                "interval" => (
                    "different-interval",
                    format!("{} {}", rows[g].pid, i + 1),
                    "worker receipt mismatch",
                ),
                "digest" => (
                    "packed-response",
                    format!("{} {}:digest", rows[g].pid, i + 1),
                    "packed native graph length or digest differs",
                ),
                "length" => (
                    "packed-response",
                    format!("{} {}:length", rows[g].pid, i + 1),
                    "invalid packed native graph dimensions",
                ),
                _ => (
                    "packed-response",
                    format!("{} {}:loss", rows[g].pid, i + 1),
                    "",
                ),
            };
            let mark = scratch.join(file);
            fs::write(&mark, marker)?;
            if mode == "loss" {
                a.step_exact(1)?;
                fs::remove_file(mark)?;
                let proof = scratch.join("solved-packed-loss");
                let event: Value = serde_json::from_slice(&fs::read(&proof)?)?;
                fs::remove_file(proof)?;
                let recovered = a
                    .world()
                    .resource::<TrajectoryProcessPool>()
                    .last_material_steps();
                if event["tick"] != i + 1
                    || event["relay"] != rows[g].pid
                    || recovered
                        .iter()
                        .enumerate()
                        .any(|(n, r)| r.restarted != (g == n))
                {
                    return Err("packed loss recovered a different group or tick".into());
                }
                let mut serial = setup(&text, None)?;
                serial.step_exact(i + 1)?;
                equal(&state(&a)?, &state(&serial)?)?;
                checks += 1;
                packed_events
                    .push(json!({"case":mode,"group":g,"event":event,"new_pid":recovered[g].pid}));
                continue;
            }
            let rejected = a.step_exact(1).is_err();
            fs::remove_file(mark)?;
            let detail = a
                .world()
                .resource::<gz_sim_core::StepFault>()
                .error()
                .ok_or("missing failure on the Bevy clock")?
                .to_string();
            if !rejected || !a.is_faulted() || !detail.contains(expected) {
                return Err(
                    format!("failed to detect independent native disagreement: {detail}").into(),
                );
            }
            equal(&state(&a)?, &before)?;
            if a.committed_update_info()?.iteration != i || a.step_exact(1).is_ok() {
                return Err("controller advanced on incompatible private cuts".into());
            }
            // A separate claimant can open the immutable cut and reproduce a valid continuation.
            let mut recovered = checkpoint::open(&cut, relay)?;
            let mut serial = setup(&text, None)?;
            serial.step_exact(49)?;
            for _ in 0..12 {
                recovered.step_exact(1)?;
                serial.step_exact(1)?;
                equal(&state(&recovered)?, &state(&serial)?)?;
                checks += 1;
            }
            let event = json!({"case":mode,"group":g,"child":rows[g].pid,"refused_tick":i+1,"retained_sha256":before["hash"],"error":detail});
            if mode == "interval" {
                seen.push(event);
            } else {
                packed_events.push(event);
            }
        }
    }
    println!(
        "{}",
        json!({"wrong_interval_receipts_rejected":seen,"packed_receipt_events":packed_events,"independent_recovered_cuts":checks,"loaded_source":reference["loads"]})
    );
    Ok(())
}
