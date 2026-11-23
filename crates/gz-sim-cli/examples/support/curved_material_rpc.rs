// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise pipe and loopback Zenoh on the same public material graph. Inspect only sockets
//! belonging to the current controller and its actual selected solver children.
use super::*;
use std::time::Instant;
pub(super) fn authored(sdf: String, transport: &str) -> Result<String> {
    match transport {
        "pipe" => Ok(sdf),
        "loopback_zenoh" => Ok(sdf.replace(
            "filename=\"gz-sim-deformables-system\"",
            "filename=\"gz-sim-deformables-system\" worker_transport='loopback_zenoh'",
        )),
        _ => Err("transport must be pipe or loopback_zenoh".into()),
    }
}
#[cfg(target_os = "linux")]
fn links(pid: u32) -> Result<std::collections::BTreeSet<String>> {
    let mut set = std::collections::BTreeSet::new();
    for entry in fs::read_dir(format!("/proc/{pid}/fd"))? {
        if let Ok(link) = fs::read_link(entry?.path())
            && let Some(s) = link
                .to_str()
                .and_then(|s| s.strip_prefix("socket:["))
                .and_then(|s| s.strip_suffix(']'))
        {
            set.insert(s.to_owned());
        }
    }
    Ok(set)
}
#[cfg(target_os = "linux")]
pub(super) fn sockets(sim: &Simulation) -> Result<Value> {
    let ctl = links(std::process::id())?;
    let p = sim.world().resource::<TrajectoryProcessPool>();
    let owners = p.material_owners();
    let sockets = fs::read_to_string("/proc/self/net/tcp")?;
    let connected: Vec<_> = sockets
        .lines()
        .skip(1)
        .filter_map(|row| {
            let f = row.split_ascii_whitespace().collect::<Vec<_>>();
            (f.len() > 9
                && f[3] == "01"
                && f[1].starts_with("0100007F:")
                && f[2].starts_with("0100007F:"))
            .then(|| (f[1], f[2], f[9]))
        })
        .collect();
    let mut result = Vec::new();
    for (g, slot) in owners.into_iter().enumerate() {
        let pid = p.group_pid(g, slot).ok_or("missing live leased solver")?;
        let own = links(pid)?;
        let direct = connected
            .iter()
            .filter(|&&(a, b, inode)| {
                own.contains(inode)
                    && connected
                        .iter()
                        .any(|&(c, d, peer)| a == d && b == c && ctl.contains(peer))
            })
            .count();
        result.push(json!({"group":g,"pid":pid,"established_controller_tcp_connections":direct}));
    }
    Ok(json!(result))
}
#[cfg(not(target_os = "linux"))]
pub(super) fn sockets(_: &Simulation) -> Result<Value> {
    Err("this socket witness uses Linux procfs".into())
}
#[cfg(unix)]
pub(super) fn suspend(pid: u32) -> Result<()> {
    use rustix::process::{Pid, Signal, kill_process};
    kill_process(
        Pid::from_raw(i32::try_from(pid)?).ok_or("invalid child")?,
        Signal::STOP,
    )?;
    Ok(())
}
#[cfg(not(unix))]
pub(super) fn suspend(_: u32) -> Result<()> {
    Err("this lease stall witness needs POSIX".into())
}
#[cfg(target_os = "linux")]
pub(super) fn brief_delay(pid: u32, ms: u64) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let fd = pidfd_open(
        Pid::from_raw(pid as i32).ok_or("invalid leased worker")?,
        PidfdFlags::empty(),
    )?;
    pidfd_send_signal(&fd, Signal::STOP)?;
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(ms));
        let _ = pidfd_send_signal(&fd, Signal::CONT);
    });
    Ok(())
}
#[cfg(not(target_os = "linux"))]
pub(super) fn brief_delay(_: u32, _: u64) -> Result<()> {
    Err("timed physical stall checks use Linux pidfds".into())
}
pub(super) fn run(
    worker: &Path,
    sdf: &Path,
    scratch: &Path,
    ticks: u32,
    stalls: bool,
) -> Result<()> {
    if ticks < 100 {
        return Err("need 100 ticks for the two-transport scenario".into());
    }
    let dir = tempfile::Builder::new()
        .prefix("gz-material-network-")
        .tempdir_in(scratch)?;
    let mut sim = setup(&fs::read_to_string(sdf)?, None)?;
    let mut actor = partitions::child(sdf, worker, dir.path(), None)?;
    let first = actor.read()?;
    let initial: Vec<String> = serde_json::from_value(actor.call("routes")?)?;
    let groups = initial.len();
    if groups < 2 {
        return Err("two or more process groups are required".into());
    }
    let mut routes = initial.clone();
    let mut placement = vec![0; groups];
    equal(&first["state"], &partitions::state(&sim)?)?;
    let (
        mut crashes,
        mut repeated,
        mut moves,
        mut changes,
        mut wrong_cut,
        mut checks,
        mut all_loaded,
    ) = (0, 0, 0, 0, 0, 1, 0);
    let mut deaths = vec![0; groups];
    let mut holds = Vec::new();
    let mut unplanned = Vec::new();
    let mut sockets_seen = Vec::new();
    let mut history = Vec::new();
    for epoch in 0..2 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ())?;
            let r = actor.call("reset")?;
            placement.fill(0);
            routes = initial.clone();
            equal(&r["state"], &partitions::state(&sim)?)?;
            equal(&actor.call("routes")?, &json!(routes))?;
            checks += 1;
        }
        let mut digest = Sha256::new();
        for t in 0..ticks {
            actor.call("prepare")?;
            prepare(&mut sim)?;
            if [6, 18, 28, 46, 68, 78, 91, 410].contains(&t) {
                let g = (changes / 2 + changes % 2) % groups;
                routes[g] = if routes[g] == "pipe" {
                    "loopback_zenoh"
                } else {
                    "pipe"
                }
                .to_owned();
                equal(
                    &actor.call(&format!("transport {g} {}", routes[g]))?,
                    &json!(routes),
                )?;
                changes += 1;
            }
            if [15, 23, 44, 62, 85, 409].contains(&t) {
                let g = moves % groups;
                placement[g] = 1 - placement[g];
                equal(&actor.call(&format!("move {g}"))?, &json!(placement))?;
                moves += 1;
            }
            if t % 39 == 10 {
                equal(&actor.call("stale-route")?, &json!(true))?;
                wrong_cut += 1;
            }
            let save = [0, 18, 41, 63, 78, 93, 410].contains(&t);
            let mut discarded = None;
            if save {
                let cut = dir.path().join(format!("saved-{epoch}-{t}"));
                let published = actor.call(&format!("save {}", cut.display()))?;
                equal(&published["state"], &partitions::state(&sim)?)?;
                if crashes % 2 == 1 {
                    discarded = Some(actor.call("advance")?["state"].clone());
                }
                actor.abort()?;
                actor = partitions::child(sdf, worker, dir.path(), Some(&cut))?;
                equal(&actor.read()?, &published)?;
                equal(&actor.call("routes")?, &json!(routes))?;
                checks += 1;
                crashes += 1;
            }
            let mut want = vec![false; groups];
            if [13, 27, 35, 48, 76, 88, 411].contains(&t) {
                let g = t as usize % groups;
                actor.call(&format!("kill {g}"))?;
                deaths[g] += 1;
                want[g] = true;
            }
            let stalled = stalls && epoch == 0 && [37, 57].contains(&t);
            let mut stalled_route = None;
            if stalled {
                let route = if t == 37 { "pipe" } else { "loopback_zenoh" };
                let g = routes
                    .iter()
                    .position(|s| s == route)
                    .ok_or("required route missing at stalled cut")?;
                let pid = actor.call(&format!("stall {g}"))?;
                want[g] = true;
                stalled_route = Some((g, route, pid));
            }
            let started = Instant::now();
            let reply = actor.call("advance")?;
            let elapsed = started.elapsed();
            if let Some((group, route, pid)) = stalled_route {
                holds.push(json!({"group":group,"transport":route,"old_pid":pid,"wall_ms":elapsed.as_millis(),"armed_us":reply["warm_deadline_us"][group],"short_timeout":reply["warm_timeouts"][group],"next_pid":reply["pids"][group]}));
            }
            let actual: Vec<bool> = serde_json::from_value(reply["restarted"].clone())?;
            if actual.len() != groups {
                return Err("missing group recovery receipts".into());
            }
            for (g, (&seen, &expected)) in actual.iter().zip(&want).enumerate() {
                if seen == expected {
                    continue;
                }
                if seen && reply["warm_timeouts"][g].as_bool() == Some(true) {
                    unplanned.push(json!({"epoch":epoch,"tick":t+1,"group":g,"transport":routes[g],"budget_us":reply["warm_deadline_us"][g],"new_pid":reply["pids"][g],"wall_us":elapsed.as_micros()}));
                } else {
                    return Err(format!("unexpected recovery at epoch {epoch} step {t} group {g}: recovered={seen} intended={expected}").into());
                }
            }
            sim.step_exact(1)?;
            let state = partitions::state(&sim)?;
            equal(&reply["state"], &state)?;
            checks += 1;
            if let Some(old) = discarded {
                equal(&state, &old)?;
                repeated += 1;
                checks += 1;
            }
            if state["loads"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| v.as_f64().unwrap() > 0.)
            {
                all_loaded += 1;
            }
            digest.update(state["hash"].as_str().unwrap());
            if [1, 7, 19, 30, 39, 49, 58, 69, 79, 94].contains(&t) {
                let observed = actor.call("tcp")?;
                for (g, route) in routes.iter().enumerate() {
                    let n = observed[g]["established_controller_tcp_connections"]
                        .as_u64()
                        .ok_or("no socket evidence")?;
                    if (n != 0) != (route == "loopback_zenoh") {
                        return Err(format!(
                            "socket transport mismatch at {t} group {g}: {observed}"
                        )
                        .into());
                    }
                }
                sockets_seen
                    .push(json!({"epoch":epoch,"tick":t+1,"routes":routes,"connections":observed}));
            }
        }
        history.push(format!("{:x}", digest.finalize()));
    }
    println!(
        "{}",
        json!({"groups":groups,"ticks_per_epoch":ticks,"complete_comparisons":checks,"worker_sigkills":deaths,"process_stalls":holds,"unplanned_budget_retries":unplanned,
        "controller_sigkills":crashes,"discarded_successors":repeated,"group_migrations":moves,"transport_changes":changes,"rejected_stale_changes":wrong_cut,
        "all_materials_loaded_ticks":all_loaded,"verified_tcp_cuts":sockets_seen,"cut_sha256":history})
    );
    Ok(())
}
