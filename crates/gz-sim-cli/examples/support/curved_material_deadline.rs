// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare an actual paused solver which resumes in time with one whose warm reply expires.
use super::*;
pub(super) fn qualify(sdf: &Path, worker: &Path, scratch: &Path, ticks: u32) -> Result<()> {
    if ticks < 95 {
        return Err("need at least 95 public ticks".into());
    }
    let dir = tempfile::Builder::new()
        .prefix("gz-warm-budget-")
        .tempdir_in(scratch)?;
    let text = fs::read_to_string(sdf)?;
    let mut local = setup(&text, None)?;
    let mut actor = partitions::child(sdf, worker, dir.path(), None)?;
    let first = actor.read()?;
    let groups = first["owners"]
        .as_array()
        .ok_or("missing group placement")?
        .len();
    if groups < 2 {
        return Err("need two distinct group owners".into());
    }
    let mut seen = Vec::new();
    let mut history = Sha256::new();
    let mut previous = Vec::new();
    let mut ordinary_armed = 0;
    for t in 0..ticks {
        if t == 11 {
            actor.call("transport 0 pipe")?;
            actor.call("transport 1 loopback_zenoh")?;
        }
        let selected = match t {
            33 => Some((0, 18)),
            42 => Some((1, 18)),
            57 => Some((0, 700)),
            83 => Some((1, 700)),
            _ => None,
        };
        let begin = std::time::Instant::now();
        let mut prior = None;
        if let Some((g, ms)) = selected {
            prior = Some(actor.call(&format!("delay {g} {ms}"))?);
        }
        let reply = actor.call("advance")?;
        let wall = begin.elapsed();
        local.step_exact(1)?;
        let s = partitions::state(&local)?;
        equal(&s, &reply["state"])?;
        let pids: Vec<u32> = serde_json::from_value(reply["pids"].clone())?;
        let warmed: Vec<Option<u64>> = serde_json::from_value(reply["warm_deadline_us"].clone())?;
        let expired: Vec<bool> = serde_json::from_value(reply["warm_timeouts"].clone())?;
        let reaped: Vec<bool> = serde_json::from_value(reply["restarted"].clone())?;
        for g in 0..groups {
            let injected = selected.filter(|v| v.0 == g);
            if let Some((_, ms)) = injected {
                let budget =
                    warmed[g].ok_or(format!("budget was not armed for group {g} at {t}"))?;
                let expected = ms == 700;
                if expected != (budget < ms * 1000) {
                    return Err(
                        format!("injected delay {ms} inappropriate for budget {budget}").into(),
                    );
                }
                if expired[g] != expected
                    || reaped[g] != expected
                    || (previous[g] != pids[g]) != expected
                {
                    return Err("unexpected warm deadline or process identity".into());
                }
                seen.push(json!({"group":g,"tick":t+1,"paused_ms":ms,"armed_us":budget,"wall_us":wall.as_micros(),
                    "retired":expired[g],"prior_pid":prior,"next_pid":pids[g],"loaded":s["loads"]}));
            } else {
                if expired[g] || reaped[g] {
                    return Err(format!("healthy group {g} was retired on tick {t}").into());
                }
                ordinary_armed += usize::from(warmed[g].is_some());
            }
        }
        history.update(s["hash"].as_str().unwrap());
        previous = pids;
    }
    println!(
        "{}",
        json!({"full_checks":ticks,"process_groups":groups,"real_delays":seen,"healthy_short_budget_observations":ordinary_armed,
        "cut_sha256":format!("{:x}",history.finalize())})
    );
    Ok(())
}
