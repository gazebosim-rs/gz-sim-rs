// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Attribute the interval between the last world finishing and the owner leaving its batch.
use super::{Event, Result, Row, Tick, duration, nearest_rank};
use serde::Serialize;
use serde_json::json;
use std::{collections::BTreeMap, fs, path::Path};

#[derive(Serialize)]
struct Interval {
    wall_us: f64,
    cpu_us: Option<f64>,
    cpu_source: Option<&'static str>,
    runqueue_raw_us: Option<f64>,
    residual_us: Option<f64>,
}
fn us(n: u64) -> f64 {
    n as f64 / 1000.
}
// All lane_done sched snapshots target the caller, including remotely-read samples.
// CPU-clock deltas are available only when both endpoints ran on the caller.
fn caller_interval(a: &Event, b: &Event) -> Result<Interval> {
    let wall = b.ns.checked_sub(a.ns).ok_or("reversed caller interval")?;
    let stats = a.sched.as_ref().zip(b.sched.as_ref());
    if stats.is_some_and(|(x, y)| x.tid != y.tid) {
        return Err("caller TID changed within the batch".into());
    }
    let rq = stats.and_then(|(x, y)| y.runqueue_ns.checked_sub(x.runqueue_ns));
    let (cpu, source) = if let Some((x, y)) = a.cpu_ns.zip(b.cpu_ns) {
        (
            Some(y.checked_sub(x).ok_or("reversed caller CPU clock")?),
            Some("thread_cpu_clock"),
        )
    } else {
        let ns = stats.and_then(|(x, y)| y.runtime_ns.checked_sub(x.runtime_ns));
        (ns, ns.map(|_| "schedstat_runtime"))
    };
    // Charges can straddle the sampled endpoints. Preserve them and withhold
    // the derived remainder if they do not fit the measured wall interval.
    let residual = cpu
        .zip(rq)
        .and_then(|(c, q)| wall.checked_sub(c)?.checked_sub(q));
    Ok(Interval {
        wall_us: us(wall),
        cpu_us: cpu.map(us),
        cpu_source: source,
        runqueue_raw_us: rq.map(us),
        residual_us: residual.map(us),
    })
}

#[derive(Serialize)]
struct Span {
    name: &'static str,
    us: f64,
}

#[derive(Serialize)]
pub(super) struct Observation {
    #[serde(skip)]
    class: &'static str,
    run: usize,
    tick: u64,
    worker: u8,
    gate_worker: u8,
    public_us: f64,
    join_us: f64,
    spans: [Span; 3],
    helper_task_finished_last: bool,
    caller_native_wall_us: f64,
    caller_native_cpu_us: f64,
    caller_task_done_to_scope_return: Interval,
    both_tasks_done_to_scope_return: Interval,
    batch_start_to_caller_task_done: Interval,
}

pub(super) fn observe(t: &Tick, owner: &Row, class: &'static str) -> Result<Observation> {
    let event = |name, helper| -> Result<&Event> {
        let mut matches = owner
            .events
            .iter()
            .filter(|e| e.name == name && e.helper == Some(helper));
        let marker = matches
            .next()
            .ok_or_else(|| format!("missing {name}/{helper}"))?;
        if matches.next().is_some() {
            return Err(format!("duplicate {name}/{helper}").into());
        }
        Ok(marker)
    };
    let batch = event("batch_start", false)?;
    let caller = event("lane_done", false)?;
    let helper = event("lane_done", true)?;
    let returned = event("scope_return", false)?;
    let end = event("batch_end", false)?;
    let last_task = if caller.ns >= helper.ns {
        caller
    } else {
        helper
    };
    let mut last_world = 0;
    let mut caller_wall = 0;
    let mut caller_cpu = 0;
    for (lane, done) in [(false, caller), (true, helper)] {
        let mut count = 0;
        for end in owner
            .events
            .iter()
            .filter(|e| e.name == "native_end" && e.helper == Some(lane))
        {
            let start = owner
                .events
                .iter()
                .find(|e| {
                    e.name == "native_start" && e.world == end.world && e.helper == end.helper
                })
                .ok_or("missing matching world start")?;
            if ![batch.ns, start.ns, end.ns, done.ns, returned.ns].is_sorted() {
                return Err("solve/task/scope exit boundaries reversed".into());
            }
            if !lane {
                caller_wall += end.ns - start.ns;
                caller_cpu += end
                    .cpu_ns
                    .and_then(|x| x.checked_sub(start.cpu_ns?))
                    .ok_or("missing caller solve CPU")?;
            }
            count += 1;
            last_world = last_world.max(end.ns);
        }
        if count == 0 {
            return Err("missing solves on measured lane".into());
        }
    }
    Ok(Observation {
        class,
        run: t.run,
        tick: t.tick,
        worker: 0,
        gate_worker: t.gate_worker,
        public_us: t.total_us,
        join_us: duration(last_world, end.ns)?,
        spans: [
            Span {
                name: "Last world to lane completion",
                us: duration(last_world, last_task.ns)?,
            },
            Span {
                name: "Lanes complete to scope return",
                us: duration(last_task.ns, returned.ns)?,
            },
            Span {
                name: "Scope return to batch end",
                us: duration(returned.ns, end.ns)?,
            },
        ],
        helper_task_finished_last: helper.ns > caller.ns,
        caller_native_wall_us: us(caller_wall),
        caller_native_cpu_us: us(caller_cpu),
        caller_task_done_to_scope_return: caller_interval(caller, returned)?,
        both_tasks_done_to_scope_return: caller_interval(last_task, returned)?,
        batch_start_to_caller_task_done: caller_interval(batch, caller)?,
    })
}

pub(super) fn write(out: &Path, layouts: &mut BTreeMap<&str, Vec<Observation>>) -> Result<()> {
    let mut worst = BTreeMap::new();
    let mut public_worst = BTreeMap::new();
    let mut summaries = BTreeMap::new();
    let mut all_ticks = BTreeMap::new();
    for (&layout, rows) in layouts {
        let max_tasks = rows
            .iter()
            .max_by(|a, b| {
                a.both_tasks_done_to_scope_return
                    .wall_us
                    .total_cmp(&b.both_tasks_done_to_scope_return.wall_us)
            })
            .ok_or("no timed ticks")?;
        let max_solve = rows
            .iter()
            .max_by(|a, b| a.join_us.total_cmp(&b.join_us))
            .unwrap();
        let stages: Vec<_> = rows
            .iter()
            .map(|r| r.both_tasks_done_to_scope_return.wall_us)
            .collect();
        let tagged = |r: &Observation| json!({"class":r.class,"observation":r});
        all_ticks.insert(layout,json!({"ticks":rows.len(),"post_solve_over_100_us":rows.iter().filter(|r|r.join_us>100.).count(),
            "post_tasks_over_100_us":stages.iter().filter(|us|**us>100.).count(),"post_tasks_p95_us":nearest_rank(&stages,95),
            "largest_post_solve":tagged(max_solve),"largest_post_tasks":tagged(max_tasks)}));
        let mut rows: Vec<_> = rows.iter().filter(|r| r.class == "landing").collect();
        rows.sort_by(|a, b| b.join_us.total_cmp(&a.join_us));
        let slowest = rows.first().ok_or("no touchdowns")?;
        let public = rows
            .iter()
            .max_by(|a, b| a.public_us.total_cmp(&b.public_us))
            .unwrap();
        let summary = |f: fn(&Observation) -> f64| {
            let xs: Vec<_> = rows.iter().copied().map(f).collect();
            json!({"median":nearest_rank(&xs,50),"p95":nearest_rank(&xs,95),"max":nearest_rank(&xs,100)})
        };
        summaries.insert(layout,json!({"landings":rows.len(),"post_solve_us":summary(|r|r.join_us),
        "post_tasks_us":summary(|r|r.both_tasks_done_to_scope_return.wall_us),
        "complete_post_tasks_accounting":rows.iter().filter(|r|r.both_tasks_done_to_scope_return.residual_us.is_some()).count(),
        "over_100_us":rows.iter().filter(|r|r.join_us>100.).count(),"observations":rows}));
        worst.insert(layout, serde_json::to_value(slowest)?);
        public_worst.insert(layout, serde_json::to_value(public)?);
    }
    for (name, table) in [
        ("join-all-ticks.json", all_ticks),
        ("join-worst.json", worst),
        ("join-public-worst.json", public_worst),
        ("join-summary.json", summaries),
    ] {
        fs::write(out.join(name), serde_json::to_vec_pretty(&table)?)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(ns: u64, cpu: Option<u64>, tid: u32, runtime: u64, queue: u64) -> Event {
        serde_json::from_value(json!({"name":"end","ns":ns,"cpu_ns":cpu,
            "sched":{"tid":tid,"runtime_ns":runtime,"runqueue_ns":queue,"slices":1}}))
        .unwrap()
    }
    #[test]
    fn uses_caller_clock_or_same_tid_runtime_and_preserves_invalid_budgets() -> Result<()> {
        let end = sample(20_000, Some(5_000), 7, 7000, 2400);
        let local = caller_interval(&sample(10_000, Some(2_000), 7, 4700, 700), &end)?;
        assert_eq!(local.cpu_source, Some("thread_cpu_clock"));
        assert_eq!(local.residual_us, Some(5.3));
        let remote = caller_interval(&sample(10_000, None, 7, 4700, 700), &end)?;
        assert_eq!(remote.cpu_source, Some("schedstat_runtime"));
        assert_eq!(remote.residual_us, Some(6.0));
        let late = caller_interval(&sample(19_000, None, 7, 4700, 700), &end)?;
        assert_eq!(late.runqueue_raw_us, Some(1.7));
        assert!(late.residual_us.is_none());
        assert!(caller_interval(&sample(10_000, None, 8, 4700, 700), &end).is_err());
        assert!(caller_interval(&end, &sample(19_000, None, 7, 8000, 3000)).is_err());
        Ok(())
    }
}
