// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Scheduler accounting for exact contact-forming ticks in a local four-worker trial.
use super::{Event, Result, Row, Tick, duration, stamp};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs, path::Path};

#[derive(Deserialize, Serialize)]
pub(super) struct Stat {
    pub(super) tid: u32,
    pub(super) runtime_ns: u64,
    pub(super) runqueue_ns: u64,
    slices: u64,
}

fn event<'a>(row: &'a Row, name: &str, peer: Option<u8>) -> Result<&'a Event> {
    let mut hits = row
        .events
        .iter()
        .filter(|e| e.name == name && e.peer == peer);
    let first = hits
        .next()
        .ok_or_else(|| format!("missing {name}/{peer:?} on tick {}", row.tick))?;
    if hits.next().is_some() {
        return Err(format!("duplicate {name}/{peer:?} on tick {}", row.tick).into());
    }
    Ok(first)
}
fn runqueue(a: &Event, b: &Event) -> Option<u64> {
    let (a, b) = (a.sched.as_ref()?, b.sched.as_ref()?);
    (a.tid == b.tid).then_some(())?;
    b.runqueue_ns.checked_sub(a.runqueue_ns)
}

#[derive(Serialize)]
pub(super) struct Observation {
    run: usize,
    tick: u64,
    worker: u8,
    lane: &'static str,
    gate_worker: u8,
    public_us: f64,
    request_to_helper_submit_us: f64,
    batch_start_to_helper_submit_us: f64,
    helper_dispatch_us: f64,
    helper_runqueue_raw_us: Option<f64>,
    helper_runqueue_us: Option<f64>,
    helper_residual_us: Option<f64>,
    helper_finishes_last: bool,
    gate_reply_submit_to_ready_us: f64,
    coordinator_fifo_us: f64,
    coordinator_fifo_cpu_us: f64,
    coordinator_fifo_runqueue_us: Option<f64>,
    coordinator_fifo_residual_us: Option<f64>,
    coordinator_fifo_over_accounted_intervals: usize,
    coordinator_fifo_intervals: Vec<Value>,
    helper_samples: [Value; 2],
}
fn us(ns: u64) -> f64 {
    ns as f64 / 1000.
}
fn sample(e: &Event) -> Value {
    json!({"ns":e.ns,"cpu_ns":e.cpu_ns,"sched":e.sched})
}
pub(super) fn observe(
    t: &Tick,
    coord: &Row,
    workers: &[BTreeMap<u64, Row>],
) -> Result<Observation> {
    let owner = &workers[0][&t.tick];
    let a = event(owner, "helper_submit", None)?;
    let b = event(owner, "helper_entry", None)?;
    let dispatch = b.ns.checked_sub(a.ns).ok_or("helper clock order")?;
    let raw = runqueue(a, b);
    let valid = raw.filter(|d| *d <= dispatch);
    let mut wall = 0;
    let mut cpu = 0;
    let mut queue = Some(0u64);
    let mut intervals = Vec::new();
    let mut over_accounted = 0;
    for id in 0..4 {
        let a = event(coord, "collect_start", Some(id))?;
        let b = event(coord, "received", Some(id))?;
        let wall_ns = b.ns.checked_sub(a.ns).ok_or("coordinator clock order")?;
        let cpu_ns = b
            .cpu_ns
            .and_then(|end| end.checked_sub(a.cpu_ns?))
            .ok_or("missing/reversed coordinator CPU clock")?;
        wall += wall_ns;
        cpu += cpu_ns;
        let rq_ns = runqueue(a, b);
        queue = queue.zip(rq_ns).and_then(|(sum, v)| sum.checked_add(v));
        if rq_ns.is_some_and(|rq| cpu_ns > wall_ns || rq > wall_ns - cpu_ns) {
            over_accounted += 1;
        }
        intervals.push(json!({"peer":id,"start":sample(a),"end":sample(b),"wall_us":us(wall_ns),"cpu_us":us(cpu_ns),"runqueue_raw_us":rq_ns.map(us)}));
    }
    let gate = &workers[usize::from(t.gate_worker)][&t.tick];
    let last = owner
        .events
        .iter()
        .filter(|e| e.name == "native_end")
        .max_by_key(|e| e.ns)
        .ok_or("no native world")?;
    let valid_queue = queue.filter(|rq| cpu <= wall && *rq <= wall - cpu);
    Ok(Observation {
        run: t.run,
        tick: t.tick,
        worker: 0,
        lane: "helper",
        gate_worker: t.gate_worker,
        public_us: t.total_us,
        request_to_helper_submit_us: duration(stamp(owner, "request", None)?, a.ns)?,
        batch_start_to_helper_submit_us: duration(stamp(owner, "batch_start", None)?, a.ns)?,
        helper_dispatch_us: us(dispatch),
        helper_runqueue_raw_us: raw.map(us),
        helper_runqueue_us: valid.map(us),
        helper_residual_us: valid.map(|q| us(dispatch - q)),
        helper_finishes_last: last.helper.ok_or("missing lane")?,
        gate_reply_submit_to_ready_us: duration(
            stamp(gate, "reply_submit", None)?,
            stamp(coord, "ready", Some(t.gate_worker))?,
        )?,
        coordinator_fifo_us: us(wall),
        coordinator_fifo_cpu_us: us(cpu),
        coordinator_fifo_runqueue_us: valid_queue.map(us),
        coordinator_fifo_residual_us: valid_queue.map(|rq| us(wall - cpu - rq)),
        coordinator_fifo_over_accounted_intervals: over_accounted,
        coordinator_fifo_intervals: intervals,
        helper_samples: [sample(a), sample(b)],
    })
}
fn quantiles(rows: &[Observation], get: impl Fn(&Observation) -> Option<f64>) -> Value {
    let mut vals: Vec<_> = rows.iter().filter_map(get).collect();
    vals.sort_by(f64::total_cmp);
    let percentile = |p: usize| {
        vals.get((vals.len() * p).div_ceil(100).saturating_sub(1))
            .copied()
    };
    json!({"valid":vals.len(),"median":percentile(50),"p95":percentile(95),"max":vals.last()})
}
fn summary(rows: &[Observation]) -> Value {
    json!({
        "n": rows.len(),
        "helper_dispatch_us":quantiles(rows,|r|Some(r.helper_dispatch_us)),
        "helper_runqueue_us":quantiles(rows,|r|r.helper_runqueue_us),
        "helper_residual_us":quantiles(rows,|r|r.helper_residual_us),
        "request_to_helper_submit_us":quantiles(rows,|r|Some(r.request_to_helper_submit_us)),
        "batch_start_to_helper_submit_us":quantiles(rows,|r|Some(r.batch_start_to_helper_submit_us)),
        "helper_runqueue_over_half":rows.iter().filter(|r|r.helper_runqueue_us.is_some_and(|q|q>r.helper_dispatch_us/2.)).count(),
        "helper_finishes_last":rows.iter().filter(|r|r.helper_finishes_last).count(),
        "populated_worker_gates":rows.iter().filter(|r|r.gate_worker==0).count(),
        "gate_reply_submit_to_ready_us":quantiles(rows,|r|Some(r.gate_reply_submit_to_ready_us)),
        "coordinator_fifo_us":quantiles(rows,|r|Some(r.coordinator_fifo_us)),
        "coordinator_fifo_cpu_us":quantiles(rows,|r|Some(r.coordinator_fifo_cpu_us)),
        "coordinator_fifo_runqueue_us":quantiles(rows,|r|r.coordinator_fifo_runqueue_us),
        "coordinator_fifo_residual_us":quantiles(rows,|r|r.coordinator_fifo_residual_us),
    })
}

pub(super) fn write(out: &Path, layouts: &mut BTreeMap<&str, Vec<Observation>>) -> Result<()> {
    let mut report = BTreeMap::new();
    let mut worst = BTreeMap::new();
    for (&name, rows) in layouts {
        rows.sort_by(|l, r| r.public_us.total_cmp(&l.public_us));
        worst.insert(
            name,
            serde_json::to_value(rows.first().ok_or("no exact landing ticks")?)?,
        );
        report.insert(name,json!({"all_landings":summary(rows),"slowest_decile":summary(&rows[..rows.len().div_ceil(10)]),"observations":rows}));
    }
    for (file, data) in [("wake-worst.json", &worst), ("wake-summary.json", &report)] {
        fs::write(out.join(file), serde_json::to_vec_pretty(data)?)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn marker(tid: u32, runqueue: u64) -> Event {
        serde_json::from_value(json!({"name":"helper_entry","ns":1000,
            "sched":{"tid":tid,"runtime_ns":100,"runqueue_ns":runqueue,"slices":4}}))
        .unwrap()
    }

    #[test]
    fn scheduler_counter_requires_same_thread_and_monotonic_samples() {
        let start = marker(13, 20);
        assert_eq!(runqueue(&start, &marker(13, 20)), Some(0));
        assert_eq!(runqueue(&start, &marker(13, 80)), Some(60));
        assert_eq!(runqueue(&start, &marker(20, 80)), None);
        assert_eq!(runqueue(&start, &marker(13, 19)), None);
        let mut missing = marker(13, 20);
        missing.sched = None;
        assert_eq!(runqueue(&missing, &start), None);
    }
}
