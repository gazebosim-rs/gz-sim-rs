// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Extract the failed barrier and its retries from the coordinator and all native workers.
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::PathBuf,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// One episode directory containing a real worker-suspension observation.
    episode: PathBuf,
    /// Full barrier traces recorded with GZ_SIM_BARRIER_TRACE_DIR.
    trace_dir: PathBuf,
    /// Write only matching coordinator/worker source rows to a fresh JSONL file.
    retained: PathBuf,
}
#[derive(Deserialize)]
struct Row {
    tick: u64,
    events: Vec<Event>,
}
#[derive(Deserialize)]
struct Event {
    name: String,
    ns: u64,
    world: Option<u64>,
}
fn stamp(events: &[Event], key: &str) -> Result<u64> {
    events
        .iter()
        .find(|e| e.name == key)
        .map(|e| e.ns)
        .ok_or_else(|| format!("missing {key}").into())
}
fn elapsed_us(start: u64, end: u64) -> Result<f64> {
    Ok(end.checked_sub(start).ok_or("nonmonotonic span")? as f64 / 1e3)
}
fn main() -> Result<()> {
    let args = Args::parse();
    let observation: Value =
        serde_json::from_slice(&fs::read(args.episode.join("episodes.jsonl"))?)?;
    let tick = observation["tick"]
        .as_u64()
        .ok_or("episode has no target tick")?;
    let mut peers = BTreeMap::<String, Vec<Value>>::new();
    let mut physical_batches = 0;
    let mut attempts = vec![];
    let mut retained = BufWriter::new(File::create_new(args.retained)?);
    let mut files = fs::read_dir(args.trace_dir)?.collect::<std::io::Result<Vec<_>>>()?;
    files.sort_by_key(|e| e.file_name());
    for entry in files {
        let name = entry
            .file_name()
            .into_string()
            .map_err(|_| "invalid trace name")?;
        if !name.ends_with(".jsonl") {
            continue;
        }
        for line in BufReader::new(File::open(entry.path())?).lines() {
            let line = line?;
            let row: Row = serde_json::from_str(&line)?;
            if row.tick != tick {
                continue;
            }
            let e = &row.events;
            let raw: Value = serde_json::from_str(&line)?;
            writeln!(retained, "{}", json!({"source":name,"row":raw}))?;
            if name.starts_with("coord-") {
                let start = stamp(e, "api_start")?;
                let end = stamp(e, "api_end")?;
                let dispatch = e
                    .iter()
                    .find(|ev| ev.name == "dispatch_start")
                    .ok_or("missing physical dispatch")?
                    .ns;
                let probe = if e.iter().any(|ev| ev.name == "placement_open_start") {
                    Some(("placement_open_start", "placement_open_end"))
                } else if e.iter().any(|ev| ev.name == "placement_read_start") {
                    Some(("placement_read_start", "placement_read_end"))
                } else {
                    None
                };
                let probe_us = probe
                    .map(|(a, b)| elapsed_us(stamp(e, a)?, stamp(e, b)?))
                    .transpose()?;
                let retry_span = |a: &str, b: &str| -> Result<Option<f64>> {
                    e.iter()
                        .any(|ev| ev.name == a)
                        .then(|| elapsed_us(stamp(e, a)?, stamp(e, b)?))
                        .transpose()
                };
                attempts.push((
                    start,
                    json!({"api_us":elapsed_us(start,end)?,"advisory_us":probe_us,
                    "confirmation_us":retry_span("step_confirm_start","step_confirm_end")?,
                    "reconstructed_prefix_and_step_us":retry_span("step_rebuild_start","step_rebuild_end")?,
                    "physical_dispatch_to_api_end_us":elapsed_us(dispatch,end)?}),
                ));
            } else if name.starts_with("worker-") {
                let mut worlds = BTreeMap::new();
                let mut native_us = 0.;
                for ev in e {
                    if ev.name == "native_start" {
                        if worlds
                            .insert(ev.world.ok_or("missing world")?, ev.ns)
                            .is_some()
                        {
                            return Err("overlapping solve in the same world".into());
                        }
                    } else if ev.name == "native_end" {
                        let start = worlds
                            .remove(&ev.world.ok_or("missing world")?)
                            .ok_or("missing native start")?;
                        native_us += elapsed_us(start, ev.ns)?;
                    }
                }
                let batch = e.iter().any(|ev| ev.name == "batch_start");
                physical_batches += usize::from(batch);
                let spans = json!({
                    "executed_batch":batch,
                    "handler_us":elapsed_us(stamp(e,"handle_start")?,stamp(e,"handle_end")?)?,
                    "summed_native_us":native_us
                });
                if !worlds.is_empty() {
                    return Err("unfinished native world".into());
                }
                peers.entry(name.clone()).or_default().push(spans);
            } else {
                return Err("unexpected trace source".into());
            }
        }
    }
    attempts.sort_by_key(|(start, _)| *start);
    let expected = if observation["first_step_retry"] == true {
        2
    } else {
        1
    };
    if attempts.len() != expected || peers.len() != 4 || physical_batches != 4 {
        return Err("incomplete coordinator or worker trace".into());
    }
    retained.flush()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"observation":observation,"attempts":attempts.into_iter().map(|(_,a)|a).collect::<Vec<_>>(),"workers":peers})
        )?
    );
    Ok(())
}
