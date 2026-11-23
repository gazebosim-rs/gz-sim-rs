// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconcile public fleet and native worker lifecycle receipts from concurrent writers.
#[path = "support/endurance/gpu.rs"]
mod gpu;
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufRead, BufReader, Read},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn prefix(path: &Path) -> Result<BufReader<std::io::Take<File>>> {
    let file = File::open(path)?;
    let len = file.metadata()?.len();
    Ok(BufReader::new(file.take(len)))
}

fn records<T: serde::de::DeserializeOwned>(
    path: &Path,
    accept: impl FnMut(T) -> Result<()>,
) -> Result<()> {
    read_records(prefix(path)?, false, accept)
}

fn read_records<T: serde::de::DeserializeOwned>(
    mut reader: impl BufRead,
    completed: bool,
    mut accept: impl FnMut(T) -> Result<()>,
) -> Result<()> {
    let mut line = String::new();
    while reader.read_line(&mut line)? > 0 {
        if !line.ends_with('\n') {
            if completed {
                return Err("incomplete final receipt in completed run".into());
            }
            break;
        }
        accept(serde_json::from_str(&line)?)?;
        line.clear();
    }
    Ok(())
}
#[derive(Deserialize)]
struct Fleet {
    epoch: u64,
    tick: u64,
    sha256: String,
    published_rows: u64,
    decks: u64,
    active_loads: u64,
    max_deck_error_m: f64,
    timed_steps: u64,
    step_total_us: u64,
    sample_us: u64,
    reset_us: u64,
}
#[derive(PartialEq, Eq)]
struct Physical {
    digest: String,
    rows: u64,
    active: u64,
}
type FleetCuts = BTreeMap<(u64, u64), String>;
fn fleet(path: &Path, reference: &mut BTreeMap<u64, Physical>) -> Result<(Value, FleetCuts)> {
    let mut cuts = BTreeMap::new();
    let (mut steps, mut time, mut audit, mut resets) = (0, 0, 0, 0);
    let mut max_error = 0f64;
    let mut last = None;
    records(path, |v: Fleet| {
        if v.decks != 100000 || v.published_rows != 404002 || v.max_deck_error_m > 1e-8 {
            return Err(format!("invalid fleet observation at {}/{}", v.epoch, v.tick).into());
        }
        let physical = Physical {
            digest: v.sha256.clone(),
            rows: v.published_rows,
            active: v.active_loads,
        };
        let matched = reference.entry(v.tick).or_insert_with(|| Physical {
            digest: v.sha256.clone(),
            rows: v.published_rows,
            active: v.active_loads,
        });
        if *matched != physical {
            return Err(format!(
                "fleet reset or worker width changed physics at {}/{}",
                v.epoch, v.tick
            )
            .into());
        }
        match last {
            None if v.epoch == 0 && v.tick == 0 => (),
            Some((epoch, tick))
                if epoch == v.epoch && v.tick > tick && v.tick - tick == v.timed_steps => {}
            Some((epoch, 4000)) if v.epoch == epoch + 1 && v.tick == 0 => (),
            _ => return Err("gap or duplicate in the fleet observation clock".into()),
        }
        last = Some((v.epoch, v.tick));
        steps += v.timed_steps;
        time += v.step_total_us;
        audit += v.sample_us;
        if v.tick == 0 {
            resets += v.reset_us;
        }
        max_error = max_error.max(v.max_deck_error_m);
        cuts.insert((v.epoch, v.tick), v.sha256);
        Ok(())
    })?;
    let out = json!({"cuts":cuts.len(),"steps":steps,"latest":last,"max_deck_error_m":max_error,"step_seconds":time as f64/1e6,"audit_seconds":audit as f64/1e6,"reset_seconds":resets as f64/1e6,"step_rtf":10000.*steps as f64/time as f64});
    Ok((out, cuts))
}
#[derive(Clone, Copy, Deserialize)]
struct Usage {
    rss_kib: u64,
    threads: u64,
    descriptors: u64,
}
#[derive(Deserialize)]
struct Lifecycle {
    epoch: u64,
    reverse: bool,
    physical_digest: String,
    elapsed_seconds: f64,
    workers: u32,
    transport: String,
    cumulative_steps: u64,
    cumulative_kills: u64,
    cumulative_reopens: u64,
    cumulative_loaded_transfers: u64,
    cumulative_withheld: u64,
    max_sphere_impulse_residual_ns: f64,
    pair_impulse_intervals: u64,
    controller: Usage,
    worker_totals: Usage,
}
#[derive(Default)]
struct Window {
    samples: u64,
    rss: u128,
    rss_max: u64,
    t_max: u64,
    d_max: u64,
}
impl Window {
    fn add(&mut self, r: Usage) {
        self.samples += 1;
        self.rss += u128::from(r.rss_kib);
        self.rss_max = self.rss_max.max(r.rss_kib);
        self.t_max = self.t_max.max(r.threads);
        self.d_max = self.d_max.max(r.descriptors);
    }
    fn report(&self) -> Value {
        json!({"samples":self.samples,"mean_rss_kib":self.rss as f64/self.samples as f64,"maximum_rss_kib":self.rss_max,"maximum_threads":self.t_max,"maximum_descriptors":self.d_max})
    }
}
#[derive(Deserialize, serde::Serialize)]
struct Continuation {
    workers: u32,
    source_epoch: u64,
    source_tick: u64,
    final_tick: u64,
    source_sha256: String,
    source_pending_effort: Option<f64>,
    physical_digest: String,
    verified_advances: u64,
    max_sphere_impulse_residual_ns: f64,
    transport: String,
}

fn sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|c| c.is_ascii_hexdigit())
}

fn switches(epochs: u64) -> u64 {
    epochs / 9 + u64::from(epochs % 9 >= 5)
}

fn read_continuation(dir: &Path, width: u32) -> Result<Option<Continuation>> {
    let file = match File::open(dir.join("continuation.json")) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e.into()),
    };
    let cut: Continuation = serde_json::from_reader(file)?;
    let half = if cut.source_epoch.is_multiple_of(2) {
        0.005
    } else {
        -0.005
    };
    if cut.workers != width
        || cut.final_tick != 1220
        || cut.source_tick > cut.final_tick
        || cut.source_epoch >= u64::MAX / 1220
        || cut.verified_advances != cut.final_tick - cut.source_tick
        || !sha256(&cut.source_sha256)
        || !sha256(&cut.physical_digest)
        || !(0.0..=1e-8).contains(&cut.max_sphere_impulse_residual_ns)
        || !matches!(cut.transport.as_str(), "Pipe" | "LoopbackZenoh")
        || cut
            .source_pending_effort
            .is_some_and(|e| !matches!(cut.source_tick, 149 | 1213) || e != half)
    {
        return Err("invalid convoy continuation receipt".into());
    }
    Ok(Some(cut))
}

#[derive(Deserialize)]
struct Completed {
    steps: u64,
    epochs: u64,
    reopens: u64,
    worker_kills: u64,
    loaded_transfers: u64,
    withheld_receipts: u64,
    // Legacy fresh campaigns published only counts.
    source_epoch: Option<u64>,
    source_tick: Option<u64>,
    source_sha256: Option<String>,
    final_epoch: Option<u64>,
    final_tick: Option<u64>,
    schedule_tick: Option<u64>,
}

fn lifecycle(
    dir: &Path,
    width: u32,
    reference: &mut BTreeMap<bool, String>,
    completed: bool,
) -> Result<Value> {
    // The writer publishes the continuation before its first complete epoch.
    // Pin the log prefix first so a live reader cannot misclassify a resumed run.
    let epoch_records = prefix(&dir.join("epochs.jsonl"))?;
    let continuation = read_continuation(dir, width)?;
    let first_epoch = continuation.as_ref().map_or(0, |cut| cut.source_epoch + 1);
    let prefix_steps = continuation.as_ref().map_or(0, |cut| cut.verified_advances);
    let mut initial_transport = continuation
        .as_ref()
        .map(|cut| cut.transport == "LoopbackZenoh");
    let mut epochs = 0;
    let mut last = json!(null);
    let mut peak = 0f64;
    let mut windows = BTreeMap::<(u64, String), [Window; 2]>::new();
    read_records(epoch_records, completed, |v: Lifecycle| {
        let expected_epoch = first_epoch.checked_add(epochs).ok_or("epoch overflow")?;
        if v.epoch != expected_epoch
            || v.epoch >= u64::MAX / 1220
            || !sha256(&v.physical_digest)
            || !(0.0..=1e-8).contains(&v.max_sphere_impulse_residual_ns)
            || !v.elapsed_seconds.is_finite()
            || v.elapsed_seconds < 0.
            || v.workers != width
            || v.reverse != (v.epoch % 2 == 1)
            || v.pair_impulse_intervals == 0
            || v.transport != "Pipe" && v.transport != "LoopbackZenoh"
        {
            return Err(format!("invalid lifecycle record {width}/{epochs}").into());
        }
        epochs += 1;
        let digest = reference
            .entry(v.reverse)
            .or_insert_with(|| v.physical_digest.clone());
        let reopens = switches(v.epoch + 1) - switches(first_epoch);
        let zenoh = v.transport == "LoopbackZenoh";
        let starts_on_zenoh = *initial_transport.get_or_insert(zenoh ^ !reopens.is_multiple_of(2));
        if zenoh != (starts_on_zenoh ^ !reopens.is_multiple_of(2))
            || *digest != v.physical_digest
            || epochs != v.cumulative_kills
            || epochs != v.cumulative_withheld
            || epochs != v.cumulative_loaded_transfers
            || v.cumulative_steps != prefix_steps + 1220 * epochs
            || v.cumulative_reopens != reopens
        {
            return Err(format!("lifecycle receipt mismatch at {width}/{epochs}").into());
        }
        peak = peak.max(v.max_sphere_impulse_residual_ns);
        let minute = ((v.elapsed_seconds / 900.).floor() as u64)
            .checked_mul(15)
            .ok_or("elapsed time exceeds the resource window clock")?;
        let bins = windows.entry((minute, v.transport)).or_default();
        bins[0].add(v.controller);
        bins[1].add(v.worker_totals);
        last = json!({"verified_epochs":epochs,"real_worker_kills":v.cumulative_kills,"withheld":v.cumulative_withheld,"loaded_transfers":v.cumulative_loaded_transfers,"reopens":v.cumulative_reopens,"steps":v.cumulative_steps,"elapsed_seconds":v.elapsed_seconds,"max_impulse_residual_ns":peak});
        Ok(())
    })?;
    let bins:Vec<_>=windows.into_iter().map(|((minute,trans),v)|json!({"minute":minute,"transport":trans,"controller":v[0].report(),"worker_totals":v[1].report()})).collect();
    if completed {
        let summary: Completed = serde_json::from_reader(File::open(dir.join("summary.json"))?)?;
        if continuation.is_some()
            || summary.source_epoch.is_some()
            || summary.source_tick.is_some()
            || summary.source_sha256.is_some()
            || summary.final_epoch.is_some()
            || summary.final_tick.is_some()
            || summary.schedule_tick.is_some()
        {
            let source_epoch = continuation.as_ref().map_or(0, |cut| cut.source_epoch);
            let source_tick = continuation.as_ref().map_or(0, |cut| cut.source_tick);
            let finished_epochs = first_epoch + epochs;
            let end = if finished_epochs == 0 {
                (0, 0)
            } else {
                (finished_epochs - 1, 1220)
            };
            if summary.source_epoch != Some(source_epoch)
                || summary.source_tick != Some(source_tick)
                || summary.source_sha256.as_deref()
                    != continuation.as_ref().map(|cut| cut.source_sha256.as_str())
                || summary.final_epoch != Some(end.0)
                || summary.final_tick != Some(end.1)
                || summary.schedule_tick != Some(finished_epochs * 1220)
            {
                return Err("terminal clock or source differs from epoch receipts".into());
            }
        }
        if summary.steps != prefix_steps + 1220 * epochs
            || summary.epochs != epochs
            || summary.worker_kills != epochs
            || summary.loaded_transfers != epochs
            || summary.withheld_receipts != epochs
            || summary.reopens != switches(first_epoch + epochs) - switches(first_epoch)
        {
            return Err(format!(
                "final summary disagrees with lifecycle receipts at width {width}"
            )
            .into());
        }
    }
    Ok(json!({"latest":last,"quarter_hours":bins,"continuation":continuation}))
}
fn fleet_comparison(dir: &Path, other: u32) -> Result<Value> {
    let mut physical = BTreeMap::new();
    let (one, left) = fleet(&dir.join("100k-1.jsonl"), &mut physical)?;
    let (many, right) = fleet(&dir.join(format!("100k-{other}.jsonl")), &mut physical)?;
    let mut matched = 0;
    for (cut, h) in &left {
        if let Some(other) = right.get(cut) {
            if h != other {
                return Err("public fleet digest mismatch".into());
            }
            matched += 1;
        }
    }
    let mut result = json!({"same_epoch_same_tick_width_matches":matched,
        "unique_physical_cuts_per_epoch":physical.len(),"width_one":one});
    let key = match other {
        4 => "width_four",
        7 => "width_seven",
        _ => return Err("unknown input width".into()),
    };
    result[key] = many;
    Ok(result)
}
#[derive(Parser)]
struct Args {
    /// Audit only the workers-3 and workers-8 directories below DIRECTORY.
    #[arg(long)]
    lifecycle_only: bool,
    /// Also require terminal summaries matching all complete epoch receipts.
    #[arg(long, requires = "lifecycle_only")]
    completed: bool,
    directory: PathBuf,
}

fn lifecycle_pair(dir: &Path, completed: bool) -> Result<Value> {
    let mut states = BTreeMap::new();
    let three = lifecycle(&dir.join("workers-3"), 3, &mut states, completed)?;
    let eight = lifecycle(&dir.join("workers-8"), 8, &mut states, completed)?;
    Ok(json!({"directional_digests":states,"three":three,"eight":eight}))
}

fn main() -> Result<()> {
    let args = Args::parse();
    let dir = args.directory;
    if args.lifecycle_only {
        println!(
            "{}",
            serde_json::to_string_pretty(&lifecycle_pair(&dir, args.completed)?)?
        );
        return Ok(());
    }
    let fleet = fleet_comparison(&dir.join("multifidelity-fleet-endurance"), 7)?;
    let sparse_dir = dir.join("multifidelity-sparse-deck-edits");
    // A prior run may have only the original width-one and width-seven receipts.
    let sparse =
        if sparse_dir.join("100k-1.jsonl").exists() || sparse_dir.join("100k-4.jsonl").exists() {
            Some(fleet_comparison(&sparse_dir, 4)?)
        } else {
            None
        };
    let lifecycle = lifecycle_pair(&dir.join("multifidelity-lifecycle-endurance"), false)?;
    let devices = if dir
        .join("multifidelity-resident-pair-recovery/episodes.jsonl")
        .exists()
    {
        Some(gpu::audit(&dir)?)
    } else {
        None
    };
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"fleet":fleet,"sparse":sparse,"device":devices,"lifecycle":lifecycle})
        )?
    );
    Ok(())
}
