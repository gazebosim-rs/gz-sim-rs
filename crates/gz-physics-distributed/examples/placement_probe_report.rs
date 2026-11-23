// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pair actual stopped-worker timings and verify the retained/replaced PID set for every arm.
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::{Path, PathBuf},
};
#[path = "support/step_confirm_report.rs"]
mod step_confirm_report;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    directory: PathBuf,
    /// Compare final-step confirmation policies and the public SDF controls.
    #[arg(long)]
    final_step: bool,
    /// Compare resident-only versus all-worker advisory probes under the same deadline.
    #[arg(long, conflicts_with = "final_step")]
    residents: bool,
    #[arg(long, default_value_t = 6)]
    blocks: u16,
}
#[derive(Deserialize)]
struct Episode {
    round: u16,
    tick: u64,
    positive_pairs_before: usize,
    before_pids: [u32; 4],
    after_pids: [u32; 4],
    elapsed_us: f64,
    first_step_retry: bool,
    unavailable_probes: u64,
    manual_probe_accepted: Option<bool>,
    #[serde(default)]
    first_failure_us: Option<f64>,
    windows_started: u64,
}
fn read(p: &Path) -> Result<(Value, Vec<Episode>)> {
    let proof: Value = serde_json::from_slice(&fs::read(p.join("proof.json"))?)?;
    let rows = fs::read_to_string(p.join("episodes.jsonl"))?
        .lines()
        .map(serde_json::from_str)
        .collect::<std::result::Result<Vec<Episode>, _>>()?;
    if proof["ticks_per_run"] != 350
        || proof["native_sha256"]
            .as_str()
            .is_none_or(|s| s.len() != 64)
        || proof["rounds"].as_u64() != Some(rows.len() as u64)
    {
        return Err("incomplete native proof".into());
    }
    Ok((proof, rows))
}
#[derive(Default)]
struct Times {
    us: [Vec<f64>; 2],
    diff: Vec<f64>,
    retries: [u64; 2],
    replacements: [u64; 2],
    unavailable: [u64; 2],
    manual: [u64; 2],
    retry_us: [Vec<f64>; 2],
}
fn median(v: &[f64]) -> f64 {
    (v[v.len() / 2] + v[(v.len() - 1) / 2]) / 2.
}
impl Times {
    fn add(&mut self, a: &Episode, b: &Episode) {
        for (i, e) in [a, b].into_iter().enumerate() {
            self.us[i].push(e.elapsed_us);
            if let Some(first) = e.first_failure_us {
                self.retry_us[i].push(e.elapsed_us - first);
            }
            self.retries[i] += u64::from(e.first_step_retry);
            self.replacements[i] += e
                .before_pids
                .iter()
                .zip(&e.after_pids)
                .filter(|(a, b)| a != b)
                .count() as u64;
            self.unavailable[i] += e.unavailable_probes;
            self.manual[i] += u64::from(e.manual_probe_accepted == Some(true));
        }
        self.diff.push(b.elapsed_us - a.elapsed_us);
    }
    fn report(mut self) -> Value {
        for t in &mut self.us {
            t.sort_by(f64::total_cmp);
        }
        self.diff.sort_by(f64::total_cmp);
        for t in &mut self.retry_us {
            t.sort_by(f64::total_cmp);
        }
        let n = self.diff.len();
        json!({"pairs":n,"retry_median_us":self.retry_us.each_ref().map(|v|(!v.is_empty()).then(||median(v))),"median_us":self.us.each_ref().map(|v|median(v)),"p95_us":self.us.each_ref().map(|v|v[(95*n).div_ceil(100)-1]),
            "median_paired_change_us":median(&self.diff),"step_retries":self.retries,"replacements":self.replacements,"unavailable_probes":self.unavailable,"manual_probe_accepted":self.manual})
    }
}
fn pair(
    base: &Path,
    name: &str,
    keys: &[String],
    suffixes: [&str; 2],
    groups: &mut BTreeMap<String, Times>,
) -> Result<usize> {
    let (a, x) = read(&base.join(format!("{name}-{}", suffixes[0])))?;
    let (b, y) = read(&base.join(format!("{name}-{}", suffixes[1])))?;
    let peer = a["peer"]
        .as_u64()
        .filter(|&p| p < 4)
        .ok_or("missing worker")? as usize;
    if a != b || x.len() != y.len() {
        return Err("native trajectory, fault or episode count differs".into());
    }
    for (j, (r, c)) in x.iter().zip(&y).enumerate() {
        if r.round != c.round
            || usize::from(r.round) != j
            || r.tick != c.tick
            || r.positive_pairs_before == 0
            || r.positive_pairs_before != c.positive_pairs_before
        {
            return Err("different fault on physical trajectory".into());
        }
        for e in [r, c] {
            if !e.elapsed_us.is_finite()
                || e.elapsed_us <= 0.
                || e.before_pids.iter().zip(&e.after_pids).enumerate().any(
                    |(i, (before, after))| {
                        (before != after) != (i == peer && a["thaw_ms"].is_null())
                    },
                )
            {
                return Err("invalid timing or PID replacements".into());
            }
        }
        for k in keys {
            groups.entry(k.clone()).or_default().add(r, c);
        }
    }
    Ok(x.len())
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.final_step {
        return step_confirm_report::run(&args.directory, args.blocks);
    }
    let suffixes = if args.residents {
        ["all", "residents"]
    } else {
        ["long-probe", "short-probe"]
    };
    let labels = if args.residents {
        ["all workers", "resident workers"]
    } else {
        ["1 second", "25 milliseconds"]
    };
    let mut groups = BTreeMap::new();
    let mut cases = 0;
    for layout in ["adjacent", "separated"] {
        for b in 0..args.blocks {
            for phase in ["start", "read"] {
                for peer in [0, 2] {
                    for thaw in ["0", "10", "100", "persist"] {
                        let tag = format!("{b}-{phase}-{peer}-{thaw}");
                        cases += pair(
                            &args.directory.join(layout),
                            &tag,
                            &[
                                format!("automatic/{thaw}"),
                                format!("automatic/{phase}/{thaw}"),
                                format!("automatic/{layout}/{phase}/{peer}/{thaw}"),
                            ],
                            suffixes,
                            &mut groups,
                        )?;
                    }
                }
            }
        }
    }
    for mode in ["manual", "fixed"] {
        for phase in ["start", "read"] {
            pair(
                &args.directory,
                &format!("control-{mode}-{phase}"),
                &[format!("{mode}/100")],
                suffixes,
                &mut groups,
            )?;
        }
    }
    let report: BTreeMap<_, _> = groups.into_iter().map(|(k, g)| (k, g.report())).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"arm_order":labels,"matched_automatic_episodes":cases,
        "verified_native_ticks_per_automatic_arm":350*cases,"measurements":report})
        )?
    );
    Ok(())
}
