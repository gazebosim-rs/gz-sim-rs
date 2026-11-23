// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Match independent migration-failure episodes and report complete transaction costs.
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    directory: PathBuf,
    #[arg(long, default_value = "matched")]
    prefix: String,
    #[arg(long,value_parser=clap::value_parser!(u32).range(1..))]
    blocks: u32,
    #[arg(long, default_value = "recover")]
    reference_arm: String,
    #[arg(long, default_value = "confirm")]
    candidate_arm: String,
}
#[derive(Deserialize)]
struct Measurement {
    policy: String,
    fault: String,
    endpoint: usize,
    before_pids: Vec<u32>,
    after_pids: Vec<u32>,
    elapsed_us: f64,
    requests: u64,
    request_bytes: u64,
    reply_bytes: u64,
}
impl Measurement {
    fn changes(&self) -> usize {
        self.before_pids
            .iter()
            .zip(&self.after_pids)
            .filter(|(a, b)| a != b)
            .count()
    }
}
struct Episode {
    physical: Value,
    moves: Vec<Measurement>,
}
fn read(dir: &Path) -> Result<Vec<Episode>> {
    let marker: Value = serde_json::from_slice(&fs::read(dir.join("result.json"))?)?;
    let mut episodes = Vec::new();
    for line in fs::read_to_string(dir.join("episodes.jsonl"))?.lines() {
        let mut physical: Value = serde_json::from_str(line)?;
        let mut moves = Vec::new();
        for m in physical["moves"].as_array_mut().ok_or("missing moves")? {
            let value = m
                .as_object_mut()
                .and_then(|o| o.remove("recovery"))
                .ok_or("missing migration observation")?;
            moves.push(serde_json::from_value(value)?);
        }
        if moves.len() != 2
            || physical["number"].as_u64()
                != marker["start"]
                    .as_u64()
                    .and_then(|n| n.checked_add(episodes.len() as u64))
        {
            return Err("wrong episode sequence or move count".into());
        }
        episodes.push(Episode { physical, moves });
    }
    if marker["episodes"].as_u64() != Some(episodes.len() as u64)
        || marker["ticks"].as_u64() != Some(episodes.len() as u64 * 350)
    {
        return Err("incomplete episode output".into());
    }
    Ok(episodes)
}
#[derive(Default)]
struct Stats {
    times: [Vec<f64>; 2],
    replaced: [usize; 2],
    calls: [u64; 2],
    bytes: [u64; 2],
    paired_pct: Vec<f64>,
}
fn median(xs: &mut [f64]) -> f64 {
    xs.sort_by(f64::total_cmp);
    (xs[xs.len() / 2] + xs[(xs.len() - 1) / 2]) / 2.
}
impl Stats {
    fn add(&mut self, prior: &Measurement, revised: &Measurement) {
        for (i, m) in [prior, revised].into_iter().enumerate() {
            self.times[i].push(m.elapsed_us);
            self.replaced[i] += m.changes();
            self.calls[i] += m.requests;
            self.bytes[i] += m.request_bytes + m.reply_bytes;
        }
        self.paired_pct
            .push(100. * (revised.elapsed_us / prior.elapsed_us - 1.));
    }
    fn report(mut self) -> Value {
        let n = self.paired_pct.len();
        let med = self.times.each_mut().map(|v| median(v));
        json!({"matched_moves":n,"median_us":med,"p95_us":self.times.each_ref().map(|v|v[(95*n).div_ceil(100)-1]),"median_paired_change_pct":median(&mut self.paired_pct),"replaced_workers":self.replaced,"requests":self.calls,"total_application_bytes":self.bytes})
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    let mut groups = BTreeMap::<String, Stats>::new();
    let mut episodes = 0u64;
    let mut next_number = None;
    for block in 0..args.blocks {
        let arm = |name| {
            read(
                &args
                    .directory
                    .join(format!("{}-{block}-{name}", args.prefix)),
            )
        };
        let prior = arm(&args.reference_arm)?;
        let revised = arm(&args.candidate_arm)?;
        if prior.len() != revised.len() {
            return Err("mismatched blocks".into());
        }
        for (a, b) in prior.iter().zip(revised) {
            let number = a.physical["number"]
                .as_u64()
                .ok_or("missing episode number")?;
            if a.physical != b.physical || next_number.is_some_and(|expected| number != expected) {
                return Err(
                    format!("episode schedule or native results differed at {number}").into(),
                );
            }
            let width = a.physical["width"].as_u64().ok_or("missing width")? as usize;
            for (i, (old, new)) in a.moves.iter().zip(&b.moves).enumerate() {
                let phase = a.physical["moves"][i]["withheld"]
                    .as_str()
                    .ok_or("missing phase")?;
                if old.policy
                    != (if args.reference_arm == "recover" {
                        "recover"
                    } else {
                        "confirm"
                    })
                    || new.policy != "confirm"
                    || old.fault != new.fault
                    || old.endpoint != new.endpoint
                    || new.endpoint >= width
                {
                    return Err("mismatched fault arms".into());
                }
                for m in [old, new] {
                    if m.before_pids.len() != width
                        || m.after_pids.len() != width
                        || !m.elapsed_us.is_finite()
                        || m.elapsed_us <= 0.
                    {
                        return Err("invalid migration observation".into());
                    }
                }
                let expect = match new.fault.as_str() {
                    "killed" | "stopped" => 1,
                    "lost" => 0,
                    "thaw" => {
                        usize::from(new.before_pids[new.endpoint] != new.after_pids[new.endpoint])
                    }
                    _ => return Err("unknown migration fault".into()),
                };
                if new.changes() != expect
                    || (new.before_pids[new.endpoint] != new.after_pids[new.endpoint])
                        != (expect == 1)
                {
                    return Err("unexpected worker replacement".into());
                }
                for key in [new.fault.clone(), format!("{}/{phase}", new.fault)] {
                    groups.entry(key).or_default().add(old, new);
                }
            }
            next_number = Some(number.checked_add(1).ok_or("episode number overflow")?);
            episodes += 1;
        }
    }
    let report: BTreeMap<_, _> = groups.into_iter().map(|(k, v)| (k, v.report())).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"arm_order":[args.reference_arm,args.candidate_arm],"matched_episodes":episodes,"ticks_per_arm":350*episodes,"body_states_per_arm":62*350*episodes,"migrations_per_arm":2*episodes,"by_fault_and_phase":report})
        )?
    );
    Ok(())
}
