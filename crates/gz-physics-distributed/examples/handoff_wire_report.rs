// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate and summarize paired pad_handoff_cost JSONL trials.
use clap::Parser;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    output: PathBuf,
    /// One independent CPU affinity campaign per file.
    #[arg(required = true)]
    campaigns: Vec<PathBuf>,
}

#[derive(PartialEq, Deserialize)]
struct Movement {
    from: u8,
    to: u8,
    component: u64,
    bodies: u32,
    positive_pairs: u32,
    positive_supports: u32,
    worlds_before: Vec<usize>,
    requests: u64,
    replies: u64,
    request_bytes: u64,
    reply_bytes: u64,
}

impl Movement {
    fn matched(&self, other: &Self) -> bool {
        (
            self.from,
            self.to,
            self.component,
            self.bodies,
            self.positive_pairs,
            self.positive_supports,
            &self.worlds_before,
            self.requests,
            self.replies,
        ) == (
            other.from,
            other.to,
            other.component,
            other.bodies,
            other.positive_pairs,
            other.positive_supports,
            &other.worlds_before,
            other.requests,
            other.replies,
        )
    }
}

#[derive(Deserialize)]
struct Trial {
    round: u16,
    layout: String,
    case: String,
    packed_requested: bool,
    packed_peers: usize,
    barrier: u64,
    movements: Vec<Movement>,
    transfer_total_us: f64,
    physics_sha256: String,
}

#[derive(Clone, Copy)]
struct Pair {
    json_us: f64,
    packed_us: f64,
    json_bytes: u64,
    packed_bytes: u64,
}

impl Pair {
    fn from_trials(json: Trial, packed: Trial) -> Self {
        assert_eq!((json.packed_peers, packed.packed_peers), (0, 4));
        assert_eq!(json.physics_sha256, packed.physics_sha256);
        assert_eq!(json.barrier, packed.barrier);
        assert!(!json.movements.is_empty());
        assert_eq!(json.movements.len(), packed.movements.len());
        for (a, b) in json.movements.iter().zip(&packed.movements) {
            assert!(a.matched(b), "transfer controls differ");
        }
        for trial in [&json, &packed] {
            assert!(trial.transfer_total_us.is_finite() && trial.transfer_total_us > 0.);
        }
        let bytes = |t: &Trial| {
            t.movements
                .iter()
                .map(|m| m.request_bytes + m.reply_bytes)
                .sum()
        };
        Self {
            json_us: json.transfer_total_us,
            packed_us: packed.transfer_total_us,
            json_bytes: bytes(&json),
            packed_bytes: bytes(&packed),
        }
    }
}

#[derive(Serialize)]
struct Stats {
    pairs: usize,
    packed_faster: usize,
    json_median_us: f64,
    packed_median_us: f64,
    json_p95_us: f64,
    packed_p95_us: f64,
    median_paired_change_pct: f64,
    json_median_bytes: f64,
    packed_median_bytes: f64,
}

type Samples = BTreeMap<String, BTreeMap<String, Vec<Pair>>>;
type LayoutStats = BTreeMap<String, BTreeMap<String, Stats>>;

fn summarize(layouts: Samples) -> LayoutStats {
    layouts
        .into_iter()
        .map(|(layout, cases)| {
            let groups = cases
                .into_iter()
                .map(|(name, pairs)| {
                    let sorted = |select: fn(&Pair) -> f64| {
                        let mut vals: Vec<_> = pairs.iter().map(select).collect();
                        vals.sort_by(f64::total_cmp);
                        vals
                    };
                    let n = pairs.len();
                    let median = |vals: &[f64]| (vals[(n - 1) / 2] + vals[n / 2]) / 2.;
                    let json = sorted(|p| p.json_us);
                    let packed = sorted(|p| p.packed_us);
                    let rank95 = (n * 95).div_ceil(100) - 1;
                    (
                        name,
                        Stats {
                            pairs: n,
                            packed_faster: pairs.iter().filter(|p| p.packed_us < p.json_us).count(),
                            json_median_us: median(&json),
                            packed_median_us: median(&packed),
                            json_p95_us: json[rank95],
                            packed_p95_us: packed[rank95],
                            median_paired_change_pct: median(&sorted(|p| {
                                (p.packed_us / p.json_us - 1.) * 100.
                            })),
                            json_median_bytes: median(&sorted(|p| p.json_bytes as f64)),
                            packed_median_bytes: median(&sorted(|p| p.packed_bytes as f64)),
                        },
                    )
                })
                .collect();
            (layout, groups)
        })
        .collect()
}

#[derive(Serialize)]
struct Summary {
    layouts: LayoutStats,
    campaigns: BTreeMap<String, LayoutStats>,
    physics_sha256: BTreeMap<String, String>,
}

fn read_campaign(
    path: &Path,
    combined: &mut Samples,
    digests: &mut BTreeMap<String, String>,
) -> Result<LayoutStats, Box<dyn std::error::Error>> {
    let mut arms = BTreeMap::<_, [Option<Trial>; 2]>::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let t: Trial = serde_json::from_str(&line?)?;
        let digest = digests
            .entry(t.layout.clone())
            .or_insert_with(|| t.physics_sha256.clone());
        assert_eq!(*digest, t.physics_sha256);
        let key = (t.layout.clone(), t.case.clone(), t.round);
        let idx = usize::from(t.packed_requested);
        assert!(
            arms.entry(key).or_default()[idx].replace(t).is_none(),
            "duplicate trial arm"
        );
    }
    assert!(!arms.is_empty());
    let mut samples = Samples::new();
    for ((layout, case, _), trials) in arms {
        let [Some(json), Some(packed)] = trials else {
            panic!("missing trial arm")
        };
        let pair = Pair::from_trials(json, packed);
        combined
            .entry(layout.clone())
            .or_default()
            .entry(case.clone())
            .or_default()
            .push(pair);
        samples
            .entry(layout)
            .or_default()
            .entry(case)
            .or_default()
            .push(pair);
    }
    Ok(summarize(samples))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut combined = Samples::new();
    let mut physics_sha256 = BTreeMap::new();
    let mut campaigns = BTreeMap::new();
    for path in args.campaigns {
        let key = path
            .file_stem()
            .ok_or("missing campaign filename")?
            .to_string_lossy()
            .into_owned();
        let stats = read_campaign(&path, &mut combined, &mut physics_sha256)?;
        assert!(
            campaigns.insert(key, stats).is_none(),
            "duplicate campaign name"
        );
    }
    let summary = Summary {
        layouts: summarize(combined),
        campaigns,
        physics_sha256,
    };
    std::fs::write(
        args.output,
        format!("{}\n", serde_json::to_string_pretty(&summary)?),
    )?;
    Ok(())
}
