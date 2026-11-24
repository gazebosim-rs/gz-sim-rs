// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Report matched distributed friction timings and validate invariants across all layouts and recoveries.
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    matrix: PathBuf,
    report: PathBuf,
    #[arg(required=true,num_args=1..)]
    bench: Vec<PathBuf>,
}
#[derive(Deserialize)]
struct Run {
    pointwise: bool,
    extended: bool,
    split: bool,
    packed: bool,
    width: u8,
    drive: String,
    round: usize,
    ticks: u64,
    native_site_microsteps: u64,
    loaded_pair_receipts: u64,
    exact_patch_points: u64,
    #[serde(default)]
    actual_signed_tangent_checks: u64,
    #[serde(default)]
    additional_spin_barrier_points: u64,
    max_barrier_tangent_residual_ns: Option<f64>,
    max_barrier_tangent_moment_residual_nms: Option<f64>,
    eligible: u64,
    violations: u64,
    maximum_ratio: f64,
    worst_cone_excess_ns: f64,
    max_tangent_linear_residual_ns: f64,
    max_tangent_angular_residual_nms: f64,
    elapsed_step_ns: u64,
    p95_step_ns: u64,
    trajectory_sha256: String,
    events: Vec<Value>,
}
fn read(path: &Path) -> Result<Vec<Run>> {
    BufReader::new(File::open(path.join("trials.jsonl"))?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect()
}
fn median(data: &[f64]) -> f64 {
    let mut a = data.to_vec();
    a.sort_by(f64::total_cmp);
    (a[a.len() / 2] + a[(a.len() - 1) / 2]) * 0.5
}
fn pct(values: &[f64], p: usize) -> f64 {
    let mut a = values.to_vec();
    a.sort_by(f64::total_cmp);
    a[(p * a.len()).div_ceil(100) - 1]
}
fn matched(a: &[u64], b: &[u64]) -> Value {
    let ga = a.iter().map(|&v| v as f64).collect::<Vec<_>>();
    let pa = b.iter().map(|&v| v as f64).collect::<Vec<_>>();
    let diffs = ga
        .iter()
        .zip(&pa)
        .map(|(g, p)| 100. * (p / g - 1.))
        .collect::<Vec<_>>();
    let mut rng = fastrand::Rng::with_seed(0x706f696e74);
    let mut resamples = vec![];
    let mut group = vec![0.; diffs.len()];
    for _ in 0..4000 {
        for v in &mut group {
            *v = diffs[rng.usize(..diffs.len())];
        }
        resamples.push(median(&group));
    }
    resamples.sort_by(f64::total_cmp);
    json!({"grouped_median_ns":median(&ga),"per_point_median_ns":median(&pa),
        "grouped_p95_ns":pct(&ga,95),"per_point_p95_ns":pct(&pa,95),
        "median_paired_change_percent":median(&diffs),"bootstrap_median_change_95_percent":[resamples[99],resamples[3899]],
        "per_point_faster_pairs":diffs.iter().filter(|p|**p<0.).count()})
}
/// Reopen the retained solver bytes using the combined immutable diagnostic, independently of the per-run channel.
fn recheck_cuts(matrix: &Path) -> Result<(usize, usize)> {
    use gz_physics_distributed::pool::NativeContactSnapshot;
    let mut total = 0;
    let mut archives = 0;
    for drive in ["mixed", "spin"] {
        for model in ["grouped", "pointwise"] {
            let file = File::open(matrix.join(format!("{drive}-{model}-corners.jsonl")))?;
            let mut stages = BTreeMap::<String, Vec<Value>>::new();
            for l in BufReader::new(file).lines() {
                let mut r: Value = serde_json::from_str(&l?)?;
                let fields = r.as_object_mut().ok_or("corner record")?;
                let stage = fields.remove("stage").unwrap();
                fields.remove("direct_block_tangent_ns");
                stages
                    .entry(stage.as_str().ok_or("stage")?.to_owned())
                    .or_default()
                    .push(r);
            }
            for (stage, mut recorded) in stages {
                let image = NativeContactSnapshot::open(
                    matrix
                        .join(format!("native-cuts-{drive}-{model}"))
                        .join(stage),
                )?;
                let mut read = vec![];
                for (owner, site) in image.patches.iter().enumerate() {
                    for patch in &site.finite {
                        for corner in &patch.points {
                            read.push(json!({"tick":image.frame.tick,"owner":owner,"pair":[patch.a,patch.b],"pointwise":model=="pointwise",
                    "normal_on_table":patch.normal_on_a,"normal_ns":corner.normal_impulse,"lever_table_m":corner.lever_a,
                    "lever_block_m":corner.lever_b,"archived_table_tangent_ns":corner.tangent_impulse_on_a}));
                        }
                    }
                }
                recorded.sort_by_key(Value::to_string);
                read.sort_by_key(Value::to_string);
                assert_eq!(recorded, read);
                total += read.len();
                archives += 1;
            }
        }
    }
    assert_eq!((archives, total), (26, 208));
    Ok((archives, total))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let rows = read(&args.matrix)?;
    let mut modes = BTreeSet::new();
    let mut hashes = BTreeMap::<(String, bool), String>::new();
    let mut signatures = BTreeMap::new();
    let mut runs = 0;
    let mut ticks = 0;
    let mut micros = 0;
    let mut receipts = 0;
    let mut corners = 0;
    let mut signed = 0;
    let mut extra = 0;
    let mut public_linear = 0.0_f64;
    let mut public_angular = 0.0_f64;
    for v in rows {
        assert!(modes.insert((
            v.drive.clone(),
            v.pointwise,
            v.extended,
            v.packed,
            v.split,
            v.width
        )));
        let key = (v.drive.clone(), v.pointwise);
        assert_eq!(
            hashes
                .entry(key)
                .or_insert_with(|| v.trajectory_sha256.clone()),
            &v.trajectory_sha256
        );
        let sig = json!({
            "drive": v.drive,
            "pointwise": v.pointwise,
            "eligible": v.eligible,
            "violations": v.violations,
            "max_ratio": v.maximum_ratio,
            "worst_cone_excess_ns": v.worst_cone_excess_ns,
            "tangent_linear_residual_ns": v.max_tangent_linear_residual_ns,
            "tangent_angular_residual_nms": v.max_tangent_angular_residual_nms,
            "trajectory_sha256": v.trajectory_sha256
        });
        let previous = signatures.insert(format!("{}-{}", v.drive, v.pointwise), sig.clone());
        assert!(previous.is_none() || previous == Some(sig));
        assert_eq!(v.events.len(), 5);
        assert_eq!(v.exact_patch_points, 48);
        let kinds = v
            .events
            .iter()
            .map(|e| e["event"]["kind"].as_str().unwrap())
            .collect::<Vec<_>>();
        assert_eq!(
            kinds,
            [
                "opposite_drives",
                "lost_final_ack",
                "loaded_handoff",
                "owner_replacement",
                "disk_resume"
            ]
        );
        assert_eq!(v.events[2]["event"]["members"], json!([1, 2]));
        runs += 1;
        ticks += v.ticks;
        micros += v.native_site_microsteps;
        receipts += v.loaded_pair_receipts;
        corners += v.exact_patch_points;
        signed += v.actual_signed_tangent_checks;
        extra += v.additional_spin_barrier_points;
        public_linear = public_linear.max(v.max_barrier_tangent_residual_ns.unwrap_or(0.));
        public_angular =
            public_angular.max(v.max_barrier_tangent_moment_residual_nms.unwrap_or(0.));
    }
    assert_eq!(runs, 192);
    if signed > 0 {
        assert_eq!(signed, 4616);
        assert_eq!(extra, 16);
    }
    assert_eq!(hashes.len(), 8);
    let retained = if signed > 0 {
        Some(recheck_cuts(&args.matrix)?)
    } else {
        None
    };
    let mut timings = vec![];
    for dir in args.bench {
        let trials = read(&dir)?;
        let total = trials.len();
        let mut table = BTreeMap::new();
        for v in trials {
            assert!(
                v.events.len() == 1
                    && v.events[0]["event"]["kind"] == "opposite_drives"
                    && v.exact_patch_points == 0
            );
            assert_eq!(
                hashes.get(&(v.drive.clone(), v.pointwise)),
                Some(&v.trajectory_sha256)
            );
            assert!(table.insert((v.round, v.split, v.pointwise), v).is_none());
        }
        for split in [false, true] {
            let (mut grouped, mut per, mut group_tail, mut per_tail) =
                (vec![], vec![], vec![], vec![]);
            for r in 0..total / 4 {
                let g = &table[&(r, split, false)];
                let p = &table[&(r, split, true)];
                grouped.push(g.elapsed_step_ns);
                per.push(p.elapsed_step_ns);
                group_tail.push(g.p95_step_ns);
                per_tail.push(p.p95_step_ns);
            }
            timings.push(json!({
                "campaign": dir.file_name().unwrap().to_string_lossy(),
                "split": split,
                "pairs": total/4,
                "public_step_total": matched(&grouped,&per),
                "within_run_step_p95": matched(&group_tail,&per_tail)
            }));
        }
    }
    serde_json::to_writer_pretty(
        File::create_new(args.report)?,
        &json!({
            "matrix": {"runs":runs,"exact_barriers":ticks,"native_site_microsteps":micros,"loaded_receipts":receipts,"saved_patch_points":corners,"actual_signed_tangent_checks":signed,"additional_spin_barrier_points":extra,"retained_native_files_and_corner_records_reopened":retained,"max_public_tangent_linear_residual_ns":public_linear,"max_public_tangent_angular_residual_nms":public_angular,"signatures":signatures},
            "benchmarks": timings
        }),
    )?;
    Ok(())
}
