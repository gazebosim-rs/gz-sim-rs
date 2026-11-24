// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired timing of moving contact loads using one coordinator and two worker schedulers.
#[path = "support/landing_intervals.rs"]
mod landing_intervals;
use clap::{Parser, Subcommand};
use plotters::prelude::*;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const LAYOUTS: [&str; 2] = ["adjacent", "separated"];
const WINDOWS: [&str; 6] = ["all", "change", "steady", "formation", "release", "landing"];

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Action,
}

#[derive(Subcommand)]
enum Action {
    /// Warm each pair, then retain both time series and reject differing physics.
    Run {
        #[arg(long)]
        profile: PathBuf,
        #[arg(long)]
        reference_worker: PathBuf,
        #[arg(long)]
        weighted_worker: PathBuf,
        /// Optional third worker. Rotate all six build orders and record three matched comparisons.
        #[arg(long)]
        third_worker: Option<PathBuf>,
        #[arg(long, default_value_t = 8, value_parser = clap::value_parser!(u16).range(1..))]
        rounds: u16,
        /// Fresh output directory for one CPU affinity set.
        out: PathBuf,
    },
    /// Compare coordinator binaries with identical worker, placement and physical inputs.
    Coordinators {
        #[arg(long)]
        reference: PathBuf,
        #[arg(long)]
        revised: PathBuf,
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, default_value = "split")]
        placement: String,
        #[arg(long,default_value_t=32,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        out: PathBuf,
    },
    /// Compare two placements with the same worker, coordinator and public inputs.
    Placement {
        #[arg(long)]
        profile: PathBuf,
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, default_value = "single")]
        reference: String,
        #[arg(long, default_value = "split")]
        revised: String,
        #[arg(long)]
        reference_helper_limit: Option<usize>,
        #[arg(long)]
        revised_helper_limit: Option<usize>,
        #[arg(long, default_value_t = 8, value_parser = clap::value_parser!(u16).range(1..))]
        rounds: u16,
        out: PathBuf,
    },
    /// Summarize independent affinity sets and draw the matched p95 results.
    Report {
        /// Legend for the reference worker used in every input set.
        #[arg(long, default_value = "World count")]
        reference_label: String,
        /// Legend for the revised worker in the timing figure.
        #[arg(long, default_value = "Estimated work")]
        current_label: String,
        /// Plot steady, four-tick landing windows and the six exact touchdowns.
        #[arg(long)]
        landings: bool,
        #[arg(long)]
        out: PathBuf,
        #[arg(required = true)]
        sets: Vec<PathBuf>,
    },
}

#[derive(Deserialize)]
struct Profile {
    physics_sha256: String,
    placement: Option<String>,
    helper_limit: Option<usize>,
}
#[derive(Deserialize)]
struct Sample {
    tick: usize,
    elapsed_us: f64,
    pad_pairs: [u32; 2],
}
#[derive(Serialize, Deserialize)]
struct Trial {
    round: u16,
    layout: String,
    reference_first: bool,
    physics_sha256: String,
    windows: BTreeMap<String, Times>,
}
#[derive(Serialize, Deserialize)]
struct Times {
    ticks: usize,
    reference_p95_us: f64,
    weighted_p95_us: f64,
    reference_max_us: f64,
    weighted_max_us: f64,
    reference_total_ms: f64,
    weighted_total_ms: f64,
}
fn read_lines<T: serde::de::DeserializeOwned>(path: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(path)?)
        .lines()
        .map(|s| Ok(serde_json::from_str(&s?)?))
        .collect()
}

#[derive(Clone, Copy)]
struct Variant<'a> {
    alternate_profile: Option<&'a Path>,
    worker: &'a Path,
    placement: Option<&'a str>,
    helper_limit: Option<usize>,
}
impl<'a> From<&'a Path> for Variant<'a> {
    fn from(worker: &'a Path) -> Self {
        Self {
            alternate_profile: None,
            worker,
            placement: None,
            helper_limit: None,
        }
    }
}

fn measure(
    profile: &Path,
    variant: Variant<'_>,
    layout: &str,
    path: &Path,
) -> Result<(Profile, Vec<Sample>)> {
    let worker = variant.worker;
    let mut command = Command::new(variant.alternate_profile.unwrap_or(profile));
    if let Some(placement) = variant.placement {
        command.args(["--placement", placement]);
    }
    if let Some(limit) = variant.helper_limit {
        command.arg("--helper-limit").arg(limit.to_string());
    }
    let output = command
        .args(["--layout", layout, "--worker"])
        .arg(worker)
        .arg("--rows")
        .arg(path)
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "{} failed for {layout}: {}",
            worker.display(),
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    let summary: Profile = serde_json::from_slice(&output.stdout)?;
    if let Some(placement) = variant.placement {
        if summary.placement.as_deref() != Some(placement)
            || summary.helper_limit != variant.helper_limit
        {
            return Err(
                format!("profile did not confirm placement {placement} and helper limit").into(),
            );
        }
        File::create_new(path.with_extension("summary.json"))?.write_all(&output.stdout)?;
    }
    Ok((summary, read_samples(path)?))
}

fn read_samples(path: &Path) -> Result<Vec<Sample>> {
    let rows: Vec<Sample> = read_lines(path)?;
    if rows.is_empty()
        || rows
            .windows(2)
            .any(|w| w[0].tick.checked_add(1) != Some(w[1].tick))
        || rows
            .iter()
            .any(|r| !r.elapsed_us.is_finite() || r.elapsed_us <= 0.)
    {
        return Err(format!("invalid measured series in {}", path.display()).into());
    }
    Ok(rows)
}

/// Tick of a change in either pad's positive pair count plus its next three ticks.
/// The first measured tick begins in the settled state, verified by the fixture.
fn windows(rows: &[Sample]) -> [Vec<usize>; 6] {
    let mut indices: [Vec<_>; 6] = Default::default();
    let (mut formation_until, mut release_until) = (None, None);
    for (i, row) in rows.iter().enumerate() {
        if i > 0 {
            let before = &rows[i - 1].pad_pairs;
            if row.pad_pairs.iter().zip(before).any(|(a, b)| a > b) {
                formation_until = Some(i + 3);
            }
            if row.pad_pairs.iter().zip(before).any(|(a, b)| a < b) {
                release_until = Some(i + 3);
            }
        }
        let formation = formation_until.is_some_and(|end| end >= i);
        let release = release_until.is_some_and(|end| end >= i);
        for (v, yes) in indices.iter_mut().zip([
            true,
            formation || release,
            !formation && !release,
            formation,
            release,
            formation_until == Some(i + 3),
        ]) {
            if yes {
                v.push(i);
            }
        }
    }
    indices
}
fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_unstable_by(f64::total_cmp);
    (xs[(xs.len() - 1) / 2] + xs[xs.len() / 2]) * 0.5
}
fn stats(rows: &[Sample], indices: &[usize]) -> (f64, f64, f64) {
    let mut xs: Vec<_> = indices.iter().map(|&i| rows[i].elapsed_us).collect();
    let total = xs.iter().sum::<f64>() / 1000.;
    xs.sort_unstable_by(f64::total_cmp);
    (
        xs[(95 * xs.len()).div_ceil(100) - 1],
        xs[xs.len() - 1],
        total,
    )
}

type Measurement = (Profile, Vec<Sample>);

fn validate_contacts(a: &[Sample], b: &[Sample], layout: &str, round: u16) -> Result<()> {
    if a.len() != b.len()
        || a.iter()
            .zip(b)
            .any(|(a, b)| a.tick != b.tick || a.pad_pairs != b.pad_pairs)
    {
        return Err(format!("contact masks or clock differ for {layout} round {round}").into());
    }
    Ok(())
}

fn record(
    records: &mut BufWriter<File>,
    layout: &str,
    round: u16,
    reference_first: bool,
    digest: &str,
    a: &[Sample],
    b: &[Sample],
) -> Result<()> {
    validate_contacts(a, b, layout, round)?;
    let mut cuts = BTreeMap::new();
    for (name, indices) in WINDOWS.into_iter().zip(windows(a)) {
        if indices.is_empty() {
            return Err(format!("no {name} ticks for {layout}").into());
        }
        let (reference_p95_us, reference_max_us, reference_total_ms) = stats(a, &indices);
        let (weighted_p95_us, weighted_max_us, weighted_total_ms) = stats(b, &indices);
        cuts.insert(
            name.to_owned(),
            Times {
                ticks: indices.len(),
                reference_p95_us,
                weighted_p95_us,
                reference_max_us,
                weighted_max_us,
                reference_total_ms,
                weighted_total_ms,
            },
        );
    }
    serde_json::to_writer(
        &mut *records,
        &Trial {
            round,
            layout: layout.into(),
            reference_first,
            physics_sha256: digest.to_owned(),
            windows: cuts,
        },
    )?;
    writeln!(records)?;
    records.flush()?;
    Ok(())
}

fn run(
    profile: &Path,
    reference: Variant<'_>,
    weighted: Variant<'_>,
    third: Option<Variant<'_>>,
    rounds: u16,
    out: &Path,
) -> Result<()> {
    fs::create_dir(out)?;
    let mut policies = vec![("reference", reference), ("weighted", weighted)];
    let mut comparisons = vec![(0, 1, out.to_owned())];
    if let Some(w) = third {
        policies.push(("third", w));
        comparisons.push((0, 2, out.join("reference-third")));
        comparisons.push((1, 2, out.join("weighted-third")));
        for (_, _, dir) in &comparisons[1..] {
            fs::create_dir(dir)?;
        }
    }
    let mut writers = Vec::new();
    for (a, b, dir) in comparisons {
        writers.push((
            a,
            b,
            BufWriter::new(File::create_new(dir.join("paired.jsonl"))?),
        ));
    }
    const ORDERS: [[usize; 3]; 6] = [
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
        [1, 0, 2],
        [0, 2, 1],
    ];
    let mut digests = BTreeMap::new();
    for round in 0..=rounds {
        for j in 0..LAYOUTS.len() {
            let which = (j + usize::from(round)) % LAYOUTS.len();
            let layout = LAYOUTS[which];
            let sequence = usize::from(round) + which;
            let order: &[usize] = if third.is_some() {
                &ORDERS[sequence % ORDERS.len()]
            } else if sequence.is_multiple_of(2) {
                &[0, 1]
            } else {
                &[1, 0]
            };
            let mut results: [Option<Measurement>; 3] = [None, None, None];
            let mut rank = [0; 3];
            for (position, &index) in order.iter().enumerate() {
                let (label, worker) = policies[index];
                let path = out.join(format!("{layout}-{round}-{label}.jsonl"));
                let measurement = measure(profile, worker, layout, &path)?;
                let digest = &measurement.0.physics_sha256;
                if digest.len() != 64
                    || digests.entry(layout).or_insert_with(|| digest.clone()) != digest
                {
                    return Err(format!("trajectory differs for {layout} round {round}").into());
                }
                results[index] = Some(measurement);
                rank[index] = position;
            }
            if round > 0 {
                for (a, b, writer) in &mut writers {
                    let (p, prior) = results[*a].as_ref().unwrap();
                    let (_, next) = results[*b].as_ref().unwrap();
                    record(
                        writer,
                        layout,
                        round,
                        rank[*a] < rank[*b],
                        &p.physics_sha256,
                        prior,
                        next,
                    )?;
                }
            }
        }
    }
    Ok(())
}

#[derive(Serialize)]
struct WindowSummary {
    pairs: usize,
    ticks_per_run: usize,
    reference_p95_us: f64,
    weighted_p95_us: f64,
    paired_p95_reduction_pct: f64,
    reference_first_p95_reduction_pct: Option<f64>,
    revised_first_p95_reduction_pct: Option<f64>,
    p95_improved: usize,
    reference_run_max_us: f64,
    weighted_run_max_us: f64,
    worst_reference_us: f64,
    worst_weighted_us: f64,
    paired_total_reduction_pct: f64,
    weighted_p95_change_from_steady_pct: f64,
}
#[derive(Serialize)]
struct LayoutSummary {
    physics_sha256: String,
    windows: BTreeMap<String, WindowSummary>,
}
type Summary = BTreeMap<String, LayoutSummary>;
fn summarize(trials: &[Trial]) -> Result<Summary> {
    let mut result = Summary::new();
    for layout in LAYOUTS {
        let selected: Vec<_> = trials.iter().filter(|t| t.layout == layout).collect();
        let first = selected.first().ok_or("missing scene")?;
        if selected
            .iter()
            .any(|t| t.physics_sha256 != first.physics_sha256)
        {
            return Err("inconsistent scene digest".into());
        }
        let steady: Vec<_> = selected
            .iter()
            .map(|t| t.windows.get("steady").ok_or("missing steady window"))
            .collect::<std::result::Result<_, _>>()?;
        let mut by_window = BTreeMap::new();
        for w in WINDOWS {
            // Earlier trial sets contain only the four-tick landing window.
            if w == "landing" && selected.iter().all(|t| !t.windows.contains_key(w)) {
                continue;
            }
            let ts: Vec<_> = selected
                .iter()
                .map(|t| t.windows.get(w).ok_or("missing window"))
                .collect::<std::result::Result<_, _>>()?;
            let ticks_per_run = ts[0].ticks;
            if ts.iter().any(|t| t.ticks != ticks_per_run) {
                return Err("timing masks differ across trials".into());
            }
            let med = |f: fn(&Times) -> f64| median(ts.iter().map(|t| f(t)).collect());
            let reduction = |t: &Times| 100. * (1. - t.weighted_p95_us / t.reference_p95_us);
            let by_order = |reference_first| {
                let values: Vec<_> = selected
                    .iter()
                    .zip(&ts)
                    .filter(|(trial, _)| trial.reference_first == reference_first)
                    .map(|(_, t)| reduction(t))
                    .collect();
                (!values.is_empty()).then(|| median(values))
            };
            by_window.insert(
                w.to_owned(),
                WindowSummary {
                    pairs: ts.len(),
                    ticks_per_run,
                    reference_p95_us: med(|t| t.reference_p95_us),
                    weighted_p95_us: med(|t| t.weighted_p95_us),
                    paired_p95_reduction_pct: med(reduction),
                    reference_first_p95_reduction_pct: by_order(true),
                    revised_first_p95_reduction_pct: by_order(false),
                    p95_improved: ts
                        .iter()
                        .filter(|t| t.weighted_p95_us < t.reference_p95_us)
                        .count(),
                    reference_run_max_us: med(|t| t.reference_max_us),
                    weighted_run_max_us: med(|t| t.weighted_max_us),
                    worst_reference_us: ts.iter().map(|t| t.reference_max_us).fold(0., f64::max),
                    worst_weighted_us: ts.iter().map(|t| t.weighted_max_us).fold(0., f64::max),
                    weighted_p95_change_from_steady_pct: median(
                        ts.iter()
                            .zip(&steady)
                            .map(|(t, s)| 100. * (t.weighted_p95_us / s.weighted_p95_us - 1.))
                            .collect(),
                    ),
                    paired_total_reduction_pct: med(|t| {
                        100. * (1. - t.weighted_total_ms / t.reference_total_ms)
                    }),
                },
            );
        }
        result.insert(
            layout.to_owned(),
            LayoutSummary {
                physics_sha256: first.physics_sha256.clone(),
                windows: by_window,
            },
        );
    }
    Ok(result)
}

fn plot(
    out: &Path,
    combined: &Summary,
    reference_label: &str,
    current_label: &str,
    landings: bool,
) -> Result<()> {
    let root = SVGBackend::new(out, (980, 430)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "Public tick latency as contact loads move",
            ("sans-serif", 25),
        )?
        .split_evenly((1, 2));
    let (keys, names) = if landings {
        (
            ["steady", "formation", "landing"],
            ["Steady", "Landing window", "Worst of six"],
        )
    } else {
        (
            ["all", "change", "steady"],
            ["All ticks", "Near change", "Steady"],
        )
    };
    if combined
        .values()
        .any(|l| keys.iter().any(|k| !l.windows.contains_key(*k)))
    {
        return Err("missing plot window".into());
    }
    let max = combined
        .values()
        .flat_map(|l| {
            keys.map(|w| {
                l.windows[w]
                    .reference_p95_us
                    .max(l.windows[w].weighted_p95_us)
            })
        })
        .fold(0., f64::max)
        * 1.22;
    for (area, layout) in areas.iter().zip(LAYOUTS) {
        let mut chart = ChartBuilder::on(area)
            .margin(15)
            .caption(
                format!(
                    "{} pad IDs",
                    if layout == "adjacent" {
                        "Adjacent"
                    } else {
                        "Separated"
                    }
                ),
                ("sans-serif", 18),
            )
            .x_label_area_size(42)
            .y_label_area_size(75)
            .build_cartesian_2d(0.5..3.5, 0.0..max)?;
        chart
            .configure_mesh()
            .disable_x_mesh()
            .max_light_lines(1)
            .x_labels(3)
            .x_label_formatter(&|x| match *x {
                1.0 => names[0].into(),
                2.0 => names[1].into(),
                3.0 => names[2].into(),
                _ => String::new(),
            })
            .y_desc("Median trial p95 (µs)")
            .draw()?;
        for (j, (label, color)) in [
            (reference_label, RGBColor(159, 109, 42)),
            (current_label, RGBColor(26, 121, 139)),
        ]
        .into_iter()
        .enumerate()
        {
            chart
                .draw_series(keys.into_iter().enumerate().map(|(i, w)| {
                    let s = &combined[layout].windows[w];
                    let x = (i + 1) as f64 + if j == 0 { -0.19 } else { 0.02 };
                    let y = if j == 0 {
                        s.reference_p95_us
                    } else {
                        s.weighted_p95_us
                    };
                    Rectangle::new([(x, 0.), (x + 0.17, y)], color.filled())
                }))?
                .label(label)
                .legend(move |(x, y)| {
                    Rectangle::new([(x, y - 4), (x + 14, y + 4)], color.filled())
                });
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
/// Pool raw touchdown samples as well as reporting the distribution of per-run percentiles.
type Pooled = BTreeMap<(String, &'static str, &'static str), Vec<f64>>;
fn collect_landings(
    pooled: &mut Pooled,
    dir: &Path,
    trials: &[Trial],
) -> Result<landing_intervals::Set> {
    let mut groups = landing_intervals::Set::new();
    let mut seen = std::collections::BTreeSet::new();
    let (root, tags) = match dir.file_name().and_then(|n| n.to_str()) {
        Some("reference-third") => (
            dir.parent().ok_or("missing raw trials")?,
            ["reference", "third"],
        ),
        Some("weighted-third") => (
            dir.parent().ok_or("missing raw trials")?,
            ["weighted", "third"],
        ),
        _ => (dir, ["reference", "weighted"]),
    };
    for t in trials {
        if t.round == 0 || !seen.insert((&t.layout, t.round)) {
            return Err("duplicate or warmup trial in measured set".into());
        }
        let mut touchdowns = [vec![], vec![]];
        let [prior, revised] = tags
            .map(|tag| read_samples(&root.join(format!("{}-{}-{tag}.jsonl", t.layout, t.round))));
        let (prior, revised) = (prior?, revised?);
        validate_contacts(&prior, &revised, &t.layout, t.round)?;
        for (arm, (policy, raw)) in [("reference", prior), ("revised", revised)]
            .into_iter()
            .enumerate()
        {
            let [_, _, _, formation, _, landing] = windows(&raw);
            touchdowns[arm] = landing.iter().map(|&i| raw[i].elapsed_us).collect();
            for (name, indices) in [("formation", formation), ("landing", landing)] {
                let expected = t.windows.get(name).ok_or("missing landing data")?;
                if indices.is_empty() || indices.len() != expected.ticks {
                    return Err("raw window differs from trial record".into());
                }
                let per_run_p95 = stats(&raw, &indices).0;
                let recorded = if policy == "reference" {
                    expected.reference_p95_us
                } else {
                    expected.weighted_p95_us
                };
                if per_run_p95 != recorded {
                    return Err("raw p95 differs from trial record".into());
                }
                pooled
                    .entry((t.layout.clone(), name, policy))
                    .or_default()
                    .extend(indices.into_iter().map(|i| raw[i].elapsed_us));
            }
        }
        let w = &t.windows["formation"];
        groups
            .entry((t.layout.clone(), t.reference_first))
            .or_default()
            .push(landing_intervals::Pair {
                touchdowns,
                window_change: 100. * (w.weighted_p95_us / w.reference_p95_us - 1.),
            });
    }
    Ok(groups)
}
fn summarize_pooled(pooled: &mut Pooled) -> Vec<serde_json::Value> {
    pooled
        .iter_mut()
        .map(|((layout, window, policy), samples)| {
            samples.sort_unstable_by(f64::total_cmp);
            let n = samples.len();
            serde_json::json!({"layout":layout,"window":window,"policy":policy,"ticks":n,
            "p95_us":samples[(95*n).div_ceil(100)-1],"worst_us":samples[n-1]})
        })
        .collect()
}
fn main() -> Result<()> {
    match Args::parse().command {
        Action::Run {
            profile,
            reference_worker,
            weighted_worker,
            third_worker,
            rounds,
            out,
        } => run(
            &profile,
            reference_worker.as_path().into(),
            weighted_worker.as_path().into(),
            third_worker.as_deref().map(Variant::from),
            rounds,
            &out,
        ),
        Action::Coordinators {
            reference,
            revised,
            worker,
            placement,
            rounds,
            out,
        } => {
            let prior = Variant {
                worker: &worker,
                placement: Some(&placement),
                helper_limit: None,
                alternate_profile: None,
            };
            run(
                &reference,
                prior,
                Variant {
                    alternate_profile: Some(&revised),
                    ..prior
                },
                None,
                rounds,
                &out,
            )
        }
        Action::Placement {
            profile,
            worker,
            reference,
            revised,
            reference_helper_limit,
            revised_helper_limit,
            rounds,
            out,
        } => {
            if reference == revised && reference_helper_limit == revised_helper_limit {
                return Err("placements must differ".into());
            }
            run(
                &profile,
                Variant {
                    alternate_profile: None,
                    worker: &worker,
                    placement: Some(&reference),
                    helper_limit: reference_helper_limit,
                },
                Variant {
                    alternate_profile: None,
                    worker: &worker,
                    placement: Some(&revised),
                    helper_limit: revised_helper_limit,
                },
                None,
                rounds,
                &out,
            )
        }
        Action::Report {
            out,
            sets,
            reference_label,
            current_label,
            landings,
        } => {
            let mut trials = vec![];
            let mut pooled = Pooled::new();
            let mut by_set = BTreeMap::new();
            let mut landings_by_set = BTreeMap::new();
            let mut interval_sets = Vec::new();
            for dir in sets {
                let rows: Vec<Trial> = read_lines(&dir.join("paired.jsonl"))?;
                let key = dir.canonicalize()?;
                if landings {
                    let mut samples = Pooled::new();
                    interval_sets.push(collect_landings(&mut samples, &key, &rows)?);
                    landings_by_set.insert(key.clone(), summarize_pooled(&mut samples));
                    for (group, mut values) in samples {
                        pooled.entry(group).or_default().append(&mut values);
                    }
                }
                if by_set.insert(key, summarize(&rows)?).is_some() {
                    return Err("duplicate trial directory".into());
                }
                trials.extend(rows);
            }
            let combined = summarize(&trials)?;
            fs::create_dir_all(&out)?;
            if landings {
                fs::write(
                    out.join("paired-intervals.json"),
                    serde_json::to_vec_pretty(&landing_intervals::report(&interval_sets))?,
                )?;
                fs::write(
                    out.join("pooled-landings.json"),
                    serde_json::to_vec_pretty(&summarize_pooled(&mut pooled))?,
                )?;
                fs::write(
                    out.join("pooled-landings-by-set.json"),
                    serde_json::to_vec_pretty(&landings_by_set)?,
                )?;
            }
            plot(
                &out.join("p95.svg"),
                &combined,
                &reference_label,
                &current_label,
                landings,
            )?;
            fs::write(
                out.join("summary.json"),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"reference":reference_label,"revised":current_label,"combined":combined,"by_set":by_set}),
                )?,
            )?;
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paired_report_keeps_ab_and_ba_results_separate() {
        let make_trial = |layout: &str, reference_first, revised| Trial {
            layout: layout.into(),
            round: if reference_first { 1 } else { 2 },
            reference_first,
            physics_sha256: "a".repeat(64),
            windows: WINDOWS
                .into_iter()
                .map(|w| {
                    (
                        w.into(),
                        Times {
                            ticks: 6,
                            reference_p95_us: 100.,
                            weighted_p95_us: revised,
                            reference_max_us: 100.,
                            weighted_max_us: revised,
                            reference_total_ms: 0.6,
                            weighted_total_ms: revised * 0.006,
                        },
                    )
                })
                .collect(),
        };
        let mut trials: Vec<_> = LAYOUTS
            .iter()
            .map(|layout| make_trial(layout, true, 75.))
            .collect();
        let first_only = summarize(&trials).unwrap();
        assert_eq!(
            first_only["adjacent"].windows["landing"].revised_first_p95_reduction_pct,
            None
        );
        trials.extend(LAYOUTS.iter().map(|layout| make_trial(layout, false, 150.)));
        let both = summarize(&trials).unwrap();
        let window = &both["adjacent"].windows["landing"];
        assert_eq!(window.reference_first_p95_reduction_pct, Some(25.));
        assert_eq!(window.revised_first_p95_reduction_pct, Some(-50.));
        assert_eq!(window.paired_p95_reduction_pct, -12.5);
    }

    #[test]
    fn overlapping_transition_windows_are_counted_once_and_clipped_at_the_end() {
        let pairs = [
            [2, 2],
            [0, 2],
            [0, 2],
            [2, 0],
            [2, 0],
            [2, 0],
            [2, 0],
            [2, 0],
        ];
        let rows: Vec<_> = pairs
            .into_iter()
            .enumerate()
            .map(|(tick, pad_pairs)| Sample {
                tick,
                pad_pairs,
                elapsed_us: (tick + 1) as f64,
            })
            .collect();
        let [all, change, steady, formation, release, landing] = windows(&rows);
        assert_eq!(landing, vec![3]);
        assert_eq!(change, vec![1, 2, 3, 4, 5, 6]);
        assert_eq!(steady, vec![0, 7]);
        assert_eq!(formation, vec![3, 4, 5, 6]);
        assert_eq!(release, change);
        assert_eq!(stats(&rows, &all), (8., 8., 0.036));
        assert_eq!(windows(&rows[..5])[1], vec![1, 2, 3, 4]);
    }
}
