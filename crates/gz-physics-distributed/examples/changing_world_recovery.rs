// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export the verified changing-load run and recorded contact/p95 gauges to gz-web-rs.
#[path = "support/changing_worlds.rs"]
mod fixture;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[path = "support/changing_recovery.rs"]
mod trial;

use clap::{Parser, ValueEnum};
use gz_physics_distributed::physics::{Body, Primitive};
use plotters::prelude::*;
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum, default_value_t = fixture::Layout::Adjacent)]
    layout: fixture::Layout,
    /// Recover the worker immediately before the migrated cargo's touchdown.
    #[arg(long)]
    landing: bool,
    /// Optional per-layout measured baseline worst landing, with disjoint elapsed spans.
    #[arg(long, requires = "landing")]
    worst_landings: Option<PathBuf>,
    /// Optional post-change worst touchdown sample on the same scale as the baseline.
    #[arg(long, requires = "worst_landings")]
    revised_worst_landings: Option<PathBuf>,
    /// Optional matched layout/transition p95 summary from the benchmark.
    #[arg(long)]
    timings: Option<PathBuf>,
    /// Selected worker for the shared-queue versus early-helper-wake trial.
    #[arg(long, value_enum, requires_all = ["timings", "landing"])]
    replay_policy: Option<ReplayPolicy>,
    /// Worker used for playback, identified by its column in the paired benchmark.
    #[arg(long, value_enum, conflicts_with = "replay_policy", requires_all = ["timings", "landing"])]
    selected_worker: Option<SelectedWorker>,
    /// Slowest post-solve interval on an exact touchdown for the reference worker.
    #[arg(long, requires = "selected_worker")]
    post_solve: Option<PathBuf>,
    /// Matching post-solve report for the revised worker.
    #[arg(long, requires = "post_solve")]
    revised_post_solve: Option<PathBuf>,
    /// Longest caller scope return across all measured ticks of the reference worker.
    #[arg(long, requires = "selected_worker")]
    all_tick_joins: Option<PathBuf>,
    /// Matching all-tick scope return sample for the revised worker.
    #[arg(long, requires = "all_tick_joins")]
    revised_all_tick_joins: Option<PathBuf>,
    /// Helper dispatch counters on the baseline profile's exact touchdown.
    #[arg(long, requires_all = ["replay_policy", "worst_landings"])]
    helper_wake: Option<PathBuf>,
    /// Matching helper dispatch counters for the experimental worker.
    #[arg(long, requires_all = ["helper_wake", "revised_worst_landings"])]
    revised_helper_wake: Option<PathBuf>,
    /// Optional observed helper assignments from the fixed-owner adjacent-pad benchmark.
    #[arg(long)]
    lane_report: Option<PathBuf>,
    /// Fresh output directory for the archive, trace and plot.
    directory: PathBuf,
}

#[derive(Clone, Copy, ValueEnum)]
enum ReplayPolicy {
    SharedQueue,
    EarlyHelperWake,
}
impl ReplayPolicy {
    fn labels(self) -> [&'static str; 2] {
        match self {
            Self::SharedQueue => ["shared_queue_selected", "early_helper_wake_trial"],
            Self::EarlyHelperWake => ["shared_queue_prior", "early_helper_wake_selected"],
        }
    }
}
#[derive(Clone, Copy, ValueEnum)]
enum SelectedWorker {
    Reference,
    Revised,
}
impl SelectedWorker {
    fn labels(self, doc: &Value) -> trial::Result<[String; 2]> {
        let name = |column: &str| -> trial::Result<String> {
            let text = doc[column].as_str().ok_or("missing worker name")?;
            let words: Vec<_> = text
                .split(|ch: char| !ch.is_ascii_alphanumeric())
                .filter(|word| !word.is_empty())
                .collect();
            if words.is_empty() {
                return Err("empty worker name".into());
            }
            Ok(words.join("_").to_ascii_lowercase())
        };
        let prior = name("reference")?;
        let revised = name("revised")?;
        if prior == revised {
            return Err("ambiguous worker names".into());
        }
        Ok(match self {
            Self::Reference => [format!("{prior}_selected"), format!("{revised}_trial")],
            Self::Revised => [format!("{prior}_prior"), format!("{revised}_selected")],
        })
    }
}
#[derive(Deserialize)]
struct Timings {
    #[serde(skip)]
    labels: [String; 2],
    reference_p95_us: f64,
    weighted_p95_us: f64,
    pairs: usize,
}
impl Timings {
    fn read(path: &Path, layout: &str, args: &Args) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let (reference, window) = if args.replay_policy.is_some() {
            ("Shared queue", "formation")
        } else if args.landing {
            ("Weighted static split", "formation")
        } else {
            ("World count", "change")
        };
        if args.selected_worker.is_none() && doc["reference"] != reference {
            return Err(format!("expected benchmark reference {reference}").into());
        }
        let w = doc
            .pointer(&format!("/combined/{layout}/windows/{window}"))
            .ok_or("missing requested benchmark timings")?;
        if args.landing && w["ticks_per_run"] != 24 {
            return Err("expected six four-tick landing windows".into());
        }
        let mut t: Self = serde_json::from_value(w.clone())?;
        t.labels = if let Some(selected) = args.selected_worker {
            selected.labels(&doc)?
        } else if let Some(policy) = args.replay_policy {
            policy.labels().map(str::to_owned)
        } else if args.landing {
            ["prior", "revised"].map(str::to_owned)
        } else {
            ["count", "weighted"].map(str::to_owned)
        };
        if t.pairs == 0
            || [t.reference_p95_us, t.weighted_p95_us]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.)
        {
            return Err("invalid paired timing summary".into());
        }
        Ok(t)
    }

    fn draw(&self, view: &mut View, landing: bool) {
        let max = (self.reference_p95_us.max(self.weighted_p95_us) / 100.).ceil() * 100.;
        let labels = self.labels.each_ref().map(String::as_str);
        let window = if landing {
            "landing_window"
        } else {
            "transition"
        };
        for (i, (name, val, color)) in [
            (labels[0], self.reference_p95_us, [0.87, 0.57, 0.11, 1.]),
            (labels[1], self.weighted_p95_us, [0.10, 0.65, 0.71, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let z = 1.0 - 0.5 * i as f64;
            view.cuboid(
                20_000 + 4 * i as u64,
                &format!("{name}_p95_scale_{max:.0}_us"),
                [0., -5., z],
                [8., 0.06, 0.06],
                [0.53, 0.54, 0.57, 1.],
            );
            let span = 8. * val / max;
            view.cuboid(
                20_002 + 4 * i as u64,
                &format!(
                    "{name}_fixed_placement_{window}_p95_{val:.1}_us_{}_pairs",
                    self.pairs
                ),
                [(span - 8.) / 2., -5., z],
                [span, 0.19, 0.23],
                color,
            );
        }
    }
}
#[derive(Deserialize)]
struct LaneReport {
    lane_states: Vec<LaneState>,
}
#[derive(Deserialize)]
struct LaneState {
    tick: u64,
    helper_base_ids: Vec<u64>,
    input_pair_receipts: [usize; 2],
    positive_pairs: [usize; 2],
}
#[derive(Deserialize)]
struct Span {
    name: String,
    us: f64,
}
#[derive(Deserialize)]
struct WorstLanding {
    tick: u64,
    gate_worker: u8,
    run: usize,
    total_us: f64,
    spans: Vec<Span>,
}
impl WorstLanding {
    fn read(path: &Path, layout: &str) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let row: Self =
            serde_json::from_value(doc.get(layout).ok_or("missing landing layout")?.clone())?;
        let sum: f64 = row.spans.iter().map(|s| s.us).sum();
        if !fixture::LAUNCHES.iter().any(|t| t + 60 == row.tick)
            || row.gate_worker >= 4
            || row.run == 0
            || row.spans.is_empty()
            || row.spans.len() > 7
            || !row.total_us.is_finite()
            || row.total_us <= 0.
            || row
                .spans
                .iter()
                .any(|s| !s.us.is_finite() || s.us < 0. || s.name.is_empty())
            || (sum - row.total_us).abs() > 0.05
        {
            return Err("invalid exclusive landing spans".into());
        }
        Ok(row)
    }
    fn draw(&self, v: &mut View, row: usize, scale_us: f64, name: &str) {
        let colors = [
            [0.31, 0.56, 0.80, 1.],
            [0.64, 0.41, 0.72, 1.],
            [0.33, 0.69, 0.56, 1.],
            [0.81, 0.37, 0.23, 1.],
            [0.35, 0.58, 0.35, 1.],
            [0.81, 0.59, 0.21, 1.],
            [0.42, 0.49, 0.59, 1.],
        ];
        let mut left = -4.;
        for (i, s) in self.spans.iter().enumerate() {
            let width = 8. * s.us / scale_us;
            if width > 0. {
                v.cuboid(
                    50_000 + row as u64 * 100 + i as u64 * 2,
                    &format!(
                        "{name}_profile_run_{}_tick_{}_gate_worker_{}_{}_{:.2}_of_{:.2}_us",
                        self.run,
                        self.tick,
                        self.gate_worker,
                        s.name.replace(' ', "_"),
                        s.us,
                        self.total_us
                    ),
                    [left + width / 2., -5., 2.0 - 0.5 * row as f64],
                    [width, 0.2, 0.23],
                    colors[i],
                );
            }
            left += width;
        }
    }
}

#[derive(Deserialize)]
struct HelperWake {
    tick: u64,
    run: usize,
    worker: u8,
    lane: String,
    helper_dispatch_us: f64,
    helper_runqueue_us: Option<f64>,
    helper_residual_us: Option<f64>,
}
impl HelperWake {
    fn read(path: &Path, layout: &str, sample: &WorstLanding) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let h: Self =
            serde_json::from_value(doc.get(layout).ok_or("missing helper sample")?.clone())?;
        if h.tick != sample.tick
            || h.run != sample.run
            || h.worker != 0
            || h.lane != "helper"
            || !h.helper_dispatch_us.is_finite()
            || h.helper_dispatch_us <= 0.
        {
            return Err("helper sample disagrees with the baseline touchdown".into());
        }
        match (h.helper_runqueue_us, h.helper_residual_us) {
            (Some(q), Some(r))
                if [q, r].iter().all(|x| x.is_finite() && *x >= 0.)
                    && (q + r - h.helper_dispatch_us).abs() < 0.005 =>
            {
                Ok(h)
            }
            (None, None) => Ok(h),
            _ => Err("invalid helper dispatch decomposition".into()),
        }
    }
    fn draw(&self, v: &mut View, row: usize, name: &str, scale: f64) {
        let components = match (self.helper_runqueue_us, self.helper_residual_us) {
            (Some(q), Some(r)) => [("kernel_runqueue", q), ("residual", r)],
            _ => [("dispatch_wall_only", self.helper_dispatch_us), ("", 0.)],
        };
        let mut left = -4.;
        for (i, (stage, us)) in components.into_iter().enumerate() {
            if us > 0. {
                let width = 8. * us / scale;
                v.cuboid(
                    51_000 + 100 * row as u64 + 2 * i as u64,
                    &format!(
                        "{name}_helper_dispatch_run_{}_tick_{}_worker_{}_{}_{:.2}_of_{:.2}_us",
                        self.run, self.tick, self.worker, stage, us, self.helper_dispatch_us
                    ),
                    [left + width / 2., -5., 3.1 - 0.5 * row as f64],
                    [width, 0.2, 0.23],
                    [[0.71, 0.39, 0.74, 1.], [0.52, 0.61, 0.66, 1.]][i],
                );
                left += width;
            }
        }
    }
}

#[derive(Deserialize)]
struct PostSolve {
    #[serde(skip)]
    all_tick_class: Option<String>,
    run: usize,
    tick: u64,
    worker: u8,
    gate_worker: u8,
    public_us: f64,
    join_us: f64,
    spans: [Span; 3],
    both_tasks_done_to_scope_return: CallerAccounting,
}

#[derive(Deserialize)]
struct CallerAccounting {
    wall_us: f64,
    cpu_us: Option<f64>,
    runqueue_raw_us: Option<f64>,
    residual_us: Option<f64>,
}
impl CallerAccounting {
    fn parts(&self) -> trial::Result<Option<[f64; 3]>> {
        let Some(rest) = self.residual_us else {
            return Ok(None);
        };
        let parts = [
            self.cpu_us.ok_or("missing caller CPU")?,
            self.runqueue_raw_us.ok_or("missing caller runqueue")?,
            rest,
        ];
        if parts.iter().any(|v| !v.is_finite() || *v < 0.)
            || (parts.iter().sum::<f64>() - self.wall_us).abs() > 0.005
        {
            return Err("invalid scope return accounting".into());
        }
        Ok(Some(parts))
    }
}
impl PostSolve {
    fn read(path: &Path, layout: &str) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let row: Self =
            serde_json::from_value(doc.get(layout).ok_or("missing post-solve layout")?.clone())?;
        if !fixture::LAUNCHES.iter().any(|t| t + 60 == row.tick) {
            return Err("expected a touchdown in the landing-only sample".into());
        }
        row.validated()
    }

    fn read_all_ticks(path: &Path, layout: &str) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let pick = &doc[layout]["largest_post_tasks"];
        let mut row: Self = serde_json::from_value(pick["observation"].clone())?;
        let tick = row.tick;
        if !(fixture::SETTLED + 1..=fixture::END).contains(&tick) {
            return Err("scope return outside measured ticks".into());
        }
        let class = fixture::LAUNCHES
            .iter()
            .find_map(|&launch| match tick.checked_sub(launch) {
                Some(0) => Some("release"),
                Some(1..=3) => Some("after release"),
                Some(60) => Some("landing"),
                Some(61..=63) => Some("after landing"),
                _ => None,
            })
            .unwrap_or("steady");
        if pick["class"] != class {
            return Err("scope return event class disagrees with the fixture tick".into());
        }
        row.all_tick_class = Some(class.replace(' ', "_"));
        row.validated()
    }

    fn validated(self) -> trial::Result<Self> {
        let row = self;
        let labels = [
            "Last world to lane completion",
            "Lanes complete to scope return",
            "Scope return to batch end",
        ];
        let sum: f64 = row.spans.iter().map(|s| s.us).sum();
        if row.run == 0
            || row.worker != 0
            || row.gate_worker >= 4
            || !row.public_us.is_finite()
            || !row.join_us.is_finite()
            || row.join_us <= 0.
            || row.public_us < row.join_us
            || row
                .spans
                .iter()
                .zip(labels)
                .any(|(s, label)| s.name != label || !s.us.is_finite() || s.us < 0.)
            || (sum - row.join_us).abs() > 0.005
        {
            return Err("invalid post-solve stage accounting".into());
        }
        if !row.both_tasks_done_to_scope_return.wall_us.is_finite()
            || (row.both_tasks_done_to_scope_return.wall_us - row.spans[1].us).abs() > 0.005
        {
            return Err("scope return interval differs from wall stages".into());
        }
        row.both_tasks_done_to_scope_return.parts()?;
        Ok(row)
    }

    fn draw_rows(
        samples: &[Option<Self>; 2],
        view: &mut View,
        names: [&str; 2],
    ) -> trial::Result<()> {
        let scale = samples
            .iter()
            .flatten()
            .map(|s| s.join_us)
            .fold(0., f64::max);
        for (row, entry) in samples.iter().enumerate() {
            if let Some(sample) = entry {
                sample.draw(view, row, names[row], scale)?;
            }
        }
        Ok(())
    }

    fn draw(&self, view: &mut View, row: usize, policy: &str, scale: f64) -> trial::Result<()> {
        let mut left = -4.;
        let (label, base_id) = match &self.all_tick_class {
            Some(class) => (format!("all_ticks_scope_return_{class}"), 53_000),
            None => ("post_solve".to_owned(), 52_000),
        };
        let mut segment = |index: u64, stage: &str, us: f64, color| {
            let width = 8. * us / scale;
            if width > 0. {
                view.cuboid(
                    base_id + row as u64 * 100 + index * 2,
                    &format!(
                        "{policy}_{label}_run_{}_tick_{}_worker_{}_{}_{us:.2}_of_{:.2}_us",
                        self.run, self.tick, self.worker, stage, self.join_us
                    ),
                    [left + width / 2., -5., 3.1 - 0.5 * row as f64],
                    [width, 0.2, 0.23],
                    color,
                );
                left += width;
            }
        };
        segment(
            0,
            "last_world_to_lane_completion",
            self.spans[0].us,
            [0.17, 0.64, 0.68, 1.],
        );
        if let Some([cpu, queue, residual]) = self.both_tasks_done_to_scope_return.parts()? {
            segment(1, "scope_return_cpu", cpu, [0.89, 0.47, 0.24, 1.]);
            segment(2, "scope_return_runnable", queue, [0.69, 0.35, 0.76, 1.]);
            segment(3, "scope_return_residual", residual, [0.54, 0.61, 0.67, 1.]);
        } else {
            segment(
                4,
                "scope_return_wall",
                self.spans[1].us,
                [0.61, 0.51, 0.69, 1.],
            );
        }
        segment(
            5,
            "scope_return_to_batch_end",
            self.spans[2].us,
            [0.29, 0.55, 0.81, 1.],
        );
        Ok(())
    }
}

struct Measurements {
    named_profiles: bool,
    timings: Option<Timings>,
    profiles: [Option<WorstLanding>; 2],
    join: [Option<PostSolve>; 2],
    all_ticks: [Option<PostSolve>; 2],
    wake: [Option<HelperWake>; 2],
}
struct Display {
    offsets: BTreeMap<u64, [f64; 2]>,
    pads: [[f64; 2]; 2],
    base_ids: Vec<u64>,
    max_id: u64,
    schedule: trial::Schedule,
    landing: bool,
}
impl Display {
    fn new(groups: &[Vec<Body>], schedule: trial::Schedule, landing: bool) -> Self {
        let mid: [f64; 2] = std::array::from_fn(|a| {
            groups.iter().map(|g| g[0].position[a]).sum::<f64>() / groups.len() as f64
        });
        let mut offsets = BTreeMap::new();
        let mut pads = [[0.; 2]; 2];
        let mut pad_centers = pads.iter_mut();
        for g in groups {
            let center: [f64; 2] = std::array::from_fn(|a| 0.44 * (g[0].position[a] - mid[a]));
            let offset = std::array::from_fn(|a| center[a] - g[0].position[a]);
            for b in g {
                offsets.insert(b.id, offset);
            }
            if g.len() == 25 {
                *pad_centers.next().expect("two cargo groups") = center;
            }
        }
        Self {
            max_id: *offsets.keys().last().unwrap(),
            offsets,
            schedule,
            landing,
            pads,
            base_ids: groups.iter().map(|g| g[0].id).collect(),
        }
    }
    fn frame(
        &self,
        f: &trial::Frame<'_>,
        prior: [usize; 2],
        measurements: &Measurements,
        observed: Option<&LaneState>,
    ) -> trial::Result<(String, String)> {
        let mut v = View::default();
        v.cuboid(
            2,
            "ground",
            [0., 0., -0.055],
            [24., 14., 0.11],
            [0.31, 0.36, 0.42, 1.],
        );
        for (b, &owner) in f.bodies.iter().zip(&f.owners) {
            let geom = match b.primitive {
                Primitive::Sphere => json!({"kind":"sphere","radius":b.size[0]/2.}),
                Primitive::Box => json!({"kind":"box","size":b.size}),
                _ => return Err("unsupported fixture shape".into()),
            };
            let [dx, dy] = self.offsets[&b.id];
            let color = if owner == 0 {
                [0.14, 0.48, 0.86, 1.]
            } else {
                [0.98, 0.50, 0.11, 1.]
            };
            v.add(
                8 + 2 * (b.id + self.max_id * u64::from(owner)),
                &format!("body_{}_owner_{owner}", b.id),
                [b.position[0] + dx, b.position[1] + dy, b.position[2]],
                b.orientation,
                geom,
                color,
            );
        }
        if let Some(lane) = observed {
            if lane.input_pair_receipts != prior || lane.positive_pairs != f.loaded_pairs {
                return Err(format!("lane report disagrees with physics at {}", f.tick).into());
            }
            for &id in &lane.helper_base_ids {
                if !self.base_ids.contains(&id) {
                    return Err("unknown helper world".into());
                }
                let b = f
                    .bodies
                    .iter()
                    .find(|b| b.id == id)
                    .ok_or("missing helper world")?;
                let [dx, dy] = self.offsets[&id];
                v.cuboid(
                    40_000 + id * 2,
                    &format!("world_{id}_on_helper_thread"),
                    [
                        b.position[0] + dx,
                        b.position[1] + dy + b.size[1] / 2. + 0.4,
                        0.2,
                    ],
                    [0.9, 0.32, 0.4],
                    [0.64, 0.27, 0.77, 1.],
                );
            }
        }
        for (i, (&now, &prior)) in f.loaded_pairs.iter().zip(&prior).enumerate() {
            let [x, y] = self.pads[i];
            for (j, (name, n, color)) in [
                (
                    if self.landing {
                        "previous_pair_receipts"
                    } else {
                        "cost_input_previous_pairs"
                    },
                    prior,
                    [0.91, 0.62, 0.12, 1.],
                ),
                ("loaded_pairs", now, [0.1, 0.75, 0.78, 1.]),
            ]
            .into_iter()
            .enumerate()
            {
                let x = x - 0.3 + 0.6 * j as f64;
                let y = y - 2.;
                let id = 10_000 + (i * 8 + j * 4) as u64;
                v.cuboid(
                    id,
                    &format!("pad_{}_{name}_scale_24", i + 1),
                    [x, y, 1.0],
                    [0.10, 0.10, 2.],
                    [0.46, 0.48, 0.52, 1.],
                );
                // The pointer rests on the ground for zero contacts, reaches the top for 24.
                v.cuboid(
                    id + 2,
                    &format!("pad_{}_{name}", i + 1),
                    [x, y, 2. * (n as f64 / 24.)],
                    [0.48, 0.30, 0.12],
                    color,
                );
            }
        }
        let names = match (&measurements.timings, measurements.named_profiles) {
            (Some(t), true) => t.labels.each_ref().map(String::as_str),
            _ => ["baseline", "revised"],
        };
        if (40..fixture::SETTLED).contains(&f.tick) {
            PostSolve::draw_rows(&measurements.all_ticks, &mut v, names)?;
        }
        if f.tick >= fixture::SETTLED && f.tick < self.schedule.migrate {
            if let Some(t) = &measurements.timings {
                t.draw(&mut v, self.landing);
            }
            let scale = measurements
                .profiles
                .iter()
                .flatten()
                .map(|p| p.total_us)
                .fold(0., f64::max);
            for (row, sample) in measurements.profiles.iter().enumerate() {
                if let Some(sample) = sample {
                    sample.draw(&mut v, row, scale, names[row]);
                }
            }
            PostSolve::draw_rows(&measurements.join, &mut v, names)?;
            let wake_scale = measurements
                .wake
                .iter()
                .flatten()
                .map(|h| h.helper_dispatch_us)
                .fold(0., f64::max);
            for (row, wake) in measurements.wake.iter().enumerate() {
                if let Some(h) = wake {
                    h.draw(&mut v, row, names[row], wake_scale);
                }
            }
        }
        for (i, (tick, name)) in self.schedule.events().into_iter().enumerate() {
            if f.tick >= tick {
                v.cuboid(
                    30_000 + 2 * i as u64,
                    &format!("{name}_tick_{tick}"),
                    [-2.4 + 1.6 * i as f64, -5., 0.3],
                    [0.8, 0.8, 0.6],
                    [0.16, 0.73, 0.48, 1.],
                );
            }
        }
        Ok(v.finish(f.tick))
    }
}
fn plot(
    path: &Path,
    counts: &[([usize; 2], [usize; 2])],
    schedule: trial::Schedule,
) -> trial::Result<()> {
    let area = SVGBackend::new(path, (1080, 480)).into_drawing_area();
    area.fill(&WHITE)?;
    let area = area.titled(
        "Cargo contacts release and reform across recovery",
        ("sans-serif", 24),
    )?;
    for (i, row) in area.split_evenly((2, 1)).into_iter().enumerate() {
        let mut chart = ChartBuilder::on(&row)
            .margin_right(42)
            .margin_left(13)
            .margin_top(8)
            .margin_bottom(4)
            .x_label_area_size(36)
            .y_label_area_size(65)
            .build_cartesian_2d(0.65_f64..fixture::END as f64 / 100., 0.0_f64..33.)?;
        chart
            .configure_mesh()
            .disable_x_mesh()
            .max_light_lines(1)
            .x_desc("Simulation time (s)")
            .y_desc(format!("Pad {} pairs", i + 1))
            .draw()?;
        for (t, name) in schedule.events() {
            let x = t as f64 / 100.;
            chart.draw_series([PathElement::new(
                [(x, 0.), (x, 27.)],
                RGBColor(149, 155, 164),
            )])?;
            if i == 0 {
                chart.draw_series([Text::new(name, (x, 30.), ("sans-serif", 14))])?;
            }
        }
        for (predicted, label, color) in [
            (true, "Previous tick receipts", RGBColor(205, 130, 25)),
            (false, "Loaded pairs", RGBColor(12, 135, 150)),
        ] {
            let points = (fixture::SETTLED as usize - 3..counts.len()).flat_map(|t| {
                let x = t as f64 / 100.;
                let value = |t: usize| {
                    if predicted {
                        counts[t - 1].0[i]
                    } else {
                        counts[t].1[i]
                    }
                };
                [(x, value(t - 1) as f64), (x, value(t) as f64)]
            });
            chart
                .draw_series(LineSeries::new(
                    points,
                    color.stroke_width(if predicted { 5 } else { 2 }),
                ))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 20, y)], color.stroke_width(3))
                });
        }
        if i == 1 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperRight)
                .background_style(WHITE.mix(0.9))
                .draw()?;
        }
    }
    area.present()?;
    Ok(())
}
fn main() -> trial::Result<()> {
    let args = Args::parse();
    let schedule = if args.landing {
        trial::LANDING
    } else {
        trial::CHANGING
    };
    let variant = args.layout.to_possible_value().expect("named layout");
    let layout = variant.get_name();
    let timings = args
        .timings
        .as_deref()
        .map(|p| Timings::read(p, layout, &args))
        .transpose()?;
    let worst = args
        .worst_landings
        .as_deref()
        .map(|p| WorstLanding::read(p, layout))
        .transpose()?;
    let revised = args
        .revised_worst_landings
        .as_deref()
        .map(|p| WorstLanding::read(p, layout))
        .transpose()?;
    let read_wake = |path: Option<&Path>,
                     profile: Option<&WorstLanding>|
     -> trial::Result<Option<HelperWake>> {
        path.map(|p| {
            HelperWake::read(
                p,
                layout,
                profile.ok_or("matching landing profile required")?,
            )
        })
        .transpose()
    };
    let wake = [
        read_wake(args.helper_wake.as_deref(), worst.as_ref())?,
        read_wake(args.revised_helper_wake.as_deref(), revised.as_ref())?,
    ];
    let measurements = Measurements {
        timings,
        profiles: [worst, revised],
        named_profiles: args.replay_policy.is_some() || args.selected_worker.is_some(),
        all_ticks: [
            args.all_tick_joins
                .as_deref()
                .map(|p| PostSolve::read_all_ticks(p, layout))
                .transpose()?,
            args.revised_all_tick_joins
                .as_deref()
                .map(|p| PostSolve::read_all_ticks(p, layout))
                .transpose()?,
        ],
        join: [
            args.post_solve
                .as_deref()
                .map(|p| PostSolve::read(p, layout))
                .transpose()?,
            args.revised_post_solve
                .as_deref()
                .map(|p| PostSolve::read(p, layout))
                .transpose()?,
        ],
        wake,
    };
    let lanes: Option<LaneReport> = args
        .lane_report
        .as_ref()
        .map(|p| -> trial::Result<_> { Ok(serde_json::from_reader(File::open(p)?)?) })
        .transpose()?;
    if let Some(r) = &lanes
        && (r.lane_states.first().map(|s| s.tick) != Some(fixture::SETTLED + 1)
            || r.lane_states.windows(2).any(|w| w[0].tick >= w[1].tick))
    {
        return Err("expected ordered settled lane measurements".into());
    }
    fs::create_dir(&args.directory)?;
    let groups = fixture::seeds(args.layout);
    let view = Display::new(&groups, schedule, args.landing);
    let mut trace = BufWriter::new(File::create_new(args.directory.join("distributed.jsonl"))?);
    let mut stream = Vec::new();
    let mut counts: Vec<([usize; 2], [usize; 2])> = Vec::new();
    let report = trial::qualify(
        &args.worker,
        &args.directory.join("native.gzpool"),
        &groups,
        schedule,
        |f| {
            let observed = lanes
                .as_ref()
                .filter(|_| f.tick < schedule.migrate)
                .and_then(|r| r.lane_states.iter().rev().find(|s| s.tick <= f.tick));
            stream.push(view.frame(
                f,
                counts.last().map_or([0; 2], |c| c.0),
                &measurements,
                observed,
            )?);
            counts.push((f.pad_receipts, f.loaded_pairs));
            serde_json::to_writer(&mut trace, f)?;
            writeln!(&mut trace)?;
            Ok(())
        },
    )?;
    trace.flush()?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    if args.landing {
        use sha2::{Digest, Sha256};
        let source_hash = |p: &PathBuf| -> trial::Result<String> {
            Ok(format!("{:x}", Sha256::digest(fs::read(p)?)))
        };
        let inputs = json!({
            "layout":layout,
            "selected_worker":args.selected_worker.map(|w| w.to_possible_value().unwrap().get_name().to_owned()),
            "all_tick_join_summary_sha256":args.all_tick_joins.as_ref().map(source_hash).transpose()?,
            "revised_all_tick_join_summary_sha256":args.revised_all_tick_joins.as_ref().map(source_hash).transpose()?,
            "post_solve_summary_sha256":args.post_solve.as_ref().map(source_hash).transpose()?,
            "revised_post_solve_summary_sha256":args.revised_post_solve.as_ref().map(source_hash).transpose()?,
            "worker_sha256":source_hash(&args.worker)?,
            "replay_policy":args.replay_policy.map(|p| p.to_possible_value().unwrap().get_name().to_owned()),
            "revised_helper_wake_summary_sha256":args.revised_helper_wake.as_ref().map(source_hash).transpose()?,
            "helper_wake_summary_sha256":args.helper_wake.as_ref().map(source_hash).transpose()?,
            "window":"six landings, touchdown and following three ticks",
            "timing_summary_sha256":args.timings.as_ref().map(source_hash).transpose()?,
            "worst_landing_summary_sha256":args.worst_landings.as_ref().map(source_hash).transpose()?,
            "revised_worst_landing_summary_sha256":args.revised_worst_landings.as_ref().map(source_hash).transpose()?,
        });
        fs::write(
            args.directory.join("measurement-sources.json"),
            serde_json::to_vec_pretty(&inputs)?,
        )?;
    }
    plot(
        &args.directory.join("changing-contacts.svg"),
        &counts,
        schedule,
    )?;
    let summary = serde_json::to_string_pretty(&report)?;
    fs::write(args.directory.join("qualification.json"), &summary)?;
    println!("{summary}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn post_solve_stages_close_on_a_touchdown_and_use_distinct_policy_names() -> trial::Result<()> {
        let dir = tempfile::tempdir()?;
        let path = dir.path().join("join.json");
        let mut row = json!({
            "tick":140,"run":2,"worker":0,"gate_worker":1,"public_us":99.0,"join_us":3.0,
            "both_tasks_done_to_scope_return":{"wall_us":2.0,"cpu_us":0.5,"runqueue_raw_us":0.5,"residual_us":1.0},
            "spans":[{"name":"Last world to lane completion","us":0.0},
                     {"name":"Lanes complete to scope return","us":2.0},
                     {"name":"Scope return to batch end","us":1.0}]
        });
        let read = |row: &Value| -> trial::Result<PostSolve> {
            fs::write(&path, serde_json::to_vec(&json!({"adjacent":row}))?)?;
            PostSolve::read(&path, "adjacent")
        };
        assert_eq!(read(&row)?.join_us, 3.);
        row["both_tasks_done_to_scope_return"]["residual_us"] = json!(2.);
        assert!(read(&row).is_err());
        row["both_tasks_done_to_scope_return"]["residual_us"] = Value::Null;
        assert!(
            read(&row)?
                .both_tasks_done_to_scope_return
                .parts()?
                .is_none()
        );
        row["join_us"] = json!(11.);
        assert!(read(&row).is_err());
        row["join_us"] = json!(3.);
        row["tick"] = json!(141);
        assert!(read(&row).is_err());
        let mut all_ticks =
            json!({"adjacent":{"largest_post_tasks":{"class":"steady", "observation":row}}});
        fs::write(&path, serde_json::to_vec(&all_ticks)?)?;
        assert!(PostSolve::read_all_ticks(&path, "adjacent").is_err());
        all_ticks["adjacent"]["largest_post_tasks"]["class"] = json!("after landing");
        fs::write(&path, serde_json::to_vec(&all_ticks)?)?;
        assert_eq!(
            PostSolve::read_all_ticks(&path, "adjacent")?
                .all_tick_class
                .as_deref(),
            Some("after_landing")
        );
        let report = json!({"reference":"Shared queue","revised":"Inline caller"});
        assert_eq!(
            SelectedWorker::Revised.labels(&report)?,
            ["shared_queue_prior", "inline_caller_selected"]
        );
        assert!(
            SelectedWorker::Reference
                .labels(&json!({"reference":"a-b","revised":"A B"}))
                .is_err()
        );
        Ok(())
    }

    #[test]
    fn wake_segments_require_a_matching_touchdown_and_complete_accounting() -> trial::Result<()> {
        let dir = tempfile::tempdir()?;
        let path = dir.path().join("wake.json");
        let profile = WorstLanding {
            tick: 140,
            run: 1,
            gate_worker: 3,
            total_us: 100.,
            spans: Vec::new(),
        };
        let mut row =
            json!({"tick":140,"run":1,"worker":0,"lane":"helper","helper_dispatch_us":10.});
        let read = |row: &Value| -> trial::Result<HelperWake> {
            fs::write(&path, serde_json::to_vec(&json!({"adjacent":row}))?)?;
            HelperWake::read(&path, "adjacent", &profile)
        };
        assert!(read(&row)?.helper_runqueue_us.is_none());
        row["helper_runqueue_us"] = json!(4.);
        assert!(read(&row).is_err());
        row["helper_residual_us"] = json!(7.);
        assert!(read(&row).is_err());
        row["helper_residual_us"] = json!(6.);
        assert_eq!(read(&row)?.helper_runqueue_us, Some(4.));
        row["tick"] = json!(220);
        assert!(read(&row).is_err());
        Ok(())
    }
}
