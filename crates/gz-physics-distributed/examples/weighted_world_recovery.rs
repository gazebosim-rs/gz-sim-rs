// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay measured contact loads and fault events for the first-heavy worker benchmark.
#[path = "support/heterogeneous_worlds.rs"]
mod fixture;
#[path = "support/weighted_contact_plot.rs"]
mod plot;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[path = "support/local_world_recovery.rs"]
mod trial;
use clap::Parser;
use gz_physics_distributed::physics::{Body, Primitive};
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};

const _: () = assert!(trial::SETTLED == fixture::WARMUP as u64);

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Optional matched timing summary for this pre-handoff layout.
    #[arg(long)]
    timings: Option<PathBuf>,
    /// Fresh output directory for the trace, native checkpoint and gz-web-rs recording.
    directory: PathBuf,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Timings {
    layout: String,
    rounds: u32,
    baseline_p95_us: f64,
    weighted_p95_us: f64,
}

impl Timings {
    fn validate(&self) -> trial::Result<()> {
        if self.layout != "first"
            || self.rounds == 0
            || ![self.baseline_p95_us, self.weighted_p95_us]
                .iter()
                .all(|&x| x.is_finite() && x > 0.)
        {
            return Err(
                "expected positive paired p95 measurements of the first-heavy layout".into(),
            );
        }
        Ok(())
    }

    fn draw(&self, view: &mut View) {
        let span = 8.;
        let limit = (self.baseline_p95_us.max(self.weighted_p95_us) / 100.).ceil() * 100.;
        for (i, (label, value, color)) in [
            ("previous", self.baseline_p95_us, [0.94, 0.53, 0.12, 1.]),
            ("weighted", self.weighted_p95_us, [0.08, 0.69, 0.70, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let z = 1.0 - i as f64 * 0.55;
            let width = span * value / limit;
            view.cuboid(
                20_000 + 4 * i as u64,
                &format!("{label}_timing_scale_0_to_{limit:.0}_us"),
                [0., -5., z],
                [span, 0.09, 0.10],
                [0.50, 0.52, 0.55, 1.],
            );
            view.cuboid(
                20_002 + 4 * i as u64,
                &format!(
                    "{label}_{}_tick_p95_{value:.1}_us_median_of_{}",
                    fixture::TICKS,
                    self.rounds
                ),
                [(width - span) / 2., -5., z],
                [width, 0.16, 0.26],
                color,
            );
        }
    }
}

struct Display {
    offsets: BTreeMap<u64, [f64; 2]>,
    base_ids: Vec<u64>,
    ground_scale: f64,
    max_body_id: u64,
}

impl Display {
    fn new(groups: &[Vec<Body>]) -> Self {
        // Translate each entire independent world by a constant vector to fit
        // all eight on screen. Within-world poses, shape sizes and gravity are exact.
        let center: [f64; 2] = std::array::from_fn(|a| {
            groups.iter().map(|g| g[0].position[a]).sum::<f64>() / groups.len() as f64
        });
        let mut offsets = BTreeMap::new();
        for group in groups {
            let offset = std::array::from_fn(|a| {
                0.44 * (group[0].position[a] - center[a]) - group[0].position[a]
            });
            for b in group {
                offsets.insert(b.id, offset);
            }
        }
        let max_mass = groups
            .iter()
            .map(|g| g.iter().map(|b| b.mass).sum::<f64>())
            .fold(0., f64::max);
        Self {
            max_body_id: *offsets.keys().last().unwrap(),
            offsets,
            base_ids: groups.iter().map(|g| g[0].id).collect(),
            ground_scale: (max_mass * fixture::GRAVITY[2].abs() * 0.01 * 1.2 * 10.).ceil() / 10.,
        }
    }

    fn frame(
        &self,
        f: &trial::Frame<'_>,
        timings: Option<&Timings>,
    ) -> trial::Result<(String, String)> {
        let mut view = View::default();
        view.cuboid(
            2,
            "shared_ground",
            [0., 0., -0.055],
            [24., 13., 0.11],
            [0.31, 0.36, 0.42, 1.],
        );
        for (b, &owner) in f.bodies.iter().zip(&f.owners) {
            let shape = match b.primitive {
                Primitive::Sphere => json!({"kind":"sphere","radius":b.size[0]*0.5}),
                Primitive::Box => json!({"kind":"box","size":b.size}),
                _ => return Err("unsupported benchmark primitive".into()),
            };
            let [dx, dy] = self.offsets[&b.id];
            let pos = [b.position[0] + dx, b.position[1] + dy, b.position[2]];
            let color = if owner == 0 {
                [0.14, 0.48, 0.86, 1.]
            } else {
                [0.98, 0.50, 0.11, 1.]
            };
            view.add(
                8 + 2 * (b.id + self.max_body_id * u64::from(owner)),
                &format!("body_{}_owner_{owner}", b.id),
                pos,
                b.orientation,
                shape,
                color,
            );
        }
        if f.tick >= trial::SETTLED {
            for (i, &base) in self.base_ids.iter().enumerate() {
                let b = f
                    .bodies
                    .iter()
                    .find(|b| b.id == base)
                    .ok_or("missing base")?;
                let load = f
                    .floors
                    .iter()
                    .find(|p| p.body == base)
                    .ok_or("missing settled floor impulse")?
                    .normal_impulse;
                let [dx, dy] = self.offsets[&base];
                let x = b.position[0] + dx;
                let y = b.position[1] + dy - b.size[1] * 0.5 - 0.5;
                let id = 10_000 + 4 * i as u64;
                view.cuboid(
                    id,
                    &format!("base_{base}_scale_0_to_{:.1}_Ns", self.ground_scale),
                    [x, y, 1.],
                    [0.12, 0.12, 2.],
                    [0.53, 0.56, 0.59, 1.],
                );
                view.cuboid(
                    id + 2,
                    &format!("base_{base}_ground_impulse"),
                    [x, y, 2. * load / self.ground_scale],
                    [0.48, 0.25, 0.15],
                    [0.65, 0.30, 0.79, 1.],
                );
            }
        }
        if f.tick < trial::HANDOFF
            && let Some(t) = timings
        {
            t.draw(&mut view);
        }
        for (i, (at, name)) in [
            (trial::HANDOFF, "heavy_handoff"),
            (trial::RETRY, "ack_retry_committed"),
            (trial::REPLACEMENT, "new_owner_recovered"),
            (trial::RESTORE, "four_worker_archive_restored"),
        ]
        .into_iter()
        .enumerate()
        {
            if f.tick >= at {
                view.cuboid(
                    30_000 + 2 * i as u64,
                    &format!("{name}_tick_{at}"),
                    [-2.4 + 1.6 * i as f64, -5., 0.3],
                    [0.8, 0.8, 0.6],
                    [0.16, 0.73, 0.48, 1.],
                );
            }
        }
        Ok(view.finish(f.tick))
    }
}

fn main() -> trial::Result<()> {
    let args = Args::parse();
    let timings: Option<Timings> = args
        .timings
        .map(|p| -> trial::Result<_> { Ok(serde_json::from_reader(File::open(p)?)?) })
        .transpose()?;
    if let Some(t) = &timings {
        t.validate()?;
    }
    let layout = fixture::Layout::First;
    let groups = fixture::seeds(layout);
    let display = Display::new(&groups);
    fs::create_dir(&args.directory)?;
    let mut trace = BufWriter::new(File::create_new(args.directory.join("distributed.jsonl"))?);
    let mut stream = Vec::new();
    let mut points = Vec::new();
    let report = trial::qualify(
        &args.worker,
        &args.directory.join("native.gzpool"),
        groups,
        0,
        |f| {
            serde_json::to_writer(&mut trace, f)?;
            writeln!(trace)?;
            stream.push(display.frame(f, timings.as_ref())?);
            points.push(plot::Point::read(
                f,
                display.base_ids[0],
                display.base_ids[1],
            ));
            Ok(())
        },
    )?;
    trace.flush()?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    plot::write(&args.directory.join("contact-impulses.svg"), &points)?;
    if let Some(timings) = timings {
        fs::write(
            args.directory.join("timings.json"),
            serde_json::to_string_pretty(&timings)?,
        )?;
    }
    let json = serde_json::to_string_pretty(&report)?;
    fs::write(args.directory.join("qualification.json"), &json)?;
    println!("{json}");
    Ok(())
}
