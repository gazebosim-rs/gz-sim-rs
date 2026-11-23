// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two fault-injected distributed placements of the same moving loads in gz-web-rs.
#[path = "support/changing_worlds.rs"]
mod fixture;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[allow(dead_code)]
#[path = "support/changing_recovery.rs"]
mod trial;
use clap::{Parser, ValueEnum};
use fixture::{Layout, Placement};
use gz_physics_distributed::physics::{Body, Primitive};
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
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
    #[arg(long, value_enum, default_value_t = Layout::Adjacent)]
    layout: Layout,
    /// Overlay the paired fixed-placement landing-window results.
    #[arg(long)]
    timings: Option<PathBuf>,
    /// Compare a one-helper cap on the single owner with zero helpers on split owners.
    #[arg(long)]
    equal_lanes: bool,
    /// A second set of fixed-placement landing-window timings with no helpers on either arm.
    #[arg(long, requires_all = ["equal_lanes","timings"])]
    serial_control: Option<PathBuf>,
    /// Fresh output directory for both physical traces, native checkpoints and the combined replay.
    directory: PathBuf,
}

const NAMES: [&str; 2] = ["one_owner", "two_owners"];
const BLUE: [f64; 4] = [0.14, 0.48, 0.86, 1.];
const ORANGE: [f64; 4] = [0.98, 0.50, 0.11, 1.];
const COUNT: [f64; 4] = [0.09, 0.73, 0.65, 1.];

#[derive(Deserialize)]
struct Timings {
    reference_p95_us: f64,
    weighted_p95_us: f64,
    pairs: usize,
    ticks_per_run: usize,
    #[serde(skip)]
    labels: [&'static str; 2],
    #[serde(skip)]
    serial: bool,
}
impl Timings {
    fn read(path: &Path, layout: &str, equal_lanes: bool, serial: bool) -> trial::Result<Self> {
        let doc: Value = serde_json::from_reader(File::open(path)?)?;
        let (expected, labels) = match (equal_lanes, serial) {
            (_, true) => (
                ["One owner, 0 helper", "Two owners, no helpers"],
                [
                    "one_owner_zero_helper_control",
                    "two_owners_zero_helper_control",
                ],
            ),
            (true, false) => (
                ["One owner, 1 helper", "Two owners, no helpers"],
                ["one_owner_one_helper", "two_owners_no_helpers"],
            ),
            (false, false) => (["One owner", "Two owners"], NAMES),
        };
        if doc["reference"] != expected[0] || doc["revised"] != expected[1] {
            return Err("expected one-owner / two-owner benchmark".into());
        }
        let mut row: Self =
            serde_json::from_value(doc["combined"][layout]["windows"]["formation"].clone())?;
        if row.ticks_per_run != 24
            || row.pairs == 0
            || [row.reference_p95_us, row.weighted_p95_us]
                .iter()
                .any(|&x| !x.is_finite() || x <= 0.)
        {
            return Err("invalid paired formation window".into());
        }
        row.labels = labels;
        row.serial = serial;
        Ok(row)
    }

    fn draw(&self, view: &mut View, arm: usize, y: f64) {
        let times = [self.reference_p95_us, self.weighted_p95_us];
        let scale = (times[0].max(times[1]) / 100.).ceil() * 100.;
        let length = 8. * times[arm] / scale;
        let id = 16_000 + u64::from(self.serial) * 20 + arm as u64 * 4;
        view.cuboid(
            id + 2,
            &format!(
                "{}_landing_window_p95_{:.1}_us_{}_pairs_scale_{scale}_us",
                self.labels[arm], times[arm], self.pairs
            ),
            [(length - 8.) / 2., y, 0.55],
            [length, 0.24, 0.23],
            if self.serial {
                [0.56, 0.49, 0.82, 1.]
            } else {
                [0.82, 0.66, 0.24, 1.]
            },
        );
    }
}

struct Display {
    shifts: BTreeMap<u64, [f64; 2]>,
}

impl Display {
    fn new(groups: &[Vec<Body>]) -> Self {
        let mut shifts = BTreeMap::new();
        for g in groups {
            let center = g[0].position;
            let shift = [-0.56 * center[0], -0.56 * center[1]];
            shifts.extend(g.iter().map(|b| (b.id, shift)));
        }
        Self { shifts }
    }

    fn draw(
        &self,
        view: &mut View,
        f: &trial::Frame<'_>,
        arm: usize,
        timings: Option<&Timings>,
        control: Option<&Timings>,
    ) -> trial::Result<()> {
        let panel_y = [6.4, -6.4][arm];
        let name = NAMES[arm];
        if arm == 0 {
            view.cuboid(
                2,
                "shared_display_floor",
                [0., 0., -0.055],
                [24., 24., 0.11],
                [0.31, 0.36, 0.42, 1.],
            );
        }
        for (body, &owner) in f.bodies.iter().zip(&f.owners) {
            let shape = match body.primitive {
                Primitive::Box => json!({"kind":"box","size":body.size}),
                Primitive::Sphere => json!({"kind":"sphere","radius":body.size[0] / 2.}),
                _ => return Err("unexpected fixture shape".into()),
            };
            let [x, y] = self.shifts[&body.id];
            let pos = [
                body.position[0] + x,
                body.position[1] + y + panel_y,
                body.position[2],
            ];
            let color = *[BLUE, ORANGE]
                .get(usize::from(owner))
                .ok_or("unexpected owner")?;
            let id = 100 + 2 * (arm as u64 * 1_000 + body.id + 100 * u64::from(owner));
            view.add(
                id,
                &format!("{name}_body_{}_worker_{owner}", body.id),
                pos,
                body.orientation,
                shape,
                color,
            );
        }
        let measurement = match f.tick {
            40..fixture::SETTLED => control,
            tick if (fixture::SETTLED..trial::LANDING.migrate).contains(&tick) => timings,
            _ => None,
        };
        if let Some(t) = measurement {
            t.draw(view, arm, panel_y - 5.4);
        } else if arm == 0 {
            // The physical traces are compared for the complete scene every tick.
            // Share the pair of pointers across panels to keep all 124 bodies visible.
            for (i, count) in f.loaded_pairs.into_iter().enumerate() {
                view.cuboid(
                    12_000
                        + 100 * u64::from(f.tick >= trial::LANDING.migrate && timings.is_some())
                        + 2 * i as u64,
                    &format!("both_placements_pad_{}_loaded_contacts_out_of_24", i + 1),
                    [-1. + 2. * i as f64, 0., count as f64 / 12.],
                    [0.5, 0.35, 0.15],
                    COUNT,
                );
            }
        }
        if arm == 0
            && let Some((i, (tick, event))) = trial::LANDING
                .events()
                .into_iter()
                .enumerate()
                .rev()
                .find(|(_, (tick, _))| f.tick >= *tick)
        {
            view.cuboid(
                20_000 + 2 * i as u64,
                &format!("{event}_tick_{tick}"),
                [-4. + 1.5 * i as f64, 1.2, 0.28],
                [0.85, 0.6, 0.56],
                [0.85, 0.43, 0.23, 1.],
            );
        }
        Ok(())
    }
}

fn main() -> trial::Result<()> {
    let args = Args::parse();
    let layout = args.layout.to_possible_value().expect("named layout");
    let timings = args
        .timings
        .as_ref()
        .map(|p| Timings::read(p, layout.get_name(), args.equal_lanes, false))
        .transpose()?;
    let control = args
        .serial_control
        .as_ref()
        .map(|p| Timings::read(p, layout.get_name(), true, true))
        .transpose()?;
    let limits = if args.equal_lanes {
        [Some(1), Some(0)]
    } else {
        [None, None]
    };
    let groups = fixture::seeds(args.layout);
    let display = Display::new(&groups);
    fs::create_dir(&args.directory)?;
    let mut views = Vec::new();
    let mut states = Vec::new();
    for (arm, placement) in [Placement::Single, Placement::Split]
        .into_iter()
        .enumerate()
    {
        let dir = args.directory.join(NAMES[arm]);
        fs::create_dir(&dir)?;
        let mut trace = BufWriter::new(File::create_new(dir.join("distributed.jsonl"))?);
        let mut physics = Sha256::new();
        let report = trial::qualify_scenario(
            &args.worker,
            &dir.join("native.gzpool"),
            &groups,
            trial::Scenario {
                schedule: trial::LANDING,
                placement,
                helper_limit: limits[arm],
                split_wire: None,
            },
            |frame| {
                let tick = usize::try_from(frame.tick)?;
                let data =
                    bincode::serialize(&(frame.tick, &frame.bodies, &frame.pairs, &frame.floors))?;
                physics.update(&data);
                let digest: [u8; 32] = Sha256::digest(&data).into();
                if arm == 0 {
                    if tick != states.len() {
                        return Err("discontinuous reference clock".into());
                    }
                    states.push(digest);
                    views.push(View::default());
                } else if states.get(tick) != Some(&digest) {
                    return Err(format!("physical states or receipts differ at tick {tick}").into());
                }
                display.draw(
                    &mut views[tick],
                    frame,
                    arm,
                    timings.as_ref(),
                    control.as_ref(),
                )?;
                serde_json::to_writer(&mut trace, frame)?;
                writeln!(trace)?;
                Ok(())
            },
        )?;
        trace.flush()?;
        fs::write(
            dir.join("qualification.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
        fs::write(
            dir.join("physics.sha256"),
            format!("{:x}\n", physics.finalize()),
        )?;
    }
    if states.len() != fixture::END as usize + 1 {
        return Err("incomplete paired replay".into());
    }
    let stream = views
        .into_iter()
        .enumerate()
        .map(|(tick, v)| v.finish(tick as u64));
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    let hash = |p: &PathBuf| -> trial::Result<String> {
        Ok(format!("{:x}", Sha256::digest(fs::read(p)?)))
    };
    fs::write(
        args.directory.join("sources.json"),
        serde_json::to_vec_pretty(&json!({
            "layout":layout.get_name(), "timed_placements":NAMES,
            "worker_sha256":hash(&args.worker)?,
            "timing_sha256":args.timings.as_ref().map(hash).transpose()?,
            "helper_limits":limits, "serial_control_sha256":args.serial_control.as_ref().map(hash).transpose()?,
            "identical_physical_frames":states.len()
        }))?,
    )?;
    println!(
        "Matched {} physical frames across both fault-injected placements.",
        states.len()
    );
    Ok(())
}
