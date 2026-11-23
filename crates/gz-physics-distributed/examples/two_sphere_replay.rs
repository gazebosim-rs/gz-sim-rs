// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render measured bodies, contact impulses and ownership through the gz-web-rs replay format.
#[path = "support/hinge_rod.rs"]
mod hinge_rod;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
#[path = "support/two_sphere_timings.rs"]
mod two_sphere_timings;

use clap::Parser;
use gz_physics_distributed::multi_sphere::{MultiSphereObservation, MultiSphereSpec};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use primitive_replay_view::View;
use rapier3d_f64::math::{Rotation, Vector};
use serde::Deserialize;
use serde_json::json;
use std::{
    collections::BTreeSet,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};

const COLORS: [[f64; 4]; 4] = [
    [0.13, 0.57, 0.93, 1.],
    [0.95, 0.42, 0.14, 1.],
    [0.84, 0.19, 0.58, 1.],
    [0.16, 0.78, 0.35, 1.],
];
const GRAY: [f64; 4] = [0.24, 0.30, 0.38, 1.];
const GOLD: [f64; 4] = [0.99, 0.76, 0.16, 1.];
const ID: [f64; 4] = [0., 0., 0., 1.];
const SCALE: f64 = 2.2;

#[derive(Parser)]
struct Args {
    /// Qualifying scene containing the two moving spheres.
    sdf: PathBuf,
    /// states.jsonl from the two_sphere_hinge example.
    states: PathBuf,
    /// Create the replay and frame index in this new directory.
    output: PathBuf,
    #[arg(long, value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: u8,
    /// Verify and show an independently loaded sphere pair that later meets the hinge.
    #[arg(long)]
    pair_island: bool,
    /// Matched --samples outputs from the previous build, one file per repetition.
    #[arg(long, requires = "after_samples")]
    before_samples: Vec<PathBuf>,
    /// Matching files from the current build. Each gauge shows the median for that tick.
    #[arg(long, requires = "before_samples")]
    after_samples: Vec<PathBuf>,
}
#[derive(Deserialize)]
struct Row {
    tick: u64,
    owners: [u8; 3],
    worker: MultiSphereObservation,
    event: Option<serde_json::Value>,
}
impl Row {
    fn loads(&self) -> [f64; 3] {
        let [a, b] = self
            .worker
            .reactions
            .map(|links| links.iter().map(|r| r.normal_impulse).sum());
        [a, b, self.worker.sphere_reaction.normal_impulse]
    }
}
fn sphere(
    view: &mut View,
    id: u64,
    name: &str,
    at: [f64; 3],
    rotation: [f64; 4],
    radius: f64,
    color: [f64; 4],
) {
    view.add(
        id,
        name,
        at,
        rotation,
        json!({"kind":"sphere","radius":radius}),
        color,
    );
}
fn draw(
    spec: &MultiSphereSpec,
    row: &Row,
    workers: u8,
    max_load: f64,
    origin: Vector,
    frame: u64,
    cost: Option<&two_sphere_timings::TickCost>,
) -> (String, String) {
    let mut v = View::default();
    let project = |p: Vector| (p - origin) * SCALE + Vector::new(0., 0., 0.72);
    for (i, (rest, body, name)) in [
        (
            &spec.hinge.links[0],
            &row.worker.hinge.links[0],
            "hinge_base",
        ),
        (
            &spec.hinge.links[1],
            &row.worker.hinge.links[1],
            "hinge_child",
        ),
        (&spec.spheres[0], &row.worker.spheres[0], "sphere_A"),
        (&spec.spheres[1], &row.worker.spheres[1], "sphere_B"),
    ]
    .into_iter()
    .enumerate()
    {
        let id = 10 + 6 * i as u64;
        let at = Vector::from(body.position);
        sphere(
            &mut v,
            id,
            name,
            project(at).to_array(),
            body.orientation,
            rest.radius * SCALE,
            COLORS[i],
        );
        if i < 2 {
            let local = Rotation::from_array(rest.orientation).inverse()
                * (Vector::from(spec.hinge.anchor) - Vector::from(rest.position));
            let pivot = at + Rotation::from_array(body.orientation) * local;
            if local.length() > 0. {
                hinge_rod::rod(
                    &mut v,
                    id + 2,
                    &format!("{name}_to_joint"),
                    project(at),
                    project(pivot),
                    local.length() * SCALE,
                    COLORS[i],
                );
            }
            if i == 0 {
                sphere(
                    &mut v,
                    id + 4,
                    "revolute_axis",
                    project(pivot).to_array(),
                    ID,
                    0.075,
                    GOLD,
                );
            }
        }
    }
    // Pads are a separate display area; only the body visuals use physical coordinates.
    let slot_y = |slot| (f64::from(workers - 1) * 0.5 - f64::from(slot)) * 0.59;
    for slot in 0..workers {
        v.cuboid(
            60 + 2 * u64::from(slot),
            &format!("worker_{slot}"),
            [-2.96, slot_y(slot), 0.16],
            [0.99, 0.52, 0.14],
            GRAY,
        );
    }
    for (i, (name, color)) in [
        ("owner_hinge", COLORS[0]),
        ("owner_A", COLORS[2]),
        ("owner_B", COLORS[3]),
    ]
    .into_iter()
    .enumerate()
    {
        sphere(
            &mut v,
            80 + 2 * i as u64,
            name,
            [-3.29 + i as f64 * 0.33, slot_y(row.owners[i]), 0.36],
            ID,
            0.131,
            color,
        );
    }
    for (i, impulse) in row.loads().into_iter().enumerate() {
        let y = 1.06 - 1.06 * i as f64;
        let name = ["A", "B", "A_B"][i];
        v.cuboid(
            92 + 4 * i as u64,
            &format!("impulse_scale_{name}"),
            [2.20, y, 0.35],
            [0.055, 0.86, 0.06],
            GRAY,
        );
        sphere(
            &mut v,
            94 + 4 * i as u64,
            &format!("normal_impulse_{name}"),
            [2.20, y + 0.43 - 0.86 * impulse / max_load, 0.43],
            ID,
            0.103,
            [COLORS[2], COLORS[3], GOLD][i],
        );
    }
    if let Some(cost) = cost {
        // Two parallel rails share a linear 0–1 ms scale. A tick that misses its
        // simulation deadline lands at the end stop; the index keeps the full value.
        for (i, (name, ns, color)) in [
            ("previous", cost.before_ns, [0.96, 0.49, 0.18, 1.]),
            ("current", cost.after_ns, [0.12, 0.85, 0.89, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let x = 3.02 + i as f64 * 0.24;
            let id = 108 + i as u64 * 4;
            v.cuboid(
                id,
                &format!("wall_time_{name}_scale_1ms"),
                [x, 0., 0.34],
                [0.07, 2.90, 0.05],
                GRAY,
            );
            let y = 1.45 - 2.90 * ns.min(1_000_000) as f64 / 1_000_000.;
            sphere(
                &mut v,
                id + 2,
                &format!("wall_time_{name}"),
                [x, y, 0.43],
                ID,
                0.10,
                color,
            );
        }
    }
    v.finish_at(frame, row.tick * 1_000_000)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let spec = distributed_multi_sphere_spec(
        &fs::read_to_string(&args.sdf)?,
        &args.sdf.to_string_lossy(),
        "default",
    )?;
    let mut rows = BufReader::new(File::open(args.states)?)
        .lines()
        .map(|line| -> Result<Row, Box<dyn Error>> { Ok(serde_json::from_str(&line?)?) })
        .collect::<Result<Vec<_>, _>>()?;
    let costs = if args.before_samples.is_empty() {
        None
    } else {
        let costs = two_sphere_timings::compare(
            &args.before_samples,
            &args.after_samples,
            rows.len(),
            args.workers,
        )?;
        // The qualification injects manual transfers. Timed runs follow the normal
        // route, whose actual worker IDs belong on the timing playback's pads.
        for (row, cost) in rows.iter_mut().zip(&costs) {
            row.owners = cost.owners;
        }
        Some(costs)
    };
    let mut frames_at = BTreeSet::new();
    let mut max_load = 0.0_f64;
    let mut shared_tick = None;
    let mut sphere_collision = None;
    let mut independent_load = None;
    let mut merged_load = None;
    for (i, row) in rows.iter().enumerate() {
        if row.tick != i as u64 || row.owners.iter().any(|&s| s >= args.workers) {
            return Err("noncontiguous trace or invalid owner".into());
        }
        let loads = row.loads();
        max_load = loads.into_iter().fold(max_load, f64::max);
        if loads[..2].iter().all(|v| *v > 0.) && shared_tick.is_none() {
            if row.owners != [row.owners[0]; 3] {
                return Err("simultaneous impulses without shared ownership".into());
            }
            shared_tick = Some(i);
        }
        if loads[2] > 0. && sphere_collision.is_none() {
            sphere_collision = Some(i);
        }
        if loads[2] > 0. {
            let [hinge, a, b] = row.owners;
            if a != b {
                return Err("sphere contact on distinct workers".into());
            }
            if a != hinge {
                if loads[..2] != [0., 0.] {
                    return Err("separate hinge received an impulse from the sphere pair".into());
                }
                independent_load.get_or_insert(i);
            } else if independent_load.is_some() && loads[..2].iter().any(|&v| v > 0.) {
                merged_load.get_or_insert(i);
            }
        }
        if i % 8 == 0 || i + 1 == rows.len() || row.event.is_some() || loads.iter().any(|x| *x > 0.)
        {
            frames_at.insert(i);
        }
        if i > 0 && rows[i - 1].owners != row.owners {
            frames_at.extend([i - 1, i]);
        }
    }
    if args.pair_island {
        if independent_load.is_none() || merged_load.is_none() {
            return Err(
                "recording lacks independent sphere contact or its later coupled hinge impact"
                    .into(),
            );
        }
    } else if shared_tick.is_none() {
        return Err("recording lacks a simultaneous positive impulse on both spheres".into());
    }
    let mut origin =
        (Vector::from(spec.spheres[0].position) + Vector::from(spec.spheres[1].position)) * 0.5;
    if args.pair_island {
        let centers = spec
            .hinge
            .links
            .into_iter()
            .chain(spec.spheres)
            .map(|link| Vector::from(link.position));
        let (lo, hi) = centers.fold((origin, origin), |(lo, hi), p| (lo.min(p), hi.max(p)));
        origin = (lo + hi) * 0.5;
    }
    let ticks: Vec<_> = frames_at.into_iter().collect();
    let frame_at = |tick: Option<usize>| {
        tick.map(|i| ticks.binary_search(&i).expect("loaded frame included") + 1)
    };
    let frames = ticks.iter().enumerate().map(|(i, &n)| {
        draw(
            &spec,
            &rows[n],
            args.workers,
            max_load,
            origin,
            i as u64 + 1,
            costs.as_ref().map(|series| &series[n]),
        )
    });
    fs::create_dir(&args.output)?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), frames)?;
    let mut index = json!({
            "ticks":ticks,"shared_tick":shared_tick,"shared_frame":frame_at(shared_tick),"max_normal_impulse_ns":max_load,
            "sphere_collision_tick": sphere_collision, "sphere_collision_frame":frame_at(sphere_collision),
            "shared_owners":shared_tick.map(|i|rows[i].owners),"shared_normal_impulses_ns":shared_tick.map(|i|rows[i].loads()),
            "independent_pair_tick":independent_load,"independent_pair_frame":frame_at(independent_load),
            "merged_impact_tick":merged_load,"merged_impact_frame":frame_at(merged_load),"pair_island":args.pair_island,"workers":args.workers,
            "sphere_origin_m":origin.to_array(),
            "display_scale":SCALE
    });
    if let Some(costs) = costs {
        index["tick_costs"] = serde_json::to_value(costs)?;
        index["before_runs"] = args.before_samples.len().into();
        index["after_runs"] = args.after_samples.len().into();
    }
    fs::write(
        args.output.join("index.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    Ok(())
}
