// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Seeded oriented box convoys, native joins and full-state contact comparison.
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Coordinator,
    physics::{Body, Island, SimulationStep, Snapshot},
    transport::WorkerProcess,
};
use serde::Serialize;
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 20261119631)]
    seed: u64,
    #[arg(long, default_value_t = 100)]
    cases: u64,
    /// Extend selected seeds past the original 335-step observation window.
    #[arg(long, default_value_t=335, value_parser=clap::value_parser!(u32).range(1..=8000))]
    ticks: u32,
    /// Public solver step in milliseconds. Use the same simulated duration for comparisons.
    #[arg(long, default_value = "10", value_parser = parse_step)]
    step_ms: SimulationStep,
    /// Write committed body poses, closest surfaces and inter-train impulse for a single seed.
    #[arg(long, requires = "clearance")]
    trace: bool,
    /// Measure the closest finite surfaces of opposing trains at every native barrier.
    #[arg(long)]
    clearance: bool,
    #[arg(long)]
    stop_unix: Option<u64>,
}
#[path = "support/seeded.rs"]
mod seeded;
#[path = "support/train_fixture.rs"]
mod train_fixture;
#[path = "support/train_geometry.rs"]
mod train_geometry;
use train_geometry::{PairGeometry, inspect_pair};
fn distance<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter().zip(b).fold(0f64, |r, (x, y)| r.hypot(x - y))
}
#[derive(Default, Serialize)]
struct Trial {
    seed: u64,
    ticks: u32,
    step_ms: u64,
    bitwise_frames: u32,
    initial_load: [Option<u32>; 2],
    warm_route: Option<u32>,
    closest_cross_surfaces_m: Option<f64>,
    peak_cross_impulse_ns: f64,
    /// Left train momentum change minus its commanded external impulse during a public tick.
    peak_unexplained_left_impulse_ns: Option<f64>,
    transfer: Option<u32>,
    first_cross_impact: Option<u32>,
    first_difference: Option<u32>,
    first_contact_difference: Option<u32>,
    position_m: f64,
    velocity_m_s: f64,
    angle_rad: f64,
    spin_rad_s: f64,
    impulse_n_s: f64,
    before_join_position_m: f64,
    /// Geometry at each transition from separated to overlapping, for --trace.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    crossings: Vec<Crossing>,
    error: Option<String>,
}
#[derive(Serialize)]
struct Crossing {
    tick: u32,
    pair: [u64; 2],
    before: PairGeometry,
    after: PairGeometry,
    reported_pair_impulse_ns: f64,
}

fn record_crossings(t: &mut Trial, tick: u32, prior: &[Body], current: &Snapshot) -> Result<()> {
    let horizon = current.step.duration().as_secs_f64();
    // Both snapshots use the stable order of these four seeded public IDs.
    for i in 0..2 {
        for j in 2..4 {
            let a = &current.bodies[i];
            let b = &current.bodies[j];
            let after = inspect_pair(a, b, horizon)?;
            if after.sat_gap_m >= 0. {
                continue;
            }
            let before = inspect_pair(&prior[i], &prior[j], horizon)?;
            if before.sat_gap_m > 0. {
                let found = current.impulses.iter().find(|p| p.a == a.id && p.b == b.id);
                t.crossings.push(Crossing {
                    tick,
                    pair: [a.id, b.id],
                    before,
                    after,
                    reported_pair_impulse_ns: found.map_or(0., |p| p.normal_impulse),
                });
            }
        }
    }
    Ok(())
}

fn closest_cross_surfaces(bodies: &[Body]) -> Result<f64> {
    use rapier3d_f64::{
        parry::{query, shape::Cuboid},
        prelude::{Pose, Rotation, Vector},
    };
    let solids = bodies
        .iter()
        .map(|b| {
            let [x, y, z, w] = b.orientation;
            (
                b.id,
                Pose::from_parts(Vector::from(b.position), Rotation::from_xyzw(x, y, z, w)),
                Cuboid::new(Vector::from(b.size) * 0.5),
            )
        })
        .collect::<Vec<_>>();
    let mut result = f64::INFINITY;
    for (_, a, shape_a) in solids.iter().filter(|b| b.0 <= 2) {
        for (_, b, shape_b) in solids.iter().filter(|b| b.0 >= 3) {
            let separation = query::distance(a, shape_a, b, shape_b)?.distance;
            let signed = if separation == 0. {
                query::contact(a, shape_a, b, shape_b, 0.)?.map_or(0., |contact| contact.dist)
            } else {
                separation
            };
            result = result.min(signed);
        }
    }
    Ok(result)
}
fn parse_step(ms: &str) -> std::result::Result<SimulationStep, String> {
    ms.parse::<u64>()
        .ok()
        .and_then(|m| SimulationStep::from_duration(std::time::Duration::from_millis(m)))
        .ok_or_else(|| "expected 1, 2, 5 or 10 milliseconds".to_owned())
}

#[derive(Serialize)]
struct TraceFrame<'a> {
    tick: u32,
    time_s: f64,
    bodies: &'a [Body],
    gap_m: f64,
    impulse_ns: f64,
}

fn trial(t: &mut Trial, args: &Args, mut trace: Option<&mut BufWriter<File>>) -> Result<()> {
    let step = args.step_ms;
    let worker = &args.worker;
    let (bodies, inputs) = train_fixture::train_fixture(t.seed);
    let reversed = t.seed % 2 == 1;
    let mut sides = [bodies[..2].to_vec(), bodies[2..].to_vec()];
    if reversed {
        sides.swap(0, 1);
    }
    let peers = [
        WorkerProcess::spawn_with_step(worker, t.seed, 0, &sides[0], step)?,
        WorkerProcess::spawn_with_step(worker, t.seed, 1, &sides[1], step)?,
    ];
    let mut group = Coordinator::new(t.seed, peers, 1024)?;
    group.enable_rolling_journals();
    let mut prior_trace_bodies = trace.as_ref().map(|_| bodies.clone());
    let mut oracle = Island::restore(&Snapshot {
        bodies,
        step,
        ..Default::default()
    })?;
    let left_momentum = |bodies: &[Body]| -> [f64; 3] {
        std::array::from_fn(|k| {
            bodies
                .iter()
                .filter(|b| b.id <= 2)
                .map(|b| b.mass * b.linear[k])
                .sum()
        })
    };
    let mut previous_left = if args.clearance {
        Some(left_momentum(&oracle.snapshot().bodies))
    } else {
        None
    };
    for tick in 1..=args.ticks {
        let was_loaded = group
            .frame()
            .workers
            .iter()
            .all(|w| w.snapshot.impulses.iter().any(|p| p.normal_impulse > 1e-5));
        let frame = group.advance_in_field_overlapped([0.; 3], &inputs)?;
        oracle.step_with_wrenches(&inputs)?;
        let expected = oracle.snapshot();
        if let Some(previous) = previous_left {
            let next = left_momentum(&expected.bodies);
            let dt = expected.step.duration().as_secs_f64();
            let residual = distance(
                next,
                std::array::from_fn(|k| previous[k] + inputs[0].force[k] * dt),
            );
            t.peak_unexplained_left_impulse_ns = Some(
                t.peak_unexplained_left_impulse_ns
                    .unwrap_or(0.)
                    .max(residual),
            );
            previous_left = Some(next);
            let gap = closest_cross_surfaces(&expected.bodies)?;
            if let Some(prior) = &mut prior_trace_bodies {
                if gap < 0. {
                    record_crossings(t, tick, prior, &expected)?;
                }
                prior.clone_from(&expected.bodies);
            }
            if let Some(out) = trace.as_deref_mut() {
                let impulse_ns = expected
                    .impulses
                    .iter()
                    .filter(|p| p.a <= 2 && p.b >= 3)
                    .map(|p| p.normal_impulse)
                    .fold(0., f64::max);
                serde_json::to_writer(
                    &mut *out,
                    &TraceFrame {
                        tick,
                        time_s: f64::from(tick) * dt,
                        bodies: &expected.bodies,
                        gap_m: gap,
                        impulse_ns,
                    },
                )?;
                writeln!(out)?;
            }
            t.closest_cross_surfaces_m =
                Some(t.closest_cross_surfaces_m.map_or(gap, |old| old.min(gap)));
        }
        t.ticks = tick;
        let together = frame.body(1).unwrap().0 == frame.body(4).unwrap().0;
        if together && t.transfer.is_none() {
            t.transfer = Some(tick);
            if was_loaded {
                t.warm_route = Some(tick);
            }
        }
        let mut same = true;
        for b in &expected.bodies {
            let a = frame.body(b.id).unwrap().1;
            same &= a == b;
            let diff = distance(a.position, b.position);
            t.position_m = t.position_m.max(diff);
            if !together {
                t.before_join_position_m = t.before_join_position_m.max(diff);
            }
            t.velocity_m_s = t.velocity_m_s.max(distance(a.linear, b.linear));
            t.spin_rad_s = t.spin_rad_s.max(distance(a.angular, b.angular));
            let chord = distance(a.orientation, b.orientation)
                .min(distance(a.orientation, b.orientation.map(|n| -n)));
            t.angle_rad = t.angle_rad.max(4. * (chord / 2.).min(1.).asin());
        }
        let mut pairs = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter())
            .collect::<Vec<_>>();
        pairs.sort_by_key(|p| (p.a, p.b));
        if pairs
            .iter()
            .map(|p| (p.a, p.b))
            .ne(expected.impulses.iter().map(|p| (p.a, p.b)))
        {
            t.first_contact_difference.get_or_insert(tick);
            same = false;
        }
        for p in pairs {
            if p.a <= 2 && p.b >= 3 {
                t.peak_cross_impulse_ns = t.peak_cross_impulse_ns.max(p.normal_impulse);
            }
            if p.normal_impulse > 1e-5 {
                match (p.a, p.b) {
                    (1, 2) => {
                        t.initial_load[0].get_or_insert(tick);
                    }
                    (3, 4) => {
                        t.initial_load[1].get_or_insert(tick);
                    }
                    (1..=2, 3..=4) => {
                        t.first_cross_impact.get_or_insert(tick);
                    }
                    _ => {}
                }
            }
            if let Some(r) = expected.impulses.iter().find(|r| r.a == p.a && r.b == p.b) {
                t.impulse_n_s = t
                    .impulse_n_s
                    .max((r.normal_impulse - p.normal_impulse).abs());
                same &= p == r;
            }
        }
        if same {
            t.bitwise_frames += 1;
        } else {
            t.first_difference.get_or_insert(tick);
        }
        if t.warm_route == Some(tick) {
            group.compact()?;
            group.recover(if reversed { 1 } else { 0 })?;
        }
    }
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.trace && args.cases != 1 {
        return Err("--trace requires --cases 1".into());
    }
    fs::create_dir(&args.output)?;
    let mut trace = if args.trace {
        Some(BufWriter::new(File::create_new(
            args.output.join("trace.jsonl"),
        )?))
    } else {
        None
    };
    let mut output = BufWriter::new(File::create_new(args.output.join("trials.jsonl"))?);
    let mut exact = 0;
    let mut loaded = 0;
    let mut faults = 0;
    let mut cases = 0;
    for i in 0..args.cases {
        if args.stop_unix.is_some_and(|end| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs()
                >= end
        }) {
            break;
        }
        let mut t = Trial {
            step_ms: args.step_ms.duration().as_millis() as u64,
            seed: args.seed.checked_add(i).ok_or("seed overflow")?,
            ..Default::default()
        };
        if let Err(e) = trial(&mut t, &args, trace.as_mut()) {
            t.error = Some(e.to_string());
        }
        cases += 1;
        faults += u64::from(t.error.is_some());
        loaded += u64::from(t.warm_route.is_some());
        exact += u64::from(t.bitwise_frames == args.ticks);
        serde_json::to_writer(&mut output, &t)?;
        writeln!(output)?;
        output.flush()?;
        if i % 100 == 0 {
            println!("cases={cases}, warm_routes={loaded}, bitwise={exact}, faults={faults}");
        }
    }
    if let Some(mut out) = trace {
        out.flush()?;
    }
    println!("final cases={cases}, warm_routes={loaded}, bitwise={exact}, faults={faults}");
    Ok(())
}
