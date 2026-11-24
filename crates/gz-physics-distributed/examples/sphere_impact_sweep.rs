// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Oblique sphere impacts against an uninterrupted native solve and free-space conservation.
#[path = "support/seeded.rs"]
mod seeded;
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Coordinator,
    physics::{Body, Island, Primitive, SimulationStep, Snapshot, fixture},
    transport::WorkerProcess,
};
use seeded::Rng;
use serde::Serialize;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 926117321)]
    seed: u64,
    #[arg(long, default_value_t = 32)]
    cases: u64,
    #[arg(long)]
    stop_unix: Option<u64>,
    /// Override the public step while retaining the seed's exact initial physical state.
    #[arg(long, value_parser=parse_step)]
    step_ms: Option<SimulationStep>,
}
#[derive(Default, Serialize)]
struct Trial {
    seed: u64,
    step_ms: u64,
    friction: f64,
    frames: u64,
    exact_frames: u64,
    impact: Option<u64>,
    analytic_impact_s: f64,
    transferred: bool,
    impact_impulse: f64,
    /// Continuous perfectly inelastic normal impulse for the authored free flight.
    analytical_normal_impulse: f64,
    /// Final single-world COM velocity residual for the smooth subset.
    smooth_final_velocity_error: Option<f64>,
    /// Smooth sphere contacts exert no spin torque.
    smooth_final_spin_error: Option<f64>,
    max_linear_momentum_error: f64,
    max_angular_momentum_error: f64,
    max_energy_gain: f64,
    max_position_delta: f64,
    first_difference: Option<u64>,
    error: Option<String>,
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(u, v)| u * v).sum()
}
fn subtract(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
fn invariants(bodies: &[Body]) -> ([f64; 3], [f64; 3], f64) {
    let mut p = [0.; 3];
    let mut h = [0.; 3];
    let mut e = 0.;
    for b in bodies {
        let linear = b.linear.map(|v| v * b.mass);
        let inertia = 0.4 * b.mass * (0.5 * b.size[0]).powi(2);
        for k in 0..3 {
            let (j, l) = ((k + 1) % 3, (k + 2) % 3);
            p[k] += linear[k];
            h[k] += inertia * b.angular[k] + b.position[j] * linear[l] - b.position[l] * linear[j];
        }
        e += 0.5 * (b.mass * dot(b.linear, b.linear) + inertia * dot(b.angular, b.angular));
    }
    (p, h, e)
}
fn parse_step(ms: &str) -> std::result::Result<SimulationStep, String> {
    ms.parse::<u64>()
        .ok()
        .and_then(|m| SimulationStep::from_duration(std::time::Duration::from_millis(m)))
        .ok_or_else(|| "expected a public step of 1, 2, 5 or 10 ms".to_owned())
}
fn trial(t: &mut Trial, worker: &Path, override_step: Option<SimulationStep>) -> Result<()> {
    let mut rng = Rng(t.seed);
    let step = override_step.unwrap_or(
        [
            SimulationStep::Ms1,
            SimulationStep::Ms2,
            SimulationStep::Ms5,
            SimulationStep::Ms10,
        ][(t.seed % 4) as usize],
    );
    let dt = step.duration().as_secs_f64();
    t.step_ms = step.duration().as_millis() as u64;
    let yaw = rng.number(-314.0 / 100.0, 314.0 / 100.0);
    let pitch = rng.number(-1.5, 1.5);
    let (s, c) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let rotate = |[x, y, z]: [f64; 3]| {
        [
            c * cp * x - s * y + c * sp * z,
            s * cp * x + c * y + s * sp * z,
            -sp * x + cp * z,
        ]
    };
    let radii = [rng.number(0.12, 0.54), rng.number(0.12, 0.54)];
    let radius = radii[0] + radii[1];
    let side = radius * rng.number(-0.82, 0.82);
    let hit = (radius * radius - side * side).sqrt();
    let speed = rng.number(1.1, 6.);
    t.analytic_impact_s = rng.number(0.08, 0.19);
    let start = hit + speed * t.analytic_impact_s;
    let drift = rotate([
        rng.number(-1., 1.),
        rng.number(-1., 1.),
        rng.number(-1., 1.),
    ]);
    t.friction = if t.seed.is_multiple_of(3) {
        0.
    } else {
        rng.number(0.02, 0.86)
    };
    let bodies: Vec<_> = (0..2)
        .map(|i| {
            let sign = if i == 0 { -1. } else { 1. };
            let v = rotate([-0.5 * sign * speed, 0., 0.]);
            Body {
                id: 10 + i as u64,
                primitive: Primitive::Sphere,
                size: [2. * radii[i]; 3],
                mass: rng.number(0.2, 4.),
                friction: t.friction,
                position: rotate([0.5 * sign * start, 0.5 * sign * side, 0.]),
                angular: std::array::from_fn(|_| rng.number(-5., 5.)),
                linear: std::array::from_fn(|k| v[k] + drift[k]),
                ..fixture()[0].promoted()
            }
        })
        .collect();
    let initial = invariants(&bodies);
    // At continuous first touch the unrotated separation is [hit, side, 0].
    let normal = rotate([hit / radius, side / radius, 0.]);
    let closing = dot(subtract(bodies[0].linear, bodies[1].linear), normal);
    t.analytical_normal_impulse = closing / (bodies[0].mass.recip() + bodies[1].mass.recip());
    let smooth_expected: Vec<_> = bodies
        .iter()
        .enumerate()
        .map(|(i, body)| {
            let signed = if i == 0 {
                -t.analytical_normal_impulse
            } else {
                t.analytical_normal_impulse
            };
            let velocity = std::array::from_fn(|k| body.linear[k] + normal[k] * signed / body.mass);
            (velocity, body.angular)
        })
        .collect();
    let first = (t.seed % 2) as usize;
    let make = |owner: usize| {
        WorkerProcess::spawn_with_step(
            worker,
            t.seed,
            owner as u8,
            std::slice::from_ref(&bodies[owner ^ first]),
            step,
        )
    };
    let mut world = Coordinator::new(t.seed, [make(0)?, make(1)?], 512)?;
    let mut native = Island::restore(&Snapshot {
        bodies,
        step,
        ..Default::default()
    })?;
    for tick in 1..=(0.31 / dt).round() as u64 {
        world.advance_routed_overlapped()?;
        native.step()?;
        let expected = native.snapshot();
        let actual = world.frame();
        let mut same = true;
        for b in &expected.bodies {
            let moved = actual.body(b.id).unwrap().1;
            same &= moved == b;
            t.max_position_delta = t
                .max_position_delta
                .max(norm(subtract(moved.position, b.position)));
        }
        let mut pairs = actual
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter());
        same &= pairs.by_ref().eq(expected.impulses.iter());
        t.frames += 1;
        t.exact_frames += u64::from(same);
        if !same {
            t.first_difference.get_or_insert(tick);
        }
        let (p, h, e) = invariants(&expected.bodies);
        t.max_linear_momentum_error = t
            .max_linear_momentum_error
            .max(norm(subtract(p, initial.0)));
        t.max_angular_momentum_error = t
            .max_angular_momentum_error
            .max(norm(subtract(h, initial.1)));
        t.max_energy_gain = t.max_energy_gain.max(e - initial.2);
        for p in &expected.impulses {
            t.impact_impulse += p.normal_impulse;
            if p.normal_impulse > 0. && t.impact.is_none() {
                t.impact = Some(tick);
                world.transfer_component(10)?;
                t.transferred = true;
            }
        }
    }
    if t.friction == 0. {
        let final_frame = native.snapshot();
        let (mut velocity, mut spin) = (0_f64, 0_f64);
        for (body, (ideal_velocity, ideal_spin)) in final_frame.bodies.iter().zip(&smooth_expected)
        {
            velocity = velocity.max(norm(subtract(body.linear, *ideal_velocity)));
            spin = spin.max(norm(subtract(body.angular, *ideal_spin)));
        }
        t.smooth_final_velocity_error = Some(velocity);
        t.smooth_final_spin_error = Some(spin);
    }
    for mut worker in world.into_peers() {
        worker.shutdown()?;
    }
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    let mut writer = BufWriter::new(File::create_new(&args.output)?);
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
            seed: args.seed.checked_add(i).ok_or("seed overflow")?,
            ..Default::default()
        };
        if let Err(error) = trial(&mut t, &args.worker, args.step_ms) {
            t.error = Some(error.to_string());
        }
        serde_json::to_writer(&mut writer, &t)?;
        writeln!(writer)?;
        writer.flush()?;
        if (i + 1).is_multiple_of(1000) {
            eprintln!("completed {} oblique sphere trials", i + 1);
        }
    }
    Ok(())
}
