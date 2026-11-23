// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Present both measured foot–lift trajectories and each lane's last landing impulse.
#[path = "support_reacquisition_view/plots.rs"]
mod plots;
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde::{Deserialize, de::DeserializeOwned};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    io::BufReader,
    path::{Path, PathBuf},
    time::Duration,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Directory containing trajectory.json and summary.json.
    measurements: PathBuf,
    /// Fresh destination for replay and response plot.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Body {
    position: [f64; 3],
    orientation: [f64; 4],
}
#[derive(Deserialize)]
struct Sample {
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    vertical_gap_m: f64,
}
#[derive(Deserialize)]
struct Step {
    tick: u64,
    time_s: f64,
    prescribed: bool,
    switched: Sample,
    reference: Sample,
}
#[derive(Deserialize)]
struct Geometry {
    foot_half_extents: [f64; 3],
    upper_half_extents: [f64; 3],
    pad_half_extents: [f64; 3],
}
#[derive(Deserialize)]
struct Trace {
    dt_s: f64,
    geometry: Geometry,
    frames: Vec<Step>,
}
#[derive(Deserialize)]
struct Landing {
    tick: u64,
    normal_impulse_ns: f64,
    window_normal_impulse_ns: f64,
}
#[derive(Deserialize)]
struct Cycle {
    cycle: usize,
    departure_tick: u64,
    promotion_tick: u64,
    switched_landing: Landing,
    reference_landing: Landing,
    stop_tick: u64,
    demotion_tick: u64,
}
#[derive(Deserialize)]
struct Summary {
    window_steps: usize,
    cycles: Vec<Cycle>,
}

const FOOT: [[f64; 4]; 2] = [[0.14, 0.84, 0.73, 1.], [0.42, 0.70, 1., 1.]];
const PAD: [f64; 4] = [0.99, 0.67, 0.19, 1.];
const RULE: [f64; 4] = [0.57, 0.66, 0.73, 1.];

fn read<T: DeserializeOwned>(path: &Path) -> Result<T> {
    Ok(serde_json::from_reader(BufReader::new(fs::File::open(
        path,
    )?))?)
}

fn cuboid(half: [f64; 3]) -> Value {
    json!({"kind":"box","size":half.map(|h|2.*h)})
}

impl Sample {
    fn bodies<'a>(&'a self, g: &Geometry) -> [(&'a Body, &'static str, [f64; 3]); 3] {
        [
            (&self.pad, "pad", g.pad_half_extents),
            (&self.foot, "foot", g.foot_half_extents),
            (&self.upper, "upper", g.upper_half_extents),
        ]
    }
}

fn latest(cycles: &[Cycle], tick: u64, reference: bool) -> f64 {
    cycles
        .iter()
        .rev()
        .map(|c| {
            if reference {
                &c.reference_landing
            } else {
                &c.switched_landing
            }
        })
        .find(|l| l.tick <= tick)
        .map_or(0., |l| l.normal_impulse_ns)
}

fn main() -> Result<()> {
    let args = Args::parse();
    let trace: Trace = read(&args.measurements.join("trajectory.json"))?;
    let summary: Summary = read(&args.measurements.join("summary.json"))?;
    let cycles = &summary.cycles;
    let dt_ns = u64::try_from(Duration::try_from_secs_f64(trace.dt_s)?.as_nanos())?;
    let end = trace.frames.last().ok_or("empty trajectory")?.tick;
    if dt_ns == 0 || cycles.len() < 2 {
        return Err("expected repeated measured loss and landing intervals".into());
    }
    let mut min = DVec3::splat(f64::INFINITY);
    let mut max = DVec3::splat(f64::NEG_INFINITY);
    for (i, step) in trace.frames.iter().enumerate() {
        if step.tick != i as u64 || (step.time_s - step.tick as f64 * trace.dt_s).abs() > 1e-9 {
            return Err(format!("trajectory clock mismatch at sample {i}").into());
        }
        for lane in [&step.switched, &step.reference] {
            if !lane.normal_impulse_ns.is_finite() || lane.normal_impulse_ns < 0. {
                return Err(format!("invalid reaction at {i}").into());
            }
            for (body, _, shape) in lane.bodies(&trace.geometry) {
                let p = DVec3::from(body.position);
                let q = DQuat::from_array(body.orientation);
                if !p.is_finite()
                    || !q.is_normalized()
                    || shape.iter().any(|h| !h.is_finite() || *h <= 0.)
                {
                    return Err("invalid measured pose or shape".into());
                }
                let extent = (q * DVec3::X).abs() * shape[0]
                    + (q * DVec3::Y).abs() * shape[1]
                    + (q * DVec3::Z).abs() * shape[2];
                min = min.min(p - extent);
                max = max.max(p + extent);
            }
        }
    }
    let mut events = BTreeSet::new();
    let mut captures = BTreeMap::new();
    let mut landing_max = 0.0_f64;
    for cycle in cycles {
        for (reference, l) in [
            (false, &cycle.switched_landing),
            (true, &cycle.reference_landing),
        ] {
            let s = trace
                .frames
                .get(l.tick as usize)
                .ok_or("landing outside recording")?;
            let actual = if reference {
                s.reference.normal_impulse_ns
            } else {
                s.switched.normal_impulse_ns
            };
            let window_end = (l.tick as usize)
                .checked_add(summary.window_steps)
                .ok_or("window overflow")?;
            let sum: f64 = trace
                .frames
                .get(l.tick as usize..window_end)
                .ok_or("incomplete landing window")?
                .iter()
                .map(|s| {
                    if reference {
                        s.reference.normal_impulse_ns
                    } else {
                        s.switched.normal_impulse_ns
                    }
                })
                .sum();
            if (sum - l.window_normal_impulse_ns).abs() > 1e-12 {
                return Err("landing impulse sum disagrees with trace".into());
            }
            if l.normal_impulse_ns <= 0. || (l.normal_impulse_ns - actual).abs() > 1e-12 {
                return Err(format!(
                    "landing summary disagrees with step {} in cycle {}",
                    l.tick, cycle.cycle
                )
                .into());
            }
            landing_max = landing_max.max(actual);
        }
        for tick in [
            cycle.departure_tick,
            cycle.promotion_tick,
            cycle.switched_landing.tick,
            cycle.reference_landing.tick,
            cycle.stop_tick,
            cycle.demotion_tick,
        ] {
            if tick > end {
                return Err("cycle event after end".into());
            }
            events.extend(tick.saturating_sub(1)..=(tick + 1).min(end));
        }
        captures.insert(
            format!("landing_{}", cycle.cycle),
            cycle.switched_landing.tick,
        );
    }
    let flight = trace
        .frames
        .iter()
        .filter(|s| s.switched.normal_impulse_ns == 0. && s.switched.vertical_gap_m > 0.)
        .max_by(|a, b| {
            a.switched
                .vertical_gap_m
                .total_cmp(&b.switched.vertical_gap_m)
        })
        .ok_or("no measured free flight")?
        .tick;
    events.insert(flight);
    captures.insert("airborne".into(), flight);
    captures.insert("settled".into(), cycles.last().unwrap().demotion_tick);
    let stride = trace.frames.len().div_ceil(65);
    events.extend((0..=end).step_by(stride));
    events.insert(end);
    let width = max.x - min.x;
    let center = (max.x + min.x) * 0.5;
    let z0 = min.z - 0.16 * width;
    let spacing = width * 1.28;
    let view = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let mut archive = Vec::with_capacity(events.len());
    for (i, &tick) in events.iter().enumerate() {
        let step = &trace.frames[tick as usize];
        let mut frame = Frame::default();
        for (j, (name, lane, shift)) in [
            ("switched", &step.switched, -spacing * 0.5),
            ("always_dynamic", &step.reference, spacing * 0.5),
        ]
        .into_iter()
        .enumerate()
        {
            let transform = |p: DVec3| view * (p - DVec3::X * center) + DVec3::Y * shift;
            for (body, id, shape) in lane.bodies(&trace.geometry) {
                let color = match id {
                    "foot" => FOOT[j],
                    "pad" => PAD,
                    _ => [0.85, 0.88, 0.94, 1.],
                };
                frame.add(
                    &format!("{name}_{id}"),
                    transform(body.position.into()),
                    view * DQuat::from_array(body.orientation),
                    cuboid(shape),
                    color,
                );
            }
            for (row, (id, level, color)) in [
                (
                    "tier",
                    if j == 0 && step.prescribed { 0. } else { 1. },
                    FOOT[j],
                ),
                ("landing", latest(cycles, tick, j == 1) / landing_max, PAD),
            ]
            .into_iter()
            .enumerate()
            {
                let point = |level: f64, dz: f64| {
                    transform(DVec3::new(
                        center + (level - 0.5) * 0.80 * width,
                        0.,
                        z0 - row as f64 * width * 0.13 + dz * width,
                    ))
                };
                frame.edge(
                    &format!("{name}_{id}_scale"),
                    point(0., 0.),
                    point(1., 0.),
                    0.80 * width,
                    0.011 * width,
                    RULE,
                );
                for (label, level) in [("min", 0.), ("max", 1.)] {
                    frame.edge(
                        &format!("{name}_{id}_{label}"),
                        point(level, -0.02),
                        point(level, 0.02),
                        0.04 * width,
                        0.010 * width,
                        RULE,
                    );
                }
                frame.add(
                    &format!("{name}_{id}"),
                    point(level, 0.021),
                    view,
                    cuboid([0.020 * width, 0.016 * width, 0.025 * width]),
                    color,
                );
            }
        }
        archive.push(frame.finish(
            Duration::from_nanos(tick.checked_mul(dt_ns).ok_or("clock overflow")?),
            i + 1,
        ));
    }
    fs::create_dir_all(&args.output)?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), archive)?;
    let index = json!({"kind":"support-reacquisition-v1","dt_ns":dt_ns,"ticks":events,"captures":captures,"spacing":spacing,"width":width,"center":center,"gauge_z":z0,"max_landing_ns":landing_max});
    fs::write(
        args.output.join("frames.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    plots::draw(&trace, &summary, &args.output.join("response.svg"))
}
