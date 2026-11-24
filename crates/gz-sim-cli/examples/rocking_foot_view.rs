// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show the local rocking-foot handoff alongside its always-dynamic control.
#[path = "rocking_foot_view/response_plot.rs"]
mod response_plot;
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde::Deserialize;
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{error::Error, fs, path::PathBuf, time::Duration};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Directory containing trajectory.json and summary.json.
    source: PathBuf,
    /// Output for the native replay, frame index and response plot.
    output: PathBuf,
}
#[derive(Deserialize)]
struct Trace {
    dt_s: f64,
    geometry: Geometry,
    frames: Vec<Sample>,
}
#[derive(Deserialize)]
struct Geometry {
    ledge_center: [f64; 3],
    ledge_half_extents: [f64; 3],
    foot_half_extents: [f64; 3],
    upper_half_extents: [f64; 3],
    witness_local_m: [f64; 3],
}
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    time_s: f64,
    switched: Lane,
    reference: Lane,
}
#[derive(Deserialize)]
struct Lane {
    foot: Pose,
    upper: Pose,
    normal_impulse_ns: f64,
    edge_distance_m: f64,
}
#[derive(Deserialize)]
struct Pose {
    position: [f64; 3],
    orientation: [f64; 4],
}

const LEDGE: [f64; 4] = [0.28, 0.37, 0.48, 1.];
const EDGE: [f64; 4] = [1.0, 0.74, 0.24, 1.];
const DYNAMIC: [f64; 4] = [0.13, 0.85, 0.72, 1.];
const CONTROL: [f64; 4] = [0.40, 0.67, 0.99, 1.];

fn main() -> Result<()> {
    let Args { source, output } = Args::parse();
    let data: Value = serde_json::from_slice(&fs::read(source.join("trajectory.json"))?)?;
    let summary: Value = serde_json::from_slice(&fs::read(source.join("summary.json"))?)?;
    let trace: Trace = Deserialize::deserialize(&data)?;
    let event = |k: &str| summary[k].as_u64().ok_or_else(|| format!("missing {k}"));
    let demotion = event("demotion_tick")?;
    let crossing = event("crossing_tick")?;
    let resumed = event("promotion_tick")?;
    let index = |tick: u64| -> Result<&Sample> {
        trace
            .frames
            .iter()
            .find(|s| s.tick == tick)
            .ok_or_else(|| format!("missing tick {tick}").into())
    };
    for t in [demotion, crossing, resumed] {
        index(t)?;
    }
    let first = index(resumed)?;
    let dt_ns = Duration::try_from_secs_f64(trace.dt_s)?.as_nanos();
    if dt_ns == 0 || dt_ns > u64::MAX as u128 {
        return Err("unsupported native clock".into());
    }
    let dt_ns = dt_ns as u64;
    let g = &trace.geometry;
    let ledge_half = DVec3::from(g.ledge_half_extents);
    let ledge_origin = DVec3::from(g.ledge_center);
    let foot = DVec3::from(g.foot_half_extents);
    let upper = DVec3::from(g.upper_half_extents);
    let edge_x = ledge_origin.x + ledge_half.x;
    let left = ledge_origin.x - ledge_half.x;
    let bottom = ledge_origin.z - ledge_half.z;
    let mut xmin = left;
    let mut xmax = edge_x;
    let mut zmin = bottom;
    for (i, s) in trace.frames.iter().enumerate() {
        let ns = s.tick.checked_mul(dt_ns).ok_or("invalid tick")?;
        if s.tick != i as u64
            || ns.abs_diff(Duration::try_from_secs_f64(s.time_s)?.as_nanos() as u64) > 1
        {
            return Err(format!("frame {i} is off the recorded physics clock").into());
        }
        for lane in [&s.switched, &s.reference] {
            for (pose, half) in [(&lane.foot, foot), (&lane.upper, upper)] {
                let p = DVec3::from(pose.position);
                let q = DQuat::from_array(pose.orientation);
                if !p.is_finite()
                    || !q.is_normalized()
                    || !half.is_finite()
                    || half.min_element() <= 0.
                {
                    return Err("invalid recorded pose or shape".into());
                }
                let ext = (q * DVec3::X).abs() * half.x
                    + (q * DVec3::Y).abs() * half.y
                    + (q * DVec3::Z).abs() * half.z;
                xmin = xmin.min(p.x - ext.x);
                xmax = xmax.max(p.x + ext.x);
                zmin = zmin.min(p.z - ext.z);
            }
        }
    }
    let lane_width = xmax - xmin;
    let spacing = 1.18 * lane_width;
    let center_x = (xmax + xmin) / 2.;
    let gauge_z = zmin - lane_width * 0.12;
    let floor_size = (2. * ledge_half).to_array();
    let view = DQuat::from_rotation_z(-std::f64::consts::FRAC_PI_2);
    let max_load = first
        .switched
        .normal_impulse_ns
        .max(first.reference.normal_impulse_ns);
    // Both lanes may already be clear of the ledge at resumption.
    let max_load = if max_load > 0. {
        max_load
    } else {
        trace
            .frames
            .iter()
            .map(|s| s.reference.normal_impulse_ns)
            .fold(0., f64::max)
    };
    let edge_span = trace
        .frames
        .iter()
        .filter(|s| demotion <= s.tick && s.tick <= resumed)
        .flat_map(|s| {
            [
                s.switched.edge_distance_m.abs(),
                s.reference.edge_distance_m.abs(),
            ]
        })
        .fold(0., f64::max);
    if max_load <= 0. || edge_span <= 0. {
        return Err("missing loaded, moving support in trace".into());
    }
    let stride = (trace.frames.len() / 48).max(1);
    let mut ticks = Vec::new();
    let mut archive = Vec::new();
    for (i, s) in trace.frames.iter().enumerate() {
        if i % stride != 0
            && i + 1 != trace.frames.len()
            && ![demotion, crossing, resumed]
                .iter()
                .any(|t| t.abs_diff(s.tick) <= 1)
        {
            continue;
        }
        let mut f = Frame::default();
        for (name, lane, shift, first_impulse, foot_color) in [
            (
                "switched",
                &s.switched,
                spacing / 2.,
                first.switched.normal_impulse_ns,
                DYNAMIC,
            ),
            (
                "always_dynamic",
                &s.reference,
                -spacing / 2.,
                first.reference.normal_impulse_ns,
                CONTROL,
            ),
        ] {
            let point = |p: DVec3| view * (p - DVec3::X * center_x) + DVec3::Y * shift;
            let cuboid = |size: [f64; 3]| json!({"kind":"box","size":size});
            f.add(
                &format!("{name}_ledge"),
                point(ledge_origin),
                view,
                cuboid(floor_size),
                LEDGE,
            );
            // Highlight the station of the physical edge on the visible side face.
            let stripe = foot.x * 0.06;
            f.add(
                &format!("{name}_edge"),
                point(DVec3::new(
                    edge_x - stripe / 2.,
                    ledge_origin.y - ledge_half.y - stripe / 4.,
                    ledge_origin.z,
                )),
                view,
                cuboid([stripe, stripe / 2., 2. * ledge_half.z]),
                EDGE,
            );
            for (key, p, extent, color) in [
                ("foot", &lane.foot, foot, foot_color),
                ("upper", &lane.upper, upper, [0.83, 0.86, 0.90, 1.]),
            ] {
                f.add(
                    &format!("{name}_{key}"),
                    point(p.position.into()),
                    view * DQuat::from_array(p.orientation),
                    cuboid((2. * extent).to_array()),
                    color,
                );
            }
            let toe = DVec3::from(lane.foot.position)
                + DQuat::from_array(lane.foot.orientation) * DVec3::from(g.witness_local_m);
            if (toe.x - edge_x - lane.edge_distance_m).abs() > 1e-12 {
                return Err(format!(
                    "toe witness disagrees with signed edge distance at {}",
                    s.tick
                )
                .into());
            }
            f.add(
                &format!("{name}_toe_witness"),
                point(toe),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":foot.min_element()*0.14}),
                EDGE,
            );
            // Latch the first resumed impulse for both lanes at exactly the same clock.
            let width = 0.74 * lane_width;
            let gauge = |fraction: f64, height: f64| {
                point(DVec3::new(
                    center_x + (fraction - 0.5) * width,
                    ledge_origin.y,
                    gauge_z + height,
                ))
            };
            let magnitude = if s.tick < resumed {
                0.
            } else {
                first_impulse / max_load
            };
            let position = (lane.edge_distance_m / edge_span).clamp(-1., 1.) * 0.5 + 0.5;
            for (label, fraction, offset, marker) in [
                ("toe_relative_to_edge", position, 0., EDGE),
                (
                    "first_reaction",
                    magnitude,
                    -0.13 * lane_width,
                    if name == "switched" { DYNAMIC } else { CONTROL },
                ),
            ] {
                f.edge(
                    &format!("{name}_{label}_scale"),
                    gauge(0., offset),
                    gauge(1., offset),
                    width,
                    lane_width * 0.012,
                    LEDGE,
                );
                let h = lane_width * 0.054;
                for (tick_name, u) in [("left", 0.), ("middle", 0.5), ("right", 1.)] {
                    f.edge(
                        &format!("{name}_{label}_{tick_name}"),
                        gauge(u, offset - h / 2.),
                        gauge(u, offset + h / 2.),
                        h,
                        lane_width * 0.011,
                        LEDGE,
                    );
                }
                f.add(
                    &format!("{name}_{label}"),
                    gauge(fraction, offset + lane_width * 0.03),
                    view,
                    cuboid([lane_width * 0.027, lane_width * 0.027, lane_width * 0.065]),
                    marker,
                );
            }
        }
        ticks.push(s.tick);
        archive.push(f.finish(Duration::from_nanos(dt_ns * s.tick), archive.len() + 1));
    }
    fs::create_dir_all(&output)?;
    gz_sim_cli::export_archive(&output.join("replay"), Default::default(), archive)?;
    fs::write(
        output.join("frames.json"),
        serde_json::to_vec_pretty(&json!({
            "ticks":ticks, "dt_ns":dt_ns,"demotion_tick":demotion,"crossing_tick":crossing,"promotion_tick":resumed,
            "spacing_m":spacing,"center_x":center_x,"display_rotation_xyzw":view.to_array(),
            "display_ledge_center":ledge_origin.to_array(),"display_ledge_size":floor_size,
            "gauge_z":gauge_z,"gauge_width_m":0.74*lane_width,"gauge_max_ns":max_load,"edge_ruler_half_span_m":edge_span,
            "first_switched_ns":first.switched.normal_impulse_ns,"first_reference_ns":first.reference.normal_impulse_ns
        }))?,
    )?;
    response_plot::draw(&data, &summary, &output.join("response.svg"))
}
