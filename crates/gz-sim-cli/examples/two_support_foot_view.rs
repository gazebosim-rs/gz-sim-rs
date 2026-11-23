// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the individual ground reactions in two independently stepped Bevy worlds.
#[path = "two_support_foot_view/cache.rs"]
mod cache;
#[path = "two_support_foot_view/plots.rs"]
mod plots;
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde::Deserialize;
use serde_json::json;
use soft_replay::Frame;
use std::{error::Error, fs, path::PathBuf, time::Duration};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Paired trajectory.json and summary.json.
    measurements: PathBuf,
    /// A fresh directory for the replay, plots and frame index.
    output: PathBuf,
}
#[path = "two_support_foot_view/trace.rs"]
mod trace;
use trace::{Events, Sample, Trace};
const SUPPORT_COLORS: [[f64; 4]; 2] = [[0.96, 0.64, 0.17, 1.], [0.78, 0.43, 0.86, 1.]];
const SWITCHED: [f64; 4] = [0.16, 0.84, 0.74, 1.];
const REFERENCE: [f64; 4] = [0.43, 0.71, 1., 1.];
const RULE: [f64; 4] = [0.51, 0.58, 0.65, 1.];

fn cuboid(half: [f64; 3]) -> serde_json::Value {
    json!({"kind":"box","size":half.map(|n|2.*n)})
}

fn main() -> Result<()> {
    let args = Args::parse();
    let trace: Trace = serde_json::from_reader(std::io::BufReader::new(fs::File::open(
        args.measurements.join("trajectory.json"),
    )?))?;
    let summary: serde_json::Value =
        serde_json::from_slice(&fs::read(args.measurements.join("summary.json"))?)?;
    let events: Events = Deserialize::deserialize(&summary)?;
    let g = &trace.geometry;
    let a = DVec3::from(g.support_a_center);
    let b = DVec3::from(g.support_b_center);
    let radius_a = DVec3::from(g.support_a_half_extents);
    let radius_b = DVec3::from(g.support_b_half_extents);
    let dt_ns = u64::try_from(Duration::try_from_secs_f64(trace.dt_s)?.as_nanos())?;
    let mut min = (a - radius_a).min(b - radius_b);
    let mut max = (a + radius_a).max(b + radius_b);
    let mut max_impulse = 0.0f64;
    for (i, s) in trace.frames.iter().enumerate() {
        if dt_ns == 0
            || s.tick != i as u64
            || Duration::try_from_secs_f64(s.time_s)?
                .as_nanos()
                .abs_diff(u128::from(s.tick) * u128::from(dt_ns))
                > 1
        {
            return Err(format!("off-clock sample at {i}").into());
        }
        if s.support_b_present != (s.tick < events.loss_tick)
            || s.prescribed != (events.demotion_tick < s.tick && s.tick < events.promotion_tick)
        {
            return Err(format!(
                "support or fidelity timeline disagrees with summary at tick {}",
                s.tick
            )
            .into());
        }
        for lane in [&s.switched, &s.reference] {
            for (body, shape) in [
                (&lane.foot, g.foot_half_extents),
                (&lane.upper, g.upper_half_extents),
            ] {
                let p = DVec3::from(body.position);
                let q = DQuat::from_array(body.orientation);
                let extent = (q * DVec3::X).abs() * shape[0]
                    + (q * DVec3::Y).abs() * shape[1]
                    + (q * DVec3::Z).abs() * shape[2];
                if !p.is_finite() || !q.is_normalized() {
                    return Err("invalid native pose".into());
                }
                min = min.min(p - extent);
                max = max.max(p + extent);
            }
            for impulse in lane.normal_impulses_ns {
                if !impulse.is_finite() || impulse < 0. {
                    return Err("invalid normal impulse".into());
                }
                max_impulse = max_impulse.max(impulse);
            }
            if !s.support_b_present && lane.normal_impulses_ns[1] != 0. {
                return Err("removed support delivered a reaction".into());
            }
        }
    }
    if max_impulse <= 0. {
        return Err("expected loaded supports".into());
    }
    let mut shares = trace
        .frames
        .iter()
        .filter(|s| s.tick < events.demotion_tick)
        .filter_map(|s| {
            let [a, b] = s.switched.normal_impulses_ns;
            (a > 0. && b > 0.).then_some((a / (a + b), s.tick))
        });
    let (first_share, first_tick) = shares
        .next()
        .ok_or("no jointly loaded sample before demotion")?;
    let mut peak_a = (first_share, first_tick);
    let mut peak_b = peak_a;
    for row in shares {
        if row.0 > peak_a.0 {
            peak_a = row;
        }
        if row.0 < peak_b.0 {
            peak_b = row;
        }
    }
    let events_of_interest = [
        events.load_transfer_tick,
        events.demotion_tick,
        events.loss_tick,
        events.promotion_tick,
        peak_a.1,
        peak_b.1,
    ];
    let end = trace.frames.last().ok_or("empty trajectory")?.tick;
    if events_of_interest.iter().any(|t| *t > end) {
        return Err("event outside recorded interval".into());
    }
    let width = max.x - min.x;
    let spacing = width * 1.21;
    let cx = (min.x + max.x) / 2.;
    let ruler_width = width * 0.80;
    let gauge_z = min.z - width * 0.15;
    let view = DQuat::from_rotation_z(-std::f64::consts::FRAC_PI_2);
    let stride = trace.frames.len().div_ceil(54).max(1);
    let mut ticks = Vec::new();
    let mut frames = Vec::new();
    for (i, s) in trace.frames.iter().enumerate() {
        if i % stride != 0
            && s.tick != end
            && !events_of_interest.iter().any(|t| s.tick.abs_diff(*t) <= 2)
        {
            continue;
        }
        let mut frame = Frame::default();
        for (name, lane, shift, foot_color) in [
            ("switched", &s.switched, spacing / 2., SWITCHED),
            ("always_dynamic", &s.reference, -spacing / 2., REFERENCE),
        ] {
            let point = |p: DVec3| view * (p - DVec3::X * cx) + DVec3::Y * shift;
            frame.add(
                &format!("{name}_support_a"),
                point(a),
                view,
                cuboid(g.support_a_half_extents),
                SUPPORT_COLORS[0],
            );
            for (id, body, shape, color) in [
                ("foot", &lane.foot, g.foot_half_extents, foot_color),
                (
                    "upper",
                    &lane.upper,
                    g.upper_half_extents,
                    [0.84, 0.88, 0.94, 1.],
                ),
            ] {
                frame.add(
                    &format!("{name}_{id}"),
                    point(body.position.into()),
                    view * DQuat::from_array(body.orientation),
                    cuboid(shape),
                    color,
                );
            }
            for (j, support) in ["a", "b"].iter().enumerate() {
                let z = gauge_z - j as f64 * width * 0.17;
                let gauge = |fraction: f64, dz: f64| {
                    point(DVec3::new(cx + (fraction - 0.5) * ruler_width, 0., z + dz))
                };
                frame.edge(
                    &format!("{name}_{support}_scale"),
                    gauge(0., 0.),
                    gauge(1., 0.),
                    ruler_width,
                    width * 0.013,
                    RULE,
                );
                for (label, at) in [("zero", 0.), ("half", 0.5), ("full", 1.)] {
                    let h = width * 0.058;
                    frame.edge(
                        &format!("{name}_{support}_{label}"),
                        gauge(at, -h / 2.),
                        gauge(at, h / 2.),
                        h,
                        width * 0.013,
                        RULE,
                    );
                }
                frame.add(
                    &format!("{name}_{support}_load"),
                    gauge(lane.normal_impulses_ns[j] / max_impulse, width * 0.034),
                    view,
                    cuboid([width * 0.018, width * 0.018, width * 0.041]),
                    SUPPORT_COLORS[j],
                );
            }
        }
        // Add the two removable actors last to keep all surviving native IDs stable.
        if s.support_b_present {
            for (name, shift) in [
                ("switched", spacing / 2.),
                ("always_dynamic", -spacing / 2.),
            ] {
                frame.add(
                    &format!("{name}_support_b"),
                    view * (b - DVec3::X * cx) + DVec3::Y * shift,
                    view,
                    cuboid(g.support_b_half_extents),
                    SUPPORT_COLORS[1],
                );
            }
        }
        ticks.push(s.tick);
        frames.push(frame.finish(Duration::from_nanos(s.tick * dt_ns), frames.len() + 1));
    }
    fs::create_dir_all(&args.output)?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), frames)?;
    fs::write(
        args.output.join("frames.json"),
        serde_json::to_vec_pretty(&json!({
           "kind":"two-support-foot-v1","ticks":ticks,"dt_ns":dt_ns,
           "load_transfer_tick":events.load_transfer_tick,"demotion_tick":events.demotion_tick,"loss_tick":events.loss_tick,"promotion_tick":events.promotion_tick,
           "peak_a_share_tick":peak_a.1,"peak_b_share_tick":peak_b.1,"peak_a_share":peak_a.0,"peak_b_share":1.-peak_b.0,
           "center_x":cx,"spacing_m":spacing,"lane_width":width,"gauge_width":ruler_width,"gauge_z":gauge_z,"gauge_max_ns":max_impulse,
           "generation":"Both support boxes retain their full authored size. Support B retires at the measured scene clock."
        }))?,
    )?;
    plots::draw(&trace, &events, &args.output.join("response.svg"))?;
    cache::draw(&summary, &args.output.join("support-cache.svg"))
}
