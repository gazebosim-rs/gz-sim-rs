// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::MeasuredBox;
use super::{BLUE, GOLD, GRAY, GREEN, Result, View, body, gauge};
use rapier3d_f64::math::Vector;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    time_ns: u64,
    bodies: [Member; 2],
    normal_total: f64,
    events: Vec<Value>,
}
#[derive(Deserialize)]
struct Member {
    id: u64,
    size: [f64; 3],
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
}
pub(super) fn export(src: &Path, out: &Path) -> Result<()> {
    let read = |name: &str| -> Result<Vec<Sample>> {
        BufReader::new(File::open(src.join(name))?)
            .lines()
            .map(|r| Ok(serde_json::from_str(&r?)?))
            .collect()
    };
    let group = read("grouped.jsonl")?;
    let point = read("pointwise.jsonl")?;
    if group.len() != 121 || group.len() != point.len() {
        return Err("incomplete authored native trace".into());
    }
    let mut stream = vec![];
    let mut samples = vec![];
    let mut events = 0;
    for (k, (g, p)) in group.iter().zip(&point).enumerate() {
        let t = k as u64;
        if g.tick != t
            || p.tick != t
            || g.time_ns != t * 10_000_000
            || p.time_ns != g.time_ns
            || g.events != p.events
        {
            return Err("input or clock mismatch".into());
        }
        let mut v = View::default();
        let base = 10_000 + t * 100;
        events += g.events.len();
        for (lane, (tag, r, offset, color)) in
            [("grouped", g, -1.65, BLUE), ("per_corner", p, 1.65, GREEN)]
                .into_iter()
                .enumerate()
        {
            let mut dynamic = None;
            for (i, m) in r.bodies.iter().enumerate() {
                let is_block = m.size == [0.8, 0.4, 0.2];
                if is_block {
                    dynamic = Some(m);
                } else if m.size != [8., 3., 0.4] {
                    return Err("unrecognized authored geometry".into());
                }
                let id = 20 + 20 * lane as u64 + 6 * i as u64;
                body(
                    &mut v,
                    id,
                    &format!(
                        "{tag}_{}_public_{}",
                        if is_block { "block" } else { "table" },
                        m.id
                    ),
                    MeasuredBox {
                        size: m.size,
                        position: m.position,
                        orientation: m.orientation,
                        stripe: is_block,
                    },
                    Vector::new(0., offset, 0.),
                    if is_block { color } else { GRAY },
                );
            }
            let b = dynamic.ok_or("dynamic member")?;
            let sy = if lane == 0 { -3.45 } else { 3.45 };
            let dy = if lane == 0 { -0.24 } else { 0.24 };
            for (n, (field, val, scale, c)) in [
                ("linear_x_m_s", b.velocity[0], 0.56, color),
                ("yaw_rad_s", b.angular_velocity[2], 0.20, color),
                ("normal_impulse_N_s", r.normal_total, 7., GOLD),
            ]
            .into_iter()
            .enumerate()
            {
                gauge(
                    &mut v,
                    base + lane as u64 * 10 + 2 * n as u64,
                    &format!("tick_{t}_{tag}_{field}"),
                    val,
                    [-2.0, sy + dy * n as f64, 0.05],
                    scale,
                    c,
                );
            }
        }
        stream.push(v.finish_at(t + 1, g.time_ns));
        samples.push(json!({"tick":t,"grouped_normal_ns":g.normal_total,"per_corner_normal_ns":p.normal_total,"events":g.events}));
    }
    if events == 0 {
        return Err("missing actual drive".into());
    }
    fs::create_dir(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), stream)?;
    serde_json::to_writer_pretty(
        File::create(out.join("index.json"))?,
        &json!({"samples":samples}),
    )?;
    println!(
        "Exported 121 checked grouped and per-corner Bevy SDF observations, including {events} recorded input events."
    );
    Ok(())
}
