// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record a genuinely turning finite native pad and an actual material response.
#[path = "soft_replay/articulated_event.rs"]
mod impact;
mod soft_replay;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/rotating_pad.rs"]
mod source;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use soft_replay::Frame;
use source::*;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::Duration,
};
const PIVOT: DVec3 = DVec3::new(0., 0., 0.62);
const STEEL: [f64; 4] = [0.20, 0.42, 0.65, 1.];
const GREEN: [f64; 4] = [0.08, 0.76, 0.47, 1.];
#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    cloth: bool,
    /// Continue the sheet through its next, unsupported collision.
    #[arg(long, requires = "cloth")]
    diagnostic_tail: bool,
    #[arg(long)]
    measurements: Option<PathBuf>,
    /// Compare the returning sheet with an identically integrated single-event control.
    #[arg(long, requires = "cloth", conflicts_with = "diagnostic_tail")]
    recurrent: bool,
}
fn record(rotor: &Rotor, f: &mut Frame, name: &str, shift: f64, previous: Duration) -> Value {
    let off = shift * DVec3::Y;
    let q = rotor.state();
    let center = DVec3::from(q.position);
    let nodes = rotor.material().particles();
    let points: Vec<_> = nodes.iter().map(|p| DVec3::from(p.position)).collect();
    let raw: Vec<_> = nodes
        .iter()
        .map(|p| json!({"position":p.position,"v":p.velocity,"inv":p.inverse_mass}))
        .collect();
    let ball = |f: &mut Frame, id: &str, p: DVec3, r: f64, color| {
        f.add(
            &format!("{name}_{id}"),
            p + off,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":r}),
            color,
        )
    };
    let edge = |f: &mut Frame, id: &str, a: DVec3, b: DVec3, len, width, color| {
        f.edge(&format!("{name}_{id}"), a + off, b + off, len, width, color)
    };
    f.add(
        &format!("{name}_mount"),
        off + DVec3::new(0., 0., 0.29),
        DQuat::IDENTITY,
        json!({"kind":"box","size":[0.070,0.070,0.58]}),
        [0.43, 0.47, 0.51, 1.],
    );
    f.add(
        &format!("{name}_floor"),
        off + DVec3::new(0.17, 0.12, -0.015),
        DQuat::IDENTITY,
        json!({"kind":"box","size":[1.09,1.08,0.03]}),
        [0.17, 0.20, 0.24, 1.],
    );
    ball(f, "pivot", PIVOT, 0.045, STEEL);
    ball(f, "pad", center, 0.035, [0.1, 0.70, 0.90, 1.]);
    edge(f, "arm", PIVOT, center, 0.45, 0.032, STEEL);
    for (i, &p) in points.iter().enumerate() {
        ball(f, &format!("node_{i}"), p, 0.016, GREEN);
    }
    if points.len() == 2 {
        edge(f, "strand", points[0], points[1], 0.70, 0.014, GREEN);
    } else {
        for (face, ids) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
            let [a, b, c] = ids.map(|i| points[i]);
            let lens = if face == 0 {
                [0.58, 0.50f64.hypot(0.58), 0.50]
            } else {
                [0.50, 0.58, 0.50f64.hypot(0.58)]
            };
            for (dir, (o, p, q)) in [(a, b, c), (b, a, c), (c, a, b)].into_iter().enumerate() {
                for k in 1..=4 {
                    let v = k as f64 / 4.;
                    edge(
                        f,
                        &format!("fiber_{face}_{dir}_{k}"),
                        o.lerp(p, v),
                        o.lerp(q, v),
                        lens[dir] * v,
                        if k == 4 { 0.010 } else { 0.004 },
                        GREEN,
                    );
                }
            }
        }
    }
    let hit = rotor.material().latest_articulated_impact();
    let ends = hit
        .map(|receipt| {
            let l = &receipt.material.loads[0];
            let on = DVec3::from(l.on_rigid);
            let at =
                center + DQuat::from_array(q.orientation) * DVec3::from(receipt.local_points[0]);
            let soft = l
                .site
                .feature
                .vertices()
                .iter()
                .zip(l.site.weights)
                .map(|(&i, w)| w * points[i])
                .sum::<DVec3>();
            ([at, soft], [on, -on])
        })
        .unwrap_or(([center; 2], [DVec3::ZERO; 2]));
    for side in 0..2 {
        for k in 1..=3 {
            let p = ends.0[side] + (0.28 * k as f64 / 3.) * ends.1[side];
            ball(
                f,
                &format!("impulse_{side}_{k}"),
                p,
                if k == 3 { 0.015 } else { 0.009 },
                if side == 0 {
                    [0.95, 0.28, 0.16, 1.]
                } else {
                    [1., 0.76, 0.21, 1.]
                },
            );
        }
    }
    json!({"pad":{"position":q.position,"q":q.orientation,"v":q.velocity,"w":q.angular_velocity},"particles":raw,"event":hit.filter(|e|e.time>previous&&e.time<=rotor.sim.elapsed()).map(impact::event)})
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let (world, control, names, count) = if a.recurrent {
        let first = CLOTH.replace("0.003</max_step_size>", "0.000004</max_step_size>");
        let active = include_str!("../../../examples/rotating_recurrent_cloth.sdf")
            .replace("0.001</max_step_size>", "0.000004</max_step_size>");
        (active, first, ["recurrent", "first_only"], 161)
    } else if a.cloth {
        let s = CLOTH.replace("0.003</max_step_size>", "0.001</max_step_size>");
        (
            s.clone(),
            s.replace("friction=\"0.35\"", "friction=\"0\""),
            ["friction", "smooth"],
            if a.diagnostic_tail { 151 } else { 129 },
        )
    } else {
        let s = CABLE.replace("0.4</max_step_size>", "0.001</max_step_size>");
        let mut off = s.clone();
        let start = off.find("<articulated_impact").unwrap();
        let end = off.find("</articulated_impact>").unwrap() + "</articulated_impact>".len();
        off.replace_range(start..end, "");
        (s, off, ["arc", "clear_chord"], 201)
    };
    let mut pair = [Rotor::new(&world)?, Rotor::new(&control)?];
    let mut writer = a
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut archive = Vec::new();
    for i in 0..count {
        let mut f = Frame::default();
        let mut r = json!({"ordinal":i+1,"cloth":a.cloth});
        for (lane, rotor) in pair.iter_mut().enumerate() {
            let last = rotor.sim.elapsed();
            if i > 0 {
                rotor.advance(if a.recurrent { 500 } else { 2 })?;
            }
            r[names[lane]] = record(rotor, &mut f, names[lane], [0.68, -0.68][lane], last);
            if a.recurrent {
                r[names[lane]]["count"] = json!(rotor.material().articulated_impact_count());
            }
            r["time_ns"] = json!(rotor.sim.elapsed().as_nanos() as u64);
        }
        archive.push(f.finish(pair[0].sim.elapsed(), i + 1));
        if let Some(w) = &mut writer {
            serde_json::to_writer(&mut *w, &r)?;
            writeln!(w)?;
        }
    }
    if !a.cloth {
        // The cable crosses the disk on the circle at the midpoint angle.
        // Its distance to the complete chord includes both physical radii.
        let actual = pair[1].state();
        let c = DVec3::from(actual.position) - PIVOT;
        let first = 0.45 * DVec3::X;
        let pos = DVec3::from(pair[1].material().particles()[0].position);
        let xy = DVec3::new(pos.x, pos.y, 0.);
        let d = c - first;
        let near = first + d * ((xy - first).dot(d) / d.length_squared()).clamp(0., 1.);
        let gap = near.distance(xy) - 0.04;
        if gap < 0.09 {
            return Err(
                "400 ms straight chord unexpectedly encountered or approached the cable".into(),
            );
        }
        println!(
            "Complete no-impulse chord clearance from the actual native endpoints and cable {gap:.12} m."
        );
    }
    if let Some(w) = &mut writer {
        w.flush()?;
    }
    println!(
        "Recorded {count} ordinary comparison frames with complete actual native and material states and their shared off-grid impulse."
    );
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), archive)?;
    Ok(())
}
