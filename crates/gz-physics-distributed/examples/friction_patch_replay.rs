// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay measured coupled friction and saved tilted-plane contact witnesses.
#[path = "support/authored_friction_view.rs"]
mod authored;
#[path = "support/pool_friction_view.rs"]
mod pool;
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::{Parser, ValueEnum};
use gz_physics_distributed::physics::Body;
use primitive_replay_view::View;
use rapier3d_f64::math::{Rotation, Vector};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Clone, Copy, ValueEnum)]
enum Mode {
    Distributed,
    Authored,
    Coupled,
    Tilted,
}
#[derive(Parser)]
struct Args {
    mode: Mode,
    evidence: PathBuf,
    directory: PathBuf,
}
fn file(p: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
fn num(x: &Value) -> Result<f64> {
    x.as_f64().ok_or_else(|| "missing scalar".into())
}
fn v3(x: &Value) -> Result<Vector> {
    Ok(Vector::from(serde_json::from_value::<[f64; 3]>(x.clone())?))
}
const BLUE: [f64; 4] = [0.16, 0.52, 0.84, 1.];
const ORANGE: [f64; 4] = [0.97, 0.47, 0.19, 1.];
const GREEN: [f64; 4] = [0.30, 0.75, 0.52, 1.];
const GOLD: [f64; 4] = [0.96, 0.79, 0.33, 1.];
const GRAY: [f64; 4] = [0.33, 0.38, 0.42, 1.];
struct MeasuredBox {
    size: [f64; 3],
    position: [f64; 3],
    orientation: [f64; 4],
    stripe: bool,
}
impl From<&Body> for MeasuredBox {
    fn from(b: &Body) -> Self {
        Self {
            size: b.size,
            position: b.position,
            orientation: b.orientation,
            stripe: b.id == 2,
        }
    }
}
fn body(v: &mut View, id: u64, name: &str, b: MeasuredBox, offset: Vector, color: [f64; 4]) {
    let p = Vector::from(b.position) + offset;
    v.add(
        id,
        name,
        p.to_array(),
        b.orientation,
        json!({"kind":"box","size":b.size}),
        color,
    );
    if b.stripe {
        let r = Rotation::from_array(b.orientation);
        let mark = p + r * Vector::new(b.size[0] * 0.24, 0., b.size[2] * 0.51);
        v.add(
            id + 2,
            &format!("{name}_material_x"),
            mark.to_array(),
            b.orientation,
            json!({"kind":"box","size":[0.46*b.size[0],0.035,0.018]}),
            GOLD,
        );
    }
}
fn gauge(
    v: &mut View,
    id: u64,
    name: &str,
    value: f64,
    anchor: [f64; 3],
    scale: f64,
    color: [f64; 4],
) {
    let [x, y, z] = anchor;
    v.cuboid(
        id,
        &format!("{name}_{value:.8e}"),
        [x + value * scale * 0.5, y, z],
        [(scale * value).abs().max(0.022), 0.12, 0.12],
        color,
    );
}
fn coupled(row: &Value, v: &mut View, last: &str) -> Result<Value> {
    let t = row["tick"].as_u64().ok_or("missing tick")?;
    let base = 10_000 + t * 100;
    let owner = row["owner"].as_u64().ok_or("owner")?;
    if row["bodies"] != row["reference_bodies"] {
        return Err("unmatched worker".into());
    }
    for (lane, (tag, slice, dy, color)) in [
        (
            "distributed",
            row,
            -1.70,
            if owner == 0 { BLUE } else { ORANGE },
        ),
        ("per_corner", &row["point_control"], 1.70, GREEN),
    ]
    .into_iter()
    .enumerate()
    {
        let [table, block]: [Body; 2] = serde_json::from_value(slice["bodies"].clone())?;
        let offset = Vector::new(0., dy, 0.);
        body(
            v,
            20 + 10 * lane as u64,
            &format!("{tag}_prescribed_table"),
            (&table).into(),
            offset,
            GRAY,
        );
        let id = if lane == 0 { 100 + 10 * owner } else { 180 };
        let model = if lane == 0 {
            format!("{tag}_cuboid_owner_{owner}")
        } else {
            format!("{tag}_cuboid")
        };
        body(v, id, &model, (&block).into(), offset, color);
        let samples = slice["microsteps"].as_array().ok_or("samples missing")?;
        let mut w = 0.;
        let mut b = 0.;
        let mut exceeded = 0;
        for s in samples {
            w += num(&s["work_j"])?;
            b += num(&s["point_bound_j"])?;
            if s["eligible"] == true && num(&s["ratio"])? > 1.01 {
                exceeded += 1;
            }
        }
        if (w - num(&slice["work_j"])?).abs() > 1e-12
            || (b - num(&slice["point_bound_j"])?).abs() > 1e-12
            || slice["violations"] != exceeded
        {
            return Err("microstep totals".into());
        }
        let sy = if lane == 0 { -3.55 } else { 3.55 };
        gauge(
            v,
            base + lane as u64 * 10,
            &format!("tick_{t}_{tag}_measured_work_J"),
            w,
            [-2., sy, 0.03],
            10.,
            if exceeded > 0 { ORANGE } else { GREEN },
        );
        gauge(
            v,
            base + lane as u64 * 10 + 2,
            &format!("tick_{t}_{tag}_pointwise_limit_J"),
            b,
            [-2., sy + if lane == 0 { -0.26 } else { 0.26 }, 0.03],
            10.,
            GOLD,
        );
    }
    v.cuboid(
        base + 30,
        &format!("tick_{t}_latest_{last}"),
        [2.70, -3.55, 0.06],
        [0.18, 0.18, 0.15],
        if owner == 0 { BLUE } else { ORANGE },
    );
    Ok(json!({
        "tick": t,
        "latest": last,
        "owner": owner,
        "work_j": row["work_j"],
        "bound_j": row["point_bound_j"],
        "point_control_work_j": row["point_control"]["work_j"],
        "point_control_bound_j": row["point_control"]["point_bound_j"]
    }))
}
fn tilted(row: &Value, v: &mut View, last: &str) -> Result<Value> {
    let t = row["tick"].as_u64().ok_or("tick")?;
    let base = 10_000 + t * 100;
    let origin = v3(&row["ground"]["point"])?;
    let normal = v3(&row["ground"]["normal"])?;
    let q = Rotation::from_rotation_arc(Vector::Z, normal);
    let p = v3(&row["body"]["position"])?;
    let rot: [f64; 4] = serde_json::from_value(row["body"]["orientation"].clone())?;
    let size: [f64; 3] = serde_json::from_value(row["body"]["size"].clone())?;
    let owner = row["owner"].as_u64().ok_or("owner")?;
    let colors = [BLUE, ORANGE, GREEN];
    v.add(
        10,
        "finite_display_of_shared_infinite_plane",
        (origin - 0.035 * normal).to_array(),
        q.to_array(),
        json!({"kind":"box","size":[4.25,3.1,0.07]}),
        GRAY,
    );
    let id = 100 + 10 * owner;
    v.add(
        id,
        &format!("grounded_body_27_owner_{owner}"),
        p.to_array(),
        rot,
        json!({"kind":"box","size":size}),
        colors[owner as usize],
    );
    let mark = p + Rotation::from_array(rot) * Vector::new(0.18, 0., 0.104);
    v.add(
        180,
        "block_positive_material_x",
        mark.to_array(),
        rot,
        json!({"kind":"box","size":[0.38,0.036,0.024]}),
        GOLD,
    );
    let inset = origin + q * Vector::new(-0.76, -2.60, 0.);
    v.add(
        200,
        "tilted_plane_pressure_inset",
        (inset - 0.035 * normal).to_array(),
        q.to_array(),
        json!({"kind":"box","size":[2.7,1.75,0.07]}),
        GRAY,
    );
    let mut count = 0;
    let mut total = 0.;
    let mut max_plane = 0.0_f64;
    let mut data = vec![];
    let patches = row["patches"].as_array().ok_or("patches")?;
    let mut pts = vec![];
    for patch in patches {
        if patch["body"] != 27 || (v3(&patch["normal_on_body"])? - normal).length() > 1e-12 {
            return Err("wrong patch".into());
        }
        for pt in patch["points"].as_array().ok_or("points")? {
            pts.push(pt)
        }
    }
    let center = if pts.is_empty() {
        Vector::ZERO
    } else {
        pts.iter().try_fold(Vector::ZERO, |v, p| {
            Ok::<_, Box<dyn Error>>(v + v3(&p["plane_witness"])?)
        })? / pts.len() as f64
    };
    for pt in pts {
        let pos = v3(&pt["plane_witness"])?;
        let n = num(&pt["normal_impulse"])?;
        let plane = (pos - origin).dot(normal).abs();
        max_plane = max_plane.max(plane);
        let h = (n * row["substeps"].as_f64().unwrap_or(1.) / 0.01 * 0.075).max(0.025);
        let origin2 = inset + 2.45 * (pos - center);
        v.add(
            base + count * 4,
            &format!("tick_{t}_ground_witness_{count}"),
            pos.to_array(),
            q.to_array(),
            json!({"kind":"sphere","radius":0.046}),
            GOLD,
        );
        v.add(
            base + count * 4 + 2,
            &format!("tick_{t}_corner_{count}_normal_{n:.8e}_N_s"),
            (origin2 + h * normal * 0.5).to_array(),
            q.to_array(),
            json!({"kind":"box","size":[0.12,0.12,h]}),
            GOLD,
        );
        count += 1;
        total += n;
        data.push(json!({"witness":pos.to_array(),"normal_ns":n}));
    }
    if max_plane > 1e-10 || (total - num(&row["last_native_normal_ns"])?).abs() > 1e-10 {
        return Err("world plane or native point sum".into());
    }
    gauge(
        v,
        base + 28,
        &format!("tick_{t}_signed_angular_speed_rad_s"),
        num(&row["angular_speed_rad_s"])?,
        [4.0, -4.5, 0.6],
        0.14,
        BLUE,
    );
    v.cuboid(
        base + 30,
        &format!("tick_{t}_latest_{last}"),
        [4.2, -4.5, 0.6],
        [0.18, 0.18, 0.15],
        colors[owner as usize],
    );
    Ok(
        json!({"tick":t,"latest":last,"owner":owner,"witnesses":data,"last_normal_ns":total,"max_plane_m":max_plane}),
    )
}
fn main() -> Result<()> {
    let a = Args::parse();
    if matches!(a.mode, Mode::Distributed) {
        return pool::export(&a.evidence, &a.directory);
    }
    if matches!(a.mode, Mode::Authored) {
        return authored::export(&a.evidence, &a.directory);
    }
    let mut rows = vec![];
    if matches!(a.mode, Mode::Tilted) {
        rows.push(file(&a.evidence.join("initial.json"))?);
    }
    for l in BufReader::new(File::open(a.evidence.join("frames.jsonl"))?).lines() {
        rows.push(serde_json::from_str(&l?)?)
    }
    let mut stream = vec![];
    let mut samples = vec![];
    let mut last = "before_drive".to_string();
    for (i, row) in rows.iter().enumerate() {
        let tick = row["tick"].as_u64().ok_or("tick")?;
        for event in row["events"].as_array().ok_or("events")? {
            let e = event.get("event").unwrap_or(event);
            last = format!("{}_at_{tick}", e["kind"].as_str().ok_or("event label")?);
        }
        let mut v = View::default();
        if tick != i as u64 + u64::from(matches!(a.mode, Mode::Coupled)) {
            return Err("nonconsecutive native tick".into());
        }
        samples.push(match a.mode {
            Mode::Coupled => coupled(row, &mut v, &last)?,
            Mode::Tilted => tilted(row, &mut v, &last)?,
            Mode::Authored | Mode::Distributed => unreachable!(),
        });
        stream.push(v.finish_at(i as u64 + 1, i as u64 * 10_000_000));
    }
    let required = if matches!(a.mode, Mode::Tilted) {
        121
    } else {
        120
    };
    if stream.len() != required {
        return Err("missing source frames".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    serde_json::to_writer_pretty(
        File::create(a.directory.join("index.json"))?,
        &json!({"samples":samples}),
    )?;
    println!("Verified and exported {required} consecutive measured source frames.");
    Ok(())
}
