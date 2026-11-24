// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{BLUE, GOLD, GRAY, GREEN, MeasuredBox, ORANGE, Result, View, body, gauge};
use gz_physics_distributed::physics::{Body, ContactImpulse};
use rapier3d_f64::math::Vector;
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Deserialize)]
struct Metric {
    work_j: f64,
    point_bound_j: f64,
    violations: u64,
    microsteps: Vec<Value>,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    time_ns: u64,
    bodies: Vec<Body>,
    reference: Vec<Body>,
    owners: Vec<u8>,
    pointwise: bool,
    pairs: Vec<ContactImpulse>,
    events: Vec<Value>,
    metrics: Vec<Metric>,
    substeps: u8,
}
fn rows(p: &Path) -> Result<Vec<Frame>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect()
}
pub(super) fn export(src: &Path, out: &Path) -> Result<()> {
    let data = [
        rows(&src.join("grouped.jsonl"))?,
        rows(&src.join("pointwise.jsonl"))?,
    ];
    let summary: Vec<Value> = serde_json::from_reader(File::open(src.join("summary.json"))?)?;
    let mut digests: [Sha256; 2] = Default::default();
    let mut stream = vec![];
    let mut samples = vec![];
    let mut latest = ["cold_start".to_owned(), "cold_start".to_owned()];
    let mut all_excess = [0, 0];
    let mut all_events = [0, 0];
    for (i, (g, p)) in data[0].iter().zip(&data[1]).enumerate() {
        let t = i as u64 + 1;
        if g.tick != t || p.tick != t || g.time_ns != p.time_ns || g.time_ns != t * 10_000_000 {
            return Err("distributed comparison clocks differ".into());
        }
        let mut v = View::default();
        let base = 2_000_000 + t * 100;
        let mut ids = vec![];
        for (lane, (name, r)) in [("grouped", g), ("per_corner", p)].into_iter().enumerate() {
            if r.pointwise != (lane == 1)
                || r.bodies != r.reference
                || r.bodies.len() != 4
                || r.metrics.len() != 2
                || r.owners.len() != 4
            {
                return Err("missing checked distributed bodies".into());
            }
            digests[lane].update(bincode::serialize(&(t, &r.bodies, &r.pairs))?);
            for e in &r.events {
                latest[lane] = format!("{}_at_{t}", e["kind"].as_str().ok_or("event")?);
                all_events[lane] += 1;
            }
            for site in 0..2 {
                // Arrange the two sites into rows and the two profiles into columns using translations alone.
                let x = if lane == 0 { -4.35 } else { 4.35 };
                let dy = if site == 0 { -1.65 } else { 1.65 - 20. };
                for j in 0..2 {
                    let idx = 2 * site + j;
                    let b = &r.bodies[idx];
                    let expected = 1 + 10 * site as u64 + j as u64;
                    if b.id != expected {
                        return Err("unexpected physical membership".into());
                    }
                    let owner = r.owners[idx];
                    let id = 100 + (lane * 4 + idx) as u64 * 80 + u64::from(owner) * 6;
                    let mut geom = MeasuredBox::from(b);
                    geom.stripe = j == 1;
                    let label = format!(
                        "{name}_site_{site}_{}_{}_owner_{owner}",
                        if j == 0 { "table" } else { "block" },
                        b.id
                    );
                    body(
                        &mut v,
                        id,
                        &label,
                        geom,
                        Vector::new(x, dy, 0.),
                        if j == 0 {
                            GRAY
                        } else if lane == 0 {
                            BLUE
                        } else {
                            GREEN
                        },
                    );
                    ids.push(
                        json!({"profile":name,"site":site,"body":b.id,"owner":owner,"label":label}),
                    );
                }
                let metric = &r.metrics[site];
                let work = metric
                    .microsteps
                    .iter()
                    .map(|m| m["work_j"].as_f64().unwrap())
                    .sum::<f64>();
                let bound = metric
                    .microsteps
                    .iter()
                    .map(|m| m["point_bound_j"].as_f64().unwrap())
                    .sum::<f64>();
                let violations = metric
                    .microsteps
                    .iter()
                    .filter(|m| m["eligible"] == true && m["ratio"].as_f64().unwrap() > 1.01)
                    .count() as u64;
                if work != metric.work_j
                    || bound != metric.point_bound_j
                    || violations != metric.violations
                    || metric.microsteps.len() != usize::from(r.substeps)
                {
                    return Err("per-step work does not reproduce distributed study".into());
                }
                all_excess[lane] += violations;
                let y = if site == 0 { -3.43 } else { 3.43 };
                let arrow = if site == 0 { -1. } else { 1. };
                let id = base + (lane * 2 + site) as u64 * 10;
                gauge(
                    &mut v,
                    id,
                    &format!("tick_{t}_{name}_site_{site}_opposing_work_j"),
                    work,
                    [x - 1.8, y, 0.1],
                    9.,
                    if violations > 0 { ORANGE } else { GREEN },
                );
                gauge(
                    &mut v,
                    id + 2,
                    &format!("tick_{t}_{name}_site_{site}_four_point_bound_j"),
                    bound,
                    [x - 1.8, y + arrow * 0.27, 0.1],
                    9.,
                    GOLD,
                );
            }
            v.cuboid(
                base + 50 + lane as u64 * 2,
                &format!("tick_{t}_{name}_latest_{}", latest[lane]),
                [if lane == 0 { -1.2 } else { 7.5 }, -3.43, 0.1],
                [0.25, 0.25, 0.2],
                if lane == 0 { BLUE } else { GREEN },
            );
        }
        samples.push(
            json!({"tick":t,"objects":ids,"events_grouped":g.events,"events_per_corner":p.events}),
        );
        stream.push(v.finish_at(t, (t - 1) * 10_000_000));
    }
    let mut referenced = vec![];
    for (lane, h) in digests.into_iter().enumerate() {
        let digest = format!("{:x}", h.finalize());
        let run = summary
            .iter()
            .find(|v| {
                v["pointwise"] == (lane == 1)
                    && v["extended"] == false
                    && v["split"] == true
                    && v["packed"] == true
                    && v["width"] == 4
                    && v["drive"] == "Mixed"
            })
            .ok_or("missing representative run")?;
        if run["trajectory_sha256"] != digest
            || run["violations"] != all_excess[lane]
            || all_events[lane] != 5
        {
            return Err("unmatched native digest, violation count or faults".into());
        }
        referenced.push(run);
    }
    if stream.len() != 120 || data[0].len() != data[1].len() || all_excess[1] != 0 {
        return Err("incomplete distributed recording".into());
    }
    fs::create_dir(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), stream)?;
    serde_json::to_writer_pretty(
        File::create(out.join("index.json"))?,
        &json!({"runs":referenced,"samples":samples}),
    )?;
    println!(
        "Verified 120 physical barriers per profile, two complete distributed site pairs, native physical digests, actual ownership and all work/corner bounds."
    );
    Ok(())
}
