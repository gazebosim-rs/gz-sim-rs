// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeSet, time::Duration};
pub(super) fn write(dir: &Path, runs: &[Trial]) -> Result<()> {
    let out = dir.join("viewer");
    fs::create_dir(&out)?;
    let impact = runs[2]
        .frames
        .iter()
        .find(|f| f.normal_impulse_ns > 0.)
        .ok_or("missing impact")?
        .tick;
    let mut ticks = (0..=END).step_by(8).collect::<BTreeSet<_>>();
    let mut marks = serde_json::Map::new();
    for (label, t) in [
        ("end_of_shove", KICK_END),
        ("ground_impact", impact),
        ("end", END),
    ] {
        ticks.insert(t);
        marks.insert(label.to_owned(), json!(t));
    }
    for v in marks.values_mut() {
        let t = v.as_u64().unwrap() as usize;
        *v = json!(ticks.iter().position(|n| *n == t).unwrap());
    }
    let max_j = runs
        .iter()
        .flat_map(|r| &r.frames)
        .map(|f| f.normal_impulse_ns)
        .fold(0., f64::max);
    let axes = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let mut archive = vec![];
    for (seq, &tick) in ticks.iter().enumerate() {
        let mut scene = soft_replay::Frame::default();
        for (i, run) in runs.iter().enumerate() {
            let f = &run.frames[tick];
            let offset = (i as f64 - 1.5) * 0.59 * DVec3::Y;
            let colors = if run.welded {
                [[0.14, 0.82, 0.76, 1.], [0.93, 0.68, 0.22, 1.]]
            } else {
                [[0.40, 0.66, 0.96, 1.], [0.84, 0.70, 0.96, 1.]]
            };
            for ((name, link), color) in [("foot", &f.foot), ("upper", &f.upper)]
                .into_iter()
                .zip(colors)
            {
                scene.add(
                    &format!("{} {name}", run.name),
                    offset + axes * DVec3::from(link.p),
                    axes * DQuat::from_array(link.q),
                    json!({"kind":"box","size":link.size}),
                    color,
                );
            }
            if run.ground {
                scene.add(
                    &format!("{} floor section", run.name),
                    offset - 0.01 * DVec3::Z,
                    DQuat::IDENTITY,
                    json!({"kind":"box","size":[0.35,0.48,0.02]}),
                    [0.35, 0.39, 0.45, 1.],
                );
                let center = offset - 0.08 * DVec3::Z;
                scene.edge(
                    &format!("{} impulse scale", run.name),
                    center - 0.17 * DVec3::Y,
                    center + 0.17 * DVec3::Y,
                    0.34,
                    0.006,
                    [0.48, 0.56, 0.65, 1.],
                );
                scene.add(
                    &format!("{} ground impulse", run.name),
                    center + (f.normal_impulse_ns / max_j - 0.5) * 0.34 * DVec3::Y,
                    DQuat::IDENTITY,
                    json!({"kind":"box","size":[0.015,0.012,0.03]}),
                    colors[1],
                );
            }
        }
        archive.push(scene.finish(Duration::from_micros(tick as u64 * 500), seq + 1));
    }
    gz_sim_cli::export_archive_at(
        &out.join("replay"),
        Default::default(),
        0,
        Duration::ZERO,
        archive,
    )?;
    fs::write(
        out.join("frames.json"),
        serde_json::to_vec_pretty(
            &json!({"kind":"authored-archive-v1","dt_ns":1,"welded_foot":true,"captures":marks}),
        )?,
    )?;
    Ok(())
}
