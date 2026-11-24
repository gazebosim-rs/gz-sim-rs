// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeSet, path::Path, time::Duration};
fn cuboid(size: [f64; 3]) -> serde_json::Value {
    json!({"kind":"box","size":size})
}

pub(super) fn write(dir: &Path, histories: &[MaterialRun]) -> Result<()> {
    let out = dir.join("viewer");
    fs::create_dir(&out)?;
    let stride = 10;
    let mut ticks = (1..END).step_by(stride).collect::<BTreeSet<_>>();
    let ret = histories[0].samples[RETURN..]
        .iter()
        .find(|s| s.panels[1].normal_impulse_ns > 0.)
        .ok_or("missing return")?
        .tick;
    let mut marks = serde_json::Map::new();
    for (name, tick) in [
        ("both_loaded", SETTLE),
        ("right_panel_lowered", LOWER_END),
        ("right_panel_returns", ret),
        ("final_load", END),
    ] {
        ticks.insert(tick);
        marks.insert(name.to_owned(), json!(tick));
    }
    for mark in marks.values_mut() {
        let tick = mark.as_u64().unwrap() as usize;
        *mark = json!(ticks.iter().position(|v| *v == tick).unwrap());
    }
    let max_force = histories
        .iter()
        .flat_map(|h| &h.samples)
        .flat_map(|s| &s.panels)
        .map(|p| p.force_on_foot_n[2])
        .fold(0., f64::max);
    let axes = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let colors = [[0.10, 0.80, 0.78, 1.], [0.99, 0.66, 0.16, 1.]];
    let mut archive = Vec::new();
    for (sequence, tick) in ticks.iter().enumerate() {
        let mut frame = soft_replay::Frame::default();
        for (lane, h) in histories.iter().enumerate() {
            let s = &h.samples[tick - 1];
            let offset = DVec3::Y * (lane as f64 - 0.5);
            let place = |p: [f64; 3]| axes * DVec3::from(p) + offset;
            frame.add(
                &format!("{} foot", h.name),
                place(s.center_m),
                axes * DQuat::from_array(s.orientation_xyzw),
                cuboid([0.36, 0.2, 0.12]),
                [0.78, 0.85, 0.96, 1.],
            );
            for (i, p) in s.panels.iter().enumerate() {
                let side = ["left", "right"][i];
                frame.add(
                    &format!("{} {side} panel", h.name),
                    place(p.center_m),
                    axes,
                    cuboid([0.4, 0.32, 0.06]),
                    colors[i],
                );
                // A fixed rail and a traveling cursor keep the archive's size and shape stable.
                let center = place([p.center_m[0], 0., -0.18]);
                frame.edge(
                    &format!("{} {side} load scale", h.name),
                    center - 0.14 * DVec3::Y,
                    center + 0.14 * DVec3::Y,
                    0.28,
                    0.004,
                    [0.48, 0.55, 0.61, 1.],
                );
                let cursor = center + (p.force_on_foot_n[2] / max_force - 0.5) * 0.28 * DVec3::Y;
                frame.add(
                    &format!("{} {side} normal load", h.name),
                    cursor,
                    DQuat::IDENTITY,
                    cuboid([0.014, 0.01, 0.023]),
                    colors[i],
                );
            }
        }
        archive.push(frame.finish(Duration::from_micros(*tick as u64 * 500), sequence + 1));
    }
    gz_sim_cli::export_archive_at(
        &out.join("replay"),
        Default::default(),
        0,
        Duration::from_micros(500),
        archive,
    )?;
    fs::write(
        out.join("frames.json"),
        serde_json::to_vec_pretty(&json!({"kind":"authored-archive-v1","dt_ns":1,
       "moving_seam":true,"origin_ns":500000,"captures":marks,"load_scale_n":max_force}))?,
    )?;
    Ok(())
}
