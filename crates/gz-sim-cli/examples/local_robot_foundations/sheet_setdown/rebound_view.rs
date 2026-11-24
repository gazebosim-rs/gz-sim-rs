// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::fmt::Write;

/// Place the three measured copies side by side, at unchanged physical scale.
pub(super) fn save(out: &Path, seed: &Seed, lanes: &[Vec<Sample>]) -> Result<()> {
    let Boundary::TranslatingCuboid {
        center,
        half_extents,
        ..
    } = seed.bench
    else {
        return Err("missing finite receiver".into());
    };
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><physics name='base' type='ignored'><max_step_size>0.001</max_step_size></physics>",
    );
    let mut names = Vec::new();
    let colors = ["1 .48 .16", ".15 .72 .52", ".24 .49 .96"];
    let mut offsets = Vec::new();
    for (lane, (rate, color)) in [16, 64, 4000].into_iter().zip(colors).enumerate() {
        let shift = (lane as f64 - 1.) * (2. * half_extents[0] + 0.025);
        offsets.push(shift);
        let label = format!("sheet_{rate}");
        write!(
            xml,
            "<model name='{label}'><static>true</static><link name='deck'><pose>{} {} {} 0 0 0</pose>",
            center[0] + shift,
            center[1],
            center[2]
        )?;
        let [x, y, z] = half_extents.map(|v| 2. * v);
        let shape = format!("<geometry><box><size>{x} {y} {z}</size></box></geometry>");
        write!(
            xml,
            "<collision name='bench'>{shape}</collision><visual name='bench'>{shape}<material><diffuse>.30 .34 .39 1</diffuse></material></visual>"
        )?;
        for node in 0..arm_sheet::COUNT {
            write!(
                xml,
                "<visual name='p{node}'><geometry><sphere><radius>{}</radius></sphere></geometry><material><diffuse>{color} 1</diffuse></material></visual>",
                arm_sheet::RADIUS
            )?;
            names.push(format!("default::{label}::deck::p{node}"));
        }
        xml.push_str("</link></model>");
    }
    xml.push_str("</world></sdf>");
    let mut r = Recording::new(&xml, "matched-rebound-view.sdf", "default")?;
    let mut truth = Vec::new();
    for i in 0..=HORIZON_MS as usize {
        if i > 0 {
            r.advance()?;
        }
        let (frame, catalog) =
            gz_sim_cli::primitive_frame(&r.sim, &r.sdf, "default", i as u64 + 1)?;
        let mut frame: Value = serde_json::from_str(&frame)?;
        let positions: Vec<_> = lanes
            .iter()
            .zip(&offsets)
            .flat_map(|(lane, x)| {
                lane[i]
                    .positions
                    .iter()
                    .map(move |p| [p[0] + x, p[1], p[2]])
            })
            .collect();
        for v in frame["visuals"].as_array_mut().ok_or("missing visuals")? {
            if let Some(index) = names.iter().position(|name| v["name"] == *name) {
                v["translation"] = json!(positions[index]);
            }
        }
        truth.push(json!({"time_ns":i as u64*MS,"positions":positions}));
        r.frames.push((frame.to_string(), catalog));
    }
    r.save(out)?;
    write_json(&out.join("particle-frames.json"), &json!(truth))?;
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"particle_visuals":names,"captures":{"arrival":1,"coarse_peak":17,"middle_peak":35,"fine_peak":49,"later":75},"lane_rates":[16,64,4000],"lane_offsets_x_m":offsets}),
    )
}
