// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pressure markers alongside the unchanged native robot.
use super::*;
use std::fmt::Write;

const MARKER: &str = "default::source::body::pressure_";

pub(super) fn world(xml: &str) -> String {
    let mut gauges = String::new();
    // The source link origin is (0.32, 0, 0.06). Each cluster belongs to a jaw.
    for (level, color) in [
        "1 0.63 0.10",
        "0.12 0.84 0.52",
        "0.30 0.62 1",
        "0.70 0.42 0.98",
    ]
    .into_iter()
    .enumerate()
    {
        for side in 0..2 {
            let x = 0.27 + 0.115 * side as f64 + 0.022 * level as f64;
            write!(gauges,r#"<visual name="pressure_{level}_{side}"><pose>{x} 0 0.31 0 0 0</pose><geometry><sphere><radius>0.007</radius></sphere></geometry><material><diffuse>{color} 1</diffuse></material></visual>"#).unwrap();
        }
    }
    for (label, force) in [("zero", 0.), ("one_newton", 1.)] {
        let z = 0.31 + height(force);
        write!(gauges,r#"<visual name="squeeze_{label}"><pose>0.36 0.012 {z} 0 0 0</pose><geometry><box><size>0.222 0.003 0.002</size></box></geometry><material><diffuse>0.63 0.68 0.74 1</diffuse></material></visual>"#).unwrap();
    }
    xml.replacen(
        "<visual name=\"sheet_0\"",
        &(gauges + "<visual name=\"sheet_0\""),
        1,
    )
}

fn height(force: f64) -> f64 {
    0.06 * force.asinh()
}

pub(super) fn capture(r: &mut Recording, forces: &[[f64; 2]; 4]) -> Result<()> {
    let (raw, catalog) =
        gz_sim_cli::primitive_frame(&r.sim, &r.sdf, &r.world, r.frames.len() as u64 + 1)?;
    let mut scene: Value = serde_json::from_str(&raw)?;
    let mut found = 0;
    for visual in scene["visuals"]
        .as_array_mut()
        .ok_or("missing gripper visuals")?
    {
        let Some(suffix) = visual["name"].as_str().and_then(|s| s.strip_prefix(MARKER)) else {
            continue;
        };
        let (level, side) = suffix.split_once('_').ok_or("invalid pressure marker")?;
        let value = forces
            .get(level.parse::<usize>()?)
            .and_then(|p| p.get(side.parse::<usize>().ok()?))
            .ok_or("unknown pressure marker")?;
        let z = visual["translation"][2]
            .as_f64()
            .ok_or("missing marker height")?;
        visual["translation"][2] = json!(z + height(*value));
        found += 1;
    }
    if found != 8 {
        return Err("incomplete pressure gauges".into());
    }
    r.frames.push((scene.to_string(), catalog));
    Ok(())
}

pub(super) fn save(r: Recording, out: &Path, frames: &[Value]) -> Result<()> {
    let last = frames.len() - 1;
    let origin = frames[0]["time_ns"]
        .as_u64()
        .ok_or("missing physical origin")?;
    gz_sim_cli::export_archive_at(
        &out.join("replay"),
        Default::default(),
        0,
        Duration::from_nanos(origin),
        r.frames,
    )?;
    write_json(&out.join("particle-frames.json"), &json!(frames))?;
    let nodes = (0..arm_sheet::COUNT)
        .map(arm_sheet::name)
        .collect::<Vec<_>>();
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"right","particle_visuals":nodes,"captures":{"early":last/4,"middle":last/2,"late":3*last/4,"end":last}}),
    )
}
