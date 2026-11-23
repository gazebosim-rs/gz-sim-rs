// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare response traffic from two real SDF runs with identical GPU observations.
#[path = "support/fleet_visual.rs"]
mod visual;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};
use visual::{box_visual, catalog};

fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("resident_reply_replay RESULTS_DIRECTORY")?,
    );
    let load = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_reader(File::open(
            root.join(name).join("index.json"),
        )?)?)
    };
    let a = load("portable")?;
    let b = load("observations")?;
    let rows = a["frames"].as_array().ok_or("missing portable frames")?;
    let compact = b["frames"].as_array().ok_or("missing observation frames")?;
    if rows.len() != compact.len() {
        return Err("different frame counts".into());
    }
    let physical = fs::read(root.join("observations/replay/frames.jsonl"))?;
    if physical != fs::read(root.join("portable/replay/frames.jsonl"))? {
        return Err("SDF runs published different scenes".into());
    }
    let mut lines = BufReader::new(physical.as_slice()).lines();
    lines.next().ok_or("missing header")??;
    let mut frames = Vec::new();
    let mut measurements = Vec::new();
    let mut cumulative = [0u64; 2];
    for (left, right) in rows.iter().zip(compact) {
        for key in [
            "tick",
            "pairs",
            "cut_sha256",
            "imu",
            "killed",
            "stopped",
            "restarts",
            "workers",
        ] {
            if left[key] != right[key] {
                return Err(format!("different {key} at {}", left["tick"]).into());
            }
        }
        let raw: Value = serde_json::from_str(&lines.next().ok_or("missing physical frame")??)?;
        let mut scene: Value = serde_json::from_str(raw["scene"].as_str().ok_or("missing scene")?)?;
        let mut models: Value =
            serde_json::from_str(raw["catalog"].as_str().ok_or("missing catalog")?)?;
        for (i, (row, name, color)) in [
            (left, "portable_reply_bytes", [0.96, 0.46, 0.09]),
            (right, "measured_reply_bytes", [0.06, 0.72, 0.61]),
        ]
        .into_iter()
        .enumerate()
        {
            let bytes = row["reply_bytes"].as_u64().ok_or("missing byte count")?;
            cumulative[i] += bytes;
            // Each marker moves up a fixed metre scale as cumulative reply traffic grows.
            let height = cumulative[i] as f64 / 150000.;
            let label = format!("default::{name}");
            let id = 40 + i as u64 * 3;
            scene["visuals"].as_array_mut().unwrap().push(box_visual(
                id + 1,
                &format!("{label}::marker"),
                [-0.04, -0.70 - i as f64 * 0.18, height - 0.15],
                [0., 0., 0., 1.],
                [0.13, 0.13, 0.035],
                color,
            ));
            scene["visuals"].as_array_mut().unwrap().push(box_visual(
                id + 2,
                &format!("{label}::scale"),
                [-0.04, -0.70 - i as f64 * 0.18, 0.35],
                [0., 0., 0., 1.],
                [0.018, 0.018, 1.0],
                [0.34, 0.39, 0.43],
            ));
            models["models"]
                .as_array_mut()
                .unwrap()
                .push(catalog(id, &label, &[id + 1, id + 2]));
        }
        measurements.push(json!({"tick":left["tick"],"cumulative_portable_bytes":cumulative[0],"cumulative_measured_bytes":cumulative[1],
            "portable_wait_us":left["wait_us"],"measured_wait_us":right["wait_us"],"cut_sha256":left["cut_sha256"],"pairs":left["pairs"]}));
        frames.push((
            serde_json::to_string(&scene)?,
            serde_json::to_string(&models)?,
        ));
    }
    let dir = root.join("comparison");
    fs::create_dir_all(&dir)?;
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    fs::write(
        dir.join("index.json"),
        serde_json::to_vec(&json!({"frames":measurements,"bytes":cumulative}))?,
    )?;
    println!(
        "{} matching SDF cuts, IMU samples and complete primitive scenes. Portable replies {} bytes, measured-row replies {} bytes.",
        rows.len(),
        cumulative[0],
        cumulative[1]
    );
    Ok(())
}
