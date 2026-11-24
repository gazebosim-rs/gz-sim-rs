// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the canonical two-finger replay to its public encoders and contact impulses.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::collections::BTreeMap;
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn main() -> Result<()> {
    let [trace, archive]: [PathBuf; 2] = std::env::args_os()
        .skip(1)
        .map(PathBuf::from)
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|_| "usage: two_finger_view TRACE.jsonl CAPTURE/epoch-0")?;
    let mut observations = BTreeMap::new();
    let mut trace = BufReader::new(File::open(trace)?).lines();
    let profile: Value = serde_json::from_str(&trace.next().ok_or("missing profile")??)?;
    for line in trace {
        let row: Value = serde_json::from_str(&line?)?;
        // A held input changes at the same clock. Its observation repeats the preceding reaction.
        observations.insert(row["step"].as_u64().ok_or("missing step")?, row);
    }
    let mut frames = BTreeMap::new();
    for line in BufReader::new(File::open(archive.join("frames.jsonl"))?).lines() {
        let record: Value = serde_json::from_str(&line?)?;
        if record["kind"] == "frame" {
            let scene: Value = serde_json::from_str(record["scene"].as_str().ok_or("scene text")?)?;
            let time = scene["seconds"].as_u64().ok_or("seconds")? * 1_000_000_000
                + scene["nanoseconds"].as_u64().ok_or("nanoseconds")?;
            if time != frames.len() as u64 * 1_000_000 {
                return Err("expected exactly one frame per millisecond".into());
            }
            frames.insert(time, scene);
        }
    }
    let dt_ns = profile["step_ns"].as_u64().ok_or("step size")?;
    if dt_ns != 100_000 || observations.len() != 2901 || frames.len() != 291 {
        return Err("expected the full canonical 290 ms run and its initial state".into());
    }
    let dt = dt_ns as f64 * 1e-9;
    let mut center = DVec3::new(0., 0., 0.30);
    let mut velocity = DVec3::ZERO;
    let mut parcel_error = DVec3::ZERO;
    let mut pad_error_m: f64 = 0.;
    for (expected, (step, row)) in observations.into_iter().enumerate() {
        if step != expected as u64 || row["time_ns"] != step * dt_ns {
            return Err("discontinuous trace".into());
        }
        if step > 0 {
            let mut delta_v = -9.81 * dt * DVec3::Z;
            let pairs = row["pair_reactions_ns"]
                .as_array()
                .ok_or("missing pair reactions")?;
            let contacts = row["pair_impulses"].as_array();
            if contacts.map_or(0, Vec::len) != pairs.len() {
                return Err("missing vector for a contact pair".into());
            }
            if let Some(contacts) = contacts {
                for pair in contacts {
                    if !matches!(pair["a"].as_u64(), Some(1 | 2)) || pair["b"] != 3 {
                        return Err("expected only pad-on-parcel impulses".into());
                    }
                    let n: [f64; 3] = serde_json::from_value(pair["normal_on_a_ns"].clone())?;
                    let t: [f64; 3] = serde_json::from_value(pair["tangent_on_a_ns"].clone())?;
                    delta_v -= (DVec3::from(n) + DVec3::from(t)) / 0.20;
                }
            }
            // Impulses are published per public tick. Center them in that interval.
            center += (velocity + 0.5 * delta_v) * dt;
            velocity += delta_v;
        }
        if let Some(scene) = frames.remove(&(dt_ns * step)) {
            let joints = row["scalar_joints"].as_array().ok_or("missing encoders")?;
            let position = |id: u64| -> Result<f64> {
                Ok(joints
                    .iter()
                    .find(|j| j["id"] == id)
                    .and_then(|j| j["position"].as_f64())
                    .ok_or("missing joint position")?)
            };
            let rotation = DQuat::from_rotation_z(position(12)?);
            let visuals = scene["visuals"].as_array().ok_or("missing visuals")?;
            let point = |name: &str| -> Result<DVec3> {
                let v = visuals
                    .iter()
                    .find(|v| v["name"] == name)
                    .ok_or("missing visual")?;
                Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(
                    v["translation"].clone(),
                )?))
            };
            parcel_error =
                parcel_error.max((point("default::parcel::body::sphere")? - center).abs());
            for (name, x) in [
                ("default::gripper::left::pad", position(13)? - 0.025),
                ("default::gripper::right::pad", 0.025 - position(14)?),
            ] {
                let predicted = rotation * DVec3::new(x, 0., 0.30);
                pad_error_m = pad_error_m.max(predicted.distance(point(name)?));
            }
        }
    }
    if !frames.is_empty() || pad_error_m > 1e-12 || parcel_error.length() > 25e-6 {
        return Err(format!(
            "unmatched frame or trajectory error: {pad_error_m} m, {parcel_error} m"
        )
        .into());
    }
    println!(
        "{}",
        json!({"max_pad_center_error_m":pad_error_m,"max_parcel_error_by_axis_m":parcel_error.to_array(),"final_velocity_from_impulses_m_s":velocity.to_array()})
    );
    Ok(())
}
