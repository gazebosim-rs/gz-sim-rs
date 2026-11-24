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

/// The journal publishes commands before the next advancing tick. Only the held inputs may change.
fn same_clock_input(previous: &Value, row: &Value) -> bool {
    let command = row["command"].as_u64();
    matches!(command, Some(1 | 2 | 4 | 5 | 7 | 8 | 10 | 11))
        && previous["command"].as_u64().and_then(|v| v.checked_add(1)) == command
        && row["inputs"] != previous["inputs"]
        && row["held_joint_forces"] != previous["held_joint_forces"]
        && [
            "step",
            "time_ns",
            "epoch",
            "scene",
            "physics",
            "contacts",
            "scalar_joints",
            "pair_reactions_ns",
            "pair_impulses",
        ]
        .iter()
        .all(|k| row[k] == previous[k])
}

fn push_observation(observations: &mut Vec<Value>, row: Value) -> Result<()> {
    let step = row["step"].as_u64().ok_or("missing step")?;
    if step == observations.len() as u64 {
        observations.push(row);
    } else {
        let previous = observations.last_mut().ok_or("missing initial state")?;
        if !same_clock_input(previous, &row) {
            return Err("out-of-order step or a same-clock input changed physical state".into());
        }
        *previous = row;
    }
    Ok(())
}

fn parcel_impulse(row: &Value) -> Result<DVec3> {
    let mut impulse = DVec3::ZERO;
    let pairs = row["pair_reactions_ns"]
        .as_array()
        .ok_or("missing pair reactions")?;
    let contacts = row["pair_impulses"].as_array();
    if contacts.map_or(0, Vec::len) != pairs.len() {
        return Err("missing vector for a contact pair".into());
    }
    if let Some(contacts) = contacts {
        let mut previous_pad = 0;
        for (pair, scalar) in contacts.iter().zip(pairs) {
            let pad = pair["a"].as_u64().ok_or("missing pad id")?;
            if !matches!(pad, 1 | 2)
                || pair["b"] != 3
                || pad <= previous_pad
                || scalar[0] != pad
                || scalar[1] != pair["b"]
            {
                return Err("expected matching ordered pad-on-parcel pairs".into());
            }
            previous_pad = pad;
            let load = scalar[2].as_f64().ok_or("missing scalar normal impulse")?;
            let n = DVec3::from(serde_json::from_value::<[f64; 3]>(
                pair["normal_on_a_ns"].clone(),
            )?);
            let t = DVec3::from(serde_json::from_value::<[f64; 3]>(
                pair["tangent_on_a_ns"].clone(),
            )?);
            // A sphere loads one planar face on either side. The impulse on the
            // left pad points toward negative X, and on the right toward positive X.
            let outward = if pad == 1 { -n.x } else { n.x };
            if (load - n.length()).abs() > 8.0 * f64::EPSILON * load
                || (load > 0.0 && outward <= 0.0)
            {
                return Err("normal vector disagrees with its pad load or direction".into());
            }
            impulse -= n + t;
        }
    }
    Ok(impulse)
}

fn main() -> Result<()> {
    let [trace, archive]: [PathBuf; 2] = std::env::args_os()
        .skip(1)
        .map(PathBuf::from)
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|_| "usage: two_finger_view TRACE.jsonl CAPTURE/epoch-0")?;
    let mut observations = Vec::new();
    let mut trace = BufReader::new(File::open(trace)?).lines();
    let profile: Value = serde_json::from_str(&trace.next().ok_or("missing profile")??)?;
    for line in trace {
        let row: Value = serde_json::from_str(&line?)?;
        push_observation(&mut observations, row)?;
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
    for (step, row) in observations.into_iter().enumerate() {
        let step = step as u64;
        if row["time_ns"] != step * dt_ns {
            return Err("discontinuous trace".into());
        }
        if step > 0 {
            let mut delta_v = -9.81 * dt * DVec3::Z;
            delta_v += parcel_impulse(&row)? / 0.20;
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

#[cfg(test)]
mod tests {
    use super::*;

    fn tick(step: u64, command: u64) -> Value {
        json!({"step":step,"command":command,"time_ns":step*100_000,
            "physics":step,"contacts":step,"inputs":command,"held_joint_forces":[command]})
    }

    #[test]
    fn only_input_commands_can_repeat_an_unchanged_physical_tick() {
        let mut rows = vec![];
        push_observation(&mut rows, tick(0, 0)).unwrap();
        push_observation(&mut rows, tick(0, 1)).unwrap();
        push_observation(&mut rows, tick(0, 2)).unwrap();
        assert_eq!(rows.len(), 1);
        assert!(push_observation(&mut rows, tick(2, 3)).is_err());
        assert!(push_observation(&mut rows, tick(0, 3)).is_err());
        push_observation(&mut rows, tick(1, 3)).unwrap();
        assert!(push_observation(&mut rows, tick(0, 4)).is_err());
        for key in ["physics", "contacts", "pair_impulses", "scalar_joints"] {
            let mut changed = tick(1, 4);
            changed[key] = json!("changed");
            assert!(push_observation(&mut rows, changed).is_err(), "{key}");
        }
    }

    #[test]
    fn per_pad_checks_catch_corruption_that_preserves_the_total() {
        let left = json!({"a":1,"b":3,"normal_on_a_ns":[-2,0,0],"tangent_on_a_ns":[0,0,-1]});
        let right = json!({"a":2,"b":3,"normal_on_a_ns":[2,0,0],"tangent_on_a_ns":[0,0,-1]});
        let row = json!({"pair_reactions_ns":[[1,3,2],[2,3,2]],"pair_impulses":[left,right]});
        assert_eq!(parcel_impulse(&row).unwrap(), DVec3::Z * 2.0);
        for (first, second) in [(2, 1), (1, 1)] {
            let mut altered = row.clone();
            altered["pair_impulses"][0]["a"] = json!(first);
            altered["pair_impulses"][1]["a"] = json!(second);
            assert!(parcel_impulse(&altered).is_err());
        }
        let mut normals = row.clone();
        normals["pair_impulses"][0]["normal_on_a_ns"] = json!([2, 0, 0]);
        normals["pair_impulses"][1]["normal_on_a_ns"] = json!([-2, 0, 0]);
        assert!(parcel_impulse(&normals).is_err());
        let mut scalars = row;
        scalars["pair_reactions_ns"][0][2] = json!(1);
        scalars["pair_reactions_ns"][1][2] = json!(3);
        assert!(parcel_impulse(&scalars).is_err());
    }
}
