// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare native encoder samples and the public local replay journal at the same physics instants.
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 3 {
        return Err("usage: position_compare NATIVE.csv RUST.jsonl COMPARISON.json".into());
    }
    let mut rust = BTreeMap::new();
    let mut input = BufReader::new(File::open(&args[1])?).lines();
    let profile: Value = serde_json::from_str(&input.next().ok_or("missing profile")??)?;
    let id = profile["observed_entities"]["default::joint_position_controller_test::j1"]
        .as_u64()
        .ok_or("missing joint identity")?;
    for line in input {
        let row: Value = serde_json::from_str(&line?)?;
        let step = row["step"].as_u64().ok_or("missing step")?;
        if row["time_ns"].as_u64() != Some(step * 1_000_000) || row["epoch"].as_u64() != Some(0) {
            return Err("unexpected candidate clock".into());
        }
        let joints = row["scalar_joints"]
            .as_array()
            .ok_or("missing joint samples")?;
        let joint = joints
            .iter()
            .find(|j| j["id"] == id)
            .ok_or("missing joint")?;
        let sample = [
            joint["position"].as_f64().ok_or("invalid position")?,
            joint["velocity"].as_f64().ok_or("invalid velocity")?,
        ];
        if rust
            .insert(step, sample)
            .is_some_and(|previous| previous != sample)
        {
            return Err("changing joint at a fixed time".into());
        }
    }
    if rust.len() != 1251 {
        return Err("incomplete candidate run".into());
    }
    let mut native = BufReader::new(File::open(&args[0])?).lines();
    if native.next().ok_or("empty native trace")?? != "tick,position_rad,velocity_rad_s" {
        return Err("unknown native schema".into());
    }
    let mut max_error = [(0., 0); 2];
    let mut endpoints = Vec::new();
    let mut count = 0;
    for row in native {
        let line = row?;
        let fields: Vec<_> = line.split(',').collect();
        if fields.len() != 3 {
            return Err("invalid native trace".into());
        }
        let tick = fields[0].parse::<u64>()?;
        count += 1;
        if tick != count {
            return Err("unexpected native clock".into());
        }
        let reference = [fields[1].parse::<f64>()?, fields[2].parse()?];
        let candidate = rust.get(&tick).ok_or("unmatched native sample")?;
        for i in 0..2 {
            let error = (candidate[i] - reference[i]).abs();
            if !error.is_finite() {
                return Err("nonfinite comparison".into());
            }
            if error > max_error[i].0 {
                max_error[i] = (error, tick);
            }
        }
        let target = match tick {
            450 => 1.,
            850 => -0.5,
            1250 => 0.25,
            _ => continue,
        };
        endpoints.push(json!({"tick":tick,"target_rad":target,"native_rad":reference[0],"rust_rad":candidate[0]}));
    }
    let passed = count == 1250
        && max_error[0].0 < 0.10
        && endpoints.len() == 3
        && endpoints.iter().all(|p| {
            let target = p["target_rad"].as_f64().unwrap();
            ["native_rad", "rust_rad"]
                .iter()
                .all(|k| (p[k].as_f64().unwrap() - target).abs() < 0.01)
        });
    let metrics = json!({"passed":passed,"samples":count,"max_angle_error_rad":max_error[0].0,
        "max_angle_error_tick":max_error[0].1, "max_speed_error_rad_s":max_error[1].0,"max_speed_error_tick":max_error[1].1,"endpoints":endpoints});
    std::fs::write(&args[2], serde_json::to_string_pretty(&metrics)?)?;
    if !passed {
        return Err("matched-sample or endpoint criterion failed".into());
    }
    Ok(())
}
