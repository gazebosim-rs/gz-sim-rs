// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn percentile(v: &mut [u64], p: usize) -> u64 {
    v.sort_unstable();
    v[(p * v.len()).div_ceil(100) - 1]
}
fn main() -> Result<(), Box<dyn Error>> {
    let d = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let mut report = json!({});
    for version in ["baseline", "stages", "optimized"] {
        let mut runs = vec![];
        for k in 1..=3 {
            let samples: Value =
                serde_json::from_slice(&fs::read(d.join(format!("{version}{k}/cost.json")))?)?;
            assert_eq!(samples.as_array().unwrap().len(), 72);
            let mut stats = json!({});
            for lane in [
                "bevy_crossing_ns",
                "bevy_smooth_ns",
                "mechanics_crossing_ns",
                "mechanics_smooth_ns",
            ] {
                // Drop the first six samples for all variants and control the CPU.
                let mut v = samples
                    .as_array()
                    .unwrap()
                    .iter()
                    .skip(6)
                    .map(|row| row[lane].as_u64().unwrap())
                    .collect::<Vec<_>>();
                stats[lane] = json!({"p50":percentile(&mut v,50),"p95":percentile(&mut v,95)});
            }
            runs.push(stats);
        }
        report[version] = runs.into();
    }
    fs::write(
        d.join("checks/readback.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!("{report}");
    Ok(())
}
