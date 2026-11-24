// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare feedback with frozen torques under the same measured load, reference and time step.
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufRead, BufReader, Write},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
#[derive(Deserialize)]
struct Run {
    first_fall: Option<u64>,
    nonfoot_support_ticks: u64,
    max_torso_tilt_rad: f64,
    last: Last,
}
#[derive(Deserialize)]
struct Last {
    foot_centers_m: [[f64; 3]; 2],
    feet_force_n: [f64; 2],
}
fn read(p: &Path) -> Result<Run> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
pub fn write(root: &Path) -> Result<()> {
    let data = root.join("perturbations");
    let reference = BufReader::new(File::open(data.join("recorded.jsonl"))?).lines();
    let mut replay = BufReader::new(File::open(data.join("open-0.jsonl"))?).lines();
    let mut exact = 0;
    for original in reference {
        let mut original: serde_json::Value = serde_json::from_str(&original?)?;
        let mut copy: serde_json::Value =
            serde_json::from_str(&replay.next().ok_or("truncated open-loop recording")??)?;
        for item in [&mut original, &mut copy] {
            let values = item.as_object_mut().ok_or("expected a measured cut")?;
            values.remove("joint_error_max_rad");
            values.remove("joint_error_rms_rad");
            for joint in values
                .get_mut("motion_joints")
                .and_then(|v| v.as_array_mut())
                .ok_or("missing observed gait joints")?
            {
                joint
                    .as_object_mut()
                    .ok_or("invalid gait joint row")?
                    .remove("target_rad");
            }
        }
        if original != copy {
            return Err(format!("unforced torque recording diverged at {exact}").into());
        }
        exact += 1;
    }
    if replay.next().is_some() || exact != 1001 {
        return Err("expected 1001 paired unforced measurements".into());
    }
    let mut out = File::create(root.join("swing-perturbations.csv"))?;
    writeln!(
        out,
        "world_axis,chest_force_N,impulse_Ns,control,fall_ms,peak_torso_tilt_rad,nonfoot_support_ticks,settled_forward_foot_mm,terminal_left_N,terminal_right_N"
    )?;
    for axis in ["x", "y"] {
        for force in [-100, -60, -30, 30, 60, 100] {
            for mode in ["closed", "open"] {
                let record = read(&data.join(format!("{mode}-{axis}-{force}.json")))?;
                writeln!(
                    out,
                    "{axis},{force},{:.3},{mode},{},{:.5},{},{:.4},{:.3},{:.3}",
                    force as f64 * 0.08,
                    record.first_fall.map(|v| v.to_string()).unwrap_or_default(),
                    record.max_torso_tilt_rad,
                    record.nonfoot_support_ticks,
                    1000. * (record.last.foot_centers_m[1][0] - 0.025),
                    record.last.feet_force_n[0],
                    record.last.feet_force_n[1]
                )?;
            }
        }
    }
    println!(
        "Unperturbed frozen-torque recording matched all {exact} physical/torque measurement rows."
    );
    Ok(())
}
