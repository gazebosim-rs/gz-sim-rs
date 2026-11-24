// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recreate complete native link states from 25 offsets to measure the kinematic proxy error.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn load(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn vec3(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(v.clone())?))
}
fn quat(v: &Value) -> Result<DQuat> {
    Ok(DQuat::from_array(serde_json::from_value(v.clone())?))
}

fn angle(a: DQuat, b: DQuat) -> f64 {
    let (a, b) = (a.normalize(), b.normalize());
    let (minus, plus) = ((a - b).length(), (a + b).length());
    4. * minus.min(plus).atan2(minus.max(plus))
}
fn main() -> Result<()> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 4 {
        return Err(
            "cut_pose_calibration SUPPORT_REACQUISITION PRIMARY_SURVEY NEW_SCRATCH OUTPUT_JSON"
                .into(),
        );
    }
    let bin = Path::new(&args[0]);
    let archive = Path::new(&args[1]);
    let scratch = Path::new(&args[2]);
    fs::create_dir(scratch)?;
    std::os::unix::fs::symlink(archive.join("cases").canonicalize()?, scratch.join("cases"))?;
    let mut chosen = BTreeMap::new();
    for id in 0..1250 {
        chosen
            .entry(165 + 5 * ((id * 613 % 1250) / 50))
            .or_insert(id);
    }
    let mut rows = Vec::new();
    let mut max = [0_f64; 4];
    for (offset, id) in chosen {
        let source = load(&archive.join("cases").join(format!("case-{id:04}.json")))?;
        for trial in source["result"]["trials"].as_array().ok_or("trials")? {
            let stroke = trial["stroke"].as_u64().ok_or("stroke")?;
            let status = Command::new(bin)
                .arg("--survey-replay")
                .arg(id.to_string())
                .arg("--survey-stroke")
                .arg(stroke.to_string())
                .arg(scratch)
                .output()?;
            if !status.status.success() {
                return Err(format!(
                    "case {id}/{stroke}: {}",
                    String::from_utf8_lossy(&status.stderr)
                )
                .into());
            }
            let trace = load(&scratch.join("traces/adaptive.json"))?;
            let s = &trace["frames"][0];
            let cut = &trial["cut"];
            if s["tick"] != cut["tick"]
                || trace["geometry"]["foot_half_extents"] != json!([0.18, 0.10, 0.06])
                || trace["geometry"]["pad_half_extents"] != json!([0.33, 0.18, 0.06])
            {
                return Err("wrong donor clock or fixture geometry".into());
            }
            let f = &s["foot"];
            let u = &s["upper"];
            let pad = &s["pad"];
            let p = vec3(&cut["com_m"])?;
            let v = vec3(&cut["com_velocity_m_s"])?;
            let w = vec3(&cut["foot_angular_velocity_rad_s"])?;
            let q = quat(&cut["foot_pose_xyzw"])?;
            let native_foot = vec3(&f["position"])?;
            let native_upper = vec3(&u["position"])?;
            let native_vfoot = vec3(&f["velocity"])?;
            let native_vupper = vec3(&u["velocity"])?;
            let pad_position = vec3(&pad["position"])?;
            let pad_velocity = vec3(&pad["velocity"])?;
            if p.distance((native_foot + native_upper) * 0.5) > 1e-12
                || v.distance((native_vfoot + native_vupper) * 0.5) > 1e-12
                || (pad_position.x - offset as f64 / 1000.).abs() > 1e-12
                || pad_position.y.abs() > 1e-12
                || angle(DQuat::IDENTITY, quat(&pad["orientation"])?) > 1e-12
                || pad_velocity.truncate().length() > 1e-12
                || vec3(&pad["angular_velocity"])?.length() > 1e-12
            {
                return Err("stored COM or pad does not reproduce native donor".into());
            }
            let lever = q * DVec3::new(0., 0., 0.07);
            let error = [
                (p - lever)
                    .distance(native_foot)
                    .max((p + lever).distance(native_upper)),
                (v - w.cross(lever))
                    .distance(native_vfoot)
                    .max((v + w.cross(lever)).distance(native_vupper)),
                w.distance(vec3(&u["angular_velocity"])?),
                angle(q, quat(&u["orientation"])?),
            ];
            for (x, e) in max.iter_mut().zip(error) {
                *x = x.max(e);
            }
            rows.push(json!({"case":id,"offset_mm":offset,"stroke":stroke,"tick":s["tick"],"native_donor":s,"max_error":error}));
        }
    }
    fs::write(
        &args[3],
        serde_json::to_vec_pretty(
            &json!({"format":"locked-sole-pose-calibration-v1","offsets":25,"cuts":rows.len(),
        "selection":"lowest factorial case index at each support offset, all available strokes; no outcome-based selection",
        "error_axes":["link center m","link velocity m/s","upper-to-foot angular velocity rad/s","upper-to-foot orientation rad"],"maximum":max,"donors":rows}),
        )?,
    )?;
    fs::remove_dir_all(scratch)?;
    Ok(())
}
