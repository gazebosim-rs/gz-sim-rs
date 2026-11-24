// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read the source point loads independently of the archive serializer.
use super::{Result, same, vector};
use serde_json::Value;

pub(super) fn verify(scene: &Value, trace: &Value, index: &Value, tick: u64) -> Result<()> {
    let steps = trace["frames"]
        .as_array()
        .ok_or("missing measured frames")?;
    let row = steps
        .binary_search_by_key(&tick, |s| s["tick"].as_u64().unwrap_or(u64::MAX))
        .map_err(|_| "no native sample for clock")?;
    let step = &steps[row];
    let visuals = scene["visuals"].as_array().ok_or("scene visuals missing")?;
    if visuals.len() != 28 {
        return Err(format!("unexpected visual count {} at {tick}", visuals.len()).into());
    }
    let v = |key: &str| index[key].as_f64().ok_or("missing scale");
    let z = v("sole_z")?;
    let scale = v("full_corner_ns")?;
    let [half_x, half_y, _] = vector::<3>(&trace["geometry"]["foot_half_extents"])?;
    for (lane, shift) in [
        ("switched", -v("spacing")? / 2.),
        ("reference", v("spacing")? / 2.),
    ] {
        let find = |name: &str| {
            visuals
                .iter()
                .find(|v| v["name"] == format!("default::{lane}_{name}::visual"))
                .ok_or("missing expected visual")
        };
        for name in ["foot", "upper", "pad"] {
            let r = &step[lane][name];
            let [x, y, h] = vector(&r["position"])?;
            let [qx, qy, qz, qw] = vector(&r["orientation"])?;
            let c = std::f64::consts::FRAC_1_SQRT_2;
            let b = find(name)?;
            if !same(&b["translation"], [-y, shift + x, h])?
                || !same(
                    &b["rotation"],
                    [c * (qx - qy), c * (qy + qx), c * (qz + qw), c * (qw - qz)],
                )?
                || b["shape"]["kind"] != "box"
                || !same(
                    &b["shape"]["size"],
                    vector::<3>(&trace["geometry"][format!("{name}_half_extents")])?
                        .map(|h| 2. * h),
                )?
            {
                return Err(
                    format!("source body pose/shape mismatch {lane}/{name} at {tick}").into(),
                );
            }
        }
        let mut loads = [0.; 4];
        for p in step[lane]["points"]
            .as_array()
            .ok_or("point list missing")?
        {
            let [x, y, _] = vector::<3>(&p["foot_local_m"])?;
            let n = 2 * usize::from(x >= 0.) + usize::from(y >= 0.);
            loads[n] += p["normal_impulse_ns"]
                .as_f64()
                .ok_or("point load missing")?;
        }
        for (n, f) in loads.into_iter().enumerate() {
            let x = shift
                + if n >= 2 {
                    half_x * 0.57
                } else {
                    -half_x * 0.57
                };
            let h = z + if n % 2 == 1 {
                half_y * 0.55
            } else {
                -half_y * 0.55
            };
            let o = find(&format!("corner_{n}"))?;
            if !same(
                &o["translation"],
                [0.026, x + (f / scale - 0.5) * (0.84 * half_x), h],
            )? || o["shape"]["kind"] != "sphere"
                || !super::close(
                    o["shape"]["radius"].as_f64().ok_or("radius missing")?,
                    0.010,
                )
            {
                return Err(format!("corner {n} load mismatch for {lane} at {tick}").into());
            }
        }
        let x = if lane == "switched" && step["prescribed"] == true {
            -half_x
        } else {
            half_x
        };
        if !same(
            &find("tier")?["translation"],
            [0.011, shift + x, z - 1.62 * half_y],
        )? {
            return Err(format!("tier mismatch for {lane} at {tick}").into());
        }
    }
    Ok(())
}
