// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Relate each rendered round tool and floor to the contact impulse committed during the tick.
use super::{Result, close, same, vector};
use serde_json::{Value, json};
use std::collections::BTreeMap;

fn number(row: &Value, key: &str) -> Result<f64> {
    row[key]
        .as_f64()
        .ok_or_else(|| format!("missing {key}").into())
}

struct Contact {
    normal: f64,
    tangent: f64,
    gap: f64,
    offset: f64,
    marker: [f64; 3],
}
fn contact(s: &Value, tool: &Value, ground: &Value) -> Result<Contact> {
    let base = vector::<3>(&s["base"])?;
    let tip = vector::<3>(&s["tip"])?;
    let radius = number(s, "tool_radius_m")?;
    let floor = base[2] + number(s, "ground_z_m")?;
    let gap = number(s, "surface_gap_m")?;
    let center = vector::<3>(&ground["translation"])?;
    if tool["shape"]["kind"] != "sphere"
        || radius <= 0.
        || !close(number(&tool["shape"], "radius")?, radius)
        || ground["shape"]["kind"] != "box"
        || !same(&ground["shape"]["size"], [1.5, 0.8, 0.04])?
        || !same(&ground["rotation"], [0., 0., 0., 1.])?
        || !close(center[2] + 0.02, floor)
    {
        return Err("visible tool or ground geometry differs from the contact model".into());
    }
    if !close(tip[2] - radius - floor, gap) {
        return Err("measured signed gap differs from the visible sphere bottom and plane".into());
    }
    let normal = number(s, "normal_impulse_ns")?;
    let tangent = number(s, "tangent_impulse_ns")?;
    let offset = number(s, "first_contact_offset_s")?;
    let mu = number(s, "friction_mu")?;
    if normal < 0.
        || mu < 0.
        || tangent.abs() > mu * normal + 1e-12
        || (normal == 0. && (tangent != 0. || offset != -1.))
        || (normal > 0. && !(0.0..=0.001).contains(&offset))
    {
        return Err("invalid finite-tick unilateral impulse, friction bound, or timing".into());
    }
    Ok(Contact {
        normal,
        tangent,
        gap,
        offset,
        marker: [
            tip[0],
            tip[1],
            if normal > 0. {
                tip[2] + radius + 0.013
            } else {
                floor - 0.02
            },
        ],
    })
}

/// Validate the normal indicator against device impulses and count checks by public tick.
pub(super) fn verify(frames: &[Value], samples: &[Value]) -> Result<Value> {
    if frames.len() != samples.len() || frames.len() < 2 {
        return Err("missing public contact ticks".into());
    }
    let mut stationary = BTreeMap::new();
    let mut first = None;
    let mut first_per_arm = BTreeMap::new();
    let mut counts = [0_usize; 2];
    let mut total_n = 0.;
    let mut total_t = 0.;
    let mut min_gap = f64::INFINITY;
    for (tick, (f, s)) in frames.iter().zip(samples).enumerate() {
        let ns = (tick as u64) * 1_000_000;
        if f["ns"] != ns || s["time_ns"] != ns {
            return Err("expected one measurement per public millisecond".into());
        }
        let visuals: BTreeMap<_, _> = f["scene"]["visuals"]
            .as_array()
            .ok_or("missing contact visuals")?
            .iter()
            .map(|v| (v["name"].as_str(), v))
            .collect();
        for a in s["arms"].as_array().ok_or("missing arm measurements")? {
            let id = a["lane_id"].as_u64().ok_or("missing contact lane")?;
            let part = |name: &str| {
                visuals
                    .get(&Some(format!("default::arm_{id}::{name}").as_str()))
                    .copied()
                    .ok_or_else(|| format!("missing {name} on lane {id}"))
            };
            let tool = part("tool")?;
            let ground = part("ground")?;
            let measurement =
                contact(a, tool, ground).map_err(|e| format!("arm {id} tick {tick}: {e}"))?;
            if stationary
                .insert(id, &ground["translation"])
                .is_some_and(|old| old != &ground["translation"])
            {
                return Err("contact plane moved during playback".into());
            }
            let loaded = usize::from(measurement.normal > 0.);
            let marker = part("contact_marker")?;
            if !same(&tool["color"], [0.94, 0.46, 0.10, 1.])?
                || marker["shape"]["kind"] != "sphere"
                || !close(number(&marker["shape"], "radius")?, 0.008)
                || !same(&marker["color"], [1., 0.16, 0.08, 1.])?
                || !same(&marker["translation"], measurement.marker)?
            {
                return Err(
                    "visible marker disagrees with the final impulse committed by the tick".into(),
                );
            }
            counts[loaded] += 1;
            min_gap = min_gap.min(measurement.gap);
            total_n += measurement.normal;
            total_t += measurement.tangent;
            if loaded == 1 {
                if tick == 0 {
                    return Err("initial state carries a step impulse".into());
                }
                first_per_arm
                    .entry(id)
                    .or_insert((tick as f64 - 1.) + measurement.offset * 1000.);
                first.get_or_insert(tick);
            }
        }
    }
    if counts.contains(&0) {
        return Err("expected distinct contact and free-flight indicators".into());
    }
    Ok(
        json!({"positive_normal_samples":counts[1],"zero_normal_samples":counts[0],"first_positive_tick":first,"first_contact_ms_by_lane":first_per_arm,"sum_normal_impulse_ns":total_n,"sum_signed_tangent_impulse_ns":total_t,"minimum_measured_gap_m":min_gap}),
    )
}
#[cfg(test)]
#[path = "gpu_tool_visuals_tests.rs"]
mod tests;
