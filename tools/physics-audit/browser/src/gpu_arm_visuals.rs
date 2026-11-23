// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare captured two-link visuals to device-observed joint states and endpoints.
use super::{Result, close, same, vector};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

type Point = [f64; 3];
type Fixture = (Point, [f64; 2]);

fn arm(visuals: &BTreeMap<&str, &Value>, sample: &Value, id: u64) -> Result<Fixture> {
    let visual = |part: &str| {
        visuals
            .get(format!("default::arm_{id}::{part}").as_str())
            .copied()
            .ok_or_else(|| format!("missing arm {id} {part} visual"))
    };
    let base = vector(&sample["base"])?;
    let elbow = vector(&sample["elbow"])?;
    let tip = vector(&sample["tip"])?;
    if !same(&visual("base")?["translation"], base)? || !same(&visual("tool")?["translation"], tip)?
    {
        return Err("base or tip differs from the observed arm".into());
    }
    let [a, b] = vector::<2>(&sample["angles"][0])?;
    let [c, d] = vector::<2>(&sample["angles"][1])?;
    if !close(a * a + b * b, 1.) || !close(c * c + d * d, 1.) {
        return Err("device joint orientation differs from unit length".into());
    }
    let links = [
        ("upper", base, elbow, [a, 0., b]),
        ("forearm", elbow, tip, [a * c - b * d, 0., b * c + a * d]),
    ];
    let mut lengths = [0.; 2];
    for (i, (part, start, end, axis)) in links.into_iter().enumerate() {
        let v = visual(part)?;
        let [length, _, _] = vector::<3>(&v["shape"]["size"])?;
        let [x, y, z, w] = vector::<4>(&v["rotation"])?;
        if v["shape"]["kind"] != "box"
            || length <= 0.
            || !close(x, 0.)
            || !close(z, 0.)
            || !close(y * y + w * w, 1.)
        {
            return Err("expected a planar rigid link centered on its local X axis".into());
        }
        let rotated = [1. - 2. * y * y, 0., -2. * y * w];
        if !(0..3).all(|j| close(rotated[j], axis[j]) && close(start[j] + length * axis[j], end[j]))
        {
            return Err(
                format!("{part} endpoints disagree with the two measured joint angles").into(),
            );
        }
        let midpoint = std::array::from_fn::<_, 3, _>(|j| 0.5 * (start[j] + end[j]));
        if !same(&v["translation"], midpoint)? {
            return Err(format!("{part} is displaced from the link midpoint").into());
        }
        lengths[i] = length;
    }
    Ok((base, lengths))
}

pub(super) fn verify(frames: &[Value], samples: &[Value]) -> Result<()> {
    if frames.is_empty() || samples.len() != frames.len() {
        return Err("GPU arm observations and native replay require matching frame counts".into());
    }
    let mut fixtures = BTreeMap::new();
    for (frame, sample) in frames.iter().zip(samples) {
        let time = sample["time_ns"].as_u64().ok_or("missing GPU frame time")?;
        if frame["ns"].as_u64() != Some(time) {
            return Err("GPU arm and native replay clocks differ".into());
        }
        let mut visuals = BTreeMap::new();
        for v in frame["scene"]["visuals"]
            .as_array()
            .ok_or("missing native visuals")?
        {
            if visuals
                .insert(v["name"].as_str().ok_or("missing native name")?, v)
                .is_some()
            {
                return Err("duplicate native visual name".into());
            }
        }
        let mut ids = BTreeSet::new();
        for s in sample["arms"].as_array().ok_or("missing GPU arm samples")? {
            let id = s["lane_id"].as_u64().ok_or("missing GPU lane id")?;
            if !ids.insert(id) {
                return Err("duplicate GPU arm lane".into());
            }
            let fixture =
                arm(&visuals, s, id).map_err(|e| format!("arm {id} at {time} ns: {e}"))?;
            if fixtures
                .insert(id, fixture)
                .is_some_and(|prev| prev != fixture)
            {
                return Err("arm base or link length changed during playback".into());
            }
        }
        if ids.len() != 9 || !ids.iter().copied().eq(0..9) {
            return Err("expected the nine distinct GPU arm lanes".into());
        }
    }
    Ok(())
}
#[cfg(test)]
#[path = "gpu_arm_visuals_tests.rs"]
mod tests;
