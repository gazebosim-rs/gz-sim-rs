// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read both native files and independently bin the loads ending in each displayed interval.
use super::{Result, same, vector};
use serde_json::Value;
use std::{fs, path::Path};

pub(super) fn input(source: &Path, index: &Value) -> Result<Value> {
    let summary = index["activity_source"]
        .as_str()
        .map(|file| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(source.join(file))?)?) })
        .transpose()?;
    index["lanes"]
        .as_array()
        .ok_or("lane descriptions")?
        .iter()
        .map(|lane| {
            let raw = fs::read(source.join(lane["file"].as_str().ok_or("trace path")?))?;
            let mut trace: Value = serde_json::from_slice(&raw)?;
            if let Some(summary) = &summary {
                let row = summary["runs"]
                    .as_array()
                    .ok_or("missing controls")?
                    .iter()
                    .find(|r| {
                        r["cycle"] == 3
                            && r["bottom_dwell_shift_us"] == 0
                            && r["history"] == "reference"
                            && r["policy"] == lane["policy"]
                    })
                    .ok_or("missing selected policy")?;
                trace["selected_ticks"] = row["selected_ticks"].clone();
            }
            Ok(trace)
        })
        .collect::<Result<Vec<_>>>()
        .map(Value::Array)
}
pub(super) fn verify(scene: &Value, raw: &Value, index: &Value, tick: u64) -> Result<()> {
    let clocks = index["ticks"].as_array().ok_or("viewer grid")?;
    let pos = clocks
        .binary_search_by_key(&tick, |n| n.as_u64().unwrap_or(u64::MAX))
        .map_err(|_| "unexpected viewer clock")?;
    let precise = index["origin_ns"].is_u64();
    let cut = index[if precise { "origin_ns" } else { "origin_us" }]
        .as_u64()
        .ok_or("cut time")?;
    if precise
        && (index["dt_ns"] != 1
            || index["kind"] != "native-comparison-v1"
            || index["origin_us"] != cut / 1000)
    {
        return Err("invalid sub-microsecond source clock".into());
    }
    let clock = |s: &Value| {
        let us = s["time_us"].as_u64()?;
        if precise {
            s["time_ns"].as_u64().or_else(|| us.checked_mul(1000))
        } else {
            Some(us)
        }
    };
    let last = if pos == 0 {
        cut
    } else {
        cut + clocks[pos - 1].as_u64().ok_or("bad clock")?
    };
    let time = cut + tick;
    let v = |field: &str| index[field].as_f64().ok_or("missing gauge scale");
    let z = v("sole_z")?;
    let visuals = scene["visuals"].as_array().ok_or("no physical scene")?;
    let energy = index["kind"] == "contact-energy-v1";
    let activity = index["activity_source"].is_string();
    let rebound = index["kind"] == "rebound-gate-v1";
    let budget = index["kind"] == "rebound-budget-v1";
    let flight = index["kind"] == "rebound-flight-v1";
    let weak = index["kind"] == "weak-recontact-v1";
    let outer = index["kind"] == "outer-divergence-v1";
    let native = index["kind"] == "native-comparison-v1";
    let has_reference_gauges = weak || outer || native;
    let pair_only = native && index["pair_totals_only"] == true;
    let single = native && index["single_body"] == true;
    let has_opponent = native && index["has_opponent"] == true;
    let face = index["load_face"].as_str();
    let traces = raw.as_array().ok_or("missing source trajectories")?;
    let count = traces.len();
    if !(2..=3).contains(&count) {
        return Err("expected two or three native trajectories".into());
    }
    let visual_count = if pair_only {
        (11 - usize::from(single)) * count
    } else if has_reference_gauges {
        (20 - usize::from(single)) * count
    } else if rebound || flight {
        38
    } else if activity {
        40
    } else if energy || budget {
        36
    } else {
        28
    };
    let samples = traces.iter().filter_map(|t| {
        let frames = t["frames"].as_array()?;
        let at = frames.binary_search_by_key(&time, |r| clock(r).unwrap_or(u64::MAX)).ok()?;
        frames.get(at)
    });
    let impact_count: usize = samples.clone().filter_map(|r| r["interval_impacts"].as_array()).map(Vec::len).sum();
    let interval_count: usize = samples.clone().filter_map(|r| r["contact_intervals"].as_array()).map(Vec::len).sum();
    let matched_count = samples.filter(|r| r["matched_step"].is_object()).count();
    if visuals.len()
        != visual_count + 2 * impact_count + 4 * matched_count + interval_count
            + (usize::from(has_opponent)
                + 8 * usize::from(face.is_some())
                + 4 * usize::from(index["support_impulses"] == true))
                * count
    {
        return Err("unexpected visual count".into());
    }
    for (k, trace) in traces.iter().enumerate() {
        let geom = &trace["geometry"];
        let [hx, hy, _] = vector::<3>(&geom["foot_half_extents"])?;
        let frames = trace["frames"].as_array().ok_or("missing native frames")?;
        let at = frames
            .binary_search_by_key(&time, |r| clock(r).unwrap_or(u64::MAX))
            .map_err(|_| "unmeasured body pose")?;
        let shift = (k as f64 - (count - 1) as f64 * 0.5) * v("spacing")?;
        let label = index["lanes"][k]["label"].as_str().ok_or("missing lane")?;
        let find = |name: &str| {
            visuals
                .iter()
                .find(|b| b["name"] == format!("default::{label}_{name}::visual"))
                .ok_or("missing visual")
        };
        if let Some(m) = frames[at]["matched_step"].as_object() {
            let wall = m["wall_p95_ns"].as_u64().ok_or("missing p95 step time")?;
            let error = m["impulse_error_ns"].as_f64().ok_or("missing measured impulse error")?;
            for (label,frac,h) in [
                ("mechanics_impulse_error".to_owned(),error/v("max_mechanics_impulse_error_ns")?,2.),
                (format!("step_p95_{wall}ns"),wall as f64/v("max_step_wall_p95_ns")?,2.5),
            ] {
                if !same(&find(&label)?["translation"],[0.023,shift+(2.*frac-1.)*hx,z-h*hy])? {
                    return Err(format!("wrong measured step gauge {label}").into());
                }
            }
        }
        if let Some(parts) = frames[at]["contact_intervals"].as_array() {
            let total = (time - last) * if precise { 1 } else { 1000 };
            let mut previous = 0;
            for p in parts {
                let end = p["end_ns"].as_u64().ok_or("missing native boundary")?;
                let duration = p["duration_ns"].as_u64().ok_or("missing native duration")?;
                if end <= previous || end - previous != duration {return Err("noncontiguous solver intervals".into());}
                let fraction = duration as f64 / total as f64;
                let mid = shift + hx * ((end + previous) as f64 / total as f64 - 1.);
                let (kind,color) = if p["material"] == true {("proved_contact",[0.05,0.82,0.68,1.])} else {("native_contact",[0.65,0.69,0.74,1.])};
                let bar = find(&format!("{kind}_to_{end}ns"))?;
                if !same(&bar["translation"],[0.023,mid,z-2.*hy])?
                    || !same(&bar["shape"]["size"],[0.012,1.92*hx*fraction,0.016])? || !same(&bar["color"],color)? {
                    return Err("wrong solver interval bar".into());
                }
                previous = end;
            }
            if previous != total {return Err("solver intervals did not reach the public frame".into());}
        }
        if let Some(hits) = frames[at]["interval_impacts"].as_array() {
            for (h, hit) in hits.iter().enumerate() {
                let elapsed = hit["elapsed_ns"].as_u64().ok_or("missing interval impact clock")?;
                let to_ns = if precise { 1 } else { 1000 };
                if at == 0 || elapsed > (time - last) * to_ns { return Err("misdated interval impact".into()); }
                let [x, y, z] = vector::<3>(&hit["point_m"])?;
                let [jx, jy, jz] = vector::<3>(&hit["impulse_ns"])?;
                let label = format!("impact_{h}_{elapsed}ns");
                let point = find(&label)?;
                let arrow = find(&format!("{label}_impulse"))?;
                let position = [-y, x + shift, z];
                let scale = v("impulse_display_m_per_ns")?;
                let delta = [-jy * scale, jx * scale, jz * scale];
                let length = delta.into_iter().map(|a| a*a).sum::<f64>().sqrt();
                let [qx,qy,qz,qw] = vector::<4>(&arrow["rotation"])?;
                let axis = [2.*(qx*qz+qw*qy), 2.*(qy*qz-qw*qx), 1.-2.*(qx*qx+qy*qy)];
                if point["shape"]["kind"] != "sphere"
                    || !same(&point["translation"], position)?
                    || point["shape"]["radius"].as_f64() != Some(0.012)
                    || arrow["shape"]["kind"] != "box"
                    || !same(&arrow["shape"]["size"], [0.008,0.008,length])?
                    || !same::<3>(&arrow["translation"], std::array::from_fn(|k| position[k]+0.5*delta[k]))?
                    || axis.iter().zip(delta).any(|(a,d)| (a*length-d).abs() > 1e-12)
                {
                    return Err(format!("wrong interval impact in {label}").into());
                }
            }
        }
        let roles: &[&str] = if single {
            &["foot", "pad"]
        } else {
            &["foot", "upper", "pad"]
        };
        if single && (!frames[at]["upper"].is_null() || !geom["upper_half_extents"].is_null()) {
            return Err("single-body recording contains an upper link".into());
        }
        if has_opponent == geom["opponent_half_extents"].is_null()
            || has_opponent == frames[at]["opponent"].is_null()
        {
            return Err("archive disagrees with measured opponent presence".into());
        }
        for name in roles
            .iter()
            .copied()
            .chain(has_opponent.then_some("opponent"))
        {
            let r = &frames[at][name];
            let [x, y, h] = vector::<3>(&r["position"])?;
            let [qx, qy, qz, qw] = vector::<4>(&r["orientation"])?;
            let d = std::f64::consts::FRAC_1_SQRT_2;
            let b = find(name)?;
            if !same(&b["translation"], [-y, x + shift, h])?
                || !same(
                    &b["rotation"],
                    [d * (qx - qy), d * (qx + qy), d * (qz + qw), d * (qw - qz)],
                )?
                || b["shape"]["kind"] != "box"
                || !same(
                    &b["shape"]["size"],
                    vector::<3>(&geom[format!("{name}_half_extents")])?.map(|v| 2. * v),
                )?
            {
                return Err(format!("wrong {label} body at source time {time}").into());
            }
        }
        let mut corners = [0.; 4];
        let mut sum = 0.;
        if pair_only {
            if frames[at]["corner_ns"].is_array() {
                return Err("unexpected quadrant receipts in pair-only replay".into());
            }
        } else if rebound || budget || flight || has_reference_gauges {
            corners = vector::<4>(&frames[at]["corner_ns"])?;
        } else {
            for native in &frames[1..at + 1] {
                let t = native["time_us"].as_u64().ok_or("missing native clock")?;
                if t > last {
                    for (dest, value) in corners.iter_mut().zip(vector::<4>(&native["corner_ns"])?)
                    {
                        *dest += value;
                    }
                }
                if t <= cut + 5000 {
                    sum += native["normal_impulse_ns"]
                        .as_f64()
                        .ok_or("missing impulse")?;
                }
            }
        }
        for (i, n) in corners
            .into_iter()
            .enumerate()
            .take(if pair_only { 0 } else { 4 })
        {
            let x = shift + (if i >= 2 { 0.57 } else { -0.57 }) * hx;
            let h = z + (if i % 2 == 1 { 0.55 } else { -0.55 }) * hy;
            if let Some(face) = face {
                let [_, _, hz] = vector::<3>(&geom["foot_half_extents"])?;
                let r = &frames[at]["foot"];
                let [qx, qy, qz, w] = vector::<4>(&r["orientation"])?;
                let local = [
                    if i >= 2 { hx } else { -hx },
                    if i % 2 == 1 { hy } else { -hy },
                    if face == "top" { hz } else { -hz },
                ];
                let cross = |v: [f64; 3]| {
                    [
                        qy * v[2] - qz * v[1],
                        qz * v[0] - qx * v[2],
                        qx * v[1] - qy * v[0],
                    ]
                };
                let origin = vector::<3>(&r["position"])?;
                let distance = 0.6 * hx.min(hy) * n / v("full_corner_ns")?;
                for (name, dz, radius) in [
                    ("face_corner", 0.0, 0.04 * hx.min(hy)),
                    ("face_load", distance, 0.06 * hx.min(hy)),
                ] {
                    let offset = [
                        local[0],
                        local[1],
                        local[2] + if face == "top" { dz } else { -dz },
                    ];
                    let t = cross(offset);
                    let tt = cross(t);
                    let [ax, ay, az]: [f64; 3] =
                        std::array::from_fn(|j| origin[j] + offset[j] + 2. * (w * t[j] + tt[j]));
                    let marker = find(&format!("{name}_{i}"))?;
                    if !same(&marker["translation"], [-ay, ax + shift, az])?
                        || marker["shape"]["kind"] != "sphere"
                        || !marker["shape"]["radius"]
                            .as_f64()
                            .is_some_and(|r| super::close(r, radius))
                    {
                        return Err(format!(
                            "incorrect {face} material corner or load at source time {time}"
                        )
                        .into());
                    }
                }
            }
            let dot = find(&format!("corner_{i}"))?;
            if dot["shape"]["kind"] != "sphere"
                || !same(
                    &dot["translation"],
                    [0.026, x + (n / v("full_corner_ns")? - 0.5) * 0.84 * hx, h],
                )?
            {
                return Err(format!("wrong corner {i} at source time {time}").into());
            }
        }
        if activity {
            let ticks = trace["selected_ticks"]
                .as_array()
                .ok_or("missing selected ticks")?;
            let selected = ticks
                .iter()
                .any(|tick| tick.as_u64() == Some(time.div_ceil(500)));
            let x = shift + if selected { hx } else { -hx };
            if !same(
                &find("solver_activity")?["translation"],
                [0.023, x, z - 2.65 * hy],
            )? {
                return Err(format!("wrong native solver activity at source time {time}").into());
            }
        }
        if energy {
            let amount = |frame: &Value, field: &str| -> Result<f64> {
                Ok(frame[field]
                    .as_f64()
                    .ok_or_else(|| format!("missing {field}"))?)
            };
            let read_energy = |r: &Value| -> Result<[f64; 2]> {
                if r.get("lift_work_j").is_some() {
                    Ok([amount(r, "lift_work_j")?, amount(r, "mechanical_change_j")?])
                } else {
                    Ok([
                        amount(r, "support_work_j")?,
                        amount(r, "potential_j")?
                            + amount(r, "kinetic_translation_j")?
                            + amount(r, "kinetic_rotation_j")?,
                    ])
                }
            };
            let initial = read_energy(&frames[0])?;
            let current = read_energy(&frames[at])?;
            let lo = v("energy_min_j")?;
            let hi = v("energy_max_j")?;
            for (i, name) in ["lift_work", "mechanical_change"].iter().enumerate() {
                let h = z - (1.61 + 0.52 * i as f64) * hy;
                let value = current[i] - initial[i];
                let x = |j: f64| shift + hx * (2. * (j - lo) / (hi - lo) - 1.);
                if !same(&find(name)?["translation"], [0.023, x(value), h])?
                    || !same(
                        &find(&format!("{name}_zero"))?["translation"],
                        [0.012, x(0.), h],
                    )?
                {
                    return Err(format!("wrong {label} {name} at source time {time}").into());
                }
            }
        } else if has_reference_gauges {
            let s = &frames[at];
            let fine = &traces[count - 1]["frames"][pos];
            if clock(fine) != Some(time) {
                return Err("unmatched fine reference time".into());
            }
            let assembly_com = |frame: &Value| -> Result<[f64; 3]> {
                let a = vector::<3>(&frame["foot"]["position"])?;
                if single {
                    let p = if geom["foot_com_local"].is_null() {
                        [0.; 3]
                    } else {
                        vector::<3>(&geom["foot_com_local"])?
                    };
                    let [qx, qy, qz, w] = vector::<4>(&frame["foot"]["orientation"])?;
                    let cross = |b: [f64; 3]| {
                        [
                            qy * b[2] - qz * b[1],
                            qz * b[0] - qx * b[2],
                            qx * b[1] - qy * b[0],
                        ]
                    };
                    let t = cross(p);
                    let qt = cross(t);
                    return Ok(std::array::from_fn(|i| {
                        a[i] + p[i] + 2. * (w * t[i] + qt[i])
                    }));
                }
                let b = vector::<3>(&frame["upper"]["position"])?;
                Ok(std::array::from_fn(|i| (a[i] + b[i]) * 0.5))
            };
            let center = assembly_com(s)?;
            let reference = assembly_com(fine)?;
            let error_um = center
                .into_iter()
                .zip(reference)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt()
                * 1e6;
            let impulse = s["normal_impulse_ns"]
                .as_f64()
                .ok_or("missing ground reaction")?;
            let steps = s["native_steps"].as_u64().ok_or("missing native solves")?;
            if let Some(mechanical) = index["lanes"][k]["analytical_ticks"].as_array() {
                let tick = s["tick"].as_u64().ok_or("missing source tick")?;
                if pos > 0 && (steps == 0) != mechanical.iter().any(|v| v.as_u64() == Some(tick)) {
                    return Err("native public budget disagrees with per-tick mechanics".into());
                }
            }
            let fourth = if native {
                let spin = vector::<3>(&s["foot"]["angular_velocity"])?;
                let reference = vector::<3>(&fine["foot"]["angular_velocity"])?;
                let distance = spin
                    .into_iter()
                    .zip(reference)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt();
                (
                    "spin_difference",
                    distance / v("max_spin_difference_rad_s")?,
                )
            } else if weak {
                (
                    "weak_trigger",
                    f64::from(s["weak_start"].as_bool().ok_or("missing weak admission")?),
                )
            } else {
                let matched = assembly_com(&raw[1]["frames"][pos])?;
                let distance = center
                    .into_iter()
                    .zip(matched)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt();
                (
                    "distance_from_matched",
                    distance * 1e6 / v("max_matched_error_um")?,
                )
            };
            for (j, (name, fraction)) in [
                ("pair_impulse", impulse / v("peak_pair_ns")?),
                ("com_error", error_um / v("max_com_error_um")?),
                (
                    "native_steps",
                    steps as f64 / if native { v("max_native_steps")? } else { 4. },
                ),
                fourth,
            ]
            .into_iter()
            .enumerate()
            {
                let y = shift + (2. * fraction - 1.) * hx;
                let h = z - (if pair_only { 0. } else { 1.62 } + 0.5 * j as f64) * hy;
                let actual = vector::<3>(&find(name)?["translation"])?;
                // A nonzero local COM makes two independent quaternion rotations enter
                // a subtraction. Amplify their geometric roundoff by the gauge scale.
                let mut y_tolerance = 1e-12;
                if single && name == "com_error" && !geom["foot_com_local"].is_null() {
                    let offset = vector::<3>(&geom["foot_com_local"])?;
                    let local_length = offset.iter().map(|v| v.abs()).sum::<f64>();
                    if local_length > 0. {
                        let magnitude = center
                            .iter()
                            .chain(&reference)
                            .map(|v| v.abs())
                            .fold(0., f64::max)
                            + local_length;
                        let scale = 2. * hx * 1e6 / v("max_com_error_um")?;
                        y_tolerance += 32. * f64::EPSILON * magnitude * scale;
                    }
                }
                if (actual[0] - 0.023).abs() >= 1e-12
                    || (actual[1] - y).abs() >= y_tolerance
                    || (actual[2] - h).abs() >= 1e-12
                {
                    return Err(format!("wrong {label} {name} at source time {time}: expected [0.023, {y}, {h}], got {actual:?} (gauge axis tolerance {y_tolerance})").into());
                }
            }
            if index["support_impulses"] == true {
                let loads = vector::<2>(&s["support_normal_ns"])?;
                if !pair_only
                    || !has_opponent
                    || loads.iter().any(|x| !x.is_finite() || *x < 0.)
                    || (loads[0] + loads[1] - impulse).abs() > 1e-9
                {
                    return Err("invalid two-support reaction source".into());
                }
                for (j, (name, load)) in ["first_support_impulse", "second_support_impulse"]
                    .into_iter()
                    .zip(loads)
                    .enumerate()
                {
                    let expected = [
                        0.023,
                        shift + hx * (2. * load / v("peak_pair_ns")? - 1.),
                        z - (2.0 + 0.5 * j as f64) * hy,
                    ];
                    if !same(&find(name)?["translation"], expected)? {
                        return Err(format!("wrong {label} {name} at {time}").into());
                    }
                }
            } else if !s["support_normal_ns"].is_null() {
                return Err("archive omitted measured per-support loads".into());
            }
        } else if flight {
            let r = &frames[at];
            let mean_z = |sample: &Value, field: &str| -> Result<f64> {
                let foot = vector::<3>(&sample["foot"][field])?;
                let upper = vector::<3>(&sample["upper"][field])?;
                Ok((foot[2] + upper[2]) * 0.5)
            };
            let resumed = frames[1..=at]
                .iter()
                .any(|r| r["normal_impulse_ns"].as_f64().is_some_and(|j| j > 0.));
            let dt = (time - cut) as f64 * 1e-6;
            let expected = mean_z(&frames[0], "position")? + mean_z(&frames[0], "velocity")? * dt
                - 0.5 * 9.81 * dt * dt;
            let delta = if resumed {
                -0.12
            } else {
                0.5 + (mean_z(r, "position")? - expected) * 1e6 / (2. * v("error_scale_um")?)
            };
            let z0 = z - 1.62 * hy;
            for (j, (key, fraction)) in [
                ("signed_height_error", delta),
                (
                    "normal_reaction",
                    r["normal_impulse_ns"]
                        .as_f64()
                        .ok_or("missing native normal reaction")?
                        / v("peak_pair_ns")?,
                ),
                ("contact_resumed", f64::from(resumed)),
            ]
            .into_iter()
            .enumerate()
            {
                if !same(
                    &find(key)?["translation"],
                    [
                        0.023,
                        shift + hx * (2. * fraction - 1.),
                        z0 - 0.5 * j as f64 * hy,
                    ],
                )? {
                    return Err(format!("wrong {label} {key} at source time {time}").into());
                }
            }
            if !same(&find("error_zero")?["translation"], [0.01, shift, z0])? {
                return Err("incorrect ballistic zero mark".into());
            }
        } else if budget {
            let r = &frames[at];
            for (j, (indicator, key, scale)) in [
                ("pair_impulse", "normal_impulse_ns", "peak_pair_ns"),
                ("native_steps", "native_steps", "max_native_steps"),
                (
                    "solver_iterations",
                    "solver_iterations",
                    "max_solver_iterations",
                ),
            ]
            .into_iter()
            .enumerate()
            {
                let quantity = r[key].as_f64().ok_or("missing native budget observation")?;
                let dz = z - (1.62 + 0.5 * j as f64) * hy;
                if !same(
                    &find(indicator)?["translation"],
                    [0.023, shift + hx * (2. * quantity / v(scale)? - 1.), dz],
                )? {
                    return Err(format!("wrong {label} {indicator} at source time {time}").into());
                }
            }
        } else if rebound {
            let r = &frames[at];
            let lo = v("gap_min_m")?;
            let hi = v("gap_max_m")?;
            let clearance = match &r["gap_m"] {
                Value::Null => -0.12,
                n => (n.as_f64().ok_or("invalid native clearance")? - lo) / (hi - lo),
            };
            for (j, (name, fraction)) in [
                ("supported_clearance", clearance),
                (
                    "pair_impulse",
                    r["normal_impulse_ns"]
                        .as_f64()
                        .ok_or("missing native impulse")?
                        / v("peak_pair_ns")?,
                ),
                (
                    "solver_activity",
                    f64::from(r["selected"].as_bool().ok_or("missing selected flag")?),
                ),
            ]
            .into_iter()
            .enumerate()
            {
                let z = z - (1.62 + 0.5 * j as f64) * hy;
                if !same(
                    &find(name)?["translation"],
                    [0.023, shift + (2. * fraction - 1.) * hx, z],
                )? {
                    return Err(format!("wrong {label} {name} at source time {time}").into());
                }
            }
            if !same(
                &find("clearance_zero")?["translation"],
                [
                    0.012,
                    shift + hx * (-2. * lo / (hi - lo) - 1.),
                    z - 1.62 * hy,
                ],
            )? {
                return Err(format!("wrong {label} clearance zero at source time {time}").into());
            }
        } else if !same(
            &find("five_ms_impulse")?["translation"],
            [
                0.023,
                shift + hx * (2. * sum / v("window_full_ns")? - 1.),
                z - 1.65 * hy,
            ],
        )? {
            return Err(format!("wrong accumulated impulse at source time {time}").into());
        }
    }
    Ok(())
}
