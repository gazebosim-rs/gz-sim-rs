// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct both endpoint solutions of each saved native set-valued receipt.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::new(f(&x[0]), f(&x[1]), f(&x[2]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(root.join(file))?)?)
    };
    let all = read("native.json")?;
    let rows = all.as_array().unwrap();
    let mut counts = [[0; 2]; 2];
    let mut mode_count = BTreeMap::new();
    let mut max = BTreeMap::<String, f64>::new();
    let mut peak = |key: &str, x: f64| {
        assert!(x.is_finite());
        let e = max.entry(key.to_string()).or_default();
        *e = e.max(x.abs());
    };
    for row in rows {
        let r = &row["result"];
        let yes = r["admitted"].as_bool().unwrap();
        let free = row["free_child"].as_bool().unwrap();
        counts[usize::from(free)][usize::from(yes)] += 1;
        let base = rows
            .iter()
            .find(|x| {
                x["frame"] == "base"
                    && ["mode", "mu", "roll", "yaw_deg", "free_child"]
                        .iter()
                        .all(|k| x[*k] == row[*k])
            })
            .unwrap();
        let rb = &base["result"];
        assert_eq!(rb["admitted"], yes);
        if !yes {
            continue;
        }
        let mu = f(&row["mu"]);
        let mode = row["mode"].as_str().unwrap();
        *mode_count.entry(mode).or_insert(0) += 1;
        assert_eq!(r["native_mode"], mode);
        let rot = DQuat::from_array(std::array::from_fn(|i| f(&row["q"][i])));
        let axis = v(&r["axis"]);
        let n = v(&r["pad_normal"]);
        let lo = f(&r["range_ns"][0]);
        let hi = f(&r["range_ns"][1]);
        assert!(r["uniqueness_bound"].as_f64().unwrap() < 1.);
        assert!(if mode == "both-stick" {
            hi > lo
        } else {
            hi == lo
        });
        peak("max_width_ns", hi - lo);
        for k in 0..2 {
            let mut loss = 0.;
            let mut longitudinal = 0.;
            for i in 0..2 {
                let j = v(&r["endpoints"][k]["native_ns"][i]);
                let ref_j = v(&r["endpoints"][k]["independent_ns"][i]);
                let jb = v(&rb["endpoints"][k]["native_ns"][i]);
                let before = v(&r["before_relative_m_s"][i]);
                let after = v(&r["after_relative_m_s"][i]);
                let normal = j.dot(n);
                let tangent = j - normal * n;
                assert!(normal > 0. && before.dot(n) < 0. && after.dot(n).abs() < 1e-12);
                assert!(
                    (normal - f(&r["normal_ns"][i])).abs() < 1e-12
                        && tangent.length() <= mu * normal + 1e-14
                );
                let slide =
                    (mode == "first-sticks" && i == 1) || (mode == "second-sticks" && i == 0);
                if slide {
                    assert!(after.length() > 0.1);
                    peak(
                        "max_sliding_direction_residual_ns",
                        (tangent + mu * normal * after.normalize()).length(),
                    );
                } else {
                    assert!(after.length() < 1e-12);
                }
                longitudinal += j.dot(axis);
                if i == 0 {
                    assert!((j.dot(axis) - f(&r["range_ns"][k])).abs() < 1e-12);
                }
                loss -= 0.5 * j.dot(before + after);
                peak("max_frame_covariance_ns", j.distance(rot * jb));
                peak("max_endpoint_error_ns", j.distance(ref_j));
            }
            assert!((longitudinal - f(&r["longitudinal_total_ns"])).abs() < 1e-12);
            peak("max_reconstructed_loss_residual_j", loss - f(&r["loss_j"]));
        }
        for k in [
            "momentum_residual_ns",
            "angular_residual_nms",
            "energy_residual_j",
            "relative_loss_residual_j",
            "world_work_residual_j",
            "point_speed_residual_m_s",
            "site_position_residual_m",
        ] {
            peak(k, f(&r[k]));
        }
    }
    let a = read("traces/native.json")?;
    let b = read("traces/physical.json")?;
    for (x, y) in a["frames"]
        .as_array()
        .unwrap()
        .iter()
        .zip(b["frames"].as_array().unwrap())
    {
        let c = |z: &Value| 0.5 * (v(&z["foot"]["position"]) + v(&z["upper"]["position"]));
        peak("max_replay_com_m", c(x).distance(c(y)));
        for i in 0..4 {
            peak(
                "max_replay_normal_ns",
                f(&x["corner_ns"][i]) - f(&y["corner_ns"][i]),
            );
        }
    }
    assert_eq!(counts, [[48, 240], [96, 192]]);
    assert_eq!(mode_count.len(), 3);
    assert!(
        max.iter()
            .filter(|(key, _)| key.as_str() != "max_width_ns")
            .all(|(_, m)| *m < 1e-12)
    );
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"trials":rows.len(),"declined_accepted_locked_free":counts,"accepted_by_mode":mode_count,"maxima":max})
        )?
    );
    Ok(())
}
