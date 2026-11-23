// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::new(f(&x[0]), f(&x[1]), f(&x[2]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(root.join(file))?)?)
    };
    let exact = read("exact.json")?;
    let flights = read("flights.json")?;
    let mut count = [[0u64; 2]; 2];
    let mut residual = [0f64; 7];
    let mut cone = 0f64;
    let mut time_error = 0f64;
    let mut span = [f64::INFINITY, f64::NEG_INFINITY];
    let mut measure = |q: &Value, r: &Value| {
        let j = v(&r["impulse_ns"]);
        let n = v(&r["normal"]);
        let normal = j.dot(n);
        let t = j - n * normal;
        let out = v(&r["loaded_after_m_s"]);
        let mu = f(&q["mu"]);
        assert!(normal > 0. && out.dot(n).abs() < 1e-11);
        assert!(
            r["cone_margins_kg_inv"]
                .as_array()
                .unwrap()
                .iter()
                .all(|x| f(x) > 0.)
        );
        assert!(f(&r["opening_before_m_s"]) < 0. && f(&r["opening_after_m_s"]) > 0.);
        assert!(t.length() <= mu * normal + 1e-12);
        if r["sticking"] == true {
            assert!(out.length() < 1e-11);
        } else {
            cone = cone.max((t + mu * normal * out.normalize()).length());
        }
        residual[0] = residual[0].max(j.distance(v(&r["oracle_ns"])));
        residual[1] = residual[1].max(f(&r["outgoing_velocity_error_m_s"]));
        residual[2] = residual[2].max(f(&r["loss_residual_j"]));
        residual[3] = residual[3].max(f(&r["geometric_residual_m"]));
        if !r["momentum_residual_ns"].is_null() {
            for (idx, key) in [
                "momentum_residual_ns",
                "angular_residual_nms",
                "energy_residual_j",
            ]
            .into_iter()
            .enumerate()
            {
                residual[4 + idx] = residual[4 + idx].max(f(&r[key]));
            }
        }
    };
    for q in exact.as_array().unwrap() {
        let r = &q["result"];
        let yes = r["admitted"].as_bool().unwrap();
        count[usize::from(q["free"].as_bool().unwrap())][usize::from(yes)] += 1;
        if yes {
            assert_eq!(r["site"], q["active"]);
            assert_eq!(r["sticking"], q["sticking"]);
            measure(q, r);
        }
    }
    for q in flights.as_array().unwrap() {
        let r = &q["result"];
        measure(q, r);
        assert_eq!(r["prior_reaction_ns"], 0.);
        assert_eq!(
            r["site"].as_u64().unwrap(),
            u64::from(f(&q["offset_y_m"]) > 0.)
        );
        time_error = time_error.max((f(&r["known_ns"]) - f(&r["actual_ns"])).abs());
        span[0] = span[0].min(f(&r["opening_after_m_s"]));
        span[1] = span[1].max(f(&r["opening_after_m_s"]));
    }
    assert_eq!(count, [[396, 684], [540, 540]]);
    assert_eq!(flights.as_array().unwrap().len(), 384);
    assert!(residual.iter().all(|x| *x < 1e-11) && time_error <= 1. && cone < 1e-12);
    let public = read("public.json")?;
    let demo = &public[2]["events"][0];
    let mut first = vec![];
    for lane in public.as_array().unwrap() {
        let label = lane["label"].as_str().unwrap();
        let t = read(&format!("traces/{label}.json"))?;
        for frame in t["frames"].as_array().unwrap() {
            let sum = frame["corner_ns"]
                .as_array()
                .unwrap()
                .iter()
                .map(f)
                .sum::<f64>();
            assert!((sum - f(&frame["normal_impulse_ns"])).abs() < 1e-13);
        }
        first.push(t["frames"][5]["corner_ns"].clone());
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"cold_trials":exact.as_array().unwrap().len(),"declined_accepted_locked_free":count,"fractional_ballast_trials":flights.as_array().unwrap().len(),"max_clock_error_ns":time_error,"max_residual_jvector_pointvelocity_dissipation_geometry_momentum_angular_energy":residual,"max_sliding_friction_residual_ns":cone,"ballast_opening_speed_range_m_s":span,"demo":demo,"native_quadrants_first_ordinary_refined_registered":first})
        )?
    );
    Ok(())
}
