// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn quant(mut x: Vec<f64>, p: f64) -> f64 {
    x.sort_by(f64::total_cmp);
    x[(p * x.len() as f64).ceil() as usize - 1]
}
fn vec(x: &Value) -> DVec3 {
    DVec3::new(f(&x[0]), f(&x[1]), f(&x[2]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let base = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(base.join(file))?)?)
    };
    let field = read("native.json")?;
    let mut gaps = vec![];
    let mut segments = 0;
    let mut only = 0;
    let keys = [
        "max_integral_error_ns",
        "max_site_error_m",
        "max_linear_residual_ns",
        "max_angular_residual_nms",
        "max_state_error",
        "max_twist_error",
        "max_force_envelope_violation_n",
    ];
    let mut max = [0f64; 7];
    for r in field.as_array().unwrap() {
        let v = &r["result"];
        let n = v["segments"].as_u64().unwrap();
        if r["both_bear"].as_bool().unwrap() {
            assert!(n > 0);
            gaps.push((f(&v["physical_end_seconds"]) - f(&v["smooth_seconds"])) * 1e6);
            assert!(gaps.last().unwrap() > &0.);
        } else {
            assert_eq!(n, 0);
            only += 1;
        }
        segments += n;
        for (i, k) in keys.iter().enumerate() {
            max[i] = max[i].max(f(&v[k]));
        }
    }
    assert_eq!(only, 300);
    assert_eq!(gaps.len(), 84);
    assert!(max.iter().all(|v| *v < 1e-9));
    let costs = read("cost.json")?;
    let mut timing = vec![];
    for label in ["ordinary", "projection", "edge"] {
        let mut samples = vec![];
        let mut landing = vec![];
        let mut medians = vec![];
        for r in costs
            .as_array()
            .unwrap()
            .iter()
            .filter(|r| r["mode"] == label)
        {
            let values = r["step_cost_ns"].as_array().unwrap();
            assert_eq!(values.len(), 100);
            landing.push(f(&values[0]) * 1e-3);
            let settles = values[1..98]
                .iter()
                .map(|v| f(v) * 1e-3)
                .collect::<Vec<_>>();
            medians.push(quant(settles.clone(), 0.5));
            samples.extend(settles);
        }
        assert_eq!(medians.len(), 9);
        timing.push(json!({"mode":label,"bearing_sample_count":samples.len(),"p50_us":quant(samples.clone(),0.5),"p95_us":quant(samples,0.95),"between_run_median_range_us":[quant(medians.clone(),1./9.),quant(medians,1.)],"first_tick_p50_us":quant(landing,0.5)}));
    }
    let prior = read("../multifidelity-two-bearing/traces/cyclic.json")?;
    let ref_frames = prior["frames"].as_array().unwrap();
    let public = read("public.json")?;
    let mut lanes = vec![];
    for (i, label) in ["ordinary", "refined", "edge"].into_iter().enumerate() {
        let trace = read(&format!("traces/{label}.json"))?;
        let rows = trace["frames"].as_array().unwrap();
        assert_eq!(rows.len(), 106);
        assert_eq!(
            rows[1]["normal_impulse_ns"],
            public[i]["first_tick_reaction_ns"]
        );
        let mut error = [0f64; 3];
        for k in 1..=98 {
            let a = &rows[k];
            let b = &ref_frames[k];
            let com =
                |s: &Value| (vec(&s["foot"]["position"]) + vec(&s["upper"]["position"])) * 0.5;
            error[0] = error[0].max(com(a).distance(com(b)));
            error[1] = error[1].max(
                vec(&a["foot"]["angular_velocity"]).distance(vec(&b["foot"]["angular_velocity"])),
            );
            for (x, y) in a["corner_ns"]
                .as_array()
                .unwrap()
                .iter()
                .zip(b["corner_ns"].as_array().unwrap())
            {
                if k > 1 {
                    error[2] = error[2].max((f(x) - f(y)).abs());
                }
            }
        }
        lanes.push(json!({"label":label,"max_com_error_m":error[0],"max_spin_error_rad_s":error[1],"max_site_error_ns_excluding_impact":error[2]}));
    }
    assert_eq!(public[2]["last_bearing_tick"], 98);
    assert!(f(&lanes[2]["max_spin_error_rad_s"]) < 1e-10);
    assert!(f(&lanes[2]["max_site_error_ns_excluding_impact"]) < 1e-11);
    let a = json!({"trials":384,"transfers":only,"two_loaded":gaps.len(),"segments":segments,"earliest_handoff_us":quant(gaps.clone(),1./gaps.len() as f64),"furthest_handoff_us":quant(gaps,1.),"residuals":keys.into_iter().zip(max).collect::<std::collections::BTreeMap<_,_>>(),"costs":timing,"public_errors_before_handoff":lanes,"last_bearing_tick":98});
    fs::write(
        base.join("checks/readback.json"),
        serde_json::to_vec_pretty(&a)?,
    )?;
    println!("{a}");
    Ok(())
}
