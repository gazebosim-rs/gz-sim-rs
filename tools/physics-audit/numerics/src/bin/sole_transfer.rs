// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v3(v: &Value) -> DVec3 {
    DVec3::new(f(&v[0]), f(&v[1]), f(&v[2]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let base = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(base.join(file))?)?)
    };
    let trials = read("transfers.json")?;
    let mut error = [0f64; 7];
    let mut min = 1.;
    let mut max = 0f64;
    for t in trials.as_array().unwrap() {
        let r = &t["result"];
        let at = f(&r["event_s"]);
        let bracket = &r["event_bracket_s"];
        assert!(f(&bracket[0]) <= at && at <= f(&bracket[1]) && at > 0. && at < 0.0005);
        error[0] = error[0].max((at - f(&r["phase_s"])).abs());
        min = f64::min(min, at);
        max = max.max(at);
        error[1] = error[1].max((f(&r["independent_pressure_ns"]) - f(&r["pressure_ns"])).abs());
        for k in 0..2 {
            assert!(!r["old"].as_array().unwrap().contains(&r["new"][k]));
            for (a, b) in [
                ("first_smooth_ns", "reference_first_ns"),
                ("next_smooth_ns", "reference_next_ns"),
            ] {
                assert!(f(&r[a][k]) > 0.);
                error[2] = error[2].max((f(&r[a][k]) - f(&r[b][k])).abs());
            }
        }
        for (i, k) in ["c", "v", "w"].iter().enumerate() {
            error[3 + i] =
                error[3 + i].max(v3(&r["end"][k]).distance(v3(&r["independent_end"][k])));
        }
        error[6] = error[6].max(f(&r["momentum_residual_ns"]));
    }
    let count = trials.as_array().unwrap().len();
    assert_eq!(count, 210);
    assert!(error.iter().all(|x| *x < 1e-10));
    let public = read("public.json")?;
    for s in public.as_array().unwrap() {
        let tr = read(&format!("traces/{}.json", s["label"].as_str().unwrap()))?;
        assert_eq!(tr["frames"][1]["normal_impulse_ns"], s["first_tick_ns"]);
    }
    let a = json!({"matched_physical_returns":42,"fractional_transfers":count,"earliest_ns":min*1e9,"latest_ns":max*1e9,"max_time_error_s":error[0],"max_impulse_error_ns":error[1],"max_subspan_impulse_error_ns":error[2],"max_com_error_m":error[3],"max_velocity_error_m_s":error[4],"max_spin_error_rad_s":error[5],"max_momentum_residual_ns":error[6],"replay":public});
    fs::write(
        base.join("checks/readback.json"),
        serde_json::to_vec_pretty(&a)?,
    )?;
    println!("{a}");
    Ok(())
}
