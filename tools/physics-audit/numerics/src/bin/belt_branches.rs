// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply branch phase directory")?,
    );
    let read = |f: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(f))?)?)
    };
    let a = read("branches.json")?;
    let mut counts = [0u64; 4];
    let mut errors = [0f64; 6];
    for v in a.as_array().ok_or("field")? {
        let r = &v["result"];
        let hold = r["admits_held_stick"].as_bool().ok_or("holding")?;
        let slip = r["sampled_held_slip_solutions"]
            .as_u64()
            .ok_or("intervals")?
            > 0;
        counts[usize::from(hold) + 2 * usize::from(slip)] += 1;
        for (e, k) in errors.iter_mut().zip(
            r["errors_torque_force_normal_initial_work_acceleration_interval_energy"]
                .as_array()
                .ok_or("errors")?,
        ) {
            *e = e.max(k.as_f64().ok_or("finite")?);
        }
    }
    assert_eq!(counts, [0, 72, 12, 348]);
    assert!(errors.iter().all(|x| *x < 1e-9));
    let p = read("public.json")?;
    let read_frames = |name: &str| -> Result<Vec<Value>, Box<dyn Error>> {
        Ok(read(&format!("traces/{name}.json"))?["frames"]
            .as_array()
            .ok_or("frames")?
            .clone())
    };
    let stick = read_frames("sticking")?;
    let slip = read_frames("slipping")?;
    let native = read_frames("ordinary")?;
    assert_eq!(stick.len(), slip.len());
    let dist = |a: &Value, b: &Value| -> f64 {
        let center = |frame: &Value, j: usize| {
            0.5 * (frame["foot"]["position"][j].as_f64().unwrap()
                + frame["upper"]["position"][j].as_f64().unwrap())
        };
        (0..3)
            .map(|j| (center(a, j) - center(b, j)).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    let mut common = 0;
    let mut end = [0.; 3];
    let mut points = [String::new(), String::new()];
    let threshold = p["common_incoming_s"].as_f64().unwrap();
    for (k, (a, b)) in stick.iter().zip(&slip).enumerate() {
        let tn = a["time_us"].as_u64().unwrap() as f64 * 1e-6;
        let d = dist(a, b);
        end = [d, dist(a, &native[k]), dist(b, &native[k])];
        if tn < threshold {
            assert!(d < 1e-15);
            common += 1;
        }
        for (l, v) in end[..2].iter().enumerate() {
            points[l].push_str(&format!(
                "{:.2},{:.2} ",
                70. + tn / 0.053 * 660.,
                263. - v * 1e5
            ));
        }
    }
    assert!((end[0] - p["last_branch_distance_m"].as_f64().unwrap()).abs() < 1e-13);
    let chart = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="775" height="325" viewBox="0 0 775 325"><rect width="775" height="325" fill="white"/><g font-family="sans-serif" fill="#243943"><text x="48" y="30" font-size="20">Departure from one feasible sticking trajectory</text><text x="48" y="55" font-size="13">Center displacement (mm)</text><path d="M70 64V263H735" stroke="#798b94" fill="none"/><text x="48" y="167">1</text><text x="48" y="267">0</text><text x="66" y="289">0</text><text x="698" y="289">53 ms</text><path d="M172.28 77V263" stroke="#848b87" stroke-dasharray="4 4"/><text x="185" y="82" font-size="12">Toe belt reaches zero slip</text><polyline points="{}" stroke="#178270" fill="none" stroke-width="3"/><polyline points="{}" stroke="#bf6229" fill="none" stroke-width="2"/><text x="469" y="111" fill="#178270">Feasible immediate slip</text><text x="469" y="213" fill="#bf6229">One ordinary native solve</text></g></svg>"##,
        points[0], points[1]
    );
    fs::write(dir.join("branch-distance.svg"), chart)?;
    let result = json!({"incoming":counts.iter().sum::<u64>(),"neither_stickonly_sampledsliponly_both":counts,"maximum_analytic_errors":errors,"matched_common_frames":common,"replay_end_distance_slip_stick_native_stick_native_slip_m":end,"reference":p});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}
