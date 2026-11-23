// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn p95(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[(v.len() * 95).div_ceil(100) - 1]
}
fn main() -> Result<(), Box<dyn Error>> {
    let d = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(d.join(name))?)?)
    };
    let field = read("cycles.json")?;
    let mut errors = [0f64; 7];
    let mut width = 0f64;
    let mut time = [f64::INFINITY, 0.];
    let mut samples = 0;
    let mut arms = 0;
    let mut modes = std::collections::BTreeMap::<String, Vec<f64>>::new();
    let mut native_windows = 0;
    let mut native_error = [0f64; 5];
    let mut advanced = 0u64;
    for row in field.as_array().unwrap() {
        let v = f(&row["speed"]);
        let inertia = f(&row["inertia"]);
        let r = ((inertia - 0.02) / (inertia + 0.02)).max(0.);
        let p = &row["physical"];
        assert!((r - f(&p["speed_ratio"])).abs() < 4e-16);
        assert!((0.2 * v * (1. + r) / (1. - r) - f(&p["sum_instantaneous_ns"])).abs() < 1e-12);
        let window = p["settled_s"].as_array().unwrap();
        width = width.max(f(&window[1]) - f(&window[0]));
        time[0] = time[0].min(f(&window[0]));
        time[1] = time[1].max(f(&window[1]));
        for k in 0..7 {
            errors[k] = errors[k].max(f(
                &p["errors_com_velocity_spin_force_smooth_impulse_speed"][k]
            ));
        }
        for branch in p["tested_spatial_cycles"].as_array().unwrap() {
            samples += 1;
            assert!(
                (f(&branch["spatial_ns"][1]) - f(&branch["before_and_after_ns"][1])).abs() < 1e-10
            );
            if branch["mode"].as_str().unwrap().starts_with("Edge") {
                arms += 1;
            }
        }
        for lane in row["native"].as_array().unwrap() {
            let name = lane["mode"].as_str().unwrap();
            modes
                .entry(name.into())
                .or_default()
                .push(f(&lane["max_spin_after_settling_rad_s"]));
            if name == "limit" {
                let measured = lane["registered_settle_window_s"].as_array().unwrap();
                assert!(
                    f(&measured[0]) <= f(&window[1]) + 1e-8
                        && f(&measured[1]) >= f(&window[0]) - 1e-8
                );
                assert!(
                    (f(&lane["sum_registered_jumps_ns"]) - f(&p["sum_instantaneous_ns"])).abs()
                        < 1e-7
                );
                assert_eq!(lane["last_registered_tick"], lane["last_tick"]);
                for k in 0..5 {
                    native_error[k] = native_error[k].max(f(&lane["limit_errors"][k]));
                }
                advanced += lane["limit_steps"].as_u64().unwrap();
                native_windows += 1;
            }
        }
    }
    assert_eq!(field.as_array().unwrap().len(), 96);
    assert!(errors.iter().all(|e| *e < 1e-10));
    assert!(width < 2.6e-8);
    let spin: Value = modes
        .into_iter()
        .map(|(name, spin)| (name, json!(p95(spin))))
        .collect::<serde_json::Map<_, _>>()
        .into();
    let replay = read("public.json")?;
    let manifest = read("comparison.json")?;
    for lane in manifest["lanes"].as_array().unwrap() {
        let tr = read(lane["file"].as_str().unwrap())?;
        assert_eq!(tr["frames"].as_array().unwrap().len(), 111);
    }
    let mut costs = std::collections::BTreeMap::<String, Vec<[u64; 2]>>::new();
    if d.join("cost.json").exists() {
        let runs = read("cost.json")?;
        assert_eq!(runs.as_array().unwrap().len(), 90);
        for run in runs.as_array().unwrap() {
            for tick in run["ticks"].as_array().unwrap() {
                let class = tick["kind"].as_str().unwrap();
                if class == "other" {
                    continue;
                }
                let key = format!("{} / {}", run["mode"].as_str().unwrap(), class);
                costs.entry(key).or_default().push([
                    tick["caller_cpu_ns"].as_u64().unwrap(),
                    tick["wall_ns"].as_u64().unwrap(),
                ]);
            }
        }
    }
    let costs=costs.into_iter().map(|(key,values)|{
        let nd=values.len();
        let metrics=[0,1].map(|i|{
            let mut v=values.iter().map(|k|k[i]).collect::<Vec<_>>(); v.sort_unstable();
            [v[nd.div_ceil(2)-1],v[(nd*95).div_ceil(100)-1]]
        });
        (key,json!({"samples":nd,"caller_cpu_p50_p95_ns":metrics[0],"wall_p50_p95_ns":metrics[1]}))
    }).collect::<std::collections::BTreeMap<_,_>>();
    let summary = json!({"costs":costs,"fixtures":96,"checked_spatial_cycles":samples,"two_bearing_jumps":arms,"settling_times_s":time,"max_settling_interval_s":width,"max_spatial_com_velocity_spin_force_smooth_jump_outgoing_errors":errors,"p95_postsettle_peak_spin_rad_s":spin,"registered_worlds":native_windows,"registered_segments":advanced,"registered_max_impulse_time_com_velocity_spin":native_error,"replay":replay});
    fs::write(
        d.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
