// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn number(v: &Value) -> f64 {
    v.as_f64().expect("finite measurement")
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply the phase directory")?,
    );
    let read = |s: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(s))?)?)
    };
    let field = read("field.json")?;
    let mut counts = [0usize; 3];
    let mut orbit = 0;
    let mut errors = [0f64; 4];
    let mut native_errors = [0f64; 7];
    let mut native = 0;
    let mut spans = 0u64;
    let mut exit_error = 0f64;
    let mut coverage = [1f64, 0.];
    let mut vertices = std::collections::BTreeSet::new();
    for row in field.as_array().unwrap() {
        let data = &row["result"];
        let mode = data["mode"].as_str().unwrap();
        let k = match mode {
            "supported" => 0,
            "pressure exit" => 1,
            "unavailable" => 2,
            _ => panic!("unknown physical mode"),
        };
        counts[k] += 1;
        if row["orbit"] == true {
            orbit += 1;
            assert_eq!(k, 1);
        }
        let fraction = number(&data["initial_overlap_fraction"]);
        assert!(fraction > 0. && fraction < 1. + 1e-12);
        coverage = [coverage[0].min(fraction), coverage[1].max(fraction)];
        if k == 2 {
            assert!(number(&data["initial_pressure_margin_m"]) <= 0.);
        } else {
            vertices.insert(data["initial_vertices"].as_u64().unwrap());
            for (metric, raw) in errors
                .iter_mut()
                .zip(data["errors_force_cp_area_alpha"].as_array().unwrap())
            {
                *metric = metric.max(number(raw));
            }
            if k == 1 {
                exit_error = exit_error
                    .max((number(&data["physical_exit_s"]) - number(&data["exit_s"])).abs());
                assert!(data["pad_edges"].as_u64().unwrap().is_power_of_two());
                assert!(number(&data["released"]["force_n"]) > 0.);
            }
        }
        if let Some(runs) = data["registered"].as_array() {
            for control in runs {
                native += 1;
                let n = control["committed"].as_u64().unwrap();
                spans += n;
                if k == 2 {
                    assert_eq!(n, 0);
                }
                if k == 1 {
                    assert_eq!(
                        control["first_native_handoff_tick"].as_u64(),
                        Some((number(&data["exit_s"]) / 0.0005).ceil() as u64)
                    );
                }
                for (metric, raw) in native_errors.iter_mut().zip(
                    control["errors_cp_area_force_moment_com_velocity_spin"]
                        .as_array()
                        .unwrap(),
                ) {
                    *metric = metric.max(number(raw));
                }
            }
        }
    }
    assert_eq!(counts, [86, 68, 50]);
    assert_eq!(orbit, 12);
    assert!(exit_error < 5e-13 && errors.iter().all(|x| *x < 1e-10));
    assert!(native == 0 || native == 408 && spans > 1 && native_errors.iter().all(|e| *e < 1e-9));
    let manifest = read("comparison.json")?;
    let capture = manifest["capture_ticks"]
        .as_object()
        .unwrap()
        .values()
        .next()
        .unwrap()
        .as_u64()
        .unwrap();
    for l in manifest["lanes"].as_array().unwrap() {
        let trace = read(l["file"].as_str().unwrap())?;
        assert!(trace["frames"].as_array().unwrap().len() > capture as usize);
    }
    let result = json!({"fixtures":204,"supported_through_horizon":counts[0],"departures":counts[1],"initially_outside":counts[2],"hidden_departures":orbit,"area_fraction":coverage,"polygon_vertices":vertices,"max_force_center_area_acceleration_errors":errors,"max_exit_time_error_s":exit_error,"registered_worlds":native,"registered_spans":spans,"registered_cp_area_force_moment_com_velocity_spin_errors":native_errors,"capture_tick":capture});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
