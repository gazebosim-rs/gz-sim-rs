// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply native sole transfer directory")?,
    );
    let read = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(name))?)?)
    };
    let field = read("transfers.json")?;
    let mut reports = 0;
    let mut states = 0;
    let mut error = [0f64; 6];
    let mut calls = std::collections::BTreeSet::new();
    for row in field.as_array().unwrap() {
        let d = &row["result"];
        let phase = f(&d["phase_s"]);
        let j = f(&d["independent_pressure_ns"]);
        let sites = |name: &str| {
            d[name]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_u64().unwrap() as usize)
                .collect::<Vec<_>>()
        };
        for m in d["registered"].as_array().unwrap() {
            for n in [m, &m["alternate_grid"][0]] {
                let t = f(&n["time_s"]);
                assert!((f(&n["event_bracket_s"][0])..=f(&n["event_bracket_s"][1])).contains(&t));
                calls.insert(n["calls"].as_u64().unwrap());
                reports += 1;
                error[0] = error[0].max((t - phase).abs());
                error[1] = error[1].max((f(&n["pressure_ns"]) - j).abs());
                let mut total = j;
                for (a, b, key) in [
                    ("before_equivalent_ns", "reference_first_ns", "old"),
                    ("after_equivalent_ns", "reference_next_ns", "new"),
                ] {
                    for (i, k) in sites(key).into_iter().enumerate() {
                        error[2] = error[2].max((f(&n[a][k]) - f(&d[b][i])).abs());
                    }
                    total += n[a].as_array().unwrap().iter().map(f).sum::<f64>();
                }
                assert!((total - f(&n["public_normal_ns"])).abs() < 1e-10);
            }
            for i in 0..3 {
                error[i + 3] = error[i + 3].max(f(&m["state_errors"][i]));
            }
            states += 1;
        }
    }
    assert_eq!((reports, states), (840, 420));
    assert_eq!(calls.into_iter().collect::<Vec<_>>(), [1, 8]);
    assert!(error.iter().all(|v| *v < 1e-9));
    let public = read("public.json")?;
    for lane in public.as_array().unwrap() {
        let file = format!("traces/{}.json", lane["mode"].as_str().unwrap());
        assert_eq!(
            lane["first_public_ns"],
            read(&file)?["frames"][1]["normal_impulse_ns"]
        );
    }
    let summary = json!({"physical_returns":42,"clock_phases":5,"world_orientations":2,"authored_cases":states,"authored_and_alternate_native_receipts":reports,"native_grids":[1,8],"max_time_error_s":error[0],"max_jump_error_ns":error[1],"max_equivalent_endpoint_integral_error_ns":error[2],"max_com_error_m":error[3],"max_velocity_error_m_s":error[4],"max_spin_error_rad_s":error[5],"replay":public});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
