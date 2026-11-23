// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec2;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn endpoint(v: &Value) -> (f64, DVec2) {
    let j = [0, 1, 2, 3].map(|i| f(&v[i]));
    assert!(j.into_iter().all(|x| x >= -1e-14));
    let t = j.iter().sum::<f64>();
    (
        t,
        DVec2::new(j[2] + j[3] - j[0] - j[1], j[1] + j[3] - j[0] - j[2]) / t,
    )
}
fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(root.join(file))?)?)
    };
    let mut modes = BTreeMap::new();
    let mut n = 0;
    let mut residuals = [0f64; 9];
    let mut max_range = 0f64;
    let keys = [
        "normal_error_ns",
        "cop_error",
        "site_error_m",
        "port_error_m_s",
        "work_error_j",
        "equivalent_error",
        "linear_residual_ns",
        "angular_residual_nms",
        "energy_residual_j",
    ];
    for file in ["physical.json", "manufactured.json"] {
        let v = read(file)?;
        for r in v.as_array().unwrap() {
            assert_eq!(r["result"]["native"].as_array().unwrap().len(), 4);
            for s in r["result"]["native"].as_array().unwrap() {
                n += 1;
                let locked = !s["free"].as_bool().unwrap();
                *modes
                    .entry(format!(
                        "{}:{}",
                        if locked { "locked" } else { "free" },
                        s["mode"].as_str().unwrap()
                    ))
                    .or_insert(0usize) += 1;
                let base = endpoint(&s["lo_loads_ns"]);
                let other = endpoint(&s["hi_loads_ns"]);
                let c = DVec2::new(f(&s["center_normalized"][0]), f(&s["center_normalized"][1]));
                assert!(
                    (base.0 - f(&s["normal_ns"])).abs() < 1e-12
                        && (other.0 - base.0).abs() < 1e-12
                        && c.distance(base.1) < 1e-12
                        && c.distance(other.1) < 1e-12
                );
                if locked {
                    assert!((base.0 - f(&r["result"]["resultant_ns"])).abs() < 1e-12);
                }
                let width = f(&s["shift_width_ns"]);
                max_range = max_range.max(width);
                for k in 0..4 {
                    let delta = f(&s["hi_loads_ns"][k]) - f(&s["lo_loads_ns"][k]);
                    assert!((delta - [1., -1., -1., 1.][k] * width).abs() < 1e-12);
                }
                for (i, key) in keys.iter().enumerate() {
                    residuals[i] = residuals[i].max(f(&s[key]));
                }
            }
        }
    }
    assert_eq!(n, 1464);
    assert!(residuals.iter().all(|x| *x < 1e-11));
    let flights = read("flights.json")?;
    let fz = flights.as_array().unwrap();
    assert_eq!(fz.len(), 40);
    let mut clock = 0f64;
    for s in fz {
        assert_eq!(s["prior_load_ns"], 0.);
        clock = clock.max((f(&s["target_ns"]) - f(&s["actual_ns"])).abs());
        assert!((f(&s["normal_ns"]) - f(&s["expected_ns"])).abs() < 1e-11);
    }
    assert!(clock <= 1.);
    let public = read("public.json")?;
    for s in public.as_array().unwrap() {
        let tr = read(&format!("traces/{}.json", s["mode"].as_str().unwrap()))?;
        assert_eq!(tr["frames"].as_array().unwrap().len(), 25);
        assert_eq!(tr["frames"][1]["normal_impulse_ns"], s["first_public_ns"]);
        assert!(
            tr["frames"]
                .as_array()
                .unwrap()
                .iter()
                .all(|r| r.get("corner_ns").is_none())
        );
    }
    let a = json!({"native_and_authored_cases":n,"physical_and_designed_configs":366,"active_modes":modes,"maximum_diagonal_freedom_ns":max_range,"residuals":keys.into_iter().zip(residuals).collect::<BTreeMap<_,_>>(),"fractional_flights":fz.len(),"max_clock_error_ns":clock,"replay":public[2]});
    fs::write(
        root.join("checks/readback.json"),
        serde_json::to_vec_pretty(&a)?,
    )?;
    println!("{a}");
    Ok(())
}
