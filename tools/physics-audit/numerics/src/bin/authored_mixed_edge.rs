// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check per-case SDF admissions, timing, set widths and public reaction accounting.
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(path.join(file))?)?)
    };
    let tangent = read("tangent.json")?;
    let flights = read("flights.json")?;
    let mut counts = [[0u32; 2]; 2];
    let mut modes = BTreeMap::new();
    let mut clock = 0f64;
    let mut end_error = 0f64;
    let mut width = 0f64;
    for x in tangent.as_array().unwrap() {
        let r = &x["result"];
        let yes = r["admitted"].as_bool().unwrap();
        counts[usize::from(x["free"].as_bool().unwrap())][usize::from(yes)] += 1;
        if !yes {
            assert_eq!(r["unchanged"], true);
            continue;
        }
        assert_eq!(r["mode"], x["mode"]);
        assert_eq!(r["calls"], 8);
        end_error = end_error.max(f(&r["endpoints_error_ns"]));
        let span = f(&r["range_ns"][1]) - f(&r["range_ns"][0]);
        assert!(if x["mode"] == "both-stick" {
            span > 0.
        } else {
            span == 0.
        });
        width = width.max(span);
    }
    for x in flights.as_array().unwrap() {
        let r = &x["result"];
        assert_eq!(x["mode"], r["mode"]);
        *modes.entry(r["mode"].as_str().unwrap()).or_insert(0) += 1;
        assert_eq!(r["prior_reaction_ns"], 0.);
        assert_eq!(r["calls"], 8);
        clock = clock.max((f(&r["expected_ns"]) - f(&r["actual_ns"])).abs());
        end_error = end_error.max(f(&r["endpoints_error_ns"]));
        assert!(f(&r["pair_scalar_ns"]) + 1e-14 >= f(&r["normal_ns"][0]) + f(&r["normal_ns"][1]));
    }
    assert_eq!(counts, [[48, 240], [96, 192]]);
    assert_eq!(modes.len(), 3);
    assert!(modes.values().all(|n| *n == 64));
    assert!(clock <= 1. && end_error < 1e-12);
    let public = read("public.json")?;
    let mut loads = vec![];
    for x in public.as_array().unwrap() {
        let label = x["label"].as_str().unwrap();
        let trace = read(&format!("traces/{label}.json"))?;
        let frames = trace["frames"].as_array().unwrap();
        assert_eq!(frames.len(), 49);
        for (tick, row) in frames.iter().enumerate() {
            assert_eq!(row["tick"], tick);
            assert!(
                (row["corner_ns"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(f)
                    .sum::<f64>()
                    - f(&row["normal_impulse_ns"]))
                .abs()
                    < 1e-13
            );
        }
        loads.push(f(&frames[5]["normal_impulse_ns"]));
    }
    let ev = public[2]["events"]
        .as_array()
        .unwrap()
        .iter()
        .find(|q| !q["jump"].as_array().unwrap().is_empty())
        .unwrap();
    assert_eq!(ev["tick"], 5);
    for s in ev["jump"].as_array().unwrap() {
        assert_eq!(s["at_ns"], 138197);
        assert_eq!(s["mode"], "FirstSticks");
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"declined_accepted_locked_free":counts,"fractional_cases":flights.as_array().unwrap().len(),"fractional_modes":modes,"max_clock_error_ns":clock,"max_set_endpoint_error_ns":end_error,"max_range_width_ns":width,"replay_tick5_ordinary_refined_registered_ns":loads,"registered_jump":ev})
        )?
    );
    Ok(())
}
