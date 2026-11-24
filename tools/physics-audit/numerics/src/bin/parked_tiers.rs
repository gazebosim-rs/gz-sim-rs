// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconcile measured motion, support impulse, strict dwell and wake times.
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
fn read(p: &Path, name: &str) -> Value {
    serde_json::from_slice(&fs::read(p.join(name)).unwrap()).unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn pair(v: &Value) -> [f64; 2] {
    serde_json::from_value(v.clone()).unwrap()
}
fn held(p: &Path) -> Value {
    let data = read(p, "public.json");
    let a = data["tiered"].as_array().unwrap();
    let b = data["dynamic"].as_array().unwrap();
    let ta = read(p, "traces/tiered.json");
    let tb = read(p, "traces/dynamic.json");
    let mut max = 0f64;
    let mut poses = 0;
    assert_eq!(a.len(), 198);
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(
            x["prescribed"],
            (17..=32).contains(&i) || (41..=56).contains(&i)
        );
        for body in ["foot", "upper", "pad", "opponent"] {
            assert_eq!(ta["frames"][i][body], tb["frames"][i][body]);
            poses += 1;
        }
        let j = pair(&x["pair_ns"]);
        if x["prescribed"] == true {
            assert_eq!(j, [0.; 2]);
        } else {
            for (l, r) in j.into_iter().zip(pair(&y["pair_ns"])) {
                max = max.max((l - r).abs());
            }
        }
        assert_eq!(x["wake"].as_str(), (i == 57).then_some("MotionStarted"));
    }
    assert!(max < 5e-15);
    assert_eq!(pair(&a[57]["pair_ns"])[1], 0.);
    json!({"matched_body_state_readings":poses,"held_public_ticks":32,"maximum_per_pad_error_outside_hold_ns":max,"first_stationary_resumed_ns":a[33]["pair_ns"],"first_loss_resumed_ns":a[57]["pair_ns"],"wake_start_s":0.028,"compared_through_s":0.0985})
}
fn cycles(rows: &[Value]) -> Value {
    let mut errors = [0f64; 5];
    let mut first = 0f64;
    let mut count = 0;
    let mut held = [u64::MAX, 0];
    let mut delay = [u64::MAX, 0];
    for r in rows {
        let t = &r["tiered"];
        assert_eq!(t["wakes"], json!([17, 111, 205]));
        let transitions = t["transitions"].as_array().unwrap();
        assert_eq!(transitions.len(), 7);
        for (i, v) in transitions.iter().enumerate() {
            assert_eq!(v["prescribed"], i % 2 == 0);
        }
        for i in 0..3 {
            let at = [17, 111, 205][i] + 36;
            let physical_stop = (at - 1) as f64 * 0.0005 + n(&r["stop_s"]);
            let demote = u(&transitions[2 * i + 2]["tick"]);
            let elapsed = (demote - 1) as f64 * 0.0005 - physical_stop;
            assert!(elapsed + 1e-12 >= n(&r["dwell_ms"]) * 0.001);
            let ns = (elapsed * 1e9).round() as u64;
            delay = [delay[0].min(ns), delay[1].max(ns)];
        }
        let h = u(&r["prescribed_ticks"]);
        held = [held[0].min(h), held[1].max(h)];
        for (k, v) in r["errors_position_velocity_spin_quaternion_load"]
            .as_array()
            .unwrap()
            .iter()
            .enumerate()
        {
            errors[k] = errors[k].max(n(v));
        }
        for load in r["first_resumed"].as_array().unwrap() {
            count += 1;
            for (a, b) in pair(&load["switched"])
                .into_iter()
                .zip(pair(&load["registered_dynamic"]))
            {
                first = first.max((a - b).abs());
            }
        }
    }
    assert!(errors[..4].iter().all(|v| *v == 0.));
    assert_eq!(rows.len(), 36);
    assert_eq!(count, 108);
    json!({"trials":rows.len(),"first_resumptions":count,"prescribed_ticks_min_max":held,"physical_stop_to_automatic_demotion_ns":delay,"max_first_resumed_each_pad_ns":first,"maximum_difference_position_velocity_spin_orientation_subsequent_load":errors})
}
fn main() -> Result<(), Box<dyn Error>> {
    let p = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let summary = if p.join("cycles.json").exists() {
        let rows = read(&p, "cycles.json");
        let now = cycles(rows.as_array().unwrap());
        if p.ends_with("multifidelity-pressure-provenance") {
            let before = read(&p, "../multifidelity-shared-tier-rearming/cycles.json");
            let old = cycles(before.as_array().unwrap());
            for (a, b) in before
                .as_array()
                .unwrap()
                .iter()
                .zip(rows.as_array().unwrap())
            {
                for key in [
                    "mu",
                    "heading",
                    "stop_s",
                    "dwell_ms",
                    "tiered",
                    "always_dynamic",
                    "prescribed_ticks",
                ] {
                    assert_eq!(a[key], b[key]);
                }
            }
            assert!(n(&old["max_first_resumed_each_pad_ns"]) > 0.001);
            assert_eq!(n(&now["max_first_resumed_each_pad_ns"]), 0.);
            json!({"before":old,"after":now})
        } else {
            now
        }
    } else {
        held(&p)
    };
    fs::write(
        p.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
