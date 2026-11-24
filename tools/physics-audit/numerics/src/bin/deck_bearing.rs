// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn n(v: &Value) -> f64 {
    v.as_f64().expect("finite number")
}
fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply the phase directory")?,
    );
    let get = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(out.join(name))?)?)
    };
    let tips = get("tips.json")?;
    let mut errors = [0f64; 7];
    let mut edge = [0usize; 4];
    let mut ends = std::collections::BTreeMap::new();
    let mut duration = [f64::INFINITY, 0f64];
    let mut horizontal = [f64::INFINITY, 0f64];
    let mut spin = 0f64;
    let mut native = [0f64; 8];
    let mut worlds = 0;
    let mut spans = 0u64;
    let mut event = 0f64;
    let mut early = Vec::new();
    let mut max_qualified_horizontal = 0f64;
    for item in tips.as_array().unwrap() {
        edge[item["support_edge"].as_u64().unwrap() as usize] += 1;
        *ends.entry(item["end"].as_str().unwrap()).or_insert(0usize) += 1;
        let t = n(&item["duration_s"]);
        duration = [duration[0].min(t), duration[1].max(t)];
        let v = n(&item["horizontal_impulse_ns"]);
        horizontal = [horizontal[0].min(v), horizontal[1].max(v)];
        spin = spin.max(n(&item["max_tilt_rad"]));
        for (a, v) in errors.iter_mut().zip(
            item["errors_com_velocity_spin_area_vector_angular_cp"]
                .as_array()
                .unwrap(),
        ) {
            *a = a.max(n(v));
        }
        if let Some(controls) = item["registered"].as_array() {
            for row in controls {
                worlds += 1;
                spans += row["positive_spans"].as_u64().unwrap();
                event = event.max((n(&row["face_exit_s"]) - n(&row["fraction_s"])).abs());
                max_qualified_horizontal =
                    max_qualified_horizontal.max(n(&row["max_horizontal_tick_impulse_ns"]));
                let h = row["first_ordinary_tick"].as_u64().unwrap();
                let stop = row["first_possible_ordinary_tick"].as_u64().unwrap();
                assert!(h <= stop + 1);
                early.push(stop.saturating_sub(h));
                for (a, v) in native.iter_mut().zip(
                    row["errors_vector_receipt_scalar_vector_angular_com_v_w_q"]
                        .as_array()
                        .unwrap(),
                ) {
                    *a = a.max(n(v));
                }
            }
        }
    }
    early.sort();
    let sizes = tips.as_array().unwrap().len();
    assert_eq!(sizes, 56);
    assert!(edge[0] > 20 && edge[1] > 20);
    assert!(errors.iter().all(|v| *v < 1e-10));
    assert!(horizontal[0] > 0.);
    assert!(
        worlds == 0 || worlds == 3 * sizes && native.iter().all(|v| *v < 2e-8) && event < 2e-10
    );
    let ends_early = if early.is_empty() {
        Value::Null
    } else {
        json!({"exact_tick":early.iter().filter(|v|**v==0).count(),"p95":early[(early.len()*95).div_ceil(100)-1],"max":early[early.len()-1]})
    };
    let summary = json!({"trajectories":sizes,"deck_sides":edge,"physical_end":ends,"duration_s":duration,"max_tilt_rad":spin,"horizontal_impulse_ns":horizontal,"max_errors_com_velocity_spin_scalar_vector_angular_cp":errors,"registered_worlds":worlds,"registered_intervals":spans,"registered_max_errors_vector_receipt_scalar_vector_angular_com_v_w_q":native,"max_exit_phase_error_s":event,"registered_max_horizontal_tick_impulse_ns":max_qualified_horizontal,"early_handoff_public_ticks":ends_early});
    fs::write(
        out.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
