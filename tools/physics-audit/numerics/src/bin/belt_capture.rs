// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply belt-capture directory")?,
    );
    let read = |f: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(f))?)?)
    };
    let raw = read("capture.json")?;
    let rows = raw.as_array().ok_or("field")?;
    let mut count = [0u64; 3];
    let mut difficult = 0;
    let mut error = [0f64; 8];
    let mut gap = 0f64;
    let mut min_separator = f64::INFINITY;
    let mut svg = String::from(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="775" height="369" viewBox="0 0 775 369"><rect width="775" height="369" fill="white"/><g font-family="sans-serif" fill="#273945"><text x="55" y="32" font-size="20">Admissible toe load while a moving belt holds the sole</text><text x="55" y="56" font-size="13">Fraction of normal load on the stationary-relative toe belt</text><path d="M72 77V278H702" stroke="#8c9ca3" fill="none"/><text x="47" y="82">1</text><text x="47" y="280">0</text>"##,
    );
    for row in rows {
        let r = &row["result"];
        match r["mode"].as_str() {
            Some("feasible") => {
                count[0] += 1;
                difficult +=
                    u64::from(r["positive_heel_requires_distributed_static_traction"] == true);
                let x = |t: &str, k: usize| r[t][k].as_f64().unwrap();
                gap = gap
                    .max(x("inner", 0) - x("outer", 0))
                    .max(x("outer", 1) - x("inner", 1));
                for (v, t) in error.iter_mut().zip(
                    r["errors_force_torque_cone_share_power_dual_bound_closedform"]
                        .as_array()
                        .ok_or("measures")?,
                ) {
                    *v = v.max(t.as_f64().ok_or("nonfinite")?);
                }
            }
            Some("infeasible") => {
                count[1] += 1;
                min_separator = min_separator.min(r["separation"].as_f64().ok_or("separator")?);
            }
            _ => count[2] += 1,
        }
        if row["rotation"] == 0.37
            && row["side"] == 1
            && row["gravity_tangent"] == 0.
            && row["center_coordinate"] == -0.045
        {
            let mu = row["static_mu"].as_f64().ok_or("mu")?;
            let sym = row["symmetric"] == true;
            let x = 90. + 620. * mu + if sym { -3. } else { 3. };
            let color = if sym { "#3974a2" } else { "#ba6227" };
            if r["mode"] == "feasible" {
                let y = |i: usize| 278. - 200. * r["inner"][i].as_f64().unwrap();
                svg += &format!(
                    r#"<path d="M{x:.2} {:.2}V{:.2}" stroke="{color}" stroke-width="5"/>"#,
                    y(0),
                    y(1)
                );
            } else {
                svg += &format!(
                    r#"<path d="M{:.2} 267l6 6m-6 0l6 -6" stroke="{color}" stroke-width="2"/>"#,
                    x - 3.
                );
            }
        }
    }
    assert_eq!(count, [456, 84, 0]);
    assert_eq!(difficult, 84);
    assert!(gap < 1e-8 && error.iter().all(|v| *v < 5e-9) && min_separator > 0.);
    for mu in [0.035, 0.18, 0.36, 0.65, 0.92] {
        svg += &format!(
            r#"<text x="{:.2}" y="302" font-size="12">{mu}</text>"#,
            80. + 620. * mu
        );
    }
    svg += r##"<text x="225" y="321">Toe static friction coefficient</text><text x="123" y="349" fill="#3974a2">Symmetric pads</text><text x="324" y="349" fill="#ba6227">Skewed pads</text><text x="514" y="349" font-size="12">× equilibrium impossible</text></g></svg>"##;
    fs::write(dir.join("static-loads.svg"), svg)?;
    let p = read("public.json")?;
    let result = json!({"incoming_states":rows.len(),"feasible_infeasible_unresolved":count,"requires_distributed_static_traction_for_positive_heel":difficult,"max_inner_outer_fraction_gap":gap,"max_force_torque_cone_share_power_dual_bound_closedform_errors":error,"minimum_separation_margin":min_separator,"demonstrated_45ms_toe_fraction":p["admissible_held_toe_fraction"],"selected_finite_fraction":p["held_toe_fraction"],"native":p["native"]});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
