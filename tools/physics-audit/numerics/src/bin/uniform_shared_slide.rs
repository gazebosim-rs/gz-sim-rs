// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply the uniform-slide phase directory")?,
    );
    let read = |f: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(f))?)?)
    };
    let field = read("uniform.json")?;
    let rows = field.as_array().ok_or("missing worlds")?;
    let mut spans = 0;
    let mut errors = [0f64; 11];
    let mut endings = BTreeMap::new();
    for row in rows {
        let r = &row["result"];
        spans += r["spans"].as_u64().ok_or("spans")?;
        *endings
            .entry(r["end"].as_str().ok_or("handoff")?)
            .or_insert(0) += 1;
        let n = r["last_registered_tick"].as_u64().ok_or("tick")?;
        if r["end"] == "slip reaches zero" {
            let t = r["first_zero_slip_s"].as_f64().ok_or("stop clock")?;
            assert!((n as f64) * 0.0005 < t && t <= (n + 1) as f64 * 0.0005 + 1e-14);
        }
        for(e,v) in errors.iter_mut().zip(r["errors_traction_speed_share_center_work_normal_tangent_position_velocity_spin_energy"].as_array().ok_or("errors")?){*e=e.max(v.as_f64().ok_or("nonfinite diagnostic")?);}
        assert!(r["dissipated_j"].as_f64().ok_or("dissipated")? >= 0.);
    }
    assert_eq!(rows.len(), 96);
    assert!(spans > 30000 && endings.get("slip reaches zero").is_some_and(|&n| n >= 32));
    assert!(errors.iter().all(|v| *v < 1e-9));
    let demo = read("public.json")?;
    let r = &demo["registered"];
    let mut energy = [0.; 2];
    let mut points = [String::new(), String::new()];
    for sample in demo["measurements"].as_array().ok_or("samples")? {
        let x = 80.
            + sample["tick"].as_u64().ok_or("tick")? as f64
                / r["last_registered_tick"].as_u64().ok_or("end")? as f64
                * 590.;
        for (k, key) in ["dissipation_j", "belt_work_j"].into_iter().enumerate() {
            energy[k] += sample[key].as_f64().ok_or("energy")?;
            let y = 286. - energy[k] * 650.;
            points[k].push_str(&format!("{x:.2},{y:.2} "));
        }
    }
    assert!((energy[0] - r["dissipated_j"].as_f64().ok_or("loss")?).abs() < 1e-12);
    assert!((energy[1] - r["belt_work_j"].as_f64().ok_or("work")?).abs() < 1e-12);
    let svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="765" height="351" viewBox="0 0 765 351"><rect width="765" height="351" fill="white"/><g font-family="sans-serif" fill="#263945"><text x="60" y="30" font-size="20">Energy during uniform kinetic sliding</text><text x="60" y="55" font-size="13">Cumulative joules</text><path d="M80 63V286H670" fill="none" stroke="#586975"/><text x="45" y="94">0.30</text><text x="45" y="225">0.10</text><text x="58" y="290">0</text><text x="75" y="313">0</text><text x="630" y="313">82.5 ms</text><polyline points="{}" fill="none" stroke="#167165" stroke-width="3"/><polyline points="{}" fill="none" stroke="#b45826" stroke-width="3"/><text x="385" y="69" fill="#167165">Dissipated in relative slip</text><text x="385" y="211" fill="#b45826">Work from prescribed belts</text></g></svg>"##,
        points[0], points[1]
    );
    fs::write(dir.join("contact-work.svg"), svg)?;
    let result = json!({"worlds":rows.len(),"spans":spans,"ending_counts":endings,"peak_errors_traction_speed_fraction_pressure_work_normal_tangent_position_velocity_spin_energy":errors,"replay_last_registered_tick":r["last_registered_tick"],"replay_first_stop_s":r["first_zero_slip_s"],"replay_dissipation_and_belt_work_j":energy});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
