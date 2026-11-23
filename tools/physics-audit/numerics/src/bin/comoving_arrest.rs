// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply comoving-arrest directory")?,
    );
    let read = |f: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(f))?)?)
    };
    let field = read("arrests.json")?;
    let rows = field.as_array().ok_or("world list")?;
    let mut count = [0; 3];
    let mut err = [0f64; 11];
    let mut times = 0f64;
    for row in rows {
        let r = &row["result"];
        for (v, n) in count.iter_mut().zip(
            r["spans_kinetic_mixed_rest"]
                .as_array()
                .ok_or("phase categories")?,
        ) {
            *v += n.as_u64().ok_or("ticks")?;
        }
        let got = r["physical_stops_s"].as_array().ok_or("stops")?;
        assert_eq!(got.len(), 3);
        for (t, e) in got.iter().zip([
            row["initial_stop_s"].as_f64().ok_or("first stop")?,
            0.042 + 0.011621537,
            0.080 + 0.017408771,
        ]) {
            times = times.max((t.as_f64().ok_or("clock")? - e).abs());
        }
        for (v,e) in err.iter_mut().zip(r["errors_clock_fraction_speed_center_siteimpulse_totalimpulse_work_COM_velocity_spin_energy"].as_array().ok_or("metrics")?){*v=v.max(e.as_f64().ok_or("finite metric")?);}
    }
    assert_eq!(rows.len(), 96);
    assert_eq!(count.iter().sum::<u64>(), 96 * 220);
    assert_eq!(count[1], 288);
    assert!(times < 2e-14 && err.iter().all(|v| *v < 1e-8));
    let demo = read("public.json")?;
    let cells = demo["samples"].as_array().ok_or("samples")?;
    let mut j = 0.;
    let mut w = 0.;
    let mut paths = [String::new(), String::new()];
    let mut last = 0.;
    for row in cells {
        let slip = row["kinetic_seconds"].as_f64().ok_or("slip")?;
        let rest = row["rest_seconds"].as_f64().ok_or("rest")?;
        assert!((slip + rest - 0.0005).abs() < 1e-15);
        let tick = row["tick"].as_u64().ok_or("tick")?;
        j += row["dissipated_j"].as_f64().ok_or("loss")?;
        w += row["belt_work_j"].as_f64().ok_or("work")?;
        let x = 70. + 640. * tick as f64 / 220.;
        paths[0].push_str(&format!("{x:.2},{:.2} ", 162. - j * 1150.));
        paths[1].push_str(&format!("{x:.2},{:.2} ", 162. - w * 1150.));
        last = x;
    }
    let mut chart = String::from(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="780" height="332" viewBox="0 0 780 332"><rect width="780" height="332" fill="white"/><g font-family="sans-serif" fill="#243943"><text x="52" y="32" font-size="20">Energy exchange as the belts arrest the foot three times</text><text x="52" y="58" font-size="13">Cumulative energy (J)</text><path d="M70 69V259H714M70 162H714" fill="none" stroke="#8a98a0"/><text x="40" y="166">0</text><text x="16" y="238">−0.06</text><text x="66" y="285">0</text><text x="653" y="285">110 ms</text>"##,
    );
    for t in demo["field"]["physical_stops_s"]
        .as_array()
        .ok_or("demo times")?
    {
        let x = 70. + (last - 70.) * t.as_f64().ok_or("stop")? / 0.11;
        chart +=
            &format!(r##"<path d="M{x:.2} 74V258" stroke="#9caca5" stroke-dasharray="4 4"/>"##);
    }
    for (points, color) in paths.iter().zip(["#147c68", "#ae532b"]) {
        chart += &format!(
            r#"<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3"/>"#
        );
    }
    chart += r##"<text x="82" y="104" fill="#147c68">Relative-slip dissipation</text><text x="458" y="225" fill="#ae532b">Prescribed belt work</text><text x="195" y="311" font-size="13">Dashed lines mark the three independently predicted stops</text></g></svg>"##;
    fs::write(dir.join("arrest-energy.svg"), chart)?;
    let result = json!({"worlds":rows.len(),"actual_arrests":rows.len()*3,"public_ticks_by_kinetic_mixed_rest":count,"max_arrival_clock_error_s":times,"peak_errors_clock_fraction_speed_center_siteimpulse_totalimpulse_work_COM_velocity_spin_energy":err,"replay_physical_stops_s":demo["field"]["physical_stops_s"],"replay_dissipated_and_support_work_j":[j,w]});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
