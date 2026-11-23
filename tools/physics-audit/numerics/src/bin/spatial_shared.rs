// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let data: Value = serde_json::from_slice(&fs::read(dir.join("spatial.json"))?)?;
    let mut spans = 0;
    let mut both = 0;
    let mut errors = [0f64; 9];
    let mut varying = 0;
    for row in data.as_array().unwrap() {
        let r = &row["result"];
        spans += r["spans"].as_u64().unwrap();
        both += r["positive_both"].as_u64().unwrap();
        assert_eq!(r["end"], "horizon");
        assert_eq!(r["last_supported_tick"], 240);
        let a = &r["selected_fraction_toe"];
        if a[1].as_f64().unwrap() - a[0].as_f64().unwrap() > 1e-6 {
            varying += 1;
        }
        for (e, v) in errors.iter_mut().zip(
            r["fraction_outer_site_center_moment_vector_com_velocity_spin"]
                .as_array()
                .unwrap(),
        ) {
            *e = e.max(v.as_f64().unwrap());
        }
    }
    assert_eq!(data.as_array().unwrap().len(), 216);
    assert_eq!(spans, 51840);
    assert!(errors.iter().all(|v| *v < 1e-9));
    let result = json!({"sdf_worlds":216,"registered_spans":spans,"positive_both":both,"worlds_changing_representative":varying,"max_errors_fraction_outer_site_center_moment_vector_com_velocity_spin":errors});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
