// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply sweep phase directory")?,
    );
    let data: Value = serde_json::from_slice(&fs::read(dir.join("sweep.json"))?)?;
    let a = data.as_array().ok_or("field")?;
    let mut matches = 0;
    let mut recovered = 0;
    let mut hidden = 0;
    let mut err = [0f64; 3];
    for r in a {
        let m = r["minimum_margin_m"].as_f64().ok_or("physical margin")?;
        let held = r["full_registered_tick"].as_bool().ok_or("qualified")?;
        assert_eq!(held, m > 0.);
        matches += usize::from(held);
        recovered += usize::from(held && r["earlier_symmetric_erosion_declines"] == true);
        hidden += usize::from(
            m < 0.
                && r["initial_margin_m"].as_f64().unwrap() > 0.
                && r["end_margin_m"].as_f64().unwrap() > 0.,
        );
        for (v, k) in err
            .iter_mut()
            .zip(r["errors_COM_impulse_center"].as_array().unwrap())
        {
            *v = v.max(k.as_f64().ok_or("finite")?);
        }
    }
    assert_eq!((a.len(), matches, recovered, hidden), (324, 180, 54, 54));
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(
            &json!({"sdf_worlds":a.len(),"whole_intervals_certified":matches,"monotone_inward_recovered":recovered,"internal_loss_rejected_with_both_endpoints_supported":hidden,"max_errors_COM_impulse_center":err}),
        )?,
    )?;
    Ok(())
}
