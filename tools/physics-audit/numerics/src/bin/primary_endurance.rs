// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent readback of the end-to-end study, using output rows alone.
use serde_json::{Value, json};
use std::{collections::BTreeSet, error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("pass results directory")?);
    let runs: Value = serde_json::from_slice(&fs::read(path.join("epochs.json"))?)?;
    let mut report = vec![];
    for run in runs.as_array().unwrap() {
        let label = run["policy"].as_str().unwrap();
        let live = label != "ordinary";
        let mut hit_count = 0;
        let mut request_count = 0;
        let mut totals = 0;
        let mut holds = 0;
        let mut attempts = 0;
        let mut wrench = f64::default();
        let mut balance = f64::default();
        let (mut residual_z, mut residual_norm, mut tangent_z) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut coalesced = 0;
        let epochs = run["epochs"].as_array().unwrap();
        assert_eq!(epochs.len(), 4);
        for (epoch, e) in epochs.iter().enumerate() {
            assert_eq!(e["epoch"], epoch);
            assert_eq!(e["public_ticks"], 7160);
            let rows = e["physical_landings"].as_array().unwrap();
            let requests = e["pair_requests"].as_array().unwrap();
            let physical: Vec<_> = rows
                .iter()
                .filter(|x| !x["drive_stroke"].is_null())
                .collect();
            assert_eq!(physical.len(), [16, 11, 13, 16][epoch]);
            let keys: BTreeSet<_> = physical
                .iter()
                .map(|p| (p["tick"].as_u64().unwrap(), p["rig"].as_u64().unwrap()))
                .collect();
            for p in &physical {
                assert!(p["peak_free_gap_m"].as_f64().unwrap() > 0.001);
                assert!(p["normal_impulse_ns"].as_f64().unwrap() > 0.);
                assert_eq!(p["selected"], live);
                let mut norm2 = 0.;
                for i in 0..3 {
                    let n = p["normal_on_foot_ns"][i].as_f64().unwrap();
                    let t = p["tangent_on_foot_ns"][i].as_f64().unwrap();
                    let momentum = p["momentum_on_foot_ns"][i].as_f64().unwrap();
                    let reconstructed = n + t - momentum;
                    assert!(
                        (reconstructed - p["vector_residual_ns"][i].as_f64().unwrap()).abs()
                            < 1e-13
                    );
                    norm2 += reconstructed.powi(2);
                    if i == 2 {
                        residual_z = residual_z.max(reconstructed.abs());
                        tangent_z = tangent_z.max(t.abs());
                    }
                }
                residual_norm = residual_norm.max(norm2.sqrt());
                balance = balance.max(
                    (p["normal_impulse_ns"].as_f64().unwrap()
                        - p["momentum_balance_ns"].as_f64().unwrap())
                    .abs(),
                );
            }
            let observed: Vec<_> = requests
                .iter()
                .filter(|p| p["tick"].as_u64().unwrap() > 1400)
                .collect();
            let request_keys: BTreeSet<_> = observed
                .iter()
                .map(|p| (p["tick"].as_u64().unwrap(), p["rig"].as_u64().unwrap()))
                .collect();
            if live {
                assert_eq!(request_keys, keys);
            } else {
                assert!(request_keys.is_empty());
            }
            assert_eq!(request_keys.len(), observed.len());
            let ticks: BTreeSet<_> = request_keys.iter().map(|k| k.0).collect();
            coalesced += request_keys.len() - ticks.len();
            let tier = e["tier_attempts"].as_array().unwrap();
            holds += tier.iter().filter(|p| p["admitted"] == true).count();
            attempts += tier.len();
            wrench = wrench.max(e["max_witness_velocity_error_m_s"].as_f64().unwrap());
            match epoch {
                1 | 2 => {
                    let boundary = e["edits"][0]["tick"].as_u64().unwrap();
                    let before: Vec<_> = rows
                        .iter()
                        .filter(|p| p["tick"].as_u64().unwrap() < boundary)
                        .collect();
                    let clean: Vec<_> = epochs[0]["physical_landings"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .filter(|p| p["tick"].as_u64().unwrap() < boundary)
                        .collect();
                    assert_eq!(before, clean);
                    assert!(
                        physical
                            .iter()
                            .filter(|p| p["tick"].as_u64().unwrap() >= boundary)
                            .all(|p| p["rig"].as_u64().unwrap() == if epoch == 1 { 1 } else { 0 })
                    );
                }
                3 => {
                    for field in [
                        "native_state_digest",
                        "physical_landings",
                        "tier_attempts",
                        "pair_requests",
                    ] {
                        assert_eq!(e[field], epochs[0][field]);
                    }
                }
                _ => {}
            }
            totals += e["public_ticks"].as_u64().unwrap();
            hit_count += physical.len();
            request_count += request_keys.len();
        }
        report.push(json!({"policy":label,"public_ticks":totals,"post_settle_landings":hit_count,"post_settle_requests":request_count,
        "coalesced_post_settle_requests":coalesced,"admitted_demotions":holds,"attempted_demotions":attempts,
        "max_normal_scalar_minus_vertical_momentum_residual_ns":balance,"max_true_vertical_residual_ns":residual_z,
        "max_vector_momentum_residual_ns":residual_norm,"max_vertical_tangential_contribution_ns":tangent_z,"max_unbound_velocity_error_m_s":wrench}));
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
