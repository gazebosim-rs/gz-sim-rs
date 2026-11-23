// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit incoming states, observed motion, readiness and native friction dispatch.
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("pass phase root")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("study.json"))?)?;
    let cross: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("crossover.json"))?)?;
    assert_eq!(rows.len(), 96);
    assert_eq!(cross.len(), 64);
    let mut groups = 0;
    let mut articulated = 0;
    let mut rigid_equal = 0;
    let mut mixed_equal = 0;
    for data in [&rows, &cross] {
        let mixed = std::ptr::eq(data, &cross);
        for r in data.iter().filter(|r| r["per_point"] == false) {
            let p = data
                .iter()
                .find(|p| {
                    p["per_point"] == true
                        && ["mu", "matched", "placement", "rigid", "policy"]
                            .iter()
                            .all(|k| r[*k] == p[*k])
                })
                .unwrap();
            let mut expected = r.clone();
            expected["per_point"] = true.into();
            if mixed {
                assert_eq!(expected, *p);
                mixed_equal += 1;
            } else if r["rigid"] == false {
                assert_eq!(expected, *p);
                articulated += 1;
            } else {
                rigid_equal += usize::from(expected == *p);
            }
        }
        for r in data
            .iter()
            .filter(|r| r["policy"] == "ordinary" && (!mixed || r["matched"] == true))
        {
            let group: Vec<_> = data
                .iter()
                .filter(|p| {
                    ["mu", "matched", "placement", "rigid", "per_point"]
                        .iter()
                        .all(|k| r[*k] == p[*k])
                })
                .collect();
            assert_eq!(group.len(), 4);
            for p in group {
                if mixed {
                    assert_eq!(p["prefix_tick"], 1472);
                    assert_eq!(p["prefix"], r["prefix"]);
                } else {
                    assert_eq!(p["first_branch_cut"], r["first_branch_cut"]);
                    assert_eq!(p["first_branch_state"], r["first_branch_state"]);
                }
            }
            groups += 1;
        }
    }
    let mut decisions = vec![];
    for matched in [false, true] {
        for p in cross
            .iter()
            .filter(|p| p["mu"].is_null() && p["per_point"] == false && p["matched"] == matched)
        {
            let r = &p["rigs"][0];
            let a = &r["at_attempt"];
            assert_eq!(r["target_tick"], 2120);
            let spin = a["max_link_spin_rad_s"].as_f64().unwrap();
            let rot = r["future_20ms_rotation_path_rad"].as_f64().unwrap();
            if p["policy"] == "ordinary" {
                assert!(spin < 0.001 && a["eligible"] == true);
            }
            if p["policy"] == "relax8" {
                assert!(spin > 0.001 && r["gate_reason"].as_str().unwrap().contains("settled"));
                if matched {
                    assert!(rot > 0.0015);
                }
            }
            decisions.push(json!({"policy":p["policy"],"matched":matched,"angular_speed_rad_s":spin,"future_rotation_path_mrad":rot*1e3,
                "future_max_com_um":r["future_20ms_max_com_move_m"].as_f64().unwrap()*1e6,"eligible":a["eligible"],
                "start_continuously_eligible_tick":r["first_20ms_eligible_after_attempt"]}));
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"cold_trials":rows.len(),"mixed_trials":cross.len(),"exact_start_groups":groups,
        "identical_cold_articulation_friction_model_pairs":articulated,"identical_cold_rigid_friction_model_pairs":rigid_equal,
        "identical_mixed_friction_model_pairs":mixed_equal,"default_material_centered_decisions":decisions})
        )?
    );
    Ok(())
}
