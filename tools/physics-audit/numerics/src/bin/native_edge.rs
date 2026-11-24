// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct per-site virtual work, Coulomb directions and frame covariance on saved native transactions.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| f(&a[i])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(path.join("native.json"))?)?;
    let mut count = [[0u64; 2]; 2];
    let mut error = 0f64;
    let mut covariance = 0f64;
    let mut balance = [0f64; 5];
    let mut law = 0f64;
    for row in &rows {
        let r = &row["result"];
        let active = r["admitted"].as_bool().unwrap();
        let free = row["free_child"].as_bool().unwrap();
        count[usize::from(free)][usize::from(active)] += 1;
        let base = rows
            .iter()
            .find(|b| {
                b["frame"] == "base"
                    && b["case"] == row["case"]
                    && b["mu"] == row["mu"]
                    && b["yaw_deg"] == row["yaw_deg"]
                    && b["free_child"] == free
            })
            .unwrap();
        assert_eq!(base["result"]["admitted"], active);
        if !active {
            assert_eq!(r["reference_decline"], base["result"]["reference_decline"]);
            continue;
        }
        let rot = DQuat::from_array(std::array::from_fn(|i| f(&r["frozen_body_frame"][i])));
        let mu = f(&row["mu"]);
        let mut loss = 0.;
        let mut ke = 0.;
        for s in r["sites"].as_array().unwrap() {
            let i = s["site"].as_u64().unwrap() as usize;
            let j = v(&s["impulse_ns"]);
            let n = v(&s["world_normal"]);
            let before = v(&s["incoming_m_s"]);
            let after = v(&s["outgoing_m_s"]);
            let jn = j.dot(n);
            let tangent = j - n * jn;
            let slip = after - n * after.dot(n);
            assert!(jn > 0. && before.dot(n) < 0. && after.dot(n).abs() < 1e-12);
            law = law.max((tangent + mu * jn * slip.normalize()).length());
            let expected_loss = -0.5 * j.dot(before + after);
            assert!((expected_loss - f(&s["loss_j"])).abs() < 1e-13);
            loss += expected_loss;
            ke += f(&s["energy_j"]);
            error = error.max(j.distance(v(&r["oracle_sites_ns"][i])));
            let base_site = base["result"]["sites"]
                .as_array()
                .unwrap()
                .iter()
                .find(|p| p["site"] == i as u64)
                .unwrap();
            covariance = covariance.max(j.distance(rot * v(&base_site["impulse_ns"])));
        }
        assert!(
            (loss - f(&r["total_loss_j"])).abs() < 1e-12
                && (ke - f(&r["total_energy_j"])).abs() < 1e-12
        );
        for (i, key) in [
            "linear_residual_ns",
            "angular_residual_nms",
            "native_energy_residual_j",
            "site_work_residual_j",
            "loss_residual_j",
        ]
        .into_iter()
        .enumerate()
        {
            balance[i] = balance[i].max(f(&r[key]));
        }
    }
    assert_eq!(count, [[36, 156], [72, 120]]);
    assert!(
        error < 1e-13 && covariance < 1e-13 && law < 1e-13 && balance.iter().all(|b| *b < 1e-12)
    );
    let read = |s: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(
            path.join(format!("traces/{s}.json")),
        )?)?)
    };
    let native = read("native")?;
    let analytic = read("analytical")?;
    let mut com = 0f64;
    let mut load = 0f64;
    for (a, b) in native["frames"]
        .as_array()
        .unwrap()
        .iter()
        .zip(analytic["frames"].as_array().unwrap())
    {
        let c = |f: &Value| (v(&f["foot"]["position"]) + v(&f["upper"]["position"])) * 0.5;
        com = com.max(c(a).distance(c(b)));
        for k in 0..4 {
            load = load.max((f(&a["corner_ns"][k]) - f(&b["corner_ns"][k])).abs());
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"total":rows.len(),"declined_admitted_locked_free":count,"max_oracle_difference_ns":error,"max_frame_covariance_ns":covariance,"max_coulomb_residual_ns":law,"balance_residuals_p_h_kinetic_work_loss":balance,"max_native_vs_analytic_replay_com_m":com,"max_native_vs_analytic_replay_quadrant_ns":load})
        )?
    );
    Ok(())
}
