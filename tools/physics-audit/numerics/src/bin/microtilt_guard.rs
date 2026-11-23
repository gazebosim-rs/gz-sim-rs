// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate registered flight exclusions against original point impulses and full native calls.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|k| f(&a[k])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("pass study directory")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("first.json"))?)?;
    let public: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("public.json"))?)?;
    let mut counts = [0u64; 4];
    let mut protected = [0u64; 4];
    let mut worst = [0f64; 4];
    let mut wrong = [0f64; 4];
    let mut prefix_error = 0f64;
    let mut balance = 0f64;
    let mut plot_data = Vec::new();
    for (i, row) in rows.iter().enumerate() {
        let o = &row["physical"];
        let goal = v(&o["impulse_ns"]);
        assert!(f(&o["other_corner_clearance_bound_m"]) > 0.85e-6);
        let corner = o["first_corner"].as_u64().unwrap() as usize;
        for obs in row["samples"].as_array().unwrap() {
            let fixed = obs["single_rigid"].as_bool().unwrap();
            let active = obs["guarded_policy"].as_bool().unwrap();
            let idx = usize::from(fixed) * 2 + usize::from(active);
            let full = obs["full"].as_array().unwrap();
            let prefix = obs["prefix"].as_array().unwrap();
            assert_eq!(full.len(), 8);
            assert_eq!(obs["prefix_guard_differences"], 0);
            assert!(f(&obs["start_error"]) < 1e-12);
            let mut t = 0;
            let mut end = 0;
            for (k, sub) in full.iter().enumerate() {
                let h = sub["duration_ns"].as_u64().unwrap();
                t += h;
                let flag = sub["guarded"].as_bool().unwrap();
                if flag {
                    assert!(fixed && active && v(&sub["impulse_ns"]) == DVec3::ZERO);
                    protected[idx] += 1;
                }
                if let Some(p) = prefix.get(k) {
                    let hp = p["duration_ns"].as_u64().unwrap();
                    end += hp;
                    assert_eq!(p["end_ns"], end);
                    assert!(hp <= h);
                    if k + 1 < prefix.len() || hp == h {
                        assert_eq!(p["guarded"], flag);
                    }
                    if p["guarded"] == true {
                        assert_eq!(p["normal_ns"], 0.);
                    }
                }
            }
            assert_eq!(t, 500000);
            assert_eq!(o["common_cut_ns"], end);
            prefix_error = prefix_error.max(f(&obs["prefix_state_and_impulse_difference"]));
            let j = v(&obs["impulse_ns"]);
            let err = j.distance(goal);
            let j_other = obs["quadrants_ns"]
                .as_array()
                .unwrap()
                .iter()
                .enumerate()
                .filter(|(k, _)| *k != corner)
                .map(|(_, p)| f(p))
                .sum::<f64>();
            assert!(
                (err - f(&obs["impulse_error_ns"])).abs() < 1e-12
                    && (j_other - f(&obs["wrong_corner_ns"])).abs() < 1e-12
            );
            balance = balance.max(v(&obs["moment_nms"]).distance(v(&obs["delta_h_nms"])));
            worst[idx] = worst[idx].max(err);
            wrong[idx] = wrong[idx].max(j_other);
            counts[idx] += 1;
            if fixed {
                plot_data.push((i as f64, active, err));
            }
        }
    }
    assert_eq!(counts, [18; 4]);
    assert!(wrong[2] > 0.70 && wrong[3] == 0. && worst[3] < 1.75e-5);
    assert!(prefix_error < 1e-12 && balance < 1e-12);
    let mut tick_guards = 0;
    let mut late_spins = [vec![], vec![], vec![]];
    for row in &public {
        let r = &row["result"];
        let policy = r["policy"].as_str().unwrap();
        let fixed = r["single_rigid"].as_bool().unwrap();
        assert!(f(&r["cold_start_difference"]) < 1e-12);
        let calls = r["events"].as_array().unwrap();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0]["tick"], 1531);
        if policy == "guarded" {
            for g in r["guarded_native"].as_array().unwrap() {
                let flags = g["guarded"].as_array().unwrap();
                let positives = flags.iter().filter(|x| x.as_bool() == Some(true)).count();
                assert!(fixed || positives == 0);
                if positives == flags.len() {
                    assert_eq!(g["normal_ns"], 0.);
                }
                tick_guards += positives;
            }
            if !fixed {
                let twin = public
                    .iter()
                    .map(|row2| (row2, &row2["result"]))
                    .find(|(row2, v)| {
                        row2["mu"] == row["mu"]
                            && row2["case"] == row["case"]
                            && v["single_rigid"] == false
                            && v["policy"] == "prospective"
                    })
                    .unwrap()
                    .1;
                assert_eq!(twin["states"], r["states"]);
            }
        }
        if fixed {
            let lane = match policy {
                "relax8" => 0,
                "prospective" => 1,
                "guarded" => 2,
                _ => unreachable!(),
            };
            let gate = r["states"]
                .as_array()
                .unwrap()
                .iter()
                .find(|s| s["tick"] == 2120)
                .unwrap();
            late_spins[lane].push(v(&gate["state"]["spin_rad_s"]).length());
        }
    }
    assert_eq!(public.len(), 108);
    let max_spins = late_spins
        .each_ref()
        .map(|s| s.iter().copied().fold(0., f64::max));
    let path = dir.join("guarded-impulse.svg");
    let area = SVGBackend::new(&path, (1080, 480)).into_drawing_area();
    area.fill(&WHITE)?;
    let mut plot = ChartBuilder::on(&area)
        .caption(
            "Standalone rigid foot at the verified single-corner cutoff",
            ("sans-serif", 23),
        )
        .margin(18)
        .x_label_area_size(52)
        .y_label_area_size(74)
        .build_cartesian_2d(-0.7..17.7, (1e-7f64..2.).log_scale())?;
    plot.configure_mesh()
        .x_desc("Matched friction, reflection and arrival cases")
        .y_desc("Vector impulse error (N·s)")
        .light_line_style(RGBColor(230, 232, 234))
        .draw()?;
    for (flag, name, color) in [
        (false, "Timed eight", RGBColor(180, 85, 42)),
        (true, "Timed eight with flight guard", RGBColor(37, 125, 91)),
    ] {
        plot.draw_series(
            plot_data
                .iter()
                .filter(|(_, f, _)| *f == flag)
                .map(|(x, _, y)| Circle::new((*x, *y), 4, color.filled())),
        )?
        .label(name)
        .legend(move |(x, y)| Circle::new((x + 10, y), 4, color.filled()));
    }
    plot.configure_series_labels()
        .border_style(BLACK)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    area.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"matched_controls":counts,"public_returns":public.len(),"order":["locked_timed","locked_guarded","rigid_timed","rigid_guarded"],"guarded_full_native_calls":protected,"public_guarded_native_calls":tick_guards,"max_prefix_state_and_impulse_error":prefix_error,"max_angular_balance_nms":balance,"max_wrong_corner_ns":wrong,"max_impulse_error_ns":worst,"max_gate_spin_rad_s_uniform_timed_guarded":max_spins})
        )?
    );
    Ok(())
}
