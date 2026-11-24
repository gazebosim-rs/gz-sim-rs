// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify true public clocks and reference horizons across actual Bevy work budgets.
#[path = "shared/impact_measurements.rs"]
mod measures;
use glam::DVec3;
use measures::*;
use serde_json::Value as V;
use std::{fs, path::Path};
fn load(p: &Path) -> V {
    serde_json::from_reader(fs::File::open(p).unwrap()).unwrap()
}
fn com(b: &V) -> DVec3 {
    vec(&b["position"]) + quat(&b["orientation"]) * (0.07 * DVec3::Z)
}
fn field(a: &mut Audit, p: &Path, label: &str) {
    for d in arr(&load(p)["donors"]) {
        for c in arr(&d["cases"]) {
            a.count(&format!("{label}_cases"));
            let w = &c["world"];
            let reference = arr(&w["analytic"]);
            let length = reference.len() - 1;
            let horizon = num(&w["free_until_s"]);
            assert!(length <= 8 && length as f64 * 0.0005 <= horizon + 1e-15);
            assert!(horizon > num(&w["initial_certified_s"]) * 0.49);
            if length < 8 {
                assert!(horizon < (length + 1) as f64 * 0.0005);
            }
            if length > 0 {
                a.count(&format!("{label}_one_full_tick"));
            }
            if length == 8 {
                a.count(&format!("{label}_eight_full_ticks"));
            }
            for lanes in arr(&w["runs"]).chunks_exact(2) {
                let n = int(&lanes[0]["n"]);
                let name = format!("{label}_budget_{n}");
                let mut any = false;
                assert_eq!(lanes[0]["n"], lanes[1]["n"]);
                let (control, new) = (arr(&lanes[0]["frames"]), arr(&lanes[1]["frames"]));
                for (index, (b, actual)) in control.iter().zip(new).enumerate() {
                    let dt = (index as u64) * 500000;
                    for f in [actual, b] {
                        for (site, v) in arr(&f["data"]).iter().enumerate() {
                            assert_eq!(int(&v["time_ns"]), dt);
                            assert_eq!(v["normal_impulse_ns"], f["loads"][site]);
                            assert!(v["corner_ns"].is_null());
                        }
                    }
                    let active = int(&actual["closed"]);
                    if index == 1 {
                        if active > 0 {
                            a.count(&format!("{name}_closure"));
                        }
                        if int(&actual["calls"]) > 1 {
                            a.count(&format!("{name}_selected"));
                        }
                    }
                    any |= active > 0;
                    if !any {
                        assert_eq!(actual["data"], b["data"]);
                    }
                    if index == 0 || index > length {
                        continue;
                    }
                    let ex = &reference[index]["data"][0]["foot"];
                    for (k, f) in [("ordinary", b), ("enabled", actual)] {
                        let g = format!("{name}_{k}");
                        let foot = &f["data"][0]["foot"];
                        if index == 1 {
                            for side in 0..2 {
                                let expected = num(&w["pair_reactions"][side]);
                                let error = (num(&f["loads"][side]) - expected).abs();
                                a.sample(&g, "first_reaction_error_ns", error);
                                if active > 0 {
                                    a.sample(
                                        &format!("{g}_admitted"),
                                        "first_reaction_error_ns",
                                        error,
                                    );
                                }
                            }
                            a.sample(&g, "first_com_error_m", com(foot).distance(com(ex)));
                            a.sample(
                                &g,
                                "first_spin_error_rad_s",
                                vec(&foot["angular_velocity"])
                                    .distance(vec(&ex["angular_velocity"])),
                            );
                        } else {
                            for side in 0..2 {
                                a.sample(&g, "later_spurious_pair_ns", num(&f["loads"][side]));
                            }
                        }
                        if index == length {
                            a.sample(&g, "last_com_error_m", com(foot).distance(com(ex)));
                            a.sample(
                                &g,
                                "last_spin_error_rad_s",
                                vec(&foot["angular_velocity"])
                                    .distance(vec(&ex["angular_velocity"])),
                            );
                        }
                    }
                }
            }
        }
    }
}
fn main() {
    let p = std::env::args().nth(1).unwrap();
    let root = Path::new(&p);
    let mut a = Audit::default();
    for (f, g) in [
        ("period.json", "original"),
        ("period_heldout.json", "heldout"),
    ] {
        field(&mut a, &root.join(f), g);
    }
    let out = a.report();
    assert_eq!(out["counts"]["original_cases"], 28);
    assert_eq!(out["counts"]["heldout_cases"], 41);
    fs::create_dir_all(root.join("checks")).unwrap();
    fs::write(
        root.join("checks/period.json"),
        serde_json::to_string_pretty(&out).unwrap(),
    )
    .unwrap();
    println!(
        "Verified 69 physical cuts, six budgets, eight 500 µs updates and reference censoring."
    );
}
