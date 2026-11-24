// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent tolerance, endpoint, cohort and CPU accounting for early opposing closure.
#[path = "shared/impact_measurements.rs"]
mod measures;
use measures::*;
use serde_json::{Value as V, json};
use std::{fs, path::Path};
fn load(p: &Path) -> V {
    serde_json::from_reader(fs::File::open(p).unwrap()).unwrap()
}
fn cases(v: &V) -> impl Iterator<Item = (&V, &V)> {
    arr(&v["donors"])
        .iter()
        .flat_map(|d| arr(&d["cases"]).iter().map(move |c| (d, c)))
}
fn cohort(a: &mut Audit, d: &V, group: &str) {
    for (donor, c) in cases(d) {
        a.count(&format!("{group}_cases"));
        if group == "heldout" {
            assert!(int(&donor["id"]) >= 2600);
        }
        let physical = c["reference_kind"].as_str().unwrap();
        a.count(&format!("{group}_{physical}"));
        let r = &c["result"]["reference"];
        let full = int(&c["full_events"]);
        for (k, h) in arr(&c["result"]["goals"]).iter().enumerate() {
            let g = format!("{group}_{k}");
            let Some(x) = h["candidate"].as_object() else {
                a.count(&format!("{g}_declined"));
                continue;
            };
            assert_eq!(physical, "accumulation_then_free_release");
            assert!((quat(&x["q"]).length() - 1.).abs() < 1e-12);
            a.count(&format!("{g}_accepted"));
            let y = &h["compare"];
            assert!(y["in_both_free_windows"] == true);
            let ep = num(&y["endpoint_s"]);
            let limit =
                num(&x["offset_s"]) + num(&x["remaining_time_s"]) + num(&x["propagated"]["time_s"]);
            assert!(
                ep >= limit
                    && ep
                        <= num(&x["offset_s"]) - num(&x["propagated"]["time_s"])
                            + num(&x["free_after_s"])
            );
            let prior = int(&x["prior_strikes"]);
            assert!(prior <= full);
            a.sample(&g, "finite_events", prior as f64);
            a.sample(&g, "saved_events", (full - prior) as f64);
            a.sample(&g, "closure_time_cap_ns", num(&x["remaining_time_s"]) * 1e9);
            a.sample(&g, "prior_offset_ns", num(&x["offset_s"]) * 1e9);
            a.sample(&g, "time_error_ns", num(&x["propagated"]["time_s"]) * 1e9);
            if int(&x["tracked_impulses"]) > 0 {
                a.count(&format!("{g}_tightened"));
            }
            for j in 0..2 {
                let delta = vec(&x["sum"][j]).distance(vec(&r["sum"][j]));
                let bounds = num(&x["radius"][j]) + num(&r["radius"][j]);
                assert!(
                    num(&x["radius"][j]) <= num(&h["radius_goal"])
                        && delta <= bounds
                        && (delta - num(&y["impulse_difference_ns"][j])).abs() < 1e-14
                        && (bounds - num(&y["impulse_combined_radii_ns"][j])).abs() < 1e-14
                );
                a.sample(&g, "pair_radius_ns", num(&x["radius"][j]));
                a.sample(&g, "pair_difference_ns", delta);
                assert!(num(&x["minimum_outward"][j]) > 0.);
            }
            for (key, bound) in [
                ("com_difference_m", "com_bound_m"),
                ("spin_difference_rad_s", "spin_bound_rad_s"),
                ("velocity_difference_m_s", "velocity_bound_m_s"),
                ("momentum_difference_nms", "momentum_bound_nms"),
            ] {
                assert!(
                    num(&y[key]) <= num(&y[bound]),
                    "{group} {} {k} {key} {} > {}",
                    donor["id"],
                    y[key],
                    y[bound]
                );
                a.sample(&g, key, num(&y[key]));
            }
        }
    }
}
fn cost(a: &mut Audit, p: &Path, group: &str, ref_data: &V) {
    for ((rd, rc), (d, c)) in cases(ref_data).zip(cases(&load(p))) {
        assert!(
            rd["id"] == d["id"] && rc["delay_ns"] == c["delay_ns"] && rc["gate_mu"] == c["gate_mu"]
        );
        let accepted = rc["result"]["goals"][1]["candidate"].is_object();
        let full = arr(&c["result"]["full_ns"]);
        let early = arr(&c["result"]["early_ns"]);
        assert!(full.len() == 8 && early.len() == 8);
        for (x, y) in full.iter().zip(early) {
            a.sample(group, "full_us", num(x) * 1e-3);
            a.sample(group, "early_us", num(y) * 1e-3);
            a.sample(
                &format!("{group}_{}", if accepted { "accepted" } else { "declined" }),
                "early_us",
                num(y) * 1e-3,
            );
        }
    }
}
fn main() {
    let arg = std::env::args().nth(1).unwrap();
    let p = Path::new(&arg);
    let original = load(&p.join("compact.json"));
    let held = load(&p.join("heldout.json"));
    assert_eq!(int(&held["primary_inputs"]), 96);
    assert_eq!(int(&held["attempts"]), 1152);
    assert_eq!(cases(&original).count(), 75);
    assert_eq!(cases(&held).count(), 145);
    let mut a = Audit::default();
    cohort(&mut a, &original, "original");
    cohort(&mut a, &held, "heldout");
    cost(&mut a, &p.join("cpu6-a/cost.json"), "initial", &original);
    for name in ["cpu6-ui-a", "cpu6-ui-b", "cpu10-a", "cpu10-b"] {
        if p.join(name).join("cost.json").is_file() {
            cost(&mut a, &p.join(name).join("cost.json"), name, &original);
        }
    }
    let out = a.report();
    fs::write(
        p.join("readback.json"),
        serde_json::to_vec_pretty(&json!({"accepted_and_declined":220,"audit":out})).unwrap(),
    )
    .unwrap();
    println!(
        "Checked 220 ordered lower cuts at four error/flight budgets and all paired timing samples."
    );
}
