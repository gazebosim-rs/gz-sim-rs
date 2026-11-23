// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Held-out arrival ordering, work and momentum checks independent of the engine.
#[path = "shared/impact_measurements.rs"]
mod measures;
use glam::{DMat3, DVec3};
use measures::*;
use serde_json::Value as V;
use std::{fs, path::Path};
fn read(p: &Path) -> V {
    serde_json::from_reader(fs::File::open(p).unwrap()).unwrap()
}
fn center(b: &V) -> DVec3 {
    vec(&b["position"]) + quat(&b["orientation"]) * (0.07 * DVec3::Z)
}
fn quantities(b: &V, i: DMat3) -> (DVec3, DVec3, f64) {
    let v = vec(&b["velocity"]);
    let w = vec(&b["angular_velocity"]);
    let q = quat(&b["orientation"]);
    let l = q * (i * (q.conjugate() * w));
    (v, l, v.length_squared() + 0.5 * w.dot(l))
}
fn physics(a: &mut Audit, g: &str, d: &V, c: &V, t: &V) {
    let oracle = &t["oracle"];
    let actual = &t["runs"][1];
    let baseline = &t["runs"][0];
    let r = &actual["native"];
    assert!(oracle.is_object() && t["order"] == "lower_first" && actual["calls"] == 1);
    let t0 = num(&oracle["floor_at_s"]);
    let limit = num(&oracle["limit_after_s"]);
    let dt = num(&t["dt_ns"]) * 1e-9;
    let lower_error = (num(&r["first_lower"]) - t0).abs();
    assert!(lower_error <= num(&r["lower_time_error_s"]));
    assert!(
        num(&r["time_bound_s"][0]) <= limit
            && limit <= num(&r["time_bound_s"][1])
            && num(&r["time_bound_s"][1]) < dt
    );
    a.sample(g, "floor_time_error_ns", lower_error * 1e9);
    a.sample(
        g,
        "limit_interval_ns",
        (num(&r["time_bound_s"][1]) - num(&r["time_bound_s"][0])) * 1e9,
    );
    let n = [vec(&r["normals"][0]), vec(&r["normals"][1])];
    let supports = [vec(&d["pad_velocity"]), vec(&c["roof_velocity"])];
    let tensor = DMat3::from_cols_array(&serde_json::from_value(d["tensor"].clone()).unwrap());
    let initial = &t["incoming"];
    let end = &actual["after"];
    let (v0, l0, k0) = quantities(initial, tensor);
    let (v1, l1, k1) = quantities(end, tensor);
    let mut linear = DVec3::ZERO;
    let first = vec(&r["lower_normal"]);
    let mut angular = vec(&r["lower_lever"]).cross(first);
    let mut loss = num(&r["first_lower_loss_j"]);
    let mut sums = [first.dot(n[0]), 0.];
    let mut last_site = 0;
    let mut last = num(&r["first_lower"]);
    for h in arr(&r["finite"]) {
        let k = int(&h["site"]) as usize;
        let j = num(&h["normal_ns"]);
        let e = num(&h["dissipated_j"]);
        assert!(k == 1 - last_site && j > 0. && e >= 0. && num(&h["at_s"]) > last);
        last = num(&h["at_s"]);
        last_site = k;
        sums[k] += j;
        angular += vec(&h["lever"]).cross(n[k] * j);
        loss += e;
    }
    let energy = &r["energy"];
    let radius = num(&energy["radius_j"]);
    let dissipated = num(&energy["dissipated_j"]);
    let mut work = 0.;
    for k in 0..2 {
        let integrated = num(&actual["loads"][k]);
        let tail = num(&r["terminal_ns"][k]);
        let bound = num(&r["radius"][k]);
        let j = n[k] * integrated;
        linear += j;
        angular += vec(&r["terminal_levers"][k]).cross(n[k] * tail);
        assert!((sums[k] + tail - integrated).abs() < 1e-12);
        let w = j.dot(supports[k]);
        assert!((w - num(&energy["support_work_j"][k])).abs() < 1e-11);
        work += w;
        let reference = num(&oracle["loads"][k]);
        let delta = (integrated - reference).abs();
        assert!(delta <= bound + num(&oracle["radii"][k]) && bound <= 1e-4);
        a.sample(g, "pair_error_ns", delta);
        a.sample(
            g,
            "baseline_pair_error_ns",
            (num(&baseline["loads"][k]) - reference).abs(),
        );
        a.sample(g, "pair_radius_ns", bound);
    }
    let gravity = vec(&d["gravity"]);
    let grav = 2. * gravity.dot(center(end) - center(initial));
    let linear_err = linear.distance(2. * (v1 - v0 - gravity * dt));
    let angular_err = angular.distance(l1 - l0);
    assert!(
        linear_err < 1e-10 && angular_err < 1e-10,
        "{} {} {} {:?} {linear_err} {angular_err}",
        d["id"],
        c["gate_mu"],
        t["start_to_lower_s"],
        t["dt_ns"]
    );
    assert!(
        (grav - num(&energy["gravity_work_j"])).abs() < 1e-10
            && (k1 - k0 - num(&energy["kinetic_change_j"])).abs() < 1e-10
    );
    assert!((work + grav - (k1 - k0) - dissipated).abs() < 1e-9);
    assert!(dissipated + radius + 1e-10 >= loss && radius.is_finite() && radius > 0.);
    let independent_loss = num(&oracle["loss_j"]);
    let oracle_budget = 1e-8
        + num(&oracle["bounds"]["momentum_error_nms"]) * vec(&end["angular_velocity"]).length()
        + 2. * num(&oracle["bounds"]["linear_speed_error_m_s"]) * v1.length();
    let de = (dissipated - independent_loss).abs();
    assert!(de <= radius + oracle_budget);
    a.sample(g, "dissipation_error_j", de);
    a.sample(g, "energy_radius_j", radius);
    a.sample(g, "contact_loss_j", dissipated);
    a.sample(g, "linear_closure_ns", linear_err);
    a.sample(g, "angular_closure_nms", angular_err);
    for (name, b) in [("", end), ("baseline_", &baseline["after"])] {
        let ex = &oracle["after"];
        let pos = center(b).distance(center(ex));
        let spin = vec(&b["angular_velocity"]).distance(vec(&ex["angular_velocity"]));
        if name.is_empty() {
            assert!(
                pos <= num(&r["com_error_m"]) + num(&oracle["bounds"]["com_error_m"]) + 1e-13
                    && spin
                        <= num(&r["spin_error_rad_s"]) + num(&oracle["bounds"]["spin_error_rad_s"])
            );
        }
        a.sample(g, &format!("{name}com_error_m"), pos);
        a.sample(g, &format!("{name}spin_error_rad_s"), spin);
    }
}
fn field(a: &mut Audit, root: &Path, name: &str) {
    let filename = format!("{name}.json");
    let current = read(&root.join(&filename));
    let old = read(&root.join("baseline").join(&filename));
    assert_eq!(arr(&current["donors"]).len(), arr(&old["donors"]).len());
    for (d, donor) in arr(&current["donors"]).iter().zip(arr(&old["donors"])) {
        assert_eq!(d["id"], donor["id"]);
        assert_eq!(d["mode"], donor["mode"]);
        assert_eq!(arr(&d["cases"]).len(), arr(&donor["cases"]).len());
        for (c, old) in arr(&d["cases"]).iter().zip(arr(&donor["cases"])) {
            assert_eq!(c["gate_mu"], old["gate_mu"]);
            assert_eq!(c["delay_ns"], old["delay_ns"]);
            assert_eq!(arr(&c["trials"]).len(), arr(&old["trials"]).len());
            for (t, b) in arr(&c["trials"]).iter().zip(arr(&old["trials"])) {
                let here = &t["runs"];
                let prior = &b["runs"];
                assert_eq!(t["incoming"], b["incoming"]);
                assert_eq!(t["dt_ns"], b["dt_ns"]);
                let matched = if t["matched_previous_clock"].is_array() {
                    a.count(&format!("{name}_corrected_clock"));
                    &t["matched_previous_clock"]
                } else {
                    here
                };
                for k in ["after", "calls", "loads"] {
                    assert_eq!(matched[0][k], prior[0][k]);
                }
                assert_eq!(here[1]["elapsed_ns"], t["dt_ns"]);
                if matched[1]["native"].is_object() {
                    a.count(&format!("{name}_matched_old_clock_closed"));
                }
                a.count(&format!("{name}_all"));
                a.count(&format!("{name}_{}", t["order"].as_str().unwrap()));
                let good = here[1]["native"].is_object();
                let was = prior[1]["native"].is_object();
                let group = if t["order"] == "prior_roof" {
                    "before_roof"
                } else if t["oracle"].is_object() {
                    "known_series"
                } else {
                    "free_or_unresolved"
                };
                a.count(&format!("{name}_{group}"));
                if was {
                    a.count(&format!("{name}_previously_closed"));
                }
                if good {
                    a.count(&format!("{name}_closed"));
                    if !was {
                        a.count(&format!("{name}_newly_closed"));
                    }
                    physics(a, name, d, c, t);
                } else {
                    if was {
                        a.count(&format!("{name}_widened_refusal"));
                    }
                    for key in ["after", "loads", "calls"] {
                        assert_eq!(here[1][key], here[0][key]);
                    }
                    if let Some(f) = here[1]["failures"].as_array().and_then(|a| a.last()) {
                        let k = f["stage"].as_str().unwrap();
                        a.count(&format!("{name}_{group}_{k}"));
                    }
                }
            }
        }
    }
}
fn timing(a: &mut Audit, root: &Path, cohort: &str) {
    let measured = read(&root.join(cohort).join("arrival_cost.json"));
    for d in arr(&measured["donors"]) {
        for c in arr(&d["cases"]) {
            let w = &c["world"];
            let tag = if w["accepted"] == true {
                "accepted"
            } else {
                "fallback"
            };
            for (k, n) in ["baseline", "projected"].into_iter().enumerate() {
                for series in arr(&w["each_update_ns"][k]) {
                    let t = arr(series);
                    assert_eq!(t.len(), 8);
                    for group in [format!("{cohort}_all"), format!("{cohort}_{tag}")] {
                        a.sample(&group, &format!("{n}_landing_us"), num(&t[0]) * 1e-3);
                        a.sample(&group, &format!("{n}_next_us"), num(&t[1]) * 1e-3);
                        a.sample(
                            &group,
                            &format!("{n}_full_8_us"),
                            t.iter().map(num).sum::<f64>() * 1e-3,
                        );
                        a.sample(
                            &group,
                            &format!("{n}_peak_us"),
                            t.iter().map(num).fold(0., f64::max) * 1e-3,
                        );
                    }
                }
                for j in arr(&w["late_loads"][k]) {
                    a.sample(
                        &format!("{cohort}_{tag}"),
                        &format!("{n}_later_pair_ns"),
                        num(j),
                    );
                }
            }
        }
    }
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let root = Path::new(&path);
    let mut a = Audit::default();
    for s in ["phase", "phase_heldout"] {
        field(&mut a, root, s);
    }
    for s in ["cpu10-a", "cpu10-b"] {
        timing(&mut a, root, s);
    }
    let v = a.report();
    assert_eq!(v["counts"]["phase_all"], 670);
    assert_eq!(v["counts"]["phase_heldout_all"], 1274);
    fs::create_dir_all(root.join("checks")).unwrap();
    fs::write(
        root.join("checks/arrival.json"),
        serde_json::to_string_pretty(&v).unwrap(),
    )
    .unwrap();
    println!(
        "Verified 1944 actual matched SDF starts, controls, momenta, work and all CPU repeats."
    );
}
