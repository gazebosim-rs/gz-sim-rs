// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check witnessed timing, signed pair loads and their spatial impulse against two independent fields.
#[path = "shared/impact_measurements.rs"]
mod measures;
use glam::{DMat3, DVec3};
use measures::*;
use serde_json::Value as V;
use std::{fs, path::Path};
fn read(p: &Path) -> V {
    serde_json::from_reader(fs::File::open(p).unwrap()).unwrap()
}
fn com(b: &V) -> DVec3 {
    vec(&b["position"]) + quat(&b["orientation"]) * DVec3::new(0., 0., 0.07)
}
fn numerical(a: &mut Audit, root: &Path, name: &str) {
    let doc = read(&root.join(format!("{name}.json")));
    for d in arr(&doc["donors"]) {
        let inertia = DMat3::from_cols_array(&serde_json::from_value(d["tensor"].clone()).unwrap());
        for c in arr(&d["cases"]) {
            a.count(&format!("{name}_cases"));
            a.count(&format!("{name}_{}", c["reference_kind"].as_str().unwrap()));
            let x = &c["source_lower"];
            assert_eq!(x["mu"], 0.);
            let h = num(&c["dt_ns"]) * 1e-9;
            let oracle = &c["oracle"];
            let runs = arr(&c["runs"]);
            assert_eq!(runs.len(), 4);
            for (work, lanes) in runs.chunks_exact(2).enumerate() {
                let (control, actual) = (&lanes[0], &lanes[1]);
                assert_eq!(actual["calls"], control["calls"]);
                assert_eq!(control["enabled"], false);
                assert_eq!(actual["enabled"], true);
                assert!(control["native"].is_null());
                let r = &actual["native"];
                if work == 1 {
                    assert_eq!(r, &runs[1]["native"]);
                }
                let api = if work == 0 { "direct" } else { "bevy" };
                if r.is_null() {
                    assert_eq!(actual["after"], control["after"]);
                    assert_eq!(actual["loads"], control["loads"]);
                    a.count(&format!("{name}_{api}_fallback"));
                    continue;
                }
                a.count(&format!("{name}_{api}_closed"));
                let group = format!("{name}_{api}");
                assert_eq!(actual["calls"], 1);
                assert!(oracle.is_object());
                assert_eq!(r["corners"], oracle["corners"]);
                let (ot, span) = (num(&oracle["offset_s"]), num(&oracle["gap_cap_s"]));
                assert!(
                    num(&r["time_bound_s"][0]) <= ot + span && ot <= num(&r["time_bound_s"][1])
                );
                assert!(num(&r["time_bound_s"][1]) < h);
                assert!(num(&r["other_gap_m"]) > 0. && num(&r["footprint_margin_m"]) > 0.);
                let ns = [vec(&r["normals"][0]), vec(&r["normals"][1])];
                let first = vec(&r["lower_normal"]);
                let mut expected = first;
                let mut angular = vec(&r["lower_lever"]).cross(first);
                let mut summed = [first.dot(ns[0]), 0.];
                let mut prev = num(&r["first_lower"]);
                for p in arr(&r["finite"]) {
                    let site = int(&p["site"]) as usize;
                    let j = num(&p["normal_ns"]);
                    let t = num(&p["at_s"]);
                    assert!(t > prev && t < h && j > 0.);
                    prev = t;
                    angular += vec(&p["lever"]).cross(ns[site] * j);
                    expected += ns[site] * j;
                    summed[site] += j;
                }
                for k in 0..2 {
                    assert!(num(&r["acceleration_floor"][k]) > 0.);
                    let j = num(&r["terminal_ns"][k]);
                    angular += vec(&r["terminal_levers"][k]).cross(ns[k] * j);
                    expected += ns[k] * j;
                    summed[k] += j;
                    let got = num(&actual["loads"][k]);
                    assert!((got - summed[k]).abs() < 1e-12);
                    let err = (got - num(&oracle["loads"][k])).abs();
                    let radius = num(&r["radius"][k]);
                    assert!(
                        radius > 0. && radius <= 1e-4 && err <= radius + num(&oracle["radius"][k])
                    );
                    a.sample(&group, "pair_radius_ns", radius);
                    a.sample(&group, "pair_error_ns", err);
                    a.sample(
                        &group,
                        "baseline_pair_error_ns",
                        (num(&control["loads"][k]) - num(&oracle["loads"][k])).abs(),
                    );
                }
                let got = &actual["after"];
                let baseline = &control["after"];
                let end = &oracle["after"];
                assert_eq!(r["end"], *got);
                let q = quat(&got["orientation"]);
                let p = 2. * (vec(&got["velocity"]) - vec(&x["velocity"]) - vec(&d["gravity"]) * h);
                let dl = q * (inertia * (q.conjugate() * vec(&got["angular_velocity"])))
                    - vec(&x["angular_momentum"]);
                assert!(p.distance(expected) < 1e-10 && dl.distance(angular) < 1e-10);
                a.sample(&group, "linear_closure_ns", p.distance(expected));
                a.sample(&group, "angular_closure_nms", dl.distance(angular));
                for (g, b) in [("", got), ("baseline_", baseline)] {
                    let pos = com(b).distance(com(end));
                    let spin = vec(&b["angular_velocity"]).distance(vec(&end["angular_velocity"]));
                    if g.is_empty() {
                        assert!(
                            pos <= num(&r["com_error_m"])
                                + num(&oracle["bounds"]["com_error_m"])
                                + 1e-13
                                && spin
                                    <= num(&r["spin_error_rad_s"])
                                        + num(&oracle["bounds"]["spin_error_rad_s"])
                        );
                    }
                    a.sample(&group, &format!("{g}com_error_m"), pos);
                    a.sample(&group, &format!("{g}spin_error_rad_s"), spin);
                }
            }
        }
    }
}
fn cost(a: &mut Audit, root: &Path, name: &str) {
    let x = read(&root.join(name).join("timings.json"));
    for d in arr(&x["donors"]) {
        for c in arr(&d["cases"]) {
            let cohort = if c["admitted"] == true {
                "closed"
            } else {
                "fallback"
            };
            for (i, lane) in ["baseline_ns", "projection_ns"].into_iter().enumerate() {
                for ns in arr(&c["wall_ns"][i]) {
                    a.sample(&format!("{name}_{cohort}"), lane, num(ns));
                    a.sample(&format!("{name}_all"), lane, num(ns));
                }
            }
        }
    }
}
fn main() {
    let p = std::env::args().nth(1).unwrap();
    let root = Path::new(&p);
    let mut a = Audit::default();
    for n in ["native", "heldout"] {
        numerical(&mut a, root, n);
    }
    for n in ["cpu6-a", "cpu6-b", "cpu10-a", "cpu10-b"] {
        if root.join(n).join("timings.json").is_file() {
            cost(&mut a, root, n);
        }
    }
    let report = a.report();
    assert_eq!(report["counts"]["native_cases"], 75);
    assert_eq!(report["counts"]["heldout_cases"], 145);
    fs::create_dir_all(root.join("checks")).unwrap();
    fs::write(
        root.join("checks/report.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    println!("{}", report["counts"]);
}
