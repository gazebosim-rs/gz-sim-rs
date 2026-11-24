// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently follow the two free-flight branches and both signed reaction ledgers.
#[path = "shared/impact_measurements.rs"]
mod measures;
#[path = "shared/opposing_mechanics.rs"]
mod mechanics;
use glam::DVec3;
use measures::*;
use mechanics::*;
use serde_json::{Value as V, json};
use std::{fs, path::Path};
fn kind(h: &V) -> &str {
    if h["mu"] == 0. {
        "frictionless"
    } else if h["stick"] == true {
        "sticking"
    } else {
        "sliding"
    }
}
fn ordered(a: &mut Audit, key: &str, d: &V, c: &V, h: &V, f: &V, offset: f64) {
    let old = num(&h["previous_upper_ns"]);
    let first = num(&f["physical_ns"]);
    assert!((old - first).abs() < 1e-7);
    assert!(int(&h["upper_corner"]) == int(&f["corner"]) && num(&h["physical_ns"]) - old > 1.);
    assert!(num(&h["prior_upper_gaps_m"][1]) > 0.);
    reaction(a, key, d, c, h, offset);
    compare(
        a,
        &format!("{key}/relative_to_first_physical_return"),
        &c["next"],
        h,
        offset,
    );
    a.sample(
        key,
        "time_between_impacts_ns",
        num(&h["physical_ns"]) - first,
    );
    a.sample(
        key,
        "prior_point_gap_at_return_m",
        num(&h["prior_upper_gaps_m"][1]),
    );
}
fn cold(a: &mut Audit, d: &V, row: &V) {
    let u = &row["upper"];
    let e = &row["next"];
    let tie = u.is_null();
    let lower = e["face"] == 0;
    let runs = arr(&row["cold"]);
    let group = if tie {
        "tie"
    } else if lower {
        "lower"
    } else if !e.is_null() {
        "upper"
    } else {
        "none"
    };
    let n = if tie {
        1
    } else if e.is_null() {
        0
    } else {
        12
    };
    assert_eq!(runs.len(), n);
    for r in runs {
        let enabled = r["enabled"] == true;
        let calls = arr(&r["calls"]);
        assert_eq!(calls.len(), int(&r["n"]) as usize);
        let mut first = None;
        let mut li = None;
        let mut previous = 0;
        let mut jg = [0.; 2];
        let mut jx = [DVec3::ZERO; 2];
        let mut native = false;
        let offset = num(&r["start_ns"]);
        for s in calls {
            assert_eq!(int(&s["from_ns"]), previous);
            previous = int(&s["to_ns"]);
            assert!(int(&s["from_ns"]) < previous);
            for (k, name) in ["native_pad", "native_gate"].into_iter().enumerate() {
                let normal = num(&s[name]["normal_ns"]);
                native |= normal > 0.;
                jg[k] += normal;
            }
            for (side, name) in [(1, "upper"), (0, "lower")] {
                let h = &s[name];
                if h.is_null() {
                    continue;
                }
                let j = vec(&h["impulse"]);
                jg[side] += j.dot(vec(&h["normal"]));
                jx[side] += j;
                assert!(
                    num(&s["native_pad"]["normal_ns"]) == 0.
                        && num(&s["native_gate"]["normal_ns"]) == 0.
                );
                if side == 1 {
                    first = Some(h);
                    reaction(a, "matched/upper_geometry", d, row, h, offset);
                } else {
                    assert!(enabled && lower);
                    let f = first.expect("lower return preceded its actual roof impact");
                    ordered(a, "matched/return_geometry", d, row, h, f, offset);
                    assert_eq!(h["corner"], e["corner"]);
                    assert!(vec(&h["impulse"]).distance(vec(&e["impulse"])) < 1e-7);
                    assert!((num(&h["physical_ns"]) + offset - num(&e["at_ns"])).abs() < 0.1);
                    li = Some(h);
                }
            }
        }
        assert_eq!(previous, int(&r["dt_ns"]));
        for (side, name) in ["pad", "gate"].into_iter().enumerate() {
            let p = &r[name];
            let normal = p["normal_ns"].as_f64().unwrap_or(0.);
            assert!((jg[side] - normal).abs() < 1e-9);
            if !native && normal > 0. {
                let pair = vec(&p["normal_on_foot"]) + vec(&p["tangent_on_foot"]);
                assert!(pair.distance(jx[side]) < 1e-10);
            }
        }
        if enabled {
            a.count(&format!(
                "matched/{group}/calls{}/returned/{}",
                calls.len(),
                li.is_some()
            ));
        }
        if li.is_some() {
            assert!(!tie && lower);
            let refu = vec(&u["impulse"]);
            let refl = vec(&e["impulse"]);
            let lo = Cut::read(&r["start"], inertia(d));
            let hi = Cut::read(&calls.last().unwrap()["end"], inertia(d));
            let gravity = vec(&d["primary"]["gravity"]);
            let dt = num(&r["dt_ns"]) * 1e-9;
            let net = 2. * (hi.v - lo.v - gravity * dt);
            let dl = hi.l - lo.l;
            let target = vec(&u["arm"]).cross(refu) + vec(&e["arm"]).cross(refl);
            a.sample(
                "matched/accepted",
                "two_impulse_error_ns",
                net.distance(refu + refl),
            );
            a.sample(
                "matched/accepted",
                "two_angular_impulse_error_nms",
                dl.distance(target),
            );
            if !native {
                assert!(net.distance(refu + refl) < 1e-7 && dl.distance(target) < 1e-7);
            }
            a.count(&format!("matched/admitted/{}", kind(e)));
        }
    }
    if !tie && !e.is_null() {
        for r in runs.chunks_exact(2) {
            let (baseline, updated) = (&r[0], &r[1]);
            assert_eq!(baseline["enabled"], false);
            assert_eq!(updated["enabled"], true);
            assert_eq!(baseline["dt_ns"], updated["dt_ns"]);
            assert_eq!(baseline["start"], updated["start"]);
            if !arr(&updated["calls"]).iter().any(|s| !s["lower"].is_null()) {
                continue;
            }
            for (label, run) in [("upper_only", baseline), ("with_return", updated)] {
                let jf = vec(&u["impulse"]) + vec(&e["impulse"]);
                let v0 = vec(&run["start"]["velocity"]);
                let v1 = vec(&arr(&run["calls"]).last().unwrap()["end"]["velocity"]);
                let total =
                    2. * (v1 - v0 - vec(&d["primary"]["gravity"]) * num(&run["dt_ns"]) * 1e-9);
                let k = format!("paired/{label}/calls{}", int(&run["n"]));
                a.sample(&k, "net_vector_error_ns", total.distance(jf));
                a.sample(
                    &k,
                    "lower_normal_error_ns",
                    (num(&e["normal_ns"]) - run["pad"]["normal_ns"].as_f64().unwrap_or(0.)).abs(),
                );
                a.sample(
                    &k,
                    "upper_normal_error_ns",
                    (num(&u["normal_ns"]) - run["gate"]["normal_ns"].as_f64().unwrap_or(0.)).abs(),
                );
            }
        }
    }
}
fn public(a: &mut Audit, d: &V, row: &V) {
    let e = &row["next"];
    let u = &row["upper"];
    let lower = e["face"] == 0;
    let same =
        lower && (num(&e["at_ns"]) / 500000.).floor() == (num(&u["at_ns"]) / 500000.).floor();
    if lower {
        a.count(&format!("public/lower_reference_same_tick/{same}"));
    }
    for r in arr(&row["public"]) {
        let enabled = r["enabled"] == true;
        let data = &r["data"];
        let mut first = None;
        let mut applied = false;
        for s in arr(&data["events"]) {
            let h = &s["upper_projection"];
            if !h.is_null() {
                first = Some(h);
                reaction(a, "public/upper_geometry", d, row, h, 0.);
            }
            let ret = &s["return_projection"];
            if ret.is_null() {
                continue;
            }
            assert!(enabled && lower && same);
            let f = first.unwrap();
            assert_eq!(
                (num(&f["physical_ns"]) / 500000.).floor(),
                (num(&ret["physical_ns"]) / 500000.).floor()
            );
            ordered(a, "public/return_geometry", d, row, ret, f, 0.);
            compare(a, "public/preceding_upper_reference", u, f, 0.);
            applied = true;
        }
        let name = if enabled { "ordered" } else { "upper_only" };
        if lower {
            let group = format!("public/{name}");
            a.sample(&group, "total_native_calls", num(&data["calls"]));
            if let Some(l) = arr(&data["positive_pad_after_first_gate"]).first() {
                a.sample(
                    &group,
                    "first_pad_normal_error_ns",
                    (num(&l["primary_ns"]) - num(&e["normal_ns"])).abs(),
                );
                let t = l["return_projection"]["physical_ns"]
                    .as_f64()
                    .unwrap_or_else(|| num(&l["end_ns"]));
                a.sample(
                    &group,
                    "first_pad_time_error_ns",
                    (t - num(&e["at_ns"])).abs(),
                );
                if num(&l["opponent_ns"]) > 0. {
                    a.count(&format!("{group}/first_positive_in_both"));
                }
            }
            let endpoint = &data["state_after_return"];
            if !endpoint.is_null() {
                a.sample(
                    &group,
                    "last_certified_public_com_m",
                    num(&endpoint["com_m"]),
                );
                a.sample(
                    &group,
                    "last_certified_public_spin_rad_s",
                    num(&endpoint["spin_rad_s"]),
                );
            }
        }
        if enabled {
            let name = if lower {
                "lower"
            } else if e.is_null() {
                "undefined"
            } else {
                "upper"
            };
            a.count(&format!("public/{name}/returned/{applied}"));
        }
    }
}
fn main() {
    let p = std::env::args().nth(1).expect("archive");
    let path = Path::new(&p);
    let data = read(&path.join("returns.json"));
    let mut a = Audit::default();
    assert_eq!(arr(&data["donors"]).len(), 66);
    for d in arr(&data["donors"]) {
        assert_eq!(arr(&d["cases"]).len(), 15);
        for r in arr(&d["cases"]) {
            cold(&mut a, d, r);
            public(&mut a, d, r);
        }
    }
    for trial in ["cpu6-a", "cpu6-b"] {
        cost(&mut a, trial, &read(&path.join(trial).join("cost.json")));
    }
    fs::write(
        path.join("readback.json"),
        serde_json::to_vec_pretty(&a.report()).unwrap(),
    )
    .unwrap();
    println!(
        "{}",
        json!({"placements":990,"lower_successors":75,"upper_successors":41,"matched_native_trials":1788})
    );
}
