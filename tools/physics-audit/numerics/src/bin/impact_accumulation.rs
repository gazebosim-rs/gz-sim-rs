// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read the independently bounded tail and measure complete native pair reactions.
#[path = "shared/impact_measurements.rs"]
mod measures;
use glam::{DMat3, DQuat, DVec3};
use measures::*;
use serde_json::{Value as V, json};
use std::{fs, path::Path};
fn load(p: &Path, n: &str) -> V {
    serde_json::from_reader(fs::File::open(p.join(n)).unwrap()).unwrap()
}
fn tensor(d: &V) -> DMat3 {
    DMat3::from_cols_array(&serde_json::from_value(d["tensor"].clone()).unwrap())
}
fn world(q: DQuat, i: DMat3, x: DVec3) -> DVec3 {
    q * (i * (q.conjugate() * x))
}
fn corner(k: usize) -> DVec3 {
    DVec3::new(
        if k & 2 == 0 { -0.18 } else { 0.18 },
        if k & 1 == 0 { -0.10 } else { 0.10 },
        if k & 4 == 0 { -0.06 } else { 0.06 },
    ) - DVec3::Z * 0.07
}
fn materials(a: &mut Audit, d: &V, c: &V) -> usize {
    let rows = arr(&c["material_controls"]);
    assert_eq!(rows.len(), 16);
    let mut classes = std::collections::BTreeSet::new();
    for (index, m) in rows.iter().enumerate() {
        let layout = m["kind"].as_str().unwrap();
        classes.insert(layout);
        let lmu = num(&m["mu"][0]);
        let umu = num(&m["mu"][1]);
        let group = format!("material_{lmu:.2}_{umu:.2}");
        assert_eq!(lmu, [0., 0.15, 0.45, 0.85][index / 4]);
        assert_eq!(umu, [0., 0.15, 0.45, 0.85][index % 4]);
        a.count(&format!("{group}_{layout}"));
        a.count(&format!("material_{layout}"));
        if m["third_dt_s"].is_array() {
            assert!(num(&m["third_dt_s"][0]) > 0. && int(&m["finite_events"]) >= 1);
            a.count("material_third");
        }
        if m["integrated_world_impulses"].is_array() {
            a.count("material_with_free_horizon");
            for k in 0..2 {
                let finite = vec(&m["finite_impulses"][k]);
                let tail = vec(&m["tail"]["frozen"][k]);
                let total = vec(&m["integrated_world_impulses"][k]);
                assert!(
                    total.distance(finite + tail) < 1e-11
                        && num(&m["terminal_vector_bounds"][k]) > 0.
                );
            }
        }
        if lmu != 0. || umu != 0. || layout != "accumulation_then_free_release" {
            continue;
        }
        let e = &m["last_conditional_event"];
        let q = quat(&e["orientation"]);
        let v = vec(&e["after_velocity"]);
        let l = vec(&e["after_angular_momentum"]);
        let inv = tensor(d).inverse();
        let ev = [&c["lower"], &c["upper"]];
        let n = ev.map(|p| vec(&p["normal"]));
        let arms = ev.map(|p| q * corner(int(&p["corner"]) as usize));
        let supports = [vec(&d["pad_velocity"]), vec(&c["gate_velocity"])];
        let vn = [0, 1].map(|k| (v - supports[k] + world(q, inv, l).cross(arms[k])).dot(n[k]));
        let matrix = [0, 1].map(|r| {
            [0, 1].map(|s| n[r].dot(n[s] / 2. + world(q, inv, arms[s].cross(n[s])).cross(arms[r])))
        });
        let [[h, b], [cc, f]] = matrix;
        let det = h * f - b * cc;
        assert!(det > 0. && b < 0. && cc < 0. && (b - cc).abs() < 1e-12);
        let exact = [
            (-vn[0] * f + b * vn[1]) / det,
            (-h * vn[1] + cc * vn[0]) / det,
        ];
        for k in 0..2 {
            let numerical = vec(&m["tail"]["frozen"][k]);
            let error = numerical.distance(n[k] * exact[k]);
            assert!(error < 2e-10);
            a.sample("zero_friction", "direct_schur_vector_error_ns", error);
        }
    }
    if classes.len() > 1 {
        a.count("geometry_changes_outcome_with_material");
    }
    rows.len()
}
fn paired(a: &mut Audit, d: &V, c: &V, p: &V) {
    let dt = int(&p["dt_ns"]) as f64 * 1e-9;
    let bounds = &p["bounds"];
    let afterwards = dt - num(&p["last_offset_s"]);
    let after = &p["after"];
    assert!(afterwards > 0. && afterwards <= num(&bounds["free_after_s"]));
    a.sample(
        "reference",
        "certified_free_ns",
        num(&bounds["free_after_s"]) * 1e9,
    );
    let g = vec(&d["primary"]["gravity"]);
    let i = tensor(d);
    let l0 = &c["lower"];
    let v0 = vec(&l0["after_velocity"]);
    let mut sums = [DVec3::ZERO; 2];
    let mut torque = DVec3::ZERO;
    let mut osc = 0.;
    let t0 = num(&l0["at_ns"]) * 1e-9;
    let vp = [vec(&d["pad_velocity"]), vec(&c["gate_velocity"])];
    for r in arr(&p["finite_strikes"]) {
        let h = &r["event"];
        let k = int(&h["face"]) as usize;
        let j = vec(&h["impulse"]);
        sums[k] += j;
        torque += vec(&h["arm"]).cross(j);
        osc += num(&h["loss_j"]);
        a.sample("finite", "work_support_j", j.dot(vp[k]));
    }
    let last = arr(&p["finite_strikes"]).last().unwrap();
    let q = quat(&last["event"]["orientation"]);
    let n = vec(&l0["normal"]);
    let r = [&c["lower"], &c["upper"]].map(|e| q * corner(int(&e["corner"]) as usize));
    let mut radii = [0.; 2];
    let fix = &p["conditional_proof"]["frozen"];
    for k in 0..2 {
        let t = vec(&p["frozen_tail_ns"][k]);
        sums[k] += t;
        torque += r[k].cross(t);
        radii[k] = num(&p["world_vector_radii"][k]);
        assert!(
            sums[k].distance(vec(&p["world_impulses"][k])) < 1e-10
                && ((if k == 0 { n } else { -n }).dot(sums[k]) - num(&p["normal_sums_ns"][k]))
                    .abs()
                    < 1e-10
        );
    }
    osc += num(&fix["loss_j"]);
    assert!(osc > 0.);
    let ref_v = vec(&after["velocity"]);
    let ref_w = vec(&after["angular_velocity"]);
    let ref_l = world(quat(&after["orientation"]), i, ref_w);
    let ref_net = sums[0] + sums[1];
    let lin = (2. * (ref_v - v0 - g * dt)).distance(ref_net);
    let ang = (ref_l - vec(&l0["after_angular_momentum"])).distance(torque);
    assert!(lin < 1e-10 && ang < 1e-10);
    a.sample("reference", "nominal_linear_balance_ns", lin);
    a.sample("reference", "nominal_angular_balance_nms", ang);
    a.sample("reference", "uncertainty_net_ns", radii.iter().sum());
    a.sample("reference", "after_last_ns", afterwards * 1e9);
    let p_ref = vec(&after["position"]) + quat(&after["orientation"]) * DVec3::Z * 0.07;
    for test in arr(&p["controls"]) {
        let remote = test["remote_lower"] == true;
        let group = format!(
            "{}_{}_{}",
            if test["point"] == true {
                "point"
            } else {
                "group"
            },
            int(&test["calls"]),
            if remote { "remote" } else { "both" }
        );
        assert!((num(&test["start_ns"]) * 1e-9 - t0).abs() < 1e-14);
        let fi = &test["after"];
        let w = vec(&fi["angular_velocity"]);
        let q = quat(&fi["orientation"]);
        let vel = vec(&fi["velocity"]);
        let pos = vec(&fi["position"]) + q * DVec3::Z * 0.07;
        let net = vec(&test["momentum_impulse"]);
        assert!(
            (2. * (vel - vec(&test["before"]["velocity"]) - g * dt)).distance(net) < 1e-10
                && v0.distance(vec(&test["before"]["velocity"])) < 1e-10
        );
        let lower = (net.distance(ref_net) - radii[0] - radii[1]).max(0.);
        a.sample(&group, "net_error_lower_ns", lower);
        a.sample(
            &group,
            "com_error_lower_m",
            (pos.distance(p_ref) - num(&bounds["com_error_m"])).max(0.),
        );
        a.sample(
            &group,
            "spin_error_lower_rad_s",
            (w.distance(ref_w) - num(&bounds["spin_error_rad_s"])).max(0.),
        );
        for k in 0..2 {
            let label = if k == 0 { "lower" } else { "upper" };
            let signed = num(&test[format!("normal_{label}")]) - num(&p["normal_sums_ns"][k]);
            a.sample(&group, &format!("{label}_normal_bias_ns"), signed);
            if let Some(v) = test[format!("{label}_vector")].as_array() {
                let impulse = vec(&V::Array(v.clone()));
                a.sample(
                    &group,
                    &format!("{label}_error_lower_ns"),
                    (impulse.distance(sums[k]) - radii[k]).max(0.),
                );
            }
        }
    }
}
fn main() {
    let arg = std::env::args().nth(1).unwrap();
    let p = Path::new(&arg);
    let r = load(p, "terminal.json");
    let m = load(p, "materials.json");
    let mut a = Audit::default();
    let mut pairs = 0;
    let mut variants = 0;
    for (d, md) in arr(&r["donors"]).iter().zip(arr(&m["donors"])) {
        assert!(d["id"] == md["id"] && d["mode"] == md["mode"]);
        for (c, mc) in arr(&d["cases"]).iter().zip(arr(&md["cases"])) {
            assert!(c["lower"] == mc["lower"]);
            variants += materials(&mut a, md, mc);
            if c["actual"].is_object() {
                paired(&mut a, d, c, &c["actual"]);
                pairs += 1;
            }
        }
    }
    assert_eq!((pairs, variants), (57, 1200));
    let report = a.report();
    fs::write(
        p.join("readback.json"),
        serde_json::to_vec_pretty(
            &json!({"native_matched":pairs,"material_cases":variants,"audit":report}),
        )
        .unwrap(),
    )
    .unwrap();
    println!(
        "Verified {pairs} whole accumulation windows, pair impulse bounds and {variants} matched-material branches."
    );
}
