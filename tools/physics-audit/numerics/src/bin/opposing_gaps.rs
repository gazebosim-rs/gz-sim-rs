// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent conditional two-site flight, impulse and terminal-limit readback.
#[path = "shared/impact_measurements.rs"]
mod measures;
use glam::{DMat3, DQuat, DVec3};
use measures::*;
use plotters::prelude::*;
use serde_json::{Value as V, json};
use std::{env, fs, path::Path};
fn tensor(d: &V) -> DMat3 {
    DMat3::from_cols_array(&serde_json::from_value(d["tensor"].clone()).unwrap())
}
fn interval(v: &V) -> [f64; 3] {
    let r = [num(&v[0]), num(&v[1]), num(&v[2])];
    assert!(r[0] > 0. && r[0] <= r[1] && r[1] <= r[2]);
    r
}
fn omega(q: DQuat, inv: DMat3, l: DVec3) -> DVec3 {
    q * (inv * (q.conjugate() * l))
}
fn plane(c: &V, d: &V, k: usize) -> DVec3 {
    if k == 0 {
        vec(&d["pad_velocity"])
    } else {
        vec(&c["gate_velocity"])
    }
}
fn material(k: usize) -> DVec3 {
    DVec3::new(
        if k & 2 == 0 { -0.18 } else { 0.18 },
        if k & 1 == 0 { -0.10 } else { 0.10 },
        if k & 4 == 0 { -0.06 } else { 0.06 },
    ) - 0.07 * DVec3::Z
}
fn point_accel(q: DQuat, w: DVec3, r: DVec3, i: DMat3, inv: DMat3, g: DVec3) -> DVec3 {
    let a = inv * ((i * w).cross(w));
    g + q * (a.cross(r) + w.cross(w.cross(r)))
}
// Integrate Euler's body-rate equation and attitude together. An augmented
// pair integrates acceleration to initial-zero signed velocity and height in
// normalized time. Dividing out dt² keeps the test well-scaled below 1e-200 m.
#[derive(Clone, Copy)]
struct Y {
    q: DQuat,
    w: DVec3,
    u: f64,
    h: f64,
}
impl Y {
    fn add(self, k: Y, t: f64) -> Self {
        Self {
            q: self.q + k.q * t,
            w: self.w + k.w * t,
            u: self.u + k.u * t,
            h: self.h + k.h * t,
        }
    }
}
fn propagated(e: &V, site: usize, n: DVec3, dt: f64, i: DMat3, g: DVec3) -> (f64, DQuat, DVec3) {
    let q = quat(&e["orientation"]);
    let inv = i.inverse();
    let mut y = Y {
        q,
        w: inv * (q.conjugate() * vec(&e["after_angular_momentum"])),
        u: 0.,
        h: 0.,
    };
    let rhs = |z: Y| {
        let q = z.q.normalize();
        let w = z.w;
        let dq = q * DQuat::from_xyzw(w.x, w.y, w.z, 0.) * (dt / 2.);
        Y {
            q: dq,
            w: inv * ((i * w).cross(w)) * dt,
            u: n.dot(point_accel(q, w, material(site), i, inv, g)),
            h: z.u,
        }
    };
    let step = 1. / 16.;
    for _ in 0..16 {
        let a = rhs(y);
        let b = rhs(y.add(a, step / 2.));
        let c = rhs(y.add(b, step / 2.));
        let d = rhs(y.add(c, step));
        y = y
            .add(a, step / 6.)
            .add(b, step / 3.)
            .add(c, step / 3.)
            .add(d, step / 6.);
        y.q = y.q.normalize();
    }
    (y.h, y.q, y.w)
}
fn verify_hit(a: &mut Audit, d: &V, c: &V, e: &V) {
    let i = tensor(d);
    let inv = i.inverse();
    let q = quat(&e["orientation"]);
    let k = int(&e["face"]) as usize;
    let r = q * material(int(&e["corner"]) as usize);
    let n = vec(&e["normal"]);
    let v = vec(&e["velocity"]);
    let l = vec(&e["angular_momentum"]);
    let v2 = vec(&e["after_velocity"]);
    let l2 = vec(&e["after_angular_momentum"]);
    let j = vec(&e["impulse"]);
    let vp = plane(c, d, k);
    let before = v - vp + omega(q, inv, l).cross(r);
    let after = v2 - vp + omega(q, inv, l2).cross(r);
    let norm = j.dot(n);
    let tang = j - n * norm;
    let mu = num(&e["mu"]);
    let loss = -0.5 * j.dot(before + after);
    assert!(
        r.distance(vec(&e["arm"])) < 1e-12
            && before.distance(vec(&e["active_prior_velocity"])) < 1e-10
            && after.distance(vec(&e["after_relative_velocity"])) < 1e-10
    );
    assert!(
        norm > 0.
            && before.dot(n) < 0.
            && after.dot(n).abs() < 1e-10
            && (norm - num(&e["normal_ns"])).abs() < 1e-10
    );
    assert!(
        tang.length() <= mu * norm + 1e-10 && (loss - num(&e["loss_j"])).abs() < 1e-9 && loss >= 0.
    );
    if e["stick"] == true {
        assert!(after.length() < 1e-10)
    } else if mu > 0. {
        assert!(tang.distance(-mu * norm * after.normalize()) < 1e-10);
    }
    let dk = (v2 - v).dot(v2 + v) + 0.5 * (l2 - l).dot(omega(q, inv, l2 + l));
    let linear = (2. * (v2 - v)).distance(j);
    let angular = (l2 - l).distance(r.cross(j));
    let work = (dk - j.dot(vp) + loss).abs();
    assert!(linear < 1e-10 && angular < 1e-10 && work < 1e-9);
    a.sample("all_hits", "linear_residual_ns", linear);
    a.sample("all_hits", "angular_residual_nms", angular);
    a.sample("all_hits", "work_residual_j", work);
}
fn verified_limit(a: &mut Audit, d: &V, c: &V, last: &V, p: &V) {
    let i = tensor(d);
    let inv = i.inverse();
    let q = quat(&last["orientation"]);
    let v = vec(&last["after_velocity"]);
    let l = vec(&last["after_angular_momentum"]);
    let fixed = &p["frozen"];
    let sites = [&c["lower"], &c["upper"]];
    let n = sites.map(|s| vec(&s["normal"]));
    let r = sites.map(|s| q * material(int(&s["corner"]) as usize));
    let impulse = [0, 1].map(|k| vec(&fixed["impulses"][k]));
    let dp = impulse[0] + impulse[1];
    let dl = r[0].cross(impulse[0]) + r[1].cross(impulse[1]);
    let vf = vec(&fixed["velocity"]);
    let lf = vec(&fixed["momentum"]);
    assert!((2. * (vf - v)).distance(dp) < 1e-11 && (lf - l).distance(dl) < 1e-11);
    let dk = (vf - v).dot(vf + v) + 0.5 * (lf - l).dot(omega(q, inv, l + lf));
    let support = impulse[0].dot(plane(c, d, 0)) + impulse[1].dot(plane(c, d, 1));
    assert!((dk - support + num(&fixed["loss_j"])).abs() < 1e-10);
    let h = num(&p["remaining_time_cap_s"]);
    let ratio = num(&p["normal_speed_pair_ratio_cap"]);
    let sum = num(&p["total_impulse_bound_ns"]);
    assert!(
        ratio < 1.
            && ratio > 0.
            && num(&p["interarrival_ratio_cap"]) <= 0.25
            && sum < num(&p["trial_impulse_limit_ns"])
    );
    assert!(num(&p["other_gap_m"]) > 0. && num(&p["footprint_margin_m"]) > 0. && h > 0.);
    for k in 0..2 {
        let normal = impulse[k].dot(n[k]);
        let lo = num(&p["normal_sum_bounds_ns"][k][0]);
        let hi = num(&p["normal_sum_bounds_ns"][k][1]);
        let radius = num(&p["world_vector_radii"][k]);
        let diff = impulse[k].distance(vec(&p["world_vector_centers"][k]));
        assert!(
            normal >= lo - 5e-11
                && normal <= hi + 5e-11
                && diff <= radius + 5e-11
                && num(&p["both_release_accel_floor"][k]) > 0.
                && num(&p["unique_friction_margin"][k]) > 0.
        );
        let u = (vf - plane(c, d, k) + omega(q, inv, lf).cross(r[k])).dot(n[k]);
        assert!((u - num(&fixed["normal_velocity"][k])).abs() < 1e-11 && u.abs() < 2e-11);
        a.sample("limit", "normal_sum_ns", normal);
        a.sample("limit", "vector_radius_ns", radius);
    }
    assert!(impulse.iter().map(|j| j.length()).sum::<f64>() < sum + 1e-11);
    a.sample("limit", "remaining_time_s", h);
    a.sample("limit", "pair_contraction", ratio);
    a.sample("limit", "impulse_bound_ns", sum);
}
fn verify(a: &mut Audit, d: &V, c: &V) -> (f64, f64, bool) {
    a.count("third_strike");
    let seq = arr(&c["sequence"]);
    let first = &seq[0];
    let mut prev = &c["lower"];
    let g = vec(&d["primary"]["gravity"]);
    let i = tensor(d);
    let seed = interval(&c["seed"]["gap_m"]);
    let mut gap = seed;
    let mut flo = 0.;
    let prior_dt = (num(&c["lower"]["at_ns"]) - num(&c["upper"]["at_ns"])) * 1e-9;
    let up = &c["upper"];
    let (released, _, _) = propagated(
        up,
        int(&up["corner"]) as usize,
        vec(&up["normal"]),
        prior_dt,
        i,
        g,
    );
    assert!(
        released >= seed[0] / prior_dt / prior_dt - 1e-7
            && released <= seed[2] / prior_dt / prior_dt + 1e-7
    );
    for (index, entry) in seq.iter().enumerate() {
        let e = &entry["event"];
        let t = interval(&entry["relative_interval_s"]);
        let active = int(&e["face"]) as usize;
        assert!(active == (index + 1) % 2 && int(&entry["ordinal"]) == index as u64 + 3);
        assert!(
            gap == interval(&entry["incoming_gap_m"])
                && num(&entry["other_gap_m"]) > 0.
                && num(&entry["finite_margin_m"]) > 0.
        );
        let (bend, q, w) = propagated(
            prev,
            int(&prev["corner"]) as usize,
            vec(&prev["normal"]),
            t[1],
            i,
            g,
        );
        let outgoing = interval(&entry["released_height_m"]);
        if t[1] > 1e-145 {
            let low = outgoing[0] / t[1] / t[1];
            let high = outgoing[2] / t[1] / t[1];
            let tol = 1e-7 * (1. + bend.abs());
            assert!(
                bend >= low - tol && bend <= high + tol,
                "{bend} outside {low}..{high}"
            );
        }
        let qa = quat(&e["orientation"]);
        let omega_error =
            (qa * (i.inverse() * (qa.conjugate() * vec(&e["angular_momentum"])))).distance(q * w);
        assert!(omega_error < 1e-8 && (num(&entry["released_accel_floor"]) > 0.));
        assert!(q.dot(qa).abs() > 1. - 1e-11);
        let n = vec(&e["normal"]);
        let site = int(&e["corner"]) as usize;
        let (target_bend, _, _) = propagated(prev, site, n, t[1], i, g);
        let q0 = quat(&prev["orientation"]);
        let w0 = omega(q0, i.inverse(), vec(&prev["after_angular_momentum"]));
        let vn = n.dot(
            vec(&prev["after_velocity"]) - plane(c, d, active) + w0.cross(q0 * material(site)),
        );
        // Compare in m/s; never subtract the two nearly coincident world points.
        let residual = (gap[1] / t[1] + vn + target_bend * t[1]).abs();
        assert!(residual < 2e-8, "crossing residual {residual}");
        a.sample("all_hits", "crossing_residual_m_s", residual);
        verify_hit(a, d, c, e);
        if index == 0 {
            flo = outgoing[1];
        }
        gap = outgoing;
        prev = e;
    }
    let t = interval(&first["relative_interval_s"])[1] * 1e9;
    a.sample("third", "relative_time_ns", t);
    a.sample("third", "lower_gap_m", flo);
    let abs = num(&first["released_height_from_world_pose_m"]);
    if abs <= 0. {
        a.count("third_world_pose_nonpositive");
    }
    if (abs / flo - 1.).abs() > 0.25 {
        a.count("third_world_pose_over_quarter_error");
    }
    if t < 1. {
        a.count("sub_ns_third");
        assert!(arr(&c["native"]).is_empty());
    }
    if let Some(old) = c["original_third"].as_object() {
        a.count("previously_ordered");
        let delta = (num(&old["at_ns"]) - num(&first["event"]["at_ns"])).abs();
        a.sample("third", "older_time_diff_ns", delta);
        assert!(delta < 3e-6);
    }
    if let Some(z) = c["seed"]["fixed_normal_control"].as_object() {
        let sol = &z["simultaneous_ns"];
        let alt = &z["alternating_ns"];
        let err = (num(&sol[0]) - num(&alt[0])).hypot(num(&sol[1]) - num(&alt[1]));
        assert!(err < 1e-9 && num(&z["rho"]) < 1.);
        a.sample("frictionless_fixed_pose", "two_site_impulse_error_ns", err);
    }
    for k in arr(&c["native"]) {
        let group = format!(
            "native_{}_{}_{}",
            if k["point"] == true { "point" } else { "group" },
            int(&k["calls"]),
            if k["remote_lower"] == true {
                "remote"
            } else {
                "present"
            }
        );
        let ground = num(&k["normal_lower"]);
        let j = vec(&first["event"]["impulse"]);
        let err = vec(&k["momentum_impulse"]).distance(j);
        a.sample(&group, "net_vector_error_ns", err);
        a.sample(&group, "spurious_lower_ns", ground);
        if ground > 1e-8 {
            a.count(&format!("{group}_spurious_ground"));
        }
        if let (Some(_), Some(_)) = (k["upper_vector"].as_array(), k["lower_vector"].as_array()) {
            let roof = vec(&k["upper_vector"]);
            let floor = vec(&k["lower_vector"]);
            assert!((roof + floor).distance(vec(&k["momentum_impulse"])) < 1e-8);
            a.sample(&group, "upper_vector_error_ns", roof.distance(j));
        }
    }
    let kind = c["limit"]["kind"].as_str().unwrap();
    a.count(kind);
    if kind == "accumulation_then_free_release" {
        verified_limit(a, d, c, prev, &c["limit"]["proof"]);
    }
    if kind == "departing" {
        assert!(num(&c["limit"]["free"]["other_gap_floor_m"]) > 0.);
    }
    (t, flo, c["original_third"].is_null())
}
fn main() {
    let arg = env::args().nth(1).unwrap();
    let p = Path::new(&arg);
    let raw: V =
        serde_json::from_reader(fs::File::open(p.join("classified.json")).unwrap()).unwrap();
    let mut a = Audit::default();
    let mut pts = vec![];
    for d in arr(&raw["donors"]) {
        for c in arr(&d["cases"]) {
            pts.push(verify(&mut a, d, c));
        }
    }
    assert_eq!(pts.len(), 75);
    let report = a.report();
    assert_eq!(int(&report["counts"]["accumulation_then_free_release"]), 57);
    fs::write(
        p.join("readback.json"),
        serde_json::to_vec_pretty(&json!({"qualified":pts.len(),"audit":report})).unwrap(),
    )
    .unwrap();
    let img = p.join("positive-clearance.svg");
    let root = SVGBackend::new(&img, (860, 500)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption(
            "Upper reimpact while the lower material corner is separated",
            ("sans-serif", 22),
        )
        .margin(14)
        .x_label_area_size(54)
        .y_label_area_size(76)
        .build_cartesian_2d(
            (0.1f64..1e5f64).log_scale(),
            (1e-18f64..1e-5f64).log_scale(),
        )
        .unwrap();
    chart
        .configure_mesh()
        .x_desc("Elapsed time after lower strike (ns)")
        .y_desc("Lower clearance (m)")
        .x_label_formatter(&|v| format!("{v:.0e}"))
        .y_label_formatter(&|v| format!("{v:.0e}"))
        .draw()
        .unwrap();
    chart
        .draw_series(LineSeries::new([(1., 1e-18), (1., 1e-5)], BLACK.mix(0.3)))
        .unwrap();
    chart
        .draw_series(LineSeries::new(
            [(0.1, 1e-10), (1e5, 1e-10)],
            BLACK.mix(0.3),
        ))
        .unwrap();
    for (t, g, added) in pts {
        chart
            .draw_series([Circle::new(
                (t, g),
                4,
                if added {
                    RGBColor(191, 86, 22)
                } else {
                    RGBColor(26, 112, 132)
                }
                .filled(),
            )])
            .unwrap();
    }
    root.present().unwrap();
    println!(
        "Verified finite-gap trajectories, observed native impulses and 57 conditional limiting envelopes."
    );
}
