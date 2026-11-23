// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare simultaneous-contact refusals, physical point laws, resolved sequences and native friction.
use glam::{DMat2, DMat3, DQuat, DVec3};
use serde_json::{Value as V, json};
use std::{fs::File, path::Path};
#[path = "shared/impact_measurements.rs"]
mod measures;
use measures::*;
fn kind(e: &V) -> &'static str {
    if e["mu"] == 0. {
        "frictionless"
    } else if e["stick"] == true {
        "sticking"
    } else {
        "sliding"
    }
}
fn local(c: &V, e: &V) -> DQuat {
    let q = quat(&c["primary"]["pad_q"]);
    if int(&e["face"]) == 0 {
        q
    } else {
        q * DQuat::from_xyzw(1., 0., 0., 0.)
    }
}
fn law(a: &mut Audit, group: &str, c: &V, e: &V) {
    let q = local(c, e);
    let j = vec(&e["impulse"]);
    let arm = vec(&e["arm"]);
    let before = vec(&e["active_prior_velocity"]);
    let after = vec(&e["after_relative_velocity"]);
    let k = DMat3::from_cols_array(&serde_json::from_value(e["delassus_columns"].clone()).unwrap());
    let local_j = q.conjugate() * j;
    let mu = num(&e["mu"]);
    let jt = local_j.truncate();
    let gap = num(&e["other_gap_m"]);
    let margin = num(&e["finite_margin_m"]);
    assert!((q * DVec3::Z).distance(vec(&e["normal"])) < 1e-12);
    assert!(local_j.z > 0. && gap > 1e-10 && margin > 1e-9 && num(&e["schur_margin"]) > 1e-9);
    let post = q.conjugate() * after;
    assert!(post.z.abs() < 1e-8);
    assert!((before + q * (k * local_j)).distance(after) < 1e-8);
    let linear = 2. * (vec(&e["after_velocity"]) - vec(&e["velocity"]));
    let angular = vec(&e["after_angular_momentum"]) - vec(&e["angular_momentum"]);
    assert!(linear.distance(j) < 1e-10 && angular.distance(arm.cross(j)) < 1e-10);
    let loss = -0.5 * j.dot(before + after);
    assert!(loss > 0. && (loss - num(&e["loss_j"])).abs() < 1e-9 * (1. + loss));
    assert!(jt.length() <= mu * local_j.z + 1e-9);
    if kind(e) == "sliding" {
        assert!((jt.length() - mu * local_j.z).abs() < 1e-9);
        assert!(jt.normalize().dot(post.truncate().normalize()) < -1. + 1e-9);
    }
    if kind(e) == "sticking" {
        assert!(post.length() < 1e-8);
    }
    a.count(&format!("{group}/{}", kind(e)));
    a.sample(group, "other_gap_m", gap);
    a.sample(group, "finite_margin_m", margin);
    a.sample(group, "normal_ns", local_j.z);
    a.sample(group, "dissipated_j", loss);
}
// Reconstruct the fixed point of inverse-tangential-effective-mass iteration
// and Euclidean projection onto the native friction disk. This uses independent
// measured incoming velocity, contact inertia, and material only.
fn preconditioned(k: DMat3, u: DVec3, mu: f64) -> DVec3 {
    let z = k.z_axis;
    let ct = z.truncate();
    let tt = DMat2::from_cols(k.x_axis.truncate(), k.y_axis.truncate());
    let sc = DMat2::from_cols(
        k.x_axis.truncate() - ct * (k.x_axis.z / z.z),
        k.y_axis.truncate() - ct * (k.y_axis.z / z.z),
    );
    let b = -u.truncate() + ct * (u.z / z.z);
    let response = |lambda: f64| {
        let jt = (sc + lambda * tt).inverse() * b;
        let normal = (-u.z - ct.dot(jt)) / z.z;
        (jt.extend(normal), jt.length() - mu * normal)
    };
    if mu == 0. {
        return DVec3::new(0., 0., -u.z / z.z);
    }
    let (stick, f) = response(0.);
    if f <= 0. {
        return stick;
    }
    let mut hi = 1.;
    for _ in 0..96 {
        if response(hi).1 < 0. {
            break;
        }
        hi *= 2.;
    }
    assert!(response(hi).1 < 0.);
    let mut lo = 0.;
    for _ in 0..72 {
        let t = (lo + hi) / 2.;
        if response(t).1 > 0. {
            lo = t;
        } else {
            hi = t;
        }
    }
    response((lo + hi) / 2.).0
}
fn cold(a: &mut Audit, category: &str, c: &V, e: &V, data: &V) {
    let j = vec(&e["impulse"]);
    let q = local(c, e);
    let k = DMat3::from_cols_array(&serde_json::from_value(e["delassus_columns"].clone()).unwrap());
    let u = q.conjugate() * vec(&e["active_prior_velocity"]);
    let mu = num(&e["mu"]);
    let native_fixed = preconditioned(k, u, mu);
    let tt = DMat2::from_cols(k.x_axis.truncate(), k.y_axis.truncate());
    for row in arr(data) {
        let calls = int(&row["calls"]);
        assert!(calls <= int(&row["span_ns"]));
        let kind = kind(e);
        let common = format!("{category}/{kind}");
        let lane = format!(
            "{common}/{}-pgs{}-calls{calls}-{}",
            if row["point_friction"] == true {
                "point"
            } else {
                "grouped"
            },
            int(&row["pgs"]),
            if row["remote_other"] == true {
                "remote"
            } else {
                "both"
            }
        );
        let target = if int(&e["face"]) == 1 {
            &row["upper"]
        } else {
            &row["lower"]
        };
        let other = if int(&e["face"]) == 1 {
            &row["lower"]
        } else {
            &row["upper"]
        };
        let normal = num(&target["normal_ns"]);
        assert!((arr(&target["corners"]).iter().map(num).sum::<f64>() - normal).abs() < 1e-9);
        assert_eq!(int(&e["face"]), int(&row["face"]));
        a.sample(&lane, "other_normal_ns", num(&other["normal_ns"]));
        a.sample(&lane, "wrong_corners_ns", num(&row["wrong_corners_ns"]));
        if row["chosen_impulse"].is_null() {
            a.count(&format!("{lane}/unobservable tangent"));
            continue;
        }
        let impulse = vec(&row["chosen_impulse"]);
        let local = q.conjugate() * impulse;
        let v = u + k * local;
        let error = j.distance(impulse);
        assert!((error - num(&row["vector_error_ns"])).abs() < 1e-9);
        a.sample(&lane, "physical_vector_error_ns", error);
        a.sample(
            &lane,
            "native_fixed_point_error_ns",
            local.distance(native_fixed),
        );
        a.sample(&lane, "native_duration_ns", num(&row["span_ns"]));
        if kind == "sliding" && local.truncate().length() > 0. && v.truncate().length() > 1e-7 {
            let jt = local.truncate().normalize();
            let scalar = -jt.dot(v.truncate().normalize());
            let weighted = -jt.dot((tt.inverse() * v.truncate()).normalize());
            assert!((scalar - num(&row["direction"]["opposition_cos"])).abs() < 1e-8);
            a.sample(&lane, "opposing_physical_slip_cos", scalar);
            a.sample(&lane, "opposing_inverse_inertia_slip_cos", weighted);
            a.sample(
                &lane,
                "friction_disk_residual_abs_ns",
                (local.truncate().length() - mu * local.z).abs(),
            );
        }
    }
}
fn main() {
    let dir = std::env::args_os().nth(1).unwrap();
    let p = Path::new(&dir);
    let v: V = serde_json::from_reader(File::open(p.join("mechanics.json")).unwrap()).unwrap();
    let mut a = Audit::default();
    let mut cases = 0;
    let mut qualified = 0;
    for c in arr(&v["cases"]) {
        for s in arr(&c["cases"]) {
            cases += 1;
            let d = &s["data"];
            let delay = int(&s["delay_ns"]);
            if d["upper"].is_null() {
                assert!(delay <= 8);
                a.count("upper admission refused");
                continue;
            }
            qualified += 1;
            let upper = &d["upper"];
            law(&mut a, "upper", c, upper);
            let when = num(&upper["at_ns"]);
            assert!((when - num(&c["primary"]["arrival_ns"]) - delay as f64).abs() < 0.001);
            let native = &d["native"];
            cold(&mut a, "upper", c, upper, &native["cold"]);
            a.count(&format!("upper follow/{}", d["follow"].as_str().unwrap()));
            a.sample(
                "public ceiling",
                "first_native_endpoint_error_us",
                (num(&native["first_gate_end_ns"]) - when) / 1e3,
            );
            a.sample(
                "public ceiling",
                "first_native_normal_error_ns",
                (num(&native["first_gate_normal_ns"]) - num(&upper["normal_ns"])).abs(),
            );
            if native["first_interval_inside_physical_window"] == true {
                a.count("first native endpoint in qualified flight");
            }
            if num(&native["simultaneous_native_lower_ns"]) > 0. {
                a.count("first native interval also loads lower");
            }
            let mut prev = upper;
            let mut expected = &d["next"];
            for (n, stage) in arr(&d["subsequent"]).iter().enumerate() {
                let event = &stage["event"];
                let at = num(&event["at_ns"]);
                assert_eq!(event, expected);
                assert!(at > num(&prev["at_ns"]));
                law(&mut a, "subsequent", c, event);
                if n == 0 {
                    let face = if int(&event["face"]) == 0 {
                        "lower-return"
                    } else {
                        "upper-rehit"
                    };
                    a.count(face);
                    a.sample(face, "separation_from_second_us", (at - when) / 1e3);
                    cold(&mut a, face, c, event, &stage["matched"]["cold"]);
                }
                a.sample(
                    "all intervals",
                    "two_integrator_attitude_difference_rad",
                    num(&stage["attitude_comparison_rad"]),
                );
                expected = &stage["next"];
                prev = event;
            }
            assert_eq!(expected.is_object(), d["sequence_budget_reached"] == true);
            a.count(&format!(
                "subsequent isolated strikes/{}",
                arr(&d["subsequent"]).len()
            ));
        }
    }
    assert_eq!(cases, 990);
    assert_eq!(qualified, 594);
    let result = json!({"placements":cases,"certified_upper":qualified,"audit":a.report()});
    serde_json::to_writer_pretty(File::create(p.join("readback.json")).unwrap(), &result).unwrap();
    println!("{qualified} independent upper strikes in {cases} matched placements");
}
