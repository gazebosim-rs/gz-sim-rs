// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct tensor response from independent principal moments and actual native impulses.
use glam::{DMat3, DQuat, DVec3};
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
fn read(p: &Path) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn s(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn q(v: &V) -> DQuat {
    DQuat::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn xyz(v: &V) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn mat(v: &V) -> DMat3 {
    DMat3::from_cols_array(&serde_json::from_value(v.clone()).unwrap())
}
fn max_norm(m: DMat3) -> f64 {
    m.to_cols_array()
        .into_iter()
        .map(f64::abs)
        .fold(0., f64::max)
}
fn quantile(mut xs: Vec<f64>) -> V {
    xs.sort_by(f64::total_cmp);
    let n = xs.len();
    json!({"cases":n,"above_1e-10":xs.iter().filter(|x|**x>1e-10).count(),"p50":xs[(n-1)/2],"p95":xs[(95*n).div_ceil(100)-1],"max":xs[n-1]})
}
fn main() {
    let folder = std::env::args().nth(1).unwrap();
    let dir = Path::new(&folder);
    let survey = read(&dir.join("frames.json"));
    let featured = &survey["featured"];
    let cases = survey["cases"].as_array().unwrap();
    let mut groups = BTreeMap::<String, BTreeMap<String, Vec<f64>>>::new();
    let mut ids = BTreeSet::new();
    for c in cases {
        let id = c["id"].as_u64().unwrap() as usize;
        assert!(ids.insert(id));
        let body = q(&c["body_orientation"]);
        let phi = body * q(&c["principal_to_link"]);
        let inv = xyz(&c["principal"]).recip();
        let col = |axis: DVec3| phi * (inv * (phi.conjugate() * axis));
        let reference = DMat3::from_cols(col(DVec3::X), col(DVec3::Y), col(DVec3::Z));
        assert!(max_norm(reference - mat(&c["expected_inverse"])) / max_norm(reference) < 2e-15);
        for key in [
            "original_rigid_inverse",
            "original_member_inverse",
            "native_rigid_inverse",
            "native_member_inverse",
        ] {
            let actual = mat(&c[key]);
            let error = max_norm(actual - reference) / max_norm(reference);
            if key.starts_with("native") {
                assert!(error <= 2e-11);
            }
            for g in [
                "all".into(),
                c["class"].as_str().unwrap().to_owned(),
                format!("scale_{}", s(&c["scale"])),
            ] {
                groups
                    .entry(g)
                    .or_default()
                    .entry(key.into())
                    .or_default()
                    .push(error);
            }
        }
        if featured["id"] == c["id"] {
            let body_rot = DMat3::from_quat(body);
            let old = body_rot.transpose() * mat(&c["original_rigid_inverse"]).inverse() * body_rot;
            assert!(max_norm(mat(&featured["effective_old_tensor"]) - old) < 1e-13);
        }
    }
    assert_eq!(ids.len(), 1728);
    let mut stats = BTreeMap::new();
    for (g, series) in groups {
        stats.insert(
            g,
            series
                .into_iter()
                .map(|(s, xs)| (s, quantile(xs)))
                .collect::<BTreeMap<_, _>>(),
        );
    }
    let impact = read(&dir.join("impact.json"));
    let ore = &impact["reference"];
    let normal = xyz(&ore["normal"]);
    let j = s(&ore["impulse_ns"]);
    let r = ore["corner"].as_u64().unwrap();
    let rot = q(&ore["exact_at"]["foot"]["orientation"]);
    let arm = rot
        * DVec3::new(
            if r >= 2 { 0.18 } else { -0.18 },
            if r % 2 == 1 { 0.1 } else { -0.1 },
            -0.13,
        );
    let e = arm.cross(normal);
    let i = mat(&featured["expected_tensor"]);
    let rw = DMat3::from_quat(rot);
    let expected =
        s(&ore["closing_speed_m_s"]) / (0.5 + e.dot(rw * i.inverse() * rw.transpose() * e));
    assert!((expected - j).abs() < 1e-13);
    let com = |b: &V| {
        xyz(&b["foot"]["position"]) + q(&b["foot"]["orientation"]) * DVec3::new(0., 0., 0.07)
    };
    let mut checked = vec![];
    for run in impact["trials"].as_array().unwrap() {
        let total: DVec3 = run["ticks"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| xyz(&t["j"]))
            .sum();
        let ej = total.distance(j * normal);
        let ec = com(&run["end"]).distance(com(&ore["exact_end"]));
        let ew = xyz(&run["end"]["foot"]["angular_velocity"])
            .distance(xyz(&ore["exact_end"]["foot"]["angular_velocity"]));
        for (value, name) in [
            (ej, "impulse_vector_error_ns"),
            (ec, "com_error_m"),
            (ew, "spin_error_rad_s"),
        ] {
            assert!((value - s(&run["metrics"][name])).abs() < 1e-12);
        }
        checked.push(json!({"label":run["label"],"native_calls":run["native_calls"],"first_end_ns":run["first_positive_end_ns"],"impulse_error_ns":ej,"com_error_m":ec,"spin_error_rad_s":ew}));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"constructors_per_case":2,"axes_per_constructor":3,"survey":stats,"featured_id":featured["id"],"closed_form_ns":expected,"impact":checked})).unwrap());
}
