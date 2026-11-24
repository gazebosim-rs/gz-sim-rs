// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check native anchor, time-grid, zero-spin separation and solver setting controls.
use glam::DVec3;
use serde_json::{Value as V, json};
use std::{collections::BTreeSet, fs::File, path::Path};
fn read(p: &Path) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn n(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn xyz(v: &V) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let path = Path::new(&path);
    let data = read(&path.join("early.json"));
    let refs = read(&path.join("../multifidelity-inertia-timing/inertia.json"));
    let rows = data["cases"].as_array().unwrap();
    let mut combinations = BTreeSet::new();
    let mut loaded_records = 0;
    let mut summaries = vec![];
    let mut sample = None;
    for r in rows {
        let id = u(&r["id"]) as usize;
        let c = &refs["cases"][id];
        assert!(c["certificate"]["reason"].is_null());
        let x = &r["work"];
        let calls = u(&x["calls"]);
        let temporal = u(&x["temporal"]);
        let relax = u(&x["relax"]);
        let fresh = x["fresh"] == true;
        let locked = r["locked"] == true;
        let frame = r["frame"].as_str().unwrap();
        assert!(combinations.insert((id, locked, frame, calls, temporal, relax, fresh)));
        assert_eq!(
            (n(&c["arrival_s"]) * 1e9).round() as u64,
            u(&r["analytic_arrival_ns"])
        );
        assert!((n(&c["impulse_ns"]) - n(&r["analytic_impulse_ns"])).abs() < 1e-11);
        let first = u(&r["first_positive_end_ns"]);
        let pre = n(&r["pre_arrival_ns"]);
        let k = (first * calls).div_ceil(500000);
        assert_eq!(k * 500000 / calls, first);
        assert!(pre <= n(&r["total_normal_ns"]) + 1e-12);
        if first >= u(&r["analytic_arrival_ns"]) {
            assert_eq!(pre, 0.)
        } else {
            assert!(pre > 0.)
        };
        let mut first_j = 0.;
        for p in r["rows_at_first_reaction"].as_array().unwrap() {
            let normal = xyz(&p["normal_on_foot"]);
            assert!((normal.length() - 1.).abs() < 1e-12);
            let gap = n(&p["geometric_gap_m"]);
            let sg = n(&p["solver_gap_at_update_m"]);
            assert!(gap.is_finite() && sg.is_finite());
            for f in [
                "local_foot",
                "local_pad",
                "solver_foot_anchor",
                "solver_pad_anchor",
            ] {
                assert!(xyz(&p[f]).is_finite());
            }
            let j = n(&p["impulse_ns"]);
            assert!(j >= 0.);
            first_j += j;
            loaded_records += 1;
        }
        assert!(first_j > 0. && first_j <= n(&r["total_normal_ns"]) + 1e-12);
        if !locked && id == 125 && calls == 1 && temporal == 4 && !fresh {
            if relax == 0 {
                assert_eq!(pre, 0.)
            } else {
                assert!(pre > 0.36 && pre < 0.44);
                assert_eq!(first, 2000000);
                let expected =
                    (n(&c["arrival_s"]) - first as f64 * 1e-9) * n(&c["closing_speed_m_s"]);
                assert!((n(&r["first_interval_Euler_geometry_m"][1]) - expected).abs() < 1e-13);
                if frame == "world" && relax == 16 {
                    sample = Some(r.clone());
                }
            }
        }
        if frame == "world" && calls == 1 && temporal == 4 && (relax == 0 || relax == 16) {
            summaries.push(json!({"id":id,"locked":locked,"fresh":fresh,"relax":relax,"first_ns":first,"pre_ns":pre,
                "analytic_end_gap_m":r["first_interval_Euler_geometry_m"][1],"native_end_gap_m":r["first_interval_native_geometry_m"][1]}));
        }
    }
    assert_eq!(combinations.len(), 2460);
    println!("{}",serde_json::to_string_pretty(&json!({"trials":combinations.len(),"loaded_native_point_rows":loaded_records,"controls":summaries,"zero_spin_control":sample.unwrap()})).unwrap());
}
