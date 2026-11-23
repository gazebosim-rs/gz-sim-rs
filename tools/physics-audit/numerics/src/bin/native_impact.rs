// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recompute point impulses and native momentum balance independently of the recorder.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{fs::File, path::PathBuf};
fn rows(x: &Value) -> &[Value] {
    x.as_array().unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn vec(v: &Value) -> DVec3 {
    DVec3::new(n(&v[0]), n(&v[1]), n(&v[2]))
}
fn q(body: &Value) -> DQuat {
    DQuat::from_array(std::array::from_fn(|i| n(&body["orientation"][i]))).normalize()
}
fn read(p: PathBuf) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn same(a: DVec3, b: DVec3) {
    assert!(a.distance(b) < 2e-12, "{a:?} != {b:?}")
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-12, "{a} != {b}")
}
fn com(b: &Value) -> DVec3 {
    (vec(&b["foot"]["position"]) + vec(&b["upper"]["position"])) * 0.5
}
fn momentum(b: &Value) -> (DVec3, DVec3) {
    let (mut p, mut h) = (DVec3::ZERO, DVec3::ZERO);
    for k in ["foot", "upper"] {
        let body = &b[k];
        let v = vec(&body["velocity"]);
        p += v;
        let rotation = q(body);
        let spin = rotation.conjugate() * vec(&body["angular_velocity"]);
        h += rotation * (DVec3::new(0.0201, 0.0201, 0.025) * spin)
            + (vec(&body["position"]) - com(b)).cross(v);
    }
    (p, h)
}
fn main() {
    let path = PathBuf::from(std::env::args().nth(1).expect("phase directory"));
    let native = read(path.join("native.json"));
    let previous = read(path.join("../multifidelity-outer-divergence/study.json"));
    let mut cases = vec![];
    let (mut native_count, mut points_count, mut boundaries) = (0usize, 0usize, 0usize);
    for case in rows(&native["cases"]) {
        let name = case["fixture"]["name"].as_str().unwrap();
        let old = rows(&previous["cases"])
            .iter()
            .find(|c| c["fixture"]["name"] == name)
            .unwrap();
        let trials = &rows(&old["segments"]).last().unwrap()["interventions"];
        let mut result = vec![];
        let mut first_locations = vec![];
        for grid in rows(&case["grids"]) {
            let calls = u(&grid["substeps"]) as usize;
            let old_key = match calls {
                4 => "matched_four",
                8 => "matched_eight",
                16 => "matched_sixteen",
                _ => panic!(),
            };
            let old_window = rows(&trials[old_key]["samples"]);
            let samples = rows(&grid["samples"]);
            assert_eq!(samples.len(), calls * (old_window.len() - 1));
            let onset = u(&grid["first_positive_outer_tick"]);
            let mut hx_n = DVec3::ZERO;
            let mut hx_t = DVec3::ZERO;
            let mut max_linear: f64 = 0.;
            let mut max_angular: f64 = 0.;
            let mut onset_ns = 0.;
            let mut first = vec![];
            for (j, batch) in samples.chunks_exact(calls).enumerate() {
                let old = &old_window[j + 1];
                let mut corners = [0.; 4];
                for (k, s) in batch.iter().enumerate() {
                    native_count += 1;
                    let tick = u(&case["cut_tick"]) + 1 + j as u64;
                    assert_eq!(u(&s["tick"]), tick);
                    assert_eq!(u(&s["substep"]), k as u64 + 1);
                    let dt = u(&s["dt_ns"]);
                    assert_eq!(dt * calls as u64, 500000);
                    assert_eq!(u(&s["time_ns"]), (tick - 1) * 500000 + (k as u64 + 1) * dt);
                    let b = &s["before"];
                    let mut jn = DVec3::ZERO;
                    let mut jt = DVec3::ZERO;
                    let mut h = DVec3::ZERO;
                    let mut sum = 0.;
                    let mut world = vec![];
                    for p in rows(&s["points"]) {
                        points_count += 1;
                        let corner = u(&p["corner"]) as usize;
                        let value = n(&p["normal_ns"]);
                        let normal = vec(&p["normal_on_foot"]) * value;
                        let tangent = vec(&p["tangent_impulse_ns"]);
                        let x = vec(&p["world_m"]);
                        same(
                            vec(&b["foot"]["position"]) + q(&b["foot"]) * vec(&p["foot_local_m"]),
                            x,
                        );
                        same(
                            vec(&b["pad"]["position"]) + q(&b["pad"]) * vec(&p["pad_local_m"]),
                            x,
                        );
                        same(normal, vec(&p["normal_impulse_ns"]));
                        close(vec(&p["normal_on_foot"]).dot(tangent), 0.);
                        let hn = (x - com(b)).cross(normal);
                        let ht = (x - com(b)).cross(tangent);
                        same(hn + ht, vec(&p["angular_impulse_nms"]));
                        if tick == onset {
                            hx_n += hn;
                            hx_t += ht;
                        }
                        jn += normal;
                        jt += tangent;
                        h += hn + ht;
                        corners[corner] += value;
                        sum += value;
                        world.push((corner, x));
                    }
                    close(sum, n(&s["normal_ns"]));
                    if tick == onset {
                        onset_ns += sum;
                    }
                    if first.is_empty() && sum > 0. {
                        world.sort_by_key(|x| x.0);
                        first = world;
                    }
                    same(jn, vec(&s["normal_impulse_ns"]));
                    same(jt, vec(&s["tangent_impulse_ns"]));
                    same(h, vec(&s["angular_impulse_nms"]));
                    let (p0, h0) = momentum(b);
                    let (p1, h1) = momentum(&s["after"]);
                    let rp = p1 - p0 - DVec3::Z * (-2. * 9.81 * dt as f64 * 1e-9) - jn - jt;
                    let rh = h1 - h0 - h;
                    same(rp, vec(&s["linear_residual_ns"]));
                    same(rh, vec(&s["angular_residual_nms"]));
                    max_angular = max_angular.max(rh.length());
                    max_linear = max_linear.max(rp.length());
                }
                for (k, total) in corners.into_iter().enumerate() {
                    close(total, n(&old["corner_ns"][k]));
                }
                let end = &batch.last().unwrap()["after"];
                same(com(end), vec(&old["com_m"]));
                same(
                    vec(&end["foot"]["angular_velocity"]),
                    vec(&old["foot_w_rad_s"]),
                );
                boundaries += 1;
            }
            same(hx_n + hx_t, vec(&grid["onset_angular_impulse_nms"]));
            close(onset_ns, n(&grid["onset_normal_ns"]));
            close(max_angular, n(&grid["max_angular_residual_nms"]));
            close(max_linear, n(&grid["max_linear_residual_ns"]));
            first_locations.push(first);
            result.push(json!({"substeps":calls,"primary_normal_ns":onset_ns,"primary_normal_angular_nms":hx_n.to_array(),
                "primary_tangent_angular_nms":hx_t.to_array(),"max_linear_residual_ns":max_linear,"max_angular_residual_nms":max_angular}));
        }
        let a = &first_locations[1];
        let b = &first_locations[2];
        assert_eq!(a.len(), 4);
        assert_eq!(a.len(), b.len());
        let max = a
            .iter()
            .zip(b)
            .map(|((i, x), (j, y))| {
                assert_eq!(i, j);
                x.distance(*y)
            })
            .fold(0., f64::max);
        cases.push(json!({"fixture":name,"matched_8_16_first_positive_max_quadrant_world_point_distance_m":max,"grids":result}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"native_steps_checked":native_count,"solver_points_checked":points_count,
        "outer_boundaries_reproduced_from_preceding_phase":boundaries,"cases":cases})
        )
        .unwrap()
    );
}
