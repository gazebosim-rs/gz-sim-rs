// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate nominal strikes, simultaneous refusals, live interval proofs and public cost.
use glam::DVec3;
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};

fn arr(v: &V) -> &[V] {
    v.as_array().unwrap()
}
fn f(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn n(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn v3(v: &V) -> DVec3 {
    DVec3::new(f(&v[0]), f(&v[1]), f(&v[2]))
}
fn read(p: &Path) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
#[derive(Default)]
struct Measures(BTreeMap<String, BTreeMap<String, Vec<f64>>>);
impl Measures {
    fn add(&mut self, key: &str, m: &str, s: f64) {
        assert!(s.is_finite());
        self.0
            .entry(key.into())
            .or_default()
            .entry(m.into())
            .or_default()
            .push(s);
    }
    fn summary(self) -> V {
        json!(self.0.into_iter().map(|(k,ms)|(k,ms.into_iter().map(|(m,mut v)|{
            v.sort_by(f64::total_cmp);let len=v.len();(m,json!({"n":len,"min":v[0],"p50":v[len.div_ceil(2)-1],"p95":v[(95*len).div_ceil(100)-1],"max":v[len-1]}))
        }).collect::<BTreeMap<_,_>>())).collect::<BTreeMap<_,_>>())
    }
}
fn lane(named: bool) -> &'static str {
    if named { "named" } else { "ordinary" }
}
fn study(path: &Path) -> V {
    let v = read(&path.join("study.json"));
    let mut stats = Measures::default();
    let mut modes = BTreeMap::<String, usize>::new();
    let mut counts = BTreeMap::<String, BTreeMap<String, usize>>::new();
    let mut trials = BTreeSet::new();
    let mut feature = V::Null;
    let mut guard_count = 0;
    let mut after_gate = 0;
    let mut declined = BTreeSet::new();
    for donor in arr(&v["controls"]) {
        let id = n(&donor["id"]);
        let mode = donor["mode"].as_str().unwrap();
        *modes.entry(mode.into()).or_default() += 1;
        let when = f(&donor["primary_ns"]);
        let true_j = v3(&donor["reference"]["target_impulse"]);
        for c in arr(&donor["cases"]) {
            let physical = f(&c["physical_gate"]["time_ns"]);
            let delay = c["delay_ns"].as_i64().unwrap();
            assert!(trials.insert((id, mode, delay)));
            assert!((when + delay as f64 - physical).abs() < 0.001);
            assert!(f(&c["physical_gate"]["closing_lower_m_s"]) > 0.);
            assert!(f(&c["physical_gate"]["finite_face_margin_m"]) > 0.);
            stats.add(
                "geometry",
                "finite_face_margin_m",
                f(&c["physical_gate"]["finite_face_margin_m"]),
            );
            stats.add(
                "geometry",
                "closing_lower_m_s",
                f(&c["physical_gate"]["closing_lower_m_s"]),
            );
            for l in arr(&c["lanes"]) {
                let named = l["named"].as_bool().unwrap();
                let lane = lane(named);
                let k = format!("{delay}/{lane}");
                let r = &l["run"];
                let calls = arr(&r["native_intervals"]);
                let ticks = arr(&r["ticks"]);
                let applications = arr(&r["applications"]);
                assert_eq!(ticks.len(), 8);
                let mut onset = None;
                let mut endpoint = 0;
                let mut positive = 0;
                let mut guarded = 0;
                let mut projected = 0;
                for (t, tick) in ticks.iter().enumerate() {
                    let start = 500000 * t as u64;
                    let end = start + 500000;
                    let parts: Vec<_> = calls
                        .iter()
                        .filter(|c| n(&c["end_ns"]) > start && n(&c["end_ns"]) <= end)
                        .collect();
                    assert_eq!(n(&tick["calls"]) as usize, parts.len());
                    let mut pair = 0.;
                    let mut gate = 0.;
                    for step in parts {
                        let lo = n(&step["start_ns"]);
                        let hi = n(&step["end_ns"]);
                        assert_eq!(lo, endpoint);
                        assert!(hi > lo);
                        endpoint = hi;
                        let primary = f(&step["primary_ns"]);
                        let other = f(&step["opponent_ns"]);
                        let project = step["projected"].as_bool().unwrap();
                        projected += usize::from(project);
                        if other > 0. {
                            onset.get_or_insert(hi);
                            positive += 1;
                        }
                        pair += primary;
                        gate += other;
                        if !step["certificate"].is_null() {
                            let cert = &step["certificate"];
                            let lower = f(&cert["certified_minimum_gap_m"]);
                            let actual = f(&cert["independent_sampled_minimum_gap_m"]);
                            assert!(named && lower >= 0. && actual > 0. && actual + 1e-11 >= lower);
                            assert!(f(&cert["free_endpoint_com_m"]) < 1e-10);
                            assert!(f(&cert["free_endpoint_attitude_rad"]) < 1e-8);
                            assert_eq!(other, 0.);
                            if !project {
                                assert_eq!(primary, 0.);
                            }
                            assert!(
                                !(lo as f64 + 0.001 < physical && physical + 0.001 < hi as f64)
                            );
                            after_gate += usize::from(onset.is_some());
                            for m in [
                                "independent_sampled_minimum_gap_m",
                                "certified_minimum_gap_m",
                                "free_endpoint_com_m",
                                "free_endpoint_attitude_rad",
                            ] {
                                stats.add("guarded_calls", m, f(&cert[m]));
                            }
                            guarded += 1;
                        }
                    }
                    assert!((pair - f(&tick["primary_normal_ns"])).abs() < 1e-11);
                    assert!((gate - f(&tick["opponent_normal_ns"])).abs() < 1e-11);
                    assert!((gate / 0.0005 - f(&tick["mean_opponent_force_n"])).abs() < 1e-8);
                }
                assert_eq!(endpoint, 4000000);
                assert_eq!(Some(n(&r["first_opponent_ns"])), onset);
                assert_eq!(projected, applications.len());
                assert!(projected <= 1);
                if delay <= 20000 {
                    assert_eq!(projected, 0);
                }
                if let Some(a) = applications.first() {
                    assert!((n(&a["at_ns"]) as f64 - when).abs() < 0.501);
                    let vector = v3(&a["world"]).distance(true_j);
                    assert!(vector < 1e-6);
                    stats.add(&k, "projected_vector_error_ns", vector);
                } else if named && delay >= 750000 {
                    declined.insert((id, mode));
                }
                for (m, inc) in [
                    ("worlds", 1),
                    ("primary_projections", projected),
                    ("guarded_calls", guarded),
                    ("positive_gate_calls", positive),
                ] {
                    *counts
                        .entry(k.clone())
                        .or_default()
                        .entry(m.into())
                        .or_default() += inc;
                }
                guard_count += guarded;
                stats.add(
                    &k,
                    "first_gate_endpoint_error_us",
                    (onset.unwrap() as f64 - physical) * 0.001,
                );
                if delay >= 750000 {
                    let last = ((physical - 0.001) / 500000.).floor() as usize;
                    let end = last as f64 * 500000.;
                    assert!(end > when && end < physical);
                    let true_normal = true_j.dot(v3(&donor["reference"]["normal"]));
                    let scalar: f64 = ticks[..last]
                        .iter()
                        .map(|t| f(&t["primary_normal_ns"]))
                        .sum();
                    stats.add(
                        &k,
                        "last_pregate_primary_normal_error_ns",
                        (scalar - true_normal).abs(),
                    );
                    let landing = (when / 500000.).ceil() as usize - 1;
                    stats.add(
                        &k,
                        "primary_public_com_um",
                        1e6 * f(&ticks[landing]["unopposed_reference_com_m"]),
                    );
                    let landing_impulse: DVec3 = ticks[..=landing]
                        .iter()
                        .map(|t| v3(&t["net_foot_impulse"]))
                        .sum();
                    stats.add(
                        &k,
                        "primary_public_vector_error_ns",
                        landing_impulse.distance(true_j),
                    );
                    let total: DVec3 = ticks[..last]
                        .iter()
                        .map(|t| v3(&t["net_foot_impulse"]))
                        .sum();
                    stats.add(
                        &k,
                        "last_pregate_com_um",
                        1e6 * f(&ticks[last - 1]["unopposed_reference_com_m"]),
                    );
                    stats.add(
                        &k,
                        "last_pregate_net_impulse_error_ns",
                        total.distance(true_j),
                    );
                    stats.add(&k, "last_pregate_public_us", end * 0.001);
                }
                if id == 1160 && mode == "slide" && delay == 750000 {
                    if feature.is_null() {
                        feature = json!({});
                    }
                    feature[lane] = json!({"primary_ns":when,"gate_ns":physical,"first_delivered_gate_ns":onset,"projection":applications,"com_at_2500us_um":1e6*f(&ticks[4]["unopposed_reference_com_m"]),"first_public_gate_ns":ticks[5]["opponent_normal_ns"],"mean_gate_force_n":ticks[5]["mean_opponent_force_n"],"native_calls":calls.len()});
                }
            }
        }
    }
    assert_eq!(trials.len(), 594);
    assert_eq!(counts["750000/named"]["primary_projections"], 65);
    assert_eq!(counts["1100000/named"]["primary_projections"], 65);
    assert_eq!(declined.len(), 1);
    json!({"donors":arr(&v["controls"]).len(),"modes":modes,"matched_worlds":trials.len(),"guarded_calls":guard_count,"guarded_calls_after_first_native_reaction":after_gate,"positive_delay_declines":declined,"counts":counts,"statistics":stats.summary(),"featured":feature})
}
fn costs(p: &Path) -> V {
    let mut all = Measures::default();
    let mut pairs = BTreeMap::<(u64, String, i64, u64), BTreeSet<bool>>::new();
    let raw = read(&p.join("cost.json"));
    for row in arr(&raw["rows"]) {
        let named = row["named"].as_bool().unwrap();
        let delay = row["delay_ns"].as_i64().unwrap();
        let l = lane(named);
        let key = format!("{delay}/{l}");
        let pair = (
            n(&row["id"]),
            row["mode"].as_str().unwrap().to_owned(),
            delay,
            n(&row["repeat"]),
        );
        assert!(pairs.entry(pair).or_default().insert(named));
        let landing = n(&row["primary_tick"]) as usize - 1;
        let gate = n(&row["gate_tick"]) as usize - 1;
        for clock in ["wall_ns", "cpu_ns"] {
            let times = arr(&row[clock]);
            assert_eq!(times.len(), 8);
            for tag in [&key[..], l] {
                for t in times {
                    all.add(tag, &format!("all_{clock}"), f(t));
                }
                all.add(tag, &format!("primary_{clock}"), f(&times[landing]));
                all.add(tag, &format!("gate_{clock}"), f(&times[gate]));
                if landing + 1 < 8 {
                    all.add(
                        tag,
                        &format!("after_primary_{clock}"),
                        f(&times[landing + 1]),
                    );
                }
                all.add(tag, &format!("total_{clock}"), times.iter().map(f).sum());
            }
        }
    }
    assert_eq!(pairs.len(), 66 * 3 * 6);
    assert!(pairs.values().all(|s| s.len() == 2));
    all.summary()
}
fn main() {
    let dir = std::env::args_os()
        .nth(1)
        .expect("give the named-opponent artifact directory");
    let p = Path::new(&dir);
    let result = json!({"study":study(p),"cpu6_a":costs(&p.join("cpu6-a")),"cpu6_b":costs(&p.join("cpu6-b"))});
    serde_json::to_writer_pretty(File::create(p.join("readback.json")).unwrap(), &result).unwrap();
    println!(
        "{} matched worlds, {} physically checked native intervals",
        result["study"]["matched_worlds"], result["study"]["guarded_calls"]
    );
}
