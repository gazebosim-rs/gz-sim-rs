// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read every isolated native landing independently of the probe's population summaries.
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
type Stats = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn main() {
    let arg = std::env::args_os().nth(1).unwrap();
    let data: Value =
        serde_json::from_reader(File::open(Path::new(&arg).join("single.json")).unwrap()).unwrap();
    let mut ids = BTreeSet::new();
    let mut stats = Stats::new();
    let mut counts = BTreeMap::new();
    for c in data["cases"].as_array().unwrap() {
        let id = c["id"].as_u64().unwrap();
        assert!(ids.insert(id));
        let mut profiles = BTreeSet::new();
        for row in c["trials"].as_array().unwrap() {
            let n = row["budget"].as_u64().unwrap();
            let on = row["sweep"].as_bool().unwrap();
            assert!(profiles.insert((n, on)));
            let calls = row["native_calls"].as_u64().unwrap();
            let selection = row["selected"].as_u64().unwrap();
            assert_eq!(calls, 8 + (n - 1) * selection);
            let early = row["early_ns"].as_f64().unwrap();
            let lag = (row["first_ns"].as_u64().unwrap() as f64
                - c["arrival_ns"].as_u64().unwrap() as f64)
                / 1000.;
            if n == 8 && on {
                assert!(selection >= 1 && early == 0. && lag >= 0.);
            }
            for set in ["all", if id >= 640 { "held_out" } else { "original" }] {
                let k = format!("{set}/{n:02}/{}", if on { "swept" } else { "baseline" });
                let sum = counts.entry(k.clone()).or_insert([0, 0, 0]);
                sum[0] += 1;
                sum[1] += u64::from(selection == 0);
                sum[2] += u64::from(early > 0.);
                let result = stats.entry(k).or_default();
                for (name, x) in [
                    ("first_endpoint_abs_us", lag.abs()),
                    (
                        "impulse_error_ns",
                        row["metrics"]["impulse_vector_error_ns"].as_f64().unwrap(),
                    ),
                    (
                        "spin_error_rad_s",
                        row["metrics"]["spin_error_rad_s"].as_f64().unwrap(),
                    ),
                    (
                        "com_error_um",
                        row["metrics"]["com_error_m"].as_f64().unwrap() * 1e6,
                    ),
                ] {
                    assert!(x.is_finite() && x >= 0.);
                    result.entry(name.to_owned()).or_default().push(x);
                }
            }
        }
        assert_eq!(profiles.len(), 6);
    }
    assert_eq!(ids.len(), 421);
    let result = stats
        .into_iter()
        .map(|(k, group)| {
            (
                k,
                group
                    .into_iter()
                    .map(|(key, mut a)| {
                        a.sort_by(f64::total_cmp);
                        let n = a.len();
                        (
                            key,
                            json!({"n":n,"p95":a[(n*95).div_ceil(100)-1],"max":a[n-1]}),
                        )
                    })
                    .collect::<BTreeMap<_, _>>(),
            )
        })
        .collect::<BTreeMap<_, _>>();
    println!("{}",serde_json::to_string_pretty(&json!({"counts_fields":["cases","no_nomination","early_reaction"],"counts":counts,"groups":result})).unwrap());
}
