// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check native four-corner impulses against observed velocities and an independent impact map.
use serde_json::{Value, json};
use std::{fs::File, path::Path};
fn load(p: &Path) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn xs(v: &Value) -> &[Value] {
    v.as_array().unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() < tol, "{a} {b}")
}
fn main() {
    let p = std::env::args().nth(1).unwrap();
    let p = Path::new(&p);
    let response = load(&p.join("response.json"));
    let mut records = 0;
    for &(name, tilted) in &[("level-fast", false), ("corner-a-fast", true)] {
        for sweeps in [1, 16] {
            for (file, rep) in [("locked", "locked_pair"), ("rigid", "single_rigid")] {
                let stream = load(&p.join(format!("{name}-{file}-{sweeps}.json")));
                let summary = xs(&response["results"])
                    .iter()
                    .find(|v| {
                        v["fixture"] == name
                            && v["representation"] == rep
                            && v["control"] == "approach"
                            && v["relaxation"] == sweeps
                    })
                    .unwrap();
                let mut total = 0.;
                let mut largest = 0.;
                let mut spike_time = 0;
                for row in xs(&stream) {
                    let j = xs(&row["points"])
                        .iter()
                        .map(|p| n(&p["normal_ns"]))
                        .sum::<f64>();
                    close(j, n(&row["normal_ns"]), 1e-12);
                    close(j, xs(&row["corner_ns"]).iter().map(n).sum::<f64>(), 1e-12);
                    if j > largest {
                        largest = j;
                        spike_time = row["time_ns"].as_u64().unwrap();
                    }
                    if !tilted && rep == "locked_pair" {
                        for (i, pi) in [[-0.18, -0.1], [-0.18, 0.1], [0.18, -0.1], [0.18, 0.1]]
                            .iter()
                            .enumerate()
                        {
                            let mut delta = 0.;
                            for (k, pk) in [[-0.18, -0.1], [-0.18, 0.1], [0.18, -0.1], [0.18, 0.1]]
                                .iter()
                                .enumerate()
                            {
                                let w = 0.5 + (pi[0] * pk[0] + pi[1] * pk[1]) / 0.05;
                                close(w, n(&row["native_mobility"][i][k]), 1e-8);
                                delta += w * n(&row["corner_ns"][k]);
                            }
                            close(
                                delta,
                                n(&row["post_speed_m_s"][i]) - n(&row["pre_speed_m_s"][i]),
                                3e-5,
                            );
                        }
                    }
                    records += 1;
                    total += j;
                }
                close(total, n(&summary["normal_ns"]), 1e-11);
                assert_eq!(spike_time, summary["dominant_time_ns"].as_u64().unwrap());
            }
        }
    }
    let mut timed = 0;
    for file in ["cost.json", "checks/cost-repeat.json"] {
        let result = load(&p.join(file));
        for lane in xs(&result["results"]) {
            let samples = xs(&lane["tick_ns"]);
            assert_eq!(samples.len(), 56);
            let mut primary: Vec<_> = samples.iter().map(|a| a[4].as_u64().unwrap()).collect();
            let mut all: Vec<_> = samples
                .iter()
                .flat_map(|a| xs(a).iter().map(|x| x.as_u64().unwrap()))
                .collect();
            timed += all.len();
            for (values, column) in [(&mut primary, "primary_p95_ns"), (&mut all, "all_p95_ns")] {
                values.sort_unstable();
                assert_eq!(
                    values[(95 * values.len()).div_ceil(100) - 1],
                    lane[column].as_u64().unwrap()
                );
            }
        }
    }
    assert_eq!(timed, 25088);
    println!(
        "{}",
        json!({"trials":xs(&response["results"]).len(),"readback_events":records,"timed_actuator_ticks":timed})
    );
}
