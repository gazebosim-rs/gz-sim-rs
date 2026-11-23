// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct equal-work and equal-time comparisons from the saved native receipts.
use serde_json::{Value as V, json};
use std::{collections::BTreeMap, env, fs, path::PathBuf};

fn rows(v: &V) -> &[V] {
    v.as_array().unwrap()
}
fn x(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn distance(a: &V, b: &V) -> f64 {
    assert_eq!(rows(a).len(), rows(b).len());
    rows(a)
        .iter()
        .zip(rows(b))
        .map(|(u, v)| (x(u) - x(v)).powi(2))
        .sum::<f64>()
        .sqrt()
}
fn first(cell: &V) -> &V {
    &cell["samples"][0]
}
fn last(cell: &V) -> &V {
    rows(&cell["samples"]).last().unwrap()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-11 * (1. + a.abs()), "{a} vs {b}");
}
fn metrics(a: &V, b: &V) -> V {
    json!({"onset_normal_abs_ns":(x(&first(a)["normal_ns"])-x(&first(b)["normal_ns"])).abs(),
        "onset_corner_l2_ns":distance(&first(a)["corner_ns"],&first(b)["corner_ns"]),
        "onset_tangent_l2_ns":distance(&first(a)["tangent_on_foot_ns"],&first(b)["tangent_on_foot_ns"]),
        "onset_spin_l2_rad_s":distance(&first(a)["spin_rad_s"],&first(b)["spin_rad_s"]),
        "next_positive_outer_tick_changed":a["next_load_after_unloaded_outer_ns_after_cut"].as_u64().map(|n|n.div_ceil(500_000))!=b["next_load_after_unloaded_outer_ns_after_cut"].as_u64().map(|n|n.div_ceil(500_000)),
        "after_13ms_com_distance_mm":1000.*distance(&last(a)["com_m"],&last(b)["com_m"])})
}
fn distribution(vals: &[V], k: &str) -> V {
    let mut vec = vals.iter().map(|e| x(&e[k])).collect::<Vec<_>>();
    vec.sort_by(f64::total_cmp);
    let len = vec.len();
    json!({"min":vec[0],"median":(vec[(len-1)/2]+vec[len/2])*0.5,"max":vec[len-1]})
}
fn main() {
    let path = PathBuf::from(env::args_os().nth(1).unwrap());
    let src: V =
        serde_json::from_reader(fs::File::open(path.join("cadence.json")).unwrap()).unwrap();
    let mut checks = BTreeMap::<String, Vec<V>>::new();
    let mut highlights = vec![];
    let mut count = 0;
    let mut snapshots = 0;
    let mut accuracy = BTreeMap::<String, Vec<V>>::new();
    let mut rankings = vec![];
    let ns = src["step_us"].as_u64().unwrap() * 1000;
    for fixture in rows(&src["cases"]) {
        for cut in rows(&fixture["cuts"]) {
            let mut bank = BTreeMap::new();
            let mut equal_onsets = BTreeMap::new();
            for cell in rows(&cut["cells"]) {
                let onset = cell["onset"].as_str().unwrap();
                let follow = cell["follow"].as_str().unwrap();
                assert!(bank.insert((onset, follow), cell).is_none());
                if let Some(prev) = equal_onsets.insert(onset, first(cell)) {
                    assert_eq!(prev, first(cell));
                }
                let mut first_hit = None;
                let mut unloaded_since_first = false;
                let mut reload = None;
                let mut sum = 0.;
                let cut_tick = cut["cut"]["tick"].as_u64().unwrap();
                let samples = rows(&cell["samples"]);
                assert_eq!(samples.len(), 26);
                for (i, s) in samples.iter().enumerate() {
                    let ends = rows(
                        &cell[if i == 0 {
                            "onset_native_endpoints_ns"
                        } else {
                            "follow_native_endpoints_ns"
                        }],
                    );
                    assert_eq!(ends.len(), s["native_calls"].as_u64().unwrap() as usize);
                    assert_eq!(ends.last().unwrap().as_u64(), Some(ns));
                    assert!(ends.windows(2).all(|p| p[0].as_u64() < p[1].as_u64()));
                    assert_eq!(s["tick"].as_u64().unwrap(), cut_tick + i as u64 + 1);
                    let n = x(&s["normal_ns"]);
                    close(rows(&s["corner_ns"]).iter().map(x).sum::<f64>(), n);
                    sum += n;
                    if n > 0. {
                        let off = s["first_loaded_offset_ns"].as_u64().unwrap();
                        assert!(ends.iter().any(|t| t.as_u64() == Some(off)));
                        let tick_ns = ns * i as u64 + off;
                        if first_hit.is_none() {
                            first_hit = Some(tick_ns);
                        } else if unloaded_since_first && reload.is_none() {
                            reload = Some(tick_ns);
                        }
                    } else {
                        assert!(s["first_loaded_offset_ns"].is_null());
                        unloaded_since_first |= first_hit.is_some();
                    }
                    snapshots += 1;
                }
                assert_eq!(
                    cell["primary_first_receipt_ns_after_cut"].as_u64(),
                    first_hit
                );
                assert_eq!(
                    cell["next_load_after_unloaded_outer_ns_after_cut"].as_u64(),
                    reload
                );
                close(x(&cell["normal_total_ns"]), sum);
                count += 1;
            }
            let get = |onset, follow| bank[&(onset, follow)];
            let r = get("32x4", "32x4");
            let mut compare = |label: String, a: &V, b: &V| {
                checks.entry(label).or_default().push(metrics(a, b));
            };
            // Alter only the first outer tick and hold the subsequent 25 at 16 x 4.
            for (x, y) in [
                ("4x16", "8x8"),
                ("8x8", "16x4"),
                ("4x16", "16x4"),
                ("4x8", "8x4"),
                ("8x16", "16x8"),
                ("16x8", "32x4"),
                ("16x4", "32x4"),
                ("8x4", "8x8"),
                ("16x4", "16x8"),
                ("4x4_early", "4x4_late"),
                ("8x4_early", "8x4_late"),
                ("16x4_early", "16x4_late"),
            ] {
                compare(
                    format!("onset_only_{x}_vs_{y}"),
                    get(x, "16x4"),
                    get(y, "16x4"),
                );
            }
            for grid in ["4x16", "8x8", "4x4", "8x4", "32x4"] {
                let common = get("16x4", "16x4");
                compare(
                    format!("only_onset_{grid}_vs_16x4"),
                    get(grid, "16x4"),
                    common,
                );
                compare(
                    format!("only_follow_{grid}_vs_16x4"),
                    get("16x4", grid),
                    common,
                );
                compare(format!("all_{grid}_vs_16x4"), get(grid, grid), common);
            }
            let mut block = vec![];
            for g in ["4x16", "8x8", "16x4"] {
                let row = get(g, g);
                let result = metrics(row, r);
                close(
                    x(&result["after_13ms_com_distance_mm"]) * 1e-3,
                    x(&row["after_13ms_vs_32x4"]["com_m"]),
                );
                assert_eq!(
                    x(&first(row)["native_calls"]) * x(&first(row)["solver_iterations"]),
                    64.
                );
                accuracy
                    .entry(g.to_string())
                    .or_default()
                    .push(result.clone());
                block.push((g, x(&result["after_13ms_com_distance_mm"])));
                if fixture["fixture"]["name"] == "outer-fast" && cut["stroke"] == 2 {
                    highlights.push(json!({"grid":g,"normal_ns":first(row)["normal_ns"],"corner_ns":first(row)["corner_ns"],"spin_rad_s":first(row)["spin_rad_s"],
                        "onset_ns_after_cut":row["primary_first_receipt_ns_after_cut"],"reload_ns_after_cut":row["next_load_after_unloaded_outer_ns_after_cut"],"vs_32x4":result}));
                }
            }
            block.sort_by(|a, b| a.1.total_cmp(&b.1));
            rankings.push(json!({"case":fixture["fixture"]["name"],"stroke":cut["stroke"],"rank_by_com_error_vs_32x4":block}));
        }
    }
    let summarize = |m: BTreeMap<String, Vec<V>>| {
        m.into_iter()
            .map(|(key, values)| {
                let mut metric = serde_json::Map::new();
                for k in [
                    "onset_normal_abs_ns",
                    "onset_corner_l2_ns",
                    "onset_tangent_l2_ns",
                    "onset_spin_l2_rad_s",
                    "after_13ms_com_distance_mm",
                ] {
                    metric.insert(k.into(), distribution(&values, k));
                }
                metric.insert(
                    "different_next_positive_outer_count".into(),
                    json!(
                        values
                            .iter()
                            .filter(|r| r["next_positive_outer_tick_changed"] == true)
                            .count()
                    ),
                );
                (key, V::Object(metric))
            })
            .collect::<serde_json::Map<_, _>>()
    };
    let result = json!({"schedules_checked":count,"native_outer_boundaries_checked":snapshots,"matched_cuts":rankings.len(),
        "effect_magnitudes_across_cuts":summarize(checks),"equal_64_uniform_vs_32x4":summarize(accuracy),
        "rank_by_endpoint":rankings,"outer_fast_second_primary":highlights});
    fs::write(
        path.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
}
