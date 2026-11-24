// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent native endpoint and SVG pixel-grid oracle. Arguments are the audit and view.
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::Path,
};

type Key = (usize, usize, bool, usize); // mm, drive percent, tier holds, stroke
type Endpoints = [[f64; 10]; 2]; // adaptive, onset, signed change, three grid distances, four reference-specific changes
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const ITERS: [usize; 2] = [10, 26];
const GRIDS: [u64; 4] = [512, 1024, 2048, 4096];
const WINDOWS: [&str; 2] = ["at_5ms", "at_13ms"];
fn json_file(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn array(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn n(v: &Value) -> Result<usize> {
    Ok(usize::try_from(v.as_u64().ok_or("missing index")?)?)
}
fn d(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing number".into())
}
fn near(a: f64, b: f64) -> Result<()> {
    if (a - b).abs() > 1e-11 {
        return Err(format!("expected {a:.15}, found {b:.15}").into());
    }
    Ok(())
}
fn point(
    paths: &[Value],
    method: &str,
    steps: Option<u64>,
    iterations: Option<u64>,
    tick: usize,
) -> Result<[f64; 3]> {
    let b = paths
        .iter()
        .find(|p| {
            p["method"] == method
                && p["calls"].as_u64() == steps
                && p["iterations"].as_u64() == iterations
        })
        .ok_or("missing path")?;
    let s = array(&b["record"]["samples"])?
        .iter()
        .find(|e| e["i"].as_u64() == Some(tick as u64))
        .ok_or("missing endpoint")?;
    let p = &s["com_m"];
    Ok([d(&p[0])?, d(&p[1])?, d(&p[2])?])
}
fn mm(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
        * 1000.
}
fn numbers(m: &Value) -> Result<[f64; 10]> {
    let r = &m["change_by_reference_mm"];
    let a = &m["successive_grid_distance_mm"];
    Ok([
        d(&m["adaptive_vs_4096_mm"])?,
        d(&m["onset_8x8_vs_4096_mm"])?,
        d(&m["change_mm"])?,
        d(&a[0])?,
        d(&a[1])?,
        d(&a[2])?,
        d(&r[0])?,
        d(&r[1])?,
        d(&r[2])?,
        d(&r[3])?,
    ])
}
fn check_ranking(raw: impl Iterator<Item = [f64; 10]>, report: &Value) -> Result<()> {
    let all = raw
        .map(|v| {
            [6, 7, 8, 9].map(|i| match v[i].partial_cmp(&0.).unwrap() {
                std::cmp::Ordering::Less => 0,
                std::cmp::Ordering::Equal => 1,
                std::cmp::Ordering::Greater => 2,
            })
        })
        .collect::<Vec<_>>();
    for k in 0..4 {
        for rank in 0..3 {
            assert_eq!(
                n(&report["counts_by_reference"][k][rank])?,
                all.iter().filter(|v| v[k] == rank).count()
            );
        }
    }
    for stage in 0..3 {
        for after in 0..3 {
            for before in 0..3 {
                assert_eq!(
                    n(&report["successive_transitions"][stage][before][after])?,
                    all.iter()
                        .filter(|v| v[stage] == before && v[stage + 1] == after)
                        .count()
                );
            }
        }
    }
    assert_eq!(
        n(&report["same_at_all_four"])?,
        all.iter()
            .filter(|v| v[1..].iter().all(|&k| k == v[0]))
            .count()
    );
    Ok(())
}
#[path = "shared/svg_cells.rs"]
mod svg_cells;
fn verify_rank_figure(path: &Path, summary: &Value) -> Result<()> {
    let xml = fs::read_to_string(path)?;
    let mut rectangles = BTreeMap::new();
    let mut texts = BTreeMap::new();
    let mut lines = xml.lines();
    while let Some(t) = lines.next() {
        if t.starts_with("<rect ")
            && svg_cells::attr(t, "width")? == "80"
            && svg_cells::attr(t, "height")? == "55"
        {
            let x = svg_cells::attr(t, "x")?.parse::<usize>()?;
            let y = svg_cells::attr(t, "y")?.parse::<usize>()?;
            assert!(
                rectangles
                    .insert((x, y), svg_cells::attr(t, "fill")?)
                    .is_none()
            );
        }
        if t.starts_with("<text ") {
            let x = svg_cells::attr(t, "x")?.parse::<usize>()?;
            let y = svg_cells::attr(t, "y")?.parse::<usize>()?;
            let label = lines.next().ok_or("missing rank text")?;
            if let Ok(integer) = label.parse::<usize>() {
                texts.insert((x, y), integer);
            }
        }
    }
    let xs = rectangles
        .keys()
        .map(|(x, _)| *x)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    let ys = rectangles
        .keys()
        .map(|(_, y)| *y)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    assert_eq!(
        (xs.len(), ys.len(), rectangles.len(), texts.len()),
        (9, 6, 54, 54)
    );
    for ((x, y), color) in rectangles {
        let col = xs.binary_search(&x).unwrap();
        let row = ys.binary_search(&y).unwrap();
        let count = n(
            &summary[WINDOWS[row / 3]]["grid_ranking"]["successive_transitions"][col / 3][row % 3]
                [col % 3],
        )?;
        assert_eq!(texts[&(x + 40, y + 27)], count);
        assert_eq!(
            color,
            if count == 0 {
                "#F6F7F7"
            } else if row % 3 == col % 3 {
                "#BDD9D7"
            } else {
                "#F5CBB1"
            }
        );
    }
    Ok(())
}
fn verify_plot(
    svg: &Path,
    known: &BTreeSet<(usize, usize, bool)>,
    raw: &BTreeMap<Key, Endpoints>,
    window: usize,
    metric: usize,
    limit: f64,
) -> Result<()> {
    svg_cells::check(svg, |key @ (offset, speed, holds, _)| {
        if let Some(endpoint) = raw.get(&key) {
            svg_cells::color(endpoint[window][metric], limit, metric == 2)
        } else if known.contains(&(offset, speed, holds)) {
            "#404147".into()
        } else {
            "#D3D7DB".into()
        }
    })
}
fn main() -> Result<()> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: view_readback AUDIT VIEW".into());
    }
    let root = Path::new(&args[0]);
    let view = Path::new(&args[1]);
    let s = json_file(&view.join("summary.json"))?;
    let mut raw = BTreeMap::<Key, Endpoints>::new();
    let mut completed = BTreeSet::new();
    let mut cut_ticks = BTreeMap::new();
    for idx in array(&s["completed_case_indices"])? {
        let id = n(idx)?;
        let case = json_file(&root.join("cases").join(format!("case-{id:04}.json")))?;
        let fixture = &case["result"]["fixture"];
        let mm = (d(&fixture["offset_m"])? * 1000.).round() as usize;
        let pct = (d(&fixture["speed_scale"])? * 100.).round() as usize;
        let holds = fixture["switched"].as_bool().ok_or("missing holds")?;
        assert!(completed.insert((mm, pct, holds)));
        for trial in array(&case["result"]["trials"])? {
            let key = (mm, pct, holds, n(&trial["stroke"])?);
            let paths = array(&trial["paths"])?;
            let mut windows = [[0.; 10]; 2];
            for (i, tick) in ITERS.into_iter().enumerate() {
                let mut grid = [[0.; 3]; 4];
                for (k, sub) in GRIDS.into_iter().enumerate() {
                    grid[k] = point(paths, "uniform", Some(sub), Some(4), tick)?;
                }
                let coarse = point(paths, "adaptive", None, None, tick)?;
                let candidate = point(paths, "onset", Some(8), Some(8), tick)?;
                let a = crate::mm(coarse, grid[3]);
                let o = crate::mm(candidate, grid[3]);
                let changes = grid.map(|r| crate::mm(candidate, r) - crate::mm(coarse, r));
                windows[i] = [
                    a,
                    o,
                    o - a,
                    crate::mm(grid[0], grid[1]),
                    crate::mm(grid[1], grid[2]),
                    crate::mm(grid[2], grid[3]),
                    changes[0],
                    changes[1],
                    changes[2],
                    changes[3],
                ];
            }
            assert!(raw.insert(key, windows).is_none());
            cut_ticks.insert(key, (id, n(&trial["cut"]["tick"])?));
        }
    }
    let cells = fs::read_to_string(view.join("cells.jsonl"))?;
    let mut seen = BTreeSet::new();
    let mut outlier = None;
    for l in cells.lines() {
        let c: Value = serde_json::from_str(l)?;
        let site = &c["site"];
        let key = (
            n(&site["offset_mm"])?,
            n(&site["drive_percent"])?,
            site["holds"].as_bool().ok_or("holds")?,
            n(&site["stroke"])?,
        );
        let expected = raw.get(&key).ok_or("emitted unknown cell")?;
        assert!(seen.insert(key));
        assert_eq!(cut_ticks[&key], (n(&site["index"])?, n(&site["cut_tick"])?));
        for (i, k) in WINDOWS.iter().enumerate() {
            for (a, b) in expected[i].into_iter().zip(numbers(&c[*k])?) {
                near(a, b)?;
            }
        }
        let candidate = (expected[1][2], n(&site["index"])?, key.3);
        if outlier.is_none_or(|old: (f64, usize, usize)| {
            candidate.0 > old.0
                || (candidate.0 == old.0 && (candidate.1, candidate.2) < (old.1, old.2))
        }) {
            outlier = Some(candidate);
        }
    }
    assert_eq!(seen.len(), raw.len());
    assert_eq!(n(&s["completed_fixtures"])?, completed.len());
    assert_eq!(n(&s["available_landings"])?, raw.len());
    assert_eq!(
        n(&s["unavailable_landings_in_recorded_fixtures"])?,
        completed.len() * 3 - raw.len()
    );
    if let Some((delta, index, stroke)) = outlier {
        near(delta, d(&s["selection"]["at_13ms"]["change_mm"])?)?;
        assert_eq!(
            (index, stroke),
            (
                n(&s["selection"]["site"]["index"])?,
                n(&s["selection"]["site"]["stroke"])?
            )
        );
    }
    for (w, key) in WINDOWS.into_iter().enumerate() {
        let changes: Vec<_> = raw.values().map(|m| m[w][2]).collect();
        let measured = &s[key];
        check_ranking(raw.values().map(|m| m[w]), &measured["grid_ranking"])?;
        for panel in array(&s["panels"])? {
            let held = panel["holds"].as_bool().ok_or("panel tier history")?;
            let stroke = n(&panel["stroke"])?;
            check_ranking(
                raw.iter()
                    .filter(|(k, _)| k.2 == held && k.3 == stroke)
                    .map(|(_, m)| m[w]),
                &panel[key]["grid_ranking"],
            )?;
        }
        assert_eq!(
            n(&measured["improved"])?,
            changes.iter().filter(|x| **x < 0.).count()
        );
        for j in 0..6 {
            let name = match j {
                0 => "adaptive_vs_4096_mm",
                1 => "onset_8x8_vs_4096_mm",
                2 => "change_mm",
                _ => "successive_mm",
            };
            let summary = if j < 3 {
                &measured[name]
            } else {
                &measured[name][j - 3]
            };
            let mut nums: Vec<_> = raw.values().map(|m| m[w][j]).collect();
            nums.sort_unstable_by(f64::total_cmp);
            if nums.is_empty() {
                continue;
            }
            for (quantile, numerator) in [("p50", 50), ("p95", 95)] {
                near(
                    nums[(nums.len() * numerator).div_ceil(100) - 1],
                    d(&summary[quantile])?,
                )?;
            }
            near(
                nums.iter().sum::<f64>() / nums.len() as f64,
                d(&summary["mean"])?,
            )?;
        }
        for metric in 2..6 {
            let name = if metric == 2 {
                "onset".into()
            } else {
                format!("grid-{}-{}", GRIDS[metric - 3], GRIDS[metric - 2])
            };
            let t = ITERS[w] / 2;
            let limit =
                d(&s["color_saturation_p95_mm"][if metric == 2 { "onset" } else { "grid" }])?;
            verify_plot(
                &view.join(format!("{name}-{t}ms.svg")),
                &completed,
                &raw,
                w,
                metric,
                limit,
            )?;
        }
    }
    verify_rank_figure(&view.join("reference-ranking.svg"), &s)?;
    let report = json!({"recorded_fixtures":completed.len(),"matched_landings":raw.len(),"raw_metrics_checked":raw.len()*20,"rank_aggregates_checked":14*40,"plots":9,"ranking_heatmap_labels_and_colors_verified":54,"plotted_cells_checked":30000,
        "max_metric_tolerance_mm":1e-11,"svg_cell_rgb":"exact","outlier_checked":outlier.is_some()});
    fs::write(
        view.join("readback.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}
