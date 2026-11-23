// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross native iteration convergence with graph and solver-row ordering on matched second landings.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const N: [u64; 4] = [8, 64, 512, 4096];
const R: [u64; 6] = [1, 2, 4, 8, 16, 32];
const M: [&str; 4] = [
    "warm_sorted",
    "cold_sorted",
    "warm_exchanged",
    "cold_exchanged",
];
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn check(b: bool, msg: &str) -> Result<()> {
    if b { Ok(()) } else { Err(msg.into()) }
}
fn n(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "invalid finite measurement".into())
}
fn vec3(v: &Value) -> Result<DVec3> {
    let p = DVec3::from_array(serde_json::from_value(v.clone())?);
    check(p.is_finite(), "invalid vector")?;
    Ok(p)
}
fn i(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing coordinate".into())
}
fn cases(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing treatments".into())
}
fn peak(a: &[DVec3; 4]) -> f64 {
    let mut largest = 0.0_f64;
    for (i, p) in a.iter().enumerate() {
        for q in &a[i + 1..] {
            largest = largest.max(p.distance(*q));
        }
    }
    largest
}
fn center(v: &[DVec3; 4]) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|k| {
        let mut x = v.map(|p| p.to_array()[k]);
        x.sort_by(f64::total_cmp);
        0.5 * (x[1] + x[2])
    }))
}
fn distribution(mut a: Vec<f64>) -> Value {
    a.sort_by(f64::total_cmp);
    let q = |fraction: f64| {
        (!a.is_empty()).then(|| a[((a.len() as f64 * fraction).ceil() as usize).max(1) - 1])
    };
    json!({"n":a.len(),"median":q(0.5),"p95":q(0.95),"max":a.last()})
}
#[derive(Clone, Copy)]
struct Response {
    j: DVec3,
    h: DVec3,
    com: DVec3,
    spin: DVec3,
    jz: f64,
    dp: f64,
    dh: f64,
}
fn response(t: &Value) -> Result<Response> {
    let normal = vec3(&t["normal_vector_ns"])?;
    let tangent = vec3(&t["tangent_vector_ns"])?;
    let j = vec3(&t["world_impulse_ns"])?;
    let h = vec3(&t["moment_nms"])?;
    let dp = (vec3(&t["end_momentum_ns"])? - vec3(&t["start_momentum_ns"])?
        + DVec3::Z * (2. * 9.81 * 0.0005)
        - j)
        .length();
    let dh = (vec3(&t["end_angular_momentum_nms"])? - vec3(&t["start_angular_momentum_nms"])? - h)
        .length();
    let quads = cases(&t["quadrant_ns"])?;
    let sum = quads.iter().map(n).collect::<Result<Vec<_>>>()?;
    check(
        sum.len() == 4
            && sum.iter().all(|x| *x >= 0.)
            && (sum.iter().sum::<f64>() - n(&t["normal_ns"])?).abs() < 1e-8
            && (normal + tangent).distance(j) < 1e-11
            && (n(&t["linear_residual_ns"])? - dp).abs() < 1e-12
            && (n(&t["angular_residual_nms"])? - dh).abs() < 1e-12,
        "source momentum check failed",
    )?;
    Ok(Response {
        j,
        h,
        dp,
        dh,
        spin: vec3(&t["end_spin_rad_s"])?,
        com: vec3(&t["end_com_m"])?,
        jz: n(&t["normal_ns"])?,
    })
}
fn analyze(src: &Value, original: &Value) -> Result<Value> {
    let k = i(&src["case"])?;
    let slot = i(&src["slot"])?;
    let held = slot % 2 == 1;
    check(
        src["format"] == "secondary-native-relaxation-grid-v2"
            && original["format"] == "second-landing-native-row-order-cohort-v1"
            && original["case"] == k
            && original["slot"] == slot
            && original["held"] == held
            && src["held"] == held
            && src["status"] == original["status"]
            && src["fixture"] == original["fixture"],
        "native case or study split differed",
    )?;
    let mut out = json!({"case":k,"slot":slot,"held":held,"status":src["status"]});
    if src["status"] == "censored" {
        out["reason"] = src["reason"].clone();
        return Ok(out);
    }
    check(
        src["incoming"] == original["incoming"]
            && src["cut_tick"] == original["incoming"]["cut_tick"],
        "incoming state changed",
    )?;
    let orig = cases(&original["branches"])?;
    let mut all = BTreeMap::new();
    for t in cases(&src["treatments"])? {
        let calls = i(&t["calls"])?;
        let passes = i(&t["passes"])?;
        let graph = t["graph"].as_str().ok_or("missing graph")?;
        check(
            N.contains(&calls)
                && R.contains(&passes)
                && M.contains(&graph)
                && i(&t["loaded_calls"])? <= calls
                && i(&t["reordered_rows"])? <= i(&t["visited_rows"])?,
            "invalid factor coordinate",
        )?;
        let a = response(t)?;
        let j = a.j;
        if calls == 4096 && passes == 1 {
            let control = orig
                .iter()
                .find(|r| r["graph"] == graph)
                .ok_or("missing row-order control")?;
            let p = &control["first_public"];
            check(
                control["first_native_ns"] == t["first_ns"]
                    && j.distance(vec3(&p["normal_vector_ns"])? + vec3(&p["tangent_vector_ns"])?)
                        < 1e-9
                    && a.h
                        .distance(vec3(&p["normal_moment_nms"])? + vec3(&p["tangent_moment_nms"])?)
                        < 1e-9,
                "row-order control changed",
            )?;
        }
        check(
            all.insert((calls, passes, graph), a).is_none(),
            "duplicate treatment",
        )?;
    }
    check(all.len() == 96, "incomplete native factorial")?;
    let fine = M.map(|g| all[&(4096, 32, g)].j);
    let reference = center(&fine);
    // Assess both existing graph histories independently. A common ordering
    // can be reproducible yet bias the physical prediction in either direction.
    let original_j = |name: &str| -> Result<DVec3> {
        let b = orig
            .iter()
            .find(|r| r["graph"] == name)
            .ok_or("missing original history")?;
        Ok(vec3(&b["first_public"]["normal_vector_ns"])?
            + vec3(&b["first_public"]["tangent_vector_ns"])?)
    };
    for (key, graph, sorted) in [
        ("retained", "retained", "warm_sorted"),
        ("rebuilt", "cold", "cold_sorted"),
    ] {
        let old = original_j(graph)?;
        let ordered = all[&(4096, 1, sorted)].j;
        let differences = fine.map(|target| ordered.distance(target) - old.distance(target));
        let tolerance = 1e-7; // Smaller than a tenth of a micro-newton-second.
        let class = if differences.iter().all(|v| *v < -tolerance) {
            "closer_all"
        } else if differences.iter().all(|v| *v > tolerance) {
            "farther_all"
        } else if differences.iter().all(|v| v.abs() <= tolerance) {
            "within_tolerance"
        } else {
            "reference_dependent"
        };
        out["ordering_vs_unmodified"][key] = json!({"classification":class,"original_error_to_center_ns":old.distance(reference),
          "ordered_error_to_center_ns":ordered.distance(reference),"signed_change_by_reference_ns":differences});
    }
    out["fine_graph_order_spread_ns"] = json!(peak(&fine));
    let mut cells = Vec::new();
    for calls in N {
        for (pi, passes) in R.into_iter().enumerate() {
            let set = M.map(|g| all[&(calls, passes, g)]);
            let j = set.map(|r| r.j);
            let ns = set.map(|r| r.jz);
            let max_ref = j.iter().map(|x| x.distance(reference)).fold(0., f64::max);
            let prior = (pi > 0).then(|| {
                M.iter()
                    .map(|&g| {
                        all[&(calls, passes, g)]
                            .j
                            .distance(all[&(calls, R[pi - 1], g)].j)
                    })
                    .fold(0., f64::max)
            });
            cells.push(json!({"calls":calls,"passes":passes,"vector_diameter_ns":peak(&j),"warm_order_ns":j[0].distance(j[2]),"cold_order_ns":j[1].distance(j[3]),
                       "same_order_graph_ns":j[0].distance(j[1]).max(j[2].distance(j[3])),
                       "lowest_normal_ns":ns.into_iter().fold(f64::INFINITY,f64::min),"highest_normal_ns":ns.into_iter().fold(0.,f64::max),
                       "max_linear_defect_ns":set.map(|r| r.dp).into_iter().fold(0.,f64::max),
                       "max_angular_defect_nms":set.map(|r|r.dh).into_iter().fold(0.,f64::max),
                       "moment_diameter_nms":peak(&set.map(|r|r.h)),"spin_diameter_rad_s":peak(&set.map(|r|r.spin)),
                       "com_diameter_um":peak(&set.map(|r|r.com))*1e6,"vector_to_fine_median_ns":max_ref,"max_previous_pass_change_ns":prior}));
        }
    }
    out["cells"] = json!(cells);
    out["empirical_center_ns"] = json!(reference.to_array());
    Ok(out)
}
fn draw(
    dir: &Path,
    name: &str,
    title: &str,
    subtitle: &str,
    summary: &Value,
    metrics: [(&str, &str); 2],
) -> Result<()> {
    let file = dir.join(name);
    let root = SVGBackend::new(&file, (1230, 750)).into_drawing_area();
    root.fill(&WHITE)?;
    let (header, panels) = root.split_vertically(90);
    header.draw(&Text::new(title, (28, 35), ("sans-serif", 28).into_font()))?;
    header.draw(&Text::new(
        subtitle,
        (28, 67),
        ("sans-serif", 17).into_font(),
    ))?;
    for (area, (metric, title)) in panels.split_evenly((2, 1)).into_iter().zip(metrics) {
        area.draw(&Text::new(title, (30, 25), ("sans-serif", 20).into_font()))?;
        let cells = cases(&summary["holdout"]["cells"])?;
        for (di, calls) in N.iter().enumerate() {
            let y = 48 + di as i32 * 56;
            area.draw(&Text::new(
                format!("{calls} calls"),
                (35, y + 29),
                ("sans-serif", 17).into_font(),
            ))?;
            for (ri, passes) in R.iter().enumerate() {
                let r = cells
                    .iter()
                    .find(|x| x["passes"] == *passes && x["calls"] == *calls)
                    .ok_or("missing plot cell")?;
                let x = 148 + ri as i32 * 173;
                let val = r[metric]["p95"].as_f64();
                let color = val
                    .map(|v| {
                        let k = ((v.max(1e-7).log10() + 7.) / 7.5).clamp(0., 1.);
                        RGBColor(
                            (224. - 175. * k) as u8,
                            (244. - 100. * k) as u8,
                            (239. - 111. * k) as u8,
                        )
                    })
                    .unwrap_or(RGBColor(230, 230, 230));
                area.draw(&Rectangle::new([(x, y), (x + 161, y + 48)], color.filled()))?;
                area.draw(&Text::new(
                    val.map(|v| {
                        if v < 1e-3 {
                            format!("{v:.2e}")
                        } else {
                            format!("{v:.4}")
                        }
                    })
                    .unwrap_or_else(|| "pending".into()),
                    (x + 26, y + 31),
                    ("sans-serif", 18).into_font(),
                ))?;
                if di == 3 {
                    area.draw(&Text::new(
                        format!("{passes} passes"),
                        (x + 25, y + 72),
                        ("sans-serif", 16).into_font(),
                    ))?;
                }
            }
        }
    }
    root.present()?;
    Ok(())
}
#[path = "../secondary_natural.rs"]
mod natural;
#[path = "../secondary_refresh.rs"]
mod refresh;
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args
        .first()
        .is_some_and(|x| x == "--features" || x == "--travel")
        && args.len() == 3
    {
        return refresh::run(
            Path::new(&args[1]),
            Path::new(&args[2]),
            args[0] == "--travel",
        );
    }
    if args.first().is_some_and(|x|x=="--registered") && args.len()==3 {return refresh::registered(Path::new(&args[1]),Path::new(&args[2]));}
    if args.first().is_some_and(|x| x == "--natural") && args.len() == 3 {
        return natural::run(Path::new(&args[1]), Path::new(&args[2]));
    }
    if args
        .first()
        .is_some_and(|x| x == "--cost" || x == "--features-cost" || x == "--travel-cost" || x == "--material-cost" || x == "--event-cost")
        && args.len() == 3
    {
        return natural::cost(
            Path::new(&args[1]),
            Path::new(&args[2]),
            args[0] != "--cost",
            args[0] == "--travel-cost",
            args[0]=="--material-cost",
            args[0]=="--event-cost",
        );
    }
    check(args.len() == 2, "secondary_relaxation_field STUDY OUTPUT")?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    fs::create_dir_all(out)?;
    let mut rows = Vec::new();
    let mut slots = BTreeSet::new();
    let mut controls = 0;
    for entry in fs::read_dir(dir.join("cases"))? {
        let p = entry?.path();
        if p.extension().is_none_or(|e| e != "json") {
            continue;
        }
        let src = read(&p)?;
        let k = i(&src["case"])?;
        let old = read(&dir.parent().ok_or("missing sibling")?.join(format!(
            "multifidelity-secondary-order-field/cases/case-{k:03}.json"
        )))?;
        let row = analyze(&src, &old)?;
        check(slots.insert(i(&row["slot"])?), "duplicate fixture")?;
        controls += usize::from(row["status"] == "measured") * 4;
        rows.push(row);
    }
    rows.sort_by_key(|r| r["slot"].as_u64());
    let mut summary = json!({"published":rows.len(),"missing_slots":(0..50).filter(|s|!slots.contains(s)).collect::<Vec<_>>(),"crossed_4096_one_pass_controls":controls});
    for (label, held) in [("calibration", false), ("holdout", true)] {
        let eligible: Vec<_> = rows
            .iter()
            .filter(|r| r["held"] == held && r["status"] == "measured")
            .collect();
        let mut aggregate = vec![];
        for (ni, calls) in N.into_iter().enumerate() {
            for (pi, passes) in R.into_iter().enumerate() {
                let mut cell = json!({"calls":calls,"passes":passes});
                for metric in [
                    "vector_diameter_ns",
                    "warm_order_ns",
                    "cold_order_ns",
                    "same_order_graph_ns",
                    "moment_diameter_nms",
                    "spin_diameter_rad_s",
                    "com_diameter_um",
                    "vector_to_fine_median_ns",
                    "max_previous_pass_change_ns",
                    "max_linear_defect_ns",
                    "max_angular_defect_nms",
                ] {
                    let numbers = eligible
                        .iter()
                        .filter_map(|r| r["cells"][ni * R.len() + pi][metric].as_f64())
                        .collect();
                    cell[metric] = distribution(numbers);
                }
                aggregate.push(cell);
            }
        }
        summary[label] = json!({"measured":eligible.len(),"cells":aggregate});
        for graph in ["retained", "rebuilt"] {
            for group in [
                "closer_all",
                "farther_all",
                "within_tolerance",
                "reference_dependent",
            ] {
                summary[label]["ordering_vs_unmodified"][graph][group] = json!(
                    eligible
                        .iter()
                        .filter(|r| r["ordering_vs_unmodified"][graph]["classification"] == group)
                        .count()
                );
            }
            for key in ["original_error_to_center_ns", "ordered_error_to_center_ns"] {
                summary[label]["ordering_vs_unmodified"][graph][key] = distribution(
                    eligible
                        .iter()
                        .filter_map(|r| r["ordering_vs_unmodified"][graph][key].as_f64())
                        .collect(),
                );
            }
        }
    }
    draw(
        out,
        "secondary-relaxation.svg",
        "Fixed-time convergence of secondary support reactions",
        &format!(
            "{} / 50 published matched fixtures  ·  held-out p95 (N·s)  ·  four actual graph and row-order realizations",
            rows.len()
        ),
        &summary,
        [
            (
                "vector_diameter_ns",
                "Largest disagreement across the four realizations",
            ),
            (
                "vector_to_fine_median_ns",
                "Largest deviation from the empirical 4,096-call / 32-pass center",
            ),
        ],
    )?;
    summary["cases"] = json!(rows);
    fs::write(
        out.join("readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}
